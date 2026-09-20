// license:BSD-3-Clause
// copyright-holders:superctr
/****************************************************************************

    Mitsubishi M60205-0601FP, Roland's SH-bus peripheral gate array.

    Fitted as IC1 on the JV-1080, IC13 on the XP-60 and IC8 on the MT-300.
    It sits on the CPU bus behind one chip select, subdivides that select
    into twelve of its own, generates wait states, scans a switch matrix and
    an LED matrix, counts a quadrature encoder, and drives an HD44780.  Its
    sixty-four registers are undocumented; what is modelled here is what the
    JV-1080's firmware uses (xp/docs/jv1080.md).

****************************************************************************/

#include "emu.h"
#include "m60205.h"


DEFINE_DEVICE_TYPE(M60205, m60205_device, "m60205", "Roland M60205 gate array")

m60205_device::m60205_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, M60205, tag, owner, clock)
	, m_int_cb(*this)
	, m_lcd_control_cb(*this)
	, m_lcd_data_cb(*this)
	, m_led_cb(*this)
	, m_scan_cb(*this, 0x00)
	, m_port_cb(*this, 0x00)
	, m_encoder_cb(*this, 0x00)
{
}

void m60205_device::device_start()
{
	m_timer_a = timer_alloc(FUNC(m60205_device::tick_a), this);
	m_timer_b = timer_alloc(FUNC(m60205_device::tick_b), this);

	save_item(NAME(m_pending));
	save_item(NAME(m_int_state));
	save_item(NAME(m_config));
	save_item(NAME(m_led));
	save_item(NAME(m_scan_state));
	save_item(NAME(m_port_state));
	save_item(NAME(m_encoder_state));
	save_item(NAME(m_encoder_delta));
	save_item(NAME(m_fifo));
	save_item(NAME(m_fifo_head));
	save_item(NAME(m_fifo_tail));
	save_item(NAME(m_divider));
}

void m60205_device::device_reset()
{
	m_pending = 0;
	m_int_state = 0;
	std::fill(std::begin(m_config), std::end(m_config), 0);
	std::fill(std::begin(m_led), std::end(m_led), 0);
	std::fill(std::begin(m_scan_state), std::end(m_scan_state), 0);
	m_port_state = 0;
	m_encoder_state = 0;
	m_encoder_delta = 0;
	m_fifo_head = m_fifo_tail = 0;
	m_divider = 0;

	m_int_cb(0);

	m_timer_a->adjust(attotime::from_hz(1000), 0, attotime::from_hz(1000));
	m_timer_b->adjust(attotime::from_hz(250), 0, attotime::from_hz(250));
}


void m60205_device::raise(unsigned source)
{
	m_pending |= 1 << source;
	update_int();
}

void m60205_device::update_int()
{
	const int state = m_pending != 0;
	if (state != m_int_state)
	{
		m_int_state = state;
		m_int_cb(state);
	}
}

void m60205_device::queue_key(u8 code)
{
	const u8 next = (m_fifo_tail + 1) % FIFO_SIZE;
	if (next == m_fifo_head)
		return;

	m_fifo[m_fifo_tail] = code;
	m_fifo_tail = next;
	raise(SOURCE_KEY);
}


TIMER_CALLBACK_MEMBER(m60205_device::tick_a)
{
	for (unsigned line = 0; line < SCAN_LINES; line++)
	{
		const u8 state = m_scan_cb[line]();
		u8 changed = state ^ m_scan_state[line];
		m_scan_state[line] = state;

		for (unsigned bit = 0; changed != 0; bit++, changed >>= 1)
			if (BIT(changed, 0))
				queue_key((line * 8 + bit) | (BIT(state, bit) ? 0x80 : 0x00));
	}

	const u8 port = m_port_cb();
	u8 changed = port ^ m_port_state;
	m_port_state = port;

	for (unsigned bit = 0; changed != 0; bit++, changed >>= 1)
		if (BIT(changed, 0))
			queue_key((0x20 + bit) | (BIT(port, bit) ? 0x80 : 0x00));

	const u8 encoder = m_encoder_cb();
	const s8 step = encoder - m_encoder_state;
	m_encoder_state = encoder;
	if (step != 0)
	{
		m_encoder_delta += step;
		raise(SOURCE_ENCODER);
	}

	raise(SOURCE_TIMER_A);
	raise(8);
}

TIMER_CALLBACK_MEMBER(m60205_device::tick_b)
{
	raise(SOURCE_TIMER_B);
}


u8 m60205_device::read(offs_t offset)
{
	switch (offset)
	{
	case 0x00:
		return m_fifo_head != m_fifo_tail ? 0x01 : 0x00;

	case 0x10: case 0x11: case 0x12: case 0x13:
	case 0x14: case 0x15: case 0x16: case 0x17:
		return m_led[offset & 7];

	case 0x3c:
	{
		// the number of the pending source, lowest first; reading clears it
		if (m_pending == 0)
			return 0x0f;

		unsigned source = 0;
		while (!BIT(m_pending, source))
			source++;

		if (!machine().side_effects_disabled())
		{
			m_pending &= ~(1 << source);
			update_int();
		}
		return source;
	}

	case 0x3d:
	{
		const s16 delta = m_encoder_delta;
		if (!machine().side_effects_disabled())
			m_encoder_delta = 0;
		return std::clamp<s16>(delta, -128, 127) & 0xff;
	}

	case 0x3e:
	{
		if (m_fifo_head == m_fifo_tail)
			return 0;

		const u8 code = m_fifo[m_fifo_head];
		if (!machine().side_effects_disabled())
		{
			m_fifo_head = (m_fifo_head + 1) % FIFO_SIZE;
			if (m_fifo_head != m_fifo_tail)
				raise(SOURCE_KEY);
		}
		return code;
	}

	default:
		if (offset >= 0x20 && offset < 0x30)
			return m_config[offset - 0x20];
		if (!machine().side_effects_disabled())
			logerror("%s: read %02x\n", machine().describe_context(), offset);
		return 0;
	}
}

void m60205_device::write(offs_t offset, u8 data)
{
	switch (offset)
	{
	case 0x10: case 0x11: case 0x12: case 0x13:
	case 0x14: case 0x15: case 0x16: case 0x17:
		m_led[offset & 7] = data;
		m_led_cb(offset & 7, data);
		break;

	case 0x38:
		m_lcd_control_cb(data);
		break;

	case 0x39:
		m_lcd_data_cb(data);
		break;

	default:
		if (offset >= 0x20 && offset < 0x30)
			m_config[offset - 0x20] = data;
		else
			logerror("%s: write %02x = %02x\n", machine().describe_context(), offset, data);
		break;
	}
}
