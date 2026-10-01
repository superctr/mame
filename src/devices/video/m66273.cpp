// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Mitsubishi M66273FP LCD controller with 19 200-byte VRAM

    A display-only controller for STN panels: binary or four grey levels,
    single or dual scan, a four- or eight-bit LCD data bus, with its whole
    VRAM on the host bus.  The two intermediate grey levels are frame-rate
    controlled by two pattern tables of sixteen 4 x 4 dot patterns, one per
    frame; the screen is drawn with each dot at its average over the sixteen
    frames, as levels 0 (never driven) to 16 (always driven).

    The host's VRAM access is arbitrated by cycle stealing during the
    display section of each line; CSE is high while the controller steals
    cycles and low in the part of the horizontal sync period, LPW less CSW
    characters, in which the host has the VRAM to itself.

    Not emulated: the WAIT output and the host-side timing.

***************************************************************************/

#include "emu.h"
#include "m66273.h"

#include "screen.h"

#define VERBOSE 0
#include "logmacro.h"


DEFINE_DEVICE_TYPE(M66273, m66273_device, "m66273", "Mitsubishi M66273 LCD Controller")


m66273_device::m66273_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, M66273, tag, owner, clock)
	, device_video_interface(mconfig, *this)
	, m_cse_cb(*this)
	, m_lcdenb_cb(*this)
	, m_cse_timer(nullptr)
	, m_r1(0)
	, m_r2(0)
	, m_cr(0x28)
	, m_lpw(0x04)
	, m_csw(0x02)
	, m_slt(0x78)
	, m_sa1(0)
	, m_sa2(SCREEN2_BASE)
	, m_sa1l(0)
	, m_sa2l(SCREEN2_BASE & 0xff)
	, m_mt(0)
	, m_idx(0)
	, m_cse(0)
{
}

void m66273_device::device_start()
{
	m_vram = make_unique_clear<u8[]>(VRAM_SIZE);
	std::fill(std::begin(m_pattern), std::end(m_pattern), 0);
	m_cse_timer = timer_alloc(FUNC(m66273_device::cse_tick), this);

	save_pointer(NAME(m_vram), VRAM_SIZE);
	save_item(NAME(m_pattern));
	save_item(NAME(m_r1));
	save_item(NAME(m_r2));
	save_item(NAME(m_cr));
	save_item(NAME(m_lpw));
	save_item(NAME(m_csw));
	save_item(NAME(m_slt));
	save_item(NAME(m_sa1));
	save_item(NAME(m_sa2));
	save_item(NAME(m_sa1l));
	save_item(NAME(m_sa2l));
	save_item(NAME(m_mt));
	save_item(NAME(m_idx));
	save_item(NAME(m_cse));
	save_item(NAME(m_char_period));
}

void m66273_device::device_reset()
{
	reset_registers();
}

void m66273_device::device_clock_changed()
{
	update_timing();
}

void m66273_device::reset_registers()
{
	m_r1 = 0;
	m_r2 = 0;
	m_cr = 0x28;
	m_lpw = 0x04;
	m_csw = 0x02;
	m_slt = 0x78;
	m_sa1 = 0;
	m_sa1l = 0;
	m_sa2 = SCREEN2_BASE;
	m_sa2l = SCREEN2_BASE & 0xff;
	m_mt = 0;
	m_idx = 0;
	m_lcdenb_cb(0);
	update_timing();
}


//-------------------------------------------------
//  timing: a line is CR display characters and LPW
//  sync characters, a character one MAINCLK (two in
//  the four-bit binary modes), and a frame 2 x SLT lines
//-------------------------------------------------

void m66273_device::set_cse(int state)
{
	if (state != m_cse)
	{
		m_cse = state;
		m_cse_cb(state);
	}
}

void m66273_device::update_timing()
{
	m_cse_timer->adjust(attotime::never);

	const int div = std::min((m_r1 & R1_DIV) >> 3, 4);
	const u32 mainclk = clock() >> div;
	const int clocks = ((m_r2 & (R2_GRAY | R2_8BIT)) == 0) ? 2 : 1;
	m_char_period = mainclk ? attotime::from_hz(mainclk) * clocks : attotime::never;

	const int lines = m_slt ? 2 * m_slt : 512;
	const int width = m_cr * ((m_r2 & R2_GRAY) ? 4 : 8);
	const int height = lines * ((m_r2 & R2_DUAL) ? 2 : 1);
	if (mainclk && width && (m_cr + m_lpw) && has_screen())
	{
		const attotime frame = m_char_period * ((m_cr + m_lpw) * lines);
		screen().configure(width, height, rectangle(0, width - 1, 0, height - 1), frame);
	}

	if (!displaying() || !mainclk)
		set_cse(0);
	else if (m_csw >= m_lpw)
		set_cse(1);
	else
	{
		set_cse(1);
		m_cse_timer->adjust(m_char_period * m_cr);
	}
}

TIMER_CALLBACK_MEMBER(m66273_device::cse_tick)
{
	if (m_cse)
	{
		set_cse(0);
		m_cse_timer->adjust(m_char_period * (m_lpw - m_csw));
	}
	else
	{
		set_cse(1);
		m_cse_timer->adjust(m_char_period * (m_csw + m_cr));
	}
}


//-------------------------------------------------
//  the host interface
//-------------------------------------------------

u8 m66273_device::read(offs_t offset)
{
	offset &= 0x7fff;
	if (offset < VRAM_SIZE)
		return m_vram[offset];
	if ((offset & 0x7f00) == 0x5000)
		return reg_r(offset & 0xff);
	return 0xff;
}

void m66273_device::write(offs_t offset, u8 data)
{
	offset &= 0x7fff;
	if (offset < VRAM_SIZE)
		m_vram[offset] = data;
	else if ((offset & 0x7f00) == 0x5000)
		reg_w(offset & 0xff, data);
}

u8 m66273_device::reg_r(offs_t offset)
{
	if (offset & 1)
		return 0xff;

	const int reg = offset >> 1;
	switch (reg)
	{
	case 0x00: return m_r1;
	case 0x01: return m_r2 & 0x37;
	case 0x06: return m_sa1 & 0xff;
	case 0x07: return m_sa1 >> 8;
	case 0x08: return m_sa2 & 0xff;
	case 0x09: return m_sa2 >> 8;
	case 0x0b: return m_idx & 0xff;
	case 0x0c: return m_idx >> 8;
	case 0x0d:
	{
		const u8 data = (m_idx < VRAM_SIZE) ? m_vram[m_idx] : 0xff;
		if (!machine().side_effects_disabled())
			m_idx = (m_idx + 1) & 0x7fff;
		return data;
	}
	default:
		if (reg >= 0x10 && reg < 0x50)
			return m_pattern[reg - 0x10];
		return 0xff;
	}
}

void m66273_device::reg_w(offs_t offset, u8 data)
{
	if (offset & 1)
		return;

	const int reg = offset >> 1;
	switch (reg)
	{
	case 0x00:
	{
		const u8 old = m_r1;
		if (data & R1_RESET)
		{
			reset_registers();
			m_r1 = R1_RESET;
		}
		else
			m_r1 = data;
		if ((old ^ m_r1) & R1_LCDE)
			m_lcdenb_cb(BIT(m_r1, 0));
		update_timing();
		break;
	}

	case 0x01:
		m_r2 = data & 0x37;
		update_timing();
		break;

	case 0x02:
		m_cr = data;
		update_timing();
		break;

	case 0x03:
		m_lpw = data;
		update_timing();
		break;

	case 0x04:
		m_csw = data;
		update_timing();
		break;

	case 0x05:
		m_slt = data;
		update_timing();
		break;

	case 0x06:
		m_sa1l = data;
		break;

	case 0x07:
		m_sa1 = ((data & 0x7f) << 8 | m_sa1l) & 0x7ffe;
		break;

	case 0x08:
		m_sa2l = data;
		break;

	case 0x09:
		m_sa2 = ((data & 0x7f) << 8 | m_sa2l) & 0x7ffe;
		break;

	case 0x0a:
		m_mt = data;
		break;

	case 0x0b:
		m_idx = (m_idx & 0x7f00) | data;
		break;

	case 0x0c:
		m_idx = (m_idx & 0x00ff) | (data & 0x7f) << 8;
		break;

	case 0x0d:
		if (m_idx < VRAM_SIZE)
			m_vram[m_idx] = data;
		m_idx = (m_idx + 1) & 0x7fff;
		break;

	default:
		if (reg >= 0x10 && reg < 0x50)
		{
			if (m_r1 & R1_DISP)
				LOG("%s: grey-scale pattern %d written with the display on\n", machine().describe_context(), reg - 0x10);
			else
				m_pattern[reg - 0x10] = data;
		}
		else
			logerror("%s: write %02x to unemulated register R%d\n", machine().describe_context(), data, reg + 1);
		break;
	}
}


//-------------------------------------------------
//  the display
//-------------------------------------------------

u32 m66273_device::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	const bool rev = m_r1 & R1_REV;
	if (!displaying() || !m_cr)
	{
		bitmap.fill(rev ? LEVELS - 1 : 0, cliprect);
		return 0;
	}

	// each intermediate level's duty at each dot of the 4 x 4 matrix, over its sixteen frames
	u8 duty[2][4][4];
	for (int table = 0; table < 2; table++)
		for (int line = 0; line < 4; line++)
			for (int dot = 0; dot < 4; dot++)
			{
				int count = 0;
				for (int frame = 0; frame < 16; frame++)
					count += BIT(m_pattern[table * 32 + frame * 2 + (line >> 1)], (line & 1) * 4 + 3 - dot);
				duty[table][line][dot] = count;
			}

	const bool gray = m_r2 & R2_GRAY;
	const bool dual = m_r2 & R2_DUAL;
	const int swap = (m_r2 & R2_SWAP) ? 1 : 0;
	const int lines = m_slt ? 2 * m_slt : 512;

	for (int y = cliprect.top(); y <= cliprect.bottom(); y++)
	{
		const bool second = dual && y >= lines;
		const int row = second ? y - lines : y;
		const unsigned base = !dual ? 0 : second ? SCREEN2_BASE : 0;
		const unsigned size = !dual ? VRAM_SIZE : second ? VRAM_SIZE - SCREEN2_BASE : SCREEN2_BASE;
		const unsigned start = ((second ? m_sa2 : m_sa1) - base) + row * m_cr;
		u16 *dest = &bitmap.pix(y, cliprect.left());

		for (int x = cliprect.left(); x <= cliprect.right(); x++)
		{
			const int ch = gray ? x >> 2 : x >> 3;
			if (ch >= m_cr)
			{
				*dest++ = rev ? LEVELS - 1 : 0;
				continue;
			}
			const u8 byte = m_vram[base + (((start + ch) % size) ^ swap)];
			int level;
			if (gray)
			{
				const int c = (byte >> (6 - 2 * (x & 3))) & 3;
				level = (c == 0) ? 0 : (c == 3) ? LEVELS - 1 : duty[c - 1][row & 3][x & 3];
			}
			else
				level = BIT(byte, 7 - (x & 7)) ? LEVELS - 1 : 0;
			*dest++ = rev ? LEVELS - 1 - level : level;
		}
	}
	return 0;
}
