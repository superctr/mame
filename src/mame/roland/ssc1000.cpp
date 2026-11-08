// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Roland SSC-1000 key scanner

    High-level: the host's key events arrive through key_w() with their
    velocity already decided; the per-key tables and the set-up bytes are
    stored but not used.

***************************************************************************/

#include "emu.h"
#include "ssc1000.h"

DEFINE_DEVICE_TYPE(SSC1000, ssc1000_device, "ssc1000", "Roland SSC-1000 Key Scanner")

ssc1000_device::ssc1000_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, SSC1000, tag, owner, clock)
	, m_int_callback(*this)
{
}

void ssc1000_device::device_start()
{
	m_int_timer = timer_alloc(FUNC(ssc1000_device::int_raise), this);

	save_item(NAME(m_table));
	save_item(NAME(m_setup));
	save_item(NAME(m_control));
	save_item(NAME(m_queue_key));
	save_item(NAME(m_queue_vel));
	save_item(NAME(m_head));
	save_item(NAME(m_count));
	save_item(NAME(m_int));
}

void ssc1000_device::device_reset()
{
	std::fill(&m_table[0][0], &m_table[0][0] + sizeof(m_table), 0);
	std::fill(std::begin(m_setup), std::end(m_setup), 0);
	m_control = 0x03;
	m_head = 0;
	m_count = 0;
	m_int = false;
	m_int_timer->adjust(attotime::never);
	m_int_callback(CLEAR_LINE);
}

void ssc1000_device::update_int()
{
	bool const state = m_count && !(m_control & 0x03);
	if (state != m_int)
	{
		m_int = state;
		m_int_callback(state ? ASSERT_LINE : CLEAR_LINE);
	}
}

TIMER_CALLBACK_MEMBER(ssc1000_device::int_raise)
{
	update_int();
}

void ssc1000_device::key_w(u8 key, u8 velocity)
{
	if (m_count == QUEUE)
		return;
	unsigned const at = (m_head + m_count) % QUEUE;
	m_queue_key[at] = (key & 0x7f) | (velocity ? 0x80 : 0x00);
	m_queue_vel[at] = velocity & 0x7f;
	m_count++;
	update_int();
}

u8 ssc1000_device::read(offs_t offset)
{
	switch (offset & 0x3ff)
	{
	case 0x208:
	{
		u8 const data = m_count ? m_queue_vel[m_head] : 0;
		if (m_count && !machine().side_effects_disabled())
		{
			m_head = (m_head + 1) % QUEUE;
			m_count--;
			if (m_int)
			{
				m_int = false;
				m_int_callback(CLEAR_LINE);
			}
			if (m_count)
				m_int_timer->adjust(attotime::from_usec(20));
		}
		return data;
	}
	case 0x209:
		return m_count ? m_queue_key[m_head] : 0;
	default:
		if (offset < 0x200)
			return m_table[offset >> 7][offset & 0x7f];
		if (offset < 0x206)
			return m_setup[offset - 0x200];
		if (offset == 0x206)
			return m_control;
		return 0;
	}
}

void ssc1000_device::write(offs_t offset, u8 data)
{
	offset &= 0x3ff;
	if (offset < 0x200)
		m_table[offset >> 7][offset & 0x7f] = data;
	else if (offset < 0x206)
		m_setup[offset - 0x200] = data;
	else if (offset == 0x206)
	{
		m_control = data;
		update_int();
	}
}
