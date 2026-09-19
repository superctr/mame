// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    h8570_pwm.cpp

    H8/570 PWM timer: a 16-bit counter, three compare registers, a six-bit
    output latch loaded from ODR0-ODR2 on the matches.

***************************************************************************/

#include "emu.h"
#include "h8570_pwm.h"

DEFINE_DEVICE_TYPE(H8570_PWM, h8570_pwm_device, "h8570_pwm", "H8/570 PWM timer")

h8570_pwm_device::h8570_pwm_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, H8570_PWM, tag, owner, clock)
	, m_cpu(*this, DEVICE_SELF_OWNER)
	, m_intc(*this, finder_base::DUMMY_TAG)
	, m_out_cb(*this)
	, m_icf_cb(*this)
	, m_irq_base(0)
	, m_tcr(0), m_tmsr(0), m_odl(0), m_odr{0, 0, 0}, m_temp(0)
	, m_ocr{0xffff, 0xffff, 0xffff}, m_tmr(0)
	, m_last_clock_update(0), m_event_time(0)
{
}

void h8570_pwm_device::device_start()
{
	save_item(NAME(m_tcr));
	save_item(NAME(m_tmsr));
	save_item(NAME(m_odl));
	save_item(NAME(m_odr));
	save_item(NAME(m_temp));
	save_item(NAME(m_ocr));
	save_item(NAME(m_tmr));
	save_item(NAME(m_last_clock_update));
	save_item(NAME(m_event_time));
}

void h8570_pwm_device::device_reset()
{
	m_tcr = 0;
	m_tmsr = 0;
	m_odl = 0;
	std::fill(std::begin(m_odr), std::end(m_odr), 0);
	m_temp = 0;
	std::fill(std::begin(m_ocr), std::end(m_ocr), 0xffff);
	m_tmr = 0;
	m_last_clock_update = 0;
	m_event_time = 0;
}

int h8570_pwm_device::divider() const
{
	static const int div[4] = { 1, 4, 8, 16 };
	return div[(m_tcr & TCR_CKS) >> 4];
}

u32 h8570_pwm_device::ticks_to_match() const
{
	u32 ticks = 0x10000;
	for (u16 ocr : m_ocr)
		ticks = std::min<u32>(ticks, ((ocr - m_tmr) & 0xffff) + 1);
	return ticks;
}

void h8570_pwm_device::match(int n)
{
	const u8 odl = (m_tcr & TCR_OMS) ? (m_odr[n] & 0x3f) : ((m_odl & ~(1 << n)) | (m_odr[n] & (1 << n)));
	if (odl != m_odl)
	{
		m_odl = odl;
		m_out_cb(m_odl);
	}

	if (!BIT(m_tmsr, n))
	{
		m_tmsr |= 1 << n;
		if (BIT(m_tmsr, 3 + n))
			m_intc->internal_interrupt(m_irq_base + n);
	}

	if (n == 0 && (m_tmsr & TMSR_TRE0))
		m_icf_cb(6);
	if (n == 2 && (m_tmsr & TMSR_TRE2))
		m_icf_cb(7);
}

void h8570_pwm_device::update_counter(u64 cur_time)
{
	if (!cur_time)
		cur_time = m_cpu->total_cycles();

	if (!(m_tcr & TCR_TCE))
	{
		m_last_clock_update = cur_time;
		return;
	}

	const int div = divider();
	u64 delta = cur_time / div - m_last_clock_update / div;
	m_last_clock_update = cur_time;

	while (delta)
	{
		const u32 ticks = ticks_to_match();
		if (delta < ticks)
		{
			m_tmr += delta;
			break;
		}

		const u16 matched = m_tmr + ticks - 1;
		m_tmr = matched + 1;
		delta -= ticks;
		for (int n = 0; n < 3; n++)
			if (m_ocr[n] == matched)
				match(n);
		if ((m_tcr & TCR_FRM) && m_ocr[0] == matched)
			m_tmr = 0;
	}
}

void h8570_pwm_device::recalc_event(u64 cur_time)
{
	const bool update_cpu = cur_time == 0;
	const u64 old_event_time = m_event_time;

	if (!cur_time)
		cur_time = m_cpu->total_cycles();

	if (m_tcr & TCR_TCE)
	{
		const int div = divider();
		m_event_time = (cur_time / div + ticks_to_match()) * div;
	}
	else
		m_event_time = 0;

	if (old_event_time != m_event_time && update_cpu)
		m_cpu->internal_update();
}

u64 h8570_pwm_device::internal_update(u64 current_time)
{
	while (m_event_time && current_time >= m_event_time)
	{
		update_counter(m_event_time);
		recalc_event(m_event_time);
	}

	return m_event_time;
}

void h8570_pwm_device::notify_standby(int state)
{
	if (!state && m_event_time)
	{
		const u64 delta = m_cpu->total_cycles() - m_cpu->standby_time();
		m_event_time += delta;
		m_last_clock_update += delta;
	}
}

u8 h8570_pwm_device::tcr_r()
{
	return m_tcr;
}

void h8570_pwm_device::tcr_w(u8 data)
{
	update_counter();
	m_tcr = data;
	recalc_event();
}

u8 h8570_pwm_device::tmsr_r()
{
	update_counter();
	return m_tmsr;
}

void h8570_pwm_device::tmsr_w(u8 data)
{
	update_counter();
	m_tmsr = (data & ~TMSR_OCF) | (m_tmsr & data & TMSR_OCF);
}

u8 h8570_pwm_device::odl_r()
{
	return m_odl | 0xc0;
}

void h8570_pwm_device::odl_w(u8 data)
{
	update_counter();
	if ((data & 0x3f) != m_odl)
	{
		m_odl = data & 0x3f;
		m_out_cb(m_odl);
	}
}

u8 h8570_pwm_device::odr_r(offs_t offset)
{
	return m_odr[offset] | 0xc0;
}

void h8570_pwm_device::odr_w(offs_t offset, u8 data)
{
	m_odr[offset] = data & 0x3f;
}

u8 h8570_pwm_device::ocr_r(offs_t offset)
{
	const u16 v = m_ocr[offset >> 1];
	return BIT(offset, 0) ? v & 0xff : v >> 8;
}

void h8570_pwm_device::ocr_w(offs_t offset, u8 data)
{
	if (!BIT(offset, 0))
	{
		m_temp = data;
		return;
	}
	update_counter();
	m_ocr[offset >> 1] = (u16(m_temp) << 8) | data;
	recalc_event();
}

u8 h8570_pwm_device::tmr_r(offs_t offset)
{
	if (offset)
		return m_temp;
	update_counter();
	if (!machine().side_effects_disabled())
		m_temp = m_tmr & 0xff;
	return m_tmr >> 8;
}

void h8570_pwm_device::tmr_w(offs_t offset, u8 data)
{
	if (!offset)
	{
		m_temp = data;
		return;
	}
	update_counter();
	m_tmr = (u16(m_temp) << 8) | data;
	recalc_event();
}
