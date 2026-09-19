// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    h8570_pwm.h

    H8/570 PWM timer

***************************************************************************/

#ifndef MAME_CPU_H8500_H8570_PWM_H
#define MAME_CPU_H8500_H8570_PWM_H

#pragma once

#include "cpu/h8/h8_cpu_base.h"
#include "cpu/h8/h8_intc_base.h"

class h8570_pwm_device : public device_t
{
public:
	h8570_pwm_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);
	template<typename T, typename U> h8570_pwm_device(const machine_config &mconfig, const char *tag, device_t *owner, T &&cpu, U &&intc, int irq_base)
		: h8570_pwm_device(mconfig, tag, owner, 0)
	{
		m_cpu.set_tag(std::forward<T>(cpu));
		m_intc.set_tag(std::forward<U>(intc));
		m_irq_base = irq_base;
	}

	auto out_cb() { return m_out_cb.bind(); }
	auto icf_cb() { return m_icf_cb.bind(); }

	u8 tcr_r();
	void tcr_w(u8 data);
	u8 tmsr_r();
	void tmsr_w(u8 data);
	u8 odl_r();
	void odl_w(u8 data);
	u8 odr_r(offs_t offset);
	void odr_w(offs_t offset, u8 data);
	u8 ocr_r(offs_t offset);
	void ocr_w(offs_t offset, u8 data);
	u8 tmr_r(offs_t offset);
	void tmr_w(offs_t offset, u8 data);

	u64 internal_update(u64 current_time);
	void notify_standby(int state);

protected:
	enum {
		TCR_OMS  = 0x08,
		TCR_CKS  = 0x30,
		TCR_FRM  = 0x40,
		TCR_TCE  = 0x80,

		TMSR_OCF  = 0x07,
		TMSR_OCIE = 0x38,
		TMSR_TRE0 = 0x40,
		TMSR_TRE2 = 0x80
	};

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	int divider() const;
	u32 ticks_to_match() const;
	void match(int n);
	void update_counter(u64 cur_time = 0);
	void recalc_event(u64 cur_time = 0);

	required_device<h8_cpu_base> m_cpu;
	required_device<h8_intc_base> m_intc;
	devcb_write8 m_out_cb;
	devcb_write8 m_icf_cb;
	int m_irq_base;

	u8 m_tcr, m_tmsr, m_odl, m_odr[3], m_temp;
	u16 m_ocr[3], m_tmr;
	u64 m_last_clock_update, m_event_time;
};

DECLARE_DEVICE_TYPE(H8570_PWM, h8570_pwm_device)

#endif // MAME_CPU_H8500_H8570_PWM_H
