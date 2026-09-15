// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    sh_wdt.h

    SH watchdog timer

***************************************************************************/

#ifndef MAME_CPU_SH_SH_WDT_H
#define MAME_CPU_SH_SH_WDT_H

#pragma once

#include "sh_intc.h"

class sh_wdt_device : public device_t
{
public:
	sh_wdt_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	template <typename T> sh_wdt_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock, T &&intc, int vector)
		: sh_wdt_device(mconfig, tag, owner, clock)
	{
		m_intc.set_tag(std::forward<T>(intc));
		m_vector = vector;
	}

	void map(address_map &map) ATTR_COLD;

	u8 tcsr_r();
	u8 tcnt_r();
	u8 rstcsr_r();
	void timer_w(offs_t offset, u16 data, u16 mem_mask = ~0);
	void reset_w(offs_t offset, u16 data, u16 mem_mask = ~0);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	enum : u8
	{
		TCSR_OVF   = 0x80,
		TCSR_WT_IT = 0x40,
		TCSR_TME   = 0x20,
		TCSR_CKS   = 0x07,

		RSTCSR_WOVF = 0x80,
		RSTCSR_RSTE = 0x40
	};

	TIMER_CALLBACK_MEMBER(overflow);

	u8 counter() const;
	int prescaler() const;
	void update_timer();

	required_device<sh_intc_device> m_intc;
	int m_vector;

	emu_timer *m_timer;
	attotime m_count_time;

	u8 m_tcsr;
	u8 m_tcnt;
	u8 m_rstcsr;
};

DECLARE_DEVICE_TYPE(SH_WDT, sh_wdt_device)

#endif // MAME_CPU_SH_SH_WDT_H
