// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    sh_wdt.cpp

    SH watchdog timer

    An eight bit up counter off a divided peripheral clock.  In interval
    timer mode, the mode it comes out of reset in, an overflow raises the
    interval timer interrupt and the counter carries on; in watchdog timer
    mode it sets WOVF instead and, with RSTE, resets the chip.

    The three registers are written as words with a key in the upper byte,
    0x5a for the counter and 0xa5 for the two control registers, and read
    as bytes.

    TODO list (not comprehensive):
    - the reset RSTE asks for, and the WDTOVF pin
    - the standby control the module shares its address block with

***************************************************************************/

#include "emu.h"
#include "sh_wdt.h"

#define LOG_READ (1U << 1)
#define LOG_WRITE (1U << 2)

// #define VERBOSE (LOG_GENERAL | LOG_READ | LOG_WRITE)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(SH_WDT, sh_wdt_device, "sh_wdt", "SH watchdog timer")

sh_wdt_device::sh_wdt_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, SH_WDT, tag, owner, clock)
	, m_intc(*this, finder_base::DUMMY_TAG)
	, m_vector(0)
{
}

void sh_wdt_device::device_start()
{
	m_timer = timer_alloc(FUNC(sh_wdt_device::overflow), this);

	save_item(NAME(m_count_time));
	save_item(NAME(m_tcsr));
	save_item(NAME(m_tcnt));
	save_item(NAME(m_rstcsr));
}

void sh_wdt_device::device_reset()
{
	m_count_time = attotime::zero;
	m_tcsr = 0x18;
	m_tcnt = 0;
	m_rstcsr = 0x1f;
	m_timer->adjust(attotime::never);
}

void sh_wdt_device::map(address_map &map)
{
	map(0x00, 0x00).r(FUNC(sh_wdt_device::tcsr_r));
	map(0x01, 0x01).r(FUNC(sh_wdt_device::tcnt_r));
	map(0x03, 0x03).r(FUNC(sh_wdt_device::rstcsr_r));
	map(0x00, 0x01).w(FUNC(sh_wdt_device::timer_w));
	map(0x02, 0x03).w(FUNC(sh_wdt_device::reset_w));
}

int sh_wdt_device::prescaler() const
{
	constexpr int shift[8] = { 1, 6, 7, 8, 9, 10, 12, 13 };
	return shift[m_tcsr & TCSR_CKS];
}

u8 sh_wdt_device::counter() const
{
	if (!(m_tcsr & TCSR_TME))
		return m_tcnt;
	const u64 elapsed = (machine().time() - m_count_time).as_ticks(clock()) >> prescaler();
	return u8(m_tcnt + elapsed);
}

void sh_wdt_device::update_timer()
{
	if (!(m_tcsr & TCSR_TME))
	{
		m_timer->adjust(attotime::never);
		return;
	}
	m_count_time = machine().time();
	m_timer->adjust(attotime::from_ticks(u64(0x100 - m_tcnt) << prescaler(), clock()));
}

TIMER_CALLBACK_MEMBER(sh_wdt_device::overflow)
{
	m_tcnt = 0;
	if (m_tcsr & TCSR_WT_IT)
		m_rstcsr |= RSTCSR_WOVF;
	else
	{
		m_tcsr |= TCSR_OVF;
		m_intc->internal_interrupt(m_vector);
	}
	update_timer();
}

u8 sh_wdt_device::tcsr_r()
{
	LOGMASKED(LOG_READ, "tcsr_r %02x\n", m_tcsr);
	return m_tcsr;
}

u8 sh_wdt_device::tcnt_r()
{
	const u8 data = counter();
	LOGMASKED(LOG_READ, "tcnt_r %02x\n", data);
	return data;
}

u8 sh_wdt_device::rstcsr_r()
{
	return m_rstcsr;
}

void sh_wdt_device::timer_w(offs_t offset, u16 data, u16 mem_mask)
{
	LOGMASKED(LOG_WRITE, "timer_w %04x\n", data);
	if ((data >> 8) == 0x5a)
	{
		m_tcnt = data & 0xff;
		update_timer();
	}
	else if ((data >> 8) == 0xa5)
	{
		m_tcnt = counter();
		// OVF only clears, and only by a zero written over a one
		m_tcsr = (m_tcsr & data & TCSR_OVF) | (data & ~TCSR_OVF) | 0x18;
		// clearing TME initialises the counter
		if (!(m_tcsr & TCSR_TME))
			m_tcnt = 0;
		update_timer();
	}
}

void sh_wdt_device::reset_w(offs_t offset, u16 data, u16 mem_mask)
{
	LOGMASKED(LOG_WRITE, "reset_w %04x\n", data);
	if ((data >> 8) == 0x5a)
		m_rstcsr &= ~RSTCSR_WOVF;
	else if ((data >> 8) == 0xa5)
		m_rstcsr = (data & RSTCSR_RSTE) | 0x1f;
}
