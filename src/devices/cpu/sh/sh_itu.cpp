// license:BSD-3-Clause
// copyright-holders:Olivier Galibert, superctr
/***************************************************************************

    sh_itu.cpp

    SH 16-bit integrated timer pulse unit

***************************************************************************/

#include "emu.h"
#include "sh_itu.h"

#include "sh_intc.h"
#include "sh_mcu.h"

#define LOG_REGS (1U << 1)
#define LOG_CLOCK (1U << 2)

#define VERBOSE (0)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(SH_ITU, sh_itu_device, "sh_itu", "SH 16-bit integrated timer pulse unit")
DEFINE_DEVICE_TYPE(SH_ITU_CHANNEL, sh_itu_channel_device, "sh_itu_channel", "SH 16-bit integrated timer pulse unit channel")

sh_itu_device::sh_itu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, SH_ITU, tag, owner, clock),
	m_cpu(*this, finder_base::DUMMY_TAG),
	m_timer_channel(*this, "%u", 0)
{
}

void sh_itu_device::device_start()
{
	save_item(NAME(m_tstr));
	save_item(NAME(m_tsnc));
	save_item(NAME(m_tmdr));
	save_item(NAME(m_tfcr));
	save_item(NAME(m_tocr));
}

void sh_itu_device::device_reset()
{
	m_tstr = 0;
	m_tsnc = 0;
	m_tmdr = 0x80;
	m_tfcr = 0;
	m_tocr = 0;
}

u8 sh_itu_device::tstr_r()
{
	return m_tstr | 0xe0;
}

void sh_itu_device::tstr_w(u8 data)
{
	m_tstr = data & 0x1f;
	LOGMASKED(LOG_REGS, "tstr_w %02x\n", m_tstr);
	for(int i = 0; i != 5; i++)
		m_timer_channel[i]->set_enable(BIT(m_tstr, i));
}

u8 sh_itu_device::tsnc_r()
{
	return m_tsnc | 0xe0;
}

void sh_itu_device::tsnc_w(u8 data)
{
	m_tsnc = data & 0x1f;
	LOGMASKED(LOG_REGS, "tsnc_w %02x\n", m_tsnc);
}

u8 sh_itu_device::tmdr_r()
{
	return m_tmdr | 0x80;
}

void sh_itu_device::tmdr_w(u8 data)
{
	m_tmdr = data & 0x7f;
	LOGMASKED(LOG_REGS, "tmdr_w %02x\n", m_tmdr);
}

u8 sh_itu_device::tfcr_r()
{
	return m_tfcr | 0xc0;
}

void sh_itu_device::tfcr_w(u8 data)
{
	m_tfcr = data & 0x3f;
	LOGMASKED(LOG_REGS, "tfcr_w %02x\n", m_tfcr);
}

u8 sh_itu_device::tocr_r()
{
	return m_tocr | 0x7f;
}

void sh_itu_device::tocr_w(u8 data)
{
	m_tocr = data & 0x80;
	LOGMASKED(LOG_REGS, "tocr_w %02x\n", m_tocr);
}


sh_itu_channel_device::sh_itu_channel_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, SH_ITU_CHANNEL, tag, owner, clock),
	m_cpu(*this, finder_base::DUMMY_TAG),
	m_intc(*this, finder_base::DUMMY_TAG)
{
}

void sh_itu_channel_device::device_start()
{
	m_channel_active = false;
	device_reset();

	save_item(NAME(m_gr_clearing));
	save_item(NAME(m_tcr));
	save_item(NAME(m_tior));
	save_item(NAME(m_tier));
	save_item(NAME(m_tsr));
	save_item(NAME(m_clock_type));
	save_item(NAME(m_clock_divider));
	save_item(NAME(m_tcnt));
	save_item(NAME(m_gr));
	save_item(NAME(m_br));
	save_item(NAME(m_last_clock_update));
	save_item(NAME(m_event_time));
	save_item(NAME(m_phase));
	save_item(NAME(m_counter_cycle));
	save_item(NAME(m_channel_active));
}

void sh_itu_channel_device::device_reset()
{
	m_gr_clearing = GR_CLEAR_NONE;
	m_tcr = 0;
	m_tior = 0x08;
	m_tier = 0;
	m_tsr = 0;
	m_clock_type = DIV_1;
	m_clock_divider = 0;
	m_tcnt = 0;
	std::fill(m_gr.begin(), m_gr.end(), 0xffff);
	std::fill(m_br.begin(), m_br.end(), 0xffff);
	m_last_clock_update = 0;
	m_event_time = 0;
	m_phase = 0;
	m_counter_cycle = 0x10000;
}

u8 sh_itu_channel_device::tcr_r()
{
	return m_tcr;
}

void sh_itu_channel_device::tcr_w(u8 data)
{
	update_counter();
	m_tcr = data & 0x7f;
	LOGMASKED(LOG_REGS, "tcr_w %02x\n", m_tcr);

	switch(m_tcr & 0x60) {
	case 0x00:
		m_gr_clearing = GR_CLEAR_NONE;
		break;
	case 0x20:
		m_gr_clearing = 0;
		break;
	case 0x40:
		m_gr_clearing = 1;
		break;
	case 0x60:
		m_gr_clearing = GR_CLEAR_SYNC;
		break;
	}

	if((m_tcr & 4) == 0) {
		m_clock_type = DIV_1;
		m_clock_divider = m_tcr & 3;
		LOGMASKED(LOG_CLOCK, "clock divider %d\n", 1 << m_clock_divider);
		m_phase = 0;
	} else {
		m_clock_type = INPUT_A + (m_tcr & 3);
		m_clock_divider = 0;
		m_phase = 0;
		LOGMASKED(LOG_CLOCK, "counting input %c\n", 'a' + (m_tcr & 3));
	}
	recalc_event();
}

u8 sh_itu_channel_device::tior_r()
{
	return m_tior;
}

void sh_itu_channel_device::tior_w(u8 data)
{
	m_tior = data;
	LOGMASKED(LOG_REGS, "tior_w %02x\n", m_tior);
}

u8 sh_itu_channel_device::tier_r()
{
	return m_tier | 0xf8;
}

void sh_itu_channel_device::tier_w(u8 data)
{
	update_counter();
	m_tier = data & 7;
	LOGMASKED(LOG_REGS, "tier_w %02x\n", m_tier);
	recalc_event();
}

u8 sh_itu_channel_device::tsr_r()
{
	if(!machine().side_effects_disabled())
		update_counter();
	return m_tsr | 0xf8;
}

void sh_itu_channel_device::tsr_w(u8 data)
{
	update_counter();
	m_tsr &= data & 7;
	recalc_event();
}

u16 sh_itu_channel_device::tcnt_r()
{
	if(!machine().side_effects_disabled())
		update_counter();
	return m_tcnt;
}

void sh_itu_channel_device::tcnt_w(offs_t, u16 data, u16 mem_mask)
{
	update_counter();
	COMBINE_DATA(&m_tcnt);
	recalc_event();
}

u16 sh_itu_channel_device::gr_r(offs_t reg)
{
	return m_gr[reg];
}

void sh_itu_channel_device::gr_w(offs_t reg, u16 data, u16 mem_mask)
{
	update_counter();
	COMBINE_DATA(&m_gr[reg]);
	recalc_event();
}

u16 sh_itu_channel_device::br_r(offs_t reg)
{
	return m_br[reg];
}

void sh_itu_channel_device::br_w(offs_t reg, u16 data, u16 mem_mask)
{
	COMBINE_DATA(&m_br[reg]);
}

void sh_itu_channel_device::set_enable(bool enable)
{
	if(m_channel_active == enable)
		return;
	update_counter();
	m_channel_active = enable;
	recalc_event();
}

u64 sh_itu_channel_device::internal_update(u64 current_time)
{
	while(m_event_time && current_time >= m_event_time) {
		update_counter(m_event_time);
		recalc_event(m_event_time);
	}

	return m_event_time;
}

void sh_itu_channel_device::recalc_event(u64 cur_time)
{
	if(!m_channel_active) {
		m_event_time = 0;
		return;
	}

	bool update_cpu = cur_time == 0;
	u64 old_event_time = m_event_time;

	if(m_clock_type != DIV_1) {
		m_event_time = 0;
		if(old_event_time && update_cpu)
			m_cpu->internal_update();
		return;
	}

	if(!cur_time)
		cur_time = m_cpu->current_cycles();

	u32 event_delay = 0xffffffff;
	if(m_gr_clearing >= 0)
		m_counter_cycle = m_gr[m_gr_clearing] + 1;
	else
		m_counter_cycle = 0x10000;

	if((m_tier & IRQ_V) && (m_counter_cycle == 0x10000 || m_tcnt >= m_counter_cycle))
		event_delay = 0x10000 - m_tcnt;

	for(int i = 0; i != 2; i++)
		if(BIT(m_tier, i)) {
			u32 new_delay = 0xffffffff;
			u16 cmp = m_gr[i] + 1;
			if(cmp > m_tcnt) {
				if(m_tcnt >= m_counter_cycle || cmp <= m_counter_cycle)
					new_delay = cmp - m_tcnt;
			} else if(cmp <= m_counter_cycle) {
				if(m_tcnt < m_counter_cycle)
					new_delay = (m_counter_cycle - m_tcnt) + cmp;
				else
					new_delay = (0x10000 - m_tcnt) + cmp;
			}

			if(event_delay > new_delay)
				event_delay = new_delay;
		}

	if(event_delay != 0xffffffff)
		m_event_time = ((((cur_time + (1ULL << m_clock_divider) - m_phase) >> m_clock_divider) + event_delay - 1) << m_clock_divider) + m_phase;
	else
		m_event_time = 0;

	if(old_event_time != m_event_time && update_cpu)
		m_cpu->internal_update();
}

void sh_itu_channel_device::update_counter(u64 cur_time)
{
	if(m_clock_type != DIV_1)
		return;

	if(!cur_time)
		cur_time = m_cpu->current_cycles();

	if(!m_channel_active) {
		m_last_clock_update = cur_time;
		return;
	}

	u64 base_time = m_last_clock_update;
	m_last_clock_update = cur_time;
	u64 new_time = cur_time;
	if(m_clock_divider) {
		base_time = (base_time + m_phase) >> m_clock_divider;
		new_time = (new_time + m_phase) >> m_clock_divider;
	}
	if(new_time == base_time)
		return;

	u16 prev = m_tcnt;
	u64 delta = new_time - base_time;
	u64 tt = m_tcnt + delta;

	if(prev >= m_counter_cycle) {
		if(tt >= 0x10000)
			m_tcnt = (tt - 0x10000) % m_counter_cycle;
		else
			m_tcnt = tt;
	} else
		m_tcnt = tt % m_counter_cycle;

	for(int i = 0; i != 2; i++) {
		u16 cmp = m_gr[i] + 1;
		bool match = m_tcnt == cmp || (tt == cmp && tt == m_counter_cycle);
		if(!match) {
			if(prev >= m_counter_cycle)
				match = (cmp > prev && tt >= cmp) || (cmp <= m_counter_cycle && m_tcnt < m_counter_cycle && (delta - (0x10000 - prev)) >= cmp);
			else if(cmp <= m_counter_cycle)
				match = delta >= m_counter_cycle || (prev < cmp && tt >= cmp) || (m_tcnt <= prev && m_tcnt >= cmp);
		}

		if(match) {
			m_tsr |= 1 << i;
			if(BIT(m_tier, i))
				m_intc->internal_interrupt(m_interrupt[i]);
		}
	}

	if(tt >= 0x10000 && (m_counter_cycle == 0x10000 || prev >= m_counter_cycle)) {
		m_tsr |= IRQ_V;
		if(m_tier & IRQ_V)
			m_intc->internal_interrupt(m_interrupt[2]);
	}
}
