// license:BSD-3-Clause
// copyright-holders:Olivier Galibert

// Base for SH microcontrollers with on-chip peripheral modules

#include "emu.h"
#include "sh_mcu.h"

sh_mcu_device::sh_mcu_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock, int cpu_type, address_map_constructor internal_map, int addrlines, u32 address_mask) :
	sh2_device(mconfig, type, tag, owner, clock, cpu_type, internal_map, addrlines, address_mask)
{
}

void sh_mcu_device::device_start()
{
	sh2_device::device_start();

	m_event_timer = timer_alloc(FUNC(sh_mcu_device::event_timer_tick), this);
}

void sh_mcu_device::set_internal_interrupt(int level, u32 vector)
{
	m_sh2_state->internal_irq_level = level;
	m_internal_irq_vector = vector;
	m_test_irq = 1;
}

void sh_mcu_device::internal_update()
{
	internal_update(current_cycles());
}

void sh_mcu_device::add_event(u64 &event_time, u64 new_event)
{
	if(!new_event)
		return;
	if(!event_time || event_time > new_event)
		event_time = new_event;
}

void sh_mcu_device::recompute_timer(u64 event_time)
{
	if(!event_time) {
		m_event_timer->adjust(attotime::never);
		return;
	}

	m_event_timer->adjust(attotime::from_ticks(2*event_time + 1, 2*clock()) - machine().time());
}

TIMER_CALLBACK_MEMBER(sh_mcu_device::event_timer_tick)
{
	internal_update();
}
