// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
/***************************************************************************

    sh_mcu.h

    Base for SH microcontrollers with on-chip peripheral modules

***************************************************************************/

#ifndef MAME_CPU_SH_SH_MCU_H
#define MAME_CPU_SH_SH_MCU_H

#pragma once

#include "sh2.h"

class sh_mcu_device : public sh2_device
{
public:
	u64 current_cycles() const { return machine().time().as_ticks(clock()); }

	void set_internal_interrupt(int level, u32 vector);
	void internal_update();

	virtual void do_sci_tx(int sci, int state) {}
	virtual void do_sci_clk(int sci, int state) {}
	virtual u16 do_read_adc(int port) { return 0; }
	virtual u16 do_read_port16(int port) { return 0xffff; }
	virtual void do_write_port16(int port, u16 data, u16 ddr) {}
	virtual u32 do_read_port32(int port) { return 0xffffffff; }
	virtual void do_write_port32(int port, u32 data, u32 ddr) {}

protected:
	sh_mcu_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock, int cpu_type, address_map_constructor internal_map, int addrlines, u32 address_mask);

	virtual void device_start() override ATTR_COLD;

	virtual void internal_update(u64 current_time) = 0;

	void add_event(u64 &event_time, u64 new_event);
	void recompute_timer(u64 event_time);

private:
	emu_timer *m_event_timer;

	TIMER_CALLBACK_MEMBER(event_timer_tick);
};

#endif // MAME_CPU_SH_SH_MCU_H
