// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    i80c196kb.cpp

    MCS96, 80C196KB: the c196 instruction set on the 8x9x peripherals

    Not emulated: the register windows selected by WSR, Timer2's
    extensions, the TI/RI/HSI4/T2 interrupts through the upper vectors,
    and NMI.

***************************************************************************/

#include "emu.h"
#include "i80c196kb.h"
#include "i8xc196d.h"

DEFINE_DEVICE_TYPE(I80C196KB, i80c196kb_device, "i80c196kb", "Intel 80C196KB")

i80c196kb_device::i80c196kb_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	i8x9x_device(mconfig, I80C196KB, tag, owner, clock, 16, address_map_constructor(FUNC(i80c196kb_device::kb_regs), this)),
	m_ioc2(0),
	m_extint1(false)
{
}

std::unique_ptr<util::disasm_interface> i80c196kb_device::create_disassembler()
{
	return std::make_unique<i8xc196_disassembler>();
}

void i80c196kb_device::device_start()
{
	i8x9x_device::device_start();
	cycles_scaling = 2;

	save_item(NAME(m_ioc2));
	save_item(NAME(m_extint1));
}

void i80c196kb_device::device_reset()
{
	i8x9x_device::device_reset();
	m_ioc2 = 0;
}

void i80c196kb_device::kb_regs(address_map &map)
{
	internal_regs(map);
	map(0x0b, 0x0b).w(FUNC(i80c196kb_device::ioc2_w));
	map(0x12, 0x12).rw(FUNC(i80c196kb_device::int_pending1_r), FUNC(i80c196kb_device::int_pending1_w));
	map(0x13, 0x13).rw(FUNC(i80c196kb_device::int_mask1_r), FUNC(i80c196kb_device::int_mask1_w));
	map(0x14, 0x14).rw(FUNC(i80c196kb_device::wsr_r), FUNC(i80c196kb_device::wsr_w));
}

void i80c196kb_device::ioc2_w(u8 data)
{
	m_ioc2 = data;
}

u8 i80c196kb_device::wsr_r()
{
	return wsr;
}

void i80c196kb_device::wsr_w(u8 data)
{
	if((data & 0x7f) && !machine().side_effects_disabled())
		logerror("register window %d selected, not emulated (%04x)\n", data & 0x7f, PPC);
	wsr = data;
}

void i80c196kb_device::execute_set_input(int linenum, int state)
{
	if(linenum == EXTINT_LINE) {
		if(!m_extint1 && state) {
			pending_irq1 |= 0x20;
			check_irq();
		}
		m_extint1 = state;
	}
	i8x9x_device::execute_set_input(linenum, state);
}

void i80c196kb_device::do_exec_partial()
{
}

#include "cpu/mcs96/i80c196kb.hxx"
