// license:BSD-3-Clause
// copyright-holders:Angelo Salese

#include "emu.h"
#include "sh7032.h"

DEFINE_DEVICE_TYPE(SH7032,  sh7032_device,  "sh7032",  "Hitachi SH-1 (SH7032)")
DEFINE_DEVICE_TYPE(SH7034,  sh7034_device,  "sh7034",  "Hitachi SH-1 (SH7034)")


sh7032_device::sh7032_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: sh7032_device(mconfig, SH7032, tag, owner, clock, address_map_constructor(FUNC(sh7032_device::sh7032_map), this))
{
}

sh7032_device::sh7032_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, address_map_constructor internal_map)
	: sh2_device(mconfig, type, tag, owner, clock, CPU_TYPE_SH1, internal_map, 28, 0xc7ffffff)
{
}

void sh7032_device::device_start()
{
	sh2_device::device_start();

	save_item(NAME(m_sh7032_regs));
}

void sh7032_device::device_reset()
{
	sh2_device::device_reset();

	std::fill(std::begin(m_sh7032_regs), std::end(m_sh7032_regs), 0);
}

void sh7032_device::sh7032_map(address_map &map)
{
//  fall-back
	map(0x05fffe00, 0x05ffffff).rw(FUNC(sh7032_device::sh7032_r), FUNC(sh7032_device::sh7032_w)); // SH-7032H internal i/o
}

uint16_t sh7032_device::sh7032_r(offs_t offset)
{
	return m_sh7032_regs[offset];
}

void sh7032_device::sh7032_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	COMBINE_DATA(&m_sh7032_regs[offset]);
}


sh7034_device::sh7034_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: sh7032_device(mconfig, SH7034, tag, owner, clock, address_map_constructor(FUNC(sh7034_device::sh7034_map), this))
{
	m_isdrc = false;
}

void sh7034_device::sh7034_map(address_map &map)
{
	map(0x00000000, 0x0000ffff).rom().region(DEVICE_SELF, 0);
	map(0x07fff000, 0x07ffffff).ram();

	sh7032_map(map);
}
