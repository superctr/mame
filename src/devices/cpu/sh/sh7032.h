// license:BSD-3-Clause
// copyright-holders:Angelo Salese

// SH7032, sh1 variant

#ifndef MAME_CPU_SH_SH7032_H
#define MAME_CPU_SH_SH7032_H

#pragma once

#include "sh2.h"

class sh7032_device : public sh2_device
{
public:
	sh7032_device(const machine_config &mconfig, const char *_tag, device_t *_owner, uint32_t _clock);

protected:
	sh7032_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, address_map_constructor internal_map);

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	void sh7032_map(address_map &map) ATTR_COLD;

private:
	uint16_t sh7032_r(offs_t offset);
	void sh7032_w(offs_t offset, uint16_t data, uint16_t mem_mask = ~0);

	uint16_t m_sh7032_regs[0x200];
};

// SH7034, sh1 variant with on-chip mask ROM

class sh7034_device : public sh7032_device
{
public:
	sh7034_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

private:
	void sh7034_map(address_map &map) ATTR_COLD;
};

DECLARE_DEVICE_TYPE(SH7032, sh7032_device)
DECLARE_DEVICE_TYPE(SH7034, sh7034_device)

#endif // MAME_CPU_SH_SH7032_H
