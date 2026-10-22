// license:BSD-3-Clause
// copyright-holders:superctr
/****************************************************************************

    Roland UIPC: the host side of the M37640/M37641 USB controller's bus
    interface unit, as the SC-8820, SC-8850, SD-80, SD-90 and XV-5050 reach
    it.  The controller's program is an undumped mask ROM, so only its
    power-on announcement is played back.

****************************************************************************/

#ifndef MAME_ROLAND_ROLAND_UIPC_H
#define MAME_ROLAND_ROLAND_UIPC_H

#pragma once

#include <vector>


class roland_uipc_device : public device_t
{
public:
	static constexpr u16 IFSEL = 0x100;

	struct announce_byte
	{
		u8 status;
		u16 data;
	};

	roland_uipc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	void set_announcement(std::initializer_list<announce_byte> bytes) { m_announce = bytes; }

	// IFSEL0-IFSEL3, the controller's interface select pins
	auto ifsel_callback() { return m_ifsel_cb.bind(); }

	// host to controller: +0 data, +1 status
	u8 tx_r(offs_t offset);
	void tx_w(offs_t offset, u8 data);

	// controller to host: +0 data, +1 status
	u8 rx_r(offs_t offset);
	void rx_w(offs_t offset, u8 data);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	devcb_read8 m_ifsel_cb;
	std::vector<announce_byte> m_announce;
	u32 m_step;
};

DECLARE_DEVICE_TYPE(ROLAND_UIPC, roland_uipc_device)

#endif // MAME_ROLAND_ROLAND_UIPC_H
