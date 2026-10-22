// license:BSD-3-Clause
// copyright-holders:superctr
/****************************************************************************

    Roland UIPC (M37640/M37641 USB controller, host side)

    Two one-byte channels, each a data byte at +0 and a status byte at +1.
    Status bit 1 of the host-to-controller channel is set while the last
    byte written has not been taken; bit 0 of the controller-to-host channel
    is set while a byte waits there, and bits 4-7 are that byte's tag.

    Each firmware waits at power-on for the controller to announce itself
    before it goes on.  This plays the machine's announcement back, takes
    every byte the host writes at once, and then goes quiet.

****************************************************************************/

#include "emu.h"
#include "roland_uipc.h"

#define LOG_HANDSHAKE (1U << 1)
#define LOG_WRITE     (1U << 2)

#define VERBOSE (0)
#include "logmacro.h"


DEFINE_DEVICE_TYPE(ROLAND_UIPC, roland_uipc_device, "roland_uipc", "Roland UIPC USB controller (HLE)")

roland_uipc_device::roland_uipc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, ROLAND_UIPC, tag, owner, clock)
	, m_ifsel_cb(*this, 0)
	, m_step(0)
{
}

void roland_uipc_device::device_start()
{
	save_item(NAME(m_step));
}

void roland_uipc_device::device_reset()
{
	m_step = 0;
}

u8 roland_uipc_device::tx_r(offs_t offset)
{
	return 0x00;
}

void roland_uipc_device::tx_w(offs_t offset, u8 data)
{
	LOGMASKED(LOG_WRITE, "%s: tx %d = %02x\n", machine().describe_context(), offset, data);
}

u8 roland_uipc_device::rx_r(offs_t offset)
{
	if (m_step >= m_announce.size())
		return 0x00;

	const announce_byte &b = m_announce[m_step];
	if (BIT(offset, 0))
		return b.status;

	const u8 data = (b.data == IFSEL) ? m_ifsel_cb() : u8(b.data);
	if (!machine().side_effects_disabled())
	{
		LOGMASKED(LOG_HANDSHAKE, "%s: announcement byte %02x\n", machine().describe_context(), data);
		m_step++;
	}
	return data;
}

void roland_uipc_device::rx_w(offs_t offset, u8 data)
{
	LOGMASKED(LOG_WRITE, "%s: rx %d = %02x\n", machine().describe_context(), offset, data);
}
