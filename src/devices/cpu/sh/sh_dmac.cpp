// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
/***************************************************************************

    sh_dmac.cpp

    SH DMA controller

    Dual address transfers by auto-request and by external request on the
    DREQ pins (level or falling edge sampling), one unit every two clocks,
    the transfer end flag and its interrupt.  The on-chip module requests
    (MTU, A/D, SCI) are not connected yet.

***************************************************************************/

#include "emu.h"
#include "sh_dmac.h"

#include "sh_intc.h"
#include "sh_mcu.h"

#define LOG_REGS (1U << 1)
#define LOG_XFER (1U << 2)

#define VERBOSE 0
#include "logmacro.h"

DEFINE_DEVICE_TYPE(SH_DMAC, sh_dmac_device, "sh_dmac", "SH DMA controller")
DEFINE_DEVICE_TYPE(SH_DMAC_CHANNEL, sh_dmac_channel_device, "sh_dmac_channel", "SH DMA controller channel")

sh_dmac_device::sh_dmac_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, SH_DMAC, tag, owner, clock),
	m_cpu(*this, finder_base::DUMMY_TAG)
{
}

void sh_dmac_device::device_start()
{
	save_item(NAME(m_dmaor));
}

void sh_dmac_device::device_reset()
{
	m_dmaor = 0;
}

u16 sh_dmac_device::dmaor_r()
{
	return m_dmaor;
}

// the NMI and address error flags only clear, by a write of 0 after a read of 1
void sh_dmac_device::dmaor_w(offs_t, u16 data, u16 mem_mask)
{
	u16 value = m_dmaor;
	COMBINE_DATA(&value);
	m_dmaor = (value & 0x0301) | (m_dmaor & value & 0x0006);
	LOGMASKED(LOG_REGS, "dmaor_w %04x\n", m_dmaor);
	for (int i = 0; i < 4; i++) {
		auto *channel = subdevice<sh_dmac_channel_device>(util::string_format("%d", i).c_str());
		if (channel)
			channel->dmaor_changed();
	}
}


sh_dmac_channel_device::sh_dmac_channel_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, SH_DMAC_CHANNEL, tag, owner, clock),
	m_cpu(*this, finder_base::DUMMY_TAG),
	m_intc(*this, finder_base::DUMMY_TAG),
	m_dmac(*this, finder_base::DUMMY_TAG),
	m_id(0)
{
}

void sh_dmac_channel_device::device_start()
{
	m_timer = timer_alloc(FUNC(sh_dmac_channel_device::tick), this);

	save_item(NAME(m_sar));
	save_item(NAME(m_dar));
	save_item(NAME(m_dmatcr));
	save_item(NAME(m_chcr));
	save_item(NAME(m_dreq));
	save_item(NAME(m_edge));
}

void sh_dmac_channel_device::device_reset()
{
	m_sar = 0;
	m_dar = 0;
	m_dmatcr = 0;
	m_chcr = 0;
	m_dreq = false;
	m_edge = false;
	m_timer->adjust(attotime::never);
}

u32 sh_dmac_channel_device::sar_r()
{
	return m_sar;
}

void sh_dmac_channel_device::sar_w(offs_t, u32 data, u32 mem_mask)
{
	COMBINE_DATA(&m_sar);
	LOGMASKED(LOG_REGS, "sar_w %08x\n", m_sar);
}

u32 sh_dmac_channel_device::dar_r()
{
	return m_dar;
}

void sh_dmac_channel_device::dar_w(offs_t, u32 data, u32 mem_mask)
{
	COMBINE_DATA(&m_dar);
	LOGMASKED(LOG_REGS, "dar_w %08x\n", m_dar);
}

u32 sh_dmac_channel_device::dmatcr_r()
{
	return m_dmatcr;
}

void sh_dmac_channel_device::dmatcr_w(offs_t, u32 data, u32 mem_mask)
{
	COMBINE_DATA(&m_dmatcr);
	m_dmatcr &= 0xffff;
	LOGMASKED(LOG_REGS, "dmatcr_w %08x\n", m_dmatcr);
}

u32 sh_dmac_channel_device::chcr_r()
{
	return m_chcr;
}

// the transfer end flag only clears, by a write of 0 after a read of 1
void sh_dmac_channel_device::chcr_w(offs_t, u32 data, u32 mem_mask)
{
	u32 value = m_chcr;
	COMBINE_DATA(&value);
	m_chcr = (value & 0x001fff7d) | (m_chcr & value & CHCR_TE);
	LOGMASKED(LOG_REGS, "chcr_w %08x\n", m_chcr);
	request_check();
}

void sh_dmac_channel_device::dmaor_changed()
{
	request_check();
}

bool sh_dmac_channel_device::armed() const
{
	return m_dmac->transfer_allowed() && (m_chcr & (CHCR_DE | CHCR_TE)) == CHCR_DE;
}

// the pin is sampled on its falling edge or its low level, as DS says
void sh_dmac_channel_device::dreq_w(int state)
{
	const bool asserted = state != 0;
	const bool edge = asserted && !m_dreq;
	m_dreq = asserted;
	m_edge |= edge;
	if (!external() || !armed())
		return;
	if (edge || (asserted && !(m_chcr & CHCR_DS)))
		request_check();
}

void sh_dmac_channel_device::request_check()
{
	if (!armed()) {
		m_timer->adjust(attotime::never);
		return;
	}
	bool wanted = false;
	switch (resource()) {
	case RS_AUTO:
		wanted = true;
		break;
	case RS_EXTERNAL_DUAL:
	case RS_EXTERNAL_SINGLE_TO_DEVICE:
	case RS_EXTERNAL_SINGLE_FROM_DEVICE:
		wanted = m_dreq;
		break;
	default:
		logerror("channel %d: on-chip module request %d not supported\n", m_id, resource());
		break;
	}
	if (wanted && m_timer->remaining().is_never())
		m_timer->adjust(attotime::from_ticks(2, m_cpu->clock()));
}

TIMER_CALLBACK_MEMBER(sh_dmac_channel_device::tick)
{
	if (!armed())
		return;
	m_edge = false;
	transfer_unit();
	// a level sampled request keeps going while the pin is held; an edge sampled
	// one takes one unit a request, including an edge made during the unit
	if (!armed())
		return;
	if (resource() == RS_AUTO || (external() && m_dreq && (!(m_chcr & CHCR_DS) || m_edge)))
		m_timer->adjust(attotime::from_ticks(2, m_cpu->clock()));
	else if (external())
		m_dreq = false;
}

void sh_dmac_channel_device::transfer_unit()
{
	address_space &space = m_cpu->space(AS_PROGRAM);
	const int size = 1 << BIT(m_chcr, 3, 2);
	const int sm = BIT(m_chcr, 12, 2), dm = BIT(m_chcr, 14, 2);

	LOGMASKED(LOG_XFER, "channel %d: %08x -> %08x size %d remaining %04x\n", m_id, m_sar, m_dar, size, m_dmatcr);

	switch (size) {
	case 1: space.write_byte(m_dar, space.read_byte(m_sar)); break;
	case 2: space.write_word(m_dar, space.read_word(m_sar)); break;
	case 4: space.write_dword(m_dar, space.read_dword(m_sar)); break;
	}

	if (sm == 1)
		m_sar += size;
	else if (sm == 2)
		m_sar -= size;
	if (dm == 1)
		m_dar += size;
	else if (dm == 2)
		m_dar -= size;

	m_dmatcr = (m_dmatcr - 1) & 0xffff;
	if (m_dmatcr == 0) {
		m_chcr |= CHCR_TE;
		if (m_chcr & CHCR_IE)
			m_intc->internal_interrupt(72 + 4 * m_id);
	}
}
