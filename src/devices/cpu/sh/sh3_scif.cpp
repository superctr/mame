// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    SH-3 (SH7709) serial communication interface with FIFO: the IrDA
    channel and the SCIF channel, asynchronous mode.

    Sixteen-byte transmit and receive FIFOs with trigger levels, the
    status word's receive-error counts, the receive-data-ready timeout,
    and the four interrupt outputs (error, receive, break, transmit),
    each a level that the SCSCR enables gate.  Synchronous mode, the
    IrDA encoding, the clock pins and the loopback bit are not modelled.

***************************************************************************/

#include "emu.h"
#include "sh3_scif.h"


DEFINE_DEVICE_TYPE(SH3_SCIF, sh3_scif_device, "sh3_scif", "SH-3 SCIF")

sh3_scif_device::sh3_scif_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, SH3_SCIF, tag, owner, clock)
	, device_serial_interface(mconfig, *this)
	, m_txd_cb(*this)
	, m_eri_cb(*this)
	, m_rxi_cb(*this)
	, m_bri_cb(*this)
	, m_txi_cb(*this)
	, m_rx_timer(nullptr)
{
}


void sh3_scif_device::map(address_map &map)
{
	map(0x0, 0x0).rw(FUNC(sh3_scif_device::scsmr_r), FUNC(sh3_scif_device::scsmr_w));
	map(0x2, 0x2).rw(FUNC(sh3_scif_device::scbrr_r), FUNC(sh3_scif_device::scbrr_w));
	map(0x4, 0x4).rw(FUNC(sh3_scif_device::scscr_r), FUNC(sh3_scif_device::scscr_w));
	map(0x6, 0x6).w(FUNC(sh3_scif_device::scftdr_w));
	map(0x8, 0x9).rw(FUNC(sh3_scif_device::scssr_r), FUNC(sh3_scif_device::scssr_w));
	map(0xa, 0xa).r(FUNC(sh3_scif_device::scfrdr_r));
	map(0xc, 0xc).rw(FUNC(sh3_scif_device::scfcr_r), FUNC(sh3_scif_device::scfcr_w));
	map(0xe, 0xf).r(FUNC(sh3_scif_device::scfdr_r));
}


void sh3_scif_device::device_start()
{
	m_rx_timer = timer_alloc(FUNC(sh3_scif_device::rx_timeout), this);

	save_item(NAME(m_scsmr));
	save_item(NAME(m_scbrr));
	save_item(NAME(m_scscr));
	save_item(NAME(m_scfcr));
	save_item(NAME(m_scssr));
	save_item(NAME(m_per_count));
	save_item(NAME(m_fer_count));
	save_item(NAME(m_tx_busy));
	save_item(NAME(m_irq_state));
}

void sh3_scif_device::device_reset()
{
	m_scsmr = 0;
	m_scbrr = 0xff;
	m_scscr = 0;
	m_scfcr = 0;
	m_scssr = SSR_TEND | SSR_TDFE;
	m_per_count = 0;
	m_fer_count = 0;
	m_tx_busy = false;
	m_irq_state = 0;

	tx_flush();
	rx_flush();
	transmit_register_reset();
	receive_register_reset();
	update_format();
	m_txd_cb(1);
	update_interrupts();
}


//-------------------------------------------------
//  the line format and rate, from SCSMR and SCBRR
//-------------------------------------------------

void sh3_scif_device::update_format()
{
	const parity_t parity = (m_scsmr & SMR_PE) ? ((m_scsmr & SMR_OE) ? PARITY_ODD : PARITY_EVEN) : PARITY_NONE;
	set_data_frame(1, (m_scsmr & SMR_CHR) ? 7 : 8, parity, (m_scsmr & SMR_STOP) ? STOP_BITS_2 : STOP_BITS_1);

	const u32 divider = (32 << (2 * (m_scsmr & SMR_CKS))) * (m_scbrr + 1);
	set_rate(clock(), divider);
}

int sh3_scif_device::rx_trigger() const
{
	static const int trigger[4] = { 1, 4, 8, 14 };
	return trigger[(m_scfcr & FCR_RTRG) >> 6];
}

int sh3_scif_device::tx_trigger() const
{
	static const int trigger[4] = { 8, 4, 2, 1 };
	return trigger[(m_scfcr & FCR_TTRG) >> 4];
}


//-------------------------------------------------
//  the FIFOs
//-------------------------------------------------

void sh3_scif_device::tx_flush()
{
	while (!m_tx_fifo.empty())
		m_tx_fifo.dequeue();
}

void sh3_scif_device::rx_flush()
{
	while (!m_rx_fifo.empty())
		m_rx_fifo.dequeue();
	m_rx_timer->adjust(attotime::never);
}

void sh3_scif_device::tx_next()
{
	if ((m_scscr & SCR_TE) && !m_tx_busy && !m_tx_fifo.empty())
	{
		transmit_register_setup(m_tx_fifo.dequeue());
		m_tx_busy = true;
		update_status();
	}
}

void sh3_scif_device::tra_callback()
{
	m_txd_cb(transmit_register_get_data_bit());
}

void sh3_scif_device::tra_complete()
{
	m_tx_busy = false;
	if (m_tx_fifo.empty())
	{
		m_scssr |= SSR_TEND;
		update_status();
	}
	else
		tx_next();
}

void sh3_scif_device::rcv_complete()
{
	receive_register_extract();
	if (!(m_scscr & SCR_RE))
		return;

	const u8 data = get_received_char();
	if (is_receive_framing_error())
	{
		m_scssr |= SSR_ER | (data ? SSR_FER : SSR_BRK);
		if (data && m_fer_count < 15)
			m_fer_count++;
	}
	if (is_receive_parity_error())
	{
		m_scssr |= SSR_ER | SSR_PER;
		if (m_per_count < 15)
			m_per_count++;
	}
	if (!m_rx_fifo.full())
		m_rx_fifo.enqueue(data);

	// receive data ready: fewer bytes than the trigger and no more arriving
	// for fifteen bit times
	m_rx_timer->adjust(attotime::from_hz(clock()) * ((32 << (2 * (m_scsmr & SMR_CKS))) * (m_scbrr + 1) * 15));
	update_status();
}

TIMER_CALLBACK_MEMBER(sh3_scif_device::rx_timeout)
{
	if (!m_rx_fifo.empty() && m_rx_fifo.queue_length() < rx_trigger())
	{
		m_scssr |= SSR_DR;
		update_interrupts();
	}
}


//-------------------------------------------------
//  status and interrupts.  TDFE and RDF follow the FIFO counts against
//  their triggers; the other flags are set here and cleared by the host
//  writing them back as zero.
//-------------------------------------------------

void sh3_scif_device::update_status()
{
	if (m_tx_fifo.queue_length() <= tx_trigger())
		m_scssr |= SSR_TDFE;
	else
		m_scssr &= ~SSR_TDFE;

	if (m_rx_fifo.queue_length() >= rx_trigger())
		m_scssr |= SSR_RDF;
	else
		m_scssr &= ~SSR_RDF;

	update_interrupts();
}

void sh3_scif_device::update_interrupts()
{
	u8 state = 0;
	if ((m_scscr & SCR_RIE) && (m_scssr & SSR_ER))
		state |= 1;
	if ((m_scscr & SCR_RIE) && (m_scssr & (SSR_RDF | SSR_DR)))
		state |= 2;
	if ((m_scscr & SCR_RIE) && (m_scssr & SSR_BRK))
		state |= 4;
	if ((m_scscr & SCR_TIE) && (m_scssr & SSR_TDFE))
		state |= 8;

	const u8 changed = state ^ m_irq_state;
	m_irq_state = state;
	if (BIT(changed, 0))
		m_eri_cb(BIT(state, 0));
	if (BIT(changed, 1))
		m_rxi_cb(BIT(state, 1));
	if (BIT(changed, 2))
		m_bri_cb(BIT(state, 2));
	if (BIT(changed, 3))
		m_txi_cb(BIT(state, 3));
}


//-------------------------------------------------
//  registers
//-------------------------------------------------

u8 sh3_scif_device::scsmr_r()
{
	return m_scsmr;
}

void sh3_scif_device::scsmr_w(u8 data)
{
	m_scsmr = data;
	update_format();
}

u8 sh3_scif_device::scbrr_r()
{
	return m_scbrr;
}

void sh3_scif_device::scbrr_w(u8 data)
{
	m_scbrr = data;
	update_format();
}

u8 sh3_scif_device::scscr_r()
{
	return m_scscr;
}

void sh3_scif_device::scscr_w(u8 data)
{
	const u8 old = m_scscr;
	m_scscr = data;
	if ((data & SCR_TE) && !(old & SCR_TE))
		tx_next();
	if (!(data & SCR_RE) && (old & SCR_RE))
		receive_register_reset();
	update_interrupts();
}

void sh3_scif_device::scftdr_w(u8 data)
{
	if (!m_tx_fifo.full())
		m_tx_fifo.enqueue(data);
	m_scssr &= ~SSR_TEND;
	update_status();
	tx_next();
}

u16 sh3_scif_device::scssr_r()
{
	return m_scssr | (m_per_count << 12) | (m_fer_count << 8);
}

void sh3_scif_device::scssr_w(offs_t offset, u16 data, u16 mem_mask)
{
	constexpr u16 clearable = SSR_ER | SSR_TEND | SSR_BRK | SSR_FER | SSR_PER | SSR_DR;
	m_scssr &= data | ~clearable;
	if (!(m_scssr & (SSR_FER | SSR_PER)))
	{
		m_fer_count = 0;
		m_per_count = 0;
	}
	update_status();
}

u8 sh3_scif_device::scfrdr_r()
{
	u8 data = 0;
	if (!m_rx_fifo.empty())
	{
		data = m_rx_fifo.dequeue();
		if (m_rx_fifo.empty())
		{
			m_scssr &= ~SSR_DR;
			m_rx_timer->adjust(attotime::never);
		}
		update_status();
	}
	return data;
}

u8 sh3_scif_device::scfcr_r()
{
	return m_scfcr;
}

void sh3_scif_device::scfcr_w(u8 data)
{
	m_scfcr = data;
	if (data & FCR_TFRST)
		tx_flush();
	if (data & FCR_RFRST)
		rx_flush();
	update_status();
}

u16 sh3_scif_device::scfdr_r()
{
	return (m_tx_fifo.queue_length() << 8) | m_rx_fifo.queue_length();
}
