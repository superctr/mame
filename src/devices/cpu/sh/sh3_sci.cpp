// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    SH-3 serial communication interface (SCI), channel 0 of the SH7708
    and SH7709 families, asynchronous mode.

    Double-buffered transmitter and receiver on the internal baud rate
    generator, the status flags with their read-before-clear rule, and the
    four interrupt outputs (receive error, receive data full, transmit
    data empty, transmit end), each a level that the SCSCR enables gate.
    Synchronous mode, the multiprocessor format, the SCK pin and the smart
    card interface are not modelled.  The SH-3's SCI has no DMAC request.

***************************************************************************/

#include "emu.h"
#include "sh3_sci.h"

#define LOG_UNIMPL  (1U << 1)

#define VERBOSE (LOG_UNIMPL)
#include "logmacro.h"


DEFINE_DEVICE_TYPE(SH3_SCI, sh3_sci_device, "sh3_sci", "SH-3 SCI")

sh3_sci_device::sh3_sci_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, SH3_SCI, tag, owner, clock)
	, device_serial_interface(mconfig, *this)
	, m_txd_cb(*this)
	, m_eri_cb(*this)
	, m_rxi_cb(*this)
	, m_txi_cb(*this)
	, m_tei_cb(*this)
{
}


void sh3_sci_device::map(address_map &map)
{
	map(0x0, 0x0).rw(FUNC(sh3_sci_device::scsmr_r), FUNC(sh3_sci_device::scsmr_w));
	map(0x2, 0x2).rw(FUNC(sh3_sci_device::scbrr_r), FUNC(sh3_sci_device::scbrr_w));
	map(0x4, 0x4).rw(FUNC(sh3_sci_device::scscr_r), FUNC(sh3_sci_device::scscr_w));
	map(0x6, 0x6).rw(FUNC(sh3_sci_device::sctdr_r), FUNC(sh3_sci_device::sctdr_w));
	map(0x8, 0x8).rw(FUNC(sh3_sci_device::scssr_r), FUNC(sh3_sci_device::scssr_w));
	map(0xa, 0xa).r(FUNC(sh3_sci_device::scrdr_r));
	map(0xc, 0xc).rw(FUNC(sh3_sci_device::scscmr_r), FUNC(sh3_sci_device::scscmr_w));
}


void sh3_sci_device::device_start()
{
	save_item(NAME(m_scsmr));
	save_item(NAME(m_scbrr));
	save_item(NAME(m_scscr));
	save_item(NAME(m_sctdr));
	save_item(NAME(m_scssr));
	save_item(NAME(m_scrdr));
	save_item(NAME(m_scscmr));
	save_item(NAME(m_ssr_read));
	save_item(NAME(m_tx_busy));
	save_item(NAME(m_irq_state));
}

void sh3_sci_device::device_reset()
{
	m_scsmr = 0;
	m_scbrr = 0xff;
	m_scscr = 0;
	m_sctdr = 0xff;
	m_scssr = SSR_TDRE | SSR_TEND;
	m_scrdr = 0;
	m_scscmr = 0;
	m_ssr_read = 0;
	m_tx_busy = false;
	m_irq_state = 0;

	transmit_register_reset();
	receive_register_reset();
	update_format();
	m_txd_cb(1);
}

void sh3_sci_device::device_clock_changed()
{
	update_format();
}


//-------------------------------------------------
//  the line format and rate, from SCSMR and SCBRR
//-------------------------------------------------

void sh3_sci_device::update_format()
{
	if (m_scsmr & SMR_CA)
		LOGMASKED(LOG_UNIMPL, "synchronous mode is not supported\n");
	if (m_scsmr & SMR_MP)
		LOGMASKED(LOG_UNIMPL, "multiprocessor format is not supported\n");
	if (m_scscr & 0x02)
		LOGMASKED(LOG_UNIMPL, "external clock is not supported\n");

	const parity_t parity = (m_scsmr & SMR_PE) ? ((m_scsmr & SMR_OE) ? PARITY_ODD : PARITY_EVEN) : PARITY_NONE;
	set_data_frame(1, (m_scsmr & SMR_CHR) ? 7 : 8, parity, (m_scsmr & SMR_STOP) ? STOP_BITS_2 : STOP_BITS_1);

	if (clock())
		set_rate(clock(), (32 << (2 * (m_scsmr & SMR_CKS))) * (m_scbrr + 1));
}


//-------------------------------------------------
//  the transmitter: SCTDR moves into the shift
//  register once TDRE has been cleared, setting
//  TDRE again; with TDRE still set when the frame
//  ends, TEND is set and the line idles at mark
//-------------------------------------------------

void sh3_sci_device::tx_start()
{
	if (!(m_scscr & SCR_TE) || (m_scssr & SSR_TDRE) || m_tx_busy)
		return;

	transmit_register_setup(m_sctdr);
	m_tx_busy = true;
	m_scssr |= SSR_TDRE;
	update_interrupts();
}

void sh3_sci_device::tra_callback()
{
	m_txd_cb(transmit_register_get_data_bit());
}

void sh3_sci_device::tra_complete()
{
	m_tx_busy = false;
	if (m_scssr & SSR_TDRE)
	{
		m_scssr |= SSR_TEND;
		update_interrupts();
	}
	else
		tx_start();
}


//-------------------------------------------------
//  the receiver: a frame is lost to an overrun
//  while RDRF is set, and nothing is received at
//  all while an error flag is set
//-------------------------------------------------

void sh3_sci_device::rcv_complete()
{
	receive_register_extract();
	if (!(m_scscr & SCR_RE) || (m_scssr & SSR_ERRORS))
		return;

	if (m_scssr & SSR_RDRF)
		m_scssr |= SSR_ORER;
	else
	{
		m_scrdr = get_received_char();
		if (is_receive_framing_error())
			m_scssr |= SSR_FER;
		if (is_receive_parity_error())
			m_scssr |= SSR_PER;
		if (!(m_scssr & SSR_ERRORS))
			m_scssr |= SSR_RDRF;
	}
	update_interrupts();
}


//-------------------------------------------------
//  interrupts
//-------------------------------------------------

void sh3_sci_device::update_interrupts()
{
	u8 state = 0;
	if ((m_scscr & SCR_RIE) && (m_scssr & SSR_ERRORS))
		state |= 1;
	if ((m_scscr & SCR_RIE) && (m_scssr & SSR_RDRF))
		state |= 2;
	if ((m_scscr & SCR_TIE) && (m_scssr & SSR_TDRE))
		state |= 4;
	if ((m_scscr & SCR_TEIE) && (m_scssr & SSR_TEND))
		state |= 8;

	const u8 changed = state ^ m_irq_state;
	m_irq_state = state;
	if (BIT(changed, 0))
		m_eri_cb(BIT(state, 0));
	if (BIT(changed, 1))
		m_rxi_cb(BIT(state, 1));
	if (BIT(changed, 2))
		m_txi_cb(BIT(state, 2));
	if (BIT(changed, 3))
		m_tei_cb(BIT(state, 3));
}


//-------------------------------------------------
//  registers
//-------------------------------------------------

u8 sh3_sci_device::scsmr_r()
{
	return m_scsmr;
}

void sh3_sci_device::scsmr_w(u8 data)
{
	m_scsmr = data;
	update_format();
}

u8 sh3_sci_device::scbrr_r()
{
	return m_scbrr;
}

void sh3_sci_device::scbrr_w(u8 data)
{
	m_scbrr = data;
	update_format();
}

u8 sh3_sci_device::scscr_r()
{
	return m_scscr;
}

void sh3_sci_device::scscr_w(u8 data)
{
	const u8 old = m_scscr;
	m_scscr = data;

	if ((old & SCR_TE) && !(data & SCR_TE))
	{
		transmit_register_reset();
		m_tx_busy = false;
		m_scssr |= SSR_TDRE | SSR_TEND;
		m_txd_cb(1);
	}
	if ((old & SCR_RE) && !(data & SCR_RE))
		receive_register_reset();
	if ((old ^ data) & SCR_CKE)
		update_format();
	update_interrupts();
}

u8 sh3_sci_device::sctdr_r()
{
	return m_sctdr;
}

void sh3_sci_device::sctdr_w(u8 data)
{
	m_sctdr = data;
}

u8 sh3_sci_device::scssr_r()
{
	if (!machine().side_effects_disabled())
		m_ssr_read |= m_scssr & SSR_CLEARABLE;
	return m_scssr;
}

void sh3_sci_device::scssr_w(u8 data)
{
	const u8 cleared = m_ssr_read & ~data & SSR_CLEARABLE;
	m_ssr_read &= ~cleared;
	m_scssr = (m_scssr & ~(cleared | SSR_MPBT)) | (data & SSR_MPBT);

	if (cleared & SSR_TDRE)
	{
		if (m_scscr & SCR_TE)
		{
			m_scssr &= ~SSR_TEND;
			tx_start();
		}
		else
			m_scssr |= SSR_TDRE;
	}
	update_interrupts();
}

u8 sh3_sci_device::scrdr_r()
{
	return m_scrdr;
}

u8 sh3_sci_device::scscmr_r()
{
	return m_scscmr;
}

void sh3_sci_device::scscmr_w(u8 data)
{
	if (data & 0x01)
		LOGMASKED(LOG_UNIMPL, "smart card interface is not supported\n");
	m_scscmr = data;
}
