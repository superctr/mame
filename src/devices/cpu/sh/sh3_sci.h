// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    SH-3 serial communication interface (SCI), channel 0 of the SH7708
    and SH7709 families, asynchronous mode.

***************************************************************************/

#ifndef MAME_CPU_SH_SH3_SCI_H
#define MAME_CPU_SH_SH3_SCI_H

#pragma once

#include "diserial.h"


class sh3_sci_device : public device_t, public device_serial_interface
{
public:
	sh3_sci_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	auto txd_handler() { return m_txd_cb.bind(); }
	auto eri_handler() { return m_eri_cb.bind(); }
	auto rxi_handler() { return m_rxi_cb.bind(); }
	auto txi_handler() { return m_txi_cb.bind(); }
	auto tei_handler() { return m_tei_cb.bind(); }

	void map(address_map &map) ATTR_COLD;

	void rxd_w(int state) { rx_w(state); }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_clock_changed() override;

	virtual void tra_callback() override;
	virtual void tra_complete() override;
	virtual void rcv_complete() override;

private:
	enum : u8
	{
		SMR_CA   = 0x80,
		SMR_CHR  = 0x40,
		SMR_PE   = 0x20,
		SMR_OE   = 0x10,
		SMR_STOP = 0x08,
		SMR_MP   = 0x04,
		SMR_CKS  = 0x03,

		SCR_TIE  = 0x80,
		SCR_RIE  = 0x40,
		SCR_TE   = 0x20,
		SCR_RE   = 0x10,
		SCR_MPIE = 0x08,
		SCR_TEIE = 0x04,
		SCR_CKE  = 0x03,

		SSR_TDRE = 0x80,
		SSR_RDRF = 0x40,
		SSR_ORER = 0x20,
		SSR_FER  = 0x10,
		SSR_PER  = 0x08,
		SSR_TEND = 0x04,
		SSR_MPB  = 0x02,
		SSR_MPBT = 0x01,

		SSR_ERRORS = SSR_ORER | SSR_FER | SSR_PER,
		SSR_CLEARABLE = SSR_TDRE | SSR_RDRF | SSR_ERRORS
	};

	u8 scsmr_r();
	void scsmr_w(u8 data);
	u8 scbrr_r();
	void scbrr_w(u8 data);
	u8 scscr_r();
	void scscr_w(u8 data);
	u8 sctdr_r();
	void sctdr_w(u8 data);
	u8 scssr_r();
	void scssr_w(u8 data);
	u8 scrdr_r();
	u8 scscmr_r();
	void scscmr_w(u8 data);

	void update_format();
	void update_interrupts();
	void tx_start();

	devcb_write_line m_txd_cb, m_eri_cb, m_rxi_cb, m_txi_cb, m_tei_cb;

	u8 m_scsmr, m_scbrr, m_scscr, m_sctdr, m_scssr, m_scrdr, m_scscmr;
	u8 m_ssr_read;
	bool m_tx_busy;
	u8 m_irq_state;
};

DECLARE_DEVICE_TYPE(SH3_SCI, sh3_sci_device)

#endif // MAME_CPU_SH_SH3_SCI_H
