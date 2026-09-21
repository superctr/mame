// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    SH-3 (SH7709) serial communication interface with FIFO: the IrDA
    channel and the SCIF channel, asynchronous mode.

***************************************************************************/

#ifndef MAME_CPU_SH_SH3_SCIF_H
#define MAME_CPU_SH_SH3_SCIF_H

#pragma once

#include "diserial.h"


class sh3_scif_device : public device_t, public device_serial_interface
{
public:
	sh3_scif_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	auto txd_handler() { return m_txd_cb.bind(); }
	auto eri_handler() { return m_eri_cb.bind(); }
	auto rxi_handler() { return m_rxi_cb.bind(); }
	auto bri_handler() { return m_bri_cb.bind(); }
	auto txi_handler() { return m_txi_cb.bind(); }

	void map(address_map &map) ATTR_COLD;

	void rxd_w(int state) { rx_w(state); }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	virtual void tra_callback() override;
	virtual void tra_complete() override;
	virtual void rcv_complete() override;

private:
	enum : u8
	{
		SMR_CHR  = 0x40,
		SMR_PE   = 0x20,
		SMR_OE   = 0x10,
		SMR_STOP = 0x08,
		SMR_CKS  = 0x03,

		SCR_TIE  = 0x80,
		SCR_RIE  = 0x40,
		SCR_TE   = 0x20,
		SCR_RE   = 0x10,

		FCR_RTRG = 0xc0,
		FCR_TTRG = 0x30,
		FCR_TFRST = 0x04,
		FCR_RFRST = 0x02
	};

	enum : u16
	{
		SSR_ER   = 0x0080,
		SSR_TEND = 0x0040,
		SSR_TDFE = 0x0020,
		SSR_BRK  = 0x0010,
		SSR_FER  = 0x0008,
		SSR_PER  = 0x0004,
		SSR_RDF  = 0x0002,
		SSR_DR   = 0x0001
	};

	u8 scsmr_r();
	void scsmr_w(u8 data);
	u8 scbrr_r();
	void scbrr_w(u8 data);
	u8 scscr_r();
	void scscr_w(u8 data);
	void scftdr_w(u8 data);
	u16 scssr_r();
	void scssr_w(offs_t offset, u16 data, u16 mem_mask);
	u8 scfrdr_r();
	u8 scfcr_r();
	void scfcr_w(u8 data);
	u16 scfdr_r();

	void update_format();
	void update_status();
	void update_interrupts();
	void tx_next();
	void rx_flush();
	void tx_flush();
	int rx_trigger() const;
	int tx_trigger() const;
	TIMER_CALLBACK_MEMBER(rx_timeout);

	devcb_write_line m_txd_cb, m_eri_cb, m_rxi_cb, m_bri_cb, m_txi_cb;

	util::fifo<u8, 16> m_tx_fifo, m_rx_fifo;
	emu_timer *m_rx_timer;

	u8 m_scsmr, m_scbrr, m_scscr, m_scfcr;
	u16 m_scssr;
	u8 m_per_count, m_fer_count;
	bool m_tx_busy;
	u8 m_irq_state;
};

DECLARE_DEVICE_TYPE(SH3_SCIF, sh3_scif_device)

#endif // MAME_CPU_SH_SH3_SCIF_H
