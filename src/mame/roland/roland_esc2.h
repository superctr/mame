// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Roland ESC2 (MB8AA4181)

***************************************************************************/

#ifndef MAME_ROLAND_ROLAND_ESC2_H
#define MAME_ROLAND_ROLAND_ESC2_H

#pragma once

#include "roland_esc2_dsp.h"

#include "cpu/armv7m/armv7m.h"
#include "diserial.h"

#include <unordered_map>


class mb8aa4181_mfs_device : public device_t, public device_serial_interface
{
public:
	mb8aa4181_mfs_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	auto txd_cb() { return m_txd_cb.bind(); }
	auto sot_cb() { return m_sot_cb.bind(); }
	auto sin_cb() { return m_sin_cb.bind(); }
	auto rx_irq_cb() { return m_rx_irq_cb.bind(); }
	auto tx_irq_cb() { return m_tx_irq_cb.bind(); }
	auto status_irq_cb() { return m_status_irq_cb.bind(); }

	void rxd_w(int state);

	u32 read(offs_t offset, u32 mem_mask);
	void write(offs_t offset, u32 data, u32 mem_mask);

	void rom_mode(u32 mode);
	void rom_baud(u32 baud);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	virtual void tra_callback() override;
	virtual void tra_complete() override;
	virtual void rcv_complete() override;

private:
	static constexpr unsigned FIFO_SIZE = 64;

	enum : u16 {
		SCR_UPCL = 0x8000, SCR_RIE = 0x1000, SCR_TIE = 0x0800, SCR_TBIE = 0x0400, SCR_RXE = 0x0200, SCR_TXE = 0x0100,
		SMR_MD = 0x00e0, SMR_SBL = 0x0008, SMR_BDS = 0x0004, SMR_SOE = 0x0001,
		SSR_REC = 0x8000, SSR_PE = 0x2000, SSR_FRE = 0x1000, SSR_ORE = 0x0800, SSR_RDRF = 0x0400, SSR_TDRE = 0x0200, SSR_TBI = 0x0100,
		ESCR_L = 0x0007,
		IBCR_MSS = 0x8000, IBCR_ACT = 0x4000, IBCR_CNDE = 0x0800, IBCR_INTE = 0x0400, IBCR_BER = 0x0200, IBCR_INT = 0x0100,
		IBSR_FBT = 0x80, IBSR_RACK = 0x40, IBSR_RSA = 0x20, IBSR_TRX = 0x10, IBSR_AL = 0x08, IBSR_RSC = 0x04, IBSR_SPC = 0x02, IBSR_BB = 0x01,
		FCR_FTIE = 0x0200, FCR_FDRQ = 0x0400, FCR_FSEL = 0x0100, FCR_FCL2 = 0x0008, FCR_FCL1 = 0x0004, FCR_FE2 = 0x0002, FCR_FE1 = 0x0001
	};

	devcb_write_line m_txd_cb;
	devcb_write8 m_sot_cb;
	devcb_read8 m_sin_cb;
	devcb_write_line m_rx_irq_cb;
	devcb_write_line m_tx_irq_cb;
	devcb_write_line m_status_irq_cb;

	u16 m_scr_smr;
	u16 m_ssr_escr;
	u16 m_rdr;
	u16 m_tdr;
	u16 m_bgr;
	u16 m_fcr;
	u8 m_rx_threshold;
	u8 m_tx_fifo[FIFO_SIZE];
	u8 m_rx_fifo[FIFO_SIZE];
	u8 m_tx_head, m_tx_count;
	u8 m_rx_head, m_rx_count;
	bool m_tx_busy;
	bool m_tdr_full;
	int m_rxd;
	u8 m_csio_data;
	u8 m_ibsr;
	u16 m_i2c_address;
	bool m_i2c_active;
	emu_timer *m_csio_timer;
	std::unordered_map<offs_t, u32> m_regs;

	bool i2c_mode() const;
	bool csio_mode() const;
	u32 i2c_read(offs_t offset);
	void i2c_write(offs_t offset, u32 data, u32 mem_mask);
	void i2c_start();
	TIMER_CALLBACK_MEMBER(csio_done);
	bool tx_fifo_enabled() const;
	bool rx_fifo_enabled() const;
	void update_frame();
	void update_rate();
	void restart_rx();
	void update_irq();
	void start_tx();
	void fifo_reset(bool fifo2);
};


class mb8aa4181_device : public cortex_m3_device
{
public:
	mb8aa4181_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	template <typename T> void set_flash_tag(T &&tag) { m_flash.set_tag(std::forward<T>(tag)); }

	template <unsigned N> auto txd_cb() { return m_mfs[N].lookup()->txd_cb(); }
	template <unsigned N> auto sot_cb() { return m_mfs[N].lookup()->sot_cb(); }
	template <unsigned N> auto sin_cb() { return m_mfs[N].lookup()->sin_cb(); }
	template <unsigned N> auto gpio_out_cb() { return m_gpio_out_cb[N].bind(); }
	template <unsigned N> auto gpio_in_cb() { return m_gpio_in_cb[N].bind(); }
	template <unsigned N> auto adc_in_cb() { return m_adc_in_cb[N].bind(); }
	template <unsigned N> void rxd_w(int state) { m_mfs[N]->rxd_w(state); }
	template <unsigned N> void exint_w(int state) { exint_in(N, state); }
	auto sfi_cs_cb() { return m_sfi_cs_cb.bind(); }
	auto sfi_tx_cb() { return m_sfi_tx_cb.bind(); }
	auto sfi_rx_cb() { return m_sfi_rx_cb.bind(); }

protected:
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	required_region_ptr<u32> m_flash;
	required_device_array<mb8aa4181_mfs_device, 8> m_mfs;
	required_device<mb8aa4181_dsp_device> m_dsp;
	memory_share_creator<u32> m_iram;
	std::unordered_map<offs_t, u32> m_regs;
	std::unordered_map<offs_t, u32> m_logged;
	devcb_write32::array<8> m_gpio_out_cb;
	devcb_read32::array<8> m_gpio_in_cb;
	devcb_read16::array<8> m_adc_in_cb;
	devcb_write_line m_sfi_cs_cb;
	devcb_write8 m_sfi_tx_cb;
	devcb_read8 m_sfi_rx_cb;

	u32 m_gpio_out[8];
	u32 m_dma_flags;
	u32 m_sfi[16];
	u32 m_exint[8];
	u8 m_exint_level;
	u8 m_exint_pending;
	u32 m_sfi_rx_left;
	u32 m_sfi_tx_left;
	double m_converter[4];
	u32 m_adc_ctrl;
	u32 m_adc_config;
	u32 m_adc_status;
	u16 m_adc_data[8];
	emu_timer *m_adc_timer;
	u32 m_timer_load[2];
	u32 m_timer_bgload[2];
	u32 m_timer_ctrl[2];
	bool m_timer_int[2];
	emu_timer *m_timer[2];

	void internal_map(address_map &map) ATTR_COLD;

	u32 bootrom_r(offs_t offset, u32 mem_mask);
	u32 bootrom_call(u32 entry);
	u32 flash_r(offs_t offset);
	void sfi_start(u8 command);
	void sfi_end();
	u8 sfi_status();
	u32 sfi_r(offs_t offset);
	void sfi_w(offs_t offset, u32 data, u32 mem_mask);
	u32 unmapped_r(offs_t offset, u32 mem_mask);
	void unmapped_w(offs_t offset, u32 data, u32 mem_mask);

	u32 timebase_r(offs_t offset);
	u32 converter_r(offs_t offset);
	void converter_w(offs_t offset, u32 data, u32 mem_mask);
	u32 adc_r(offs_t offset);
	void adc_w(offs_t offset, u32 data, u32 mem_mask);
	TIMER_CALLBACK_MEMBER(adc_done);

	u32 dmaflag_r(offs_t offset);
	void dmaflag_w(offs_t offset, u32 data, u32 mem_mask);
	void rom_dma(u32 desc);

	void event_w(offs_t offset, u32 data, u32 mem_mask);

	void exint_in(unsigned channel, int state);
	void exint_update();
	u32 exint_reg_r(offs_t offset);
	void exint_reg_w(offs_t offset, u32 data, u32 mem_mask);

	u32 gpio_r(offs_t offset);
	void gpio_w(offs_t offset, u32 data, u32 mem_mask);

	u32 dualtimer_r(offs_t offset);
	void dualtimer_w(offs_t offset, u32 data, u32 mem_mask);
	TIMER_CALLBACK_MEMBER(dualtimer_expired);
	attotime dualtimer_period(int which) const;
	void dualtimer_start(int which, u32 count);
	u32 dualtimer_value(int which) const;
	void dualtimer_irq(int which);
};

DECLARE_DEVICE_TYPE(MB8AA4181_MFS, mb8aa4181_mfs_device)
DECLARE_DEVICE_TYPE(MB8AA4181, mb8aa4181_device)

#endif // MAME_ROLAND_ROLAND_ESC2_H
