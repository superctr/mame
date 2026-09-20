// license:BSD-3-Clause
// copyright-holders:superctr

// SH7034, sh1 variant with on-chip peripherals

#ifndef MAME_CPU_SH_SH7034_H
#define MAME_CPU_SH_SH7034_H

#pragma once

#include "sh_mcu.h"
#include "sh_intc.h"
#include "sh_itu.h"
#include "sh_port.h"
#include "sh_sci.h"

class sh7034_device : public sh_mcu_device
{
public:
	sh7034_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	template <int Sci> void sci_rx_w(int state) { m_sci[Sci]->do_rx_w(state); }
	template <int Sci> void sci_clk_w(int state) { m_sci[Sci]->do_clk_w(state); }
	template <int Sci> auto write_sci_tx() { return m_sci_tx[Sci].bind(); }
	template <int Sci> auto write_sci_clk() { return m_sci_clk[Sci].bind(); }

	template <int Channel> void dreq_w(int state) { m_dreq[Channel] = state; dma_check(); }

	template <int Port> auto read_adc() { return m_read_adc[Port].bind(); }

	auto read_porta()  { return m_read_port16 [0].bind(); }
	auto write_porta() { return m_write_port16[0].bind(); }
	auto read_portb()  { return m_read_port16 [1].bind(); }
	auto write_portb() { return m_write_port16[1].bind(); }
	auto read_portc()  { return m_read_port16 [2].bind(); }

	virtual u16 do_read_port16(int port) override { return m_read_port16[port](); }
	virtual void do_write_port16(int port, u16 data, u16 ddr) override { m_write_port16[port](0, data, ddr); }

	virtual void do_sci_tx(int sci, int state) override { m_sci_tx[sci](state); }
	virtual void do_sci_clk(int sci, int state) override { m_sci_clk[sci](state); }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void sh2_exception_internal(const char *message, int irqline, int vector) override;
	virtual void execute_set_input(int irqline, int state) override;

	virtual void internal_update(u64 current_time) override;

	void sh7034_map(address_map &map) ATTR_COLD;
	void peripheral_map(address_map &map) ATTR_COLD;

private:
	required_device<sh_intc_device> m_intc;
	required_device<sh_itu_device> m_itu;
	required_device_array<sh_itu_channel_device, 5> m_itu_channel;
	required_device_array<sh_sci_device, 2> m_sci;
	required_device<sh_port16_device> m_porta;
	required_device<sh_port16_device> m_portb;

	devcb_write_line::array<2> m_sci_tx, m_sci_clk;
	devcb_read16::array<3> m_read_port16;
	devcb_read16::array<8> m_read_adc;
	devcb_write16::array<2> m_write_port16;

	// Direct memory access controller (DMAC)
	u32 m_dma_sar[4], m_dma_dar[4];
	u16 m_dma_tcr[4], m_dma_chcr[4];
	u16 m_dmaor;
	bool m_dreq[4];
	bool m_dma_running;

	// A/D converter
	u16 m_addr[4];
	u8 m_adcsr, m_adcr;
	emu_timer *m_adc_timer;

	// Bus state controller (BSC)
	u16 m_bcr, m_wcr[3], m_dcr, m_pcr, m_rcr, m_rtcsr, m_rtcnt, m_rtcor;

	// User break controller (UBC)
	u16 m_bar[2], m_bamr[2], m_bbr;

	// Watchdog timer (WDT)
	u8 m_wdt_tcsr, m_wdt_tcnt, m_wdt_rstcsr;
	u8 m_sbycr;

	// Pin function controller (PFC)
	u16 m_pacr[2], m_pbcr[2], m_cascr;

	// Programmable timing pattern controller (TPC)
	u8 m_tpmr, m_tpcr, m_nder[2], m_ndr[2];

	template <int Channel> u32 dma_sar_r() { return m_dma_sar[Channel]; }
	template <int Channel> void dma_sar_w(offs_t, u32 data, u32 mem_mask);
	template <int Channel> u32 dma_dar_r() { return m_dma_dar[Channel]; }
	template <int Channel> void dma_dar_w(offs_t, u32 data, u32 mem_mask);
	template <int Channel> u16 dma_tcr_r() { return m_dma_tcr[Channel]; }
	template <int Channel> void dma_tcr_w(offs_t, u16 data, u16 mem_mask);
	template <int Channel> u16 dma_chcr_r() { return m_dma_chcr[Channel]; }
	template <int Channel> void dma_chcr_w(offs_t, u16 data, u16 mem_mask);
	u16 dmaor_r() { return m_dmaor; }
	void dmaor_w(offs_t, u16 data, u16 mem_mask);
	void dma_check();
	void dma_run(int channel);

	u16 bcr_r() { return m_bcr; }
	void bcr_w(offs_t, u16 data, u16 mem_mask) { COMBINE_DATA(&m_bcr); }
	template <int Reg> u16 wcr_r() { return m_wcr[Reg]; }
	template <int Reg> void wcr_w(offs_t, u16 data, u16 mem_mask) { COMBINE_DATA(&m_wcr[Reg]); }
	u16 dcr_r() { return m_dcr; }
	void dcr_w(offs_t, u16 data, u16 mem_mask) { COMBINE_DATA(&m_dcr); }
	u16 pcr_r() { return m_pcr; }
	void pcr_w(offs_t, u16 data, u16 mem_mask) { COMBINE_DATA(&m_pcr); }
	u16 rcr_r() { return m_rcr | 0x1f; }
	void rcr_w(offs_t, u16 data, u16 mem_mask);
	u16 rtcsr_r() { return m_rtcsr | 0x07; }
	void rtcsr_w(offs_t, u16 data, u16 mem_mask);
	u16 rtcnt_r() { return m_rtcnt; }
	void rtcnt_w(offs_t, u16 data, u16 mem_mask);
	u16 rtcor_r() { return m_rtcor; }
	void rtcor_w(offs_t, u16 data, u16 mem_mask);

	template <int Reg> u16 bar_r() { return m_bar[Reg]; }
	template <int Reg> void bar_w(offs_t, u16 data, u16 mem_mask) { COMBINE_DATA(&m_bar[Reg]); }
	template <int Reg> u16 bamr_r() { return m_bamr[Reg]; }
	template <int Reg> void bamr_w(offs_t, u16 data, u16 mem_mask) { COMBINE_DATA(&m_bamr[Reg]); }
	u16 bbr_r() { return m_bbr; }
	void bbr_w(offs_t, u16 data, u16 mem_mask) { COMBINE_DATA(&m_bbr); }

	u8 wdt_tcsr_r() { return m_wdt_tcsr | 0x18; }
	u8 wdt_tcnt_r() { return m_wdt_tcnt; }
	u8 wdt_rstcsr_r() { return m_wdt_rstcsr | 0x1f; }
	void wdt_w(offs_t, u16 data, u16 mem_mask);
	void wdt_rstcsr_w(offs_t, u16 data, u16 mem_mask);
	u8 sbycr_r() { return m_sbycr | 0x7f; }
	void sbycr_w(u8 data) { m_sbycr = data & 0x80; }

	template <int Reg> u16 pacr_r() { return m_pacr[Reg]; }
	template <int Reg> void pacr_w(offs_t, u16 data, u16 mem_mask) { COMBINE_DATA(&m_pacr[Reg]); }
	template <int Reg> u16 pbcr_r() { return m_pbcr[Reg]; }
	template <int Reg> void pbcr_w(offs_t, u16 data, u16 mem_mask) { COMBINE_DATA(&m_pbcr[Reg]); }
	u16 cascr_r() { return m_cascr | 0x3fff; }
	void cascr_w(offs_t, u16 data, u16 mem_mask) { COMBINE_DATA(&m_cascr); }
	u16 pcdr_r() { return m_read_port16[2](); }

	u8 tpmr_r() { return m_tpmr | 0xf0; }
	void tpmr_w(u8 data) { m_tpmr = data & 0x0f; }
	u8 tpcr_r() { return m_tpcr; }
	void tpcr_w(u8 data) { m_tpcr = data; }
	template <int Reg> u8 nder_r() { return m_nder[Reg]; }
	template <int Reg> void nder_w(u8 data) { m_nder[Reg] = data; }
	template <int Reg> u8 ndr_r() { return m_ndr[Reg]; }
	template <int Reg> void ndr_w(u8 data) { m_ndr[Reg] = data; }

	u16 addr_r(offs_t reg) { return m_addr[reg]; }
	u8 adcsr_r() { return m_adcsr; }
	void adcsr_w(u8 data);
	u8 adcr_r() { return m_adcr | 0x7f; }
	void adcr_w(u8 data) { m_adcr = data & 0x80; }
	void adc_start();
	TIMER_CALLBACK_MEMBER(adc_done);

	u16 adc_default(int port);
	u16 port16_default_r(int port);
	void port16_default_w(int port, u16 data);
};

DECLARE_DEVICE_TYPE(SH7034, sh7034_device)

#endif // MAME_CPU_SH_SH7034_H
