// license:BSD-3-Clause
// copyright-holders:superctr

// SH7034, sh1 variant with on-chip peripherals

#include "emu.h"
#include "sh7034.h"

#define LOG_DMA (1U << 1)

#define VERBOSE (0)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(SH7034, sh7034_device, "sh7034", "Hitachi SH-1 (SH7034)")

enum {
	VECTOR_DMAC0 = 72,
	VECTOR_ITU0  = 80,
	VECTOR_SCI0  = 100,
	VECTOR_ADC   = 112
};

sh7034_device::sh7034_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	sh_mcu_device(mconfig, SH7034, tag, owner, clock, CPU_TYPE_SH1, address_map_constructor(FUNC(sh7034_device::sh7034_map), this), 28, 0xc7ffffff),
	m_intc(*this, "intc"),
	m_itu(*this, "itu"),
	m_itu_channel(*this, "itu:%u", 0),
	m_sci(*this, "sci%d", 0),
	m_porta(*this, "porta"),
	m_portb(*this, "portb"),
	m_sci_tx(*this),
	m_sci_clk(*this),
	m_read_port16(*this, 0xffff),
	m_read_adc(*this, 0),
	m_write_port16(*this)
{
	m_isdrc = false;

	for(unsigned int i = 0; i != m_read_adc.size(); i++)
		m_read_adc[i].bind().set([this, i]() { return adc_default(i); });

	for(unsigned int i = 0; i != m_read_port16.size(); i++)
		m_read_port16[i].bind().set([this, i]() { return port16_default_r(i); });
	for(unsigned int i = 0; i != m_write_port16.size(); i++)
		m_write_port16[i].bind().set([this, i](u16 data) { port16_default_w(i, data); });
}

u16 sh7034_device::adc_default(int port)
{
	logerror("read of un-hooked adc %d\n", port);
	return 0;
}

u16 sh7034_device::port16_default_r(int port)
{
	if(!machine().side_effects_disabled())
		logerror("read of un-hooked port %c\n", 'a' + port);
	return 0xffff;
}

void sh7034_device::port16_default_w(int port, u16 data)
{
	logerror("write of un-hooked port %c %04x\n", 'a' + port, data);
}

void sh7034_device::device_start()
{
	sh_mcu_device::device_start();

	m_adc_timer = timer_alloc(FUNC(sh7034_device::adc_done), this);

	save_item(NAME(m_addr));
	save_item(NAME(m_adcsr));
	save_item(NAME(m_adcr));
	save_item(NAME(m_dma_sar));
	save_item(NAME(m_dma_dar));
	save_item(NAME(m_dma_tcr));
	save_item(NAME(m_dma_chcr));
	save_item(NAME(m_dmaor));
	save_item(NAME(m_dreq));
	save_item(NAME(m_bcr));
	save_item(NAME(m_wcr));
	save_item(NAME(m_dcr));
	save_item(NAME(m_pcr));
	save_item(NAME(m_rcr));
	save_item(NAME(m_rtcsr));
	save_item(NAME(m_rtcnt));
	save_item(NAME(m_rtcor));
	save_item(NAME(m_bar));
	save_item(NAME(m_bamr));
	save_item(NAME(m_bbr));
	save_item(NAME(m_wdt_tcsr));
	save_item(NAME(m_wdt_tcnt));
	save_item(NAME(m_wdt_rstcsr));
	save_item(NAME(m_sbycr));
	save_item(NAME(m_pacr));
	save_item(NAME(m_pbcr));
	save_item(NAME(m_cascr));
	save_item(NAME(m_tpmr));
	save_item(NAME(m_tpcr));
	save_item(NAME(m_nder));
	save_item(NAME(m_ndr));

	m_dma_running = false;
}

void sh7034_device::device_reset()
{
	sh2_device::device_reset();

	std::fill(std::begin(m_dma_sar), std::end(m_dma_sar), 0);
	std::fill(std::begin(m_dma_dar), std::end(m_dma_dar), 0);
	std::fill(std::begin(m_dma_tcr), std::end(m_dma_tcr), 0);
	std::fill(std::begin(m_dma_chcr), std::end(m_dma_chcr), 0);
	std::fill(std::begin(m_dreq), std::end(m_dreq), true);
	m_dmaor = 0;

	std::fill(std::begin(m_addr), std::end(m_addr), 0);
	m_adcsr = 0;
	m_adcr = 0;
	m_adc_timer->adjust(attotime::never);

	m_bcr = 0;
	std::fill(std::begin(m_wcr), std::end(m_wcr), 0xffff);
	m_dcr = 0;
	m_pcr = 0;
	m_rcr = 0;
	m_rtcsr = 0;
	m_rtcnt = 0;
	m_rtcor = 0xff;

	std::fill(std::begin(m_bar), std::end(m_bar), 0);
	std::fill(std::begin(m_bamr), std::end(m_bamr), 0);
	m_bbr = 0;

	m_wdt_tcsr = 0;
	m_wdt_tcnt = 0;
	m_wdt_rstcsr = 0;
	m_sbycr = 0;

	std::fill(std::begin(m_pacr), std::end(m_pacr), 0);
	std::fill(std::begin(m_pbcr), std::end(m_pbcr), 0);
	m_cascr = 0;

	m_tpmr = 0;
	m_tpcr = 0xff;
	std::fill(std::begin(m_nder), std::end(m_nder), 0);
	std::fill(std::begin(m_ndr), std::end(m_ndr), 0);
}

void sh7034_device::device_add_mconfig(machine_config &config)
{
	SH_INTC(config, m_intc, *this);
	m_intc->set_level_sense_bit(false);

	SH_ITU(config, m_itu, *this);
	for(int i = 0; i != 5; i++)
		SH_ITU_CHANNEL(config, m_itu_channel[i], *this, m_intc, VECTOR_ITU0 + 4*i, i >= 3 ? 2 : 0);

	SH_SCI(config, m_sci[0], 0, *this, m_intc, VECTOR_SCI0 + 0, VECTOR_SCI0 + 1, VECTOR_SCI0 + 2, VECTOR_SCI0 + 3);
	SH_SCI(config, m_sci[1], 1, *this, m_intc, VECTOR_SCI0 + 4, VECTOR_SCI0 + 5, VECTOR_SCI0 + 6, VECTOR_SCI0 + 7);

	SH_PORT16(config, m_porta, *this, 0, 0x0000, 0x0000);
	SH_PORT16(config, m_portb, *this, 1, 0x0000, 0x0000);
}

void sh7034_device::execute_set_input(int irqline, int state)
{
	if(irqline == INPUT_LINE_NMI)
		sh2_device::execute_set_input(irqline, state);
	else
		m_intc->set_input(irqline, state);
}

void sh7034_device::sh2_exception_internal(const char *message, int irqline, int vector)
{
	sh2_device::sh2_exception_internal(message, irqline, vector);
	m_intc->interrupt_taken(irqline, vector);
}

void sh7034_device::internal_update(u64 current_time)
{
	u64 event_time = 0;

	for(int i = 0; i != 5; i++)
		add_event(event_time, m_itu_channel[i]->internal_update(current_time));
	add_event(event_time, m_sci[0]->internal_update(current_time));
	add_event(event_time, m_sci[1]->internal_update(current_time));

	recompute_timer(event_time);
}

void sh7034_device::sh7034_map(address_map &map)
{
	map(0x00000000, 0x0000ffff).rom().region(DEVICE_SELF, 0);
	map(0x07fff000, 0x07ffffff).ram();

	peripheral_map(map);
}

void sh7034_device::peripheral_map(address_map &map)
{
	map(0x05fffec0, 0x05fffec0).rw(m_sci[0], FUNC(sh_sci_device::smr_r), FUNC(sh_sci_device::smr_w));
	map(0x05fffec1, 0x05fffec1).rw(m_sci[0], FUNC(sh_sci_device::brr_r), FUNC(sh_sci_device::brr_w));
	map(0x05fffec2, 0x05fffec2).rw(m_sci[0], FUNC(sh_sci_device::scr_r), FUNC(sh_sci_device::scr_w));
	map(0x05fffec3, 0x05fffec3).rw(m_sci[0], FUNC(sh_sci_device::tdr_r), FUNC(sh_sci_device::tdr_w));
	map(0x05fffec4, 0x05fffec4).rw(m_sci[0], FUNC(sh_sci_device::ssr_r), FUNC(sh_sci_device::ssr_w));
	map(0x05fffec5, 0x05fffec5).r(m_sci[0], FUNC(sh_sci_device::rdr_r));
	map(0x05fffec8, 0x05fffec8).rw(m_sci[1], FUNC(sh_sci_device::smr_r), FUNC(sh_sci_device::smr_w));
	map(0x05fffec9, 0x05fffec9).rw(m_sci[1], FUNC(sh_sci_device::brr_r), FUNC(sh_sci_device::brr_w));
	map(0x05fffeca, 0x05fffeca).rw(m_sci[1], FUNC(sh_sci_device::scr_r), FUNC(sh_sci_device::scr_w));
	map(0x05fffecb, 0x05fffecb).rw(m_sci[1], FUNC(sh_sci_device::tdr_r), FUNC(sh_sci_device::tdr_w));
	map(0x05fffecc, 0x05fffecc).rw(m_sci[1], FUNC(sh_sci_device::ssr_r), FUNC(sh_sci_device::ssr_w));
	map(0x05fffecd, 0x05fffecd).r(m_sci[1], FUNC(sh_sci_device::rdr_r));

	map(0x05fffee0, 0x05fffee7).r(FUNC(sh7034_device::addr_r));
	map(0x05fffef8, 0x05fffef8).rw(FUNC(sh7034_device::adcsr_r), FUNC(sh7034_device::adcsr_w));
	map(0x05fffef9, 0x05fffef9).rw(FUNC(sh7034_device::adcr_r), FUNC(sh7034_device::adcr_w));

	map(0x05ffff00, 0x05ffff00).rw(m_itu, FUNC(sh_itu_device::tstr_r), FUNC(sh_itu_device::tstr_w));
	map(0x05ffff01, 0x05ffff01).rw(m_itu, FUNC(sh_itu_device::tsnc_r), FUNC(sh_itu_device::tsnc_w));
	map(0x05ffff02, 0x05ffff02).rw(m_itu, FUNC(sh_itu_device::tmdr_r), FUNC(sh_itu_device::tmdr_w));
	map(0x05ffff03, 0x05ffff03).rw(m_itu, FUNC(sh_itu_device::tfcr_r), FUNC(sh_itu_device::tfcr_w));
	map(0x05ffff31, 0x05ffff31).rw(m_itu, FUNC(sh_itu_device::tocr_r), FUNC(sh_itu_device::tocr_w));

	for(int i = 0; i != 5; i++) {
		const offs_t base = 0x05ffff04 + (i == 4 ? 0x2e : 0x0a * i);
		map(base + 0, base + 0).rw(m_itu_channel[i], FUNC(sh_itu_channel_device::tcr_r), FUNC(sh_itu_channel_device::tcr_w));
		map(base + 1, base + 1).rw(m_itu_channel[i], FUNC(sh_itu_channel_device::tior_r), FUNC(sh_itu_channel_device::tior_w));
		map(base + 2, base + 2).rw(m_itu_channel[i], FUNC(sh_itu_channel_device::tier_r), FUNC(sh_itu_channel_device::tier_w));
		map(base + 3, base + 3).rw(m_itu_channel[i], FUNC(sh_itu_channel_device::tsr_r), FUNC(sh_itu_channel_device::tsr_w));
		map(base + 4, base + 5).rw(m_itu_channel[i], FUNC(sh_itu_channel_device::tcnt_r), FUNC(sh_itu_channel_device::tcnt_w));
		map(base + 6, base + 9).rw(m_itu_channel[i], FUNC(sh_itu_channel_device::gr_r), FUNC(sh_itu_channel_device::gr_w));
		if(i >= 3)
			map(base + 10, base + 13).rw(m_itu_channel[i], FUNC(sh_itu_channel_device::br_r), FUNC(sh_itu_channel_device::br_w));
	}

	map(0x05ffff40, 0x05ffff43).rw(FUNC(sh7034_device::dma_sar_r<0>), FUNC(sh7034_device::dma_sar_w<0>));
	map(0x05ffff44, 0x05ffff47).rw(FUNC(sh7034_device::dma_dar_r<0>), FUNC(sh7034_device::dma_dar_w<0>));
	map(0x05ffff48, 0x05ffff49).rw(FUNC(sh7034_device::dmaor_r), FUNC(sh7034_device::dmaor_w));
	map(0x05ffff4a, 0x05ffff4b).rw(FUNC(sh7034_device::dma_tcr_r<0>), FUNC(sh7034_device::dma_tcr_w<0>));
	map(0x05ffff4e, 0x05ffff4f).rw(FUNC(sh7034_device::dma_chcr_r<0>), FUNC(sh7034_device::dma_chcr_w<0>));
	map(0x05ffff50, 0x05ffff53).rw(FUNC(sh7034_device::dma_sar_r<1>), FUNC(sh7034_device::dma_sar_w<1>));
	map(0x05ffff54, 0x05ffff57).rw(FUNC(sh7034_device::dma_dar_r<1>), FUNC(sh7034_device::dma_dar_w<1>));
	map(0x05ffff5a, 0x05ffff5b).rw(FUNC(sh7034_device::dma_tcr_r<1>), FUNC(sh7034_device::dma_tcr_w<1>));
	map(0x05ffff5e, 0x05ffff5f).rw(FUNC(sh7034_device::dma_chcr_r<1>), FUNC(sh7034_device::dma_chcr_w<1>));
	map(0x05ffff60, 0x05ffff63).rw(FUNC(sh7034_device::dma_sar_r<2>), FUNC(sh7034_device::dma_sar_w<2>));
	map(0x05ffff64, 0x05ffff67).rw(FUNC(sh7034_device::dma_dar_r<2>), FUNC(sh7034_device::dma_dar_w<2>));
	map(0x05ffff6a, 0x05ffff6b).rw(FUNC(sh7034_device::dma_tcr_r<2>), FUNC(sh7034_device::dma_tcr_w<2>));
	map(0x05ffff6e, 0x05ffff6f).rw(FUNC(sh7034_device::dma_chcr_r<2>), FUNC(sh7034_device::dma_chcr_w<2>));
	map(0x05ffff70, 0x05ffff73).rw(FUNC(sh7034_device::dma_sar_r<3>), FUNC(sh7034_device::dma_sar_w<3>));
	map(0x05ffff74, 0x05ffff77).rw(FUNC(sh7034_device::dma_dar_r<3>), FUNC(sh7034_device::dma_dar_w<3>));
	map(0x05ffff7a, 0x05ffff7b).rw(FUNC(sh7034_device::dma_tcr_r<3>), FUNC(sh7034_device::dma_tcr_w<3>));
	map(0x05ffff7e, 0x05ffff7f).rw(FUNC(sh7034_device::dma_chcr_r<3>), FUNC(sh7034_device::dma_chcr_w<3>));

	map(0x05ffff84, 0x05ffff8d).rw(m_intc, FUNC(sh_intc_device::ipr_r), FUNC(sh_intc_device::ipr_w));
	map(0x05ffff8e, 0x05ffff8f).rw(m_intc, FUNC(sh_intc_device::icr_r), FUNC(sh_intc_device::icr_w));

	map(0x05ffff90, 0x05ffff91).rw(FUNC(sh7034_device::bar_r<0>), FUNC(sh7034_device::bar_w<0>));
	map(0x05ffff92, 0x05ffff93).rw(FUNC(sh7034_device::bar_r<1>), FUNC(sh7034_device::bar_w<1>));
	map(0x05ffff94, 0x05ffff95).rw(FUNC(sh7034_device::bamr_r<0>), FUNC(sh7034_device::bamr_w<0>));
	map(0x05ffff96, 0x05ffff97).rw(FUNC(sh7034_device::bamr_r<1>), FUNC(sh7034_device::bamr_w<1>));
	map(0x05ffff98, 0x05ffff99).rw(FUNC(sh7034_device::bbr_r), FUNC(sh7034_device::bbr_w));

	map(0x05ffffa0, 0x05ffffa1).rw(FUNC(sh7034_device::bcr_r), FUNC(sh7034_device::bcr_w));
	map(0x05ffffa2, 0x05ffffa3).rw(FUNC(sh7034_device::wcr_r<0>), FUNC(sh7034_device::wcr_w<0>));
	map(0x05ffffa4, 0x05ffffa5).rw(FUNC(sh7034_device::wcr_r<1>), FUNC(sh7034_device::wcr_w<1>));
	map(0x05ffffa6, 0x05ffffa7).rw(FUNC(sh7034_device::wcr_r<2>), FUNC(sh7034_device::wcr_w<2>));
	map(0x05ffffa8, 0x05ffffa9).rw(FUNC(sh7034_device::dcr_r), FUNC(sh7034_device::dcr_w));
	map(0x05ffffaa, 0x05ffffab).rw(FUNC(sh7034_device::pcr_r), FUNC(sh7034_device::pcr_w));
	map(0x05ffffac, 0x05ffffad).rw(FUNC(sh7034_device::rcr_r), FUNC(sh7034_device::rcr_w));
	map(0x05ffffae, 0x05ffffaf).rw(FUNC(sh7034_device::rtcsr_r), FUNC(sh7034_device::rtcsr_w));
	map(0x05ffffb0, 0x05ffffb1).rw(FUNC(sh7034_device::rtcnt_r), FUNC(sh7034_device::rtcnt_w));
	map(0x05ffffb2, 0x05ffffb3).rw(FUNC(sh7034_device::rtcor_r), FUNC(sh7034_device::rtcor_w));

	map(0x05ffffb8, 0x05ffffb9).w(FUNC(sh7034_device::wdt_w));
	map(0x05ffffb8, 0x05ffffb8).r(FUNC(sh7034_device::wdt_tcsr_r));
	map(0x05ffffb9, 0x05ffffb9).r(FUNC(sh7034_device::wdt_tcnt_r));
	map(0x05ffffba, 0x05ffffbb).w(FUNC(sh7034_device::wdt_rstcsr_w));
	map(0x05ffffbb, 0x05ffffbb).r(FUNC(sh7034_device::wdt_rstcsr_r));
	map(0x05ffffbc, 0x05ffffbc).rw(FUNC(sh7034_device::sbycr_r), FUNC(sh7034_device::sbycr_w));

	map(0x05ffffc0, 0x05ffffc1).rw(m_porta, FUNC(sh_port16_device::dr_r), FUNC(sh_port16_device::dr_w));
	map(0x05ffffc2, 0x05ffffc3).rw(m_portb, FUNC(sh_port16_device::dr_r), FUNC(sh_port16_device::dr_w));
	map(0x05ffffc4, 0x05ffffc5).rw(m_porta, FUNC(sh_port16_device::io_r), FUNC(sh_port16_device::io_w));
	map(0x05ffffc6, 0x05ffffc7).rw(m_portb, FUNC(sh_port16_device::io_r), FUNC(sh_port16_device::io_w));
	map(0x05ffffc8, 0x05ffffc9).rw(FUNC(sh7034_device::pacr_r<0>), FUNC(sh7034_device::pacr_w<0>));
	map(0x05ffffca, 0x05ffffcb).rw(FUNC(sh7034_device::pacr_r<1>), FUNC(sh7034_device::pacr_w<1>));
	map(0x05ffffcc, 0x05ffffcd).rw(FUNC(sh7034_device::pbcr_r<0>), FUNC(sh7034_device::pbcr_w<0>));
	map(0x05ffffce, 0x05ffffcf).rw(FUNC(sh7034_device::pbcr_r<1>), FUNC(sh7034_device::pbcr_w<1>));
	map(0x05ffffd0, 0x05ffffd1).r(FUNC(sh7034_device::pcdr_r));
	map(0x05ffffee, 0x05ffffef).rw(FUNC(sh7034_device::cascr_r), FUNC(sh7034_device::cascr_w));

	map(0x05fffff0, 0x05fffff0).rw(FUNC(sh7034_device::tpmr_r), FUNC(sh7034_device::tpmr_w));
	map(0x05fffff1, 0x05fffff1).rw(FUNC(sh7034_device::tpcr_r), FUNC(sh7034_device::tpcr_w));
	map(0x05fffff2, 0x05fffff2).rw(FUNC(sh7034_device::nder_r<1>), FUNC(sh7034_device::nder_w<1>));
	map(0x05fffff3, 0x05fffff3).rw(FUNC(sh7034_device::nder_r<0>), FUNC(sh7034_device::nder_w<0>));
	map(0x05fffff4, 0x05fffff4).rw(FUNC(sh7034_device::ndr_r<1>), FUNC(sh7034_device::ndr_w<1>));
	map(0x05fffff5, 0x05fffff5).rw(FUNC(sh7034_device::ndr_r<0>), FUNC(sh7034_device::ndr_w<0>));
	map(0x05fffff6, 0x05fffff6).rw(FUNC(sh7034_device::ndr_r<1>), FUNC(sh7034_device::ndr_w<1>));
	map(0x05fffff7, 0x05fffff7).rw(FUNC(sh7034_device::ndr_r<0>), FUNC(sh7034_device::ndr_w<0>));
}


// A/D converter

void sh7034_device::adcsr_w(u8 data)
{
	const bool was_running = BIT(m_adcsr, 5);
	m_adcsr = (m_adcsr & data & 0x80) | (data & 0x3f);
	if(BIT(m_adcsr, 5)) {
		if(!was_running)
			adc_start();
	} else
		m_adc_timer->adjust(attotime::never);
}

void sh7034_device::adc_start()
{
	// 266 or 134 states per channel, four channels in scan mode
	const int states = (BIT(m_adcsr, 3) ? 134 : 266) * (BIT(m_adcsr, 4) ? (m_adcsr & 3) + 1 : 1);
	m_adc_timer->adjust(attotime::from_ticks(states, clock()));
}

TIMER_CALLBACK_MEMBER(sh7034_device::adc_done)
{
	const int group = BIT(m_adcsr, 2) ? 4 : 0;
	if(BIT(m_adcsr, 4))
		for(int i = 0; i <= (m_adcsr & 3); i++)
			m_addr[i] = m_read_adc[group + i]() << 6;
	else
		m_addr[m_adcsr & 3] = m_read_adc[group + (m_adcsr & 3)]() << 6;

	m_adcsr |= 0x80;
	if(BIT(m_adcsr, 4))
		adc_start();
	else
		m_adcsr &= ~0x20;

	if(BIT(m_adcsr, 6))
		m_intc->internal_interrupt(VECTOR_ADC);
}


// Bus state controller

void sh7034_device::rcr_w(offs_t, u16 data, u16 mem_mask)
{
	if((data & 0xff00) == 0x5a00)
		m_rcr = data & 0xe0;
}

void sh7034_device::rtcsr_w(offs_t, u16 data, u16 mem_mask)
{
	if((data & 0xff00) == 0xa500)
		m_rtcsr = (m_rtcsr & data & 0x80) | (data & 0x78);
}

void sh7034_device::rtcnt_w(offs_t, u16 data, u16 mem_mask)
{
	if((data & 0xff00) == 0xa500)
		m_rtcnt = data & 0xff;
}

void sh7034_device::rtcor_w(offs_t, u16 data, u16 mem_mask)
{
	if((data & 0xff00) == 0xa500)
		m_rtcor = data & 0xff;
}


// Watchdog timer

void sh7034_device::wdt_w(offs_t, u16 data, u16 mem_mask)
{
	switch(data & 0xff00) {
	case 0x5a00:
		m_wdt_tcnt = data & 0xff;
		break;
	case 0xa500:
		m_wdt_tcsr = data & 0xff;
		break;
	}
}

void sh7034_device::wdt_rstcsr_w(offs_t, u16 data, u16 mem_mask)
{
	if((data & 0xff00) == 0xa500)
		m_wdt_rstcsr = (m_wdt_rstcsr & data & 0x80) | (data & 0x60);
	else if((data & 0xff00) == 0x5a00)
		m_wdt_rstcsr = (m_wdt_rstcsr & 0x80) | (data & 0x60);
}


// Direct memory access controller

template <int Channel> void sh7034_device::dma_sar_w(offs_t, u32 data, u32 mem_mask)
{
	COMBINE_DATA(&m_dma_sar[Channel]);
}

template <int Channel> void sh7034_device::dma_dar_w(offs_t, u32 data, u32 mem_mask)
{
	COMBINE_DATA(&m_dma_dar[Channel]);
}

template <int Channel> void sh7034_device::dma_tcr_w(offs_t, u16 data, u16 mem_mask)
{
	COMBINE_DATA(&m_dma_tcr[Channel]);
}

template <int Channel> void sh7034_device::dma_chcr_w(offs_t, u16 data, u16 mem_mask)
{
	const u16 old = m_dma_chcr[Channel];
	COMBINE_DATA(&m_dma_chcr[Channel]);
	m_dma_chcr[Channel] = (m_dma_chcr[Channel] & ~2) | (old & m_dma_chcr[Channel] & 2);
	dma_check();
}

void sh7034_device::dmaor_w(offs_t, u16 data, u16 mem_mask)
{
	const u16 old = m_dmaor;
	COMBINE_DATA(&m_dmaor);
	m_dmaor = (m_dmaor & ~6) | (old & m_dmaor & 6);
	dma_check();
}

void sh7034_device::dma_check()
{
	if(m_dma_running || !BIT(m_dmaor, 0) || (m_dmaor & 6))
		return;

	m_dma_running = true;
	for(int channel = 0; channel != 4; channel++) {
		const u16 chcr = m_dma_chcr[channel];
		if((chcr & 3) != 1)
			continue;

		const int rs = (chcr >> 8) & 15;
		if(rs >= 4 && rs != 12)
			continue;
		if(rs < 4 && !m_dreq[channel])
			continue;

		dma_run(channel);
	}
	m_dma_running = false;
}

void sh7034_device::dma_run(int channel)
{
	const u16 chcr = m_dma_chcr[channel];
	const int size = BIT(chcr, 3) ? 2 : 1;
	const int dm = (chcr >> 14) & 3;
	const int sm = (chcr >> 12) & 3;
	const int dd = dm == 1 ? size : dm == 2 ? -size : 0;
	const int sd = sm == 1 ? size : sm == 2 ? -size : 0;

	u32 count = m_dma_tcr[channel] ? m_dma_tcr[channel] : 0x10000;
	LOGMASKED(LOG_DMA, "dma %d %08x -> %08x, %x %s\n", channel, m_dma_sar[channel], m_dma_dar[channel], count, size == 2 ? "words" : "bytes");

	while(count--) {
		if(size == 2)
			m_program->write_word(m_dma_dar[channel] & m_am, m_program->read_word(m_dma_sar[channel] & m_am));
		else
			m_program->write_byte(m_dma_dar[channel] & m_am, m_program->read_byte(m_dma_sar[channel] & m_am));
		m_dma_sar[channel] += sd;
		m_dma_dar[channel] += dd;
	}

	m_dma_tcr[channel] = 0;
	m_dma_chcr[channel] |= 2;
	if(BIT(chcr, 2))
		m_intc->internal_interrupt(VECTOR_DMAC0 + 2*channel);
}
