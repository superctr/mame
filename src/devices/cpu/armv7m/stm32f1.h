// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    STMicroelectronics STM32F1 (Cortex-M3 microcontrollers)

***************************************************************************/

#ifndef MAME_CPU_ARMV7M_STM32F1_H
#define MAME_CPU_ARMV7M_STM32F1_H

#pragma once

#include "armv7m.h"
#include "diserial.h"

#include <unordered_map>


class stm32f1_gpio_device : public device_t
{
public:
	stm32f1_gpio_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	auto in_cb() { return m_in_cb.bind(); }
	auto out_cb() { return m_out_cb.bind(); }

	u32 read(offs_t offset);
	void write(offs_t offset, u32 data, u32 mem_mask);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	devcb_read16 m_in_cb;
	devcb_write16 m_out_cb;

	u32 m_cr[2];
	u16 m_odr;
	u32 m_lckr;

	u16 output_mask() const;
	u16 opendrain_mask() const;
	void update_output();
};


class stm32f1_timer_device : public device_t
{
public:
	stm32f1_timer_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	auto update_irq_cb() { return m_update_irq_cb.bind(); }
	auto cc_irq_cb() { return m_cc_irq_cb.bind(); }

	u32 read(offs_t offset);
	void write(offs_t offset, u32 data, u32 mem_mask);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	devcb_write_line m_update_irq_cb;
	devcb_write_line m_cc_irq_cb;

	emu_timer *m_timer;
	attotime m_start_time;
	u32 m_start_count;
	u16 m_cr1;
	u16 m_cr2;
	u16 m_smcr;
	u16 m_dier;
	u16 m_sr;
	u16 m_ccmr[2];
	u16 m_ccer;
	u16 m_psc;
	u16 m_psc_active;
	u16 m_arr;
	u16 m_arr_active;
	u16 m_rcr;
	u16 m_rep;
	u16 m_ccr[4];
	u16 m_bdtr;

	attotime tick_period() const;
	u32 counter() const;
	void start(u32 count);
	void update_event();
	void update_irq();
	TIMER_CALLBACK_MEMBER(overflow);
};


class stm32f1_adc_device : public device_t
{
public:
	stm32f1_adc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	template <unsigned N> auto in_cb() { return m_in_cb[N].bind(); }
	auto irq_cb() { return m_irq_cb.bind(); }

	u32 read(offs_t offset);
	void write(offs_t offset, u32 data, u32 mem_mask);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	devcb_read16::array<18> m_in_cb;
	devcb_write_line m_irq_cb;

	emu_timer *m_timer;
	u32 m_sr;
	u32 m_cr1;
	u32 m_cr2;
	u32 m_smpr[2];
	u32 m_jofr[4];
	u32 m_htr;
	u32 m_ltr;
	u32 m_sqr[3];
	u32 m_jsqr;
	u32 m_jdr[4];
	u32 m_dr;
	u8 m_seq;

	unsigned channel(unsigned index) const;
	attotime conversion_time(unsigned ch) const;
	void start_regular();
	void update_irq();
	TIMER_CALLBACK_MEMBER(converted);
};


class stm32f1_usart_device : public device_t, public device_serial_interface
{
public:
	stm32f1_usart_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	auto txd_cb() { return m_txd_cb.bind(); }
	auto irq_cb() { return m_irq_cb.bind(); }

	void rxd_w(int state);

	u32 read(offs_t offset);
	void write(offs_t offset, u32 data, u32 mem_mask);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	virtual void tra_callback() override;
	virtual void tra_complete() override;
	virtual void rcv_complete() override;

private:
	devcb_write_line m_txd_cb;
	devcb_write_line m_irq_cb;

	u16 m_sr;
	u16 m_rdr;
	u16 m_tdr;
	u16 m_brr;
	u16 m_cr1;
	u16 m_cr2;
	u16 m_cr3;
	u16 m_gtpr;
	bool m_tdr_full;
	bool m_tx_busy;
	bool m_sr_read;

	void update_frame();
	void update_irq();
	void start_tx();
};


class stm32f103_device : public cortex_m3_device
{
public:
	stm32f103_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	template <unsigned N> auto gpio_in_cb() { return m_gpio[N].lookup()->in_cb(); }
	template <unsigned N> auto gpio_out_cb() { return m_gpio[N].lookup()->out_cb(); }
	template <unsigned N> auto adc_in_cb() { return m_adc[0].lookup()->in_cb<N>(); }
	template <unsigned N> auto usart_txd_cb() { return m_usart[N].lookup()->txd_cb(); }
	template <unsigned N> void usart_rxd_w(int state) { m_usart[N]->rxd_w(state); }

protected:
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	required_region_ptr<u32> m_flash;
	required_device_array<stm32f1_gpio_device, 5> m_gpio;
	required_device_array<stm32f1_timer_device, 8> m_timer;
	required_device_array<stm32f1_adc_device, 3> m_adc;
	required_device_array<stm32f1_usart_device, 5> m_usart;
	std::unordered_map<offs_t, u32> m_regs;

	u32 m_rcc_cr;
	u32 m_rcc_cfgr;
	u8 m_irq_share[9];

	void internal_map(address_map &map) ATTR_COLD;

	u32 flash_r(offs_t offset);
	u32 rcc_r(offs_t offset, u32 mem_mask);
	void rcc_w(offs_t offset, u32 data, u32 mem_mask);
	u32 unmapped_r(offs_t offset, u32 mem_mask);
	void unmapped_w(offs_t offset, u32 data, u32 mem_mask);
};

DECLARE_DEVICE_TYPE(STM32F1_GPIO, stm32f1_gpio_device)
DECLARE_DEVICE_TYPE(STM32F1_TIMER, stm32f1_timer_device)
DECLARE_DEVICE_TYPE(STM32F1_ADC, stm32f1_adc_device)
DECLARE_DEVICE_TYPE(STM32F1_USART, stm32f1_usart_device)
DECLARE_DEVICE_TYPE(STM32F103, stm32f103_device)

#endif // MAME_CPU_ARMV7M_STM32F1_H
