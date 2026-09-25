// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    STMicroelectronics STM32F1 (Cortex-M3 microcontrollers)

    The high-density STM32F103 with its GPIO ports, general-purpose and
    advanced timers, ADCs and USARTs.  The clock tree is not modelled: the
    buses and timers run at the configured core clock, as they do from the
    reset clock with no prescaler set.

***************************************************************************/

#include "emu.h"
#include "stm32f1.h"

#define LOG_UNMAPPED (1U << 1)

#define VERBOSE (0)
#include "logmacro.h"

#define LOGUNMAPPED(...) LOGMASKED(LOG_UNMAPPED, __VA_ARGS__)


DEFINE_DEVICE_TYPE(STM32F1_GPIO, stm32f1_gpio_device, "stm32f1_gpio", "STM32F1 GPIO port")
DEFINE_DEVICE_TYPE(STM32F1_TIMER, stm32f1_timer_device, "stm32f1_timer", "STM32F1 timer")
DEFINE_DEVICE_TYPE(STM32F1_ADC, stm32f1_adc_device, "stm32f1_adc", "STM32F1 ADC")
DEFINE_DEVICE_TYPE(STM32F1_USART, stm32f1_usart_device, "stm32f1_usart", "STM32F1 USART")
DEFINE_DEVICE_TYPE(STM32F103, stm32f103_device, "stm32f103", "STMicroelectronics STM32F103")


//**************************************************************************
//  GPIO port
//**************************************************************************

stm32f1_gpio_device::stm32f1_gpio_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, STM32F1_GPIO, tag, owner, clock)
	, m_in_cb(*this, 0xffff)
	, m_out_cb(*this)
{
}

void stm32f1_gpio_device::device_start()
{
	save_item(NAME(m_cr));
	save_item(NAME(m_odr));
	save_item(NAME(m_lckr));
}

void stm32f1_gpio_device::device_reset()
{
	m_cr[0] = m_cr[1] = 0x44444444;
	m_odr = 0;
	m_lckr = 0;
	update_output();
}

u16 stm32f1_gpio_device::output_mask() const
{
	u16 mask = 0;
	for (int i = 0; i < 16; i++)
		if (BIT(m_cr[i >> 3], (i & 7) * 4, 2))
			mask |= 1 << i;
	return mask;
}

u16 stm32f1_gpio_device::opendrain_mask() const
{
	u16 mask = 0;
	for (int i = 0; i < 16; i++)
		if (BIT(m_cr[i >> 3], (i & 7) * 4, 2) && BIT(m_cr[i >> 3], (i & 7) * 4 + 2))
			mask |= 1 << i;
	return mask;
}

void stm32f1_gpio_device::update_output()
{
	const u16 out = output_mask();
	m_out_cb(0, (m_odr & out) | ~out, 0xffff);
}

u32 stm32f1_gpio_device::read(offs_t offset)
{
	switch (offset)
	{
	case 0: case 1:
		return m_cr[offset];
	case 2:
	{
		const u16 out = output_mask() & ~opendrain_mask();
		const u16 in = machine().side_effects_disabled() ? 0xffff : m_in_cb();
		return (in & ~out) | (m_odr & out);
	}
	case 3:
		return m_odr;
	case 6:
		return m_lckr;
	default:
		return 0;
	}
}

void stm32f1_gpio_device::write(offs_t offset, u32 data, u32 mem_mask)
{
	switch (offset)
	{
	case 0: case 1:
		COMBINE_DATA(&m_cr[offset]);
		break;
	case 3:
		m_odr = (m_odr & ~mem_mask) | (data & mem_mask);
		break;
	case 4:
		data &= mem_mask;
		m_odr = (m_odr & ~u16(data >> 16)) | u16(data);
		break;
	case 5:
		m_odr &= ~u16(data & mem_mask);
		break;
	case 6:
		COMBINE_DATA(&m_lckr);
		return;
	default:
		return;
	}
	update_output();
}


//**************************************************************************
//  Timers
//**************************************************************************

stm32f1_timer_device::stm32f1_timer_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, STM32F1_TIMER, tag, owner, clock)
	, m_update_irq_cb(*this)
	, m_cc_irq_cb(*this)
{
}

void stm32f1_timer_device::device_start()
{
	m_timer = timer_alloc(FUNC(stm32f1_timer_device::overflow), this);

	save_item(NAME(m_start_time));
	save_item(NAME(m_start_count));
	save_item(NAME(m_cr1));
	save_item(NAME(m_cr2));
	save_item(NAME(m_smcr));
	save_item(NAME(m_dier));
	save_item(NAME(m_sr));
	save_item(NAME(m_ccmr));
	save_item(NAME(m_ccer));
	save_item(NAME(m_psc));
	save_item(NAME(m_psc_active));
	save_item(NAME(m_arr));
	save_item(NAME(m_arr_active));
	save_item(NAME(m_rcr));
	save_item(NAME(m_rep));
	save_item(NAME(m_ccr));
	save_item(NAME(m_bdtr));
}

void stm32f1_timer_device::device_reset()
{
	m_timer->adjust(attotime::never);
	m_start_time = machine().time();
	m_start_count = 0;
	m_cr1 = m_cr2 = m_smcr = m_dier = m_sr = 0;
	m_ccmr[0] = m_ccmr[1] = 0;
	m_ccer = 0;
	m_psc = m_psc_active = 0;
	m_arr = m_arr_active = 0xffff;
	m_rcr = m_rep = 0;
	std::fill_n(m_ccr, 4, 0);
	m_bdtr = 0;
	update_irq();
}

attotime stm32f1_timer_device::tick_period() const
{
	return attotime::from_ticks(u64(m_psc_active) + 1, clock());
}

u32 stm32f1_timer_device::counter() const
{
	if (!BIT(m_cr1, 0))
		return m_start_count;
	const u64 ticks = (machine().time() - m_start_time).as_ticks(tick_period().as_hz());
	const u32 span = u32(m_arr_active) + 1;
	if (BIT(m_cr1, 4))
		return (m_start_count + span - ticks % span) % span;
	return (m_start_count + ticks) % span;
}

void stm32f1_timer_device::start(u32 count)
{
	m_start_count = count;
	m_start_time = machine().time();
	if (!BIT(m_cr1, 0))
	{
		m_timer->adjust(attotime::never);
		return;
	}
	const u32 remaining = BIT(m_cr1, 4) ? count + 1 : u32(m_arr_active) + 1 - count;
	m_timer->adjust(tick_period() * remaining);
}

void stm32f1_timer_device::update_event()
{
	m_psc_active = m_psc;
	m_arr_active = m_arr;
	m_rep = m_rcr;
}

void stm32f1_timer_device::update_irq()
{
	const bool update = (m_sr & m_dier & 0x0001) != 0;
	const bool cc = (m_sr & m_dier & 0x001e) != 0;
	m_update_irq_cb(update ? ASSERT_LINE : CLEAR_LINE);
	m_cc_irq_cb(cc ? ASSERT_LINE : CLEAR_LINE);
}

TIMER_CALLBACK_MEMBER(stm32f1_timer_device::overflow)
{
	if (m_rep)
		m_rep--;
	else
	{
		if (!BIT(m_cr1, 1))
		{
			update_event();
			m_sr |= 0x0001;
		}
		if (BIT(m_cr1, 3))
			m_cr1 &= ~1;
	}
	start(BIT(m_cr1, 4) ? m_arr_active : 0);
	update_irq();
}

u32 stm32f1_timer_device::read(offs_t offset)
{
	switch (offset)
	{
	case 0x00 / 4: return m_cr1;
	case 0x04 / 4: return m_cr2;
	case 0x08 / 4: return m_smcr;
	case 0x0c / 4: return m_dier;
	case 0x10 / 4: return m_sr;
	case 0x18 / 4: return m_ccmr[0];
	case 0x1c / 4: return m_ccmr[1];
	case 0x20 / 4: return m_ccer;
	case 0x24 / 4: return counter();
	case 0x28 / 4: return m_psc;
	case 0x2c / 4: return m_arr;
	case 0x30 / 4: return m_rcr;
	case 0x34 / 4: case 0x38 / 4: case 0x3c / 4: case 0x40 / 4: return m_ccr[offset - 0x34 / 4];
	case 0x44 / 4: return m_bdtr;
	default: return 0;
	}
}

void stm32f1_timer_device::write(offs_t offset, u32 data, u32 mem_mask)
{
	const u16 value = data & mem_mask;
	switch (offset)
	{
	case 0x00 / 4:
	{
		const u32 count = counter();
		m_cr1 = value & 0x03ff;
		if (!BIT(m_cr1, 7))
			m_arr_active = m_arr;
		start(count);
		break;
	}
	case 0x04 / 4: m_cr2 = value; break;
	case 0x08 / 4: m_smcr = value; break;
	case 0x0c / 4: m_dier = value; update_irq(); break;
	case 0x10 / 4: m_sr &= value; update_irq(); break;
	case 0x14 / 4:
		if (BIT(value, 0))
		{
			update_event();
			if (!BIT(m_cr1, 2))
				m_sr |= 0x0001;
			start(BIT(m_cr1, 4) ? m_arr_active : 0);
		}
		m_sr |= value & 0x00de;
		update_irq();
		break;
	case 0x18 / 4: m_ccmr[0] = value; break;
	case 0x1c / 4: m_ccmr[1] = value; break;
	case 0x20 / 4: m_ccer = value; break;
	case 0x24 / 4: start(value); break;
	case 0x28 / 4: m_psc = value; break;
	case 0x2c / 4:
	{
		m_arr = value;
		if (!BIT(m_cr1, 7))
		{
			const u32 count = counter();
			m_arr_active = value;
			start(std::min<u32>(count, value));
		}
		break;
	}
	case 0x30 / 4: m_rcr = value & 0xff; break;
	case 0x34 / 4: case 0x38 / 4: case 0x3c / 4: case 0x40 / 4: m_ccr[offset - 0x34 / 4] = value; break;
	case 0x44 / 4: m_bdtr = value; break;
	}
}


//**************************************************************************
//  ADC
//**************************************************************************

stm32f1_adc_device::stm32f1_adc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, STM32F1_ADC, tag, owner, clock)
	, m_in_cb(*this, 0)
	, m_irq_cb(*this)
{
}

void stm32f1_adc_device::device_start()
{
	m_timer = timer_alloc(FUNC(stm32f1_adc_device::converted), this);

	save_item(NAME(m_sr));
	save_item(NAME(m_cr1));
	save_item(NAME(m_cr2));
	save_item(NAME(m_smpr));
	save_item(NAME(m_jofr));
	save_item(NAME(m_htr));
	save_item(NAME(m_ltr));
	save_item(NAME(m_sqr));
	save_item(NAME(m_jsqr));
	save_item(NAME(m_jdr));
	save_item(NAME(m_dr));
	save_item(NAME(m_seq));
}

void stm32f1_adc_device::device_reset()
{
	m_timer->adjust(attotime::never);
	m_sr = m_cr1 = m_cr2 = 0;
	m_smpr[0] = m_smpr[1] = 0;
	std::fill_n(m_jofr, 4, 0);
	m_htr = 0xfff;
	m_ltr = 0;
	std::fill_n(m_sqr, 3, 0);
	m_jsqr = 0;
	std::fill_n(m_jdr, 4, 0);
	m_dr = 0;
	m_seq = 0;
	update_irq();
}

unsigned stm32f1_adc_device::channel(unsigned index) const
{
	return BIT(m_sqr[2 - index / 6], (index % 6) * 5, 5);
}

attotime stm32f1_adc_device::conversion_time(unsigned ch) const
{
	static constexpr unsigned SAMPLE[8] = { 3, 15, 27, 57, 83, 111, 143, 479 };
	const unsigned smp = ch < 10 ? BIT(m_smpr[1], ch * 3, 3) : BIT(m_smpr[0], (ch - 10) * 3, 3);
	return attotime::from_ticks(SAMPLE[smp] + 25, clock());
}

void stm32f1_adc_device::start_regular()
{
	m_seq = 0;
	m_sr |= 0x10;
	m_timer->adjust(conversion_time(channel(0)));
}

void stm32f1_adc_device::update_irq()
{
	const bool irq = (BIT(m_cr1, 5) && BIT(m_sr, 1)) || (BIT(m_cr1, 7) && BIT(m_sr, 2)) || (BIT(m_cr1, 6) && BIT(m_sr, 0));
	m_irq_cb(irq ? ASSERT_LINE : CLEAR_LINE);
}

TIMER_CALLBACK_MEMBER(stm32f1_adc_device::converted)
{
	const unsigned ch = channel(m_seq);
	const u16 value = (ch < 18 ? m_in_cb[ch]() : 0) & 0xfff;
	m_dr = BIT(m_cr2, 11) ? value << 4 : value;

	const unsigned length = BIT(m_cr1, 8) ? BIT(m_sqr[0], 20, 4) + 1 : 1;
	m_seq++;
	if (m_seq < length)
	{
		m_timer->adjust(conversion_time(channel(m_seq)));
		return;
	}

	m_sr |= 0x02;
	if (BIT(m_cr2, 1))
		start_regular();
	update_irq();
}

u32 stm32f1_adc_device::read(offs_t offset)
{
	switch (offset)
	{
	case 0x00 / 4: return m_sr;
	case 0x04 / 4: return m_cr1;
	case 0x08 / 4: return m_cr2;
	case 0x0c / 4: case 0x10 / 4: return m_smpr[offset - 0x0c / 4];
	case 0x14 / 4: case 0x18 / 4: case 0x1c / 4: case 0x20 / 4: return m_jofr[offset - 0x14 / 4];
	case 0x24 / 4: return m_htr;
	case 0x28 / 4: return m_ltr;
	case 0x2c / 4: case 0x30 / 4: case 0x34 / 4: return m_sqr[offset - 0x2c / 4];
	case 0x38 / 4: return m_jsqr;
	case 0x3c / 4: case 0x40 / 4: case 0x44 / 4: case 0x48 / 4: return m_jdr[offset - 0x3c / 4];
	case 0x4c / 4:
		if (!machine().side_effects_disabled())
		{
			m_sr &= ~0x02;
			update_irq();
		}
		return m_dr;
	default: return 0;
	}
}

void stm32f1_adc_device::write(offs_t offset, u32 data, u32 mem_mask)
{
	switch (offset)
	{
	case 0x00 / 4:
		m_sr &= data | ~0x1f;
		update_irq();
		break;
	case 0x04 / 4:
		COMBINE_DATA(&m_cr1);
		update_irq();
		break;
	case 0x08 / 4:
	{
		const u32 old = m_cr2;
		COMBINE_DATA(&m_cr2);
		m_cr2 &= ~(0x0000000c | 0x00600000);
		const bool swstart = BIT(data, 22) && BIT(m_cr2, 20) && BIT(m_cr2, 17, 3) == 7;
		if (BIT(old, 0) && BIT(m_cr2, 0) && (swstart || (BIT(data, 0) && !(data & ~old & 0x00ffffff & ~1))))
			start_regular();
		if (!BIT(m_cr2, 0))
			m_timer->adjust(attotime::never);
		break;
	}
	case 0x0c / 4: case 0x10 / 4: COMBINE_DATA(&m_smpr[offset - 0x0c / 4]); break;
	case 0x14 / 4: case 0x18 / 4: case 0x1c / 4: case 0x20 / 4: COMBINE_DATA(&m_jofr[offset - 0x14 / 4]); break;
	case 0x24 / 4: COMBINE_DATA(&m_htr); break;
	case 0x28 / 4: COMBINE_DATA(&m_ltr); break;
	case 0x2c / 4: case 0x30 / 4: case 0x34 / 4: COMBINE_DATA(&m_sqr[offset - 0x2c / 4]); break;
	case 0x38 / 4: COMBINE_DATA(&m_jsqr); break;
	}
}


//**************************************************************************
//  USART
//**************************************************************************

stm32f1_usart_device::stm32f1_usart_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, STM32F1_USART, tag, owner, clock)
	, device_serial_interface(mconfig, *this)
	, m_txd_cb(*this)
	, m_irq_cb(*this)
{
}

void stm32f1_usart_device::device_start()
{
	save_item(NAME(m_sr));
	save_item(NAME(m_rdr));
	save_item(NAME(m_tdr));
	save_item(NAME(m_brr));
	save_item(NAME(m_cr1));
	save_item(NAME(m_cr2));
	save_item(NAME(m_cr3));
	save_item(NAME(m_gtpr));
	save_item(NAME(m_tdr_full));
	save_item(NAME(m_tx_busy));
	save_item(NAME(m_sr_read));
}

void stm32f1_usart_device::device_reset()
{
	m_sr = 0x00c0;
	m_rdr = m_tdr = 0;
	m_brr = 0;
	m_cr1 = m_cr2 = m_cr3 = 0;
	m_gtpr = 0;
	m_tdr_full = false;
	m_tx_busy = false;
	m_sr_read = false;
	receive_register_reset();
	transmit_register_reset();
	update_frame();
	m_txd_cb(1);
	update_irq();
}

void stm32f1_usart_device::update_frame()
{
	const bool parity = BIT(m_cr1, 10);
	const int bits = (BIT(m_cr1, 12) ? 9 : 8) - (parity ? 1 : 0);
	static constexpr stop_bits_t STOP[4] = { STOP_BITS_1, STOP_BITS_0, STOP_BITS_2, STOP_BITS_1_5 };
	set_data_frame(1, bits, !parity ? PARITY_NONE : BIT(m_cr1, 9) ? PARITY_ODD : PARITY_EVEN, STOP[BIT(m_cr2, 12, 2)]);
}

void stm32f1_usart_device::update_irq()
{
	const u16 enabled = (m_cr1 & 0x01f0) | (BIT(m_cr1, 8) ? 0x0001 : 0);
	const bool irq = (m_sr & enabled & 0x01f1) || (BIT(m_cr3, 0) && BIT(m_cr1, 5) && (m_sr & 0x000e));
	m_irq_cb(irq ? ASSERT_LINE : CLEAR_LINE);
}

void stm32f1_usart_device::start_tx()
{
	if (m_tx_busy || !m_tdr_full || !BIT(m_cr1, 13) || !BIT(m_cr1, 3))
		return;
	m_tdr_full = false;
	m_tx_busy = true;
	m_sr |= 0x0080;
	transmit_register_setup(m_tdr);
	update_irq();
}

void stm32f1_usart_device::tra_callback()
{
	m_txd_cb(transmit_register_get_data_bit());
}

void stm32f1_usart_device::tra_complete()
{
	m_tx_busy = false;
	start_tx();
	if (!m_tx_busy)
		m_sr |= 0x0040;
	update_irq();
}

void stm32f1_usart_device::rcv_complete()
{
	receive_register_extract();
	if (!BIT(m_cr1, 13) || !BIT(m_cr1, 2))
		return;
	if (BIT(m_sr, 5))
		m_sr |= 0x0008;
	else
	{
		m_rdr = get_received_char();
		m_sr |= 0x0020;
		if (is_receive_parity_error())
			m_sr |= 0x0001;
		if (is_receive_framing_error())
			m_sr |= 0x0002;
	}
	update_irq();
}

void stm32f1_usart_device::rxd_w(int state)
{
	device_serial_interface::rx_w(state);
}

u32 stm32f1_usart_device::read(offs_t offset)
{
	switch (offset)
	{
	case 0x00 / 4:
		if (!machine().side_effects_disabled())
			m_sr_read = true;
		return m_sr;
	case 0x04 / 4:
		if (!machine().side_effects_disabled())
		{
			m_sr &= ~0x0020;
			if (m_sr_read)
				m_sr &= ~0x001f;
			m_sr_read = false;
			update_irq();
		}
		return m_rdr;
	case 0x08 / 4: return m_brr;
	case 0x0c / 4: return m_cr1;
	case 0x10 / 4: return m_cr2;
	case 0x14 / 4: return m_cr3;
	case 0x18 / 4: return m_gtpr;
	default: return 0;
	}
}

void stm32f1_usart_device::write(offs_t offset, u32 data, u32 mem_mask)
{
	const u16 value = data & mem_mask;
	switch (offset)
	{
	case 0x00 / 4:
		m_sr &= value | ~0x0360;
		update_irq();
		break;
	case 0x04 / 4:
		m_tdr = value & 0x1ff;
		m_tdr_full = true;
		m_sr &= ~0x0080;
		if (m_sr_read)
			m_sr &= ~0x0040;
		m_sr_read = false;
		start_tx();
		update_irq();
		break;
	case 0x08 / 4:
		m_brr = value;
		if (m_brr >= 16)
			set_rate(clock(), m_brr);
		break;
	case 0x0c / 4:
		if ((m_cr1 ^ value) & 0x1600)
		{
			m_cr1 = value & 0x3fff;
			update_frame();
		}
		m_cr1 = value & 0x3fff;
		start_tx();
		update_irq();
		break;
	case 0x10 / 4:
		if ((m_cr2 ^ value) & 0x3000)
		{
			m_cr2 = value & 0x7f7f;
			update_frame();
		}
		m_cr2 = value & 0x7f7f;
		break;
	case 0x14 / 4:
		m_cr3 = value & 0x07ff;
		update_irq();
		break;
	case 0x18 / 4:
		m_gtpr = value;
		break;
	}
}


//**************************************************************************
//  The microcontroller
//**************************************************************************

stm32f103_device::stm32f103_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: cortex_m3_device(mconfig, STM32F103, tag, owner, clock, address_map_constructor(FUNC(stm32f103_device::internal_map), this))
	, m_flash(*this, DEVICE_SELF)
	, m_gpio(*this, "gpio%c", 'a')
	, m_timer(*this, "tim%u", 1U)
	, m_adc(*this, "adc%u", 1U)
	, m_usart(*this, "usart%u", 1U)
{
	set_num_irq(60);
	set_priority_bits(4);
}

void stm32f103_device::device_add_mconfig(machine_config &config)
{
	for (unsigned i = 0; i < 5; i++)
		STM32F1_GPIO(config, m_gpio[i], DERIVED_CLOCK(1, 1));

	static constexpr int TIMER_UP_IRQ[8] = { 25, 28, 29, 30, 50, 54, 55, 44 };
	static constexpr int TIMER_CC_IRQ[8] = { 27, 28, 29, 30, 50, 54, 55, 46 };
	for (unsigned i = 0; i < 8; i++)
	{
		STM32F1_TIMER(config, m_timer[i], DERIVED_CLOCK(1, 1));
		if (TIMER_UP_IRQ[i] == TIMER_CC_IRQ[i])
		{
			m_timer[i]->update_irq_cb().set([this, i] (int state) { m_irq_share[i] = (m_irq_share[i] & 2) | (state ? 1 : 0); set_irq_line(TIMER_UP_IRQ[i], m_irq_share[i] ? ASSERT_LINE : CLEAR_LINE); });
			m_timer[i]->cc_irq_cb().set([this, i] (int state) { m_irq_share[i] = (m_irq_share[i] & 1) | (state ? 2 : 0); set_irq_line(TIMER_UP_IRQ[i], m_irq_share[i] ? ASSERT_LINE : CLEAR_LINE); });
		}
		else
		{
			m_timer[i]->update_irq_cb().set([this, i] (int state) { set_irq_line(TIMER_UP_IRQ[i], state); });
			m_timer[i]->cc_irq_cb().set([this, i] (int state) { set_irq_line(TIMER_CC_IRQ[i], state); });
		}
	}

	for (unsigned i = 0; i < 3; i++)
		STM32F1_ADC(config, m_adc[i], DERIVED_CLOCK(1, 2));
	m_adc[0]->irq_cb().set([this] (int state) { m_irq_share[8] = (m_irq_share[8] & 2) | (state ? 1 : 0); set_irq_line(18, m_irq_share[8] ? ASSERT_LINE : CLEAR_LINE); });
	m_adc[1]->irq_cb().set([this] (int state) { m_irq_share[8] = (m_irq_share[8] & 1) | (state ? 2 : 0); set_irq_line(18, m_irq_share[8] ? ASSERT_LINE : CLEAR_LINE); });
	m_adc[2]->irq_cb().set([this] (int state) { set_irq_line(47, state); });

	static constexpr int USART_IRQ[5] = { 37, 38, 39, 52, 53 };
	for (unsigned i = 0; i < 5; i++)
	{
		STM32F1_USART(config, m_usart[i], DERIVED_CLOCK(1, 1));
		m_usart[i]->irq_cb().set([this, i] (int state) { set_irq_line(USART_IRQ[i], state); });
	}
}

void stm32f103_device::internal_map(address_map &map)
{
	map(0x00000000, 0x5fffffff).rw(FUNC(stm32f103_device::unmapped_r), FUNC(stm32f103_device::unmapped_w));
	map(0x00000000, 0x0007ffff).r(FUNC(stm32f103_device::flash_r));
	map(0x08000000, 0x0807ffff).r(FUNC(stm32f103_device::flash_r));
	map(0x20000000, 0x2000ffff).ram();

	static constexpr u32 TIMER_BASE[8] = { 0x40012c00, 0x40000000, 0x40000400, 0x40000800, 0x40000c00, 0x40001000, 0x40001400, 0x40013400 };
	for (unsigned i = 0; i < 8; i++)
		map(TIMER_BASE[i], TIMER_BASE[i] + 0x3ff).rw(m_timer[i], FUNC(stm32f1_timer_device::read), FUNC(stm32f1_timer_device::write));

	static constexpr u32 USART_BASE[5] = { 0x40013800, 0x40004400, 0x40004800, 0x40004c00, 0x40005000 };
	for (unsigned i = 0; i < 5; i++)
		map(USART_BASE[i], USART_BASE[i] + 0x3ff).rw(m_usart[i], FUNC(stm32f1_usart_device::read), FUNC(stm32f1_usart_device::write));

	for (unsigned i = 0; i < 5; i++)
		map(0x40010800 + 0x400 * i, 0x40010bff + 0x400 * i).rw(m_gpio[i], FUNC(stm32f1_gpio_device::read), FUNC(stm32f1_gpio_device::write));

	static constexpr u32 ADC_BASE[3] = { 0x40012400, 0x40012800, 0x40013c00 };
	for (unsigned i = 0; i < 3; i++)
		map(ADC_BASE[i], ADC_BASE[i] + 0x3ff).rw(m_adc[i], FUNC(stm32f1_adc_device::read), FUNC(stm32f1_adc_device::write));

	map(0x40021000, 0x400213ff).rw(FUNC(stm32f103_device::rcc_r), FUNC(stm32f103_device::rcc_w));
}

void stm32f103_device::device_start()
{
	const offs_t length = std::min<offs_t>(m_flash.bytes(), 0x80000);
	if (length >= 4)
	{
		space(AS_PROGRAM).install_rom(0x00000000, length - 1, &m_flash[0]);
		space(AS_PROGRAM).install_rom(0x08000000, 0x08000000 + length - 1, &m_flash[0]);
	}

	cortex_m3_device::device_start();

	save_item(NAME(m_rcc_cr));
	save_item(NAME(m_rcc_cfgr));
	save_item(NAME(m_irq_share));
}

void stm32f103_device::device_reset()
{
	m_regs.clear();
	m_rcc_cr = 0x00000083;
	m_rcc_cfgr = 0;
	std::fill_n(m_irq_share, std::size(m_irq_share), 0);

	cortex_m3_device::device_reset();
}

u32 stm32f103_device::flash_r(offs_t offset)
{
	return offset < m_flash.length() ? m_flash[offset] : 0xffffffff;
}

u32 stm32f103_device::rcc_r(offs_t offset, u32 mem_mask)
{
	switch (offset)
	{
	case 0x00 / 4:
		return m_rcc_cr;
	case 0x04 / 4:
		return m_rcc_cfgr;
	default:
		return unmapped_r(0x40021000 / 4 + offset, mem_mask);
	}
}

void stm32f103_device::rcc_w(offs_t offset, u32 data, u32 mem_mask)
{
	switch (offset)
	{
	case 0x00 / 4:
	{
		u32 cr = m_rcc_cr;
		COMBINE_DATA(&cr);
		cr = (cr & 0x010d00f9) | (m_rcc_cr & 0x0000ff00);
		if (BIT(cr, 0))
			cr |= 1 << 1;
		if (BIT(cr, 16))
			cr |= 1 << 17;
		if (BIT(cr, 24))
			cr |= 1 << 25;
		m_rcc_cr = cr;
		break;
	}
	case 0x04 / 4:
		COMBINE_DATA(&m_rcc_cfgr);
		m_rcc_cfgr = (m_rcc_cfgr & ~0x0000000c) | (BIT(m_rcc_cfgr, 0, 2) << 2);
		break;
	default:
		unmapped_w(0x40021000 / 4 + offset, data, mem_mask);
		break;
	}
}

u32 stm32f103_device::unmapped_r(offs_t offset, u32 mem_mask)
{
	const auto it = m_regs.find(offset);
	const u32 data = it != m_regs.end() ? it->second : 0;
	if (!machine().side_effects_disabled())
		LOGUNMAPPED("%s: read %08x & %08x = %08x\n", machine().describe_context(), offset * 4, mem_mask, data);
	return data;
}

void stm32f103_device::unmapped_w(offs_t offset, u32 data, u32 mem_mask)
{
	LOGUNMAPPED("%s: write %08x = %08x & %08x\n", machine().describe_context(), offset * 4, data, mem_mask);
	COMBINE_DATA(&m_regs[offset]);
}
