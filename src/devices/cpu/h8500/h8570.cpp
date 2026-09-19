// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Hitachi H8/570

    An H8/500 with no on-chip ROM, 2 KB of RAM, one SCI, an A/D converter,
    a watchdog, a PWM timer and the ISP, a microprogrammed sub-processor
    whose program is fixed in the part.  The ISP itself is not emulated:
    this device carries the registers the CPU shares with it, and the
    board models what the sub-processor does with them.

***************************************************************************/

#include "emu.h"
#include "h8570.h"

DEFINE_DEVICE_TYPE(H8570, h8570_device, "h8570", "Hitachi H8/570")

h8570_device::h8570_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: h8500_device(mconfig, H8570, tag, owner, clock, 20, 16, 11, 3, address_map_constructor(FUNC(h8570_device::internal_map), this))
	, m_intc(*this, "intc")
	, m_adc(*this, "adc")
	, m_port1(*this, "port1")
	, m_port5(*this, "port5")
	, m_port6(*this, "port6")
	, m_port7(*this, "port7")
	, m_port8(*this, "port8")
	, m_port9(*this, "port9")
	, m_port10(*this, "port10")
	, m_port11(*this, "port11")
	, m_port12(*this, "port12")
	, m_pwm(*this, "pwm")
	, m_watchdog(*this, "watchdog")
	, m_pwm_out_cb(*this)
	, m_isf_ack_cb(*this)
	, m_isp_reset_cb(*this)
	, m_isp_ibe_cb(*this)
	, m_dr_write_cb(*this)
	, m_isf(0), m_icf(0), m_ief(0), m_ioie(0), m_cle(0)
	, m_iof(0)
	, m_egf(0), m_ever(0), m_ipr(0), m_icsr(0x20), m_fedge(0), m_redge(0)
	, m_dr{}
	, m_syscr{}
	, m_wsc(0), m_ramcr(0xff), m_sbycr(0x7f)
{
}

void h8570_device::device_add_mconfig(machine_config &config)
{
	H8570_INTC(config, m_intc, *this);
	H8_ADC_3337(config, m_adc, *this, m_intc, 60);
	H8_PORT(config, m_port1, *this, h8500_device::PORT_1, 0x00, 0x00);
	H8_PORT(config, m_port5, *this, h8500_device::PORT_5, 0x00, 0x00);
	H8_PORT(config, m_port6, *this, h8500_device::PORT_6, 0x00, 0x00);
	H8_PORT(config, m_port7, *this, h8500_device::PORT_7, 0x00, 0x00, 0xff);
	H8_PORT(config, m_port8, *this, h8500_device::PORT_8, 0x00, 0x00);
	H8_PORT(config, m_port9, *this, h8500_device::PORT_9, 0x00, 0x00);
	H8_PORT(config, m_port10, *this, h8500_device::PORT_10, 0x01, 0x00);
	H8_PORT(config, m_port11, *this, h8500_device::PORT_11, 0x00, 0x00);
	H8_PORT(config, m_port12, *this, h8500_device::PORT_12, 0x00, 0x00);
	H8_SCI(config, m_sci[0], 0, *this, m_intc, 56, 57, 58, 59);
	H8570_PWM(config, m_pwm, *this, m_intc, 36);
	m_pwm->out_cb().set(FUNC(h8570_device::pwm_out_w));
	m_pwm->icf_cb().set(FUNC(h8570_device::pwm_icf_w));
	H8_WATCHDOG(config, m_watchdog, *this, m_intc, 34, h8_watchdog_device::H);
}

void h8570_device::device_start()
{
	h8500_device::device_start();

	save_item(NAME(m_isf));
	save_item(NAME(m_icf));
	save_item(NAME(m_ief));
	save_item(NAME(m_ioie));
	save_item(NAME(m_cle));
	save_item(NAME(m_iof));
	save_item(NAME(m_egf));
	save_item(NAME(m_ever));
	save_item(NAME(m_ipr));
	save_item(NAME(m_icsr));
	save_item(NAME(m_fedge));
	save_item(NAME(m_redge));
	save_item(NAME(m_dr));
	save_item(NAME(m_syscr));
	save_item(NAME(m_wsc));
	save_item(NAME(m_ramcr));
	save_item(NAME(m_sbycr));
}

void h8570_device::device_reset()
{
	h8500_device::device_reset();

	m_isf = m_icf = m_ief = m_ioie = m_cle = 0;
	m_egf = m_ever = m_ipr = 0;
	m_icsr = 0x20;
	m_fedge = m_redge = 0;
	std::fill(std::begin(m_syscr), std::end(m_syscr), 0);
	m_wsc = 0;
	m_ramcr = 0xff;
	m_sbycr = 0x7f;
	m_isp_reset_cb(1);
	m_isp_ibe_cb(1);
}

void h8570_device::internal_map(address_map &map)
{
	map(0xf680, 0xfe7f).ram().share("iram");
	register_field_map(map, 0);
	if (h8_maximum_mode())
	{
		map(0xff680, 0xffe7f).ram().share("iram");
		register_field_map(map, 0xf0000);
	}
}

void h8570_device::register_field_map(address_map &map, offs_t base)
{
	map(base + 0xfe80, base + 0xfe87).r(m_adc, FUNC(h8_adc_device::addr8_r));
	map(base + 0xfe88, base + 0xfe88).rw(m_adc, FUNC(h8_adc_device::adcsr_r), FUNC(h8_adc_device::adcsr_w));
	map(base + 0xfe89, base + 0xfe89).rw(m_adc, FUNC(h8_adc_device::adcr_r), FUNC(h8_adc_device::adcr_w));
	map(base + 0xfe8a, base + 0xfe8b).rw(m_watchdog, FUNC(h8_watchdog_device::wd_r), FUNC(h8_watchdog_device::wd_w));

	map(base + 0xfe8c, base + 0xfe8c).rw(m_port1, FUNC(h8_port_device::port_r), FUNC(h8_port_device::dr_w));
	map(base + 0xfe90, base + 0xfe90).rw(m_port5, FUNC(h8_port_device::port_r), FUNC(h8_port_device::dr_w));
	map(base + 0xfe91, base + 0xfe91).rw(m_port6, FUNC(h8_port_device::port_r), FUNC(h8_port_device::dr_w));
	map(base + 0xfe92, base + 0xfe92).r(m_port7, FUNC(h8_port_device::port_r));
	map(base + 0xfe93, base + 0xfe93).rw(m_port8, FUNC(h8_port_device::port_r), FUNC(h8_port_device::dr_w));
	map(base + 0xfe94, base + 0xfe94).rw(m_port9, FUNC(h8_port_device::port_r), FUNC(h8_port_device::dr_w));
	map(base + 0xfe95, base + 0xfe95).rw(m_port10, FUNC(h8_port_device::port_r), FUNC(h8_port_device::dr_w));
	map(base + 0xfe96, base + 0xfe96).rw(m_port11, FUNC(h8_port_device::port_r), FUNC(h8_port_device::dr_w));
	map(base + 0xfe97, base + 0xfe97).rw(m_port12, FUNC(h8_port_device::port_r), FUNC(h8_port_device::dr_w));

	map(base + 0xfe98, base + 0xfe98).rw(m_sci[0], FUNC(h8_sci_device::smr_r), FUNC(h8_sci_device::smr_w));
	map(base + 0xfe99, base + 0xfe99).rw(m_sci[0], FUNC(h8_sci_device::brr_r), FUNC(h8_sci_device::brr_w));
	map(base + 0xfe9a, base + 0xfe9a).rw(m_sci[0], FUNC(h8_sci_device::scr_r), FUNC(h8_sci_device::scr_w));
	map(base + 0xfe9b, base + 0xfe9b).rw(m_sci[0], FUNC(h8_sci_device::tdr_r), FUNC(h8_sci_device::tdr_w));
	map(base + 0xfe9c, base + 0xfe9c).rw(m_sci[0], FUNC(h8_sci_device::ssr_r), FUNC(h8_sci_device::ssr_w));
	map(base + 0xfe9d, base + 0xfe9d).r(m_sci[0], FUNC(h8_sci_device::rdr_r));

	map(base + 0xfea0, base + 0xfea0).rw(m_pwm, FUNC(h8570_pwm_device::tcr_r), FUNC(h8570_pwm_device::tcr_w));
	map(base + 0xfea1, base + 0xfea1).rw(m_pwm, FUNC(h8570_pwm_device::tmsr_r), FUNC(h8570_pwm_device::tmsr_w));
	map(base + 0xfea2, base + 0xfea2).rw(m_pwm, FUNC(h8570_pwm_device::odl_r), FUNC(h8570_pwm_device::odl_w));
	map(base + 0xfea3, base + 0xfea5).rw(m_pwm, FUNC(h8570_pwm_device::odr_r), FUNC(h8570_pwm_device::odr_w));
	map(base + 0xfea6, base + 0xfeab).rw(m_pwm, FUNC(h8570_pwm_device::ocr_r), FUNC(h8570_pwm_device::ocr_w));
	map(base + 0xfeac, base + 0xfead).rw(m_pwm, FUNC(h8570_pwm_device::tmr_r), FUNC(h8570_pwm_device::tmr_w));

	map(base + 0xfeb0, base + 0xfebf).rw(FUNC(h8570_device::isp_r), FUNC(h8570_device::isp_w));
	map(base + 0xfec0, base + 0xfeff).rw(FUNC(h8570_device::dr_r), FUNC(h8570_device::dr_w));

	map(base + 0xff18, base + 0xff19).rw(FUNC(h8570_device::isp_ctl_r), FUNC(h8570_device::isp_ctl_w));
	map(base + 0xff23, base + 0xff25).rw(FUNC(h8570_device::syscr_r), FUNC(h8570_device::syscr_w));
	map(base + 0xff28, base + 0xff29).rw(FUNC(h8570_device::edge_r), FUNC(h8570_device::edge_w));

	map(base + 0xff2c, base + 0xff2c).rw(m_port1, FUNC(h8_port_device::ff_r), FUNC(h8_port_device::ddr_w));
	map(base + 0xff30, base + 0xff30).rw(m_port5, FUNC(h8_port_device::ff_r), FUNC(h8_port_device::ddr_w));
	map(base + 0xff31, base + 0xff31).rw(m_port6, FUNC(h8_port_device::ff_r), FUNC(h8_port_device::ddr_w));
	map(base + 0xff33, base + 0xff33).rw(m_port8, FUNC(h8_port_device::ff_r), FUNC(h8_port_device::ddr_w));
	map(base + 0xff34, base + 0xff34).rw(m_port9, FUNC(h8_port_device::ff_r), FUNC(h8_port_device::ddr_w));
	map(base + 0xff35, base + 0xff35).rw(m_port10, FUNC(h8_port_device::ff_r), FUNC(h8_port_device::ddr_w));
	map(base + 0xff36, base + 0xff36).rw(m_port11, FUNC(h8_port_device::ff_r), FUNC(h8_port_device::ddr_w));
	map(base + 0xff37, base + 0xff37).rw(m_port12, FUNC(h8_port_device::ff_r), FUNC(h8_port_device::ddr_w));

	map(base + 0xff40, base + 0xff40).rw(m_intc, FUNC(h8500_intc_device::ipra_r), FUNC(h8500_intc_device::ipra_w));
	map(base + 0xff41, base + 0xff41).rw(m_intc, FUNC(h8500_intc_device::iprb_r), FUNC(h8500_intc_device::iprb_w));
	map(base + 0xff42, base + 0xff42).rw(m_intc, FUNC(h8500_intc_device::iprc_r), FUNC(h8500_intc_device::iprc_w));
	map(base + 0xff43, base + 0xff43).rw(m_intc, FUNC(h8500_intc_device::iprd_r), FUNC(h8500_intc_device::iprd_w));
	map(base + 0xff44, base + 0xff44).rw(m_intc, FUNC(h8500_intc_device::dtea_r), FUNC(h8500_intc_device::dtea_w));
	map(base + 0xff45, base + 0xff45).rw(m_intc, FUNC(h8500_intc_device::dteb_r), FUNC(h8500_intc_device::dteb_w));
	map(base + 0xff46, base + 0xff46).rw(m_intc, FUNC(h8500_intc_device::dtec_r), FUNC(h8500_intc_device::dtec_w));
	map(base + 0xff47, base + 0xff47).rw(m_intc, FUNC(h8500_intc_device::dted_r), FUNC(h8500_intc_device::dted_w));

	map(base + 0xff48, base + 0xff4b).rw(FUNC(h8570_device::sysctl_r), FUNC(h8570_device::sysctl_w));
	map(base + 0xff4c, base + 0xff4c).rw(m_intc, FUNC(h8570_intc_device::syscr1_r), FUNC(h8570_intc_device::syscr1_w));
	map(base + 0xff4e, base + 0xff4f).rw(m_watchdog, FUNC(h8_watchdog_device::rst_r), FUNC(h8_watchdog_device::rst_w));
}

void h8570_device::isf_update(u16 before)
{
	const u16 rising = (m_isf & m_ief) & ~before;
	for (int n = 0; n < 16; n++)
		if (BIT(rising, n))
			m_intc->internal_interrupt(40 + n);
}

void h8570_device::dr_w(offs_t offset, u8 data)
{
	m_dr[offset] = data;
	m_dr_write_cb(offset, data);
}

void h8570_device::isp_raise(int n)
{
	const u16 before = m_isf & m_ief;
	m_isf |= 1 << (n & 15);
	isf_update(before);
}

void h8570_device::isp_set_icf(int n, int state)
{
	if (state)
		m_icf |= 1 << (n & 15);
	else
		m_icf &= ~(1 << (n & 15));
}

void h8570_device::isp_set_iof(u32 iof)
{
	m_iof = iof & 0xffffff;
}

u8 h8570_device::isp_r(offs_t offset)
{
	switch (offset)
	{
	case 0x0: return m_isf >> 8;
	case 0x1: return m_isf & 0xff;
	case 0x2: return m_iof >> 16;
	case 0x3: return (m_iof >> 8) & 0xff;
	case 0x4: return m_iof & 0xff;
	case 0x5: return m_egf;
	case 0x6:
	{
		const u8 v = m_icf >> 8;
		if (!machine().side_effects_disabled())
			m_icf &= ~(m_cle & 0xff00);
		return v;
	}
	case 0x7:
	{
		const u8 v = m_icf & 0xff;
		if (!machine().side_effects_disabled())
			m_icf &= ~(m_cle & 0x00ff);
		return v;
	}
	case 0x8: return m_ief >> 8;
	case 0x9: return m_ief & 0xff;
	case 0xa: return m_ioie >> 8;
	case 0xb: return m_ioie & 0xff;
	case 0xc: return m_cle >> 8;
	case 0xd: return m_cle & 0xff;
	case 0xf: return m_ever;
	default: return 0xff;
	}
}

void h8570_device::isp_w(offs_t offset, u8 data)
{
	const u16 before = m_isf & m_ief;
	const int shift = BIT(offset, 0) ? 0 : 8;
	const u16 mask = 0xff << shift;
	const u16 value = u16(data) << shift;

	switch (offset)
	{
	case 0x0:
	case 0x1:
	{
		const u16 cleared = m_isf & ~value & mask;
		m_isf = (m_isf & ~mask) | value;
		isf_update(before);
		for (int n = 0; n < 16; n++)
			if (BIT(cleared, n))
				m_isf_ack_cb(n);
		break;
	}
	case 0x6:
	case 0x7:
		m_icf = (m_icf & ~mask) | value;
		break;
	case 0x8:
	case 0x9:
		m_ief = (m_ief & ~mask) | value;
		isf_update(before);
		break;
	case 0xa:
	case 0xb:
		m_ioie = (m_ioie & ~mask) | value;
		break;
	case 0xc:
	case 0xd:
		m_cle = (m_cle & ~mask) | value;
		break;
	case 0xf:
		m_ever = data & 0x07;
		break;
	default:
		logerror("ISP register %04X = %02X\n", 0xfeb0 + offset, data);
		break;
	}
}

u8 h8570_device::isp_ctl_r(offs_t offset)
{
	return offset ? m_icsr : m_ipr;
}

void h8570_device::isp_ctl_w(offs_t offset, u8 data)
{
	if (!offset)
	{
		m_ipr = data;
		return;
	}
	const bool held = isp_reset_held();
	m_icsr = data & 0x3f;
	if (held != isp_reset_held())
		m_isp_reset_cb(isp_reset_held());
}

u8 h8570_device::edge_r(offs_t offset)
{
	return offset ? m_redge : m_fedge;
}

void h8570_device::edge_w(offs_t offset, u8 data)
{
	(offset ? m_redge : m_fedge) = data;
}

u8 h8570_device::syscr_r(offs_t offset)
{
	return m_syscr[offset];
}

void h8570_device::syscr_w(offs_t offset, u8 data)
{
	m_syscr[offset] = data;
}

u8 h8570_device::sysctl_r(offs_t offset)
{
	switch (offset)
	{
	case 0: return m_wsc;
	case 1: return m_ramcr | 0x3f;
	case 2: return 0xc0 | (mode_control() & 7);
	default: return m_sbycr;
	}
}

void h8570_device::sysctl_w(offs_t offset, u8 data)
{
	switch (offset)
	{
	case 0:
		m_wsc = data;
		break;
	case 1:
	{
		const bool ibe = BIT(m_ramcr, 6);
		m_ramcr = data | 0x3f;
		if (ibe != BIT(m_ramcr, 6))
			m_isp_ibe_cb(BIT(m_ramcr, 6));
		break;
	}
	case 2:
		break;
	default:
		m_sbycr = data;
		break;
	}
}

void h8570_device::pwm_out_w(u8 data)
{
	m_pwm_out_cb(data);
}

void h8570_device::pwm_icf_w(u8 data)
{
	isp_set_icf(8 + data, 1);
}

void h8570_device::internal_update(u64 current_time)
{
	u64 event_time = 0;

	add_event(event_time, m_adc->internal_update(current_time));
	add_event(event_time, m_pwm->internal_update(current_time));
	add_event(event_time, m_sci[0]->internal_update(current_time));
	add_event(event_time, m_watchdog->internal_update(current_time));

	recompute_bcount(event_time);
}

void h8570_device::notify_standby(int state)
{
	m_adc->notify_standby(state);
	m_pwm->notify_standby(state);
	m_sci[0]->notify_standby(state);
	m_watchdog->notify_standby(state);
}

void h8570_device::execute_set_input(int inputnum, int state)
{
	m_intc->set_input(inputnum, state);
}

void h8570_device::interrupt_taken()
{
	standard_irq_callback(m_intc->interrupt_taken(m_taken_irq_vector), m_npc);
}

void h8570_device::update_irq_filter()
{
	m_intc->set_filter((m_sr & (SR_I2 | SR_I1 | SR_I0)) >> 8);
}

void h8570_device::irq_setup()
{
	m_sr &= ~(SR_T | SR_I2 | SR_I1 | SR_I0);
	m_sr |= std::min(m_taken_irq_level, 7) << 8;
}
