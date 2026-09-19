// license:BSD-3-Clause
// copyright-holders:superctr

#ifndef MAME_CPU_H8500_H8570_H
#define MAME_CPU_H8500_H8570_H

#pragma once

#include "h8500.h"
#include "h8500_intc.h"
#include "h8570_pwm.h"
#include "cpu/h8/h8_port.h"
#include "cpu/h8/h8_adc.h"
#include "cpu/h8/h8_sci.h"
#include "cpu/h8/h8_watchdog.h"

class h8570_device : public h8500_device
{
public:
	h8570_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	auto read_port1() { return m_read_port[PORT_1].bind(); }
	auto write_port1() { return m_write_port[PORT_1].bind(); }
	auto read_port5() { return m_read_port[PORT_5].bind(); }
	auto write_port5() { return m_write_port[PORT_5].bind(); }
	auto read_port6() { return m_read_port[PORT_6].bind(); }
	auto write_port6() { return m_write_port[PORT_6].bind(); }
	auto read_port7() { return m_read_port[PORT_7].bind(); }    // port 7 is input only
	auto read_port8() { return m_read_port[PORT_8].bind(); }
	auto write_port8() { return m_write_port[PORT_8].bind(); }
	auto read_port9() { return m_read_port[PORT_9].bind(); }
	auto write_port9() { return m_write_port[PORT_9].bind(); }
	auto read_port10() { return m_read_port[PORT_10].bind(); }
	auto write_port10() { return m_write_port[PORT_10].bind(); }
	auto read_port11() { return m_read_port[PORT_11].bind(); }
	auto write_port11() { return m_write_port[PORT_11].bind(); }
	auto read_port12() { return m_read_port[PORT_12].bind(); }
	auto write_port12() { return m_write_port[PORT_12].bind(); }

	auto write_pwm() { return m_pwm_out_cb.bind(); }

	// ISP side: the sub-processor's program is the board's to model
	auto isf_ack_cb() { return m_isf_ack_cb.bind(); }
	auto isp_reset_cb() { return m_isp_reset_cb.bind(); }
	auto isp_bus_enable_cb() { return m_isp_ibe_cb.bind(); }
	auto isp_dr_write_cb() { return m_dr_write_cb.bind(); }
	void isp_raise(int n);
	void isp_set_icf(int n, int state);
	void isp_set_iof(u32 iof);
	u8 isp_ipr() const { return m_ipr; }
	u16 isp_icf() const { return m_icf; }
	bool isp_reset_held() const { return BIT(m_icsr, 5); }
	u8 dr_r(offs_t offset) { return m_dr[offset]; }
	void dr_w(offs_t offset, u8 data);

protected:
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	virtual bool h8_maximum_mode() const noexcept override { return mode_control() == 3 || mode_control() == 5 || mode_control() == 6; }

	virtual void execute_set_input(int inputnum, int state) override;

	void internal_update(u64 current_time) override;
	using h8500_device::internal_update;
	void notify_standby(int state) override;
	virtual void interrupt_taken() override;
	virtual void update_irq_filter() override;
	virtual void irq_setup() override;

private:
	void internal_map(address_map &map) ATTR_COLD;
	void register_field_map(address_map &map, offs_t base) ATTR_COLD;

	u8 isp_r(offs_t offset);
	void isp_w(offs_t offset, u8 data);
	u8 isp_ctl_r(offs_t offset);
	void isp_ctl_w(offs_t offset, u8 data);
	u8 edge_r(offs_t offset);
	void edge_w(offs_t offset, u8 data);
	u8 syscr_r(offs_t offset);
	void syscr_w(offs_t offset, u8 data);
	u8 sysctl_r(offs_t offset);
	void sysctl_w(offs_t offset, u8 data);
	void isf_update(u16 before);
	void pwm_out_w(u8 data);
	void pwm_icf_w(u8 data);

	required_device<h8570_intc_device> m_intc;
	required_device<h8_adc_device> m_adc;
	required_device<h8_port_device> m_port1;
	required_device<h8_port_device> m_port5;
	required_device<h8_port_device> m_port6;
	required_device<h8_port_device> m_port7;
	required_device<h8_port_device> m_port8;
	required_device<h8_port_device> m_port9;
	required_device<h8_port_device> m_port10;
	required_device<h8_port_device> m_port11;
	required_device<h8_port_device> m_port12;
	required_device<h8570_pwm_device> m_pwm;
	required_device<h8_watchdog_device> m_watchdog;

	devcb_write8 m_pwm_out_cb;
	devcb_write8 m_isf_ack_cb;
	devcb_write_line m_isp_reset_cb;
	devcb_write_line m_isp_ibe_cb;
	devcb_write8 m_dr_write_cb;

	u16 m_isf, m_icf, m_ief, m_ioie, m_cle;
	u32 m_iof;
	u8 m_egf, m_ever, m_ipr, m_icsr, m_fedge, m_redge;
	u8 m_dr[64];
	u8 m_syscr[3];
	u8 m_wsc, m_ramcr, m_sbycr;
};

DECLARE_DEVICE_TYPE(H8570, h8570_device)

#endif // MAME_CPU_H8500_H8570_H
