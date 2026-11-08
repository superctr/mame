// license:BSD-3-Clause
// copyright-holders:superctr
/**********************************************************************

    Roland RC-100 remote controller

**********************************************************************/

#include "emu.h"
#include "rc100.h"

#include "cpu/mcs51/i80c51.h"

namespace {

class roland_rc100_device : public device_t, public device_msx_general_purpose_port_interface
{
public:
	roland_rc100_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	virtual u8 read() override;
	virtual void pin_6_w(int state) override;
	virtual void pin_7_w(int state) override;
	virtual void pin_8_w(int state) override;

	DECLARE_INPUT_CHANGED_MEMBER(reset_changed);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

private:
	required_device<i80c31_device> m_mcu;
	required_device<msx_general_purpose_port_device> m_ext;
	required_ioport_array<4> m_cols;
	required_ioport m_sw;
	required_ioport m_dial;
	output_finder<16> m_leds;

	u8 m_lines;
	u8 m_p1;
	u8 m_p3;
	u8 m_led[4];
	u8 m_ga_mode;
	u8 m_col;
	u8 m_dial_last;

	void program_map(address_map &map) ATTR_COLD;
	void xdata_map(address_map &map) ATTR_COLD;
	u8 ga_r(offs_t offset);
	void ga_w(offs_t offset, u8 data);
	u8 p1_r();
	void p1_w(u8 data);
	u8 p3_r();
	void p3_w(u8 data);
	void lines_sync(s32 param);
	void update_leds();
};

enum : u8
{
	LINE_CLK1  = 0x01,
	LINE_CLK2  = 0x02,
	LINE_DATA2 = 0x04
};

INPUT_PORTS_START(roland_rc100)
	PORT_START("COL0")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 Edit")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 Disk")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 MIDI")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 Func")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 Utility")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 F1")
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 F2")
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 F3")

	PORT_START("COL1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 0") PORT_CODE(KEYCODE_0_PAD)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 Enter") PORT_CODE(KEYCODE_ENTER_PAD)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 Down")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 Inc/Yes") PORT_CODE(KEYCODE_PLUS_PAD)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 Dec/No") PORT_CODE(KEYCODE_MINUS_PAD)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 1") PORT_CODE(KEYCODE_1_PAD)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 2") PORT_CODE(KEYCODE_2_PAD)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 3") PORT_CODE(KEYCODE_3_PAD)

	PORT_START("COL2")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 Right")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 Left")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 Up")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 Ins")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 Del")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 4") PORT_CODE(KEYCODE_4_PAD)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 5") PORT_CODE(KEYCODE_5_PAD)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 6") PORT_CODE(KEYCODE_6_PAD)

	PORT_START("COL3")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 Play")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 Command")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 Menu")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 Sub Menu")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 Execute")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 7") PORT_CODE(KEYCODE_7_PAD)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 8") PORT_CODE(KEYCODE_8_PAD)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RC-100 9") PORT_CODE(KEYCODE_9_PAD)

	PORT_START("SW")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("RC-100 Rec")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("RC-100 Start/Stop")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("RC-100 Reset") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_rc100_device::reset_changed), 0)

	PORT_START("DIAL")
	PORT_BIT(0xff, 0x00, IPT_DIAL) PORT_NAME("RC-100 Alpha Dial") PORT_SENSITIVITY(25) PORT_KEYDELTA(1)
INPUT_PORTS_END


ioport_constructor roland_rc100_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(roland_rc100);
}

ROM_START(roland_rc100)
	ROM_REGION(0x4000, "mcu", 0)
	ROM_LOAD("rc100_v030.bin", 0x0000, 0x4000, CRC(c4f8b47e) SHA1(5f8c32ca1498ad40d8585e8432001067e0309943))
ROM_END


roland_rc100_device::roland_rc100_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, ROLAND_RC100, tag, owner, clock)
	, device_msx_general_purpose_port_interface(mconfig, *this)
	, m_mcu(*this, "mcu")
	, m_ext(*this, "ext")
	, m_cols(*this, "COL%u", 0U)
	, m_sw(*this, "SW")
	, m_dial(*this, "DIAL")
	, m_leds(*this, "rc100_led%u", 0U)
	, m_lines(LINE_CLK1 | LINE_CLK2 | LINE_DATA2)
	, m_p1(0xff)
	, m_p3(0xff)
	, m_ga_mode(0)
	, m_col(0)
	, m_dial_last(0)
{
}

void roland_rc100_device::device_add_mconfig(machine_config &config)
{
	I80C31(config, m_mcu, 6_MHz_XTAL);
	m_mcu->set_addrmap(AS_PROGRAM, &roland_rc100_device::program_map);
	m_mcu->set_addrmap(AS_DATA, &roland_rc100_device::xdata_map);
	m_mcu->port_in_cb<1>().set(FUNC(roland_rc100_device::p1_r));
	m_mcu->port_out_cb<1>().set(FUNC(roland_rc100_device::p1_w));
	m_mcu->port_in_cb<3>().set(FUNC(roland_rc100_device::p3_r));
	m_mcu->port_out_cb<3>().set(FUNC(roland_rc100_device::p3_w));

	MSX_GENERAL_PURPOSE_PORT(config, m_ext, msx_general_purpose_port_devices, "mouse");
}

const tiny_rom_entry *roland_rc100_device::device_rom_region() const
{
	return ROM_NAME(roland_rc100);
}

void roland_rc100_device::program_map(address_map &map)
{
	map(0x0000, 0x3fff).mirror(0xc000).rom().region("mcu", 0);
}

void roland_rc100_device::xdata_map(address_map &map)
{
	map(0x0000, 0xffff).rw(FUNC(roland_rc100_device::ga_r), FUNC(roland_rc100_device::ga_w));
}

void roland_rc100_device::device_start()
{

	save_item(NAME(m_lines));
	save_item(NAME(m_p1));
	save_item(NAME(m_p3));
	save_item(NAME(m_led));
	save_item(NAME(m_ga_mode));
	save_item(NAME(m_col));
	save_item(NAME(m_dial_last));
}

void roland_rc100_device::device_reset()
{
	std::fill(std::begin(m_led), std::end(m_led), 0);
	m_col = 0;
	m_dial_last = m_dial->read();
	update_leds();
	lines_sync(m_lines);
}

INPUT_CHANGED_MEMBER(roland_rc100_device::reset_changed)
{
	m_mcu->set_input_line(INPUT_LINE_RESET, newval ? ASSERT_LINE : CLEAR_LINE);
}

u8 roland_rc100_device::ga_r(offs_t offset)
{
	u8 const reg = BIT(offset, 15) | (BIT(offset, 14) << 1) | (BIT(offset, 13) << 2);
	switch(reg) {
	case 4: {
		u8 const now = m_dial->read();
		u8 const delta = m_dial_last - now;
		if(!machine().side_effects_disabled())
			m_dial_last = now;
		return delta;
	}
	case 5: {
		u8 const data = m_cols[m_col & 3]->read();
		if(!machine().side_effects_disabled())
			m_col = (m_col + 1) & 3;
		return data;
	}
	default:
		return 0xff;
	}
}

void roland_rc100_device::ga_w(offs_t offset, u8 data)
{
	u8 const reg = BIT(offset, 15) | (BIT(offset, 14) << 1) | (BIT(offset, 13) << 2);
	switch(reg) {
	case 0: case 1: case 2: case 3:
		m_led[reg] = data;
		update_leds();
		break;
	case 4:
		m_ga_mode = data;
		break;
	case 5:
		m_col = 0;
		break;
	default:
		break;
	}
}

void roland_rc100_device::update_leds()
{
	u16 const on = m_led[0] | (m_led[2] << 8);
	for(int i = 0; i < 16; i++)
		m_leds[i] = BIT(on, i);
}

u8 roland_rc100_device::p1_r()
{
	return m_ext->read() | 0x80;
}

void roland_rc100_device::p1_w(u8 data)
{
	m_p1 = data;
	m_ext->pin_6_w(BIT(data, 4));
	m_ext->pin_7_w(BIT(data, 5));
	m_ext->pin_8_w(BIT(data, 6));
}

u8 roland_rc100_device::p3_r()
{
	return (m_sw->read() & 0x30) | (m_lines & LINE_DATA2 ? 0xce : 0xcf);
}

void roland_rc100_device::p3_w(u8 data)
{
	m_p3 = data;
}

u8 roland_rc100_device::read()
{
	return 0xc0 | (m_lines & LINE_CLK1 ? 0x10 : 0) | (m_lines & LINE_CLK2 ? 0x20 : 0)
		| (BIT(m_p1, 7) << 1) | (BIT(m_p3, 1) ? 0 : 1);
}

void roland_rc100_device::lines_sync(s32 param)
{
	m_lines = param;
	m_mcu->set_input_line(MCS51_INT1_LINE, param & LINE_CLK1 ? ASSERT_LINE : CLEAR_LINE);
	m_mcu->set_input_line(MCS51_INT0_LINE, param & LINE_CLK2 ? ASSERT_LINE : CLEAR_LINE);
}

void roland_rc100_device::pin_6_w(int state)
{
	u8 const lines = state ? (m_lines | LINE_CLK1) : (m_lines & ~LINE_CLK1);
	if(lines != m_lines) {
		m_lines = lines;
		machine().scheduler().synchronize(timer_expired_delegate(FUNC(roland_rc100_device::lines_sync), this), lines);
		machine().scheduler().perfect_quantum(attotime::from_usec(200));
	}
}

void roland_rc100_device::pin_7_w(int state)
{
	u8 const lines = state ? (m_lines | LINE_CLK2) : (m_lines & ~LINE_CLK2);
	if(lines != m_lines) {
		m_lines = lines;
		machine().scheduler().synchronize(timer_expired_delegate(FUNC(roland_rc100_device::lines_sync), this), lines);
		machine().scheduler().perfect_quantum(attotime::from_usec(200));
	}
}

void roland_rc100_device::pin_8_w(int state)
{
	u8 const lines = state ? (m_lines | LINE_DATA2) : (m_lines & ~LINE_DATA2);
	if(lines != m_lines) {
		m_lines = lines;
		machine().scheduler().synchronize(timer_expired_delegate(FUNC(roland_rc100_device::lines_sync), this), lines);
	}
}

} // anonymous namespace

DEFINE_DEVICE_TYPE_PRIVATE(ROLAND_RC100, device_msx_general_purpose_port_interface, roland_rc100_device, "roland_rc100", "Roland RC-100 Remote Controller")
