// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Roland AIRA TB-3 Touch Bassline

    An ESC2 (MB8AA4181) booting from a 2 MiB serial flash, with SDRAM on
    its external bus.  The ESC2 reads the panel itself: five knobs and the
    touch pad on its ADC, ten keys and the VALUE encoder on port 6.

***************************************************************************/

#include "emu.h"

#include "roland_esc2.h"

#include "machine/generic_spi_flash.h"
#include "machine/timer.h"
#include "bus/midi/midi.h"

#include "speaker.h"


namespace {

class tb3_state : public driver_device
{
public:
	tb3_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_spiflash(*this, "spiflash")
		, m_knob(*this, "KNOB%u", 0U)
		, m_keys(*this, "KEYS")
		, m_value(*this, "VALUE")
		, m_pad(*this, "PAD%u", 0U)
	{
	}

	void tb3(machine_config &config) ATTR_COLD;

private:
	required_device<mb8aa4181_device> m_maincpu;
	required_device<generic_spi_flash_device> m_spiflash;
	required_ioport_array<5> m_knob;
	required_ioport m_keys;
	required_ioport m_value;
	required_ioport_array<3> m_pad;

	u32 m_port0 = 0;
	u32 m_port6 = 0;
	u8 m_value_pos = 0;
	u8 m_value_phase = 0;
	s32 m_value_pending = 0;

	void mem_map(address_map &map) ATTR_COLD;

	template <unsigned N> u16 knob_r() { return (m_knob[N]->read() << 4) | 8; }
	u32 port6_r();
	u16 pad_r(unsigned channel);
	TIMER_DEVICE_CALLBACK_MEMBER(value_tick);

	virtual void machine_start() override ATTR_COLD;
};

void tb3_state::mem_map(address_map &map)
{
	map(0x60000000, 0x61ffffff).ram();
}

void tb3_state::machine_start()
{
	m_spiflash->set_rom_ptr(memregion("flash")->base());
	m_spiflash->set_rom_size(memregion("flash")->bytes());
	save_item(NAME(m_port0));
	save_item(NAME(m_port6));
	save_item(NAME(m_value_pos));
	save_item(NAME(m_value_phase));
	save_item(NAME(m_value_pending));
}

u32 tb3_state::port6_r()
{
	static constexpr u8 ROWS[3] = { 10, 12, 13 };
	const u32 keys = m_keys->read();
	u32 data = (BIT(keys, 0) << 15) | (BIT(0x3, m_value_phase) << 25) | (BIT(0x9, m_value_phase) << 24);
	u32 columns = 7;
	for (unsigned row = 0; row < 3; row++)
		if (!BIT(m_port6, ROWS[row]))
			columns &= ~BIT(keys, 1 + row * 3, 3);
	return data | (columns << 16);
}

u16 tb3_state::pad_r(unsigned channel)
{
	if (!BIT(m_pad[2]->read(), 0))
		return 0;
	const u8 lines = (BIT(m_port0, 7) << 3) | (BIT(m_port6, 7) << 2) | (BIT(m_port0, 8) << 1) | BIT(m_port6, 6);
	u16 level;
	switch (lines)
	{
	case 0b0010: level = channel == 6 ? 0x3ff : 0; break;
	case 0b0110: level = channel == 6 ? 0x3ca - (m_pad[0]->read() * (0x3ca - 0x2d) + 127) / 255 : 0; break;
	case 0b1001: level = channel == 5 ? 0x370 - (m_pad[1]->read() * (0x370 - 0x6e) + 127) / 255 : 0; break;
	case 0b1011: level = channel == 6 ? 0x200 : 0; break;
	default: level = 0; break;
	}
	return level << 2;
}

TIMER_DEVICE_CALLBACK_MEMBER(tb3_state::value_tick)
{
	const u8 pos = m_value->read();
	m_value_pending += s8(pos - m_value_pos) * 4;
	m_value_pos = pos;
	if (m_value_pending > 0)
	{
		m_value_phase = (m_value_phase + 1) & 3;
		m_value_pending--;
	}
	else if (m_value_pending < 0)
	{
		m_value_phase = (m_value_phase - 1) & 3;
		m_value_pending++;
	}
}

static INPUT_PORTS_START(tb3)
	PORT_START("KNOB0")
	PORT_ADJUSTER(128, "Effect") PORT_MINMAX(0, 255)
	PORT_START("KNOB1")
	PORT_ADJUSTER(128, "Cutoff") PORT_MINMAX(0, 255)
	PORT_START("KNOB2")
	PORT_ADJUSTER(128, "Resonance") PORT_MINMAX(0, 255)
	PORT_START("KNOB3")
	PORT_ADJUSTER(128, "Accent") PORT_MINMAX(0, 255)
	PORT_START("KNOB4")
	PORT_ADJUSTER(128, "Env Mod") PORT_MINMAX(0, 255)
	PORT_START("KEYS")
	PORT_BIT(0x001, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Key 0")
	PORT_BIT(0x002, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Key 1")
	PORT_BIT(0x004, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Key 2")
	PORT_BIT(0x008, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Key 3")
	PORT_BIT(0x010, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Key 4")
	PORT_BIT(0x020, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Key 5")
	PORT_BIT(0x040, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Key 6")
	PORT_BIT(0x080, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Play/Stop")
	PORT_BIT(0x100, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Key 8")
	PORT_BIT(0x200, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Key 9")
	PORT_START("PAD0")
	PORT_BIT(0xff, 0x80, IPT_AD_STICK_X) PORT_NAME("Pad X") PORT_SENSITIVITY(50) PORT_KEYDELTA(8)
	PORT_START("PAD1")
	PORT_BIT(0xff, 0x80, IPT_AD_STICK_Y) PORT_NAME("Pad Y") PORT_SENSITIVITY(50) PORT_KEYDELTA(8)
	PORT_START("PAD2")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_BUTTON1) PORT_NAME("Pad Touch")
	PORT_START("VALUE")
	PORT_BIT(0xff, 0x00, IPT_DIAL) PORT_NAME("Value") PORT_SENSITIVITY(25) PORT_KEYDELTA(1)
INPUT_PORTS_END

void tb3_state::tb3(machine_config &config)
{
	MB8AA4181(config, m_maincpu, 156'000'000);
	m_maincpu->set_addrmap(AS_PROGRAM, &tb3_state::mem_map);
	m_maincpu->set_flash_tag("flash");
	m_maincpu->sfi_cs_cb().set(m_spiflash, FUNC(generic_spi_flash_device::cs_w));
	m_maincpu->sfi_tx_cb().set(m_spiflash, FUNC(generic_spi_flash_device::write));
	m_maincpu->sfi_rx_cb().set(m_spiflash, FUNC(generic_spi_flash_device::read));
	m_maincpu->gpio_out_cb<0>().set([this] (u32 data) { m_port0 = data; });
	m_maincpu->gpio_out_cb<6>().set([this] (u32 data) { m_port6 = data; });
	m_maincpu->gpio_in_cb<6>().set(FUNC(tb3_state::port6_r));
	m_maincpu->adc_in_cb<0>().set(FUNC(tb3_state::knob_r<0>));
	m_maincpu->adc_in_cb<1>().set(FUNC(tb3_state::knob_r<1>));
	m_maincpu->adc_in_cb<2>().set(FUNC(tb3_state::knob_r<2>));
	m_maincpu->adc_in_cb<3>().set(FUNC(tb3_state::knob_r<3>));
	m_maincpu->adc_in_cb<4>().set(FUNC(tb3_state::knob_r<4>));
	m_maincpu->adc_in_cb<5>().set([this] () { return pad_r(5); });
	m_maincpu->adc_in_cb<6>().set([this] () { return pad_r(6); });

	SPEAKER(config, "speaker", 2).front();
	mb8aa4181_dsp_device &dsp = *m_maincpu->subdevice<mb8aa4181_dsp_device>("dsp");
	dsp.set_clock(24.576_MHz_XTAL);
	dsp.add_route(0, "speaker", 1.0, 0);
	dsp.add_route(1, "speaker", 1.0, 1);

	GENERIC_SPI_FLASH(config, m_spiflash);

	TIMER(config, "value").configure_periodic(FUNC(tb3_state::value_tick), attotime::from_msec(2));

	MIDI_PORT(config, "mdin", midiin_slot, "midiin").rxd_handler().set(m_maincpu, FUNC(mb8aa4181_device::rxd_w<3>));
	m_maincpu->txd_cb<3>().set("mdout", FUNC(midi_port_device::write_txd));
	MIDI_PORT(config, "mdout", midiout_slot, "midiout");
}

ROM_START(tb3)
	ROM_REGION32_LE(0x200000, "flash", 0)
	// the 1.10 update image
	ROM_LOAD("tb3_110.bin", 0, 0x200000, BAD_DUMP CRC(373658bb) SHA1(14a17405c59bec6af2e4e05363a33f64c8eab997))
ROM_END

} // anonymous namespace


SYST(2014, tb3, 0, 0, tb3, tb3, tb3_state, empty_init, "Roland", "TB-3 Touch Bassline", MACHINE_NOT_WORKING)
