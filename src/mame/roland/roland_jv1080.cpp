// license:BSD-3-Clause
// copyright-holders:superctr
/****************************************************************************

    Roland JV-1080 64 Voice Synthesizer Module.

    SH7034 (IC15) with its own 64 KB mask ROM, 1 MB program ROM, 128 KB of
    work DRAM and 64 KB of battery-backed SRAM.  An XP (IC22) with two DRAMs
    and four 2 MB wave ROMs drives three stereo DACs: MIX, OUTPUT1, OUTPUT2.
    The M60205 gate array (IC1) carries the panel and a 40x2 character LCD,
    which the CPU fills by DMA.

****************************************************************************/

#include "emu.h"

#include "cpu/sh/sh7034.h"
#include "m60205.h"
#include "machine/nvram.h"
#include "sound/roland_xp.h"
#include "video/hd44780.h"
#include "wavecard.h"

#include "bus/midi/midiinport.h"
#include "bus/midi/midioutport.h"
#include "emupal.h"
#include "screen.h"
#include "softlist_dev.h"
#include "speaker.h"

#include "roland_jv1080.lh"


namespace {

class roland_jv1080_state : public driver_device
{
public:
	roland_jv1080_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_xp(*this, "xp")
		, m_ga(*this, "ga")
		, m_lcd(*this, "lcd")
		, m_exp(*this, "exp%u", 1U)
		, m_leds(*this, "led%u", 0U)
	{
	}

	void jv1080(machine_config &config);
	void init_jv1080() ATTR_COLD;

private:
	void jv1080_map(address_map &map) ATTR_COLD;
	void xp_rom_map(address_map &map) ATTR_COLD;

	void led_w(offs_t offset, u8 data);
	u16 sense_r();

	void jv_palette(palette_device &palette) const ATTR_COLD;
	HD44780_PIXEL_UPDATE(lcd_pixel_update);

	required_device<sh7034_device> m_maincpu;
	required_device<roland_xp_device> m_xp;
	required_device<m60205_device> m_ga;
	required_device<hd44780_device> m_lcd;
	required_device_array<roland_srjv80_slot_device, 4> m_exp;
	output_finder<24> m_leds;
};


// SENS0-SENS4 on PB1-PB5, high while a board is fitted; 0a01a99e polls them
// as one five-bit field thirty times a second.  SENS0 is the PCM card's, and
// no card slot is emulated, so it reads empty
u16 roland_jv1080_state::sense_r()
{
	u16 data = 0xffff & ~(1 << 1);
	for (int slot = 0; slot < 4; slot++)
		if (!m_exp[slot]->sense_r())
			data &= ~(1 << (2 + slot));
	return data;
}


void roland_jv1080_state::led_w(offs_t offset, u8 data)
{
	if (offset < 3)
		for (int bit = 0; bit < 8; bit++)
			m_leds[offset * 8 + bit] = BIT(data, bit);
}


void roland_jv1080_state::jv1080_map(address_map &map)
{
	map(0x01000000, 0x0101ffff).ram();
	map(0x02000000, 0x020fffff).rom().region("progrom", 0);
	map(0x02380000, 0x0238ffff).ram().share("nvram");
	map(0x04000000, 0x04003fff).rw(m_xp, FUNC(roland_xp_device::read), FUNC(roland_xp_device::write));
	map(0x04380000, 0x0438003f).rw(m_ga, FUNC(m60205_device::read), FUNC(m60205_device::write));
}

void roland_jv1080_state::xp_rom_map(address_map &map)
{
	map(0x0000000, 0x07fffff).rom().region("waverom", 0);
	for (int slot = 0; slot < 4; slot++)
		map(0x2000000 + slot * 0x1000000, 0x27fffff + slot * 0x1000000).r(m_exp[slot], FUNC(roland_srjv80_slot_device::read));
}


// The wave ROMs are stored as dumped; the board wiring permutes address lines
// within each 256 KB block and the data lines, as on the SC-88.
void roland_jv1080_state::init_jv1080()
{
	memory_region *region = memregion("waverom");
	u8 *rom = region->base();
	const u32 size = region->bytes();

	static const u8 address_lines[18] = { 0, 4, 2, 3, 1, 13, 7, 12, 5, 10, 16, 9, 6, 8, 14, 17, 11, 15 };
	static const u8 data_lines[8] = { 2, 0, 4, 5, 7, 6, 3, 1 };

	std::vector<u8> scrambled(rom, rom + size);
	for (u32 i = 0; i < size; i++)
	{
		u32 address = i & ~0x3ffff;
		for (int bit = 0; bit < 18; bit++)
			if (BIT(i, bit))
				address |= 1 << address_lines[bit];

		const u8 source = scrambled[address];
		u8 data = 0;
		for (int bit = 0; bit < 8; bit++)
			if (BIT(source, data_lines[bit]))
				data |= 1 << bit;
		rom[i] = data;
	}
}


void roland_jv1080_state::jv_palette(palette_device &palette) const
{
	palette.set_pen_color(0, rgb_t(88, 247, 0));
	palette.set_pen_color(1, rgb_t(3, 3, 60));
}

HD44780_PIXEL_UPDATE(roland_jv1080_state::lcd_pixel_update)
{
	if (x < 5 && y < 8 && line < 2 && pos < 40)
		bitmap.pix(line * 8 + y, pos * 6 + x) = state;
}


static INPUT_PORTS_START(jv1080)
	PORT_START("SW0")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_BUTTON1)  PORT_NAME("1-8/9-16")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_BUTTON2)  PORT_NAME("F1")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_BUTTON3)  PORT_NAME("F4")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_BUTTON4)  PORT_NAME("F7")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_BUTTON5)  PORT_NAME("Utility")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_BUTTON6)  PORT_NAME("Sound B")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_BUTTON7)  PORT_NAME("Performance")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_BUTTON8)  PORT_NAME("User/Card")

	PORT_START("SW1")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_BUTTON9)  PORT_NAME("SW/Select")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_BUTTON10) PORT_NAME("F2")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_BUTTON11) PORT_NAME("F5")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_BUTTON12) PORT_NAME("F8")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_BUTTON13) PORT_NAME("EFX On/Off")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_BUTTON14) PORT_NAME("Sound C")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_BUTTON15) PORT_NAME("Patch")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_BUTTON16) PORT_NAME("Preset")

	PORT_START("SW2")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER)    PORT_NAME("Palette")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_OTHER)    PORT_NAME("F3")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_OTHER)    PORT_NAME("F6")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_OTHER)    PORT_NAME("System")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_OTHER)    PORT_NAME("Sound A")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_OTHER)    PORT_NAME("Sound D")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_OTHER)    PORT_NAME("Rhythm")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_OTHER)    PORT_NAME("Exp")

	PORT_START("SW3")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_JOYSTICK_DOWN)  PORT_NAME("Cursor Down")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_JOYSTICK_LEFT)  PORT_NAME("Cursor Left")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_JOYSTICK_UP)    PORT_NAME("Cursor Up")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_JOYSTICK_RIGHT) PORT_NAME("Cursor Right")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_OTHER)          PORT_NAME("Shift")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_OTHER)          PORT_NAME("Exit")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_OTHER)          PORT_NAME("Enter")

	PORT_START("PORT")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Inc")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Dec")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Preview")
	PORT_BIT(0xf8, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("DIAL")
	PORT_BIT(0xff, 0x00, IPT_DIAL) PORT_NAME("Value") PORT_SENSITIVITY(25) PORT_KEYDELTA(2)
INPUT_PORTS_END


void roland_jv1080_state::jv1080(machine_config &config)
{
	SH7034(config, m_maincpu, 20_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &roland_jv1080_state::jv1080_map);
	m_maincpu->write_sci_tx<0>().set("mdout", FUNC(midi_port_device::write_txd));
	// AN2-AN7 are grounded.  AN0 and AN1 are the data card's cell and the
	// internal one, each through a buffer; 0x1ff-0x2cc of full scale is healthy
	m_maincpu->read_adc<0>().set_constant(0);
	m_maincpu->read_adc<1>().set_constant(620);
	m_maincpu->read_adc<2>().set_constant(0);
	m_maincpu->read_adc<3>().set_constant(0);
	m_maincpu->read_adc<4>().set_constant(0);
	m_maincpu->read_adc<5>().set_constant(0);
	m_maincpu->read_adc<6>().set_constant(0);
	m_maincpu->read_adc<7>().set_constant(0);
	m_maincpu->read_porta().set_constant(0xffff);
	m_maincpu->write_porta().set_nop();
	m_maincpu->read_portb().set(FUNC(roland_jv1080_state::sense_r));
	m_maincpu->write_portb().set_nop();
	m_maincpu->read_portc().set_constant(0xffff);

	M60205(config, m_ga, 20_MHz_XTAL);
	m_ga->int_callback().set_inputline(m_maincpu, 5); // IRQ5
	m_ga->write_lcd_control().set(m_lcd, FUNC(hd44780_device::control_w));
	m_ga->write_lcd_data().set(m_lcd, FUNC(hd44780_device::data_w));
	m_ga->write_led().set(FUNC(roland_jv1080_state::led_w));
	m_ga->read_scan<0>().set_ioport("SW0");
	m_ga->read_scan<1>().set_ioport("SW1");
	m_ga->read_scan<2>().set_ioport("SW2");
	m_ga->read_scan<3>().set_ioport("SW3");
	m_ga->read_port().set_ioport("PORT");
	m_ga->read_encoder().set_ioport("DIAL");

	// EXP-A to EXP-D, CN501-CN504, one 8 MB board at the foot of each of the
	// XP's chip selects 2 to 5; select 1 is the PCM card's
	for (int slot = 0; slot < 4; slot++)
		ROLAND_SRJV80_SLOT(config, m_exp[slot], 0).set_image_names(util::string_format("xp-%c", 'a' + slot), util::string_format("xp-%c", 'a' + slot));
	SOFTWARE_LIST(config, "exp_list").set_original("roland_srjv80");

	NVRAM(config, "nvram", nvram_device::DEFAULT_ALL_0);

	midi_port_device &mdin(MIDI_PORT(config, "mdin", midiin_slot, "midiin"));
	mdin.rxd_handler().set(m_maincpu, FUNC(sh7034_device::sci_rx_w<0>));
	MIDI_PORT(config, "mdout", midiout_slot, "midiout");

	screen_device &screen(SCREEN(config, "screen"));
	screen.set_lcd();
	screen.set_refresh_hz(80);
	screen.set_screen_update(m_lcd, FUNC(hd44780_device::screen_update));
	screen.set_size(6 * 40, 8 * 2);
	screen.set_visarea_full();
	screen.set_palette("palette");
	config.set_default_layout(layout_roland_jv1080);

	PALETTE(config, "palette", FUNC(roland_jv1080_state::jv_palette), 2);

	HD44780(config, m_lcd, 270'000);
	m_lcd->set_lcd_size(2, 40);
	m_lcd->set_pixel_update_cb(FUNC(roland_jv1080_state::lcd_pixel_update));

	SPEAKER(config, "mix", 2).front();
	SPEAKER(config, "output1", 2).front();
	SPEAKER(config, "output2", 2).front();

	ROLAND_XP(config, m_xp, 24.576_MHz_XTAL);
	m_xp->set_addrmap(roland_xp_device::AS_WAVE, &roland_jv1080_state::xp_rom_map);
	m_xp->int_callback().set_inputline(m_maincpu, 7); // IRQ7
	m_xp->add_route(0, "mix", 1.0, 0);
	m_xp->add_route(1, "mix", 1.0, 1);
	m_xp->add_route(2, "output1", 1.0, 0);
	m_xp->add_route(3, "output1", 1.0, 1);
	m_xp->add_route(4, "output2", 1.0, 0);
	m_xp->add_route(5, "output2", 1.0, 1);
}


ROM_START(jv1080)
	ROM_REGION32_BE(0x10000, "maincpu", 0)
	ROM_LOAD("roland_r00677323_6437034c12f.ic15", 0x00000, 0x10000, CRC(609c8b8d) SHA1(075b4d60d14afa3ff3c825b8da5c8687aca84560))

	ROM_REGION32_BE(0x100000, "progrom", 0)
	ROM_LOAD("jv1080_prom.bin", 0x000000, 0x100000, CRC(0e26e075) SHA1(77f09efecf267def69b4e8f851cf124798347c03))

	ROM_REGION(0x800000, "waverom", 0)
	ROM_LOAD("jv1080_waverom1.bin", 0x000000, 0x200000, CRC(f965d95f) SHA1(6b9a177f3ac6560e27befbcb9e907cf369f0f28a))
	ROM_LOAD("jv1080_waverom2.bin", 0x200000, 0x200000, CRC(78c3b1d8) SHA1(0dffa4ab4b9ea86c338fe146d902810f66c36cfd))
	ROM_LOAD("jv1080_waverom3.bin", 0x400000, 0x200000, CRC(edb5f1aa) SHA1(21eff48c6efe434ffce89c3bc61fe2b6d1584d00))
	ROM_LOAD("jv1080_waverom4.bin", 0x600000, 0x200000, CRC(4a877d28) SHA1(a1732be64e9c86c559df9c715703bcabc2a83440))
ROM_END

} // anonymous namespace


SYST(1994, jv1080, 0, 0, jv1080, jv1080, roland_jv1080_state, init_jv1080, "Roland", "JV-1080", MACHINE_NOT_WORKING) // no expansion board or card slots
