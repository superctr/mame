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
#include "machine/nvram.h"
#include "sound/roland_xp.h"
#include "video/hd44780.h"

#include "bus/midi/midiinport.h"
#include "bus/midi/midioutport.h"
#include "emupal.h"
#include "screen.h"
#include "speaker.h"


namespace {

class roland_jv1080_state : public driver_device
{
public:
	roland_jv1080_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_xp(*this, "xp")
		, m_lcd(*this, "lcd")
	{
	}

	void jv1080(machine_config &config);
	void init_jv1080() ATTR_COLD;

private:
	void jv1080_map(address_map &map) ATTR_COLD;
	void xp_rom_map(address_map &map) ATTR_COLD;

	u8 ga_r(offs_t offset);
	void ga_w(offs_t offset, u8 data);

	void jv_palette(palette_device &palette) const ATTR_COLD;
	HD44780_PIXEL_UPDATE(lcd_pixel_update);

	required_device<sh7034_device> m_maincpu;
	required_device<roland_xp_device> m_xp;
	required_device<hd44780_device> m_lcd;
};


u8 roland_jv1080_state::ga_r(offs_t offset)
{
	if (!machine().side_effects_disabled())
		logerror("%s: gate array read %02x\n", machine().describe_context(), offset);
	return 0;
}

void roland_jv1080_state::ga_w(offs_t offset, u8 data)
{
	switch (offset)
	{
	case 0x38:
		m_lcd->control_w(data);
		break;

	case 0x39:
		m_lcd->data_w(data);
		break;

	default:
		logerror("%s: gate array write %02x = %02x\n", machine().describe_context(), offset, data);
		break;
	}
}


void roland_jv1080_state::jv1080_map(address_map &map)
{
	map(0x01000000, 0x0101ffff).ram();
	map(0x02000000, 0x020fffff).rom().region("progrom", 0);
	map(0x02380000, 0x0238ffff).ram().share("nvram");
	map(0x04000000, 0x04003fff).rw(m_xp, FUNC(roland_xp_device::read), FUNC(roland_xp_device::write));
	map(0x04380000, 0x0438003f).rw(FUNC(roland_jv1080_state::ga_r), FUNC(roland_jv1080_state::ga_w));
}

void roland_jv1080_state::xp_rom_map(address_map &map)
{
	map(0x0000000, 0x07fffff).rom().region("waverom", 0);
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
INPUT_PORTS_END


void roland_jv1080_state::jv1080(machine_config &config)
{
	SH7034(config, m_maincpu, 20_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &roland_jv1080_state::jv1080_map);
	m_maincpu->write_sci_tx<0>().set("mdout", FUNC(midi_port_device::write_txd));
	// What the eight analog inputs and the two ports carry is not established;
	// port A bit 10 changes with the channel group the firmware scans.
	m_maincpu->read_adc<0>().set_constant(0);
	m_maincpu->read_adc<1>().set_constant(0);
	m_maincpu->read_adc<2>().set_constant(0);
	m_maincpu->read_adc<3>().set_constant(0);
	m_maincpu->read_adc<4>().set_constant(0);
	m_maincpu->read_adc<5>().set_constant(0);
	m_maincpu->read_adc<6>().set_constant(0);
	m_maincpu->read_adc<7>().set_constant(0);
	m_maincpu->read_porta().set_constant(0xffff);
	m_maincpu->write_porta().set_nop();
	m_maincpu->read_portb().set_constant(0xffff);
	m_maincpu->write_portb().set_nop();
	m_maincpu->read_portc().set_constant(0xffff);

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


SYST(1994, jv1080, 0, 0, jv1080, jv1080, roland_jv1080_state, init_jv1080, "Roland", "JV-1080", MACHINE_NOT_WORKING | MACHINE_NO_SOUND)
