// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Roland Boutique, third generation (D-05, SH-01A, TR-08)

    One firmware serves all three: an ESC2 (MB8AA4181) booting from a
    4 MiB serial flash, SDRAM on its external bus, and an STM32F103-class
    sub-CPU that reads the panel and reports the model from four strap
    pins.  The ESC2 holds the sub-CPU in reset and selects its boot mode.

***************************************************************************/

#include "emu.h"

#include "roland_esc2.h"

#include "cpu/armv7m/stm32f1.h"
#include "machine/generic_spi_flash.h"
#include "bus/midi/midi.h"
#include "video/hd44780.h"

#include "emupal.h"
#include "screen.h"
#include "speaker.h"


namespace {

class boutique_state : public driver_device
{
public:
	boutique_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_subcpu(*this, "subcpu")
		, m_lcdc(*this, "lcdc")
		, m_spiflash(*this, "spiflash")
		, m_flash(*this, "flash")
		, m_subflash(*this, "subcpu")
		, m_volume(*this, "VOLUME")
		, m_mux(*this, "MUX%u", 0U)
		, m_knob(*this, "KNOB%u", 0U)
	{
	}

	void d05(machine_config &config) ATTR_COLD;
	void sh01a(machine_config &config) ATTR_COLD;
	void tr08(machine_config &config) ATTR_COLD;

	void init_boutique() ATTR_COLD;

private:
	required_device<mb8aa4181_device> m_maincpu;
	required_device<stm32f103_device> m_subcpu;
	optional_device<hd44780_device> m_lcdc;
	required_device<generic_spi_flash_device> m_spiflash;
	required_region_ptr<u32> m_flash;
	required_region_ptr<u32> m_subflash;
	required_ioport m_volume;
	optional_ioport_array<32> m_mux;
	optional_ioport_array<4> m_knob;

	u32 m_lcd_port = 0;
	u8 m_mux_select = 0;

	void mem_map(address_map &map) ATTR_COLD;
	void boutique(machine_config &config, u16 strap, const XTAL &audio) ATTR_COLD;

	void subcpu_control_w(u32 data);
	void subcpu_porte_w(u16 data);
	template <unsigned Ch> u16 panel_r();
	void lcd_port_w(u32 data);
	void lcd_data_w(u8 data);
	void lcd_palette(palette_device &palette) const ATTR_COLD;
	HD44780_PIXEL_UPDATE(lcd_pixel_update);

	virtual void machine_start() override ATTR_COLD;
};

void boutique_state::mem_map(address_map &map)
{
	map(0x60000000, 0x61ffffff).ram();
}

void boutique_state::init_boutique()
{
	const u32 length = m_flash[0x4003c / 4];
	std::copy_n(&m_flash[0x40040 / 4], std::min<u32>((length + 3) / 4, m_subflash.length()), &m_subflash[0]);
	m_subflash[0x1fff8 / 4] = m_flash[0x40020 / 4];
	m_subflash[0x1fffc / 4] = m_flash[0x40024 / 4];
}

void boutique_state::machine_start()
{
	m_spiflash->set_rom_ptr(memregion("flash")->base());
	m_spiflash->set_rom_size(memregion("flash")->bytes());
	save_item(NAME(m_lcd_port));
	save_item(NAME(m_mux_select));

	m_maincpu->exint_w<0>(1);
	m_maincpu->exint_w<7>(1);
}

void boutique_state::subcpu_control_w(u32 data)
{
	m_subcpu->set_input_line(INPUT_LINE_RESET, BIT(data, 0) ? CLEAR_LINE : ASSERT_LINE);
}

void boutique_state::subcpu_porte_w(u16 data)
{
	m_mux_select = BIT(data, 4, 3);
}

template <unsigned Ch>
u16 boutique_state::panel_r()
{
	ioport_port *port;
	if constexpr (Ch < 10)
		port = m_mux[(Ch - 6) * 8 + m_mux_select].target();
	else
		port = m_knob[Ch - 10].target();
	return port ? ((255 - port->read()) << 4) | 8 : 0xfff;
}

void boutique_state::lcd_port_w(u32 data)
{
	m_lcd_port = data;
}

void boutique_state::lcd_data_w(u8 data)
{
	if (m_lcdc && !BIT(m_lcd_port, 9))
		m_lcdc->write(BIT(m_lcd_port, 8), data);
}

void boutique_state::lcd_palette(palette_device &palette) const
{
	palette.set_pen_color(0, rgb_t(30, 36, 44));
	palette.set_pen_color(1, rgb_t(236, 244, 250));
}

HD44780_PIXEL_UPDATE(boutique_state::lcd_pixel_update)
{
	if (x < 5 && y < 8 && line < 2 && pos < 16)
		bitmap.pix(line * 9 + y, pos * 6 + x) = state;
}

static INPUT_PORTS_START(d05)
	PORT_START("VOLUME")
	PORT_ADJUSTER(80, "Volume")
INPUT_PORTS_END

static INPUT_PORTS_START(sh01a)
	PORT_INCLUDE(d05)

	PORT_START("MUX1")
	PORT_CONFNAME(0xff, 0xd5, "Switch 6.1")
	PORT_CONFSETTING(0xd5, "1")
	PORT_CONFSETTING(0x7f, "2")
	PORT_CONFSETTING(0x2a, "3")
	PORT_START("MUX2")
	PORT_CONFNAME(0xff, 0xd5, "Switch 6.2")
	PORT_CONFSETTING(0xd5, "1")
	PORT_CONFSETTING(0x7f, "2")
	PORT_CONFSETTING(0x2a, "3")
	PORT_START("MUX3")
	PORT_ADJUSTER(255, "Control 6.3") PORT_MINMAX(0, 255)
	PORT_START("MUX4")
	PORT_ADJUSTER(0, "Control 6.4") PORT_MINMAX(0, 255)
	PORT_START("MUX5")
	PORT_ADJUSTER(0, "Control 6.5") PORT_MINMAX(0, 255)
	PORT_START("MUX6")
	PORT_ADJUSTER(0, "Control 6.6") PORT_MINMAX(0, 255)
	PORT_START("MUX7")
	PORT_CONFNAME(0xff, 0xfe, "LFO Waveform")
	PORT_CONFSETTING(0xfe, "1")
	PORT_CONFSETTING(0xf4, "2")
	PORT_CONFSETTING(0xdd, "3")
	PORT_CONFSETTING(0xbd, "4")
	PORT_CONFSETTING(0x9e, "5")
	PORT_CONFSETTING(0x6f, "6")
	PORT_START("MUX10")
	PORT_ADJUSTER(128, "Control 7.2") PORT_MINMAX(0, 255)
	PORT_START("MUX11")
	PORT_ADJUSTER(32, "Envelope Release") PORT_MINMAX(0, 255)
	PORT_START("MUX12")
	PORT_ADJUSTER(255, "Envelope Sustain") PORT_MINMAX(0, 255)
	PORT_START("MUX13")
	PORT_ADJUSTER(128, "Envelope Decay") PORT_MINMAX(0, 255)
	PORT_START("MUX14")
	PORT_ADJUSTER(0, "Envelope Attack") PORT_MINMAX(0, 255)
	PORT_START("MUX15")
	PORT_CONFNAME(0xff, 0xd5, "Envelope Trigger")
	PORT_CONFSETTING(0xd5, "1")
	PORT_CONFSETTING(0x7f, "2")
	PORT_CONFSETTING(0x2a, "3")
	PORT_START("MUX16")
	PORT_CONFNAME(0xff, 0xc0, "VCA Mode")
	PORT_CONFSETTING(0x40, "1")
	PORT_CONFSETTING(0xc0, "2")
	PORT_START("MUX17")
	PORT_ADJUSTER(0, "VCF Keyboard") PORT_MINMAX(0, 255)
	PORT_START("MUX18")
	PORT_ADJUSTER(0, "VCF Modulation") PORT_MINMAX(0, 255)
	PORT_START("MUX19")
	PORT_ADJUSTER(0, "VCF Envelope") PORT_MINMAX(0, 255)
	PORT_START("MUX20")
	PORT_ADJUSTER(0, "VCF Resonance") PORT_MINMAX(0, 255)
	PORT_START("MUX21")
	PORT_ADJUSTER(255, "VCF Cutoff") PORT_MINMAX(0, 255)
	PORT_START("MUX22")
	PORT_ADJUSTER(0, "Mixer Noise") PORT_MINMAX(0, 255)
	PORT_START("MUX23")
	PORT_CONFNAME(0xff, 0xd5, "Sub Oscillator Type")
	PORT_CONFSETTING(0xd5, "1")
	PORT_CONFSETTING(0x7f, "2")
	PORT_CONFSETTING(0x2a, "3")
	PORT_START("MUX24")
	PORT_ADJUSTER(0, "Mixer Sub") PORT_MINMAX(0, 255)
	PORT_START("MUX25")
	PORT_ADJUSTER(255, "Mixer Saw") PORT_MINMAX(0, 255)
	PORT_START("MUX26")
	PORT_ADJUSTER(0, "Mixer Pulse") PORT_MINMAX(0, 255)
	PORT_START("MUX27")
	PORT_CONFNAME(0xff, 0xd5, "VCO PWM Source")
	PORT_CONFSETTING(0xd5, "1")
	PORT_CONFSETTING(0x7f, "2")
	PORT_CONFSETTING(0x2a, "3")
	PORT_START("MUX28")
	PORT_ADJUSTER(128, "VCO Pulse Width") PORT_MINMAX(0, 255)
	PORT_START("MUX29")
	PORT_CONFNAME(0xff, 0xbd, "VCO Range")
	PORT_CONFSETTING(0xfe, "64'")
	PORT_CONFSETTING(0xf4, "32'")
	PORT_CONFSETTING(0xdd, "16'")
	PORT_CONFSETTING(0xbd, "8'")
	PORT_CONFSETTING(0x9e, "4'")
	PORT_CONFSETTING(0x6f, "2'")
	PORT_START("MUX30")
	PORT_ADJUSTER(0, "VCO Modulation") PORT_MINMAX(0, 255)
	PORT_START("MUX31")
	PORT_ADJUSTER(128, "LFO Rate") PORT_MINMAX(0, 255)
	PORT_START("KNOB0")
	PORT_ADJUSTER(128, "Knob 10") PORT_MINMAX(0, 255)
	PORT_START("KNOB1")
	PORT_ADJUSTER(128, "Knob 11") PORT_MINMAX(0, 255)
INPUT_PORTS_END

void boutique_state::boutique(machine_config &config, u16 strap, const XTAL &audio)
{
	MB8AA4181(config, m_maincpu, 156'000'000);
	m_maincpu->set_addrmap(AS_PROGRAM, &boutique_state::mem_map);
	m_maincpu->set_flash_tag("flash");
	m_maincpu->gpio_out_cb<0>().set(FUNC(boutique_state::subcpu_control_w));
	m_maincpu->gpio_out_cb<6>().set(FUNC(boutique_state::lcd_port_w));
	m_maincpu->gpio_in_cb<6>().set_constant((1 << 21) | (1 << 3));
	m_maincpu->adc_in_cb<2>().set([this] () { return u16(m_volume->read() * 0xfff / 100); });
	m_maincpu->sot_cb<1>().set(FUNC(boutique_state::lcd_data_w));
	m_maincpu->sfi_cs_cb().set(m_spiflash, FUNC(generic_spi_flash_device::cs_w));
	m_maincpu->sfi_tx_cb().set(m_spiflash, FUNC(generic_spi_flash_device::write));
	m_maincpu->sfi_rx_cb().set(m_spiflash, FUNC(generic_spi_flash_device::read));

	SPEAKER(config, "speaker", 2).front();
	mb8aa4181_dsp_device &dsp = *m_maincpu->subdevice<mb8aa4181_dsp_device>("dsp");
	dsp.set_clock(audio);
	dsp.add_route(0, "speaker", 1.0, 0);
	dsp.add_route(1, "speaker", 1.0, 1);

	GENERIC_SPI_FLASH(config, m_spiflash);

	STM32F103(config, m_subcpu, 32'000'000);
	m_subcpu->gpio_in_cb<4>().set_constant(strap);
	m_subcpu->usart_txd_cb<0>().set(m_maincpu, FUNC(mb8aa4181_device::rxd_w<2>));
	m_maincpu->txd_cb<2>().set(m_subcpu, FUNC(stm32f103_device::usart_rxd_w<0>));

	MIDI_PORT(config, "mdin", midiin_slot, "midiin").rxd_handler().set(m_maincpu, FUNC(mb8aa4181_device::rxd_w<3>));
	m_maincpu->txd_cb<3>().set("mdout", FUNC(midi_port_device::write_txd));
	MIDI_PORT(config, "mdout", midiout_slot, "midiout");
}

void boutique_state::d05(machine_config &config)
{
	boutique(config, 0xfffb, 24.576_MHz_XTAL);

	screen_device &screen(SCREEN(config, "screen"));
	screen.set_lcd();
	screen.set_refresh_hz(60);
	screen.set_size(16 * 6 - 1, 2 * 9 - 1);
	screen.set_visarea_full();
	screen.set_screen_update(m_lcdc, FUNC(hd44780_device::screen_update));
	screen.set_palette("palette");

	PALETTE(config, "palette", FUNC(boutique_state::lcd_palette), 2);

	HD44780(config, m_lcdc, 270'000);
	m_lcdc->set_lcd_size(2, 16);
	m_lcdc->set_pixel_update_cb(FUNC(boutique_state::lcd_pixel_update));
}

void boutique_state::sh01a(machine_config &config)
{
	boutique(config, 0xfff3, 22.5792_MHz_XTAL);
	m_subcpu->gpio_out_cb<4>().set(FUNC(boutique_state::subcpu_porte_w));
	m_subcpu->adc_in_cb<6>().set(FUNC(boutique_state::panel_r<6>));
	m_subcpu->adc_in_cb<7>().set(FUNC(boutique_state::panel_r<7>));
	m_subcpu->adc_in_cb<8>().set(FUNC(boutique_state::panel_r<8>));
	m_subcpu->adc_in_cb<9>().set(FUNC(boutique_state::panel_r<9>));
	m_subcpu->adc_in_cb<10>().set(FUNC(boutique_state::panel_r<10>));
	m_subcpu->adc_in_cb<11>().set(FUNC(boutique_state::panel_r<11>));
}

void boutique_state::tr08(machine_config &config)
{
	boutique(config, 0xfff7, 22.5792_MHz_XTAL);
}

ROM_START(d05)
	ROM_REGION32_LE(0x400000, "flash", 0)
	// the 1.07 update image with the Roland Cloud D-50 plugin's waves at 0x200000, where the update leaves the flash blank
	ROM_LOAD("d05_flash.bin", 0, 0x400000, BAD_DUMP CRC(4e757a03) SHA1(5dcf776575444cc168122b3d14a3d78d8550c3a8))

	ROM_REGION32_LE(0x20000, "subcpu", ROMREGION_ERASEFF)
ROM_END

ROM_START(sh01a)
	ROM_REGION32_LE(0x400000, "flash", 0)
	// the 1.07 update image
	ROM_LOAD("bq3_107.bin", 0, 0x400000, BAD_DUMP CRC(e764d4ef) SHA1(8de26fe6b18925b137eafe71c861a0caf647c2bd))

	ROM_REGION32_LE(0x20000, "subcpu", ROMREGION_ERASEFF)
ROM_END

ROM_START(tr08)
	ROM_REGION32_LE(0x400000, "flash", 0)
	// the 1.07 update image
	ROM_LOAD("bq3_107.bin", 0, 0x400000, BAD_DUMP CRC(e764d4ef) SHA1(8de26fe6b18925b137eafe71c861a0caf647c2bd))

	ROM_REGION32_LE(0x20000, "subcpu", ROMREGION_ERASEFF)
ROM_END

} // anonymous namespace


SYST(2017, d05,   0, 0, d05,   d05, boutique_state, init_boutique, "Roland", "D-05 Linear Synthesizer", MACHINE_NOT_WORKING)
SYST(2016, sh01a, 0, 0, sh01a, sh01a, boutique_state, init_boutique, "Roland", "SH-01A Synthesizer", MACHINE_NOT_WORKING)
SYST(2016, tr08,  0, 0, tr08,  d05, boutique_state, init_boutique, "Roland", "TR-08 Rhythm Composer", MACHINE_NOT_WORKING)
