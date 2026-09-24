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
#include "bus/midi/midi.h"
#include "video/hd44780.h"

#include "emupal.h"
#include "screen.h"


namespace {

class boutique_state : public driver_device
{
public:
	boutique_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_subcpu(*this, "subcpu")
		, m_lcdc(*this, "lcdc")
		, m_flash(*this, "flash")
		, m_subflash(*this, "subcpu")
	{
	}

	void d05(machine_config &config) ATTR_COLD;

	void init_boutique() ATTR_COLD;

private:
	required_device<mb8aa4181_device> m_maincpu;
	required_device<stm32f103_device> m_subcpu;
	required_device<hd44780_device> m_lcdc;
	required_region_ptr<u32> m_flash;
	required_region_ptr<u32> m_subflash;

	u32 m_lcd_port = 0;

	void mem_map(address_map &map) ATTR_COLD;

	void subcpu_control_w(u32 data);
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
	save_item(NAME(m_lcd_port));
}

void boutique_state::subcpu_control_w(u32 data)
{
	m_subcpu->set_input_line(INPUT_LINE_RESET, BIT(data, 0) ? CLEAR_LINE : ASSERT_LINE);
}

void boutique_state::lcd_port_w(u32 data)
{
	m_lcd_port = data;
}

void boutique_state::lcd_data_w(u8 data)
{
	if (!BIT(m_lcd_port, 9))
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
INPUT_PORTS_END

void boutique_state::d05(machine_config &config)
{
	MB8AA4181(config, m_maincpu, 156'000'000);
	m_maincpu->set_addrmap(AS_PROGRAM, &boutique_state::mem_map);
	m_maincpu->set_flash_tag("flash");
	m_maincpu->gpio_out_cb<0>().set(FUNC(boutique_state::subcpu_control_w));
	m_maincpu->gpio_out_cb<6>().set(FUNC(boutique_state::lcd_port_w));
	m_maincpu->gpio_in_cb<6>().set_constant(1 << 21);
	m_maincpu->sot_cb<1>().set(FUNC(boutique_state::lcd_data_w));

	STM32F103(config, m_subcpu, 32'000'000);
	m_subcpu->gpio_in_cb<4>().set_constant(0xfffb);
	m_subcpu->usart_txd_cb<0>().set(m_maincpu, FUNC(mb8aa4181_device::rxd_w<2>));
	m_maincpu->txd_cb<2>().set(m_subcpu, FUNC(stm32f103_device::usart_rxd_w<0>));

	MIDI_PORT(config, "mdin", midiin_slot, "midiin").rxd_handler().set(m_maincpu, FUNC(mb8aa4181_device::rxd_w<3>));
	m_maincpu->txd_cb<3>().set("mdout", FUNC(midi_port_device::write_txd));
	MIDI_PORT(config, "mdout", midiout_slot, "midiout");

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

ROM_START(d05)
	ROM_REGION32_LE(0x400000, "flash", 0)
	ROM_LOAD("bq3_107.bin", 0, 0x400000, CRC(e764d4ef) SHA1(8de26fe6b18925b137eafe71c861a0caf647c2bd))

	ROM_REGION32_LE(0x20000, "subcpu", ROMREGION_ERASEFF)
ROM_END

} // anonymous namespace


SYST(2017, d05, 0, 0, d05, d05, boutique_state, init_boutique, "Roland", "Boutique D-05 Linear Synthesizer", MACHINE_NOT_WORKING | MACHINE_NO_SOUND)
