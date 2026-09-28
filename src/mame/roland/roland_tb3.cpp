// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Roland AIRA TB-3 Touch Bassline

    An ESC2 (MB8AA4181) booting from a 2 MiB serial flash, with SDRAM on
    its external bus.

***************************************************************************/

#include "emu.h"

#include "roland_esc2.h"

#include "machine/generic_spi_flash.h"
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
	{
	}

	void tb3(machine_config &config) ATTR_COLD;

private:
	required_device<mb8aa4181_device> m_maincpu;
	required_device<generic_spi_flash_device> m_spiflash;

	void mem_map(address_map &map) ATTR_COLD;

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
}

static INPUT_PORTS_START(tb3)
INPUT_PORTS_END

void tb3_state::tb3(machine_config &config)
{
	MB8AA4181(config, m_maincpu, 156'000'000);
	m_maincpu->set_addrmap(AS_PROGRAM, &tb3_state::mem_map);
	m_maincpu->set_flash_tag("flash");
	m_maincpu->sfi_cs_cb().set(m_spiflash, FUNC(generic_spi_flash_device::cs_w));
	m_maincpu->sfi_tx_cb().set(m_spiflash, FUNC(generic_spi_flash_device::write));
	m_maincpu->sfi_rx_cb().set(m_spiflash, FUNC(generic_spi_flash_device::read));

	SPEAKER(config, "speaker", 2).front();
	mb8aa4181_dsp_device &dsp = *m_maincpu->subdevice<mb8aa4181_dsp_device>("dsp");
	dsp.set_clock(24.576_MHz_XTAL);
	dsp.add_route(0, "speaker", 1.0, 0);
	dsp.add_route(1, "speaker", 1.0, 1);

	GENERIC_SPI_FLASH(config, m_spiflash);

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


SYST(2014, tb3, 0, 0, tb3, tb3, tb3_state, empty_init, "Roland", "TB-3 Touch Bassline", MACHINE_NOT_WORKING | MACHINE_NO_SOUND)
