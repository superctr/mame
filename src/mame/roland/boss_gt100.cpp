// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    BOSS GT-100 COSM Amp Effects Processor
    BOSS GT-001 Guitar Effects Processor

    An ESC2 (MB8AA4181) booting from a 4 MiB serial flash, with SDRAM on
    its external bus.

***************************************************************************/

#include "emu.h"

#include "roland_esc2.h"

#include "machine/generic_spi_flash.h"

#include "speaker.h"


namespace {

class gt100_state : public driver_device
{
public:
	gt100_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_spiflash(*this, "spiflash")
	{
	}

	void gt100(machine_config &config) ATTR_COLD;

private:
	required_device<mb8aa4181_device> m_maincpu;
	required_device<generic_spi_flash_device> m_spiflash;

	void mem_map(address_map &map) ATTR_COLD;

	virtual void machine_start() override ATTR_COLD;
};

void gt100_state::mem_map(address_map &map)
{
	map(0x60000000, 0x61ffffff).ram();
}

void gt100_state::machine_start()
{
	m_spiflash->set_rom_ptr(memregion("flash")->base());
	m_spiflash->set_rom_size(memregion("flash")->bytes());
}

static INPUT_PORTS_START(gt100)
INPUT_PORTS_END

void gt100_state::gt100(machine_config &config)
{
	MB8AA4181(config, m_maincpu, 156'000'000);
	m_maincpu->set_addrmap(AS_PROGRAM, &gt100_state::mem_map);
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
}

ROM_START(gt100)
	ROM_REGION32_LE(0x400000, "flash", 0)
	// the 2.12 update image
	ROM_LOAD("gt100_212.bin", 0, 0x400000, BAD_DUMP CRC(50583566) SHA1(5175f207a640b91a2c3d2fbec324dbc8d8e3a51a))
ROM_END

ROM_START(gt001)
	ROM_REGION32_LE(0x400000, "flash", 0)
	// the 1.10 update image
	ROM_LOAD("gt001_110.bin", 0, 0x400000, BAD_DUMP CRC(a78a0374) SHA1(e7e08966fdc3ea549674b475f098727278b92276))
ROM_END

} // anonymous namespace


SYST(2012, gt100, 0, 0, gt100, gt100, gt100_state, empty_init, "Boss", "GT-100 COSM Amp Effects Processor", MACHINE_NOT_WORKING | MACHINE_NO_SOUND)
SYST(2015, gt001, 0, 0, gt100, gt100, gt100_state, empty_init, "Boss", "GT-001 Guitar Effects Processor", MACHINE_NOT_WORKING | MACHINE_NO_SOUND)
