// license:BSD-3-Clause
// copyright-holders:superctr
/****************************************************************************

    Roland S-760, S-770 and S-750 16-bit samplers.

    80C196KB with an HG62E33 gate array for DRAM, DMA, MIDI and interrupts;
    MB87422 and MB87423 sampler chips, MB87424 TVF; RF5C16A video (on the
    S-760 only with the OP-760-1 option).  The boot ROM loads the system
    from floppy or SCSI into DRAM.

    Skeleton: the S-760 runs its boot ROM.

****************************************************************************/

#include "emu.h"

#include "cpu/mcs96/i80c196kb.h"
#include "machine/eepromser.h"
#include "machine/mb87030.h"
#include "machine/nscsi_bus.h"
#include "bus/nscsi/devices.h"
#include "video/rf5c16.h"
#include "video/sed1330.h"

#include "emupal.h"
#include "screen.h"

#define LOG_GA      (1U << 1)
#define LOG_IO      (1U << 2)

#define VERBOSE (LOG_GENERAL | LOG_GA | LOG_IO)
#include "logmacro.h"

namespace {

class roland_s770_state : public driver_device
{
public:
	roland_s770_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_eeprom(*this, "eeprom")
		, m_spc(*this, "spc")
		, m_lcdc(*this, "lcdc")
		, m_vdp(*this, "vdp")
		, m_boot(*this, "boot")
		, m_dram(*this, "dram", 0x100000, ENDIANNESS_LITTLE)
	{
	}

	void s760(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	void code_map(address_map &map) ATTR_COLD;
	void data_map(address_map &map) ATTR_COLD;
	void lcdc_map(address_map &map) ATTR_COLD;

	u16 ga_r(offs_t offset, u16 mem_mask);
	void ga_w(offs_t offset, u16 data, u16 mem_mask);
	void p1_w(u8 data);
	u8 p2_r();
	u16 bus_r(u16 bank, offs_t offset, u16 mem_mask);
	void bus_w(u16 bank, offs_t offset, u16 data, u16 mem_mask);
	u16 code_r(offs_t offset, u16 mem_mask);
	u16 data_r(offs_t offset, u16 mem_mask);
	void data_w(offs_t offset, u16 data, u16 mem_mask);
	u16 io_r(offs_t offset, u16 mem_mask);
	void io_w(offs_t offset, u16 data, u16 mem_mask);

	required_device<i80c196kb_device> m_maincpu;
	required_device<eeprom_serial_93cxx_device> m_eeprom;
	required_device<mb89352_device> m_spc;
	required_device<sed1330_device> m_lcdc;
	required_device<rf5c16_device> m_vdp;
	required_region_ptr<u16> m_boot;
	memory_share_creator<u16> m_dram;

	u16 m_ga[0x10];
};

void roland_s770_state::machine_start()
{
	save_item(NAME(m_ga));
}

void roland_s770_state::machine_reset()
{
	std::fill(std::begin(m_ga), std::end(m_ga), 0);
	for(int i = 0; i != 8; i++)
		m_ga[i] = 0x0400 | (i & 3) << 4;
}

u16 roland_s770_state::ga_r(offs_t offset, u16 mem_mask)
{
	if(!machine().side_effects_disabled() && offset >= 8)
		LOGMASKED(LOG_GA, "ga_r %04x & %04x = %04x (%04x)\n", 0x100 + offset*2, mem_mask, m_ga[offset], m_maincpu->pc());
	return m_ga[offset];
}

void roland_s770_state::ga_w(offs_t offset, u16 data, u16 mem_mask)
{
	if(offset >= 8)
		LOGMASKED(LOG_GA, "ga_w %04x = %04x & %04x (%04x)\n", 0x100 + offset*2, data, mem_mask, m_maincpu->pc());
	COMBINE_DATA(&m_ga[offset]);
}

void roland_s770_state::p1_w(u8 data)
{
	m_eeprom->cs_write(BIT(data, 0));
	m_eeprom->di_write(BIT(data, 2));
	m_eeprom->clk_write(BIT(data, 1));
}

u8 roland_s770_state::p2_r()
{
	return 0x7f | (m_eeprom->do_read() << 7);
}

u16 roland_s770_state::bus_r(u16 bank, offs_t offset, u16 mem_mask)
{
	u32 const adr = ((bank & 0x3ff) << 10) + (offset << 1);
	if(BIT(bank, 10))
		return m_boot[(adr & 0x7fff) >> 1];
	return m_dram[(adr & 0xfffff) >> 1];
}

void roland_s770_state::bus_w(u16 bank, offs_t offset, u16 data, u16 mem_mask)
{
	u32 const adr = ((bank & 0x3ff) << 10) + (offset << 1);
	if(BIT(bank, 10))
		LOGMASKED(LOG_GA, "write to boot ROM %05x = %04x (%04x)\n", adr, data, m_maincpu->pc());
	else
		COMBINE_DATA(&m_dram[(adr & 0xfffff) >> 1]);
}

u16 roland_s770_state::code_r(offs_t offset, u16 mem_mask)
{
	return bus_r(m_ga[offset >> 13], offset & 0x1fff, mem_mask);
}

u16 roland_s770_state::data_r(offs_t offset, u16 mem_mask)
{
	return bus_r(m_ga[4 + (offset >> 13)], offset & 0x1fff, mem_mask);
}

void roland_s770_state::data_w(offs_t offset, u16 data, u16 mem_mask)
{
	bus_w(m_ga[4 + (offset >> 13)], offset & 0x1fff, data, mem_mask);
}

u16 roland_s770_state::io_r(offs_t offset, u16 mem_mask)
{
	if(!machine().side_effects_disabled())
		LOGMASKED(LOG_IO, "io_r %04x & %04x (%04x)\n", 0xc000 + offset*2, mem_mask, m_maincpu->pc());
	return 0;
}

void roland_s770_state::io_w(offs_t offset, u16 data, u16 mem_mask)
{
	LOGMASKED(LOG_IO, "io_w %04x = %04x & %04x (%04x)\n", 0xc000 + offset*2, data, mem_mask, m_maincpu->pc());
}

void roland_s770_state::code_map(address_map &map)
{
	map(0x0000, 0xffff).r(FUNC(roland_s770_state::code_r));
}

void roland_s770_state::data_map(address_map &map)
{
	map(0x0000, 0xbfff).rw(FUNC(roland_s770_state::data_r), FUNC(roland_s770_state::data_w));
	map(0x0100, 0x011f).rw(FUNC(roland_s770_state::ga_r), FUNC(roland_s770_state::ga_w));
	map(0xc000, 0xffff).rw(FUNC(roland_s770_state::io_r), FUNC(roland_s770_state::io_w));
	map(0xe400, 0xe41f).m(m_spc, FUNC(mb89352_device::map)).umask16(0x00ff);
	map(0xe800, 0xe81f).rw(m_vdp, FUNC(rf5c16_device::read), FUNC(rf5c16_device::write)).umask16(0x00ff);
	map(0xf800, 0xf800).rw(m_lcdc, FUNC(sed1330_device::status_r), FUNC(sed1330_device::data_w));
	map(0xf802, 0xf802).rw(m_lcdc, FUNC(sed1330_device::data_r), FUNC(sed1330_device::command_w));
}

void roland_s770_state::lcdc_map(address_map &map)
{
	map(0x0000, 0x1fff).mirror(0xe000).ram();
}

void roland_s770_state::s760(machine_config &config)
{
	I80C196KB(config, m_maincpu, 16_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &roland_s770_state::data_map);
	m_maincpu->set_addrmap(AS_OPCODES, &roland_s770_state::code_map);
	m_maincpu->out_p1_cb().set(FUNC(roland_s770_state::p1_w));
	m_maincpu->in_p2_cb().set(FUNC(roland_s770_state::p2_r));

	EEPROM_93C46_16BIT(config, m_eeprom);

	auto &scsi(NSCSI_BUS(config, "scsi"));
	NSCSI_CONNECTOR(config, "scsi:0", default_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:1", default_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:2", default_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:3", default_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:4", default_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:5", default_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:6", default_scsi_devices, nullptr);

	MB89352(config, m_spc, 8_MHz_XTAL);
	scsi.set_external_device(7, m_spc);

	PALETTE(config, "palette", palette_device::MONOCHROME_INVERTED);

	screen_device &lcd(SCREEN(config, "lcd").set_lcd());
	lcd.set_refresh_hz(60);
	lcd.set_size(160, 64);
	lcd.set_visarea_full();
	lcd.set_screen_update(m_lcdc, FUNC(sed1330_device::screen_update));
	lcd.set_palette("palette");

	SED1330(config, m_lcdc, 8_MHz_XTAL);
	m_lcdc->set_screen("lcd");
	m_lcdc->set_addrmap(0, &roland_s770_state::lcdc_map);

	screen_device &crt(SCREEN(config, "crt"));
	crt.set_raw(14.318181_MHz_XTAL, 912, 0, 640, 262, 0, 200);
	crt.set_screen_update(m_vdp, FUNC(rf5c16_device::screen_update));

	RF5C16(config, m_vdp, 14.318181_MHz_XTAL);
	m_vdp->set_screen("crt");
}

ROM_START(s760)
	ROM_REGION16_LE(0x8000, "boot", 0)
	ROM_LOAD("s760_1.0.1.bin", 0x0000, 0x8000, CRC(ac9d8be1) SHA1(c77dbbb6510e27d4ee15e55440e713795c4ee74e))
ROM_END

} // anonymous namespace

SYST(1993, s760, 0, 0, s760, 0, roland_s770_state, empty_init, "Roland", "S-760 Digital Sampler", MACHINE_NOT_WORKING | MACHINE_NO_SOUND)
