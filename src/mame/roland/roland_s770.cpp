// license:BSD-3-Clause
// copyright-holders:superctr
/****************************************************************************

    Roland S-760, S-770 and S-750 16-bit samplers.

    80C196KB with an HG62E33 gate array for DRAM, DMA, MIDI and interrupts;
    MB87422 and MB87423 sampler chips, MB87424 TVF; RF5C16A video (on the
    S-760 only with the OP-760-1 option).  The boot ROM loads the system
    from floppy or SCSI into DRAM.

    Skeleton: the S-760 loads its system from floppy and reaches Perform Play.

****************************************************************************/

#include "emu.h"

#include "cpu/mcs96/i80c196kb.h"
#include "imagedev/floppy.h"
#include "formats/flopimg.h"
#include "machine/eepromser.h"
#include "machine/mb87030.h"
#include "machine/nscsi_bus.h"
#include "machine/upd765.h"
#include "bus/nscsi/devices.h"
#include "video/rf5c16.h"
#include "video/sed1330.h"

#include "emupal.h"
#include "screen.h"

#define LOG_GA      (1U << 1)
#define LOG_IO      (1U << 2)
#define LOG_DMA     (1U << 3)

#define VERBOSE (LOG_GENERAL | LOG_GA | LOG_IO | LOG_DMA)
#include "logmacro.h"

namespace {

class roland_s770_state : public driver_device
{
public:
	roland_s770_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_eeprom(*this, "eeprom")
		, m_fdc(*this, "fdc")
		, m_floppy(*this, "fdc:0")
		, m_spc(*this, "spc")
		, m_lcdc(*this, "lcdc")
		, m_vdp(*this, "vdp")
		, m_keys(*this, "ROW%u", 0U)
		, m_boot(*this, "boot")
		, m_dram(*this, "dram", 0x100000, ENDIANNESS_LITTLE)
		, m_wave(*this, "wave", 0x200000, ENDIANNESS_LITTLE)
	{
	}

	void s760(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	enum : u8 {
		IRQ_FDC = 0,
		IRQ_DMA = 4
	};

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
	u16 sp_r(offs_t offset, u16 mem_mask);
	void sp_w(offs_t offset, u16 data, u16 mem_mask);
	u8 panel_r(offs_t offset);
	void panel_w(offs_t offset, u8 data);

	void irq_w(int source, int state);
	void update_irq();
	void fdc_drq_w(int state);
	void spc_dreq_w(int state);
	void dma_service();
	TIMER_CALLBACK_MEMBER(fdc_tc);
	void dma_byte_w(u8 data);
	u8 dma_byte_r();

	static void floppy_formats(format_registration &fr);

	required_device<i80c196kb_device> m_maincpu;
	required_device<eeprom_serial_93cxx_device> m_eeprom;
	required_device<upd72069_device> m_fdc;
	required_device<floppy_connector> m_floppy;
	required_device<mb89352_device> m_spc;
	required_device<sed1330_device> m_lcdc;
	required_device<rf5c16_device> m_vdp;
	required_ioport_array<4> m_keys;
	required_region_ptr<u16> m_boot;
	memory_share_creator<u16> m_dram;
	memory_share_creator<u16> m_wave;

	u16 m_ga[0x10];
	u8 m_irq_pending;
	bool m_irq_line;
	u8 m_dma_ctrl;
	u32 m_dma_addr;
	u16 m_dma_count;
	bool m_dma_active;
	bool m_fdc_drq;
	bool m_spc_dreq;
	u8 m_panel[0x10];
	u8 m_key_row;
	emu_timer *m_fdc_tc;
	u32 m_wave_addr;
	u16 m_wave_latch;
};

void roland_s770_state::machine_start()
{
	m_fdc_tc = timer_alloc(timer_expired_delegate(FUNC(roland_s770_state::fdc_tc), this));
	save_item(NAME(m_ga));
	save_item(NAME(m_irq_pending));
	save_item(NAME(m_irq_line));
	save_item(NAME(m_dma_ctrl));
	save_item(NAME(m_dma_addr));
	save_item(NAME(m_dma_count));
	save_item(NAME(m_dma_active));
	save_item(NAME(m_fdc_drq));
	save_item(NAME(m_spc_dreq));
	save_item(NAME(m_panel));
	save_item(NAME(m_key_row));
	save_item(NAME(m_wave_addr));
	save_item(NAME(m_wave_latch));
}

void roland_s770_state::machine_reset()
{
	std::fill(std::begin(m_ga), std::end(m_ga), 0);
	for(int i = 0; i != 8; i++)
		m_ga[i] = 0x0400 | (i & 3) << 4;
	std::fill(std::begin(m_panel), std::end(m_panel), 0);
	m_irq_pending = 0;
	m_irq_line = false;
	m_dma_ctrl = 0;
	m_dma_addr = 0;
	m_dma_count = 0;
	m_dma_active = false;
	m_fdc_drq = false;
	m_spc_dreq = false;
	m_key_row = 0;
	m_wave_addr = 0;
	m_wave_latch = 0;
}

void roland_s770_state::irq_w(int source, int state)
{
	if(state)
		m_irq_pending |= 1 << source;
	else
		m_irq_pending &= ~(1 << source);
	update_irq();
}

void roland_s770_state::update_irq()
{
	bool const line = m_irq_pending & (m_ga[0xa] >> 8);
	if(line != m_irq_line) {
		m_irq_line = line;
		m_maincpu->set_input_line(i80c196kb_device::EXTINT_LINE, line ? ASSERT_LINE : CLEAR_LINE);
	}
}

u16 roland_s770_state::ga_r(offs_t offset, u16 mem_mask)
{
	u16 data = m_ga[offset];
	if(offset == 0xe && ACCESSING_BITS_8_15 && !machine().side_effects_disabled()) {
		if(BIT(m_irq_pending, IRQ_DMA)) {
			m_irq_pending &= ~(1 << IRQ_DMA);
			update_irq();
		}
	}
	if(!machine().side_effects_disabled() && offset >= 8)
		LOGMASKED(LOG_GA, "ga_r %04x & %04x = %04x (%04x)\n", 0x100 + offset*2, mem_mask, data, m_maincpu->pc());
	return data;
}

void roland_s770_state::ga_w(offs_t offset, u16 data, u16 mem_mask)
{
	if(offset >= 8)
		LOGMASKED(LOG_GA, "ga_w %04x = %04x & %04x (%04x)\n", 0x100 + offset*2, data, mem_mask, m_maincpu->pc());
	COMBINE_DATA(&m_ga[offset]);

	switch(offset) {
	case 0xa:
		if(ACCESSING_BITS_8_15)
			update_irq();
		break;
	case 0xe:
		if(ACCESSING_BITS_8_15) {
			m_dma_ctrl = data >> 8;
			m_dma_addr = m_ga[0xd] | (m_ga[0xe] & 0x000f) << 16;
			m_dma_count = m_ga[0xc];
			m_dma_active = true;
			m_fdc->tc_w(false);
			LOGMASKED(LOG_DMA, "dma start ctrl %02x addr %05x count %04x (%04x)\n", m_dma_ctrl, m_dma_addr, m_dma_count, m_maincpu->pc());
			dma_service();
		}
		break;
	}
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

void roland_s770_state::dma_byte_w(u8 data)
{
	u32 const adr = m_dma_addr & 0xfffff;
	u16 &w = m_dram[adr >> 1];
	w = BIT(adr, 0) ? (w & 0x00ff) | (data << 8) : (w & 0xff00) | data;
}

u8 roland_s770_state::dma_byte_r()
{
	u32 const adr = m_dma_addr & 0xfffff;
	return m_dram[adr >> 1] >> (BIT(adr, 0) ? 8 : 0);
}

void roland_s770_state::fdc_drq_w(int state)
{
	m_fdc_drq = state;
	dma_service();
}

void roland_s770_state::spc_dreq_w(int state)
{
	m_spc_dreq = state;
	dma_service();
}

void roland_s770_state::dma_service()
{
	while(m_dma_active) {
		bool const fdc = BIT(m_dma_ctrl, 7);
		bool const to_device = BIT(m_dma_ctrl, 5);
		if(fdc ? !m_fdc_drq : !m_spc_dreq)
			return;

		if(fdc) {
			if(to_device)
				m_fdc->dma_w(dma_byte_r());
			else {
				dma_byte_w(m_fdc->dma_r());
			}
		} else {
			if(to_device)
				m_spc->dma_w(dma_byte_r());
			else
				dma_byte_w(m_spc->dma_r());
		}
		m_dma_addr++;
		m_ga[0xd] = m_dma_addr;
		m_ga[0xe] = (m_ga[0xe] & 0xff00) | ((m_dma_addr >> 16) & 0x0f);
		if(!--m_dma_count) {
			m_dma_active = false;
			if(fdc)
				m_fdc_tc->adjust(attotime::zero);
			LOGMASKED(LOG_DMA, "dma end at %05x\n", m_dma_addr);
			irq_w(IRQ_DMA, 1);
		}
	}
}

TIMER_CALLBACK_MEMBER(roland_s770_state::fdc_tc)
{
	m_fdc->tc_w(true);
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
	if(offset == 0x203a >> 1 && !machine().side_effects_disabled()) {
		u8 const active = m_irq_pending & (m_ga[0xa] >> 8);
		if(active)
			return 0x0120 + 4 * std::countr_zero(active);
	}
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

u16 roland_s770_state::sp_r(offs_t offset, u16 mem_mask)
{
	switch(offset) {
	case 0:
		return 0x0002;
	case 5: {
		u16 const data = m_wave_latch;
		if(!machine().side_effects_disabled()) {
			m_wave_addr++;
			m_wave_latch = (m_panel[0] & 0x30) == 0x10 ? m_wave[m_wave_addr & (m_wave.length() - 1)] : 0xffff;
		}
		return data;
	}
	default:
		return io_r(offset, mem_mask);
	}
}

void roland_s770_state::sp_w(offs_t offset, u16 data, u16 mem_mask)
{
	switch(offset) {
	case 4:
		if((m_panel[0] & 0x30) == 0x10)
			COMBINE_DATA(&m_wave[m_wave_addr & (m_wave.length() - 1)]);
		m_wave_addr++;
		break;
	case 6:
		m_wave_addr = (m_wave_addr & 0x0000ffff) | (data << 16);
		break;
	case 7:
		m_wave_addr = (m_wave_addr & 0xffff0000) | data;
		break;
	default:
		io_w(offset, data, mem_mask);
		break;
	}
}

u8 roland_s770_state::panel_r(offs_t offset)
{
	switch(offset) {
	case 1: {
		floppy_image_device *floppy = m_floppy->get_device();
		bool const disk = floppy && floppy->exists();
		bool const hd = disk && floppy->get_variant() == floppy_image::DSHD;
		return (disk ? 0x01 : 0x00) | (hd ? 0x00 : 0x02);
	}
	case 5: {
		u8 const data = m_keys[m_key_row & 3]->read();
		if(!machine().side_effects_disabled())
			m_key_row++;
		return data;
	}
	case 9:
		return 0;
	default:
		if(!machine().side_effects_disabled())
			LOGMASKED(LOG_IO, "panel_r %04x (%04x)\n", 0xf000 + offset*2, m_maincpu->pc());
		return m_panel[offset];
	}
}

void roland_s770_state::panel_w(offs_t offset, u8 data)
{
	if(offset == 5)
		m_key_row = 0;
	else if(offset != 6 && offset != 7 && offset != 8)
		LOGMASKED(LOG_IO, "panel_w %04x = %02x (%04x)\n", 0xf000 + offset*2, data, m_maincpu->pc());
	m_panel[offset] = data;
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
	map(0xc000, 0xc01f).rw(FUNC(roland_s770_state::sp_r), FUNC(roland_s770_state::sp_w));
	map(0xe000, 0xe003).m(m_fdc, FUNC(upd72069_device::map)).umask16(0x00ff);
	map(0xe400, 0xe41f).m(m_spc, FUNC(mb89352_device::map)).umask16(0x00ff);
	map(0xe800, 0xe81f).rw(m_vdp, FUNC(rf5c16_device::read), FUNC(rf5c16_device::write)).umask16(0x00ff);
	map(0xf000, 0xf01f).rw(FUNC(roland_s770_state::panel_r), FUNC(roland_s770_state::panel_w)).umask16(0x00ff);
	map(0xf800, 0xf800).rw(m_lcdc, FUNC(sed1330_device::status_r), FUNC(sed1330_device::data_w));
	map(0xf802, 0xf802).rw(m_lcdc, FUNC(sed1330_device::data_r), FUNC(sed1330_device::command_w));
}

void roland_s770_state::lcdc_map(address_map &map)
{
	map(0x0000, 0x1fff).mirror(0xe000).ram();
}

static INPUT_PORTS_START(s760)
	PORT_START("ROW0")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNKNOWN) PORT_NAME("Key 0.0")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_UNKNOWN) PORT_NAME("Key 0.1")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_UNKNOWN) PORT_NAME("Key 0.2")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNKNOWN) PORT_NAME("Key 0.3")
	PORT_BIT(0xf0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("ROW1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Mode") PORT_CODE(KEYCODE_M)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_UNKNOWN) PORT_NAME("Key 1.1")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_UNKNOWN) PORT_NAME("Key 1.2")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNKNOWN) PORT_NAME("Key 1.3")
	PORT_BIT(0xf0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("ROW2")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNKNOWN) PORT_NAME("Key 2.0")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_UNKNOWN) PORT_NAME("Key 2.1")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_UNKNOWN) PORT_NAME("Key 2.2")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNKNOWN) PORT_NAME("Key 2.3")
	PORT_BIT(0xf0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("ROW3")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNKNOWN) PORT_NAME("Key 3.0")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_UNKNOWN) PORT_NAME("Key 3.1")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_UNKNOWN) PORT_NAME("Key 3.2")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNKNOWN) PORT_NAME("Key 3.3")
	PORT_BIT(0xf0, IP_ACTIVE_LOW, IPT_UNUSED)
INPUT_PORTS_END

void roland_s770_state::floppy_formats(format_registration &fr)
{
	fr.add_pc_formats();
}

static void s770_floppies(device_slot_interface &device)
{
	device.option_add("35hd", FLOPPY_35_HD);
}

void roland_s770_state::s760(machine_config &config)
{
	I80C196KB(config, m_maincpu, 16_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &roland_s770_state::data_map);
	m_maincpu->set_addrmap(AS_OPCODES, &roland_s770_state::code_map);
	m_maincpu->out_p1_cb().set(FUNC(roland_s770_state::p1_w));
	m_maincpu->in_p2_cb().set(FUNC(roland_s770_state::p2_r));

	EEPROM_93C46_16BIT(config, m_eeprom);

	UPD72069(config, m_fdc, 16_MHz_XTAL / 2);
	m_fdc->set_ready_line_connected(false);
	m_fdc->intrq_wr_callback().set([this](int state) { irq_w(IRQ_FDC, state); });
	m_fdc->drq_wr_callback().set(FUNC(roland_s770_state::fdc_drq_w));
	FLOPPY_CONNECTOR(config, m_floppy, s770_floppies, "35hd", roland_s770_state::floppy_formats).enable_sound(true);

	auto &scsi(NSCSI_BUS(config, "scsi"));
	NSCSI_CONNECTOR(config, "scsi:0", default_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:1", default_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:2", default_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:3", default_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:4", default_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:5", default_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:6", default_scsi_devices, nullptr);

	MB89352(config, m_spc, 8_MHz_XTAL);
	m_spc->out_dreq_callback().set(FUNC(roland_s770_state::spc_dreq_w));
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

SYST(1993, s760, 0, 0, s760, s760, roland_s770_state, empty_init, "Roland", "S-760 Digital Sampler", MACHINE_NOT_WORKING | MACHINE_NO_SOUND)
