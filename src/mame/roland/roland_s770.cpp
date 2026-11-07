// license:BSD-3-Clause
// copyright-holders:superctr
/****************************************************************************

    Roland S-760, S-770 and S-750 16-bit samplers.

    80C196KB with an HG62E33 gate array for DRAM, DMA, MIDI and interrupts;
    MB87422 and MB87423 sampler chips, MB87424 TVF; RF5C16A video (on the
    S-760 only with the OP-760-1 option).  The boot ROM loads the system
    from floppy or SCSI into DRAM.

    The S-760 loads its system from floppy and shows Perform Play on the LCD
    and, with the OP-760-1, on the CRT.  The MU-1 mouse and MIDI work, volumes
    load from floppy, and the sampler chips play through the TVF and the MEQ to
    the two stereo outputs; the serial inputs (sampling) and the digital
    outputs are not emulated, and there is no RC-100.

****************************************************************************/

#include "emu.h"

#include "cpu/mcs96/i80c196kb.h"
#include "imagedev/floppy.h"
#include "formats/flopimg.h"
#include "machine/eepromser.h"
#include "machine/i28f010.h"
#include "machine/mb87030.h"
#include "machine/nscsi_bus.h"
#include "machine/upd765.h"
#include "sound/roland_meq.h"
#include "sound/roland_sp.h"
#include "sound/roland_tvf.h"
#include "bus/midi/midi.h"
#include "bus/nscsi/devices.h"
#include "bus/roland/extport.h"
#include "video/rf5c16.h"
#include "video/sed1330.h"
#include "video/t6963c.h"

#include "diserial.h"
#include "emupal.h"
#include "screen.h"
#include "speaker.h"

#define LOG_GA      (1U << 1)
#define LOG_IO      (1U << 2)
#define LOG_DMA     (1U << 3)

#define VERBOSE (LOG_GENERAL | LOG_GA | LOG_IO | LOG_DMA)
#include "logmacro.h"

namespace {

class roland_s770_state : public driver_device, public device_serial_interface
{
public:
	roland_s770_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, device_serial_interface(mconfig, *this)
		, m_maincpu(*this, "maincpu")
		, m_midiout(*this, "mdout")
		, m_eeprom(*this, "eeprom")
		, m_fdc(*this, "fdc")
		, m_floppy(*this, "fdc:0")
		, m_spc(*this, "spc")
		, m_ext_port(*this, "extport")
		, m_lcdc(*this, "lcdc")
		, m_vdp(*this, "vdp")
		, m_t6963c(*this, "t6963c")
		, m_flash(*this, "flash")
		, m_sp(*this, "sp")
		, m_tvf(*this, "tvf")
		, m_meq(*this, "meq")
		, m_keys(*this, "ROW%u", 0U)
		, m_option(*this, "OPTION")
		, m_dial(*this, "DIAL")
		, m_digital_in(*this, "DIGITAL_IN")
		, m_boot(*this, "boot")
		, m_dram(*this, "dram", 0x100000, ENDIANNESS_LITTLE)
	{
	}

	void s760(machine_config &config) ATTR_COLD;
	void sp700(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	enum : u8 {
		IRQ_FDC = 0,
		IRQ_SAMPLER = 2,
		IRQ_DMA = 4,
		IRQ_MIDI_RX = 6
	};

	virtual void rcv_complete() override;
	virtual void tra_callback() override;

	void code_map(address_map &map) ATTR_COLD;
	void data_map(address_map &map) ATTR_COLD;
	void sp700_data_map(address_map &map) ATTR_COLD;
	void common(machine_config &config) ATTR_COLD;
	void lcdc_map(address_map &map) ATTR_COLD;
	void t6963c_map(address_map &map) ATTR_COLD;

	u16 ga_r(offs_t offset, u16 mem_mask);
	void ga_w(offs_t offset, u16 data, u16 mem_mask);
	void p1_w(u8 data);
	u8 digital_in_status() const;
	u8 dial_r();
	u8 p2_r();
	u16 bus_r(u16 bank, offs_t offset, u16 mem_mask);
	void bus_w(u16 bank, offs_t offset, u16 data, u16 mem_mask);
	u16 code_r(offs_t offset, u16 mem_mask);
	u16 data_r(offs_t offset, u16 mem_mask);
	void data_w(offs_t offset, u16 data, u16 mem_mask);
	u16 io_r(offs_t offset, u16 mem_mask);
	void io_w(offs_t offset, u16 data, u16 mem_mask);
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
	required_device<midi_port_device> m_midiout;
	required_device<eeprom_serial_93cxx_device> m_eeprom;
	optional_device<upd72069_device> m_fdc;
	optional_device<floppy_connector> m_floppy;
	required_device<mb89352_device> m_spc;
	optional_device<roland_ext_port_device> m_ext_port;
	optional_device<sed1330_device> m_lcdc;
	optional_device<rf5c16_device> m_vdp;
	optional_device<t6963c_device> m_t6963c;
	optional_device<intel_28f010_device> m_flash;
	required_device<roland_sp_device> m_sp;
	required_device<roland_tvf_device> m_tvf;
	required_device<roland_meq_device> m_meq;
	required_ioport_array<4> m_keys;
	required_ioport m_option;
	required_ioport m_dial;
	u8 m_dial_last;
	u8 m_dial_idle;
	s8 m_dial_steps;
	optional_ioport m_digital_in;
	required_region_ptr<u16> m_boot;
	memory_share_creator<u16> m_dram;

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
	u16 m_wave_dma_word;
	u8 m_midi_rx;
	bool m_midi_rx_full;
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
	save_item(NAME(m_wave_dma_word));
	save_item(NAME(m_midi_rx));
	save_item(NAME(m_midi_rx_full));
	save_item(NAME(m_dial_last));
	save_item(NAME(m_dial_idle));
	save_item(NAME(m_dial_steps));

	set_data_frame(1, 8, PARITY_NONE, STOP_BITS_1);
	set_rcv_rate(31250);
	set_tra_rate(31250);
}

void roland_s770_state::machine_reset()
{
	std::fill(std::begin(m_ga), std::end(m_ga), 0);
	for(int i = 0; i != 8; i++)
		m_ga[i] = 0x0780 | (i & 3) << 4;
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
	m_dial_last = m_dial->read();
	m_dial_idle = 0;
	m_dial_steps = 0;
	m_wave_dma_word = 0;
	m_midi_rx = 0;
	m_midi_rx_full = false;

	// the SP-700 configures its wave memory through F400-F406, not yet read; until then its
	// on-board memory is always the host port's
	if(m_flash)
		m_sp->host_bank_w(1);
	receive_register_reset();
	transmit_register_reset();
}

void roland_s770_state::rcv_complete()
{
	receive_register_extract();
	m_midi_rx = get_received_char();
	m_midi_rx_full = true;
	irq_w(IRQ_MIDI_RX, 1);
}

void roland_s770_state::tra_callback()
{
	m_midiout->write_txd(transmit_register_get_data_bit());
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
	if(offset == 0xa)
		data = (data & 0xff00) | (is_transmit_register_empty() ? 0x02 : 0x00) | (m_midi_rx_full ? 0x01 : 0x00);
	else if(offset == 0xb) {
		data = (data & 0xff00) | m_midi_rx;
		if(ACCESSING_BITS_0_7 && !machine().side_effects_disabled()) {
			m_midi_rx_full = false;
			irq_w(IRQ_MIDI_RX, 0);
		}
		return data;
	}
	if(offset == 0xe && ACCESSING_BITS_8_15 && !machine().side_effects_disabled()) {
		if(BIT(m_irq_pending, IRQ_DMA)) {
			m_irq_pending &= ~(1 << IRQ_DMA);
			update_irq();
		}
	}
	if(!machine().side_effects_disabled() && offset >= 8 && offset != 0xa)
		LOGMASKED(LOG_GA, "ga_r %04x & %04x = %04x (%04x)\n", 0x100 + offset*2, mem_mask, data, m_maincpu->pc());
	return data;
}

void roland_s770_state::ga_w(offs_t offset, u16 data, u16 mem_mask)
{
	if(offset == 0xb && ACCESSING_BITS_0_7) {
		if(is_transmit_register_empty())
			transmit_register_setup(data & 0xff);
		else
			LOGMASKED(LOG_GA, "midi tx overrun %02x\n", data & 0xff);
		return;
	}
	if(offset >= 8 && offset != 0xa)
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
			if(m_fdc)
				m_fdc->tc_w(false);
			LOGMASKED(LOG_DMA, "dma start ctrl %02x addr %05x count %04x (%04x)\n", m_dma_ctrl, m_dma_addr, m_dma_count, m_maincpu->pc());
			dma_service();
		}
		break;
	}
}

u8 roland_s770_state::dial_r()
{
	if(machine().side_effects_disabled())
		return 0;
	u8 const keys = m_dial->read();
	u8 const pressed = keys & ~m_dial_last;
	m_dial_last = keys;
	if(BIT(pressed, 0))
		m_dial_steps--;
	if(BIT(pressed, 1))
		m_dial_steps++;
	if(m_dial_idle) {
		m_dial_idle--;
		return 0;
	}
	if(!m_dial_steps)
		return 0;
	bool const up = m_dial_steps > 0;
	m_dial_steps += up ? -1 : 1;
	m_dial_idle = 4;
	return up ? 0x02 : 0x03;
}

u8 roland_s770_state::digital_in_status() const
{
	static const u8 FIN[4] = { 0x00, 0x00, 0x10, 0x18 };
	u8 const in = m_digital_in ? m_digital_in->read() : 0;
	if(!(in & 3))
		return 0x00;
	return 0x80 | FIN[in & 3] | (BIT(in, 2) ? 0x40 : 0x00);
}

void roland_s770_state::p1_w(u8 data)
{
	static const u32 RATES[4] = { 44100, 44100, 48000, 32000 };
	u8 const in = m_digital_in ? m_digital_in->read() : 0;
	u32 const rate = BIT(data, 5) && (in & 3) ? RATES[in & 3] : RATES[data >> 6];
	if(rate != m_sp->clock()) {
		m_sp->set_unscaled_clock(rate);
		m_tvf->set_unscaled_clock(rate);
		m_meq->set_unscaled_clock(rate);
	}

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
		bool const to_device = !BIT(m_dma_ctrl, 0);
		bool const wave = (m_dma_ctrl & 0xc0) == 0x80 && (BIT(m_dma_ctrl, 3) || (m_dma_ctrl & 7) == 7);
		bool const fdc = !wave && BIT(m_dma_ctrl, 7);
		if(!wave && (fdc ? !m_fdc_drq : !m_spc_dreq))
			return;

		if(wave) {
			bool const high = BIT(m_dma_count, 0);
			if(to_device) {
				u8 const data = dma_byte_r();
				if(!high)
					m_wave_dma_word = data;
				else
					m_sp->host_write(m_wave_dma_word | data << 8, 0xffff);
			} else {
				if(!high)
					m_wave_dma_word = m_sp->host_read();
				dma_byte_w(m_wave_dma_word >> (high ? 8 : 0));
			}
		} else if(fdc) {
			if(to_device)
				m_fdc->dma_w(dma_byte_r());
			else
				dma_byte_w(m_fdc->dma_r());
		} else {
			if(to_device)
				m_spc->dma_w(dma_byte_r());
			else
				dma_byte_w(m_spc->dma_r());
		}
		m_dma_addr++;
		m_ga[0xd] = m_dma_addr;
		m_ga[0xe] = (m_ga[0xe] & 0xff00) | ((m_dma_addr >> 16) & 0x0f);
		m_ga[0xc] = --m_dma_count;
		if(!m_dma_count) {
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
	if(m_fdc)
		m_fdc->tc_w(true);
}

u16 roland_s770_state::bus_r(u16 bank, offs_t offset, u16 mem_mask)
{
	u32 const adr = ((bank & 0x3ff) << 10) + (offset << 1);
	if(BIT(bank, 10)) {
		if(m_flash && (adr & 0xe0000) == 0x40000) {
			u16 data = 0;
			if(ACCESSING_BITS_0_7)
				data |= m_flash->read(adr);
			if(ACCESSING_BITS_8_15)
				data |= m_flash->read(adr + 1) << 8;
			return data;
		}
		return m_boot[(adr & (m_boot.bytes() - 1)) >> 1];
	}
	return m_dram[(adr & 0xfffff) >> 1];
}

void roland_s770_state::bus_w(u16 bank, offs_t offset, u16 data, u16 mem_mask)
{
	u32 const adr = ((bank & 0x3ff) << 10) + (offset << 1);
	if(BIT(bank, 10) && m_flash && (adr & 0xe0000) == 0x40000) {
		if(ACCESSING_BITS_0_7)
			m_flash->write(adr, data);
		if(ACCESSING_BITS_8_15)
			m_flash->write(adr + 1, data >> 8);
	} else if(BIT(bank, 10))
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

u8 roland_s770_state::panel_r(offs_t offset)
{
	switch(offset) {
	case 1: {
		floppy_image_device *floppy = m_floppy ? m_floppy->get_device() : nullptr;
		bool const disk = floppy && floppy->exists();
		bool const hd = disk && floppy->get_variant() == floppy_image::DSHD;
		return (disk ? 0x01 : 0x00) | (hd ? 0x00 : 0x02) | digital_in_status();
	}
	case 5: {
		u8 const data = (m_keys[m_key_row & 3]->read() & 0x3f) | m_option->read();
		if(!machine().side_effects_disabled())
			m_key_row++;
		return data;
	}
	case 4:
		return m_ext_port ? m_ext_port->read() : 0xff;
	case 9:
		return dial_r();
	default:
		if(!machine().side_effects_disabled())
			LOGMASKED(LOG_IO, "panel_r %04x (%04x)\n", 0xf000 + offset*2, m_maincpu->pc());
		return m_panel[offset];
	}
}

void roland_s770_state::panel_w(offs_t offset, u8 data)
{
	if(offset == 0)
		m_sp->host_bank_w((data >> 4) & 3);
	if(offset == 4) {
		if(m_ext_port)
			m_ext_port->write(data);
	}
	else if(offset == 5)
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
	map(0xc000, 0xc7ff).rw(m_sp, FUNC(roland_sp_device::read), FUNC(roland_sp_device::write));
	map(0xd000, 0xd07f).rw(m_tvf, FUNC(roland_tvf_device::read), FUNC(roland_tvf_device::write));
	map(0xdc00, 0xdc0b).rw(m_meq, FUNC(roland_meq_device::read), FUNC(roland_meq_device::write));
	map(0xe000, 0xe003).m(m_fdc, FUNC(upd72069_device::map)).umask16(0x00ff);
	map(0xe400, 0xe41f).m(m_spc, FUNC(mb89352_device::map)).umask16(0x00ff);
	map(0xe800, 0xe81f).rw(m_vdp, FUNC(rf5c16_device::read), FUNC(rf5c16_device::write)).umask16(0x00ff);
	map(0xf000, 0xf01f).rw(FUNC(roland_s770_state::panel_r), FUNC(roland_s770_state::panel_w)).umask16(0x00ff);
	map(0xf800, 0xf800).rw(m_lcdc, FUNC(sed1330_device::status_r), FUNC(sed1330_device::data_w));
	map(0xf802, 0xf802).rw(m_lcdc, FUNC(sed1330_device::data_r), FUNC(sed1330_device::command_w));
}

void roland_s770_state::sp700_data_map(address_map &map)
{
	map(0x0000, 0xbfff).rw(FUNC(roland_s770_state::data_r), FUNC(roland_s770_state::data_w));
	map(0x0100, 0x011f).rw(FUNC(roland_s770_state::ga_r), FUNC(roland_s770_state::ga_w));
	map(0xc000, 0xffff).rw(FUNC(roland_s770_state::io_r), FUNC(roland_s770_state::io_w));
	map(0xc000, 0xc7ff).rw(m_sp, FUNC(roland_sp_device::read), FUNC(roland_sp_device::write));
	map(0xd000, 0xd07f).rw(m_tvf, FUNC(roland_tvf_device::read), FUNC(roland_tvf_device::write));
	map(0xd400, 0xd40b).rw(m_meq, FUNC(roland_meq_device::read), FUNC(roland_meq_device::write));
	map(0xd800, 0xd81f).m(m_spc, FUNC(mb89352_device::map)).umask16(0x00ff);
	map(0xdc00, 0xdc03).rw(m_t6963c, FUNC(t6963c_device::read), FUNC(t6963c_device::write)).umask16(0x00ff);
	map(0xf000, 0xf01f).rw(FUNC(roland_s770_state::panel_r), FUNC(roland_s770_state::panel_w)).umask16(0x00ff);
}

void roland_s770_state::lcdc_map(address_map &map)
{
	map(0x0000, 0x1fff).mirror(0xe000).ram();
}

void roland_s770_state::t6963c_map(address_map &map)
{
	map(0x0000, 0x1fff).mirror(0x6000).ram();
}

static INPUT_PORTS_START(s760)
	PORT_START("ROW0")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNKNOWN) PORT_NAME("Key 0.0")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("S1/Dec") PORT_CODE(KEYCODE_MINUS)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("S2/Inc") PORT_CODE(KEYCODE_EQUALS)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNKNOWN) PORT_NAME("Key 0.3")
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_UNKNOWN) PORT_NAME("Key 0.4")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Preview") PORT_CODE(KEYCODE_P)
	PORT_BIT(0xc0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("ROW1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Mode") PORT_CODE(KEYCODE_M)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Exit") PORT_CODE(KEYCODE_BACKSPACE)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Up") PORT_CODE(KEYCODE_UP)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNKNOWN) PORT_NAME("Key 1.3")
	PORT_BIT(0xf0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("ROW2")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("F1") PORT_CODE(KEYCODE_F1)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_UNKNOWN) PORT_NAME("Key 2.1")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_UNKNOWN) PORT_NAME("Key 2.2")
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNKNOWN) PORT_NAME("Key 2.3")
	PORT_BIT(0xf0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("ROW3")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_UNKNOWN) PORT_NAME("Key 3.0")
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_UNKNOWN) PORT_NAME("Key 3.1")
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Down") PORT_CODE(KEYCODE_DOWN)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_UNKNOWN) PORT_NAME("Key 3.3")
	PORT_BIT(0xf0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("DIAL")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Value -") PORT_CODE(KEYCODE_COMMA)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Value +") PORT_CODE(KEYCODE_STOP)

	PORT_START("OPTION")
	PORT_CONFNAME(0xc0, 0x00, "Option board")
	PORT_CONFSETTING(0xc0, DEF_STR(None))
	PORT_CONFSETTING(0x00, "OP-760-1")
	PORT_CONFSETTING(0x40, "Option ID 01")

	PORT_START("DIGITAL_IN")
	PORT_CONFNAME(0x03, 0x00, "OP-760-1 digital input")
	PORT_CONFSETTING(0x00, "Not connected")
	PORT_CONFSETTING(0x01, "44.1 kHz")
	PORT_CONFSETTING(0x02, "48 kHz")
	PORT_CONFSETTING(0x03, "32 kHz")
	PORT_CONFNAME(0x04, 0x00, "Digital input emphasis")
	PORT_CONFSETTING(0x00, DEF_STR(Off))
	PORT_CONFSETTING(0x04, DEF_STR(On))
INPUT_PORTS_END

void roland_s770_state::floppy_formats(format_registration &fr)
{
	fr.add_pc_formats();
}

static void s770_floppies(device_slot_interface &device)
{
	device.option_add("35hd", FLOPPY_35_HD);
}

// as the boot ROM and system 2.24 initialise it, then Controller Mouse+CRT and Exclusive RX on
static const u16 s760_eeprom[0x40] = {
	0x0702, 0x0000, 0x0001, 0x0000, 0x0801, 0x0000, 0x2e31, 0x3030,
	0x6f52, 0x616c, 0x646e, 0x6556, 0x2e72, 0x3120, 0x302e, 0x3631,
	0x0100, 0x00ff, 0x0000, 0x0100, 0x0000, 0x0000, 0x00e1, 0x00e1,
	0x00e1, 0x00e1, 0x0000, 0x0000, 0x0001, 0x0000, 0x0000, 0x0000,
	0x7f08, 0x0000, 0x0001, 0x00ff, 0x0001, 0x3c3c, 0x3c3c, 0x7f7f,
	0x7f7f, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	0x1918, 0x1b1a, 0x1d1c, 0x1f1e, 0x4f4e, 0x5150, 0x5352, 0x5554,
	0x1110, 0x1312, 0x1514, 0x1716, 0x4746, 0x4948, 0x4b4a, 0x4d4c,
};

void roland_s770_state::common(machine_config &config)
{
	I80C196KB(config, m_maincpu, 16_MHz_XTAL);
	m_maincpu->set_addrmap(AS_OPCODES, &roland_s770_state::code_map);
	m_maincpu->out_p1_cb().set(FUNC(roland_s770_state::p1_w));
	m_maincpu->in_p2_cb().set(FUNC(roland_s770_state::p2_r));

	EEPROM_93C46_16BIT(config, m_eeprom).default_data(s760_eeprom, sizeof(s760_eeprom));

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

	MIDI_PORT(config, "mdin", midiin_slot, "midiin").rxd_handler().set(FUNC(roland_s770_state::rx_w));
	MIDI_PORT(config, m_midiout, midiout_slot, "midiout");

	SPEAKER(config, "speaker", 2).front();

	ROLAND_SP(config, m_sp, 44100);
	m_sp->int_callback().set([this](int state) { irq_w(IRQ_SAMPLER, state); });
	ROLAND_TVF(config, m_tvf, 44100);
	// sampler voices 4-15 and 20-31 play, into TVF voices 0-23; 0-3 and 16-19 are never keyed
	for(int n = 0; n != roland_sp_device::VOICES; n++)
		m_sp->add_route(n, m_tvf, 1.0, n < 16 ? (n + 28) & 31 : n - 8);
	ROLAND_MEQ(config, m_meq, 44100);
	for(int n = 0; n != roland_meq_device::VOICES; n++)
		m_tvf->add_route(n, m_meq, 1.0, n);
}

// the MEQ steps at which system 2.24's program takes each TVF voice
static const u16 s760_meq_slots[24][2] = {
	{ 0x25, 0x26 }, { 0x2e, 0x2f }, { 0x37, 0x38 }, { 0x40, 0x41 }, { 0x49, 0x4a }, { 0x52, 0x53 },
	{ 0x5b, 0x5c }, { 0x60, 0x61 }, { 0x69, 0x6a }, { 0x72, 0x73 }, { 0x78, 0x79 }, { 0x7c, 0x7d },
	{ 0xa3, 0xa4 }, { 0xac, 0xad }, { 0x9a, 0x9b }, { 0x9e, 0x9f }, { 0xa7, 0xa8 }, { 0xb0, 0xb1 },
	{ 0xba, 0xbb }, { 0xbe, 0xbf }, { 0x08, 0x09 }, { 0x11, 0x12 }, { 0x1a, 0x1b }, { 0x21, 0x22 }
};

void roland_s770_state::s760(machine_config &config)
{
	common(config);
	m_maincpu->set_addrmap(AS_PROGRAM, &roland_s770_state::data_map);

	// SDO1's latches in step order: DIGITAL OUT 1 L, STEREO OUT 1 R, DIGITAL OUT 1 R, STEREO OUT 1 L;
	// SDO2's likewise for the second pair
	m_meq->set_voice_slots(s760_meq_slots);
	m_meq->port3_callback().set(m_sp, FUNC(roland_sp_device::record_w));
	m_meq->set_synchronous(true);
	SPEAKER(config, "out2", 2).front();
	m_meq->add_route(3, "speaker", 1.0, 0);
	m_meq->add_route(1, "speaker", 1.0, 1);
	m_meq->add_route(7, "out2", 1.0, 0);
	m_meq->add_route(5, "out2", 1.0, 1);
	SPEAKER(config, "dout1", 2).front();
	SPEAKER(config, "dout2", 2).front();
	m_meq->add_route(0, "dout1", 1.0, 0);
	m_meq->add_route(2, "dout1", 1.0, 1);
	m_meq->add_route(4, "dout2", 1.0, 0);
	m_meq->add_route(6, "dout2", 1.0, 1);

	auto &adc(MICROPHONE(config, "adc", 2).front());
	adc.add_route(0, m_meq, 1.0, roland_meq_device::INPUT_SERIAL);
	adc.add_route(1, m_meq, 1.0, roland_meq_device::INPUT_SERIAL + 1);

	auto &digital_in(MICROPHONE(config, "digital_in", 2).front());
	digital_in.add_route(0, m_meq, 1.0, roland_meq_device::INPUT_SERIAL + 2);
	digital_in.add_route(1, m_meq, 1.0, roland_meq_device::INPUT_SERIAL + 3);
	m_meq->set_serial_right_first(1, true);

	UPD72069(config, m_fdc, 16_MHz_XTAL / 2);
	m_fdc->set_ready_line_connected(false);
	m_fdc->intrq_wr_callback().set([this](int state) { irq_w(IRQ_FDC, state); });
	m_fdc->drq_wr_callback().set(FUNC(roland_s770_state::fdc_drq_w));
	FLOPPY_CONNECTOR(config, m_floppy, s770_floppies, "35hd", roland_s770_state::floppy_formats).enable_sound(true);

	ROLAND_EXT_PORT(config, m_ext_port, roland_ext_port_devices, "mouse");

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

void roland_s770_state::sp700(machine_config &config)
{
	common(config);
	m_maincpu->set_addrmap(AS_PROGRAM, &roland_s770_state::sp700_data_map);

	INTEL_28F010(config, m_flash);

	PALETTE(config, "palette", palette_device::MONOCHROME_INVERTED);

	T6963C(config, m_t6963c);
	m_t6963c->set_addrmap(0, &roland_s770_state::t6963c_map);

	screen_device &lcd(SCREEN(config, "lcd").set_lcd());
	lcd.set_refresh_hz(60);
	lcd.set_size(240, 64);
	lcd.set_visarea_full();
	lcd.set_screen_update(m_t6963c, FUNC(t6963c_device::screen_update));
	lcd.set_palette("palette");
}

ROM_START(s760)
	ROM_REGION16_LE(0x8000, "boot", 0)
	ROM_LOAD("s760_1.0.1.bin", 0x0000, 0x8000, CRC(ac9d8be1) SHA1(c77dbbb6510e27d4ee15e55440e713795c4ee74e))
ROM_END

// the EPROMs and the flash in the upper half of the 2 MB bus: flash at 140000, IC55 at 1A0000,
// IC54 at 1C0000 with the boot block at 1E0000
ROM_START(sp700)
	ROM_REGION16_LE(0x100000, "boot", ROMREGION_ERASEFF)
	ROM_LOAD("sp700_ic55_1.12.bin", 0xa0000, 0x20000, CRC(476eac70) SHA1(83ed101295fbba9cd850054bc7d90c478853f604))
	ROM_LOAD("sp700_ic54_1.12.bin", 0xc0000, 0x40000, CRC(0b25f092) SHA1(55aa0a232e23b21e8f0088ca98413ff8868c6ae1))

	ROM_REGION(0x400, "t6963c:cgrom", ROMREGION_ERASE00)
	ROM_LOAD("t6963c_0101.bin", 0x000, 0x400, NO_DUMP)
ROM_END

} // anonymous namespace

SYST(1993, s760, 0, 0, s760, s760, roland_s770_state, empty_init, "Roland", "S-760 Digital Sampler", MACHINE_NOT_WORKING)
SYST(1996, sp700, 0, 0, sp700, s760, roland_s770_state, empty_init, "Roland", "SP-700 16-bit Sample Player", MACHINE_NOT_WORKING | MACHINE_NO_SOUND)
