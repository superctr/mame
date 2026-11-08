// license:BSD-3-Clause
// copyright-holders:superctr
/****************************************************************************

    Roland S-760, S-770 and S-750 16-bit samplers, and the SP-700 player.

    80C196KB with an HG62E33 gate array for DRAM, DMA, MIDI and interrupts;
    MB87422 and MB87423 sampler chips, MB87424 TVF; RF5C16A video (on the
    S-760 only with the OP-760-1 option).  The boot ROM loads the system
    from floppy or SCSI into DRAM; the SP-700 runs from EPROM.

    The S-760 mixes through the MEQ to two stereo and two digital outputs and
    samples through it; the S-770 and S-750 mix through the MIX to the stereo
    pair and six individual outputs.  Panels, the value dial, the MU-1 mouse,
    the RC-100 on the EXT CTRL port, MIDI, floppy and SCSI CD-ROM work.

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
#include "sound/roland_mix.h"
#include "sound/roland_sp.h"
#include "sound/roland_tvf.h"
#include "bus/midi/midi.h"
#include "bus/nscsi/devices.h"
#include "bus/msx/ctrl/ctrl.h"
#include "bus/msx/ctrl/mouse.h"
#include "bus/roland/rc100.h"
#include "video/rf5c16.h"
#include "video/sed1330.h"
#include "video/t6963c.h"

#include "diserial.h"
#include "emupal.h"
#include "screen.h"
#include "speaker.h"

#include "ssc1000.h"

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
		, m_adc(*this, "adc")
		, m_lcdc(*this, "lcdc")
		, m_vdp(*this, "vdp")
		, m_t6963c(*this, "t6963c")
		, m_flash(*this, "flash")
		, m_sp(*this, "sp")
		, m_tvf(*this, "tvf")
		, m_meq(*this, "meq")
		, m_mix(*this, "mix")
		, m_keys(*this, "ROW%u", 0U)
		, m_option(*this, "OPTION")
		, m_dial(*this, "DIAL")
		, m_mod(*this, "MOD")
		, m_scratch(*this, "SCRATCH")
		, m_digital_in(*this, "DIGITAL_IN")
		, m_columns(*this, "COL%u", 0U)
		, m_keyscan(*this, "keyscan")
		, m_velocity(*this, "VELOCITY")
		, m_wave_mem(*this, "WAVE_MEM")
		, m_simms(*this, "SIMM%u", 1U)
		, m_boot(*this, "boot")
		, m_dram(*this, "dram", 0x100000, ENDIANNESS_LITTLE)
		, m_leds(*this, "led%u", 0U)
	{
	}

	void s760(machine_config &config) ATTR_COLD;
	void sp700(machine_config &config) ATTR_COLD;
	void s770(machine_config &config) ATTR_COLD;
	void s750turbo(machine_config &config) ATTR_COLD;
	void dj70(machine_config &config) ATTR_COLD;
	void dj70mk2(machine_config &config) ATTR_COLD;

	DECLARE_INPUT_CHANGED_MEMBER(encoder_step);
	DECLARE_INPUT_CHANGED_MEMBER(key_changed);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;
	void wave_map();
	void monitor_update();
	void postload();

private:
	enum : u8
	{
		IRQ_FDC = 0,
		IRQ_SAMPLER = 2,
		IRQ_KEYSCAN = 3,
		IRQ_DMA = 4,
		IRQ_MIDI_RX = 6
	};

	static inline const attotime SCRATCH_POLL = attotime::from_hz(60);

	enum : u8
	{
		LED_PERFORM, LED_PATCH, LED_PARTIAL, LED_DISK, LED_SYSTEM, LED_SHIFT, LED_HOME, LED_MARK,
		LED_LIST, LED_LIST_RED, LED_NAME, LED_NAME_RED, LED_COMMAND, LED_COMMAND_RED, LED_MIDI, LED_SCSI,
		LED_MODE, LED_F1, LED_F2, LED_F3, LED_JUMP, LED_GRAPHIC, LED_SOUND,
		LED_COUNT
	};

	enum lamps_t : u8
	{
		LAMPS_NONE,
		LAMPS_S760,
		LAMPS_S770
	};

	virtual void rcv_complete() override;
	virtual void tra_callback() override;

	void code_map(address_map &map) ATTR_COLD;
	void data_map(address_map &map) ATTR_COLD;
	void sp700_data_map(address_map &map) ATTR_COLD;
	void common_core(machine_config &config) ATTR_COLD;
	void common(machine_config &config) ATTR_COLD;
	void s770_data_map(address_map &map) ATTR_COLD;
	void dj70_data_map(address_map &map) ATTR_COLD;
	void lcdc_map(address_map &map) ATTR_COLD;
	void t6963c_map(address_map &map) ATTR_COLD;

	u16 ga_r(offs_t offset, u16 mem_mask);
	void ga_w(offs_t offset, u16 data, u16 mem_mask);
	void p1_w(u8 data);
	void set_rate(u32 rate);
	void s770_aes_w(offs_t offset, u16 data, u16 mem_mask);
	void sp700_p1_w(u8 data);
	u8 sp700_p0_r();
	void lamps_update();
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
	TIMER_CALLBACK_MEMBER(hsi_clock);
	TIMER_CALLBACK_MEMBER(scratch_poll);
	TIMER_CALLBACK_MEMBER(scratch_step);
	void dma_byte_w(u8 data);
	u8 dma_byte_r();

	static void floppy_formats(format_registration &fr);

	required_device<i80c196kb_device> m_maincpu;
	required_device<midi_port_device> m_midiout;
	required_device<eeprom_serial_93cxx_device> m_eeprom;
	optional_device<upd72069_device> m_fdc;
	optional_device<floppy_connector> m_floppy;
	required_device<mb89352_device> m_spc;
	optional_device<msx_general_purpose_port_device> m_ext_port;
	optional_device<sed1330_device> m_lcdc;
	optional_device<rf5c16_device> m_vdp;
	optional_device<t6963c_device> m_t6963c;
	optional_device<intel_28f010_device> m_flash;
	required_device<roland_sp_device> m_sp;
	required_device<roland_tvf_device> m_tvf;
	optional_device<roland_meq_device> m_meq;
	optional_device<roland_mix_device> m_mix;
	optional_device<microphone_device> m_adc;
	optional_ioport_array<4> m_keys;
	optional_ioport m_option;
	optional_ioport m_dial;
	u8 m_dial_last;
	u8 m_dial_idle;
	s8 m_dial_steps;
	optional_ioport m_mod;
	optional_ioport m_scratch;
	emu_timer *m_scratch_poll;
	emu_timer *m_scratch_step;
	u16 m_scratch_last;
	s32 m_scratch_pending;
	attotime m_scratch_period;
	bool m_scratch_level;
	optional_ioport m_digital_in;
	optional_ioport_array<6> m_columns;
	optional_device<ssc1000_device> m_keyscan;
	optional_ioport m_velocity;
	optional_ioport m_wave_mem;
	optional_ioport_array<2> m_simms;
	required_region_ptr<u16> m_boot;
	memory_share_creator<u16> m_dram;
	output_finder<LED_COUNT> m_leds;

	u16 m_ga[0x10];
	u8 m_irq_pending;
	bool m_irq_line;
	u8 m_dma_ctrl;
	u32 m_dma_addr;
	u16 m_dma_count;
	bool m_dma_active;
	bool m_dma_in_service;
	bool m_old_ga = false;
	bool m_fdc_int_f002 = false;
	bool m_adele = false;
	lamps_t m_lamps = LAMPS_NONE;
	bool m_fdc_drq;
	bool m_spc_dreq;
	u8 m_panel[0x10];
	bool m_monitor;
	u8 m_key_row;
	u8 m_p1;
	emu_timer *m_fdc_tc;
	emu_timer *m_hsi_clock;
	bool m_hsi_level;
	u16 m_wave_dma_word;
	u32 m_wave_bytes = 0x2000000;
	u32 m_wave_base_block = 0;
	u16 *m_wave;
	u8 m_wave_bank;
	u16 m_aes[0x10];
	u8 m_midi_rx;
	bool m_midi_rx_full;
};

void roland_s770_state::machine_start()
{
	m_fdc_tc = timer_alloc(timer_expired_delegate(FUNC(roland_s770_state::fdc_tc), this));
	m_hsi_clock = timer_alloc(timer_expired_delegate(FUNC(roland_s770_state::hsi_clock), this));
	m_hsi_level = false;
	m_scratch_poll = timer_alloc(timer_expired_delegate(FUNC(roland_s770_state::scratch_poll), this));
	m_scratch_step = timer_alloc(timer_expired_delegate(FUNC(roland_s770_state::scratch_step), this));
	m_scratch_last = 0;
	m_scratch_pending = 0;
	m_scratch_period = attotime::never;
	m_scratch_level = false;
	std::fill(std::begin(m_ga), std::end(m_ga), 0);
	m_irq_pending = 0;
	m_irq_line = false;
	save_item(NAME(m_hsi_level));
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
	save_item(NAME(m_monitor));
	save_item(NAME(m_key_row));
	save_item(NAME(m_p1));
	save_item(NAME(m_wave_dma_word));
	save_item(NAME(m_aes));
	save_item(NAME(m_midi_rx));
	save_item(NAME(m_midi_rx_full));
	save_item(NAME(m_dial_last));
	save_item(NAME(m_dial_idle));
	save_item(NAME(m_dial_steps));
	save_item(NAME(m_scratch_last));
	save_item(NAME(m_scratch_pending));
	save_item(NAME(m_scratch_period));
	save_item(NAME(m_scratch_level));

	m_wave = reinterpret_cast<u16 *>(machine().memory().share_alloc(*this, "wave", 16, m_wave_bytes, ENDIANNESS_LITTLE)->ptr());
	m_wave_bank = 0;
	save_item(NAME(m_wave_bank));
	machine().save().register_postload(save_prepost_delegate(FUNC(roland_s770_state::postload), this));

	set_data_frame(1, 8, PARITY_NONE, STOP_BITS_1);
	set_rcv_rate(31250);
	set_tra_rate(31250);
}

void roland_s770_state::wave_map()
{
	unsigned const blocks = roland_sp_device::WAVE_BLOCKS, words = roland_sp_device::BLOCK_WORDS;
	for (unsigned b = 0; b != blocks; b++)
		m_sp->set_wave_block(b, nullptr);
	auto map = [&](unsigned &at, u16 *mem, unsigned count)
	{
		for (unsigned b = 0; b != count && at != blocks; b++)
			m_sp->set_wave_block(at++, mem + b * words);
	};

	if (m_simms[0] && m_adele)
	{
		unsigned at = 0;
		map(at, m_wave + 0x100000, m_simms[0]->read() * 8);
		map(at, m_wave + 0x900000, m_simms[1]->read() * 8);
		if (!at)
			at = 0x60;
		map(at, m_wave, 0x10);
	}
	else if (m_simms[0])
	{
		// on-board 2 MB first in the RAM, then socket 1, then socket 2
		u16 *const internal = m_wave;
		u16 *const socket1 = m_wave + 0x100000;
		u16 *const socket2 = m_wave + 0x900000;
		unsigned const size1 = m_simms[0]->read() * 8, size2 = m_simms[1]->read() * 8;
		unsigned at = 0;
		switch (m_wave_bank)
		{
		case 1:
			map(at, socket1, size1);
			map(at, socket2, size2);
			break;
		case 2:
			map(at, internal, 0x10);
			map(at, socket1, size1);
			break;
		case 0:
			map(at, socket1, size1);
			at = std::max(at, 0x70U);
			if (at < 0x80)
				map(at, internal + (at - 0x70) * words, 0x80 - at);
			break;
		}
	}
	else
	{
		unsigned at = m_wave_base_block;
		map(at, m_wave, m_wave_mem->read() * 8);
	}
}

void roland_s770_state::machine_reset()
{
	std::fill(std::begin(m_ga), std::end(m_ga), 0);
	for (int i = 0; i != 8; i++)
		m_ga[i] = 0x0780 | (i & 3) << 4;
	std::fill(std::begin(m_panel), std::end(m_panel), 0);
	m_irq_pending = 0;
	m_irq_line = false;
	m_maincpu->set_input_line(i80c196kb_device::EXTINT_LINE, CLEAR_LINE);
	m_dma_ctrl = 0;
	m_dma_addr = 0;
	m_dma_count = 0;
	m_dma_active = false;
	m_dma_in_service = false;
	m_fdc_drq = false;
	m_spc_dreq = false;
	m_key_row = 0;
	m_dial_last = m_dial ? m_dial->read() : 0;
	m_p1 = 0xff;
	m_dial_idle = 0;
	m_dial_steps = 0;
	m_wave_dma_word = 0;
	std::fill(std::begin(m_aes), std::end(m_aes), 0);
	m_midi_rx = 0;
	m_midi_rx_full = false;
	lamps_update();

	m_wave_bank = 0;
	wave_map();
	m_monitor = false;
	monitor_update();

	// stands in for the CPU's PWM output, which the core does not drive
	if (m_flash)
		m_hsi_clock->adjust(attotime::from_hz(16_MHz_XTAL / 256), 0, attotime::from_hz(16_MHz_XTAL / 256));
	if (m_scratch)
	{
		m_scratch_last = m_scratch->read();
		m_scratch_pending = 0;
		m_scratch_level = false;
		m_scratch_step->adjust(attotime::never);
		m_scratch_poll->adjust(SCRATCH_POLL, 0, SCRATCH_POLL);
	}
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
	if (state)
		m_irq_pending |= 1 << source;
	else
		m_irq_pending &= ~(1 << source);
	update_irq();
}

void roland_s770_state::update_irq()
{
	bool const line = m_irq_pending & (m_ga[0xa] >> 8);
	if (line != m_irq_line)
	{
		m_irq_line = line;
		m_maincpu->set_input_line(i80c196kb_device::EXTINT_LINE, line ? ASSERT_LINE : CLEAR_LINE);
	}
}

u16 roland_s770_state::ga_r(offs_t offset, u16 mem_mask)
{
	u16 data = m_ga[offset];
	if (offset == 0xa)
		data = (data & 0xff00) | (is_transmit_register_empty() ? 0x02 : 0x00) | (m_midi_rx_full ? 0x01 : 0x00);
	else if (offset == 0xb)
	{
		data = (data & 0xff00) | m_midi_rx;
		if (ACCESSING_BITS_0_7 && !machine().side_effects_disabled())
		{
			m_midi_rx_full = false;
			irq_w(IRQ_MIDI_RX, 0);
		}
		return data;
	}
	if (offset == 0xe && ACCESSING_BITS_8_15 && !machine().side_effects_disabled())
	{
		if (BIT(m_irq_pending, IRQ_DMA))
		{
			m_irq_pending &= ~(1 << IRQ_DMA);
			update_irq();
		}
	}
	if (!machine().side_effects_disabled() && offset >= 8 && offset != 0xa)
		LOGMASKED(LOG_GA, "ga_r %04x & %04x = %04x (%04x)\n", 0x100 + offset*2, mem_mask, data, m_maincpu->pc());
	return data;
}

void roland_s770_state::ga_w(offs_t offset, u16 data, u16 mem_mask)
{
	if (offset == 0xb && ACCESSING_BITS_0_7)
	{
		if (is_transmit_register_empty())
			transmit_register_setup(data & 0xff);
		else
			LOGMASKED(LOG_GA, "midi tx overrun %02x\n", data & 0xff);
		return;
	}
	if (offset >= 8 && offset != 0xa)
		LOGMASKED(LOG_GA, "ga_w %04x = %04x & %04x (%04x)\n", 0x100 + offset*2, data, mem_mask, m_maincpu->pc());
	COMBINE_DATA(&m_ga[offset]);

	switch (offset)
	{
	case 0xa:
		if (ACCESSING_BITS_8_15)
			update_irq();
		break;
	case 0xe:
		if (ACCESSING_BITS_8_15)
		{
			m_dma_ctrl = data >> 8;
			m_dma_addr = m_ga[0xd] | (m_ga[0xe] & 0x000f) << 16;
			m_dma_count = m_ga[0xc];
			m_dma_active = true;
			if (m_fdc)
				m_fdc->tc_w(false);
			LOGMASKED(LOG_DMA, "dma start ctrl %02x addr %05x count %04x (%04x)\n", m_dma_ctrl, m_dma_addr, m_dma_count, m_maincpu->pc());
			dma_service();
		}
		break;
	}
}

u8 roland_s770_state::dial_r()
{
	if (!m_dial || machine().side_effects_disabled())
		return 0;
	u8 const keys = m_dial->read();
	u8 const pressed = keys & ~m_dial_last;
	m_dial_last = keys;
	if (BIT(pressed, 0))
		m_dial_steps--;
	if (BIT(pressed, 1))
		m_dial_steps++;
	if (m_dial_idle)
	{
		m_dial_idle--;
		return 0;
	}
	if (!m_dial_steps)
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
	if (!(in & 3))
		return 0x00;
	return 0x80 | FIN[in & 3] | (BIT(in, 2) ? 0x40 : 0x00);
}

void roland_s770_state::set_rate(u32 rate)
{
	if (rate != m_sp->clock())
	{
		m_sp->set_unscaled_clock(rate);
		m_tvf->set_unscaled_clock(rate);
		if (m_meq)
			m_meq->set_unscaled_clock(rate);
		if (m_mix)
			m_mix->set_unscaled_clock(rate);
	}
}

void roland_s770_state::s770_aes_w(offs_t offset, u16 data, u16 mem_mask)
{
	COMBINE_DATA(&m_aes[offset & 0xf]);
	if (offset == 3 && !m_mix)
		set_rate(BIT(m_aes[3], 1) ? 48000 : 44100);
}

void roland_s770_state::p1_w(u8 data)
{
	static const u32 RATES[4] = { 44100, 44100, 48000, 32000 };
	u8 const in = m_digital_in ? m_digital_in->read() : 0;
	if (!m_old_ga)
		set_rate(BIT(data, 5) && (in & 3) ? RATES[in & 3] : RATES[data >> 6]);

	m_eeprom->cs_write(BIT(data, 0));
	m_eeprom->di_write(BIT(data, 2));
	m_eeprom->clk_write(BIT(data, 1));
}

void roland_s770_state::sp700_p1_w(u8 data)
{
	static const u8 LEDS[4][4] = {
		{ LED_PARTIAL, LED_DISK, LED_PERFORM, LED_PATCH },
		{ LED_SHIFT, LED_MIDI, LED_LIST_RED, LED_LIST },
		{ LED_HOME, LED_SYSTEM, LED_NAME_RED, LED_NAME },
		{ LED_MARK, LED_SCSI, LED_COMMAND_RED, LED_COMMAND } };

	m_p1 = data;
	u8 const column = ~data & 0x0f;
	if (std::has_single_bit(column))
		for (int row = 0; row != 4; row++)
			m_leds[LEDS[std::countr_zero(column)][row]] = BIT(data, 4 + row);
}

void roland_s770_state::lamps_update()
{
	static const u8 S760[7] = { LED_COMMAND, LED_COMMAND_RED, LED_MODE, LED_SHIFT, LED_F1, LED_F2, LED_F3 };
	static const u8 S770[8] = { LED_JUMP, LED_GRAPHIC, LED_SYSTEM, LED_SOUND, LED_PERFORM, LED_COUNT, LED_MIDI, LED_SCSI };

	if (m_lamps == LAMPS_S760)
	{
		for (int i = 0; i != 7; i++)
			m_leds[S760[i]] = BIT(m_panel[8], i);
		m_leds[LED_SCSI] = !BIT(m_panel[0], 6);
		m_leds[LED_MIDI] = !BIT(m_panel[0], 7);
	}
	else if (m_lamps == LAMPS_S770)
	{
		for (int i = 0; i != 8; i++)
			if (S770[i] != LED_COUNT)
				m_leds[S770[i]] = BIT(m_panel[8], i);
	}
}

u8 roland_s770_state::sp700_p0_r()
{
	u8 data = 0;
	for (int i = 0; i != 4; i++)
		if (!BIT(m_p1, i))
			data |= m_columns[i]->read();
	return data;
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
	if (m_dma_in_service)
		return;
	m_dma_in_service = true;
	while (m_dma_active)
	{
		bool const to_device = !BIT(m_dma_ctrl, 0);
		u8 const group = m_dma_ctrl >> 6;
		bool const wave = m_old_ga ? (m_dma_ctrl & 0xe4) == 0x64 : group == 2 && BIT(m_dma_ctrl, 5);
		bool const fdc = group == 3 || (group == 2 && !BIT(m_dma_ctrl, 5) && m_old_ga);
		bool const scsi_wave = m_old_ga ? (m_dma_ctrl & 0xe3) == 0x40 : group == 2 && !BIT(m_dma_ctrl, 5);
		if (!wave && (fdc ? !m_fdc_drq : !m_spc_dreq))
			break;

		if (scsi_wave)
		{
			bool const high = BIT(m_dma_count, 0);
			if (to_device)
			{
				u8 const data = m_spc->dma_r();
				if (!high)
					m_wave_dma_word = data;
				else
					m_sp->wave_data_w(m_wave_dma_word | data << 8, 0xffff);
			}
			else
			{
				if (!high)
					m_wave_dma_word = m_sp->wave_data_r();
				m_spc->dma_w(m_wave_dma_word >> (high ? 8 : 0));
			}
		}
		else if (wave)
		{
			bool const high = BIT(m_dma_count, 0);
			if (to_device)
			{
				u8 const data = dma_byte_r();
				if (!high)
					m_wave_dma_word = data;
				else
					m_sp->wave_data_w(m_wave_dma_word | data << 8, 0xffff);
			}
			else
			{
				if (!high)
					m_wave_dma_word = m_sp->wave_data_r();
				dma_byte_w(m_wave_dma_word >> (high ? 8 : 0));
			}
		}
		else if (fdc)
		{
			if (to_device)
				m_fdc->dma_w(dma_byte_r());
			else
				dma_byte_w(m_fdc->dma_r());
		}
		else
		{
			if (to_device)
				m_spc->dma_w(dma_byte_r());
			else
				dma_byte_w(m_spc->dma_r());
		}
		m_dma_addr++;
		m_ga[0xd] = m_dma_addr;
		m_ga[0xe] = (m_ga[0xe] & 0xff00) | ((m_dma_addr >> 16) & 0x0f);
		m_ga[0xc] = --m_dma_count;
		if (!m_dma_count)
		{
			m_dma_active = false;
			if (fdc)
				m_fdc_tc->adjust(attotime::zero);
			LOGMASKED(LOG_DMA, "dma end at %05x\n", m_dma_addr);
			irq_w(IRQ_DMA, 1);
		}
	}
	m_dma_in_service = false;
}

INPUT_CHANGED_MEMBER(roland_s770_state::encoder_step)
{
	if (!newval)
		return;
	m_maincpu->set_input_line(i8x9x_device::T2UPDN_LINE, param);
	for (int level : { 1, 0 })
	{
		m_maincpu->set_input_line(i8x9x_device::T2CLK_LINE, level);
		m_maincpu->set_input_line(i8x9x_device::HSI1_LINE, level);
	}
}

TIMER_CALLBACK_MEMBER(roland_s770_state::hsi_clock)
{
	m_hsi_level = !m_hsi_level;
	m_maincpu->set_input_line(i8x9x_device::HSI0_LINE, m_hsi_level);
}

TIMER_CALLBACK_MEMBER(roland_s770_state::scratch_poll)
{
	u16 const pos = m_scratch->read();
	m_scratch_pending += util::sext(pos - m_scratch_last, 12);
	m_scratch_last = pos;
	if (m_scratch_pending)
	{
		m_scratch_period = std::max(SCRATCH_POLL / u32(std::abs(m_scratch_pending)), attotime::from_usec(20));
		if (m_scratch_step->remaining().is_never())
			m_scratch_step->adjust(m_scratch_period);
	}
}

TIMER_CALLBACK_MEMBER(roland_s770_state::scratch_step)
{
	if (!m_scratch_pending)
		return;
	m_maincpu->set_input_line(i8x9x_device::HSI1_LINE, m_scratch_pending < 0);
	m_scratch_pending += m_scratch_pending < 0 ? 1 : -1;
	m_scratch_level = !m_scratch_level;
	m_maincpu->set_input_line(i8x9x_device::HSI0_LINE, m_scratch_level);
	if (m_scratch_pending)
		m_scratch_step->adjust(m_scratch_period);
}

TIMER_CALLBACK_MEMBER(roland_s770_state::fdc_tc)
{
	if (m_fdc)
		m_fdc->tc_w(true);
}

u16 roland_s770_state::bus_r(u16 bank, offs_t offset, u16 mem_mask)
{
	u32 const adr = ((bank & 0x3ff) << 10) + (offset << 1);
	if (BIT(bank, 10))
	{
		if (m_flash && (adr & 0xe0000) == 0x40000)
		{
			u16 data = 0;
			if (ACCESSING_BITS_0_7)
				data |= m_flash->read(adr);
			if (ACCESSING_BITS_8_15)
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
	if (BIT(bank, 10) && m_flash && (adr & 0xe0000) == 0x40000)
	{
		if (ACCESSING_BITS_0_7)
			m_flash->write(adr, data);
		if (ACCESSING_BITS_8_15)
			m_flash->write(adr + 1, data >> 8);
	}
	else if (BIT(bank, 10))
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
	if (offset == 0x203a >> 1 && !machine().side_effects_disabled())
	{
		u8 const active = m_irq_pending & (m_ga[0xa] >> 8);
		if (active)
			return (bus_r(m_ga[4 + (offset >> 13)], offset & 0x1fff, mem_mask) & ~0x1c) | 4 * std::countr_zero(active);
	}
	return bus_r(m_ga[4 + (offset >> 13)], offset & 0x1fff, mem_mask);
}

void roland_s770_state::data_w(offs_t offset, u16 data, u16 mem_mask)
{
	bus_w(m_ga[4 + (offset >> 13)], offset & 0x1fff, data, mem_mask);
}

u16 roland_s770_state::io_r(offs_t offset, u16 mem_mask)
{
	if (!machine().side_effects_disabled())
		LOGMASKED(LOG_IO, "io_r %04x & %04x (%04x)\n", 0xc000 + offset*2, mem_mask, m_maincpu->pc());
	return 0;
}

void roland_s770_state::io_w(offs_t offset, u16 data, u16 mem_mask)
{
	LOGMASKED(LOG_IO, "io_w %04x = %04x & %04x (%04x)\n", 0xc000 + offset*2, data, mem_mask, m_maincpu->pc());
}

u8 roland_s770_state::panel_r(offs_t offset)
{
	switch (offset)
	{
	case 1:
	{
		floppy_image_device *floppy = m_floppy ? m_floppy->get_device() : nullptr;
		bool const disk = floppy && floppy->exists();
		bool const hd = disk && floppy->get_variant() == floppy_image::DSHD;
		u8 const b7 = m_fdc_int_f002 ? (BIT(m_irq_pending, IRQ_FDC) ? 0x80 : 0x00) : digital_in_status();
		return (floppy ? floppy->dskchg_r() : 0) | (hd ? 0x00 : 0x02) | b7;
	}
	case 5:
	{
		if (m_keyscan)
		{
			unsigned const col = (m_panel[4] >> 4) & 7;
			return col < 6 && m_columns[col] ? m_columns[col]->read() : 0xff;
		}
		u8 data = m_keys[m_key_row & 3] ? m_keys[m_key_row & 3]->read() : 0xff;
		if (m_option)
			data = (data & 0x3f) | m_option->read();
		if (!machine().side_effects_disabled())
			m_key_row++;
		return data;
	}
	case 4:
		return m_ext_port ? m_ext_port->read() : 0xff;
	case 9:
		return dial_r();
	default:
		if (!machine().side_effects_disabled())
			LOGMASKED(LOG_IO, "panel_r %04x (%04x)\n", 0xf000 + offset*2, m_maincpu->pc());
		return m_panel[offset];
	}
}

void roland_s770_state::panel_w(offs_t offset, u8 data)
{
	if (offset == 0 && m_simms[0] && !m_adele && ((data >> 4) & 3) != m_wave_bank)
	{
		m_wave_bank = (data >> 4) & 3;
		wave_map();
	}
	if (offset == 3 && m_ext_port)
	{
		if (!BIT(data, 0))
			m_ext_port->pin_6_w(1);
		if (!BIT(data, 1))
			m_ext_port->pin_7_w(1);
	}
	else if (offset == 4)
	{
		if (m_ext_port)
		{
			if (BIT(m_panel[3], 0))
				m_ext_port->pin_6_w(BIT(data, 4));
			if (BIT(m_panel[3], 1))
				m_ext_port->pin_7_w(BIT(data, 5));
			m_ext_port->pin_8_w(BIT(data, 6));
		}
	}
	else if (offset == 5)
		m_key_row = data;
	else if (offset != 3 && offset != 6 && offset != 7 && offset != 8)
		LOGMASKED(LOG_IO, "panel_w %04x = %02x (%04x)\n", 0xf000 + offset*2, data, m_maincpu->pc());
	m_panel[offset] = data;
	if (offset == 0)
	{
		m_monitor = !BIT(data, 2);
		monitor_update();
		if (m_mix)
			set_rate(BIT(data, 5) ? 48000 : 44100);
	}
	if (offset == 0 || offset == 8)
		lamps_update();
}

INPUT_CHANGED_MEMBER(roland_s770_state::key_changed)
{
	if (m_keyscan)
		m_keyscan->key_w(param, newval ? (m_velocity ? m_velocity->read() : 100) : 0);
}

void roland_s770_state::monitor_update()
{
	if (!m_mix)
		return;
	float const gain = m_monitor ? 1.0 : 0.0;
	for (int c = 0; c != 2; c++)
		m_adc->set_route_gain(c, subdevice<speaker_device>("speaker"), c, gain);
}

void roland_s770_state::postload()
{
	wave_map();
	monitor_update();
	lamps_update();
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

void roland_s770_state::s770_data_map(address_map &map)
{
	map(0x0000, 0xbfff).rw(FUNC(roland_s770_state::data_r), FUNC(roland_s770_state::data_w));
	map(0x0100, 0x011f).rw(FUNC(roland_s770_state::ga_r), FUNC(roland_s770_state::ga_w));
	map(0xc000, 0xffff).rw(FUNC(roland_s770_state::io_r), FUNC(roland_s770_state::io_w));
	map(0xc000, 0xc7ff).rw(m_sp, FUNC(roland_sp_device::read), FUNC(roland_sp_device::write));
	map(0xd000, 0xd07f).rw(m_tvf, FUNC(roland_tvf_device::read), FUNC(roland_tvf_device::write));
	map(0xd400, 0xd41f).w(FUNC(roland_s770_state::s770_aes_w));
	map(0xd800, 0xd87f).rw(m_mix, FUNC(roland_mix_device::read), FUNC(roland_mix_device::write));
	map(0xe000, 0xe003).m(m_fdc, FUNC(upd72069_device::map)).umask16(0x00ff);
	map(0xe400, 0xe41f).m(m_spc, FUNC(mb89352_device::map)).umask16(0x00ff);
	map(0xe800, 0xe81f).rw(m_vdp, FUNC(rf5c16_device::read), FUNC(rf5c16_device::write)).umask16(0x00ff);
	map(0xf000, 0xf01f).rw(FUNC(roland_s770_state::panel_r), FUNC(roland_s770_state::panel_w)).umask16(0x00ff);
	map(0xf400, 0xf403).rw(m_t6963c, FUNC(t6963c_device::read), FUNC(t6963c_device::write)).umask16(0x00ff);
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
	map(0xf400, 0xf41f).w(FUNC(roland_s770_state::s770_aes_w));
}

void roland_s770_state::dj70_data_map(address_map &map)
{
	map(0x0000, 0xbfff).rw(FUNC(roland_s770_state::data_r), FUNC(roland_s770_state::data_w));
	map(0x0100, 0x011f).rw(FUNC(roland_s770_state::ga_r), FUNC(roland_s770_state::ga_w));
	map(0xc000, 0xffff).rw(FUNC(roland_s770_state::io_r), FUNC(roland_s770_state::io_w));
	map(0xc000, 0xc7ff).rw(m_sp, FUNC(roland_sp_device::read), FUNC(roland_sp_device::write));
	map(0xd000, 0xd07f).rw(m_tvf, FUNC(roland_tvf_device::read), FUNC(roland_tvf_device::write));
	map(0xd400, 0xd41f).w(FUNC(roland_s770_state::s770_aes_w));
	map(0xd800, 0xd87f).rw(m_mix, FUNC(roland_mix_device::read), FUNC(roland_mix_device::write));
	map(0xe000, 0xe003).m(m_fdc, FUNC(upd72069_device::map)).umask16(0x00ff);
	map(0xe400, 0xe41f).m(m_spc, FUNC(mb89352_device::map)).umask16(0x00ff);
	map(0xe800, 0xefff).rw(m_keyscan, FUNC(ssc1000_device::read), FUNC(ssc1000_device::write)).umask16(0x00ff);
	map(0xf000, 0xf01f).rw(FUNC(roland_s770_state::panel_r), FUNC(roland_s770_state::panel_w)).umask16(0x00ff);
	map(0xf400, 0xf403).rw(m_t6963c, FUNC(t6963c_device::read), FUNC(t6963c_device::write)).umask16(0x00ff);
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
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Command") PORT_CODE(KEYCODE_C)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("S1/Dec") PORT_CODE(KEYCODE_MINUS)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("S2/Inc") PORT_CODE(KEYCODE_EQUALS)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Right") PORT_CODE(KEYCODE_RIGHT)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Value Push (Menu)") PORT_CODE(KEYCODE_ENTER)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Volume Push (Preview)") PORT_CODE(KEYCODE_P)
	PORT_BIT(0xc0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("ROW1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Mode") PORT_CODE(KEYCODE_M)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Exit") PORT_CODE(KEYCODE_BACKSPACE)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Up") PORT_CODE(KEYCODE_UP)
	PORT_BIT(0xf8, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("ROW2")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("F1") PORT_CODE(KEYCODE_F1)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("F3") PORT_CODE(KEYCODE_F3)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Left") PORT_CODE(KEYCODE_LEFT)
	PORT_BIT(0xf8, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("ROW3")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("F2") PORT_CODE(KEYCODE_F2)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Shift") PORT_CODE(KEYCODE_LSHIFT)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Down") PORT_CODE(KEYCODE_DOWN)
	PORT_BIT(0xf8, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("DIAL")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Value -") PORT_CODE(KEYCODE_COMMA)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Value +") PORT_CODE(KEYCODE_STOP)

	PORT_START("OPTION")
	PORT_CONFNAME(0xc0, 0x00, "Option board")
	PORT_CONFSETTING(0xc0, DEF_STR(None))
	PORT_CONFSETTING(0x00, "OP-760-1")
	PORT_CONFSETTING(0x40, "Option ID 01")

	PORT_START("SIMM1")
	PORT_CONFNAME(0x1f, 0, "SIMM socket 1")
	PORT_CONFSETTING(0, DEF_STR(None))
	PORT_CONFSETTING(8, "8 MB")
	PORT_CONFSETTING(16, "16 MB")

	PORT_START("SIMM2")
	PORT_CONFNAME(0x1f, 0, "SIMM socket 2")
	PORT_CONFSETTING(0, DEF_STR(None))
	PORT_CONFSETTING(8, "8 MB")
	PORT_CONFSETTING(16, "16 MB")

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

static INPUT_PORTS_START(s770_panel)
	PORT_START("ROW0")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Right") PORT_CODE(KEYCODE_RIGHT)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Down") PORT_CODE(KEYCODE_DOWN)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Left") PORT_CODE(KEYCODE_LEFT)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("S2") PORT_CODE(KEYCODE_EQUALS)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Up") PORT_CODE(KEYCODE_UP)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("S1") PORT_CODE(KEYCODE_MINUS)
	PORT_BIT(0xc0, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("ROW1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Exit") PORT_CODE(KEYCODE_BACKSPACE)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("F5") PORT_CODE(KEYCODE_F5)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("F4") PORT_CODE(KEYCODE_F4)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("F3") PORT_CODE(KEYCODE_F3)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("F2") PORT_CODE(KEYCODE_F2)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("F1") PORT_CODE(KEYCODE_F1)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Jump") PORT_CODE(KEYCODE_J)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("ROW2")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Index") PORT_CODE(KEYCODE_I)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Mark") PORT_CODE(KEYCODE_M)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Command") PORT_CODE(KEYCODE_C)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Graphic") PORT_CODE(KEYCODE_G)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Sound Play") PORT_CODE(KEYCODE_SPACE)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("System") PORT_CODE(KEYCODE_Y)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Sound") PORT_CODE(KEYCODE_S)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Performance") PORT_CODE(KEYCODE_P)

	PORT_START("DIAL")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Value -") PORT_CODE(KEYCODE_COMMA)
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Value +") PORT_CODE(KEYCODE_STOP)
INPUT_PORTS_END

static INPUT_PORTS_START(s770)
	PORT_INCLUDE(s770_panel)

	PORT_START("WAVE_MEM")
	PORT_CONFNAME(0x3f, 2, "Wave memory")
	PORT_CONFSETTING(2, "2 MB")
	PORT_CONFSETTING(4, "4 MB")
	PORT_CONFSETTING(6, "6 MB")
	PORT_CONFSETTING(8, "8 MB")
	PORT_CONFSETTING(10, "10 MB")
	PORT_CONFSETTING(12, "12 MB")
	PORT_CONFSETTING(14, "14 MB")
	PORT_CONFSETTING(16, "16 MB")
INPUT_PORTS_END

static INPUT_PORTS_START(s750)
	PORT_INCLUDE(s770_panel)

	PORT_START("WAVE_MEM")
	PORT_CONFNAME(0x3f, 2, "Wave memory")
	PORT_CONFSETTING(2, "2 MB")
	PORT_CONFSETTING(4, "4 MB")
	PORT_CONFSETTING(6, "6 MB")
	PORT_CONFSETTING(8, "8 MB")
	PORT_CONFSETTING(10, "10 MB")
	PORT_CONFSETTING(12, "12 MB")
	PORT_CONFSETTING(14, "14 MB")
	PORT_CONFSETTING(16, "16 MB")
	PORT_CONFSETTING(18, "18 MB")
INPUT_PORTS_END

static INPUT_PORTS_START(dj70_panel)
	PORT_START("COL0")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Pad A") PORT_CODE(KEYCODE_1)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Pad B") PORT_CODE(KEYCODE_2)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Pad C") PORT_CODE(KEYCODE_3)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Pad D") PORT_CODE(KEYCODE_4)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Pad E") PORT_CODE(KEYCODE_5)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Pad F") PORT_CODE(KEYCODE_6)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Pad G") PORT_CODE(KEYCODE_7)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Pad H / Scratch") PORT_CODE(KEYCODE_8)

	PORT_START("COL1")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Start/Stop") PORT_CODE(KEYCODE_SPACE)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Sample/Cue") PORT_CODE(KEYCODE_A)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Hold") PORT_CODE(KEYCODE_H)
	PORT_BIT(0xf8, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("COL2")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Octave Up") PORT_CODE(KEYCODE_PGUP)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Octave Down") PORT_CODE(KEYCODE_PGDN)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Jump/Menu") PORT_CODE(KEYCODE_J)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Shift") PORT_CODE(KEYCODE_LSHIFT)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("F1") PORT_CODE(KEYCODE_F1)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("F2") PORT_CODE(KEYCODE_F2)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("F3") PORT_CODE(KEYCODE_F3)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("F4") PORT_CODE(KEYCODE_F4)

	PORT_START("COL3")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("F5") PORT_CODE(KEYCODE_F5)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Exit") PORT_CODE(KEYCODE_BACKSPACE)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Graphic") PORT_CODE(KEYCODE_G)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Cursor Up") PORT_CODE(KEYCODE_UP)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Cursor Down") PORT_CODE(KEYCODE_DOWN)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Cursor Left") PORT_CODE(KEYCODE_LEFT)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Cursor Right") PORT_CODE(KEYCODE_RIGHT)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Execute - (S1)") PORT_CODE(KEYCODE_MINUS)

	PORT_START("COL4")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Execute + (S2)") PORT_CODE(KEYCODE_EQUALS)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Perform") PORT_CODE(KEYCODE_P)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Sound") PORT_CODE(KEYCODE_S)
	PORT_BIT(0x08, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("System") PORT_CODE(KEYCODE_Y)
	PORT_BIT(0x10, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Play") PORT_CODE(KEYCODE_ENTER)
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Index") PORT_CODE(KEYCODE_I)
	PORT_BIT(0x40, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Mark") PORT_CODE(KEYCODE_M)
	PORT_BIT(0x80, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Command") PORT_CODE(KEYCODE_C)

	PORT_START("COL5")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Disk") PORT_CODE(KEYCODE_D)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("RPS") PORT_CODE(KEYCODE_R)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Rec") PORT_CODE(KEYCODE_E)
	PORT_BIT(0xf8, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("DIAL")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Value -") PORT_CODE(KEYCODE_COMMA)
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Value +") PORT_CODE(KEYCODE_STOP)

	PORT_START("KEYS0")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("C3") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 0)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("C#3") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 1)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("D3") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 2)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("D#3") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 3)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("E3") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 4)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F3") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 5)
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F#3") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 6)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("G3") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 7)

	PORT_START("KEYS1")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("G#3") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 8)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("A3") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 9)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("A#3") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 10)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("B3") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 11)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("C4") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 12)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("C#4") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 13)
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("D4") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 14)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("D#4") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 15)

	PORT_START("KEYS2")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("E4") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 16)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F4") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 17)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F#4") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 18)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("G4") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 19)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("G#4") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 20)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("A4") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 21)
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("A#4") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 22)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("B4") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 23)

	PORT_START("KEYS3")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("C5") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 24)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("C#5") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 25)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("D5") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 26)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("D#5") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 27)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("E5") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 28)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F5") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 29)
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F#5") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 30)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("G5") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 31)

	PORT_START("KEYS4")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("G#5") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 32)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("A5") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 33)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("A#5") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 34)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("B5") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 35)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("C6") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::key_changed), 36)

	PORT_START("VELOCITY")
	PORT_CONFNAME(0x7f, 100, "Key velocity")
	PORT_CONFSETTING(32, "32")
	PORT_CONFSETTING(64, "64")
	PORT_CONFSETTING(100, "100")
	PORT_CONFSETTING(127, "127")
INPUT_PORTS_END

static INPUT_PORTS_START(dj70_controls)
	PORT_START("BEND")
	PORT_BIT(0x3ff, 0x200, IPT_PADDLE) PORT_NAME("Pitch Bender") PORT_MINMAX(0x000, 0x3ff) PORT_SENSITIVITY(50) PORT_KEYDELTA(32) PORT_CENTERDELTA(64) PORT_CODE_DEC(KEYCODE_Q) PORT_CODE_INC(KEYCODE_W)

	PORT_START("MOD")
	PORT_BIT(0x3ff, 0x000, IPT_PEDAL) PORT_NAME("Modulation") PORT_MINMAX(0x000, 0x3ff) PORT_SENSITIVITY(50) PORT_KEYDELTA(32) PORT_CODE_INC(KEYCODE_O)

	PORT_START("SCRATCH")
	PORT_BIT(0xfff, 0x000, IPT_DIAL) PORT_NAME("Scratch Wheel") PORT_SENSITIVITY(50) PORT_KEYDELTA(10) PORT_CODE_DEC(KEYCODE_Z) PORT_CODE_INC(KEYCODE_X)

	PORT_START("FOOTSW")
	PORT_BIT(0x20, IP_ACTIVE_LOW, IPT_OTHER) PORT_NAME("Foot Switch") PORT_CODE(KEYCODE_F)
	PORT_BIT(0xdf, IP_ACTIVE_HIGH, IPT_UNUSED)
INPUT_PORTS_END

static INPUT_PORTS_START(dj70)
	PORT_INCLUDE(dj70_panel)
	PORT_INCLUDE(dj70_controls)

	PORT_START("WAVE_MEM")
	PORT_CONFNAME(0x3f, 2, "Wave memory")
	PORT_CONFSETTING(2, "2 MB")
	PORT_CONFSETTING(4, "4 MB")
INPUT_PORTS_END

static INPUT_PORTS_START(dj70mk2)
	PORT_INCLUDE(dj70_panel)
	PORT_INCLUDE(dj70_controls)

	PORT_START("SIMM1")
	PORT_CONFNAME(0x1f, 0, "SIMM socket 1")
	PORT_CONFSETTING(0, DEF_STR(None))
	PORT_CONFSETTING(8, "8 MB")
	PORT_CONFSETTING(16, "16 MB")

	PORT_START("SIMM2")
	PORT_CONFNAME(0x1f, 0, "SIMM socket 2")
	PORT_CONFSETTING(0, DEF_STR(None))
	PORT_CONFSETTING(8, "8 MB")
	PORT_CONFSETTING(16, "16 MB")
INPUT_PORTS_END

static INPUT_PORTS_START(sp700)
	PORT_START("COL0")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Perform") PORT_CODE(KEYCODE_Q)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Patch") PORT_CODE(KEYCODE_W)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Partial") PORT_CODE(KEYCODE_E)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Disk") PORT_CODE(KEYCODE_R)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("System") PORT_CODE(KEYCODE_T)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Jump") PORT_CODE(KEYCODE_J)
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("F1") PORT_CODE(KEYCODE_F1)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("F2") PORT_CODE(KEYCODE_F2)

	PORT_START("COL1")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("List") PORT_CODE(KEYCODE_L)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Next") PORT_CODE(KEYCODE_PGDN)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("S2/Inc") PORT_CODE(KEYCODE_EQUALS)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Right") PORT_CODE(KEYCODE_RIGHT)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Shift") PORT_CODE(KEYCODE_LSHIFT)
	PORT_BIT(0xe0, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("COL2")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Name") PORT_CODE(KEYCODE_N)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Up") PORT_CODE(KEYCODE_UP)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Down") PORT_CODE(KEYCODE_DOWN)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Home") PORT_CODE(KEYCODE_HOME)
	PORT_BIT(0x60, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("F3") PORT_CODE(KEYCODE_F3)

	PORT_START("COL3")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Command") PORT_CODE(KEYCODE_C)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Previous") PORT_CODE(KEYCODE_PGUP)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("S1/Dec") PORT_CODE(KEYCODE_MINUS)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Left") PORT_CODE(KEYCODE_LEFT)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Mark") PORT_CODE(KEYCODE_M)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Exit") PORT_CODE(KEYCODE_BACKSPACE)
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("F5") PORT_CODE(KEYCODE_F5)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("F4") PORT_CODE(KEYCODE_F4)

	PORT_START("ENCODER")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Value -") PORT_CODE(KEYCODE_COMMA) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::encoder_step), 1)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Value +") PORT_CODE(KEYCODE_STOP) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(roland_s770_state::encoder_step), 0)

	PORT_START("WAVE_MEM")
	PORT_CONFNAME(0x3f, 8, "Wave memory")
	PORT_CONFSETTING(8, "8 MB")
	PORT_CONFSETTING(16, "16 MB")
	PORT_CONFSETTING(24, "24 MB")
	PORT_CONFSETTING(32, "32 MB")
INPUT_PORTS_END

void roland_s770_state::floppy_formats(format_registration &fr)
{
	fr.add_pc_formats();
}

static void s770_floppies(device_slot_interface &device)
{
	device.option_add("35hd", FLOPPY_35_HD);
}

static void s7_ext_ctrl_devices(device_slot_interface &device)
{
	device.option_add("mouse", MSX_MOUSE);
	device.option_add("rc100", ROLAND_RC100);
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

void roland_s770_state::common_core(machine_config &config)
{
	I80C196KB(config, m_maincpu, 16_MHz_XTAL);
	m_maincpu->set_timer2_external(true);
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
	m_spc->out_dreq_callback().set(FUNC(roland_s770_state::spc_dreq_w));
	scsi.set_external_device(7, m_spc);

	MIDI_PORT(config, "mdin", midiin_slot, "midiin").rxd_handler().set(FUNC(roland_s770_state::rx_w));
	MIDI_PORT(config, m_midiout, midiout_slot, "midiout");

	SPEAKER(config, "speaker", 2).front();

	ROLAND_SP(config, m_sp, 44100);
	m_sp->int_callback().set([this](int state) { irq_w(IRQ_SAMPLER, state); });
	ROLAND_TVF(config, m_tvf, 44100);
	// sampler voices 4-15 and 20-31 play, into TVF voices 0-23; 0-3 and 16-19 are never keyed
	for (int n = 0; n != roland_sp_device::VOICES; n++)
		m_sp->add_route(n, m_tvf, 1.0, n < 16 ? (n + 28) & 31 : n - 8);
}

void roland_s770_state::common(machine_config &config)
{
	common_core(config);
	m_eeprom->default_data(s760_eeprom, sizeof(s760_eeprom));
	ROLAND_MEQ(config, m_meq, 44100);
	// TVF WO0-17 drive MEQ WB5-22
	for (int n = 0; n != roland_meq_device::VOICES; n++)
		m_tvf->add_route(n, m_meq, 0.25, n);
}

void roland_s770_state::s760(machine_config &config)
{
	common(config);
	m_lamps = LAMPS_S760;
	m_wave_bytes = 34 << 20;
	m_maincpu->set_addrmap(AS_PROGRAM, &roland_s770_state::data_map);

	// SDO1's latches in step order: DIGITAL OUT 1 L, STEREO OUT 1 R, DIGITAL OUT 1 R, STEREO OUT 1 L;
	// SDO2's likewise for the second pair
	m_meq->port3_callback().set(m_sp, FUNC(roland_sp_device::record_w));
	m_meq->set_synchronous(true);
	SPEAKER(config, "out2", 2).front();
	m_meq->add_route(roland_meq_device::output(0, 3), "speaker", 1.0, 0);
	m_meq->add_route(roland_meq_device::output(0, 1), "speaker", 1.0, 1);
	m_meq->add_route(roland_meq_device::output(1, 3), "out2", 1.0, 0);
	m_meq->add_route(roland_meq_device::output(1, 1), "out2", 1.0, 1);
	SPEAKER(config, "dout1", 2).front();
	SPEAKER(config, "dout2", 2).front();
	m_meq->add_route(roland_meq_device::output(0, 0), "dout1", 1.0, 0);
	m_meq->add_route(roland_meq_device::output(0, 2), "dout1", 1.0, 1);
	m_meq->add_route(roland_meq_device::output(1, 0), "dout2", 1.0, 0);
	m_meq->add_route(roland_meq_device::output(1, 2), "dout2", 1.0, 1);

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

	MSX_GENERAL_PURPOSE_PORT(config, m_ext_port, s7_ext_ctrl_devices, "mouse");

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
	m_vdp->set_middle_level(0);
}

static const u16 sp700_meq_latches[roland_meq_device::LATCHES] = { 0x01d, 0x03d, 0x05d, 0x07d, 0x09d, 0x0bd, 0x0dd, 0x0ff };

void roland_s770_state::sp700(machine_config &config)
{
	common(config);
	m_maincpu->set_addrmap(AS_PROGRAM, &roland_s770_state::sp700_data_map);

	m_maincpu->out_p1_cb().set(FUNC(roland_s770_state::sp700_p1_w));
	m_maincpu->in_p0_cb().set(FUNC(roland_s770_state::sp700_p0_r));

	m_meq->set_latch_steps(1, sp700_meq_latches);
	SPEAKER(config, "out_b", 2).front();
	SPEAKER(config, "out_c", 2).front();
	SPEAKER(config, "out_d", 2).front();
	m_meq->add_route(roland_meq_device::output(1, 6), "speaker", 1.0, 0);
	m_meq->add_route(roland_meq_device::output(1, 7), "speaker", 1.0, 1);
	m_meq->add_route(roland_meq_device::output(1, 0), "out_b", 1.0, 0);
	m_meq->add_route(roland_meq_device::output(1, 1), "out_b", 1.0, 1);
	m_meq->add_route(roland_meq_device::output(1, 2), "out_c", 1.0, 0);
	m_meq->add_route(roland_meq_device::output(1, 3), "out_c", 1.0, 1);
	m_meq->add_route(roland_meq_device::output(1, 4), "out_d", 1.0, 0);
	m_meq->add_route(roland_meq_device::output(1, 5), "out_d", 1.0, 1);

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

// as the system's EEPROM initialise leaves it, the controller set to the mouse
static const u16 s770_eeprom[0x40] = {
	0x0107, 0x0801, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x01ff, 0x0000, 0x0000, 0x0000, 0xff00,
	0x0000, 0x0000, 0x0000, 0x3c7f, 0x0000, 0x0000, 0x0001, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
	0x4f52, 0x616c, 0x646e, 0x2020, 0x2020, 0x2020, 0x2020, 0x2020,
	0x2020, 0x2020, 0x2020, 0x2020, 0x2020, 0x2020, 0x2020, 0x2020,
};

void roland_s770_state::s770(machine_config &config)
{
	common_core(config);
	m_old_ga = true;
	m_lamps = LAMPS_S770;
	m_maincpu->set_clock(12_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &roland_s770_state::s770_data_map);
	m_wave_bytes = 18 << 20;
	m_wave_base_block = 0x70;
	m_eeprom->default_data(s770_eeprom, sizeof(s770_eeprom));

	ROLAND_MIX(config, m_mix, 44100);
	// TVF WO0-19 drive MIX DI0-19
	for (int n = 0; n != roland_mix_device::VOICES; n++)
		m_tvf->add_route(n, m_mix, 0.125, n);
	m_mix->add_route(roland_mix_device::OUTPUT_LEFT, "speaker", 2.0, 0);
	m_mix->add_route(roland_mix_device::OUTPUT_RIGHT, "speaker", 2.0, 1);
	m_sp->set_record_inputs();
	auto &adc(MICROPHONE(config, "adc", 2).front());
	// the A/D's left word reaches the recorder of voice 20, the right voice 4's
	adc.add_route(0, m_sp, 1.0, 1);
	adc.add_route(1, m_sp, 1.0, 0);
	adc.add_route(0, "speaker", 0.0, 0);
	adc.add_route(1, "speaker", 0.0, 1);
	auto &digital_in(MICROPHONE(config, "digital_in", 2).front());
	digital_in.add_route(0, m_mix, 1.0, roland_mix_device::INPUT_SERIAL);
	digital_in.add_route(1, m_mix, 1.0, roland_mix_device::INPUT_SERIAL + 1);
	m_mix->serial_callback().set(m_sp, FUNC(roland_sp_device::record_w));
	SPEAKER(config, "dout", 2).front();
	m_mix->add_route(roland_mix_device::OUTPUT_SERIAL, "dout", 1.0, 0);
	m_mix->add_route(roland_mix_device::OUTPUT_SERIAL + 1, "dout", 1.0, 1);

	SPEAKER(config, "indiv12", 2).front();
	SPEAKER(config, "indiv34", 2).front();
	SPEAKER(config, "indiv56", 2).front();
	for (int n = 0; n != 6; n++)
		m_mix->add_route(roland_mix_device::OUTPUT_INDIVIDUAL + n, n < 2 ? "indiv12" : n < 4 ? "indiv34" : "indiv56", 2.0 * 22 / 39, n & 1);

	UPD72069(config, m_fdc, 8_MHz_XTAL);
	m_fdc->set_ready_line_connected(false);
	m_fdc->intrq_wr_callback().set([this](int state) { irq_w(IRQ_FDC, state); });
	m_fdc->drq_wr_callback().set(FUNC(roland_s770_state::fdc_drq_w));
	FLOPPY_CONNECTOR(config, m_floppy, s770_floppies, "35hd", roland_s770_state::floppy_formats).enable_sound(true);

	MSX_GENERAL_PURPOSE_PORT(config, m_ext_port, s7_ext_ctrl_devices, "mouse");

	PALETTE(config, "palette", palette_device::MONOCHROME_INVERTED);

	T6963C(config, m_t6963c);
	m_t6963c->set_addrmap(0, &roland_s770_state::t6963c_map);

	screen_device &lcd(SCREEN(config, "lcd").set_lcd());
	lcd.set_refresh_hz(60);
	lcd.set_size(240, 64);
	lcd.set_visarea_full();
	lcd.set_screen_update(m_t6963c, FUNC(t6963c_device::screen_update));
	lcd.set_palette("palette");

	screen_device &crt(SCREEN(config, "crt"));
	crt.set_raw(14.318181_MHz_XTAL, 912, 0, 640, 262, 0, 200);
	crt.set_screen_update(m_vdp, FUNC(rf5c16_device::screen_update));

	RF5C16(config, m_vdp, 14.318181_MHz_XTAL);
	m_vdp->set_screen("crt");
	m_vdp->set_middle_level(0);
}

void roland_s770_state::s750turbo(machine_config &config)
{
	s770(config);
	m_maincpu->set_clock(16_MHz_XTAL);
}

void roland_s770_state::dj70(machine_config &config)
{
	s770(config);
	m_lamps = LAMPS_NONE;
	m_maincpu->set_addrmap(AS_PROGRAM, &roland_s770_state::dj70_data_map);
	m_wave_bytes = 4 << 20;
	m_wave_base_block = 0x60;
	m_fdc_int_f002 = true;
	config.device_remove("vdp");
	config.device_remove("crt");
	config.device_remove("extport");
	config.device_remove("digital_in");
	config.device_remove("dout");
	config.device_remove("indiv12");
	config.device_remove("indiv34");
	config.device_remove("indiv56");

	// individual 1 and 2 are the headphones' cue: averaged, the stereo stage, then x4.9 into both sides
	m_mix->reset_routes();
	m_mix->add_route(roland_mix_device::OUTPUT_LEFT, "speaker", 2.0, 0);
	m_mix->add_route(roland_mix_device::OUTPUT_RIGHT, "speaker", 2.0, 1);
	SPEAKER(config, "phones", 2).front();
	for (int n = 0; n != 2; n++)
		for (int side = 0; side != 2; side++)
			m_mix->add_route(roland_mix_device::OUTPUT_INDIVIDUAL + n, "phones", 2.0 * 0.5 * 4.9, side);

	m_maincpu->ach1_cb().set_ioport("BEND");
	m_maincpu->ach2_cb().set([this] () -> u16 { return 0x3ff - m_mod->read(); });
	m_maincpu->in_p0_cb().set_ioport("FOOTSW");

	SSC1000(config, m_keyscan, 8_MHz_XTAL);
	m_keyscan->int_callback().set([this](int state) { irq_w(IRQ_KEYSCAN, state); });
}

void roland_s770_state::dj70mk2(machine_config &config)
{
	dj70(config);
	m_wave_bytes = 34 << 20;
	m_adele = true;
}

ROM_START(s770)
	ROM_REGION16_LE(0x4000, "boot", 0)
	ROM_SYSTEM_BIOS(0, "v112", "Monitor ROM 1.12")
	ROMX_LOAD("s770_1.12.bin", 0x0000, 0x4000, CRC(ca5f8352) SHA1(2a9ac99b862cef9f005bbadbcf2ce158553fb3fc), ROM_BIOS(0))
	ROM_SYSTEM_BIOS(1, "v100", "Monitor ROM 1.00")
	ROMX_LOAD("s770_1.00.bin", 0x0000, 0x4000, CRC(d1ed0a5b) SHA1(8980b5041dfe1be43074bab2cb50333d9cc5a830), ROM_BIOS(1))
ROM_END

// an MB27C128 read as a 27C256: two identical halves
ROM_START(s750)
	ROM_REGION16_LE(0x8000, "boot", 0)
	ROM_LOAD("s750_1.02.bin", 0x0000, 0x8000, CRC(29d9ed2d) SHA1(867ada312386ec8d01c262c35ba6ef189dff2223))
ROM_END

ROM_START(s750turbo)
	ROM_REGION16_LE(0x4000, "boot", 0)
	ROM_LOAD("s750turbo_1.50.bin", 0x0000, 0x4000, CRC(4f4dcaec) SHA1(db88637aeb09eb0e20928ae16beaed29a4713cf0))
ROM_END

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
ROM_END

// the EPROM in the upper half of the 1 MB boot space, its boot block at E0000
ROM_START(dj70)
	ROM_REGION16_LE(0x100000, "boot", ROMREGION_ERASEFF)
	ROM_LOAD("dj70_ic22_1.01.bin", 0x80000, 0x80000, CRC(af37ef1d) SHA1(241bb7bb23bee61d4d577280241a0e67f2091bbd))
ROM_END

ROM_START(dj70mk2)
	ROM_REGION16_LE(0x100000, "boot", ROMREGION_ERASEFF)
	ROM_LOAD("dj70mk2_ic22_1.02.bin", 0x80000, 0x80000, CRC(3ffb0003) SHA1(f69e0076728cf980916e50e0b7a9488346201f1f))
ROM_END

} // anonymous namespace

SYST(1990, s770, 0, 0, s770, s770, roland_s770_state, empty_init, "Roland", "S-770 Digital Sampler", MACHINE_SUPPORTS_SAVE)
SYST(1991, s750, s770, 0, s770, s750, roland_s770_state, empty_init, "Roland", "S-750 Digital Sampler", MACHINE_SUPPORTS_SAVE)
SYST(1993, s750turbo, s770, 0, s750turbo, s750, roland_s770_state, empty_init, "Roland", "S-750 Digital Sampler (Turbo Kit)", MACHINE_SUPPORTS_SAVE)
SYST(1993, s760, 0, 0, s760, s760, roland_s770_state, empty_init, "Roland", "S-760 Digital Sampler", MACHINE_SUPPORTS_SAVE)
SYST(1993, sp700, 0, 0, sp700, sp700, roland_s770_state, empty_init, "Roland", "SP-700 16-bit Sample Player", MACHINE_SUPPORTS_SAVE)
SYST(1992, dj70, 0, 0, dj70, dj70, roland_s770_state, empty_init, "Roland", "DJ-70 Sampling Workstation", 0)
SYST(1996, dj70mk2, dj70, 0, dj70mk2, dj70mk2, roland_s770_state, empty_init, "Roland", "DJ-70mkII Sampling Workstation", 0)
