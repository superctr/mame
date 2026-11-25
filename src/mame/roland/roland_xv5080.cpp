// license:BSD-3-Clause
// copyright-holders:superctr
/****************************************************************************

    Roland XV-3080 and XV-5080 synthesizer modules

    XV-3080 main board:
    - IC3 HD6437042F33 SH-2 with its own ROM, 8.25 MHz crystal, x4 PLL
    - IC4 TC160G22AF-1253 gate array: the panel, the LEDs, the interrupts
    - IC1 LH28F160S5T program flash, 2 MB
    - IC6/IC84 uPD431000 SRAM, 256 KB, battery backed
    - IC5 VG2618165 DRAM, 2 MB
    - IC92/IC93 RA09-002 (TC203C180AF) XP PCM chips, 24.576 MHz
    - IC95/IC96 LC324260 effect RAM
    - IC26/IC29 uPD23C128040 wave mask ROMs, 16 MB each
    - IC71/IC74/IC79 AK4324 DACs
    - L4052B1J000 LCD, 40 x 2 characters

    XV-5080 main board:
    - IC3 HD6437042AA13F SH-2 with its own ROM, 8.25 MHz crystal, x4 PLL
    - IC4 TC160G22AF-1253 gate array
    - IC1 LH28F160S5T program flash, 2 MB
    - IC6/IC84 uPD431000 SRAM, 256 KB; IC117/IC118 MSM5117805 DRAM, 4 MB
    - IC12/IC13 TC223C660CF-503 (RA08-503) XV tone generators, 384 fs
    - IC18/IC19 VG2618165 effect RAM
    - IC26/IC29 uPD23C128040 wave mask ROMs, 16 MB each
    - IC51 SED1335F0B LCD controller, IC52 TC55257 32 KB, RCM6048T-A LCD
    - IC53 SYM53CF92 SCSI controller, 25 MHz
    - IC107 M38881M2 sub CPU
    - IC71/IC74/IC79/IC82 AK4324 DACs
    - two 72-pin SIMM slots, 16 to 64 MB each; SmartMedia slot

    TODO:
    - the XV-5080's sub CPU, R-BUS, S/PDIF output and external word clocks
    - the XV-3080's link to the expansion boards' ID PROMs on port E

****************************************************************************/

#include "emu.h"

#include "wavecard.h"

#include "bus/midi/midiinport.h"
#include "bus/midi/midioutport.h"
#include "bus/nscsi/devices.h"
#include "cpu/sh/sh7042.h"
#include "machine/ncr53c90.h"
#include "machine/nscsi_bus.h"
#include "machine/nvram.h"
#include "machine/smartmed.h"
#include "sound/roland_xp.h"
#include "sound/roland_xv.h"
#include "video/hd44780.h"
#include "video/sed1330.h"

#include "emupal.h"
#include "screen.h"
#include "softlist_dev.h"
#include "speaker.h"

#include <algorithm>
#include <memory>

#include "roland_xv3080.lh"
#include "roland_xv5080.lh"

#define LOG_GA      (1U << 1)
#define LOG_PORT    (1U << 2)

#define VERBOSE (0)
#include "logmacro.h"


namespace {

class xv_state : public driver_device
{
public:
	void init_waverom() ATTR_COLD;

protected:
	static constexpr int GA_SOURCE_KEY = 0;
	static constexpr int GA_SOURCE_ENCODER = 1;
	static constexpr int GA_SOURCE_SENSING = 7;
	static constexpr int GA_SOURCE_TICK = 9;

	static constexpr int GA_TICK_HZ = 1000;
	static constexpr int GA_SENSING_HZ = 10;
	static constexpr int GA_SCAN_HZ = 120;

	static constexpr int DISPLAY_REQUEST_US = 40;
	static constexpr int LCDC_REQUEST_NS = 1000;

	static constexpr u8 GA_LEDS = 0x10;
	static constexpr u8 GA_LCD_COMMAND = 0x38;
	static constexpr u8 GA_LCD_DATA = 0x39;
	static constexpr u8 GA_CLOCK = 0x38;
	static constexpr u8 GA_CARD = 0x39;
	static constexpr u8 GA_DIRECT_ROW = 0x3a;
	static constexpr u8 GA_SWITCHES = 0x3b;
	static constexpr u8 GA_ENABLE_LOW = 0x3e;
	static constexpr u8 GA_ENABLE_HIGH = 0x3f;
	static constexpr u8 GA_SOURCE = 0x40;
	static constexpr u8 GA_ENCODER = 0x42;
	static constexpr u8 GA_KEY = 0x43;

	xv_state(const machine_config &mconfig, device_type type, const char *tag, int display_channel)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_exp(*this, "exp%u", 0U)
		, m_keys(*this, "KEY%u", 0U)
		, m_dial(*this, "VALUE")
		, m_panel(*this, "PANEL")
		, m_leds(*this, "led%u", 0U)
		, m_display_channel(display_channel)
	{
	}

	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

	void common(machine_config &config) ATTR_COLD;
	void common_map(address_map &map) ATTR_COLD;

	u8 ga_r(offs_t offset);
	virtual void ga_w(offs_t offset, u8 data);
	virtual u8 ga_switches_r(u8 reg) = 0;
	void ga_raise(int source);
	void ga_lower(int source);
	void ga_update();
	bool ga_enabled(int source) const;
	void key_enqueue(u8 code);
	void key_deliver();
	TIMER_CALLBACK_MEMBER(ga_scan);
	TIMER_CALLBACK_MEMBER(ga_tick);
	TIMER_CALLBACK_MEMBER(display_request);
	void porte_w(offs_t offset, u16 data, u16 ddr);
	void lcd_palette(palette_device &palette) const ATTR_COLD;

	required_device<sh7042_device> m_maincpu;
	required_device_array<roland_srjv80_slot_device, 4> m_exp;
	required_ioport_array<8> m_keys;
	required_ioport m_dial;
	required_ioport m_panel;
	output_finder<32> m_leds;

	const int m_display_channel;
	emu_timer *m_ga_scan_timer = nullptr;
	emu_timer *m_ga_tick_timer = nullptr;
	emu_timer *m_display_timer = nullptr;
	u8 m_ga_regs[0x100]{};
	u16 m_ga_requests = 0;
	u8 m_ga_key = 0;
	u8 m_key_fifo[64]{};
	u8 m_key_head = 0;
	u8 m_key_tail = 0;
	bool m_key_busy = false;
	u8 m_key_state[8]{};
	u8 m_dial_position = 0;
	s32 m_encoder = 0;
};

class xv3080_state : public xv_state
{
public:
	xv3080_state(const machine_config &mconfig, device_type type, const char *tag)
		: xv_state(mconfig, type, tag, 0)
		, m_xp(*this, "xp%u", 0U)
		, m_srx(*this, "srx%u", 0U)
		, m_lcd(*this, "lcd")
	{
	}

	void xv3080(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;
	virtual void ga_w(offs_t offset, u8 data) override;
	virtual u8 ga_switches_r(u8 reg) override;

private:
	void xv3080_map(address_map &map) ATTR_COLD;
	void xp_rom_map(address_map &map) ATTR_COLD;

	TIMER_CALLBACK_MEMBER(ga_sensing);
	void pump_slave(int state);
	u32 master_link_r(offs_t strobe);
	u32 slave_link_r(offs_t strobe);

	required_device_array<roland_xp_device, 2> m_xp;
	required_device_array<roland_srx_slot_device, 2> m_srx;
	required_device<hd44780_device> m_lcd;

	emu_timer *m_ga_sensing_timer = nullptr;
};

class xv5080_state : public xv_state
{
public:
	xv5080_state(const machine_config &mconfig, device_type type, const char *tag)
		: xv_state(mconfig, type, tag, 1)
		, m_xv(*this, "xv%u", 0U)
		, m_srx(*this, "srx%u", 0U)
		, m_lcdc(*this, "lcdc")
		, m_scsi(*this, "scsic")
		, m_card(*this, "card")
		, m_simm(*this, "SIMM")
	{
	}

	void xv5080(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;
	virtual void ga_w(offs_t offset, u8 data) override;
	virtual u8 ga_switches_r(u8 reg) override;

private:
	static constexpr XTAL XV_CLOCK_44K = 33.8688_MHz_XTAL / 2;
	static constexpr XTAL XV_CLOCK_48K = 36.864_MHz_XTAL / 2;

	static constexpr offs_t SIMM_BASE = 0x0c000000;
	static constexpr offs_t SIMM_WINDOW = 0x02000000;

	void xv5080_map(address_map &map) ATTR_COLD;
	void xv_wave_map(address_map &map) ATTR_COLD;
	void lcdc_map(address_map &map) ATTR_COLD;

	void simm_install();
	u8 scsi_dma_r();
	void scsi_dma_w(u8 data);
	void scsi_drq_w(int state);
	void scsi_dack();
	u8 card_r();
	void card_w(u8 data);
	u16 porte_r();
	void lcdc_data_w(u8 data);
	void lcdc_command_w(u8 data);
	void lcdc_taken();

	required_device_array<roland_xv_device, 2> m_xv;
	required_device_array<roland_srx_slot_device, 4> m_srx;
	required_device<sed1330_device> m_lcdc;
	required_device<ncr53cf94_device> m_scsi;
	required_device<smartmedia_image_device> m_card;
	required_ioport m_simm;

	std::unique_ptr<u16 []> m_simm_ram[2];
	bool m_scsi_drq = false;
};


void xv_state::init_waverom()
{
	roland_srx_slot_device::descramble_rom(memregion("waverom")->base(), memregion("waverom")->bytes());
}

void xv_state::machine_start()
{
	m_ga_scan_timer = timer_alloc(FUNC(xv_state::ga_scan), this);
	m_ga_tick_timer = timer_alloc(FUNC(xv_state::ga_tick), this);
	m_display_timer = timer_alloc(FUNC(xv_state::display_request), this);

	save_item(NAME(m_ga_regs));
	save_item(NAME(m_ga_requests));
	save_item(NAME(m_ga_key));
	save_item(NAME(m_key_fifo));
	save_item(NAME(m_key_head));
	save_item(NAME(m_key_tail));
	save_item(NAME(m_key_busy));
	save_item(NAME(m_key_state));
	save_item(NAME(m_dial_position));
	save_item(NAME(m_encoder));
}

void xv_state::machine_reset()
{
	std::fill(std::begin(m_ga_regs), std::end(m_ga_regs), 0);
	std::fill(std::begin(m_key_state), std::end(m_key_state), 0);
	m_ga_requests = 0;
	m_ga_key = 0;
	m_key_head = m_key_tail = 0;
	m_key_busy = false;
	m_dial_position = m_dial->read();
	m_encoder = 0;

	m_ga_scan_timer->adjust(attotime::from_hz(GA_SCAN_HZ), 0, attotime::from_hz(GA_SCAN_HZ));
	m_ga_tick_timer->adjust(attotime::from_hz(GA_TICK_HZ), 0, attotime::from_hz(GA_TICK_HZ));
	m_display_timer->adjust(attotime::from_usec(DISPLAY_REQUEST_US), 0, attotime::from_usec(DISPLAY_REQUEST_US));
}

void xv3080_state::machine_start()
{
	xv_state::machine_start();
	m_ga_sensing_timer = timer_alloc(FUNC(xv3080_state::ga_sensing), this);
}

void xv3080_state::machine_reset()
{
	xv_state::machine_reset();
	m_ga_sensing_timer->adjust(attotime::from_hz(GA_SENSING_HZ), 0, attotime::from_hz(GA_SENSING_HZ));
}

void xv5080_state::machine_start()
{
	xv_state::machine_start();

	for (int slot = 0; slot < 2; slot++)
	{
		m_simm_ram[slot] = make_unique_clear<u16 []>(SIMM_WINDOW);
		save_pointer(NAME(m_simm_ram[slot]), SIMM_WINDOW, slot);
	}
	save_item(NAME(m_scsi_drq));
}

void xv5080_state::machine_reset()
{
	xv_state::machine_reset();
	simm_install();
	for (auto &xv : m_xv)
		xv->set_unscaled_clock(XV_CLOCK_44K);
}


void xv5080_state::simm_install()
{
	const ioport_value sizes = m_simm->read();
	for (int slot = 0; slot < 2; slot++)
	{
		const int size = BIT(sizes, slot * 2, 2);
		const offs_t base = SIMM_BASE + slot * SIMM_WINDOW;
		const offs_t cells = size ? (0x00400000 << size) : 0;
		for (auto &xv : m_xv)
		{
			address_space &space = xv->space(roland_xv_device::AS_WAVE);
			space.unmap_readwrite(base, base + SIMM_WINDOW - 1);
			if (cells)
				space.install_ram(base, base + cells - 1, (SIMM_WINDOW - 1) & ~(cells - 1), m_simm_ram[slot].get());
		}
	}
}

u8 xv5080_state::scsi_dma_r()
{
	const u8 data = m_scsi->dma_r();
	scsi_dack();
	return data;
}

void xv5080_state::scsi_dma_w(u8 data)
{
	m_scsi->dma_w(data);
	scsi_dack();
}

void xv5080_state::scsi_drq_w(int state)
{
	m_scsi_drq = state;
	m_maincpu->dreq_w(0, state);
}

void xv5080_state::scsi_dack()
{
	if (!machine().side_effects_disabled() && m_scsi_drq)
	{
		m_maincpu->dreq_w(0, 0);
		m_maincpu->dreq_w(0, 1);
	}
}

u8 xv5080_state::card_r()
{
	if (BIT(m_ga_regs[GA_CARD], 1))
		return 0xff;
	return m_card->data_r();
}

void xv5080_state::card_w(u8 data)
{
	const u8 lines = m_ga_regs[GA_CARD];
	if (BIT(lines, 1))
		return;
	if (BIT(lines, 2))
		m_card->command_w(data);
	else if (BIT(lines, 0))
		m_card->address_w(data);
	else
		m_card->data_w(data);
}

u16 xv5080_state::porte_r()
{
	u16 data = 0xc000 | 0x0100;
	if (m_card->is_present())
	{
		data |= 0x0040;
		if (!m_card->is_protected())
			data |= 0x0080;
		if (m_card->is_busy())
			data &= ~0x0100;
	}
	return data;
}


bool xv_state::ga_enabled(int source) const
{
	switch (source)
	{
	case GA_SOURCE_KEY: return BIT(m_ga_regs[GA_ENABLE_LOW], 0);
	case GA_SOURCE_ENCODER: return BIT(m_ga_regs[GA_ENABLE_LOW], 1);
	case GA_SOURCE_TICK: return BIT(m_ga_regs[GA_ENABLE_HIGH], 0);
	case GA_SOURCE_SENSING: return BIT(m_ga_regs[GA_ENABLE_HIGH], 1);
	default: return true;
	}
}

void xv_state::ga_update()
{
	m_maincpu->set_input_line(0, m_ga_requests ? ASSERT_LINE : CLEAR_LINE);
}

void xv_state::ga_raise(int source)
{
	if (!ga_enabled(source))
		return;
	m_ga_requests |= 1 << source;
	ga_update();
}

void xv_state::ga_lower(int source)
{
	m_ga_requests &= ~(1 << source);
	m_maincpu->set_input_line(0, CLEAR_LINE);
	ga_update();
}

void xv_state::key_enqueue(u8 code)
{
	const u8 next = (m_key_tail + 1) % std::size(m_key_fifo);
	if (next == m_key_head)
		return;
	m_key_fifo[m_key_tail] = code;
	m_key_tail = next;
}

void xv_state::key_deliver()
{
	if (m_key_busy || m_key_head == m_key_tail)
		return;
	m_ga_key = m_key_fifo[m_key_head];
	m_key_head = (m_key_head + 1) % std::size(m_key_fifo);
	m_key_busy = true;
	ga_raise(GA_SOURCE_KEY);
}

TIMER_CALLBACK_MEMBER(xv_state::ga_scan)
{
	const u8 dial = m_dial->read();
	const s8 moved = s8(dial - m_dial_position);
	m_dial_position = dial;
	if (moved != 0)
	{
		m_encoder = std::clamp<s32>(m_encoder + moved, -128, 127);
		ga_raise(GA_SOURCE_ENCODER);
	}

	for (int row = 0; row < 8; row++)
	{
		const u8 now = m_keys[row]->read();
		const u8 changed = now ^ m_key_state[row];
		m_key_state[row] = now;
		for (int column = 0; column < 8; column++)
			if (BIT(changed, column))
				key_enqueue((row << 3) | column | (BIT(now, column) ? 0x80 : 0x00));
	}
	key_deliver();
}

TIMER_CALLBACK_MEMBER(xv_state::ga_tick)
{
	ga_raise(GA_SOURCE_TICK);
}

TIMER_CALLBACK_MEMBER(xv3080_state::ga_sensing)
{
	ga_raise(GA_SOURCE_SENSING);
}

void xv_state::porte_w(offs_t offset, u16 data, u16 ddr)
{
	LOGMASKED(LOG_PORT, "%s: port E = %04x (ddr %04x)\n", machine().describe_context(), data, ddr);
}

TIMER_CALLBACK_MEMBER(xv_state::display_request)
{
	m_maincpu->dreq_w(m_display_channel, 1);
	m_maincpu->dreq_w(m_display_channel, 0);
}

void xv5080_state::lcdc_data_w(u8 data)
{
	m_lcdc->data_w(data);
	lcdc_taken();
}

void xv5080_state::lcdc_command_w(u8 data)
{
	m_lcdc->command_w(data);
	lcdc_taken();
}

void xv5080_state::lcdc_taken()
{
	if (!machine().side_effects_disabled())
		m_display_timer->adjust(attotime::from_nsec(LCDC_REQUEST_NS), 0, attotime::from_usec(DISPLAY_REQUEST_US));
}

u8 xv3080_state::ga_switches_r(u8 reg)
{
	u8 data = 0xff;
	if (reg == GA_SWITCHES)
	{
		if (!BIT(m_panel->read(), 0))
			data &= ~0x01;
		if (!BIT(m_panel->read(), 1))
			data &= ~0x02;
	}
	return data;
}

u8 xv5080_state::ga_switches_r(u8 reg)
{
	u8 data = 0xff;
	if (reg == GA_SWITCHES)
	{
		if (!BIT(m_panel->read(), 0))
			data &= ~0x02;
		if (!BIT(m_panel->read(), 1))
			data &= ~0x04;
	}
	return data;
}

u8 xv_state::ga_r(offs_t offset)
{
	const u8 reg = offset & 0xff;
	u8 data = m_ga_regs[reg];
	switch (reg)
	{
	case GA_SOURCE:
		data = 0;
		for (int source = 15; source >= 0; source--)
			if (BIT(m_ga_requests, source))
			{
				data = source;
				break;
			}
		if (!machine().side_effects_disabled())
			ga_lower(data);
		break;

	case GA_ENCODER:
		data = u8(m_encoder);
		if (!machine().side_effects_disabled())
			m_encoder = 0;
		break;

	case GA_KEY:
		data = m_ga_key;
		if (!machine().side_effects_disabled())
		{
			m_key_busy = false;
			key_deliver();
		}
		break;

	case 0x00: case 0x01: case 0x02: case 0x03:
	case 0x04: case 0x05: case 0x06: case 0x07:
		data = m_keys[reg]->read();
		break;

	case GA_DIRECT_ROW:
	case GA_SWITCHES:
		data = ga_switches_r(reg);
		break;

	default:
		if (!machine().side_effects_disabled())
			LOGMASKED(LOG_GA, "%s: gate array read %03x = %02x\n", machine().describe_context(), offset, data);
		break;
	}
	return data;
}

void xv_state::ga_w(offs_t offset, u8 data)
{
	const u8 reg = offset & 0xff;
	m_ga_regs[reg] = data;
	switch (reg)
	{
	case GA_LEDS + 0:
	case GA_LEDS + 1:
	case GA_LEDS + 2:
	case GA_LEDS + 3:
		for (int i = 0; i < 8; i++)
			m_leds[(reg - GA_LEDS) * 8 + i] = BIT(data, i);
		break;

	case GA_ENABLE_LOW:
	case GA_ENABLE_HIGH:
		LOGMASKED(LOG_GA, "%s: gate array enables %02x = %02x\n", machine().describe_context(), reg, data);
		break;

	default:
		LOGMASKED(LOG_GA, "%s: gate array write %03x = %02x\n", machine().describe_context(), offset, data);
		break;
	}
}

void xv3080_state::ga_w(offs_t offset, u8 data)
{
	xv_state::ga_w(offset, data);
	switch (offset & 0xff)
	{
	case GA_LCD_COMMAND: m_lcd->control_w(data); break;
	case GA_LCD_DATA:    m_lcd->data_w(data);    break;
	}
}

void xv5080_state::ga_w(offs_t offset, u8 data)
{
	xv_state::ga_w(offset, data);
	if ((offset & 0xff) == GA_CLOCK)
		for (auto &xv : m_xv)
			xv->set_unscaled_clock(BIT(data, 0) ? XV_CLOCK_48K : XV_CLOCK_44K);
}


void xv5080_state::xv_wave_map(address_map &map)
{
	constexpr offs_t SRX_BASE[4] = { 0x04000000, 0x07000000, 0x08000000, 0x0b000000 };

	map.unmap_value_high();
	map(0x00000000, 0x00ffffff).mirror(0x01000000).rom().region("waverom", 0);
	for (int slot = 0; slot < 4; slot++)
		map(0x02000000 + slot * 0x800000, 0x027fffff + slot * 0x800000).umask16(0x00ff).r(m_exp[slot], FUNC(roland_srjv80_slot_device::read));
	for (int slot = 0; slot < 4; slot++)
		map(SRX_BASE[slot], SRX_BASE[slot] + 0xffffff).r(m_srx[slot], FUNC(roland_srx_slot_device::read16));
}

void xv3080_state::pump_slave(int state)
{
	m_xp[1]->run_frame();
}

u32 xv3080_state::master_link_r(offs_t strobe)
{
	return m_xp[0]->port_a_out_r(strobe);
}

u32 xv3080_state::slave_link_r(offs_t strobe)
{
	return m_xp[1]->port_a_out_r(strobe);
}


void xv_state::common_map(address_map &map)
{
	map(0x00000000, 0x0003ffff).rom().region("cpurom", 0);
	map(0x00400000, 0x007fffff).ram();
	map(0x006c0000, 0x006c0fff).rw(FUNC(xv_state::ga_r), FUNC(xv_state::ga_w));
	map(0x00800000, 0x0083ffff).mirror(0x003c0000).ram().share("nvram");
	map(0x00d00000, 0x00efffff).rom().region("progrom", 0);
}

void xv3080_state::xv3080_map(address_map &map)
{
	common_map(map);
	map(0x00200000, 0x00203fff).mirror(0x0007c000).rw(m_xp[0], FUNC(roland_xp_device::read), FUNC(roland_xp_device::write));
	map(0x00280000, 0x00283fff).mirror(0x0007c000).rw(m_xp[1], FUNC(roland_xp_device::read), FUNC(roland_xp_device::write));
	map(0x01000000, 0x011fffff).ram();
}

void xv5080_state::xv5080_map(address_map &map)
{
	common_map(map);
	map(0x00200000, 0x002001ff).mirror(0x0007fe00).rw(m_xv[0], FUNC(roland_xv_device::read), FUNC(roland_xv_device::write));
	map(0x00280000, 0x002801ff).mirror(0x0007fe00).rw(m_xv[1], FUNC(roland_xv_device::read), FUNC(roland_xv_device::write));
	map(0x00500000, 0x00500000).rw(FUNC(xv5080_state::scsi_dma_r), FUNC(xv5080_state::scsi_dma_w));
	map(0x005c0000, 0x005c0000).r(m_lcdc, FUNC(sed1330_device::data_r)).w(FUNC(xv5080_state::lcdc_data_w));
	map(0x005c0001, 0x005c0001).r(m_lcdc, FUNC(sed1330_device::status_r)).w(FUNC(xv5080_state::lcdc_command_w));
	map(0x00640000, 0x00640000).rw(FUNC(xv5080_state::card_r), FUNC(xv5080_state::card_w));
	map(0x00680000, 0x0068000f).m(m_scsi, FUNC(ncr53cf94_device::map));
	map(0x01000000, 0x013fffff).ram();
}

void xv3080_state::xp_rom_map(address_map &map)
{
	map.unmap_value_high();
	map(0x0000000, 0x1ffffff).rom().region("waverom", 0);
	for (int slot = 0; slot < 4; slot++)
		map(0x2000000 + slot * 0x800000, 0x27fffff + slot * 0x800000).r(m_exp[slot], FUNC(roland_srjv80_slot_device::read));
	for (int slot = 0; slot < 2; slot++)
		map(0x4000000 + slot * 0x2000000, 0x5ffffff + slot * 0x2000000).r(m_srx[slot], FUNC(roland_srx_slot_device::read));
}

void xv5080_state::lcdc_map(address_map &map)
{
	map(0x0000, 0x7fff).mirror(0x8000).ram();
}


void xv_state::lcd_palette(palette_device &palette) const
{
	palette.set_pen_color(0, rgb_t(0x50, 0xc8, 0x60));
	palette.set_pen_color(1, rgb_t(0x00, 0x20, 0x08));
}


INPUT_PORTS_START(xv3080)
	PORT_START("KEY0")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Exp")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Preset")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Card")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("User")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("GM")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Rhythm")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Patch")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Perform")
	PORT_START("KEY1")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("8/16 H / Tone Select 4")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("7/15 G / Tone Select 3")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("6/14 F / Tone Select 2")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("5/13 E / Tone Select 1")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("4/12 D / Tone Switch 4")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("3/11 C / Tone Switch 3")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("2/10 B / Tone Switch 2")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("1/9 A / Tone Switch 1")
	PORT_START("KEY2")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Exit") PORT_CODE(KEYCODE_ESC)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Palette")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("System")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Utility") PORT_CODE(KEYCODE_U)
	PORT_BIT(0x3c, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_START("KEY3")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Part Select 1-8/9-16")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("MIDI Message / RX")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Dec") PORT_CODE(KEYCODE_MINUS_PAD)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Cursor Up") PORT_CODE(KEYCODE_UP)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Cursor Right") PORT_CODE(KEYCODE_RIGHT)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Inc") PORT_CODE(KEYCODE_PLUS_PAD)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Patch Finder")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_START("KEY4")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Cursor Left") PORT_CODE(KEYCODE_LEFT)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Cursor Down") PORT_CODE(KEYCODE_DOWN)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Effects On/Off")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Undo")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Enter") PORT_CODE(KEYCODE_ENTER)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Shift") PORT_CODE(KEYCODE_LSHIFT)
	PORT_BIT(0x03, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_START("KEY5")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Info / TVA / Info")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("TVF")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Pitch / Tune")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("LFO / Preview")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Part / Wave / MIDI")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("MIDI / Control / Control")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Effects / Effects")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Common / Common / Setup")
	PORT_START("KEY6")
	PORT_BIT(0xff, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_START("KEY7")
	PORT_BIT(0xff, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("VALUE")
	PORT_BIT(0xff, 0x00, IPT_DIAL) PORT_SENSITIVITY(25) PORT_KEYDELTA(4) PORT_CODE_DEC(KEYCODE_MINUS) PORT_CODE_INC(KEYCODE_EQUALS) PORT_NAME("Value")

	PORT_START("PANEL")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Phrase Preview (Volume push)") PORT_CODE(KEYCODE_P)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Sound List (Value push)") PORT_CODE(KEYCODE_ENTER_PAD)
	PORT_BIT(0xfc, IP_ACTIVE_LOW, IPT_UNUSED)
INPUT_PORTS_END

INPUT_PORTS_START(xv5080)
	PORT_START("KEY0")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Exp")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Preset")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Card")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("User")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("GM")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Rhythm")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Patch")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Perform")
	PORT_START("KEY1")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("8/24")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("7/23")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("6/22")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("5/21")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("4/20")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("3/19")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("2/18")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("1/17")
	PORT_START("KEY2")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("16/32")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("15/31")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("14/30")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("13/29")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("12/28")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("11/27")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("10/26")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("9/25")
	PORT_START("KEY3")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Part Select")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("MIDI Message")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Dec") PORT_CODE(KEYCODE_MINUS_PAD)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Cursor Up") PORT_CODE(KEYCODE_UP)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Cursor Right") PORT_CODE(KEYCODE_RIGHT)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Inc") PORT_CODE(KEYCODE_PLUS_PAD)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Patch Finder")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_START("KEY4")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Cursor Left") PORT_CODE(KEYCODE_LEFT)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Cursor Down") PORT_CODE(KEYCODE_DOWN)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Effects")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Disk")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Undo")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Shift") PORT_CODE(KEYCODE_LSHIFT)
	PORT_BIT(0x03, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_START("KEY5")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Exit") PORT_CODE(KEYCODE_ESC)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("F6") PORT_CODE(KEYCODE_F6)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("F5") PORT_CODE(KEYCODE_F5)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("F4") PORT_CODE(KEYCODE_F4)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("F3") PORT_CODE(KEYCODE_F3)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("F2") PORT_CODE(KEYCODE_F2)
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("F1") PORT_CODE(KEYCODE_F1)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("System/Utility") PORT_CODE(KEYCODE_U)
	PORT_START("KEY6")
	PORT_BIT(0xff, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_START("KEY7")
	PORT_BIT(0xff, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("VALUE")
	PORT_BIT(0xff, 0x00, IPT_DIAL) PORT_SENSITIVITY(25) PORT_KEYDELTA(4) PORT_CODE_DEC(KEYCODE_MINUS) PORT_CODE_INC(KEYCODE_EQUALS) PORT_NAME("Value")

	PORT_START("PANEL")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Phrase Preview (Volume push)") PORT_CODE(KEYCODE_P)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Sound List (Value push)") PORT_CODE(KEYCODE_ENTER_PAD)
	PORT_BIT(0xfc, IP_ACTIVE_LOW, IPT_UNUSED)

	PORT_START("SIMM")
	PORT_CONFNAME(0x03, 0x00, "SIMM A")
	PORT_CONFSETTING(0x00, "None")
	PORT_CONFSETTING(0x01, "16 MB")
	PORT_CONFSETTING(0x02, "32 MB")
	PORT_CONFSETTING(0x03, "64 MB")
	PORT_CONFNAME(0x0c, 0x00, "SIMM B")
	PORT_CONFSETTING(0x00, "None")
	PORT_CONFSETTING(0x04, "16 MB")
	PORT_CONFSETTING(0x08, "32 MB")
	PORT_CONFSETTING(0x0c, "64 MB")
INPUT_PORTS_END


void xv_state::common(machine_config &config)
{
	m_maincpu->read_porta().set_constant(0xffffffff);
	m_maincpu->read_portb().set_constant(0xffff);
	m_maincpu->read_portc().set_constant(0xffff);
	m_maincpu->read_portd().set_constant(0xffffffff);
	m_maincpu->read_porte().set_constant(0xc000);
	m_maincpu->write_porte().set(FUNC(xv_state::porte_w));
	m_maincpu->read_portf().set_constant(0xffff);
	m_maincpu->read_adc<0>().set_constant(0x266);
	m_maincpu->read_adc<1>().set_constant(0x266);
	m_maincpu->read_adc<2>().set_constant(0x266);
	m_maincpu->read_adc<3>().set_constant(0x266);
	m_maincpu->read_adc<4>().set_constant(0x266);
	m_maincpu->read_adc<5>().set_constant(0x266);
	m_maincpu->read_adc<6>().set_constant(0x266);
	m_maincpu->read_adc<7>().set_constant(0x266);

	midi_port_device &mdin(MIDI_PORT(config, "mdin", midiin_slot, "midiin"));
	mdin.rxd_handler().set(m_maincpu, FUNC(sh7042_device::sci_rx_w<0>));
	midi_port_device &mdout(MIDI_PORT(config, "mdout", midiout_slot, "midiout"));
	m_maincpu->write_sci_tx<0>().set(mdout, FUNC(midi_port_device::write_txd));

	NVRAM(config, "nvram", nvram_device::DEFAULT_ALL_0);

	for (int slot = 0; slot < 4; slot++)
		ROLAND_SRJV80_SLOT(config, m_exp[slot], 0).set_image_names(util::string_format("xp-%c", 'a' + slot), util::string_format("xp-%c", 'a' + slot)).set_open(0xffff);
	SOFTWARE_LIST(config, "exp_list").set_original("roland_srjv80");
	SOFTWARE_LIST(config, "srx_list").set_original("roland_srx");

	PALETTE(config, "palette", FUNC(xv_state::lcd_palette), 2);
}

void xv3080_state::xv3080(machine_config &config)
{
	SH7042A(config, m_maincpu, 8.25_MHz_XTAL * 4);
	m_maincpu->set_addrmap(AS_PROGRAM, &xv3080_state::xv3080_map);
	common(config);

	for (int slot = 0; slot < 2; slot++)
		ROLAND_SRX_SLOT(config, m_srx[slot], 0).set_image_names(util::string_format("xp-%c", 'e' + slot), util::string_format("xp-%c", 'e' + slot)).set_open(0xffff);

	screen_device &screen(SCREEN(config, "screen"));
	screen.set_lcd();
	screen.set_refresh_hz(60);
	screen.set_screen_update(m_lcd, FUNC(hd44780_device::screen_update));
	screen.set_size(6 * 40, 9 * 2);
	screen.set_visarea_full();
	screen.set_palette("palette");
	config.set_default_layout(layout_roland_xv3080);

	HD44780(config, m_lcd, 270'000);
	m_lcd->set_lcd_size(2, 40);

	SPEAKER(config, "mix", 2).front();
	SPEAKER(config, "direct1", 2).front();
	SPEAKER(config, "direct2", 2).front();

	ROLAND_XP(config, m_xp[0], 24.576_MHz_XTAL);
	m_xp[0]->set_addrmap(roland_xp_device::AS_WAVE, &xv3080_state::xp_rom_map);
	m_xp[0]->int_callback().set_inputline(m_maincpu, 1);
	m_xp[0]->add_route(0, "mix", 1.0, 0);
	m_xp[0]->add_route(1, "mix", 1.0, 1);
	m_xp[0]->add_route(2, "direct1", 1.0, 0);
	m_xp[0]->add_route(3, "direct1", 1.0, 1);
	m_xp[0]->add_route(4, "direct2", 1.0, 0);
	m_xp[0]->add_route(5, "direct2", 1.0, 1);
	m_xp[0]->port_a_in_callback().set(FUNC(xv3080_state::slave_link_r));
	m_xp[0]->frame_callback().set(FUNC(xv3080_state::pump_slave));

	ROLAND_XP(config, m_xp[1], 24.576_MHz_XTAL);
	m_xp[1]->set_addrmap(roland_xp_device::AS_WAVE, &xv3080_state::xp_rom_map);
	m_xp[1]->int_callback().set_inputline(m_maincpu, 2);
	m_xp[1]->set_pumped(true);
	m_xp[1]->port_a_in_callback().set(FUNC(xv3080_state::master_link_r));
}

void xv5080_state::xv5080(machine_config &config)
{
	SH7042A(config, m_maincpu, 8.25_MHz_XTAL * 4);
	m_maincpu->set_addrmap(AS_PROGRAM, &xv5080_state::xv5080_map);
	common(config);
	m_maincpu->read_porte().set(FUNC(xv5080_state::porte_r));

	midi_port_device &mdin2(MIDI_PORT(config, "mdin2", midiin_slot, "midiin"));
	mdin2.rxd_handler().set(m_maincpu, FUNC(sh7042_device::sci_rx_w<1>));

	for (int slot = 0; slot < 4; slot++)
		ROLAND_SRX_SLOT(config, m_srx[slot], 0).set_image_names(util::string_format("xp-%c", 'e' + slot), util::string_format("xp-%c", 'e' + slot)).set_open(0xffff);

	screen_device &screen(SCREEN(config, "screen"));
	screen.set_lcd();
	screen.set_refresh_hz(60);
	screen.set_screen_update(m_lcdc, FUNC(sed1330_device::screen_update));
	screen.set_size(320, 80);
	screen.set_visarea_full();
	screen.set_palette("palette");
	config.set_default_layout(layout_roland_xv5080);

	SED1330(config, m_lcdc, 10_MHz_XTAL);
	m_lcdc->set_screen("screen");
	m_lcdc->set_addrmap(0, &xv5080_state::lcdc_map);

	SPEAKER(config, "mix", 2).front();
	SPEAKER(config, "outb", 2).front();
	SPEAKER(config, "outc", 2).front();
	SPEAKER(config, "outd", 2).front();

	ROLAND_XV(config, m_xv[0], XV_CLOCK_44K);
	m_xv[0]->set_addrmap(roland_xv_device::AS_WAVE, &xv5080_state::xv_wave_map);
	m_xv[0]->int_callback().set_inputline(m_maincpu, 1);
	m_xv[0]->set_link(m_xv[1]);
	m_xv[0]->add_route(0, "mix", 1.0, 0);
	m_xv[0]->add_route(1, "mix", 1.0, 1);
	m_xv[0]->add_route(2, "outb", 1.0, 0);
	m_xv[0]->add_route(3, "outb", 1.0, 1);
	m_xv[0]->add_route(4, "outc", 1.0, 0);
	m_xv[0]->add_route(5, "outc", 1.0, 1);
	m_xv[0]->add_route(6, "outd", 1.0, 0);
	m_xv[0]->add_route(7, "outd", 1.0, 1);

	ROLAND_XV(config, m_xv[1], XV_CLOCK_44K);
	m_xv[1]->set_addrmap(roland_xv_device::AS_WAVE, &xv5080_state::xv_wave_map);
	m_xv[1]->int_callback().set_inputline(m_maincpu, 2);

	nscsi_bus_device &scsi(NSCSI_BUS(config, "scsi"));
	NSCSI_CONNECTOR(config, "scsi:0", default_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:1", default_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:2", default_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:3", default_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:4", default_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:5", default_scsi_devices, nullptr);
	NSCSI_CONNECTOR(config, "scsi:6", default_scsi_devices, nullptr);
	NCR53CF94(config, m_scsi, 25_MHz_XTAL);
	scsi.set_external_device(7, m_scsi);
	m_scsi->irq_handler_cb().set_inputline(m_maincpu, 3);
	m_scsi->drq_handler_cb().set(FUNC(xv5080_state::scsi_drq_w));

	SMARTMEDIA(config, m_card);
}


ROM_START(xv3080)
	ROM_REGION32_BE(0x40000, "cpurom", 0)
	ROM_LOAD("hd6437042f33_ver1.00.ic3", 0x00000, 0x40000, CRC(b9f76b27) SHA1(52ff93d702cad686d091eeaa507604645b39e91c))

	ROM_REGION32_BE(0x200000, "progrom", 0)
	ROM_LOAD("xv-3080_v1.11.ic1", 0x000000, 0x200000, CRC(4d5b2473) SHA1(3c94673a837eca602664ab62aaf36140f677039d))

	ROM_REGION(0x2000000, "waverom", ROMREGION_ERASE00)
	ROM_LOAD("upd23c128040lgy-849.ic26", 0x0000000, 0x1000000, CRC(528aad5b) SHA1(5ced38ba29f9e6188ac1767933dc2bd9eac172fd))
	ROM_LOAD("upd23c128040lgy-850.ic29", 0x1000000, 0x1000000, CRC(17163f60) SHA1(4f360f09fe4e9185cf4a52df6aeb1e72c477fcff))
ROM_END

ROM_START(xv5080)
	ROM_DEFAULT_BIOS("v1.30")
	ROM_SYSTEM_BIOS(0, "v1.30", "ROM Version 1.30")
	ROM_SYSTEM_BIOS(1, "v1.28", "ROM Version 1.28")

	ROM_REGION32_BE(0x40000, "cpurom", 0)
	ROM_LOAD("hd6437042aa13f_ver1.00.ic3", 0x00000, 0x40000, CRC(f749a5bd) SHA1(3d2a9ca8cd10130e71353f82f440e46d44a3daf0))

	ROM_REGION32_BE(0x200000, "progrom", 0)
	ROMX_LOAD("xv-5080_v1.30.ic1", 0x000000, 0x200000, CRC(598f5c14) SHA1(14fdb7464c718a074530439235f92199a25e2ee3), ROM_BIOS(0))
	ROMX_LOAD("xv-5080_v1.28.ic1", 0x000000, 0x200000, CRC(91da248e) SHA1(cc928f20c7367d1cee789f30af50434381a8e4c7), ROM_BIOS(1))

	ROM_REGION16_LE(0x2000000, "waverom", ROMREGION_ERASE00)
	ROM_LOAD("upd23c128040lgy-849.ic26", 0x0000000, 0x1000000, CRC(528aad5b) SHA1(5ced38ba29f9e6188ac1767933dc2bd9eac172fd))
	ROM_LOAD("upd23c128040lgy-850.ic29", 0x1000000, 0x1000000, CRC(17163f60) SHA1(4f360f09fe4e9185cf4a52df6aeb1e72c477fcff))

	ROM_REGION(0x8000, "iomcu", 0)
	ROM_LOAD("m38881m2-069fp.ic107", 0x0000, 0x8000, NO_DUMP)
ROM_END

} // anonymous namespace


//    YEAR  NAME    PARENT  COMPAT  MACHINE  INPUT   CLASS         INIT          COMPANY   FULLNAME   FLAGS
SYST( 2000, xv3080, 0,      0,      xv3080,  xv3080, xv3080_state, init_waverom, "Roland", "XV-3080", MACHINE_SUPPORTS_SAVE )
SYST( 2000, xv5080, 0,      0,      xv5080,  xv5080, xv5080_state, init_waverom, "Roland", "XV-5080", MACHINE_SUPPORTS_SAVE )
