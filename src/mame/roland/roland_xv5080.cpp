// license:BSD-3-Clause
// copyright-holders:superctr
/****************************************************************************

    Roland XV-3080 and XV-5080 128-voice synthesizer modules.

    Both run the same SH7042 platform: a 256 kB on-chip mask ROM that boots
    the machine and carries the flash updater, a 2 MB program flash, battery
    backed SRAM for the user banks, DRAM for the edit buffers, the
    TC160G22AF gate array (IC4) for the panel, the LEDs and the interrupt
    sources, and two tone generator chips side by side on CS0.

    XV-3080 main board (service notes, parts list):
    - IC3 HD6437042F33 SH-2, 8.25 MHz crystal, x4 PLL, on-chip ROM enabled
    - IC92/IC93 RA09-002 (XP6, TC203C180AF) PCM chips at 24.576 MHz
    - IC4 TC160G22AF-1253 gate array
    - IC6/IC84 uPD431000 SRAM, 256 kB, battery backed
    - IC5 VG2618165 DRAM, 2 MB; IC95/IC96 LC324260 are the XPs' effect RAM
    - IC1 LH28F160S5T program flash, 2 MB
    - IC26/IC29 uPD23C128040 wave mask ROMs, 16 MB each (undumped; the
      set is in hand descrambled, see the ROM definitions)
    - IC71/IC74/IC79 AK4324 DACs
    - a 40 x 2 character LCD (L4052B1J000) behind the gate array

    XV-5080 main board:
    - IC3 HD6437042AA13F SH-2, the same crystal
    - IC12/IC13 TC223C660CF-503 (RA08-503) "XV" tone generators
    - IC4 TC160G22AF-1253 gate array
    - IC6/IC84 uPD431000 SRAM, 256 kB; IC117/IC118 MSM5117805 DRAM, 4 MB;
      IC18/IC19 VG2618165 are the XVs' RAM
    - IC1 LH28F160S5T program flash, 2 MB
    - IC26/IC29 the same two wave mask ROMs
    - IC51 SED1335F0B LCD controller with a 32 kB TC55257 (IC52) of its
      own, driving a 320 x 80 graphic LCD (RCM6048T-A)
    - IC71/IC74/IC79/IC82 AK4324 DACs
    - IC107 M38881M2 for the SmartMedia, SCSI and R-BUS side (undumped)

    The bus map follows giulioz's emulator of both machines: the two tone
    generator windows at 0x00200000 and 0x00280000 (CS0), the graphic LCD
    at 0x005c0000 and the gate array at 0x006c0000 (CS1), the battery SRAM
    at 0x00800000 (CS2), the program flash at 0x00d00000 (CS3) and the DRAM
    at 0x01000000.  IRQ0 is the gate array, IRQ1 and IRQ2 the two chips.

    Both machines boot their firmware: the XV-3080 to its PERFORM/PLAY
    screen, the XV-5080 through its splash to the expansion board status
    page.  The display bytes travel by DMA, channel 0 under DREQ0 on the
    XV-3080 (with the transfer end interrupt), channel 1 under DREQ1 on the
    XV-5080 (polled), one request every 40 us; the firmware's kernel
    dispatches its tasks from the watchdog's interval timer interrupt and
    ticks from MTU1 (XV-3080) or MTU2 (XV-5080).  The XV-3080 also bit
    bangs a serial link on port E (PE12 clock, PE9 out, PE13 in, PE10/PE11
    selects) to read the expansion boards' ID PROMs; no board is fitted,
    so nothing answers.

    The XV-3080 plays MIDI once its battery SRAM holds a factory reset
    (UTILITY, cursor right to UTIL 2, FACTORY RESET, ENTER, DEC to lift the
    write protect, ENTER, ENTER, ENTER); on a blank SRAM it boots to "User
    Memory Damaged" and no part takes a note.  XP6 #0 drives all three
    DACs, port B being MIX OUT and the phones, C and D the two DIRECT OUT
    pairs; #1 has no DAC and reaches them through #0 over the port A bus.

    Not done: the XV chips are their host interface only (sound/roland_xv),
    which answers the memory scan and the interrupt path but plays nothing,
    and the XV-5080's blank SRAM still wants its factory reset; neither
    wave ROM is dumped (the descrambled set stands in).

****************************************************************************/

#include "emu.h"

#include "bus/midi/midiinport.h"
#include "bus/midi/midioutport.h"
#include "cpu/sh/sh7042.h"
#include "machine/nvram.h"
#include "sound/roland_xp.h"
#include "sound/roland_xv.h"
#include "video/hd44780.h"
#include "video/sed1330.h"

#include "emupal.h"
#include "screen.h"
#include "speaker.h"

#include <algorithm>

#define LOG_GA      (1U << 1)

#define VERBOSE (LOG_GENERAL | LOG_GA)
#include "logmacro.h"


namespace {

class xv3080_state : public driver_device
{
public:
	xv3080_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_xp(*this, "xp%u", 0U)
		, m_lcd(*this, "lcd")
		, m_keys(*this, "KEY%u", 0U)
		, m_dial(*this, "VALUE")
		, m_panel(*this, "PANEL")
		, m_leds(*this, "led%u", 0U)
	{
	}

	void xv3080(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

	void common(machine_config &config) ATTR_COLD;
	void common_map(address_map &map) ATTR_COLD;
	void xv3080_map(address_map &map) ATTR_COLD;
	void xp_rom_map(address_map &map) ATTR_COLD;

	u8 ga_r(offs_t offset);
	void ga_w(offs_t offset, u8 data);
	virtual u8 ga_switches_r(u8 reg);
	void ga_raise(int source);
	void ga_lower(int source);
	void ga_update();
	bool ga_enabled(int source) const;
	void key_enqueue(u8 code);
	void key_deliver();
	TIMER_CALLBACK_MEMBER(ga_scan);
	TIMER_CALLBACK_MEMBER(ga_tick);
	TIMER_CALLBACK_MEMBER(ga_sensing);
	TIMER_CALLBACK_MEMBER(display_request);
	void porte_w(offs_t offset, u16 data, u16 ddr);
	void lcd_palette(palette_device &palette) const ATTR_COLD;
	void pump_slave(int state);
	u32 master_link_r(offs_t strobe);
	u32 slave_link_r(offs_t strobe);

	required_device<sh7042_device> m_maincpu;
	optional_device_array<roland_xp_device, 2> m_xp;
	optional_device<hd44780_device> m_lcd;
	required_ioport_array<8> m_keys;
	required_ioport m_dial;
	required_ioport m_panel;
	output_finder<32> m_leds;

	emu_timer *m_ga_scan_timer = nullptr;
	emu_timer *m_ga_tick_timer = nullptr;
	emu_timer *m_ga_sensing_timer = nullptr;
	emu_timer *m_display_timer = nullptr;
	int m_display_channel = 0;
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

class xv5080_state : public xv3080_state
{
public:
	xv5080_state(const machine_config &mconfig, device_type type, const char *tag)
		: xv3080_state(mconfig, type, tag)
		, m_xv(*this, "xv%u", 0U)
		, m_lcdc(*this, "lcdc")
	{
	}

	void xv5080(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;
	virtual u8 ga_switches_r(u8 reg) override;

	void xv5080_map(address_map &map) ATTR_COLD;
	void xv_wave_map(address_map &map) ATTR_COLD;
	void lcdc_map(address_map &map) ATTR_COLD;

	required_device_array<roland_xv_device, 2> m_xv;
	required_device<sed1330_device> m_lcdc;
};


// the gate array's sources
static constexpr int GA_SOURCE_KEY = 0;
static constexpr int GA_SOURCE_ENCODER = 1;
static constexpr int GA_SOURCE_SENSING = 7;
static constexpr int GA_SOURCE_TICK = 9;

static constexpr int GA_TICK_HZ = 1000;
static constexpr int GA_SENSING_HZ = 10;
static constexpr int GA_SCAN_HZ = 120;

// the display takes a byte by DMA every 40 us
static constexpr int DISPLAY_REQUEST_US = 40;

// the gate array's registers, by the byte offset in its window
static constexpr u8 GA_LEDS = 0x10;         // 0x10-0x13
static constexpr u8 GA_LCD_COMMAND = 0x38;  // XV-3080: the character LCD's instruction and data registers
static constexpr u8 GA_LCD_DATA = 0x39;
static constexpr u8 GA_DIRECT_ROW = 0x3a;
static constexpr u8 GA_SWITCHES = 0x3b;
static constexpr u8 GA_ENABLE_LOW = 0x3e;
static constexpr u8 GA_ENABLE_HIGH = 0x3f;
static constexpr u8 GA_SOURCE = 0x40;
static constexpr u8 GA_ENCODER = 0x42;
static constexpr u8 GA_KEY = 0x43;


void xv3080_state::machine_start()
{
	m_ga_scan_timer = timer_alloc(FUNC(xv3080_state::ga_scan), this);
	m_ga_tick_timer = timer_alloc(FUNC(xv3080_state::ga_tick), this);
	m_ga_sensing_timer = timer_alloc(FUNC(xv3080_state::ga_sensing), this);
	m_display_timer = timer_alloc(FUNC(xv3080_state::display_request), this);

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

void xv3080_state::machine_reset()
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
	if (m_xp[0])
		m_ga_sensing_timer->adjust(attotime::from_hz(GA_SENSING_HZ), 0, attotime::from_hz(GA_SENSING_HZ));
	m_display_timer->adjust(attotime::from_usec(DISPLAY_REQUEST_US), 0, attotime::from_usec(DISPLAY_REQUEST_US));
}

void xv5080_state::machine_start()
{
	xv3080_state::machine_start();
}

void xv5080_state::machine_reset()
{
	xv3080_state::machine_reset();
}


//-------------------------------------------------
//  the TC160G22AF gate array
//
//  Sixteen sources share IRQ0.  Reading 0x40 names the highest one waiting
//  and takes it; a source's own data register - the key code at 0x43, the
//  encoder movement at 0x42 - is read by its handler.  0x3e enables the key
//  (bit 0) and encoder (bit 1) sources, 0x3f the 1 kHz tick, source 9
//  (bit 0), and the XV-3080's active sensing, source 7 (bit 1): the boot
//  ROM's key scan polls IRQ0 with only the key source enabled, so a
//  periodic source raising the line there would read as a key.
//
//  A key event is a code with bit 7 set on press, delivered one at a time:
//  the next waits until the firmware has read the last.  0x10-0x13 are the
//  32 LED drives.  0x3a and 0x3b are direct inputs: 0x3b carries PREVIEW
//  and the encoder's push, at different bits on the two machines.
//-------------------------------------------------

bool xv3080_state::ga_enabled(int source) const
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

void xv3080_state::ga_update()
{
	m_maincpu->set_input_line(0, m_ga_requests ? ASSERT_LINE : CLEAR_LINE);
}

void xv3080_state::ga_raise(int source)
{
	if (!ga_enabled(source))
		return;
	m_ga_requests |= 1 << source;
	ga_update();
}

// the acknowledge ends the pulse; another waiting source starts a fresh one
// so an edge triggered IRQ0 sees it too
void xv3080_state::ga_lower(int source)
{
	m_ga_requests &= ~(1 << source);
	m_maincpu->set_input_line(0, CLEAR_LINE);
	ga_update();
}

void xv3080_state::key_enqueue(u8 code)
{
	const u8 next = (m_key_tail + 1) % std::size(m_key_fifo);
	if (next == m_key_head)
		return;
	m_key_fifo[m_key_tail] = code;
	m_key_tail = next;
}

void xv3080_state::key_deliver()
{
	if (m_key_busy || m_key_head == m_key_tail)
		return;
	m_ga_key = m_key_fifo[m_key_head];
	m_key_head = (m_key_head + 1) % std::size(m_key_fifo);
	m_key_busy = true;
	ga_raise(GA_SOURCE_KEY);
}

TIMER_CALLBACK_MEMBER(xv3080_state::ga_scan)
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

TIMER_CALLBACK_MEMBER(xv3080_state::ga_tick)
{
	ga_raise(GA_SOURCE_TICK);
}

TIMER_CALLBACK_MEMBER(xv3080_state::ga_sensing)
{
	ga_raise(GA_SOURCE_SENSING);
}

void xv3080_state::porte_w(offs_t offset, u16 data, u16 ddr)
{
	LOG("%s: port E = %04x (ddr %04x)\n", machine().describe_context(), data, ddr);
}

// the display's bytes travel by DMA, one for each request the gate array
// pulses on the channel's DREQ pin: channel 0 on the XV-3080, 1 on the XV-5080
TIMER_CALLBACK_MEMBER(xv3080_state::display_request)
{
	m_maincpu->dreq_w(m_display_channel, 1);
	m_maincpu->dreq_w(m_display_channel, 0);
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
	else if (reg == 0x05)
		data = BIT(m_panel->read(), 2) ? 0x00 : 0x01;
	return data;
}

u8 xv3080_state::ga_r(offs_t offset)
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

	case GA_DIRECT_ROW:
	case GA_SWITCHES:
	case 0x05:
		data = ga_switches_r(reg);
		break;

	default:
		if (!machine().side_effects_disabled())
			LOGMASKED(LOG_GA, "%s: gate array read %03x = %02x\n", machine().describe_context(), offset, data);
		break;
	}
	return data;
}

void xv3080_state::ga_w(offs_t offset, u8 data)
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

	case GA_LCD_COMMAND:
	case GA_LCD_DATA:
		if (m_lcd)
		{
			if (reg == GA_LCD_DATA)
				m_lcd->data_w(data);
			else
				m_lcd->control_w(data);
		}
		else
			LOGMASKED(LOG_GA, "%s: gate array write %03x = %02x\n", machine().describe_context(), offset, data);
		break;

	default:
		LOGMASKED(LOG_GA, "%s: gate array write %03x = %02x\n", machine().describe_context(), offset, data);
		break;
	}
}


//-------------------------------------------------
//  the XV-5080's wave memory as its chips see it, one space for both: the
//  two mask ROMs at cell 0, two bytes a cell, the low byte first; the four
//  SR-JV80 slots at 0x02000000, 0x02800000, 0x03000000 and 0x03800000 (one
//  byte a cell), the four SRX slots at 0x04000000, 0x06000000, 0x08000000
//  and 0x0a000000, the two SIMM slots at 0x0c000000 and 0x0e000000.  None
//  of the slots is filled.
//-------------------------------------------------

void xv5080_state::xv_wave_map(address_map &map)
{
	map(0x00000000, 0x00ffffff).rom().region("waverom", 0);
}


//-------------------------------------------------
//  the two XP6s read each other over the port A bus, and #0's frame
//  clocks #1's
//-------------------------------------------------

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


//-------------------------------------------------
//  address maps
//-------------------------------------------------

void xv3080_state::common_map(address_map &map)
{
	map(0x00000000, 0x0003ffff).rom().region("cpurom", 0);
	map(0x00400000, 0x007fffff).ram();
	map(0x006c0000, 0x006c0fff).rw(FUNC(xv3080_state::ga_r), FUNC(xv3080_state::ga_w));
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
	map(0x005c0000, 0x005c0000).rw(m_lcdc, FUNC(sed1330_device::data_r), FUNC(sed1330_device::data_w));
	map(0x005c0001, 0x005c0001).rw(m_lcdc, FUNC(sed1330_device::status_r), FUNC(sed1330_device::command_w));
	map(0x01000000, 0x013fffff).ram();
}

// IC26 on chip select 0 and IC29 on chip select 1, word-wide, the
// descrambled set's 32 regions of 1 MB in order; chip selects 2 and 3 are
// the four SR-JV80 sockets (8 MB each) and 4 to 7 the two EXP connectors,
// which the firmware's boot scan probes and must find empty.
void xv3080_state::xp_rom_map(address_map &map)
{
	map(0x0000000, 0x1ffffff).rom().region("waverom", 0);
}

void xv5080_state::lcdc_map(address_map &map)
{
	map(0x0000, 0x7fff).mirror(0x8000).ram(); // TC55257 32 KB
}


//-------------------------------------------------
//  displays
//-------------------------------------------------

void xv3080_state::lcd_palette(palette_device &palette) const
{
	palette.set_pen_color(0, rgb_t(0xf8, 0xc8, 0x40)); // backlight
	palette.set_pen_color(1, rgb_t(0x20, 0x10, 0x00)); // dot on
}


//-------------------------------------------------
//  inputs
//-------------------------------------------------

static INPUT_PORTS_START(xv3080)
	// the switch matrix of the panel boards (service notes page 28), coded
	// by the gate array as the scan line YSS0-YSS5 in bits 5-3 and the data
	// line PD0-PD7 in bits 2-0; sixteen of the codes checked against the
	// display, the rest follow the schematic
	PORT_START("KEY0")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Exp")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Preset")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Card")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("User")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("GS")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Rhythm")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Patch")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Perform")
	PORT_START("KEY1")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("8/16")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("7/15")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("6/14")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("5/13")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("4/12")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("3/11")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("2/10")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("1/9")
	PORT_START("KEY2")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Exit") PORT_CODE(KEYCODE_ESC)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Edit")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("System")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Utility") PORT_CODE(KEYCODE_U)
	PORT_BIT(0x3c, IP_ACTIVE_HIGH, IPT_UNUSED)
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
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Undo")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Enter") PORT_CODE(KEYCODE_ENTER)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Shift") PORT_CODE(KEYCODE_LSHIFT)
	PORT_BIT(0x03, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_START("KEY5")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("TVA")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("TVF")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Patch (tone)")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("LFO")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Wave")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Control")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Effects (tone)")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Common")
	PORT_START("KEY6")
	PORT_BIT(0xff, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_START("KEY7")
	PORT_BIT(0xff, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("VALUE")
	PORT_BIT(0xff, 0x00, IPT_DIAL) PORT_SENSITIVITY(25) PORT_KEYDELTA(4) PORT_CODE_DEC(KEYCODE_MINUS) PORT_CODE_INC(KEYCODE_EQUALS) PORT_NAME("Value")

	PORT_START("PANEL")
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Preview") PORT_CODE(KEYCODE_P)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Value (push)") PORT_CODE(KEYCODE_ENTER_PAD)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Exit") PORT_CODE(KEYCODE_BACKSPACE)
	PORT_BIT(0xf8, IP_ACTIVE_LOW, IPT_UNUSED)
INPUT_PORTS_END

static INPUT_PORTS_START(xv5080)
	// the XV-5080's matrix (service notes page 36): PANEL-A's five rows on
	// YSS0-YSS4 in the same order and column order as the XV-3080's, the
	// function keys of PANEL-B on YSS5; read off the schematic, one code
	// checked against the display
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
	PORT_BIT(0x01, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Preview") PORT_CODE(KEYCODE_P)
	PORT_BIT(0x02, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Value (push)") PORT_CODE(KEYCODE_ENTER_PAD)
	PORT_BIT(0x04, IP_ACTIVE_LOW, IPT_KEYPAD) PORT_NAME("Exit (direct)") PORT_CODE(KEYCODE_BACKSPACE)
	PORT_BIT(0xf8, IP_ACTIVE_LOW, IPT_UNUSED)
INPUT_PORTS_END


//-------------------------------------------------
//  machine configuration
//-------------------------------------------------

void xv3080_state::common(machine_config &config)
{
	// every unwired port pin reads high, except on port E where only PE14
	// and PE15 are pulled up; every analog input reads a healthy battery
	m_maincpu->read_porta().set_constant(0xffffffff);
	m_maincpu->read_portb().set_constant(0xffff);
	m_maincpu->read_portc().set_constant(0xffff);
	m_maincpu->read_portd().set_constant(0xffffffff);
	m_maincpu->read_porte().set_constant(0xc000);
	m_maincpu->write_porte().set(FUNC(xv3080_state::porte_w));
	m_maincpu->read_portf().set_constant(0xffff);
	m_maincpu->read_adc<0>().set_constant(0x266);
	m_maincpu->read_adc<1>().set_constant(0x266);
	m_maincpu->read_adc<2>().set_constant(0x266);
	m_maincpu->read_adc<3>().set_constant(0x266);
	m_maincpu->read_adc<4>().set_constant(0x266);
	m_maincpu->read_adc<5>().set_constant(0x266);
	m_maincpu->read_adc<6>().set_constant(0x266);
	m_maincpu->read_adc<7>().set_constant(0x266);

	// MIDI IN on RXD0, MIDI OUT on TXD0
	midi_port_device &mdin(MIDI_PORT(config, "mdin", midiin_slot, "midiin"));
	mdin.rxd_handler().set(m_maincpu, FUNC(sh7042_device::sci_rx_w<0>));
	MIDI_PORT(config, "mdout", midiout_slot, "midiout");
	m_maincpu->write_sci_tx<0>().set("mdout", FUNC(midi_port_device::write_txd));

	NVRAM(config, "nvram", nvram_device::DEFAULT_ALL_0);

	PALETTE(config, "palette", FUNC(xv3080_state::lcd_palette), 2);
}

void xv3080_state::xv3080(machine_config &config)
{
	SH7042A(config, m_maincpu, 8.25_MHz_XTAL * 4);
	m_maincpu->set_addrmap(AS_PROGRAM, &xv3080_state::xv3080_map);
	common(config);

	screen_device &screen(SCREEN(config, "screen"));
	screen.set_lcd();
	screen.set_refresh_hz(60);
	screen.set_screen_update("lcd", FUNC(hd44780_device::screen_update));
	screen.set_size(6 * 40, 9 * 2);
	screen.set_visarea_full();
	screen.set_palette("palette");

	HD44780(config, m_lcd, 270'000);
	m_lcd->set_lcd_size(2, 40);

	// the three AK4324s all hang off XP6 #0 (IC92): SDOB is MIX OUT and the
	// phones, SDOC and SDOD DIRECT OUT 1 and 2; #1 (IC93) has no DAC and
	// reaches them through #0 over the port A bus, its frame clocked by #0
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
	m_display_channel = 1;

	screen_device &screen(SCREEN(config, "screen"));
	screen.set_lcd();
	screen.set_refresh_hz(60);
	screen.set_screen_update("lcdc", FUNC(sed1330_device::screen_update));
	screen.set_size(320, 80);
	screen.set_visarea_full();
	screen.set_palette("palette");

	SED1330(config, m_lcdc, 10_MHz_XTAL); // SED1335F0B
	m_lcdc->set_screen("screen");
	m_lcdc->set_addrmap(0, &xv5080_state::lcdc_map);

	SPEAKER(config, "speaker", 2).front();

	ROLAND_XV(config, m_xv[0], 0);
	m_xv[0]->set_addrmap(roland_xv_device::AS_WAVE, &xv5080_state::xv_wave_map);
	m_xv[0]->int_callback().set_inputline(m_maincpu, 1);
	m_xv[0]->add_route(0, "speaker", 1.0, 0);
	m_xv[0]->add_route(1, "speaker", 1.0, 1);

	ROLAND_XV(config, m_xv[1], 0);
	m_xv[1]->set_addrmap(roland_xv_device::AS_WAVE, &xv5080_state::xv_wave_map);
	m_xv[1]->int_callback().set_inputline(m_maincpu, 2);
	m_xv[1]->add_route(0, "speaker", 1.0, 0);
	m_xv[1]->add_route(1, "speaker", 1.0, 1);
}


//-------------------------------------------------
//  ROM definitions
//-------------------------------------------------

ROM_START(xv3080)
	ROM_REGION32_BE(0x40000, "cpurom", 0)
	ROM_LOAD("hd6437042f33_ver1.00.ic3", 0x00000, 0x40000, CRC(b9f76b27) SHA1(52ff93d702cad686d091eeaa507604645b39e91c))

	ROM_REGION32_BE(0x200000, "progrom", 0)
	ROM_LOAD("xv-3080_v1.11.ic1", 0x000000, 0x200000, CRC(4d5b2473) SHA1(3c94673a837eca602664ab62aaf36140f677039d))

	ROM_REGION(0x2000000, "waverom", ROMREGION_ERASE00)
	ROM_LOAD("upd23c128040lgy-849.ic26", 0x0000000, 0x1000000, NO_DUMP)
	ROM_LOAD("upd23c128040lgy-850.ic29", 0x1000000, 0x1000000, NO_DUMP)
	// the pair as Roland's JV-1080 and SRX plugins carry it: the 32 MB
	// wave set descrambled, one header ("XV3080ROM_Ver001", 1999-10-13)
	// at the front, where the two chips meet not marked
	ROM_LOAD("xv3080rom_ver001.bin", 0x0000000, 0x2000000, BAD_DUMP CRC(34e32c1a) SHA1(258f124ae67e4da4a0ae332c4bac79bb96acaf29))
ROM_END

ROM_START(xv5080)
	ROM_REGION32_BE(0x40000, "cpurom", 0)
	ROM_LOAD("hd6437042aa13f_ver1.00.ic3", 0x00000, 0x40000, CRC(f749a5bd) SHA1(3d2a9ca8cd10130e71353f82f440e46d44a3daf0))

	ROM_REGION32_BE(0x200000, "progrom", 0)
	ROM_LOAD("xv-5080_v1.30.ic1", 0x000000, 0x200000, CRC(598f5c14) SHA1(14fdb7464c718a074530439235f92199a25e2ee3))

	ROM_REGION16_LE(0x2000000, "waverom", ROMREGION_ERASE00)
	ROM_LOAD("upd23c128040lgy-849.ic26", 0x0000000, 0x1000000, NO_DUMP)
	ROM_LOAD("upd23c128040lgy-850.ic29", 0x1000000, 0x1000000, NO_DUMP)
	// the pair as Roland's JV-1080 and SRX plugins carry it: the 32 MB
	// wave set descrambled, one header ("XV3080ROM_Ver001", 1999-10-13)
	// at the front, where the two chips meet not marked
	ROM_LOAD("xv3080rom_ver001.bin", 0x0000000, 0x2000000, BAD_DUMP CRC(34e32c1a) SHA1(258f124ae67e4da4a0ae332c4bac79bb96acaf29))

	ROM_REGION(0x8000, "iomcu", 0)
	ROM_LOAD("m38881m2-069fp.ic107", 0x0000, 0x8000, NO_DUMP)
ROM_END

} // anonymous namespace


//    YEAR  NAME    PARENT  COMPAT  MACHINE  INPUT   CLASS         INIT        COMPANY   FULLNAME   FLAGS
SYST( 2000, xv3080, 0,      0,      xv3080,  xv3080, xv3080_state, empty_init, "Roland", "XV-3080", MACHINE_NOT_WORKING )
SYST( 2000, xv5080, 0,      0,      xv5080,  xv5080, xv5080_state, empty_init, "Roland", "XV-5080", MACHINE_NOT_WORKING )
