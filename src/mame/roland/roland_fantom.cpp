// license:BSD-3-Clause
// copyright-holders:superctr
/****************************************************************************

    Roland Fantom (FA-76), 2001.

    An SH7709A carrying one XV tone generator -- the XV-5080's chip and the
    XV-5080's 32 MB wave set, wave list and sample records byte for byte --
    with a 76-note keyboard, a sequencer, a floppy drive and three wave
    expansion sockets: one SR-JV80 and two SRX.

    The flash holds a boot block and then the program, zlib packed in the
    same container the SD-90 uses; the boot block inflates it into the
    SDRAM at 0x0c000000 and jumps to 0x0c001000.  There is no on-chip ROM:
    the first instruction of the flash is the reset vector.

    Main board, from the service notes:

        IC3     HD6417709AF133      SH-3, 16.5 MHz x 8 = 132 MHz
        IC5     TC58FVB321FT-10     program flash, 32 Mbit, 2M x 16
        IC4/7   HY57V641620HGT-P    SDRAM, 64 Mbit each, 4M x 16, 16 MB
        IC12    BR24C08F            EEPROM, 8 kbit, serial
        IC13    M66273FP            LCD controller
        IC24    S1L50282F32H000     key scan, 24 MHz
        IC25    FDC37C78            floppy disk controller
        IC31    TC223C660CF-503     the XV, 16.9344 MHz
        IC37    GM71V18163CT-6      the XV's effect DRAM, 16 Mbit, 1M x 16
        IC48/51 uPD23C128040LGY     wave mask ROM, 128 Mbit each, 8M x 16
        IC59/62 AK4393-VF-E2        D/A
        IC68    TC9271FS            digital out

    The display is an LM320191, 320 x 240 dots, backlit, driven by the
    M66273FP out of its own 19 200-byte VRAM.  CN7 is the SR-JV80 socket
    and CN10 and CN11 the two SRX, which the test mode calls slots A, B
    and C.

    The wave mask ROMs are the XV-3080's and the XV-5080's, the same two
    Roland stock numbers (02010023 and 02010056) in all three machines, and
    the machine's own wave list and sample records -- which its firmware
    carries in the clear -- are those machines' byte for byte.

    The interrupt inputs, from the service notes and confirmed by the
    firmware's own vector table: IRQ0 the tone generator, IRQ2 the key
    scan, IRQ3 the floppy controller, IRQ4 one phase of the value encoder,
    and DREQ0/DACK0/TCLK the floppy controller's DMA.  MIDI is the IrDA
    serial channel, the serial EEPROM is bit-banged on port C (PC6 SCL,
    PC7 SDA) and the encoder is read on port F.

    The key scan chip presents two words: an event, its status in the high
    byte and the key in the low byte, and a 14-bit time stamp.  The status
    bits, as the firmware's own state machine reads them: bit 6 a keyboard
    event, bit 4 the first contact closed, bit 5 the second contact closed,
    bit 2 the time stamp wrapped since the last event.  The firmware takes
    the velocity from the time between the two contacts, in eighths, with
    767 the slowest stroke it distinguishes.  The unit of that time is not
    known; 100 us is assumed here.  How the panel buttons come through the
    same chip is not known yet, so the panel has no inputs.

    The firmware composes every screen after the splash in SDRAM and moves
    it to the LCD controller's VRAM on DMA channel 1, with the controller's
    cycle-steal enable as the request; the request is held asserted here.
    The program flash keeps the machine's user data in its top half, and
    the firmware identifies the chip by manufacturer code before it will
    program it.

    State: boots to the PERFORMANCE PLAY screen, takes MIDI and plays.  The
    panel buttons, the value encoder and the analogue inputs have no
    inputs, and the floppy controller's DMA request is not connected.

****************************************************************************/

#include "emu.h"

#include "bus/midi/midiinport.h"
#include "bus/midi/midioutport.h"
#include "cpu/sh/sh3comn.h"
#include "cpu/sh/sh3_scif.h"
#include "cpu/sh/sh4.h"
#include "imagedev/floppy.h"
#include "machine/i2cmem.h"
#include "machine/intelfsh.h"
#include "machine/upd765.h"
#include "sound/roland_xv.h"
#include "wavecard.h"

#include "emupal.h"
#include "screen.h"
#include "speaker.h"

#include <deque>


//**************************************************************************
//  the S1L50282F key scan
//**************************************************************************

class fantom_keyscan_device : public device_t
{
public:
	fantom_keyscan_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	auto int_handler() { return m_int_cb.bind(); }

	u16 read(offs_t offset);

	// a key going down closes its first contact at once and its second
	// after the travel time; going up, the second opens at once and the
	// first after the travel time
	void key_w(int key, int state, const attotime &travel);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	static constexpr unsigned KEYS = 88;
	static constexpr unsigned TIME_BITS = 14;

	struct event
	{
		u8 status = 0;
		u8 key = 0;
		u16 time = 0;
	};

	struct pending
	{
		attotime when;
		u8 key;
		u8 contact;
		u8 state;
	};

	void contact_w(int key, int contact, int state);
	void push(u8 status, u8 key);
	void update_int();
	u16 stamp() const;
	TIMER_CALLBACK_MEMBER(wrap);
	TIMER_CALLBACK_MEMBER(travel);

	devcb_write_line m_int_cb;
	emu_timer *m_wrap_timer;
	emu_timer *m_travel_timer;

	util::fifo<event, 64> m_fifo;
	std::deque<pending> m_pending;
	event m_latch;
	u8 m_contacts[KEYS];
	u8 m_int;
};

DEFINE_DEVICE_TYPE(FANTOM_KEYSCAN, fantom_keyscan_device, "fantom_keyscan", "Roland Fantom key scan")

fantom_keyscan_device::fantom_keyscan_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, FANTOM_KEYSCAN, tag, owner, clock)
	, m_int_cb(*this)
	, m_wrap_timer(nullptr)
	, m_travel_timer(nullptr)
{
}

void fantom_keyscan_device::device_start()
{
	m_wrap_timer = timer_alloc(FUNC(fantom_keyscan_device::wrap), this);
	m_travel_timer = timer_alloc(FUNC(fantom_keyscan_device::travel), this);

	save_item(NAME(m_latch.status));
	save_item(NAME(m_latch.key));
	save_item(NAME(m_latch.time));
	save_item(NAME(m_contacts));
	save_item(NAME(m_int));
}

void fantom_keyscan_device::device_reset()
{
	while (!m_fifo.empty())
		m_fifo.dequeue();
	m_pending.clear();
	m_latch = event();
	std::fill(std::begin(m_contacts), std::end(m_contacts), 0);
	m_int = 0;
	m_int_cb(0);

	const attotime period = clocks_to_attotime(1 << TIME_BITS);
	m_wrap_timer->adjust(period, 0, period);
	m_travel_timer->adjust(attotime::never);
}

u16 fantom_keyscan_device::stamp() const
{
	return attotime_to_clocks(machine().time()) & ((1 << TIME_BITS) - 1);
}

void fantom_keyscan_device::push(u8 status, u8 key)
{
	if (m_fifo.full())
		return;
	m_fifo.enqueue(event{ status, key, stamp() });
	update_int();
}

void fantom_keyscan_device::update_int()
{
	const u8 state = m_fifo.empty() ? 0 : 1;
	if (state != m_int)
	{
		m_int = state;
		m_int_cb(state);
	}
}

TIMER_CALLBACK_MEMBER(fantom_keyscan_device::wrap)
{
	push(0x04, 0);
}

void fantom_keyscan_device::contact_w(int key, int contact, int state)
{
	const u8 mask = 1 << contact;
	const u8 contacts = (m_contacts[key] & ~mask) | (state ? mask : 0);
	if (contacts == m_contacts[key])
		return;
	m_contacts[key] = contacts;
	push(0x40 | (contacts << 4), key);
}

void fantom_keyscan_device::key_w(int key, int state, const attotime &travel)
{
	if (key < 0 || key >= int(KEYS))
		return;
	contact_w(key, state ? 0 : 1, state);
	m_pending.push_back(pending{ machine().time() + travel, u8(key), u8(state ? 1 : 0), u8(state ? 1 : 0) });
	if (m_pending.size() == 1)
		m_travel_timer->adjust(travel);
}

TIMER_CALLBACK_MEMBER(fantom_keyscan_device::travel)
{
	while (!m_pending.empty() && m_pending.front().when <= machine().time())
	{
		const pending &p = m_pending.front();
		contact_w(p.key, p.contact, p.state);
		m_pending.pop_front();
	}
	if (!m_pending.empty())
		m_travel_timer->adjust(m_pending.front().when - machine().time());
}

u16 fantom_keyscan_device::read(offs_t offset)
{
	if (offset == 0)
	{
		if (!machine().side_effects_disabled())
		{
			m_latch = m_fifo.empty() ? event() : m_fifo.dequeue();
			update_int();
		}
		return (m_latch.status << 8) | m_latch.key;
	}
	return m_latch.time;
}


namespace {

//**************************************************************************
//  the machine
//**************************************************************************

class fantom_state : public driver_device
{
public:
	fantom_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_flash(*this, "flash")
		, m_fdc(*this, "fdc")
		, m_xv(*this, "xv")
		, m_exp(*this, "expa")
		, m_srx(*this, "exp%c", 'b')
		, m_eeprom(*this, "eeprom")
		, m_keyscan(*this, "keyscan")
		, m_velocity(*this, "VELOCITY")
	{
	}

	void fantom(machine_config &config) ATTR_COLD;

	DECLARE_INPUT_CHANGED_MEMBER(key_changed);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	void fantom_map(address_map &map) ATTR_COLD;
	void fantom_io_map(address_map &map) ATTR_COLD;
	void xv_wave_map(address_map &map) ATTR_COLD;

	u64 portc_r();
	void portc_w(u64 data);

	u8 vram_r(offs_t offset) { return m_vram[offset]; }
	void vram_w(offs_t offset, u8 data) { m_vram[offset] = data; }
	u8 lcdc_r(offs_t offset) { return m_lcdc[offset >> 1]; }
	void lcdc_w(offs_t offset, u8 data) { m_lcdc[offset >> 1] = data; }
	u32 screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	void lcd_palette(palette_device &palette) const ATTR_COLD;

	required_device<sh7709_device> m_maincpu;
	required_device<tc58fvb321_device> m_flash;
	required_device<n82077aa_device> m_fdc;
	required_device<roland_xv_device> m_xv;
	required_device<srjv80_slot_device> m_exp;
	required_device_array<srx_slot_device, 2> m_srx;
	required_device<i2c_24c08_device> m_eeprom;
	required_device<fantom_keyscan_device> m_keyscan;
	required_ioport m_velocity;

	static constexpr int LCD_WIDTH = 320, LCD_HEIGHT = 240;
	static constexpr int VRAM_BYTES = LCD_WIDTH * LCD_HEIGHT / 4;

	std::unique_ptr<u8[]> m_vram;
	u8 m_lcdc[0x50]{};
};


void fantom_state::machine_start()
{
	m_vram = make_unique_clear<u8[]>(VRAM_BYTES);
	save_pointer(NAME(m_vram), VRAM_BYTES);
	save_item(NAME(m_lcdc));
}

void fantom_state::machine_reset()
{
	// the M66273FP's cycle-steal window, which the firmware's frame transfer
	// waits for on DMA channel 1: always open here
	m_maincpu->dreq_w<1>(1);
}


void fantom_state::lcd_palette(palette_device &palette) const
{
	for (int i = 0; i < 4; i++)     // pen 0 dark, pen 3 fully lit
		palette.set_pen_color(i, rgb_t(0xff * i / 3, 0xff * i / 3, 0xff * i / 3));
}


//-------------------------------------------------
//  the display.  The M66273FP's own VRAM is 19 200 bytes and every byte of
//  it is on the bus; the firmware sets the controller to single scan, four
//  grey levels and eighty characters a line, which is a 320 x 240 panel
//  with two bits a pixel, the leftmost pixel in the top bits.  R1's REV bit
//  is set, so the largest value is the background.  Nothing else the
//  controller does -- the scroll registers, the grey-scale pattern table it
//  loads into R17 to R80 -- is emulated.
//-------------------------------------------------

u32 fantom_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	const bool on = BIT(m_lcdc[0], 0);      // R1 bit 0, LCDE
	const int rev = BIT(m_lcdc[0], 1) ? 3 : 0;  // R1 bit 1, REV

	for (int y = cliprect.top(); y <= cliprect.bottom(); y++)
	{
		u16 *dest = &bitmap.pix(y, cliprect.left());
		for (int x = cliprect.left(); x <= cliprect.right(); x++)
		{
			const u8 byte = m_vram[y * (LCD_WIDTH / 4) + (x >> 2)];
			const int level = (byte >> (6 - 2 * (x & 3))) & 3;
			*dest++ = on ? (level ^ rev) : 0;
		}
	}
	return 0;
}


//-------------------------------------------------
//  port C: the serial EEPROM's I2C bus, PC6 the clock and PC7 the data,
//  which the firmware drives as an open drain by switching the pin
//  between output and input
//-------------------------------------------------

u64 fantom_state::portc_r()
{
	return 0x3f | (m_eeprom->read_sda() << 7);
}

void fantom_state::portc_w(u64 data)
{
	const u16 pccr = data >> 16;
	const bool sda_out = ((pccr >> 14) & 3) == 1;
	const bool scl_out = ((pccr >> 12) & 3) == 1;
	m_eeprom->write_sda(sda_out ? BIT(data, 7) : 1);
	m_eeprom->write_scl(scl_out ? BIT(data, 6) : 1);
}


//-------------------------------------------------
//  the keyboard: E1 to G7, key 0 to 75 to the scan chip
//-------------------------------------------------

INPUT_CHANGED_MEMBER(fantom_state::key_changed)
{
	static const int travel_units[8] = { 760, 480, 320, 200, 136, 80, 16, 16 };
	const attotime travel = m_keyscan->clocks_to_attotime(travel_units[m_velocity->read() & 7]);
	m_keyscan->key_w(param, newval, travel);
}


//-------------------------------------------------
//  the CPU's own bus.  Each chip select carries one part, and the service
//  notes name them: CS0 the flash, CS3 the SDRAM, CS4 the key scan, CS5
//  the floppy controller, CS6 the tone generator and the display, which
//  two gates split by A15.
//-------------------------------------------------

void fantom_state::fantom_map(address_map &map)
{
	map(0x00000000, 0x003fffff).rw(m_flash, FUNC(tc58fvb321_device::read), FUNC(tc58fvb321_device::write));
	map(0x0c000000, 0x0cffffff).ram();
	map(0x10000000, 0x10000003).r(m_keyscan, FUNC(fantom_keyscan_device::read));
	map(0x14000000, 0x14000007).m(m_fdc, FUNC(n82077aa_device::map));
	map(0x18000000, 0x180001ff).mirror(0x7e00).rw(m_xv, FUNC(roland_xv_device::read), FUNC(roland_xv_device::write));
	map(0x18008000, 0x1800caff).rw(FUNC(fantom_state::vram_r), FUNC(fantom_state::vram_w));
	map(0x1800d000, 0x1800d09f).rw(FUNC(fantom_state::lcdc_r), FUNC(fantom_state::lcdc_w));
}

void fantom_state::fantom_io_map(address_map &map)
{
	map(SH3_PORT_C, SH3_PORT_C + 7).rw(FUNC(fantom_state::portc_r), FUNC(fantom_state::portc_w));
}


//-------------------------------------------------
//  the wave memory as the XV sees it.  The chip's own chip selects, as on
//  the XV-5080: 0 and 1 the two mask ROMs, 2 and 3 the SR-JV80 sockets and
//  4 to 7 the SRX connectors.  Which sub-slot each of this machine's three
//  connectors takes is unread; the first of each kind is assumed.
//-------------------------------------------------

void fantom_state::xv_wave_map(address_map &map)
{
	map(0x00000000, 0x00ffffff).rom().region("waverom", 0);
	map(0x02000000, 0x027fffff).r(m_exp, FUNC(srjv80_slot_device::read)).umask16(0x00ff);
	map(0x04000000, 0x04ffffff).r(m_srx[0], FUNC(srx_slot_device::read16));
	map(0x07000000, 0x07ffffff).r(m_srx[1], FUNC(srx_slot_device::read16));
}


void fantom_state::fantom(machine_config &config)
{
	SH7709(config, m_maincpu, 16.5_MHz_XTAL * 8, ENDIANNESS_BIG);   // HD6417709AF133
	m_maincpu->set_addrmap(AS_PROGRAM, &fantom_state::fantom_map);
	m_maincpu->set_addrmap(AS_IO, &fantom_state::fantom_io_map);

	TC58FVB321(config, m_flash);    // the program flash, whose top half the firmware keeps its user data in

	I2C_24C08(config, m_eeprom);    // BR24C08F

	FANTOM_KEYSCAN(config, m_keyscan, 10000);     // the time stamp unit, a choice
	m_keyscan->int_handler().set_inputline(m_maincpu, 2);   // IRQ2, low level

	N82077AA(config, m_fdc, 24_MHz_XTAL, n82077aa_device::mode_t::PS2);   // FDC37C78
	m_fdc->intrq_wr_callback().set_inputline(m_maincpu, 3);               // IRQ3
	FLOPPY_CONNECTOR(config, "fdc:0", "35hd", FLOPPY_35_HD, true, floppy_image_device::default_pc_floppy_formats);

	// the LM320191 panel on the M66273FP's four-bit output
	screen_device &screen(SCREEN(config, "screen").set_lcd());
	screen.set_refresh_hz(60);
	screen.set_size(LCD_WIDTH, LCD_HEIGHT);
	screen.set_visarea_full();
	screen.set_screen_update(FUNC(fantom_state::screen_update));
	screen.set_palette("palette");
	PALETTE(config, "palette", FUNC(fantom_state::lcd_palette), 4);

	SRJV80_SLOT(config, m_exp, 0);      // CN7, slot A
	for (auto &srx : m_srx)             // CN10 and CN11, slots B and C
		SRX_SLOT(config, srx, 0);

	// OUTPUT A and OUTPUT B, one AK4393 each
	SPEAKER(config, "outa", 2).front();
	SPEAKER(config, "outb", 2).front();

	ROLAND_XV(config, m_xv, 16.9344_MHz_XTAL);
	m_xv->set_addrmap(roland_xv_device::AS_WAVE, &fantom_state::xv_wave_map);
	m_xv->int_callback().set_inputline(m_maincpu, 0);   // IRQ0, the falling edge
	m_xv->add_route(0, "outa", 1.0, 0);
	m_xv->add_route(1, "outa", 1.0, 1);
	m_xv->add_route(2, "outb", 1.0, 0);
	m_xv->add_route(3, "outb", 1.0, 1);

	// MIDI on the SH7709's IrDA channel
	midi_port_device &mdin(MIDI_PORT(config, "mdin", midiin_slot, "midiin"));
	mdin.rxd_handler().set(m_maincpu->irda(), FUNC(sh3_scif_device::rxd_w));
	MIDI_PORT(config, "mdout", midiout_slot, "midiout");
	m_maincpu->irda().txd_handler().set("mdout", FUNC(midi_port_device::write_txd));
}


static INPUT_PORTS_START(fantom)
	PORT_START("VELOCITY")
	PORT_CONFNAME(0x07, 0x03, "Key velocity")
	PORT_CONFSETTING(0x00, "pp")
	PORT_CONFSETTING(0x01, "p")
	PORT_CONFSETTING(0x02, "mp")
	PORT_CONFSETTING(0x03, "mf")
	PORT_CONFSETTING(0x04, "f")
	PORT_CONFSETTING(0x05, "ff")
	PORT_CONFSETTING(0x06, "fff")

#define FANTOM_KEY(port, bit, key, name, code) \
	PORT_BIT(1 << (bit), IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(name) code PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(fantom_state::key_changed), key)

	PORT_START("KEY0")
	FANTOM_KEY(0, 0,  0, "E1", )
	FANTOM_KEY(0, 1,  1, "F1", )
	FANTOM_KEY(0, 2,  2, "F#1", )
	FANTOM_KEY(0, 3,  3, "G1", )
	FANTOM_KEY(0, 4,  4, "G#1", )
	FANTOM_KEY(0, 5,  5, "A1", )
	FANTOM_KEY(0, 6,  6, "A#1", )
	FANTOM_KEY(0, 7,  7, "B1", )

	PORT_START("KEY1")
	FANTOM_KEY(1, 0,  8, "C2", )
	FANTOM_KEY(1, 1,  9, "C#2", )
	FANTOM_KEY(1, 2, 10, "D2", )
	FANTOM_KEY(1, 3, 11, "D#2", )
	FANTOM_KEY(1, 4, 12, "E2", )
	FANTOM_KEY(1, 5, 13, "F2", )
	FANTOM_KEY(1, 6, 14, "F#2", )
	FANTOM_KEY(1, 7, 15, "G2", )

	PORT_START("KEY2")
	FANTOM_KEY(2, 0, 16, "G#2", )
	FANTOM_KEY(2, 1, 17, "A2", )
	FANTOM_KEY(2, 2, 18, "A#2", )
	FANTOM_KEY(2, 3, 19, "B2", )
	FANTOM_KEY(2, 4, 20, "C3", PORT_CODE(KEYCODE_Z))
	FANTOM_KEY(2, 5, 21, "C#3", PORT_CODE(KEYCODE_S))
	FANTOM_KEY(2, 6, 22, "D3", PORT_CODE(KEYCODE_X))
	FANTOM_KEY(2, 7, 23, "D#3", PORT_CODE(KEYCODE_D))

	PORT_START("KEY3")
	FANTOM_KEY(3, 0, 24, "E3", PORT_CODE(KEYCODE_C))
	FANTOM_KEY(3, 1, 25, "F3", PORT_CODE(KEYCODE_V))
	FANTOM_KEY(3, 2, 26, "F#3", PORT_CODE(KEYCODE_G))
	FANTOM_KEY(3, 3, 27, "G3", PORT_CODE(KEYCODE_B))
	FANTOM_KEY(3, 4, 28, "G#3", PORT_CODE(KEYCODE_H))
	FANTOM_KEY(3, 5, 29, "A3", PORT_CODE(KEYCODE_N))
	FANTOM_KEY(3, 6, 30, "A#3", PORT_CODE(KEYCODE_J))
	FANTOM_KEY(3, 7, 31, "B3", PORT_CODE(KEYCODE_M))

	PORT_START("KEY4")
	FANTOM_KEY(4, 0, 32, "C4", PORT_CODE(KEYCODE_Q))
	FANTOM_KEY(4, 1, 33, "C#4", PORT_CODE(KEYCODE_2))
	FANTOM_KEY(4, 2, 34, "D4", PORT_CODE(KEYCODE_W))
	FANTOM_KEY(4, 3, 35, "D#4", PORT_CODE(KEYCODE_3))
	FANTOM_KEY(4, 4, 36, "E4", PORT_CODE(KEYCODE_E))
	FANTOM_KEY(4, 5, 37, "F4", PORT_CODE(KEYCODE_R))
	FANTOM_KEY(4, 6, 38, "F#4", PORT_CODE(KEYCODE_5))
	FANTOM_KEY(4, 7, 39, "G4", PORT_CODE(KEYCODE_T))

	PORT_START("KEY5")
	FANTOM_KEY(5, 0, 40, "G#4", PORT_CODE(KEYCODE_6))
	FANTOM_KEY(5, 1, 41, "A4", PORT_CODE(KEYCODE_Y))
	FANTOM_KEY(5, 2, 42, "A#4", PORT_CODE(KEYCODE_7))
	FANTOM_KEY(5, 3, 43, "B4", PORT_CODE(KEYCODE_U))
	FANTOM_KEY(5, 4, 44, "C5", PORT_CODE(KEYCODE_I))
	FANTOM_KEY(5, 5, 45, "C#5", PORT_CODE(KEYCODE_9))
	FANTOM_KEY(5, 6, 46, "D5", PORT_CODE(KEYCODE_O))
	FANTOM_KEY(5, 7, 47, "D#5", PORT_CODE(KEYCODE_0))

	PORT_START("KEY6")
	FANTOM_KEY(6, 0, 48, "E5", PORT_CODE(KEYCODE_P))
	FANTOM_KEY(6, 1, 49, "F5", )
	FANTOM_KEY(6, 2, 50, "F#5", )
	FANTOM_KEY(6, 3, 51, "G5", )
	FANTOM_KEY(6, 4, 52, "G#5", )
	FANTOM_KEY(6, 5, 53, "A5", )
	FANTOM_KEY(6, 6, 54, "A#5", )
	FANTOM_KEY(6, 7, 55, "B5", )

	PORT_START("KEY7")
	FANTOM_KEY(7, 0, 56, "C6", )
	FANTOM_KEY(7, 1, 57, "C#6", )
	FANTOM_KEY(7, 2, 58, "D6", )
	FANTOM_KEY(7, 3, 59, "D#6", )
	FANTOM_KEY(7, 4, 60, "E6", )
	FANTOM_KEY(7, 5, 61, "F6", )
	FANTOM_KEY(7, 6, 62, "F#6", )
	FANTOM_KEY(7, 7, 63, "G6", )

	PORT_START("KEY8")
	FANTOM_KEY(8, 0, 64, "G#6", )
	FANTOM_KEY(8, 1, 65, "A6", )
	FANTOM_KEY(8, 2, 66, "A#6", )
	FANTOM_KEY(8, 3, 67, "B6", )
	FANTOM_KEY(8, 4, 68, "C7", )
	FANTOM_KEY(8, 5, 69, "C#7", )
	FANTOM_KEY(8, 6, 70, "D7", )
	FANTOM_KEY(8, 7, 71, "D#7", )

	PORT_START("KEY9")
	FANTOM_KEY(9, 0, 72, "E7", )
	FANTOM_KEY(9, 1, 73, "F7", )
	FANTOM_KEY(9, 2, 74, "F#7", )
	FANTOM_KEY(9, 3, 75, "G7", )
	PORT_BIT(0xf0, IP_ACTIVE_HIGH, IPT_UNUSED)

#undef FANTOM_KEY
INPUT_PORTS_END


ROM_START(fantom)
	// out of Roland's own updater, Fantom118_SMF.exe: a boot block, then
	// the program in two zlib containers, and the top half left erased
	ROM_REGION16_BE(0x400000, "flash", 0)
	ROM_LOAD("fantom_v1.18.ic5", 0x000000, 0x400000, CRC(f9fae390) SHA1(86f6924893d1496b491fc124173d349694c30c1b))

	ROM_REGION16_LE(0x2000000, "waverom", ROMREGION_ERASE00)
	ROM_LOAD("upd23c128040lgy-849.ic48", 0x0000000, 0x1000000, NO_DUMP)
	ROM_LOAD("upd23c128040lgy-850.ic51", 0x1000000, 0x1000000, NO_DUMP)
	// the pair as Roland's JV-1080 and SRX plugins carry it: the XV-3080
	// and XV-5080's 32 MB wave set, which this machine's own wave list and
	// sample records say it shares byte for byte
	ROM_LOAD("xv3080rom_ver001.bin", 0x0000000, 0x2000000, BAD_DUMP CRC(34e32c1a) SHA1(258f124ae67e4da4a0ae332c4bac79bb96acaf29))
ROM_END

} // anonymous namespace


//    YEAR  NAME    PARENT  COMPAT  MACHINE  INPUT   CLASS         INIT        COMPANY   FULLNAME        FLAGS
SYST( 2001, fantom, 0,      0,      fantom,  fantom, fantom_state, empty_init, "Roland", "Fantom (FA-76)", MACHINE_NOT_WORKING )
