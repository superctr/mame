// license:BSD-3-Clause
// copyright-holders:superctr
/****************************************************************************

    EDIROL SD-90 (2001) and SD-80 (2002) Studio Canvas.

    Two USB sound modules on one firmware: an SH-3 with two XV tone
    generators over a wave ROM set of their own, and a USB block with its
    own 8-bit CPU.  The SD-90 adds a second effect processor, the MR3, and
    audio inputs; the SD-80 is the same machine without them.

    SD-80 main board, from the service notes:

        IC2     HD6417706           the CPU, an SH-3
        IC3     LH28F160BJE-BTL80   program flash, 16 Mbit, 1M x 16
        IC4/6   LC3816161ET-70      SDRAM, 4 MB the pair
        IC19/27 TC223C660CF-503     the two XVs, Roland's RA08-503
        IC26/30 GM71C18163CJ-6      one 16 Mbit effect DRAM beside each
        IC28/29 uPD23C128040ALGY    wave mask ROM, 128 Mbit each
        IC17    M37641M8-137FP      the USB block's 8-bit CPU, its own ROM
        IC32/34 PCM1716E            the two DACs, with a TC9271FS (IC33)
                                    driving the digital output
        LB IC7  RCM2072M-B          the display, 20 characters by 2 lines

    X1 is 24.000 MHz at the M37641, X2 16.9344 MHz -- 384 x 44.1 kHz -- on
    the wave side, and the CPU's EXTAL takes 12 MHz off the USB block
    through IC5, a TC7SH04FU inverter.  In clock mode 1 PLL circuit 2
    multiplies that by four, and the boot block's FRQCR of 0x0112 doubles it
    again for the CPU: 96 MHz internal, 48 MHz bus, 24 MHz peripheral, which
    is the 31250 baud the two serial channels ask for.  The SD-90's own
    service notes are not in hand and which SH-3 it carries is unread; its
    firmware uses no register that tells one family member from another.

    There is no on-chip ROM: the flash's first instruction is the reset
    vector, and the boot block sets the bus up, inflates the program out of
    two zlib containers into the SDRAM and enters it.  The SD-80's dump is
    the flash with the two bytes of every word exchanged.

    The interrupts, read out of the firmware's own dispatch table -- it
    indexes on INTEVT2 and every entry here has a handler of its own:

        SD-80   TMU0, TMU1, TMU2, the watchdog's interval timer, IRQ0 the
                XV at 0x14000000, IRQ1 the XV at 0x15000000, IRQ2 the USB
                controller's IBF and IRQ3 its OBF, IRQ5 a pin read back on
                SCPDR, the SCI and the SCIF
        SD-90   TMU0, TMU1, the watchdog, IRQ0 and IRQ1 the two XVs, IRQ2
                its 0x18800000 mailbox, PINT0-7, the IrDA channel and the SCIF

    So both machines' MIDI is two channels of the CPU's own: the SD-80's
    SCI and SCIF, the SD-90's IrDA channel and SCIF.

    State: **the SD-80 runs.**  It boots, inflates the program into the
    SDRAM at 0x08001000, reaches its play screen, takes MIDI on the SCIF,
    and its panel, LEDs and value dial work; there is nothing to hear,
    because the wave mask ROMs are undumped.  The SCI's MIDI port is the
    core's to implement, and the answer its USB controller gives at
    power-on is a stub of two bytes.  The SD-90 boots its loader out of the
    flash into the SDRAM at 0x883de000, gets past the word its loader polls
    after each of the commands it sends the tone generator, inflates its
    program and enters it,
    and stops in the driver for its own area 6 device, polling +8 for the
    1 that would say a command had been taken.

****************************************************************************/

#include "emu.h"

#include "bus/midi/midiinport.h"
#include "bus/midi/midioutport.h"
#include "cpu/sh/sh3_scif.h"
#include "cpu/sh/sh3comn.h"
#include "cpu/sh/sh4.h"
#include "sound/roland_xv.h"
#include "video/hd44780.h"

#include "emupal.h"
#include "screen.h"
#include "speaker.h"

#define LOG_UIPC    (1U << 1)

#define VERBOSE (LOG_GENERAL)
#include "logmacro.h"


namespace {

class sd90_state : public driver_device
{
public:
	sd90_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_xv(*this, "xv%u", 0U)
		, m_lcd(*this, "lcd")
		, m_dial(*this, "DIAL")
		, m_leds(*this, "led_%u", 0U)
	{
	}

	void sd80(machine_config &config) ATTR_COLD;
	void sd90(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD { m_uipc_step = 0; }

	void common(machine_config &config) ATTR_COLD;

	void sd80_map(address_map &map) ATTR_COLD;
	void sd90_map(address_map &map) ATTR_COLD;
	void sd80_io_map(address_map &map) ATTR_COLD;
	void xv_wave_map(address_map &map) ATTR_COLD;

	template <int Channel> u8 uipc_r(offs_t offset);
	template <int Channel> void uipc_w(offs_t offset, u8 data);
	template <int Device> u8 sd90_area6_r(offs_t offset);
	template <int Device> void sd90_area6_w(offs_t offset, u8 data);

	void led_w(offs_t offset, u8 data);
	u64 scp_r();
	bool enca() const { return m_encoder_phase != 1 && m_encoder_phase != 2; }
	TIMER_CALLBACK_MEMBER(step_encoder);

	void lcd_palette(palette_device &palette) const ATTR_COLD;

	enum { LED_GS, LED_MIDI, LED_SYSTEM, LED_NATIVE, LED_USB, LED_EFFECTS, LED_GM2, LED_PREVIEW, LED_INST_DRUM, LEDS };

	u32 m_uipc_step = 0;
	u8 m_dial_last = 0;
	s8 m_dial_pending = 0;
	u8 m_encoder_phase = 0;

	emu_timer *m_encoder_timer = nullptr;

	required_device<sh7709_device> m_maincpu;
	required_device_array<roland_xv_device, 2> m_xv;
	optional_device<hd44780_device> m_lcd;
	optional_ioport m_dial;
	output_finder<LEDS> m_leds;
};


void sd90_state::machine_start()
{
	m_encoder_timer = timer_alloc(FUNC(sd90_state::step_encoder), this);
	m_encoder_timer->adjust(attotime::from_hz(1000), 0, attotime::from_hz(1000));

	save_item(NAME(m_uipc_step));
	save_item(NAME(m_dial_last));
	save_item(NAME(m_dial_pending));
	save_item(NAME(m_encoder_phase));
}


//-------------------------------------------------
//  the display, an RCM2072M-B of 20 characters by two lines on the LCD
//  board, blue backlit
//-------------------------------------------------

void sd90_state::lcd_palette(palette_device &palette) const
{
	palette.set_pen_color(0, rgb_t(0x20, 0x50, 0xd0));  // backlight
	palette.set_pen_color(1, rgb_t(0xe8, 0xf0, 0xff));  // dot on
}


//-------------------------------------------------
//  the front panel, which is on the tone generator: ten buttons and nine
//  LEDs, three strobes each way, in the chip's own numbering
//-------------------------------------------------

void sd90_state::led_w(offs_t offset, u8 data)
{
	static const int led[3][3] = {
		{ LED_GS,     LED_MIDI,    LED_SYSTEM },
		{ LED_NATIVE, LED_USB,     LED_EFFECTS },
		{ LED_GM2,    LED_PREVIEW, LED_INST_DRUM }
	};
	const int strobe = offset >> 3, line = offset & 7;
	if (strobe < 3 && line < 3)
		m_leds[led[strobe][line]] = data;
}


//-------------------------------------------------
//  the value encoder, which is not: XENCA on SCPT5 is IRQ5 and the
//  handler reads XENCB back on SCPDR, so a detent is a quadrature cycle
//-------------------------------------------------

u64 sd90_state::scp_r()
{
	const bool encb = m_encoder_phase < 2;
	return 0xcf | (enca() << 5) | (encb << 4);   // SCPT3, XENCSW, is not wired
}

TIMER_CALLBACK_MEMBER(sd90_state::step_encoder)
{
	if (!m_dial)
		return;

	const u8 now = m_dial->read();
	m_dial_pending += s8(now - m_dial_last);
	m_dial_last = now;

	if (!m_dial_pending)
		return;

	const int step = m_dial_pending > 0 ? 1 : -1;
	const bool was = enca();
	m_encoder_phase = (m_encoder_phase + step) & 3;
	if (!m_encoder_phase)
		m_dial_pending -= step;

	// one edge, one interrupt: the handler takes XENCB for the direction
	// and clears its own bit in IRR0
	if (was != enca())
		m_maincpu->set_input_line(5, enca() ? CLEAR_LINE : ASSERT_LINE);
}


//-------------------------------------------------
//  the USB controller's mailboxes.  IC13A, a TC74VHCT139A, decodes the
//  chip select XCSIPC into XS0 and XS1, which are the S0 and S1 pins of
//  IC17's bus interface unit, with the CPU's A1 at its A0 and XRD and XWR
//  at its R and W; its IBF and OBF come back as IRQ2 and IRQ3.  So each
//  window is one channel: data at +0, status at +2, bit 1 of
//  UIPC(0) set while the byte just written is still there and bit 0 of
//  UIPC(1) set while one waits, with bits 4-7 of the status the byte's
//  tag -- the same channels, bits and tags as the SC-8850's.
//
//  At power-on the firmware waits for an untagged 0xaa from the
//  controller, reads one byte after it, and then pushes two blocks of its
//  own across the channel.  Nothing here emulates the M37641M8, whose
//  program is a mask ROM of its own and undumped, so this answers that
//  announcement and then goes quiet, which is what gets the machine past
//  its power-on wait.
//-------------------------------------------------

template <int Channel>
u8 sd90_state::uipc_r(offs_t offset)
{
	// the outbound channel always takes the byte at once
	if (Channel == 0)
		return 0;

	static const u8 boot[][2] = { { 0x01, 0xaa }, { 0x01, 0x00 } };

	if (m_uipc_step >= std::size(boot))
		return 0;

	if (offset == 0)
	{
		const u8 data = boot[m_uipc_step][1];
		if (!machine().side_effects_disabled())
		{
			LOGMASKED(LOG_UIPC, "%s: uipc announcement byte %02x\n", machine().describe_context(), data);
			m_uipc_step++;
		}
		return data;
	}

	return boot[m_uipc_step][0];
}

template <int Channel>
void sd90_state::uipc_w(offs_t offset, u8 data)
{
	LOGMASKED(LOG_UIPC, "%s: uipc(%d) write %x = %02x\n", machine().describe_context(), Channel, offset, data);
}


//-------------------------------------------------
//  what the SD-90 has there instead: the MR3 by its driver's shape, and a
//  command port -- write the command to +8, read +8 back for 1, wait for
//  a result code at +13 and take the reply from +16 -- which is where the
//  machine now stops
//-------------------------------------------------

template <int Device>
u8 sd90_state::sd90_area6_r(offs_t offset)
{
	if (!machine().side_effects_disabled())
		LOGMASKED(LOG_UIPC, "%s: %s read %x\n", machine().describe_context(), Device ? "mailbox" : "mr3", offset);
	return 0;
}

template <int Device>
void sd90_state::sd90_area6_w(offs_t offset, u8 data)
{
	LOGMASKED(LOG_UIPC, "%s: %s write %x = %02x\n", machine().describe_context(), Device ? "mailbox" : "mr3", offset, data);
}


//-------------------------------------------------
//  the CPU's own bus, as the boot block declares it and the program's own
//  literals reach it.  Areas 5 and 6 are declared PCMCIA space, which is
//  how the boards' chip selects are wired rather than a card anywhere.
//-------------------------------------------------

void sd90_state::sd80_map(address_map &map)
{
	map(0x00000000, 0x001fffff).rom().region("progrom", 0);         // area 0, IC3
	map(0x08000000, 0x083fffff).ram();                              // area 2, IC4 and IC6
//  map(0x0c000000, 0x0fffffff)                                     // area 3, declared SDRAM and unpopulated
//  map(0x10000000, 0x13ffffff)                                     // area 4, which nothing reaches
	map(0x14000000, 0x140001ff).rw(m_xv[0], FUNC(roland_xv_device::read), FUNC(roland_xv_device::write));    // area 5, IC19
	map(0x15000000, 0x150001ff).rw(m_xv[1], FUNC(roland_xv_device::read), FUNC(roland_xv_device::write));    // IC27
	map(0x18000000, 0x18000003).rw(FUNC(sd90_state::uipc_r<0>), FUNC(sd90_state::uipc_w<0>));   // area 6, XS0
	map(0x19000000, 0x19000003).rw(FUNC(sd90_state::uipc_r<1>), FUNC(sd90_state::uipc_w<1>));   // XS1
}

void sd90_state::sd90_map(address_map &map)
{
	map(0x00000000, 0x002fffff).rom().region("progrom", 0);
	map(0x08000000, 0x083fffff).ram();
	map(0x14000000, 0x140001ff).rw(m_xv[0], FUNC(roland_xv_device::read), FUNC(roland_xv_device::write));
	map(0x15000000, 0x150001ff).rw(m_xv[1], FUNC(roland_xv_device::read), FUNC(roland_xv_device::write));
	map(0x18000000, 0x1800003f).rw(FUNC(sd90_state::sd90_area6_r<0>), FUNC(sd90_state::sd90_area6_w<0>));    // the MR3
	map(0x18800000, 0x1880003f).rw(FUNC(sd90_state::sd90_area6_r<1>), FUNC(sd90_state::sd90_area6_w<1>));    // its mailbox
}


void sd90_state::sd80_io_map(address_map &map)
{
	map(SH3_PORT_SC, SH3_PORT_SC + 7).r(FUNC(sd90_state::scp_r));
}


//-------------------------------------------------
//  the wave memory as both chips see it: the two mask ROMs at cell 0, two
//  bytes a cell, and no expansion socket on either machine
//-------------------------------------------------

void sd90_state::xv_wave_map(address_map &map)
{
	map(0x00000000, 0x00ffffff).rom().region("waverom", 0);
}


void sd90_state::common(machine_config &config)
{
	// the HD6417706 the SD-80 carries has no device of its own here, and
	// which SH-3 the SD-90 has is unread; EXTAL x 4 in the chip's clock
	// mode 1, then x 2 for the CPU out of the boot block's FRQCR
	SH7709(config, m_maincpu, 24_MHz_XTAL / 2 * 8, ENDIANNESS_BIG);

	// OUTPUT 1 and OUTPUT 2, one PCM1716E each; which chip's DAC pairs they
	// are, and which way the transport link runs, is unread, so the XV-5080's
	// arrangement stands in
	SPEAKER(config, "out1", 2).front();
	SPEAKER(config, "out2", 2).front();

	ROLAND_XV(config, m_xv[0], 16.9344_MHz_XTAL);   // IC19 on the SD-80, with IC26 its effect DRAM
	m_xv[0]->set_addrmap(roland_xv_device::AS_WAVE, &sd90_state::xv_wave_map);
	m_xv[0]->int_callback().set_inputline(m_maincpu, 0);    // IRQ0
	m_xv[0]->set_link(m_xv[1]);
	m_xv[0]->add_route(0, "out1", 1.0, 0);
	m_xv[0]->add_route(1, "out1", 1.0, 1);
	m_xv[0]->add_route(2, "out2", 1.0, 0);
	m_xv[0]->add_route(3, "out2", 1.0, 1);

	ROLAND_XV(config, m_xv[1], 16.9344_MHz_XTAL);   // IC27, with IC30
	m_xv[1]->set_addrmap(roland_xv_device::AS_WAVE, &sd90_state::xv_wave_map);
	m_xv[1]->int_callback().set_inputline(m_maincpu, 1);    // IRQ1
}

void sd90_state::sd80(machine_config &config)
{
	common(config);
	m_maincpu->set_addrmap(AS_PROGRAM, &sd90_state::sd80_map);

	// the display is on IC19's own LCD pins, LP0-LP7 with RS and LE, and
	// the CPU's D/A channel 1 sets its contrast, which is not modelled
	screen_device &screen(SCREEN(config, "screen"));
	screen.set_lcd();
	screen.set_refresh_hz(60);
	screen.set_screen_update("lcd", FUNC(hd44780_device::screen_update));
	screen.set_size(6 * 20, 9 * 2);
	screen.set_visarea_full();
	screen.set_palette("palette");
	PALETTE(config, "palette", FUNC(sd90_state::lcd_palette), 2);

	HD44780(config, m_lcd, 270'000);
	m_lcd->set_lcd_size(2, 20);
	m_xv[0]->lcd_callback().set(m_lcd, FUNC(hd44780_device::write));

	m_maincpu->set_addrmap(AS_IO, &sd90_state::sd80_io_map);
	m_xv[0]->switch_callback().set_ioport("PANEL");
	m_xv[0]->led_callback().set(FUNC(sd90_state::led_w));

	// which of the two jacks the SCIF is has not been read; the other is
	// the SCI at 0xfffffe80, which this core carries as registers only
	midi_port_device &mdin(MIDI_PORT(config, "mdin", midiin_slot, "midiin"));
	mdin.rxd_handler().set(m_maincpu->scif(), FUNC(sh3_scif_device::rxd_w));
	MIDI_PORT(config, "mdout", midiout_slot, "midiout");
	m_maincpu->scif().txd_handler().set("mdout", FUNC(midi_port_device::write_txd));
}

void sd90_state::sd90(machine_config &config)
{
	common(config);
	m_maincpu->set_addrmap(AS_PROGRAM, &sd90_state::sd90_map);

	// the IrDA channel and the SCIF, which way round is unread
	midi_port_device &mdin1(MIDI_PORT(config, "mdin1", midiin_slot, "midiin"));
	mdin1.rxd_handler().set(m_maincpu->irda(), FUNC(sh3_scif_device::rxd_w));
	MIDI_PORT(config, "mdout1", midiout_slot, "midiout");
	m_maincpu->irda().txd_handler().set("mdout1", FUNC(midi_port_device::write_txd));

	midi_port_device &mdin2(MIDI_PORT(config, "mdin2", midiin_slot, "midiin"));
	mdin2.rxd_handler().set(m_maincpu->scif(), FUNC(sh3_scif_device::rxd_w));
	MIDI_PORT(config, "mdout2", midiout_slot, "midiout");
	m_maincpu->scif().txd_handler().set("mdout2", FUNC(midi_port_device::write_txd));
}


static INPUT_PORTS_START(sd80)
	PORT_START("PANEL")   // strobe times eight plus line, as the chip numbers them
	PORT_BIT(0x00000002, IP_ACTIVE_HIGH, IPT_BUTTON1) PORT_NAME("Cursor Left") PORT_CODE(KEYCODE_LEFT)
	PORT_BIT(0x00000008, IP_ACTIVE_HIGH, IPT_BUTTON1) PORT_NAME("Inst/Drum") PORT_CODE(KEYCODE_I)
	PORT_BIT(0x00000100, IP_ACTIVE_HIGH, IPT_BUTTON1) PORT_NAME("Enter") PORT_CODE(KEYCODE_ENTER)
	PORT_BIT(0x00000200, IP_ACTIVE_HIGH, IPT_BUTTON1) PORT_NAME("Cursor Right") PORT_CODE(KEYCODE_RIGHT)
	PORT_BIT(0x00000400, IP_ACTIVE_HIGH, IPT_BUTTON1) PORT_NAME("Part Down") PORT_CODE(KEYCODE_OPENBRACE)
	PORT_BIT(0x00000800, IP_ACTIVE_HIGH, IPT_BUTTON1) PORT_NAME("Effects") PORT_CODE(KEYCODE_E)
	PORT_BIT(0x00010000, IP_ACTIVE_HIGH, IPT_BUTTON1) PORT_NAME("Preview") PORT_CODE(KEYCODE_SPACE)
	PORT_BIT(0x00020000, IP_ACTIVE_HIGH, IPT_BUTTON1) PORT_NAME("Shift") PORT_CODE(KEYCODE_LSHIFT)
	PORT_BIT(0x00040000, IP_ACTIVE_HIGH, IPT_BUTTON1) PORT_NAME("Part Up") PORT_CODE(KEYCODE_CLOSEBRACE)
	PORT_BIT(0x00080000, IP_ACTIVE_HIGH, IPT_BUTTON1) PORT_NAME("System") PORT_CODE(KEYCODE_S)

	PORT_START("DIAL")
	PORT_BIT(0xff, 0x00, IPT_DIAL) PORT_NAME("Value") PORT_SENSITIVITY(25) PORT_KEYDELTA(2)
INPUT_PORTS_END


static INPUT_PORTS_START(sd90)
INPUT_PORTS_END


//-------------------------------------------------
//  ROM definitions.  Neither wave ROM is dumped.
//-------------------------------------------------

#define ROM_WAVEROM \
	ROM_REGION16_LE(0x2000000, "waverom", ROMREGION_ERASE00) \
	ROM_LOAD("upd23c128040algy525mjh.ic28", 0x0000000, 0x1000000, NO_DUMP) \
	ROM_LOAD("upd23c128040algy526mkh.ic29", 0x1000000, 0x1000000, NO_DUMP)

ROM_START(sd80)
	// the flash read off IC3, the two bytes of every word exchanged
	ROM_REGION64_BE(0x200000, "progrom", 0)
	ROM_LOAD16_WORD_SWAP("sd80_v1.01.ic3", 0x000000, 0x200000, CRC(e0cfce2e) SHA1(8fcf6df19cf0c32bc07e464fdc77fd31a02e6c32))

	ROM_WAVEROM
ROM_END

ROM_START(sd90)
	// out of Roland's own updater, the f00001.mid off the SD-90 Version
	// Updating Disk, which carries the flash as Roland exclusive: a boot
	// block, the program in two zlib containers, the effect data and the
	// rest left erased
	ROM_REGION64_BE(0x300000, "progrom", 0)
	ROM_LOAD("sd90_v1.03.bin", 0x000000, 0x300000, CRC(e44fb343) SHA1(447cc7edabedfe3fb4cb7c280f44eb6eca50d7fe))

	ROM_WAVEROM
ROM_END

} // anonymous namespace


//    YEAR  NAME  PARENT  COMPAT  MACHINE  INPUT  CLASS       INIT        COMPANY   FULLNAME  FLAGS
SYST( 2001, sd90, 0,      0,      sd90,    sd90,  sd90_state, empty_init, "Roland", "SD-90 Studio Canvas", MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
SYST( 2002, sd80, 0,      0,      sd80,    sd80,  sd90_state, empty_init, "Roland", "SD-80 Studio Canvas", MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
