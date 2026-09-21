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

    The bus, as the boot block declares it and the program's own literals
    reach it: area 0 the flash, areas 2 and 3 synchronous DRAM with the 4 MB
    on area 2, area 5 sixteen bits wide with one XV at 0x14000000 and the
    other at 0x15000000, and area 6 eight bits wide, carrying a byte channel
    the firmware polls for a status at offset 2 and moves bytes through at
    offset 0.  The SD-80 gives that channel two windows, 0x18000000 and
    0x19000000, and the SD-90 one at 0x18000000 and a second device at
    0x18800000; what they are is unread, and the USB block is the candidate.
    Nothing reaches area 4.  Areas 5 and 6 are declared PCMCIA space, which
    is how the boards' chip selects are wired rather than a card anywhere.

    The interrupts, read out of the firmware's own dispatch table -- it
    indexes on INTEVT2 and every entry here has a handler of its own:

        SD-80   TMU0, TMU1, TMU2, the watchdog's interval timer, IRQ0 the
                XV at 0x14000000, IRQ1 the XV at 0x15000000, IRQ2 and IRQ3
                the byte channel's two windows, IRQ5 a pin read back on
                SCPDR, the SCI and the SCIF
        SD-90   TMU0, TMU1, the watchdog, IRQ0 and IRQ1 the two XVs, IRQ2
                the byte channel, PINT0-7, the IrDA channel and the SCIF

    So both machines' MIDI is two channels of the CPU's own: the SD-80 puts
    one on the SCI at 0xfffffe80, which this core carries as registers only,
    and the other on the SCIF; the SD-90 uses the IrDA channel and the SCIF.

    The wave mask ROMs are a set of the machines' own -- not the XV-3080 and
    XV-5080 set the Fantom shares, which the two firmwares' own wave list and
    sample records rule out -- and both are undumped, so there is nothing for
    the tone generators to play.

    State: a skeleton, and each machine stops at the first thing it is owed.
    The SD-80 runs its boot block, inflates the program into the SDRAM at
    0x08001000, enters it and then waits for the byte channel's read window
    to present a byte.  The SD-90's boot block copies its loader to the top
    of the SDRAM and waits there for bit 13 of the tone generator's word 0x24
    -- a word the XV-5080's firmware only ever writes -- before it will
    inflate anything.

****************************************************************************/

#include "emu.h"

#include "bus/midi/midiinport.h"
#include "bus/midi/midioutport.h"
#include "cpu/sh/sh3_scif.h"
#include "cpu/sh/sh4.h"
#include "sound/roland_xv.h"

#include "speaker.h"

#define LOG_LINK    (1U << 1)

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
	{
	}

	void sd80(machine_config &config) ATTR_COLD;
	void sd90(machine_config &config) ATTR_COLD;

protected:
	void common(machine_config &config) ATTR_COLD;

	void sd80_map(address_map &map) ATTR_COLD;
	void sd90_map(address_map &map) ATTR_COLD;
	void xv_wave_map(address_map &map) ATTR_COLD;

	template <int Window> u8 link_r(offs_t offset);
	template <int Window> void link_w(offs_t offset, u8 data);

	required_device<sh7709_device> m_maincpu;
	required_device_array<roland_xv_device, 2> m_xv;
};


//-------------------------------------------------
//  the byte channel on area 6: a status at offset 2, whose bit 1 the
//  firmware waits to see clear before it writes, and the bytes themselves
//  at offset 0.  Logged and nothing else.
//-------------------------------------------------

template <int Window>
u8 sd90_state::link_r(offs_t offset)
{
	if (!machine().side_effects_disabled())
		LOGMASKED(LOG_LINK, "%s: link %d read %x\n", machine().describe_context(), Window, offset);
	return 0;
}

template <int Window>
void sd90_state::link_w(offs_t offset, u8 data)
{
	LOGMASKED(LOG_LINK, "%s: link %d write %x = %02x\n", machine().describe_context(), Window, offset, data);
}


//-------------------------------------------------
//  the CPU's own bus, by area: the flash, the SDRAM, the two tone
//  generators and the byte channel
//-------------------------------------------------

void sd90_state::sd80_map(address_map &map)
{
	map(0x00000000, 0x001fffff).rom().region("progrom", 0);
	map(0x08000000, 0x083fffff).ram();
	map(0x14000000, 0x140001ff).rw(m_xv[0], FUNC(roland_xv_device::read), FUNC(roland_xv_device::write));
	map(0x15000000, 0x150001ff).rw(m_xv[1], FUNC(roland_xv_device::read), FUNC(roland_xv_device::write));
	map(0x18000000, 0x18000003).rw(FUNC(sd90_state::link_r<0>), FUNC(sd90_state::link_w<0>));
	map(0x19000000, 0x19000003).rw(FUNC(sd90_state::link_r<1>), FUNC(sd90_state::link_w<1>));
}

void sd90_state::sd90_map(address_map &map)
{
	map(0x00000000, 0x002fffff).rom().region("progrom", 0);
	map(0x08000000, 0x083fffff).ram();
	map(0x14000000, 0x140001ff).rw(m_xv[0], FUNC(roland_xv_device::read), FUNC(roland_xv_device::write));
	map(0x15000000, 0x150001ff).rw(m_xv[1], FUNC(roland_xv_device::read), FUNC(roland_xv_device::write));
	map(0x18000000, 0x18000003).rw(FUNC(sd90_state::link_r<0>), FUNC(sd90_state::link_w<0>));
	map(0x18800000, 0x18800003).rw(FUNC(sd90_state::link_r<1>), FUNC(sd90_state::link_w<1>));
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

	// MIDI 1 is the SCI at 0xfffffe80, which this core does not carry as a
	// serial device; MIDI 2 is the SCIF
	midi_port_device &mdin2(MIDI_PORT(config, "mdin2", midiin_slot, "midiin"));
	mdin2.rxd_handler().set(m_maincpu->scif(), FUNC(sh3_scif_device::rxd_w));
	MIDI_PORT(config, "mdout2", midiout_slot, "midiout");
	m_maincpu->scif().txd_handler().set("mdout2", FUNC(midi_port_device::write_txd));
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


static INPUT_PORTS_START(sd90)
INPUT_PORTS_END


//-------------------------------------------------
//  ROM definitions.  Neither wave ROM is dumped, and the descrambled
//  XV-3080/XV-5080 set is not theirs to stand in with: no sample record of
//  these machines' 2226 is one of that set's 3111, and only 77 of their 589
//  wave names appear in its 1083.
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
SYST( 2002, sd80, 0,      0,      sd80,    sd90,  sd90_state, empty_init, "Roland", "SD-80 Studio Canvas", MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
