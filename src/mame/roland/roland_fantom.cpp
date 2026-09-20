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

    CN7 is the SR-JV80 socket and CN10 and CN11 the two SRX, which the test
    mode calls slots A, B and C.

    The wave mask ROMs are the XV-3080's and the XV-5080's, the same two
    Roland stock numbers (02010023 and 02010056) in all three machines, and
    the machine's own wave list and sample records -- which its firmware
    carries in the clear -- are those machines' byte for byte.

    State: the boot block runs, sets the bus up, inflates the program into
    the SDRAM and enters it.  It then spins in a loop that waits on a queue
    an interrupt has to fill, so the next thing this wants is the interrupt
    sources: the key scan at IC24, the tone generator, and whatever else
    area 6 carries.

****************************************************************************/

#include "emu.h"

#include "cpu/sh/sh4.h"
#include "sound/roland_xv.h"
#include "wavecard.h"

#include "speaker.h"


namespace {

class fantom_state : public driver_device
{
public:
	fantom_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_xv(*this, "xv")
		, m_exp(*this, "expa")
		, m_srx(*this, "exp%c", 'b')
	{
	}

	void fantom(machine_config &config) ATTR_COLD;

private:
	void fantom_map(address_map &map) ATTR_COLD;
	void xv_wave_map(address_map &map) ATTR_COLD;

	required_device<sh7709_device> m_maincpu;
	required_device<roland_xv_device> m_xv;
	required_device<srjv80_slot_device> m_exp;
	required_device_array<srx_slot_device, 2> m_srx;
};


//-------------------------------------------------
//  the CPU's own bus.  The boot block sets the bus controller up and the
//  areas it declares are the map: area 0 the flash, area 3 the SDRAM,
//  area 4 the XV, area 5 the floppy controller and area 6 the panel and
//  the display.  Only the first two are settled; the rest are where the
//  firmware's own literals reach, and what sits at each address in areas
//  4 and 6 is unread.
//-------------------------------------------------

void fantom_state::fantom_map(address_map &map)
{
	map(0x00000000, 0x003fffff).rom().region("progrom", 0);
	map(0x0c000000, 0x0cffffff).ram();
	map(0x10000000, 0x100001ff).rw(m_xv, FUNC(roland_xv_device::read), FUNC(roland_xv_device::write));
	map(0x14000000, 0x14000007).noprw();    // the FDC's eight byte registers
	map(0x18000000, 0x1800ffff).noprw();    // the panel, and the LCD controller at 0xd000
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

	SRJV80_SLOT(config, m_exp, 0);      // CN7, slot A
	for (auto &srx : m_srx)             // CN10 and CN11, slots B and C
		SRX_SLOT(config, srx, 0);

	// OUTPUT A and OUTPUT B, one AK4393 each
	SPEAKER(config, "outa", 2).front();
	SPEAKER(config, "outb", 2).front();

	ROLAND_XV(config, m_xv, 16.9344_MHz_XTAL);
	m_xv->set_addrmap(roland_xv_device::AS_WAVE, &fantom_state::xv_wave_map);
	m_xv->add_route(0, "outa", 1.0, 0);
	m_xv->add_route(1, "outa", 1.0, 1);
	m_xv->add_route(2, "outb", 1.0, 0);
	m_xv->add_route(3, "outb", 1.0, 1);
}


static INPUT_PORTS_START(fantom)
INPUT_PORTS_END


ROM_START(fantom)
	// out of Roland's own updater, Fantom118_SMF.exe: a boot block, then
	// the program in two zlib containers, and the top half left erased
	ROM_REGION64_BE(0x400000, "progrom", 0)
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
SYST( 2001, fantom, 0,      0,      fantom,  fantom, fantom_state, empty_init, "Roland", "Fantom (FA-76)", MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
