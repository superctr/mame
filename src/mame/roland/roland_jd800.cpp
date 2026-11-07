// license:BSD-3-Clause
// copyright-holders:AJR
/****************************************************************************

    Skeleton driver for Roland JD-800 synthesizer.

    The H8/532 runs with its on-chip ROM enabled, and that ROM is not
    dumped, so nothing boots yet.  The program ROM shows the MEQ on an
    8-bit bus at E000 and the effects DSP's host port at E010.

****************************************************************************/

#include "emu.h"
//#include "bus/midi/midi.h"
//#include "cpu/h8/h8330.h"
#include "cpu/h8500/h8532.h"
#include "sound/roland_meq.h"

#include "speaker.h"

#define LOG_IO (1U << 1)
#define VERBOSE (0)
#include "logmacro.h"
//#include "machine/nvram.h"
//#include "machine/ssc1000.h"


namespace {

class roland_jd800_state : public driver_device
{
public:
	roland_jd800_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_meq(*this, "meq")
	{
	}

	void jd800(machine_config &config);

private:
	void mem_map(address_map &map) ATTR_COLD;

	u8 dsp_r(offs_t offset);
	void dsp_w(offs_t offset, u8 data);
	u8 io_r(offs_t offset);
	void io_w(offs_t offset, u8 data);

	required_device<h8532_device> m_maincpu;
	required_device<roland_meq_device> m_meq;
};


u8 roland_jd800_state::dsp_r(offs_t offset)
{
	if(!machine().side_effects_disabled())
		LOGMASKED(LOG_IO, "dsp_r %04x (%x)\n", 0xe010 + offset, m_maincpu->pc());
	return 0;
}

void roland_jd800_state::dsp_w(offs_t offset, u8 data)
{
	LOGMASKED(LOG_IO, "dsp_w %04x = %02x (%x)\n", 0xe010 + offset, data, m_maincpu->pc());
}

u8 roland_jd800_state::io_r(offs_t offset)
{
	if(!machine().side_effects_disabled())
		LOGMASKED(LOG_IO, "io_r %04x (%x)\n", 0xf100 + offset, m_maincpu->pc());
	return 0;
}

void roland_jd800_state::io_w(offs_t offset, u8 data)
{
	LOGMASKED(LOG_IO, "io_w %04x = %02x (%x)\n", 0xf100 + offset, data, m_maincpu->pc());
}


void roland_jd800_state::mem_map(address_map &map)
{
	map(0x0e000, 0x0e00b).rw(m_meq, FUNC(roland_meq_device::read8), FUNC(roland_meq_device::write8));
	map(0x0e010, 0x0e017).rw(FUNC(roland_jd800_state::dsp_r), FUNC(roland_jd800_state::dsp_w));
	map(0x0f100, 0x0f11f).rw(FUNC(roland_jd800_state::io_r), FUNC(roland_jd800_state::io_w));
	map(0xc0000, 0xfffff).rom().region("progrom", 0);
}


static INPUT_PORTS_START(jd800)
INPUT_PORTS_END

void roland_jd800_state::jd800(machine_config &config)
{
	HD6435328(config, m_maincpu, 20_MHz_XTAL); // MD2 = VCC, MD1 = MD0 = GND
	m_maincpu->set_addrmap(AS_PROGRAM, &roland_jd800_state::mem_map);

	//SSC1000(config, "keybc", 20_MHz_XTAL / 2);

	//NVRAM(config, "nvram", nvram_device::DEFAULT_ALL_0); // HM62256LFP-12SLT (@ IC29) + CR2032 battery

	//H8330(config, "iomcu", 20_MHz_XTAL / 2).set_disable(); // MD0 = MD1 = +5V (single-chip mode)

	//MB87731A(config, "pcm", 26.195_MHz_XTAL);
	//MB87424A(config, "tvf", 26.195_MHz_XTAL);
	//TC23SC260AF(config, "dsp", 28.224_MHz_XTAL);

	SPEAKER(config, "speaker", 2).front();

	// the patch mode's program: its final left and right leave on the third serial port
	ROLAND_MEQ(config, m_meq, 28.224_MHz_XTAL / 640);
	m_meq->add_route(roland_meq_device::output(2, 0), "speaker", 1.0, 0);
	m_meq->add_route(roland_meq_device::output(2, 1), "speaker", 1.0, 1);
}

ROM_START(jd800)
	ROM_REGION(0x8000, "maincpu", ROMREGION_ERASE00)
	ROM_LOAD("r15199765.ic36", 0x0000, 0x8000, NO_DUMP)

	ROM_REGION(0x40000, "progrom", 0)
	ROM_LOAD("roland_r15209315_9119-d.ic31", 0x00000, 0x40000, CRC(04565c05) SHA1(08c0fdc79e6d8d1b5cd3261c977da0244428057c)) // version 1.01

	ROM_REGION(0x4000, "iomcu", ROMREGION_ERASE00) // H8/330
	ROM_LOAD("r15199742.ic5", 0x0000, 0x4000, NO_DUMP)

	ROM_REGION(0x300000, "pcm", 0)
	ROM_LOAD("roland_r15209290_mb838000-20_2a6-aa_9114-r11.ic25", 0x000000, 0x100000, CRC(d743cec8) SHA1(7ac9943f1769913d7bbf95c5cca76ca711179641))
	ROM_LOAD("roland_r15209291_mb838000-20_2a7-aa_9119-r15.ic26", 0x100000, 0x100000, CRC(06098658) SHA1(5c364e3b9c1b206fe94fbaa190caa17b01189937))
	ROM_LOAD("roland_r15209305_mb838000-20_2a8-aa_9102-r01.ic1",  0x200000, 0x100000, CRC(5c83e539) SHA1(2861baa9a6f42a1257c241927815ed366becd7dc)) // on card board
ROM_END

} // anonymous namespace


SYST(1991, jd800, 0, 0, jd800, jd800, roland_jd800_state, empty_init, "Roland", "JD-800 Programmable Synthesizer", MACHINE_NO_SOUND | MACHINE_NOT_WORKING)
