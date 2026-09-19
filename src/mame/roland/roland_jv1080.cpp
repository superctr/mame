// license:BSD-3-Clause
// copyright-holders:superctr
/****************************************************************************

    Roland JV-1080 64 Voice Synthesizer Module.

****************************************************************************/

#include "emu.h"

#include "cpu/sh/sh7032.h"
#include "sound/roland_xp.h"

#include "speaker.h"


namespace {

class roland_jv1080_state : public driver_device
{
public:
	roland_jv1080_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_xp(*this, "xp")
	{
	}

	void jv1080(machine_config &config);

private:
	void jv1080_map(address_map &map) ATTR_COLD;
	void xp_rom_map(address_map &map) ATTR_COLD;

	u8 ga_r(offs_t offset);
	void ga_w(offs_t offset, u8 data);

	required_device<sh7034_device> m_maincpu;
	required_device<roland_xp_device> m_xp;
};


u8 roland_jv1080_state::ga_r(offs_t offset)
{
	if (!machine().side_effects_disabled())
		logerror("%s: gate array read %02x\n", machine().describe_context(), offset);
	return 0;
}

void roland_jv1080_state::ga_w(offs_t offset, u8 data)
{
	logerror("%s: gate array write %02x = %02x\n", machine().describe_context(), offset, data);
}


void roland_jv1080_state::jv1080_map(address_map &map)
{
	map(0x01000000, 0x0101ffff).ram();
	map(0x02000000, 0x020fffff).rom().region("progrom", 0);
	map(0x02380000, 0x0238ffff).ram();
	map(0x04000000, 0x04003fff).rw(m_xp, FUNC(roland_xp_device::read), FUNC(roland_xp_device::write));
	map(0x04380000, 0x0438003f).rw(FUNC(roland_jv1080_state::ga_r), FUNC(roland_jv1080_state::ga_w));
}

void roland_jv1080_state::xp_rom_map(address_map &map)
{
	map(0x0000000, 0x07fffff).rom().region("waverom", 0);
}


static INPUT_PORTS_START(jv1080)
INPUT_PORTS_END


void roland_jv1080_state::jv1080(machine_config &config)
{
	SH7034(config, m_maincpu, 20_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &roland_jv1080_state::jv1080_map);

	SPEAKER(config, "mix", 2).front();
	SPEAKER(config, "output1", 2).front();
	SPEAKER(config, "output2", 2).front();

	ROLAND_XP(config, m_xp, 24.576_MHz_XTAL);
	m_xp->set_addrmap(roland_xp_device::AS_WAVE, &roland_jv1080_state::xp_rom_map);
	m_xp->int_callback().set_inputline(m_maincpu, 14);
	m_xp->add_route(0, "mix", 1.0, 0);
	m_xp->add_route(1, "mix", 1.0, 1);
	m_xp->add_route(2, "output1", 1.0, 0);
	m_xp->add_route(3, "output1", 1.0, 1);
	m_xp->add_route(4, "output2", 1.0, 0);
	m_xp->add_route(5, "output2", 1.0, 1);
}


ROM_START(jv1080)
	ROM_REGION32_BE(0x10000, "maincpu", 0)
	ROM_LOAD("roland_r00677323_6437034c12f.ic15", 0x00000, 0x10000, CRC(609c8b8d) SHA1(075b4d60d14afa3ff3c825b8da5c8687aca84560))

	ROM_REGION32_BE(0x100000, "progrom", 0)
	ROM_LOAD("jv1080_prom.bin", 0x000000, 0x100000, CRC(0e26e075) SHA1(77f09efecf267def69b4e8f851cf124798347c03))

	ROM_REGION(0x800000, "waverom", 0)
	ROM_LOAD("jv1080_waverom1.bin", 0x000000, 0x200000, CRC(f965d95f) SHA1(6b9a177f3ac6560e27befbcb9e907cf369f0f28a))
	ROM_LOAD("jv1080_waverom2.bin", 0x200000, 0x200000, CRC(78c3b1d8) SHA1(0dffa4ab4b9ea86c338fe146d902810f66c36cfd))
	ROM_LOAD("jv1080_waverom3.bin", 0x400000, 0x200000, CRC(edb5f1aa) SHA1(21eff48c6efe434ffce89c3bc61fe2b6d1584d00))
	ROM_LOAD("jv1080_waverom4.bin", 0x600000, 0x200000, CRC(4a877d28) SHA1(a1732be64e9c86c559df9c715703bcabc2a83440))
ROM_END

} // anonymous namespace


SYST(1994, jv1080, 0, 0, jv1080, jv1080, roland_jv1080_state, empty_init, "Roland", "JV-1080", MACHINE_NOT_WORKING | MACHINE_NO_SOUND)
