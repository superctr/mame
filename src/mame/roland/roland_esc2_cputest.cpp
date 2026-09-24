// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Test-only machine for the Cortex-M3 core: the memories of the Roland
    ESC2 (MB8AA4181) with the Boutique application loaded where its boot
    ROM would copy it.  Nothing but memory is modelled; the undumped boot
    ROM is a stub whose API table at 0x02003000 points every entry at a
    function returning zero.  This exists to trace the core against a
    reference and is not a driver.

***************************************************************************/

#include "emu.h"

#include "cpu/armv7m/armv7m.h"


namespace {

class esc2_cputest_state : public driver_device
{
public:
	esc2_cputest_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_bootrom(*this, "bootrom")
		, m_sdram(*this, "sdram")
		, m_appli(*this, "appli")
	{
	}

	void esc2_cputest(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	required_device<cortex_m3_device> m_maincpu;
	required_shared_ptr<u32> m_bootrom;
	required_shared_ptr<u32> m_sdram;
	required_region_ptr<u32> m_appli;

	void mem_map(address_map &map) ATTR_COLD;
};

void esc2_cputest_state::mem_map(address_map &map)
{
	map(0x01000000, 0x0102ffff).ram();
	map(0x02000000, 0x02003fff).ram().share(m_bootrom);
	map(0x20000000, 0x2000ffff).ram();
	map(0x40008000, 0x4000a0ff).ram();
	map(0x40045000, 0x40045fff).ram();
	map(0x60000000, 0x61ffffff).ram().share(m_sdram);
}

void esc2_cputest_state::machine_start()
{
	std::copy_n(&m_appli[0], m_appli.length(), &m_sdram[0]);
	m_bootrom[0] = 0x47702000;
	std::fill_n(&m_bootrom[0x3000 / 4], 0x1000 / 4, 0x02000001);
}

void esc2_cputest_state::machine_reset()
{
	m_maincpu->set_state_int(armv7m_device::ARMV7M_SP, 0x01011490);
	m_maincpu->set_state_int(armv7m_device::ARMV7M_PC, 0x60000000);
	m_maincpu->set_state_int(armv7m_device::ARMV7M_XPSR, 0x01000000);
}

void esc2_cputest_state::esc2_cputest(machine_config &config)
{
	CORTEX_M3(config, m_maincpu, 1'000'000);
	m_maincpu->set_addrmap(AS_PROGRAM, &esc2_cputest_state::mem_map);
	m_maincpu->set_num_irq(160);
}

ROM_START(esc2cputest)
	ROM_REGION32_LE(0x188000, "appli", ROMREGION_ERASE00)
	ROM_LOAD("60000000_appli.bin", 0, 0x187064, CRC(efe8f787) SHA1(35d5faeeef1e4bbc7f112bbf34b3ead8437b0679))
ROM_END

} // anonymous namespace


SYST(2016, esc2cputest, 0, 0, esc2_cputest, 0, esc2_cputest_state, empty_init, "Roland", "ESC2 Cortex-M3 test (Boutique application, test only)", MACHINE_NOT_WORKING | MACHINE_NO_SOUND_HW)
