// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Test-only machines for the Cortex-M3 core.

    esc2cputest: the memories of the Roland ESC2 (MB8AA4181) with the
    Boutique application loaded where its boot ROM would copy it.  Nothing
    but memory is modelled; the undumped boot ROM is a stub whose API table
    at 0x02003000 points every entry at a function returning zero.

    armv7mfuzz: runs the vectors of mame/scripts/armv7m_fuzz.py on the
    core, the interpreter and the recompiler both, named by the environment
    variable ARMV7M_FUZZ ("MEMFILE VECFILE OUTFILE"), and exits.

    These exist to trace the core against a reference and are not drivers.

***************************************************************************/

#include "emu.h"

#include "cpu/armv7m/armv7m.h"

#include <array>
#include <bit>
#include <fstream>
#include <sstream>


DECLARE_DEVICE_TYPE(ARMV7M_FUZZ_CPU, armv7m_fuzz_cpu_device)

class armv7m_fuzz_cpu_device : public cortex_m3_device
{
public:
	armv7m_fuzz_cpu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
		: cortex_m3_device(mconfig, ARMV7M_FUZZ_CPU, tag, owner, clock, address_map_constructor())
	{
	}

	int run_cycles(int cycles)
	{
		icount() = cycles;
		run();
		return icount();
	}
};

DEFINE_DEVICE_TYPE(ARMV7M_FUZZ_CPU, armv7m_fuzz_cpu_device, "armv7m_fuzz_cpu", "ARM Cortex-M3 (fuzz runner)")


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



class armv7m_fuzz_state : public driver_device
{
public:
	armv7m_fuzz_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_cpu(*this, { "interp", "drc" })
	{
	}

	void armv7m_fuzz(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	struct region
	{
		u32 base, size;
		std::vector<u8> data;
	};

	struct vector_in
	{
		u32 magic;
		u32 steps;
		u32 code_addr;
		u32 code_len;
		u8 code[64];
		u32 npokes;
		u32 pokes[8][3];
		u32 regs[16];
		u32 xpsr, msp, psp, control, primask, basepri, faultmask;
	};

	struct result_out
	{
		u32 magic;
		u32 status;
		u32 log_mask;
		u32 regs[16];
		u32 xpsr, msp, psp, control, primask, basepri, faultmask;
		u32 cfsr, hfsr;
		u32 unmapped_read, unmapped_write;
		u32 frame[8];
		u32 nwrites;
		u32 writes[32][3];
	};

	required_device_array<armv7m_fuzz_cpu_device, 2> m_cpu;
	std::vector<region> m_regions;
	std::vector<std::pair<u32, u8>> m_undo;
	std::vector<std::array<u32, 3>> m_writes;
	bool m_record = false;
	bool m_any_unmapped_read = false, m_any_unmapped_write = false;
	u32 m_unmapped_read = 0, m_unmapped_write = 0;
	bool m_reset_request = false;
	std::string m_files;
	emu_timer *m_timer = nullptr;

	void mem_map(address_map &map) ATTR_COLD;
	u32 unmapped_r(offs_t offset, u32 mem_mask);
	void unmapped_w(offs_t offset, u32 data, u32 mem_mask);
	u8 *find(u32 address);
	void poke(u32 address, int size, u32 data);
	u32 peek(u32 address, int size);
	void restore();
	void setup(armv7m_fuzz_cpu_device &cpu, const vector_in &v);
	u32 read_scs(armv7m_fuzz_cpu_device &cpu, u32 address);
	result_out result(armv7m_fuzz_cpu_device &cpu);
	TIMER_CALLBACK_MEMBER(run_vectors);
};

void armv7m_fuzz_state::mem_map(address_map &map)
{
	map(0x00000000, 0xffffffff).rw(FUNC(armv7m_fuzz_state::unmapped_r), FUNC(armv7m_fuzz_state::unmapped_w));
}

u32 armv7m_fuzz_state::unmapped_r(offs_t offset, u32 mem_mask)
{
	if (!machine().side_effects_disabled() && !m_any_unmapped_read)
	{
		m_any_unmapped_read = true;
		m_unmapped_read = offset * 4 + std::countr_zero(mem_mask) / 8;
	}
	return 0;
}

void armv7m_fuzz_state::unmapped_w(offs_t offset, u32 data, u32 mem_mask)
{
	if (!m_any_unmapped_write)
	{
		m_any_unmapped_write = true;
		m_unmapped_write = offset * 4 + std::countr_zero(mem_mask) / 8;
	}
}

u8 *armv7m_fuzz_state::find(u32 address)
{
	for (auto &r : m_regions)
		if (address - r.base < r.size)
			return &r.data[address - r.base];
	return nullptr;
}

void armv7m_fuzz_state::poke(u32 address, int size, u32 data)
{
	for (int i = 0; i < size; i++)
	{
		u8 *const p = find(address + i);
		if (p)
		{
			m_undo.emplace_back(address + i, *p);
			*p = u8(data >> (8 * i));
		}
	}
}

u32 armv7m_fuzz_state::peek(u32 address, int size)
{
	u32 value = 0;
	for (int i = 0; i < size; i++)
	{
		u8 *const p = find(address + i);
		if (p)
			value |= u32(*p) << (8 * i);
	}
	return value;
}

void armv7m_fuzz_state::restore()
{
	for (auto it = m_undo.rbegin(); it != m_undo.rend(); ++it)
		*find(it->first) = it->second;
	m_undo.clear();
}

u32 armv7m_fuzz_state::read_scs(armv7m_fuzz_cpu_device &cpu, u32 address)
{
	u32 saved[16];
	for (int i = 0; i < 16; i++)
		saved[i] = cpu.state_int(armv7m_device::ARMV7M_R0 + i);
	const u32 xpsr = cpu.state_int(armv7m_device::ARMV7M_XPSR);
	const bool rec = m_record;
	m_record = false;
	poke(0xfff0, 2, 0x6808);
	cpu.set_state_int(armv7m_device::ARMV7M_R1, address);
	cpu.set_state_int(armv7m_device::ARMV7M_PC, 0xfff0);
	cpu.set_state_int(armv7m_device::ARMV7M_XPSR, 0x01000000);
	cpu.run_cycles(1);
	const u32 value = cpu.state_int(armv7m_device::ARMV7M_R0);
	for (int i = 0; i < 16; i++)
		cpu.set_state_int(armv7m_device::ARMV7M_R0 + i, saved[i]);
	cpu.set_state_int(armv7m_device::ARMV7M_XPSR, xpsr);
	m_record = rec;
	return value;
}

void armv7m_fuzz_state::setup(armv7m_fuzz_cpu_device &cpu, const vector_in &v)
{
	m_record = false;
	m_reset_request = false;
	cpu.reset();
	for (u32 i = 0; i < v.code_len && i < 64; i++)
		poke(v.code_addr + i, 1, v.code[i]);
	for (u32 i = 0; i < v.npokes && i < 8; i++)
	{
		const u32 addr = v.pokes[i][0], size = v.pokes[i][1], data = v.pokes[i][2];
		if (addr >= 0xe0000000)
		{
			poke(0xfff0, 2, size == 4 ? 0x6008 : size == 2 ? 0x8008 : 0x7008);
			cpu.set_state_int(armv7m_device::ARMV7M_R0, data);
			cpu.set_state_int(armv7m_device::ARMV7M_R1, addr);
			cpu.set_state_int(armv7m_device::ARMV7M_PC, 0xfff0);
			cpu.set_state_int(armv7m_device::ARMV7M_XPSR, 0x01000000);
			cpu.run_cycles(1);
		}
		else
			poke(addr, size, data);
	}

	cpu.set_state_int(armv7m_device::ARMV7M_CONTROL, v.control);
	cpu.set_state_int(armv7m_device::ARMV7M_MSP, v.msp);
	cpu.set_state_int(armv7m_device::ARMV7M_PSP, v.psp);
	for (int i = 0; i < 16; i++)
		cpu.set_state_int(armv7m_device::ARMV7M_R0 + i, v.regs[i]);
	cpu.set_state_int(armv7m_device::ARMV7M_XPSR, v.xpsr);
	cpu.set_state_int(armv7m_device::ARMV7M_PRIMASK, v.primask);
	cpu.set_state_int(armv7m_device::ARMV7M_BASEPRI, v.basepri);
	cpu.set_state_int(armv7m_device::ARMV7M_FAULTMASK, v.faultmask);

	m_writes.clear();
	m_any_unmapped_read = m_any_unmapped_write = false;
	m_unmapped_read = m_unmapped_write = 0;
	m_record = true;
}

armv7m_fuzz_state::result_out armv7m_fuzz_state::result(armv7m_fuzz_cpu_device &cpu)
{
	m_record = false;
	result_out r{};
	r.magic = 0x52534c54;
	for (int i = 0; i < 16; i++)
		r.regs[i] = cpu.state_int(armv7m_device::ARMV7M_R0 + i);
	r.xpsr = cpu.state_int(armv7m_device::ARMV7M_XPSR);
	r.msp = cpu.state_int(armv7m_device::ARMV7M_MSP);
	r.psp = cpu.state_int(armv7m_device::ARMV7M_PSP);
	r.control = cpu.state_int(armv7m_device::ARMV7M_CONTROL);
	r.primask = cpu.state_int(armv7m_device::ARMV7M_PRIMASK);
	r.basepri = cpu.state_int(armv7m_device::ARMV7M_BASEPRI);
	r.faultmask = cpu.state_int(armv7m_device::ARMV7M_FAULTMASK);
	r.status = (m_any_unmapped_read ? 1 : 0) | (m_any_unmapped_write ? 2 : 0) | (m_reset_request ? 4 : 0);
	r.unmapped_read = m_unmapped_read;
	r.unmapped_write = m_unmapped_write;
	for (int i = 0; i < 8; i++)
		r.frame[i] = peek(r.regs[13] + 4 * i, 4);
	r.cfsr = read_scs(cpu, 0xe000ed28);
	r.hfsr = read_scs(cpu, 0xe000ed2c);
	r.nwrites = std::min<u32>(m_writes.size(), 32);
	for (u32 i = 0; i < r.nwrites; i++)
		for (int k = 0; k < 3; k++)
			r.writes[i][k] = m_writes[i][k];
	return r;
}

TIMER_CALLBACK_MEMBER(armv7m_fuzz_state::run_vectors)
{
	std::istringstream args(m_files);
	std::string memfile, vecfile, outfile;
	args >> memfile >> vecfile >> outfile;
	std::ifstream vin(vecfile, std::ios::binary);
	std::ofstream vout(outfile, std::ios::binary);
	vector_in v;
	while (vin.read(reinterpret_cast<char *>(&v), sizeof(v)))
	{
		std::vector<int> cycles;
		setup(*m_cpu[0], v);
		for (u32 i = 0; i < v.steps; i++)
		{
			cycles.push_back(1 - m_cpu[0]->run_cycles(1));
			if (m_cpu[0]->state_int(armv7m_device::ARMV7M_XPSR) & 0x1ff)
				break;
		}
		m_record = false;
		restore();

		setup(*m_cpu[1], v);
		int budget = 1;
		for (size_t i = 0; i + 1 < cycles.size(); i++)
			budget += cycles[i];
		m_cpu[1]->run_cycles(budget);
		const result_out r = result(*m_cpu[1]);
		vout.write(reinterpret_cast<const char *>(&r), sizeof(r));
		restore();
	}
	vout.close();
	machine().schedule_exit();
}

void armv7m_fuzz_state::machine_start()
{
	m_timer = timer_alloc(FUNC(armv7m_fuzz_state::run_vectors), this);
	const char *const files = getenv("ARMV7M_FUZZ");
	m_files = files ? files : "";

	std::istringstream args(m_files);
	std::string memfile;
	args >> memfile;
	std::ifstream f(memfile, std::ios::binary);
	u32 hdr[2];
	while (f.read(reinterpret_cast<char *>(hdr), 8))
	{
		region r{ hdr[0], hdr[1], std::vector<u8>(hdr[1]) };
		f.read(reinterpret_cast<char *>(r.data.data()), hdr[1]);
		m_regions.push_back(std::move(r));
	}

	for (auto &cpu : m_cpu)
	{
		address_space &space = cpu->space(AS_PROGRAM);
		for (auto &r : m_regions)
			space.install_ram(r.base, r.base + r.size - 1, r.data.data());
		space.install_write_tap(0x00000000, 0xffffffff, "record",
				[this] (offs_t offset, u32 &data, u32 mem_mask)
				{
					const unsigned shift = std::countr_zero(mem_mask);
					const u32 address = offset + shift / 8;
					const int size = std::popcount(mem_mask) / 8;
					if (m_record)
						m_writes.push_back({ address, u32(size), (data & mem_mask) >> shift });
					for (int i = 0; i < size; i++)
					{
						u8 *const p = find(address + i);
						if (p)
							m_undo.emplace_back(address + i, *p);
					}
				});
	}
}

void armv7m_fuzz_state::machine_reset()
{
	for (auto &cpu : m_cpu)
		cpu->suspend(SUSPEND_REASON_DISABLE, true);
	if (!m_files.empty())
		m_timer->adjust(attotime::zero);
}

void armv7m_fuzz_state::armv7m_fuzz(machine_config &config)
{
	for (auto &cpu : m_cpu)
	{
		ARMV7M_FUZZ_CPU(config, cpu, 1'000'000);
		cpu->set_addrmap(AS_PROGRAM, &armv7m_fuzz_state::mem_map);
		cpu->sysresetreq_cb().set([this] (int state) { if (state) m_reset_request = true; });
	}
	m_cpu[0]->set_force_no_drc(true);
}


ROM_START(esc2cputest)
	ROM_REGION32_LE(0x188000, "appli", ROMREGION_ERASE00)
	ROM_LOAD("60000000_appli.bin", 0, 0x187064, CRC(efe8f787) SHA1(35d5faeeef1e4bbc7f112bbf34b3ead8437b0679))
ROM_END

ROM_START(armv7mfuzz)
ROM_END

} // anonymous namespace


SYST(2016, armv7mfuzz,  0, 0, armv7m_fuzz,  0, armv7m_fuzz_state,  empty_init, "Roland", "ARMv7-M core fuzz runner (test only)", MACHINE_NOT_WORKING | MACHINE_NO_SOUND_HW)
SYST(2016, esc2cputest, 0, 0, esc2_cputest, 0, esc2_cputest_state, empty_init, "Roland", "ESC2 Cortex-M3 test (Boutique application, test only)", MACHINE_NOT_WORKING | MACHINE_NO_SOUND_HW)
