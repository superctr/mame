// license:BSD-3-Clause
// copyright-holders:superctr
// thanks-to: giulioz
/*
 * Roland CSP (Toshiba TC6088AF), microcoded effects processor.
 *
 * One sample clocks up to 1024 instructions, with parallel coefficient
 * memory, two 30-bit accumulators, a 512-word internal delay ring and
 * external delay RAM. Audio crosses the SC parallel and TR serial buses;
 * the board pumps the device with run_once(). The scheduler is idle.
 *
 * The debugger's 64-bit program words combine PRAM in bits 23:0 and
 * CRAM in bits 47:32. The data space also exposes CRAM independently.
 * Host accesses use the JD-990's most-significant-byte-first strap.
 *
 * Unverified: sample budget, jump timing, bit 18, SC channel order,
 * configuration bits and physical external-memory transfers.
 */

#include "emu.h"
#include "roland_csp.h"
#include "roland_cspd.h"

DEFINE_DEVICE_TYPE(ROLAND_CSP, roland_csp_device, "roland_csp", "Roland CSP")

roland_csp_device::roland_csp_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: cpu_device(mconfig, ROLAND_CSP, tag, owner, clock)
	, m_program_config("program", ENDIANNESS_BIG, 64, 10, -3, address_map_constructor(FUNC(roland_csp_device::program_map), this))
	, m_coefficient_config("coefficients", ENDIANNESS_BIG, 16, 10, -1, address_map_constructor(FUNC(roland_csp_device::coefficient_map), this))
{
}

void roland_csp_device::program_map(address_map &map)
{
	map(0, 0x3ff).rw(FUNC(roland_csp_device::program_r), FUNC(roland_csp_device::program_w));
}

void roland_csp_device::coefficient_map(address_map &map)
{
	map(0, 0x3ff).rw(FUNC(roland_csp_device::coefficient_r), FUNC(roland_csp_device::coefficient_w));
}

device_memory_interface::space_config_vector roland_csp_device::memory_space_config() const
{
	return { { AS_PROGRAM, &m_program_config }, { AS_DATA, &m_coefficient_config } };
}

std::unique_ptr<util::disasm_interface> roland_csp_device::create_disassembler()
{
	return std::make_unique<roland_csp_disassembler>();
}

void roland_csp_device::device_start()
{
	m_eram = make_unique_clear<s32[]>(ERAM_SIZE);
	set_icountptr(m_icount);
	state_add(STATE_GENPC, "GENPC", m_pc).noshow();
	state_add(STATE_GENPCBASE, "CURPC", m_pc).noshow();
	state_add(0, "PC", m_pc);
	state_add(1, "ACCA", m_acc[0]);
	state_add(2, "ACCB", m_acc[1]);
	state_add(3, "IRAMP", m_iram_pos);
	state_add(4, "ERAMP", m_eram_pos);
	state_add(5, "MUL0", m_multiplier[0]);
	state_add(6, "MUL1", m_multiplier[1]);
	state_add(7, "TAP", m_tap);
	state_add(8, "SLOT", m_slot);
	save_pointer(NAME(m_eram), ERAM_SIZE);
	save_item(NAME(m_program));
	save_item(NAME(m_coefficients));
	save_item(NAME(m_iram));
	save_item(NAME(m_acc));
	save_item(NAME(m_history));
	save_item(NAME(m_multiplier));
	save_item(NAME(m_tap));
	save_item(NAME(m_serial_in));
	save_item(NAME(m_serial_out));
	save_item(NAME(m_parallel_in));
	save_item(NAME(m_parallel_pos));
	save_item(NAME(m_iram_pos));
	save_item(NAME(m_eram_pos));
	save_item(NAME(m_eram_mask));
	save_item(NAME(m_eram_read));
	save_item(NAME(m_eram_write));
	save_item(NAME(m_eram_start));
	save_item(NAME(m_eram_active));
	save_item(NAME(m_eram_command));
	save_item(NAME(m_eram_address));
	save_item(NAME(m_eram_data));
	save_item(NAME(m_gate_age));
	save_item(NAME(m_gate_negative));
	save_item(NAME(m_configuration));
	save_item(NAME(m_enable));
	save_item(NAME(m_host_read));
	save_item(NAME(m_pc));
	save_item(NAME(m_slot));
	save_item(NAME(m_halted));
}

void roland_csp_device::device_reset()
{
	std::fill_n(m_program, PROGRAM_SIZE, 0);
	std::fill_n(m_coefficients, PROGRAM_SIZE, 0);
	std::fill_n(m_iram, IRAM_SIZE, 0);
	std::fill_n(m_eram.get(), ERAM_SIZE, 0);
	std::fill_n(m_serial_in, 32, 0);
	std::fill_n(m_serial_out, 32, 0);
	std::fill_n(m_parallel_in, 24, 0);
	for (auto &history : m_history)
		std::fill_n(history, 4, 0);
	m_acc[0] = m_acc[1] = 0;
	m_multiplier[0] = m_multiplier[1] = 0;
	m_eram_start[0] = m_eram_start[1] = 0;
	m_eram_active[0] = m_eram_active[1] = false;
	m_eram_command[0] = m_eram_command[1] = 0;
	m_eram_address[0] = m_eram_address[1] = 0;
	m_tap = m_eram_read = m_eram_write = m_eram_data = 0;
	m_parallel_pos = m_iram_pos = m_eram_pos = 0;
	m_gate_age = 0;
	m_gate_negative = false;
	m_configuration = m_enable = m_host_read = 0;
	m_pc = m_slot = m_icount = 0;
	m_halted = true;
}

u64 roland_csp_device::program_r(offs_t offset)
{
	return m_program[offset & 0x3ff] | (u64(m_coefficients[offset & 0x3ff]) << 32);
}

void roland_csp_device::program_w(offs_t offset, u64 data, u64 mem_mask)
{
	const u64 value = (program_r(offset) & ~mem_mask) | (data & mem_mask);
	m_program[offset & 0x3ff] = value & 0xffffff;
	m_coefficients[offset & 0x3ff] = value >> 32;
}

u16 roland_csp_device::coefficient_r(offs_t offset)
{
	return m_coefficients[offset & 0x3ff];
}

void roland_csp_device::coefficient_w(offs_t offset, u16 data, u16 mem_mask)
{
	u16 &value = m_coefficients[offset & 0x3ff];
	value = (value & ~mem_mask) | (data & mem_mask);
}

u8 roland_csp_device::host_r(offs_t offset)
{
	offset &= 0x3fff;
	if (offset >= 1 && offset <= 3)
		return m_host_read >> ((3 - offset) * 8);
	return 0;
}

void roland_csp_device::host_w(offs_t offset, u8 data)
{
	offset &= 0x3fff;
	if (offset < 0x800)
	{
		const unsigned shift = BIT(offset, 0) ? 0 : 8;
		coefficient_w(offset >> 1, u16(data) << shift, 0xff << shift);
	}
	else if (offset >= 0x805 && offset <= 0x807)
	{
		const unsigned shift = (offset - 0x805) * 8;
		m_configuration = (m_configuration & ~(0xff << shift)) | (u32(data) << shift);
	}
	else if (offset == 0x80b)
		m_enable = data;
	else if (offset >= 0x1000 && offset < 0x2000 && (offset & 3))
	{
		const unsigned shift = (3 - (offset & 3)) * 8;
		program_w((offset - 0x1000) >> 2, u64(data) << shift, u64(0xff) << shift);
	}
	else if (offset >= 0x2000 && offset < 0x2800)
		m_host_read = m_coefficients[(offset - 0x2000) >> 1];
	else if (offset >= 0x3000)
		m_host_read = m_program[(offset - 0x3000) >> 2];
}

void roland_csp_device::eram_clock(u32 word)
{
	const int assembling = int(m_pc) - m_eram_start[0];
	const int pending = int(m_pc) - m_eram_start[1];
	if (m_eram_active[1] && pending == 7)
		m_eram_data = m_eram_write;
	if (m_eram_active[1] && pending == 10)
	{
		const u32 address = m_eram_address[1] & m_eram_mask;
		if (m_eram_command[1] == 4)
			m_eram[address] = narrow(m_eram_data);
		else
			m_eram_read = m_eram[address];
		m_eram_active[1] = false;
	}
	if (m_eram_active[0] && assembling >= 1 && assembling <= 5)
		m_eram_address[0] += (word >> 20) << ((assembling - 1) * 4);
	if (m_eram_active[0] && assembling == 5)
	{
		u32 address = m_eram_address[0] & 0x3ffff;
		if (m_eram_command[0] == 8 || m_eram_command[0] == 12)
			address = (address & 1) + ((u32(m_tap) & 0xffffc00) >> 10);
		if (m_eram_command[0] != 8)
			address += m_eram_pos;
		m_eram_address[1] = address;
		m_eram_command[1] = m_eram_command[0];
		m_eram_start[1] = m_eram_start[0];
		m_eram_data = m_eram_write;
		m_eram_active[1] = true;
		m_eram_active[0] = false;
	}
	if (BIT(word, 19))
	{
		m_eram_active[0] = true;
		m_eram_start[0] = m_pc;
		m_eram_command[0] = (word >> 20) & 12;
		m_eram_address[0] = 0;
	}
}

s32 roland_csp_device::store(unsigned mode, unsigned offset, u16 coefficient)
{
	const unsigned address = (offset + m_iram_pos) & 0x1ff;
	if (!mode)
		return m_iram[address];
	if (mode == 1)
	{
		const s32 value = m_parallel_in[m_parallel_pos];
		m_parallel_pos = (m_parallel_pos + 1) % 24;
		return m_iram[address] = value;
	}
	const bool special = mode < 4;
	const unsigned acc = special ? ((offset >= 0x188 && offset <= 0x18f) || (offset >= 0x1d0 && offset <= 0x1ef)) : (mode & 1);
	const s32 raw = m_history[acc][2];
	s32 value = (mode == 3 || mode >= 6) ? saturate(raw) : narrow(raw);
	if (!special)
		return m_iram[address] = value;
	if (offset >= 0x172 && offset <= 0x177)
	{
		const bool condition[] = { true, value > 0, raw < -0x800000 || raw > 0x7fffff, value < 0, value == 0, value >= 0 };
		if (condition[offset - 0x172])
			m_pc = coefficient;
	}
	else if (offset >= 0x180 && offset <= 0x18f)
	{
		switch (offset & 7)
		{
		case 2: m_host_read = value & 0xffffff; break;
		case 3: m_eram_write = value; break;
		case 5:
			m_tap = raw;
			m_multiplier[0] = (value & 0x3ff) << 13;
			m_multiplier[1] = value;
			break;
		case 6: m_multiplier[0] = value; break;
		case 7: m_multiplier[1] = value; break;
		}
	}
	else if (offset >= 0x190 && offset <= 0x1af)
		value = m_serial_in[offset - 0x190];
	else if (offset >= 0x1b0 && offset <= 0x1ef)
		m_serial_out[(offset - 0x1b0) & 31] = value;
	else if (offset >= 0x1f0)
		value = m_iram[address] = m_eram_read;
	if (offset < 0x190 || (offset >= 0x1b0 && offset <= 0x1ef))
		value = mode == 3 ? saturate(m_history[0][2]) : narrow(m_history[0][2]);
	return value;
}

void roland_csp_device::multiply(unsigned opcode, unsigned shift, s32 operand, u16 coefficient)
{
	s32 factor = s16(coefficient);
	bool invert = false;
	if (opcode >= 12 && coefficient >= 1 && coefficient <= 7)
	{
		const s32 reg = m_multiplier[coefficient & 1];
		switch (coefficient)
		{
		case 1: factor = 0x7ff0; break;
		case 2: case 3:
			factor = 0x8000 + (~(reg < 0 ? ~reg : reg) >> 8) + (reg < 0);
			invert = reg < 0;
			break;
		case 4: case 5: factor = ~(reg >> 8); break;
		case 6: case 7: factor = reg >> 8; break;
		}
	}
	s64 product = s64(operand) * factor;
	if (invert)
		product = -product;
	const bool sign = (operand < 0) != (factor < 0);
	if (opcode == 4 || opcode == 5 || opcode == 8 || opcode == 9)
		product = sign ? ~product : product;
	else if (opcode == 6 || opcode == 7)
	{
		if (!product)
			product = sign ? -s64(0x4000000000) : s64(0x3fffffffff);
		else
			product = (product < 0 ? product : ~product) & 0x3fffffffff;
	}
	product >>= shift;
	if (opcode == 10 || opcode == 11)
		product >>= 15;
	const unsigned dest = opcode == 1 || opcode == 3 || opcode == 11 || opcode == 13 || opcode == 15;
	const bool replace = opcode == 2 || opcode == 3 || opcode == 4 || opcode == 6 || opcode == 8 || opcode == 12 || opcode == 13;
	const s64 total = product + (replace ? 0 : m_acc[dest]);
	m_acc[dest] = accumulator(total);
	if (opcode == 5)
	{
		m_gate_age = 1;
		m_gate_negative = total < 0;
	}
}

void roland_csp_device::step()
{
	const u32 word = m_program[m_pc];
	const u16 coefficient = m_coefficients[m_pc];
	eram_clock(word);
	m_pc++;
	if (m_gate_age && ++m_gate_age > 9)
		m_gate_age = 0;
	s32 operand = store((word >> 9) & 7, word & 0x1ff, coefficient);
	if ((word & 0x1ff) == 1)
		operand = 0x8000;
	else if ((word & 0x1ff) == 2)
		operand = 0x400000;
	if (m_gate_age && (m_gate_negative ? m_gate_age >= 6 && m_gate_age <= 9 : m_gate_age >= 4 && m_gate_age <= 5))
		operand = 0;
	static constexpr unsigned shift[] = { 15, 14, 13, 11 };
	multiply((word >> 12) & 15, shift[(word >> 16) & 3], operand, coefficient);
	for (unsigned n = 0; n < 2; n++)
	{
		for (unsigned h = 3; h; h--)
			m_history[n][h] = m_history[n][h - 1];
		m_history[n][0] = m_acc[n];
	}
	if (++m_slot == PROGRAM_SIZE || m_pc >= PROGRAM_SIZE)
		finish_sample();
}

void roland_csp_device::finish_sample()
{
	m_iram_pos = (m_iram_pos - 1) & 0x1ff;
	m_eram_pos = (m_eram_pos - 1) & (ERAM_SIZE - 1);
	m_pc = m_slot = m_parallel_pos = 0;
}

void roland_csp_device::execute_run()
{
	while (m_icount > 0)
	{
		if (m_halted || BIT(m_configuration, 0))
		{
			m_icount = 0;
			return;
		}
		debugger_instruction_hook(m_pc);
		step();
		m_icount--;
	}
}

void roland_csp_device::run_once(unsigned slots)
{
	m_halted = false;
	for (unsigned n = 0; !BIT(m_configuration, 0) && n < std::min(slots, PROGRAM_SIZE); n++)
	{
		m_icount = 1;
		execute_run();
		if (!m_slot)
			break;
	}
	if (m_slot)
		finish_sample();
	m_halted = true;
}
