// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Roland MR3 effect processor

***************************************************************************/

#include "emu.h"
#include "roland_mr3.h"

#include <algorithm>
#include <cmath>

#define LOG_HOST    (1U << 1)
#define LOG_DECODE  (1U << 2)

#define VERBOSE (LOG_GENERAL)
#include "logmacro.h"


DEFINE_DEVICE_TYPE(ROLAND_MR3, roland_mr3_device, "roland_mr3", "Roland MR3")

namespace {

constexpr int SHIFTS[4] = { 0, 1, 2, 6 };

constexpr s32 sext(u32 value, int bits)
{
	return s32(value << (32 - bits)) >> (32 - bits);
}

constexpr s32 coefficient(u32 word)
{
	return s32(u32(sext(word & 0xfff, 12)) << 12 << SHIFTS[(word >> 12) & 3]);
}

constexpr s32 special_coefficient(u32 word)
{
	return s32(u32(sext((word >> 8) & 15, 4)) << 20 << SHIFTS[(word >> 12) & 3]);
}

constexpr bool condition(int arg, s64 value)
{
	return (!BIT(arg, 0) || value >= 0) && (!BIT(arg, 1) || value < 0) && (!BIT(arg, 2) || value != 0);
}

} // anonymous namespace


roland_mr3_device::roland_mr3_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, ROLAND_MR3, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
	, m_stream(nullptr)
{
}

void roland_mr3_device::map(address_map &map)
{
	map(0x00, 0x03).rw(FUNC(roland_mr3_device::data_r), FUNC(roland_mr3_device::data_w));
	map(0x04, 0x05).w(FUNC(roland_mr3_device::command_w));
	map(0x06, 0x07).lw8(NAME([this] (offs_t offset, u8 data) { control_w(0x06 + offset, data); }));
	map(0x08, 0x0f).r(FUNC(roland_mr3_device::result_r));
	map(0x1a, 0x1a).lw8(NAME([this] (u8 data) { control_w(0x1a, data); }));
	map(0x1e, 0x1e).r(FUNC(roland_mr3_device::status_r));
}

void roland_mr3_device::device_start()
{
	m_stream = stream_alloc(LANES, LANES, clock() / CLOCKS_PER_SAMPLE);

	save_item(NAME(m_program));
	save_item(NAME(m_data));
	save_item(NAME(m_last_command));
	save_item(NAME(m_control));
	save_item(NAME(m_ring));
	save_item(NAME(m_eram));
	save_item(NAME(m_k));
	save_item(NAME(m_address));
	save_item(NAME(m_acc));
	save_item(NAME(m_history));
	save_item(NAME(m_history_pos));
	save_item(NAME(m_operand));
	save_item(NAME(m_ring_pos));
	save_item(NAME(m_eram_pos));
	save_item(NAME(m_second_pos));
	save_item(NAME(m_second));
	save_item(NAME(m_result));
	save_item(NAME(m_input));
	save_item(NAME(m_output));
	save_item(STRUCT_MEMBER(m_blank, row));
	save_item(STRUCT_MEMBER(m_blank, path));
	save_item(NAME(m_blank_count));
	save_item(STRUCT_MEMBER(m_pending, row));
	save_item(STRUCT_MEMBER(m_pending, path));
	save_item(STRUCT_MEMBER(m_pending, holds));
	save_item(NAME(m_pending_count));

	std::fill(std::begin(m_program), std::end(m_program), 0);
	for (int pc = 0; pc < ROWS; pc++)
		decode(pc);
}

void roland_mr3_device::device_reset()
{
	m_data = 0;
	m_last_command = 0;
	std::fill(std::begin(m_control), std::end(m_control), 0);
	std::fill(std::begin(m_ring), std::end(m_ring), 0);
	std::fill(std::begin(m_eram), std::end(m_eram), 0);
	std::fill(std::begin(m_k), std::end(m_k), 0);
	m_address = 0;
	m_acc[0] = m_acc[1] = 0;
	for (auto &h : m_history)
		h[0] = h[1] = 0;
	m_history_pos = 0;
	m_operand = 0;
	m_ring_pos = 0;
	m_eram_pos = 0;
	m_second_pos = 0;
	for (auto &s : m_second)
		std::fill(std::begin(s), std::end(s), 0);
	std::fill(std::begin(m_result), std::end(m_result), 0);
	std::fill(std::begin(m_input), std::end(m_input), 0);
	std::fill(std::begin(m_output), std::end(m_output), 0);
	m_blank_count = 0;
	m_pending_count = 0;
}

void roland_mr3_device::device_clock_changed()
{
	if (m_stream)
		m_stream->set_sample_rate(clock() / CLOCKS_PER_SAMPLE);
}

void roland_mr3_device::device_post_load()
{
	for (int pc = 0; pc < ROWS; pc++)
		decode(pc);
}


//-------------------------------------------------
//  host port
//-------------------------------------------------

u32 roland_mr3_device::data_r()
{
	return m_data;
}

void roland_mr3_device::data_w(offs_t offset, u32 data, u32 mem_mask)
{
	COMBINE_DATA(&m_data);
}

void roland_mr3_device::command_w(offs_t offset, u16 data, u16 mem_mask)
{
	m_stream->update();

	const u16 cmd = data & 0xfe00;
	const int addr = data & (ROWS - 1);
	m_last_command = data;

	switch (cmd)
	{
	case CMD_WRITE:
		m_program[addr] = m_data;
		break;

	case CMD_READ:
		m_data = m_program[addr];
		return;

	case CMD_COEFFICIENT:
		m_program[addr] = (m_program[addr] & ~0x3fffU) | (m_data & 0x3fff);
		break;

	case CMD_EXTENDED:
		m_program[addr] = (m_program[addr] & ~0x3fffU) | ((m_data >> 11) & 0x3fff);
		m_program[(addr + 1) & (ROWS - 1)] = (m_program[(addr + 1) & (ROWS - 1)] & ~0x7ffU) | (m_data & 0x7ff);
		decode((addr + 2) & (ROWS - 1));
		break;

	default:
		LOGMASKED(LOG_HOST, "%s: command %04x data %08x\n", machine().describe_context(), data, m_data);
		return;
	}

	decode(addr);
	decode((addr + 1) & (ROWS - 1));
}

u16 roland_mr3_device::result_r(offs_t offset)
{
	if (!machine().side_effects_disabled())
		m_stream->update();
	return u16(narrow(m_result[offset & (RESULTS - 1)]) >> 8);
}

u8 roland_mr3_device::status_r()
{
	return 0;
}

void roland_mr3_device::control_w(offs_t offset, u8 data)
{
	LOGMASKED(LOG_HOST, "%s: control %02x = %02x\n", machine().describe_context(), offset, data);
	m_control[offset & 0x1f] = data;
}


//-------------------------------------------------
//  decoding
//-------------------------------------------------

void roland_mr3_device::decode(int pc)
{
	const u32 word = m_program[pc];
	const int ctl = (word >> 22) & 31;
	const int cell = (word >> 14) & 255;
	const u32 op = word & 0x3fff;
	row &r = m_rows[pc];
	r = row{ OP_INVALID, u8(ctl >> 4), u8(cell), 0, false, false, false, false, coefficient(word), 0 };

	if (ctl == 0x0f)
	{
		const int arg = cell & 0x7f;
		r.path = cell >> 7;
		r.arg = arg;
		if (arg == 0x40)
		{
			r.kind = OP_CONTINUE;
			r.coefficient = s32((word & 0x7ff) << 1 << SHIFTS[(word >> 12) & 3]);
		}
		else if (op && arg < 8)
		{
			r.kind = OP_JUMP;
			r.target = op & (ROWS - 1);
		}
		else if (arg >= 0x30 && arg <= 0x37)
		{
			r.kind = OP_INPUT;
			r.arg = arg & 7;
		}
		else if (op)
			r.kind = OP_INVALID;
		else if (arg == 0x10 || arg == 0x11)
		{
			r.kind = OP_SEND;
			r.arg = arg & 1;
		}
		else if (arg >= 0x28 && arg <= 0x2b)
		{
			r.kind = OP_K_LOAD;
			r.arg = arg - 0x28;
		}
		else if (arg == 0x0a)
			r.kind = OP_ADDRESS;
		else if (arg >= 0x20 && arg <= 0x23)
		{
			r.kind = OP_HOST;
			r.arg = arg - 0x20;
		}
		else
			r.kind = OP_TRANSFER;
	}
	else if (ctl == 0x1f)
	{
		const int k = (op >> 4) & 3;
		const int nibble = op & 15;
		r.path = BIT(op, 7);
		r.arg = k;
		if (!BIT(op, 6))
		{
			if (nibble == 0xc || nibble == 0xe)
				r.kind = OP_NOP;
			else if (nibble <= 0xb && !(op & 0xf00))
			{
				r.kind = OP_K_FORM;
				r.start = BIT(nibble, 0);
				r.stores = BIT(nibble, 1);
				r.flag = BIT(nibble, 3);
				r.coefficient = (BIT(nibble, 2) ? -1 : 1) << SHIFTS[(op >> 12) & 3];
			}
		}
		else
		{
			r.coefficient = special_coefficient(op);
			if (k == 3 && nibble <= 1 && !cell)
			{
				r.kind = OP_INTERPOLATED;
				r.start = BIT(nibble, 0);
			}
			else if (k == 0 && (nibble == 6 || nibble == 7))
			{
				r.kind = OP_WRAP;
				r.start = BIT(nibble, 0);
			}
			else if (k == 0 && (nibble == 8 || nibble == 9))
			{
				r.kind = OP_POSITIVE;
				r.start = BIT(nibble, 0);
			}
			else if (k == 0 && (nibble == 0xb || nibble == 0xd || nibble == 0xf))
			{
				r.kind = OP_COMPARE;
				r.gates = nibble != 0xb;
				r.flag = nibble == 0xf;
			}
			else if (k == 0 && nibble == 1)
				r.kind = OP_SIGN;
			else if (k == 0 && nibble == 3)
				r.kind = OP_TRIANGLE;
			else if (k == 1 && (nibble == 1 || nibble == 3))
			{
				r.kind = OP_RETURN;
				r.arg = nibble >> 1;
			}
			else if (k == 2 && (nibble == 0 || nibble == 2) && !cell)
				r.kind = OP_NOP;
		}
	}
	else
	{
		const int kind = ctl & 15;
		r.start = BIT(kind, 0);
		if (kind <= 3)
		{
			r.kind = OP_ACC;
			r.stores = BIT(kind, 1);
		}
		else if (kind <= 7)
		{
			const u32 previous = m_program[(pc - 1) & (ROWS - 1)];
			r.kind = OP_EXT;
			r.stores = BIT(kind, 1);
			r.target = ((previous >> 27) & 3) << 13 | ((word >> 27) & 31) << 8 | cell;
		}
		else if (kind <= 9)
		{
			r.kind = OP_ABS;
			r.gates = kind == 8;
		}
		else if (kind <= 0xb)
		{
			r.kind = OP_STORE_ABS;
			r.gates = ctl == 0x0a;
		}
		else if (kind <= 0xd)
			r.kind = OP_CROSS;
	}

	if (r.kind == OP_INVALID && word)
		LOGMASKED(LOG_DECODE, "row %03x: unread word %08x\n", pc, word);
}


//-------------------------------------------------
//  arithmetic
//-------------------------------------------------

s64 roland_mr3_device::saturate(s64 value)
{
	return std::clamp<s64>(value, -WIDE_ONE, WIDE_ONE - ONE);
}

s64 roland_mr3_device::wrap(s64 value)
{
	s64 v = (value + WIDE_ONE) % (2 * WIDE_ONE);
	if (v < 0)
		v += 2 * WIDE_ONE;
	return v - WIDE_ONE;
}

s64 roland_mr3_device::read_cell(int cell) const
{
	switch (cell)
	{
	case 0: return 0;
	case 1: return ONE >> 19;
	case 2: return ONE >> 13;
	case 3: return ONE >> 7;
	case 4: return ONE >> 1;
	default: return m_ring[(cell - m_ring_pos) & (CELLS - 1)];
	}
}

void roland_mr3_device::write_cell(int cell, s64 value)
{
	if (cell > 4)
		m_ring[(cell - m_ring_pos) & (CELLS - 1)] = value;
}

int roland_mr3_device::eram_index(int offset) const
{
	const int i = offset - m_eram_pos;
	return i < 0 ? i + ERAM_WORDS : i;
}


//-------------------------------------------------
//  the comparison gate
//-------------------------------------------------

bool roland_mr3_device::blanked(int pc, int path) const
{
	for (int i = m_blank_count - 1; i >= 0; i--)
		if (m_blank[i].row == pc)
			return m_blank[i].path == path;
	return false;
}

void roland_mr3_device::gate(int armed, s64 value, int path)
{
	int n = 0;
	static constexpr int windows[2][3] = { { 4, 5, 0 }, { 6, 7, 8 } };
	for (int k : windows[value < 0 ? 1 : 0])
		if (k)
			m_blank[n++] = blank_entry{ s16(armed + k), u8(path) };
	m_blank_count = n;
}

void roland_mr3_device::arm(int pc, int path, bool holds)
{
	for (int i = 0; i < m_pending_count; i++)
		if (m_pending[i].row + 1 == pc && m_pending[i].holds)
			return;
	if (m_pending_count < MAX_PENDING)
		m_pending[m_pending_count++] = pending_entry{ s16(pc), u8(path), holds };
}

void roland_mr3_device::settle(int pc)
{
	int n = 0;
	for (int i = 0; i < m_pending_count; i++)
	{
		const pending_entry p = m_pending[i];
		if (p.row + 1 == pc)
			gate(p.row, m_acc[p.path], p.path);
		else if (p.row + 1 > pc)
			m_pending[n++] = p;
	}
	m_pending_count = n;
}


//-------------------------------------------------
//  execution
//-------------------------------------------------

int roland_mr3_device::step(int pc)
{
	const row &r = m_rows[pc];
	const int path = r.path;
	const bool blank = m_blank_count && blanked(pc, path);
	s64 &acc = m_acc[path];
	int taken = -1;

	switch (r.kind)
	{
	case OP_ACC:
	{
		s64 operand;
		if (r.stores)
		{
			operand = narrow(aged(path));
			write_cell(r.cell, operand);
		}
		else
			operand = read_cell(r.cell);
		if (blank)
			operand = 0;
		acc = (r.start ? 0 : acc) + operand * r.coefficient;
		m_operand = operand;
		break;
	}

	case OP_EXT:
	{
		const int where = eram_index(r.target);
		s64 operand;
		if (r.stores)
		{
			operand = narrow(aged(path));
			m_eram[where] = s32(operand) & ~((1 << (Q + 1 - ERAM_BITS)) - 1);
		}
		else
			operand = m_eram[where];
		if (blank)
			operand = 0;
		acc = (r.start ? 0 : acc) + operand * r.coefficient;
		m_operand = operand;
		break;
	}

	case OP_ABS:
	{
		const s64 operand = blank ? 0 : read_cell(r.cell);
		acc = (r.start ? 0 : acc) + std::abs(operand) * r.coefficient;
		m_operand = operand;
		if (r.gates)
			arm(pc, path, true);
		break;
	}

	case OP_STORE_ABS:
	{
		const s64 stored = narrow(aged(path));
		write_cell(r.cell, stored);
		const s64 operand = blank ? 0 : std::abs(stored);
		acc = (r.start ? 0 : acc) + operand * r.coefficient;
		m_operand = operand;
		if (r.gates)
			arm(pc, path, true);
		break;
	}

	case OP_CROSS:
	{
		const s64 stored = narrow(aged(path ^ 1));
		write_cell(r.cell, stored);
		const s64 operand = blank ? 0 : stored;
		acc = (r.start ? 0 : acc) + operand * r.coefficient;
		m_operand = operand;
		break;
	}

	case OP_CONTINUE:
		if (!blank)
			acc += m_operand * r.coefficient;
		break;

	case OP_K_LOAD:
		m_k[r.arg] = narrow(aged(path));
		break;

	case OP_ADDRESS:
		m_address = aged(path);
		break;

	case OP_HOST:
		m_result[r.arg] = aged(path);
		break;

	case OP_TRANSFER:
		if (r.arg >= 0x18 && r.arg <= 0x1f)
			m_output[r.arg - 0x18] = aged(path);
		break;

	case OP_INPUT:
	{
		const s64 operand = blank ? 0 : m_input[r.arg];
		acc = operand * r.coefficient;
		m_operand = operand;
		break;
	}

	case OP_SEND:
		m_second[r.arg][m_second_pos] = narrow(aged(path));
		break;

	case OP_RETURN:
	{
		const s64 operand = m_second[r.arg][m_second_pos ^ 1];
		acc = blank ? 0 : operand * r.coefficient;
		m_operand = operand;
		break;
	}

	case OP_TRIANGLE:
	{
		const s64 operand = read_cell(r.cell);
		acc = blank ? 0 : (ONE - std::abs(operand)) * r.coefficient;
		m_operand = operand;
		break;
	}

	case OP_K_FORM:
	{
		s64 operand;
		if (r.stores)
		{
			operand = narrow(aged(path));
			write_cell(r.cell, operand);
		}
		else
			operand = read_cell(r.cell);
		const s64 k = blank ? 0 : m_k[r.arg];
		const s64 gain = (r.flag ? ONE - k : k) * r.coefficient;
		acc = (r.start ? 0 : acc) + operand * gain;
		m_operand = operand;
		break;
	}

	case OP_JUMP:
		if (condition(r.arg, aged(path)))
			taken = r.target;
		break;

	case OP_INTERPOLATED:
	{
		const s64 whole = m_address >> (2 * Q - 11);
		const s64 fraction = m_address & ((s64(1) << (2 * Q - 11)) - 1);
		const auto at = [this] (s64 position) { s64 i = (position - m_eram_pos) % ERAM_WORDS; return m_eram[i < 0 ? i + ERAM_WORDS : i]; };
		if (r.start)
		{
			const s64 value = (at(whole) * ((s64(1) << (2 * Q - 11)) - fraction)) >> (Q - 11);
			acc = blank ? 0 : value;
		}
		else
		{
			const s64 value = (at(whole + 1) * fraction) >> (Q - 11);
			acc += blank ? 0 : value;
		}
		break;
	}

	case OP_WRAP:
	case OP_POSITIVE:
	{
		const s64 value = r.kind == OP_WRAP ? wrap(aged(path)) >> Q : std::max<s64>(0, saturate(aged(path))) >> Q;
		write_cell(r.cell, value);
		acc = (r.start ? 0 : acc) + (blank ? 0 : r.coefficient) * value;
		m_operand = value;
		break;
	}

	case OP_COMPARE:
	{
		const s64 operand = read_cell(r.cell);
		acc = (blank ? 0 : r.coefficient) * operand;
		m_operand = operand;
		if (r.gates)
			arm(pc, path, r.flag);
		break;
	}

	case OP_SIGN:
	{
		const s64 operand = read_cell(r.cell);
		acc = blank ? 0 : (operand >= 0 ? s64(r.coefficient) : -s64(r.coefficient)) << Q;
		m_operand = operand;
		break;
	}

	default:
		break;
	}

	m_history[m_history_pos][0] = m_acc[0];
	m_history[m_history_pos][1] = m_acc[1];
	m_history_pos = m_history_pos == 2 ? 0 : m_history_pos + 1;
	if (m_pending_count)
		settle(pc);
	return taken;
}

void roland_mr3_device::execute_sample()
{
	int target = -1;
	bool slot = false;
	for (int pc = 0; pc < ROWS; )
	{
		const int taken = step(pc);
		if (slot)
		{
			slot = false;
			if (target >= 0)
			{
				if (target <= pc)
					break;
				pc = target;
				target = -1;
				continue;
			}
		}
		else if (taken >= 0)
		{
			target = taken;
			slot = true;
		}
		pc++;
	}

	int n = 0;
	for (int i = 0; i < m_blank_count; i++)
		if (m_blank[i].row >= ROWS)
			m_blank[n++] = blank_entry{ s16(m_blank[i].row - ROWS), m_blank[i].path };
	m_blank_count = n;
	m_pending_count = 0;

	m_ring_pos++;
	m_eram_pos = m_eram_pos == ERAM_WORDS - 1 ? 0 : m_eram_pos + 1;
	m_second_pos ^= 1;
}

void roland_mr3_device::sound_stream_update(sound_stream &stream)
{
	for (int i = 0; i < stream.samples(); i++)
	{
		for (int lane = 0; lane < LANES; lane++)
			m_input[lane] = std::clamp<s64>(s64(std::floor(stream.get(lane, i) * ONE)), -ONE, ONE - 1);

		execute_sample();

		for (int lane = 0; lane < LANES; lane++)
			stream.put(lane, i, sound_stream::sample_t(saturate(m_output[lane])) / sound_stream::sample_t(WIDE_ONE));
	}
}
