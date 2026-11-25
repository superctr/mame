// license:BSD-3-Clause
// copyright-holders:superctr
/*
 *  Roland LSP (MB87837) disassembler
 */
#include "emu.h"
#include "roland_lspd.h"

namespace {

const char *const ACCUMULATOR[4] = { "", "accA", "accB", "accA!" };

// offsets 1 to 4 read a power of two instead of memory, so the coefficient becomes an immediate
constexpr int IMMEDIATE[5] = { 0, 7, 12, 17, 22 };

void operand(std::ostream &stream, u8 offset)
{
	if (offset >= 1 && offset <= 4)
		util::stream_format(stream, "#%d", IMMEDIATE[offset]);
	else
		util::stream_format(stream, "$%02x", offset);
}

void coefficient_text(std::ostream &stream, u8 coefficient, int shift)
{
	util::stream_format(stream, "*%c%02x>>%d", BIT(coefficient, 7) ? '-' : '+', BIT(coefficient, 7) ? (-s8(coefficient) & 0xff) : coefficient, shift);
}

void product(std::ostream &stream, u8 offset, u8 coefficient, int shift)
{
	operand(stream, offset);
	coefficient_text(stream, coefficient, shift);
}

void feed(std::ostream &stream, char acc, bool replace, const char *value, u8 coefficient, int shift)
{
	if (*value && coefficient)
	{
		util::stream_format(stream, "acc%c %s %s", acc, replace ? " =" : "+=", value);
		coefficient_text(stream, coefficient, shift);
	}
	else if (replace)
		util::stream_format(stream, "acc%c  = 0", acc);
}

} // anonymous namespace

void roland_lsp_disassembler::describe(std::ostream &stream, offs_t pc, u32 word)
{
	const int op = (word >> 21) & 7;
	const int store = (word >> 19) & 3;
	const int eram = (word >> 16) & 7;
	const int shift = BIT(word, 15) ? 5 : 7;
	const u8 offset = (word >> 8) & 0x7f;
	const u8 coefficient = word & 0xff;
	const int slot = offset & 0x1f;
	const bool replace = BIT(offset, 5);
	const bool special = op >= 6 && BIT(offset, 6);
	const char acc = (op & 1) ? 'B' : 'A';
	const bool low = op == 4 && BIT(coefficient, 6);

	if (!special && !low && store)
		util::stream_format(stream, "[$%02x] = %s, ", offset, ACCUMULATOR[store]);

	if (op < 4 || op == 5)
	{
		if (op == 5)
			stream << "accA  = |";
		else
			util::stream_format(stream, "acc%c %s ", (op & 2) ? 'B' : 'A', (op & 1) ? " =" : "+=");
		product(stream, offset, coefficient, shift);
		if (op == 5)
			stream << '|';
	}
	else if (op == 4)
	{
		util::stream_format(stream, "acc%c %s %c", BIT(coefficient, 4) ? 'B' : 'A', BIT(coefficient, 3) ? " =" : "+=", BIT(coefficient, 2) ? '-' : ' ');
		if (low)
			stream << "prev";
		else
			operand(stream, offset);
		if (coefficient & 3)
			util::stream_format(stream, "*mul%d%s>>%d", BIT(coefficient, 1), low ? ".lo" : ".hi", shift + (low ? 8 : 0));
		else if (low)
			util::stream_format(stream, "*%02x>>%d", coefficient, shift + 8);
		else
			coefficient_text(stream, coefficient, shift);
		if (low && store)
			util::stream_format(stream, ", [$%02x] = %s", offset, ACCUMULATOR[store]);
	}
	else if (!special)
	{
		util::stream_format(stream, "acc%c %s ", acc, replace ? " =" : "+=");
		product(stream, offset, coefficient, shift);
	}
	else if (slot == 0x10 && !store)
		util::stream_format(stream, "acc%c %s prev*%02x>>%d", acc, replace ? " =" : "+=", coefficient, 8 + shift);
	else if (slot >= 0x1a)
	{
		util::stream_format(stream, "acc%c %s ", acc, replace ? " =" : "+=");
		if (slot >= 0x1e)
			stream << "audio_in";
		else
			util::stream_format(stream, "eram%d", slot - 0x1a);
		coefficient_text(stream, coefficient, shift);
	}
	else
	{
		const char *value = ACCUMULATOR[store];
		switch (slot)
		{
		case 0x0d: case 0x0e: case 0x0f:
			util::stream_format(stream, "%s %03x", (slot == 0x0d) ? "jmpn" : (slot == 0x0e) ? "jmpp" : "jmp ", coefficient << 1);
			if (store)
				util::stream_format(stream, ", %s", value);
			break;
		case 0x10: util::stream_format(stream, "eram = %s", value); break;
		case 0x12: stream << "skip"; break;
		case 0x13: util::stream_format(stream, "tap:mul0 = %s", value); break;
		case 0x14: case 0x15: util::stream_format(stream, "mul%d = %s", slot - 0x14, value); break;
		case 0x16: util::stream_format(stream, "[$76] = %s", value); break;
		case 0x18: util::stream_format(stream, "audio_out = %s", value); break;
		default:   util::stream_format(stream, "slot%02x = %s", slot, value); break;
		}
		if (replace || (store && coefficient))
		{
			stream << ", ";
			feed(stream, acc, replace, value, coefficient, shift);
		}
	}

	if (eram)
		util::stream_format(stream, "  ; e%d", eram);
}

offs_t roland_lsp_disassembler::disassemble(std::ostream &stream, offs_t pc, const data_buffer &opcodes, const data_buffer &params)
{
	describe(stream, pc, opcodes.r32(pc) & 0xffffff);
	return 1 | SUPPORTED;
}
