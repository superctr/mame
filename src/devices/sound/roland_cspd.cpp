// license:BSD-3-Clause
// copyright-holders:superctr
// thanks-to: giulioz
/* Roland CSP (TC6088AF) disassembler */

#include "emu.h"
#include "roland_cspd.h"

roland_csp_disassembler::instruction roland_csp_disassembler::decode(u32 word)
{
	return { u16(word & 0x1ff), u8((word >> 9) & 7), u8((word >> 12) & 15),
		u8((word >> 16) & 3), u8(BIT(word, 18)), u8(BIT(word, 19)), u8((word >> 20) & 15) };
}

offs_t roland_csp_disassembler::disassemble(std::ostream &stream, offs_t pc, const data_buffer &opcodes, const data_buffer &params)
{
	const u64 raw = opcodes.r64(pc);
	const instruction s = decode(raw);
	const u16 coefficient = raw >> 32;
	static const char *const op[] = { "maca", "macb", "lda", "ldb", "absa", "absa.g", "nega", "nega.acc",
		"absa2", "absa2.acc", "maca.lo", "macb.lo", "mula", "mulb", "mula.acc", "mulb.acc" };
	static const char *const store[] = { "", "sc", "spec", "spec.sat", "iram.a", "iram.b", "iram.a.sat", "iram.b.sat" };
	static const unsigned shift[] = { 15, 14, 13, 11 };
	util::stream_format(stream, "%s $%03x, #%04x >>%u", op[s.opcode], s.offset, coefficient, shift[s.scale]);
	if (s.store)
		util::stream_format(stream, ", %s[$%03x]", store[s.store], s.offset);
	if ((s.store == 2 || s.store == 3) && s.offset >= 0x172 && s.offset <= 0x177)
		util::stream_format(stream, ", jump $%03x", coefficient);
	if (s.eram_start)
		util::stream_format(stream, " ; eram %x", s.eram_nib);
	else if (s.eram_nib)
		util::stream_format(stream, " ; +%x", s.eram_nib);
	if (s.bit18)
		stream << " ; b18";
	return 1 | SUPPORTED;
}
