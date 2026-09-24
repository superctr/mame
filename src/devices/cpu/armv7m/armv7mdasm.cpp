// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    armv7mdasm.cpp

    ARMv7-M Thumb-2 disassembler, in unified assembler syntax.

***************************************************************************/

#include "emu.h"
#include "armv7mdasm.h"

const char *const armv7m_disassembler::s_regs[16] = {
	"r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
	"r8", "r9", "r10", "r11", "r12", "sp", "lr", "pc"
};

const char *const armv7m_disassembler::s_conds[16] = {
	"eq", "ne", "cs", "cc", "mi", "pl", "vs", "vc",
	"hi", "ls", "ge", "lt", "gt", "le", "", "nv"
};

const char *const armv7m_disassembler::s_shifts[4] = {
	"lsl", "lsr", "asr", "ror"
};

u32 armv7m_disassembler::opcode_alignment() const
{
	return 2;
}

std::string armv7m_disassembler::imm(u32 value)
{
	return value < 10 ? util::string_format("#%u", value) : util::string_format("#0x%x", value);
}

std::string armv7m_disassembler::simm(s32 value)
{
	if (value < 0)
		return value > -10 ? util::string_format("#-%d", -value) : util::string_format("#-0x%x", u32(-s64(value)));
	return imm(value);
}

std::string armv7m_disassembler::mem(const char *base, u32 offset)
{
	return offset ? util::string_format("[%s, %s]", base, imm(offset)) : util::string_format("[%s]", base);
}

std::string armv7m_disassembler::reglist(u32 registers)
{
	std::string result = "{";
	bool first = true;
	for (int i = 0; i < 16; i++)
	{
		if (!BIT(registers, i))
			continue;
		if (!first)
			result += ", ";
		result += s_regs[i];
		first = false;
	}
	return result + "}";
}

std::string armv7m_disassembler::shift_imm(int type, unsigned imm5)
{
	if (type == 0 && imm5 == 0)
		return "";
	if (type == 3 && imm5 == 0)
		return ", rrx";
	if ((type == 1 || type == 2) && imm5 == 0)
		imm5 = 32;
	return util::string_format(", %s #%u", s_shifts[type], imm5);
}

std::string armv7m_disassembler::spec_reg(unsigned sysm, unsigned mask, bool write)
{
	static const char *const psrs[8] = { "apsr", "iapsr", "eapsr", "xpsr", nullptr, "ipsr", "epsr", "iepsr" };
	static const char *const others[5] = { "primask", "basepri", "basepri_max", "faultmask", "control" };
	std::string name;
	if (sysm < 8 && psrs[sysm])
	{
		name = psrs[sysm];
		if (write && sysm < 4)
		{
			switch (mask)
			{
			case 1: name += "_g"; break;
			case 2: name += "_nzcvq"; break;
			case 3: name += "_nzcvqg"; break;
			default: break;
			}
		}
	}
	else if (sysm == 8)
		name = "msp";
	else if (sysm == 9)
		name = "psp";
	else if (sysm >= 16 && sysm <= 20)
		name = others[sysm - 16];
	else
		name = util::string_format("sysm_%02x", sysm);
	return name;
}

u32 armv7m_disassembler::thumb_expand_imm(u32 imm12)
{
	if (!(imm12 & 0xc00))
	{
		const u32 imm8 = imm12 & 0xff;
		switch ((imm12 >> 8) & 3)
		{
		case 0: return imm8;
		case 1: return imm8 * 0x00010001;
		case 2: return imm8 * 0x01000100;
		default: return imm8 * 0x01010101;
		}
	}
	const u32 value = 0x80 | (imm12 & 0x7f);
	const unsigned rot = (imm12 >> 7) & 0x1f;
	return (value >> rot) | (value << (32 - rot));
}

offs_t armv7m_disassembler::disassemble(std::ostream &stream, offs_t pc, const data_buffer &opcodes, const data_buffer &params)
{
	const u16 op = opcodes.r16(pc);
	if ((op & 0xe000) == 0xe000 && (op & 0x1800))
		return dasm32(stream, pc, (u32(op) << 16) | opcodes.r16(pc + 2));
	return dasm16(stream, pc, op);
}


//-------------------------------------------------
//  16-bit encodings
//-------------------------------------------------

offs_t armv7m_disassembler::dasm16(std::ostream &stream, offs_t pc, u16 op)
{
	const u32 flags = 2 | SUPPORTED;
	const char *const rd = s_regs[op & 7];
	const char *const rn = s_regs[(op >> 3) & 7];
	const char *const rm6 = s_regs[(op >> 6) & 7];
	const char *const r8 = s_regs[(op >> 8) & 7];

	switch (op >> 11)
	{
	case 0x00: case 0x01: case 0x02:
		{
			static const char *const names[3] = { "lsls", "lsrs", "asrs" };
			const unsigned imm5 = (op >> 6) & 0x1f;
			const int type = op >> 11;
			if (type == 0 && imm5 == 0)
				util::stream_format(stream, "%-8s%s, %s", "movs", rd, rn);
			else
				util::stream_format(stream, "%-8s%s, %s, #%u", names[type], rd, rn, imm5 ? imm5 : 32);
			return flags;
		}
	case 0x03:
		{
			const char *const name = (op & 0x200) ? "subs" : "adds";
			if (op & 0x400)
				util::stream_format(stream, "%-8s%s, %s, #%u", name, rd, rn, (op >> 6) & 7);
			else
				util::stream_format(stream, "%-8s%s, %s, %s", name, rd, rn, rm6);
			return flags;
		}
	case 0x04: util::stream_format(stream, "%-8s%s, %s", "movs", r8, imm(op & 0xff)); return flags;
	case 0x05: util::stream_format(stream, "%-8s%s, %s", "cmp", r8, imm(op & 0xff)); return flags;
	case 0x06: util::stream_format(stream, "%-8s%s, %s", "adds", r8, imm(op & 0xff)); return flags;
	case 0x07: util::stream_format(stream, "%-8s%s, %s", "subs", r8, imm(op & 0xff)); return flags;

	case 0x08:
		if (!(op & 0x400))
		{
			static const char *const names[16] = {
				"ands", "eors", "lsls", "lsrs", "asrs", "adcs", "sbcs", "rors",
				"tst", "rsbs", "cmp", "cmn", "orrs", "muls", "bics", "mvns"
			};
			const unsigned opc = (op >> 6) & 0xf;
			if (opc == 9)
				util::stream_format(stream, "%-8s%s, %s, #0", names[opc], rd, rn);
			else if (opc == 13)
				util::stream_format(stream, "%-8s%s, %s, %s", names[opc], rd, rn, rd);
			else
				util::stream_format(stream, "%-8s%s, %s", names[opc], rd, rn);
			return flags;
		}
		else
		{
			const unsigned dn = ((op >> 4) & 8) | (op & 7);
			const unsigned m = (op >> 3) & 0xf;
			switch ((op >> 8) & 3)
			{
			case 0:
				util::stream_format(stream, "%-8s%s, %s", "add", s_regs[dn], s_regs[m]);
				return flags;
			case 1:
				util::stream_format(stream, "%-8s%s, %s", "cmp", s_regs[dn], s_regs[m]);
				return flags;
			case 2:
				util::stream_format(stream, "%-8s%s, %s", "mov", s_regs[dn], s_regs[m]);
				return flags;
			default:
				if (op & 0x80)
				{
					util::stream_format(stream, "%-8s%s", "blx", s_regs[m]);
					return flags | STEP_OVER;
				}
				util::stream_format(stream, "%-8s%s", "bx", s_regs[m]);
				return flags | (m == 14 ? STEP_OUT : 0);
			}
		}
	case 0x09:
		{
			const u32 target = ((pc + 4) & ~3) + ((op & 0xff) << 2);
			util::stream_format(stream, "%-8s%s, [pc, %s] ; 0x%08x", "ldr", r8, imm((op & 0xff) << 2), target);
			return flags;
		}

	case 0x0a: case 0x0b:
		{
			static const char *const names[8] = { "str", "strh", "strb", "ldrsb", "ldr", "ldrh", "ldrb", "ldrsh" };
			util::stream_format(stream, "%-8s%s, [%s, %s]", names[(op >> 9) & 7], rd, rn, rm6);
			return flags;
		}
	case 0x0c: case 0x0d:
		util::stream_format(stream, "%-8s%s, %s", (op & 0x800) ? "ldr" : "str", rd, mem(rn, ((op >> 6) & 0x1f) << 2));
		return flags;
	case 0x0e: case 0x0f:
		util::stream_format(stream, "%-8s%s, %s", (op & 0x800) ? "ldrb" : "strb", rd, mem(rn, (op >> 6) & 0x1f));
		return flags;
	case 0x10: case 0x11:
		util::stream_format(stream, "%-8s%s, %s", (op & 0x800) ? "ldrh" : "strh", rd, mem(rn, ((op >> 6) & 0x1f) << 1));
		return flags;
	case 0x12: case 0x13:
		util::stream_format(stream, "%-8s%s, %s", (op & 0x800) ? "ldr" : "str", r8, mem("sp", (op & 0xff) << 2));
		return flags;
	case 0x14:
		util::stream_format(stream, "%-8s%s, pc, %s ; 0x%08x", "adr", r8, imm((op & 0xff) << 2), ((pc + 4) & ~3) + ((op & 0xff) << 2));
		return flags;
	case 0x15:
		util::stream_format(stream, "%-8s%s, sp, %s", "add", r8, imm((op & 0xff) << 2));
		return flags;

	case 0x16: case 0x17:
		switch ((op >> 8) & 0xf)
		{
		case 0x0:
			util::stream_format(stream, "%-8ssp, sp, %s", (op & 0x80) ? "sub" : "add", imm((op & 0x7f) << 2));
			return flags;
		case 0x1: case 0x3: case 0x9: case 0xb:
			{
				const u32 target = pc + 4 + (((op >> 2) & 0x3e) | ((op >> 3) & 0x40));
				util::stream_format(stream, "%-8s%s, 0x%08x", (op & 0x800) ? "cbnz" : "cbz", rd, target);
				return flags | STEP_COND;
			}
		case 0x2:
			{
				static const char *const names[4] = { "sxth", "sxtb", "uxth", "uxtb" };
				util::stream_format(stream, "%-8s%s, %s", names[(op >> 6) & 3], rd, rn);
				return flags;
			}
		case 0x4: case 0x5:
			util::stream_format(stream, "%-8s%s", "push", reglist((op & 0xff) | ((op & 0x100) << 6)));
			return flags;
		case 0x6:
			if ((op & 0xffe0) == 0xb660)
			{
				util::stream_format(stream, "%-8s%s%s", (op & 0x10) ? "cpsid" : "cpsie", (op & 2) ? "i" : "", (op & 1) ? "f" : "");
				return flags;
			}
			break;
		case 0xa:
			{
				static const char *const names[4] = { "rev", "rev16", nullptr, "revsh" };
				if (names[(op >> 6) & 3])
				{
					util::stream_format(stream, "%-8s%s, %s", names[(op >> 6) & 3], rd, rn);
					return flags;
				}
				break;
			}
		case 0xc: case 0xd:
			util::stream_format(stream, "%-8s%s", "pop", reglist((op & 0xff) | ((op & 0x100) << 7)));
			return flags | ((op & 0x100) ? STEP_OUT : 0);
		case 0xe:
			util::stream_format(stream, "%-8s%s", "bkpt", imm(op & 0xff));
			return flags;
		case 0xf:
			if (op & 0xf)
			{
				const unsigned firstcond = (op >> 4) & 0xf;
				const unsigned mask = op & 0xf;
				std::string name = "it";
				const unsigned count = 3 - std::countr_zero(mask);
				for (unsigned i = 0; i < count; i++)
					name += (BIT(mask, 3 - i) == BIT(firstcond, 0)) ? 't' : 'e';
				util::stream_format(stream, "%-8s%s", name, firstcond == 14 ? "al" : s_conds[firstcond]);
				return flags;
			}
			else
			{
				static const char *const names[5] = { "nop", "yield", "wfe", "wfi", "sev" };
				const unsigned hint = (op >> 4) & 0xf;
				if (hint < 5)
					util::stream_format(stream, "%s", names[hint]);
				else
					util::stream_format(stream, "%-8s%s", "hint", imm(hint));
				return flags;
			}
		default:
			break;
		}
		break;

	case 0x18: case 0x19:
		{
			const u32 registers = op & 0xff;
			const unsigned n = (op >> 8) & 7;
			if (op & 0x800)
				util::stream_format(stream, "%-8s%s%s, %s", "ldm", r8, BIT(registers, n) ? "" : "!", reglist(registers));
			else
				util::stream_format(stream, "%-8s%s!, %s", "stm", r8, reglist(registers));
			return flags;
		}

	case 0x1a: case 0x1b:
		{
			const unsigned cond = (op >> 8) & 0xf;
			if (cond == 14)
			{
				util::stream_format(stream, "%-8s%s", "udf", imm(op & 0xff));
				return flags;
			}
			if (cond == 15)
			{
				util::stream_format(stream, "%-8s%s", "svc", imm(op & 0xff));
				return flags | STEP_OVER;
			}
			util::stream_format(stream, "b%-7s0x%08x", s_conds[cond], pc + 4 + util::sext(u32(op & 0xff) << 1, 9));
			return flags | STEP_COND;
		}
	case 0x1c:
		util::stream_format(stream, "%-8s0x%08x", "b", pc + 4 + util::sext(u32(op & 0x7ff) << 1, 12));
		return flags;
	}

	util::stream_format(stream, "undefined 0x%04x", op);
	return flags;
}


//-------------------------------------------------
//  32-bit encodings
//-------------------------------------------------

offs_t armv7m_disassembler::dasm32(std::ostream &stream, offs_t pc, u32 op)
{
	switch ((op >> 27) & 3)
	{
	case 1:
		if (op & 0x04000000)
			return dasm_coprocessor(stream, op);
		if (op & 0x02000000)
			return dasm_dp_shifted(stream, op);
		if (op & 0x00400000)
			return dasm_dual_exclusive(stream, pc, op);
		return dasm_ldm_stm(stream, op);

	case 2:
		if (op & 0x8000)
			return dasm_branch_misc(stream, pc, op);
		if (op & 0x02000000)
			return dasm_dp_plain(stream, pc, op);
		return dasm_dp_modified(stream, op);

	default:
		{
			const u32 op2 = (op >> 20) & 0x7f;
			if (op2 & 0x40)
				return dasm_coprocessor(stream, op);
			if ((op2 & 0x71) == 0x00)
				return dasm_store(stream, op);
			if ((op2 & 0x61) == 0x01)
				return dasm_load(stream, pc, op);
			if ((op2 & 0x70) == 0x20)
				return dasm_dp_register(stream, op);
			if ((op2 & 0x78) == 0x30)
				return dasm_multiply(stream, op);
			if ((op2 & 0x78) == 0x38)
				return dasm_long_multiply(stream, op);
			break;
		}
	}
	util::stream_format(stream, "undefined 0x%08x", op);
	return 4 | SUPPORTED;
}

offs_t armv7m_disassembler::dasm_coprocessor(std::ostream &stream, u32 op)
{
	const u32 op1 = (op >> 20) & 0x3f;
	const unsigned coproc = (op >> 8) & 0xf;
	const bool two = op & 0x10000000;
	const unsigned crd = (op >> 12) & 0xf;
	const unsigned n = (op >> 16) & 0xf;

	if ((op1 & 0x3e) == 0 || (op1 & 0x30) == 0x30)
	{
		util::stream_format(stream, "undefined 0x%08x", op);
		return 4 | SUPPORTED;
	}
	if (op1 == 4 || op1 == 5)
	{
		util::stream_format(stream, "%-8sp%u, #%u, %s, %s, c%u", std::string(op1 == 4 ? "mcrr" : "mrrc") + (two ? "2" : ""),
				coproc, (op >> 4) & 0xf, s_regs[crd], s_regs[n], op & 0xf);
		return 4 | SUPPORTED;
	}
	if (!(op1 & 0x20))
	{
		const bool p = op & 0x01000000;
		const bool u = op & 0x00800000;
		const bool w = op & 0x00200000;
		const u32 offset = (op & 0xff) << 2;
		std::string name = std::string((op1 & 1) ? "ldc" : "stc") + (two ? "2" : "") + ((op & 0x00400000) ? "l" : "");
		if (p)
			util::stream_format(stream, "%-8sp%u, c%u, [%s, %s]%s", name, coproc, crd, s_regs[n], simm(u ? offset : -s32(offset)), w ? "!" : "");
		else if (w)
			util::stream_format(stream, "%-8sp%u, c%u, [%s], %s", name, coproc, crd, s_regs[n], simm(u ? offset : -s32(offset)));
		else
			util::stream_format(stream, "%-8sp%u, c%u, [%s], {%u}", name, coproc, crd, s_regs[n], op & 0xff);
		return 4 | SUPPORTED;
	}
	if (!(op & 0x10))
		util::stream_format(stream, "%-8sp%u, #%u, c%u, c%u, c%u, #%u", std::string("cdp") + (two ? "2" : ""), coproc, (op >> 20) & 0xf, crd, n, op & 0xf, (op >> 5) & 7);
	else
		util::stream_format(stream, "%-8sp%u, #%u, %s, c%u, c%u, #%u", std::string((op1 & 1) ? "mrc" : "mcr") + (two ? "2" : ""), coproc, (op >> 21) & 7,
				((op1 & 1) && crd == 15) ? "apsr_nzcv" : s_regs[crd], n, op & 0xf, (op >> 5) & 7);
	return 4 | SUPPORTED;
}

offs_t armv7m_disassembler::dasm_ldm_stm(std::ostream &stream, u32 op)
{
	const unsigned mode = (op >> 23) & 3;
	const unsigned n = (op >> 16) & 0xf;
	const bool wback = op & 0x00200000;
	const bool load = op & 0x00100000;
	const u32 registers = op & 0xffff;
	if (mode != 1 && mode != 2)
	{
		util::stream_format(stream, "undefined 0x%08x", op);
		return 4 | SUPPORTED;
	}
	const u32 flags = 4 | SUPPORTED | ((load && BIT(registers, 15)) ? STEP_OUT : 0);
	if (n == 13 && wback && ((mode == 1 && load) || (mode == 2 && !load)))
	{
		util::stream_format(stream, "%-8s%s", load ? "pop.w" : "push.w", reglist(registers));
		return flags;
	}
	const char *name = mode == 1 ? (load ? "ldm.w" : "stm.w") : (load ? "ldmdb" : "stmdb");
	util::stream_format(stream, "%-8s%s%s, %s", name, s_regs[n], wback ? "!" : "", reglist(registers));
	return flags;
}

offs_t armv7m_disassembler::dasm_dual_exclusive(std::ostream &stream, offs_t pc, u32 op)
{
	const unsigned n = (op >> 16) & 0xf;
	const unsigned t = (op >> 12) & 0xf;
	const unsigned t2 = (op >> 8) & 0xf;
	const bool p = op & 0x01000000;
	const bool u = op & 0x00800000;
	const bool w = op & 0x00200000;
	const bool l = op & 0x00100000;

	if (p || w)
	{
		const u32 offset = (op & 0xff) << 2;
		const std::string off = simm(u ? offset : -s32(offset));
		const char *name = l ? "ldrd" : "strd";
		if (p)
		{
			if (!offset && u && !w)
				util::stream_format(stream, "%-8s%s, %s, [%s]", name, s_regs[t], s_regs[t2], s_regs[n]);
			else
				util::stream_format(stream, "%-8s%s, %s, [%s, %s]%s", name, s_regs[t], s_regs[t2], s_regs[n], off, w ? "!" : "");
		}
		else
			util::stream_format(stream, "%-8s%s, %s, [%s], %s", name, s_regs[t], s_regs[t2], s_regs[n], off);
		if (l && n == 15 && p)
			util::stream_format(stream, " ; 0x%08x", u ? pc + 4 + offset : pc + 4 - offset);
		return 4 | SUPPORTED;
	}

	if (!u)
	{
		const u32 offset = (op & 0xff) << 2;
		const std::string mem = offset ? util::string_format("[%s, %s]", s_regs[n], imm(offset)) : util::string_format("[%s]", s_regs[n]);
		if (l)
			util::stream_format(stream, "%-8s%s, %s", "ldrex", s_regs[t], mem);
		else
			util::stream_format(stream, "%-8s%s, %s, %s", "strex", s_regs[t2], s_regs[t], mem);
		return 4 | SUPPORTED;
	}

	const unsigned op3 = (op >> 4) & 0xf;
	if (!l)
	{
		if (op3 == 4 || op3 == 5)
		{
			util::stream_format(stream, "%-8s%s, %s, [%s]", op3 == 4 ? "strexb" : "strexh", s_regs[op & 0xf], s_regs[t], s_regs[n]);
			return 4 | SUPPORTED;
		}
	}
	else
	{
		switch (op3)
		{
		case 0:
			util::stream_format(stream, "%-8s[%s, %s]", "tbb", s_regs[n], s_regs[op & 0xf]);
			return 4 | SUPPORTED;
		case 1:
			util::stream_format(stream, "%-8s[%s, %s, lsl #1]", "tbh", s_regs[n], s_regs[op & 0xf]);
			return 4 | SUPPORTED;
		case 4: case 5:
			util::stream_format(stream, "%-8s%s, [%s]", op3 == 4 ? "ldrexb" : "ldrexh", s_regs[t], s_regs[n]);
			return 4 | SUPPORTED;
		}
	}
	util::stream_format(stream, "undefined 0x%08x", op);
	return 4 | SUPPORTED;
}

offs_t armv7m_disassembler::dasm_dp_shifted(std::ostream &stream, u32 op)
{
	static const char *const names[16] = {
		"and", "bic", "orr", "orn", "eor", nullptr, nullptr, nullptr,
		"add", nullptr, "adc", "sbc", nullptr, "sub", "rsb", nullptr
	};
	const unsigned opc = (op >> 21) & 0xf;
	const bool s = op & 0x00100000;
	const unsigned n = (op >> 16) & 0xf;
	const unsigned d = (op >> 8) & 0xf;
	const unsigned m = op & 0xf;
	const int type = (op >> 4) & 3;
	const unsigned imm5 = ((op >> 10) & 0x1c) | ((op >> 6) & 3);
	const std::string sh = shift_imm(type, imm5);

	if (!names[opc])
	{
		util::stream_format(stream, "undefined 0x%08x", op);
		return 4 | SUPPORTED;
	}
	if (d == 15 && s && (opc == 0 || opc == 4 || opc == 8 || opc == 13))
	{
		static const char *const tests[16] = { "tst.w", nullptr, nullptr, nullptr, "teq", nullptr, nullptr, nullptr, "cmn.w", nullptr, nullptr, nullptr, nullptr, "cmp.w" };
		util::stream_format(stream, "%-8s%s, %s%s", tests[opc], s_regs[n], s_regs[m], sh);
		return 4 | SUPPORTED;
	}
	if (opc == 2 && n == 15)
	{
		if (type == 0 && imm5 == 0)
			util::stream_format(stream, "%-8s%s, %s", s ? "movs.w" : "mov.w", s_regs[d], s_regs[m]);
		else if (type == 3 && imm5 == 0)
			util::stream_format(stream, "%-8s%s, %s", s ? "rrxs" : "rrx", s_regs[d], s_regs[m]);
		else
			util::stream_format(stream, "%s%-*s%s, %s, #%u", s_shifts[type], 5, s ? "s.w" : ".w", s_regs[d], s_regs[m], (type == 1 || type == 2) && !imm5 ? 32 : imm5);
		return 4 | SUPPORTED;
	}
	if (opc == 3 && n == 15)
	{
		util::stream_format(stream, "%-8s%s, %s%s", s ? "mvns.w" : "mvn.w", s_regs[d], s_regs[m], sh);
		return 4 | SUPPORTED;
	}
	util::stream_format(stream, "%-8s%s, %s, %s%s", std::string(names[opc]) + (s ? "s" : "") + ".w", s_regs[d], s_regs[n], s_regs[m], sh);
	return 4 | SUPPORTED;
}

offs_t armv7m_disassembler::dasm_dp_modified(std::ostream &stream, u32 op)
{
	static const char *const names[16] = {
		"and", "bic", "orr", "orn", "eor", nullptr, nullptr, nullptr,
		"add", nullptr, "adc", "sbc", nullptr, "sub", "rsb", nullptr
	};
	const unsigned opc = (op >> 21) & 0xf;
	const bool s = op & 0x00100000;
	const unsigned n = (op >> 16) & 0xf;
	const unsigned d = (op >> 8) & 0xf;
	const u32 value = thumb_expand_imm(((op >> 15) & 0x800) | ((op >> 4) & 0x700) | (op & 0xff));

	if (!names[opc])
	{
		util::stream_format(stream, "undefined 0x%08x", op);
		return 4 | SUPPORTED;
	}
	if (d == 15 && s && (opc == 0 || opc == 4 || opc == 8 || opc == 13))
	{
		static const char *const tests[16] = { "tst.w", nullptr, nullptr, nullptr, "teq", nullptr, nullptr, nullptr, "cmn.w", nullptr, nullptr, nullptr, nullptr, "cmp.w" };
		util::stream_format(stream, "%-8s%s, %s", tests[opc], s_regs[n], imm(value));
		return 4 | SUPPORTED;
	}
	if ((opc == 2 || opc == 3) && n == 15)
	{
		util::stream_format(stream, "%-8s%s, %s", std::string(opc == 2 ? "mov" : "mvn") + (s ? "s" : "") + ".w", s_regs[d], imm(value));
		return 4 | SUPPORTED;
	}
	util::stream_format(stream, "%-8s%s, %s, %s", std::string(names[opc]) + (s ? "s" : "") + ".w", s_regs[d], s_regs[n], imm(value));
	return 4 | SUPPORTED;
}

offs_t armv7m_disassembler::dasm_dp_plain(std::ostream &stream, offs_t pc, u32 op)
{
	const unsigned opc = (op >> 20) & 0x1f;
	const unsigned n = (op >> 16) & 0xf;
	const unsigned d = (op >> 8) & 0xf;
	const u32 imm12 = ((op >> 15) & 0x800) | ((op >> 4) & 0x700) | (op & 0xff);
	const unsigned lsb = ((op >> 10) & 0x1c) | ((op >> 6) & 3);

	switch (opc)
	{
	case 0x00:
	case 0x0a:
		if (n == 15)
		{
			const u32 base = (pc + 4) & ~3;
			util::stream_format(stream, "%-8s%s, pc, %s ; 0x%08x", opc ? "subw" : "addw", s_regs[d], imm(imm12), opc ? base - imm12 : base + imm12);
		}
		else
			util::stream_format(stream, "%-8s%s, %s, %s", opc ? "subw" : "addw", s_regs[d], s_regs[n], imm(imm12));
		return 4 | SUPPORTED;
	case 0x04:
	case 0x0c:
		util::stream_format(stream, "%-8s%s, %s", opc == 4 ? "movw" : "movt", s_regs[d], imm(((op >> 4) & 0xf000) | imm12));
		return 4 | SUPPORTED;
	case 0x10: case 0x12: case 0x18: case 0x1a:
		{
			const bool sh = opc & 2;
			if (sh && !lsb)
				break;
			const unsigned sat = (opc & 8) ? (op & 0x1f) : (op & 0x1f) + 1;
			util::stream_format(stream, "%-8s%s, #%u, %s%s", (opc & 8) ? "usat" : "ssat", s_regs[d], sat, s_regs[n],
					sh ? util::string_format(", asr #%u", lsb) : lsb ? util::string_format(", lsl #%u", lsb) : std::string());
			return 4 | SUPPORTED;
		}
	case 0x14:
	case 0x1c:
		util::stream_format(stream, "%-8s%s, %s, #%u, #%u", opc == 0x14 ? "sbfx" : "ubfx", s_regs[d], s_regs[n], lsb, (op & 0x1f) + 1);
		return 4 | SUPPORTED;
	case 0x16:
		{
			const unsigned msb = op & 0x1f;
			const unsigned width = msb >= lsb ? msb - lsb + 1 : 0;
			if (n == 15)
				util::stream_format(stream, "%-8s%s, #%u, #%u", "bfc", s_regs[d], lsb, width);
			else
				util::stream_format(stream, "%-8s%s, %s, #%u, #%u", "bfi", s_regs[d], s_regs[n], lsb, width);
			return 4 | SUPPORTED;
		}
	}
	util::stream_format(stream, "undefined 0x%08x", op);
	return 4 | SUPPORTED;
}

offs_t armv7m_disassembler::dasm_branch_misc(std::ostream &stream, offs_t pc, u32 op)
{
	const unsigned op1 = (op >> 12) & 7;
	const unsigned hwop = (op >> 20) & 0x7f;

	if ((op1 & 5) == 0)
	{
		if ((hwop & 0x38) != 0x38)
		{
			const u32 offset = ((op >> 26) & 1) << 20 | ((op >> 11) & 1) << 19 | ((op >> 13) & 1) << 18 | ((op >> 16) & 0x3f) << 12 | (op & 0x7ff) << 1;
			util::stream_format(stream, "b%s%-*s0x%08x", s_conds[(op >> 22) & 0xf], 5, ".w", pc + 4 + util::sext(offset, 21));
			return 4 | SUPPORTED | STEP_COND;
		}
		switch (hwop)
		{
		case 0x38: case 0x39:
			util::stream_format(stream, "%-8s%s, %s", "msr", spec_reg(op & 0xff, (op >> 10) & 3, true), s_regs[(op >> 16) & 0xf]);
			return 4 | SUPPORTED;
		case 0x3a:
			if (!((op >> 8) & 7))
			{
				const unsigned hint = op & 0xff;
				static const char *const names[5] = { "nop.w", "yield.w", "wfe.w", "wfi.w", "sev.w" };
				if (hint < 5)
					util::stream_format(stream, "%s", names[hint]);
				else if (hint == 0x14)
					util::stream_format(stream, "csdb");
				else if ((hint & 0xf0) == 0xf0)
					util::stream_format(stream, "%-8s#%u", "dbg", hint & 0xf);
				else
					util::stream_format(stream, "%-8s%s", "hint.w", imm(hint));
				return 4 | SUPPORTED;
			}
			break;
		case 0x3b:
			{
				static const char *const barriers[16] = {
					"#0", "#1", "oshst", "osh", "#4", "#5", "nshst", "nsh",
					"#8", "#9", "ishst", "ish", "#12", "#13", "st", "sy"
				};
				const unsigned option = op & 0xf;
				switch ((op >> 4) & 0xf)
				{
				case 2:
					util::stream_format(stream, "clrex");
					return 4 | SUPPORTED;
				case 4:
					if (option == 0)
						util::stream_format(stream, "ssbb");
					else if (option == 4)
						util::stream_format(stream, "pssbb");
					else
						util::stream_format(stream, "%-8s%s", "dsb", barriers[option]);
					return 4 | SUPPORTED;
				case 5:
					util::stream_format(stream, "%-8s%s", "dmb", barriers[option]);
					return 4 | SUPPORTED;
				case 6:
					util::stream_format(stream, "%-8s%s", "isb", barriers[option]);
					return 4 | SUPPORTED;
				}
				break;
			}
		case 0x3e: case 0x3f:
			util::stream_format(stream, "%-8s%s, %s", "mrs", s_regs[(op >> 8) & 0xf], spec_reg(op & 0xff, 0, false));
			return 4 | SUPPORTED;
		case 0x7f:
			if (op1 == 2)
			{
				util::stream_format(stream, "%-8s%s", "udf.w", imm(((op >> 4) & 0xf000) | (op & 0xfff)));
				return 4 | SUPPORTED;
			}
			break;
		}
		util::stream_format(stream, "undefined 0x%08x", op);
		return 4 | SUPPORTED;
	}

	if ((op1 & 5) == 4)
	{
		util::stream_format(stream, "undefined 0x%08x", op);
		return 4 | SUPPORTED;
	}

	const u32 s = (op >> 26) & 1;
	const u32 i1 = ~((op >> 13) ^ s) & 1;
	const u32 i2 = ~((op >> 11) ^ s) & 1;
	const u32 offset = s << 24 | i1 << 23 | i2 << 22 | ((op >> 16) & 0x3ff) << 12 | (op & 0x7ff) << 1;
	const u32 target = pc + 4 + util::sext(offset, 25);
	if (op1 & 4)
	{
		util::stream_format(stream, "%-8s0x%08x", "bl", target);
		return 4 | SUPPORTED | STEP_OVER;
	}
	util::stream_format(stream, "%-8s0x%08x", "b.w", target);
	return 4 | SUPPORTED;
}

offs_t armv7m_disassembler::dasm_store(std::ostream &stream, u32 op)
{
	static const char *const names[3] = { "strb", "strh", "str" };
	const unsigned op1 = (op >> 21) & 7;
	const unsigned n = (op >> 16) & 0xf;
	const unsigned t = (op >> 12) & 0xf;
	if ((op1 & 3) == 3 || n == 15)
	{
		util::stream_format(stream, "undefined 0x%08x", op);
		return 4 | SUPPORTED;
	}
	const std::string name = names[op1 & 3];

	if (op1 & 4)
	{
		util::stream_format(stream, "%-8s%s, %s", name + ".w", s_regs[t], mem(s_regs[n], op & 0xfff));
		return 4 | SUPPORTED;
	}
	if (op & 0x800)
	{
		const bool p = op & 0x400;
		const bool u = op & 0x200;
		const bool w = op & 0x100;
		const std::string off = simm(u ? s32(op & 0xff) : -s32(op & 0xff));
		if (!p && !w)
			util::stream_format(stream, "undefined 0x%08x", op);
		else if (p && u && !w)
			util::stream_format(stream, "%-8s%s, [%s, %s]", name + "t", s_regs[t], s_regs[n], off);
		else if (n == 13 && p && !u && w && (op & 0xff) == 4 && op1 == 2)
			util::stream_format(stream, "%-8s{%s}", "push.w", s_regs[t]);
		else if (p)
			util::stream_format(stream, "%-8s%s, [%s, %s]%s", name, s_regs[t], s_regs[n], off, w ? "!" : "");
		else
			util::stream_format(stream, "%-8s%s, [%s], %s", name, s_regs[t], s_regs[n], off);
		return 4 | SUPPORTED;
	}
	if (op & 0x7c0)
	{
		util::stream_format(stream, "undefined 0x%08x", op);
		return 4 | SUPPORTED;
	}
	const unsigned sh = (op >> 4) & 3;
	util::stream_format(stream, "%-8s%s, [%s, %s%s]", name + ".w", s_regs[t], s_regs[n], s_regs[op & 0xf], sh ? util::string_format(", lsl #%u", sh) : std::string());
	return 4 | SUPPORTED;
}

offs_t armv7m_disassembler::dasm_load(std::ostream &stream, offs_t pc, u32 op)
{
	const bool sign = op & 0x01000000;
	const bool imm12_form = op & 0x00800000;
	const unsigned sz = (op >> 21) & 3;
	const unsigned n = (op >> 16) & 0xf;
	const unsigned t = (op >> 12) & 0xf;
	if (sz == 3 || (sz == 2 && sign))
	{
		util::stream_format(stream, "undefined 0x%08x", op);
		return 4 | SUPPORTED;
	}

	static const char *const names[2][3] = { { "ldrb", "ldrh", "ldr" }, { "ldrsb", "ldrsh", nullptr } };
	std::string name = names[sign][sz];
	const unsigned form = (op >> 6) & 0x3f;
	const bool wback_or_unpriv = n != 15 && !imm12_form && ((form & 0x24) == 0x24 || (form & 0x3c) == 0x38);
	const bool hint = t == 15 && sz != 2 && !wback_or_unpriv && !(n == 15 && sz == 1 && !sign);
	if (hint)
		name = sz == 0 ? (sign ? "pli" : "pld") : "hint";
	u32 flags = 4 | SUPPORTED;

	std::string operand = hint ? "" : util::string_format("%s, ", s_regs[t]);
	if (n == 15)
	{
		const u32 offset = op & 0xfff;
		const u32 base = (pc + 4) & ~3;
		util::stream_format(stream, "%-8s%s[pc, %s] ; 0x%08x", hint ? name : name + ".w", operand, simm(imm12_form ? s32(offset) : -s32(offset)), imm12_form ? base + offset : base - offset);
		return flags;
	}
	if (imm12_form)
	{
		util::stream_format(stream, "%-8s%s%s", hint ? name : name + ".w", operand, mem(s_regs[n], op & 0xfff));
		return flags;
	}
	const unsigned op2 = (op >> 6) & 0x3f;
	if (op2 == 0)
	{
		const unsigned sh = (op >> 4) & 3;
		util::stream_format(stream, "%-8s%s[%s, %s%s]", hint ? name : name + ".w", operand, s_regs[n], s_regs[op & 0xf], sh ? util::string_format(", lsl #%u", sh) : std::string());
		return flags;
	}
	if ((op2 & 0x24) == 0x24 || (op2 & 0x3c) == 0x30 || (op2 & 0x3c) == 0x38)
	{
		const bool p = op & 0x400;
		const bool u = op & 0x200;
		const bool w = op & 0x100;
		const std::string off = simm(u ? s32(op & 0xff) : -s32(op & 0xff));
		if ((op2 & 0x3c) == 0x38)
			util::stream_format(stream, "%-8s%s[%s, %s]", name + "t", operand, s_regs[n], off);
		else if (t == 15 && sz == 2 && n == 13 && !p && u && w && (op & 0xff) == 4)
		{
			util::stream_format(stream, "%-8s{pc}", "pop.w");
			flags |= STEP_OUT;
		}
		else if (n == 13 && !p && u && w && (op & 0xff) == 4 && sz == 2)
			util::stream_format(stream, "%-8s{%s}", "pop.w", s_regs[t]);
		else if (p)
			util::stream_format(stream, "%-8s%s[%s, %s]%s", name, operand, s_regs[n], off, w ? "!" : "");
		else
			util::stream_format(stream, "%-8s%s[%s], %s", name, operand, s_regs[n], off);
		return flags;
	}
	util::stream_format(stream, "undefined 0x%08x", op);
	return flags;
}

offs_t armv7m_disassembler::dasm_dp_register(std::ostream &stream, u32 op)
{
	const unsigned op1 = (op >> 20) & 0xf;
	const unsigned op2 = (op >> 4) & 0xf;
	const unsigned n = (op >> 16) & 0xf;
	const unsigned d = (op >> 8) & 0xf;
	const unsigned m = op & 0xf;

	if ((op & 0xf000) == 0xf000)
	{
		if (op1 < 8 && op2 == 0)
		{
			util::stream_format(stream, "%s%-*s%s, %s, %s", s_shifts[op1 >> 1], 5, (op1 & 1) ? "s.w" : ".w", s_regs[d], s_regs[n], s_regs[m]);
			return 4 | SUPPORTED;
		}
		if (op1 < 8 && (op2 & 8) && n == 15 && op1 != 2 && op1 != 3 && op1 < 6)
		{
			static const char *const names[6] = { "sxth.w", "uxth.w", nullptr, nullptr, "sxtb.w", "uxtb.w" };
			const unsigned rot = (op >> 4) & 3;
			util::stream_format(stream, "%-8s%s, %s%s", names[op1], s_regs[d], s_regs[m], rot ? util::string_format(", ror #%u", rot * 8) : std::string());
			return 4 | SUPPORTED;
		}
		if ((op1 & 0xc) == 0x8 && (op2 & 0xc) == 0x8)
		{
			static const char *const names[16] = {
				nullptr, nullptr, nullptr, nullptr, "rev.w", "rev16.w", "rbit", "revsh.w",
				nullptr, nullptr, nullptr, nullptr, "clz", nullptr, nullptr, nullptr
			};
			const char *name = names[((op1 & 3) << 2) | (op2 & 3)];
			if (name)
			{
				util::stream_format(stream, "%-8s%s, %s", name, s_regs[d], s_regs[m]);
				return 4 | SUPPORTED;
			}
		}
	}
	util::stream_format(stream, "undefined 0x%08x", op);
	return 4 | SUPPORTED;
}

offs_t armv7m_disassembler::dasm_multiply(std::ostream &stream, u32 op)
{
	const unsigned op1 = (op >> 20) & 7;
	const unsigned op2 = (op >> 4) & 3;
	const unsigned n = (op >> 16) & 0xf;
	const unsigned a = (op >> 12) & 0xf;
	const unsigned d = (op >> 8) & 0xf;
	const unsigned m = op & 0xf;
	if (!(op & 0xc0) && op1 == 0)
	{
		if (op2 == 0 && a == 15)
		{
			util::stream_format(stream, "%-8s%s, %s, %s", "mul.w", s_regs[d], s_regs[n], s_regs[m]);
			return 4 | SUPPORTED;
		}
		if (op2 <= 1)
		{
			util::stream_format(stream, "%-8s%s, %s, %s, %s", op2 ? "mls" : "mla", s_regs[d], s_regs[n], s_regs[m], s_regs[a]);
			return 4 | SUPPORTED;
		}
	}
	util::stream_format(stream, "undefined 0x%08x", op);
	return 4 | SUPPORTED;
}

offs_t armv7m_disassembler::dasm_long_multiply(std::ostream &stream, u32 op)
{
	const unsigned op1 = (op >> 20) & 7;
	const unsigned op2 = (op >> 4) & 0xf;
	const unsigned n = (op >> 16) & 0xf;
	const unsigned lo = (op >> 12) & 0xf;
	const unsigned hi = (op >> 8) & 0xf;
	const unsigned m = op & 0xf;
	if ((op1 == 1 || op1 == 3) && op2 == 0xf)
	{
		util::stream_format(stream, "%-8s%s, %s, %s", op1 == 1 ? "sdiv" : "udiv", s_regs[hi], s_regs[n], s_regs[m]);
		return 4 | SUPPORTED;
	}
	if (op2 == 0 && (op1 == 0 || op1 == 2 || op1 == 4 || op1 == 6))
	{
		static const char *const names[4] = { "smull", "umull", "smlal", "umlal" };
		util::stream_format(stream, "%-8s%s, %s, %s, %s", names[op1 >> 1], s_regs[lo], s_regs[hi], s_regs[n], s_regs[m]);
		return 4 | SUPPORTED;
	}
	util::stream_format(stream, "undefined 0x%08x", op);
	return 4 | SUPPORTED;
}
