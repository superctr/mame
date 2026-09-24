// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    armv7mdasm.h

    ARMv7-M Thumb-2 disassembler.

***************************************************************************/

#ifndef MAME_CPU_ARMV7M_ARMV7MDASM_H
#define MAME_CPU_ARMV7M_ARMV7MDASM_H

#pragma once

class armv7m_disassembler : public util::disasm_interface
{
public:
	armv7m_disassembler() = default;
	virtual ~armv7m_disassembler() = default;

	virtual u32 opcode_alignment() const override;
	virtual offs_t disassemble(std::ostream &stream, offs_t pc, const data_buffer &opcodes, const data_buffer &params) override;

private:
	static const char *const s_regs[16];
	static const char *const s_conds[16];
	static const char *const s_shifts[4];

	static std::string imm(u32 value);
	static std::string simm(s32 value);
	static std::string mem(const char *base, u32 offset);
	static std::string reglist(u32 registers);
	static std::string shift_imm(int type, unsigned imm5);
	static std::string spec_reg(unsigned sysm, unsigned mask, bool write);
	static u32 thumb_expand_imm(u32 imm12);

	offs_t dasm16(std::ostream &stream, offs_t pc, u16 op);
	offs_t dasm32(std::ostream &stream, offs_t pc, u32 op);
	offs_t dasm_ldm_stm(std::ostream &stream, u32 op);
	offs_t dasm_dual_exclusive(std::ostream &stream, offs_t pc, u32 op);
	offs_t dasm_dp_shifted(std::ostream &stream, u32 op);
	offs_t dasm_dp_modified(std::ostream &stream, u32 op);
	offs_t dasm_dp_plain(std::ostream &stream, offs_t pc, u32 op);
	offs_t dasm_branch_misc(std::ostream &stream, offs_t pc, u32 op);
	offs_t dasm_store(std::ostream &stream, u32 op);
	offs_t dasm_load(std::ostream &stream, offs_t pc, u32 op);
	offs_t dasm_dp_register(std::ostream &stream, u32 op);
	offs_t dasm_multiply(std::ostream &stream, u32 op);
	offs_t dasm_long_multiply(std::ostream &stream, u32 op);
	offs_t dasm_coprocessor(std::ostream &stream, u32 op);
};

#endif // MAME_CPU_ARMV7M_ARMV7MDASM_H
