// license:BSD-3-Clause
// copyright-holders:superctr
// thanks-to: giulioz
#ifndef MAME_SOUND_ROLAND_CSPD_H
#define MAME_SOUND_ROLAND_CSPD_H

#pragma once

class roland_csp_disassembler : public util::disasm_interface
{
public:
	struct instruction
	{
		u16 offset;
		u8 store, opcode, scale, bit18, eram_start, eram_nib;
	};
	static instruction decode(u32 word);
	virtual u32 opcode_alignment() const override { return 1; }
	virtual offs_t disassemble(std::ostream &stream, offs_t pc, const data_buffer &opcodes, const data_buffer &params) override;
};

#endif
