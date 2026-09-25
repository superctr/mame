// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    armv7mfe.h

    Front end for the ARMv7-M recompiler.

***************************************************************************/

#ifndef MAME_CPU_ARMV7M_ARMV7MFE_H
#define MAME_CPU_ARMV7M_ARMV7MFE_H

#pragma once

#include "armv7m.h"

#include "cpu/drcfe.h"


class armv7m_device::opcode_desc : public opcode_desc_base<opcode_desc, 1>
{
public:
	u32 opcode;
	u8 it;
	u8 it_next;
	bool fetched;

	bool is_t32() const { return length == 4; }

	void reset(offs_t curpc, bool in_delay_slot)
	{
		opcode_desc_base::reset(curpc, in_delay_slot);
		opcode = 0;
		it = 0;
		it_next = 0;
		fetched = false;
	}
};


class armv7m_device::frontend : public drc_frontend_base<opcode_desc>
{
public:
	frontend(armv7m_device &cpu, u32 window_start, u32 window_end, u32 max_sequence);
	~frontend();

	const opcode_desc *describe_code(offs_t startpc, u8 start_it, bool priv);

private:
	bool describe(opcode_desc &desc, const opcode_desc *prev);
	bool fetchable(offs_t pc) const;
	void describe_t16(opcode_desc &desc);
	void describe_t32(opcode_desc &desc);

	armv7m_device &m_cpu;
	offs_t m_start_pc;
	u8 m_start_it;
	bool m_start_priv;
};

#endif // MAME_CPU_ARMV7M_ARMV7MFE_H
