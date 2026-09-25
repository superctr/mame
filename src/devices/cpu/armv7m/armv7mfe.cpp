// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    armv7mfe.cpp

    Front end for the ARMv7-M recompiler.

***************************************************************************/

#include "emu.h"
#include "armv7mfe.h"

#include "cpu/drcfe.ipp"


armv7m_device::frontend::frontend(armv7m_device &cpu, u32 window_start, u32 window_end, u32 max_sequence)
	: drc_frontend_base<opcode_desc>(0, window_start, window_end, max_sequence)
	, m_cpu(cpu)
	, m_start_pc(0)
	, m_start_it(0)
	, m_start_priv(false)
{
}

armv7m_device::frontend::~frontend()
{
}

const armv7m_device::opcode_desc *armv7m_device::frontend::describe_code(offs_t startpc, u8 start_it, bool priv)
{
	m_start_pc = startpc;
	m_start_it = start_it;
	m_start_priv = priv;
	return do_describe_code(
			[this] (opcode_desc &desc, const opcode_desc *prev) { return describe(desc, prev); },
			startpc);
}

bool armv7m_device::frontend::fetchable(offs_t pc) const
{
	return m_cpu.drc_fetchable(pc, m_start_priv);
}

bool armv7m_device::frontend::describe(opcode_desc &desc, const opcode_desc *prev)
{
	if (prev)
		desc.it = prev->it_next;
	else
		desc.it = (desc.pc == m_start_pc) ? m_start_it : 0;
	desc.it_next = (desc.it & 7) ? ((desc.it & 0xe0) | ((desc.it << 1) & 0x1f)) : 0;
	desc.length = 2;

	if ((desc.pc & 1) || !fetchable(desc.pc))
	{
		desc.set_end_sequence();
		return true;
	}

	const u16 op = m_cpu.m_cache.read_word(desc.pc);
	if ((op & 0xe000) == 0xe000 && (op & 0x1800))
	{
		desc.length = 4;
		if (!fetchable(desc.pc + 2))
		{
			desc.set_end_sequence();
			return true;
		}
		desc.opcode = (u32(op) << 16) | m_cpu.m_cache.read_word(desc.pc + 2);
		desc.fetched = true;
		describe_t32(desc);
	}
	else
	{
		desc.opcode = op;
		desc.fetched = true;
		describe_t16(desc);
	}
	return true;
}

void armv7m_device::frontend::describe_t16(opcode_desc &desc)
{
	const u16 op = desc.opcode;
	const bool in_it = desc.it & 0xf;

	if ((op & 0xff00) == 0xbf00 && (op & 0xf))
	{
		if (in_it)
			desc.set_end_sequence();
		else
			desc.it_next = op & 0xff;
		return;
	}

	if ((op & 0xf800) == 0xe000)
	{
		desc.targetpc = desc.pc + 4 + util::sext(u32(op & 0x7ff) << 1, 12);
		if (in_it)
			desc.set_is_conditional_branch();
		else
		{
			desc.set_is_unconditional_branch();
			desc.set_end_sequence();
		}
		return;
	}

	if ((op & 0xf000) == 0xd000)
	{
		const unsigned cond = (op >> 8) & 0xf;
		if (cond >= 14)
		{
			desc.set_end_sequence();
			return;
		}
		if (!in_it)
		{
			desc.targetpc = desc.pc + 4 + util::sext(u32(op & 0xff) << 1, 9);
			desc.set_is_conditional_branch();
		}
		return;
	}

	if ((op & 0xf500) == 0xb100)
	{
		if (!in_it)
		{
			desc.targetpc = desc.pc + 4 + (((op >> 2) & 0x3e) | ((op >> 3) & 0x40));
			desc.set_is_conditional_branch();
		}
		return;
	}

	if ((op & 0xff00) == 0x4700 || (op & 0xff87) == 0x4687 || (op & 0xff87) == 0x4487 || (op & 0xff00) == 0xbd00)
	{
		if (in_it)
			desc.set_is_conditional_branch();
		else
		{
			desc.set_is_unconditional_branch();
			desc.set_end_sequence();
		}
		return;
	}

	if ((op & 0xff00) == 0xbe00)
		desc.set_end_sequence();
}

void armv7m_device::frontend::describe_t32(opcode_desc &desc)
{
	const u32 op = desc.opcode;
	const bool in_it = desc.it & 0xf;
	bool dynamic = false;

	if ((op & 0xf8008000) == 0xf0008000)
	{
		const unsigned op1 = (op >> 12) & 7;
		const unsigned hwop = (op >> 20) & 0x7f;
		if ((op1 & 5) == 0)
		{
			if ((hwop & 0x38) != 0x38 && !in_it)
			{
				const u32 imm = ((op >> 26) & 1) << 20 | ((op >> 11) & 1) << 19 | ((op >> 13) & 1) << 18 | ((op >> 16) & 0x3f) << 12 | (op & 0x7ff) << 1;
				desc.targetpc = desc.pc + 4 + util::sext(imm, 21);
				desc.set_is_conditional_branch();
			}
			return;
		}
		if ((op1 & 5) == 4)
		{
			desc.set_end_sequence();
			return;
		}
		const u32 s = (op >> 26) & 1;
		const u32 i1 = ~((op >> 13) ^ s) & 1;
		const u32 i2 = ~((op >> 11) ^ s) & 1;
		const u32 imm = s << 24 | i1 << 23 | i2 << 22 | ((op >> 16) & 0x3ff) << 12 | (op & 0x7ff) << 1;
		desc.targetpc = desc.pc + 4 + util::sext(imm, 25);
		if (in_it)
			desc.set_is_conditional_branch();
		else
		{
			desc.set_is_unconditional_branch();
			desc.set_end_sequence();
		}
		return;
	}

	switch ((op >> 27) & 3)
	{
	case 1:
		if (!(op & 0x06000000))
		{
			if (!(op & 0x00400000))
				dynamic = (op & 0x00108000) == 0x00108000;
			else
				dynamic = (op & 0xfff0ffe0) == 0xe8d0f000;
		}
		break;

	case 3:
		{
			const u32 op2 = (op >> 20) & 0x7f;
			if ((op2 & 0x61) == 0x01 && (op2 & 0x40) == 0)
				dynamic = ((op >> 12) & 0xf) == 15 && ((op >> 21) & 3) == 2 && !(op & 0x01000000);
			break;
		}
	}

	if (dynamic)
	{
		if (in_it)
			desc.set_is_conditional_branch();
		else
		{
			desc.set_is_unconditional_branch();
			desc.set_end_sequence();
		}
	}
}
