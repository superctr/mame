// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    armv7mdrc.cpp

    Recompiler for the ARMv7-M core.

***************************************************************************/

#include "emu.h"
#include "armv7m.h"
#include "armv7mdasm.h"
#include "armv7mfe.h"

#include "cpu/drcumlsh.h"

#include "emuopts.h"

#include <bit>
#include <cstdlib>
#include <map>


namespace {

constexpr size_t CACHE_SIZE = 32 * 1024 * 1024;
constexpr u32 COMPILE_BACKWARDS_BYTES = 128;
constexpr u32 COMPILE_FORWARDS_BYTES = 512;
constexpr u32 COMPILE_MAX_SEQUENCE = 64;
constexpr u32 COMPILE_MAX_INSTRUCTIONS = 16384;

enum : u32
{
	EXIT_NORMAL = 0,
	EXIT_MISSING_CODE,
	EXIT_RESOLVE,
	EXIT_INTERP
};

enum : int
{
	HOOK_NONE = 0,
	HOOK_RECORD,
	HOOK_REPLAY,
	HOOK_WATCH
};

enum : u32
{
	STATUS_OK = 0,
	STATUS_RESOLVE = 1,
	STATUS_EXIT = 2
};

inline void alloc_handle(drcuml_state &drcuml, uml::code_handle *&handleptr, const char *name)
{
	if (!handleptr)
		handleptr = drcuml.handle_alloc(name);
}

} // anonymous namespace


struct armv7m_device::compiler_state
{
	u32 labelnum = 1;
	u32 skip = 0;
	u32 seq_lo = 0;
	u32 seq_hi = 0;
	const opcode_desc *cut = nullptr;
	bool priv = false;
	u32 privmode = 0;
	std::map<u32, const opcode_desc *> descs;

	u32 next_label() { return labelnum++; }
};


struct armv7m_device::verify_state
{
	struct access
	{
		bool write;
		u32 address;
		int size;
		u8 flags;
		u32 value;
		bool ok;
		int fault;
	};

	struct snapshot
	{
		internal_state core;
		u32 sp_other, control, primask, basepri, faultmask;
		u32 cfsr, dfsr, fault_addr;
		bool excl, event, lockup;
		int sleep;
	};

	std::vector<access> log;
	size_t replay_pos = 0;
	snapshot before;
	u64 stamp_before = 0;
	u64 blocks = 0;
	u64 instructions = 0;
	u64 divergences = 0;
};


struct armv7m_device::c_funcs
{
	static void interpret(armv7m_device &cpu) { cpu.drc_interpret(); }
	static void bx_high(armv7m_device &cpu) { cpu.drc_bx_high(); }
	static void irq_check(armv7m_device &cpu) { cpu.drc_irq_check(); }
	template <int Type> static void shift(armv7m_device &cpu)
	{
		u32 carry = cpu.m_core->c;
		cpu.m_core->arg0 = cpu.shift_c(cpu.m_core->arg0, Type, cpu.m_core->arg1 & 0xff, carry);
		cpu.m_core->arg1 = carry;
	}
	template <int Size, u8 Flags> static void read(armv7m_device &cpu) { cpu.drc_mem_read(Size, Flags); }
	template <int Size, u8 Flags> static void write(armv7m_device &cpu) { cpu.drc_mem_write(Size, Flags); }
	template <int Size, u8 Flags> static void log_read(armv7m_device &cpu) { cpu.drc_verify_log(false, cpu.m_core->arg0, Size, Flags, cpu.m_core->arg1); }
	template <int Size, u8 Flags> static void log_write(armv7m_device &cpu) { cpu.drc_verify_log(true, cpu.m_core->arg0, Size, Flags, cpu.m_core->arg1); }
};


//-------------------------------------------------
//  setup
//-------------------------------------------------

void armv7m_device::drc_deleter::operator()(drc_cache *ptr) const { delete ptr; }
void armv7m_device::drc_deleter::operator()(drcuml_state *ptr) const { delete ptr; }
void armv7m_device::drc_deleter::operator()(frontend *ptr) const { delete ptr; }
void armv7m_device::drc_deleter::operator()(verify_state *ptr) const { delete ptr; }

void armv7m_device::drc_start()
{
	m_drc = allow_drc() && !debugger_enabled();
	if (!m_drc)
		return;

	m_drccache.reset(new drc_cache(CACHE_SIZE + sizeof(internal_state)));
	m_drccache->allocate_cache(mconfig().options().drc_rwx());
	m_core = m_drccache->alloc_near<internal_state>();

	m_drc_perm = std::make_unique<u8[]>(1 << 20);

	m_drcuml.reset(new drcuml_state(*this, *m_drccache, 0, 512, 32, 1, COMPILE_FORWARDS_BYTES));
	for (int i = 0; i < 16; i++)
		m_drcuml->symbol_add(&m_core->r[i], sizeof(u32), util::string_format("r%d", i).c_str());
	m_drcuml->symbol_add(&m_core->pc, sizeof(u32), "pc");
	m_drcuml->symbol_add(&m_core->n, sizeof(u32), "n");
	m_drcuml->symbol_add(&m_core->z, sizeof(u32), "z");
	m_drcuml->symbol_add(&m_core->c, sizeof(u32), "c");
	m_drcuml->symbol_add(&m_core->v, sizeof(u32), "v");
	m_drcuml->symbol_add(&m_core->q, sizeof(u32), "q");
	m_drcuml->symbol_add(&m_core->it, sizeof(u32), "it");
	m_drcuml->symbol_add(&m_core->tbit, sizeof(u32), "tbit");
	m_drcuml->symbol_add(&m_core->icount, sizeof(int), "icount");
	m_drcuml->symbol_add(&m_core->check_irq, sizeof(u32), "check_irq");
	m_drcuml->symbol_add(&m_core->limit, sizeof(int), "limit");
	m_drcuml->symbol_add(&m_core->arg0, sizeof(u32), "arg0");
	m_drcuml->symbol_add(&m_core->arg1, sizeof(u32), "arg1");
	m_drcuml->symbol_add(&m_core->arg2, sizeof(u32), "arg2");
	m_drcuml->symbol_add(&m_core->status, sizeof(u32), "status");

	m_drcfe.reset(new frontend(*this, COMPILE_BACKWARDS_BYTES, COMPILE_FORWARDS_BYTES, COMPILE_MAX_SEQUENCE));

	const char *const verify = getenv("ARMV7M_DRC_VERIFY");
	if (verify && atoi(verify))
		m_verify.reset(new verify_state());

	drc_generate_invariant();
	m_drc_dirty = true;
	m_drc_mpu.clear();

	m_drc_notifier = space(AS_PROGRAM).add_change_notifier(
			[this] (read_or_write mode)
			{
				m_drc_dirty = true;
				m_core->smc = 1;
				m_core->check_irq = 1;
			});
}

void armv7m_device::drc_stop()
{
	if (m_verify)
		osd_printf_info("%s: recompiler verify: %d runs, %d instructions, %d divergences\n", tag(),
				m_verify->blocks, m_verify->instructions, m_verify->divergences);
	m_drcfe.reset();
	m_drcuml.reset();
}

void armv7m_device::drc_flush()
{
	if (m_drc)
	{
		m_drc_dirty = true;
		m_drc_stamp = total_cycles();
	}
}


//-------------------------------------------------
//  timing
//-------------------------------------------------

u64 armv7m_device::drc_stamp() const
{
	return executing() ? total_cycles() : m_drc_outside - m_core->icount;
}

void armv7m_device::drc_sync()
{
	const u64 now = drc_stamp();
	if (now != m_drc_stamp)
	{
		const u64 delta = now - m_drc_stamp;
		m_drc_stamp = now;
		advance_cycles(int(delta));
	}
}

u32 armv7m_device::drc_cycles_to_event() const
{
	if (!(m_syst_csr & 1) || !(m_syst_csr & 2) || (!m_syst_cvr && !m_syst_rvr))
		return ~u32(0);
	return systick_cycles_to_event();
}

void armv7m_device::drc_update_limit()
{
	const s64 limit = s64(m_core->icount) - s64(drc_cycles_to_event());
	m_core->limit = int(std::max<s64>(limit, 0));
}


bool armv7m_device::drc_fetchable(u32 pc, bool priv) const
{
	if (!mpu_permits(pc, true, false, priv, m_mpu_ctrl & 1))
		return false;
	return const_cast<armv7m_device *>(this)->space(AS_PROGRAM).get_read_ptr(pc & ~3) != nullptr;
}

bool armv7m_device::drc_mpu_unchanged() const
{
	bool same = m_drc_mpu.size() == 1 + 2 * m_mpu_regions && m_drc_mpu[0] == m_mpu_ctrl;
	for (unsigned r = 0; same && r < m_mpu_regions; r++)
		same = m_drc_mpu[1 + 2 * r] == m_mpu_rbar[r] && m_drc_mpu[2 + 2 * r] == m_mpu_rasr[r];
	return same;
}

bool armv7m_device::drc_can_continue()
{
	if (m_core->smc || m_sleep != SLEEP_NONE || m_lockup || !m_core->tbit)
		return false;
	if ((privileged() ? 0x100 : 0) != m_core->privmode || !drc_mpu_unchanged())
		return false;
	if ((m_mpu_ctrl & 1) && !(m_mpu_ctrl & 2) && execution_priority() < 0)
		return false;
	if (m_core->check_irq)
	{
		int prio;
		const int exc = pending_exception(prio);
		if (exc && group_priority(prio) < execution_priority())
			return false;
		m_core->check_irq = 0;
	}
	return true;
}

void armv7m_device::drc_irq_check()
{
	m_core->status = drc_can_continue() ? STATUS_OK : STATUS_EXIT;
}

void armv7m_device::drc_check_mpu()
{
	if (drc_mpu_unchanged())
		return;
	m_drc_mpu.assign(1 + 2 * m_mpu_regions, 0);
	m_drc_mpu[0] = m_mpu_ctrl;
	for (unsigned r = 0; r < m_mpu_regions; r++)
	{
		m_drc_mpu[1 + 2 * r] = m_mpu_rbar[r];
		m_drc_mpu[2 + 2 * r] = m_mpu_rasr[r];
	}

	const bool enabled = m_mpu_ctrl & 1;
	std::vector<u64> cuts{ 0, u64(1) << 32 };
	if (enabled)
	{
		for (unsigned r = 0; r < m_mpu_regions; r++)
		{
			const u32 rasr = m_mpu_rasr[r];
			if (!(rasr & 1))
				continue;
			unsigned lsbit = ((rasr >> 1) & 0x1f) + 1;
			if (lsbit < 5)
				lsbit = 5;
			const u64 size = u64(1) << std::min(lsbit, 32U);
			const u64 start = lsbit >= 32 ? 0 : (m_mpu_rbar[r] & ~u32(size - 1));
			for (int k = 0; k <= 8; k++)
				cuts.push_back(start + (lsbit >= 8 ? k * (size >> 3) : (k ? size : 0)));
		}
	}
	std::sort(cuts.begin(), cuts.end());
	cuts.erase(std::unique(cuts.begin(), cuts.end()), cuts.end());

	std::fill_n(&m_drc_perm[0], 1 << 20, 0);
	for (size_t i = 0; i + 1 < cuts.size(); i++)
	{
		const u64 a = cuts[i], b = cuts[i + 1];
		if (a >= (u64(1) << 32))
			break;
		const u32 address = u32(a);
		const u8 perm =
				(mpu_permits(address, false, false, true, enabled) ? 1 : 0) |
				(mpu_permits(address, false, true, true, enabled) ? 2 : 0) |
				(mpu_permits(address, false, false, false, enabled) ? 4 : 0) |
				(mpu_permits(address, false, true, false, enabled) ? 8 : 0);
		const u64 first = (a + 0xfff) >> 12;
		const u64 last = std::min<u64>(b >> 12, 1 << 20);
		if (first < last)
			std::fill(&m_drc_perm[first], &m_drc_perm[last], perm);
	}
	std::fill(&m_drc_perm[0xe0000000 >> 12], &m_drc_perm[0xe0040000 >> 12], 0);
	if (m_bitband)
	{
		std::fill(&m_drc_perm[0x22000000 >> 12], &m_drc_perm[0x24000000 >> 12], 0);
		std::fill(&m_drc_perm[0x42000000 >> 12], &m_drc_perm[0x44000000 >> 12], 0);
	}
	m_drcuml->reset();
}


//-------------------------------------------------
//  execution
//-------------------------------------------------

void armv7m_device::execute_run_drc()
{
	if (m_drc_dirty)
	{
		m_drcuml->reset();
		m_drc_dirty = false;
	}
	if (!executing())
		m_drc_outside = m_drc_stamp + m_core->icount;
	drc_sync();

	while (m_core->icount > 0)
	{
		if (m_core->check_irq)
		{
			m_core->check_irq = 0;
			m_cycles = 0;
			if (take_interrupt())
			{
				m_core->icount -= m_cycles;
				advance_cycles(m_cycles);
				m_drc_stamp = drc_stamp();
				continue;
			}
		}

		if (m_sleep != SLEEP_NONE || m_lockup)
		{
			const int burn = int(std::min<u32>(m_core->icount, std::max<u32>(systick_cycles_to_event(), 1)));
			m_core->icount -= burn;
			advance_cycles(burn);
			m_drc_stamp = drc_stamp();
			continue;
		}

		if (!m_core->tbit || ((m_mpu_ctrl & 1) && !(m_mpu_ctrl & 2) && execution_priority() < 0))
		{
			step();
			m_drc_stamp = drc_stamp();
			continue;
		}

		if (m_drc_dirty)
		{
			m_drcuml->reset();
			m_drc_dirty = false;
		}
		drc_check_mpu();
		m_core->privmode = privileged() ? 0x100 : 0;
		m_core->smc = 0;
		drc_update_limit();
		m_core->vcount = 0;
		if (m_verify)
			drc_verify_begin();
		const int result = m_drcuml->execute(*m_entry);
		if (m_verify)
			drc_verify_end(result);
		drc_sync();

		switch (result)
		{
		case EXIT_MISSING_CODE:
			drc_compile(m_core->it | m_core->privmode, m_core->pc);
			break;

		case EXIT_RESOLVE:
			step_resolve(m_core->exit_pc);
			m_core->icount -= m_cycles;
			advance_cycles(m_cycles);
			m_drc_stamp = drc_stamp();
			break;

		case EXIT_INTERP:
			step();
			m_drc_stamp = drc_stamp();
			break;
		}
	}
}

void armv7m_device::drc_interpret()
{
	const u32 pc = m_core->pc;
	const u32 op = m_core->arg0;
	const unsigned length = m_core->arg1 & 0xff;
	const u32 expected_it = m_core->arg1 >> 8;

	drc_sync();
	m_cycles = 1;
	m_fault_exc = 0;
	m_post_exc = 0;
	m_exc_return_pending = false;
	m_it_next = (m_core->it & 7) ? ((m_core->it & 0xe0) | ((m_core->it << 1) & 0x1f)) : 0;
	m_next_pc = pc + length;
	const int hook = m_drc_hook;
	if (hook == HOOK_NONE)
		m_drc_hook = HOOK_WATCH;
	if (length == 4)
	{
		if (!in_it() || condition(m_core->it >> 4))
			execute_t32(op);
	}
	else if (!in_it() || condition(m_core->it >> 4) || (op & 0xff00) == 0xbe00)
		execute_t16(op);
	m_drc_hook = hook;
	m_core->code_lo = m_core->code_hi = 0;

	m_core->exit_pc = pc;
	if (m_fault_exc || m_exc_return_pending || m_post_exc)
	{
		if (!m_fault_exc)
		{
			m_core->pc = m_next_pc;
			m_core->it = m_it_next;
		}
		m_core->status = STATUS_RESOLVE;
		return;
	}

	m_core->pc = m_next_pc;
	m_core->it = m_it_next;
	drc_update_limit();
	m_core->icount -= m_cycles;
	const bool leave = m_core->pc != pc + length || m_core->it != expected_it || !drc_can_continue();
	m_core->status = leave ? STATUS_EXIT : STATUS_OK;
}

void armv7m_device::drc_mem_read(int size, u8 flags)
{
	const u32 address = m_core->arg0;
	const bool ppb = (address >> 18) == (0xe0000000 >> 18) || ((address + size - 1) >> 18) == (0xe0000000 >> 18);
	if (ppb)
		drc_sync();
	u32 value = 0;
	const bool ok = mem_read(address, size, value, flags);
	if (ppb)
		drc_update_limit();
	m_core->arg1 = value;
	if (ok)
		m_core->status = STATUS_OK;
	else
	{
		m_cycles = 1;
		m_post_exc = 0;
		m_exc_return_pending = false;
		m_core->exit_pc = m_core->pc;
		m_core->status = STATUS_RESOLVE;
	}
}

void armv7m_device::drc_mem_write(int size, u8 flags)
{
	const u32 address = m_core->arg0;
	const bool ppb = (address >> 18) == (0xe0000000 >> 18) || ((address + size - 1) >> 18) == (0xe0000000 >> 18);
	if (ppb)
		drc_sync();
	const bool ok = mem_write(address, size, m_core->arg1, flags);
	if (ppb)
		drc_update_limit();
	if (ok)
		m_core->status = STATUS_OK;
	else
	{
		m_cycles = 1;
		m_post_exc = 0;
		m_exc_return_pending = false;
		m_core->exit_pc = m_core->pc;
		m_core->status = STATUS_RESOLVE;
	}
}


//-------------------------------------------------
//  compilation
//-------------------------------------------------

void armv7m_device::drc_compile(u32 mode, offs_t pc)
{
	compiler_state compiler;
	compiler.priv = mode & 0x100;
	compiler.privmode = mode & 0x100;
	const opcode_desc *const desclist = m_drcfe->describe_code(pc, mode & 0xff, compiler.priv);

	const opcode_desc *cut = nullptr;
	u32 estimate = 0;
	for (const opcode_desc *desc = desclist; desc && !cut; desc = desc->next())
	{
		u32 registers = 0;
		if (desc->is_t32() && ((desc->opcode >> 25) & 0x7f) == 0x74)
			registers = desc->opcode & 0xffff;
		else if (!desc->is_t32() && ((desc->opcode & 0xf000) == 0xc000 || (desc->opcode & 0xf600) == 0xb400))
			registers = desc->opcode & 0x1ff;
		const u32 cost = 80 + 24 * std::popcount(registers);
		if (estimate + cost > COMPILE_MAX_INSTRUCTIONS - 512 && desc != desclist)
			cut = desc;
		else
			estimate += cost;
	}
	for (const opcode_desc *desc = desclist; desc != cut; desc = desc->next())
		compiler.descs[desc->pc] = desc;
	compiler.cut = cut;

	bool override = false;
	while (true)
	{
		try
		{
			drcuml_block &block(m_drcuml->begin_block(COMPILE_MAX_INSTRUCTIONS));

			const opcode_desc *seqlast = nullptr;
			for (const opcode_desc *seqhead = desclist; seqhead && seqhead != cut; seqhead = seqlast->next())
			{
				for (seqlast = seqhead; seqlast; seqlast = seqlast->next())
					if (seqlast->end_sequence() || seqlast->next() == cut)
						break;

				const u32 seqmode = seqhead->it | compiler.privmode;
				if (override || !m_drcuml->hash_exists(seqmode, seqhead->pc))
					UML_HASH(block, seqmode, seqhead->pc);
				else if (seqhead == desclist)
				{
					override = true;
					UML_HASH(block, seqmode, seqhead->pc);
				}
				else
				{
					UML_LABEL(block, seqhead->pc | 0x80000000);
					UML_HASHJMP(block, seqmode, seqhead->pc, *m_nocode);
					continue;
				}

				drc_generate_sequence(block, compiler, seqhead, seqlast);
			}

			block.end();
			return;
		}
		catch (drcuml_block::abort_compilation &)
		{
			m_drcuml->reset();
		}
	}
}

void armv7m_device::drc_generate_invariant()
{
	drcuml_block &block(m_drcuml->begin_invariant_block(64));

	alloc_handle(*m_drcuml, m_entry, "entry");
	alloc_handle(*m_drcuml, m_nocode, "nocode");
	alloc_handle(*m_drcuml, m_exit_pc, "exit_pc");
	alloc_handle(*m_drcuml, m_exit_pcset, "exit_pcset");
	alloc_handle(*m_drcuml, m_exit_resolve, "exit_resolve");
	alloc_handle(*m_drcuml, m_exit_interp, "exit_interp");
	alloc_handle(*m_drcuml, m_exit_branch, "exit_branch");

	UML_HANDLE(block, *m_entry);
	UML_OR(block, uml::I0, uml::mem(&m_core->it), uml::mem(&m_core->privmode));
	UML_HASHJMP(block, uml::I0, uml::mem(&m_core->pc), *m_nocode);

	UML_HANDLE(block, *m_nocode);
	UML_GETEXP(block, uml::I0);
	UML_MOV(block, uml::mem(&m_core->pc), uml::I0);
	UML_EXIT(block, EXIT_MISSING_CODE);

	UML_HANDLE(block, *m_exit_pc);
	UML_GETEXP(block, uml::I0);
	UML_MOV(block, uml::mem(&m_core->pc), uml::I0);
	UML_HANDLE(block, *m_exit_pcset);
	UML_EXIT(block, EXIT_NORMAL);

	UML_HANDLE(block, *m_exit_resolve);
	UML_EXIT(block, EXIT_RESOLVE);

	UML_HANDLE(block, *m_exit_interp);
	UML_GETEXP(block, uml::I0);
	UML_MOV(block, uml::mem(&m_core->pc), uml::I0);
	UML_EXIT(block, EXIT_INTERP);

	UML_HANDLE(block, *m_exit_branch);
	UML_CMP(block, uml::mem(&m_core->icount), uml::mem(&m_core->limit));
	UML_EXHc(block, uml::COND_LE, *m_exit_pcset, 0);
	UML_CMP(block, uml::mem(&m_core->tbit), 0);
	UML_EXHc(block, uml::COND_E, *m_exit_pcset, 0);
	UML_CMP(block, uml::mem(&m_core->check_irq), 0);
	UML_JMPc(block, uml::COND_E, 1);
	UML_CALLC(block, &c_funcs::irq_check, this);
	UML_CMP(block, uml::mem(&m_core->status), STATUS_OK);
	UML_EXHc(block, uml::COND_NE, *m_exit_pcset, 0);
	UML_LABEL(block, 1);
	UML_OR(block, uml::I0, uml::mem(&m_core->it), uml::mem(&m_core->privmode));
	UML_HASHJMP(block, uml::I0, uml::mem(&m_core->pc), *m_nocode);

	block.end();
}

void armv7m_device::drc_generate_sequence(drcuml_block &block, compiler_state &compiler, const opcode_desc *seqhead, const opcode_desc *seqlast)
{
	if (seqhead->is_branch_target())
		UML_LABEL(block, seqhead->pc | 0x80000000);
	drc_generate_checksum(block, seqhead, seqlast);

	compiler.seq_lo = seqhead->pc;
	compiler.seq_hi = seqlast->pc + seqlast->length;

	bool native = true;
	for (const opcode_desc *desc = seqhead; desc != seqlast->next(); desc = desc->next())
	{
		native = drc_generate_instruction(block, compiler, desc);
		if (desc == seqlast || !desc->fetched)
			break;
		const opcode_desc *const next = desc->next();
		if (next->pc != desc->pc + desc->length || next->it != desc->it_next)
			drc_generate_goto(block, compiler, desc->it_next, desc->pc + desc->length);
	}

	if (seqlast->fetched && (!seqlast->is_unconditional_branch() || !native))
	{
		const opcode_desc *const next = seqlast->next() != compiler.cut ? seqlast->next() : nullptr;
		const u32 nextpc = seqlast->pc + seqlast->length;
		if (!next || next->pc != nextpc || next->it != seqlast->it_next || m_verify)
			drc_generate_goto(block, compiler, seqlast->it_next, nextpc);
	}
}

void armv7m_device::drc_generate_checksum(drcuml_block &block, const opcode_desc *seqhead, const opcode_desc *seqlast)
{
	std::vector<std::pair<u32, u16>> halfwords;
	for (const opcode_desc *desc = seqhead; desc != seqlast->next(); desc = desc->next())
	{
		if (!desc->fetched)
			continue;
		if (desc->is_t32())
		{
			halfwords.emplace_back(desc->pc, u16(desc->opcode >> 16));
			halfwords.emplace_back(desc->pc + 2, u16(desc->opcode));
		}
		else
			halfwords.emplace_back(desc->pc, u16(desc->opcode));
	}

	address_space &space = this->space(AS_PROGRAM);
	for (size_t i = 0; i < halfwords.size(); )
	{
		const u32 address = halfwords[i].first;
		u8 *const base = reinterpret_cast<u8 *>(space.get_read_ptr(address & ~3)) + (address & 3);
		u8 *const next = reinterpret_cast<u8 *>(space.get_read_ptr((address + 2) & ~3));
		if (i + 1 < halfwords.size() && halfwords[i + 1].first == address + 2 && next && next + ((address + 2) & 3) == base + 2)
		{
			UML_LOAD(block, uml::I0, base, 0, uml::SIZE_DWORD, uml::SCALE_x1);
			UML_CMP(block, uml::I0, u32(halfwords[i].second) | (u32(halfwords[i + 1].second) << 16));
			i += 2;
		}
		else
		{
			UML_LOAD(block, uml::I0, base, 0, uml::SIZE_WORD, uml::SCALE_x1);
			UML_CMP(block, uml::I0, halfwords[i].second);
			i++;
		}
		UML_EXHc(block, uml::COND_NE, *m_nocode, seqhead->pc);
	}
}

bool armv7m_device::drc_generate_instruction(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc)
{
	if (!desc->fetched)
	{
		UML_EXH(block, *m_exit_interp, desc->pc);
		return false;
	}

	if (m_verify)
		UML_ADD(block, uml::mem(&m_core->vcount), uml::mem(&m_core->vcount), 1);

	if (drc_generate_native(block, compiler, desc))
		return true;
	drc_generate_interpret(block, compiler, desc);
	return false;
}

void armv7m_device::drc_generate_interpret(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc)
{
	const uml::code_label ok = compiler.next_label();
	UML_MOV(block, uml::mem(&m_core->pc), desc->pc);
	UML_MOV(block, uml::mem(&m_core->arg0), desc->opcode);
	UML_MOV(block, uml::mem(&m_core->arg1), desc->length | (desc->it_next << 8));
	UML_MOV(block, uml::mem(&m_core->code_lo), compiler.seq_lo);
	UML_MOV(block, uml::mem(&m_core->code_hi), compiler.seq_hi);
	UML_CALLC(block, &c_funcs::interpret, this);
	UML_CMP(block, uml::mem(&m_core->status), STATUS_OK);
	UML_JMPc(block, uml::COND_E, ok);
	UML_CMP(block, uml::mem(&m_core->status), STATUS_RESOLVE);
	UML_EXHc(block, uml::COND_E, *m_exit_resolve, 0);
	UML_EXH(block, *m_exit_pcset, 0);
	UML_LABEL(block, ok);
	UML_CMP(block, uml::mem(&m_core->icount), uml::mem(&m_core->limit));
	UML_EXHc(block, uml::COND_LE, *m_exit_pcset, 0);
}

//-------------------------------------------------
//  native code generation
//-------------------------------------------------

void armv7m_device::drc_generate_begin(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc)
{
	compiler.skip = 0;
	if (desc->it & 0xf)
	{
		const unsigned cond = desc->it >> 4;
		if (cond < 14)
		{
			compiler.skip = compiler.next_label();
			drc_generate_condition(block, cond, compiler.skip);
		}
	}
}

void armv7m_device::drc_generate_end(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc)
{
	if (compiler.skip)
	{
		const uml::code_label done = compiler.next_label();
		UML_JMP(block, done);
		UML_LABEL(block, compiler.skip);
		drc_generate_epilogue(block, compiler, desc, 1, false);
		UML_LABEL(block, done);
		compiler.skip = 0;
	}
}

void armv7m_device::drc_generate_condition(drcuml_block &block, unsigned cond, u32 false_label)
{
	using namespace uml;
	switch (cond)
	{
	case 0: UML_CMP(block, mem(&m_core->z), 0); UML_JMPc(block, COND_E, false_label); break;
	case 1: UML_CMP(block, mem(&m_core->z), 0); UML_JMPc(block, COND_NE, false_label); break;
	case 2: UML_CMP(block, mem(&m_core->c), 0); UML_JMPc(block, COND_E, false_label); break;
	case 3: UML_CMP(block, mem(&m_core->c), 0); UML_JMPc(block, COND_NE, false_label); break;
	case 4: UML_CMP(block, mem(&m_core->n), 0); UML_JMPc(block, COND_E, false_label); break;
	case 5: UML_CMP(block, mem(&m_core->n), 0); UML_JMPc(block, COND_NE, false_label); break;
	case 6: UML_CMP(block, mem(&m_core->v), 0); UML_JMPc(block, COND_E, false_label); break;
	case 7: UML_CMP(block, mem(&m_core->v), 0); UML_JMPc(block, COND_NE, false_label); break;
	case 8:
		UML_CMP(block, mem(&m_core->c), 0);
		UML_JMPc(block, COND_E, false_label);
		UML_CMP(block, mem(&m_core->z), 0);
		UML_JMPc(block, COND_NE, false_label);
		break;
	case 9:
		UML_XOR(block, I3, mem(&m_core->z), 1);
		UML_AND(block, I3, I3, mem(&m_core->c));
		UML_JMPc(block, COND_NZ, false_label);
		break;
	case 10: UML_CMP(block, mem(&m_core->n), mem(&m_core->v)); UML_JMPc(block, COND_NE, false_label); break;
	case 11: UML_CMP(block, mem(&m_core->n), mem(&m_core->v)); UML_JMPc(block, COND_E, false_label); break;
	case 12:
		UML_CMP(block, mem(&m_core->z), 0);
		UML_JMPc(block, COND_NE, false_label);
		UML_CMP(block, mem(&m_core->n), mem(&m_core->v));
		UML_JMPc(block, COND_NE, false_label);
		break;
	case 13:
		UML_XOR(block, I3, mem(&m_core->n), mem(&m_core->v));
		UML_OR(block, I3, I3, mem(&m_core->z));
		UML_JMPc(block, COND_Z, false_label);
		break;
	default:
		break;
	}
}

void armv7m_device::drc_generate_epilogue(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc, int cycles, bool check_irq)
{
	using namespace uml;
	const u32 nextpc = desc->pc + desc->length;
	if (desc->it_next != desc->it)
		UML_MOV(block, mem(&m_core->it), desc->it_next);
	UML_SUB(block, mem(&m_core->icount), mem(&m_core->icount), cycles);
	UML_CMP(block, mem(&m_core->icount), mem(&m_core->limit));
	UML_EXHc(block, COND_LE, *m_exit_pc, nextpc);
	if (check_irq)
	{
		const u32 skip = compiler.next_label();
		UML_CMP(block, mem(&m_core->check_irq), 0);
		UML_JMPc(block, COND_E, skip);
		UML_CALLC(block, &c_funcs::irq_check, this);
		UML_CMP(block, mem(&m_core->status), STATUS_OK);
		UML_EXHc(block, COND_NE, *m_exit_pc, nextpc);
		UML_LABEL(block, skip);
	}
}

void armv7m_device::drc_generate_goto(drcuml_block &block, compiler_state &compiler, u8 it, u32 pc)
{
	if (m_verify)
	{
		UML_EXH(block, *m_exit_pc, pc);
		return;
	}
	const auto target = compiler.descs.find(pc);
	if (target != compiler.descs.end() && target->second->it == it && target->second->is_branch_target())
		UML_JMP(block, pc | 0x80000000);
	else
		UML_HASHJMP(block, it | compiler.privmode, pc, *m_nocode);
}

void armv7m_device::drc_generate_branch(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc, u32 target, int cycles)
{
	using namespace uml;
	if (desc->it_next != desc->it)
		UML_MOV(block, mem(&m_core->it), desc->it_next);
	UML_SUB(block, mem(&m_core->icount), mem(&m_core->icount), cycles);
	UML_CMP(block, mem(&m_core->icount), mem(&m_core->limit));
	UML_EXHc(block, COND_LE, *m_exit_pc, target);
	drc_generate_goto(block, compiler, desc->it_next, target);
}

void armv7m_device::drc_generate_dynamic_branch(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc, int cycles, int kind)
{
	using namespace uml;
	const uml::code_label high = compiler.next_label();
	if (kind == 2)
	{
		UML_CMP(block, I0, 0xf0000000);
		UML_JMPc(block, COND_AE, high);
	}
	if (desc->it_next != desc->it)
		UML_MOV(block, mem(&m_core->it), desc->it_next);
	if (kind != 0)
		UML_AND(block, mem(&m_core->tbit), I0, 1);
	UML_AND(block, mem(&m_core->pc), I0, ~u32(1));
	UML_SUB(block, mem(&m_core->icount), mem(&m_core->icount), cycles);
	UML_EXH(block, m_verify ? *m_exit_pcset : *m_exit_branch, 0);
	if (kind == 2)
	{
		UML_LABEL(block, high);
		UML_MOV(block, mem(&m_core->arg0), I0);
		UML_MOV(block, mem(&m_core->arg1), cycles);
		UML_MOV(block, mem(&m_core->arg2), desc->length | (desc->it_next << 8));
		UML_MOV(block, mem(&m_core->pc), desc->pc);
		UML_CALLC(block, &c_funcs::bx_high, this);
		UML_CMP(block, mem(&m_core->status), STATUS_RESOLVE);
		UML_EXHc(block, COND_E, *m_exit_resolve, 0);
		UML_EXH(block, *m_exit_pcset, 0);
	}
}

void armv7m_device::drc_bx_high()
{
	const u32 value = m_core->arg0;
	const u32 pc = m_core->pc;
	m_cycles = m_core->arg1;
	m_fault_exc = 0;
	m_post_exc = 0;
	m_exc_return_pending = false;
	m_core->exit_pc = pc;
	m_core->it = m_core->arg2 >> 8;
	if (m_handler)
	{
		m_exc_return = value;
		m_exc_return_pending = true;
		m_core->pc = pc + (m_core->arg2 & 0xff);
		m_core->status = STATUS_RESOLVE;
	}
	else
	{
		m_core->tbit = value & 1;
		m_core->pc = value & ~1;
		m_core->icount -= m_cycles;
		m_core->status = STATUS_EXIT;
	}
}

void armv7m_device::drc_load_reg(drcuml_block &block, const uml::parameter &dst, int n, const opcode_desc *desc)
{
	if (n == 15)
		UML_MOV(block, dst, desc->pc + 4);
	else
		UML_MOV(block, dst, uml::mem(&m_core->r[n]));
}

void armv7m_device::drc_generate_set_reg(drcuml_block &block, int d, const uml::parameter &value)
{
	if (d == 13)
		UML_AND(block, uml::mem(&m_core->r[13]), value, ~u32(3));
	else
		UML_MOV(block, uml::mem(&m_core->r[d]), value);
}

void armv7m_device::drc_generate_nz(drcuml_block &block, const uml::parameter &value)
{
	using namespace uml;
	UML_TEST(block, value, value);
	UML_SETc(block, COND_S, mem(&m_core->n));
	UML_SETc(block, COND_Z, mem(&m_core->z));
}

void armv7m_device::drc_generate_flags(drcuml_block &block, bool subtract)
{
	using namespace uml;
	UML_SETc(block, COND_S, mem(&m_core->n));
	UML_SETc(block, COND_Z, mem(&m_core->z));
	UML_SETc(block, subtract ? COND_NC : COND_C, mem(&m_core->c));
	UML_SETc(block, COND_V, mem(&m_core->v));
}

void armv7m_device::drc_generate_fast_check(drcuml_block &block, compiler_state &compiler, const uml::parameter &address, int size, bool write, u32 slow)
{
	using namespace uml;
	if (size > 1)
	{
		UML_TEST(block, address, size - 1);
		UML_JMPc(block, COND_NZ, slow);
	}
	UML_SHR(block, I3, address, 12);
	UML_LOAD(block, I3, &m_drc_perm[0], I3, SIZE_BYTE, SCALE_x1);
	UML_TEST(block, I3, 1 << ((write ? 1 : 0) + (compiler.priv ? 0 : 2)));
	UML_JMPc(block, COND_Z, slow);
}

void armv7m_device::drc_generate_read(drcuml_block &block, compiler_state &compiler, int size, u8 flags)
{
	using namespace uml;
	const operand_size opsize = size == 1 ? SIZE_BYTE : size == 2 ? SIZE_WORD : SIZE_DWORD;
	const uml::code_label slow = compiler.next_label();
	const uml::code_label done = compiler.next_label();
	if (!(flags & MEM_UNPRIV))
	{
		drc_generate_fast_check(block, compiler, I0, size, false, slow);
		UML_READ(block, I2, I0, opsize, SPACE_PROGRAM);
		if (m_verify)
		{
			UML_MOV(block, mem(&m_core->arg0), I0);
			UML_MOV(block, mem(&m_core->arg1), I2);
			UML_CALLC(block, (size == 1 ? &c_funcs::log_read<1, 0> : size == 2 ? &c_funcs::log_read<2, 0> : flags ? &c_funcs::log_read<4, MEM_ALIGNED> : &c_funcs::log_read<4, 0>), this);
		}
		UML_JMP(block, done);
	}
	UML_LABEL(block, slow);
	UML_MOV(block, mem(&m_core->arg0), I0);
	switch (size | (flags << 4))
	{
	case 0x01: UML_CALLC(block, (&c_funcs::read<1, 0>), this); break;
	case 0x02: UML_CALLC(block, (&c_funcs::read<2, 0>), this); break;
	case 0x04: UML_CALLC(block, (&c_funcs::read<4, 0>), this); break;
	case 0x14: UML_CALLC(block, (&c_funcs::read<4, MEM_ALIGNED>), this); break;
	case 0x21: UML_CALLC(block, (&c_funcs::read<1, MEM_UNPRIV>), this); break;
	case 0x22: UML_CALLC(block, (&c_funcs::read<2, MEM_UNPRIV>), this); break;
	case 0x24: UML_CALLC(block, (&c_funcs::read<4, MEM_UNPRIV>), this); break;
	default: throw emu_fatalerror("armv7m: bad read %d/%d\n", size, flags);
	}
	UML_CMP(block, mem(&m_core->status), STATUS_OK);
	UML_EXHc(block, COND_NE, *m_exit_resolve, 0);
	UML_MOV(block, I2, mem(&m_core->arg1));
	UML_LABEL(block, done);
}

void armv7m_device::drc_generate_write(drcuml_block &block, compiler_state &compiler, int size, u8 flags)
{
	using namespace uml;
	const operand_size opsize = size == 1 ? SIZE_BYTE : size == 2 ? SIZE_WORD : SIZE_DWORD;
	const uml::code_label slow = compiler.next_label();
	const uml::code_label done = compiler.next_label();
	if (!(flags & MEM_UNPRIV))
	{
		drc_generate_fast_check(block, compiler, I0, size, true, slow);
		UML_WRITE(block, I0, I2, opsize, SPACE_PROGRAM);
		if (m_verify)
		{
			UML_MOV(block, mem(&m_core->arg0), I0);
			if (size == 4)
				UML_MOV(block, mem(&m_core->arg1), I2);
			else
				UML_AND(block, mem(&m_core->arg1), I2, (1U << (8 * size)) - 1);
			UML_CALLC(block, (size == 1 ? &c_funcs::log_write<1, 0> : size == 2 ? &c_funcs::log_write<2, 0> : flags ? &c_funcs::log_write<4, MEM_ALIGNED> : &c_funcs::log_write<4, 0>), this);
		}
		UML_JMP(block, done);
	}
	UML_LABEL(block, slow);
	UML_MOV(block, mem(&m_core->arg0), I0);
	UML_MOV(block, mem(&m_core->arg1), I2);
	switch (size | (flags << 4))
	{
	case 0x01: UML_CALLC(block, (&c_funcs::write<1, 0>), this); break;
	case 0x02: UML_CALLC(block, (&c_funcs::write<2, 0>), this); break;
	case 0x04: UML_CALLC(block, (&c_funcs::write<4, 0>), this); break;
	case 0x14: UML_CALLC(block, (&c_funcs::write<4, MEM_ALIGNED>), this); break;
	case 0x21: UML_CALLC(block, (&c_funcs::write<1, MEM_UNPRIV>), this); break;
	case 0x22: UML_CALLC(block, (&c_funcs::write<2, MEM_UNPRIV>), this); break;
	case 0x24: UML_CALLC(block, (&c_funcs::write<4, MEM_UNPRIV>), this); break;
	default: throw emu_fatalerror("armv7m: bad write %d/%d\n", size, flags);
	}
	UML_CMP(block, mem(&m_core->status), STATUS_OK);
	UML_EXHc(block, COND_NE, *m_exit_resolve, 0);
	UML_LABEL(block, done);
	drc_generate_code_write_check(block, compiler, I0, size);
}

void armv7m_device::drc_generate_code_write_check(drcuml_block &block, compiler_state &compiler, const uml::parameter &address, u32 bytes)
{
	using namespace uml;
	const u32 skip = compiler.next_label();
	UML_SUB(block, I3, address, compiler.seq_lo - bytes + 1);
	UML_CMP(block, I3, compiler.seq_hi - compiler.seq_lo + bytes - 1);
	UML_JMPc(block, COND_AE, skip);
	UML_MOV(block, mem(&m_core->smc), 1);
	UML_MOV(block, mem(&m_core->check_irq), 1);
	UML_LABEL(block, skip);
}

void armv7m_device::drc_generate_ldm(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc, int n, u32 registers, bool wback, bool decrement)
{
	using namespace uml;
	const unsigned count = std::popcount(registers);
	const uml::code_label slow = compiler.next_label();
	const uml::code_label done = compiler.next_label();

	UML_MOV(block, mem(&m_core->pc), desc->pc);
	drc_load_reg(block, I1, n, desc);
	if (decrement)
		UML_SUB(block, I4, I1, 4 * count);
	else
		UML_MOV(block, I4, I1);

	drc_generate_fast_check(block, compiler, I4, 4, false, slow);
	UML_ADD(block, I0, I4, 4 * (count - 1));
	drc_generate_fast_check(block, compiler, I0, 1, false, slow);
	for (unsigned i = 0, k = 0; i < 16; i++)
	{
		if (!BIT(registers, i))
			continue;
		UML_ADD(block, I0, I4, 4 * k++);
		UML_READ(block, I2, I0, SIZE_DWORD, SPACE_PROGRAM);
		if (m_verify)
		{
			UML_MOV(block, mem(&m_core->arg0), I0);
			UML_MOV(block, mem(&m_core->arg1), I2);
			UML_CALLC(block, (&c_funcs::log_read<4, MEM_ALIGNED>), this);
		}
		UML_MOV(block, mem(&m_core->temp[i]), I2);
	}
	UML_JMP(block, done);

	UML_LABEL(block, slow);
	for (unsigned i = 0, k = 0; i < 16; i++)
	{
		if (!BIT(registers, i))
			continue;
		UML_ADD(block, mem(&m_core->arg0), I4, 4 * k++);
		UML_CALLC(block, (&c_funcs::read<4, MEM_ALIGNED>), this);
		UML_CMP(block, mem(&m_core->status), STATUS_OK);
		UML_EXHc(block, COND_NE, *m_exit_resolve, 0);
		UML_MOV(block, mem(&m_core->temp[i]), mem(&m_core->arg1));
	}

	UML_LABEL(block, done);
	for (int i = 0; i < 15; i++)
	{
		if (!BIT(registers, i))
			continue;
		if (i == 13)
			UML_AND(block, mem(&m_core->r[13]), mem(&m_core->temp[13]), ~u32(3));
		else
			UML_MOV(block, mem(&m_core->r[i]), mem(&m_core->temp[i]));
	}
	if (wback && !BIT(registers, n))
	{
		if (decrement)
			UML_SUB(block, I1, I1, 4 * count);
		else
			UML_ADD(block, I1, I1, 4 * count);
		drc_generate_set_reg(block, n, I1);
	}
	if (BIT(registers, 15))
	{
		UML_MOV(block, I0, mem(&m_core->temp[15]));
		drc_generate_dynamic_branch(block, compiler, desc, 1 + count + 2, 2);
	}
	else
		drc_generate_epilogue(block, compiler, desc, 1 + count, true);
}

void armv7m_device::drc_generate_stm(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc, int n, u32 registers, bool wback, bool decrement)
{
	using namespace uml;
	const unsigned count = std::popcount(registers);
	const uml::code_label slow = compiler.next_label();
	const uml::code_label done = compiler.next_label();

	UML_MOV(block, mem(&m_core->pc), desc->pc);
	drc_load_reg(block, I1, n, desc);
	if (decrement)
		UML_SUB(block, I4, I1, 4 * count);
	else
		UML_MOV(block, I4, I1);

	drc_generate_fast_check(block, compiler, I4, 4, true, slow);
	UML_ADD(block, I0, I4, 4 * (count - 1));
	drc_generate_fast_check(block, compiler, I0, 1, true, slow);
	for (unsigned i = 0, k = 0; i < 16; i++)
	{
		if (!BIT(registers, i))
			continue;
		UML_ADD(block, I0, I4, 4 * k++);
		drc_load_reg(block, I2, i, desc);
		UML_WRITE(block, I0, I2, SIZE_DWORD, SPACE_PROGRAM);
		if (m_verify)
		{
			UML_MOV(block, mem(&m_core->arg0), I0);
			UML_MOV(block, mem(&m_core->arg1), I2);
			UML_CALLC(block, (&c_funcs::log_write<4, MEM_ALIGNED>), this);
		}
	}
	UML_JMP(block, done);

	UML_LABEL(block, slow);
	for (unsigned i = 0, k = 0; i < 16; i++)
	{
		if (!BIT(registers, i))
			continue;
		UML_ADD(block, mem(&m_core->arg0), I4, 4 * k++);
		drc_load_reg(block, mem(&m_core->arg1), i, desc);
		UML_CALLC(block, (&c_funcs::write<4, MEM_ALIGNED>), this);
		UML_CMP(block, mem(&m_core->status), STATUS_OK);
		UML_EXHc(block, COND_NE, *m_exit_resolve, 0);
	}

	UML_LABEL(block, done);
	drc_generate_code_write_check(block, compiler, I4, 4 * count);
	if (wback)
	{
		if (decrement)
			UML_SUB(block, I1, I1, 4 * count);
		else
			UML_ADD(block, I1, I1, 4 * count);
		drc_generate_set_reg(block, n, I1);
	}
	drc_generate_epilogue(block, compiler, desc, 1 + count, true);
}

void armv7m_device::drc_generate_shift_imm(drcuml_block &block, int type, unsigned amount, bool want_carry)
{
	using namespace uml;
	switch (type)
	{
	case 0:
		if (!amount)
			return;
		if (want_carry)
			UML_ROLAND(block, I1, I0, amount, 1);
		UML_SHL(block, I0, I0, amount);
		break;
	case 1:
		if (want_carry)
			UML_ROLAND(block, I1, I0, (33 - amount) & 31, 1);
		if (amount == 32)
			UML_MOV(block, I0, 0);
		else
			UML_SHR(block, I0, I0, amount);
		break;
	case 2:
		if (want_carry)
			UML_ROLAND(block, I1, I0, (33 - amount) & 31, 1);
		UML_SAR(block, I0, I0, std::min(amount, 31U));
		break;
	case 3:
		UML_ROR(block, I0, I0, amount);
		if (want_carry)
			UML_ROLAND(block, I1, I0, 1, 1);
		break;
	default:
		if (want_carry)
			UML_AND(block, I1, I0, 1);
		UML_SHR(block, I0, I0, 1);
		UML_ROLINS(block, I0, mem(&m_core->c), 31, 0x80000000);
		break;
	}
}

void armv7m_device::drc_generate_shift_reg(drcuml_block &block, int type, const uml::parameter &value, const uml::parameter &amount, int d, bool setflags)
{
	using namespace uml;
	UML_MOV(block, mem(&m_core->arg0), value);
	UML_MOV(block, mem(&m_core->arg1), amount);
	switch (type)
	{
	case 0: UML_CALLC(block, &c_funcs::shift<0>, this); break;
	case 1: UML_CALLC(block, &c_funcs::shift<1>, this); break;
	case 2: UML_CALLC(block, &c_funcs::shift<2>, this); break;
	default: UML_CALLC(block, &c_funcs::shift<3>, this); break;
	}
	UML_MOV(block, I0, mem(&m_core->arg0));
	UML_MOV(block, mem(&m_core->r[d]), I0);
	if (setflags)
	{
		drc_generate_nz(block, I0);
		UML_MOV(block, mem(&m_core->c), mem(&m_core->arg1));
	}
}

bool armv7m_device::drc_dp_valid(unsigned opc, int d, int n, int m, bool setflags)
{
	const bool test = d == 15 && setflags && (opc == 0 || opc == 4 || opc == 8 || opc == 13);
	const bool move = (opc == 2 || opc == 3) && n == 15;
	const bool sp_arith = (opc == 8 || opc == 13) && n == 13;
	if (!(opc <= 4 || opc == 8 || opc == 10 || opc == 11 || opc == 13 || opc == 14))
		return false;
	if (d == 15 && !test)
		return false;
	if (d == 13 && !sp_arith && !(opc == 2 && n == 15 && m != -1 && !setflags))
		return false;
	if (n == 15 && !move)
		return false;
	if (n == 13 && !sp_arith)
		return false;
	if (m == 15 || (m == 13 && !sp_arith && !(opc == 2 && n == 15 && !setflags && d != 13)))
		return false;
	return true;
}

void armv7m_device::drc_generate_dp(drcuml_block &block, const opcode_desc *desc, unsigned opc, int d, int n, int carry, bool setflags)
{
	using namespace uml;
	const bool move = (opc == 2 || opc == 3) && n == 15;
	bool logical = true;

	if (!move)
		drc_load_reg(block, I0, n, desc);
	switch (opc)
	{
	case 0: UML_AND(block, I0, I0, I2); break;
	case 1: UML_XOR(block, I3, I2, ~u32(0)); UML_AND(block, I0, I0, I3); break;
	case 2:
		if (move)
			UML_MOV(block, I0, I2);
		else
			UML_OR(block, I0, I0, I2);
		break;
	case 3:
		UML_XOR(block, I3, I2, ~u32(0));
		if (move)
			UML_MOV(block, I0, I3);
		else
			UML_OR(block, I0, I0, I3);
		break;
	case 4: UML_XOR(block, I0, I0, I2); break;
	case 8:
		UML_ADD(block, I0, I0, I2);
		if (setflags)
			drc_generate_flags(block, false);
		logical = false;
		break;
	case 10:
		UML_CARRY(block, mem(&m_core->c), 0);
		UML_ADDC(block, I0, I0, I2);
		if (setflags)
			drc_generate_flags(block, false);
		logical = false;
		break;
	case 11:
		UML_XOR(block, I3, mem(&m_core->c), 1);
		UML_CARRY(block, I3, 0);
		UML_SUBB(block, I0, I0, I2);
		if (setflags)
			drc_generate_flags(block, true);
		logical = false;
		break;
	case 13:
		UML_SUB(block, I0, I0, I2);
		if (setflags)
			drc_generate_flags(block, true);
		logical = false;
		break;
	case 14:
		UML_SUB(block, I0, I2, I0);
		if (setflags)
			drc_generate_flags(block, true);
		logical = false;
		break;
	}
	if (logical && setflags)
	{
		drc_generate_nz(block, I0);
		if (carry == 2)
			UML_MOV(block, mem(&m_core->c), I1);
		else if (carry >= 0)
			UML_MOV(block, mem(&m_core->c), carry);
	}
	if (d != 15)
		drc_generate_set_reg(block, d, I0);
}

bool armv7m_device::drc_generate_native(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc)
{
	return desc->is_t32() ? drc_generate_t32(block, compiler, desc) : drc_generate_t16(block, compiler, desc);
}

bool armv7m_device::drc_generate_t16(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc)
{
	using namespace uml;
	const u16 op = desc->opcode;
	const bool in_it = desc->it & 0xf;
	const bool last_in_it = (desc->it & 0xf) == 0x8;
	const bool setflags = !in_it;
	const u32 pc = desc->pc;

	switch (op >> 12)
	{
	case 0x0: case 0x1: case 0x2: case 0x3:
		{
			const int d = op & 7;
			const int n = (op >> 3) & 7;
			switch ((op >> 11) & 7)
			{
			case 0: case 1: case 2:
				{
					const int type = (op >> 11) & 3;
					const unsigned imm5 = (op >> 6) & 0x1f;
					if (type == 0 && imm5 == 0)
					{
						if (in_it)
							return false;
						drc_generate_begin(block, compiler, desc);
						UML_MOV(block, I0, mem(&m_core->r[n]));
						UML_MOV(block, mem(&m_core->r[d]), I0);
						drc_generate_nz(block, I0);
					}
					else
					{
						drc_generate_begin(block, compiler, desc);
						UML_MOV(block, I0, mem(&m_core->r[n]));
						drc_generate_shift_imm(block, type, imm5 ? imm5 : 32, setflags);
						UML_MOV(block, mem(&m_core->r[d]), I0);
						if (setflags)
						{
							drc_generate_nz(block, I0);
							UML_MOV(block, mem(&m_core->c), I1);
						}
					}
					break;
				}
			case 3:
				drc_generate_begin(block, compiler, desc);
				UML_MOV(block, I0, mem(&m_core->r[n]));
				if (op & 0x400)
					UML_MOV(block, I2, (op >> 6) & 7);
				else
					UML_MOV(block, I2, mem(&m_core->r[(op >> 6) & 7]));
				if (op & 0x200)
					UML_SUB(block, I0, I0, I2);
				else
					UML_ADD(block, I0, I0, I2);
				if (setflags)
					drc_generate_flags(block, op & 0x200);
				UML_MOV(block, mem(&m_core->r[d]), I0);
				break;
			case 4:
				drc_generate_begin(block, compiler, desc);
				UML_MOV(block, mem(&m_core->r[(op >> 8) & 7]), op & 0xff);
				if (setflags)
				{
					UML_MOV(block, mem(&m_core->n), 0);
					UML_MOV(block, mem(&m_core->z), (op & 0xff) ? 0 : 1);
				}
				break;
			case 5:
				drc_generate_begin(block, compiler, desc);
				UML_CMP(block, mem(&m_core->r[(op >> 8) & 7]), op & 0xff);
				drc_generate_flags(block, true);
				break;
			case 6:
			case 7:
				drc_generate_begin(block, compiler, desc);
				if ((op >> 11) & 1)
					UML_SUB(block, I0, mem(&m_core->r[(op >> 8) & 7]), op & 0xff);
				else
					UML_ADD(block, I0, mem(&m_core->r[(op >> 8) & 7]), op & 0xff);
				if (setflags)
					drc_generate_flags(block, (op >> 11) & 1);
				UML_MOV(block, mem(&m_core->r[(op >> 8) & 7]), I0);
				break;
			}
			drc_generate_epilogue(block, compiler, desc, 1, false);
			drc_generate_end(block, compiler, desc);
			return true;
		}

	case 0x4:
		if (op & 0x800)
		{
			drc_generate_begin(block, compiler, desc);
			UML_MOV(block, mem(&m_core->pc), pc);
			UML_MOV(block, I0, ((pc + 4) & ~3) + ((op & 0xff) << 2));
			drc_generate_read(block, compiler, 4, 0);
			UML_MOV(block, mem(&m_core->r[(op >> 8) & 7]), I2);
			drc_generate_epilogue(block, compiler, desc, 2, true);
			drc_generate_end(block, compiler, desc);
			return true;
		}
		if (!(op & 0x400))
		{
			const int dn = op & 7;
			const int m = (op >> 3) & 7;
			const unsigned opc = (op >> 6) & 0xf;
			if (opc == 2 || opc == 3 || opc == 4 || opc == 7)
			{
				drc_generate_begin(block, compiler, desc);
				drc_generate_shift_reg(block, opc == 7 ? 3 : opc - 2, mem(&m_core->r[dn]), mem(&m_core->r[m]), dn, setflags);
				drc_generate_epilogue(block, compiler, desc, 1, false);
				drc_generate_end(block, compiler, desc);
				return true;
			}
			drc_generate_begin(block, compiler, desc);
			UML_MOV(block, I0, mem(&m_core->r[dn]));
			UML_MOV(block, I2, mem(&m_core->r[m]));
			switch (opc)
			{
			case 0x0: UML_AND(block, I0, I0, I2); break;
			case 0x1: UML_XOR(block, I0, I0, I2); break;
			case 0x5:
				UML_CARRY(block, mem(&m_core->c), 0);
				UML_ADDC(block, I0, I0, I2);
				if (setflags)
					drc_generate_flags(block, false);
				break;
			case 0x6:
				UML_XOR(block, I3, mem(&m_core->c), 1);
				UML_CARRY(block, I3, 0);
				UML_SUBB(block, I0, I0, I2);
				if (setflags)
					drc_generate_flags(block, true);
				break;
			case 0x8: UML_AND(block, I0, I0, I2); break;
			case 0x9:
				UML_SUB(block, I0, 0, I2);
				if (setflags)
					drc_generate_flags(block, true);
				break;
			case 0xa: UML_CMP(block, I0, I2); drc_generate_flags(block, true); break;
			case 0xb: UML_ADD(block, I0, I0, I2); drc_generate_flags(block, false); break;
			case 0xc: UML_OR(block, I0, I0, I2); break;
			case 0xd: UML_MULULW(block, I0, I0, I2); break;
			case 0xe: UML_XOR(block, I3, I2, ~u32(0)); UML_AND(block, I0, I0, I3); break;
			case 0xf: UML_XOR(block, I0, I2, ~u32(0)); break;
			}
			if (opc == 0x8)
				drc_generate_nz(block, I0);
			else if (opc != 0xa && opc != 0xb)
			{
				UML_MOV(block, mem(&m_core->r[dn]), I0);
				if (setflags && (opc <= 1 || opc >= 0xc))
					drc_generate_nz(block, I0);
			}
			drc_generate_epilogue(block, compiler, desc, 1, false);
			drc_generate_end(block, compiler, desc);
			return true;
		}
		{
			const int dn = ((op >> 4) & 8) | (op & 7);
			const int m = (op >> 3) & 0xf;
			switch ((op >> 8) & 3)
			{
			case 0:
				if (dn == 15 && ((in_it && !last_in_it) || m == 15))
					return false;
				drc_generate_begin(block, compiler, desc);
				drc_load_reg(block, I0, dn, desc);
				drc_load_reg(block, I2, m, desc);
				UML_ADD(block, I0, I0, I2);
				if (dn == 15)
					drc_generate_dynamic_branch(block, compiler, desc, 3, 0);
				else
				{
					drc_generate_set_reg(block, dn, I0);
					drc_generate_epilogue(block, compiler, desc, 1, false);
				}
				drc_generate_end(block, compiler, desc);
				return true;
			case 1:
				if ((dn < 8 && m < 8) || dn == 15 || m == 15)
					return false;
				drc_generate_begin(block, compiler, desc);
				UML_CMP(block, mem(&m_core->r[dn]), mem(&m_core->r[m]));
				drc_generate_flags(block, true);
				drc_generate_epilogue(block, compiler, desc, 1, false);
				drc_generate_end(block, compiler, desc);
				return true;
			case 2:
				if (dn == 15 && in_it && !last_in_it)
					return false;
				drc_generate_begin(block, compiler, desc);
				drc_load_reg(block, I0, m, desc);
				if (dn == 15)
					drc_generate_dynamic_branch(block, compiler, desc, 3, 0);
				else
				{
					drc_generate_set_reg(block, dn, I0);
					drc_generate_epilogue(block, compiler, desc, 1, false);
				}
				drc_generate_end(block, compiler, desc);
				return true;
			default:
				if ((op & 7) || (in_it && !last_in_it) || ((op & 0x80) && m == 15))
					return false;
				drc_generate_begin(block, compiler, desc);
				drc_load_reg(block, I0, m, desc);
				if (op & 0x80)
				{
					UML_MOV(block, mem(&m_core->r[14]), (pc + 2) | 1);
					drc_generate_dynamic_branch(block, compiler, desc, 3, 1);
				}
				else
					drc_generate_dynamic_branch(block, compiler, desc, 3, 2);
				drc_generate_end(block, compiler, desc);
				return true;
			}
		}

	case 0x5: case 0x6: case 0x7: case 0x8: case 0x9:
		{
			const int t = op & 7;
			const int n = (op >> 3) & 7;
			int size = 4;
			bool load_op = op & 0x800;
			bool sign = false;
			drc_generate_begin(block, compiler, desc);
			UML_MOV(block, mem(&m_core->pc), pc);
			switch (op >> 12)
			{
			case 0x5:
				UML_ADD(block, I0, mem(&m_core->r[n]), mem(&m_core->r[(op >> 6) & 7]));
				switch ((op >> 9) & 7)
				{
				case 0: size = 4; load_op = false; break;
				case 1: size = 2; load_op = false; break;
				case 2: size = 1; load_op = false; break;
				case 3: size = 1; load_op = true; sign = true; break;
				case 4: size = 4; load_op = true; break;
				case 5: size = 2; load_op = true; break;
				case 6: size = 1; load_op = true; break;
				default: size = 2; load_op = true; sign = true; break;
				}
				break;
			case 0x6:
				UML_ADD(block, I0, mem(&m_core->r[n]), ((op >> 6) & 0x1f) << 2);
				break;
			case 0x7:
				size = 1;
				UML_ADD(block, I0, mem(&m_core->r[n]), (op >> 6) & 0x1f);
				break;
			case 0x8:
				size = 2;
				UML_ADD(block, I0, mem(&m_core->r[n]), ((op >> 6) & 0x1f) << 1);
				break;
			default:
				UML_ADD(block, I0, mem(&m_core->r[13]), (op & 0xff) << 2);
				break;
			}
			const int rt = (op >> 12) == 0x9 ? (op >> 8) & 7 : t;
			if (load_op)
			{
				drc_generate_read(block, compiler, size, 0);
				if (sign)
					UML_SEXT(block, I2, I2, size == 1 ? SIZE_BYTE : SIZE_WORD);
				UML_MOV(block, mem(&m_core->r[rt]), I2);
			}
			else
			{
				UML_MOV(block, I2, mem(&m_core->r[rt]));
				drc_generate_write(block, compiler, size, 0);
			}
			drc_generate_epilogue(block, compiler, desc, load_op ? 2 : 1, true);
			drc_generate_end(block, compiler, desc);
			return true;
		}

	case 0xa:
		drc_generate_begin(block, compiler, desc);
		if (op & 0x800)
			UML_ADD(block, mem(&m_core->r[(op >> 8) & 7]), mem(&m_core->r[13]), (op & 0xff) << 2);
		else
			UML_MOV(block, mem(&m_core->r[(op >> 8) & 7]), ((pc + 4) & ~3) + ((op & 0xff) << 2));
		drc_generate_epilogue(block, compiler, desc, 1, false);
		drc_generate_end(block, compiler, desc);
		return true;

	case 0xb:
		switch ((op >> 8) & 0xf)
		{
		case 0x0:
			drc_generate_begin(block, compiler, desc);
			if (op & 0x80)
				UML_SUB(block, mem(&m_core->r[13]), mem(&m_core->r[13]), (op & 0x7f) << 2);
			else
				UML_ADD(block, mem(&m_core->r[13]), mem(&m_core->r[13]), (op & 0x7f) << 2);
			drc_generate_epilogue(block, compiler, desc, 1, false);
			drc_generate_end(block, compiler, desc);
			return true;

		case 0x1: case 0x3: case 0x9: case 0xb:
			{
				if (in_it)
					return false;
				const u32 target = pc + 4 + (((op >> 2) & 0x3e) | ((op >> 3) & 0x40));
				const uml::code_label skip = compiler.next_label();
				UML_CMP(block, mem(&m_core->r[op & 7]), 0);
				UML_JMPc(block, (op & 0x800) ? COND_E : COND_NE, skip);
				drc_generate_branch(block, compiler, desc, target, 3);
				UML_LABEL(block, skip);
				drc_generate_epilogue(block, compiler, desc, 1, false);
				return true;
			}

		case 0x2:
			drc_generate_begin(block, compiler, desc);
			switch ((op >> 6) & 3)
			{
			case 0: UML_SEXT(block, mem(&m_core->r[op & 7]), mem(&m_core->r[(op >> 3) & 7]), SIZE_WORD); break;
			case 1: UML_SEXT(block, mem(&m_core->r[op & 7]), mem(&m_core->r[(op >> 3) & 7]), SIZE_BYTE); break;
			case 2: UML_AND(block, mem(&m_core->r[op & 7]), mem(&m_core->r[(op >> 3) & 7]), 0xffff); break;
			default: UML_AND(block, mem(&m_core->r[op & 7]), mem(&m_core->r[(op >> 3) & 7]), 0xff); break;
			}
			drc_generate_epilogue(block, compiler, desc, 1, false);
			drc_generate_end(block, compiler, desc);
			return true;

		case 0x4: case 0x5:
			{
				const u32 registers = (op & 0xff) | ((op & 0x100) << 6);
				if (!registers)
					return false;
				drc_generate_begin(block, compiler, desc);
				drc_generate_stm(block, compiler, desc, 13, registers, true, true);
				drc_generate_end(block, compiler, desc);
				return true;
			}

		case 0xa:
			{
				const unsigned kind = (op >> 6) & 3;
				if (kind == 2)
					return false;
				drc_generate_begin(block, compiler, desc);
				UML_MOV(block, I0, mem(&m_core->r[(op >> 3) & 7]));
				switch (kind)
				{
				case 0:
					UML_BSWAP(block, I0, I0);
					break;
				case 1:
					UML_ROLAND(block, I1, I0, 24, 0x00ff00ff);
					UML_ROLAND(block, I0, I0, 8, 0xff00ff00);
					UML_OR(block, I0, I0, I1);
					break;
				default:
					UML_ROLAND(block, I1, I0, 24, 0xff);
					UML_ROLAND(block, I0, I0, 8, 0xff00);
					UML_OR(block, I0, I0, I1);
					UML_SEXT(block, I0, I0, SIZE_WORD);
					break;
				}
				UML_MOV(block, mem(&m_core->r[op & 7]), I0);
				drc_generate_epilogue(block, compiler, desc, 1, false);
				drc_generate_end(block, compiler, desc);
				return true;
			}

		case 0xc: case 0xd:
			{
				const u32 registers = (op & 0xff) | ((op & 0x100) << 7);
				if (!registers || ((registers & 0x8000) && in_it && !last_in_it))
					return false;
				drc_generate_begin(block, compiler, desc);
				drc_generate_ldm(block, compiler, desc, 13, registers, true, false);
				drc_generate_end(block, compiler, desc);
				return true;
			}

		case 0xf:
			if (op & 0xf)
			{
				const unsigned firstcond = (op >> 4) & 0xf;
				if (firstcond == 15 || (firstcond == 14 && std::popcount(unsigned(op & 0xf)) != 1) || in_it)
					return false;
				drc_generate_epilogue(block, compiler, desc, 1, false);
				return true;
			}
			if (((op >> 4) & 0xf) >= 2 && ((op >> 4) & 0xf) <= 4)
				return false;
			drc_generate_begin(block, compiler, desc);
			drc_generate_epilogue(block, compiler, desc, 1, false);
			drc_generate_end(block, compiler, desc);
			return true;

		default:
			return false;
		}

	case 0xc:
		{
			const int n = (op >> 8) & 7;
			const u32 registers = op & 0xff;
			if (!registers)
				return false;
			if (!(op & 0x800) && BIT(registers, n) && (registers & ((1 << n) - 1)))
				return false;
			drc_generate_begin(block, compiler, desc);
			if (op & 0x800)
				drc_generate_ldm(block, compiler, desc, n, registers, !BIT(registers, n), false);
			else
				drc_generate_stm(block, compiler, desc, n, registers, true, false);
			drc_generate_end(block, compiler, desc);
			return true;
		}

	case 0xd:
		{
			const unsigned cond = (op >> 8) & 0xf;
			if (cond >= 14 || in_it)
				return false;
			const u32 target = pc + 4 + util::sext(u32(op & 0xff) << 1, 9);
			const uml::code_label skip = compiler.next_label();
			drc_generate_condition(block, cond, skip);
			drc_generate_branch(block, compiler, desc, target, 3);
			UML_LABEL(block, skip);
			drc_generate_epilogue(block, compiler, desc, 1, false);
			return true;
		}

	case 0xe:
		if (in_it && !last_in_it)
			return false;
		drc_generate_begin(block, compiler, desc);
		drc_generate_branch(block, compiler, desc, pc + 4 + util::sext(u32(op & 0x7ff) << 1, 12), 3);
		drc_generate_end(block, compiler, desc);
		return true;
	}
	return false;
}

bool armv7m_device::drc_generate_t32(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc)
{
	using namespace uml;
	const u32 op = desc->opcode;
	const bool in_it = desc->it & 0xf;
	const bool last_in_it = (desc->it & 0xf) == 0x8;
	const u32 pc = desc->pc;
	const int n = (op >> 16) & 0xf;

	switch ((op >> 27) & 3)
	{
	case 1:
		if (op & 0x04000000)
			return false;
		if (op & 0x02000000)
		{
			const unsigned opc = (op >> 21) & 0xf;
			const bool setflags = op & 0x00100000;
			const int d = (op >> 8) & 0xf;
			const int m = op & 0xf;
			const unsigned imm5 = ((op >> 10) & 0x1c) | ((op >> 6) & 3);
			int type = (op >> 4) & 3;
			unsigned amount = imm5;
			if (type == 1 || type == 2)
				amount = imm5 ? imm5 : 32;
			else if (type == 3 && !imm5)
				type = 4;
			if ((op & 0x8000) || (n == 13 && (opc == 8 || opc == 13) && (type != 0 || amount > 3)))
				return false;
			if (!drc_dp_valid(opc, d, n, m, setflags))
				return false;
			const bool shifted = type != 0 || amount != 0;
			const bool want_carry = setflags && opc <= 4 && shifted;
			drc_generate_begin(block, compiler, desc);
			UML_MOV(block, I0, mem(&m_core->r[m]));
			if (shifted)
				drc_generate_shift_imm(block, type, amount, want_carry);
			UML_MOV(block, I2, I0);
			drc_generate_dp(block, desc, opc, d, n, want_carry ? 2 : -1, setflags);
			drc_generate_epilogue(block, compiler, desc, 1, false);
			drc_generate_end(block, compiler, desc);
			return true;
		}
		if (op & 0x00400000)
		{
			const int t = (op >> 12) & 0xf;
			const int t2 = (op >> 8) & 0xf;
			const bool p = op & 0x01000000;
			const bool u = op & 0x00800000;
			const bool w = op & 0x00200000;
			const bool l = op & 0x00100000;
			if (p || w)
			{
				if (t == 13 || t == 15 || t2 == 13 || t2 == 15 || (w && (n == t || n == t2)))
					return false;
				if (l && (t == t2 || (n == 15 && (w || ((pc + 4) & 3)))))
					return false;
				if (!l && n == 15)
					return false;
				const u32 imm = (op & 0xff) << 2;
				drc_generate_begin(block, compiler, desc);
				UML_MOV(block, mem(&m_core->pc), pc);
				drc_load_reg(block, I1, n, desc);
				if (u)
					UML_ADD(block, I4, I1, imm);
				else
					UML_SUB(block, I4, I1, imm);
				UML_MOV(block, I0, p ? I4 : I1);
				if (l)
				{
					drc_generate_read(block, compiler, 4, MEM_ALIGNED);
					UML_MOV(block, mem(&m_core->temp[0]), I2);
					UML_ADD(block, I0, p ? I4 : I1, 4);
					drc_generate_read(block, compiler, 4, MEM_ALIGNED);
					if (w && n != 15)
						drc_generate_set_reg(block, n, I4);
					UML_MOV(block, mem(&m_core->r[t]), mem(&m_core->temp[0]));
					UML_MOV(block, mem(&m_core->r[t2]), I2);
				}
				else
				{
					UML_MOV(block, I2, mem(&m_core->r[t]));
					drc_generate_write(block, compiler, 4, MEM_ALIGNED);
					UML_ADD(block, I0, p ? I4 : I1, 4);
					UML_MOV(block, I2, mem(&m_core->r[t2]));
					drc_generate_write(block, compiler, 4, MEM_ALIGNED);
					if (w)
						drc_generate_set_reg(block, n, I4);
				}
				drc_generate_epilogue(block, compiler, desc, 3, true);
				drc_generate_end(block, compiler, desc);
				return true;
			}
			if (!u || !l)
				return false;
			const unsigned op3 = (op >> 4) & 0xf;
			if (op3 != 0 && op3 != 1)
				return false;
			const int m = op & 0xf;
			if ((op & 0xff00) != 0xf000 || n == 13 || m == 13 || m == 15 || (in_it && !last_in_it))
				return false;
			drc_generate_begin(block, compiler, desc);
			UML_MOV(block, mem(&m_core->pc), pc);
			drc_load_reg(block, I0, n, desc);
			if (op3)
			{
				UML_SHL(block, I2, mem(&m_core->r[m]), 1);
				UML_ADD(block, I0, I0, I2);
			}
			else
				UML_ADD(block, I0, I0, mem(&m_core->r[m]));
			drc_generate_read(block, compiler, op3 ? 2 : 1, 0);
			UML_SHL(block, I0, I2, 1);
			UML_ADD(block, I0, I0, pc + 4);
			drc_generate_dynamic_branch(block, compiler, desc, 5, 0);
			drc_generate_end(block, compiler, desc);
			return true;
		}
		{
			const bool wback = op & 0x00200000;
			const bool load_op = op & 0x00100000;
			const u32 registers = op & 0xffff;
			const unsigned mode = (op >> 23) & 3;
			if (mode != 1 && mode != 2)
				return false;
			if (n == 15 || std::popcount(registers) < 2 || (registers & 0x2000) || (wback && BIT(registers, n)))
				return false;
			if (load_op && ((registers & 0xc000) == 0xc000 || ((registers & 0x8000) && in_it && !last_in_it)))
				return false;
			if (!load_op && (registers & 0x8000))
				return false;
			drc_generate_begin(block, compiler, desc);
			if (load_op)
				drc_generate_ldm(block, compiler, desc, n, registers, wback, mode == 2);
			else
				drc_generate_stm(block, compiler, desc, n, registers, wback, mode == 2);
			drc_generate_end(block, compiler, desc);
			return true;
		}

	case 2:
		if (op & 0x8000)
		{
			const unsigned op1 = (op >> 12) & 7;
			const unsigned hwop = (op >> 20) & 0x7f;
			if ((op1 & 5) == 0)
			{
				if ((hwop & 0x38) != 0x38)
				{
					if (in_it)
						return false;
					const u32 imm = ((op >> 26) & 1) << 20 | ((op >> 11) & 1) << 19 | ((op >> 13) & 1) << 18 | ((op >> 16) & 0x3f) << 12 | (op & 0x7ff) << 1;
					const unsigned cond = (op >> 22) & 0xf;
					const uml::code_label skip = compiler.next_label();
					drc_generate_condition(block, cond, skip);
					drc_generate_branch(block, compiler, desc, pc + 4 + util::sext(imm, 21), 3);
					UML_LABEL(block, skip);
					drc_generate_epilogue(block, compiler, desc, 1, false);
					return true;
				}
				bool nop = false;
				if (hwop == 0x3a)
					nop = !((op >> 8) & 7) && (op & 0x000f2800) == 0x000f0000 && ((op & 0xff) < 2 || (op & 0xff) > 4);
				else if (hwop == 0x3b)
					nop = (op & 0x000f2f00) == 0x000f0f00 && ((op >> 4) & 0xf) >= 4 && ((op >> 4) & 0xf) <= 6;
				if (!nop)
					return false;
				drc_generate_begin(block, compiler, desc);
				drc_generate_epilogue(block, compiler, desc, 1, false);
				drc_generate_end(block, compiler, desc);
				return true;
			}
			if ((op1 & 5) == 4)
				return false;
			if (!(op1 & 4) && in_it && !last_in_it)
				return false;
			const u32 s = (op >> 26) & 1;
			const u32 i1 = ~((op >> 13) ^ s) & 1;
			const u32 i2 = ~((op >> 11) ^ s) & 1;
			const u32 imm = s << 24 | i1 << 23 | i2 << 22 | ((op >> 16) & 0x3ff) << 12 | (op & 0x7ff) << 1;
			drc_generate_begin(block, compiler, desc);
			if (op1 & 4)
				UML_MOV(block, mem(&m_core->r[14]), (pc + 4) | 1);
			drc_generate_branch(block, compiler, desc, pc + 4 + util::sext(imm, 25), 3);
			drc_generate_end(block, compiler, desc);
			return true;
		}
		if (op & 0x02000000)
		{
			const unsigned opc = (op >> 20) & 0x1f;
			const int d = (op >> 8) & 0xf;
			const u32 imm12 = ((op >> 15) & 0x800) | ((op >> 4) & 0x700) | (op & 0xff);
			const unsigned lsb = ((op >> 10) & 0x1c) | ((op >> 6) & 3);
			switch (opc)
			{
			case 0x00:
			case 0x0a:
				if (d == 15 || (d == 13 && n != 13))
					return false;
				drc_generate_begin(block, compiler, desc);
				if (n == 15)
					UML_MOV(block, I0, ((pc + 4) & ~3) + (opc ? -imm12 : imm12));
				else if (opc)
					UML_SUB(block, I0, mem(&m_core->r[n]), imm12);
				else
					UML_ADD(block, I0, mem(&m_core->r[n]), imm12);
				drc_generate_set_reg(block, d, I0);
				break;
			case 0x04:
			case 0x0c:
				{
					if (d == 13 || d == 15)
						return false;
					const u32 imm16 = ((op >> 4) & 0xf000) | imm12;
					drc_generate_begin(block, compiler, desc);
					if (opc == 0x04)
						UML_MOV(block, mem(&m_core->r[d]), imm16);
					else
						UML_ROLINS(block, mem(&m_core->r[d]), imm16, 16, 0xffff0000);
					break;
				}
			case 0x14:
			case 0x1c:
				{
					const unsigned widthm1 = op & 0x1f;
					if ((op & 0x04000020) || d == 13 || d == 15 || n == 13 || n == 15 || lsb + widthm1 > 31)
						return false;
					drc_generate_begin(block, compiler, desc);
					if (opc == 0x14)
					{
						UML_SHL(block, I0, mem(&m_core->r[n]), 31 - (lsb + widthm1));
						UML_SAR(block, I0, I0, 31 - widthm1);
					}
					else
						UML_ROLAND(block, I0, mem(&m_core->r[n]), (32 - lsb) & 31, 0xffffffffU >> (31 - widthm1));
					UML_MOV(block, mem(&m_core->r[d]), I0);
					break;
				}
			case 0x16:
				{
					const unsigned msb = op & 0x1f;
					if ((op & 0x04000020) || d == 13 || d == 15 || n == 13 || msb < lsb)
						return false;
					const u32 mask = (0xffffffffU >> (31 - msb + lsb)) << lsb;
					drc_generate_begin(block, compiler, desc);
					if (n == 15)
						UML_AND(block, mem(&m_core->r[d]), mem(&m_core->r[d]), ~mask);
					else
						UML_ROLINS(block, mem(&m_core->r[d]), mem(&m_core->r[n]), lsb, mask);
					break;
				}
			default:
				return false;
			}
			drc_generate_epilogue(block, compiler, desc, 1, false);
			drc_generate_end(block, compiler, desc);
			return true;
		}
		{
			const unsigned opc = (op >> 21) & 0xf;
			const bool setflags = op & 0x00100000;
			const int d = (op >> 8) & 0xf;
			const u32 imm12 = ((op >> 15) & 0x800) | ((op >> 4) & 0x700) | (op & 0xff);
			if (!(imm12 & 0xc00) && (imm12 & 0x300) && !(imm12 & 0xff))
				return false;
			if (!drc_dp_valid(opc, d, n, -1, setflags))
				return false;
			const u32 imm32 = thumb_expand_imm(imm12);
			const int carry = (imm12 & 0xc00) ? int(imm32 >> 31) : -1;
			drc_generate_begin(block, compiler, desc);
			UML_MOV(block, I2, imm32);
			drc_generate_dp(block, desc, opc, d, n, carry, setflags);
			drc_generate_epilogue(block, compiler, desc, 1, false);
			drc_generate_end(block, compiler, desc);
			return true;
		}

	case 3:
		{
			const u32 op2 = (op >> 20) & 0x7f;
			const int t = (op >> 12) & 0xf;
			if (op2 & 0x40)
				return false;
			if ((op2 & 0x71) == 0x00)
			{
				const unsigned op1 = (op >> 21) & 7;
				const int size = 1 << (op1 & 3);
				if ((op1 & 3) == 3 || n == 15 || t == 15 || (t == 13 && size != 4))
					return false;
				u8 flags = 0;
				bool wback = false;
				int m = -1;
				bool p = false, u = false;
				if (!(op1 & 4))
				{
					if (op & 0x800)
					{
						p = op & 0x400;
						u = op & 0x200;
						wback = op & 0x100;
						if (!p && !wback)
							return false;
						if (p && u && !wback)
							flags = MEM_UNPRIV;
						if (wback && n == t)
							return false;
					}
					else
					{
						if (op & 0x7c0)
							return false;
						m = op & 0xf;
						if (m == 13 || m == 15)
							return false;
					}
				}
				drc_generate_begin(block, compiler, desc);
				UML_MOV(block, mem(&m_core->pc), pc);
				UML_MOV(block, I1, mem(&m_core->r[n]));
				if (op1 & 4)
					UML_ADD(block, I0, I1, op & 0xfff);
				else if (m < 0)
				{
					if (u)
						UML_ADD(block, I4, I1, op & 0xff);
					else
						UML_SUB(block, I4, I1, op & 0xff);
					UML_MOV(block, I0, p ? I4 : I1);
				}
				else
				{
					UML_SHL(block, I0, mem(&m_core->r[m]), (op >> 4) & 3);
					UML_ADD(block, I0, I0, I1);
				}
				UML_MOV(block, I2, mem(&m_core->r[t]));
				drc_generate_write(block, compiler, size, flags);
				if (wback)
					drc_generate_set_reg(block, n, I4);
				drc_generate_epilogue(block, compiler, desc, 1, true);
				drc_generate_end(block, compiler, desc);
				return true;
			}
			if ((op2 & 0x61) == 0x01)
			{
				const bool sign = op & 0x01000000;
				const bool imm12_form = op & 0x00800000;
				const unsigned sz = (op >> 21) & 3;
				if (sz == 3 || (sz == 2 && sign))
					return false;
				const int size = 1 << sz;
				bool hint_unpredictable = false;
				bool wback = false;
				u8 flags = 0;
				int m = -1;
				bool p = false, u = false;
				if (n == 15)
				{
					if (t == 15 && size == 2 && !sign)
						hint_unpredictable = true;
				}
				else if (!imm12_form)
				{
					const unsigned lop2 = (op >> 6) & 0x3f;
					if (lop2 == 0)
					{
						m = op & 0xf;
						if (m == 13 || m == 15)
							return false;
					}
					else if ((lop2 & 0x24) == 0x24 || (lop2 & 0x3c) == 0x30 || (lop2 & 0x3c) == 0x38)
					{
						p = op & 0x400;
						u = op & 0x200;
						wback = op & 0x100;
						if ((lop2 & 0x3c) == 0x38)
							flags = MEM_UNPRIV;
						if (wback || flags)
							hint_unpredictable = true;
						if (wback && n == t)
							return false;
					}
					else
						return false;
				}
				if (t == 15 && size != 4)
				{
					if (hint_unpredictable)
						return false;
					drc_generate_begin(block, compiler, desc);
					drc_generate_epilogue(block, compiler, desc, 1, false);
					drc_generate_end(block, compiler, desc);
					return true;
				}
				if ((t == 13 && size != 4) || (t == 15 && in_it && !last_in_it))
					return false;
				drc_generate_begin(block, compiler, desc);
				UML_MOV(block, mem(&m_core->pc), pc);
				if (n == 15)
				{
					const u32 base = (pc + 4) & ~3;
					UML_MOV(block, I0, imm12_form ? base + (op & 0xfff) : base - (op & 0xfff));
				}
				else if (imm12_form)
					UML_ADD(block, I0, mem(&m_core->r[n]), op & 0xfff);
				else if (m >= 0)
				{
					UML_SHL(block, I0, mem(&m_core->r[m]), (op >> 4) & 3);
					UML_ADD(block, I0, I0, mem(&m_core->r[n]));
				}
				else
				{
					UML_MOV(block, I1, mem(&m_core->r[n]));
					if (u)
						UML_ADD(block, I4, I1, op & 0xff);
					else
						UML_SUB(block, I4, I1, op & 0xff);
					UML_MOV(block, I0, p ? I4 : I1);
				}
				drc_generate_read(block, compiler, size, flags);
				if (sign)
					UML_SEXT(block, I2, I2, size == 1 ? SIZE_BYTE : SIZE_WORD);
				if (wback)
					drc_generate_set_reg(block, n, I4);
				if (t == 15)
				{
					UML_MOV(block, I0, I2);
					drc_generate_dynamic_branch(block, compiler, desc, 4, 2);
				}
				else
				{
					drc_generate_set_reg(block, t, I2);
					drc_generate_epilogue(block, compiler, desc, 2, true);
				}
				drc_generate_end(block, compiler, desc);
				return true;
			}
			if ((op2 & 0x70) == 0x20)
			{
				const unsigned op1 = (op >> 20) & 0xf;
				const unsigned rop2 = (op >> 4) & 0xf;
				const int d = (op >> 8) & 0xf;
				const int m = op & 0xf;
				if ((op & 0xf000) != 0xf000)
					return false;
				if (op1 < 8 && rop2 == 0)
				{
					if (d == 13 || d == 15 || n == 13 || n == 15 || m == 13 || m == 15)
						return false;
					drc_generate_begin(block, compiler, desc);
					drc_generate_shift_reg(block, op1 >> 1, mem(&m_core->r[n]), mem(&m_core->r[m]), d, op1 & 1);
					drc_generate_epilogue(block, compiler, desc, 1, false);
					drc_generate_end(block, compiler, desc);
					return true;
				}
				if (op1 < 8 && (rop2 & 8))
				{
					if (n != 15 || op1 == 2 || op1 == 3 || op1 > 5)
						return false;
					if ((op & 0x40) || d == 13 || d == 15 || m == 13 || m == 15)
						return false;
					const unsigned rot = 8 * ((op >> 4) & 3);
					drc_generate_begin(block, compiler, desc);
					UML_MOV(block, I0, mem(&m_core->r[m]));
					if (rot)
						UML_ROR(block, I0, I0, rot);
					switch (op1)
					{
					case 0: UML_SEXT(block, I0, I0, SIZE_WORD); break;
					case 1: UML_AND(block, I0, I0, 0xffff); break;
					case 4: UML_SEXT(block, I0, I0, SIZE_BYTE); break;
					default: UML_AND(block, I0, I0, 0xff); break;
					}
					UML_MOV(block, mem(&m_core->r[d]), I0);
					drc_generate_epilogue(block, compiler, desc, 1, false);
					drc_generate_end(block, compiler, desc);
					return true;
				}
				if ((op1 & 0xc) == 0x8 && (rop2 & 0xc) == 0x8)
				{
					const unsigned kind = ((op1 & 3) << 2) | (rop2 & 3);
					if (kind != 4 && kind != 5 && kind != 7 && kind != 0xc)
						return false;
					if (n != m || d == 13 || d == 15 || m == 13 || m == 15)
						return false;
					drc_generate_begin(block, compiler, desc);
					UML_MOV(block, I0, mem(&m_core->r[m]));
					switch (kind)
					{
					case 4:
						UML_BSWAP(block, I0, I0);
						break;
					case 5:
						UML_ROLAND(block, I1, I0, 24, 0x00ff00ff);
						UML_ROLAND(block, I0, I0, 8, 0xff00ff00);
						UML_OR(block, I0, I0, I1);
						break;
					case 7:
						UML_ROLAND(block, I1, I0, 24, 0xff);
						UML_ROLAND(block, I0, I0, 8, 0xff00);
						UML_OR(block, I0, I0, I1);
						UML_SEXT(block, I0, I0, SIZE_WORD);
						break;
					default:
						UML_LZCNT(block, I0, I0);
						break;
					}
					UML_MOV(block, mem(&m_core->r[d]), I0);
					drc_generate_epilogue(block, compiler, desc, 1, false);
					drc_generate_end(block, compiler, desc);
					return true;
				}
				return false;
			}
			if ((op2 & 0x78) == 0x30)
			{
				const unsigned op1 = (op >> 20) & 7;
				const unsigned mop2 = (op >> 4) & 3;
				const int a = (op >> 12) & 0xf;
				const int d = (op >> 8) & 0xf;
				const int m = op & 0xf;
				if ((op & 0xc0) || op1 != 0 || mop2 > 1)
					return false;
				if (d == 13 || d == 15 || n == 13 || n == 15 || m == 13 || m == 15 || a == 13 || (mop2 == 1 && a == 15))
					return false;
				drc_generate_begin(block, compiler, desc);
				UML_MULULW(block, I0, mem(&m_core->r[n]), mem(&m_core->r[m]));
				if (mop2 == 1)
					UML_SUB(block, I0, mem(&m_core->r[a]), I0);
				else if (a != 15)
					UML_ADD(block, I0, I0, mem(&m_core->r[a]));
				UML_MOV(block, mem(&m_core->r[d]), I0);
				drc_generate_epilogue(block, compiler, desc, (mop2 == 1 || a != 15) ? 2 : 1, false);
				drc_generate_end(block, compiler, desc);
				return true;
			}
			if ((op2 & 0x78) == 0x38)
			{
				const unsigned op1 = (op >> 20) & 7;
				const unsigned lop2 = (op >> 4) & 0xf;
				const int dlo = (op >> 12) & 0xf;
				const int dhi = (op >> 8) & 0xf;
				const int m = op & 0xf;
				if (lop2 != 0 || (op1 & 1))
					return false;
				if (dlo == 13 || dlo == 15 || dhi == 13 || dhi == 15 || n == 13 || n == 15 || m == 13 || m == 15 || dlo == dhi)
					return false;
				drc_generate_begin(block, compiler, desc);
				if (op1 & 2)
					UML_MULU(block, I0, I1, mem(&m_core->r[n]), mem(&m_core->r[m]));
				else
					UML_MULS(block, I0, I1, mem(&m_core->r[n]), mem(&m_core->r[m]));
				if (op1 & 4)
				{
					UML_ADD(block, I0, I0, mem(&m_core->r[dlo]));
					UML_ADDC(block, I1, I1, mem(&m_core->r[dhi]));
				}
				UML_MOV(block, mem(&m_core->r[dlo]), I0);
				UML_MOV(block, mem(&m_core->r[dhi]), I1);
				drc_generate_epilogue(block, compiler, desc, (op1 & 4) ? 4 : 3, false);
				drc_generate_end(block, compiler, desc);
				return true;
			}
			return false;
		}
	}
	return false;
}

//-------------------------------------------------
//  lockstep verification
//-------------------------------------------------

void armv7m_device::drc_verify_begin()
{
	verify_state &v = *m_verify;
	auto &s = v.before;
	s.core = *m_core;
	s.sp_other = m_sp_other;
	s.control = m_control;
	s.primask = m_primask;
	s.basepri = m_basepri;
	s.faultmask = m_faultmask;
	s.cfsr = m_cfsr;
	s.dfsr = m_dfsr;
	s.fault_addr = m_fault_addr;
	s.excl = m_excl;
	s.event = m_event;
	s.lockup = m_lockup;
	s.sleep = m_sleep;
	v.stamp_before = drc_stamp();
	v.log.clear();
	m_drc_hook = HOOK_RECORD;
}

void armv7m_device::drc_verify_end(int result)
{
	verify_state &v = *m_verify;
	m_drc_hook = HOOK_NONE;
	const u32 count = m_core->vcount;
	if (!count)
		return;

	const internal_state after = *m_core;
	const u32 after_sp_other = m_sp_other, after_control = m_control;
	const u32 after_primask = m_primask, after_basepri = m_basepri, after_faultmask = m_faultmask;
	const bool after_excl = m_excl, after_event = m_event, after_lockup = m_lockup;
	const int after_sleep = m_sleep;
	const int after_cycles = m_cycles, after_fault = m_fault_exc, after_post = m_post_exc;
	const bool after_excret_pending = m_exc_return_pending;
	const u32 after_excret = m_exc_return;
	const u64 consumed = drc_stamp() - v.stamp_before;

	const auto &s = v.before;
	*m_core = s.core;
	m_core->icount = after.icount;
	m_sp_other = s.sp_other;
	m_control = s.control;
	m_primask = s.primask;
	m_basepri = s.basepri;
	m_faultmask = s.faultmask;
	m_excl = s.excl;
	m_event = s.event;
	m_lockup = s.lockup;
	m_sleep = s.sleep;
	m_prio_dirty = true;

	m_drc_hook = HOOK_REPLAY;
	v.replay_pos = 0;
	u64 cycles = 0;
	std::string problem;
	for (u32 i = 0; i < count && problem.empty(); i++)
	{
		const u32 pc = m_core->pc;
		step_body();
		const bool pending = m_fault_exc || m_post_exc || m_exc_return_pending;
		if (i + 1 == count && result == EXIT_RESOLVE)
		{
			if (!pending)
				problem = util::string_format("recompiler resolved an exception at %08x, interpreter did not", pc);
			else if (after.exit_pc != pc)
				problem = util::string_format("exception instruction %08x, interpreter %08x", after.exit_pc, pc);
			else if (after_cycles != m_cycles || after_fault != m_fault_exc || after_post != m_post_exc || after_excret_pending != m_exc_return_pending || (m_exc_return_pending && after_excret != m_exc_return))
				problem = util::string_format("exception state cycles %d/%d fault %d/%d post %d/%d excret %d:%08x/%d:%08x",
						after_cycles, m_cycles, after_fault, m_fault_exc, after_post, m_post_exc, after_excret_pending, after_excret, m_exc_return_pending, m_exc_return);
		}
		else if (pending)
			problem = util::string_format("interpreter raised an exception at %08x (fault %d post %d excret %d)", pc, m_fault_exc, m_post_exc, m_exc_return_pending);
		else
			cycles += m_cycles;
	}
	m_drc_hook = HOOK_NONE;

	if (problem.empty() && v.replay_pos != v.log.size())
		problem = util::string_format("interpreter made %d of %d memory accesses", int(v.replay_pos), int(v.log.size()));
	if (problem.empty() && cycles != consumed)
		problem = util::string_format("cycles %d/%d", int(consumed), int(cycles));
	if (problem.empty())
	{
		static const char *const names[16] = { "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "sp", "lr", "pc" };
		for (int i = 0; i < 16 && problem.empty(); i++)
			if (after.r[i] != m_core->r[i])
				problem = util::string_format("%s %08x/%08x", names[i], after.r[i], m_core->r[i]);
		if (problem.empty() && (after.pc != m_core->pc || after.it != m_core->it || after.tbit != m_core->tbit))
			problem = util::string_format("pc %08x/%08x it %02x/%02x t %d/%d", after.pc, m_core->pc, after.it, m_core->it, after.tbit, m_core->tbit);
		if (problem.empty() && (after.n != m_core->n || after.z != m_core->z || after.c != m_core->c || after.v != m_core->v || after.q != m_core->q))
			problem = util::string_format("nzcvq %d%d%d%d%d/%d%d%d%d%d", after.n, after.z, after.c, after.v, after.q, m_core->n, m_core->z, m_core->c, m_core->v, m_core->q);
		if (problem.empty() && (after_sp_other != m_sp_other || after_control != m_control || after_primask != m_primask || after_basepri != m_basepri || after_faultmask != m_faultmask))
			problem = "special registers";
		if (problem.empty() && (after_excl != m_excl || after_event != m_event || after_sleep != m_sleep || after_lockup != m_lockup))
			problem = util::string_format("excl %d/%d event %d/%d sleep %d/%d", after_excl, m_excl, after_event, m_event, after_sleep, m_sleep);
	}

	v.blocks++;
	v.instructions += count;
	if (!problem.empty())
	{
		v.divergences++;
		if (v.divergences <= 10)
		{
			osd_printf_error("%s: recompiler divergence after %u instructions from %08x (it %02x): %s\n%s", tag(), count, s.core.pc, s.core.it, problem, drc_disassemble(s.core.pc, count + 2));
			for (size_t i = 0; i < v.log.size(); i++)
				osd_printf_error("  %s %08x/%d = %08x%s\n", v.log[i].write ? "W" : "R", v.log[i].address, v.log[i].size, v.log[i].value, v.log[i].ok ? "" : " fault");
		}
	}

	*m_core = after;
	m_sp_other = after_sp_other;
	m_control = after_control;
	m_primask = after_primask;
	m_basepri = after_basepri;
	m_faultmask = after_faultmask;
	m_excl = after_excl;
	m_event = after_event;
	m_lockup = after_lockup;
	m_sleep = after_sleep;
	m_cycles = after_cycles;
	m_fault_exc = after_fault;
	m_post_exc = after_post;
	m_exc_return_pending = after_excret_pending;
	m_exc_return = after_excret;
	m_prio_dirty = true;
}

void armv7m_device::drc_verify_log(bool write, u32 address, int size, u8 flags, u32 value)
{
	if (m_drc_hook != HOOK_RECORD)
		return;
	m_verify->log.push_back(verify_state::access{ write, address, size, flags, value, true, 0 });
}

bool armv7m_device::drc_memory_hook(bool write, u32 address, int size, u32 &value, u8 flags)
{
	if (size < 4)
		value &= (1U << (8 * size)) - 1;

	if (m_drc_hook != HOOK_REPLAY)
	{
		const int mode = m_drc_hook;
		m_drc_hook = HOOK_NONE;
		const bool ok = write ? mem_write(address, size, value, flags) : mem_read(address, size, value, flags);
		m_drc_hook = mode;
		if (mode == HOOK_RECORD)
			m_verify->log.push_back(verify_state::access{ write, address, size, flags, value, ok, m_fault_exc });
		if (write && address < m_core->code_hi && address + size > m_core->code_lo)
			m_core->smc = 1;
		return ok;
	}

	verify_state &v = *m_verify;
	if (v.replay_pos >= v.log.size())
	{
		v.replay_pos = v.log.size() + 1;
		return false;
	}
	const auto &a = v.log[v.replay_pos];
	if (a.write != write || a.address != address || a.size != size || a.flags != flags || (write && a.value != value))
	{
		v.replay_pos = v.log.size() + 1;
		return false;
	}
	v.replay_pos++;
	if (!write)
		value = a.value;
	if (!a.ok)
		m_fault_exc = a.fault;
	return a.ok;
}

std::string armv7m_device::drc_disassemble(u32 pc, int count)
{
	class buffer : public util::disasm_interface::data_buffer
	{
	public:
		buffer(address_space &space) : m_space(space) { }
		virtual u8 r8(offs_t pc) const override { return m_space.read_byte(pc); }
		virtual u16 r16(offs_t pc) const override { return m_space.read_word(pc); }
		virtual u32 r32(offs_t pc) const override { return m_space.read_dword(pc); }
		virtual u64 r64(offs_t pc) const override { return m_space.read_qword(pc); }
	private:
		address_space &m_space;
	};

	address_space &space = this->space(AS_PROGRAM);
	auto dis = machine().disable_side_effects();
	buffer opcodes(space);
	armv7m_disassembler dasm;
	std::string result;
	for (int i = 0; i < count; i++)
	{
		std::ostringstream stream;
		const offs_t length = dasm.disassemble(stream, pc, opcodes, opcodes) & util::disasm_interface::LENGTHMASK;
		result += util::string_format("  %08x: %s\n", pc, stream.str());
		pc += length ? length : 2;
	}
	return result;
}
