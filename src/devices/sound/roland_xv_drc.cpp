// license:BSD-3-Clause
// copyright-holders:superctr
// Roland XV effect DSP, recompiled through UML

#include "emu.h"
#include "roland_xv.h"

#include "emuopts.h"

#include "cpu/drcuml.h"
#include "cpu/drcumlsh.h"

#include <cstdlib>


namespace {

using namespace uml;

constexpr int PAGE = 64;
constexpr size_t CACHE_SIZE = 4 * 1024 * 1024;
constexpr u32 MAX_INSTRUCTIONS = 16384;

enum { EXECUTE_DONE = 0, EXECUTE_MISSING_CODE = 1 };

// the ALU result and the row's temporaries; the accumulators, product and latches for the sample
const parameter REG_A = I0;
const parameter REG_B = I1;
const parameter REG_P = I2;
const parameter REG_T = I3;
const parameter REG_X = I4;
const parameter REG_R = I5;
const parameter REG_S = I6;
const parameter REG_M = I7;
const parameter REG_C = I8;
const parameter REG_TAKEN = I9;

const parameter latch_reg[4] = { REG_R, REG_S, REG_M, REG_C };

// a block that may be absent: the first pass over a page only looks
struct sink
{
	drcuml_block *block;
	instruction &append() { return block->append(); }
};

} // anonymous namespace


class roland_xv_dsp_recompiler : public roland_xv_device::dsp_recompiler
{
public:
	roland_xv_dsp_recompiler(roland_xv_device &device);

	virtual void *alloc_near(size_t bytes, size_t align) override { return m_cache.alloc_near(bytes, std::align_val_t(align)); }
	virtual void reset() override;
	virtual void touched() override { m_dirty = true; }
	virtual void run() override;

private:
	using dsp_row = roland_xv_device::dsp_row;
	using dsp_state = roland_xv_device::dsp_state;

	static constexpr int ROWS = roland_xv_device::DSP_ROWS;
	static constexpr int PAGES = ROWS / PAGE;

	// a value in the flag chain as the compiler tracks it: some row's result, or one already in memory
	struct chain_value
	{
		enum kind_t : u8 { STATIC, IN_FLAG, IN_LAST };
		kind_t kind;
		u16 row;
	};

	// the chain between two hand-overs: unknown at a hash entry, then known row by row
	struct chain
	{
		chain_value flag = { chain_value::IN_FLAG, 0 };
		chain_value last = { chain_value::IN_LAST, 0 };
		int last_hold = -1;
	};

	enum commit_kind { COMMIT_ALWAYS, COMMIT_NEVER, COMMIT_IF };

	void flush();
	void refresh();
	void compile(int page);
	void walk(int page, drcuml_block *block);
	void emit_row(drcuml_block *block, int n, chain &c, commit_kind commit, int code);
	void emit_condition(drcuml_block *block, const chain &c, int code, int &constant, condition_t &cond);
	void emit_cell_read(drcuml_block &block, parameter dst, u16 address);
	void emit_cell_write(drcuml_block &block, u16 address, parameter src);
	void emit_clamp(drcuml_block &block, parameter reg);
	void emit_chain_step(drcuml_block *block, int n, chain &c);
	void emit_materialize(drcuml_block *block, const chain &c);
	void emit_charge(drcuml_block &block, int rows);
	parameter flag_value_of(const chain_value &v);
	parameter flag_raw_of(const chain_value &v);
	void demand(const chain_value &v) { if (v.kind == chain_value::STATIC) m_demanded[v.row] = true; }
	bool is_entry(int n) const { return m_entry[n]; }
	u64 row_key(int n) const;
	u64 signature(int page) const;
	void save_registers(drcuml_block &block);
	void load_registers(drcuml_block &block);

	void verify();
	void dump_program();

	roland_xv_device &m_device;
	drc_cache m_cache;
	std::unique_ptr<drcuml_state> m_drcuml;
	code_handle *m_entry_handle;
	code_handle *m_nocode;
	code_handle *m_exit;
	bool m_dirty;
	bool m_verify;
	int m_label;
	std::vector<bool> m_entry;
	std::vector<bool> m_demanded;
	std::vector<u64> m_signature;
	std::vector<bool> m_compiled;
	s32 *m_flag_value;
	s64 *m_flag_raw;
};


std::unique_ptr<roland_xv_device::dsp_recompiler> roland_xv_device::dsp_recompiler::create(roland_xv_device &device)
{
	if (!device.machine().options().drc())
		return nullptr;
	return std::make_unique<roland_xv_dsp_recompiler>(device);
}

roland_xv_dsp_recompiler::roland_xv_dsp_recompiler(roland_xv_device &device)
	: m_device(device)
	, m_cache(CACHE_SIZE)
	, m_entry_handle(nullptr)
	, m_nocode(nullptr)
	, m_exit(nullptr)
	, m_dirty(true)
	, m_verify(getenv("XV_DSP_VERIFY") != nullptr)
	, m_label(0)
	, m_entry(ROWS, false)
	, m_demanded(ROWS, false)
	, m_signature(PAGES, 0)
	, m_compiled(PAGES, false)
{
	m_cache.allocate_cache(device.mconfig().options().drc_rwx());
	m_flag_value = static_cast<s32 *>(m_cache.alloc_near(sizeof(s32) * ROWS, std::align_val_t(alignof(s32))));
	m_flag_raw = static_cast<s64 *>(m_cache.alloc_near(sizeof(s64) * ROWS, std::align_val_t(alignof(s64))));
	m_drcuml = std::make_unique<drcuml_state>(device, m_cache, 0, 1, 10, 0, 0);
}

void roland_xv_dsp_recompiler::reset()
{
	m_dirty = true;
	flush();
}

//-------------------------------------------------
//  the static code: the entry loads the registers and jumps to the row
//  the sample is at; a miss stores them and asks for the page; the exit
//  stores them and ends the sample
//-------------------------------------------------

void roland_xv_dsp_recompiler::flush()
{
	m_drcuml->reset();
	std::fill(m_compiled.begin(), m_compiled.end(), false);

	dsp_state &s = *m_device.m_dsp;
	{
		if (!m_entry_handle)
		{
			m_entry_handle = m_drcuml->handle_alloc("entry");
			m_nocode = m_drcuml->handle_alloc("nocode");
			m_exit = m_drcuml->handle_alloc("exit");
		}
		drcuml_block &block = m_drcuml->begin_block(32);
		UML_HANDLE(block, *m_entry_handle);
		load_registers(block);
		UML_HASHJMP(block, 0, mem(&s.pc), *m_nocode);
		block.end();
	}
	{
		drcuml_block &block = m_drcuml->begin_block(32);
		UML_HANDLE(block, *m_nocode);
		UML_GETEXP(block, REG_X);
		UML_MOV(block, mem(&s.pc), REG_X);
		save_registers(block);
		UML_EXIT(block, EXECUTE_MISSING_CODE);
		block.end();
	}
	{
		drcuml_block &block = m_drcuml->begin_block(32);
		UML_HANDLE(block, *m_exit);
		save_registers(block);
		UML_EXIT(block, EXECUTE_DONE);
		block.end();
	}
}

void roland_xv_dsp_recompiler::load_registers(drcuml_block &block)
{
	dsp_state &s = *m_device.m_dsp;
	UML_DLOADS(block, REG_A, &s.acc[0], 0, SIZE_DWORD, SCALE_x1);
	UML_DLOADS(block, REG_B, &s.acc[1], 0, SIZE_DWORD, SCALE_x1);
	UML_DLOADS(block, REG_P, &s.product, 0, SIZE_DWORD, SCALE_x1);
	for (int n = 0; n < 4; n++)
		UML_DLOADS(block, latch_reg[n], &s.latch[n], 0, SIZE_DWORD, SCALE_x1);
}

void roland_xv_dsp_recompiler::save_registers(drcuml_block &block)
{
	dsp_state &s = *m_device.m_dsp;
	UML_DSTORE(block, &s.acc[0], 0, REG_A, SIZE_DWORD, SCALE_x1);
	UML_DSTORE(block, &s.acc[1], 0, REG_B, SIZE_DWORD, SCALE_x1);
	UML_DSTORE(block, &s.product, 0, REG_P, SIZE_DWORD, SCALE_x1);
	for (int n = 0; n < 4; n++)
		UML_DSTORE(block, &s.latch[n], 0, latch_reg[n], SIZE_DWORD, SCALE_x1);
}

//-------------------------------------------------
//  the sample
//-------------------------------------------------

void roland_xv_dsp_recompiler::run()
{
	if (m_dirty)
		refresh();
	if (m_verify)
	{
		verify();
		return;
	}

	dsp_state &s = *m_device.m_dsp;
	s.pc = 0;
	s.steps = 0;
	for (;;)
	{
		const int result = m_drcuml->execute(*m_entry_handle);
		if (result == EXECUTE_MISSING_CODE)
			compile(s.pc / PAGE);
		else
			break;
	}
}

// the interpreter on a copy of the state, then the code on the state, and every difference is fatal
void roland_xv_dsp_recompiler::verify()
{
	roland_xv_device &d = m_device;
	dsp_state &s = *d.m_dsp;
	const dsp_state before = s;
	std::vector<u32> ring_before(d.m_ring, d.m_ring + roland_xv_device::RING_CELLS);
	std::vector<s32> bus_before(d.m_bus, d.m_bus + (roland_xv_device::IBUS_BANK - roland_xv_device::IBUS_MIX));

	d.interpret();
	const dsp_state expected = s;
	std::vector<u32> ring_expected(d.m_ring, d.m_ring + roland_xv_device::RING_CELLS);
	std::vector<s32> bus_expected(d.m_bus, d.m_bus + (roland_xv_device::IBUS_BANK - roland_xv_device::IBUS_MIX));
	std::vector<u16> tap_cell_expected(d.m_tap_cell, d.m_tap_cell + s.tap_count);
	std::vector<s32> tap_delay_expected(d.m_tap_delay, d.m_tap_delay + s.tap_count);

	s = before;
	std::copy(ring_before.begin(), ring_before.end(), d.m_ring);
	std::copy(bus_before.begin(), bus_before.end(), d.m_bus);
	s.pc = 0;
	s.steps = 0;
	for (;;)
	{
		const int result = m_drcuml->execute(*m_entry_handle);
		if (result == EXECUTE_MISSING_CODE)
			compile(s.pc / PAGE);
		else
			break;
	}

	auto fail = [&] (const char *what, int index = -1, s64 got = 0, s64 want = 0)
	{
		std::string rows;
		if (index >= 0 && what[0] == 'I')
		{
			const u16 address = (index - before.cursor) & (roland_xv_device::RING_CELLS - 1);
			for (int n = 0; n <= d.m_rows_end; n++)
			{
				const dsp_row &r = d.m_rows[n];
				if ((r.mode && r.address == address) || (r.second && r.mode2 && r.address2 == address))
					rows += util::string_format(" row %03x %04x %04x %04x", n, r.w0, r.w1, r.operand);
			}
			rows = util::string_format(" address %03x:%s", address, rows);
		}
		dump_program();
		fatalerror("%s: DSP recompiler disagrees with the interpreter on %s %d: %x, not %x (rows_end %d)%s\n", d.tag(), what, index, got, want, d.m_rows_end, rows);
	};
	for (int n = 0; n < roland_xv_device::RING_CELLS; n++)
		if (d.m_ring[n] != ring_expected[n]) fail("IRAM cell", n, d.m_ring[n], ring_expected[n]);
	for (int n = 0; n < roland_xv_device::IBUS_BANK - roland_xv_device::IBUS_MIX; n++)
		if (d.m_bus[n] != bus_expected[n]) fail("fixed cell", n + roland_xv_device::IBUS_MIX, d.m_bus[n], bus_expected[n]);
	if (s.acc[0] != expected.acc[0]) fail("A", 0, s.acc[0], expected.acc[0]);
	if (s.acc[1] != expected.acc[1]) fail("B", 0, s.acc[1], expected.acc[1]);
	if (s.product != expected.product) fail("P", 0, s.product, expected.product);
	for (int n = 0; n < 4; n++)
		if (s.latch[n] != expected.latch[n]) fail("latch", n, s.latch[n], expected.latch[n]);
	if (s.flag_value != expected.flag_value) fail("flag value", 0, s.flag_value, expected.flag_value);
	if (s.flag_raw != expected.flag_raw) fail("flag raw", 0, s.flag_raw, expected.flag_raw);
	if (s.last_hold != expected.last_hold) fail("hold", 0, s.last_hold, expected.last_hold);
	if (!s.last_hold && s.last_value != expected.last_value) fail("last value", 0, s.last_value, expected.last_value);
	if (!s.last_hold && s.last_raw != expected.last_raw) fail("last raw", 0, s.last_raw, expected.last_raw);
	if (s.tap_count != expected.tap_count) fail("tap count", 0, s.tap_count, expected.tap_count);
	for (u32 n = 0; n < s.tap_count; n++)
		if (d.m_tap_cell[n] != tap_cell_expected[n] || d.m_tap_delay[n] != tap_delay_expected[n]) fail("tap", n, d.m_tap_delay[n], tap_delay_expected[n]);
}

// the program as loaded, for a failure report: each row's words, and whether code can land on it
void roland_xv_dsp_recompiler::dump_program()
{
	FILE *f = fopen("xv_dsp_fail.txt", "w");
	if (!f)
		return;
	fprintf(f, "cursor %llx\n", (unsigned long long)m_device.m_dsp->cursor);
	for (int n = 0; n <= m_device.m_rows_end; n++)
		fprintf(f, "%03x %04x %04x %04x%s\n", n, m_device.m_rows[n].w0, m_device.m_rows[n].w1, m_device.m_rows[n].operand, m_entry[n] ? " entry" : "");
	fclose(f);
}

//-------------------------------------------------
//  what a page is compiled from.  The entries are the page starts, every
//  branch's target and the row after a branch that ends a page; the
//  signature covers each row's fields, whether it is live and an entry,
//  and the row after the page, which is the last row's delay slot.
//-------------------------------------------------

u64 roland_xv_dsp_recompiler::row_key(int n) const
{
	const dsp_row &r = m_device.m_rows[n];
	u64 key = 0;
	key |= u64(r.conditional) << 0;
	key |= u64(r.branch) << 1;
	key |= u64(r.condition) << 2;
	key |= u64(u8(r.displacement)) << 6;
	key |= u64(r.multiply) << 14;
	key |= u64(r.shift) << 15;
	key |= u64(r.mode) << 18;
	key |= u64(r.address) << 21;
	key |= u64(r.second) << 31;
	key |= u64(r.mode2) << 32;
	key |= u64(r.address2) << 35;
	key |= u64(r.cell_coefficient) << 45;
	key |= u64(r.source) << 46;
	key |= u64(r.left) << 49;
	key |= u64(r.right) << 52;
	key |= u64(r.negate_left) << 55;
	key |= u64(r.negate_right) << 56;
	key |= u64(r.to_b) << 57;
	key |= u64(r.wrap) << 58;
	key |= u64(r.hold) << 59;
	key |= u64(r.clamp) << 60;
	key |= u64(n <= m_device.m_rows_end) << 61;
	key |= u64(m_entry[n]) << 62;
	return key;
}

u64 roland_xv_dsp_recompiler::signature(int page) const
{
	u64 hash = 0xcbf29ce484222325;
	for (int n = page * PAGE; n <= page * PAGE + PAGE && n < ROWS; n++)
	{
		const u64 key = row_key(n);
		for (int byte = 0; byte < 8; byte++)
			hash = (hash ^ ((key >> (8 * byte)) & 0xff)) * 0x100000001b3;
	}
	return hash;
}

void roland_xv_dsp_recompiler::refresh()
{
	m_dirty = false;
	std::fill(m_entry.begin(), m_entry.end(), false);
	for (int n = 0; n < ROWS; n++)
	{
		if (n % PAGE == 0)
			m_entry[n] = true;
		const dsp_row &r = m_device.m_rows[n];
		if (!r.branch || n > m_device.m_rows_end)
			continue;
		m_entry[(n + 1 + r.displacement) & (ROWS - 1)] = true;
		if (n % PAGE == PAGE - 1 && n + 2 < ROWS)
			m_entry[n + 2] = true;
	}
	for (int n = 0; n + 2 < ROWS; n++)
		if (m_device.m_rows[n].branch && n <= m_device.m_rows_end && m_entry[n + 1])
			m_entry[n + 2] = true;
	for (int page = 0; page < PAGES; page++)
	{
		if (!m_compiled[page])
			continue;
		if (signature(page) == m_signature[page])
			continue;
		m_drcuml->hash_invalidate_range(page * PAGE, page * PAGE + PAGE - 1);
		m_compiled[page] = false;
	}
}

void roland_xv_dsp_recompiler::compile(int page)
{
	if (m_compiled[page])
	{
		dump_program();
		fatalerror("%s: DSP recompiler has no code for row %x of a compiled page\n", m_device.tag(), m_device.m_dsp->pc);
	}
	for (int attempt = 0; ; attempt++)
	{
		try
		{
			std::fill(m_demanded.begin(), m_demanded.end(), false);
			walk(page, nullptr);
			drcuml_block &block = m_drcuml->begin_block(MAX_INSTRUCTIONS);
			walk(page, &block);
			block.end();
			m_signature[page] = signature(page);
			m_compiled[page] = true;
			return;
		}
		catch (drcuml_block::abort_compilation &)
		{
			if (attempt)
				fatalerror("%s: DSP recompiler cannot compile page %d\n", m_device.tag(), page);
			flush();
		}
	}
}

//-------------------------------------------------
//  a page.  Without a block this only finds which rows' results are
//  wanted; with one it emits the code.  Rows run in order until a branch,
//  whose delay slot runs before the jump; the sample ends after the last
//  live row, at a branch to itself, at a branch from the last row, or
//  when the row budget is spent.
//-------------------------------------------------

void roland_xv_dsp_recompiler::walk(int page, drcuml_block *block)
{
	sink b{ block };
	const int start = page * PAGE;
	const int end = start + PAGE;
	const int rows_end = m_device.m_rows_end;
	chain c;
	int pending = 0;
	bool reachable = true;
	int n = start;
	m_label = 1;

	while (n < end)
	{
		if (n > rows_end)
		{
			if (reachable)
			{
				emit_materialize(block, c);
				if (block)
					UML_EXH(b, *m_exit, 0);
			}
			for (; n < end; n++)
				if (is_entry(n) && block)
				{
					UML_HASH(b, 0, n);
					UML_EXH(b, *m_exit, 0);
				}
			return;
		}

		if (is_entry(n))
		{
			if (reachable && n != start)
			{
				if (block)
					emit_charge(*block, pending);
				emit_materialize(block, c);
			}
			pending = 0;
			reachable = true;
			c = chain();
			if (block)
				UML_HASH(b, 0, n);
		}
		if (!reachable)
		{
			n++;
			continue;
		}

		const dsp_row &r = m_device.m_rows[n];
		if (!r.branch)
		{
			emit_row(block, n, c, r.conditional ? COMMIT_IF : COMMIT_ALWAYS, r.condition);
			pending++;
			n++;
			continue;
		}

		if (n + 1 >= ROWS)
		{
			emit_row(block, n, c, COMMIT_ALWAYS, 0);
			emit_materialize(block, c);
			if (block)
				UML_EXH(b, *m_exit, 0);
			return;
		}

		int constant = -1;
		condition_t cond = COND_ALWAYS;
		emit_condition(block, c, r.condition, constant, cond);
		if (block && constant < 0)
			UML_DSETc(b, cond, REG_TAKEN);
		emit_row(block, n, c, COMMIT_ALWAYS, 0);
		const dsp_row &slot = m_device.m_rows[n + 1];
		if (slot.branch)
			m_device.log_once(0, "branch in a delay slot");
		emit_row(block, n + 1, c, slot.branch ? COMMIT_ALWAYS : slot.conditional ? COMMIT_IF : COMMIT_ALWAYS, slot.condition);
		pending += 2;
		if (block)
			emit_charge(*block, pending);
		pending = 0;

		const int target = n + 1 + r.displacement;
		if (constant != 0)
		{
			const int fall = m_label++;
			if (block && constant < 0)
			{
				UML_DTEST(b, REG_TAKEN, REG_TAKEN);
				UML_JMPc(b, COND_Z, fall);
			}
			emit_materialize(block, c);
			if (block)
			{
				if (target == n)
					UML_EXH(b, *m_exit, 0);
				else
					UML_HASHJMP(b, 0, target & (ROWS - 1), *m_nocode);
				if (constant < 0)
					UML_LABEL(b, fall);
			}
			if (constant > 0)
				reachable = false;
		}

		// a delay slot some branch lands on runs as a row of its own from there, and both ways meet at the row after
		if (n + 1 < end && is_entry(n + 1))
		{
			const int meet = m_label++;
			if (reachable)
			{
				if (block)
					emit_charge(*block, pending);
				emit_materialize(block, c);
				if (block)
					UML_JMP(b, meet);
			}
			if (block)
				UML_HASH(b, 0, n + 1);
			c = chain();
			emit_row(block, n + 1, c, slot.branch ? COMMIT_ALWAYS : slot.conditional ? COMMIT_IF : COMMIT_ALWAYS, slot.condition);
			if (block)
				emit_charge(*block, 1);
			emit_materialize(block, c);
			if (block)
				UML_LABEL(b, meet);
			c = chain();
			pending = 0;
			reachable = true;
		}
		n += 2;
	}

	if (reachable && block)
	{
		emit_charge(*block, pending);
		emit_materialize(block, c);
		if (end < ROWS)
			UML_HASHJMP(b, 0, end, *m_nocode);
		else
			UML_EXH(b, *m_exit, 0);
	}
	else if (reachable)
		emit_materialize(block, c);
}

// the budget: the rows since the last charge, and the sample ends when it is spent
void roland_xv_dsp_recompiler::emit_charge(drcuml_block &block, int rows)
{
	if (!rows)
		return;
	dsp_state &s = *m_device.m_dsp;
	UML_ADD(block, mem(&s.steps), mem(&s.steps), rows);
	UML_CMP(block, mem(&s.steps), roland_xv_device::DSP_ROW_BUDGET);
	UML_EXHc(block, COND_GE, *m_exit, 0);
}

//-------------------------------------------------
//  the flag chain.  A condition reads the result of the last non-hold row
//  at least two rows back.  Along straight code the compiler knows which
//  row that is; the row stores its result where the condition reads it.
//  At a hash entry the chain is whatever memory holds, and code that
//  leaves for another entry writes memory first.
//-------------------------------------------------

parameter roland_xv_dsp_recompiler::flag_value_of(const chain_value &v)
{
	dsp_state &s = *m_device.m_dsp;
	switch (v.kind)
	{
	case chain_value::STATIC: return mem(&m_flag_value[v.row]);
	case chain_value::IN_LAST: return mem(&s.last_value);
	default: return mem(&s.flag_value);
	}
}

parameter roland_xv_dsp_recompiler::flag_raw_of(const chain_value &v)
{
	dsp_state &s = *m_device.m_dsp;
	switch (v.kind)
	{
	case chain_value::STATIC: return mem(&m_flag_raw[v.row]);
	case chain_value::IN_LAST: return mem(&s.last_raw);
	default: return mem(&s.flag_raw);
	}
}

// sets the condition flags for a code, or reports it constant
void roland_xv_dsp_recompiler::emit_condition(drcuml_block *block, const chain &c, int code, int &constant, condition_t &cond)
{
	sink b{ block };
	constant = -1;
	switch (code)
	{
	case 0x0: constant = 0; return;
	case 0x1: constant = 1; return;
	case 0x2: cond = COND_E; break;
	case 0x3: cond = COND_NE; break;
	case 0x6: cond = COND_AE; break;
	case 0x7: cond = COND_B; break;
	case 0x8: case 0xc: cond = COND_GE; break;
	case 0x9: case 0xd: cond = COND_L; break;
	case 0xa: cond = COND_G; break;
	case 0xb: cond = COND_LE; break;
	default:
		m_device.log_once(1, "unobserved condition code");
		constant = 0;
		return;
	}
	demand(c.flag);
	if (!block)
		return;
	if (code == 0x6 || code == 0x7)
	{
		UML_DADD(b, REG_X, flag_raw_of(c.flag), (1 << roland_xv_device::DSP_FRACTION_BITS) - 1);
		UML_DCMP(b, REG_X, (1 << (roland_xv_device::DSP_FRACTION_BITS + 1)) - 1);
	}
	else
		UML_CMP(b, flag_value_of(c.flag), 0);
}

// what execute() does to the chain at the end of a row: the previous row's result becomes
// the flag unless that row held, and this row's result is the new last one
void roland_xv_dsp_recompiler::emit_chain_step(drcuml_block *block, int n, chain &c)
{
	sink b{ block };
	dsp_state &s = *m_device.m_dsp;
	if (c.last_hold < 0)
	{
		if (block)
		{
			const int skip = m_label++;
			UML_TEST(b, mem(&s.last_hold), 1);
			UML_JMPc(b, COND_NZ, skip);
			UML_MOV(b, mem(&s.flag_value), mem(&s.last_value));
			UML_DMOV(b, mem(&s.flag_raw), mem(&s.last_raw));
			UML_LABEL(b, skip);
		}
		c.flag = { chain_value::IN_FLAG, 0 };
	}
	else if (!c.last_hold)
		c.flag = c.last;
	c.last = { chain_value::STATIC, u16(n) };
	c.last_hold = m_device.m_rows[n].hold;
}

void roland_xv_dsp_recompiler::emit_materialize(drcuml_block *block, const chain &c)
{
	sink b{ block };
	dsp_state &s = *m_device.m_dsp;
	demand(c.flag);
	if (block && c.flag.kind != chain_value::IN_FLAG)
	{
		UML_MOV(b, mem(&s.flag_value), flag_value_of(c.flag));
		UML_DMOV(b, mem(&s.flag_raw), flag_raw_of(c.flag));
	}
	if (c.last_hold == 0)
	{
		demand(c.last);
		if (block && c.last.kind == chain_value::STATIC)
		{
			UML_MOV(b, mem(&s.last_value), mem(&m_flag_value[c.last.row]));
			UML_DMOV(b, mem(&s.last_raw), mem(&m_flag_raw[c.last.row]));
		}
	}
	if (block && c.last_hold >= 0)
		UML_MOV(b, mem(&s.last_hold), c.last_hold);
}

//-------------------------------------------------
//  a row
//-------------------------------------------------

void roland_xv_dsp_recompiler::emit_clamp(drcuml_block &block, parameter reg)
{
	UML_DCMP(block, reg, 0x7fffff);
	UML_DMOVc(block, COND_G, reg, 0x7fffff);
	UML_DCMP(block, reg, -0x800000);
	UML_DMOVc(block, COND_L, reg, -0x800000);
}

// a cell into a register, 24 bits sign-extended; the index goes through REG_T
void roland_xv_dsp_recompiler::emit_cell_read(drcuml_block &block, parameter dst, u16 address)
{
	roland_xv_device &d = m_device;
	if (address < roland_xv_device::IBUS_MIX)
	{
		UML_DADD(block, REG_T, mem(&d.m_dsp->cursor), address);
		UML_DAND(block, REG_T, REG_T, roland_xv_device::RING_CELLS - 1);
		UML_DLOADS(block, dst, d.m_ring, REG_T, SIZE_DWORD, SCALE_x4);
		UML_DSHL(block, dst, dst, 40);
		UML_DSAR(block, dst, dst, 40);
	}
	else if (address < roland_xv_device::IBUS_BANK)
		UML_DLOADS(block, dst, &d.m_bus[address - roland_xv_device::IBUS_MIX], 0, SIZE_DWORD, SCALE_x1);
	else if (address < roland_xv_device::IBUS_BANK_END)
		UML_DLOADS(block, dst, &d.m_bank[address - roland_xv_device::IBUS_BANK], 0, SIZE_DWORD, SCALE_x1);
	else
		UML_DMOV(block, dst, 0);
}

void roland_xv_dsp_recompiler::emit_cell_write(drcuml_block &block, u16 address, parameter src)
{
	roland_xv_device &d = m_device;
	if (address < roland_xv_device::IBUS_MIX)
	{
		UML_DADD(block, REG_T, mem(&d.m_dsp->cursor), address);
		UML_DAND(block, REG_T, REG_T, roland_xv_device::RING_CELLS - 1);
		UML_DSTORE(block, d.m_ring, REG_T, src, SIZE_DWORD, SCALE_x4);
	}
	else if (address < roland_xv_device::IBUS_BANK)
		UML_DSTORE(block, &d.m_bus[address - roland_xv_device::IBUS_MIX], 0, src, SIZE_DWORD, SCALE_x1);
}

void roland_xv_dsp_recompiler::emit_row(drcuml_block *block, int n, chain &c, commit_kind commit, int code)
{
	using dev = roland_xv_device;
	roland_xv_device &d = m_device;
	dsp_state &s = *d.m_dsp;
	const dsp_row &r = d.m_rows[n];

	if (commit == COMMIT_IF)
	{
		int constant;
		condition_t cond;
		emit_condition(nullptr, c, code, constant, cond);
		if (constant == 0)
			commit = COMMIT_NEVER;
		else if (constant == 1)
			commit = COMMIT_ALWAYS;
	}

	// which latches this row's ALU and multiplier read, so a read into one of them lands after
	u8 consumed = 0;
	if (r.left >= dev::LEFT_R && r.left <= dev::LEFT_M)
		consumed |= 1 << (r.left - dev::LEFT_R);
	if (r.right == dev::RIGHT_R)
		consumed |= 1 << 0;
	if (r.source <= dev::SOURCE_C)
		consumed |= 1 << r.source;
	if (r.cell_coefficient || r.source == dev::SOURCE_S)
		consumed |= 1 << 3;
	const bool wanted = m_demanded[n];

	if (!block)
	{
		emit_chain_step(block, n, c);
		return;
	}
	drcuml_block &b = *block;

	u8 deferred = 0;
	for (int op = 0; op < (r.second ? 2 : 1); op++)
	{
		const int mode = op ? r.mode2 : r.mode;
		const u16 address = op ? r.address2 : r.address;
		switch (mode)
		{
		case dev::MEM_NONE:
			if (address)
			{
				const int skip = m_label++;
				UML_LOAD(b, REG_T, &s.tap_count, 0, SIZE_DWORD, SCALE_x1);
				UML_CMP(b, REG_T, dev::TAPS);
				UML_JMPc(b, COND_AE, skip);
				UML_STORE(b, d.m_tap_delay, REG_T, BIT(address, 8) ? REG_B : REG_A, SIZE_DWORD, SCALE_x4);
				UML_STORE(b, d.m_tap_cell, REG_T, address | 0x100, SIZE_WORD, SCALE_x2);
				UML_ADD(b, REG_T, REG_T, 1);
				UML_STORE(b, &s.tap_count, 0, REG_T, SIZE_DWORD, SCALE_x1);
				UML_LABEL(b, skip);
			}
			break;
		case dev::MEM_STORE_P:
		case dev::MEM_STORE_A:
		case dev::MEM_STORE_B:
			if (address < dev::IBUS_BANK)
			{
				UML_DMOV(b, REG_X, mode == dev::MEM_STORE_P ? REG_P : mode == dev::MEM_STORE_A ? REG_A : REG_B);
				emit_clamp(b, REG_X);
				emit_cell_write(b, address, REG_X);
			}
			break;
		default:
		{
			const int latch = mode - dev::MEM_READ_R;
			if (BIT(consumed, latch))
			{
				emit_cell_read(b, REG_X, address);
				UML_DMOV(b, mem(&s.scratch[latch]), REG_X);
				deferred |= 1 << latch;
			}
			else
				emit_cell_read(b, latch_reg[latch], address);
			break;
		}
		}
	}

	// the ALU into REG_T, before anything it reads changes
	const bool result = !r.hold || r.wrap || wanted;
	if (result)
	{
		if (r.hold)
			UML_DMOV(b, REG_T, r.to_b ? REG_B : REG_A);
		else
		{
			parameter left = REG_T, right = REG_X;
			bool have_left = true, have_right = true;
			switch (r.left)
			{
			case dev::LEFT_ZERO: have_left = false; break;
			case dev::LEFT_R: left = REG_R; break;
			case dev::LEFT_S: left = REG_S; break;
			case dev::LEFT_M: left = REG_M; break;
			case dev::LEFT_A: left = REG_A; break;
			case dev::LEFT_B: left = REG_B; break;
			case dev::LEFT_MAG_A:
			case dev::LEFT_MAG_B:
				UML_DSAR(b, REG_X, r.left == dev::LEFT_MAG_A ? REG_A : REG_B, 63);
				UML_DXOR(b, REG_T, r.left == dev::LEFT_MAG_A ? REG_A : REG_B, REG_X);
				UML_DSUB(b, REG_T, REG_T, REG_X);
				emit_clamp(b, REG_T);
				break;
			}
			switch (r.right)
			{
			case dev::RIGHT_ZERO: have_right = false; break;
			case dev::RIGHT_A: right = REG_A; break;
			case dev::RIGHT_B: right = REG_B; break;
			case dev::RIGHT_R: right = REG_R; break;
			case dev::RIGHT_P: right = REG_P; break;
			default:
				UML_DLOADS(b, REG_X, d.m_operands, 2 * n, SIZE_DWORD, SCALE_x4);
				break;
			}
			if (have_left && have_right)
			{
				if (r.negate_left && !r.negate_right)
					UML_DSUB(b, REG_T, right, left);
				else if (r.negate_right && !r.negate_left)
					UML_DSUB(b, REG_T, left, right);
				else
					UML_DADD(b, REG_T, left, right);
				if (r.negate_left && r.negate_right)
					UML_DSUB(b, REG_T, 0, REG_T);
			}
			else if (have_left)
			{
				if (r.negate_left)
					UML_DSUB(b, REG_T, 0, left);
				else if (!(left == REG_T))
					UML_DMOV(b, REG_T, left);
			}
			else if (have_right)
			{
				if (r.negate_right)
					UML_DSUB(b, REG_T, 0, right);
				else
					UML_DMOV(b, REG_T, right);
			}
			else
				UML_DMOV(b, REG_T, 0);
		}
		if (wanted)
			UML_DSTORE(b, &m_flag_raw[n], 0, REG_T, SIZE_QWORD, SCALE_x1);
		if (r.wrap)
		{
			UML_DSHL(b, REG_T, REG_T, 40);
			UML_DSAR(b, REG_T, REG_T, 40);
		}
		else if (r.clamp)
			emit_clamp(b, REG_T);
	}

	// the multiplier into REG_P, from the operands the row was entered with
	parameter source = REG_R;
	bool product = true;
	switch (r.source)
	{
	case dev::SOURCE_R: source = REG_R; break;
	case dev::SOURCE_S: source = REG_S; break;
	case dev::SOURCE_M: source = REG_M; break;
	case dev::SOURCE_C: source = REG_C; break;
	case dev::SOURCE_A: source = REG_A; break;
	case dev::SOURCE_B: source = REG_B; break;
	case dev::SOURCE_P: source = REG_P; break;
	default:
		d.log_once(2, "unobserved multiplier selector");
		source = 0;
		break;
	}
	parameter coefficient = REG_X;
	if (r.multiply)
		UML_DLOADS(b, REG_X, d.m_operands, 2 * n + 1, SIZE_DWORD, SCALE_x4);
	else if (r.cell_coefficient)
		coefficient = REG_C;
	else if (r.source == dev::SOURCE_M || r.source == dev::SOURCE_A || r.source == dev::SOURCE_B)
		coefficient = REG_A;
	else if (r.source == dev::SOURCE_S)
		UML_DSUB(b, REG_X, REG_C, 1 << dev::DSP_FRACTION_BITS);
	else
		product = false;
	if (product)
	{
		const int shift = dev::DSP_FRACTION_BITS - r.shift;
		UML_DMULSLW(b, REG_P, source, coefficient);
		UML_DSAR(b, REG_X, REG_P, 63);
		UML_DAND(b, REG_X, REG_X, (s64(1) << shift) - 1);
		UML_DADD(b, REG_P, REG_P, REG_X);
		UML_DSAR(b, REG_P, REG_P, shift);
		if (r.shift)
			emit_clamp(b, REG_P);
		else
			UML_DSEXT(b, REG_P, REG_P, SIZE_DWORD);
	}

	// the result into its accumulator
	if (result && commit != COMMIT_NEVER && (!r.hold || r.wrap))
	{
		const parameter acc = r.to_b ? REG_B : REG_A;
		if (commit == COMMIT_ALWAYS)
			UML_DMOV(b, acc, REG_T);
		else
		{
			int constant;
			condition_t cond;
			emit_condition(block, c, code, constant, cond);
			UML_DMOVc(b, cond, acc, REG_T);
		}
	}
	if (wanted)
		UML_DSTORE(b, &m_flag_value[n], 0, REG_T, SIZE_DWORD, SCALE_x1);

	for (int latch = 0; latch < 4; latch++)
		if (BIT(deferred, latch))
			UML_DMOV(b, latch_reg[latch], mem(&s.scratch[latch]));

	emit_chain_step(block, n, c);
}
