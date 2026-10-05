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
constexpr int MODES = 2;
constexpr size_t CACHE_SIZE = 16 * 1024 * 1024;
constexpr u32 MAX_INSTRUCTIONS = 32768;

enum { EXECUTE_DONE = 0, EXECUTE_MISSING_CODE = 1, EXECUTE_INTERPRET = 2 };

// the row's temporaries and the accumulators for the sample; the product and latches stay in the state block
const parameter REG_T = I0;
const parameter REG_X = I1;
const parameter REG_Y = I2;
const parameter REG_A = I3;
const parameter REG_B = I4;

constexpr u32 LABEL_ROW = 1;
constexpr u32 LABEL_LOCAL = 0x1000;

constexpr s64 MASK29 = 0x1fffffff;

// a block that may be absent: the first pass over a page only looks
struct sink
{
	drcuml_block *block;
	instruction *dummy;
	instruction &append() { return block ? block->append() : *dummy; }
};

} // anonymous namespace


class roland_xv_dsp_recompiler : public roland_xv_device::dsp_recompiler
{
public:
	roland_xv_dsp_recompiler(roland_xv_device &device);

	virtual void *alloc_near(size_t bytes, size_t align) override { return m_cache.alloc_near(bytes, std::align_val_t(align)); }
	virtual void reset() override;
	virtual void touched(int row) override;
	virtual void run() override;

private:
	using dev = roland_xv_device;
	using dsp_row = roland_xv_device::dsp_row;
	using dsp_fields = roland_xv_device::dsp_fields;
	using dsp_state = roland_xv_device::dsp_state;

	static constexpr int ROWS = dev::DSP_ROWS;
	static constexpr int PAGES = ROWS / PAGE;
	static constexpr int LAST_ROW = dev::DSP_ROWS_MAPPED - 1;

	// what the code reads and writes beside the device's state: the operand values, the results
	// a later condition wants, the latch reads a row defers, and the hand-over words
	struct near_state
	{
		u32 pc;
		u32 mode;
		u32 row;
		u32 commit;
		u32 taken;
		s32 scratch[4];
		s32 immediate[ROWS][2];
		s32 coefficient[ROWS];
		s32 raw[ROWS];
		u32 carry[ROWS];
	};

	// a value of the flag chain as the compiler knows it: some row's result, or what memory holds
	struct chain_value
	{
		enum kind_t : u8 { ROW, IN_FLAG, IN_LAST };
		kind_t kind;
		u16 row;
		bool operator==(const chain_value &v) const { return kind == v.kind && (kind != ROW || row == v.row); }
	};

	enum { HOLD_CLEAR = 0, HOLD_SET = 1, HOLD_DYNAMIC = 2 };
	enum { DYNAMIC = -1 };

	// what the compiler knows between two hand-overs; at a hash entry, only that memory holds it
	struct context
	{
		chain_value flag = { chain_value::IN_FLAG, 0 };
		chain_value last = { chain_value::IN_LAST, 0 };
		int last_hold = HOLD_DYNAMIC;
		int product_shift = DYNAMIC;
		bool narrow[2] = { false, false };
	};

	// a clamp's rare ends, emitted after the page's code
	struct clamp_stub
	{
		u32 high_label;
		u32 low_label;
		u32 back;
		parameter reg;
		s64 low;
		s64 high;
	};

	void flush();
	void refresh();
	void compile(int page, int mode);
	void walk(int page, int mode, drcuml_block *block);
	int emit_branch(sink &b, context &c, int n, int mode, int start, int end, bool &reachable);
	void emit_any_row(sink &b, context &c, int n, int mode);
	void emit_row(sink &b, context &c, int n, int mode, int commit, int predicate);
	void emit_access(sink &b, const context &c, int mode, u16 address, int bus_mode, u8 consumed, u8 &deferred);
	void emit_alternative(sink &b, const context &c, const dsp_row &r, int bus_mode, u8 consumed, u8 &deferred);
	void emit_cell_read(sink &b, u16 address, int bus_mode);
	void emit_cell_write(sink &b, u16 address, int bus_mode);
	void emit_clamp(sink &b, parameter reg, s64 low, s64 high);
	void emit_stubs(sink &b);
	condition_t emit_condition(sink &b, const context &c, int code);
	void emit_merge(sink &b, context &c);
	void emit_materialize(sink &b, const context &c);
	void emit_store_flag(sink &b, const context &c);
	void emit_charge(sink &b);
	void emit_goto(sink &b, int target, int mode, int start, int end);
	void emit_handback(sink &b, const context &c, int n);
	void open_segment(sink &b, const context &c, int n);
	void close_segment();
	void save_registers(sink &b);
	void load_registers(sink &b);
	void demand_raw(const chain_value &v) { if (v.kind == chain_value::ROW) m_demand_raw[v.row] = true; }
	void demand_carry(const chain_value &v) { if (v.kind == chain_value::ROW) m_demand_carry[v.row] = true; }
	u8 consumed_latches(const dsp_row &r, const dsp_fields &f) const;
	static bool same_fields(const dsp_fields &a, const dsp_fields &b);
	u64 code_key(int n) const;
	u64 signature(int page) const;

	static void run_row_callback(void *param);
	void run_row();
	void execute();
	void verify();
	void dump_program();

	roland_xv_device &m_device;
	drc_cache m_cache;
	std::unique_ptr<drcuml_state> m_drcuml;
	code_handle *m_entry_handle;
	code_handle *m_nocode;
	code_handle *m_exit;
	code_handle *m_handback;
	near_state *m_near;
	instruction m_dummy;
	bool m_dirty;
	bool m_verify;
	bool m_dry;
	u32 m_label;
	int m_steps_pending;
	u32 m_counted;
	int m_segment;
	std::vector<u32> m_segment_start;
	std::vector<u32> m_segment_length;
	std::vector<clamp_stub> m_stubs;
	std::vector<bool> m_entry;
	std::vector<u64> m_code;
	int m_rows_end;
	std::vector<bool> m_demand_raw;
	std::vector<bool> m_demand_carry;
	u64 m_signature[MODES][PAGES];
	bool m_compiled[MODES][PAGES];
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
	, m_handback(nullptr)
	, m_dirty(true)
	, m_verify(getenv("XV_DSP_VERIFY") != nullptr)
	, m_dry(false)
	, m_label(LABEL_LOCAL)
	, m_steps_pending(0)
	, m_counted(0)
	, m_segment(-1)
	, m_entry(ROWS, false)
	, m_code(ROWS, 0)
	, m_rows_end(-1)
	, m_demand_raw(ROWS, false)
	, m_demand_carry(ROWS, false)
{
	m_cache.allocate_cache(device.mconfig().options().drc_rwx());
	m_near = static_cast<near_state *>(m_cache.alloc_near(sizeof(near_state), std::align_val_t(alignof(near_state))));
	*m_near = near_state();
	m_drcuml = std::make_unique<drcuml_state>(device, m_cache, 0, MODES, 10, 0, 0);
	std::fill(&m_compiled[0][0], &m_compiled[0][0] + MODES * PAGES, false);
	std::fill(&m_signature[0][0], &m_signature[0][0] + MODES * PAGES, 0);
}

void roland_xv_dsp_recompiler::reset()
{
	m_dirty = true;
	flush();
}

//-------------------------------------------------
//  the static code: the entry loads the accumulators and jumps to the row
//  the sample is at in the work bank's mode; a miss stores them and asks
//  for the page; a hand-over stores them and has the interpreter go on
//  from a row; the exit stores them and ends the sample
//-------------------------------------------------

void roland_xv_dsp_recompiler::flush()
{
	m_drcuml->reset();
	std::fill(&m_compiled[0][0], &m_compiled[0][0] + MODES * PAGES, false);

	if (!m_entry_handle)
	{
		m_entry_handle = m_drcuml->handle_alloc("entry");
		m_nocode = m_drcuml->handle_alloc("nocode");
		m_exit = m_drcuml->handle_alloc("exit");
		m_handback = m_drcuml->handle_alloc("handback");
	}
	{
		drcuml_block &block = m_drcuml->begin_block(32);
		sink b{ &block, &m_dummy };
		UML_HANDLE(b, *m_entry_handle);
		load_registers(b);
		UML_HASHJMP(b, mem(&m_near->mode), mem(&m_near->pc), *m_nocode);
		block.end();
	}
	{
		drcuml_block &block = m_drcuml->begin_block(32);
		sink b{ &block, &m_dummy };
		UML_HANDLE(b, *m_nocode);
		UML_GETEXP(b, REG_X);
		UML_MOV(b, mem(&m_near->pc), REG_X);
		save_registers(b);
		UML_EXIT(b, EXECUTE_MISSING_CODE);
		block.end();
	}
	{
		drcuml_block &block = m_drcuml->begin_block(32);
		sink b{ &block, &m_dummy };
		UML_HANDLE(b, *m_handback);
		UML_GETEXP(b, REG_X);
		UML_MOV(b, mem(&m_near->pc), REG_X);
		save_registers(b);
		UML_EXIT(b, EXECUTE_INTERPRET);
		block.end();
	}
	{
		drcuml_block &block = m_drcuml->begin_block(32);
		sink b{ &block, &m_dummy };
		UML_HANDLE(b, *m_exit);
		save_registers(b);
		UML_EXIT(b, EXECUTE_DONE);
		block.end();
	}
}

void roland_xv_dsp_recompiler::load_registers(sink &b)
{
	dsp_state &s = *m_device.m_dsp;
	UML_DSEXT(b, REG_A, mem(&s.acc[0]), SIZE_DWORD);
	UML_DSEXT(b, REG_B, mem(&s.acc[1]), SIZE_DWORD);
}

void roland_xv_dsp_recompiler::save_registers(sink &b)
{
	dsp_state &s = *m_device.m_dsp;
	UML_MOV(b, mem(&s.acc[0]), REG_A);
	UML_MOV(b, mem(&s.acc[1]), REG_B);
}

//-------------------------------------------------
//  the sample
//-------------------------------------------------

void roland_xv_dsp_recompiler::run()
{
	if (m_dirty)
		refresh();
	if (m_verify)
		verify();
	else
		execute();
}

void roland_xv_dsp_recompiler::execute()
{
	dsp_state &s = *m_device.m_dsp;
	s.steps = 0;
	m_near->pc = 0;
	m_near->mode = s.work_phase & 1;
	for (;;)
	{
		const int result = m_drcuml->execute(*m_entry_handle);
		if (result == EXECUTE_MISSING_CODE && m_near->pc <= LAST_ROW)
			compile(m_near->pc / PAGE, m_near->mode);
		else
		{
			if (result != EXECUTE_DONE)
				m_device.interpret(m_near->pc, -1);
			break;
		}
	}
}

// a row the code leaves to the interpreter, exactly as interpret() runs one that is no branch
void roland_xv_dsp_recompiler::run_row_callback(void *param)
{
	static_cast<roland_xv_dsp_recompiler *>(param)->run_row();
}

void roland_xv_dsp_recompiler::run_row()
{
	roland_xv_device &d = m_device;
	const dsp_row &row = d.m_rows[m_near->row];
	const bool commit = d.condition(row.condition);
	d.execute(row, commit, commit || !row.condition);
	d.m_dsp->steps++;
}

// the interpreter on a copy of the state, then the code on the state, and every difference is fatal
void roland_xv_dsp_recompiler::verify()
{
	roland_xv_device &d = m_device;
	dsp_state &s = *d.m_dsp;
	constexpr int WORK = 2 * dev::WORK_CELLS;
	const dsp_state before = s;
	const std::vector<u32> iram_before(d.m_iram, d.m_iram + dev::IBUS_IRAM_END);
	const std::vector<s32> work_before(d.m_work, d.m_work + WORK);
	const std::vector<u16> late_before(d.m_late_cell, d.m_late_cell + 2);

	d.interpret();
	const dsp_state expected = s;
	const std::vector<u32> iram_expected(d.m_iram, d.m_iram + dev::IBUS_IRAM_END);
	const std::vector<s32> work_expected(d.m_work, d.m_work + WORK);
	const std::vector<u16> tap_cell_expected(d.m_tap_cell, d.m_tap_cell + dev::TAPS);
	const std::vector<s32> tap_delay_expected(d.m_tap_delay, d.m_tap_delay + dev::TAPS);
	const std::vector<u16> tap_slot_expected(d.m_tap_slot, d.m_tap_slot + dev::TAPS);
	const std::vector<u16> late_expected(d.m_late_cell, d.m_late_cell + 2);

	s = before;
	std::copy(iram_before.begin(), iram_before.end(), d.m_iram);
	std::copy(work_before.begin(), work_before.end(), d.m_work);
	std::copy(late_before.begin(), late_before.end(), d.m_late_cell);
	execute();

	auto fail = [&] (const char *what, int index, s64 got, s64 want)
	{
		dump_program();
		fatalerror("%s: DSP recompiler disagrees with the interpreter on %s %d: %x, not %x (rows_end %d, cursor %x, phase %d)\n",
				d.tag(), what, index, got, want, d.m_rows_end, before.cursor, before.work_phase);
	};
	for (int n = 0; n < dev::IBUS_IRAM_END; n++)
		if (d.m_iram[n] != iram_expected[n])
			fail("IRAM cell", n, d.m_iram[n], iram_expected[n]);
	for (int n = 0; n < WORK; n++)
		if (d.m_work[n] != work_expected[n])
			fail("work cell", n, d.m_work[n], work_expected[n]);
	if (s.acc[0] != expected.acc[0]) fail("A", 0, s.acc[0], expected.acc[0]);
	if (s.acc[1] != expected.acc[1]) fail("B", 0, s.acc[1], expected.acc[1]);
	if (s.product != expected.product) fail("P", 0, s.product, expected.product);
	if (s.product_shift != expected.product_shift) fail("product shift", 0, s.product_shift, expected.product_shift);
	for (int n = 0; n < 4; n++)
		if (s.latch[n] != expected.latch[n])
			fail("latch", n, s.latch[n], expected.latch[n]);
	if (s.flag != expected.flag) fail("flag", 0, s.flag, expected.flag);
	if (s.flag_carry != expected.flag_carry) fail("flag carry", 0, s.flag_carry, expected.flag_carry);
	if (s.last_hold != expected.last_hold) fail("hold", 0, s.last_hold, expected.last_hold);
	if (!s.last_hold && s.last != expected.last) fail("last", 0, s.last, expected.last);
	if (!s.last_hold && s.last_carry != expected.last_carry) fail("last carry", 0, s.last_carry, expected.last_carry);
	if (s.steps != expected.steps) fail("steps", 0, s.steps, expected.steps);
	if (s.cursor != expected.cursor) fail("cursor", 0, s.cursor, expected.cursor);
	if (s.work_phase != expected.work_phase) fail("work phase", 0, s.work_phase, expected.work_phase);
	if (s.tap_count != expected.tap_count) fail("tap count", 0, s.tap_count, expected.tap_count);
	for (u32 n = 0; n < s.tap_count; n++)
	{
		if (d.m_tap_cell[n] != tap_cell_expected[n]) fail("tap cell", n, d.m_tap_cell[n], tap_cell_expected[n]);
		if (d.m_tap_delay[n] != tap_delay_expected[n]) fail("tap delay", n, d.m_tap_delay[n], tap_delay_expected[n]);
		if (d.m_tap_slot[n] != tap_slot_expected[n]) fail("tap slot", n, d.m_tap_slot[n], tap_slot_expected[n]);
	}
	if (s.late_count != expected.late_count) fail("late count", 0, s.late_count, expected.late_count);
	for (u32 n = 0; n < s.late_count; n++)
		if (d.m_late_cell[n] != late_expected[n])
			fail("late cell", n, d.m_late_cell[n], late_expected[n]);
}

// the program as loaded, for a failure report: each row's words, and whether code can land on it
void roland_xv_dsp_recompiler::dump_program()
{
	FILE *f = fopen("xv_dsp_fail.txt", "w");
	if (!f)
		return;
	fprintf(f, "cursor %x phase %d\n", m_device.m_dsp->cursor, m_device.m_dsp->work_phase);
	for (int n = 0; n <= m_device.m_rows_end; n++)
		fprintf(f, "%03x %04x %04x %04x%s\n", n, m_device.m_rows[n].w0, m_device.m_rows[n].w1, m_device.m_rows[n].operand, m_entry[n] ? " entry" : "");
	fclose(f);
}

//-------------------------------------------------
//  what a page is compiled from.  The entries are the page starts, every
//  branch's target, and the row after a branch whose delay slot is an
//  entry or the next page's first row.  A row's code is its words and the
//  memory operation its third word carries; the operand values are data.
//-------------------------------------------------

u64 roland_xv_dsp_recompiler::code_key(int n) const
{
	const dsp_row &r = m_device.m_rows[n];
	return u64(r.w0) | (u64(r.w1) << 16) | (u64(r.mode2) << 32) | (u64(r.address2) << 35) | (u64(r.second) << 45);
}

// a row's operand values go straight to the code's table; only a change to its code or to the live rows recompiles
void roland_xv_dsp_recompiler::touched(int n)
{
	const dsp_row &r = m_device.m_rows[n];
	m_near->immediate[n][0] = r.fields[0].immediate;
	m_near->immediate[n][1] = r.fields[1].immediate;
	m_near->coefficient[n] = r.coefficient;
	if (code_key(n) != m_code[n] || m_device.m_rows_end != m_rows_end)
		m_dirty = true;
}

u64 roland_xv_dsp_recompiler::signature(int page) const
{
	u64 hash = 0xcbf29ce484222325;
	for (int n = page * PAGE; n <= page * PAGE + PAGE && n < ROWS; n++)
	{
		const u64 key = m_code[n] | (u64(n <= m_rows_end) << 46) | (u64(m_entry[n]) << 47);
		for (int byte = 0; byte < 8; byte++)
			hash = (hash ^ ((key >> (8 * byte)) & 0xff)) * 0x100000001b3;
	}
	return hash;
}

void roland_xv_dsp_recompiler::refresh()
{
	roland_xv_device &d = m_device;
	m_dirty = false;
	m_rows_end = d.m_rows_end;
	for (int n = 0; n < ROWS; n++)
	{
		const dsp_row &r = d.m_rows[n];
		m_near->immediate[n][0] = r.fields[0].immediate;
		m_near->immediate[n][1] = r.fields[1].immediate;
		m_near->coefficient[n] = r.coefficient;
		m_code[n] = code_key(n);
	}

	std::fill(m_entry.begin(), m_entry.end(), false);
	for (int n = 0; n < ROWS; n += PAGE)
		m_entry[n] = true;
	for (int n = 0; n <= d.m_rows_end && n < ROWS; n++)
	{
		const dsp_row &r = d.m_rows[n];
		if (r.kind == dev::ROW_BRANCH)
			m_entry[(n + 1 + r.displacement) & (ROWS - 1)] = true;
		else if (r.kind == dev::ROW_JUMP)
			m_entry[r.target] = true;
	}
	for (int n = 0; n + 2 < ROWS; n++)
	{
		const dsp_row &r = d.m_rows[n];
		if ((r.kind == dev::ROW_BRANCH || r.kind == dev::ROW_JUMP) && n <= d.m_rows_end && m_entry[n + 1])
			m_entry[n + 2] = true;
	}

	for (int page = 0; page < PAGES; page++)
	{
		const u64 sig = signature(page);
		for (int mode = 0; mode < MODES; mode++)
		{
			if (m_compiled[mode][page] && sig != m_signature[mode][page])
			{
				m_drcuml->hash_invalidate_range(page * PAGE, page * PAGE + PAGE - 1);
				m_compiled[0][page] = m_compiled[1][page] = false;
			}
		}
	}
}

void roland_xv_dsp_recompiler::compile(int page, int mode)
{
	if (m_compiled[mode][page])
	{
		dump_program();
		fatalerror("%s: DSP recompiler has no code for row %x of a compiled page\n", m_device.tag(), m_near->pc);
	}
	for (int attempt = 0; ; attempt++)
	{
		try
		{
			std::fill(m_demand_raw.begin(), m_demand_raw.end(), false);
			std::fill(m_demand_carry.begin(), m_demand_carry.end(), false);
			m_segment_start.clear();
			m_segment_length.clear();
			m_dry = true;
			walk(page, mode, nullptr);
			m_dry = false;
			drcuml_block &block = m_drcuml->begin_block(MAX_INSTRUCTIONS);
			walk(page, mode, &block);
			block.end();
			m_signature[mode][page] = signature(page);
			m_compiled[mode][page] = true;
			return;
		}
		catch (drcuml_block::abort_compilation &)
		{
			m_dry = false;
			if (attempt)
				fatalerror("%s: DSP recompiler cannot compile page %d\n", m_device.tag(), page);
			flush();
		}
	}
}

//-------------------------------------------------
//  a page.  Without a block this only finds which rows' results are
//  wanted and how long each straight run is; with one it emits the code.
//  Rows run in order to a branch, whose delay slot runs before the jump.
//  A straight run starts by checking that it ends short of the budget's
//  last slot, and hands the sample to the interpreter otherwise.
//-------------------------------------------------

void roland_xv_dsp_recompiler::walk(int page, int mode, drcuml_block *block)
{
	sink b{ block, &m_dummy };
	const int start = page * PAGE;
	const int end = start + PAGE;
	const int rows_end = m_device.m_rows_end;
	context c;
	bool reachable = false;
	m_label = LABEL_LOCAL;
	m_steps_pending = 0;
	m_counted = 0;
	m_segment = -1;
	m_stubs.clear();

	int n = start;
	while (n < end)
	{
		if (m_entry[n])
		{
			if (reachable)
			{
				emit_charge(b);
				emit_materialize(b, c);
			}
			UML_HASH(b, mode, n);
			UML_LABEL(b, LABEL_ROW + n);
			c = context();
			reachable = true;
			open_segment(b, c, n);
		}
		if (!reachable)
		{
			n++;
			continue;
		}

		if (n > rows_end)
		{
			// the drain, and the rows left to the fetch bound counted as spent
			emit_merge(b, c);
			c.last_hold = HOLD_SET;
			emit_materialize(b, c);
			dsp_state &s = *m_device.m_dsp;
			UML_ADD(b, REG_X, mem(&s.steps), m_steps_pending + dev::DSP_ROWS_MAPPED - n);
			UML_CMP(b, REG_X, dev::DSP_ROW_BUDGET);
			UML_MOVc(b, COND_A, REG_X, dev::DSP_ROW_BUDGET);
			UML_MOV(b, mem(&s.steps), REG_X);
			m_steps_pending = 0;
			UML_EXH(b, *m_exit, 0);
			reachable = false;
			n++;
			continue;
		}

		const dsp_row &r = m_device.m_rows[n];
		if (r.kind == dev::ROW_BRANCH || r.kind == dev::ROW_JUMP)
		{
			const int next = emit_branch(b, c, n, mode, start, end, reachable);
			if (reachable && next < end && !m_entry[next])
				open_segment(b, c, next);
			n = next;
			continue;
		}

		emit_any_row(b, c, n, mode);
		if (n == LAST_ROW)
		{
			emit_merge(b, c);
			c.last_hold = HOLD_SET;
			emit_charge(b);
			emit_materialize(b, c);
			UML_EXH(b, *m_exit, 0);
			reachable = false;
		}
		n++;
	}
	if (reachable)
	{
		emit_charge(b);
		emit_materialize(b, c);
		emit_goto(b, n, mode, start, end);
	}
	close_segment();
	emit_stubs(b);
}

// a straight run's budget: when its rows would reach the last slot, the interpreter takes the sample from here
void roland_xv_dsp_recompiler::open_segment(sink &b, const context &c, int n)
{
	close_segment();
	m_segment = m_dry ? int(m_segment_start.size()) : m_segment + 1;
	if (m_dry)
	{
		m_segment_start.push_back(m_counted);
		m_segment_length.push_back(0);
	}
	const u32 length = m_dry ? 0 : m_segment_length[m_segment];
	if (m_dry || length)
	{
		dsp_state &s = *m_device.m_dsp;
		const u32 ok = m_label++;
		UML_CMP(b, mem(&s.steps), dev::DSP_ROW_BUDGET - 1 - length);
		UML_JMPc(b, COND_BE, ok);
		emit_materialize(b, c);
		UML_EXH(b, *m_handback, n);
		UML_LABEL(b, ok);
	}
}

void roland_xv_dsp_recompiler::close_segment()
{
	if (m_dry && m_segment >= 0)
		m_segment_length[m_segment] = m_counted - m_segment_start[m_segment];
	if (m_dry)
		m_segment = -1;
}

// a branch and its delay slot; returns the row the walk goes on from
int roland_xv_dsp_recompiler::emit_branch(sink &b, context &c, int n, int mode, int start, int end, bool &reachable)
{
	roland_xv_device &d = m_device;
	dsp_state &s = *d.m_dsp;
	const dsp_row &r = d.m_rows[n];
	const int slot = n + 1;

	// a slot at the fetch bound, or a slot that redirects the fetch again: the interpreter's
	if (slot >= LAST_ROW || d.m_rows[slot].kind == dev::ROW_BRANCH || d.m_rows[slot].kind == dev::ROW_JUMP)
	{
		emit_handback(b, c, n);
		reachable = false;
		return slot;
	}

	const int target = r.kind == dev::ROW_JUMP ? r.target : (n + 1 + r.displacement) & (ROWS - 1);
	const bool dynamic = r.kind == dev::ROW_BRANCH && r.condition > 1;
	if (dynamic)
	{
		const condition_t cond = emit_condition(b, c, r.condition);
		UML_SETc(b, cond, mem(&m_near->taken));
	}

	// a branch to itself over an inert slot, with nothing left to fold in, ends the sample
	const dsp_row &slot_row = d.m_rows[slot];
	if (r.inert && target == n && slot_row.inert && slot_row.kind == dev::ROW_PLAIN && c.last_hold != HOLD_CLEAR)
	{
		const u32 go = m_label++;
		if (dynamic)
		{
			UML_TEST(b, mem(&m_near->taken), 1);
			UML_JMPc(b, COND_Z, go);
		}
		if (c.last_hold == HOLD_DYNAMIC)
		{
			UML_LOAD(b, REG_X, &s.last_hold, 0, SIZE_BYTE, SCALE_x1);
			UML_TEST(b, REG_X, 1);
			UML_JMPc(b, COND_Z, go);
		}
		emit_materialize(b, c);
		UML_MOV(b, mem(&s.steps), dev::DSP_ROW_BUDGET);
		m_steps_pending = 0;
		UML_EXH(b, *m_exit, 0);
		UML_LABEL(b, go);
		if (!dynamic && c.last_hold == HOLD_SET)
		{
			reachable = false;
			return slot;
		}
	}

	emit_row(b, c, n, mode, 1, 1);
	m_steps_pending++;
	m_counted++;
	emit_any_row(b, c, slot, mode);
	emit_charge(b);

	const u32 fall = m_label++;
	if (dynamic)
	{
		UML_TEST(b, mem(&m_near->taken), 1);
		UML_JMPc(b, COND_Z, fall);
	}
	emit_materialize(b, c);
	emit_goto(b, target, mode, start, end);
	if (!dynamic)
	{
		reachable = false;
		return (slot < end && m_entry[slot]) ? slot : slot + 1;
	}
	UML_LABEL(b, fall);

	// a slot code can land on runs again from its own entry, so the way past it goes to the row after
	if (slot >= end || m_entry[slot])
	{
		emit_materialize(b, c);
		emit_goto(b, slot + 1, mode, start, end);
		reachable = false;
		return slot < end ? slot : slot + 1;
	}
	return slot + 1;
}

// a row that is no branch: compiled in one body, in two when the predicate picks its fields, or run through the interpreter
void roland_xv_dsp_recompiler::emit_any_row(sink &b, context &c, int n, int mode)
{
	const dsp_row &r = m_device.m_rows[n];
	if (r.kind == dev::ROW_PREDICATED && r.special)
	{
		emit_charge(b);
		emit_materialize(b, c);
		save_registers(b);
		UML_MOV(b, mem(&m_near->row), n);
		UML_CALLC(b, run_row_callback, this);
		load_registers(b);
		c = context();
		m_counted++;
		return;
	}

	if (r.kind != dev::ROW_PREDICATED || r.condition <= 1)
		emit_row(b, c, n, mode, r.kind != dev::ROW_PREDICATED || r.condition == 1, 1);
	else if (!same_fields(r.fields[0], r.fields[1]))
	{
		const u32 other = m_label++;
		const u32 join = m_label++;
		const condition_t cond = emit_condition(b, c, r.condition ^ 1);
		UML_JMPc(b, cond, other);
		context taken = c;
		emit_row(b, taken, n, mode, 1, 1);
		UML_JMP(b, join);
		UML_LABEL(b, other);
		emit_row(b, c, n, mode, 0, 0);
		UML_LABEL(b, join);
		if (!(taken.flag == c.flag) || !(taken.last == c.last) || taken.last_hold != c.last_hold)
			fatalerror("%s: DSP recompiler: the predicate's two field sets leave row %x's flag chain apart\n", m_device.tag(), n);
		if (taken.product_shift != c.product_shift)
			c.product_shift = DYNAMIC;
		c.narrow[0] = c.narrow[0] && taken.narrow[0];
		c.narrow[1] = c.narrow[1] && taken.narrow[1];
	}
	else if (r.gated || (r.second && r.predicate >= dev::PREDICATE_PARAMETER))
		emit_row(b, c, n, mode, DYNAMIC, DYNAMIC);
	else
		emit_row(b, c, n, mode, 1, 1);
	m_steps_pending++;
	m_counted++;
}

bool roland_xv_dsp_recompiler::same_fields(const dsp_fields &a, const dsp_fields &b)
{
	return a.left == b.left && a.right == b.right && a.source == b.source && a.shift == b.shift
			&& a.negate_left == b.negate_left && a.negate_right == b.negate_right && a.clamp == b.clamp
			&& a.multiply == b.multiply && a.cell_coefficient == b.cell_coefficient;
}

// the latches a row's ALU and multiplier read as it was entered, so a read into one of them lands after
u8 roland_xv_dsp_recompiler::consumed_latches(const dsp_row &r, const dsp_fields &f) const
{
	u8 consumed = 0;
	if (!r.hold)
	{
		if (f.left >= dev::LEFT_R && f.left <= dev::LEFT_M)
			consumed |= 1 << (f.left - dev::LEFT_R);
		if (f.right == dev::RIGHT_R)
			consumed |= 1 << 0;
	}
	const bool product = f.multiply || f.cell_coefficient || f.source >= dev::SOURCE_S;
	if (product && f.source <= dev::SOURCE_C)
		consumed |= 1 << f.source;
	if (f.cell_coefficient || f.source == dev::SOURCE_S)
		consumed |= 1 << 3;
	return consumed;
}

//-------------------------------------------------
//  a row, in the order execute() takes it: the memory operations, then the
//  ALU from the operands the row was entered with, the multiplier, the
//  accumulator, the reads that wait for the row's end, and the flag chain.
//  commit and predicate are 0 or 1, or DYNAMIC for a predicate that is
//  tested here and picks the commit and the parameter-bank alternative.
//-------------------------------------------------

void roland_xv_dsp_recompiler::emit_row(sink &b, context &c, int n, int mode, int commit, int predicate)
{
	roland_xv_device &d = m_device;
	dsp_state &s = *d.m_dsp;
	const dsp_row &r = d.m_rows[n];
	const int field = predicate == 0 ? 1 : 0;
	const dsp_fields &f = r.fields[field];

	if (r.inert)
	{
		emit_merge(b, c);
		c.last_hold = HOLD_SET;
		return;
	}

	if (commit == DYNAMIC)
	{
		const condition_t cond = emit_condition(b, c, r.condition);
		UML_SETc(b, cond, mem(&m_near->commit));
	}

	// the memory operations
	const u8 consumed = consumed_latches(r, f);
	u8 deferred = 0;
	if (r.mode != dev::MEM_NONE || r.address)
		emit_access(b, c, r.mode, r.address, mode, consumed, deferred);
	if (r.second)
	{
		if (predicate == 1 || r.predicate < dev::PREDICATE_PARAMETER)
			emit_access(b, c, r.mode2, r.address2, mode, consumed, deferred);
		else if (predicate == 0)
			emit_alternative(b, c, r, mode, consumed, deferred);
		else if (r.mode2 < dev::MEM_READ_R && r.predicate == dev::PREDICATE_PARAMETER_STORE)
			emit_access(b, c, r.mode2, r.address2, mode, consumed, deferred);
		else
		{
			const u8 maybe = consumed & ((r.mode2 >= dev::MEM_READ_R) ? (1 << (r.mode2 - dev::MEM_READ_R)) : 0);
			for (int latch = 0; latch < 4; latch++)
				if (BIT(maybe, latch) && !BIT(deferred, latch))
					UML_MOV(b, mem(&m_near->scratch[latch]), mem(&s.latch[latch]));
			const u32 other = m_label++;
			const u32 join = m_label++;
			UML_TEST(b, mem(&m_near->commit), 1);
			UML_JMPc(b, COND_Z, other);
			emit_access(b, c, r.mode2, r.address2, mode, consumed, deferred);
			UML_JMP(b, join);
			UML_LABEL(b, other);
			emit_alternative(b, c, r, mode, consumed, deferred);
			UML_LABEL(b, join);
		}
	}

	// the ALU into REG_T: the left operand in REG_X, the right in REG_Y
	if (!r.hold)
	{
		bool have_left = true, have_right = true;
		switch (f.left)
		{
		case dev::LEFT_ZERO:
			have_left = false;
			break;
		case dev::LEFT_R:
		case dev::LEFT_S:
		case dev::LEFT_M:
			UML_DSEXT(b, REG_X, mem(&s.latch[f.left - dev::LEFT_R]), SIZE_DWORD);
			UML_DSHL(b, REG_X, REG_X, 4);
			break;
		case dev::LEFT_A:
		case dev::LEFT_B:
			UML_DMOV(b, REG_X, f.left == dev::LEFT_A ? REG_A : REG_B);
			if (!c.narrow[f.left - dev::LEFT_A])
				emit_clamp(b, REG_X, -0x8000000, 0x7ffffff);
			break;
		default:
			UML_DMOV(b, REG_X, f.left == dev::LEFT_MAG_A ? REG_A : REG_B);
			if (!c.narrow[f.left - dev::LEFT_MAG_A])
				emit_clamp(b, REG_X, -0x8000000, 0x7ffffff);
			UML_DSAR(b, REG_T, REG_X, 63);
			UML_DXOR(b, REG_X, REG_X, REG_T);
			UML_DSUB(b, REG_X, REG_X, REG_T);
			break;
		}
		switch (f.right)
		{
		case dev::RIGHT_ZERO:
			have_right = false;
			break;
		case dev::RIGHT_A:
		case dev::RIGHT_B:
			UML_DMOV(b, REG_Y, f.right == dev::RIGHT_A ? REG_A : REG_B);
			if (!c.narrow[f.right - dev::RIGHT_A])
				emit_clamp(b, REG_Y, -0x8000000, 0x7ffffff);
			break;
		case dev::RIGHT_R:
			UML_DSEXT(b, REG_Y, mem(&s.latch[0]), SIZE_DWORD);
			UML_DSHL(b, REG_Y, REG_Y, 4);
			break;
		case dev::RIGHT_P:
			UML_DMOV(b, REG_Y, mem(&s.product));
			emit_clamp(b, REG_Y, -0x8000000, 0x7ffffff);
			if (c.product_shift == DYNAMIC)
			{
				UML_LOAD(b, REG_T, &s.product_shift, 0, SIZE_BYTE, SCALE_x1);
				UML_DSAR(b, REG_Y, REG_Y, REG_T);
			}
			else if (c.product_shift)
				UML_DSAR(b, REG_Y, REG_Y, c.product_shift);
			break;
		default:
			UML_DSEXT(b, REG_Y, mem(&m_near->immediate[n][field]), SIZE_DWORD);
			break;
		}

		if (have_left && have_right)
		{
			if (f.negate_left && f.negate_right)
			{
				UML_DADD(b, REG_T, REG_X, REG_Y);
				UML_DSUB(b, REG_T, 0, REG_T);
			}
			else if (f.negate_left)
				UML_DSUB(b, REG_T, REG_Y, REG_X);
			else if (f.negate_right)
				UML_DSUB(b, REG_T, REG_X, REG_Y);
			else
				UML_DADD(b, REG_T, REG_X, REG_Y);
		}
		else if (have_left)
		{
			if (f.negate_left)
				UML_DSUB(b, REG_T, 0, REG_X);
			else
				UML_DMOV(b, REG_T, REG_X);
		}
		else if (have_right)
		{
			if (f.negate_right)
				UML_DSUB(b, REG_T, 0, REG_Y);
			else
				UML_DMOV(b, REG_T, REG_Y);
		}
		else
			UML_DMOV(b, REG_T, 0);
		UML_DSHL(b, REG_T, REG_T, 35);
		UML_DSAR(b, REG_T, REG_T, 35);
		if (m_demand_raw[n])
			UML_MOV(b, mem(&m_near->raw[n]), REG_T);

		if (m_demand_carry[n])
		{
			s64 constant = int(f.negate_left) + int(f.negate_right);
			if (have_left)
			{
				if (f.negate_left)
					UML_DXOR(b, REG_X, REG_X, MASK29);
				UML_DAND(b, REG_X, REG_X, MASK29);
			}
			else if (f.negate_left)
				constant += MASK29;
			if (have_right)
			{
				if (f.negate_right)
					UML_DXOR(b, REG_Y, REG_Y, MASK29);
				UML_DAND(b, REG_Y, REG_Y, MASK29);
			}
			else if (f.negate_right)
				constant += MASK29;
			if (!have_left && !have_right)
				UML_MOV(b, mem(&m_near->carry[n]), constant > MASK29 ? 1 : 0);
			else
			{
				const parameter sum = have_left ? REG_X : REG_Y;
				if (have_left && have_right)
					UML_DADD(b, REG_X, REG_X, REG_Y);
				if (constant)
					UML_DADD(b, sum, sum, constant);
				UML_DCMP(b, sum, MASK29);
				UML_SETc(b, COND_A, mem(&m_near->carry[n]));
			}
		}

		if (r.wrap)
		{
			UML_DSHL(b, REG_T, REG_T, 36);
			UML_DSAR(b, REG_T, REG_T, 36);
		}
		else if (f.clamp)
			emit_clamp(b, REG_T, -0x8000000, 0x7ffffff);
	}

	// the multiplier: the operand in REG_X, the coefficient in REG_Y
	enum { COEFFICIENT_NONE, COEFFICIENT_LOW, COEFFICIENT_OPERAND, COEFFICIENT_CELL, COEFFICIENT_A, COEFFICIENT_S } coefficient;
	if (f.multiply)
		coefficient = f.cell_coefficient ? COEFFICIENT_LOW : COEFFICIENT_OPERAND;
	else if (f.cell_coefficient)
		coefficient = COEFFICIENT_CELL;
	else if (f.source >= dev::SOURCE_M)
		coefficient = COEFFICIENT_A;
	else if (f.source == dev::SOURCE_S)
		coefficient = COEFFICIENT_S;
	else
		coefficient = COEFFICIENT_NONE;
	if (coefficient != COEFFICIENT_NONE)
	{
		if (f.source == dev::SOURCE_ZERO)
			UML_DMOV(b, mem(&s.product), 0);
		else
		{
			switch (f.source)
			{
			case dev::SOURCE_R:
			case dev::SOURCE_S:
			case dev::SOURCE_M:
			case dev::SOURCE_C:
				UML_DSEXT(b, REG_X, mem(&s.latch[f.source]), SIZE_DWORD);
				break;
			case dev::SOURCE_A:
			case dev::SOURCE_B:
				UML_DSAR(b, REG_X, f.source == dev::SOURCE_A ? REG_A : REG_B, 4);
				if (!c.narrow[f.source - dev::SOURCE_A])
					emit_clamp(b, REG_X, -0x800000, 0x7fffff);
				break;
			default:
				UML_DMOV(b, REG_X, mem(&s.product));
				UML_DSAR(b, REG_X, REG_X, 4);
				UML_DSEXT(b, REG_X, REG_X, SIZE_DWORD);
				emit_clamp(b, REG_X, -0x800000, 0x7fffff);
				break;
			}
			switch (coefficient)
			{
			case COEFFICIENT_LOW:
				UML_DSEXT(b, REG_Y, mem(&s.latch[3]), SIZE_DWORD);
				UML_DAND(b, REG_Y, REG_Y, 0xff);
				UML_DSHL(b, REG_Y, REG_Y, 15);
				break;
			case COEFFICIENT_OPERAND:
				UML_DSEXT(b, REG_Y, mem(&m_near->coefficient[n]), SIZE_DWORD);
				break;
			case COEFFICIENT_CELL:
				UML_DSEXT(b, REG_Y, mem(&s.latch[3]), SIZE_DWORD);
				UML_DSAR(b, REG_Y, REG_Y, 8);
				UML_DSHL(b, REG_Y, REG_Y, 8);
				break;
			case COEFFICIENT_A:
				UML_DSAR(b, REG_Y, REG_A, 4);
				if (!c.narrow[0])
					emit_clamp(b, REG_Y, -0x800000, 0x7fffff);
				UML_DSAR(b, REG_Y, REG_Y, 8);
				UML_DSHL(b, REG_Y, REG_Y, 8);
				break;
			default:
				UML_DSEXT(b, REG_Y, mem(&s.latch[3]), SIZE_DWORD);
				UML_DSAR(b, REG_Y, REG_Y, 8);
				UML_DOR(b, REG_Y, REG_Y, 0x8000);
				UML_DSEXT(b, REG_Y, REG_Y, SIZE_WORD);
				UML_DSHL(b, REG_Y, REG_Y, 8);
				break;
			}
			UML_DMULSLW(b, REG_X, REG_X, REG_Y);
			UML_DSAR(b, REG_X, REG_X, 19 - f.shift);
			UML_DMOV(b, mem(&s.product), REG_X);
		}
		const int shift = f.multiply && f.cell_coefficient ? 15 : 0;
		if (c.product_shift != shift)
		{
			UML_STORE(b, &s.product_shift, 0, shift, SIZE_BYTE, SCALE_x1);
			c.product_shift = shift;
		}
	}

	// the accumulator, a gated row's only when its predicate holds
	if (!r.hold)
	{
		const parameter acc = r.to_b ? REG_B : REG_A;
		const bool narrow = r.wrap || f.clamp;
		if (!r.gated || commit == 1)
		{
			UML_DMOV(b, acc, REG_T);
			c.narrow[r.to_b] = narrow;
		}
		else if (commit == DYNAMIC)
		{
			UML_TEST(b, mem(&m_near->commit), 1);
			UML_DMOVc(b, COND_NZ, acc, REG_T);
			c.narrow[r.to_b] = c.narrow[r.to_b] && narrow;
		}
	}

	for (int latch = 0; latch < 4; latch++)
		if (BIT(deferred, latch))
			UML_MOV(b, mem(&s.latch[latch]), mem(&m_near->scratch[latch]));

	// the flag chain
	emit_merge(b, c);
	if (r.hold)
		c.last_hold = HOLD_SET;
	else
	{
		c.last = { chain_value::ROW, u16(n) };
		if (!r.gated || !r.condition || commit == 1)
			c.last_hold = HOLD_CLEAR;
		else if (commit == 0)
			c.last_hold = HOLD_SET;
		else
		{
			UML_MOV(b, REG_X, mem(&m_near->commit));
			UML_XOR(b, REG_X, REG_X, 1);
			UML_STORE(b, &s.last_hold, 0, REG_X, SIZE_BYTE, SCALE_x1);
			c.last_hold = HOLD_DYNAMIC;
		}
	}
}

// one memory operation; a read into a latch this row reads lands in its scratch until the row's end
void roland_xv_dsp_recompiler::emit_access(sink &b, const context &c, int mode, u16 address, int bus_mode, u8 consumed, u8 &deferred)
{
	roland_xv_device &d = m_device;
	dsp_state &s = *d.m_dsp;
	switch (mode)
	{
	case dev::MEM_NONE:
		if (address)
		{
			const u32 skip = m_label++;
			UML_MOV(b, REG_X, mem(&s.tap_count));
			UML_CMP(b, REG_X, dev::TAPS);
			UML_JMPc(b, COND_AE, skip);
			UML_DSAR(b, REG_Y, BIT(address, 8) ? REG_B : REG_A, 4);
			UML_STORE(b, d.m_tap_delay, REG_X, REG_Y, SIZE_DWORD, SCALE_x4);
			UML_STORE(b, d.m_tap_cell, REG_X, address | 0x100, SIZE_WORD, SCALE_x2);
			UML_ADD(b, REG_Y, mem(&s.steps), m_steps_pending);
			UML_STORE(b, d.m_tap_slot, REG_X, REG_Y, SIZE_WORD, SCALE_x2);
			UML_ADD(b, mem(&s.tap_count), REG_X, 1);
			UML_LABEL(b, skip);
		}
		break;
	case dev::MEM_STORE_P:
		if (address < dev::IBUS_BANK)
		{
			UML_DMOV(b, REG_X, mem(&s.product));
			UML_DSAR(b, REG_X, REG_X, 4);
			UML_DSEXT(b, REG_X, REG_X, SIZE_DWORD);
			emit_clamp(b, REG_X, -0x800000, 0x7fffff);
			emit_cell_write(b, address, bus_mode);
		}
		break;
	case dev::MEM_STORE_A:
	case dev::MEM_STORE_B:
		if (address < dev::IBUS_BANK)
		{
			UML_DSAR(b, REG_X, mode == dev::MEM_STORE_A ? REG_A : REG_B, 4);
			if (!c.narrow[mode - dev::MEM_STORE_A])
				emit_clamp(b, REG_X, -0x800000, 0x7fffff);
			emit_cell_write(b, address, bus_mode);
		}
		break;
	default:
	{
		const int latch = mode - dev::MEM_READ_R;
		emit_cell_read(b, address, bus_mode);
		if (BIT(consumed, latch))
		{
			UML_MOV(b, mem(&m_near->scratch[latch]), REG_X);
			deferred |= 1 << latch;
		}
		else
			UML_MOV(b, mem(&s.latch[latch]), REG_X);
		break;
	}
	}
}

// a parameter-bank row's second operation when its predicate fails
void roland_xv_dsp_recompiler::emit_alternative(sink &b, const context &c, const dsp_row &r, int bus_mode, u8 consumed, u8 &deferred)
{
	if (r.mode2 >= dev::MEM_READ_R)
		emit_access(b, c, r.mode2, dev::IBUS_BANK + r.argument, bus_mode, consumed, deferred);
	else if (r.predicate == dev::PREDICATE_PARAMETER_STORE)
		emit_access(b, c, r.mode2, r.address2, bus_mode, consumed, deferred);
}

// a cell into REG_X as a 32-bit value; the ring's index and the row count go through REG_Y
void roland_xv_dsp_recompiler::emit_cell_read(sink &b, u16 address, int bus_mode)
{
	roland_xv_device &d = m_device;
	if (address < dev::IBUS_STATIC)
	{
		UML_ADD(b, REG_Y, mem(&d.m_dsp->cursor), address);
		UML_AND(b, REG_Y, REG_Y, dev::RING_CELLS - 1);
		UML_LOAD(b, REG_X, d.m_iram, REG_Y, SIZE_DWORD, SCALE_x4);
		UML_SHL(b, REG_X, REG_X, 8);
		UML_SAR(b, REG_X, REG_X, 8);
	}
	else if (address < dev::IBUS_MIX)
	{
		UML_MOV(b, REG_X, mem(&d.m_iram[address]));
		UML_SHL(b, REG_X, REG_X, 8);
		UML_SAR(b, REG_X, REG_X, 8);
	}
	else if (address < dev::IBUS_BANK)
		UML_MOV(b, REG_X, mem(&d.m_work[bus_mode * dev::WORK_CELLS + address - dev::IBUS_MIX]));
	else if (address < dev::IBUS_BANK_END)
	{
		const int index = address - dev::IBUS_BANK;
		UML_ADD(b, REG_Y, mem(&d.m_dsp->steps), m_steps_pending);
		UML_MOV(b, REG_X, mem(&d.m_bank[index]));
		UML_CMP(b, REG_Y, dev::bank_slot(index));
		UML_MOVc(b, COND_AE, REG_X, mem(&d.m_bank_next[index]));
	}
	else
		UML_MOV(b, REG_X, 0);
}

// REG_X into a cell; the ring's index goes through REG_Y
void roland_xv_dsp_recompiler::emit_cell_write(sink &b, u16 address, int bus_mode)
{
	roland_xv_device &d = m_device;
	if (address < dev::IBUS_STATIC)
	{
		UML_ADD(b, REG_Y, mem(&d.m_dsp->cursor), address);
		UML_AND(b, REG_Y, REG_Y, dev::RING_CELLS - 1);
		UML_STORE(b, d.m_iram, REG_Y, REG_X, SIZE_DWORD, SCALE_x4);
	}
	else if (address < dev::IBUS_MIX)
		UML_MOV(b, mem(&d.m_iram[address]), REG_X);
	else if (address < dev::IBUS_BANK)
		UML_MOV(b, mem(&d.m_work[bus_mode * dev::WORK_CELLS + address - dev::IBUS_MIX]), REG_X);
}

// a register clamped: two compares that seldom branch, to ends emitted after the page
void roland_xv_dsp_recompiler::emit_clamp(sink &b, parameter reg, s64 low, s64 high)
{
	const clamp_stub stub{ m_label++, m_label++, m_label++, reg, low, high };
	UML_DCMP(b, reg, high);
	UML_JMPc(b, COND_G, stub.high_label);
	UML_DCMP(b, reg, low);
	UML_JMPc(b, COND_L, stub.low_label);
	UML_LABEL(b, stub.back);
	m_stubs.push_back(stub);
}

void roland_xv_dsp_recompiler::emit_stubs(sink &b)
{
	for (const clamp_stub &stub : m_stubs)
	{
		UML_LABEL(b, stub.high_label);
		UML_DMOV(b, stub.reg, stub.high);
		UML_JMP(b, stub.back);
		UML_LABEL(b, stub.low_label);
		UML_DMOV(b, stub.reg, stub.low);
		UML_JMP(b, stub.back);
	}
	m_stubs.clear();
}

//-------------------------------------------------
//  the flag chain.  A condition reads the flag; at a row's end the last
//  result becomes the flag unless it was held.  Along straight code the
//  compiler knows which row's result each is, and that row stores it
//  where it is wanted; at a hash entry they are whatever memory holds,
//  and code that leaves for another entry writes memory first.
//-------------------------------------------------

// a condition code of two or more against the flag, as the host condition that holds when it does; uses REG_X and REG_Y
condition_t roland_xv_dsp_recompiler::emit_condition(sink &b, const context &c, int code)
{
	dsp_state &s = *m_device.m_dsp;
	const chain_value &v = c.flag;
	const parameter value = v.kind == chain_value::ROW ? mem(&m_near->raw[v.row]) : v.kind == chain_value::IN_LAST ? mem(&s.last) : mem(&s.flag);
	const bool wants_value = code != 4 && code != 5;
	const bool wants_carry = code == 4 || code == 5 || code >= 0xc;
	if (wants_value)
		demand_raw(v);
	if (wants_carry)
	{
		demand_carry(v);
		if (v.kind == chain_value::ROW)
			UML_MOV(b, REG_X, mem(&m_near->carry[v.row]));
		else
			UML_LOAD(b, REG_X, v.kind == chain_value::IN_LAST ? &s.last_carry : &s.flag_carry, 0, SIZE_BYTE, SCALE_x1);
	}
	switch (code)
	{
	case 0x2: UML_CMP(b, value, 0); return COND_E;
	case 0x3: UML_CMP(b, value, 0); return COND_NE;
	case 0x4: UML_TEST(b, REG_X, 1); return COND_NZ;
	case 0x5: UML_TEST(b, REG_X, 1); return COND_Z;
	case 0x6:
	case 0x7:
		UML_ADD(b, REG_Y, value, 0x8000000);
		UML_CMP(b, REG_Y, 0x10000000);
		return code == 0x6 ? COND_AE : COND_B;
	case 0x8: UML_CMP(b, value, 0); return COND_GE;
	case 0x9: UML_CMP(b, value, 0); return COND_L;
	case 0xa: UML_CMP(b, value, 0); return COND_G;
	case 0xb: UML_CMP(b, value, 0); return COND_LE;
	case 0xc:
	case 0xd:
		UML_CMP(b, value, 0);
		UML_SETc(b, COND_E, REG_Y);
		UML_OR(b, REG_X, REG_X, REG_Y);
		UML_TEST(b, REG_X, 1);
		return code == 0xc ? COND_NZ : COND_Z;
	default:
		UML_CMP(b, value, 0);
		UML_SETc(b, COND_NE, REG_Y);
		UML_AND(b, REG_X, REG_X, REG_Y);
		UML_TEST(b, REG_X, 1);
		return code == 0xe ? COND_NZ : COND_Z;
	}
}

// what every row does at its end before its own result: the last result becomes the flag unless it was held
void roland_xv_dsp_recompiler::emit_merge(sink &b, context &c)
{
	dsp_state &s = *m_device.m_dsp;
	if (c.last_hold == HOLD_CLEAR)
		c.flag = c.last;
	else if (c.last_hold == HOLD_DYNAMIC)
	{
		emit_store_flag(b, c);
		demand_raw(c.last);
		demand_carry(c.last);
		const u32 skip = m_label++;
		UML_LOAD(b, REG_X, &s.last_hold, 0, SIZE_BYTE, SCALE_x1);
		UML_TEST(b, REG_X, 1);
		UML_JMPc(b, COND_NZ, skip);
		if (c.last.kind == chain_value::ROW)
		{
			UML_MOV(b, mem(&s.flag), mem(&m_near->raw[c.last.row]));
			UML_MOV(b, REG_X, mem(&m_near->carry[c.last.row]));
		}
		else
		{
			UML_MOV(b, mem(&s.flag), mem(&s.last));
			UML_LOAD(b, REG_X, &s.last_carry, 0, SIZE_BYTE, SCALE_x1);
		}
		UML_STORE(b, &s.flag_carry, 0, REG_X, SIZE_BYTE, SCALE_x1);
		UML_LABEL(b, skip);
		c.flag = { chain_value::IN_FLAG, 0 };
	}
}

// the chain into memory, for code that hands over; a held last result is never read again, so it stays where it is
void roland_xv_dsp_recompiler::emit_materialize(sink &b, const context &c)
{
	dsp_state &s = *m_device.m_dsp;
	emit_store_flag(b, c);
	if (c.last_hold != HOLD_SET && c.last.kind == chain_value::ROW)
	{
		demand_raw(c.last);
		demand_carry(c.last);
		UML_MOV(b, mem(&s.last), mem(&m_near->raw[c.last.row]));
		UML_MOV(b, REG_X, mem(&m_near->carry[c.last.row]));
		UML_STORE(b, &s.last_carry, 0, REG_X, SIZE_BYTE, SCALE_x1);
	}
	if (c.last_hold != HOLD_DYNAMIC)
		UML_STORE(b, &s.last_hold, 0, c.last_hold, SIZE_BYTE, SCALE_x1);
}

void roland_xv_dsp_recompiler::emit_store_flag(sink &b, const context &c)
{
	dsp_state &s = *m_device.m_dsp;
	if (c.flag.kind == chain_value::ROW)
	{
		demand_raw(c.flag);
		demand_carry(c.flag);
		UML_MOV(b, mem(&s.flag), mem(&m_near->raw[c.flag.row]));
		UML_MOV(b, REG_X, mem(&m_near->carry[c.flag.row]));
		UML_STORE(b, &s.flag_carry, 0, REG_X, SIZE_BYTE, SCALE_x1);
	}
	else if (c.flag.kind == chain_value::IN_LAST)
	{
		UML_MOV(b, mem(&s.flag), mem(&s.last));
		UML_LOAD(b, REG_X, &s.last_carry, 0, SIZE_BYTE, SCALE_x1);
		UML_STORE(b, &s.flag_carry, 0, REG_X, SIZE_BYTE, SCALE_x1);
	}
}

// the rows since the last charge onto the sample's count
void roland_xv_dsp_recompiler::emit_charge(sink &b)
{
	if (!m_steps_pending)
		return;
	dsp_state &s = *m_device.m_dsp;
	UML_ADD(b, mem(&s.steps), mem(&s.steps), m_steps_pending);
	m_steps_pending = 0;
}

// to an entry: within the page directly, otherwise through the hash; past the mapped rows, the interpreter ends the sample
void roland_xv_dsp_recompiler::emit_goto(sink &b, int target, int mode, int start, int end)
{
	if (target > LAST_ROW)
		UML_EXH(b, *m_handback, target);
	else if (target >= start && target < end)
		UML_JMP(b, LABEL_ROW + target);
	else
		UML_HASHJMP(b, mode, target, *m_nocode);
}

void roland_xv_dsp_recompiler::emit_handback(sink &b, const context &c, int n)
{
	emit_charge(b);
	emit_materialize(b, c);
	UML_EXH(b, *m_handback, n);
}
