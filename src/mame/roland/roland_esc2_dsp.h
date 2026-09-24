// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Roland ESC2 (MB8AA4181) DSP

***************************************************************************/

#ifndef MAME_ROLAND_ROLAND_ESC2_DSP_H
#define MAME_ROLAND_ROLAND_ESC2_DSP_H

#pragma once

#include <bit>
#include <unordered_map>
#include <vector>


class mb8aa4181_dsp_device : public device_t, public device_sound_interface
{
public:
	enum
	{
		IRQ_NOTIFY0, IRQ_NOTIFY1,
		IRQ_TARGET0, IRQ_TARGET1,
		IRQ_SWITCH0, IRQ_SWITCH1,
		IRQ_COUNT
	};

	mb8aa4181_dsp_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	auto irq_cb(unsigned n) { return m_irq_cb[n].bind(); }

	u32 read(offs_t offset, u32 mem_mask);
	void write(offs_t offset, u32 data, u32 mem_mask);

	static u32 native_number(u32 data, unsigned bits);
	static double native_value(u32 raw, unsigned bits);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_post_load() override ATTR_COLD;

	virtual void sound_stream_update(sound_stream &stream) override;

private:
	static constexpr unsigned UNITS = 2;
	static constexpr unsigned PRG_WORDS = 0x4000;
	static constexpr unsigned PRG_HALFWORDS = PRG_WORDS * 2;
	static constexpr unsigned DIRECT_WORDS = 0x100;
	static constexpr unsigned TARGETS = 0x200;
	static constexpr unsigned DRP_ENTRIES = 0x400;
	static constexpr unsigned SHORT_WORDS = 0x2000;
	static constexpr unsigned BUS_WORDS = 0x200;
	static constexpr unsigned SLOTS = 0x200;
	static constexpr unsigned SAMPLE_OWNERS = 0x2000;
	static constexpr unsigned MEMORY_CELLS = 0x800000;
	static constexpr unsigned MAX_OPS = 8;
	static constexpr unsigned MAX_EXTENSION = 32;
	static constexpr unsigned STACK_DEPTH = 8;
	static constexpr unsigned FRAME_PACKETS = 0x4000;
	static constexpr unsigned OUTPUT_BUFFER = 0x4000;

	enum : u8 { KIND_A, KIND_B, KIND_C, KIND_D, KIND_E };

	enum : u8
	{
		OP_NONE, OP_UNSUPPORTED,
		OP_A,
		OP_B_SUM, OP_B_SQUARE, OP_B_NEWTON, OP_B_MOVE, OP_B_DOUBLE, OP_B_MUL, OP_B_MAC, OP_B_F2, OP_B_F3,
		OP_C_ARRIVAL_CLEAR, OP_C_ADVANCE, OP_C_CLAMP, OP_C_BIT, OP_C_FRACTION, OP_C_LOGIC, OP_C_MIN, OP_C_MAX,
		OP_C_COMPARE, OP_C_SET, OP_C_RECIPROCAL, OP_C_SEED, OP_C_EXPONENT, OP_C_LOGARITHM, OP_C_SIGN, OP_C_FLOOR,
		OP_C_FLOOR14, OP_C_FRACTION14, OP_C_SELECTOR, OP_C_ADD_SELECTOR, OP_C_ADD, OP_C_F3,
		OP_C_UNARY0, OP_C_UNARY1, OP_C_UNARY2, OP_C_UNARY3, OP_C_UNARY4, OP_C_UNARY5, OP_C_UNARY6,
		OP_C_SCALE, OP_C_SHIFT,
		OP_D_SELECT, OP_D_ADDRESS, OP_D_ADDRESS_REGISTER, OP_D_JUMP, OP_D_CALL, OP_D_NOTIFY, OP_D_CLEAR, OP_D_COUNTER, OP_D_RETURN,
		OP_E_OPERAND, OP_E_PUBLISH, OP_E_INDIRECT, OP_E_DELAY, OP_E_TOKEN, OP_E_SAMPLE, OP_E_REQUEST,
		OP_E_LOCAL, OP_E_SHORT, OP_E_DIRECT, OP_E_PARAMETER, OP_E_TARGET
	};

	enum : u8
	{
		F_SIGMA = 0x01, F_SECOND = 0x02, F_SECOND_IMMEDIATE = 0x04, F_SECOND_ALONE = 0x08,
		F_NEGATE = 0x10, F_ADDEND = 0x20, F_HALVE = 0x40, F_MOVE = 0x80,
		F_FUNCTION = 0x02, F_CONDITIONAL = 0x02, F_STORE = 0x02
	};

	struct source
	{
		double value;
		u8 reg;
		u8 mode;
	};

	struct operation
	{
		u8 kind;
		u8 type;
		u8 d, x, y;
		u8 flags;
		u16 word;
		u16 aux;
		u8 drop_true, drop_false, named;
		source s, t;
		double q;
		u32 value;
	};

	struct packet
	{
		u16 pc;
		u16 length;
		int next;
		u8 count;
		u8 selectors;
		u8 extension_length;
		operation ops[MAX_OPS];
		u8 extension[MAX_EXTENSION];
	};

	struct history
	{
		double last, visible;
		bool has_last, has_visible;

		void reset() { last = visible = 0.0; has_last = has_visible = false; }
		void push(bool has, double value)
		{
			if (has_last)
			{
				visible = last;
				has_visible = true;
			}
			has_last = has;
			last = value;
		}
	};

	struct direct_write
	{
		u16 index;
		u32 packet;
		u32 before;
	};

	struct sample_request
	{
		u32 owner;
		float value;
	};

	struct unit_state
	{
		u32 prg[PRG_WORDS];
		u32 direct[DIRECT_WORDS];
		u32 target[TARGETS][4];
		u32 drp[DRP_ENTRIES][2];

		double r[8];
		double sel[4];
		double shortmem[SHORT_WORDS];
		double local[SHORT_WORDS];
		double bus[BUS_WORDS];
		double slot[SLOTS];
		double requested[SLOTS];
		u8 slot_valid[SLOTS];
		u8 requested_valid[SLOTS];
		s32 slot_base[SLOTS / 4];
		float sample[SAMPLE_OWNERS * 16];
		u32 origin;

		history arith, comp, func;
		double arith_published;
		bool has_arith_published;
		bool comparison_published;
		bool arrival, arrival_last, arrival_visible;

		u32 stack[STACK_DEPTH];
		u8 depth;

		std::vector<packet> packets;
		std::vector<int> index;
		int first;
		bool dirty;

		s16 descriptor[SLOTS];
		std::vector<u16> reads;
		std::vector<u16> pending;
		double buffered[SLOTS];
		bool drp_dirty;

		std::vector<sample_request> samples;
		std::vector<direct_write> direct_log;
		u32 packets_run;

		u32 target_queued[TARGETS / 32];
		u32 target_pending[TARGETS / 32];
		u16 notify_pending;
	};

	devcb_write_line::array<IRQ_COUNT> m_irq_cb;

	sound_stream *m_stream;
	emu_timer *m_frame_timer;
	emu_timer *m_notify_timer[UNITS][16];
	u32 m_packet_clock;
	attotime m_frame_start;
	std::unique_ptr<unit_state[]> m_unit;
	std::unique_ptr<u32[]> m_memory;
	double m_shared[0x100];
	double m_port[0x100];
	std::unordered_map<offs_t, u32> m_regs;
	u32 m_field;
	u32 m_flags;
	u32 m_frames;
	u8 m_switch_queued;
	u8 m_switch_done;
	float m_output[OUTPUT_BUFFER][2];
	u32 m_output_read;
	u32 m_output_write;
	float m_last[2];
	std::unordered_map<u32, bool> m_unsupported;

	char m_layout[256][MAX_OPS + 1];

	TIMER_CALLBACK_MEMBER(frame);
	TIMER_CALLBACK_MEMBER(notify);
	void direct_store(unit_state &u, unsigned index, double value);

	void host_write(offs_t offset, u32 data, u32 mem_mask);
	u32 host_read(offs_t offset) const;
	void field_write(u32 data);
	u32 field_read() const;

	void irq_update(unsigned unit);
	int target_next(unsigned unit) const;

	u16 halfword(const unit_state &u, u32 pc) const;
	void decode(unit_state &u);
	void rebuild_descriptors(unit_state &u);
	void begin_frame(unit_state &u);
	void end_frame(unit_state &u);
	void run_frame(unsigned unit);

	static double number(const packet &p, unsigned offset, unsigned width);
	static u32 raw(const packet &p, unsigned offset, unsigned width);
	static source make_source(const packet &p, unsigned code, bool subtract);
	static double get(const source &s, const double *r) { return s.mode == 2 ? s.value : s.mode ? -r[s.reg] : r[s.reg]; }
	void classify(const packet &p, unsigned k, operation &op);
	bool condition(const unit_state &u, u16 word) const;
	double read_operand(unsigned unit, u16 address) const;
	void write_operand(unsigned unit, u16 address, double value);
	double shortmem(const unit_state &u, u32 address) const { return u.shortmem[address & (SHORT_WORDS - 1)]; }
	float cell(u32 address) const { return std::bit_cast<float>(m_memory[address & (MEMORY_CELLS - 1)]); }
	void set_cell(u32 address, double value) { m_memory[address & (MEMORY_CELLS - 1)] = std::bit_cast<u32>(float(value)); }
	u32 token_cell(const unit_state &u, double token) const;
	int step(unsigned unit, const packet &p, bool &call);
	void unsupported(unsigned unit, const packet &p, const operation &op);
};

DECLARE_DEVICE_TYPE(MB8AA4181_DSP, mb8aa4181_dsp_device)

#endif // MAME_ROLAND_ROLAND_ESC2_DSP_H
