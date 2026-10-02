// license:BSD-3-Clause
// copyright-holders:superctr
#ifndef MAME_SOUND_ROLAND_MR_H
#define MAME_SOUND_ROLAND_MR_H

#pragma once

class roland_mr_device : public device_t, public device_sound_interface
{
public:
	static constexpr feature_type imperfect_features() { return feature::SOUND; }

	static constexpr int ROWS = 512;
	static constexpr int CELLS = 256;
	static constexpr int ERAM_WORDS = 0x5000;
	static constexpr int ERAM_BITS = 14;
	static constexpr int LANES = 8;
	static constexpr int RESULTS = 4;
	static constexpr u32 CLOCKS_PER_SAMPLE = 384;

	enum command : u16
	{
		CMD_NONE = 0x0000, CMD_WRITE = 0x0200, CMD_READ = 0x0400, CMD_FIELD = 0x0600,
		CMD_COEFFICIENT = 0x0800, CMD_EXTENDED = 0x0a00, CMD_0C = 0x0c00, CMD_0E = 0x0e00,
		CMD_14 = 0x1400, CMD_1A = 0x1a00
	};

	roland_mr_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	void map(address_map &map) ATTR_COLD;

	u32 data_r();
	void data_w(offs_t offset, u32 data, u32 mem_mask = ~0);
	void command_w(offs_t offset, u16 data, u16 mem_mask = ~0);
	u16 result_r(offs_t offset);
	u8 status_r();
	void control_w(offs_t offset, u8 data);

	u32 program_word(int row) const { return m_program[row & (ROWS - 1)]; }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_clock_changed() override;
	virtual void device_post_load() override;
	virtual void sound_stream_update(sound_stream &stream) override;

private:
	static constexpr int Q = 23;
	static constexpr s64 ONE = s64(1) << Q;
	static constexpr s64 WIDE_ONE = s64(1) << (2 * Q);
	static constexpr int SECOND_WORDS = 2;
	static constexpr int MAX_BLANK = 3;
	static constexpr int MAX_PENDING = 8;

	enum kind : u8
	{
		OP_INVALID, OP_ACC, OP_EXT, OP_ABS, OP_STORE_ABS, OP_CROSS, OP_CONTINUE, OP_K_LOAD, OP_ADDRESS,
		OP_HOST, OP_TRANSFER, OP_K_FORM, OP_NOP, OP_INTERPOLATED, OP_WRAP, OP_POSITIVE, OP_COMPARE,
		OP_SIGN, OP_JUMP, OP_INPUT, OP_SEND, OP_RETURN, OP_TRIANGLE
	};

	struct row
	{
		u8 kind;
		u8 path;
		u8 cell;
		u8 arg;
		bool start;
		bool stores;
		bool gates;
		bool flag;
		s32 coefficient;
		u16 target;
	};

	struct blank_entry { s16 row; u8 path; };
	struct pending_entry { s16 row; u8 path; bool holds; };

	void decode(int pc);
	void execute_sample();
	int step(int pc);
	void arm(int pc, int path, bool holds);
	void settle(int pc);
	void gate(int armed, s64 value, int path);
	bool blanked(int pc, int path) const;

	s64 read_cell(int cell) const;
	void write_cell(int cell, s64 value);
	s64 aged(int path) const { return m_history[m_history_pos][path]; }
	static s64 saturate(s64 value);
	static s64 narrow(s64 value) { return saturate(value) >> Q; }
	static s64 wrap(s64 value);
	int eram_index(int offset) const;

	sound_stream *m_stream;

	u32 m_program[ROWS];
	row m_rows[ROWS];

	u32 m_data;
	u16 m_last_command;
	u8 m_control[0x20];

	s64 m_ring[CELLS];
	s32 m_eram[ERAM_WORDS];
	s64 m_k[4];
	s64 m_address;
	s64 m_acc[2];
	s64 m_history[3][2];
	u8 m_history_pos;
	s64 m_operand;
	u8 m_ring_pos;
	u16 m_eram_pos;
	u8 m_second_pos;
	s64 m_second[2][SECOND_WORDS];
	s64 m_result[RESULTS];
	s64 m_input[LANES];
	s64 m_output[LANES];

	blank_entry m_blank[MAX_BLANK];
	u8 m_blank_count;
	pending_entry m_pending[MAX_PENDING];
	u8 m_pending_count;
};

DECLARE_DEVICE_TYPE(ROLAND_MR, roland_mr_device)

#endif // MAME_SOUND_ROLAND_MR_H
