// license:BSD-3-Clause
// copyright-holders:superctr
#ifndef MAME_SOUND_ROLAND_LSP_H
#define MAME_SOUND_ROLAND_LSP_H

#pragma once

class roland_lsp_device : public cpu_device
{
public:
	static constexpr int PROGRAM_SIZE = 384;
	static constexpr int PROGRAM_BASE = 0x080;
	static constexpr u16 ADDRESS_MASK = 0x1ff;
	static constexpr int IRAM_SIZE = 0x80;
	static constexpr int ERAM_SIZE = 0x10000;

	// the slots of the two special opcodes, by slot number
	enum special_slot
	{
		SLOT_JUMP_NEGATIVE = 0x0d, SLOT_JUMP_POSITIVE = 0x0e, SLOT_JUMP = 0x0f,
		SLOT_ERAM_WRITE = 0x10, SLOT_SKIP = 0x12, SLOT_TAP = 0x13, SLOT_MULTIPLIER = 0x14,
		SLOT_STORE_76 = 0x16, SLOT_AUDIO_OUT = 0x18, SLOT_ERAM_READ = 0x1a, SLOT_AUDIO_IN = 0x1e
	};

	// the sixteen host registers at CA0-3
	enum host_register
	{
		HOST_ADDRESS_LOW = 0x00, HOST_ADDRESS_HIGH = 0x01, HOST_DATA_LOW = 0x02,
		HOST_DATA_MID = 0x03, HOST_DATA_HIGH = 0x04, HOST_CONFIGURE = 0x06,
		HOST_READ_LOW = 0x08, HOST_READ_HIGH = 0x09
	};

	roland_lsp_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	u8 host_r(offs_t offset);
	void host_w(offs_t offset, u8 data);

	// TRR in and TRS0 out, channel 0 left and 1 right, one pair per sample
	void ser_w(int channel, s32 sample) { m_serial_in[channel & 1] = sample; }
	s32 ser_r(int channel) const { return m_serial_out[channel & 1]; }
	void set_eram_block_float(bool on) { m_eram_block_float = on; }
	void run_once();

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_post_load() override ATTR_COLD;

	// device_execute_interface implementation
	virtual u64 execute_clocks_to_cycles(u64 clocks) const noexcept override { return (clocks + 1) / 2; }
	virtual u64 execute_cycles_to_clocks(u64 cycles) const noexcept override { return cycles * 2; }
	virtual u32 execute_min_cycles() const noexcept override { return 1; }
	virtual u32 execute_max_cycles() const noexcept override { return 1; }
	virtual void execute_run() override;

	// device_memory_interface implementation
	virtual space_config_vector memory_space_config() const override;

	// device_disasm_interface implementation
	virtual std::unique_ptr<util::disasm_interface> create_disassembler() override;

	enum opcode_family
	{
		OP_MAC_A = 0, OP_SET_A = 1, OP_MAC_B = 2, OP_SET_B = 3,
		OP_MUL = 4, OP_ABS = 5, OP_SPECIAL_A = 6, OP_SPECIAL_B = 7
	};

	// one instruction word, split into its fields
	struct instruction
	{
		bool nop;
		u8 opcode;
		u8 store;
		u8 eram_cmd;
		u8 offset;
		u8 coefficient;
		u8 shift;

		int slot() const { return offset & 0x1f; }
		bool replace() const { return BIT(offset, 5); }
		bool special() const { return opcode >= OP_SPECIAL_A && BIT(offset, 6); }
		bool second_operand() const { return special() && slot() == SLOT_ERAM_WRITE && !store; }
		bool write_latch() const { return special() && slot() == SLOT_ERAM_WRITE && store; }
		bool jump() const { return special() && slot() >= SLOT_JUMP_NEGATIVE && slot() <= SLOT_JUMP; }
		bool skip() const { return special() && slot() == SLOT_SKIP; }
		bool low_half() const { return opcode == OP_MUL && BIT(coefficient, 6); }
		bool high_half() const { return opcode == OP_MUL && !BIT(coefficient, 6); }
		bool mac() const { return !nop && opcode != OP_MUL && !special(); }
	};

	static constexpr instruction EMPTY = { true, 0, 0, 0, 0, 0, 7 };

	// what the multiplier's data port takes: internal memory at an offset, the bit-set constant of
	// offsets 1 to 4, or the value a special handled
	enum operand_source : u8 { FROM_MEMORY, FROM_CONSTANT, FROM_SPECIAL };

	struct operand_ref
	{
		u8 from;
		u8 offset;
	};

	static instruction decode(u32 word);

	static s32 saturate(s32 value) { return std::clamp<s32>(value, -0x800000, 0x7fffff); }
	static s32 narrow(s32 value) { return util::sext(value, 24); }
	static s32 add(s32 accumulator, s32 term) { return s32(u32(accumulator) + u32(term)); }

	void program_map(address_map &map) ATTR_COLD;

	u32 internal_r(offs_t address);
	void internal_w(offs_t address, u32 data);
	void configure(u16 word);

	void eram_clock(u8 command);
	int eram_opened(int back) const { return m_eram_cmd[back] >> 3; }
	u16 eram_base(int kind, int back) const;
	s32 eram_word(s32 value) const;
	int iram_address(int offset) const { return (offset + m_buffer_pos) & 0x7f; }

	s32 source(const instruction &s) const;
	operand_ref resolve(const instruction &s);
	s32 operand(const operand_ref &o) const;
	static void accumulate(s32 &acc, s32 term, bool replace) { acc = replace ? term : add(acc, term); }
	void mac(const instruction &s, const operand_ref &o);
	void multiply(const instruction &s, const operand_ref &o);
	void special(const instruction &s, const operand_ref &o);
	void step();
	void finish_sample();

	address_space_config m_program_config;

	std::unique_ptr<s32[]> m_eram;
	bool m_eram_block_float = true;
	u32 m_program[PROGRAM_SIZE];
	instruction m_decoded[PROGRAM_SIZE];
	s32 m_iram[IRAM_SIZE];

	int m_icount;
	u16 m_pc;
	u16 m_slot;
	u16 m_configuration;
	bool m_running;
	bool m_halted;

	s32 m_acc[2];
	s32 m_history[2][3];
	operand_ref m_hand_on;
	bool m_after_low;
	u8 m_run;
	s32 m_handed;
	s32 m_multiplier[2];
	u8 m_eram_cmd[16];
	s32 m_eram_read;
	s32 m_eram_latch;
	u16 m_eram_address;
	u16 m_tap;
	u8 m_buffer_pos;
	u16 m_eram_pos;
	bool m_jump;
	s16 m_follow;

	s32 m_serial_in[2];
	s32 m_serial_out[2];
	s32 m_audio_in;
	s32 m_audio_out;

	u32 m_host_data;
	u32 m_host_read;
	u16 m_host_address;
};

DECLARE_DEVICE_TYPE(ROLAND_LSP, roland_lsp_device)

#endif // MAME_SOUND_ROLAND_LSP_H
