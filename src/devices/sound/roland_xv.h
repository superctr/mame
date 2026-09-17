// license:BSD-3-Clause
// copyright-holders:superctr
#ifndef MAME_SOUND_ROLAND_XV_H
#define MAME_SOUND_ROLAND_XV_H

#pragma once

class roland_xv_device : public device_t, public device_memory_interface, public device_sound_interface
{
public:
	static constexpr feature_type imperfect_features() { return feature::SOUND; }

	// the wave ROMs, the expansion boards and the sample RAM, 16 bits a cell,
	// addressed by cell, one space the board shares between its chips
	enum { AS_WAVE = 0 };

	static constexpr int OBJECTS = 64;
	static constexpr int FIFO_DEPTH = 256;
	static constexpr int IRQ_REASONS = 16;
	static constexpr int BUSES = 16;
	static constexpr int BUS_CHORUS = 6;
	static constexpr int BUS_REVERB = 7;
	static constexpr int SENDS = 6;
	static constexpr u32 SAMPLE_RATE = 44100;
	static constexpr int RAMP_FRACTION_BITS = 12;
	static constexpr int OUTPUT_BITS = 18;
	static constexpr int DSP_FRACTION_BITS = 23;
	static constexpr int DSP_ROWS = 0x400;
	static constexpr int DSP_ROW_BUDGET = 0x400;
	static constexpr int RING_CELLS = 0x400;
	static constexpr int RECORDS = 192;
	static constexpr int ERAM_CELLS = 1 << 19;
	static constexpr int TAPS = 64;
	static constexpr int DAC_PAIRS = 4;
	static constexpr int OUTPUTS = 2 * DAC_PAIRS;
	static constexpr u32 ADDRESS_MASK = 0x0fffffff;
	static constexpr u32 PAGE_MASK = 0x000fffff;
	static constexpr int BANK_BYTE_WIDE = 1;

	// the registers of the window, by word number
	enum register_word
	{
		MODE = 0x02, DATA_HIGH = 0x04, DATA_LOW = 0x05, ADDRESS = 0x06, FIFO = 0x08, FIFO_CONTROL = 0x09,
		RUN_MASK = 0x0a, RUN_COMMIT = 0x0e, IRQ_MASK = 0x0f, IRQ_ACK = 0x10, IRQ_VOICE = 0x10, STATUS = 0x1b,
		XFER_COMMAND = 0x25, WRITE_ADDRESS = 0x26, WRITE_LENGTH = 0x2a, READ_GO = 0x2d, READ_ADDRESS = 0x2e,
		READ_LENGTH = 0x32, COMMAND_STROBE = 0x36,
		OBJECT_BASE = 0x60, OBJECT_END = 0x100
	};

	// the per-voice register file behind word 0x02, by word number
	enum voice_word
	{
		VOICE_CONTROL = 0x60, VOICE_CONTROL2 = 0x61, PITCH_STEP = 0x72, LOOP_FRACTION = 0x74, WAVE_SCALE = 0x76,
		PITCH_INCREMENT = 0x7c, START = 0x80, LOOP_START = 0x82, END = 0x84,
		FILTER_BAND = 0xb0, FILTER_LOW = 0xb2,
		CUTOFF_RAMP = 0x90, FEEDBACK_RAMP = 0x92, LEVEL_RAMP = 0x94, BLOCK_CONTROL = 0x96,
		SEND_PORT_A = 0x98, SEND_PORT_B = 0x9a, PITCH_RAMP = 0x9c,
		CUTOFF_INCREMENT = 0xa0, FEEDBACK_INCREMENT = 0xa2, LEVEL_INCREMENT = 0xa4,
		CUTOFF = 0xc0, FEEDBACK = 0xc1, LEVEL = 0xc2, PAIR_BOOST = 0xc3, FILTER_TYPE = 0xc4,
		SEND_BASE = 0xf0
	};

	// the interrupt reasons a voice raises, each with its own voice-number word at IRQ_VOICE + reason
	enum irq_reason
	{
		IRQ_FINISHED = 0, IRQ_PITCH_LANDED = 1, IRQ_CUTOFF_LANDED = 2, IRQ_FEEDBACK_LANDED = 3,
		IRQ_LEVEL_LANDED = 4, IRQ_SEND_A_LANDED = 6, IRQ_SEND_B_LANDED = 7, IRQ_VOICE_MARKER = 8
	};

	enum ramp_kind { RAMP_CUTOFF, RAMP_FEEDBACK, RAMP_LEVEL, RAMP_PITCH, RAMP_SEND_A, RAMP_SEND_B, RAMPS };

	// word 0xc4 bits 3:0; a type at or above FILTER_TYPES passes the input through
	enum filter_type
	{
		FILTER_LPF = 0, FILTER_BPF = 1, FILTER_HPF = 2, FILTER_PKG = 3, FILTER_NOTCH = 4,
		FILTER_LOW_SHELF = 5, FILTER_PEAK = 6, FILTER_HIGH_SHELF = 7,
		FILTER_HIGH_POLE = 8, FILTER_LOW_POLE = 9, FILTER_TYPES = 10
	};

	// word 0x60 bits 13:12
	enum sample_format { FORMAT_WIDE = 0, FORMAT_DPCM = 1 };

	// word 0x60 bits 11:10
	enum loop_mode { LOOP_NONE = 0, LOOP_FORWARD = 1, LOOP_ALTERNATE = 2 };

	// word 0x60 bits 9:8: when the voice reports itself finished
	enum end_condition { END_NEVER = 0, END_INSIDE_LOOP = 2, END_AT_END = 3 };

	// where a voice's last fetched sample lies: before either point, at the end, or from the loop start on
	enum region { REGION_END = 1, REGION_LOOP = 0, REGION_BEFORE = 2 };

	// the spaces behind word 0x06, by their top nibble
	enum host_space { SPACE_IRAM = 0x0000, SPACE_PRAM = 0x1000, SPACE_IORAM = 0x2000, SPACE_RECORDS = 0x3000 };

	// the DSP's ten-bit data addresses: IRAM and the ERAM staging cells slide under the
	// cursor, the mix cells and transport slots stay put, the parameter bank is read-only
	enum ibus
	{
		IBUS_IRAM_END = 0x280, IBUS_MIX = 0x280, IBUS_TRANSPORT = 0x2c0, IBUS_STAGING = 0x300,
		IBUS_BANK = 0x380, IBUS_BANK_END = 0x3c0,
		LINK_OUT_L = 0x2c0, LINK_OUT_R = 0x2d0, LINK_IN_L = 0x2e0, LINK_IN_R = 0x2f0, LINK_WORDS_L = 9, LINK_WORDS_R = 8,
		DAC_L = 0x2cc, DAC_R = 0x2dc
	};

	// the memory operation a row's W1 (or a second one in its W2) carries
	enum memory_mode { MEM_NONE = 0, MEM_STORE_P = 1, MEM_STORE_A = 2, MEM_STORE_B = 3, MEM_READ_R = 4, MEM_READ_S = 5, MEM_READ_M = 6, MEM_READ_C = 7 };

	// the ALU's left operand, W0 bits 10:8, and right operand, bits 6:4
	enum alu_left { LEFT_ZERO = 0, LEFT_R = 1, LEFT_S = 2, LEFT_M = 3, LEFT_A = 4, LEFT_B = 5, LEFT_MAG_A = 6, LEFT_MAG_B = 7 };
	enum alu_right { RIGHT_ZERO = 0, RIGHT_A = 1, RIGHT_B = 2, RIGHT_R = 3, RIGHT_K23 = 4, RIGHT_K19 = 5, RIGHT_K15 = 6, RIGHT_P = 7 };

	// the multiplier's operand, W0 bits 3:1
	enum multiplier_source { SOURCE_R = 0, SOURCE_S = 1, SOURCE_M = 2, SOURCE_C = 3, SOURCE_A = 4, SOURCE_B = 5, SOURCE_P = 6, SOURCE_UNKNOWN = 7 };

	roland_xv_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	auto int_callback() { return m_int_callback.bind(); }

	// the chip whose transport block this one's is linked to; the linked chip is run from this one's stream
	template <typename T> void set_link(T &&tag) { m_link.set_tag(std::forward<T>(tag)); }

	// the 512 byte window, byte wide
	u8 read(offs_t offset);
	void write(offs_t offset, u8 data);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// device_memory_interface implementation
	virtual space_config_vector memory_space_config() const override;

	// device_sound_interface implementation
	virtual void sound_stream_update(sound_stream &stream) override;

	virtual void device_post_load() override;

	// what a voice keeps beside its register file
	struct voice
	{
		u32 address = 0;
		u16 phase = 0;
		s32 predictor = 0;
		bool backward = false;
		bool launch = false;
		bool fetching = false;
		bool was_running = false;
		u8 region = REGION_BEFORE;
		bool finished = false;
		bool scaled = false;
		s32 filter_low = 0;
		s32 filter_band = 0;
		s32 ramp_current[RAMPS] = { 0 };
		s32 ramp_target[RAMPS] = { 0 };
		s32 ramp_position[RAMPS] = { 0 };
		s32 ramp_step[RAMPS] = { 0 };
		u16 ramp_remaining[RAMPS] = { 0 };
		s32 ramp_fade[RAMPS] = { 0 };
		bool ramp_armed[RAMPS] = { false };
		u8 send_slot[2] = { 0, 0 };
	};

	struct wave_cell
	{
		s32 mantissa;
		int exponent;
	};

	struct address_step
	{
		u32 address;
		bool backward;
		bool wrapped;
	};

	// a program row, its three words and their fields decoded
	struct dsp_row
	{
		u16 w0 = 0;
		u16 w1 = 0;
		u16 operand = 0;
		bool conditional = false;
		bool branch = false;
		u8 condition = 0;
		s8 displacement = 0;
		bool multiply = false;
		u8 shift = 0;
		u8 mode = 0;
		u16 address = 0;
		bool second = false;
		u8 mode2 = 0;
		u16 address2 = 0;
		bool cell_coefficient = false;
		u8 source = 0;
		u8 left = 0;
		u8 right = 0;
		bool negate_left = false;
		bool negate_right = false;
		bool to_b = false;
		bool wrap = false;
		bool hold = true;
		bool clamp = false;
		s32 immediate = 0;
	};

	struct transfer
	{
		bool write;
		u16 cell;
		u32 offset;
	};

	struct tap_request
	{
		u16 cell;
		s32 delay;
	};

	u16 object_word(int voice, int word) const { return m_object_regs[voice][word - OBJECT_BASE]; }
	u32 object_long(int voice, int word) const { return (u32(object_word(voice, word)) << 16) | object_word(voice, word + 1); }
	bool running(int voice) const { return BIT(m_run_mask, voice); }

	void run_dsp();
	void exchange();

	optional_device<roland_xv_device> m_link;
	s32 m_bus[IBUS_STAGING - IBUS_MIX];

private:
	u16 word_peek(int word);
	void word_taken(int word);
	void word_w(int word, u16 data);
	void object_w(int voice, int word, u16 data);
	void fifo_rewind();
	void fifo_push(u16 data);
	void transfer_read();
	void transfer_write();
	void update_irq();
	void raise_irq(int reason, int voice);
	void run_mask_w(int word, u16 data);

	void start_ramp(int voice, int kind, u32 value);
	void seed_ramp(int voice, int kind, s32 value);
	void set_current(int voice, int kind, s32 value);
	void send_port_w(int voice, int kind, u32 value);
	void increment_ramp(int voice, int kind, u32 value);
	u16 steps_to_target(int voice, int kind) const;
	void service_ramp(int n, int kind);

	static u32 cell_of(u32 sample, bool wide);
	static u32 in_page(u32 address, u32 index) { return (address & ~PAGE_MASK) | (index & PAGE_MASK); }
	u8 sample_byte(u32 sample);
	wave_cell cell_at(u32 address, bool wide);
	static s32 delta_of(wave_cell c);
	static s32 tap(s32 weight, wave_cell c);
	void launch(int n);
	u32 loop_fraction(int n, bool at_loop) const;
	address_step advance(int n, address_step s, u32 phase) const;
	void cross(int n, u32 address);
	s32 filter(int n, s32 sample);
	void run_voice(int n, s32 *buses);

	int object() const { return m_regs[MODE] & (OBJECTS - 1); }

	void sync();
	void frame();
	static u16 next_address(u16 address);
	void space_w(u16 address, u32 data);
	void decode_row(int n);
	void decode_transfers();
	u32 ring_index(u16 address) const { return (m_cursor + address) & (RING_CELLS - 1); }
	u32 eram_index(s32 offset) const { return (m_cursor + offset) & (ERAM_CELLS - 1); }
	s32 cell_r(u16 address) const;
	void cell_w(u16 address, s32 value);
	void execute(const dsp_row &row, bool commit);
	bool condition(int code);
	void log_once(int what, const char *text);

	static s32 clamp24(s64 value) { return s32(std::clamp<s64>(value, -0x800000, 0x7fffff)); }
	static s32 wrap20(s32 value) { return s32(u32(value) << 12) >> 12; }
	static s32 wrap18(s32 value) { return s32(u32(value) << 14) >> 14; }
	static s32 wrap16(s32 value) { return s16(value); }
	static s32 wrap24(s32 value) { return s32(u32(value) << 8) >> 8; }

	address_space_config m_wave_config;
	memory_access<32, 1, -1, ENDIANNESS_LITTLE>::specific m_wave;
	devcb_write_line m_int_callback;
	sound_stream *m_stream;

	u16 m_regs[0x100];
	u16 m_object_regs[OBJECTS][OBJECT_END - OBJECT_BASE];
	std::unique_ptr<u32[]> m_space;
	u16 m_address;
	u16 m_data_high;
	u16 m_fifo[FIFO_DEPTH];
	int m_fifo_write;
	int m_fifo_read;
	u16 m_irq_enable;
	u16 m_irq_pending;
	u8 m_irq_voice[IRQ_REASONS];
	u64 m_irq_waiting[IRQ_REASONS];
	bool m_int_state;
	u64 m_run_mask;
	voice m_voices[OBJECTS];

	roland_xv_device *m_master;
	dsp_row m_rows[DSP_ROWS];
	int m_rows_end;
	std::unique_ptr<s32[]> m_eram;
	s32 m_bank[IBUS_BANK_END - IBUS_BANK];
	u32 m_cursor;
	s32 m_acc[2];
	s32 m_product;
	s32 m_latch[4];
	s32 m_flag_value;
	s64 m_flag_raw;
	s32 m_last_value;
	s64 m_last_raw;
	bool m_last_hold;
	transfer m_transfers[RECORDS];
	int m_transfer_count;
	bool m_transfers_stale;
	tap_request m_taps[TAPS];
	int m_tap_count;
	u32 m_logged;
};

DECLARE_DEVICE_TYPE(ROLAND_XV, roland_xv_device)

#endif // MAME_SOUND_ROLAND_XV_H
