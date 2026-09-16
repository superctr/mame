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
	static constexpr int BUSES = 32;
	static constexpr int BUS_CHORUS = 6;
	static constexpr int BUS_REVERB = 7;
	static constexpr int SENDS = 6;
	static constexpr u32 SAMPLE_RATE = 44100;
	static constexpr int RAMP_FRACTION_BITS = 12;
	static constexpr int OUTPUT_BITS = 18;

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
		CUTOFF_RAMP = 0x90, FEEDBACK_RAMP = 0x92, LEVEL_RAMP = 0x94, BLOCK_CONTROL = 0x96,
		SEND_PORT_A = 0x98, SEND_PORT_B = 0x9a, PITCH_RAMP = 0x9c,
		CUTOFF_INCREMENT = 0xa0, FEEDBACK_INCREMENT = 0xa2, LEVEL_INCREMENT = 0xa4,
		CUTOFF = 0xc0, FEEDBACK = 0xc1, LEVEL = 0xc2, PAIR_BOOST = 0xc3, FILTER_TYPE = 0xc4,
		SEND_BASE = 0xf0
	};

	// the interrupt reasons a voice raises, each with its own voice-number word at IRQ_VOICE + reason
	enum irq_reason
	{
		IRQ_ONE_SHOT_END = 0, IRQ_PITCH_LANDED = 1, IRQ_CUTOFF_LANDED = 2, IRQ_LEVEL_LANDED = 4, IRQ_VOICE_MARKER = 8
	};

	enum ramp_kind { RAMP_CUTOFF, RAMP_FEEDBACK, RAMP_LEVEL, RAMP_PITCH, RAMPS };

	enum filter_type { FILTER_LPF = 0, FILTER_BPF = 1, FILTER_HPF = 2, FILTER_PKG = 3, FILTER_OFF = 7 };

	roland_xv_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	auto int_callback() { return m_int_callback.bind(); }

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
		s32 filter_low = 0;
		s32 filter_band = 0;
		s32 ramp_current[RAMPS] = { 0 };
		s32 ramp_target[RAMPS] = { 0 };
		s32 ramp_position[RAMPS] = { 0 };
		s32 ramp_step[RAMPS] = { 0 };
		u16 ramp_remaining[RAMPS] = { 0 };
		bool ramp_armed[RAMPS] = { false };
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

	u16 object_word(int voice, int word) const { return m_object_regs[voice][word - OBJECT_BASE]; }
	u32 object_long(int voice, int word) const { return (u32(object_word(voice, word)) << 16) | object_word(voice, word + 1); }
	bool running(int voice) const { return BIT(m_run_mask, voice); }

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
	void increment_ramp(int voice, int kind, u32 value);
	u16 steps_to_target(int voice, int kind) const;
	void service_ramp(int n, int kind);

	u8 sample_byte(u32 sample);
	wave_cell cell_at(u32 address);
	static s32 delta_of(wave_cell c);
	static s32 tap(s32 weight, wave_cell c);
	void launch(int n);
	u32 loop_fraction(int n, bool at_loop) const;
	address_step advance(int n, address_step s, u32 phase) const;
	s32 filter(int n, s32 sample);
	void run_voice(int n, s32 *buses);

	int object() const { return m_regs[MODE] & (OBJECTS - 1); }

	static s32 clamp24(s64 value) { return s32(std::clamp<s64>(value, -0x800000, 0x7fffff)); }
	static s32 wrap20(s32 value) { return s32(u32(value) << 12) >> 12; }
	static s32 wrap18(s32 value) { return s32(u32(value) << 14) >> 14; }

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
};

DECLARE_DEVICE_TYPE(ROLAND_XV, roland_xv_device)

#endif // MAME_SOUND_ROLAND_XV_H
