// license:BSD-3-Clause
// copyright-holders:superctr
#ifndef MAME_SOUND_ROLAND_SP_H
#define MAME_SOUND_ROLAND_SP_H

#pragma once

class roland_sp_device : public device_t, public device_sound_interface
{
public:
	static constexpr feature_type imperfect_features() { return feature::SOUND; }

	static constexpr int VOICES = 32;

	roland_sp_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	auto int_callback() { return m_int_cb.bind(); }

	static constexpr unsigned WAVE_BLOCKS = 0x100, BLOCK_WORDS = 0x10000;

	// the wave memory the board decodes, in blocks of 64K words; a block without memory reads open bus
	void set_wave_block(unsigned block, u16 *words);

	u16 read(offs_t offset, u16 mem_mask);
	void write(offs_t offset, u16 data, u16 mem_mask);

	// the wave memory port for the board's DMA: a read returns the fetched word and fetches the
	// next, a write stores and then advances
	u16 wave_data_r();
	void wave_data_w(u16 data, u16 mem_mask);

	// the recording input, a 24-bit word per channel: offset 0 the frame's first word (voice 20),
	// 1 the second (voice 4), in either order
	void record_w(offs_t offset, u32 data);

	// serial input 1 as stream inputs 0 and 1 (the first for voice 20), run one sample at a time;
	// record_w is then serial input 2, taken while C004 bit 1 selects it
	void set_record_inputs() { m_record_inputs = true; }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_clock_changed() override;

	virtual void sound_stream_update(sound_stream &stream) override;

private:
	enum : int { POS_FRACTION = 14, EVENTS = 64 };

	static constexpr int RECORD_VOICES[2] = { 20, 4 };

	// the state word's flags, and the mode byte on top of END
	enum : u32
	{
		STATE_NO_LOOP_EVENT = 1U << 31, STATE_NO_END_EVENT = 1U << 30, STATE_BACKWARD = 1U << 27,
		STATE_FLAGS = ~0x07ffffffU,
		MODE_REVERSE = 1U << 31, MODE_ALTERNATE = 1U << 30,
		END_ADDRESS = 0x00ffffff
	};

	// event codes as C002 reads them: the voice, and bit 5 for the end event
	enum : u8 { EVENT_LOOP = 0x00, EVENT_END = 0x20 };

	// the per-voice blocks the select addresses C200, C400 and C600 open
	enum : u8 { BLOCK_NONE = 0, BLOCK_STATE, BLOCK_PLAYBACK, BLOCK_LEVEL };

	struct voice
	{
		u32 end = 0;
		u16 pitch = 0;
		u32 loop_start = 0;
		u32 loop_end = 0;
		u32 level = 0;
		s64 pos = 0;
		u32 state = 0;
		bool on = false;
		bool looped = false;
		bool ended = false;
	};

	u16 wave_word(s64 adr) const { u16 const *const b = m_blocks[(adr >> 16) & 0xff]; return b ? b[adr & 0xffff] : 0xffff; }
	u16 *wave_ptr(s64 adr) const { u16 *const b = m_blocks[(adr >> 16) & 0xff]; return b ? &b[adr & 0xffff] : nullptr; }
	u32 state_r(const voice &v) const;
	void state_w(voice &v, u32 data);
	u16 playback_r(const voice &v, int word) const;
	void playback_w(voice &v, int word, u16 data);
	u16 level_r(const voice &v, int word) const;
	void level_w(voice &v, int word, u16 data);
	void key_w(int base, u16 data);
	void event(int voice, u8 kind);
	void wave_store(u16 data, u16 mem_mask);
	s32 step(int n, voice &v);
	void advance(int n, voice &v);
	bool recording(int n) const;
	s16 filtered(int c);
	void record();
	TIMER_CALLBACK_MEMBER(update_int);

	devcb_write_line m_int_cb;
	sound_stream *m_stream;
	emu_timer *m_int_timer;
	u16 *m_blocks[WAVE_BLOCKS];

	voice m_voices[VOICES];
	u16 m_key[2];
	u16 m_regs[16];
	u8 m_sel_block;
	u8 m_sel_voice;
	u8 m_sel_word;
	u32 m_state_port;

	u8 m_events[EVENTS];
	u8 m_event_head;
	u8 m_event_count;
	bool m_int_state;

	u32 m_wave_addr;
	u16 m_wave_latch;

	bool m_record_inputs;
	s32 m_record_in[2];
	u8 m_record_got;
	s16 m_record_history[2][5];
	u8 m_level_channel;
};

DECLARE_DEVICE_TYPE(ROLAND_SP, roland_sp_device)

#endif // MAME_SOUND_ROLAND_SP_H
