// license:BSD-3-Clause
// copyright-holders:superctr
// thanks-to: giulioz
#ifndef MAME_SOUND_ROLAND_SP_H
#define MAME_SOUND_ROLAND_SP_H

#pragma once

class roland_sp_device : public device_t, public device_sound_interface
{
public:
	static constexpr feature_type imperfect_features() { return feature::SOUND; }

	static constexpr int VOICES = 32;
	static constexpr u32 WAVE_BLOCKS = 0x100;
	static constexpr u32 BLOCK_WORDS = 0x1'0000;

	roland_sp_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	auto int_callback() { return m_int_cb.bind(); }
	void set_record_inputs() { m_record_inputs = true; }

	void set_wave_block(u32 block, u16 *words);

	void map(address_map &map) ATTR_COLD;

	u16 wave_data_r();
	void wave_data_w(u16 data, u16 mem_mask);

	void record_w(offs_t offset, u32 data);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_clock_changed() override;

	// device_sound_interface implementation
	virtual void sound_stream_update(sound_stream &stream) override;

private:
	enum : int { POS_FRACTION = 14, EVENTS = 64 };

	static constexpr int RECORD_VOICES[2] = { 20, 4 };

	enum : u32
	{
		STATE_NO_LOOP_EVENT = 1U << 31, STATE_NO_END_EVENT = 1U << 30, STATE_BACKWARD = 1U << 27,
		STATE_FLAGS = ~0x07ff'ffffU,
		MODE_REVERSE = 1U << 31, MODE_ALTERNATE = 1U << 30,
		END_ADDRESS = 0x00ff'ffff
	};

	enum : u8 { EVENT_LOOP = 0x00, EVENT_END = 0x20 };

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

	u16 status_r();
	template <int Base> void key_w(u16 data);
	u16 event_r();
	template <int Reg> u16 reg_r();
	template <int Reg> void reg_w(u16 data);
	template <int Reg> u16 unknown_r();
	void wave_store_w(offs_t offset, u16 data, u16 mem_mask);
	u16 wave_port_r();
	void wave_port_w(offs_t offset, u16 data, u16 mem_mask);
	void wave_addr_high_w(u16 data);
	void wave_addr_low_w(u16 data);
	u16 state_low_r();
	u16 state_high_r();
	void state_low_w(u16 data);
	void state_high_w(u16 data);
	u16 playback_r();
	void playback_w(u16 data);
	u16 ack_r();
	u16 level_r();
	void level_w(u16 data);
	u16 input_r();
	u16 alias_r(offs_t offset);
	void alias_w(offs_t offset, u16 data);
	u16 select_r(offs_t offset);
	void select_w(offs_t offset, u16 data);

	u16 wave_word(s64 adr) const { u16 const *const b = m_blocks[(adr >> 16) & 0xff]; return b ? b[adr & 0xffff] : 0xffff; }
	u16 *wave_ptr(s64 adr) const { u16 *const b = m_blocks[(adr >> 16) & 0xff]; return b ? &b[adr & 0xffff] : nullptr; }
	static u32 voice_state(const voice &v);
	static void set_voice_state(voice &v, u32 data);
	static u16 playback_word(const voice &v, int word);
	void set_playback_word(voice &v, int word, u16 data);
	static u16 level_word(const voice &v, int word);
	void set_level_word(voice &v, int word, u16 data);
	void select(offs_t offset);
	void event(int voice, u8 kind);
	void wave_store(u16 data, u16 mem_mask);
	s32 step(int n, voice &v);
	void advance(int n, voice &v);
	bool recording(int n) const;
	s16 filtered(int c);
	void record();
	TIMER_CALLBACK_MEMBER(update_int);
};

DECLARE_DEVICE_TYPE(ROLAND_SP, roland_sp_device)

#endif // MAME_SOUND_ROLAND_SP_H
