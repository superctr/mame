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

	// the host port's data register, for DMA
	u16 host_read();
	void host_write(u16 data, u16 mem_mask);

	// the recording input, a 24-bit word per channel, left (offset 0) then right
	void record_w(offs_t offset, u32 data);

	// the recording input as stream inputs instead, run one sample at a time: serial input 1 as
	// inputs 0 and 1, serial input 2 as 2 and 3, the first of each pair for voice 4
	void set_record_inputs() { m_record_inputs = true; }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_clock_changed() override;

	virtual void sound_stream_update(sound_stream &stream) override;

private:
	enum : int { POS_FRACTION = 14, EVENTS = 64 };

	static constexpr int RECORD_VOICES[2] = { 4, 20 };

	// C2's top bits and the END register's mode byte
	enum : u32 {
		C2_NO_LOOP_EVENT = 1U << 31, C2_NO_END_EVENT = 1U << 30, C2_BACKWARD = 1U << 27,
		MODE_REVERSE = 1U << 31, MODE_ALTERNATE = 1U << 30, MODE_ONE_SHOT = 1U << 29
	};

	// event codes as C002 reads them: the voice, and bit 5 for the end event
	enum : u8 { EVENT_LOOP = 0x00, EVENT_END = 0x20 };

	struct voice
	{
		u16 c4[8] = { 0 };
		u16 c6[8] = { 0 };
		s64 pos = 0;
		u32 flags = 0;
		bool on = false;
		bool looped = false;
		bool ended = false;
	};

	u16 wave_word(s64 adr) const { u16 const *const b = m_blocks[(adr >> 16) & 0xff]; return b ? b[adr & 0xffff] : 0xffff; }
	u16 *wave_ptr(s64 adr) const { u16 *const b = m_blocks[(adr >> 16) & 0xff]; return b ? &b[adr & 0xffff] : nullptr; }
	u32 c4_long(const voice &v, int word) const { return u32(v.c4[word + 1]) << 16 | v.c4[word]; }
	u32 c2_read(const voice &v) const;
	void c2_write(voice &v, u32 data);
	void key_w(int base, u16 data);
	void event(int voice, u8 kind);
	s32 step(int n, voice &v);
	void advance(int n, voice &v);
	bool recording(int n) const;
	void record();
	TIMER_CALLBACK_MEMBER(update_int);

	devcb_write_line m_int_cb;
	sound_stream *m_stream;
	emu_timer *m_int_timer;
	u16 *m_blocks[WAVE_BLOCKS];

	voice m_voices[VOICES];
	u16 m_key[2];
	u16 m_ctrl[16];
	u8 m_sel_block;
	u8 m_sel_voice;
	u8 m_sel_word;
	u32 m_c2_latch;

	u8 m_events[EVENTS];
	u8 m_event_head;
	u8 m_event_count;
	bool m_int_state;

	u32 m_host_addr;
	u16 m_host_latch;

	bool m_record_inputs;
	s32 m_record_in[2];
	u8 m_level_channel;
};

DECLARE_DEVICE_TYPE(ROLAND_SP, roland_sp_device)

#endif // MAME_SOUND_ROLAND_SP_H
