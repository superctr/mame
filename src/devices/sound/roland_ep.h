// license:BSD-3-Clause
// copyright-holders:superctr
#ifndef MAME_SOUND_ROLAND_EP_H
#define MAME_SOUND_ROLAND_EP_H

#pragma once

class roland_ep_device : public device_t, public device_memory_interface, public device_sound_interface
{
public:
	static constexpr feature_type imperfect_features() { return feature::SOUND; }

	// the wave ROMs, one byte a sample, 24 bits of address: the internal module,
	// the card and the expansion board at the bases the firmware probes
	enum { AS_WAVE = 0 };

	static constexpr int VOICES = 32;
	static constexpr u32 SAMPLE_RATE = 44100;
	static constexpr u32 ADDRESS_MASK = 0x00ffffff;
	static constexpr u32 PAGE_MASK = 0x000fffff;
	static constexpr int SAMPLE_BITS = 17;

	// the 64 words of the window
	enum register_word
	{
		CONFIG = 0x00, VOICES_IN_USE = 0x01, READ_DATA = 0x02,
		STOP_HIGH = 0x06, STOP_LOW = 0x07, END_HIGH = 0x08, END_LOW = 0x09, LOOP_HIGH = 0x0a, LOOP_LOW = 0x0b,
		START_HIGH = 0x0c, START_LOW = 0x0d, KEY_LOW = 0x0e, KEY_HIGH = 0x0f,
		PITCH = 0x17, GATE = 0x19, WAVE = 0x1d, READ_LOW = 0x1e, READ_HIGH = 0x1f,
		RESET = 0x2f, SELECT = 0x3e,
		WORDS = 0x40
	};

	roland_ep_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	// the window, a word at a time
	u16 read(offs_t offset);
	void write(offs_t offset, u16 data);

	// a voice's pitch word from a second bus master, the select left alone
	void voice_pitch_w(int voice, u16 data);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// device_memory_interface implementation
	virtual space_config_vector memory_space_config() const override;

	// device_sound_interface implementation
	virtual void sound_stream_update(sound_stream &stream) override;

private:
	struct voice
	{
		u16 regs[WORDS] = { 0 };
		u32 address = 0;
		u16 phase = 0;
		s32 predictor = 0;
		bool running = false;
	};

	struct wave_cell
	{
		s32 mantissa;
		int exponent;
	};

	u32 address_of(const voice &v, int high) const;
	u32 loop_of(const voice &v) const { return address_of(v, LOOP_HIGH); }
	u32 end_of(const voice &v) const { return address_of(v, END_HIGH); }
	u32 next(const voice &v, u32 address) const;
	wave_cell cell_at(u32 address);
	static s32 tap(s32 weight, wave_cell c);
	void key_w(int word, u16 data);
	void launch(int n);
	s32 run_voice(int n);

	static s32 wrap20(s32 value) { return s32(u32(value) << 12) >> 12; }

	address_space_config m_wave_config;
	memory_access<24, 0, 0, ENDIANNESS_LITTLE>::specific m_wave;
	sound_stream *m_stream;

	u16 m_regs[WORDS];
	voice m_voices[VOICES];
	u32 m_key_mask;
};

DECLARE_DEVICE_TYPE(ROLAND_EP, roland_ep_device)

#endif // MAME_SOUND_ROLAND_EP_H
