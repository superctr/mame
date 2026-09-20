// license:BSD-3-Clause
// copyright-holders:superctr
#ifndef MAME_SOUND_ROLAND_TVF_H
#define MAME_SOUND_ROLAND_TVF_H

#pragma once

class roland_tvf_device : public device_t, public device_sound_interface
{
public:
	static constexpr feature_type imperfect_features() { return feature::SOUND; }

	static constexpr int VOICES = 32;
	static constexpr u32 SAMPLE_RATE = 44100;
	static constexpr int RAMP_SAMPLES = 315;
	static constexpr int SAMPLE_FRACTION_BITS = 28;
	static constexpr s32 SAMPLE_ONE = 1 << SAMPLE_FRACTION_BITS;
	static constexpr s32 FULL_SCALE = (s64(13355) * SAMPLE_ONE + 5000) / 10000;
	static constexpr int COEFFICIENT_FRACTION_BITS = 28;

	// the 64 words of the window
	enum register_word
	{
		READBACK_LOW = 0x00, READBACK_HIGH = 0x01, DAMPING = 0x04, FLAGS = 0x05,
		KEY_HIGH = 0x08, KEY_LOW = 0x09, VOICES_IN_USE = 0x0a, COMMAND = 0x0c,
		CUTOFF_INITIAL = 0x10, CUTOFF = 0x18, AMPLITUDE = 0x1a,
		SELECT = 0x20, READBACK_SELECT = 0x24,
		WORDS = 0x40
	};

	// the flag byte, word 0x05 bits 15-8
	enum filter_mode { MODE_LPF = 0, MODE_HPF = 1, MODE_BPF = 2 };
	enum structure { PAIR_INDEPENDENT = 0, PAIR_SUM_THEN_FILTERS = 1, PAIR_RING_THEN_FILTERS = 2, PAIR_FILTERS_THEN_RING = 3 };

	roland_tvf_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	// the window, a word at a time
	u16 read(offs_t offset);
	void write(offs_t offset, u16 data);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// device_sound_interface implementation
	virtual void sound_stream_update(sound_stream &stream) override;

private:
	struct ramp
	{
		s32 value = 0;
		s32 start = 0;
		s32 target = 0;
		int remaining = 0;
		int length = 0;

		void set(u16 data, int samples);
		void advance();
	};

	struct voice
	{
		u16 regs[WORDS] = { 0 };
		ramp cutoff;
		ramp amplitude;
		s32 low = 0;
		s32 band = 0;
	};

	void voice_w(int n, int word, u16 data);
	void service(voice &v);
	static s64 rounded_shift(s64 value, int bits);
	static s32 saturate(s64 value) { return s32(std::clamp(value, -s64(FULL_SCALE), s64(FULL_SCALE))); }
	s32 filter(voice &v, s64 sample) const;
	s32 amplify(const voice &v, s64 sample) const { return saturate(rounded_shift(sample * v.amplitude.value, COEFFICIENT_FRACTION_BITS)); }
	int mode_of(const voice &v) const { return (v.regs[FLAGS] >> 9) & 3; }
	int structure_of(const voice &v) const { return (v.regs[FLAGS] >> 11) & 3; }
	bool mixes_second(const voice &v) const { return BIT(v.regs[FLAGS], 8); }
	std::pair<s32, s32> pair(int n, s32 first, s32 second);

	sound_stream *m_stream;

	u16 m_regs[WORDS];
	voice m_voices[VOICES];
};

DECLARE_DEVICE_TYPE(ROLAND_TVF, roland_tvf_device)

#endif // MAME_SOUND_ROLAND_TVF_H
