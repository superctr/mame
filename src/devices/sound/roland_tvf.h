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
	static constexpr float FULL_SCALE = 1.3355f;

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
	struct voice
	{
		u16 regs[WORDS] = { 0 };
		float cutoff = 0;
		float cutoff_step = 0;
		int cutoff_remaining = 0;
		float amplitude = 0;
		float amplitude_step = 0;
		int amplitude_remaining = 0;
		float low = 0;
		float band = 0;
	};

	void voice_w(int n, int word, u16 data);
	void service(voice &v);
	float filter(voice &v, float sample) const;
	static float saturate(float x) { return std::clamp(x, -FULL_SCALE, FULL_SCALE); }
	float amplify(const voice &v, float sample) const { return saturate(sample * v.amplitude); }
	int mode_of(const voice &v) const { return (v.regs[FLAGS] >> 9) & 3; }
	int structure_of(const voice &v) const { return (v.regs[FLAGS] >> 11) & 3; }
	bool mixes_second(const voice &v) const { return BIT(v.regs[FLAGS], 8); }
	std::pair<float, float> pair(int n, float first, float second);

	sound_stream *m_stream;

	u16 m_regs[WORDS];
	voice m_voices[VOICES];
};

DECLARE_DEVICE_TYPE(ROLAND_TVF, roland_tvf_device)

#endif // MAME_SOUND_ROLAND_TVF_H
