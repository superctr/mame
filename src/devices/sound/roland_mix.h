// license:BSD-3-Clause
// copyright-holders:superctr
#ifndef MAME_SOUND_ROLAND_MIX_H
#define MAME_SOUND_ROLAND_MIX_H

#pragma once

class roland_mix_device : public device_t, public device_sound_interface
{
public:
	static constexpr feature_type imperfect_features() { return feature::SOUND; }

	static constexpr int VOICES = 32;
	static constexpr int OUTPUTS = 8;

	// the outputs in the D/A's order: the stereo pair, then the six individual outputs; then the
	// serial output's pair
	enum : int { OUTPUT_LEFT = 0, OUTPUT_RIGHT = 1, OUTPUT_INDIVIDUAL = 2, OUTPUT_SERIAL = OUTPUTS };

	// the serial input's pair, after the voices
	static constexpr int INPUT_SERIAL = VOICES;

	roland_mix_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	// the serial output's words, 24 bits, left (offset 0) then right
	auto serial_callback() { return m_serial_cb.bind(); }

	u16 read(offs_t offset);
	void write(offs_t offset, u16 data);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_clock_changed() override;

	virtual void sound_stream_update(sound_stream &stream) override;

private:
	enum : offs_t { REG_CONTROL = 0x00, REG_DATA = 0x03, REG_SELECT = 0x20, WORDS = 0x40 };

	devcb_write32 m_serial_cb;
	sound_stream *m_stream;
	u16 m_regs[WORDS];
	u16 m_voice[VOICES];
	u8 m_select;
};

DECLARE_DEVICE_TYPE(ROLAND_MIX, roland_mix_device)

#endif // MAME_SOUND_ROLAND_MIX_H
