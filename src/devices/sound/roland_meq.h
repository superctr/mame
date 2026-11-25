// license:BSD-3-Clause
// copyright-holders:superctr
// thanks-to: giulioz
#ifndef MAME_SOUND_ROLAND_MEQ_H
#define MAME_SOUND_ROLAND_MEQ_H

#pragma once

class roland_meq_device : public device_t, public device_sound_interface
{
public:
	static constexpr feature_type imperfect_features() { return feature::SOUND; }

	static constexpr int VOICES = 24;
	static constexpr int SERIAL_INPUTS = 2;
	static constexpr int INPUT_SERIAL = VOICES;
	static constexpr int PORTS = 3;
	static constexpr int LATCHES = 8;
	static constexpr int STEPS = 400;

	static constexpr int output(int port, int latch) { return port * LATCHES + latch; }

	roland_meq_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	auto port3_callback() { return m_port3_cb.bind(); }

	void set_first_voice(int voice) { m_first_voice = voice; }
	void set_latch_steps(int port, const u16 (&steps)[LATCHES]);
	void set_synchronous(bool synchronous) { m_synchronous = synchronous; }
	void set_serial_right_first(int port, bool right_first) { m_serial_right_first[port] = right_first; }

	u16 read(offs_t offset);
	void write(offs_t offset, u16 data);

	u8 read8(offs_t offset);
	void write8(offs_t offset, u8 data);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_clock_changed() override;

	// device_sound_interface implementation
	virtual void sound_stream_update(sound_stream &stream) override;

private:
	enum : u32 { DATA_WORDS = 128 };

	static offs_t byte_register(offs_t offset);
	static s64 multiply(s32 x, s32 mantissa, u16 coef);
	void run(const s32 *voices, const s32 (*serial)[2]);

	devcb_write32 m_port3_cb;
	bool m_synchronous;
	bool m_serial_right_first[SERIAL_INPUTS];
	sound_stream *m_stream;
	int m_first_voice;
	bool m_latch_steps_set[PORTS];
	u16 m_latch_steps[PORTS][LATCHES];
	s8 m_latch_of_step[PORTS][STEPS];

	u32 m_program[STEPS];
	u16 m_coef[STEPS];
	s32 m_data[DATA_WORDS];
	u32 m_latch;
	u16 m_config[2];
	u8 m_byte_hi[8];

	s32 m_read_l;
	s32 m_read_q;
	s32 m_voice;
	s32 m_voice_word;
	s64 m_result;
	s32 m_stored;
	s32 m_r2;
	s32 m_r2_before;
	s32 m_port[PORTS][LATCHES];
};

DECLARE_DEVICE_TYPE(ROLAND_MEQ, roland_meq_device)

#endif // MAME_SOUND_ROLAND_MEQ_H
