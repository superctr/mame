// license:BSD-3-Clause
// copyright-holders:superctr
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

	// the stream output carrying a port's latch, latches counted in step order from the start of the sample
	static constexpr int output(int port, int latch) { return port * LATCHES + latch; }

	roland_meq_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	// the steps at which each voice input is the multiplicand
	template <unsigned N> void set_voice_slots(const u16 (&slots)[VOICES][N]) { m_slot_table = &slots[0][0]; m_slots_per_voice = N; }

	// a port whose latches are told apart by their steps, as a demultiplexer timed to the program
	// takes them: output(port, n) carries the word latched at steps[n], zero in a sample without it
	void set_latch_steps(int port, const u16 (&steps)[LATCHES]) { m_latch_steps[port] = steps; }

	// the third serial port's words as they are latched, the latch's index in the sample as the offset
	auto port3_callback() { return m_port3_cb.bind(); }

	// one sample at a time, for a machine whose CPU waits on the port 3 callback's effects
	void set_synchronous(bool synchronous) { m_synchronous = synchronous; }

	// a serial input that delivers the right word first
	void set_serial_right_first(int port, bool right_first) { m_serial_right_first[port] = right_first; }

	// the register file, a 16-bit word at a time
	u16 read(offs_t offset);
	void write(offs_t offset, u16 data);

	// the register file on an 8-bit bus: each word as two bytes, most significant first
	u8 read8(offs_t offset);
	void write8(offs_t offset, u8 data);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_clock_changed() override;

	virtual void sound_stream_update(sound_stream &stream) override;

private:
	enum : u32 { DATA_WORDS = 128 };

	static offs_t byte_register(offs_t offset);
	s32 multiply(s32 x, u16 coef) const;
	void run(const s32 *voices, const s32 (*serial)[2]);

	devcb_write32 m_port3_cb;
	bool m_synchronous;
	bool m_serial_right_first[SERIAL_INPUTS];
	sound_stream *m_stream;
	const u16 *m_slot_table;
	unsigned m_slots_per_voice;
	s8 m_slot_of_step[STEPS];
	const u16 *m_latch_steps[PORTS];
	s8 m_latch_of_step[PORTS][STEPS];

	u32 m_program[STEPS];
	u16 m_coef[STEPS];
	s32 m_data[DATA_WORDS];
	u32 m_latch;
	u16 m_config[2];
	u8 m_byte_hi[8];

	s32 m_operand_l;
	s32 m_operand_p;
	s32 m_r1;
	s32 m_r2;
	s32 m_pipe[2];
	s32 m_prev_read;
	s32 m_pending_coef;
	s32 m_port[PORTS][LATCHES];
};

DECLARE_DEVICE_TYPE(ROLAND_MEQ, roland_meq_device)

#endif // MAME_SOUND_ROLAND_MEQ_H
