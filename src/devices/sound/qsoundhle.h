// license:BSD-3-Clause
// copyright-holders:Valley Bell
/*********************************************************

    QSound DL-1425 (HLE)

*********************************************************/
#ifndef MAME_SOUND_QSOUNDHLE_H
#define MAME_SOUND_QSOUNDHLE_H

#pragma once

#include "cpu/dsp16/dsp16.h"


class qsound_hle_device : public device_t, public device_sound_interface, public device_rom_interface
{
public:
	// default 60MHz clock (divided by 2 for DSP core clock, and then by 1248 for sample rate)
	qsound_hle_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock = 60'000'000);

	DECLARE_WRITE8_MEMBER(qsound_w);
	DECLARE_READ8_MEMBER(qsound_r);

protected:
	// device_t implementation
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_sound_interface implementation
	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;

	// device_rom_interface implementation
	virtual void rom_bank_updated() override;

private:

	// MAME resources
	sound_stream *m_stream;

	uint16_t m_ram[0x800];	// DSP RAM, 2048 words, 16-bit each
	int16_t m_testInc;	// test mode increment (Y register)
	int16_t m_testOut;	// test mode output value (A0 register)
	uint16_t m_dataLatch;
	uint8_t m_busyState;
	int16_t m_out[2];
	
	uint16_t m_dspRoutine;	// offset of currently running update routine
	uint8_t m_dspRtStep;	// each routine outputs 6 samples before restarting, this indicates the current sample
	int m_updateFunc;

	inline int16_t read_sample(uint32_t offset) { return uint16_t(read_byte(offset)) << 8; }
	void write_data(uint8_t address, uint16_t data);
	
	int16_t* get_filter_table_1(uint16_t offset);
	int16_t* get_filter_table_2(uint16_t offset);
	int16_t dsp_get_sample(uint16_t bank, uint16_t ofs);
	inline void INC_MODULO(uint16_t* reg, uint16_t rb, uint16_t re);
	inline int32_t DSP_ROUND(int32_t value);
	void dsp_do_update_step();
	void dsp_update_delay();
	void dsp_update_test();	// ROM: 0018
	void dsp_copy_filter_data_1();	// ROM: 0039
	void dsp_copy_filter_data_2();	// ROM: 004F
	void dsp_init1();	// ROM: 0288
	void dsp_init2();	// ROM: 061A
	void dsp_filter_recalc(uint16_t refreshFlagAddr);	// ROM: 05DD / 099B
	void dsp_update_1();	// ROM: 0314
	void dsp_update_2();	// ROM: 06B2
	void dsp_filter_a(uint16_t i1);	// ROM: 0314/036F/03CA
	void dsp_filter_b(uint16_t i1);	// ROM: 0425/0470/04B4
	void dsp_sample_calc_1();	// ROM: 0504
	void dsp_sample_calc_2();	// ROM: 08A2
	void dsp_run_update();	// ROM: 08A2

};

DECLARE_DEVICE_TYPE(QSOUND_HLE, qsound_hle_device)

#endif // MAME_SOUND_QSOUNDHLE_H
