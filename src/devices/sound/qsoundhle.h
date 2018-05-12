// license:BSD-3-Clause
// copyright-holders:superctr, Valley Bell
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
	tiny_rom_entry const *device_rom_region() const override;
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_sound_interface implementation
	virtual void sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples) override;

	// device_rom_interface implementation
	virtual void rom_bank_updated() override;

private:

	struct qsound_voice {
		uint16_t bank;
		int16_t addr; // top word is the sample address
		uint16_t phase;
		uint16_t rate;
		int16_t loop_len;
		int16_t end_addr;
		int16_t volume;
		int16_t echo;
	};

	struct qsound_adpcm {
		uint16_t start_addr;
		uint16_t end_addr;
		uint16_t bank;
		int16_t volume;
		uint16_t flag;
		int16_t cur_vol;
		int16_t signal;
		int16_t cur_addr;
		int16_t step;
	};

	// Q1 Filter
	struct qsound_fir {
		int tap_count;	// usually 95
		int delay_pos;
		int16_t table_pos;
		int16_t taps[95];
		int16_t delay_line[95];
	};

	// Delay line
	struct qsound_delay {
		int16_t delay;
		int16_t volume;
		int16_t write_pos;
		int16_t read_pos;
		int16_t delay_line[51];
	};

	struct qsound_echo {
		uint16_t end_pos;
		
		int16_t feedback;
		int16_t length;
		int16_t last_sample;
		int16_t delay_line[1024];
		int16_t delay_pos;
	};

	// MAME resources
	sound_stream *m_stream;

	uint16_t m_data_latch;
	int16_t m_out[2];

	int16_t m_pan_tables[2][2][98];
	int16_t m_adpcm_shift[24];
	uint16_t m_filter_lut_mode1[5][95];
	uint16_t m_filter_lut_mode2[95];
	
	qsound_voice m_voice[16];
	qsound_adpcm m_adpcm[3];

	uint16_t m_voice_pan[16+3];
	int16_t m_voice_output[16+3];

	qsound_echo m_echo;

	qsound_fir m_filter[2];
	qsound_fir m_alt_filter[2];
	
	qsound_delay m_wet[2];
	qsound_delay m_dry[2];
	
	uint16_t m_state;
	uint16_t m_next_state;
	
	uint16_t m_delay_update;
	
	int m_state_counter;
	int m_ready_flag;
	
	uint16_t *register_map[256];

	void write_data(uint8_t addr, uint16_t data);
	uint16_t read_data(uint8_t addr);

	void init_register_map();
	void update_sample();

	// DSP states
	void state_init();
	void state_refresh_filter_1();
	void state_refresh_filter_2();
	void state_normal_update();

	// sub functions
	int16_t get_sample(uint16_t bank,uint16_t address);
	int16_t* get_filter_table_1(uint16_t offset);
	int16_t* get_filter_table_2(uint16_t offset);

	int16_t voice_update(struct qsound_voice *v, int32_t *echo_out );
	int16_t echo(struct qsound_echo *r,int32_t input);
	void main_update();
	int32_t fir(struct qsound_fir *f, int16_t input);
	int32_t delay(struct qsound_delay *q, int32_t input);
	void delay_update(struct qsound_delay *q);

};

DECLARE_DEVICE_TYPE(QSOUND_HLE, qsound_hle_device)

#endif // MAME_SOUND_QSOUNDHLE_H
