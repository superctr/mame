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

	void set_wave_size(u32 bytes) { m_wave_size = bytes; }

	u16 read(offs_t offset, u16 mem_mask);
	void write(offs_t offset, u16 data, u16 mem_mask);

	// the host port's bank, from the I/O gate array; the on-board memory is bank 1
	void host_bank_w(u8 bank) { m_host_bank = bank; }

	// the host port's data register, for DMA
	u16 host_read();
	void host_write(u16 data, u16 mem_mask);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	virtual void sound_stream_update(sound_stream &stream) override;

private:
	enum : int { POS_FRACTION = 14 };

	struct voice
	{
		u16 c4[8] = { 0 };
		u16 c6[8] = { 0 };
		u64 pos = 0;
		u8 flags = 0;
		bool on = false;
	};

	u16 wave_word(u64 adr) const { return m_wave_ptr[adr & (m_wave_words - 1)]; }
	u32 c2_read(const voice &v) const;
	void c2_write(voice &v, u32 data);
	void key_w(int base, u16 data);
	s32 step(voice &v);

	sound_stream *m_stream;
	u32 m_wave_size;
	u32 m_wave_words;
	u16 *m_wave_ptr;

	voice m_voices[VOICES];
	u16 m_key[2];
	u16 m_ctrl[16];
	u8 m_sel_block;
	u8 m_sel_voice;
	u8 m_sel_word;
	u32 m_c2_latch;

	u8 m_host_bank;
	u32 m_host_addr;
	u16 m_host_latch;
};

DECLARE_DEVICE_TYPE(ROLAND_SP, roland_sp_device)

#endif // MAME_SOUND_ROLAND_SP_H
