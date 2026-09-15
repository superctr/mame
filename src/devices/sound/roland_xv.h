// license:BSD-3-Clause
// copyright-holders:superctr
#ifndef MAME_SOUND_ROLAND_XV_H
#define MAME_SOUND_ROLAND_XV_H

#pragma once

class roland_xv_device : public device_t, public device_memory_interface, public device_sound_interface
{
public:
	static constexpr feature_type unemulated_features() { return feature::SOUND; }

	// the wave ROM and the sample RAM, 16 bits a cell, addressed by cell
	enum { AS_WAVE = 0 };

	static constexpr int OBJECTS = 64;
	static constexpr int FIFO_DEPTH = 256;
	static constexpr int IRQ_REASONS = 16;

	// the registers of the window, by word number
	enum register_word
	{
		MODE = 0x02, DATA_HIGH = 0x04, DATA_LOW = 0x05, ADDRESS = 0x06, FIFO = 0x08, FIFO_CONTROL = 0x09,
		IRQ_MASK = 0x0f, IRQ_ACK = 0x10, IRQ_VOICE = 0x10, STATUS = 0x1b,
		XFER_COMMAND = 0x25, WRITE_ADDRESS = 0x26, WRITE_LENGTH = 0x2a, READ_GO = 0x2d, READ_ADDRESS = 0x2e,
		READ_LENGTH = 0x32, COMMAND_STROBE = 0x36,
		OBJECT_BASE = 0x60, VOICE_COMMAND = 0x94, BLOCK_CONTROL = 0x96, OBJECT_END = 0xa0
	};

	roland_xv_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	auto int_callback() { return m_int_callback.bind(); }

	// the 512 byte window, byte wide
	u8 read(offs_t offset);
	void write(offs_t offset, u8 data);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// device_memory_interface implementation
	virtual space_config_vector memory_space_config() const override;

	// device_sound_interface implementation
	virtual void sound_stream_update(sound_stream &stream) override;

private:
	u16 word_r(int word);
	void word_w(int word, u16 data);
	void fifo_reset();
	void fifo_push(u16 data);
	u16 fifo_pop();
	void transfer_read();
	void transfer_write();
	void update_irq();

	int object() const { return m_regs[MODE] & (OBJECTS - 1); }

	address_space_config m_wave_config;
	memory_access<32, 1, -1, ENDIANNESS_BIG>::specific m_wave;
	devcb_write_line m_int_callback;
	sound_stream *m_stream;

	u16 m_regs[0x100];
	u16 m_object_regs[OBJECTS][OBJECT_END - OBJECT_BASE];
	std::unique_ptr<u32[]> m_space;
	u16 m_address;
	u16 m_data_high;
	u16 m_read_latch;
	u16 m_fifo[FIFO_DEPTH];
	int m_fifo_count;
	int m_fifo_read;
	u16 m_irq_enable;
	u16 m_irq_pending;
	u8 m_irq_voice[IRQ_REASONS];
	bool m_int_state;
};

DECLARE_DEVICE_TYPE(ROLAND_XV, roland_xv_device)

#endif // MAME_SOUND_ROLAND_XV_H
