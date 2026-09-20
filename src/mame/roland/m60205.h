// license:BSD-3-Clause
// copyright-holders:superctr
#ifndef MAME_ROLAND_M60205_H
#define MAME_ROLAND_M60205_H

#pragma once


class m60205_device : public device_t
{
public:
	m60205_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	auto int_callback() { return m_int_cb.bind(); }
	auto write_lcd_control() { return m_lcd_control_cb.bind(); }
	auto write_lcd_data() { return m_lcd_data_cb.bind(); }
	auto write_led() { return m_led_cb.bind(); }
	template <unsigned N> auto read_scan() { return m_scan_cb[N].bind(); }
	auto read_port() { return m_port_cb.bind(); }
	auto read_encoder() { return m_encoder_cb.bind(); }

	u8 read(offs_t offset);
	void write(offs_t offset, u8 data);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	static inline constexpr unsigned SCAN_LINES = 4;
	static inline constexpr unsigned FIFO_SIZE = 16;

	enum : unsigned
	{
		SOURCE_KEY = 0,
		SOURCE_ENCODER = 1,
		SOURCE_TIMER_B = 5,
		SOURCE_TIMER_A = 9
	};

	TIMER_CALLBACK_MEMBER(tick_a);
	TIMER_CALLBACK_MEMBER(tick_b);

	void raise(unsigned source);
	void update_int();
	void queue_key(u8 code);

	devcb_write_line m_int_cb;
	devcb_write8 m_lcd_control_cb;
	devcb_write8 m_lcd_data_cb;
	devcb_write8 m_led_cb;
	devcb_read8::array<SCAN_LINES> m_scan_cb;
	devcb_read8 m_port_cb;
	devcb_read8 m_encoder_cb;

	emu_timer *m_timer_a;
	emu_timer *m_timer_b;

	u16 m_pending;
	int m_int_state;
	u8 m_config[0x10];
	u8 m_led[8];
	u8 m_scan_state[SCAN_LINES];
	u8 m_port_state;
	u8 m_encoder_state;
	s16 m_encoder_delta;
	u8 m_fifo[FIFO_SIZE];
	u8 m_fifo_head;
	u8 m_fifo_tail;
	u8 m_divider;
};

DECLARE_DEVICE_TYPE(M60205, m60205_device)

#endif // MAME_ROLAND_M60205_H
