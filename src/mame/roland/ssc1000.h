// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Roland SSC-1000 key scanner

***************************************************************************/

#ifndef MAME_ROLAND_SSC1000_H
#define MAME_ROLAND_SSC1000_H

#pragma once

class ssc1000_device : public device_t
{
public:
	ssc1000_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	auto int_callback() { return m_int_callback.bind(); }

	u8 read(offs_t offset);
	void write(offs_t offset, u8 data);

	void key_w(u8 key, u8 velocity);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	static constexpr unsigned QUEUE = 64;

	TIMER_CALLBACK_MEMBER(int_raise);
	void update_int();

	devcb_write_line m_int_callback;
	emu_timer *m_int_timer;

	u8 m_table[4][0x80];
	u8 m_setup[6];
	u8 m_control;
	u8 m_queue_key[QUEUE];
	u8 m_queue_vel[QUEUE];
	u8 m_head;
	u8 m_count;
	bool m_int;
};

DECLARE_DEVICE_TYPE(SSC1000, ssc1000_device)

#endif // MAME_ROLAND_SSC1000_H
