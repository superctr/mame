// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Mitsubishi M66273FP LCD controller with 19 200-byte VRAM

***************************************************************************/

#ifndef MAME_VIDEO_M66273_H
#define MAME_VIDEO_M66273_H

#pragma once


class m66273_device : public device_t, public device_video_interface
{
public:
	static constexpr int LEVELS = 17;

	m66273_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	auto cse_callback() { return m_cse_cb.bind(); }
	auto lcdenb_callback() { return m_lcdenb_cb.bind(); }

	// MCS: the VRAM at 0000-4aff and the control registers at 5000-509e
	u8 read(offs_t offset);
	void write(offs_t offset, u8 data);

	// IOCS: the control registers at 00-9e
	u8 reg_r(offs_t offset);
	void reg_w(offs_t offset, u8 data);

	u32 screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_clock_changed() override;

private:
	static constexpr unsigned VRAM_SIZE = 0x4b00;
	static constexpr unsigned SCREEN2_BASE = 0x2580;

	enum : u8
	{
		R1_LCDE = 0x01,
		R1_REV = 0x02,
		R1_DISP = 0x04,
		R1_DIV = 0x38,
		R1_IDXON = 0x40,
		R1_RESET = 0x80,

		R2_8BIT = 0x01,
		R2_GRAY = 0x02,
		R2_DUAL = 0x04,
		R2_SWAP = 0x10,
		R2_WAITC = 0x20
	};

	void reset_registers();
	void update_timing();
	void set_cse(int state);
	bool displaying() const { return (m_r1 & (R1_RESET | R1_DISP)) == R1_DISP; }
	TIMER_CALLBACK_MEMBER(cse_tick);

	devcb_write_line m_cse_cb;
	devcb_write_line m_lcdenb_cb;
	emu_timer *m_cse_timer;

	std::unique_ptr<u8[]> m_vram;
	u8 m_pattern[64];

	u8 m_r1;
	u8 m_r2;
	u8 m_cr;
	u8 m_lpw;
	u8 m_csw;
	u8 m_slt;
	u16 m_sa1;
	u16 m_sa2;
	u8 m_sa1l;
	u8 m_sa2l;
	u8 m_mt;
	u16 m_idx;
	int m_cse;
	attotime m_char_period;
};

DECLARE_DEVICE_TYPE(M66273, m66273_device)

#endif // MAME_VIDEO_M66273_H
