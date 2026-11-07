// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Ricoh RF5C16A / RP5C16 CRT controller

***************************************************************************/

#ifndef MAME_VIDEO_RF5C16_H
#define MAME_VIDEO_RF5C16_H

#pragma once

class rf5c16_device : public device_t, public device_video_interface
{
public:
	rf5c16_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	auto int_callback() { return m_int_cb.bind(); }

	u8 read(offs_t offset);
	void write(offs_t offset, u8 data);

	u32 screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	static constexpr feature_type imperfect_features() { return feature::GRAPHICS; }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	TIMER_CALLBACK_MEMBER(vblank_tick);

private:
	devcb_write_line m_int_cb;

	std::unique_ptr<u16[]> m_vram;
	emu_timer *m_vblank_timer;

	u16 m_tr;
	u16 m_addr;
	u8 m_imask;
	u8 m_iflag;
	u8 m_mode;
	u16 m_cursor_x;
	u8 m_cursor_y;
	u8 m_bcg;
	u8 m_bfg;
	u16 m_bbg;
	u8 m_fgcol;
	u8 m_bgcol;
	u8 m_bdcol;
	u8 m_scroll;

	void transfer_write();
	void transfer_read();
	void update_int();
	u32 pen(u8 code) const;
	u8 cell_dot(u16 code, int x, int y, bool fg) const;
};

DECLARE_DEVICE_TYPE(RF5C16, rf5c16_device)

#endif // MAME_VIDEO_RF5C16_H
