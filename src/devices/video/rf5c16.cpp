// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Ricoh RF5C16A / RP5C16 CRT controller

    64K words of DRAM behind a transfer register and an auto-incrementing
    address counter; 80x25 or 40x25 text, 640x200 or 320x200 four-plane
    graphics, a 40x25 text plane over the 40-column modes, a fixed
    16-colour palette from R, G, B and light/dark, and a crosshair cursor.

    Not emulated: DMA, the dot-clock timing of transfers, master/slave
    sync, the three-level output and the cell attributes' inversions.

***************************************************************************/

#include "emu.h"
#include "rf5c16.h"

#include "screen.h"

#define VERBOSE (LOG_GENERAL)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(RF5C16, rf5c16_device, "rf5c16", "Ricoh RF5C16A CRT controller")

namespace {

enum : u8
{
	IM_DMA  = 0x10,
	IM_WORD = 0x08,
	IM_RMW  = 0x04,
	IM_READ = 0x02,
	IM_MSB  = 0x01,

	IF_V = 0x08,
	IF_H = 0x04,
	IF_B = 0x02
};

}

rf5c16_device::rf5c16_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, RF5C16, tag, owner, clock),
	device_video_interface(mconfig, *this),
	m_int_cb(*this),
	m_middle(0xaa),
	m_vblank_timer(nullptr),
	m_tr(0), m_addr(0), m_imask(0), m_iflag(0), m_mode(0),
	m_cursor_x(0), m_cursor_y(0), m_bcg(0), m_bcg_cursor(0), m_bfg(0), m_bbg(0),
	m_fgcol(0), m_bgcol(0), m_bdcol(0), m_scroll(0)
{
}

void rf5c16_device::device_start()
{
	m_vram = std::make_unique<u16[]>(0x10000);
	std::fill_n(m_vram.get(), 0x10000, 0);

	m_vblank_timer = timer_alloc(FUNC(rf5c16_device::vblank_tick), this);

	save_pointer(NAME(m_vram), 0x10000);
	save_item(NAME(m_tr));
	save_item(NAME(m_addr));
	save_item(NAME(m_imask));
	save_item(NAME(m_iflag));
	save_item(NAME(m_mode));
	save_item(NAME(m_cursor_x));
	save_item(NAME(m_cursor_y));
	save_item(NAME(m_bcg));
	save_item(NAME(m_bcg_cursor));
	save_item(NAME(m_bfg));
	save_item(NAME(m_bbg));
	save_item(NAME(m_fgcol));
	save_item(NAME(m_bgcol));
	save_item(NAME(m_bdcol));
	save_item(NAME(m_scroll));
}

void rf5c16_device::device_reset()
{
	m_imask = 0;
	m_iflag = IF_B;
	m_mode = 0;
	update_int();
	m_vblank_timer->adjust(screen().time_until_pos(screen().visible_area().bottom() + 1));
}

TIMER_CALLBACK_MEMBER(rf5c16_device::vblank_tick)
{
	m_iflag |= IF_V;
	update_int();
	m_vblank_timer->adjust(screen().time_until_pos(screen().visible_area().bottom() + 1));
}

void rf5c16_device::update_int()
{
	m_int_cb((m_imask & 0xe0) & ((m_iflag & 0x0e) << 4) ? 1 : 0);
}

void rf5c16_device::transfer_write()
{
	m_vram[m_addr] = m_tr;
	m_addr++;
}

void rf5c16_device::transfer_read()
{
	m_tr = m_vram[m_addr];
}

u8 rf5c16_device::read(offs_t offset)
{
	switch (offset & 0xf)
	{
	case 0:
	{
		u8 const data = m_tr;
		if (!machine().side_effects_disabled() && (m_imask & IM_READ))
		{
			m_addr++;
			transfer_read();
		}
		return data;
	}
	case 1:
	{
		u8 const data = m_tr >> 8;
		if (!machine().side_effects_disabled() && (m_imask & IM_READ) && !(m_imask & IM_WORD))
		{
			m_addr++;
			transfer_read();
		}
		return data;
	}
	case 2:
		return m_addr;
	case 3:
		return m_addr >> 8;
	case 4:
	{
		u8 const data = (m_imask & 0xe0) | (m_iflag & 0x0e);
		if (!machine().side_effects_disabled())
		{
			m_iflag &= ~(IF_V | IF_H);
			update_int();
		}
		return data;
	}
	default:
		if (!machine().side_effects_disabled())
			LOG("read %x\n", offset & 0xf);
		return 0;
	}
}

void rf5c16_device::write(offs_t offset, u8 data)
{
	switch (offset & 0xf)
	{
	case 0:
		m_tr = (m_tr & 0xff00) | data;
		if (m_imask & IM_READ)
			m_vram[m_addr] = m_tr;
		else
		{
			transfer_write();
			if (m_imask & IM_RMW)
				transfer_read();
		}
		break;
	case 1:
		m_tr = (m_tr & 0x00ff) | (data << 8);
		if (!(m_imask & (IM_READ | IM_WORD)))
		{
			transfer_write();
			if (m_imask & IM_RMW)
				transfer_read();
		}
		break;
	case 2:
		m_addr = (m_addr & 0xff00) | data;
		if (m_imask & (IM_READ | IM_RMW))
			transfer_read();
		break;
	case 3:
		m_addr = (m_addr & 0x00ff) | (data << 8);
		break;
	case 4:
		m_imask = data;
		update_int();
		break;
	case 5:
		m_mode = data;
		break;
	case 6:
		m_cursor_x = (m_bcg_cursor & 3) << 8 | data;
		break;
	case 7:
		m_cursor_y = data;
		break;
	case 8:
		m_bcg = data >> 3;
		m_bcg_cursor = data;
		break;
	case 9:
		m_bfg = data >> 2;
		break;
	case 0xa:
		m_bbg = (m_bbg & 0xff00) | data;
		break;
	case 0xb:
		m_bbg = (m_bbg & 0x00ff) | (data << 8);
		break;
	case 0xc:
		m_fgcol = data;
		break;
	case 0xd:
		m_bgcol = data;
		break;
	case 0xe:
		m_bdcol = data;
		break;
	case 0xf:
		m_scroll = data;
		break;
	}
}

u32 rf5c16_device::pen(u8 code) const
{
	u8 const on = BIT(code, 3) ? 0xff : m_middle;
	return rgb_t(BIT(code, 0) ? on : 0, BIT(code, 1) ? on : 0, BIT(code, 2) ? on : 0);
}

u8 rf5c16_device::cell_dot(u16 code, int x, int y, bool fg) const
{
	if (BIT(code, 11))
		y = 7 - y;
	if (BIT(code, 10))
		x = 7 - x;
	u16 const row = m_vram[((m_bcg << 11) + ((code & 0xff) << 3) + y) & 0xffff];
	u8 dot = (BIT(row, 15 - x) << 1) | BIT(row, 7 - x);
	if (BIT(code, 8))
		dot = dot == 1 ? 0 : dot == 0 ? 1 : dot;
	switch (dot)
	{
	case 1: return 0x10 | (code >> 12);
	case 2: return 0x10 | ((fg ? m_fgcol : m_bgcol) & 0xf);
	case 3: return 0x10 | ((fg ? m_fgcol : m_bgcol) >> 4);
	default: return 0;
	}
}

u32 rf5c16_device::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	static constexpr int widths[4] = { 40, 64, 80, 128 };
	int const mode = m_mode >> 6;
	bool const fg_on = BIT(m_mode, 5) && mode >= 2;
	bool const bg_on = BIT(m_mode, 4);
	int const pitch = widths[m_mode & 3];
	bool const narrow = mode != 0 && mode != 1;
	int const sh = m_scroll & 7;
	int const sv = (m_scroll >> 4) & 7;

	for (int y = cliprect.top(); y <= cliprect.bottom(); y++)
	{
		u32 *const dst = &bitmap.pix(y);
		for (int x = cliprect.left(); x <= cliprect.right(); x++)
		{
			int const dx = narrow ? x >> 1 : x;
			u8 col = 0;

			if (fg_on && dx < 320)
			{
				u16 const code = m_vram[((m_bfg << 10) + (y >> 3) * 40 + (dx >> 3)) & 0xffff];
				u8 const d = cell_dot(code, dx & 7, y & 7, true);
				if (d && !BIT(code, 9))
					col = d;
			}

			if (!col && bg_on)
			{
				int const bx = dx + sh;
				int const by = y + sv;
				if (mode == 1 || mode == 3)
				{
					u32 const w = m_bbg + by * pitch * 2 + (bx >> 3) * 2;
					u16 const gr = m_vram[w & 0xffff];
					u16 const lb = m_vram[(w + 1) & 0xffff];
					int const b = 7 - (bx & 7);
					u8 const c = BIT(gr, b) | (BIT(gr, b + 8) << 1) | (BIT(lb, b) << 2) | (BIT(lb, b + 8) << 3);
					if (c)
						col = 0x10 | c;
				}
				else
				{
					u16 const code = m_vram[(m_bbg + (by >> 3) * pitch + (bx >> 3)) & 0xffff];
					col = cell_dot(code, bx & 7, by & 7, false);
				}
			}

			if (!col && fg_on && dx < 320)
			{
				u16 const code = m_vram[((m_bfg << 10) + (y >> 3) * 40 + (dx >> 3)) & 0xffff];
				col = cell_dot(code, dx & 7, y & 7, true);
			}

			dst[x] = pen(col ? col & 0xf : m_bdcol & 0xf);
		}
	}

	int const scale = narrow ? 2 : 1;
	int const cx = m_cursor_x * scale;
	int const cy = m_cursor_y;
	for (int i = -4 * scale; i < 5 * scale; i++)
		if (cliprect.contains(cx + i, cy))
			bitmap.pix(cy, cx + i) = 0xffffff;
	for (int i = -4; i <= 4; i++)
		for (int j = 0; j < scale; j++)
			if (cliprect.contains(cx + j, cy + i))
				bitmap.pix(cy + i, cx + j) = 0xffffff;

	return 0;
}
