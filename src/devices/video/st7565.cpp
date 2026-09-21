// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Sitronix ST7565 / Epson SED1565 LCD controller

    A 65 x 132 dot column driver with the display data RAM on the chip:
    nine pages of 132 bytes, one byte to a column of eight commons, the
    ninth page one line deep for the static indicator.

    TODO:
    - the static indicator, the power saver and the busy flag
    - the serial interface

***************************************************************************/

#include "emu.h"
#include "st7565.h"

#include "screen.h"

#define LOG_COMMAND (1U << 1)
#define LOG_DATA    (1U << 2)

#define VERBOSE (LOG_GENERAL)
#include "logmacro.h"


DEFINE_DEVICE_TYPE(ST7565, st7565_device, "st7565", "Sitronix ST7565 LCD Driver")
DEFINE_DEVICE_TYPE(SED1565, sed1565_device, "sed1565", "Epson SED1565 LCD Driver")


st7565_device::st7565_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock)
	, m_columns(SEGS)
	, m_lines(65)
	, m_first_segment(0)
	, m_segment_step(1)
{
}

st7565_device::st7565_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: st7565_device(mconfig, ST7565, tag, owner, clock)
{
}

sed1565_device::sed1565_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: st7565_device(mconfig, SED1565, tag, owner, clock)
{
}

void st7565_device::set_panel(unsigned columns, unsigned lines, int first_segment, int segment_step)
{
	m_columns = columns;
	m_lines = lines;
	m_first_segment = first_segment;
	m_segment_step = segment_step;
}


void st7565_device::device_start()
{
	m_ddr = std::make_unique<uint8_t[]>(PAGES * SEGS);

	save_item(NAME(m_page));
	save_item(NAME(m_column));
	save_item(NAME(m_old_column));
	save_item(NAME(m_start_line));
	save_item(NAME(m_contrast));
	save_item(NAME(m_resistor_ratio));
	save_item(NAME(m_power));
	save_item(NAME(m_indicator));
	save_item(NAME(m_data));
	save_item(NAME(m_pending));
	save_item(NAME(m_lcd_on));
	save_item(NAME(m_column_reverse));
	save_item(NAME(m_common_reverse));
	save_item(NAME(m_display_reverse));
	save_item(NAME(m_all_on));
	save_item(NAME(m_bias));
	save_item(NAME(m_modify_write));
	save_pointer(NAME(m_ddr), PAGES * SEGS);
}

void st7565_device::device_reset()
{
	m_page = 0;
	m_column = 0;
	m_old_column = 0;
	m_start_line = 0;
	m_contrast = 0x20;
	m_resistor_ratio = 0;
	m_power = 0;
	m_indicator = 0;
	m_data = 0;
	m_pending = 0;
	m_lcd_on = false;
	m_column_reverse = false;
	m_common_reverse = false;
	m_display_reverse = false;
	m_all_on = false;
	m_bias = false;
	m_modify_write = false;
}


void st7565_device::write(offs_t offset, uint8_t data)
{
	if (BIT(offset, 0))
		data_write(data);
	else
		control_write(data);
}

uint8_t st7565_device::read(offs_t offset)
{
	return BIT(offset, 0) ? data_read() : status_read();
}

uint8_t st7565_device::status_read()
{
	return (m_column_reverse ? 0 : 0x40) | (m_lcd_on ? 0 : 0x20);
}

void st7565_device::data_write(uint8_t data)
{
	LOGMASKED(LOG_DATA, "%s: page %d column %3d <- %02x\n", machine().describe_context(), m_page, m_column, data);
	m_data = data;
	if (m_page < PAGES && m_column < SEGS)
		m_ddr[m_page * SEGS + m_column] = data;
	if (m_column < 0x83)
		m_column++;
}

uint8_t st7565_device::data_read()
{
	// the read is a word behind the column address, so the first one is a dummy
	const uint8_t data = m_data;
	if (machine().side_effects_disabled())
		return data;

	m_data = (m_page < PAGES && m_column < SEGS) ? m_ddr[m_page * SEGS + m_column] : 0;
	if (!m_modify_write && m_column < 0x83)
		m_column++;

	return data;
}

void st7565_device::control_write(uint8_t data)
{
	LOGMASKED(LOG_COMMAND, "%s: command %02x\n", machine().describe_context(), data);

	switch (m_pending)
	{
	case 0x81:                               // electronic volume register
		m_contrast = data & 0x3f;
		m_pending = 0;
		return;

	case 0xad:                               // static indicator register
		m_indicator = data & 0x03;
		m_pending = 0;
		return;
	}

	if (data == 0x81 || data == 0xad)        // the two double byte commands
		m_pending = data;
	else if ((data & 0xf0) == 0x00)          // column address, low four bits
		m_column = (m_column & 0xf0) | (data & 0x0f);
	else if ((data & 0xf0) == 0x10)          // column address, high four bits
		m_column = (m_column & 0x0f) | ((data & 0x0f) << 4);
	else if ((data & 0xf8) == 0x20)          // V5 regulator internal resistor ratio
		m_resistor_ratio = data & 0x07;
	else if ((data & 0xf8) == 0x28)          // booster, regulator and follower
		m_power = data & 0x07;
	else if ((data & 0xc0) == 0x40)          // display start line
		m_start_line = data & 0x3f;
	else if ((data & 0xfe) == 0xa0)          // segment driver direction
		m_column_reverse = BIT(data, 0);
	else if ((data & 0xfe) == 0xa2)          // bias
		m_bias = BIT(data, 0);
	else if ((data & 0xfe) == 0xa4)          // display all points
		m_all_on = BIT(data, 0);
	else if ((data & 0xfe) == 0xa6)          // normal or reverse display
		m_display_reverse = BIT(data, 0);
	else if (data == 0xac)                   // static indicator off
		m_indicator = 0;
	else if ((data & 0xfe) == 0xae)          // display on or off
		m_lcd_on = BIT(data, 0);
	else if ((data & 0xf0) == 0xb0)          // page address
		m_page = data & 0x0f;
	else if ((data & 0xf0) == 0xc0)          // common output direction
		m_common_reverse = BIT(data, 3);
	else if (data == 0xe0)                   // read/modify/write
	{
		m_modify_write = true;
		m_old_column = m_column;
	}
	else if (data == 0xee)                   // end
	{
		m_modify_write = false;
		m_column = m_old_column;
	}
	else if (data == 0xe2)                   // reset, which leaves the RAM alone
	{
		m_page = 0;
		m_column = 0;
		m_start_line = 0;
		m_contrast = 0x20;
		m_resistor_ratio = 0;
		m_indicator = 0;
		m_common_reverse = false;
		m_modify_write = false;
	}
	else if (data == 0xe3)                   // NOP
		;
	else
		logerror("%s: invalid ST7565 command: %02x\n", machine().describe_context(), data);
}


uint32_t st7565_device::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	for (int y = cliprect.top(); y <= cliprect.bottom(); y++)
	{
		const int common = m_common_reverse ? (m_lines - 1 - y) : y;
		// the start line scrolls the 64 display lines; the 65th is the indicator's
		const int line = (common < 64) ? ((common + m_start_line) & 0x3f) : 64;
		const uint8_t *const page = &m_ddr[(line >> 3) * SEGS];

		for (int x = cliprect.left(); x <= cliprect.right(); x++)
		{
			const int segment = m_first_segment + x * m_segment_step;
			const int column = m_column_reverse ? (int(SEGS) - 1 - segment) : segment;
			const bool on = m_lcd_on && (m_all_on || (column >= 0 && column < int(SEGS) && BIT(page[column], line & 7)));
			bitmap.pix(y, x) = on != m_display_reverse;
		}
	}

	return 0;
}
