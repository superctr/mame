// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Sitronix ST7565 / Epson SED1565 LCD controller

***************************************************************************/

#ifndef MAME_VIDEO_ST7565_H
#define MAME_VIDEO_ST7565_H

#pragma once


// ======================> st7565_device

class st7565_device : public device_t
{
public:
	st7565_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	// the dots of the glass, and the SEG pin behind the leftmost one; the
	// step is -1 for a panel wired the other way round
	void set_panel(unsigned columns, unsigned lines, int first_segment = 0, int segment_step = 1);

	// A0 is the low address line: 0 the command register, 1 the display data
	void write(offs_t offset, uint8_t data);
	uint8_t read(offs_t offset);

	void control_write(uint8_t data);
	uint8_t status_read();
	void data_write(uint8_t data);
	uint8_t data_read();

	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

protected:
	st7565_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	static constexpr unsigned SEGS = 132;
	static constexpr unsigned PAGES = 9;

	unsigned m_columns;
	unsigned m_lines;
	int m_first_segment;
	int m_segment_step;

	uint8_t m_page;
	uint8_t m_column;
	uint8_t m_old_column;
	uint8_t m_start_line;
	uint8_t m_contrast;
	uint8_t m_resistor_ratio;
	uint8_t m_power;
	uint8_t m_indicator;
	uint8_t m_data;
	uint8_t m_pending;
	bool m_lcd_on;
	bool m_column_reverse;
	bool m_common_reverse;
	bool m_display_reverse;
	bool m_all_on;
	bool m_bias;
	bool m_modify_write;

	std::unique_ptr<uint8_t[]> m_ddr;
};


// ======================> sed1565_device

class sed1565_device : public st7565_device
{
public:
	sed1565_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);
};


DECLARE_DEVICE_TYPE(ST7565, st7565_device)
DECLARE_DEVICE_TYPE(SED1565, sed1565_device)

#endif // MAME_VIDEO_ST7565_H
