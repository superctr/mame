// license:BSD-3-Clause
// copyright-holders:superctr
#ifndef MAME_ROLAND_DATACARD_H
#define MAME_ROLAND_DATACARD_H

#pragma once

#include "imagedev/memcard.h"


class roland_datacard_device : public device_t, public device_memcard_image_interface
{
public:
	roland_datacard_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	// the card's cells, mirrored above a small card's top; nothing drives the
	// bus with no card fitted
	u8 read(offs_t offset) { return m_size ? m_ram[offset & (m_size - 1)] : 0xff; }
	void write(offs_t offset, u8 data);

	// high while a card is fitted
	int sense_r() const { return m_size ? 1 : 0; }

	// the protect switch, which is the image's read-only state
	int protect_r() const { return (m_size && is_readonly()) ? 1 : 0; }

	virtual bool is_reset_on_load() const noexcept override { return false; }
	virtual const char *file_extensions() const noexcept override { return "bin"; }

	virtual std::pair<std::error_condition, std::string> call_load() override ATTR_COLD;
	virtual void call_unload() override ATTR_COLD;
	virtual std::pair<std::error_condition, std::string> call_create(int format_type, util::option_resolution *format_options) override ATTR_COLD;

protected:
	virtual void device_start() override ATTR_COLD;

private:
	static constexpr u32 MAX_SIZE = 0x10000;

	std::unique_ptr<u8 []> m_ram;
	u32 m_size;
};

DECLARE_DEVICE_TYPE(ROLAND_DATACARD, roland_datacard_device)

#endif // MAME_ROLAND_DATACARD_H
