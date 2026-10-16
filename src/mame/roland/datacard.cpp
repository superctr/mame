// license:BSD-3-Clause
// copyright-holders:superctr
/****************************************************************************

    Roland M-256E and M-512E memory cards.

    32 or 64 KB of battery-backed RAM a byte a cell, with a write protect
    switch, for a machine's DATA CARD slot.  A new image is an M-512E.

****************************************************************************/

#include "emu.h"
#include "datacard.h"

#include <algorithm>


DEFINE_DEVICE_TYPE(ROLAND_DATACARD, roland_datacard_device, "roland_datacard", "Roland M-256E/M-512E memory card")

roland_datacard_device::roland_datacard_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, ROLAND_DATACARD, tag, owner, clock)
	, device_memcard_image_interface(mconfig, *this)
	, m_size(0)
{
}

void roland_datacard_device::device_start()
{
	m_ram = std::make_unique<u8 []>(MAX_SIZE);
	std::fill_n(m_ram.get(), MAX_SIZE, 0xff);

	save_pointer(NAME(m_ram), MAX_SIZE);
	save_item(NAME(m_size));
}

void roland_datacard_device::write(offs_t offset, u8 data)
{
	if (m_size && !is_readonly())
		m_ram[offset & (m_size - 1)] = data;
}

std::pair<std::error_condition, std::string> roland_datacard_device::call_load()
{
	const u64 size = length();
	if (size != 0x8000 && size != 0x10000)
		return std::make_pair(image_error::INVALIDLENGTH, "Memory cards are 32 KB (M-256E) or 64 KB (M-512E)");

	fseek(0, SEEK_SET);
	if (fread(m_ram.get(), size) != size)
		return std::make_pair(std::errc::io_error, "Error reading file");

	m_size = size;
	return std::make_pair(std::error_condition(), std::string());
}

void roland_datacard_device::call_unload()
{
	if (m_size && !is_readonly())
	{
		fseek(0, SEEK_SET);
		fwrite(m_ram.get(), m_size);
	}
	std::fill_n(m_ram.get(), MAX_SIZE, 0xff);
	m_size = 0;
}

std::pair<std::error_condition, std::string> roland_datacard_device::call_create(int format_type, util::option_resolution *format_options)
{
	std::fill_n(m_ram.get(), MAX_SIZE, 0);
	if (fwrite(m_ram.get(), MAX_SIZE) != MAX_SIZE)
		return std::make_pair(std::errc::io_error, "Error writing file");

	m_size = MAX_SIZE;
	return std::make_pair(std::error_condition(), std::string());
}
