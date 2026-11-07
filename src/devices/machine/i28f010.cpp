// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Intel 28F010 128K x 8 flash memory, the first generation's command set:
    00 read, 90 identifier, 20 20 erase (the whole chip), A0 erase verify,
    40 program, C0 program verify, FF FF reset.  Program and erase complete
    at once.

***************************************************************************/

#include "emu.h"
#include "i28f010.h"

#define VERBOSE (0)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(INTEL_28F010, intel_28f010_device, "intel_28f010", "Intel 28F010 flash")

namespace {

constexpr u32 SIZE = 0x20000;

}

intel_28f010_device::intel_28f010_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, INTEL_28F010, tag, owner, clock),
	device_nvram_interface(mconfig, *this),
	m_mode(MODE_READ),
	m_reset_count(0)
{
}

void intel_28f010_device::device_start()
{
	m_data = std::make_unique<u8[]>(SIZE);
	save_pointer(NAME(m_data), SIZE);
	save_item(NAME(m_mode));
	save_item(NAME(m_reset_count));
}

void intel_28f010_device::device_reset()
{
	m_mode = MODE_READ;
	m_reset_count = 0;
}

void intel_28f010_device::nvram_default()
{
	std::fill_n(m_data.get(), SIZE, 0xff);
}

bool intel_28f010_device::nvram_read(util::read_stream &file)
{
	auto const [err, actual] = util::read(file, m_data.get(), SIZE);
	return !err && actual == SIZE;
}

bool intel_28f010_device::nvram_write(util::write_stream &file)
{
	auto const [err, actual] = util::write(file, m_data.get(), SIZE);
	return !err;
}

u8 intel_28f010_device::read(offs_t offset)
{
	offset &= SIZE - 1;
	if(m_mode == MODE_ID)
		return BIT(offset, 0) ? 0xb4 : 0x89;
	return m_data[offset];
}

void intel_28f010_device::write(offs_t offset, u8 data)
{
	offset &= SIZE - 1;
	switch(m_mode) {
	case MODE_PROGRAM_SETUP:
		LOG("program %05x = %02x\n", offset, data);
		m_data[offset] &= data;
		m_mode = MODE_READ;
		return;

	case MODE_ERASE_SETUP:
		m_mode = MODE_READ;
		if(data == 0x20) {
			LOG("erase\n");
			std::fill_n(m_data.get(), SIZE, 0xff);
			return;
		}
		break;
	}

	if(data != 0xff)
		m_reset_count = 0;
	switch(data) {
	case 0x00:
	case 0xa0:
	case 0xc0:
		m_mode = MODE_READ;
		break;
	case 0x90:
		m_mode = MODE_ID;
		break;
	case 0x20:
		m_mode = MODE_ERASE_SETUP;
		break;
	case 0x40:
		m_mode = MODE_PROGRAM_SETUP;
		break;
	case 0xff:
		if(++m_reset_count == 2) {
			m_mode = MODE_READ;
			m_reset_count = 0;
		}
		break;
	default:
		LOG("unknown command %02x at %05x\n", data, offset);
		break;
	}
}
