// license:BSD-3-Clause
// copyright-holders:superctr
/****************************************************************************

    Roland SR-JV80 wave expansion board socket.

    An 8 MB mask ROM on a board that sits on the host's wave bus, one byte
    a cell.  The JV-1080, JD-990, XV-3080 and XV-5080 all take the same
    boards; each decides for itself where its sockets answer and which pin
    reads the sense line.

****************************************************************************/

#include "emu.h"
#include "srjv80.h"


DEFINE_DEVICE_TYPE(SRJV80_SLOT, srjv80_slot_device, "srjv80_slot", "Roland SR-JV80 expansion board socket")

srjv80_slot_device::srjv80_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, SRJV80_SLOT, tag, owner, clock)
	, device_cartrom_image_interface(mconfig, *this)
	, m_wave(*this, finder_base::DUMMY_TAG, -1)
	, m_base(0)
{
}

void srjv80_slot_device::device_start()
{
}


// the board permutes nineteen address lines within each 512 KB block and the
// eight data lines; the data table is the one the JV-1080's own wave ROMs use
// and the address table is the card's own, shared with nothing
void srjv80_slot_device::descramble()
{
	static const u8 address_lines[19] = { 2, 0, 3, 4, 1, 9, 13, 10, 18, 17, 6, 15, 11, 16, 8, 5, 12, 7, 14 };
	static const u8 data_lines[8] = { 2, 0, 4, 5, 7, 6, 3, 1 };

	const std::vector<u8> scrambled(m_rom.get(), m_rom.get() + BOARD_SIZE);
	for (u32 i = 0; i < BOARD_SIZE; i++)
	{
		u32 address = i & ~0x7ffff;
		for (int bit = 0; bit < 19; bit++)
			if (BIT(i, bit))
				address |= 1 << address_lines[bit];

		const u8 source = scrambled[address];
		u8 data = 0;
		for (int bit = 0; bit < 8; bit++)
			if (BIT(source, data_lines[bit]))
				data |= 1 << bit;
		m_rom[i] = data;
	}
}

std::pair<std::error_condition, std::string> srjv80_slot_device::call_load()
{
	const u32 size = loaded_through_softlist() ? get_software_region_length("rom") : length();
	if (size != BOARD_SIZE)
		return std::make_pair(image_error::INVALIDLENGTH, "Expansion boards are 8 MB");

	m_rom = std::make_unique<u8 []>(BOARD_SIZE);
	if (loaded_through_softlist())
		std::copy_n(get_software_region("rom"), BOARD_SIZE, m_rom.get());
	else if (fread(m_rom.get(), BOARD_SIZE) != BOARD_SIZE)
	{
		m_rom.reset();
		return std::make_pair(image_error::UNSPECIFIED, "Error reading file");
	}

	descramble();
	m_wave->install_rom(m_base, m_base + BOARD_SIZE - 1, m_rom.get());
	return std::make_pair(std::error_condition(), std::string());
}

void srjv80_slot_device::call_unload()
{
	m_wave->unmap_read(m_base, m_base + BOARD_SIZE - 1);
	m_rom.reset();
}
