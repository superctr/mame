// license:BSD-3-Clause
// copyright-holders:superctr
/****************************************************************************

    Roland wave ROM boards and cards.

    A mask ROM on a board or a card that sits on the host's wave bus, one
    byte a cell, carrying its own directory and its own name.  The socket
    is the same in every machine that takes one, and so is the line swap
    the ROM wears; each host decides only where its sockets answer and
    which pin reads the sense line.

    SR-JV80 wave expansion boards are 8 MB and go in the JV-1080, the
    JD-990, the XV-3080 and the XV-5080.  SO-PCM1 and SO-JD80 PCM cards
    are 1 or 2 MB and go in the JD-800 and the JD-990.

****************************************************************************/

#include "emu.h"
#include "wavecard.h"


DEFINE_DEVICE_TYPE(SRJV80_SLOT, srjv80_slot_device, "srjv80_slot", "Roland SR-JV80 expansion board socket")
DEFINE_DEVICE_TYPE(SOPCM1_SLOT, sopcm1_slot_device, "sopcm1_slot", "Roland PCM card slot")

roland_wavecard_device::roland_wavecard_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock,
		u32 min_size, u32 max_size, const char *size_error)
	: device_t(mconfig, type, tag, owner, clock)
	, device_cartrom_image_interface(mconfig, *this)
	, m_min_size(min_size)
	, m_max_size(max_size)
	, m_size_error(size_error)
	, m_size(0)
{
}

void roland_wavecard_device::device_start()
{
}


// nineteen address lines permuted within each 512 KB block and the eight data
// lines; the data table is the one Roland's own wave ROMs use and the address
// table is the card's own
void roland_wavecard_device::descramble()
{
	static const u8 address_lines[19] = { 2, 0, 3, 4, 1, 9, 13, 10, 18, 17, 6, 15, 11, 16, 8, 5, 12, 7, 14 };
	static const u8 data_lines[8] = { 2, 0, 4, 5, 7, 6, 3, 1 };

	const std::vector<u8> scrambled(m_rom.get(), m_rom.get() + m_size);
	for (u32 i = 0; i < m_size; i++)
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

std::pair<std::error_condition, std::string> roland_wavecard_device::call_load()
{
	const u32 size = loaded_through_softlist() ? get_software_region_length("rom") : length();
	if (size < m_min_size || size > m_max_size || (size & 0x7ffff))
		return std::make_pair(image_error::INVALIDLENGTH, m_size_error);

	m_size = size;
	m_rom = std::make_unique<u8 []>(m_size);
	if (loaded_through_softlist())
		std::copy_n(get_software_region("rom"), m_size, m_rom.get());
	else if (fread(m_rom.get(), m_size) != m_size)
	{
		m_rom.reset();
		return std::make_pair(image_error::UNSPECIFIED, "Error reading file");
	}

	descramble();
	return std::make_pair(std::error_condition(), std::string());
}

void roland_wavecard_device::call_unload()
{
	m_rom.reset();
	m_size = 0;
}


srjv80_slot_device::srjv80_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: roland_wavecard_device(mconfig, SRJV80_SLOT, tag, owner, clock, 0x800000, 0x800000, "Expansion boards are 8 MB")
{
}

sopcm1_slot_device::sopcm1_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: roland_wavecard_device(mconfig, SOPCM1_SLOT, tag, owner, clock, 0x100000, 0x200000, "PCM cards are 1 or 2 MB")
{
}
