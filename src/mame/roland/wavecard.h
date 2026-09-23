// license:BSD-3-Clause
// copyright-holders:superctr
#ifndef MAME_ROLAND_WAVECARD_H
#define MAME_ROLAND_WAVECARD_H

#pragma once

#include "imagedev/cartrom.h"


class roland_wavecard_device : public device_t, public device_cartrom_image_interface
{
public:
	// the ROM, a byte a cell on a byte-wide bus and a word a cell on a
	// word-wide one, zero where nothing answers
	u8 read(offs_t offset) { return (m_rom && offset < m_size) ? m_rom[offset] : 0; }
	u16 read16(offs_t offset) { return (m_rom && (offset << 1) < m_size) ? (m_rom[offset << 1] | (m_rom[(offset << 1) | 1] << 8)) : 0; }

	// high while a card is fitted; the host decides which pin reads it and in which sense
	int sense_r() const { return m_rom ? 1 : 0; }

	// the media option names, the socket's legend on the host's display
	roland_wavecard_device &set_image_names(std::string type_name, std::string brief_type_name);

protected:
	roland_wavecard_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock,
			u32 min_size, u32 max_size, const char *size_error, const u8 *address_lines, int lines,
			const char *type_name, const char *brief_type_name);

	virtual void device_start() override ATTR_COLD;

	virtual std::pair<std::error_condition, std::string> call_load() override ATTR_COLD;
	virtual void call_unload() override ATTR_COLD;

	virtual const char *image_type_name() const noexcept override { return m_type_name.c_str(); }
	virtual const char *image_brief_type_name() const noexcept override { return m_brief_type_name.c_str(); }
	virtual const char *file_extensions() const noexcept override { return "bin"; }
	virtual bool is_reset_on_load() const noexcept override { return true; }

private:
	void descramble() ATTR_COLD;

	const u32 m_min_size, m_max_size;
	const char *const m_size_error;
	const u8 *const m_address_lines;
	const int m_lines;
	std::string m_type_name, m_brief_type_name;
	u32 m_size;
	std::unique_ptr<u8 []> m_rom;
};


class roland_srjv80_slot_device : public roland_wavecard_device
{
public:
	roland_srjv80_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

protected:
	virtual const char *image_interface() const noexcept override { return "srjv80"; }
};


class roland_srx_slot_device : public roland_wavecard_device
{
public:
	roland_srx_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

protected:
	virtual const char *image_interface() const noexcept override { return "srx"; }
};


class roland_sopcm1_slot_device : public roland_wavecard_device
{
public:
	roland_sopcm1_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

protected:
	virtual const char *image_interface() const noexcept override { return "sopcm1"; }
};

DECLARE_DEVICE_TYPE(ROLAND_SRJV80_SLOT, roland_srjv80_slot_device)
DECLARE_DEVICE_TYPE(ROLAND_SRX_SLOT, roland_srx_slot_device)
DECLARE_DEVICE_TYPE(ROLAND_SOPCM1_SLOT, roland_sopcm1_slot_device)

#endif // MAME_ROLAND_WAVECARD_H
