// license:BSD-3-Clause
// copyright-holders:superctr
#ifndef MAME_ROLAND_SRJV80_H
#define MAME_ROLAND_SRJV80_H

#pragma once

#include "imagedev/cartrom.h"


class srjv80_slot_device : public device_t, public device_cartrom_image_interface
{
public:
	srjv80_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	// the wave space the socket answers in, and where
	template <typename T> void set_wave(T &&tag, int spacenum, offs_t base)
	{
		m_wave.set_tag(std::forward<T>(tag), spacenum);
		m_base = base;
	}

	// high while a board is fitted; the host decides which pin reads it and in which sense
	int sense_r() const { return m_rom ? 1 : 0; }

protected:
	virtual void device_start() override ATTR_COLD;

	virtual std::pair<std::error_condition, std::string> call_load() override ATTR_COLD;
	virtual void call_unload() override ATTR_COLD;

	virtual const char *image_interface() const noexcept override { return "srjv80"; }
	virtual const char *file_extensions() const noexcept override { return "bin"; }
	virtual bool is_reset_on_load() const noexcept override { return true; }

private:
	static inline constexpr u32 BOARD_SIZE = 0x800000;

	void descramble() ATTR_COLD;

	required_address_space m_wave;
	offs_t m_base;
	std::unique_ptr<u8 []> m_rom;
};

DECLARE_DEVICE_TYPE(SRJV80_SLOT, srjv80_slot_device)

#endif // MAME_ROLAND_SRJV80_H
