// license:BSD-3-Clause
// copyright-holders:superctr
#ifndef MAME_ROLAND_WAVECARD_H
#define MAME_ROLAND_WAVECARD_H

#pragma once

#include "imagedev/cartrom.h"


class roland_wavecard_device : public device_t, public device_cartrom_image_interface
{
public:
	// the wave space the socket answers in, and where
	template <typename T> void set_wave(T &&tag, int spacenum, offs_t base)
	{
		m_wave.set_tag(std::forward<T>(tag), spacenum);
		m_base = base;
	}

	// high while a card is fitted; the host decides which pin reads it and in which sense
	int sense_r() const { return m_rom ? 1 : 0; }

protected:
	roland_wavecard_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock,
			u32 min_size, u32 max_size, const char *size_error);

	virtual void device_start() override ATTR_COLD;

	virtual std::pair<std::error_condition, std::string> call_load() override ATTR_COLD;
	virtual void call_unload() override ATTR_COLD;

	virtual const char *file_extensions() const noexcept override { return "bin"; }
	virtual bool is_reset_on_load() const noexcept override { return true; }

private:
	void descramble() ATTR_COLD;

	required_address_space m_wave;
	const u32 m_min_size, m_max_size;
	const char *const m_size_error;
	offs_t m_base;
	u32 m_size;
	std::unique_ptr<u8 []> m_rom;
};


class srjv80_slot_device : public roland_wavecard_device
{
public:
	srjv80_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

protected:
	virtual const char *image_interface() const noexcept override { return "srjv80"; }
};


class sopcm1_slot_device : public roland_wavecard_device
{
public:
	sopcm1_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

protected:
	virtual const char *image_interface() const noexcept override { return "sopcm1"; }
	virtual const char *image_type_name() const noexcept override { return "pcmcard"; }
	virtual const char *image_brief_type_name() const noexcept override { return "pcm"; }
};

DECLARE_DEVICE_TYPE(SRJV80_SLOT, srjv80_slot_device)
DECLARE_DEVICE_TYPE(SOPCM1_SLOT, sopcm1_slot_device)

#endif // MAME_ROLAND_WAVECARD_H
