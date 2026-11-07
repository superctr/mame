// license:BSD-3-Clause
// copyright-holders:superctr
#ifndef MAME_MACHINE_I28F010_H
#define MAME_MACHINE_I28F010_H

#pragma once

class intel_28f010_device : public device_t, public device_nvram_interface
{
public:
	intel_28f010_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	u8 read(offs_t offset);
	void write(offs_t offset, u8 data);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	virtual void nvram_default() override;
	virtual bool nvram_read(util::read_stream &file) override;
	virtual bool nvram_write(util::write_stream &file) override;

private:
	enum : u8 { MODE_READ, MODE_ID, MODE_ERASE_SETUP, MODE_PROGRAM_SETUP };

	std::unique_ptr<u8[]> m_data;
	u8 m_mode;
	u8 m_reset_count;
};

DECLARE_DEVICE_TYPE(INTEL_28F010, intel_28f010_device)

#endif // MAME_MACHINE_I28F010_H
