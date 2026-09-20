// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
/***************************************************************************

    sh_dmac.h

    SH DMA controller

***************************************************************************/

#ifndef MAME_CPU_SH_SH_DMAC_H
#define MAME_CPU_SH_SH_DMAC_H

#pragma once

// To generalize eventually
class sh_mcu_device;
class sh_intc_device;

class sh_dmac_device : public device_t {
public:
	sh_dmac_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	template <typename T> sh_dmac_device(const machine_config &mconfig, const char *tag, device_t *owner, T &&cpu)
		: sh_dmac_device(mconfig, tag, owner)
	{
		set_info(cpu);
	}

	template<typename T> void set_info(T &&cpu) { m_cpu.set_tag(std::forward<T>(cpu)); }

	u16 dmaor_r();
	void dmaor_w(offs_t, u16 data, u16 mem_mask);

	// the master enable is set and neither the NMI nor the address error flag is
	bool transfer_allowed() const { return (m_dmaor & 7) == 1; }

protected:
	required_device<sh_mcu_device> m_cpu;

	u16 m_dmaor;

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
};

class sh_dmac_channel_device : public device_t {
public:
	sh_dmac_channel_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	template <typename T, typename U, typename V> sh_dmac_channel_device(const machine_config &mconfig, const char *tag, device_t *owner, T &&cpu, U &&intc, V &&dmac, int id)
		: sh_dmac_channel_device(mconfig, tag, owner)
	{
		set_info(cpu, intc, dmac, id);
	}

	template<typename T, typename U, typename V> void set_info(T &&cpu, U &&intc, V &&dmac, int id) {
		m_cpu.set_tag(std::forward<T>(cpu));
		m_intc.set_tag(std::forward<U>(intc));
		m_dmac.set_tag(std::forward<V>(dmac));
		m_id = id;
	}

	u32 sar_r();
	void sar_w(offs_t, u32 data, u32 mem_mask);
	u32 dar_r();
	void dar_w(offs_t, u32 data, u32 mem_mask);
	u32 dmatcr_r();
	void dmatcr_w(offs_t, u32 data, u32 mem_mask);
	u32 chcr_r();
	void chcr_w(offs_t, u32 data, u32 mem_mask);

	// the DREQ pin, 1 = request asserted
	void dreq_w(int state);

	// a change of the master enable
	void dmaor_changed();

protected:
	enum {
		RS_EXTERNAL_DUAL = 0,
		RS_EXTERNAL_SINGLE_TO_DEVICE = 2,
		RS_EXTERNAL_SINGLE_FROM_DEVICE = 3,
		RS_AUTO = 4
	};

	enum {
		CHCR_DE = 0x00001,
		CHCR_TE = 0x00002,
		CHCR_IE = 0x00004,
		CHCR_TM = 0x00020,
		CHCR_DS = 0x00040
	};

	required_device<sh_mcu_device> m_cpu;
	required_device<sh_intc_device> m_intc;
	required_device<sh_dmac_device> m_dmac;

	int m_id;
	u32 m_sar, m_dar, m_dmatcr, m_chcr;
	bool m_dreq;
	emu_timer *m_timer;

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	int resource() const { return BIT(m_chcr, 8, 4); }
	bool external() const { return resource() == RS_EXTERNAL_DUAL || resource() == RS_EXTERNAL_SINGLE_TO_DEVICE || resource() == RS_EXTERNAL_SINGLE_FROM_DEVICE; }
	bool armed() const;
	void transfer_unit();
	void request_check();
	TIMER_CALLBACK_MEMBER(tick);
};

DECLARE_DEVICE_TYPE(SH_DMAC, sh_dmac_device)
DECLARE_DEVICE_TYPE(SH_DMAC_CHANNEL, sh_dmac_channel_device)

#endif
