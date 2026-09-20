// license:BSD-3-Clause
// copyright-holders:Olivier Galibert, superctr
/***************************************************************************

    sh_itu.h

    SH 16-bit integrated timer pulse unit

***************************************************************************/

#ifndef MAME_CPU_SH_SH_ITU_H
#define MAME_CPU_SH_SH_ITU_H

#pragma once

class sh_mcu_device;
class sh_intc_device;

class sh_itu_channel_device : public device_t {
public:
	enum {
		INPUT_A,
		INPUT_B,
		INPUT_C,
		INPUT_D,
		DIV_1,
		DIV_2,
		DIV_4,
		DIV_8
	};

	enum {
		GR_CLEAR_NONE = -1,
		GR_CLEAR_SYNC = -2
	};

	enum {
		IRQ_A = 0x01,
		IRQ_B = 0x02,
		IRQ_V = 0x04
	};

	sh_itu_channel_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	template <typename T, typename U> sh_itu_channel_device(const machine_config &mconfig, const char *tag, device_t *owner, T &&cpu, U &&intc, int irq_base, int br_count)
		: sh_itu_channel_device(mconfig, tag, owner)
	{
		set_info(cpu, intc);
		m_interrupt[0] = irq_base;
		m_interrupt[1] = irq_base + 1;
		m_interrupt[2] = irq_base + 2;
		m_br_count = br_count;
	}

	template<typename T, typename U> void set_info(T &&cpu, U &&intc) { m_cpu.set_tag(std::forward<T>(cpu)); m_intc.set_tag(std::forward<U>(intc)); }

	u8 tcr_r();
	void tcr_w(u8 data);
	u8 tior_r();
	void tior_w(u8 data);
	u8 tier_r();
	void tier_w(u8 data);
	u8 tsr_r();
	void tsr_w(u8 data);
	u16 tcnt_r();
	void tcnt_w(offs_t, u16 data, u16 mem_mask);
	u16 gr_r(offs_t reg);
	void gr_w(offs_t reg, u16 data, u16 mem_mask);
	u16 br_r(offs_t reg);
	void br_w(offs_t reg, u16 data, u16 mem_mask);

	void set_enable(bool enable);
	u64 internal_update(u64 current_time);

protected:
	required_device<sh_mcu_device> m_cpu;
	required_device<sh_intc_device> m_intc;
	int m_interrupt[3];
	int m_br_count;

	int m_gr_clearing;
	u8 m_tcr, m_tior, m_tier, m_tsr;
	int m_clock_type, m_clock_divider;
	u16 m_tcnt;
	std::array<u16, 2> m_gr, m_br;
	u64 m_last_clock_update, m_event_time;
	u32 m_phase, m_counter_cycle;
	bool m_channel_active;

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	void update_counter(u64 cur_time = 0);
	void recalc_event(u64 cur_time = 0);
};

class sh_itu_device : public device_t {
public:
	sh_itu_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	template <typename T> sh_itu_device(const machine_config &mconfig, const char *tag, device_t *owner, T &&cpu)
		: sh_itu_device(mconfig, tag, owner)
	{
		m_cpu.set_tag(std::forward<T>(cpu));
	}

	u8 tstr_r();
	void tstr_w(u8 data);
	u8 tsnc_r();
	void tsnc_w(u8 data);
	u8 tmdr_r();
	void tmdr_w(u8 data);
	u8 tfcr_r();
	void tfcr_w(u8 data);
	u8 tocr_r();
	void tocr_w(u8 data);

protected:
	required_device<sh_mcu_device> m_cpu;
	required_device_array<sh_itu_channel_device, 5> m_timer_channel;

	u8 m_tstr, m_tsnc, m_tmdr, m_tfcr, m_tocr;

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
};

DECLARE_DEVICE_TYPE(SH_ITU, sh_itu_device)
DECLARE_DEVICE_TYPE(SH_ITU_CHANNEL, sh_itu_channel_device)

#endif // MAME_CPU_SH_SH_ITU_H
