// license:BSD-3-Clause
// copyright-holders:superctr
// thanks-to: giulioz
#ifndef MAME_SOUND_ROLAND_CSP_H
#define MAME_SOUND_ROLAND_CSP_H

#pragma once

class roland_csp_device : public cpu_device
{
public:
	static constexpr feature_type imperfect_features() { return feature::SOUND; }
	static constexpr unsigned PROGRAM_SIZE = 1024;
	static constexpr unsigned IRAM_SIZE = 512;
	static constexpr unsigned ERAM_SIZE = 0x40000;

	roland_csp_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	u8 host_r(offs_t offset);
	void host_w(offs_t offset, u8 data);
	void ser_w(unsigned channel, s32 sample) { m_serial_in[channel & 31] = narrow(sample); }
	s32 ser_r(unsigned channel) const { return m_serial_out[channel & 31]; }
	void sc_w(unsigned channel, s32 sample) { if (channel < 24) m_parallel_in[channel] = narrow(sample); }
	void set_eram_size(u32 words) { m_eram_mask = (words - 1) & (ERAM_SIZE - 1); }
	void run_once(unsigned slots = PROGRAM_SIZE);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual u64 execute_clocks_to_cycles(u64 clocks) const noexcept override { return (clocks + 1) / 2; }
	virtual u64 execute_cycles_to_clocks(u64 cycles) const noexcept override { return cycles * 2; }
	virtual u32 execute_min_cycles() const noexcept override { return 1; }
	virtual u32 execute_max_cycles() const noexcept override { return 1; }
	virtual void execute_run() override;
	virtual space_config_vector memory_space_config() const override;
	virtual std::unique_ptr<util::disasm_interface> create_disassembler() override;

	static s32 narrow(s64 value) { return s32(u32(value) << 8) >> 8; }
	static s32 accumulator(s64 value) { return s32(u32(value) << 2) >> 2; }
	static s32 saturate(s32 value) { return std::clamp<s32>(value, -0x800000, 0x7fffff); }
	void program_map(address_map &map) ATTR_COLD;
	void coefficient_map(address_map &map) ATTR_COLD;
	u64 program_r(offs_t offset);
	void program_w(offs_t offset, u64 data, u64 mem_mask = ~u64(0));
	u16 coefficient_r(offs_t offset);
	void coefficient_w(offs_t offset, u16 data, u16 mem_mask = 0xffff);
	void step();
	void finish_sample();
	void eram_clock(u32 word);
	s32 store(unsigned mode, unsigned offset, u16 coefficient);
	void multiply(unsigned opcode, unsigned shift, s32 operand, u16 coefficient);

	address_space_config m_program_config;
	address_space_config m_coefficient_config;
	std::unique_ptr<s32[]> m_eram;
	u32 m_program[PROGRAM_SIZE];
	u16 m_coefficients[PROGRAM_SIZE];
	s32 m_iram[IRAM_SIZE];
	s32 m_acc[2];
	s32 m_history[2][4];
	s32 m_multiplier[2];
	s32 m_tap;
	s32 m_serial_in[32];
	s32 m_serial_out[32];
	s32 m_parallel_in[24];
	u8 m_parallel_pos;
	u16 m_iram_pos;
	u32 m_eram_pos;
	u32 m_eram_mask = 0x1ffff;
	s32 m_eram_read;
	s32 m_eram_write;
	u16 m_eram_start[2];
	bool m_eram_active[2];
	u8 m_eram_command[2];
	u32 m_eram_address[2];
	s32 m_eram_data;
	u8 m_gate_age;
	bool m_gate_negative;
	u32 m_configuration;
	u8 m_enable;
	u32 m_host_read;
	u16 m_pc;
	u16 m_slot;
	bool m_halted;
	int m_icount;
};

DECLARE_DEVICE_TYPE(ROLAND_CSP, roland_csp_device)

#endif
