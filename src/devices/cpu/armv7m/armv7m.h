// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    armv7m.h

    ARMv7-M (M-profile Thumb-2) CPU core, and the Cortex-M3 built on it.

***************************************************************************/

#ifndef MAME_CPU_ARMV7M_ARMV7M_H
#define MAME_CPU_ARMV7M_ARMV7M_H

#pragma once

class armv7m_device : public cpu_device
{
public:
	enum {
		ARMV7M_R0 = 0, ARMV7M_R1, ARMV7M_R2, ARMV7M_R3,
		ARMV7M_R4, ARMV7M_R5, ARMV7M_R6, ARMV7M_R7,
		ARMV7M_R8, ARMV7M_R9, ARMV7M_R10, ARMV7M_R11,
		ARMV7M_R12, ARMV7M_SP, ARMV7M_LR, ARMV7M_PC,
		ARMV7M_XPSR, ARMV7M_MSP, ARMV7M_PSP,
		ARMV7M_PRIMASK, ARMV7M_BASEPRI, ARMV7M_FAULTMASK, ARMV7M_CONTROL
	};

	// cycle counts follow the Cortex-M3 TRM table without bus wait states or pipelining
	static constexpr feature_type imperfect_features() { return feature::TIMING; }

	// configuration
	void set_num_irq(unsigned count) { m_num_irq = count; }
	void set_priority_bits(unsigned bits) { m_prio_bits = bits; }
	void set_mpu_regions(unsigned count) { m_mpu_regions = count; }
	void set_bitband(bool enable) { m_bitband = enable; }
	void set_vtor_reset(u32 value) { m_vtor_reset = value; }
	void set_systick_calib(u32 value) { m_systick_calib = value; }
	void set_systick_ref_divider(unsigned divider) { m_systick_ref_div = divider; }

	auto sysresetreq_cb() { return m_sysresetreq_cb.bind(); }
	auto lockup_cb() { return m_lockup_cb.bind(); }
	auto itm_cb() { return m_itm_cb.bind(); }

	// external interrupt inputs beyond the scheduler's input line range
	void set_irq_line(unsigned irq, int state);
	template <unsigned Irq> void irq_w(int state) { set_irq_line(Irq, state); }

protected:
	armv7m_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock, u32 cpuid, address_map_constructor internal_map = address_map_constructor());

	// device_t overrides
	virtual void device_resolve_objects() override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_post_load() override ATTR_COLD;

	// device_execute_interface overrides
	virtual u32 execute_min_cycles() const noexcept override { return 1; }
	virtual u32 execute_max_cycles() const noexcept override { return 32; }
	virtual void execute_run() override;
	virtual void execute_set_input(int inputnum, int state) override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	// device_state_interface overrides
	virtual void state_import(const device_state_entry &entry) override;
	virtual void state_export(const device_state_entry &entry) override;
	virtual void state_string_export(const device_state_entry &entry, std::string &str) const override;

	// device_disasm_interface overrides
	virtual std::unique_ptr<util::disasm_interface> create_disassembler() override;

private:
	enum : int {
		EXC_RESET = 1, EXC_NMI = 2, EXC_HARDFAULT = 3, EXC_MEMMANAGE = 4, EXC_BUSFAULT = 5,
		EXC_USAGEFAULT = 6, EXC_SVCALL = 11, EXC_DEBUGMONITOR = 12, EXC_PENDSV = 14, EXC_SYSTICK = 15
	};

	enum : u32 {
		CFSR_IACCVIOL = 1 << 0, CFSR_DACCVIOL = 1 << 1, CFSR_MUNSTKERR = 1 << 3, CFSR_MSTKERR = 1 << 4, CFSR_MMARVALID = 1 << 7,
		CFSR_IBUSERR = 1 << 8, CFSR_PRECISERR = 1 << 9, CFSR_IMPRECISERR = 1 << 10, CFSR_UNSTKERR = 1 << 11, CFSR_STKERR = 1 << 12, CFSR_BFARVALID = 1 << 15,
		CFSR_UNDEFINSTR = 1 << 16, CFSR_INVSTATE = 1 << 17, CFSR_INVPC = 1 << 18, CFSR_NOCP = 1 << 19, CFSR_UNALIGNED = 1 << 24, CFSR_DIVBYZERO = 1 << 25
	};

	enum : u32 {
		HFSR_VECTTBL = 1 << 1, HFSR_FORCED = 1 << 30, HFSR_DEBUGEVT = 1U << 31
	};

	enum : u32 {
		CCR_NONBASETHRDENA = 1 << 0, CCR_USERSETMPEND = 1 << 1, CCR_UNALIGN_TRP = 1 << 3, CCR_DIV_0_TRP = 1 << 4,
		CCR_BFHFNMIGN = 1 << 8, CCR_STKALIGN = 1 << 9
	};

	enum : int {
		ACC_NORMAL, ACC_UNPRIV, ACC_IFETCH, ACC_STACK, ACC_UNSTACK
	};

	enum : u8 {
		MEM_ALIGNED = 1, MEM_UNPRIV = 2
	};

	// configuration
	address_space_config m_program_config;
	u32 m_cpuid;
	unsigned m_num_irq;
	unsigned m_prio_bits;
	unsigned m_mpu_regions;
	bool m_bitband;
	u32 m_vtor_reset;
	u32 m_systick_calib;
	unsigned m_systick_ref_div;
	devcb_write_line m_sysresetreq_cb;
	devcb_write_line m_lockup_cb;
	devcb_write8 m_itm_cb;

	memory_access<32, 2, 0, ENDIANNESS_LITTLE>::cache m_cache;
	memory_access<32, 2, 0, ENDIANNESS_LITTLE>::specific m_program;

	// architectural state
	u32 m_r[16];
	u32 m_sp_other;
	u32 m_pc;
	u32 m_n, m_z, m_c, m_v, m_q;
	u8 m_it;
	u16 m_ipsr;
	bool m_tbit;
	bool m_handler;
	u32 m_primask, m_faultmask, m_basepri, m_control;
	bool m_excl;
	bool m_event;
	int m_sleep;
	bool m_lockup;
	int m_icount;

	// exception state
	u16 m_sys_pending;
	u16 m_sys_active;
	u8 m_shpr[12];
	unsigned m_irq_words;
	std::unique_ptr<u32[]> m_irq_enable;
	std::unique_ptr<u32[]> m_irq_pending;
	std::unique_ptr<u32[]> m_irq_active;
	std::unique_ptr<u32[]> m_irq_level;
	std::unique_ptr<u8[]> m_irq_prio;
	bool m_nmi_level;
	bool m_check_irq;
	bool m_prio_dirty;
	int m_exec_prio;
	u8 m_prio_mask;

	// system control block
	u32 m_vtor, m_prigroup, m_scr, m_ccr, m_actlr, m_shcsr_ena;
	u32 m_cfsr, m_hfsr, m_dfsr, m_fault_addr, m_afsr;

	// SysTick
	u32 m_syst_csr, m_syst_rvr, m_syst_cvr;
	u32 m_syst_prescale;

	// MPU
	u32 m_mpu_ctrl, m_mpu_rnr;
	std::unique_ptr<u32[]> m_mpu_rbar;
	std::unique_ptr<u32[]> m_mpu_rasr;

	// debug and trace blocks
	u32 m_demcr, m_dwt_ctrl, m_dwt_cyccnt, m_dwt_comp[12];
	u32 m_itm_ter, m_itm_tpr, m_itm_tcr;
	u32 m_fpb_ctrl, m_fpb_remap, m_fpb_comp[8];

	// per-instruction state
	u32 m_next_pc;
	u8 m_it_next;
	int m_fault_exc;
	int m_post_exc;
	u32 m_exc_return;
	bool m_exc_return_pending;
	int m_cycles;
	u32 m_xpsr_state, m_msp_state, m_psp_state, m_control_state;

	// registers and flags
	u32 reg(int n) const { return n == 15 ? m_pc + 4 : m_r[n]; }
	void set_reg(int n, u32 value) { m_r[n] = n == 13 ? value & ~3 : value; }
	bool in_it() const { return m_it & 0xf; }
	bool last_in_it() const { return (m_it & 0xf) == 0x8; }
	bool privileged() const { return m_handler || !(m_control & 1); }
	bool psp_active() const { return !m_handler && (m_control & 2); }
	u32 get_msp() const { return psp_active() ? m_sp_other : m_r[13]; }
	u32 get_psp() const { return psp_active() ? m_r[13] : m_sp_other; }
	void set_msp(u32 value);
	void set_psp(u32 value);
	void set_mode(bool handler, bool spsel);
	u32 apsr() const;
	u32 xpsr() const;
	void set_nzcvq(u32 value);
	bool condition(unsigned cond) const;
	void set_nz(u32 result) { m_n = result >> 31; m_z = result == 0; }
	u32 add_with_carry(u32 x, u32 y, u32 carry, bool setflags);
	static u32 thumb_expand_imm(u32 imm12);
	u32 thumb_expand_imm_c(u32 imm12, u32 &carry) const;
	u32 shift_c(u32 value, int type, unsigned amount, u32 &carry) const;
	u32 shift(u32 value, int type, unsigned amount) const { u32 c = m_c; return shift_c(value, type, amount, c); }
	void unpredictable() const;

	// program counter writes
	void branch_write_pc(u32 address) { m_next_pc = address & ~1; }
	void bx_write_pc(u32 address);
	void blx_write_pc(u32 address) { m_tbit = address & 1; m_next_pc = address & ~1; }

	// memory
	bool mem_read(u32 address, int size, u32 &value, u8 flags = 0);
	bool mem_write(u32 address, int size, u32 value, u8 flags = 0);
	bool access_aligned(u32 address, int size, u32 &value, bool write, int acctype);
	bool mpu_check(u32 address, int acctype, bool write, bool priv);
	u32 bus_read(u32 address, int size);
	void bus_write(u32 address, int size, u32 value);
	bool ppb_read(u32 address, int size, u32 &value, bool priv);
	bool ppb_write(u32 address, int size, u32 value, bool priv);
	u32 scs_read(u32 offset);
	void scs_write(u32 offset, u32 data, u32 mask);
	void fault(int exc, u32 cfsr_bits);
	void data_fault(int exc, u32 cfsr_bits, u32 address);

	// exceptions
	int group_priority(int prio) const { return prio < 0 ? prio : prio & ~((2 << m_prigroup) - 1); }
	int exception_priority(int exc) const;
	int execution_priority();
	int pending_exception(int &prio) const;
	bool exception_active(int exc) const;
	void set_exception_pending(int exc, bool state);
	void set_exception_active(int exc, bool state);
	unsigned active_count() const;
	bool fault_enabled(int exc) const;
	void check_irq() { m_check_irq = true; m_prio_dirty = true; }
	bool take_interrupt();
	void take_sync_exception(int exc, u32 return_address, u32 insn_address);
	void exception_entry(int exc, u32 return_address);
	bool push_stack(u32 return_address);
	void exception_taken(int exc);
	void exception_return(u32 exc_return);
	void deactivate(int exc);
	void enter_lockup(u32 address);
	void sample_irq_level(unsigned irq);

	// timers
	void systick_advance(u32 cycles);
	u32 systick_cycles_to_event() const;
	void advance_cycles(int cycles);

	// execution
	void step();
	void execute_t16(u16 op);
	void execute_t32(u32 op);
	void undefined();
	void nocp();
	void coprocessor(u32 op);
	void hint(unsigned op);

	void t16_shift_add_sub(u16 op);
	void t16_data_processing(u16 op);
	void t16_special(u16 op);
	void t16_load_store(u16 op);
	void t16_misc(u16 op);
	void t16_ldm_stm(u16 op);
	void t16_branch_svc(u16 op);

	void t32_ldm_stm(u32 op);
	void t32_dual_exclusive(u32 op);
	void t32_dp_shifted(u32 op);
	void t32_dp_modified_imm(u32 op);
	void t32_dp_plain_imm(u32 op);
	void t32_branch_misc(u32 op);
	void t32_store_single(u32 op);
	void t32_load(u32 op);
	void t32_dp_register(u32 op);
	void t32_multiply(u32 op);
	void t32_long_multiply(u32 op);
	void t32_msr(u32 op);
	void t32_mrs(u32 op);

	void dp_op(unsigned opc, int d, int n, int m, u32 op2, u32 carry, bool setflags);
	bool load(u32 address, int size, bool sign, u8 flags, u32 &value);
	void load_dest(int t, u32 value, u32 address);
	void store_exclusive(int d, int t, u32 address, int size);
	void load_multiple(int n, u32 registers, bool wback, bool decrement);
	void store_multiple(int n, u32 registers, bool wback, bool decrement);
	void do_svc();
	void do_bkpt();
	void do_wfi();
	void do_wfe();
};

class cortex_m3_device : public armv7m_device
{
public:
	cortex_m3_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	cortex_m3_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock, address_map_constructor internal_map);
};

DECLARE_DEVICE_TYPE(CORTEX_M3, cortex_m3_device)

#endif // MAME_CPU_ARMV7M_ARMV7M_H
