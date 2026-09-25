// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    armv7m.h

    ARMv7-M (M-profile Thumb-2) CPU core, and the Cortex-M3 built on it.

***************************************************************************/

#ifndef MAME_CPU_ARMV7M_ARMV7M_H
#define MAME_CPU_ARMV7M_ARMV7M_H

#pragma once

class drc_cache;
class drcuml_state;
class drcuml_block;
namespace uml { class code_handle; class parameter; }

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
	virtual void device_stop() override ATTR_COLD;

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

	int &icount() { return m_core->icount; }

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

	struct internal_state
	{
		u32 r[16];
		u32 pc;
		u32 n, z, c, v, q;
		u32 it;
		u32 tbit;
		int icount;
		u32 check_irq;
		int limit;
		u32 arg0, arg1, arg2;
		u32 status;
		u32 exit_pc;
		u32 vcount;
		u32 privmode;
		u32 code_lo, code_hi;
		u32 smc;
		u32 temp[16];
	};

	enum : int {
		SLEEP_NONE = 0, SLEEP_WFI, SLEEP_WFE
	};

	class frontend;
	class opcode_desc;
	struct compiler_state;
	struct verify_state;
	struct c_funcs;

	struct drc_deleter
	{
		void operator()(drc_cache *ptr) const;
		void operator()(drcuml_state *ptr) const;
		void operator()(frontend *ptr) const;
		void operator()(verify_state *ptr) const;
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
	internal_state *m_core;
	internal_state m_local_core;
	u32 m_sp_other;
	u16 m_ipsr;
	bool m_handler;
	u32 m_primask, m_faultmask, m_basepri, m_control;
	bool m_excl;
	bool m_event;
	int m_sleep;
	bool m_lockup;

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

	// recompiler
	bool m_drc;
	bool m_drc_dirty;
	int m_drc_hook;
	u64 m_drc_stamp;
	u64 m_drc_outside;
	std::unique_ptr<u8[]> m_drc_perm;
	std::vector<u32> m_drc_mpu;
	util::notifier_subscription m_drc_notifier;
	std::unique_ptr<drc_cache, drc_deleter> m_drccache;
	std::unique_ptr<drcuml_state, drc_deleter> m_drcuml;
	std::unique_ptr<frontend, drc_deleter> m_drcfe;
	std::unique_ptr<verify_state, drc_deleter> m_verify;
	uml::code_handle *m_entry;
	uml::code_handle *m_nocode;
	uml::code_handle *m_exit_pc;
	uml::code_handle *m_exit_pcset;
	uml::code_handle *m_exit_resolve;
	uml::code_handle *m_exit_interp;
	uml::code_handle *m_exit_branch;

	// registers and flags
	u32 reg(int n) const { return n == 15 ? m_core->pc + 4 : m_core->r[n]; }
	void set_reg(int n, u32 value) { m_core->r[n] = n == 13 ? value & ~3 : value; }
	bool in_it() const { return m_core->it & 0xf; }
	bool last_in_it() const { return (m_core->it & 0xf) == 0x8; }
	bool privileged() const { return m_handler || !(m_control & 1); }
	bool psp_active() const { return !m_handler && (m_control & 2); }
	u32 get_msp() const { return psp_active() ? m_sp_other : m_core->r[13]; }
	u32 get_psp() const { return psp_active() ? m_core->r[13] : m_sp_other; }
	void set_msp(u32 value);
	void set_psp(u32 value);
	void set_mode(bool handler, bool spsel);
	u32 apsr() const;
	u32 xpsr() const;
	void set_nzcvq(u32 value);
	bool condition(unsigned cond) const;
	void set_nz(u32 result) { m_core->n = result >> 31; m_core->z = result == 0; }
	u32 add_with_carry(u32 x, u32 y, u32 carry, bool setflags);
	static u32 thumb_expand_imm(u32 imm12);
	u32 thumb_expand_imm_c(u32 imm12, u32 &carry) const;
	u32 shift_c(u32 value, int type, unsigned amount, u32 &carry) const;
	u32 shift(u32 value, int type, unsigned amount) const { u32 c = m_core->c; return shift_c(value, type, amount, c); }
	void unpredictable() const;

	// program counter writes
	void branch_write_pc(u32 address) { m_next_pc = address & ~1; }
	void bx_write_pc(u32 address);
	void blx_write_pc(u32 address) { m_core->tbit = address & 1; m_next_pc = address & ~1; }

	// memory
	bool mem_read(u32 address, int size, u32 &value, u8 flags = 0);
	bool mem_write(u32 address, int size, u32 value, u8 flags = 0);
	bool access_aligned(u32 address, int size, u32 &value, bool write, int acctype);
	bool mpu_permits(u32 address, bool ifetch, bool write, bool priv, bool enabled) const;
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
	void check_irq() { m_core->check_irq = 1; m_prio_dirty = true; }
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
	void step_body();
	void step_resolve(u32 pc);
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

	// recompiler
	void drc_start() ATTR_COLD;
	void drc_stop() ATTR_COLD;
	void drc_flush();
	void execute_run_drc();
	u64 drc_stamp() const;
	void drc_sync();
	void drc_update_limit();
	u32 drc_cycles_to_event() const;
	bool drc_fetchable(u32 pc, bool priv) const;
	void drc_check_mpu();
	bool drc_mpu_unchanged() const;
	bool drc_can_continue();
	void drc_irq_check();
	void drc_interpret();
	void drc_bx_high();
	void drc_mem_read(int size, u8 flags);
	void drc_mem_write(int size, u8 flags);
	void drc_compile(u32 mode, offs_t pc);
	void drc_generate_invariant();
	void drc_generate_sequence(drcuml_block &block, compiler_state &compiler, const opcode_desc *seqhead, const opcode_desc *seqlast);
	void drc_generate_checksum(drcuml_block &block, const opcode_desc *seqhead, const opcode_desc *seqlast);
	bool drc_generate_instruction(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc);
	void drc_generate_interpret(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc);
	bool drc_generate_native(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc);
	bool drc_generate_t16(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc);
	bool drc_generate_t32(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc);
	void drc_generate_begin(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc);
	void drc_generate_end(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc);
	void drc_generate_condition(drcuml_block &block, unsigned cond, u32 false_label);
	void drc_generate_epilogue(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc, int cycles, bool check_irq);
	void drc_generate_goto(drcuml_block &block, compiler_state &compiler, u8 it, u32 pc);
	void drc_generate_branch(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc, u32 target, int cycles);
	void drc_generate_dynamic_branch(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc, int cycles, int kind);
	void drc_load_reg(drcuml_block &block, const uml::parameter &dst, int n, const opcode_desc *desc);
	void drc_generate_set_reg(drcuml_block &block, int d, const uml::parameter &value);
	void drc_generate_nz(drcuml_block &block, const uml::parameter &value);
	void drc_generate_flags(drcuml_block &block, bool subtract);
	void drc_generate_fast_check(drcuml_block &block, compiler_state &compiler, const uml::parameter &address, int size, bool write, u32 slow);
	void drc_generate_read(drcuml_block &block, compiler_state &compiler, int size, u8 flags);
	void drc_generate_write(drcuml_block &block, compiler_state &compiler, int size, u8 flags);
	void drc_generate_code_write_check(drcuml_block &block, compiler_state &compiler, const uml::parameter &address, u32 bytes);
	void drc_generate_ldm(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc, int n, u32 registers, bool wback, bool decrement);
	void drc_generate_stm(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc, int n, u32 registers, bool wback, bool decrement);
	void drc_generate_shift_reg(drcuml_block &block, int type, const uml::parameter &value, const uml::parameter &amount, int d, bool setflags);
	void drc_generate_shift_imm(drcuml_block &block, int type, unsigned amount, bool want_carry);
	static bool drc_dp_valid(unsigned opc, int d, int n, int m, bool setflags);
	void drc_generate_dp(drcuml_block &block, const opcode_desc *desc, unsigned opc, int d, int n, int carry, bool setflags);
	void drc_verify_begin();
	void drc_verify_end(int result);
	void drc_verify_log(bool write, u32 address, int size, u8 flags, u32 value);
	bool drc_memory_hook(bool write, u32 address, int size, u32 &value, u8 flags);
	std::string drc_disassemble(u32 pc, int count);
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
