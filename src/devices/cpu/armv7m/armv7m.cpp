// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    armv7m.cpp

    ARMv7-M CPU core: the Thumb-2 instruction set without the DSP and
    floating-point extensions, the M-profile exception model, and the
    system components on the Private Peripheral Bus (NVIC, SCB, SysTick,
    MPU, and the DWT, ITM and FPB in part).

***************************************************************************/

#include "emu.h"
#include "armv7m.h"
#include "armv7mdasm.h"

#include <bit>

#define LOG_UNPREDICTABLE (1U << 1)
#define LOG_ITM           (1U << 2)
#define LOG_PPB           (1U << 3)
#define LOG_EXCEPTION     (1U << 4)

#define VERBOSE (0)
#include "logmacro.h"


DEFINE_DEVICE_TYPE(CORTEX_M3, cortex_m3_device, "cortex_m3", "ARM Cortex-M3")

namespace {

enum : u32 {
	SYST_ENABLE = 1 << 0, SYST_TICKINT = 1 << 1, SYST_CLKSOURCE = 1 << 2, SYST_COUNTFLAG = 1 << 16
};

enum : u32 {
	DEMCR_MON_EN = 1 << 16, DEMCR_MON_PEND = 1 << 17, DEMCR_TRCENA = 1 << 24
};

enum : u32 {
	SCR_SLEEPONEXIT = 1 << 1, SCR_SEVONPEND = 1 << 4
};

enum : int {
	SLEEP_NONE = 0, SLEEP_WFI, SLEEP_WFE
};

// Cortex-M3 r2p1 feature registers, 0xE000ED40-0xE000ED70
const u32 cm3_id_regs[13] = {
	0x00000030, 0x00000200, 0x00100000, 0x00000000,
	0x00100030, 0x00000000, 0x01000000, 0x00000000,
	0x01100110, 0x02111000, 0x21112231, 0x01111110,
	0x01310132
};

} // anonymous namespace


armv7m_device::armv7m_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock, u32 cpuid, address_map_constructor internal_map)
	: cpu_device(mconfig, type, tag, owner, clock)
	, m_program_config("program", ENDIANNESS_LITTLE, 32, 32, 0, internal_map)
	, m_cpuid(cpuid)
	, m_num_irq(240)
	, m_prio_bits(8)
	, m_mpu_regions(8)
	, m_bitband(true)
	, m_vtor_reset(0)
	, m_systick_calib(0xc0000000)
	, m_systick_ref_div(0)
	, m_sysresetreq_cb(*this)
	, m_lockup_cb(*this)
	, m_itm_cb(*this)
{
}

cortex_m3_device::cortex_m3_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: armv7m_device(mconfig, CORTEX_M3, tag, owner, clock, 0x412fc231)
{
}

cortex_m3_device::cortex_m3_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, u32 clock, address_map_constructor internal_map)
	: armv7m_device(mconfig, type, tag, owner, clock, 0x412fc231, internal_map)
{
}

device_memory_interface::space_config_vector armv7m_device::memory_space_config() const
{
	return space_config_vector {
		std::make_pair(AS_PROGRAM, &m_program_config)
	};
}

std::unique_ptr<util::disasm_interface> armv7m_device::create_disassembler()
{
	return std::make_unique<armv7m_disassembler>();
}

void armv7m_device::device_resolve_objects()
{
	m_num_irq = std::clamp(m_num_irq, 1U, 496U);
	m_prio_bits = std::clamp(m_prio_bits, 3U, 8U);
	m_mpu_regions = std::min(m_mpu_regions, 16U);
}

void armv7m_device::device_start()
{
	space(AS_PROGRAM).cache(m_cache);
	space(AS_PROGRAM).specific(m_program);

	m_irq_words = (m_num_irq + 31) / 32;
	m_irq_enable = std::make_unique<u32[]>(m_irq_words);
	m_irq_pending = std::make_unique<u32[]>(m_irq_words);
	m_irq_active = std::make_unique<u32[]>(m_irq_words);
	m_irq_level = std::make_unique<u32[]>(m_irq_words);
	m_irq_prio = std::make_unique<u8[]>(m_num_irq);
	m_mpu_rbar = std::make_unique<u32[]>(std::max(m_mpu_regions, 1U));
	m_mpu_rasr = std::make_unique<u32[]>(std::max(m_mpu_regions, 1U));
	m_prio_mask = u8(0xff << (8 - m_prio_bits));

	std::fill_n(m_r, 16, 0);
	m_sp_other = 0;
	m_pc = 0;
	m_n = m_z = m_c = m_v = m_q = 0;
	m_it = 0;
	m_ipsr = 0;
	m_tbit = true;
	m_handler = false;
	m_primask = m_faultmask = m_basepri = m_control = 0;
	m_excl = false;
	m_event = false;
	m_sleep = SLEEP_NONE;
	m_lockup = false;
	m_icount = 0;
	m_sys_pending = m_sys_active = 0;
	std::fill_n(m_shpr, 12, 0);
	std::fill_n(&m_irq_enable[0], m_irq_words, 0);
	std::fill_n(&m_irq_pending[0], m_irq_words, 0);
	std::fill_n(&m_irq_active[0], m_irq_words, 0);
	std::fill_n(&m_irq_level[0], m_irq_words, 0);
	std::fill_n(&m_irq_prio[0], m_num_irq, 0);
	m_nmi_level = false;
	m_check_irq = true;
	m_prio_dirty = true;
	m_exec_prio = 256;
	m_vtor = m_prigroup = m_scr = m_ccr = m_actlr = m_shcsr_ena = 0;
	m_cfsr = m_hfsr = m_dfsr = m_fault_addr = m_afsr = 0;
	m_syst_csr = m_syst_rvr = m_syst_cvr = m_syst_prescale = 0;
	m_mpu_ctrl = m_mpu_rnr = 0;
	std::fill_n(&m_mpu_rbar[0], std::max(m_mpu_regions, 1U), 0);
	std::fill_n(&m_mpu_rasr[0], std::max(m_mpu_regions, 1U), 0);
	m_demcr = m_dwt_ctrl = m_dwt_cyccnt = 0;
	std::fill_n(m_dwt_comp, 12, 0);
	m_itm_ter = m_itm_tpr = m_itm_tcr = 0;
	m_fpb_ctrl = m_fpb_remap = 0;
	std::fill_n(m_fpb_comp, 8, 0);
	m_next_pc = 0;
	m_it_next = 0;
	m_fault_exc = 0;
	m_post_exc = 0;
	m_exc_return = 0;
	m_exc_return_pending = false;
	m_cycles = 0;
	m_xpsr_state = m_msp_state = m_psp_state = m_control_state = 0;

	state_add(STATE_GENPC, "GENPC", m_pc).callimport().noshow();
	state_add(STATE_GENPCBASE, "CURPC", m_pc).callimport().noshow();
	state_add(STATE_GENFLAGS, "GENFLAGS", m_xpsr_state).formatstr("%13s").noshow();
	static const char *const regnames[16] = {
		"R0", "R1", "R2", "R3", "R4", "R5", "R6", "R7",
		"R8", "R9", "R10", "R11", "R12", "SP", "LR", "PC"
	};
	for (int i = 0; i < 15; i++)
		state_add(ARMV7M_R0 + i, regnames[i], m_r[i]).callimport();
	state_add(ARMV7M_PC, "PC", m_pc).callimport();
	state_add(ARMV7M_XPSR, "XPSR", m_xpsr_state).callimport().callexport();
	state_add(ARMV7M_MSP, "MSP", m_msp_state).callimport().callexport();
	state_add(ARMV7M_PSP, "PSP", m_psp_state).callimport().callexport();
	state_add(ARMV7M_PRIMASK, "PRIMASK", m_primask).mask(1).callimport();
	state_add(ARMV7M_BASEPRI, "BASEPRI", m_basepri).mask(0xff).callimport();
	state_add(ARMV7M_FAULTMASK, "FAULTMASK", m_faultmask).mask(1).callimport();
	state_add(ARMV7M_CONTROL, "CONTROL", m_control_state).mask(3).callimport().callexport();

	save_item(NAME(m_r));
	save_item(NAME(m_sp_other));
	save_item(NAME(m_pc));
	save_item(NAME(m_n));
	save_item(NAME(m_z));
	save_item(NAME(m_c));
	save_item(NAME(m_v));
	save_item(NAME(m_q));
	save_item(NAME(m_it));
	save_item(NAME(m_ipsr));
	save_item(NAME(m_tbit));
	save_item(NAME(m_handler));
	save_item(NAME(m_primask));
	save_item(NAME(m_faultmask));
	save_item(NAME(m_basepri));
	save_item(NAME(m_control));
	save_item(NAME(m_excl));
	save_item(NAME(m_event));
	save_item(NAME(m_sleep));
	save_item(NAME(m_lockup));
	save_item(NAME(m_sys_pending));
	save_item(NAME(m_sys_active));
	save_item(NAME(m_shpr));
	save_pointer(NAME(m_irq_enable), m_irq_words);
	save_pointer(NAME(m_irq_pending), m_irq_words);
	save_pointer(NAME(m_irq_active), m_irq_words);
	save_pointer(NAME(m_irq_level), m_irq_words);
	save_pointer(NAME(m_irq_prio), m_num_irq);
	save_item(NAME(m_nmi_level));
	save_item(NAME(m_vtor));
	save_item(NAME(m_prigroup));
	save_item(NAME(m_scr));
	save_item(NAME(m_ccr));
	save_item(NAME(m_actlr));
	save_item(NAME(m_shcsr_ena));
	save_item(NAME(m_cfsr));
	save_item(NAME(m_hfsr));
	save_item(NAME(m_dfsr));
	save_item(NAME(m_fault_addr));
	save_item(NAME(m_afsr));
	save_item(NAME(m_syst_csr));
	save_item(NAME(m_syst_rvr));
	save_item(NAME(m_syst_cvr));
	save_item(NAME(m_syst_prescale));
	save_item(NAME(m_mpu_ctrl));
	save_item(NAME(m_mpu_rnr));
	save_pointer(NAME(m_mpu_rbar), std::max(m_mpu_regions, 1U));
	save_pointer(NAME(m_mpu_rasr), std::max(m_mpu_regions, 1U));
	save_item(NAME(m_demcr));
	save_item(NAME(m_dwt_ctrl));
	save_item(NAME(m_dwt_cyccnt));
	save_item(NAME(m_dwt_comp));
	save_item(NAME(m_itm_ter));
	save_item(NAME(m_itm_tpr));
	save_item(NAME(m_itm_tcr));
	save_item(NAME(m_fpb_ctrl));
	save_item(NAME(m_fpb_remap));
	save_item(NAME(m_fpb_comp));

	set_icountptr(m_icount);
}

void armv7m_device::device_post_load()
{
	check_irq();
}

void armv7m_device::device_reset()
{
	m_handler = false;
	m_primask = m_faultmask = m_basepri = 0;
	m_control = 0;
	m_sys_pending = m_sys_active = 0;
	std::fill_n(m_shpr, 12, 0);
	std::fill_n(&m_irq_enable[0], m_irq_words, 0);
	std::fill_n(&m_irq_pending[0], m_irq_words, 0);
	std::fill_n(&m_irq_active[0], m_irq_words, 0);
	std::fill_n(&m_irq_prio[0], m_num_irq, 0);
	for (unsigned i = 0; i < m_irq_words; i++)
		m_irq_pending[i] = m_irq_level[i];
	m_vtor = m_vtor_reset & ~0x7f;
	m_prigroup = 0;
	m_scr = 0;
	m_ccr = CCR_STKALIGN;
	m_actlr = 0;
	m_shcsr_ena = 0;
	m_cfsr = m_hfsr = m_dfsr = m_afsr = 0;
	m_syst_csr = 0;
	m_syst_prescale = 0;
	m_mpu_ctrl = m_mpu_rnr = 0;
	std::fill_n(&m_mpu_rbar[0], std::max(m_mpu_regions, 1U), 0);
	std::fill_n(&m_mpu_rasr[0], std::max(m_mpu_regions, 1U), 0);
	m_demcr = 0;
	m_dwt_ctrl = 0;
	m_dwt_cyccnt = 0;
	m_itm_ter = m_itm_tpr = m_itm_tcr = 0;
	m_fpb_ctrl = 0;
	m_excl = false;
	m_event = false;
	m_sleep = SLEEP_NONE;
	if (m_lockup)
		m_lockup_cb(CLEAR_LINE);
	m_lockup = false;
	m_it = 0;
	m_ipsr = 0;
	m_fault_exc = 0;
	m_post_exc = 0;
	m_exc_return_pending = false;

	m_r[13] = m_program.read_dword(m_vtor) & ~3;
	m_sp_other = 0;
	m_r[14] = 0xffffffff;
	const u32 entry = m_program.read_dword(m_vtor + 4);
	m_tbit = entry & 1;
	m_pc = entry & ~1;
	check_irq();
}


//-------------------------------------------------
//  debugger state
//-------------------------------------------------

void armv7m_device::state_import(const device_state_entry &entry)
{
	switch (entry.index())
	{
	case STATE_GENPC:
	case STATE_GENPCBASE:
	case ARMV7M_PC:
		m_pc &= ~1;
		break;
	case ARMV7M_SP:
		m_r[13] &= ~3;
		break;
	case ARMV7M_XPSR:
		set_nzcvq(m_xpsr_state);
		m_ipsr = m_xpsr_state & 0x1ff;
		m_tbit = BIT(m_xpsr_state, 24);
		m_it = ((m_xpsr_state >> 25) & 3) | ((m_xpsr_state >> 8) & 0xfc);
		break;
	case ARMV7M_MSP:
		set_msp(m_msp_state);
		break;
	case ARMV7M_PSP:
		set_psp(m_psp_state);
		break;
	case ARMV7M_CONTROL:
		m_control = (m_control & ~1) | (m_control_state & 1);
		if (!m_handler)
			set_mode(false, m_control_state & 2);
		break;
	case ARMV7M_BASEPRI:
		m_basepri &= m_prio_mask;
		check_irq();
		break;
	case ARMV7M_PRIMASK:
	case ARMV7M_FAULTMASK:
		check_irq();
		break;
	}
}

void armv7m_device::state_export(const device_state_entry &entry)
{
	switch (entry.index())
	{
	case STATE_GENFLAGS:
	case ARMV7M_XPSR:
		m_xpsr_state = xpsr();
		break;
	case ARMV7M_MSP:
		m_msp_state = get_msp();
		break;
	case ARMV7M_PSP:
		m_psp_state = get_psp();
		break;
	case ARMV7M_CONTROL:
		m_control_state = m_control;
		break;
	}
}

void armv7m_device::state_string_export(const device_state_entry &entry, std::string &str) const
{
	switch (entry.index())
	{
	case STATE_GENFLAGS:
		str = util::string_format("%c%c%c%c%c %s%s IT%02X",
				m_n ? 'N' : '.', m_z ? 'Z' : '.', m_c ? 'C' : '.', m_v ? 'V' : '.', m_q ? 'Q' : '.',
				m_handler ? "H" : "T", m_tbit ? "t" : "a", m_it);
		break;
	}
}


//-------------------------------------------------
//  registers, flags and operand helpers
//-------------------------------------------------

void armv7m_device::set_msp(u32 value)
{
	if (psp_active())
		m_sp_other = value & ~3;
	else
		m_r[13] = value & ~3;
}

void armv7m_device::set_psp(u32 value)
{
	if (psp_active())
		m_r[13] = value & ~3;
	else
		m_sp_other = value & ~3;
}

void armv7m_device::set_mode(bool handler, bool spsel)
{
	const bool old_psp = psp_active();
	m_handler = handler;
	m_control = (m_control & ~2) | (spsel ? 2 : 0);
	if (psp_active() != old_psp)
		std::swap(m_r[13], m_sp_other);
}

u32 armv7m_device::apsr() const
{
	return (m_n << 31) | (m_z << 30) | (m_c << 29) | (m_v << 28) | (m_q << 27);
}

u32 armv7m_device::xpsr() const
{
	return apsr() | (u32(m_tbit) << 24) | (u32(m_it & 3) << 25) | (u32(m_it & 0xfc) << 8) | m_ipsr;
}

void armv7m_device::set_nzcvq(u32 value)
{
	m_n = BIT(value, 31);
	m_z = BIT(value, 30);
	m_c = BIT(value, 29);
	m_v = BIT(value, 28);
	m_q = BIT(value, 27);
}

bool armv7m_device::condition(unsigned cond) const
{
	bool result;
	switch (cond >> 1)
	{
	case 0: result = m_z; break;
	case 1: result = m_c; break;
	case 2: result = m_n; break;
	case 3: result = m_v; break;
	case 4: result = m_c && !m_z; break;
	case 5: result = m_n == m_v; break;
	case 6: result = (m_n == m_v) && !m_z; break;
	default: return true;
	}
	return (cond & 1) ? !result : result;
}

u32 armv7m_device::add_with_carry(u32 x, u32 y, u32 carry, bool setflags)
{
	const u64 sum = u64(x) + u64(y) + carry;
	const u32 result = u32(sum);
	if (setflags)
	{
		m_n = result >> 31;
		m_z = result == 0;
		m_c = u32(sum >> 32);
		m_v = ((x ^ result) & (y ^ result)) >> 31;
	}
	return result;
}

u32 armv7m_device::thumb_expand_imm(u32 imm12)
{
	if (!(imm12 & 0xc00))
	{
		const u32 imm8 = imm12 & 0xff;
		switch ((imm12 >> 8) & 3)
		{
		case 0: return imm8;
		case 1: return imm8 * 0x00010001;
		case 2: return imm8 * 0x01000100;
		default: return imm8 * 0x01010101;
		}
	}
	return std::rotr<u32>(0x80 | (imm12 & 0x7f), (imm12 >> 7) & 0x1f);
}

u32 armv7m_device::thumb_expand_imm_c(u32 imm12, u32 &carry) const
{
	const u32 value = thumb_expand_imm(imm12);
	if (!(imm12 & 0xc00))
	{
		if ((imm12 & 0x300) && !(imm12 & 0xff))
			unpredictable();
		carry = m_c;
	}
	else
		carry = value >> 31;
	return value;
}

u32 armv7m_device::shift_c(u32 value, int type, unsigned amount, u32 &carry) const
{
	if (type == 4)
	{
		const u32 result = (carry << 31) | (value >> 1);
		carry = value & 1;
		return result;
	}
	if (amount == 0)
		return value;
	switch (type)
	{
	case 0:
		if (amount > 32)
		{
			carry = 0;
			return 0;
		}
		carry = (value >> (32 - amount)) & 1;
		return amount == 32 ? 0 : value << amount;
	case 1:
		if (amount > 32)
		{
			carry = 0;
			return 0;
		}
		carry = (value >> (amount - 1)) & 1;
		return amount == 32 ? 0 : value >> amount;
	case 2:
		if (amount >= 32)
		{
			carry = value >> 31;
			return u32(s32(value) >> 31);
		}
		carry = (value >> (amount - 1)) & 1;
		return u32(s32(value) >> amount);
	default:
		{
			const u32 result = std::rotr<u32>(value, amount & 31);
			carry = result >> 31;
			return result;
		}
	}
}

void armv7m_device::unpredictable() const
{
	LOGMASKED(LOG_UNPREDICTABLE, "%08x: unpredictable instruction\n", m_pc);
}

void armv7m_device::bx_write_pc(u32 address)
{
	if (m_handler && (address >> 28) == 0xf)
	{
		m_exc_return = address;
		m_exc_return_pending = true;
	}
	else
	{
		m_tbit = address & 1;
		m_next_pc = address & ~1;
	}
}


//-------------------------------------------------
//  memory system
//-------------------------------------------------

void armv7m_device::fault(int exc, u32 cfsr_bits)
{
	m_cfsr |= cfsr_bits;
	if (!m_fault_exc)
		m_fault_exc = exc;
}

void armv7m_device::data_fault(int exc, u32 cfsr_bits, u32 address)
{
	if (exc == EXC_MEMMANAGE)
	{
		m_cfsr = (m_cfsr & ~CFSR_BFARVALID) | CFSR_MMARVALID;
		m_fault_addr = address;
	}
	else
	{
		m_cfsr = (m_cfsr & ~CFSR_MMARVALID) | CFSR_BFARVALID;
		m_fault_addr = address;
	}
	fault(exc, cfsr_bits);
}

bool armv7m_device::mpu_check(u32 address, int acctype, bool write, bool priv)
{
	const bool ifetch = acctype == ACC_IFETCH;
	if ((address >> 20) == 0xe00)
	{
		if (ifetch)
			fault(EXC_MEMMANAGE, CFSR_IACCVIOL);
		return !ifetch;
	}

	if (!(m_mpu_ctrl & 1) || (!(m_mpu_ctrl & 2) && execution_priority() < 0))
	{
		if (ifetch && BIT(0xe4, address >> 29))
		{
			fault(EXC_MEMMANAGE, CFSR_IACCVIOL);
			return false;
		}
		return true;
	}

	bool hit = false;
	u32 ap = 0;
	bool xn = false;
	if ((m_mpu_ctrl & 4) && priv)
	{
		hit = true;
		ap = 3;
		xn = BIT(0xe4, address >> 29);
	}
	for (unsigned r = 0; r < m_mpu_regions; r++)
	{
		const u32 rasr = m_mpu_rasr[r];
		if (!(rasr & 1))
			continue;
		unsigned lsbit = ((rasr >> 1) & 0x1f) + 1;
		if (lsbit < 5)
			lsbit = 5;
		if (lsbit != 32 && ((address ^ m_mpu_rbar[r]) >> lsbit))
			continue;
		if (lsbit >= 8 && BIT(rasr, 8 + ((address >> (lsbit - 3)) & 7)))
			continue;
		hit = true;
		ap = (rasr >> 24) & 7;
		xn = BIT(rasr, 28);
	}
	if ((address >> 29) == 7)
		xn = true;

	bool denied = !hit;
	if (hit)
	{
		switch (ap)
		{
		case 0: denied = true; break;
		case 1: denied = !priv; break;
		case 2: denied = !priv && write; break;
		case 3: denied = false; break;
		case 5: denied = !priv || write; break;
		case 6: case 7: denied = write; break;
		default: unpredictable(); denied = true; break;
		}
		if (ifetch && xn)
			denied = true;
	}
	if (!denied)
		return true;

	switch (acctype)
	{
	case ACC_IFETCH:
		fault(EXC_MEMMANAGE, CFSR_IACCVIOL);
		break;
	case ACC_STACK:
		fault(EXC_MEMMANAGE, CFSR_MSTKERR);
		break;
	case ACC_UNSTACK:
		fault(EXC_MEMMANAGE, CFSR_MUNSTKERR);
		break;
	default:
		data_fault(EXC_MEMMANAGE, CFSR_DACCVIOL, address);
		break;
	}
	return false;
}

u32 armv7m_device::bus_read(u32 address, int size)
{
	switch (size)
	{
	case 1: return m_program.read_byte(address);
	case 2: return m_program.read_word(address);
	default: return m_program.read_dword(address);
	}
}

void armv7m_device::bus_write(u32 address, int size, u32 value)
{
	switch (size)
	{
	case 1: m_program.write_byte(address, value); break;
	case 2: m_program.write_word(address, value); break;
	default: m_program.write_dword(address, value); break;
	}
}

bool armv7m_device::access_aligned(u32 address, int size, u32 &value, bool write, int acctype)
{
	const bool priv = acctype != ACC_UNPRIV && privileged();
	if (!mpu_check(address, acctype, write, priv))
		return false;

	if ((address >> 18) == (0xe0000000 >> 18))
	{
		bool ok = write ? ppb_write(address, size, value, priv) : ppb_read(address, size, value, priv);
		if (!ok)
		{
			if (acctype == ACC_STACK)
				fault(EXC_BUSFAULT, CFSR_STKERR);
			else if (acctype == ACC_UNSTACK)
				fault(EXC_BUSFAULT, CFSR_UNSTKERR);
			else
				data_fault(EXC_BUSFAULT, CFSR_PRECISERR, address);
		}
		return ok;
	}

	if (m_bitband && ((address >> 25) == 0x11 || (address >> 25) == 0x21))
	{
		const u32 byte = (address & 0xf0000000) | ((address & 0x01ffffff) >> 5);
		const u32 container = byte & ~u32(size - 1);
		const unsigned bit = ((address >> 2) & 7) + 8 * (byte - container);
		const u32 data = bus_read(container, size);
		if (write)
			bus_write(container, size, (data & ~(1U << bit)) | ((value & 1) << bit));
		else
			value = (data >> bit) & 1;
		return true;
	}

	if (write)
		bus_write(address, size, value);
	else
		value = bus_read(address, size);
	return true;
}

bool armv7m_device::mem_read(u32 address, int size, u32 &value, u8 flags)
{
	const int acctype = (flags & MEM_UNPRIV) ? ACC_UNPRIV : ACC_NORMAL;
	if (!(address & (size - 1)))
		return access_aligned(address, size, value, false, acctype);
	if ((flags & MEM_ALIGNED) || (m_ccr & CCR_UNALIGN_TRP))
	{
		fault(EXC_USAGEFAULT, CFSR_UNALIGNED);
		return false;
	}
	u32 result = 0;
	for (int i = 0; i < size; i++)
	{
		u32 byte;
		if (!access_aligned(address + i, 1, byte, false, acctype))
			return false;
		result |= (byte & 0xff) << (8 * i);
	}
	value = result;
	return true;
}

bool armv7m_device::mem_write(u32 address, int size, u32 value, u8 flags)
{
	const int acctype = (flags & MEM_UNPRIV) ? ACC_UNPRIV : ACC_NORMAL;
	if (size < 4)
		value &= (1U << (8 * size)) - 1;
	if (!(address & (size - 1)))
		return access_aligned(address, size, value, true, acctype);
	if ((flags & MEM_ALIGNED) || (m_ccr & CCR_UNALIGN_TRP))
	{
		fault(EXC_USAGEFAULT, CFSR_UNALIGNED);
		return false;
	}
	for (int i = 0; i < size; i++)
	{
		u32 byte = (value >> (8 * i)) & 0xff;
		if (!access_aligned(address + i, 1, byte, true, acctype))
			return false;
	}
	return true;
}


//-------------------------------------------------
//  private peripheral bus
//-------------------------------------------------

bool armv7m_device::ppb_read(u32 address, int size, u32 &value, bool priv)
{
	const u32 offset = address & 0x3fffc;
	const unsigned shift = 8 * (address & 3);
	u32 data = 0;

	switch (offset >> 12)
	{
	case 0x00: // ITM
		if (offset < 0x80)
		{
			if (!priv && BIT(m_itm_tpr, offset >> 5))
				return false;
			data = (m_itm_tcr & 1) ? 1 : 0;
		}
		else
		{
			switch (offset)
			{
			case 0xe00: data = m_itm_ter; break;
			case 0xe40: data = m_itm_tpr; break;
			case 0xe80: data = m_itm_tcr; break;
			}
		}
		break;

	case 0x01: // DWT
		if (!priv)
			return false;
		switch (offset & 0xfff)
		{
		case 0x000: data = 0x40000000 | m_dwt_ctrl; break;
		case 0x004: data = m_dwt_cyccnt; break;
		case 0x01c: data = m_pc; break;
		default:
			if ((offset & 0xfff) >= 0x020 && (offset & 0xfff) < 0x060 && (offset & 0xc) != 0xc)
				data = m_dwt_comp[(((offset & 0xfff) - 0x20) >> 4) * 3 + ((offset >> 2) & 3)];
			break;
		}
		break;

	case 0x02: // FPB
		if (!priv)
			return false;
		switch (offset & 0xfff)
		{
		case 0x000: data = 0x00000260 | (m_fpb_ctrl & 1); break;
		case 0x004: data = m_fpb_remap; break;
		default:
			if ((offset & 0xfff) >= 0x008 && (offset & 0xfff) < 0x028)
				data = m_fpb_comp[((offset & 0xfff) - 8) >> 2];
			break;
		}
		break;

	case 0x0e: // SCS
		if (!priv)
			return false;
		data = scs_read(offset & 0xfff);
		break;

	default:
		if (!priv)
			return false;
		LOGMASKED(LOG_PPB, "%08x: read from unmapped PPB address %08x\n", m_pc, address);
		break;
	}

	value = data >> shift;
	if (size < 4)
		value &= (1U << (8 * size)) - 1;
	return true;
}

bool armv7m_device::ppb_write(u32 address, int size, u32 value, bool priv)
{
	const u32 offset = address & 0x3fffc;
	const unsigned shift = 8 * (address & 3);
	const u32 mask = (size == 4 ? 0xffffffff : ((1U << (8 * size)) - 1)) << shift;
	const u32 data = value << shift;

	switch (offset >> 12)
	{
	case 0x00: // ITM
		if (offset < 0x80)
		{
			if (!priv && BIT(m_itm_tpr, offset >> 5))
				return false;
			if ((m_itm_tcr & 1) && BIT(m_itm_ter, offset >> 2))
			{
				LOGMASKED(LOG_ITM, "ITM port %d: %02x\n", offset >> 2, value & 0xff);
				m_itm_cb(offset >> 2, value & 0xff);
			}
			return true;
		}
		if (!priv)
			return false;
		switch (offset)
		{
		case 0xe00: m_itm_ter = (m_itm_ter & ~mask) | (data & mask); break;
		case 0xe40: m_itm_tpr = ((m_itm_tpr & ~mask) | (data & mask)) & 0xf; break;
		case 0xe80: m_itm_tcr = ((m_itm_tcr & ~mask) | (data & mask)) & 0x007f031f; break;
		}
		return true;

	case 0x01: // DWT
		if (!priv)
			return false;
		switch (offset & 0xfff)
		{
		case 0x000: m_dwt_ctrl = ((m_dwt_ctrl & ~mask) | (data & mask)) & 0x0fffffff; break;
		case 0x004: m_dwt_cyccnt = (m_dwt_cyccnt & ~mask) | (data & mask); break;
		default:
			if ((offset & 0xfff) >= 0x020 && (offset & 0xfff) < 0x060 && (offset & 0xc) != 0xc)
			{
				u32 &reg = m_dwt_comp[(((offset & 0xfff) - 0x20) >> 4) * 3 + ((offset >> 2) & 3)];
				reg = (reg & ~mask) | (data & mask);
			}
			break;
		}
		return true;

	case 0x02: // FPB
		if (!priv)
			return false;
		// the FPB keeps its registers but does not remap or break
		switch (offset & 0xfff)
		{
		case 0x000:
			if (data & mask & 2)
				m_fpb_ctrl = data & 1;
			break;
		case 0x004: m_fpb_remap = ((m_fpb_remap & ~mask) | (data & mask)) & 0x1fffffe0; break;
		default:
			if ((offset & 0xfff) >= 0x008 && (offset & 0xfff) < 0x028)
			{
				u32 &reg = m_fpb_comp[((offset & 0xfff) - 8) >> 2];
				reg = (reg & ~mask) | (data & mask);
			}
			break;
		}
		return true;

	case 0x0e: // SCS
		if (!priv && !((offset & 0xfff) == 0xf00 && (m_ccr & CCR_USERSETMPEND)))
			return false;
		scs_write(offset & 0xfff, data, mask);
		return true;

	default:
		if (!priv)
			return false;
		LOGMASKED(LOG_PPB, "%08x: write to unmapped PPB address %08x = %08x\n", m_pc, address, value);
		return true;
	}
}

u32 armv7m_device::scs_read(u32 offset)
{
	if (offset >= 0x100 && offset < 0x340)
	{
		const unsigned word = (offset & 0x7f) >> 2;
		if (word >= m_irq_words)
			return 0;
		switch (offset >> 7)
		{
		case 2: case 3: return m_irq_enable[word];
		case 4: case 5: return m_irq_pending[word];
		case 6: return m_irq_active[word];
		default: return 0;
		}
	}
	if (offset >= 0x400 && offset < 0x5f0)
	{
		const unsigned irq = offset - 0x400;
		u32 data = 0;
		for (unsigned i = 0; i < 4; i++)
			if (irq + i < m_num_irq)
				data |= u32(m_irq_prio[irq + i]) << (8 * i);
		return data;
	}
	if (offset >= 0xd40 && offset <= 0xd70)
		return cm3_id_regs[(offset - 0xd40) >> 2];

	switch (offset)
	{
	case 0x004:
		return (m_num_irq - 1) / 32;
	case 0x008:
		return m_actlr;

	case 0x010:
		{
			u32 data = m_syst_csr;
			if (BIT(m_systick_calib, 31))
				data |= SYST_CLKSOURCE;
			if (!machine().side_effects_disabled())
				m_syst_csr &= ~SYST_COUNTFLAG;
			return data;
		}
	case 0x014:
		return m_syst_rvr;
	case 0x018:
		return m_syst_cvr;
	case 0x01c:
		return m_systick_calib;

	case 0xd00:
		return m_cpuid;
	case 0xd04:
		{
			int prio;
			const int pending = pending_exception(prio);
			bool isr_pending = false;
			for (unsigned i = 0; i < m_irq_words; i++)
				if (m_irq_pending[i])
					isr_pending = true;
			u32 data = m_ipsr;
			if (BIT(m_sys_pending, EXC_NMI))
				data |= 1U << 31;
			if (BIT(m_sys_pending, EXC_PENDSV))
				data |= 1 << 28;
			if (BIT(m_sys_pending, EXC_SYSTICK))
				data |= 1 << 26;
			if (pending && group_priority(prio) < execution_priority())
				data |= 1 << 23;
			if (isr_pending)
				data |= 1 << 22;
			data |= (pending & 0x1ff) << 12;
			if (active_count() <= 1)
				data |= 1 << 11;
			return data;
		}
	case 0xd08:
		return m_vtor;
	case 0xd0c:
		return 0xfa050000 | (m_prigroup << 8);
	case 0xd10:
		return m_scr;
	case 0xd14:
		return m_ccr;
	case 0xd18: case 0xd1c: case 0xd20:
		{
			const unsigned base = offset - 0xd18;
			return m_shpr[base] | (m_shpr[base + 1] << 8) | (m_shpr[base + 2] << 16) | (m_shpr[base + 3] << 24);
		}
	case 0xd24:
		{
			u32 data = m_shcsr_ena;
			data |= BIT(m_sys_active, EXC_MEMMANAGE) << 0;
			data |= BIT(m_sys_active, EXC_BUSFAULT) << 1;
			data |= BIT(m_sys_active, EXC_USAGEFAULT) << 3;
			data |= BIT(m_sys_active, EXC_SVCALL) << 7;
			data |= BIT(m_sys_active, EXC_DEBUGMONITOR) << 8;
			data |= BIT(m_sys_active, EXC_PENDSV) << 10;
			data |= BIT(m_sys_active, EXC_SYSTICK) << 11;
			data |= BIT(m_sys_pending, EXC_USAGEFAULT) << 12;
			data |= BIT(m_sys_pending, EXC_MEMMANAGE) << 13;
			data |= BIT(m_sys_pending, EXC_BUSFAULT) << 14;
			data |= BIT(m_sys_pending, EXC_SVCALL) << 15;
			return data;
		}
	case 0xd28:
		return m_cfsr;
	case 0xd2c:
		return m_hfsr;
	case 0xd30:
		return m_dfsr;
	case 0xd34:
	case 0xd38:
		return m_fault_addr;
	case 0xd3c:
		return m_afsr;

	case 0xd90:
		return m_mpu_regions << 8;
	case 0xd94:
		return m_mpu_regions ? m_mpu_ctrl : 0;
	case 0xd98:
		return m_mpu_regions ? m_mpu_rnr : 0;
	case 0xd9c: case 0xda4: case 0xdac: case 0xdb4:
		if (!m_mpu_regions)
			return 0;
		return m_mpu_rbar[m_mpu_rnr] | (m_mpu_rnr & 0xf);
	case 0xda0: case 0xda8: case 0xdb0: case 0xdb8:
		if (!m_mpu_regions)
			return 0;
		return m_mpu_rasr[m_mpu_rnr];

	case 0xdfc:
		return (m_demcr & ~DEMCR_MON_PEND) | (BIT(m_sys_pending, EXC_DEBUGMONITOR) ? u32(DEMCR_MON_PEND) : 0U);
	}

	LOGMASKED(LOG_PPB, "%08x: read from unmapped SCS offset %03x\n", m_pc, offset);
	return 0;
}

void armv7m_device::scs_write(u32 offset, u32 data, u32 mask)
{
	const u32 bits = data & mask;

	if (offset >= 0x100 && offset < 0x300)
	{
		const unsigned word = (offset & 0x7f) >> 2;
		if (word >= m_irq_words)
			return;
		u32 valid = ~u32(0);
		if (word == m_irq_words - 1 && (m_num_irq & 31))
			valid = (1U << (m_num_irq & 31)) - 1;
		switch (offset >> 7)
		{
		case 2:
			m_irq_enable[word] |= bits & valid;
			break;
		case 3:
			m_irq_enable[word] &= ~bits;
			break;
		case 4:
			for (unsigned i = 0; i < 32; i++)
				if (BIT(bits & valid, i))
					set_exception_pending(16 + word * 32 + i, true);
			break;
		case 5:
			m_irq_pending[word] &= ~(bits & ~m_irq_level[word]);
			break;
		}
		check_irq();
		return;
	}
	if (offset >= 0x400 && offset < 0x5f0)
	{
		const unsigned irq = offset - 0x400;
		for (unsigned i = 0; i < 4; i++)
			if (BIT(mask, 8 * i) && irq + i < m_num_irq)
				m_irq_prio[irq + i] = u8(data >> (8 * i)) & m_prio_mask;
		check_irq();
		return;
	}

	switch (offset)
	{
	case 0x008:
		m_actlr = ((m_actlr & ~mask) | bits) & 7;
		break;

	case 0x010:
		m_syst_csr = (m_syst_csr & (~mask | SYST_COUNTFLAG)) | (bits & (SYST_ENABLE | SYST_TICKINT | SYST_CLKSOURCE));
		break;
	case 0x014:
		m_syst_rvr = ((m_syst_rvr & ~mask) | bits) & 0x00ffffff;
		break;
	case 0x018:
		m_syst_cvr = 0;
		m_syst_csr &= ~SYST_COUNTFLAG;
		break;

	case 0xd04:
		if (bits & (1U << 31))
			set_exception_pending(EXC_NMI, true);
		if (bits & (1 << 28))
			set_exception_pending(EXC_PENDSV, true);
		else if (bits & (1 << 27))
			set_exception_pending(EXC_PENDSV, false);
		if (bits & (1 << 26))
			set_exception_pending(EXC_SYSTICK, true);
		else if (bits & (1 << 25))
			set_exception_pending(EXC_SYSTICK, false);
		break;
	case 0xd08:
		m_vtor = ((m_vtor & ~mask) | bits) & 0xffffff80;
		break;
	case 0xd0c:
		if ((mask & 0xffff0000) == 0xffff0000 && (data >> 16) == 0x05fa)
		{
			if (mask & 0x700)
				m_prigroup = (data >> 8) & 7;
			if (bits & 2)
			{
				m_sys_active = 0;
				std::fill_n(&m_irq_active[0], m_irq_words, 0);
				m_ipsr = 0;
				for (unsigned i = 0; i < m_irq_words; i++)
					m_irq_pending[i] |= m_irq_level[i];
			}
			if (bits & 4)
			{
				LOGMASKED(LOG_EXCEPTION, "%08x: system reset request\n", m_pc);
				if (m_sysresetreq_cb.isunset())
					pulse_input_line(INPUT_LINE_RESET, attotime::zero);
				else
					m_sysresetreq_cb(ASSERT_LINE);
			}
			else if (bits & 1)
				pulse_input_line(INPUT_LINE_RESET, attotime::zero);
		}
		break;
	case 0xd10:
		m_scr = ((m_scr & ~mask) | bits) & 0x16;
		break;
	case 0xd14:
		m_ccr = ((m_ccr & ~mask) | bits) & 0x31b;
		break;
	case 0xd18: case 0xd1c: case 0xd20:
		for (unsigned i = 0; i < 4; i++)
		{
			const unsigned exc = 4 + (offset - 0xd18) + i;
			if (BIT(mask, 8 * i) && (exc == 4 || exc == 5 || exc == 6 || exc == 11 || exc == 12 || exc == 14 || exc == 15))
				m_shpr[exc - 4] = u8(data >> (8 * i)) & m_prio_mask;
		}
		break;
	case 0xd24:
		{
			const u32 value = (scs_read(0xd24) & ~mask) | bits;
			m_shcsr_ena = value & 0x70000;
			static const u8 act[7][2] = { { 0, EXC_MEMMANAGE }, { 1, EXC_BUSFAULT }, { 3, EXC_USAGEFAULT }, { 7, EXC_SVCALL }, { 8, EXC_DEBUGMONITOR }, { 10, EXC_PENDSV }, { 11, EXC_SYSTICK } };
			static const u8 pend[4][2] = { { 12, EXC_USAGEFAULT }, { 13, EXC_MEMMANAGE }, { 14, EXC_BUSFAULT }, { 15, EXC_SVCALL } };
			for (auto &a : act)
				m_sys_active = (m_sys_active & ~(1 << a[1])) | (BIT(value, a[0]) << a[1]);
			for (auto &p : pend)
				m_sys_pending = (m_sys_pending & ~(1 << p[1])) | (BIT(value, p[0]) << p[1]);
		}
		break;
	case 0xd28:
		m_cfsr &= ~bits;
		break;
	case 0xd2c:
		m_hfsr &= ~bits;
		break;
	case 0xd30:
		m_dfsr &= ~bits;
		break;
	case 0xd34:
	case 0xd38:
		m_fault_addr = (m_fault_addr & ~mask) | bits;
		break;
	case 0xd3c:
		m_afsr &= ~bits;
		break;

	case 0xd94:
		if (m_mpu_regions)
			m_mpu_ctrl = ((m_mpu_ctrl & ~mask) | bits) & 7;
		break;
	case 0xd98:
		if (m_mpu_regions)
			m_mpu_rnr = (((m_mpu_rnr & ~mask) | bits) & 0xff) % m_mpu_regions;
		break;
	case 0xd9c: case 0xda4: case 0xdac: case 0xdb4:
		if (m_mpu_regions)
		{
			if (bits & 0x10)
				m_mpu_rnr = (data & 0xf) % m_mpu_regions;
			m_mpu_rbar[m_mpu_rnr] = ((m_mpu_rbar[m_mpu_rnr] & ~mask) | bits) & ~0x1f;
		}
		break;
	case 0xda0: case 0xda8: case 0xdb0: case 0xdb8:
		if (m_mpu_regions)
			m_mpu_rasr[m_mpu_rnr] = ((m_mpu_rasr[m_mpu_rnr] & ~mask) | bits) & 0x173fff3f;
		break;

	case 0xdfc:
		m_demcr = ((m_demcr & ~mask) | bits) & 0x010f07f1;
		if (bits & DEMCR_MON_PEND)
			set_exception_pending(EXC_DEBUGMONITOR, true);
		else if (mask & DEMCR_MON_PEND)
			set_exception_pending(EXC_DEBUGMONITOR, false);
		break;

	case 0xf00:
		if ((data & 0x1ff) < m_num_irq)
			set_exception_pending(16 + (data & 0x1ff), true);
		break;

	default:
		LOGMASKED(LOG_PPB, "%08x: write to unmapped SCS offset %03x = %08x & %08x\n", m_pc, offset, data, mask);
		break;
	}
	check_irq();
}


//-------------------------------------------------
//  exception model
//-------------------------------------------------

int armv7m_device::exception_priority(int exc) const
{
	switch (exc)
	{
	case EXC_RESET: return -3;
	case EXC_NMI: return -2;
	case EXC_HARDFAULT: return -1;
	default:
		if (exc < 16)
			return m_shpr[exc - 4];
		return m_irq_prio[exc - 16];
	}
}

bool armv7m_device::exception_active(int exc) const
{
	if (exc < 16)
		return BIT(m_sys_active, exc);
	return BIT(m_irq_active[(exc - 16) >> 5], (exc - 16) & 31);
}

void armv7m_device::set_exception_active(int exc, bool state)
{
	if (exc < 16)
		m_sys_active = (m_sys_active & ~(1 << exc)) | (state << exc);
	else
	{
		const unsigned irq = exc - 16;
		m_irq_active[irq >> 5] = (m_irq_active[irq >> 5] & ~(1U << (irq & 31))) | (u32(state) << (irq & 31));
	}
	check_irq();
}

void armv7m_device::set_exception_pending(int exc, bool state)
{
	bool was;
	if (exc < 16)
	{
		was = BIT(m_sys_pending, exc);
		m_sys_pending = (m_sys_pending & ~(1 << exc)) | (state << exc);
	}
	else
	{
		const unsigned irq = exc - 16;
		was = BIT(m_irq_pending[irq >> 5], irq & 31);
		m_irq_pending[irq >> 5] = (m_irq_pending[irq >> 5] & ~(1U << (irq & 31))) | (u32(state) << (irq & 31));
	}
	if (state && !was && (m_scr & SCR_SEVONPEND))
	{
		m_event = true;
		if (m_sleep == SLEEP_WFE)
		{
			m_sleep = SLEEP_NONE;
			m_event = false;
		}
	}
	check_irq();
}

unsigned armv7m_device::active_count() const
{
	unsigned count = std::popcount(m_sys_active);
	for (unsigned i = 0; i < m_irq_words; i++)
		count += std::popcount(m_irq_active[i]);
	return count;
}

bool armv7m_device::fault_enabled(int exc) const
{
	switch (exc)
	{
	case EXC_MEMMANAGE: return BIT(m_shcsr_ena, 16);
	case EXC_BUSFAULT: return BIT(m_shcsr_ena, 17);
	case EXC_USAGEFAULT: return BIT(m_shcsr_ena, 18);
	default: return true;
	}
}

int armv7m_device::execution_priority()
{
	if (!m_prio_dirty)
		return m_exec_prio;

	int highest = 256;
	if (BIT(m_sys_active, EXC_NMI))
		highest = -2;
	else if (BIT(m_sys_active, EXC_HARDFAULT))
		highest = -1;
	else
	{
		for (int exc = 4; exc < 16; exc++)
			if (BIT(m_sys_active, exc) && m_shpr[exc - 4] < highest)
				highest = m_shpr[exc - 4];
		for (unsigned w = 0; w < m_irq_words; w++)
		{
			u32 active = m_irq_active[w];
			while (active)
			{
				const unsigned bit = std::countr_zero(active);
				active &= active - 1;
				if (m_irq_prio[w * 32 + bit] < highest)
					highest = m_irq_prio[w * 32 + bit];
			}
		}
		if (highest < 256)
			highest = group_priority(highest);
	}

	int boosted = 256;
	if (m_basepri)
		boosted = group_priority(m_basepri);
	if (m_primask)
		boosted = 0;
	if (m_faultmask)
		boosted = -1;

	m_exec_prio = std::min(highest, boosted);
	m_prio_dirty = false;
	return m_exec_prio;
}

int armv7m_device::pending_exception(int &prio) const
{
	if (BIT(m_sys_pending, EXC_NMI))
	{
		prio = -2;
		return EXC_NMI;
	}
	if (BIT(m_sys_pending, EXC_HARDFAULT))
	{
		prio = -1;
		return EXC_HARDFAULT;
	}

	int best = 0;
	int best_prio = 256;
	if (m_sys_pending)
	{
		for (int exc = 4; exc < 16; exc++)
			if (BIT(m_sys_pending, exc) && fault_enabled(exc) && m_shpr[exc - 4] < best_prio)
			{
				best = exc;
				best_prio = m_shpr[exc - 4];
			}
	}
	for (unsigned w = 0; w < m_irq_words; w++)
	{
		u32 ready = m_irq_pending[w] & m_irq_enable[w];
		while (ready)
		{
			const unsigned bit = std::countr_zero(ready);
			ready &= ready - 1;
			if (m_irq_prio[w * 32 + bit] < best_prio)
			{
				best = 16 + w * 32 + bit;
				best_prio = m_irq_prio[w * 32 + bit];
			}
		}
	}
	prio = best_prio;
	return best;
}

void armv7m_device::sample_irq_level(unsigned irq)
{
	if (BIT(m_irq_level[irq >> 5], irq & 31))
		set_exception_pending(16 + irq, true);
}

void armv7m_device::set_irq_line(unsigned irq, int state)
{
	if (irq >= m_num_irq)
		return;
	const u32 bit = 1U << (irq & 31);
	const bool old = m_irq_level[irq >> 5] & bit;
	if (state != CLEAR_LINE)
	{
		m_irq_level[irq >> 5] |= bit;
		if (!old)
			set_exception_pending(16 + irq, true);
	}
	else
		m_irq_level[irq >> 5] &= ~bit;
}

void armv7m_device::execute_set_input(int inputnum, int state)
{
	if (inputnum == INPUT_LINE_NMI)
	{
		if (state != CLEAR_LINE && !m_nmi_level)
			set_exception_pending(EXC_NMI, true);
		m_nmi_level = state != CLEAR_LINE;
	}
	else if (inputnum >= 0)
		set_irq_line(inputnum, state);
}

void armv7m_device::enter_lockup(u32 address)
{
	LOGMASKED(LOG_EXCEPTION, "%08x: lockup at %08x\n", m_pc, address);
	m_pc = address & ~1;
	m_lockup = true;
	m_lockup_cb(ASSERT_LINE);
}

bool armv7m_device::take_interrupt()
{
	int prio;
	const int exc = pending_exception(prio);
	if (!exc)
		return false;
	const int group = group_priority(prio);
	const int ep = execution_priority();

	if (m_sleep != SLEEP_NONE)
	{
		const u32 primask = m_primask;
		m_primask = 0;
		m_prio_dirty = true;
		const bool wake = group < execution_priority();
		m_primask = primask;
		m_prio_dirty = true;
		if (wake)
		{
			if (m_sleep == SLEEP_WFE)
				m_event = false;
			m_sleep = SLEEP_NONE;
		}
	}

	if (group >= ep)
		return false;

	if (m_lockup)
	{
		m_lockup = false;
		m_lockup_cb(CLEAR_LINE);
	}
	m_sleep = SLEEP_NONE;
	exception_entry(exc, m_pc);
	return true;
}

void armv7m_device::take_sync_exception(int exc, u32 return_address, u32 insn_address)
{
	const int ep = execution_priority();
	bool escalate = false;
	switch (exc)
	{
	case EXC_MEMMANAGE:
	case EXC_BUSFAULT:
	case EXC_USAGEFAULT:
		escalate = !fault_enabled(exc) || group_priority(exception_priority(exc)) >= ep;
		break;
	case EXC_SVCALL:
		escalate = group_priority(exception_priority(exc)) >= ep;
		break;
	case EXC_DEBUGMONITOR:
		if (!(m_demcr & DEMCR_MON_EN) || group_priority(exception_priority(exc)) >= ep)
		{
			if (ep < 0)
			{
				enter_lockup(insn_address);
				return;
			}
			m_hfsr |= HFSR_DEBUGEVT;
			exc = EXC_HARDFAULT;
		}
		break;
	}

	if (escalate)
	{
		if (ep < 0)
		{
			enter_lockup(insn_address);
			return;
		}
		m_hfsr |= HFSR_FORCED;
		exc = EXC_HARDFAULT;
	}
	else if (exc == EXC_HARDFAULT && ep < 0)
	{
		enter_lockup(insn_address);
		return;
	}

	exception_entry(exc, return_address);
}

bool armv7m_device::push_stack(u32 return_address)
{
	const bool forcealign = m_ccr & CCR_STKALIGN;
	const u32 sp = m_r[13];
	const u32 frameptralign = forcealign ? (sp >> 2) & 1 : 0;
	const u32 frameptr = (sp - 0x20) & (forcealign ? ~7U : ~3U);
	const bool thread_psp = psp_active();
	m_r[13] = frameptr;

	const u32 frame[8] = {
		m_r[0], m_r[1], m_r[2], m_r[3], m_r[12], m_r[14], return_address & ~1,
		(xpsr() & ~0x200) | (frameptralign << 9)
	};
	bool ok = true;
	for (int i = 0; i < 8 && ok; i++)
	{
		u32 value = frame[i];
		ok = access_aligned(frameptr + 4 * i, 4, value, true, ACC_STACK);
	}

	if (m_handler)
		m_r[14] = 0xfffffff1;
	else
		m_r[14] = thread_psp ? 0xfffffffd : 0xfffffff9;
	m_cycles += 12;
	return ok;
}

void armv7m_device::exception_taken(int exc)
{
	LOGMASKED(LOG_EXCEPTION, "%08x: exception %d taken\n", m_pc, exc);
	const u32 vector = m_program.read_dword(m_vtor + 4 * exc);
	m_pc = vector & ~1;
	m_tbit = vector & 1;
	set_mode(true, false);
	m_ipsr = exc;
	m_it = 0;
	if (exc == EXC_DEBUGMONITOR)
		m_demcr &= ~DEMCR_MON_PEND;
	set_exception_pending(exc, false);
	set_exception_active(exc, true);
	m_excl = false;
	m_event = true;
	check_irq();

	if (exc == EXC_NMI)
		standard_irq_callback(INPUT_LINE_NMI, m_pc);
	else if (exc >= 16 && exc - 16 < INPUT_LINE_NMI)
		standard_irq_callback(exc - 16, m_pc);
}

void armv7m_device::exception_entry(int exc, u32 return_address)
{
	const int preempted = execution_priority();
	m_fault_exc = 0;
	if (push_stack(return_address))
	{
		exception_taken(exc);
		return;
	}

	int derived = m_fault_exc;
	m_fault_exc = 0;
	LOGMASKED(LOG_EXCEPTION, "%08x: fault %d stacking for exception %d\n", m_pc, derived, exc);
	if (!fault_enabled(derived) || group_priority(exception_priority(derived)) >= preempted)
	{
		if (preempted < 0)
		{
			enter_lockup(0xfffffffe);
			return;
		}
		m_hfsr |= HFSR_FORCED;
		set_exception_pending(exc, true);
		exception_taken(EXC_HARDFAULT);
	}
	else if (group_priority(exception_priority(derived)) < group_priority(exception_priority(exc)))
	{
		set_exception_pending(exc, true);
		exception_taken(derived);
	}
	else
	{
		set_exception_pending(derived, true);
		exception_taken(exc);
	}
}

void armv7m_device::deactivate(int exc)
{
	set_exception_active(exc, false);
	if (m_ipsr != EXC_NMI)
		m_faultmask = 0;
	if (exc >= 16)
		sample_irq_level(exc - 16);
	check_irq();
}

void armv7m_device::exception_return(u32 exc_return)
{
	const int returning = m_ipsr;
	const unsigned nested = active_count();
	if ((exc_return & 0x0ffffff0) != 0x0ffffff0)
		unpredictable();

	auto integrity_fault = [this, exc_return] (bool push)
	{
		m_cfsr |= CFSR_INVPC;
		if (push)
			push_stack(m_pc);
		m_r[14] = exc_return;
		const int ep = execution_priority();
		if (!fault_enabled(EXC_USAGEFAULT) || group_priority(exception_priority(EXC_USAGEFAULT)) >= ep)
		{
			if (ep < 0)
			{
				enter_lockup(0xfffffffe);
				return;
			}
			m_hfsr |= HFSR_FORCED;
			exception_taken(EXC_HARDFAULT);
		}
		else
			exception_taken(EXC_USAGEFAULT);
	};

	bool to_handler = false;
	bool use_psp = false;
	bool valid = exception_active(returning);
	switch (exc_return & 0xf)
	{
	case 0x1:
		to_handler = true;
		break;
	case 0x9:
		if (nested != 1 && !(m_ccr & CCR_NONBASETHRDENA))
			valid = false;
		break;
	case 0xd:
		if (nested != 1 && !(m_ccr & CCR_NONBASETHRDENA))
			valid = false;
		use_psp = true;
		break;
	default:
		valid = false;
		break;
	}
	if (!valid)
	{
		deactivate(returning);
		integrity_fault(false);
		return;
	}

	deactivate(returning);
	m_cycles += 12;

	int prio;
	const int chained = pending_exception(prio);
	if (chained && group_priority(prio) < execution_priority())
	{
		m_r[14] = exc_return;
		m_cycles -= 6;
		exception_taken(chained);
		return;
	}

	const u32 frameptr = use_psp ? get_psp() : get_msp();
	const bool unpriv = !to_handler && (m_control & 1);
	u32 frame[8];
	m_fault_exc = 0;
	for (int i = 0; i < 8; i++)
	{
		const bool handler = m_handler;
		m_handler = !unpriv;
		const bool ok = access_aligned(frameptr + 4 * i, 4, frame[i], false, ACC_UNSTACK);
		m_handler = handler;
		if (!ok)
		{
			const int derived = m_fault_exc;
			m_fault_exc = 0;
			m_r[14] = exc_return;
			const int ep = execution_priority();
			if (!fault_enabled(derived) || group_priority(exception_priority(derived)) >= ep)
			{
				if (ep < 0)
				{
					enter_lockup(0xfffffffe);
					return;
				}
				m_hfsr |= HFSR_FORCED;
				exception_taken(EXC_HARDFAULT);
			}
			else
				exception_taken(derived);
			return;
		}
	}

	const bool forcealign = m_ccr & CCR_STKALIGN;
	const u32 psr = frame[7];
	const u32 new_sp = (frameptr + 0x20) | ((BIT(psr, 9) && forcealign) ? 4 : 0);
	set_mode(to_handler, use_psp);
	m_r[13] = new_sp;
	for (int i = 0; i < 4; i++)
		m_r[i] = frame[i];
	m_r[12] = frame[4];
	m_r[14] = frame[5];
	if (frame[6] & 1)
		unpredictable();
	m_pc = frame[6] & ~1;
	set_nzcvq(psr);
	m_ipsr = psr & 0x1ff;
	m_tbit = BIT(psr, 24);
	m_it = ((psr >> 25) & 3) | ((psr >> 8) & 0xfc);

	if ((to_handler && m_ipsr == 0) || (!to_handler && m_ipsr != 0))
	{
		integrity_fault(true);
		return;
	}

	m_excl = false;
	m_event = true;
	if (!to_handler && !active_count() && (m_scr & SCR_SLEEPONEXIT))
		m_sleep = SLEEP_WFI;
	check_irq();
}


//-------------------------------------------------
//  timers
//-------------------------------------------------

void armv7m_device::systick_advance(u32 cycles)
{
	if (!(m_syst_csr & SYST_ENABLE))
		return;
	u32 ticks = cycles;
	if (!(m_syst_csr & SYST_CLKSOURCE) && !BIT(m_systick_calib, 31))
	{
		if (!m_systick_ref_div)
			return;
		m_syst_prescale += cycles;
		ticks = m_syst_prescale / m_systick_ref_div;
		m_syst_prescale %= m_systick_ref_div;
	}
	while (ticks)
	{
		if (!m_syst_cvr)
		{
			m_syst_cvr = m_syst_rvr;
			ticks--;
			if (!m_syst_rvr)
				break;
			continue;
		}
		if (ticks < m_syst_cvr)
		{
			m_syst_cvr -= ticks;
			break;
		}
		ticks -= m_syst_cvr;
		m_syst_cvr = 0;
		m_syst_csr |= SYST_COUNTFLAG;
		if (m_syst_csr & SYST_TICKINT)
			set_exception_pending(EXC_SYSTICK, true);
	}
}

u32 armv7m_device::systick_cycles_to_event() const
{
	if (!(m_syst_csr & SYST_ENABLE) || !(m_syst_csr & SYST_TICKINT))
		return ~u32(0);
	const u32 ticks = m_syst_cvr ? m_syst_cvr : m_syst_rvr + 1;
	if (!(m_syst_csr & SYST_CLKSOURCE) && !BIT(m_systick_calib, 31))
		return m_systick_ref_div ? ticks * m_systick_ref_div - m_syst_prescale : ~u32(0);
	return ticks;
}

void armv7m_device::advance_cycles(int cycles)
{
	systick_advance(cycles);
	if ((m_demcr & DEMCR_TRCENA) && (m_dwt_ctrl & 1))
		m_dwt_cyccnt += cycles;
}


//-------------------------------------------------
//  execution
//-------------------------------------------------

void armv7m_device::execute_run()
{
	while (m_icount > 0)
	{
		if (m_check_irq)
		{
			m_check_irq = false;
			m_cycles = 0;
			if (take_interrupt())
			{
				m_icount -= m_cycles;
				advance_cycles(m_cycles);
				continue;
			}
		}

		if (m_sleep != SLEEP_NONE || m_lockup)
		{
			const int burn = int(std::min<u32>(m_icount, std::max<u32>(systick_cycles_to_event(), 1)));
			m_icount -= burn;
			advance_cycles(burn);
			continue;
		}

		debugger_instruction_hook(m_pc);
		step();
	}
}

void armv7m_device::step()
{
	const u32 pc = m_pc;
	m_cycles = 1;
	m_fault_exc = 0;
	m_post_exc = 0;
	m_exc_return_pending = false;

	if (!mpu_check(pc, ACC_IFETCH, false, privileged()))
	{
		take_sync_exception(m_fault_exc, pc, pc);
	}
	else if (!m_tbit)
	{
		m_cfsr |= CFSR_INVSTATE;
		take_sync_exception(EXC_USAGEFAULT, pc, pc);
	}
	else
	{
		const u16 op = m_cache.read_word(pc);
		m_it_next = (m_it & 7) ? ((m_it & 0xe0) | ((m_it << 1) & 0x1f)) : 0;
		if ((op & 0xe000) == 0xe000 && (op & 0x1800))
		{
			m_next_pc = pc + 4;
			if (!((pc + 2) & 0x1e) && !mpu_check(pc + 2, ACC_IFETCH, false, privileged()))
				;
			else
			{
				const u32 op32 = (u32(op) << 16) | m_cache.read_word(pc + 2);
				if (!in_it() || condition(m_it >> 4))
					execute_t32(op32);
			}
		}
		else
		{
			m_next_pc = pc + 2;
			if (!in_it() || condition(m_it >> 4) || (op & 0xff00) == 0xbe00)
				execute_t16(op);
		}

		if (m_fault_exc)
			take_sync_exception(m_fault_exc, pc, pc);
		else
		{
			m_pc = m_next_pc;
			m_it = m_it_next;
			if (m_exc_return_pending)
				exception_return(m_exc_return);
			else if (m_post_exc)
				take_sync_exception(m_post_exc, m_pc, pc);
		}
	}

	m_icount -= m_cycles;
	advance_cycles(m_cycles);
}

void armv7m_device::undefined()
{
	fault(EXC_USAGEFAULT, CFSR_UNDEFINSTR);
}

void armv7m_device::nocp()
{
	fault(EXC_USAGEFAULT, CFSR_NOCP);
}

void armv7m_device::do_svc()
{
	m_post_exc = EXC_SVCALL;
}

void armv7m_device::do_bkpt()
{
	m_dfsr |= 2;
	fault(EXC_DEBUGMONITOR, 0);
}

void armv7m_device::do_wfi()
{
	m_sleep = SLEEP_WFI;
	check_irq();
}

void armv7m_device::do_wfe()
{
	if (m_event)
		m_event = false;
	else
	{
		m_sleep = SLEEP_WFE;
		check_irq();
	}
}

void armv7m_device::hint(unsigned op)
{
	switch (op)
	{
	case 2: do_wfe(); break;
	case 3: do_wfi(); break;
	case 4: m_event = true; break;
	default: break;
	}
}


//-------------------------------------------------
//  shared instruction bodies
//-------------------------------------------------

void armv7m_device::dp_op(unsigned opc, int d, int n, int m, u32 op2, u32 carry, bool setflags)
{
	const bool test = d == 15 && setflags && (opc == 0 || opc == 4 || opc == 8 || opc == 13);
	const bool move = (opc == 2 || opc == 3) && n == 15;
	const bool sp_arith = (opc == 8 || opc == 13) && n == 13;

	if (d == 15 && !test)
		unpredictable();
	if (d == 13 && !sp_arith && !(opc == 2 && n == 15 && m != -1 && !setflags))
		unpredictable();
	if (n == 15 && !move)
		unpredictable();
	if (n == 13 && !sp_arith)
		unpredictable();
	if (m == 15 || (m == 13 && !sp_arith && !(opc == 2 && n == 15 && !setflags && d != 13)))
		unpredictable();

	const u32 rn = move ? 0 : reg(n);
	u32 result;
	bool logical = true;
	switch (opc)
	{
	case 0: result = rn & op2; break;
	case 1: result = rn & ~op2; break;
	case 2: result = rn | op2; break;
	case 3: result = rn | ~op2; break;
	case 4: result = rn ^ op2; break;
	case 8: result = add_with_carry(rn, op2, 0, setflags); logical = false; break;
	case 10: result = add_with_carry(rn, op2, m_c, setflags); logical = false; break;
	case 11: result = add_with_carry(rn, ~op2, m_c, setflags); logical = false; break;
	case 13: result = add_with_carry(rn, ~op2, 1, setflags); logical = false; break;
	case 14: result = add_with_carry(~rn, op2, 1, setflags); logical = false; break;
	default:
		undefined();
		return;
	}
	if (logical && setflags)
	{
		set_nz(result);
		m_c = carry;
	}
	if (d != 15)
		set_reg(d, result);
}

bool armv7m_device::load(u32 address, int size, bool sign, u8 flags, u32 &value)
{
	if (!mem_read(address, size, value, flags))
		return false;
	if (sign)
		value = size == 1 ? u32(s32(s8(value))) : u32(s32(s16(value)));
	m_cycles++;
	return true;
}

void armv7m_device::load_dest(int t, u32 value, u32 address)
{
	if (t == 15)
	{
		if (address & 3)
			unpredictable();
		if (in_it() && !last_in_it())
			unpredictable();
		bx_write_pc(value);
		m_cycles += 2;
	}
	else
		set_reg(t, value);
}

void armv7m_device::load_multiple(int n, u32 registers, bool wback, bool decrement)
{
	const unsigned count = std::popcount(registers);
	const u32 base = reg(n);
	u32 address = decrement ? base - 4 * count : base;
	u32 values[16];
	for (int i = 0; i < 16; i++)
	{
		if (!BIT(registers, i))
			continue;
		if (!mem_read(address, 4, values[i], MEM_ALIGNED))
			return;
		address += 4;
	}
	for (int i = 0; i < 15; i++)
		if (BIT(registers, i))
			set_reg(i, values[i]);
	if (wback && !BIT(registers, n))
		set_reg(n, decrement ? base - 4 * count : base + 4 * count);
	if (BIT(registers, 15))
	{
		bx_write_pc(values[15]);
		m_cycles += 2;
	}
	m_cycles += count;
}

void armv7m_device::store_multiple(int n, u32 registers, bool wback, bool decrement)
{
	const unsigned count = std::popcount(registers);
	const u32 base = reg(n);
	u32 address = decrement ? base - 4 * count : base;
	for (int i = 0; i < 16; i++)
	{
		if (!BIT(registers, i))
			continue;
		if (!mem_write(address, 4, reg(i), MEM_ALIGNED))
			return;
		address += 4;
	}
	if (wback)
		set_reg(n, decrement ? base - 4 * count : base + 4 * count);
	m_cycles += count;
}


//-------------------------------------------------
//  16-bit instructions
//-------------------------------------------------

void armv7m_device::execute_t16(u16 op)
{
	switch (op >> 12)
	{
	case 0x0: case 0x1: case 0x2: case 0x3:
		t16_shift_add_sub(op);
		break;
	case 0x4:
		if (op & 0x800)
		{
			// LDR (literal)
			const u32 address = ((m_pc + 4) & ~3) + ((op & 0xff) << 2);
			u32 value;
			if (load(address, 4, false, 0, value))
				m_r[(op >> 8) & 7] = value;
		}
		else if (op & 0x400)
			t16_special(op);
		else
			t16_data_processing(op);
		break;
	case 0x5: case 0x6: case 0x7: case 0x8: case 0x9:
		t16_load_store(op);
		break;
	case 0xa:
		if (op & 0x800)
			m_r[(op >> 8) & 7] = m_r[13] + ((op & 0xff) << 2);
		else
			m_r[(op >> 8) & 7] = ((m_pc + 4) & ~3) + ((op & 0xff) << 2);
		break;
	case 0xb:
		t16_misc(op);
		break;
	case 0xc:
		t16_ldm_stm(op);
		break;
	case 0xd:
		t16_branch_svc(op);
		break;
	case 0xe:
		// B T2
		if (in_it() && !last_in_it())
			unpredictable();
		branch_write_pc(m_pc + 4 + util::sext(u32(op & 0x7ff) << 1, 12));
		m_cycles += 2;
		break;
	}
}

void armv7m_device::t16_shift_add_sub(u16 op)
{
	const bool setflags = !in_it();
	const int d = op & 7;
	const int n = (op >> 3) & 7;
	switch ((op >> 11) & 7)
	{
	case 0: case 1: case 2:
		{
			const int type = (op >> 11) & 3;
			const unsigned imm5 = (op >> 6) & 0x1f;
			if (type == 0 && imm5 == 0)
			{
				if (in_it())
					unpredictable();
				const u32 result = m_r[n];
				m_r[d] = result;
				set_nz(result);
				break;
			}
			u32 carry = m_c;
			const u32 result = shift_c(m_r[n], type, imm5 ? imm5 : 32, carry);
			m_r[d] = result;
			if (setflags)
			{
				set_nz(result);
				m_c = carry;
			}
			break;
		}
	case 3:
		{
			const u32 op2 = (op & 0x400) ? (op >> 6) & 7 : m_r[(op >> 6) & 7];
			if (op & 0x200)
				m_r[d] = add_with_carry(m_r[n], ~op2, 1, setflags);
			else
				m_r[d] = add_with_carry(m_r[n], op2, 0, setflags);
			break;
		}
	case 4:
		{
			const u32 result = op & 0xff;
			m_r[(op >> 8) & 7] = result;
			if (setflags)
				set_nz(result);
			break;
		}
	case 5:
		add_with_carry(m_r[(op >> 8) & 7], ~u32(op & 0xff), 1, true);
		break;
	case 6:
		m_r[(op >> 8) & 7] = add_with_carry(m_r[(op >> 8) & 7], op & 0xff, 0, setflags);
		break;
	case 7:
		m_r[(op >> 8) & 7] = add_with_carry(m_r[(op >> 8) & 7], ~u32(op & 0xff), 1, setflags);
		break;
	}
}

void armv7m_device::t16_data_processing(u16 op)
{
	const bool setflags = !in_it();
	const int dn = op & 7;
	const int m = (op >> 3) & 7;
	const u32 a = m_r[dn];
	const u32 b = m_r[m];
	u32 carry = m_c;
	u32 result;
	switch ((op >> 6) & 0xf)
	{
	case 0x0: result = a & b; break;
	case 0x1: result = a ^ b; break;
	case 0x2: result = shift_c(a, 0, b & 0xff, carry); break;
	case 0x3: result = shift_c(a, 1, b & 0xff, carry); break;
	case 0x4: result = shift_c(a, 2, b & 0xff, carry); break;
	case 0x5: m_r[dn] = add_with_carry(a, b, m_c, setflags); return;
	case 0x6: m_r[dn] = add_with_carry(a, ~b, m_c, setflags); return;
	case 0x7: result = shift_c(a, 3, b & 0xff, carry); break;
	case 0x8: set_nz(a & b); return;
	case 0x9: m_r[dn] = add_with_carry(~b, 0, 1, setflags); return;
	case 0xa: add_with_carry(a, ~b, 1, true); return;
	case 0xb: add_with_carry(a, b, 0, true); return;
	case 0xc: result = a | b; break;
	case 0xd:
		result = a * b;
		m_r[dn] = result;
		if (setflags)
			set_nz(result);
		return;
	case 0xe: result = a & ~b; break;
	default: result = ~b; break;
	}
	m_r[dn] = result;
	if (setflags)
	{
		set_nz(result);
		m_c = carry;
	}
}

void armv7m_device::t16_special(u16 op)
{
	const int dn = ((op >> 4) & 8) | (op & 7);
	const int m = (op >> 3) & 0xf;
	switch ((op >> 8) & 3)
	{
	case 0:
		{
			if (dn == 15 && in_it() && !last_in_it())
				unpredictable();
			if (dn == 15 && m == 15)
				unpredictable();
			const u32 result = reg(dn) + reg(m);
			if (dn == 15)
			{
				branch_write_pc(result);
				m_cycles += 2;
			}
			else
				set_reg(dn, result);
			break;
		}
	case 1:
		if ((dn < 8 && m < 8) || dn == 15 || m == 15)
			unpredictable();
		add_with_carry(reg(dn), ~reg(m), 1, true);
		break;
	case 2:
		if (dn == 15 && in_it() && !last_in_it())
			unpredictable();
		if (dn == 15)
		{
			branch_write_pc(reg(m));
			m_cycles += 2;
		}
		else
			set_reg(dn, reg(m));
		break;
	case 3:
		{
			if (op & 7)
				unpredictable();
			if (in_it() && !last_in_it())
				unpredictable();
			const u32 target = reg(m);
			if (op & 0x80)
			{
				if (m == 15)
					unpredictable();
				m_r[14] = (m_pc + 2) | 1;
				blx_write_pc(target);
			}
			else
				bx_write_pc(target);
			m_cycles += 2;
			break;
		}
	}
}

void armv7m_device::t16_load_store(u16 op)
{
	const int t = op & 7;
	const int n = (op >> 3) & 7;
	u32 address;
	int size;
	bool load_op;
	bool sign = false;

	switch (op >> 12)
	{
	case 0x5:
		address = m_r[n] + m_r[(op >> 6) & 7];
		switch ((op >> 9) & 7)
		{
		case 0: size = 4; load_op = false; break;
		case 1: size = 2; load_op = false; break;
		case 2: size = 1; load_op = false; break;
		case 3: size = 1; load_op = true; sign = true; break;
		case 4: size = 4; load_op = true; break;
		case 5: size = 2; load_op = true; break;
		case 6: size = 1; load_op = true; break;
		default: size = 2; load_op = true; sign = true; break;
		}
		break;
	case 0x6:
		size = 4;
		load_op = op & 0x800;
		address = m_r[n] + (((op >> 6) & 0x1f) << 2);
		break;
	case 0x7:
		size = 1;
		load_op = op & 0x800;
		address = m_r[n] + ((op >> 6) & 0x1f);
		break;
	case 0x8:
		size = 2;
		load_op = op & 0x800;
		address = m_r[n] + (((op >> 6) & 0x1f) << 1);
		break;
	default:
		{
			size = 4;
			load_op = op & 0x800;
			address = m_r[13] + ((op & 0xff) << 2);
			const int rt = (op >> 8) & 7;
			if (load_op)
			{
				u32 value;
				if (load(address, 4, false, 0, value))
					m_r[rt] = value;
			}
			else
				mem_write(address, 4, m_r[rt]);
			return;
		}
	}

	if (load_op)
	{
		u32 value;
		if (load(address, size, sign, 0, value))
			m_r[t] = value;
	}
	else
		mem_write(address, size, m_r[t]);
}

void armv7m_device::t16_misc(u16 op)
{
	switch ((op >> 8) & 0xf)
	{
	case 0x0:
		if (op & 0x80)
			m_r[13] -= (op & 0x7f) << 2;
		else
			m_r[13] += (op & 0x7f) << 2;
		break;

	case 0x1: case 0x3: case 0x9: case 0xb:
		{
			if (in_it())
				unpredictable();
			const u32 imm = ((op >> 2) & 0x3e) | ((op >> 3) & 0x40);
			if ((m_r[op & 7] == 0) != bool(op & 0x800))
			{
				branch_write_pc(m_pc + 4 + imm);
				m_cycles += 2;
			}
			break;
		}

	case 0x2:
		{
			const u32 value = m_r[(op >> 3) & 7];
			u32 result;
			switch ((op >> 6) & 3)
			{
			case 0: result = u32(s32(s16(value))); break;
			case 1: result = u32(s32(s8(value))); break;
			case 2: result = value & 0xffff; break;
			default: result = value & 0xff; break;
			}
			m_r[op & 7] = result;
			break;
		}

	case 0x4: case 0x5:
		{
			const u32 registers = (op & 0xff) | ((op & 0x100) << 6);
			if (!registers)
				unpredictable();
			store_multiple(13, registers, true, true);
			break;
		}

	case 0x6:
		if ((op & 0xffe0) == 0xb660)
		{
			if (op & 0xc)
				unpredictable();
			if (!(op & 3) || in_it())
				unpredictable();
			if (privileged())
			{
				if (op & 0x10)
				{
					if (op & 2)
						m_primask = 1;
					if ((op & 1) && execution_priority() > -1)
						m_faultmask = 1;
				}
				else
				{
					if (op & 2)
						m_primask = 0;
					if (op & 1)
						m_faultmask = 0;
				}
				check_irq();
			}
		}
		else
			undefined();
		break;

	case 0xa:
		{
			const u32 value = m_r[(op >> 3) & 7];
			u32 result;
			switch ((op >> 6) & 3)
			{
			case 0: result = swapendian_int32(value); break;
			case 1: result = ((value >> 8) & 0x00ff00ff) | ((value << 8) & 0xff00ff00); break;
			case 3: result = u32(s32(s16(((value >> 8) & 0xff) | ((value & 0xff) << 8)))); break;
			default: undefined(); return;
			}
			m_r[op & 7] = result;
			break;
		}

	case 0xc: case 0xd:
		{
			const u32 registers = (op & 0xff) | ((op & 0x100) << 7);
			if (!registers)
				unpredictable();
			if ((registers & 0x8000) && in_it() && !last_in_it())
				unpredictable();
			load_multiple(13, registers, true, false);
			break;
		}

	case 0xe:
		do_bkpt();
		break;

	case 0xf:
		if (op & 0xf)
		{
			const unsigned firstcond = (op >> 4) & 0xf;
			if (firstcond == 15 || (firstcond == 14 && std::popcount(unsigned(op & 0xf)) != 1) || in_it())
				unpredictable();
			m_it_next = op & 0xff;
		}
		else
			hint((op >> 4) & 0xf);
		break;

	default:
		undefined();
		break;
	}
}

void armv7m_device::t16_ldm_stm(u16 op)
{
	const int n = (op >> 8) & 7;
	const u32 registers = op & 0xff;
	if (!registers)
		unpredictable();
	if (op & 0x800)
		load_multiple(n, registers, !BIT(registers, n), false);
	else
	{
		if (BIT(registers, n) && (registers & ((1 << n) - 1)))
			unpredictable();
		store_multiple(n, registers, true, false);
	}
}

void armv7m_device::t16_branch_svc(u16 op)
{
	const unsigned cond = (op >> 8) & 0xf;
	if (cond == 14)
		undefined();
	else if (cond == 15)
		do_svc();
	else
	{
		if (in_it())
			unpredictable();
		if (condition(cond))
		{
			branch_write_pc(m_pc + 4 + util::sext(u32(op & 0xff) << 1, 9));
			m_cycles += 2;
		}
	}
}


//-------------------------------------------------
//  32-bit instructions
//-------------------------------------------------

void armv7m_device::execute_t32(u32 op)
{
	switch ((op >> 27) & 3)
	{
	case 1:
		if (op & 0x04000000)
			coprocessor(op);
		else if (op & 0x02000000)
			t32_dp_shifted(op);
		else if (op & 0x00400000)
			t32_dual_exclusive(op);
		else
			t32_ldm_stm(op);
		break;

	case 2:
		if (op & 0x8000)
			t32_branch_misc(op);
		else if (op & 0x02000000)
			t32_dp_plain_imm(op);
		else
			t32_dp_modified_imm(op);
		break;

	case 3:
		{
			const u32 op2 = (op >> 20) & 0x7f;
			if (op2 & 0x40)
				coprocessor(op);
			else if ((op2 & 0x71) == 0x00)
				t32_store_single(op);
			else if ((op2 & 0x61) == 0x01)
				t32_load(op);
			else if ((op2 & 0x70) == 0x20)
				t32_dp_register(op);
			else if ((op2 & 0x78) == 0x30)
				t32_multiply(op);
			else if ((op2 & 0x78) == 0x38)
				t32_long_multiply(op);
			else
				undefined();
			break;
		}
	}
}

void armv7m_device::coprocessor(u32 op)
{
	const u32 op1 = (op >> 20) & 0x3f;
	if ((op1 & 0x3e) == 0 || (op1 & 0x30) == 0x30)
		undefined();
	else
		nocp();
}

void armv7m_device::t32_ldm_stm(u32 op)
{
	const int n = (op >> 16) & 0xf;
	const bool wback = op & 0x00200000;
	const bool load_op = op & 0x00100000;
	u32 registers = op & 0xffff;
	const unsigned mode = (op >> 23) & 3;
	if (mode != 1 && mode != 2)
	{
		undefined();
		return;
	}
	const bool decrement = mode == 2;

	if (n == 15 || std::popcount(registers) < 2 || (registers & 0x2000))
		unpredictable();
	if (wback && BIT(registers, n))
		unpredictable();
	if (load_op)
	{
		if ((registers & 0xc000) == 0xc000)
			unpredictable();
		if ((registers & 0x8000) && in_it() && !last_in_it())
			unpredictable();
		load_multiple(n, registers, wback, decrement);
	}
	else
	{
		if (registers & 0x8000)
			unpredictable();
		store_multiple(n, registers, wback, decrement);
	}
}

void armv7m_device::t32_dual_exclusive(u32 op)
{
	const int n = (op >> 16) & 0xf;
	const int t = (op >> 12) & 0xf;
	const int t2 = (op >> 8) & 0xf;
	const bool p = op & 0x01000000;
	const bool u = op & 0x00800000;
	const bool w = op & 0x00200000;
	const bool l = op & 0x00100000;

	if (p || w)
	{
		const u32 imm = (op & 0xff) << 2;
		const u32 base = n == 15 ? m_pc + 4 : reg(n);
		const u32 offset_addr = u ? base + imm : base - imm;
		const u32 address = p ? offset_addr : base;
		if (t == 13 || t == 15 || t2 == 13 || t2 == 15)
			unpredictable();
		if (w && (n == t || n == t2))
			unpredictable();
		if (l)
		{
			if (t == t2)
				unpredictable();
			if (n == 15 && (w || ((m_pc + 4) & 3)))
				unpredictable();
			u32 v1, v2;
			if (!mem_read(address, 4, v1, MEM_ALIGNED) || !mem_read(address + 4, 4, v2, MEM_ALIGNED))
				return;
			if (w && n != 15)
				set_reg(n, offset_addr);
			set_reg(t, v1);
			set_reg(t2, v2);
		}
		else
		{
			if (n == 15)
				unpredictable();
			if (!mem_write(address, 4, reg(t), MEM_ALIGNED) || !mem_write(address + 4, 4, reg(t2), MEM_ALIGNED))
				return;
			if (w)
				set_reg(n, offset_addr);
		}
		m_cycles += 2;
		return;
	}

	if (!u)
	{
		// LDREX / STREX
		const u32 address = reg(n) + ((op & 0xff) << 2);
		if (n == 15 || t == 13 || t == 15)
			unpredictable();
		if (l)
		{
			if ((op & 0xf00) != 0xf00)
				unpredictable();
			u32 value;
			if (!mem_read(address, 4, value, MEM_ALIGNED))
				return;
			m_excl = true;
			set_reg(t, value);
			m_cycles++;
		}
		else
		{
			if (t2 == 13 || t2 == 15 || t2 == n || t2 == t)
				unpredictable();
			store_exclusive(t2, t, address, 4);
		}
		return;
	}

	const unsigned op3 = (op >> 4) & 0xf;
	if (!l)
	{
		const int d = op & 0xf;
		if (op3 != 4 && op3 != 5)
		{
			undefined();
			return;
		}
		if ((op & 0xf00) != 0xf00 || n == 15 || t == 13 || t == 15 || d == 13 || d == 15 || d == n || d == t)
			unpredictable();
		store_exclusive(d, t, reg(n), op3 == 4 ? 1 : 2);
		return;
	}

	switch (op3)
	{
	case 0: case 1:
		{
			const int m = op & 0xf;
			if ((op & 0xff00) != 0xf000 || n == 13 || m == 13 || m == 15)
				unpredictable();
			if (in_it() && !last_in_it())
				unpredictable();
			u32 offset;
			const bool half = op3 == 1;
			if (!mem_read(reg(n) + (half ? reg(m) << 1 : reg(m)), half ? 2 : 1, offset))
				return;
			branch_write_pc(m_pc + 4 + 2 * offset);
			m_cycles += 4;
			break;
		}
	case 4: case 5:
		{
			if ((op & 0xf0f) != 0xf0f || n == 15 || t == 13 || t == 15)
				unpredictable();
			u32 value;
			if (!mem_read(reg(n), op3 == 4 ? 1 : 2, value, MEM_ALIGNED))
				return;
			m_excl = true;
			set_reg(t, value);
			m_cycles++;
			break;
		}
	default:
		undefined();
		break;
	}
}

void armv7m_device::store_exclusive(int d, int t, u32 address, int size)
{
	if (address & (size - 1))
	{
		fault(EXC_USAGEFAULT, CFSR_UNALIGNED);
		return;
	}
	if (!mpu_check(address, ACC_NORMAL, true, privileged()))
		return;
	if (m_excl)
	{
		m_excl = false;
		if (!mem_write(address, size, reg(t), MEM_ALIGNED))
			return;
		set_reg(d, 0);
	}
	else
		set_reg(d, 1);
	m_cycles++;
}

void armv7m_device::t32_dp_shifted(u32 op)
{
	const unsigned opc = (op >> 21) & 0xf;
	const bool setflags = op & 0x00100000;
	const int n = (op >> 16) & 0xf;
	const int d = (op >> 8) & 0xf;
	const int m = op & 0xf;
	const unsigned imm5 = ((op >> 10) & 0x1c) | ((op >> 6) & 3);
	int type = (op >> 4) & 3;
	unsigned amount = imm5;
	if (type == 1 || type == 2)
		amount = imm5 ? imm5 : 32;
	else if (type == 3 && !imm5)
		type = 4;

	if (opc == 5 || opc == 6 || opc == 7 || opc == 9 || opc == 12 || opc == 15)
	{
		undefined();
		return;
	}
	if (op & 0x8000)
		unpredictable();
	if (n == 13 && (opc == 8 || opc == 13) && (type != 0 || amount > 3))
		unpredictable();

	u32 carry = m_c;
	const u32 op2 = shift_c(reg(m), type, amount, carry);
	dp_op(opc, d, n, m, op2, carry, setflags);
}

void armv7m_device::t32_dp_modified_imm(u32 op)
{
	const unsigned opc = (op >> 21) & 0xf;
	const bool setflags = op & 0x00100000;
	const int n = (op >> 16) & 0xf;
	const int d = (op >> 8) & 0xf;
	const u32 imm12 = ((op >> 15) & 0x800) | ((op >> 4) & 0x700) | (op & 0xff);
	u32 carry;
	const u32 imm32 = thumb_expand_imm_c(imm12, carry);
	dp_op(opc, d, n, -1, imm32, carry, setflags);
}

void armv7m_device::t32_dp_plain_imm(u32 op)
{
	const unsigned opc = (op >> 20) & 0x1f;
	const int n = (op >> 16) & 0xf;
	const int d = (op >> 8) & 0xf;
	const u32 imm12 = ((op >> 15) & 0x800) | ((op >> 4) & 0x700) | (op & 0xff);
	const unsigned lsb = ((op >> 10) & 0x1c) | ((op >> 6) & 3);
	const u32 rn = reg(n);

	switch (opc)
	{
	case 0x00:
	case 0x0a:
		{
			if (d == 15 || (d == 13 && n != 13))
				unpredictable();
			const u32 base = n == 15 ? (m_pc + 4) & ~3 : rn;
			set_reg(d, opc ? base - imm12 : base + imm12);
			break;
		}
	case 0x04:
	case 0x0c:
		{
			if (d == 13 || d == 15)
				unpredictable();
			const u32 imm16 = ((op >> 4) & 0xf000) | imm12;
			if (opc == 0x04)
				set_reg(d, imm16);
			else
				set_reg(d, (imm16 << 16) | (m_r[d] & 0xffff));
			break;
		}
	case 0x10: case 0x12: case 0x18: case 0x1a:
		{
			const bool sh = opc & 2;
			if (sh && !lsb)
			{
				undefined();
				return;
			}
			if ((op & 0x04000020) || d == 13 || d == 15 || n == 13 || n == 15)
				unpredictable();
			const s64 operand = s32(shift(rn, sh ? 2 : 0, lsb));
			const unsigned sat_imm = op & 0x1f;
			s64 lo, hi;
			if (opc & 8)
			{
				lo = 0;
				hi = (s64(1) << sat_imm) - 1;
			}
			else
			{
				lo = -(s64(1) << sat_imm);
				hi = (s64(1) << sat_imm) - 1;
			}
			s64 result = operand;
			if (operand > hi)
			{
				result = hi;
				m_q = 1;
			}
			else if (operand < lo)
			{
				result = lo;
				m_q = 1;
			}
			set_reg(d, u32(result));
			break;
		}
	case 0x14:
	case 0x1c:
		{
			const unsigned widthm1 = op & 0x1f;
			if ((op & 0x04000020) || d == 13 || d == 15 || n == 13 || n == 15)
				unpredictable();
			if (lsb + widthm1 > 31)
			{
				unpredictable();
				break;
			}
			const u32 field = (rn >> lsb) & (0xffffffffU >> (31 - widthm1));
			set_reg(d, opc == 0x14 ? u32(util::sext(field, widthm1 + 1)) : field);
			break;
		}
	case 0x16:
		{
			const unsigned msb = op & 0x1f;
			if ((op & 0x04000020) || d == 13 || d == 15 || n == 13)
				unpredictable();
			if (msb < lsb)
			{
				unpredictable();
				break;
			}
			const u32 mask = (0xffffffffU >> (31 - msb + lsb)) << lsb;
			const u32 source = n == 15 ? 0 : rn;
			set_reg(d, (m_r[d] & ~mask) | ((source << lsb) & mask));
			break;
		}
	default:
		undefined();
		break;
	}
}

void armv7m_device::t32_branch_misc(u32 op)
{
	const unsigned op1 = (op >> 12) & 7;
	const unsigned hwop = (op >> 20) & 0x7f;

	if ((op1 & 5) == 0)
	{
		if ((hwop & 0x38) != 0x38)
		{
			// B T3
			if (in_it())
				unpredictable();
			const u32 imm = ((op >> 26) & 1) << 20 | ((op >> 11) & 1) << 19 | ((op >> 13) & 1) << 18 | ((op >> 16) & 0x3f) << 12 | (op & 0x7ff) << 1;
			if (condition((op >> 22) & 0xf))
			{
				branch_write_pc(m_pc + 4 + util::sext(imm, 21));
				m_cycles += 2;
			}
			return;
		}
		switch (hwop)
		{
		case 0x38: case 0x39:
			t32_msr(op);
			break;
		case 0x3a:
			if ((op >> 8) & 7)
				undefined();
			else
			{
				if ((op & 0x000f2800) != 0x000f0000)
					unpredictable();
				hint(op & 0xff);
			}
			break;
		case 0x3b:
			if ((op & 0x000f2f00) != 0x000f0f00)
				unpredictable();
			switch ((op >> 4) & 0xf)
			{
			case 2:
				if ((op & 0xf) != 0xf)
					unpredictable();
				m_excl = false;
				break;
			case 4: case 5: case 6:
				break;
			default:
				undefined();
				break;
			}
			break;
		case 0x3e: case 0x3f:
			t32_mrs(op);
			break;
		default:
			undefined();
			break;
		}
		return;
	}

	if ((op1 & 5) == 4)
	{
		undefined();
		return;
	}

	const u32 s = (op >> 26) & 1;
	const u32 i1 = ~((op >> 13) ^ s) & 1;
	const u32 i2 = ~((op >> 11) ^ s) & 1;
	const u32 imm = s << 24 | i1 << 23 | i2 << 22 | ((op >> 16) & 0x3ff) << 12 | (op & 0x7ff) << 1;
	const u32 target = m_pc + 4 + util::sext(imm, 25);
	if (op1 & 4)
		m_r[14] = (m_pc + 4) | 1;
	else if (in_it() && !last_in_it())
		unpredictable();
	branch_write_pc(target);
	m_cycles += 2;
}

void armv7m_device::t32_msr(u32 op)
{
	const int n = (op >> 16) & 0xf;
	const unsigned mask = (op >> 10) & 3;
	const unsigned sysm = op & 0xff;
	if (mask == 0 || (mask != 2 && sysm > 3))
		unpredictable();
	if (n == 13 || n == 15 || !(sysm <= 3 || (sysm >= 5 && sysm <= 9) || (sysm >= 16 && sysm <= 20)))
		unpredictable();
	if (op & 0x00102300)
		unpredictable();
	const u32 value = reg(n);

	switch (sysm >> 3)
	{
	case 0:
		if (!BIT(sysm, 2))
		{
			if (mask & 1)
				unpredictable();
			if (mask & 2)
				set_nzcvq(value);
		}
		break;
	case 1:
		if (privileged())
		{
			if (sysm == 8)
				set_msp(value);
			else if (sysm == 9)
				set_psp(value);
		}
		break;
	case 2:
		if (!privileged())
			break;
		switch (sysm & 7)
		{
		case 0:
			m_primask = value & 1;
			break;
		case 1:
			m_basepri = value & m_prio_mask;
			break;
		case 2:
			if ((value & m_prio_mask) && ((value & m_prio_mask) < m_basepri || !m_basepri))
				m_basepri = value & m_prio_mask;
			break;
		case 3:
			if (execution_priority() > -1)
				m_faultmask = value & 1;
			break;
		case 4:
			m_control = (m_control & ~1) | (value & 1);
			if (!m_handler)
				set_mode(false, value & 2);
			break;
		}
		check_irq();
		break;
	}
}

void armv7m_device::t32_mrs(u32 op)
{
	const int d = (op >> 8) & 0xf;
	const unsigned sysm = op & 0xff;
	if (d == 13 || d == 15 || !(sysm <= 3 || (sysm >= 5 && sysm <= 9) || (sysm >= 16 && sysm <= 20)))
		unpredictable();
	if ((op & 0x001f2000) != 0x000f0000)
		unpredictable();

	u32 value = 0;
	switch (sysm >> 3)
	{
	case 0:
		if (BIT(sysm, 0))
			value |= m_ipsr;
		if (!BIT(sysm, 2))
			value |= apsr();
		break;
	case 1:
		if (privileged())
		{
			if (sysm == 8)
				value = get_msp();
			else if (sysm == 9)
				value = get_psp();
		}
		break;
	case 2:
		switch (sysm & 7)
		{
		case 0: value = privileged() ? m_primask : 0; break;
		case 1: case 2: value = privileged() ? m_basepri : 0; break;
		case 3: value = privileged() ? m_faultmask : 0; break;
		case 4: value = m_control & 3; break;
		}
		break;
	}
	set_reg(d, value);
}

void armv7m_device::t32_store_single(u32 op)
{
	const unsigned op1 = (op >> 21) & 7;
	const int n = (op >> 16) & 0xf;
	const int t = (op >> 12) & 0xf;
	const int size = 1 << (op1 & 3);
	if ((op1 & 3) == 3 || n == 15)
	{
		undefined();
		return;
	}

	u32 address;
	u32 offset_addr = 0;
	bool wback = false;
	u8 flags = 0;
	const u32 base = reg(n);
	if (op1 & 4)
		address = base + (op & 0xfff);
	else if (op & 0x800)
	{
		const bool p = op & 0x400;
		const bool u = op & 0x200;
		const bool w = op & 0x100;
		if (!p && !w)
		{
			undefined();
			return;
		}
		if (p && u && !w)
			flags = MEM_UNPRIV;
		const u32 imm = op & 0xff;
		offset_addr = u ? base + imm : base - imm;
		address = p ? offset_addr : base;
		wback = w;
		if (wback && n == t)
			unpredictable();
	}
	else
	{
		if (op & 0x7c0)
		{
			undefined();
			return;
		}
		const int m = op & 0xf;
		if (m == 13 || m == 15)
			unpredictable();
		address = base + (reg(m) << ((op >> 4) & 3));
	}
	if (t == 15 || (t == 13 && size != 4))
		unpredictable();

	if (!mem_write(address, size, reg(t), flags))
		return;
	if (wback)
		set_reg(n, offset_addr);
}

void armv7m_device::t32_load(u32 op)
{
	const bool sign = op & 0x01000000;
	const bool imm12_form = op & 0x00800000;
	const unsigned sz = (op >> 21) & 3;
	const int n = (op >> 16) & 0xf;
	const int t = (op >> 12) & 0xf;
	if (sz == 3 || (sz == 2 && sign))
	{
		undefined();
		return;
	}
	const int size = 1 << sz;

	u32 address;
	u32 offset_addr = 0;
	bool wback = false;
	bool hint_unpredictable = false;
	u8 flags = 0;
	if (n == 15)
	{
		const u32 base = (m_pc + 4) & ~3;
		address = imm12_form ? base + (op & 0xfff) : base - (op & 0xfff);
		if (t == 15 && size == 2 && !sign)
			hint_unpredictable = true;
	}
	else if (imm12_form)
		address = reg(n) + (op & 0xfff);
	else
	{
		const unsigned op2 = (op >> 6) & 0x3f;
		const u32 base = reg(n);
		if (op2 == 0)
		{
			const int m = op & 0xf;
			if (m == 13 || m == 15)
				unpredictable();
			address = base + (reg(m) << ((op >> 4) & 3));
		}
		else if ((op2 & 0x24) == 0x24 || (op2 & 0x3c) == 0x30 || (op2 & 0x3c) == 0x38)
		{
			const bool p = op & 0x400;
			const bool u = op & 0x200;
			const u32 imm = op & 0xff;
			offset_addr = u ? base + imm : base - imm;
			address = p ? offset_addr : base;
			wback = op & 0x100;
			if ((op2 & 0x3c) == 0x38)
				flags = MEM_UNPRIV;
			if (wback || flags)
				hint_unpredictable = true;
			if (wback && n == t)
				unpredictable();
		}
		else
		{
			undefined();
			return;
		}
	}

	if (t == 15 && size != 4)
	{
		if (hint_unpredictable)
			unpredictable();
		return;
	}
	if (t == 13 && size != 4)
		unpredictable();

	u32 value;
	if (!load(address, size, sign, flags, value))
		return;
	if (wback)
		set_reg(n, offset_addr);
	load_dest(t, value, address);
}

void armv7m_device::t32_dp_register(u32 op)
{
	const unsigned op1 = (op >> 20) & 0xf;
	const unsigned op2 = (op >> 4) & 0xf;
	const int n = (op >> 16) & 0xf;
	const int d = (op >> 8) & 0xf;
	const int m = op & 0xf;
	if ((op & 0xf000) != 0xf000)
	{
		undefined();
		return;
	}

	if (op1 < 8 && op2 == 0)
	{
		if (d == 13 || d == 15 || n == 13 || n == 15 || m == 13 || m == 15)
			unpredictable();
		u32 carry = m_c;
		const u32 result = shift_c(reg(n), op1 >> 1, reg(m) & 0xff, carry);
		set_reg(d, result);
		if (op1 & 1)
		{
			set_nz(result);
			m_c = carry;
		}
		return;
	}

	if (op1 < 8 && (op2 & 8))
	{
		if (n != 15 || op1 == 2 || op1 == 3 || op1 > 5)
		{
			undefined();
			return;
		}
		if ((op & 0x40) || d == 13 || d == 15 || m == 13 || m == 15)
			unpredictable();
		const u32 value = std::rotr<u32>(reg(m), 8 * ((op >> 4) & 3));
		u32 result;
		switch (op1)
		{
		case 0: result = u32(s32(s16(value))); break;
		case 1: result = value & 0xffff; break;
		case 4: result = u32(s32(s8(value))); break;
		default: result = value & 0xff; break;
		}
		set_reg(d, result);
		return;
	}

	if ((op1 & 0xc) == 0x8 && (op2 & 0xc) == 0x8)
	{
		const u32 value = reg(m);
		u32 result;
		switch (((op1 & 3) << 2) | (op2 & 3))
		{
		case 0x4: result = swapendian_int32(value); break;
		case 0x5: result = ((value >> 8) & 0x00ff00ff) | ((value << 8) & 0xff00ff00); break;
		case 0x6: result = bitswap<32>(value, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31); break;
		case 0x7: result = u32(s32(s16(((value >> 8) & 0xff) | ((value & 0xff) << 8)))); break;
		case 0xc: result = std::countl_zero(value); break;
		default:
			undefined();
			return;
		}
		if (n != m || d == 13 || d == 15 || m == 13 || m == 15)
			unpredictable();
		set_reg(d, result);
		return;
	}

	undefined();
}

void armv7m_device::t32_multiply(u32 op)
{
	const unsigned op1 = (op >> 20) & 7;
	const unsigned op2 = (op >> 4) & 3;
	const int n = (op >> 16) & 0xf;
	const int a = (op >> 12) & 0xf;
	const int d = (op >> 8) & 0xf;
	const int m = op & 0xf;
	if ((op & 0xc0) || op1 != 0 || op2 > 1)
	{
		undefined();
		return;
	}
	if (d == 13 || d == 15 || n == 13 || n == 15 || m == 13 || m == 15 || a == 13 || (op2 == 1 && a == 15))
		unpredictable();
	const u32 product = reg(n) * reg(m);
	if (op2 == 1)
		set_reg(d, reg(a) - product);
	else if (a == 15)
		set_reg(d, product);
	else
		set_reg(d, product + reg(a));
	if (op2 == 1 || a != 15)
		m_cycles++;
}

void armv7m_device::t32_long_multiply(u32 op)
{
	const unsigned op1 = (op >> 20) & 7;
	const unsigned op2 = (op >> 4) & 0xf;
	const int n = (op >> 16) & 0xf;
	const int dlo = (op >> 12) & 0xf;
	const int dhi = (op >> 8) & 0xf;
	const int m = op & 0xf;
	const u32 rn = reg(n);
	const u32 rm = reg(m);

	if ((op1 == 1 || op1 == 3) && op2 == 0xf)
	{
		if (dlo != 15 || dhi == 13 || dhi == 15 || n == 13 || n == 15 || m == 13 || m == 15)
			unpredictable();
		u32 result;
		if (!rm)
		{
			if (m_ccr & CCR_DIV_0_TRP)
			{
				fault(EXC_USAGEFAULT, CFSR_DIVBYZERO);
				return;
			}
			result = 0;
		}
		else if (op1 == 1)
			result = (rn == 0x80000000 && rm == 0xffffffff) ? rn : u32(s32(rn) / s32(rm));
		else
			result = rn / rm;
		set_reg(dhi, result);

		// early termination: approximate from the difference in significant bits
		const int dividend_bits = 32 - std::countl_zero(op1 == 1 && s32(rn) < 0 ? ~rn : rn);
		const int divisor_bits = 32 - std::countl_zero(op1 == 1 && s32(rm) < 0 ? ~rm : rm);
		m_cycles += std::clamp((dividend_bits - divisor_bits) / 4 + 1, 1, 11);
		return;
	}

	if (op2 != 0 || op1 == 1 || op1 == 3 || op1 == 5 || op1 == 7)
	{
		undefined();
		return;
	}
	if (dlo == 13 || dlo == 15 || dhi == 13 || dhi == 15 || n == 13 || n == 15 || m == 13 || m == 15 || dlo == dhi)
		unpredictable();

	u64 result;
	switch (op1)
	{
	case 0: result = u64(s64(s32(rn)) * s64(s32(rm))); break;
	case 2: result = u64(rn) * u64(rm); break;
	case 4: result = u64(s64(s32(rn)) * s64(s32(rm))) + ((u64(m_r[dhi]) << 32) | m_r[dlo]); break;
	default: result = u64(rn) * u64(rm) + ((u64(m_r[dhi]) << 32) | m_r[dlo]); break;
	}
	set_reg(dlo, u32(result));
	set_reg(dhi, u32(result >> 32));
	m_cycles += (op1 & 4) ? 3 : 2;
}
