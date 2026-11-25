// license:BSD-3-Clause
// copyright-holders:superctr, giulioz
/*
 *  Roland XP (MBCS30109 / RA01-005 / RA09-002)
 *
 *  64-voice custom sound generator with built-in microcoded effect DSP.
 *
 *  - Host window: 44 voice pages of 64 x 32 bits, CRAM/IRAM/PRAM, the control block at
 *    0x3900, four banks of mixer sends at 0x3a00, the wave ROM window at 0x3c00
 *  - Voice: DPCM reader over 1 MB ROM regions (forward, alternate, reverse), 4-tap 128-phase
 *    interpolator, Chamberlin state-variable filter, two amplitude stages, five ramps (TVA1,
 *    TVA2, RESO, TVF, PITCH), paired voice structures with booster and ring modulator
 *  - DSP: 288 slots, IRAM1/IRAM2 64 x 24 bits, IRAM3 64 x 26 bits with gain registers,
 *    64K-word external delay RAM, one multiplier and one ALU on a 29-bit accumulator
 *  - Serial: port A (six-line, bidirectional), ports B, C and D out
 *
 *  TODO:
 *  - the sample FIFO and the fetch-overload interrupt (reason 7)
 *  - the 16-bit wave bus and the alternate decoding of 0x3908
 *  - wait states
 */
#include "emu.h"
#include "roland_xp.h"

#include "roland_pcm_interp.h"

#include <algorithm>
#include <cmath>
#include <iterator>

#define LOG_VOICE   (1U << 1)
#define LOG_CRAM    (1U << 2)
#define LOG_IRAM    (1U << 3)
#define LOG_PRAM    (1U << 4)
#define LOG_RUNMASK (1U << 5)
#define LOG_CTRL    (1U << 6)
#define LOG_MIX     (1U << 7)
#define LOG_READ    (1U << 8)
#define LOG_UNKNOWN (1U << 9)

#define VERBOSE (LOG_GENERAL | LOG_UNKNOWN)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(ROLAND_XP, roland_xp_device, "roland_xp", "Roland XP")

namespace {

constexpr u32 hold_masks[4] = { 0, 7, 31, 127 };
constexpr u8 hold_shifts[4] = { 0, 3, 5, 7 };
constexpr u8 phase_dither[4] = { 0, 2, 1, 3 };
constexpr u8 shift_select[4] = { 0, 1, 2, 4 };

// the readback latch takes a page at its own width
u32 page_mask(int index)
{
	if (index <= 0x04 || (index >= 0x20 && index <= 0x26))
		return 0xfffff;
	if (index >= 0x08 && index <= 0x0b)
		return 0xfff;
	if (index >= 0x0c && index <= 0x1e)
		return 0x3ffff;
	if (index == 0x27)
		return 0xffff;
	if (index >= 0x28 && index <= 0x2a)
		return 0xffffff;
	return 0xffffffff;
}

constexpr int bit_reverse4(int v)
{
	return bitswap<4>(v, 0, 1, 2, 3);
}

} // anonymous namespace

const roland_xp_device::ramp_pages roland_xp_device::RAMPS[5] = {
	{ roland_xp_device::PITCH_SEED, roland_xp_device::PITCH_CONTROL, roland_xp_device::PITCH_TARGET, roland_xp_device::PITCH_STEP, roland_xp_device::IRQ_PITCH_DONE },
	{ roland_xp_device::TVF_SEED, roland_xp_device::TVF_CONTROL, roland_xp_device::TVF_TARGET, roland_xp_device::TVF_STEP, roland_xp_device::IRQ_TVF_DONE },
	{ roland_xp_device::RESO_SEED, roland_xp_device::RESO_CONTROL, roland_xp_device::RESO_TARGET, 0, roland_xp_device::IRQ_RESO_DONE },
	{ roland_xp_device::TVA2_SEED, roland_xp_device::TVA2_CONTROL, roland_xp_device::TVA2_TARGET, 0, roland_xp_device::IRQ_TVA2_DONE },
	{ roland_xp_device::TVA1_SEED, roland_xp_device::TVA1_CONTROL, roland_xp_device::TVA1_TARGET, roland_xp_device::TVA1_STEP, roland_xp_device::IRQ_VOICE_DONE },
};


//-------------------------------------------------
//  device
//-------------------------------------------------

roland_xp_device::roland_xp_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: cpu_device(mconfig, ROLAND_XP, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
	, m_pram_config("pram", ENDIANNESS_BIG, 32, 10, -2, address_map_constructor(FUNC(roland_xp_device::pram_map), this))
	, m_wave_config("wave", ENDIANNESS_LITTLE, 8, 27)
	, m_iram_config("iram", ENDIANNESS_BIG, 32, 10, -2, address_map_constructor(FUNC(roland_xp_device::iram_map), this))
	, m_eram_config("eram", ENDIANNESS_BIG, 32, 18, -2, address_map_constructor(FUNC(roland_xp_device::eram_map), this))
	, m_int_callback(*this)
	, m_port_out_cb(*this)
	, m_port_a_in_cb(*this, 0)
	, m_port_b_in_cb(*this, 0)
	, m_frame_cb(*this)
	, m_stream(nullptr)
	, m_bus_written(0)
	, m_run_mask(0)
	, m_run_pending(0)
	, m_read_latch(0)
	, m_write_latch(0)
	, m_frame_counter(0)
	, m_irq_event(0)
	, m_irq_active(false)
	, m_irq_frame_used(false)
	, m_int_state(false)
	, m_landing_valid(0)
	, m_program_dirty(true)
	, m_dsp_enabled(false)
	, m_parity(0)
	, m_icount(0)
	, m_pc(0)
	, m_cycle(0)
	, m_position(0)
	, m_strobe_a(0)
	, m_strobe_bcd(0)
	, m_port_a_out{ 0 }
	, m_port_word{ { 0 } }
	, m_port_a_in(0)
	, m_port_b_in(0)
	, m_port_in_enabled(false)
	, m_pumped(false)
{
}

void roland_xp_device::pram_map(address_map &map)
{
	map(0x000, 0x3ff).rw(FUNC(roland_xp_device::pram_r), FUNC(roland_xp_device::pram_w));
}

void roland_xp_device::iram_map(address_map &map)
{
	map(0x000, 0x3ff).rw(FUNC(roland_xp_device::iram_r), FUNC(roland_xp_device::iram_w));
}

void roland_xp_device::eram_map(address_map &map)
{
	map(0x00000, 0x3ffff).rw(FUNC(roland_xp_device::eram_r), FUNC(roland_xp_device::eram_w));
}

device_memory_interface::space_config_vector roland_xp_device::memory_space_config() const
{
	return space_config_vector {
		std::make_pair(AS_PROGRAM, &m_pram_config),
		std::make_pair(AS_WAVE, &m_wave_config),
		std::make_pair(AS_IRAM, &m_iram_config),
		std::make_pair(AS_ERAM, &m_eram_config)
	};
}

std::unique_ptr<util::disasm_interface> roland_xp_device::create_disassembler()
{
	return std::make_unique<roland_xp_disassembler>(static_cast<roland_xp_disassembler::info *>(this));
}

u32 roland_xp_device::pram_r(offs_t address)
{
	const int word = (PRAM_BASE >> 1) + (address % DSP_SLOTS) * 2;
	return (u32(m_regs[word]) << 16) | m_regs[word | 1];
}

void roland_xp_device::pram_w(offs_t address, u32 data)
{
	const int word = (PRAM_BASE >> 1) + (address % DSP_SLOTS) * 2;
	m_regs[word] = u16(data >> 16);
	m_regs[word | 1] = u16(data);
	m_program_dirty = true;
}

void roland_xp_device::device_start()
{
	m_regs = make_unique_clear<u16[]>(0x2000);
	m_eram = make_unique_clear<s32[]>(ERAM_SIZE);
	space(AS_WAVE).cache(m_wave_cache);
	m_stream = stream_alloc(0, OUTPUTS, clock() / 768, STREAM_SYNCHRONOUS);

	for (int i = 0; i <= 256; i++)
		m_exp_table[i] = s32(std::floor(std::exp2(14.0 + i / 256.0) + 0.5));

	set_icountptr(m_icount);

	state_add(STATE_GENPC, "GENPC", m_pc).noshow();
	state_add(STATE_GENPCBASE, "CURPC", m_pc).noshow();
	state_add(0, "PC", m_pc);
	state_add(1, "ACC", m_dsp.acc);
	state_add(2, "R", m_dsp.r);
	state_add(3, "P", m_dsp.product);
	state_add(4, "LATCH", m_dsp.latch);
	state_add(5, "G", m_dsp.gain);
	state_add(6, "CURSOR", m_dsp.cursor);
	state_add(7, "CYCLE", m_cycle);

	save_pointer(NAME(m_regs), 0x2000);
	save_pointer(NAME(m_eram), ERAM_SIZE);
	save_item(NAME(m_iram));
	save_item(NAME(m_iram_ramping));
	save_item(NAME(m_dsp.acc));
	save_item(NAME(m_dsp.product));
	save_item(NAME(m_dsp.product_prev));
	save_item(NAME(m_dsp.r));
	save_item(NAME(m_dsp.input));
	save_item(NAME(m_dsp.input_prev));
	save_item(NAME(m_dsp.now));
	save_item(NAME(m_dsp.latch));
	save_item(NAME(m_dsp.gain));
	save_item(NAME(m_dsp.payback));
	save_item(NAME(m_dsp.now_valid));
	save_item(NAME(m_dsp.cursor));
	save_item(NAME(m_landing));
	save_item(NAME(m_landing_valid));
	save_item(NAME(m_dsp_enabled));
	save_item(NAME(m_parity));
	save_item(NAME(m_pc));
	save_item(NAME(m_cycle));
	save_item(NAME(m_position));
	save_item(NAME(m_strobe_a));
	save_item(NAME(m_strobe_bcd));
	save_item(NAME(m_port_a_out));
	save_item(NAME(m_port_word));
	save_item(NAME(m_port_a_in));
	save_item(NAME(m_port_b_in));
	save_item(NAME(m_port_in_enabled));
	save_item(NAME(m_bus_written));
	save_item(NAME(m_run_mask));
	save_item(NAME(m_run_pending));
	save_item(NAME(m_read_latch));
	save_item(NAME(m_write_latch));
	save_item(NAME(m_frame_counter));
	save_item(NAME(m_irq_event));
	save_item(NAME(m_irq_active));
	save_item(NAME(m_irq_frame_used));
	save_item(NAME(m_int_state));
	save_item(STRUCT_MEMBER(m_voices, phase));
	save_item(STRUCT_MEMBER(m_voices, format));
	save_item(STRUCT_MEMBER(m_voices, fade_entry));
	save_item(STRUCT_MEMBER(m_voices, preloaded));
	save_item(STRUCT_MEMBER(m_voices, advanced));
	save_item(STRUCT_MEMBER(m_voices, sample));
}

void roland_xp_device::device_reset()
{
	std::fill_n(m_regs.get(), 0x2000, 0);
	std::fill_n(m_eram.get(), ERAM_SIZE, 0);
	m_bus_written = 0;
	std::fill(std::begin(m_iram), std::end(m_iram), 0);
	std::fill(std::begin(m_iram_ramping), std::end(m_iram_ramping), 0);
	for (voice &v : m_voices)
		v = voice();
	m_dsp = dsp_state();
	m_dsp.latch = -0x800000;
	std::fill(std::begin(m_landing), std::end(m_landing), 0);
	m_landing_valid = 0;
	m_program_dirty = true;
	m_dsp_enabled = false;
	m_parity = 0;
	m_pc = 0;
	m_cycle = 0;

	m_position = 0;
	m_strobe_a = 0;
	m_strobe_bcd = 0;
	std::fill(std::begin(m_port_a_out), std::end(m_port_a_out), 0);
	for (auto &port : m_port_word)
		port[0] = port[1] = 0;
	m_port_a_in = 0;
	m_port_b_in = 0;
	m_port_in_enabled = false;

	m_run_mask = 0;
	m_run_pending = 0;
	m_read_latch = 0;
	m_write_latch = 0;
	m_frame_counter = 0;
	m_irq_event = 0;
	m_irq_active = false;
	m_irq_frame_used = false;
	update_int();
}

void roland_xp_device::device_clock_changed()
{
	m_stream->set_sample_rate(frame_rate());
}

void roland_xp_device::device_post_load()
{
	m_program_dirty = true;
}

void roland_xp_device::update_int()
{
	if (m_irq_active != m_int_state)
	{
		m_int_state = m_irq_active;
		m_int_callback(m_int_state ? ASSERT_LINE : CLEAR_LINE);
	}
}

bool roland_xp_device::offer_irq(int voice, int reason)
{
	if (!BIT(m_regs[IRQ_STATUS >> 1], reason))
		return true;
	if (m_irq_active || m_irq_frame_used)
		return false;

	m_irq_event = u16((voice << 8) | reason);
	m_irq_active = true;
	m_irq_frame_used = true;
	update_int();
	return true;
}


//-------------------------------------------------
//  host interface
//-------------------------------------------------

u32 roland_xp_device::page(int voice, int index) const
{
	const int word = page_word(voice, index);
	return (u32(m_regs[word]) << 16) | m_regs[word | 1];
}

void roland_xp_device::set_page(int voice, int index, u32 value)
{
	const int word = page_word(voice, index);
	m_regs[word] = u16(value >> 16);
	m_regs[word | 1] = u16(value);
}

void roland_xp_device::load_latch(offs_t address, offs_t lane)
{
	if (address < CRAM_BASE)
	{
		if (BIT(address, 1))
			m_read_latch = page((address >> 2) & 63, address >> 8) & page_mask(address >> 8);
	}
	else if (address < IRAM_BASE)
		m_read_latch = m_regs[address >> 1];
	else if (address < IRAM3_TARGET_BASE)
	{
		if (BIT(address, 1))
		{
			const int c = host_cell(address);
			m_read_latch = u32(m_iram[c]) & (c >= 128 ? 0x3ffffff : 0xffffff);
		}
	}
	else if (address < PRAM_BASE)
		m_read_latch = 0;
	else if (address < RUN_MASK)
	{
		if (BIT(address, 1))
			m_read_latch = ((u32(m_regs[(address >> 1) & ~1]) << 16) | m_regs[address >> 1]) & 0x0fffffff;
	}
	else if (address >= SEND_BASE && address < ROM_WINDOW)
		m_read_latch = m_regs[address >> 1];
	else if (address >= ROM_WINDOW)
	{
		const u32 base = (u32(m_regs[ROM_BANK >> 1] & 0x7f) << 20) | (u32(m_regs[ROM_PAGE >> 1] & 0x3ff) << 10);
		const u32 select = base >> 24;
		const u8 descriptor = m_regs[(ROM_SELECT >> 1) + (select >> 1)] >> (BIT(select, 0) ? 8 : 0);
		const u32 byte = base | ((address - ROM_WINDOW) + (BIT(descriptor, 0) ? 0 : lane));
		if (BIT(descriptor, 0))
			m_read_latch = m_wave_cache.read_byte(byte) | (u32(m_wave_cache.read_byte(byte + 1)) << 8);
		else
			m_read_latch = m_wave_cache.read_byte(byte);
	}
}

u16 roland_xp_device::read(offs_t offset, u16 mem_mask)
{
	offs_t address = (offset << 1) & 0x3ffe;
	u16 data = 0;

	if (address >= VOICE_WINDOW && address < VOICE_WINDOW_END)
		address = ((address - VOICE_WINDOW) >> 2) * 0x100 + (m_regs[VOICE_SELECT >> 1] & 0x3f) * 4 + (address & 2);
	else if (address >= SEND_WINDOW && address < SEND_BASE)
		address = SEND_BASE + ((address - SEND_WINDOW) >> 1) * 0x80 + (m_regs[VOICE_SELECT >> 1] & 0x3f) * 2;

	if (address < RUN_MASK || address >= SEND_BASE)
	{
		if (!machine().side_effects_disabled())
			load_latch(address, (mem_mask == 0x00ff) ? 1 : 0);
	}
	else
	{
		if (address < ROM_SELECT && !machine().side_effects_disabled())
			commit_run_mask();

		switch (address)
		{
		case RUN_MASK: case RUN_MASK + 2: case RUN_MASK + 4: case RUN_MASK + 6:
		case DSP_MODE:
		case VOICE_SELECT:
			break;
		case READBACK_LOW:
			data = m_read_latch & 0xffff;
			break;
		case READBACK_HIGH:
			data = m_read_latch >> 16;
			break;
		case IRQ_STATUS:
			data = m_irq_event;
			break;
		case IRQ_ACK:
			if (!machine().side_effects_disabled())
			{
				m_irq_active = false;
				update_int();
			}
			break;
		case STATUS:
			data = (m_regs[STATUS >> 1] & ~0x40) | (((m_regs[DIAG_SELECT >> 1] & 0x7ff) >= 0x5a0) ? 0x40 : 0);
			break;
		default:
			data = m_regs[address >> 1];
			break;
		}
	}

	if (!machine().side_effects_disabled())
		LOGMASKED(LOG_READ, "%s: read %04x = %04x\n", machine().describe_context(), address, data);
	return data;
}

void roland_xp_device::write(offs_t offset, u16 data, u16 mem_mask)
{
	offs_t address = (offset << 1) & 0x3ffe;

	if (address >= VOICE_WINDOW && address < VOICE_WINDOW_END)
		address = ((address - VOICE_WINDOW) >> 2) * 0x100 + (m_regs[VOICE_SELECT >> 1] & 0x3f) * 4 + (address & 2);
	else if (address >= SEND_WINDOW && address < SEND_BASE)
		address = SEND_BASE + ((address - SEND_WINDOW) >> 1) * 0x80 + (m_regs[VOICE_SELECT >> 1] & 0x3f) * 2;

	if (address < CRAM_BASE)
	{
		LOGMASKED(LOG_VOICE, "%s: voice %2d page %02x%s = %04x\n", machine().describe_context(), (address >> 2) & 0x3f, address >> 8, BIT(address, 1) ? "+2" : "  ", data);
		if (!BIT(address, 1))
			m_write_latch = data;
		else
		{
			const int n = (address >> 2) & 0x3f;
			const int index = address >> 8;
			u32 value = (u32(m_write_latch) << 16) | data;
			if (index == CONTROL && m_voices[n].phase == RUNNING && !BIT(value, 10))
				value |= page(n, CONTROL) & 0x20080;
			else if (index == TVA1_CONTROL && ((value >> 14) & 3) == 2)
			{
				value |= 0x4000;
				m_voices[n].fade_entry = 1;
			}
			set_page(n, index, value);
		}
	}
	else if (address < IRAM_BASE)
	{
		LOGMASKED(LOG_CRAM, "%s: cram %3d = %04x\n", machine().describe_context(), (address - CRAM_BASE) >> 1, data);
		m_regs[address >> 1] = data;
		m_program_dirty = true;
	}
	else if (address < IRAM3_TARGET_BASE)
	{
		LOGMASKED(LOG_IRAM, "%s: iram%d %2d%s = %04x\n", machine().describe_context(), ((address - IRAM_BASE) >> 8) + 1, (address & 0xff) >> 2, BIT(address, 1) ? "+2" : "  ", data);
		if (!BIT(address, 1))
			m_write_latch = data;
		else
			write_iram(host_cell(address), (u32(m_write_latch) << 16) | data);
	}
	else if (address < PRAM_BASE)
	{
		LOGMASKED(LOG_IRAM, "%s: iram4 %2d   = %04x\n", machine().describe_context(), (address & 0xff) >> 1, data);
		m_regs[address >> 1] = data;
		write_iram_target(0xe0 + ((address >> 1) & 0x1f), data);
	}
	else if (address < RUN_MASK)
	{
		LOGMASKED(LOG_PRAM, "%s: pram %3d%s = %04x\n", machine().describe_context(), (address - PRAM_BASE) >> 2, BIT(address, 1) ? "+2" : "  ", data);
		if (!BIT(address, 1))
			m_write_latch = data;
		else
		{
			m_regs[(address >> 1) & ~1] = m_write_latch;
			m_regs[address >> 1] = data;
			m_program_dirty = true;
		}
	}
	else if (address < SEND_BASE)
	{
		m_regs[address >> 1] = data;
		if (address < ROM_SELECT)
		{
			LOGMASKED(LOG_RUNMASK, "%s: runmask %04x = %04x\n", machine().describe_context(), address, data);
			write_run_mask((address - RUN_MASK) >> 1, data);
		}
		else
		{
			LOGMASKED(LOG_CTRL, "%s: ctrl %04x = %04x\n", machine().describe_context(), address, data);
			if (address == HIGHEST_VOICE)
			{
				m_stream->set_sample_rate(frame_rate());
				m_program_dirty = true;
			}
			else if (address == DSP_CONFIG)
				m_program_dirty = true;
		}
	}
	else if (address < ROM_WINDOW)
	{
		LOGMASKED(LOG_MIX, "%s: mix bank %d voice %2d = %04x\n", machine().describe_context(), (address - SEND_BASE) >> 7, (address >> 1) & 0x3f, data);
		m_regs[address >> 1] = data;
	}
	else
		LOGMASKED(LOG_CTRL, "%s: window write %04x = %04x\n", machine().describe_context(), address, data);
}

void roland_xp_device::write_run_mask(int word, u16 data)
{
	const u64 written = u64(data) << (word * 16);
	const u64 field = u64(0xffff) << (word * 16);
	const u64 cleared = m_run_mask & field & ~written;

	m_run_mask &= ~cleared;
	m_run_pending = (m_run_pending & ~field) | (written & ~m_run_mask);

	for (int n = word * 16; n < word * 16 + 16; n++)
		if (BIT(cleared, n))
			m_voices[n].phase = IDLE;
}

void roland_xp_device::commit_run_mask()
{
	const u64 launched = m_run_pending;
	if (!launched)
		return;

	m_run_mask |= launched;
	m_run_pending = 0;
	for (int n = 0; n < MAX_VOICES; n++)
		if (BIT(launched, n))
			m_voices[n].phase = PRELOAD;
}


//-------------------------------------------------
//  ramps
//-------------------------------------------------

s32 roland_xp_device::exp_sum(s32 value) const
{
	const int index = (value >> 6) & 0xff;
	const int fraction = value & 0x3f;
	return (((64 - fraction) * m_exp_table[index]) >> 2) + ((fraction * m_exp_table[index + 1]) >> 2);
}

s32 roland_xp_device::exp_decode(s32 value) const
{
	if (value == 0)
		return 0;

	return (exp_sum(value) >> 1) >> (15 - ((value >> 14) & 15));
}

s32 roland_xp_device::cutoff_decode(s32 value) const
{
	if (value == 0)
		return 0;

	return ((exp_sum(value) << 1) >> (15 - ((value >> 14) & 15))) & 0xffffe;
}

bool roland_xp_device::linear_law(int index, u32 control)
{
	switch (index)
	{
	case RAMP_PITCH: case RAMP_TVF: case RAMP_TVA1: return ((control >> 14) & 3) == 1;
	default: return false;
	}
}

s32 roland_xp_device::linear_step(s32 target, s32 current, int rate)
{
	const s32 diff = target - current;
	return 2 * (((diff >> 3) * rate) >> 10) + (diff > 0 ? 1 : 0);
}

bool roland_xp_device::s_curve_law(int index, u32 control)
{
	return index == RAMP_TVA1 && ((control >> 14) & 3) >= 2;
}

void roland_xp_device::offer_terminal(int n, const ramp_pages &rp, u32 control)
{
	if (BIT(control, 16) && BIT(m_regs[IRQ_STATUS >> 1], rp.reason) && offer_irq(n, rp.reason))
		set_page(n, rp.control, control | 0x20000);
}

roland_xp_device::ramp_clock roland_xp_device::ramp_clock_of(int index, u32 control) const
{
	const u32 clock = (index == RAMP_TVA1) ? (m_frame_counter >> 1) : (m_frame_counter >> 3);
	const int divider = (control >> 12) & 3;
	return ramp_clock{ !(clock & hold_masks[divider]), (index == RAMP_RESO) ? 0 : s32(((clock >> hold_shifts[divider]) & 1) ^ 1) };
}

s32 roland_xp_device::step_ramp(int n, int k, u32 control, s32 current, s32 target, s32 step, s32 parity)
{
	const bool reso = k == RAMP_RESO;
	if (linear_law(k, control))
	{
		if (current != target)
		{
			const s32 moved = current + ((step + ((current < target) ? parity : 0)) >> 1);
			current = (target > current) ? std::min(moved, target) : std::max(moved, target);
		}
	}
	else
	{
		const int rate = control & 0xfff;
		const s32 diff = target - current;
		const int quantum = reso ? 4 : 1;
		s32 s = ((((diff / quantum) >> 3) * rate) >> 10) * quantum;
		if ((diff > 0 && current + s < target) || (!rate && diff))
			s += reso ? 2 : parity;
		current = std::max(current + s, 0) & (reso ? 0xffffe : 0x3ffff);
	}
	set_page(n, RAMPS[k].current, u32(current));
	return current;
}

s32 roland_xp_device::ramp_target(int n, int k) const
{
	const s32 target = s32(page(n, RAMPS[k].target) & 0x3fffe);
	return (k == RAMP_RESO) ? (target << 2) : target;
}

s32 roland_xp_device::ramp_current(int n, int k) const
{
	return s32(page(n, RAMPS[k].current) & ((k == RAMP_RESO) ? 0xfffff : 0x3ffff));
}

void roland_xp_device::launch_ramp(int n, int k)
{
	const ramp_pages &rp = RAMPS[k];
	const u32 control = page(n, rp.control);
	if (BIT(control, 17))
		return;

	const ramp_clock clock = ramp_clock_of(k, control);
	if (!clock.service)
		return;

	const s32 target = ramp_target(n, k);
	const s32 current = ramp_current(n, k);
	const s32 step = linear_law(k, control) ? linear_step(target, current, control & 0xfff) : 0;
	if (step_ramp(n, k, control, current, target, step, clock.parity) == target)
		offer_terminal(n, rp, control);
}

void roland_xp_device::fade_ramp(int n, u32 control, const ramp_clock &clock)
{
	const ramp_pages &rp = RAMPS[RAMP_TVA1];
	if (BIT(control, 17))
		return;

	const s32 previous = ramp_current(n, RAMP_TVA1);
	if (!(previous >> 1))
		offer_terminal(n, rp, control);
	if (!clock.service)
		return;

	const int rate = control & 0xfff;
	const s32 velocity = wrap20(s32(page(n, rp.step)));
	const s32 next = previous + ((velocity + clock.parity) >> 1);
	set_page(n, rp.current, u32((next < 0 || next > 0x3ffff) ? 0 : next));

	s32 steer;
	if (!(previous >> 1))
		steer = (velocity < 0) ? (velocity + rate) : -1;
	else
		steer = (previous > ramp_target(n, RAMP_TVA1)) ? (velocity - rate) : (velocity + rate);
	set_page(n, rp.step, u32(std::min(steer, -1)) & 0xfffff);
}

void roland_xp_device::service_ramp(int n, int k)
{
	const ramp_pages &rp = RAMPS[k];
	const u32 control = page(n, rp.control);
	const ramp_clock clock = ramp_clock_of(k, control);

	if (k == RAMP_TVA1 && m_voices[n].fade_entry)
	{
		m_voices[n].fade_entry = 0;
		set_page(n, rp.target, ((u32(ramp_current(n, k)) >> 3) << 2) | 1);
		if (!BIT(control, 17) && clock.service)
			set_page(n, rp.step, 0);
		return;
	}

	if (s_curve_law(k, control))
	{
		fade_ramp(n, control, clock);
		return;
	}

	const bool linear = linear_law(k, control);
	const u32 tpage = page(n, rp.target);
	const s32 target = ramp_target(n, k);
	const s32 current = ramp_current(n, k);

	if (linear && !BIT(tpage, 0))
	{
		set_page(n, rp.target, tpage | 1);
		set_page(n, rp.step, u32(linear_step(target, current, control & 0xfff)) & 0xfffff);
		return;
	}

	if (BIT(control, 17))
		return;

	const bool reports_on_service = linear && (k == RAMP_TVF || k == RAMP_TVA1);
	if (!reports_on_service && current == target)
	{
		offer_terminal(n, rp, control);
		return;
	}

	if (!clock.service)
		return;

	const s32 step = linear ? wrap20(s32(page(n, rp.step))) : 0;
	if (step_ramp(n, k, control, current, target, step, clock.parity) == target && reports_on_service)
		offer_terminal(n, rp, control);
}

void roland_xp_device::update_amplitude(int n)
{
	const s32 tva1 = page(n, TVA1_SEED) & 0x3ffff;
	const s32 tva2 = page(n, TVA2_SEED) & 0x3ffff;
	s32 amplitude;
	if (BIT(page(n, TVA2_CONTROL), 14))
		amplitude = s32(std::clamp<s64>((s64(tva1) + tva2) << 2, 0, 0xffffe)) & ~1;
	else
		amplitude = s32((s64(tva1 >> 3) * (tva2 >> 4)) >> 8) & ~1;
	set_page(n, AMPLITUDE, u32(amplitude));
}


//-------------------------------------------------
//  address generator and DPCM
//-------------------------------------------------

roland_xp_device::wave_cell roland_xp_device::cell_at(int n, u32 control, u32 address)
{
	const u8 byte = rom_byte(control, address);
	const voice &v = m_voices[n];
	if (BIT(v.format, 1))
	{
		const int shift = (byte >> 4) & 7;
		const int mantissa = shift ? ((byte & 0x0f) + 16) : (byte & 0x0f);
		return wave_cell{ BIT(byte, 7) ? -mantissa : mantissa, shift ? (shift + 5) : 6 };
	}
	if (BIT(v.format, 0))
		return wave_cell{ s8(byte), 0 };

	const u8 shifts = rom_byte(control, address >> 5);
	const int exponent = BIT(address, 4) ? (shifts >> 4) : (shifts & 0x0f);
	return wave_cell{ (exponent > 10) ? 0 : s8(byte), exponent };
}

s32 roland_xp_device::delta_of(wave_cell c)
{
	return c.mantissa << c.exponent;
}

s32 roland_xp_device::tap(s32 weight, wave_cell c)
{
	const s32 p = (weight * c.mantissa) & ~3;
	return (c.exponent <= 10) ? (p >> (10 - c.exponent)) : (p << (c.exponent - 10));
}

void roland_xp_device::launch(int n)
{
	voice &v = m_voices[n];
	u32 control = page(n, CONTROL);

	v.format = (BIT(control, 9) << 1) | BIT(control, 7);
	v.advanced = 0;
	v.preloaded = !(BIT(control, 19) && !BIT(control, 18));
	if (!v.preloaded)
		return;

	set_page(n, CONTROL, control | 0x80);

	const u32 address = page(n, ADDRESS) & 0xfffff;
	const u32 span = (address >> 5) & ~1;
	set_page(n, EXPONENTS, rom_byte(control, span) | (u32(rom_byte(control, span + 1)) << 8));
}

roland_xp_device::address_step roland_xp_device::advance(int n, u32 control, address_step s) const
{
	const u32 loop = page(n, LOOP) & 0xfffff;
	const u32 end = page(n, END) & 0xfffff;
	const bool looping = loop < end;
	const bool alternate = BIT(control, 12);

	if (!s.backward)
	{
		if (!looping)
			return { s.address >= end ? s.address : s.address + 1, false };
		if (s.address >= end)
			return alternate ? address_step{ s.address, true } : address_step{ loop, false };
		return { s.address + 1, false };
	}

	if (s.address <= loop)
	{
		if (alternate && looping)
			return { s.address, false };
		return { s.address, true };
	}
	return { s.address - 1, true };
}

bool roland_xp_device::wide_bus(u32 control) const
{
	const u32 select = (control >> 4) & 7;
	return BIT(m_regs[(ROM_SELECT >> 1) + (select >> 1)] >> (BIT(select, 0) ? 8 : 0), 0);
}

bool roland_xp_device::at_marker(int n, u32 control, address_step s) const
{
	const u32 loop = page(n, LOOP) & 0xfffff;
	const u32 pair = wide_bus(control) ? 1 : 0;
	return s.backward ? ((s.address & ~pair) <= loop) : ((s.address | pair) >= loop);
}

void roland_xp_device::marker_reached(int n)
{
	const u32 control = page(n, CONTROL);
	if (BIT(control, 16) || (BIT(control, 12) && BIT(control, 14)))
		return;
	if (BIT(control, 15) && !offer_irq(n, BIT(control, 14) ? IRQ_LOOP_ALTERNATE : IRQ_LOOP_REACHED))
		return;

	set_page(n, CONTROL, control | (BIT(control, 17) ? 0x10000 : 0x20000));
}

void roland_xp_device::update_mute(int n)
{
	const u32 control = page(n, CONTROL);
	if (BIT(control, 19) == BIT(control, 18))
		return;
	if (BIT(control, 19))
	{
		if (!BIT(m_regs[IRQ_STATUS >> 1], IRQ_MUTE_CHANGED) || !offer_irq(n, IRQ_MUTE_CHANGED))
			return;
	}
	else if (BIT(control, 15) && !offer_irq(n, IRQ_MUTE_CHANGED))
		return;

	set_page(n, CONTROL, control ^ 0x40000);
}


//-------------------------------------------------
//  the voice
//-------------------------------------------------

void roland_xp_device::deposit(int n, int bank)
{
	const u16 s = send(n, bank);
	const int word = s & 63;
	const int c = cell(0x40 + word, m_parity ^ 1);

	if (!BIT(m_bus_written, word))
	{
		m_bus_written |= u64(1) << word;
		m_iram[c] = 0;
	}
	if (bank == 2 && word == (send(n, 3) & 63))
		return;
	const s32 output = wrap24(s32(page(n, OUTPUT)));
	m_iram[c] = clamp24(m_iram[c] + (s64(output) * (s >> 6)) / 512);
}

s32 roland_xp_device::interpolate(int n, u32 control, u32 service)
{
	const u32 phase = (page(n, PHASE) >> 4) & 0x3fff;
	address_step s{ page(n, ADDRESS) & 0xfffff, bool(BIT(control, 11) ^ BIT(control, 13)) };
	s32 sum = 4 * wrap18(s32(page(n, PREDICTOR)));
	for (int i = 0; i < 3; i++)
	{
		sum += tap(roland_pcm_interp::WEIGHTS[i][phase >> 7], cell_at(n, control, s.address));
		s = advance(n, control, s);
	}
	return wrap20(sum) >> (3 - ((service >> 3) & 3));
}

void roland_xp_device::read_wave(int n, u32 control)
{
	const u32 packed = page(n, PHASE);
	u32 phase = (packed >> 4) & 0x3fff;
	const u32 address = page(n, ADDRESS) & 0xfffff;
	s32 predictor = wrap18(s32(page(n, PREDICTOR)));

	const u32 increment = page(n, INCREMENT) & 0x3ffff;
	const u32 span = address >> 6;
	const u32 accumulated = phase + (increment >> 2) + (((increment & 3) > phase_dither[m_frame_counter & 3]) ? 1 : 0);
	phase = accumulated & 0x3fff;
	address_step current{ address, bool(BIT(control, 11) ^ BIT(control, 13)) };
	for (u32 carry = accumulated >> 14; carry; carry--)
	{
		predictor = wrap18(predictor + delta_of(cell_at(n, control, current.address)));
		if (at_marker(n, control, current))
			marker_reached(n);
		const address_step next = advance(n, control, current);
		if (next.backward != current.backward)
		{
			control = page(n, CONTROL) ^ 0x2000;
			set_page(n, CONTROL, control);
		}
		current = next;
	}
	if ((current.address >> 6) != span)
	{
		const u32 exponents = (current.address >> 5) & ~1;
		set_page(n, EXPONENTS, rom_byte(control, exponents) | (u32(rom_byte(control, exponents + 1)) << 8));
	}
	set_page(n, PHASE, (phase << 4) | ((packed + (accumulated >> 14)) & 15));
	set_page(n, ADDRESS, current.address);
	set_page(n, PREDICTOR, u32(predictor) & 0x3ffff);
}

void roland_xp_device::run_voice(int n)
{
	voice &v = m_voices[n];

	if (!running(n))
	{
		set_page(n, FILTER_BAND, 0);
		set_page(n, FILTER_LOW, 0);
		set_page(n, OUTPUT, 0);
		return;
	}

	if (v.phase == STARTING || v.phase == RUNNING)
	{
		const s32 smooth = page(n, SMOOTH) & 0xffff;
		set_page(n, SMOOTH, u32(std::clamp<s32>((7 * smooth + s32((page(n, AMPLITUDE) & 0xfffff) >> 4) + 3) >> 3, 0, 0xffff)));
	}

	update_mute(n);

	switch (v.phase)
	{
	case PRELOAD:
		launch(n);
		v.phase = INITIALIZE;
		return;

	case INITIALIZE:
		set_page(n, INCREMENT, u32(exp_decode(page(n, PITCH_SEED) & 0x3ffff)) & 0x3ffff);
		set_page(n, CUTOFF, u32(cutoff_decode(page(n, TVF_SEED) & 0x3ffff)));
		launch_ramp(n, RAMP_PITCH);
		set_page(n, SERVICE, (page(n, SERVICE) & 0x1f) | 0x10000);
		v.phase = STARTING;
		return;

	case STARTING:
		launch_ramp(n, RAMP_TVF);
		set_page(n, SERVICE, (page(n, SERVICE) & 0x1f) | 0x30000);
		v.phase = RUNNING;
		return;

	default:
		break;
	}

	const u32 service = page(n, SERVICE);
	const int counter = (service + 1) & 7;
	set_page(n, SERVICE, (service & ~7) | counter);

	const u32 control = page(n, CONTROL);
	const bool sounding = sounds(control);
	s32 sample = 0;
	if (v.preloaded && !BIT(control, 10) && !(control & 0xc0000))
	{
		sample = interpolate(n, control, service);
		read_wave(n, control);
		v.advanced = 1;
	}
	else if (sounding && v.advanced)
		sample = interpolate(n, control, service);

	if (counter == 6)
		set_page(n, INCREMENT, u32(exp_decode(page(n, PITCH_SEED) & 0x3ffff)) & 0x3ffff);

	if (counter & 1)
	{
		update_amplitude(n);
		service_ramp(n, RAMP_TVA1);
	}
	else
	{
		switch (counter)
		{
		case 0:
			service_ramp(n, RAMP_TVA2);
			break;
		case 2:
			service_ramp(n, RAMP_RESO);
			break;
		case 4:
			set_page(n, CUTOFF, u32(cutoff_decode(page(n, TVF_SEED) & 0x3ffff)));
			service_ramp(n, RAMP_TVF);
			break;
		default:
			service_ramp(n, RAMP_PITCH);
			break;
		}
	}

	if (!sounding)
		return;

	if (pairs(n & ~1))
	{
		v.sample = sample;
		return;
	}

	set_page(n, OUTPUT, u32(amplify(n, filter(n, (page(n, FILTER) >> 10) & 3, sample))) & 0xffffff);
}

s32 roland_xp_device::filter(int n, int mode, s32 in)
{
	const s32 f = page(n, CUTOFF) & 0xffff0;
	const s32 q = page(n, RESO_SEED) & 0xffff0;
	s32 low = wrap24(s32(page(n, FILTER_LOW)));
	s32 band = wrap24(s32(page(n, FILTER_BAND)));
	low = clamp24(low + (s64(f) * band) / (1 << 19));
	const s32 high = s32(std::clamp<s64>(in - (s32((s64(q) * band) / (1 << 19)) + low), -0x7fffff, 0x7fffff));
	band = clamp24(band + (s64(f) * high) / (1 << 19));
	set_page(n, FILTER_LOW, u32(low) & 0xffffff);
	set_page(n, FILTER_BAND, u32(band) & 0xffffff);
	switch (mode)
	{
	case 0: return low;
	case 1: return band;
	case 2: return high;
	default: return clamp24(s64(high) - low);
	}
}

s32 roland_xp_device::amplify(int n, s32 value) const
{
	return clamp24((s64(value) * ((page(n, SMOOTH) & 0xffff) << 4)) / (1 << 19));
}

bool roland_xp_device::pairs(int n) const
{
	const u32 control = page(n, FILTER);
	const int structure = (control >> 12) & 15;
	return !BIT(n, 0) && BIT(control, 4) && structure >= 1 && structure <= 9;
}

void roland_xp_device::run_pair(int n)
{
	const int m = n + 1;
	const auto sounding = [this] (int k) { return running(k) && m_voices[k].phase == RUNNING && sounds(page(k, CONTROL)); };
	const bool sounding1 = sounding(n);
	const bool sounding2 = sounding(m);
	if (!sounding1 && !sounding2)
		return;

	set_page(m, OUTPUT, 0);

	const u32 control = page(n, FILTER);
	const int booster = (control >> 6) & 3;
	const int mode1 = (control >> 10) & 3;
	const int mode2 = (control >> 8) & 3;
	const s32 w1 = sounding1 ? m_voices[n].sample : 0;
	const s32 w2 = sounding2 ? m_voices[m].sample : 0;

	const auto boost = [booster] (s32 x) { return clamp16(s64(x) << booster); };
	const auto ring = [] (s32 modulator, s32 carrier) { return clamp24((s64(clamp16(modulator)) * carrier) / 32768); };
	const auto f1 = [this, n, mode1] (s32 x) { return filter(n, mode1, x); };
	const auto f2 = [this, m, mode2] (s32 x) { return filter(m, mode2, x); };

	s32 out;
	switch ((control >> 12) & 15)
	{
	case 1: out = f2(f1(clamp24(s64(amplify(n, w1)) + w2))); break;
	case 2: out = f2(boost(f1(clamp24(s64(amplify(n, w1)) + w2)))); break;
	case 3: out = f2(f1(boost(clamp24(s64(amplify(n, w1)) + w2)))); break;
	case 4: out = f2(f1(ring(amplify(n, w1), w2))); break;
	case 5: out = f2(f1(clamp24(s64(ring(amplify(n, w1), w2)) + w2))); break;
	case 6: out = f2(ring(amplify(n, f1(w1)), w2)); break;
	case 7: out = f2(clamp24(s64(ring(amplify(n, f1(w1)), w2)) + w2)); break;
	case 8:
	{
		const s32 modulator = amplify(n, f1(w1));
		out = ring(modulator, f2(w2));
		break;
	}
	default:
	{
		const s32 modulator = amplify(n, f1(w1));
		const s32 carrier = f2(w2);
		out = clamp24(s64(ring(modulator, carrier)) + carrier);
		break;
	}
	}

	set_page(n, OUTPUT, u32(amplify(m, out)) & 0xffffff);
}


//-------------------------------------------------
//  the DSP
//-------------------------------------------------

int roland_xp_device::cell(int word, int parity) const
{
	if (word < 0x80)
		return ((BIT(word, 6) ^ parity) << 6) | (word & 0x3f);
	if (word < 0xc0)
		return (BIT(word, 5) << 6) | 0x20 | (word & 0x1f);
	return 0x80 | (word & 0x3f);
}

s32 roland_xp_device::gain_goal(s32 cell)
{
	const int target = cell & 0x3ff;
	s32 goal = s16(u16(target << 6));
	if (target > 0 && target < 0x1ff)
		goal++;
	return goal;
}

s32 roland_xp_device::fold24(s32 value)
{
	u32 v = u32(value) & 0xffffff;
	if (BIT(v, 23) != BIT(v, 22))
		v ^= 0x7fffff;
	return wrap24(s32(v));
}

s32 roland_xp_device::wire_word(s32 word) const
{
	return clamp24(word) & (BIT(m_regs[SERIAL_FORMAT >> 1], 5) ? ~0x3ff : ~0x3f);
}

void roland_xp_device::write_iram(int c, u32 value)
{
	m_iram[c] = c >= 0x80 ? s32(value << 6) >> 6 : wrap24(s32(value));
	if (c >= 0x80)
		m_iram_ramping[c & 0x3f] = 0;
}

void roland_xp_device::write_iram_target(int word, u16 value)
{
	const int c = cell(word);
	if (word >= ramp_base())
	{
		m_iram[c] = (m_iram[c] & ~0x3ff) | (value & 0x3ff);
		m_iram_ramping[c & 0x3f] = 1;
	}
	else
	{
		m_iram[c] = value;
		m_iram_ramping[c & 0x3f] = 0;
	}
}

void roland_xp_device::update_iram_ramps()
{
	if (!(m_frame_counter & 1))
		return;

	const int threshold = bit_reverse4((m_frame_counter >> 1) & 15);
	for (int word = ramp_base(); word < 256; word++)
	{
		const int c = cell(word);
		if (!m_iram_ramping[c & 0x3f])
			continue;

		const s32 code = m_iram[c] & 0x3ff;
		if (code == 0x200)
		{
			m_iram_ramping[c & 0x3f] = 0;
			continue;
		}

		const s32 exact = s16(u16(code << 6));
		s32 current = gain_current(m_iram[c]);
		if (current == exact)
		{
			m_iram_ramping[c & 0x3f] = 0;
			continue;
		}

		s32 diff = gain_goal(m_iram[c]) - current;
		if (current > exact && !diff)
			diff = -2;
		const s64 product = s64(std::abs(diff)) * m_regs[(IRAM3_RATE >> 1) + (word & 3)];
		s32 step = s32(product >> 17);
		int fraction = int(((product & 0x1ffff) + 0x1000) >> 13);
		if (fraction >= 16)
		{
			step++;
			fraction -= 16;
		}
		if (current < exact)
		{
			if (threshold < fraction)
				step++;
			current = std::min(current + step, exact);
		}
		else
		{
			if (threshold >= 16 - fraction)
				step++;
			current = std::max(current - step, exact);
		}
		m_iram[c] = ((current << 10) | code) & 0x3ffffff;
		if (current == exact)
			m_iram_ramping[c & 0x3f] = 0;
	}
}

void roland_xp_device::decode_program()
{
	for (int i = 0; i < DSP_SLOTS; i++)
	{
		const u32 w = (u32(m_regs[(PRAM_BASE >> 1) + i * 2]) << 16) | m_regs[(PRAM_BASE >> 1) + i * 2 + 1];
		const u16 c = m_regs[(CRAM_BASE >> 1) + i];
		const s32 mantissa = s32(s16(c << 2)) >> 2;
		dsp_slot &s = m_program[i];
		s.st = (w >> 14) & 3;
		s.word = (w >> 6) & 0xff;
		s.input = (w >> 4) & 3;
		s.function = w & 0xf;
		s.ext = (w >> 25) & 7;
		s.eram_op = 0;
		s.eram_second = 0;
		s.eram_offset = 0;
		s.cram = c;
		s.coefficient = mantissa << shift_select[c >> 14];
		s.raw = BIT(c, 15) ? s32(u32(util::sext(c, 15)) << 13) : mantissa;
		s.logic = BIT(c, 15) ? s32((c & 0x7fff) << 13) : (util::sext(c, 15) & 0xffffff);
	}

	for (int i = 0; i < DSP_SLOTS - 1; )
	{
		const u32 w = (u32(m_regs[(PRAM_BASE >> 1) + i * 2]) << 16) | m_regs[(PRAM_BASE >> 1) + i * 2 + 1];
		const u32 next = (u32(m_regs[(PRAM_BASE >> 1) + i * 2 + 2]) << 16) | m_regs[(PRAM_BASE >> 1) + i * 2 + 3];
		const int op = (w >> 23) & 3;
		if (op)
		{
			m_program[i].eram_op = op;
			m_program[i].eram_offset = (((w >> 16) & 0x7f) << 9) | ((next >> 16) & 0x1ff);
			m_program[i + 1].eram_second = 1;
			i += 2;
		}
		else
			i++;
	}

	m_program_dirty = false;
}

s32 roland_xp_device::operand(const dsp_slot &s) const
{
	const dsp_state &d = m_dsp;

	switch (s.input)
	{
	case INPUT_PREVIOUS: return d.input;
	case INPUT_ACC: return clamp24(d.acc);
	case INPUT_R: return d.r;
	default: return d.latch;
	}
}

s32 roland_xp_device::factor(int select, bool complement) const
{
	const s32 acc = clamp24(m_dsp.acc);
	s32 f;

	switch (select)
	{
	case 0: f = (acc & 0xfff) << 3; return complement ? (0x7fff - f) : f;
	case 1: f = (acc & 0x7fffff) >> 8; return complement ? (0x7fff - f) : f;
	case 2: f = acc >> 8; return complement ? ~f : f;
	default: f = m_dsp.gain; return complement ? ~f : f;
	}
}

// the ALU functions both encodings share; returns whether the function is one of them
bool roland_xp_device::alu(int function, int mode, const dsp_slot &s)
{
	dsp_state &d = m_dsp;
	const s32 p = d.product;
	const s32 r = d.r;
	s64 result;

	switch (function)
	{
	case 0x1: return true;
	case 0x2: result = s64(d.acc) + r; break;
	case 0x3: result = s64(d.acc) + p; break;
	case 0x4: result = r; break;
	case 0x5: result = p; break;
	case 0x6: result = -s64(d.acc); break;
	case 0x7: result = s64(r) - d.acc; break;
	case 0x8: result = s64(p) - d.acc; break;
	case 0x9: result = s64(r) + p; break;
	case 0xa: result = std::min(d.acc, r); break;
	case 0xb: result = std::max(d.acc, r); break;

	case 0xc:
		if (mode == 2)
			result = s64(d.acc) + (p >> 13);
		else if (mode == 3)
			result = p >> 13;
		else
			return true;
		break;

	case 0xd:
		switch (mode)
		{
		case 1: result = d.acc & s.logic; break;
		case 2: result = d.acc | s.logic; break;
		case 3: result = d.acc ^ s.logic; break;
		default: return true;
		}
		break;

	case 0xe:
		if (mode >= 2 && ((d.acc < 0) != (s.raw < 0)))
			mode ^= 1;
		result = BIT(mode, 0) ? std::max(d.acc, s.raw) : std::min(d.acc, s.raw);
		break;

	case 0xf:
		switch (mode)
		{
		case 0: result = s64(d.acc) + s.raw; break;
		case 1: result = s64(r) + s.raw; break;
		case 2: result = s64(p) + s.raw; break;
		default: result = s64(s.raw) - d.acc; break;
		}
		break;

	default:
		return false;
	}

	d.acc = wrap29(result);
	return true;
}

void roland_xp_device::parallel_op(const dsp_slot &s)
{
	dsp_state &d = m_dsp;
	const u16 c = s.cram;
	const int function = c & 0xf;
	const int input_select = (c >> 4) & 3;
	const int factor_select = (c >> 6) & 3;
	const bool complement = BIT(c, 8);
	const bool multiply_issued = BIT(c, 9);
	const int post = (c >> 11) & 7;
	const int shift = shift_select[c >> 14];
	const s32 p = d.product;
	const s32 r = d.r;
	s32 input, product;
	bool payback = false;

	if (!multiply_issued)
	{
		input = d.now_valid ? d.now : 0;
		product = input;
	}
	else
	{
		switch (input_select)
		{
		case 0: input = d.input; break;
		case 1: input = clamp24(d.acc); break;
		case 2: input = r; break;
		default: input = d.latch; break;
		}
		product = multiply_q15(input, factor(factor_select, complement), shift, payback);
	}

	s64 result = d.acc;
	switch (function)
	{
	case 0x0: result = s64(d.acc) + p + r; break;
	case 0xc: result = s64(r) + p - d.acc; break;
	case 0xd: result = s64(d.acc) + p - r - d.payback; break;
	case 0xe: result = s64(p) - r - d.acc - 2 * d.payback; break;
	case 0xf: result = s64(p) - r - 2 * d.payback; break;
	default:
		alu(function, input_select, s);
		result = d.acc;
		break;
	}
	d.acc = wrap29(result);

	switch (post)
	{
	case 1:
		d.acc = std::abs(d.acc);
		break;

	case 2:
		d.acc = wrap24(d.acc);
		break;

	case 3:
	{
		const u32 v = u32(d.acc) & 0xffffff;
		d.acc = wrap24(s32((v << 1) | (((v >> 23) ^ (v >> 6) ^ (v >> 1)) & 1)));
		break;
	}

	case 4:
		d.acc = fold24(d.acc);
		break;

	case 5:
		d.acc = wrap24(d.acc);
		if (d.acc < 0)
			d.acc = ~d.acc;
		break;

	default:
		break;
	}

	if (BIT(c, 10))
		d.acc = wrap24(d.acc);

	d.input = input;
	d.product = product;
	d.payback = payback;
}

void roland_xp_device::execute(const dsp_slot &s)
{
	dsp_state &d = m_dsp;

	if (s.special(SPECIAL_PARALLEL))
	{
		parallel_op(s);
		return;
	}

	const bool multiply_issued = s.function >= 1 && s.function <= 0xc;

	s32 input = 0, product = 0;
	bool payback = false;
	if (multiply_issued)
	{
		if (s.function == 0xc && s.input == INPUT_PREVIOUS)
			input = m_port_a_in;
		else if (s.function == 0xc && s.input == INPUT_ACC)
			input = m_port_b_in;
		else
			input = operand(s);
		product = multiply(input, s.coefficient, payback);
	}

	alu(s.function, s.input, s);

	if (s.special(SPECIAL_BRANCH))
	{
		const int condition = (s.cram >> 10) & 0xf;
		bool taken;
		switch (condition)
		{
		case 0: taken = d.acc == 0; break;
		case 1: taken = d.acc != 0; break;
		case 3: case 5: case 13: case 14: taken = true; break;
		case 6: case 8: taken = d.acc >= 0; break;
		case 7: case 9: taken = d.acc < 0; break;
		case 10: taken = d.acc > 0; break;
		case 11: taken = d.acc <= 0; break;
		default: taken = false; break;
		}
		if (taken)
			m_pc = (BIT(s.cram, 9) ? m_pc + 1 + s8(s.cram) : (s.cram & 0xff)) - 1;
	}

	if (multiply_issued)
	{
		d.input = input;
		d.product = product;
		d.payback = payback;
	}
}

// a strobe presents the previous frame's word at the frame's word position, and the ports
// take their inputs after the strobe's instruction has run
void roland_xp_device::strobe(const dsp_slot &s)
{
	if (s.ext != 1 && s.ext != 2)
		return;

	const s32 word = clamp24(m_iram[cell(m_position, m_parity ^ 1)]);
	m_position++;

	if (s.ext == 1)
	{
		m_port_a_out[m_strobe_a] = word;
		m_port_a_in = (m_port_in_enabled && !m_port_a_in_cb.isunset()) ? wrap24(s32(m_port_a_in_cb(m_strobe_a))) : 0;
		m_strobe_a++;
		return;
	}

	const int port = m_strobe_bcd % 3;
	const int half = (m_strobe_bcd / 3) & 1;
	m_strobe_bcd++;

	const int descriptor = port == PORT_B ? (m_regs[SERIAL_CONFIG >> 1] >> 8) : (m_regs[DSP_CONFIG >> 1] & 0xff);
	const s32 out = (descriptor & 0xc0) ? wire_word(word) : 0;
	m_port_word[port][half] = out;
	if ((descriptor & 0xc0) && (m_regs[DSP_MODE >> 1] & 3) == 3 && !m_port_out_cb.isunset())
		m_port_out_cb(port * 2 + half, u32(out));

	if (port == PORT_B)
		m_port_b_in = (m_port_in_enabled && !m_port_b_in_cb.isunset()) ? wrap24(s32(m_port_b_in_cb(half))) : 0;
}

u32 roland_xp_device::port_a_out_r(int strobe)
{
	return (strobe < DSP_SLOTS) ? u32(m_port_a_out[strobe]) : 0;
}

void roland_xp_device::dsp_frame_start()
{
	m_bus_written = 0;
	m_irq_frame_used = false;
	m_position = 0;
	m_strobe_a = 0;
	m_strobe_bcd = 0;
	m_pc = 0;

	m_dsp_enabled = BIT(m_regs[DSP_MODE >> 1], 2);
	m_port_in_enabled = BIT(m_regs[DSP_MODE >> 1], 1);
	if (!m_dsp_enabled)
	{
		for (auto &port : m_port_word)
			port[0] = port[1] = 0;
		return;
	}

	if (m_program_dirty)
		decode_program();

	update_iram_ramps();
}

void roland_xp_device::dsp_step()
{
	if (!m_dsp_enabled)
		return;

	if (m_pc < 0 || m_pc >= DSP_SLOTS)
		return;

	const dsp_slot &s = m_program[m_pc];
	dsp_state &d = m_dsp;

	if (BIT(m_landing_valid, 0))
		d.latch = m_landing[0];
	m_landing[0] = m_landing[1];
	m_landing_valid >>= 1;

	if (s.eram_op == 1)
	{
		m_landing[1] = m_eram[(s.eram_offset + d.cursor) & 0xffff];
		m_landing_valid |= 2;
	}
	else if (s.special(SPECIAL_INDEXED_READ) && !s.eram_second)
	{
		m_landing[1] = m_eram[(d.cursor + (d.acc >> 12)) & 0xffff];
		m_landing_valid |= 2;
	}

	d.now_valid = 0;
	s32 gain_pending = 0;
	bool gain_arrives = false;
	if (s.st == 1)
	{
		const s32 v = m_iram[cell(s.word)];
		if (s.word >= ramp_base())
		{
			gain_pending = gain_current(v);
			gain_arrives = true;
		}
		else
		{
			d.now = v;
			d.now_valid = 1;
		}
	}

	const s32 acc_before = d.acc;
	const s32 r_before = d.r;
	execute(s);

	if (gain_arrives)
		d.gain = gain_pending;
	if (d.now_valid)
		d.r = d.now;

	if (s.st == 2)
		m_iram[cell(s.word)] = d.latch;
	else if (s.st == 3)
		m_iram[cell(s.word)] = clamp24(acc_before);

	if (s.eram_op == 3)
		m_eram[(s.eram_offset + d.cursor) & 0xffff] = clamp24(acc_before);
	else if (s.eram_op == 2)
		m_eram[(s.eram_offset + d.cursor) & 0xffff] = clamp24(r_before);

	strobe(s);

	m_pc++;
}

void roland_xp_device::frame_end()
{
	m_parity ^= 1;
	if (m_dsp_enabled)
		m_dsp.cursor--;
	m_frame_counter++;
	m_cycle = 0;
}

// one cycle of the frame: voice n's four cycles carry its deposits and its own run
void roland_xp_device::cycle()
{
	if (!m_cycle)
		dsp_frame_start();

	const int n = m_cycle >> 2;
	if (n < voice_count())
	{
		switch (m_cycle & 3)
		{
		case 0:
			deposit(n, 0);
			break;
		case 2:
			deposit(n, 1);
			deposit(n, 2);
			break;
		case 3:
			deposit(n, 3);
			run_voice(n);
			if (BIT(n, 0) && pairs(n - 1))
				run_pair(n - 1);
			break;
		default:
			break;
		}
	}

	if (m_pc >= 0 && m_pc < DSP_SLOTS)
		debugger_instruction_hook(m_pc);
	dsp_step();

	if (++m_cycle >= slot_count())
		frame_end();
}

void roland_xp_device::execute_run()
{
	if (m_pumped)
	{
		m_icount = 0;
		return;
	}

	while (m_icount > 0)
	{
		cycle();
		m_icount--;

		if (!m_cycle)
			m_frame_cb(1);
	}
}

void roland_xp_device::run_frame()
{
	do
		cycle();
	while (m_cycle);
}

void roland_xp_device::sound_stream_update(sound_stream &stream)
{
	for (int port = 0; port < OUTPUT_PORTS; port++)
		for (int half = 0; half < 2; half++)
			stream.put_int(port * 2 + half, 0, m_port_word[port][half], 1 << 23);
}
