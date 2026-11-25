// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Roland XV tone generator (TC223C660CF-503, RA08-503)

    - XV-5080 (two chips, linked), SD-80, SD-90, Fantom
    - 64 voices; 8-bit DPCM or 16-bit linear samples at 28-bit addresses
    - per voice: four-point interpolation, state-variable filter, Q15
      amplitude, six sends; ramped cutoff, resonance, amplitude, pitch
      and sends
    - paired voices: sum, booster and ring structures
    - effect DSP: 768 rows a sample, 512-cell ring, 2^19-cell external RAM,
      64-word parameter bank
    - FIFO transfer engine onto the wave space
    - panel scanner (64 switches, LEDs) and character LCD port
    - five serial stereo outputs and a link to a second chip

    TODO:
    - which of the filter's integrators moves first; types 0-6 and the
      saturation with moving integrators
    - format 0, loop mode 3, word 0x60 bits 9:8 and 6, words 0x72-0x7f,
      the DPCM history at a relaunch, the cache's twentieth bit, the seam
      probe's place in the sample
    - predicate mode 5's other role; ERAM arbitration between a tap, a
      fixed read and a fixed write; taps near the end of a sample
    - which work bank the host reaches
    - the I/O sequencer's other control words and the pins' timing
    - the bank widths of configuration words 0x12-0x19
    - paired structures 4-6
    - whether a one-shot's end raises reason 0; reason 8

***************************************************************************/

#include "emu.h"
#include "roland_xv.h"

#include <algorithm>
#include <bit>
#include <cstdlib>
#include <memory>


#define LOG_REGS    (1U << 1)
#define LOG_XFER    (1U << 2)
#define LOG_SPACE   (1U << 3)
#define LOG_OBJECT  (1U << 4)
#define LOG_IRQ     (1U << 5)
#define LOG_DSP     (1U << 6)

#define VERBOSE (0)
#include "logmacro.h"

namespace {

constexpr s16 INTERP_WEIGHTS[3][128] = {
	{
		3385, 3401, 3417, 3432, 3448, 3463, 3478, 3492, 3506, 3521, 3535, 3548, 3562, 3575, 3588, 3601,
		3614, 3626, 3638, 3650, 3662, 3673, 3685, 3696, 3707, 3718, 3728, 3739, 3749, 3759, 3768, 3778,
		3787, 3796, 3805, 3814, 3823, 3831, 3839, 3847, 3855, 3863, 3870, 3878, 3885, 3892, 3899, 3905,
		3912, 3918, 3924, 3930, 3936, 3942, 3948, 3953, 3958, 3963, 3968, 3973, 3978, 3983, 3987, 3991,
		3995, 4000, 4004, 4007, 4011, 4015, 4018, 4022, 4025, 4028, 4031, 4034, 4037, 4040, 4042, 4045,
		4047, 4050, 4052, 4054, 4057, 4059, 4061, 4063, 4064, 4066, 4068, 4070, 4071, 4073, 4074, 4076,
		4077, 4078, 4079, 4081, 4082, 4083, 4084, 4085, 4086, 4086, 4087, 4088, 4089, 4089, 4090, 4091,
		4091, 4092, 4092, 4093, 4093, 4094, 4094, 4094, 4094, 4095, 4095, 4095, 4095, 4095, 4095, 4095,
	},
	{
		 710,  726,  742,  758,  775,  792,  809,  826,  844,  861,  879,  897,  915,  933,  952,  971,
		 990, 1009, 1028, 1047, 1067, 1087, 1106, 1126, 1147, 1167, 1188, 1208, 1229, 1250, 1271, 1292,
		1314, 1335, 1357, 1379, 1400, 1423, 1445, 1467, 1489, 1512, 1534, 1557, 1580, 1602, 1625, 1648,
		1671, 1695, 1718, 1741, 1764, 1788, 1811, 1835, 1858, 1882, 1906, 1929, 1953, 1977, 2000, 2024,
		2048, 2071, 2095, 2119, 2143, 2166, 2190, 2214, 2237, 2261, 2284, 2308, 2331, 2355, 2378, 2401,
		2425, 2448, 2471, 2494, 2517, 2539, 2562, 2585, 2607, 2630, 2652, 2674, 2696, 2718, 2740, 2762,
		2783, 2805, 2826, 2847, 2868, 2889, 2910, 2931, 2951, 2971, 2991, 3011, 3031, 3051, 3070, 3089,
		3108, 3127, 3146, 3164, 3182, 3200, 3218, 3236, 3253, 3271, 3288, 3304, 3321, 3338, 3354, 3370,
	},
	{
		   0,    0,    0,    1,    1,    1,    2,    2,    3,    3,    3,    4,    4,    5,    5,    6,
		   6,    7,    8,    8,    9,   10,   10,   11,   12,   13,   14,   15,   16,   17,   18,   19,
		  20,   22,   23,   24,   26,   27,   29,   30,   32,   34,   36,   38,   40,   42,   44,   46,
		  49,   51,   53,   56,   59,   62,   65,   68,   71,   74,   77,   81,   84,   88,   92,   96,
		 100,  104,  109,  113,  118,  122,  127,  132,  137,  143,  148,  154,  160,  165,  171,  178,
		 184,  191,  197,  204,  211,  219,  226,  234,  241,  249,  257,  266,  274,  283,  292,  301,
		 310,  319,  329,  339,  349,  359,  369,  380,  391,  402,  413,  424,  436,  448,  460,  472,
		 484,  497,  510,  523,  536,  549,  563,  577,  591,  605,  619,  634,  648,  663,  679,  694,
	},
};

constexpr s32 RAMP_RATE[roland_xv_device::RATE_CODES] = {
	368, 184, 124, 92, 74, 62, 54, 46, 41, 37, 31, 27, 23, 21, 19, 15 };

constexpr s32 RAMP_ACCELERATION[roland_xv_device::RATE_CODES] = {
	896, 768, 640, 512, 448, 384, 320, 256, 224, 192, 160, 128, 112, 96, 80, 64 };

constexpr s32 ODD_GAIN[8] = { 0x5a40, 0x5a60, 0x5a60, 0x5a60, 0x5a64, 0x5a66, 0x5a67, 0x5a67 };

} // anonymous namespace

DEFINE_DEVICE_TYPE(ROLAND_XV, roland_xv_device, "roland_xv", "Roland XV")

roland_xv_device::roland_xv_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, ROLAND_XV, tag, owner, clock)
	, device_memory_interface(mconfig, *this)
	, device_sound_interface(mconfig, *this)
	, m_link(*this, finder_base::DUMMY_TAG)
	, m_wave_config("wave", ENDIANNESS_LITTLE, 16, 28, -1)
	, m_int_callback(*this)
	, m_switch_callback(*this, 0)
	, m_led_callback(*this)
	, m_lcd_callback(*this)
	, m_stream(nullptr)
	, m_scan_timer(nullptr)
	, m_stream_timer(nullptr)
	, m_master(nullptr)
{
}

roland_xv_device::dsp_recompiler::~dsp_recompiler() = default;

device_memory_interface::space_config_vector roland_xv_device::memory_space_config() const
{
	return space_config_vector { std::make_pair(AS_WAVE, &m_wave_config) };
}

void *roland_xv_device::alloc_near(size_t bytes, size_t align)
{
	if (m_recompiler)
		return m_recompiler->alloc_near(bytes, align);
	m_heap.emplace_back(std::make_unique<u8[]>(bytes + align));
	return reinterpret_cast<void *>((uintptr_t(m_heap.back().get()) + align - 1) & ~uintptr_t(align - 1));
}

void roland_xv_device::device_start()
{
	space(AS_WAVE).specific(m_wave);
	m_stream = stream_alloc(0, OUTPUTS, sample_rate());
	m_space = std::make_unique<u32[]>(0x1'0000);
	m_eram = std::make_unique<s32[]>(ERAM_CELLS);
	if (m_link)
		m_link->m_master = this;

	m_recompiler = dsp_recompiler::create(*this);
	m_dsp = static_cast<dsp_state *>(alloc_near(sizeof(dsp_state), alignof(dsp_state)));
	m_iram = static_cast<u32 *>(alloc_near(sizeof(u32) * IBUS_IRAM_END, alignof(u32)));
	m_work = static_cast<s32 *>(alloc_near(sizeof(s32) * 2 * WORK_CELLS, alignof(s32)));
	m_bank = static_cast<s32 *>(alloc_near(sizeof(s32) * (IBUS_BANK_END - IBUS_BANK), alignof(s32)));
	m_bank_next = static_cast<s32 *>(alloc_near(sizeof(s32) * (IBUS_BANK_END - IBUS_BANK), alignof(s32)));
	m_tap_cell = static_cast<u16 *>(alloc_near(sizeof(u16) * TAPS, alignof(u16)));
	m_tap_delay = static_cast<s32 *>(alloc_near(sizeof(s32) * TAPS, alignof(s32)));
	m_tap_slot = static_cast<u16 *>(alloc_near(sizeof(u16) * TAPS, alignof(u16)));
	m_late_cell = static_cast<u16 *>(alloc_near(sizeof(u16) * 2, alignof(u16)));
	m_bus = m_work;
	m_bus_other = m_work + WORK_CELLS;

	save_item(NAME(m_regs));
	save_item(NAME(m_object_regs));
	save_item(NAME(m_cache));
	save_item(NAME(m_high_latch));
	save_item(NAME(m_bus_latch));
	save_item(NAME(m_read_latch));
	save_item(NAME(m_run_pending));
	save_item(NAME(m_row_stage));
	save_pointer(NAME(m_space), 0x1'0000);
	save_pointer(NAME(m_iram), IBUS_IRAM_END);
	save_item(NAME(m_address));
	save_item(NAME(m_fifo));
	save_item(NAME(m_fifo_write));
	save_item(NAME(m_fifo_read));
	save_item(NAME(m_irq_enable));
	save_item(NAME(m_irq_pending));
	save_item(NAME(m_irq_voice));
	save_item(NAME(m_irq_waiting));
	save_item(NAME(m_scan_select));
	save_item(NAME(m_scan_read));
	save_item(NAME(m_scan_write));
	save_item(NAME(m_led_select));
	save_item(NAME(m_led));
	save_item(NAME(m_scan));
	save_item(NAME(m_switch_state));
	save_item(NAME(m_switch_changed));
	save_item(NAME(m_switch_index));
	m_scan_timer = timer_alloc(FUNC(roland_xv_device::scan_switches), this);
	m_stream_timer = timer_alloc(FUNC(roland_xv_device::stream_tick), this);
	save_item(NAME(m_int_state));
	save_item(NAME(m_run_mask));
	save_item(STRUCT_MEMBER(m_voices, backward));
	save_item(STRUCT_MEMBER(m_voices, fetching));
	save_item(STRUCT_MEMBER(m_voices, lap));
	save_item(STRUCT_MEMBER(m_voices, region));
	save_item(STRUCT_MEMBER(m_voices, scaled));
	save_item(STRUCT_MEMBER(m_voices, end_cell));
	save_item(STRUCT_MEMBER(m_voices, filter_low));
	save_item(STRUCT_MEMBER(m_voices, filter_band));
	save_item(STRUCT_MEMBER(m_voices, pair_smooth));
	save_item(STRUCT_MEMBER(m_voices, pair_ceiling));
	save_item(STRUCT_MEMBER(m_rows, w0));
	save_item(STRUCT_MEMBER(m_rows, w1));
	save_item(STRUCT_MEMBER(m_rows, operand));
	save_pointer(NAME(m_eram), ERAM_CELLS);
	save_pointer(NAME(m_work), 2 * WORK_CELLS);
	save_pointer(NAME(m_bank), IBUS_BANK_END - IBUS_BANK);
	save_pointer(NAME(m_bank_next), IBUS_BANK_END - IBUS_BANK);
	save_item(NAME(m_bank_live));
	save_item(NAME(m_ramp_phase));
	save_item(NAME(m_dsp->cursor));
	save_item(NAME(m_dsp->work_phase));
	save_item(NAME(m_dsp->acc));
	save_item(NAME(m_dsp->product));
	save_item(NAME(m_dsp->product_shift));
	save_item(NAME(m_dsp->latch));
	save_item(NAME(m_dsp->flag));
	save_item(NAME(m_dsp->flag_carry));
	save_item(NAME(m_dsp->last));
	save_item(NAME(m_dsp->last_carry));
	save_item(NAME(m_dsp->last_hold));
	save_pointer(NAME(m_tap_cell), TAPS);
	save_pointer(NAME(m_tap_delay), TAPS);
	save_pointer(NAME(m_tap_slot), TAPS);
	save_item(NAME(m_dsp->tap_count));
	save_pointer(NAME(m_late_cell), 2);
	save_item(NAME(m_dsp->late_count));
	save_item(NAME(m_out));

	m_rows_end = -1;
	std::fill_n(m_eram.get(), ERAM_CELLS, 0);
	clear_memories();
}

void roland_xv_device::device_post_load()
{
	m_rows_end = -1;
	for (int n = 0; n < DSP_ROWS; n++)
		decode_row(n);
	m_transfers_stale = true;
	m_io_stale = true;
	m_bus = m_work + (m_dsp->work_phase ? WORK_CELLS : 0);
	m_bus_other = m_work + (m_dsp->work_phase ? 0 : WORK_CELLS);
	if (m_recompiler)
		m_recompiler->reset();
}

void roland_xv_device::device_reset()
{
	std::fill(std::begin(m_regs), std::end(m_regs), 0);
	m_regs[0x5b] = 0x000f;
	std::fill_n(&m_regs[0x5c], 4, 0x8000);
	m_address = 0;
	m_high_latch = 0;
	m_bus_latch = 0;
	m_read_latch = 0;
	m_run_pending = true;
	std::fill(std::begin(m_fifo), std::end(m_fifo), 0);
	fifo_rewind();
	m_irq_enable = 0;
	m_irq_pending = 0;
	std::fill(std::begin(m_irq_voice), std::end(m_irq_voice), 0);
	std::fill(std::begin(m_irq_waiting), std::end(m_irq_waiting), 0);
	m_int_state = false;
	m_int_callback(0);
	m_scan_select = -1;
	m_scan_read = 0;
	m_scan_write = 0;
	m_led_select = -1;
	std::fill(std::begin(m_scan), std::end(m_scan), 0x1111);
	std::fill(std::begin(m_led), std::end(m_led), 0);
	m_switch_state = 0;
	m_switch_changed = 0;
	m_switch_index = 0;
	m_scan_timer->adjust(attotime::from_msec(4), 0, attotime::from_msec(4));
	const attotime sample = attotime::from_hz(sample_rate());
	m_stream_timer->adjust(m_master ? attotime::never : sample, 0, sample);
	m_run_mask = 0;
	m_ramp_phase = 0;
	m_threshold = 0;
	std::fill_n(m_tap_cell, TAPS, 0);
	std::fill_n(m_tap_delay, TAPS, 0);
	std::fill_n(m_tap_slot, TAPS, 0);
	std::fill_n(m_late_cell, 2, 0);
	*m_dsp = dsp_state();
	m_dsp->last_hold = 1;
	m_bus = m_work;
	m_bus_other = m_work + WORK_CELLS;
	m_transfer_count = 0;
	m_transfers_stale = true;
	m_io_transmit_count = 0;
	m_io_receive_count = 0;
	std::fill(std::begin(m_io_out), std::end(m_io_out), -1);
	m_io_stale = true;
	std::fill(std::begin(m_out), std::end(m_out), 0);
	if (m_recompiler)
		m_recompiler->reset();
}

void roland_xv_device::clear_memories()
{
	std::fill(&m_object_regs[0][0], &m_object_regs[0][0] + OBJECTS * (OBJECT_END - OBJECT_BASE), 0);
	std::fill(&m_cache[0][0], &m_cache[0][0] + OBJECTS * CACHE_CELLS, 0);
	for (int n = 0; n < OBJECTS; n++)
	{
		m_voices[n] = voice();
		for (int word : { CUTOFF_RAMP, RESONANCE_RAMP, AMPLITUDE_RAMP, SEND_PORT_A, SEND_PORT_B })
			object_ref(n, word) = RAMP_HOLD | RAMP_REUSE;
		object_ref(n, PITCH_RAMP) = PITCH_HOLD | PITCH_REUSE;
	}
	std::fill_n(m_space.get(), 0x1'0000, 0);
	for (int n = 0; n < DSP_ROWS; n++)
	{
		m_rows[n] = dsp_row();
		decode_row(n);
	}
	m_rows_end = -1;
	std::fill(std::begin(m_row_stage), std::end(m_row_stage), 0);
	std::fill_n(m_iram, IBUS_IRAM_END, 0);
	std::fill_n(m_work, 2 * WORK_CELLS, 0);
	hold_parameters();
	m_transfers_stale = true;
	m_io_stale = true;
	if (m_recompiler)
		m_recompiler->reset();
}

void roland_xv_device::device_clock_changed()
{
	const u32 rate = sample_rate();
	m_stream->set_sample_rate(rate);
	const attotime sample = attotime::from_hz(rate);
	m_stream_timer->adjust(m_master ? attotime::never : sample, 0, sample);
	if (m_link)
		m_link->set_unscaled_clock(clock());
}

void roland_xv_device::sound_stream_update(sound_stream &stream)
{
	for (int i = 0; i < stream.samples(); i++)
	{
		if (!m_master)
		{
			if (m_link)
				m_link->frame();
			frame();
			serial_out();
			exchange();
		}
		for (int n = 0; n < OUTPUTS; n++)
			stream.put_int_clamp(n, i, m_out[n], 1 << (CELL_BITS - 1));
	}
}

TIMER_CALLBACK_MEMBER(roland_xv_device::stream_tick)
{
	m_stream->update();
}

void roland_xv_device::sync()
{
	if (m_master)
		m_master->m_stream->update();
	else
		m_stream->update();
}

void roland_xv_device::frame()
{
	s64 mix[BUSES] = { 0 };
	u64 claimed = 0;
	m_threshold = ramp_threshold(m_ramp_phase);
	for (int n = 0; n < OBJECTS; n++)
	{
		const int kind = structure(n);
		if (kind != STRUCTURE_NONE)
		{
			run_pair(n, kind, mix, claimed);
			n++;
		}
		else
			run_voice(n, mix, claimed);
	}
	run_dsp(mix, claimed);
}

//-------------------------------------------------
//  the I/O sequencer
//-------------------------------------------------

void roland_xv_device::decode_io()
{
	m_io_stale = false;
	m_io_transmit_count = 0;
	m_io_receive_count = 0;
	std::fill(std::begin(m_io_out), std::end(m_io_out), -1);
	int transmit = 0, receive = 0, primary = 0;
	int strobes[SERIAL_OUTS] = { 0 };
	for (int n = 0; n < IORAM_WORDS; n++)
	{
		const u16 word = m_space[SPACE_IORAM + n];
		if (word == IO_END)
			break;
		switch (word & IO_FAMILY)
		{
		case IO_SET_TRANSMIT:
			transmit = word & (IBUS_SERIAL_CELLS - 1);
			break;

		case IO_SET_RECEIVE:
			receive = word & (IBUS_SERIAL_CELLS - 1);
			break;

		case 0:
			if (word == IO_TRANSMIT)
			{
				m_io_transmit[m_io_transmit_count++] = IBUS_TRANSPORT + transmit;
				transmit = (transmit + 1) & (IBUS_SERIAL_CELLS - 1);
			}
			else if (word == IO_RECEIVE || word == IO_RECEIVE_AUX)
			{
				m_io_receive[m_io_receive_count++] = io_receive{ u16(IBUS_TRANSPORT + receive), s16(word == IO_RECEIVE ? primary++ : -1) };
				receive = (receive + 1) & (IBUS_SERIAL_CELLS - 1);
			}
			else if (word == IO_SDO3 || (word >= IO_SDO4 && word <= IO_SDO7 && !(word & 0x0f)))
			{
				const int out = word == IO_SDO3 ? OUT_SDO3 : (word - IO_SDO4) >> 4;
				if (strobes[out] < 2)
					m_io_out[2 * out + strobes[out]++] = IBUS_TRANSPORT + transmit;
				transmit = (transmit + 1) & (IBUS_SERIAL_CELLS - 1);
			}
			break;
		}
	}
}

void roland_xv_device::serial_out()
{
	if (m_io_stale)
		decode_io();
	for (int n = 0; n < OUTPUTS; n++)
		m_out[n] = m_io_out[n] < 0 ? 0 : m_bus[m_io_out[n] - IBUS_MIX];
}

void roland_xv_device::serial_in(const s32 *words, int count)
{
	for (int n = 0; n < m_io_receive_count; n++)
	{
		const io_receive &r = m_io_receive[n];
		if (r.index < 0)
			m_bus[r.cell - IBUS_MIX] = -1;
		else if (r.index < count)
			m_bus[r.cell - IBUS_MIX] = words[r.index];
	}
}

void roland_xv_device::exchange()
{
	if (m_io_stale)
		decode_io();
	if (!m_link)
	{
		serial_in(nullptr, 0);
		return;
	}
	if (m_link->m_io_stale)
		m_link->decode_io();
	s32 mine[IORAM_WORDS], theirs[IORAM_WORDS];
	for (int n = 0; n < m_io_transmit_count; n++)
		mine[n] = m_bus[m_io_transmit[n] - IBUS_MIX];
	for (int n = 0; n < m_link->m_io_transmit_count; n++)
		theirs[n] = m_link->m_bus[m_link->m_io_transmit[n] - IBUS_MIX];
	serial_in(theirs, m_link->m_io_transmit_count);
	m_link->serial_in(mine, m_io_transmit_count);
}

//-------------------------------------------------
//  the window
//-------------------------------------------------

u8 roland_xv_device::read(offs_t offset)
{
	const int word = (offset >> 1) & 0xff;
	const bool data_port = word == DATA_HIGH || word == DATA_LOW;
	const bool side_effects = !machine().side_effects_disabled();
	if (data_port && side_effects)
		sync();
	u16 data;
	if (data_port && (m_address & 0xf000) == SPACE_PRAM)
		data = pram_r();
	else if (data_port && sixteen_bit(m_address))
		data = space_r(m_address) & 0xffff;
	else if (word == DATA_HIGH)
	{
		const u32 cell = BIT(offset, 0) ? m_read_latch : space_r(m_address);
		if (!BIT(offset, 0) && side_effects)
			m_read_latch = cell;
		data = cell >> 16;
	}
	else if (answers_latch(word))
		data = m_bus_latch;
	else
		data = word_peek(word);

	if (!BIT(offset, 0))
		return data >> 8;
	if (side_effects)
		word_taken(word, data);
	return data & 0xff;
}

void roland_xv_device::write(offs_t offset, u8 data)
{
	const int word = (offset >> 1) & 0xff;
	if (word >= OBJECT_BASE || (word >= RUN_MASK && word <= RUN_COMMIT) || word == DATA_HIGH || word == DATA_LOW || (word == MODE && BIT(data, 6)))
		sync();
	if (!BIT(offset, 0))
		m_regs[word] = (m_regs[word] & 0x00ff) | (data << 8);
	else
	{
		const u16 value = (m_regs[word] & 0xff00) | data;
		if (!BIT(word, 0))
			m_high_latch = value;
		m_bus_latch = value;
		word_w(word, value);
	}
}

bool roland_xv_device::write_only(int word)
{
	return word == 0x01 || word == 0x03 || word == 0x07 || (word >= 0x26 && word <= 0x5a);
}

bool roland_xv_device::answers_latch(int word)
{
	return word == DATA_LOW || write_only(word) || (BIT(word, 0) && ((word >= PHASE && word < 0xc0) || word >= 0xe0));
}

u16 roland_xv_device::word_peek(int word)
{
	switch (word)
	{
	case IDENTITY:
		return 0x0022;

	case FIFO:
		if (m_scan_select >= 0)
			return m_scan[(m_scan_select + m_scan_read) & 15];
		if (m_led_select >= 0)
			return m_led[(m_led_select + m_scan_read) & 15];
		return m_fifo[m_fifo_read];

	case IRQ_MASK:
		return m_irq_pending & m_irq_enable;

	case 0x24:
		return 0xffff;

	case SWITCH_INDEX:
		return m_switch_index;

	case STATUS:
		return 0;

	case RUN_COMMIT:
		return (m_run_pending ? 0x80 : 0) | (std::popcount(m_regs[RUN_MASK]) + std::popcount(m_regs[RUN_MASK + 1])
				+ std::popcount(m_regs[RUN_MASK + 2]) + std::popcount(m_regs[RUN_MASK + 3]));

	default:
		if (word >= IRQ_VOICE && word < IRQ_VOICE + IRQ_REASONS)
			return m_irq_voice[word - IRQ_VOICE];
		if (word == PITCH_RAMP + 2 || word == PITCH_RAMP + 3)
			word -= 2;
		if (word >= OBJECT_BASE && word < OBJECT_END)
			return m_object_regs[object()][word - OBJECT_BASE];
		return m_regs[word];
	}
}

void roland_xv_device::word_taken(int word, u16 data)
{
	const bool data_port = word == DATA_HIGH || word == DATA_LOW;
	if (data_port && (m_address & 0xf000) != SPACE_PRAM && !sixteen_bit(m_address))
	{
		if (word == DATA_HIGH)
			m_bus_latch = m_read_latch & 0xffff;
	}
	else if (!answers_latch(word))
		m_bus_latch = (!BIT(word, 0) && word >= PHASE && answers_latch(word | 1)) ? word_peek((word | 1) & 0xff) : data;
	switch (word)
	{
	case DATA_HIGH:
	case DATA_LOW:
		LOGMASKED(LOG_SPACE, "%s: read %04x = %08x\n", machine().describe_context(), m_address, space_r(m_address));
		if ((m_address & 0xf000) == SPACE_PRAM)
			pram_step(false);
		else if (word == DATA_LOW || sixteen_bit(m_address))
			m_address++;
		break;

	case FIFO:
		if (m_scan_select >= 0 || m_led_select >= 0)
			m_scan_read++;
		else
			m_fifo_read = (m_fifo_read + 1) % FIFO_DEPTH;
		break;
	}
}

void roland_xv_device::word_w(int word, u16 data)
{
	m_regs[word] = data;
	switch (word)
	{
	case MODE:
		LOGMASKED(LOG_REGS, "%s: mode %04x\n", machine().describe_context(), data);
		if (data & MODE_CLEAR)
			clear_memories();
		break;

	case DATA_HIGH:
	case DATA_LOW:
		if ((m_address & 0xf000) == SPACE_PRAM)
			pram_w(data);
		else if (word == DATA_LOW || sixteen_bit(m_address))
		{
			space_w(m_address, word == DATA_LOW ? (u32(m_high_latch) << 16) | data : data);
			LOGMASKED(LOG_SPACE, "%s: write %04x = %08x\n", machine().describe_context(), m_address, space_r(m_address));
			m_address++;
		}
		break;

	case ADDRESS:
		m_address = data;
		break;

	case FIFO:
		if (m_scan_select >= 0)
		{
			const int w = (m_scan_select + m_scan_write) & 15;
			m_scan[w] = (m_scan[w] & 0x1111) | (data & 0xeeee);
			m_scan_write++;
		}
		else if (m_led_select >= 0)
		{
			const int w = (m_led_select + m_scan_write) & 15;
			m_scan_write++;
			if (m_led[w] != data)
			{
				const u16 changed = m_led[w] ^ data;
				m_led[w] = data;
				for (int k = 0; k < 4; k++)
				{
					if ((changed >> (k * 4)) & 15)
						m_led_callback((w >> 1) * 8 + (w & 1) * 4 + k, (data >> (k * 4)) & 15);
				}
			}
		}
		else
			fifo_push(data);
		break;

	case FIFO_CONTROL:
		m_scan_select = -1;
		m_led_select = -1;
		m_scan_read = 0;
		m_scan_write = 0;
		if ((data & 0xfff0) == 0x0240)
			m_scan_select = data & 15;
		else if ((data & 0xfff0) == 0x0260)
			m_led_select = data & 15;
		else if ((data & 0xffc0) == 0x0200)
			m_fifo_write = m_fifo_read = data & 0x3f;
		else if (data < 0x0200)
			m_fifo_write = m_fifo_read = data;
		else
			fifo_rewind();
		break;

	case COMMAND_STROBE:
		if (!BIT(data, 15))
			break;
		LOGMASKED(LOG_XFER, "%s: command %04x (strobe %04x)\n", machine().describe_context(), m_fifo[0], data);
		if ((m_regs[MODE] & 0xffc0) == 0)
		{
			for (int i = 0, count = std::max(1, (data >> 8) & 0x3f); i < count; i++)
				m_lcd_callback(BIT(m_fifo[i], 8), m_fifo[i] & 0xff);
		}
		fifo_rewind();
		break;

	case XFER_COMMAND:
		if (BIT(data, 0))
			transfer_write();
		break;

	case READ_GO:
		if (BIT(data, 0))
			transfer_read();
		break;

	case RUN_MASK: case RUN_MASK + 1: case RUN_MASK + 2: case RUN_MASK + 3:
		run_mask_w(word, data);
		break;

	case RUN_COMMIT:
		run_mask_w(word, data);
		break;

	case IRQ_MASK:
		LOGMASKED(LOG_IRQ, "%s: interrupt mask %04x\n", machine().describe_context(), data);
		m_irq_enable = data;
		update_irq();
		break;

	case IRQ_ACK:
		m_irq_pending &= ~data;
		for (int reason = 0; reason < IRQ_REASONS; reason++)
		{
			if (BIT(data, reason) && m_irq_waiting[reason])
			{
				const int voice = std::countr_zero(m_irq_waiting[reason]);
				m_irq_waiting[reason] &= ~(u64(1) << voice);
				m_irq_voice[reason] = voice;
				m_irq_pending |= 1 << reason;
			}
		}
		present_switch();
		update_irq();
		break;

	default:
		if (word >= OBJECT_BASE)
			object_w(object(), word, data);
		else
			LOGMASKED(LOG_REGS, "%s: register %02x = %04x\n", machine().describe_context(), word, data);
		break;
	}
}


void roland_xv_device::object_w(int voice, int word, u16 data)
{
	if ((word >= PHASE && word < CUTOFF) || (word >= 0xe0 && word < 0xfc))
	{
		if (!BIT(word, 0) || (BIT(m_regs[MODE], 6) && (word == BLOCK_CONTROL + 1 || word == PARAMETER_SLOPE + 1)))
			return;
		const u16 width = word < START ? 0x3f : word < CUTOFF_RAMP ? 0xfff : word < CUTOFF_SLOPE ? 0x3ff
				: word < FILTER_BAND ? 0xf : word < SEND_BASE ? 0xff : 0x3f;
		m_object_regs[voice][word - 1 - OBJECT_BASE] = m_high_latch & width;
	}
	m_object_regs[voice][word - OBJECT_BASE] = data;
	switch (word)
	{
	case START + 1:
		launch(voice);
		break;

	case FILTER_BAND + 1:
		m_voices[voice].filter_band = util::sext(object_long(voice, FILTER_BAND), 24);
		break;

	case FILTER_LOW + 1:
		m_voices[voice].filter_low = util::sext(object_long(voice, FILTER_LOW), 24);
		break;

	case PAIR_SMOOTH + 1:
		m_voices[voice].pair_smooth = util::sext(object_long(voice, PAIR_SMOOTH), 24);
		break;

	case FILTER_TYPE:
		switch (data >> 12)
		{
		case 0x9: case 0xa: case 0xb: break;
		case 0xc: m_voices[voice].pair_ceiling = 1 << 14; break;
		case 0xd: m_voices[voice].pair_ceiling = 1 << 15; break;
		case 0xe: m_voices[voice].pair_ceiling = 1 << 16; break;
		case 0xf: m_voices[voice].pair_ceiling = 1 << 23; break;
		default: m_voices[voice].pair_ceiling = 1 << 19; break;
		}
		break;

	case AMPLITUDE_RAMP + 1:
		LOGMASKED(LOG_OBJECT, "%s: object %03x amplitude %08x\n", machine().describe_context(), m_regs[MODE], object_long(voice, AMPLITUDE_RAMP));
		return;

	case BLOCK_CONTROL + 1:
		m_bank_live |= u64(1) << voice;
		LOGMASKED(LOG_OBJECT, "%s: parameter %02x = %08x\n", machine().describe_context(), voice, object_long(voice, BLOCK_CONTROL));
		return;
	}
	LOGMASKED(LOG_OBJECT, "%s: object %03x word %02x = %04x\n", machine().describe_context(), m_regs[MODE], word, data);
}


//-------------------------------------------------
//  the run mask
//-------------------------------------------------

void roland_xv_device::run_mask_w(int word, u16 data)
{
	if (word == RUN_COMMIT)
	{
		if (!data)
		{
			m_run_pending = false;
			m_run_mask = 0;
			for (int w = 0; w < 4; w++)
				m_run_mask |= u64(m_regs[RUN_MASK + 3 - w]) << (16 * w);
		}
		LOGMASKED(LOG_REGS, "%s: run mask sync %04x\n", machine().describe_context(), data);
		return;
	}
	m_run_pending = true;
	m_run_mask &= ~(u64(u16(~data)) << (16 * (RUN_MASK + 3 - word)));
	LOGMASKED(LOG_REGS, "%s: run mask word %02x = %04x\n", machine().describe_context(), word, data);
}


//-------------------------------------------------
//  the spaces behind word 0x06
//-------------------------------------------------

bool roland_xv_device::sixteen_bit(u16 address)
{
	return (address >= IBUS_BANK && address < IBUS_BANK_END) || (address & 0xf000) == SPACE_IORAM;
}

void roland_xv_device::pram_step(bool write)
{
	const u16 slot = m_address & 3;
	if (slot == 3)
		return;
	if (slot < 2)
		m_address++;
	else if (!(m_regs[MODE] & (write ? MODE_WRITE_STEP : MODE_READ_STEP)))
		m_address &= ~3;
	else
		m_address = SPACE_PRAM | ((m_address + ((m_regs[MODE] & MODE_STEP_DOWN) ? -6 : 2)) & 0xffc);
}

u16 roland_xv_device::pram_r()
{
	const int n = (m_address >> 2) & (DSP_ROWS - 1);
	const u16 slot = m_address & 3;
	if (slot == 3)
		return m_bus_latch;
	if (slot < 2 && n < DSP_ROWS_MAPPED)
		return slot ? m_rows[n].w1 : m_rows[n].w0;
	return m_rows[std::min(n, DSP_ROWS_MAPPED - 1)].operand;
}

void roland_xv_device::pram_w(u16 data)
{
	const int n = (m_address >> 2) & (DSP_ROWS - 1);
	const u16 slot = m_address & 3;
	LOGMASKED(LOG_SPACE, "%s: row %03x word %d = %04x\n", machine().describe_context(), n, slot, data);
	if (slot < 2)
		m_row_stage[slot] = data;
	else if (n < DSP_ROWS_MAPPED)
	{
		if (slot == 2)
		{
			m_rows[n].w0 = m_row_stage[0];
			m_rows[n].w1 = m_row_stage[1];
		}
		m_rows[n].operand = data;
		row_changed(n);
	}
	pram_step(true);
}

u32 roland_xv_device::space_r(u16 address) const
{
	if (address < IBUS_IRAM_END)
		return m_iram[address] & 0xff'ffff;
	if (address < IBUS_BANK)
		return u32(m_bus[address - IBUS_MIX]) & 0xff'ffff;
	if (address < IBUS_BANK_END)
	{
		const u16 word = u16(m_bank_next[address - IBUS_BANK] >> 8);
		return (u32(word) << 16) | word;
	}
	if ((address & 0xf000) == SPACE_PRAM)
	{
		const int n = std::min<int>((address >> 2) & (DSP_ROWS - 1), DSP_ROWS_MAPPED - 1);
		switch (address & 3)
		{
		case 0: return m_rows[n].w0;
		case 1: return m_rows[n].w1;
		default: return m_rows[n].operand;
		}
	}
	if (address >= SPACE_CACHE && address < SPACE_CACHE_END)
		return m_cache[address & (OBJECTS - 1)][(address - SPACE_CACHE) / OBJECTS];
	return m_space[address];
}

void roland_xv_device::space_w(u16 address, u32 data)
{
	if (address < IBUS_IRAM_END)
		m_iram[address] = data & 0xff'ffff;
	else if (address < IBUS_BANK)
		m_bus[address - IBUS_MIX] = util::sext(data, 24);
	else if (address < IBUS_BANK_END)
	{
		if (!BIT(m_regs[MODE], 6))
			m_bank[address - IBUS_BANK] = m_bank_next[address - IBUS_BANK] = s32(s16(data)) * 256;
	}
	else if (address >= SPACE_CACHE && address < SPACE_CACHE_END)
		m_cache[address & (OBJECTS - 1)][(address - SPACE_CACHE) / OBJECTS] = data & 0xf'ffff;
	else switch (address & 0xf000)
	{
	case SPACE_IORAM:
		m_space[address] = data & 0x3ff;
		if (address < SPACE_IORAM + IORAM_WORDS)
			m_io_stale = true;
		break;

	case SPACE_RECORDS:
		m_space[address] = data;
		if (address < SPACE_RECORDS + RECORDS)
			m_transfers_stale = true;
		break;

	default:
		m_space[address] = data;
		break;
	}
}


//-------------------------------------------------
//  the FIFO and the transfer engine
//-------------------------------------------------

void roland_xv_device::fifo_rewind()
{
	m_fifo_write = 0;
	m_fifo_read = 0;
}

void roland_xv_device::fifo_push(u16 data)
{
	m_fifo[m_fifo_write] = data;
	m_fifo_write = (m_fifo_write + 1) % FIFO_DEPTH;
}

void roland_xv_device::transfer_read()
{
	const u32 address = (u32(m_regs[READ_ADDRESS]) << 16) | m_regs[READ_ADDRESS + 1];
	const u32 length = (u32(m_regs[READ_LENGTH]) << 16) | m_regs[READ_LENGTH + 1];
	fifo_rewind();
	for (u32 i = 0; i < length && i < FIFO_DEPTH; i++)
		fifo_push(m_wave.read_word(address + i));
	LOGMASKED(LOG_XFER, "%s: read %08x x %x: %04x %04x %04x %04x\n", machine().describe_context(), address, length, m_fifo[0], m_fifo[1], m_fifo[2], m_fifo[3]);
}

void roland_xv_device::transfer_write()
{
	const u32 address = (u32(m_regs[WRITE_ADDRESS]) << 16) | m_regs[WRITE_ADDRESS + 1];
	const u32 length = (u32(m_regs[WRITE_LENGTH]) << 16) | m_regs[WRITE_LENGTH + 1];
	LOGMASKED(LOG_XFER, "%s: write %08x x %x: %04x %04x %04x %04x\n", machine().describe_context(), address, length, m_fifo[0], m_fifo[1], m_fifo[2], m_fifo[3]);
	for (u32 i = 0; i < length && i < FIFO_DEPTH; i++)
		m_wave.write_word(address + i, m_fifo[i]);
	fifo_rewind();
}


//-------------------------------------------------
//  interrupts
//-------------------------------------------------

void roland_xv_device::update_irq()
{
	const bool state = (m_irq_pending & m_irq_enable) != 0;
	if (state != m_int_state)
	{
		m_int_state = state;
		m_int_callback(state ? 1 : 0);
	}
}

void roland_xv_device::raise_irq(int reason, int voice)
{
	if (BIT(m_irq_pending, reason))
	{
		m_irq_waiting[reason] |= u64(1) << voice;
		return;
	}
	m_irq_voice[reason] = voice;
	m_irq_pending |= 1 << reason;
	update_irq();
}


//-------------------------------------------------
//  the panel scanner
//-------------------------------------------------

TIMER_CALLBACK_MEMBER(roland_xv_device::scan_switches)
{
	const u64 now = m_switch_callback();
	const u64 changed = now ^ m_switch_state;
	if (!changed)
		return;
	for (int n = 0; n < 64; n++)
	{
		if (BIT(changed, n))
		{
			const int shift = (n & 3) * 4;
			u16 &w = m_scan[n >> 2];
			w = (w & ~(1 << shift)) | ((BIT(now, n) ^ 1) << shift) | ((BIT(now, n) ? 2 : 4) << shift);
			if (BIT(now, n))
				m_switch_changed |= u64(1) << n;
		}
	}
	m_switch_state = now;
	present_switch();
	update_irq();
}

void roland_xv_device::present_switch()
{
	if (BIT(m_irq_pending, IRQ_SWITCH) || !m_switch_changed)
		return;
	const int n = std::countr_zero(m_switch_changed);
	m_switch_changed &= ~(u64(1) << n);
	m_switch_index = n;
	m_irq_pending |= 1 << IRQ_SWITCH;
}


//-------------------------------------------------
//  ramps
//-------------------------------------------------

u16 roland_xv_device::ramp(u16 current, u16 target, u16 &control, s32 &slope, int use, bool event_free, u8 threshold)
{
	if ((control & RAMP_HOLD) == RAMP_HOLD)
		return current;
	const bool curve = control & RAMP_CURVE;
	if (!curve && !(control & RAMP_REUSE))
	{
		const s32 product = (s32(target) - s32(current)) * RAMP_RATE[control & 15];
		slope = (product >> 7) | ((product & 0x7f) ? 1 : 0);
	}
	control |= RAMP_REUSE;
	const bool event = control & RAMP_EVENT;
	const bool short_end = use == USE_SEND && !event;
	if (!short_end && (!curve || use == USE_PARAMETER) && current == target && (!event || event_free))
	{
		control |= RAMP_HOLD;
		return current;
	}
	s32 next = s32(current) + (slope >> 8) + (((slope & 0xff) > threshold) ? 1 : 0);
	const bool done = (curve || slope < 0) ? next < target : short_end ? next >= target : next > target;
	if (done)
	{
		if (short_end)
		{
			control |= RAMP_HOLD;
			return current;
		}
		next = target;
		if (!event || event_free)
			control |= RAMP_HOLD;
	}
	else if (curve)
		slope = std::max(-0x8'0000, slope - RAMP_ACCELERATION[control & 15]);
	return u16(std::clamp(next, 0, 0xffff));
}

void roland_xv_device::service_coefficient(int n, int i)
{
	const int command = CUTOFF_RAMP + 2 * i;
	u16 control = object_word(n, command);
	if ((control & RAMP_HOLD) == RAMP_HOLD)
		return;
	const int reason = IRQ_CUTOFF_LANDED + i;
	s32 slope = util::sext(object_long(n, CUTOFF_SLOPE + 2 * i), 20);
	object_ref(n, CUTOFF + i) = ramp(object_word(n, CUTOFF + i), object_word(n, command + 1), control, slope, USE_COEFFICIENT, !BIT(m_irq_pending, reason), m_threshold);
	object_ref(n, command) = control;
	set_long(n, CUTOFF_SLOPE + 2 * i, slope & 0xf'ffff);
	if ((control & (RAMP_HOLD | RAMP_EVENT)) == (RAMP_HOLD | RAMP_EVENT))
		raise_irq(reason, n);
}

void roland_xv_device::service_send(int n, int i)
{
	const int command = SEND_PORT_A + 2 * i;
	u16 control = object_word(n, command);
	const int slot = (control >> 7) & 7;
	if (slot >= SENDS)
		return;
	const int reason = IRQ_SEND_A_LANDED + i;
	const int level = SEND_BASE + 2 * slot + 1;
	s32 slope = util::sext(object_long(n, SEND_SLOPE_A + 2 * i), 20);
	object_ref(n, level) = ramp(object_word(n, level), object_word(n, command + 1), control, slope, USE_SEND, !BIT(m_irq_pending, reason), m_threshold);
	object_ref(n, command) = control;
	set_long(n, SEND_SLOPE_A + 2 * i, slope & 0xf'ffff);
	if ((control & (RAMP_HOLD | RAMP_EVENT)) == (RAMP_HOLD | RAMP_EVENT))
		raise_irq(reason, n);
}

void roland_xv_device::service_pitch(int n)
{
	u16 control = object_word(n, PITCH_RAMP);
	if ((control & (PITCH_REUSE | PITCH_HOLD)) == (PITCH_REUSE | PITCH_HOLD))
		return;
	const s32 current = object_long(n, PITCH_STEP) & 0x3f'ffff;
	const s32 target = ((control & 7) << 16) | object_word(n, PITCH_RAMP + 1);
	s32 slope = util::sext(object_long(n, PITCH_SLOPE), 22);
	if (!(control & PITCH_REUSE))
	{
		const s32 product = (target - current) * RAMP_RATE[(control >> 3) & 15];
		slope = (product >> 7) | ((product & 0x7f) ? 1 : 0);
		control |= PITCH_REUSE;
		set_long(n, PITCH_SLOPE, slope & 0x3f'ffff);
	}
	if (!(control & PITCH_HOLD))
	{
		s32 next = current + (slope >> 8) + (((slope & 0xff) > m_threshold) ? 1 : 0);
		if (current == target || (slope < 0 ? next < target : next > target))
		{
			next = target;
			if (!(control & PITCH_EVENT) || !BIT(m_irq_pending, IRQ_PITCH_LANDED))
			{
				control |= PITCH_HOLD;
				if (control & PITCH_EVENT)
					raise_irq(IRQ_PITCH_LANDED, n);
			}
		}
		set_long(n, PITCH_STEP, std::clamp(next, 0, 0x3f'ffff));
	}
	object_ref(n, PITCH_RAMP) = control;
}

void roland_xv_device::service_parameters()
{
	m_threshold = ramp_threshold(m_ramp_phase);
	for (u64 live = m_bank_live; live; live &= live - 1)
	{
		const int p = std::countr_zero(live);
		m_bank[p] = m_bank_next[p];
		u16 control = object_word(p, BLOCK_CONTROL);
		if ((control & RAMP_HOLD) == RAMP_HOLD)
		{
			m_bank_live &= ~(u64(1) << p);
			continue;
		}
		s32 slope = util::sext(object_long(p, PARAMETER_SLOPE), 20);
		const u16 next = ramp(u16(m_bank[p] >> 8), object_word(p, BLOCK_CONTROL + 1), control, slope, USE_PARAMETER, !BIT(m_irq_pending, IRQ_PARAMETER_LANDED), m_threshold);
		m_bank_next[p] = s32(s16(next)) * 256;
		object_ref(p, BLOCK_CONTROL) = control;
		set_long(p, PARAMETER_SLOPE, slope & 0xf'ffff);
		if ((control & (RAMP_HOLD | RAMP_EVENT)) == (RAMP_HOLD | RAMP_EVENT))
			raise_irq(IRQ_PARAMETER_LANDED, p);
	}
}

void roland_xv_device::hold_parameters()
{
	std::fill_n(m_bank, IBUS_BANK_END - IBUS_BANK, 0);
	std::fill_n(m_bank_next, IBUS_BANK_END - IBUS_BANK, 0);
	for (int p = 0; p < IBUS_BANK_END - IBUS_BANK; p++)
	{
		object_ref(p, BLOCK_CONTROL) = RAMP_HOLD | RAMP_REUSE;
		object_ref(p, BLOCK_CONTROL + 1) = 0;
		set_long(p, PARAMETER_SLOPE, 0);
	}
	m_bank_live = 0;
}


//-------------------------------------------------
//  the wave reader
//-------------------------------------------------

u32 roland_xv_device::cell_of(u32 sample, bool wide)
{
	const int bank = (sample >> 25) & 7;
	if (wide || bank == BANK_BYTE_WIDE)
		return sample & ADDRESS_MASK;
	return (sample & ADDRESS_MASK & ~0x1ff'ffff) | ((sample & ADDRESS_MASK) >> 1 & 0x1ff'ffff);
}

u8 roland_xv_device::sample_byte(u32 sample)
{
	const u16 cell = m_wave.read_word(cell_of(sample, false));
	return ((sample >> 25) & 7) != BANK_BYTE_WIDE && BIT(sample, 0) ? cell >> 8 : cell & 0xff;
}

roland_xv_device::wave_cell roland_xv_device::cell_at(u32 address, bool wide)
{
	if (wide)
		return wave_cell{ s16(m_wave.read_word(cell_of(address, true))), 0 };
	const u8 byte = sample_byte(address);
	const u8 shifts = sample_byte(in_page(address, (address & PAGE_MASK) >> 5));
	const int exponent = BIT(address, 4) ? (shifts >> 4) : (shifts & 0x0f);
	return wave_cell{ exponent > 10 ? 0 : s8(byte), exponent };
}

s32 roland_xv_device::delta_of(wave_cell c)
{
	return c.mantissa << c.exponent;
}

u16 roland_xv_device::exponent_word(u32 address)
{
	const u32 at = in_page(address, (address & PAGE_MASK & ~0x3f) >> 5);
	return sample_byte(at) | (sample_byte(at + 1) << 8);
}

void roland_xv_device::set_long(int n, int word, u32 value)
{
	object_ref(n, word) = value >> 16;
	object_ref(n, word + 1) = value & 0xffff;
}

void roland_xv_device::launch(int n)
{
	voice &v = m_voices[n];
	v.fetching = true;
	v.backward = BIT(object_word(n, VOICE_CONTROL2), 5);
	v.region = REGION_BEFORE;
	v.scaled = false;
	v.end_cell = -1;
	v.lap = false;
	set_long(n, PHASE, 0);
	object_ref(n, VOICE_CONTROL2) &= 0x1fff;
	object_ref(n, ACCUMULATOR) = 0;
}

roland_xv_device::address_step roland_xv_device::advance(int n, address_step s) const
{
	const u32 loop = object_long(n, LOOP_START) & INDEX_MASK;
	const u32 end = object_long(n, END) & INDEX_MASK;
	const u32 index = s.address & INDEX_MASK;
	const int mode = (object_word(n, VOICE_CONTROL) >> 10) & 3;

	if (!s.backward)
	{
		if (index < end)
			return { in_bank(s.address, index + 1), false, false };
		switch (mode)
		{
		case LOOP_FORWARD: return { in_bank(s.address, loop), false, true };
		case LOOP_ALTERNATE: return { s.address, true, true };
		default: return { s.address, false, false };
		}
	}

	if (mode == LOOP_ALTERNATE)
		return index <= loop ? address_step{ s.address, false, true } : address_step{ in_bank(s.address, index - 1), true, false };
	if (index > end)
		return { in_bank(s.address, index - 1), true, false };
	return mode == LOOP_FORWARD ? address_step{ in_bank(s.address, loop), true, true } : address_step{ s.address, true, false };
}

void roland_xv_device::cross(int n, u32 address)
{
	voice &v = m_voices[n];
	const u32 index = address & INDEX_MASK;
	u8 region;
	if (index == (object_long(n, END) & INDEX_MASK))
		region = REGION_END;
	else if (index == (object_long(n, LOOP_START) & INDEX_MASK))
		region = REGION_LOOP;
	else
	{
		v.region = REGION_BEFORE;
		return;
	}
	const bool arrived = region != v.region;
	v.region = region;
	const u16 control = object_word(n, VOICE_CONTROL);
	const int condition = (control >> 8) & 3;
	if (v.region == (BIT(control, 6) ? REGION_LOOP : REGION_END))
		v.scaled = true;
	if (arrived && ((v.region == REGION_END && condition == END_AT_END) || (v.region == REGION_LOOP && condition == END_INSIDE_LOOP)))
		raise_irq(IRQ_FINISHED, n);
}


//-------------------------------------------------
//  the cache
//-------------------------------------------------

void roland_xv_device::fill(int n, int consumer)
{
	voice &v = m_voices[n];
	const u16 control = object_word(n, VOICE_CONTROL);
	const bool wide = !BIT(control, 12);
	const int mode = (control >> 10) & 3;
	u16 &flags = object_ref(n, VOICE_CONTROL2);
	int producer = (flags >> 8) & 31;
	if (((producer - consumer - 1) & 31) >= CACHE_AHEAD)
		return;
	u32 acc = ((flags & 0xe000) << 3) | object_word(n, ACCUMULATOR);
	address_step s{ object_long(n, START) & ADDRESS_MASK, v.backward, false };
	while (((producer - consumer - 1) & 31) < CACHE_AHEAD)
	{
		for (int count = 4 - (s.address & 3); count; count--)
		{
			const bool fetched = v.fetching;
			if (fetched)
			{
				const wave_cell c = cell_at(s.address, wide);
				acc = (wide ? (u32(c.mantissa) & 0x3'ffff) | (v.lap ? (1 << 18) : 0) : acc + delta_of(c)) & 0x7'ffff;
				if (!wide)
					object_ref(n, EXPONENTS) = exponent_word(s.address);
				cross(n, s.address);
				if (mode == LOOP_NONE && v.region == REGION_END)
				{
					v.fetching = false;
					v.end_cell = producer;
				}
				else
				{
					s = advance(n, s);
					v.lap ^= s.wrapped;
				}
			}
			const int cell = producer & (CACHE_CELLS - 1);
			m_cache[n][cell] = fetched ? acc : 0;
			producer = (producer + 1) & 31;
		}
	}
	flags = (flags & 0x00df) | (s.backward ? 0x20 : 0) | (producer << 8) | ((acc >> 3) & 0xe000);
	object_ref(n, ACCUMULATOR) = acc & 0xffff;
	set_long(n, START, s.address);
	v.backward = s.backward;
}

s32 roland_xv_device::interpolate(int n, u32 phase, int gain) const
{
	const bool wide = !BIT(object_word(n, VOICE_CONTROL), 12);
	const int consumer = phase >> 16;
	const int fraction = (phase >> 9) & 0x7f;
	const s32 c0 = INTERP_WEIGHTS[0][fraction], c1 = INTERP_WEIGHTS[1][fraction], c2 = INTERP_WEIGHTS[2][fraction];
	const s32 weight[4] = { 4096 - c0, c0 - c1, c1 - c2, c2 };
	s32 sum = 0;
	for (int i = 0; i < 4; i++)
	{
		const s32 cell = m_cache[n][(consumer + i) & (CACHE_CELLS - 1)];
		sum += s32((s64(wide ? util::sext(cell, 18) : util::sext(cell, 19)) * weight[i]) >> 8);
	}
	return source_gain(util::sext(u32(sum), 21) >> 2, gain);
}

s32 roland_xv_device::source_gain(s32 quarters, int gain)
{
	const int e = gain >> 1;
	if (BIT(gain, 0))
		return clamp24((s64(quarters) * ODD_GAIN[e]) >> (17 - e));
	return clamp24(e >= 3 ? s64(quarters) << (e - 3) : s64(quarters) >> (3 - e));
}

u32 roland_xv_device::seam(int n, u32 phase) const
{
	const u16 seams = object_word(n, LOOP_FRACTION + 1);
	const u32 probe = ((phase & PHASE_MASK) - ((seams & 0xff) << 8)) & PHASE_MASK;
	const bool lap = BIT(m_cache[n][((probe >> 16) + 4) & (CACHE_CELLS - 1)], 18);
	if (lap == BIT(phase, 21))
		return phase;
	return ((probe + (4 << 16) + ((seams >> 8) << 8)) & PHASE_MASK) | (lap ? PHASE_LAP : 0);
}

s32 roland_xv_device::source(int n, int gain)
{
	voice &v = m_voices[n];
	u32 phase = object_long(n, PHASE) & (PHASE_MASK | PHASE_LAP);
	int consumer = (phase >> 16) & 31;
	if (v.end_cell == consumer)
	{
		set_long(n, PHASE, (phase & PHASE_LAP) | (u32(consumer) << 16));
		return 0;
	}
	s32 sample;
	const u16 control = object_word(n, VOICE_CONTROL);
	const int format = (control >> 12) & 3;
	if (format == FORMAT_FULL)
		sample = source_gain(0x7fff * 4, gain);
	else if (format == FORMAT_IDLE)
		sample = source_gain(util::sext(phase & 0xf'ffff, 20) >> 2, gain);
	else
	{
		fill(n, consumer);
		if (BIT(control, 7))
		{
			phase = seam(n, phase);
			consumer = (phase >> 16) & 31;
		}
		sample = interpolate(n, phase, gain);
	}

	u32 step = object_long(n, PITCH_STEP) & 0x3f'ffff;
	if (v.scaled)
		step = (u64(step) * object_word(n, WAVE_SCALE + 1)) >> 15;
	s64 next = s64(phase & PHASE_MASK) + step;
	if (v.end_cell >= 0 && (next >> 16) - consumer >= ((v.end_cell - consumer) & 31))
		next = s64(v.end_cell) << 16;
	set_long(n, PHASE, (u32(next) & PHASE_MASK) | (phase & PHASE_LAP));
	return sample;
}


//-------------------------------------------------
//  the filter
//-------------------------------------------------

s32 roland_xv_device::filter(int n, int type, s32 sample)
{
	voice &v = m_voices[n];
	const s32 f = object_word(n, CUTOFF);
	const s32 q = object_word(n, RESONANCE);
	const s32 band = v.filter_band;
	s32 out = sample;

	if (type >= FILTER_HIGH_POLE && type < FILTER_TYPES)
	{
		const s32 high = clamp24(s64(sample) - band);
		v.filter_band = clamp24(band + ((s64(f) * high) >> 15));
		const s32 tap = type == FILTER_HIGH_POLE ? band : high;
		out = clamp24((type == FILTER_HIGH_POLE ? high : band) + ((s64(q) * tap) >> 15));
	}
	else if (type < FILTER_HIGH_POLE)
	{
		const s32 damping = type < FILTER_LOW_SHELF ? s32((s64(q) * band) >> 15) : band;
		const s32 low = clamp24(v.filter_low + ((s64(f) * band) >> 15));
		const s32 high = clamp24(s64(sample) - low - damping);
		v.filter_low = low;
		v.filter_band = clamp24(band + ((s64(f) * high) >> 15));

		switch (type)
		{
		case FILTER_LPF: out = low; break;
		case FILTER_BPF: out = v.filter_band; break;
		case FILTER_HPF: out = high; break;
		case FILTER_PKG: out = clamp24(s64(low) - high); break;
		case FILTER_NOTCH: out = clamp24(s64(sample) - damping); break;
		case FILTER_LOW_SHELF: out = clamp24(sample + ((s64(q) * low) >> 14)); break;
		case FILTER_PEAK: out = clamp24(sample + ((s64(q) * v.filter_band) >> 14)); break;
		default: out = clamp24(sample + ((q * (s64(sample) - low - band)) >> 14)); break;
		}
	}
	set_long(n, FILTER_BAND, u32(v.filter_band) & 0xff'ffff);
	set_long(n, FILTER_LOW, u32(v.filter_low) & 0xff'ffff);
	set_long(n, FILTER_OUTPUT, u32(out) & 0xff'ffff);
	return out;
}

void roland_xv_device::service_ramps(int n)
{
	for (int i = 0; i < 3; i++)
		service_coefficient(n, i);
	for (int i = 0; i < 2; i++)
		service_send(n, i);
	service_pitch(n);
}

s32 roland_xv_device::amplitude(int n, s32 sample)
{
	const s32 out = clamp24((s64(sample) * object_word(n, AMPLITUDE)) >> 15);
	set_long(n, VOICE_OUTPUT, u32(out) & 0xff'ffff);
	return out;
}

void roland_xv_device::emit(int n, s32 output, s64 *mix, u64 &claimed) const
{
	int previous = -1;
	s64 base = 0;
	for (int slot = 0; slot < SENDS; slot++)
	{
		const int bus = object_word(n, SEND_BASE + slot * 2) & (BUSES - 1);
		const s64 level = object_word(n, SEND_BASE + slot * 2 + 1);
		const s64 old = mix[bus];
		mix[bus] = clamp24((bus == previous ? base : old) + ((output * level) >> 15));
		previous = bus;
		base = old;
		claimed |= u64(1) << bus;
	}
}

void roland_xv_device::run_voice(int n, s64 *mix, u64 &claimed)
{
	if (!running(n))
		return;
	const s32 input = source(n, object_word(n, VOICE_CONTROL) & 0xf);
	service_ramps(n);
	emit(n, amplitude(n, filter(n, object_word(n, FILTER_TYPE) & 0xf, input)), mix, claimed);
}


//-------------------------------------------------
//  paired structures
//-------------------------------------------------

int roland_xv_device::structure(int n) const
{
	if (n + 1 >= OBJECTS || !running(n))
		return STRUCTURE_NONE;
	const int kind = (object_word(n, FILTER_TYPE) >> 8) & 0xf;
	switch (kind)
	{
	case STRUCTURE_SUM: case STRUCTURE_BOOST: case STRUCTURE_BOOST_FILTERED:
	case STRUCTURE_RING: case STRUCTURE_RING_CARRIER: case STRUCTURE_RING_FILTERED: case STRUCTURE_RING_FILTERED_CARRIER:
	case STRUCTURE_RING_BOTH: case STRUCTURE_RING_BOTH_CARRIER:
		return kind;
	}
	return STRUCTURE_NONE;
}

void roland_xv_device::run_pair(int n, int kind, s64 *mix, u64 &claimed)
{
	// FIXME: a pair's voices drop gain code bit 3, fitted to a plugin; unmeasured on silicon
	const s32 w1 = source(n, object_word(n, VOICE_CONTROL) & 7);
	service_ramps(n);
	s32 w2 = 0;
	if (running(n + 1))
	{
		w2 = source(n + 1, object_word(n + 1, VOICE_CONTROL) & 7);
		service_ramps(n + 1);
	}

	const u16 types = object_word(n, FILTER_TYPE);
	const int type1 = types & 0xf, type2 = (types >> 4) & 0xf;
	voice &partner = m_voices[n + 1];
	const s32 ceiling = m_voices[n].pair_ceiling;
	const s32 rate = object_word(n + 1, PAIR_BOOST);
	const s32 gain = object_word(n, PAIR_BOOST);

	auto smooth = [&](s32 x)
	{
		const s32 d = clamp24(s64(x) - partner.pair_smooth);
		partner.pair_smooth = clamp24(partner.pair_smooth + (s64(d) * rate) / (1 << 15));
		return d;
	};
	auto boost = [&](s32 x) { return s32(std::clamp<s64>((s64(x) * gain) / (1 << 8), -ceiling, ceiling)); };
	auto ring = [](s32 a, s32 b) { return clamp24((s64(a) * b) / (1 << 15)); };

	s32 out;
	switch (kind)
	{
	case STRUCTURE_SUM:
		out = filter(n, type1, smooth(clamp24(s64(amplitude(n, w1)) + w2)));
		break;
	case STRUCTURE_BOOST:
		out = filter(n, type1, smooth(boost(clamp24(s64(amplitude(n, w1)) + w2))));
		break;
	case STRUCTURE_BOOST_FILTERED:
		out = smooth(boost(filter(n, type1, clamp24(s64(amplitude(n, w1)) + w2))));
		break;
	case STRUCTURE_RING:
	case STRUCTURE_RING_CARRIER:
		out = filter(n, type1, smooth(clamp24(s64(ring(amplitude(n, w1), w2)) + (kind == STRUCTURE_RING_CARRIER ? w2 : 0))));
		break;
	case STRUCTURE_RING_FILTERED:
	case STRUCTURE_RING_FILTERED_CARRIER:
		out = smooth(clamp24(s64(ring(amplitude(n, filter(n, type1, w1)), w2)) + (kind == STRUCTURE_RING_FILTERED_CARRIER ? w2 : 0)));
		break;
	default:
	{
		const s32 y2 = filter(n + 1, type2, w2);
		out = clamp24(s64(ring(amplitude(n, filter(n, type1, w1)), y2)) + (kind == STRUCTURE_RING_BOTH_CARRIER ? y2 : 0));
		emit(n, amplitude(n + 1, out), mix, claimed);
		return;
	}
	}
	emit(n, amplitude(n + 1, filter(n + 1, type2, out)), mix, claimed);
}


//-------------------------------------------------
//  the DSP
//-------------------------------------------------

void roland_xv_device::row_changed(int n)
{
	decode_row(n);
	if (m_recompiler)
		m_recompiler->touched(n);
}

void roland_xv_device::decode_row(int n)
{
	dsp_row &r = m_rows[n];
	u16 control = r.w0;
	u16 bus = r.w1;
	const bool conditional = BIT(r.w0, 15);
	if (!conditional)
		r.kind = ROW_PLAIN;
	else if (BIT(r.w0, 14))
		r.kind = ROW_BRANCH;
	else if ((r.w1 & 0x0c00) == 0x0c00)
		r.kind = ROW_JUMP;
	else
		r.kind = ROW_PREDICATED;
	const bool predicated = r.kind == ROW_PREDICATED;
	r.condition = r.w1 & 0xf;
	r.predicate = predicated ? (r.w1 >> 8) & 0xf : 0;
	r.argument = (r.w1 >> 4) & 0x3f;
	r.target = r.w1 & 0x3ff;
	r.displacement = s8((r.w1 >> 4) & 0xff);
	const bool parameter = predicated && r.predicate >= PREDICATE_PARAMETER && r.predicate <= PREDICATE_PARAMETER_STORE;
	const bool flow_memory = (r.kind == ROW_BRANCH || r.kind == ROW_JUMP) && BIT(r.w1, 12);
	if (conditional)
	{
		control &= 0x3fff;
		if (BIT(r.w1, 12) && !BIT(r.w1, 15))
			control |= 0x4000;
		bus &= 0xe000;
	}
	r.mode = (bus >> 10) & 7;
	r.address = bus & 0x3ff;
	r.mode2 = (r.operand >> 10) & 7;
	r.address2 = r.operand & 0x3ff;
	r.second = flow_memory || ((parameter || (BIT(control, 14) && !BIT(bus, 15))) && r.mode2 != MEM_NONE);
	const u16 literal = (flow_memory || (r.second && parameter)) ? (r.operand & 0xe000) : r.second ? 0 : r.operand;
	r.coefficient = s32(s16(literal)) * 256;
	r.gated = predicated && (r.predicate == PREDICATE_WRITE || r.predicate == PREDICATE_SHIFT || r.predicate == PREDICATE_SPECIAL);
	r.special = predicated && (r.predicate == PREDICATE_SHIFT || r.predicate == PREDICATE_SPECIAL);
	r.hold = !(control & 0x3ff0) && !(predicated && r.predicate <= PREDICATE_SPECIAL);
	r.wrap = BIT(control, 13);
	r.to_b = BIT(control, 12);
	for (int f = 0; f < 2; f++)
	{
		u16 c = control;
		u32 coefficient_mode = ((bus >> 13) & 7) | ((control & 1) << 3);
		u32 source = (control >> 1) & 7;
		if (f && predicated)
		{
			const u32 alternate = r.argument & 0xf;
			switch (r.predicate)
			{
			case PREDICATE_LEFT: c = (c & ~0x0f00) | (alternate << 8); break;
			case PREDICATE_RIGHT: c = (c & ~0x00f0) | (alternate << 4); break;
			case PREDICATE_SOURCE: source = alternate & 7; break;
			case PREDICATE_COEFFICIENT: coefficient_mode = alternate; break;
			}
		}
		dsp_fields &d = r.fields[f];
		d.left = (c >> 8) & 7;
		d.right = (c >> 4) & 7;
		d.negate_left = BIT(c, 11);
		d.negate_right = BIT(c, 7);
		d.source = source;
		d.multiply = BIT(coefficient_mode, 2);
		d.cell_coefficient = BIT(coefficient_mode, 3);
		d.shift = (0x4210 >> (4 * (coefficient_mode & 3))) & 0xf;
		const bool immediate = d.right == RIGHT_K23 || d.right == RIGHT_K19 || d.right == RIGHT_K15;
		d.clamp = immediate && (d.left != LEFT_ZERO || d.negate_right);
		if (d.right == RIGHT_K23)
			d.immediate = s32(literal) * 16;
		else if (d.right == RIGHT_K19)
			d.immediate = s32(literal) * 256;
		else
			d.immediate = s32(s16(literal)) * 4096;
	}
	const dsp_fields &f = r.fields[0];
	r.inert = !predicated && r.hold && !r.second && r.mode == MEM_NONE && !r.address
			&& !f.multiply && !f.cell_coefficient && f.source == SOURCE_R;

	const bool nop = !r.w0 && !r.w1;
	if (!nop && n > m_rows_end)
		m_rows_end = n;
	else if (nop && n == m_rows_end)
	{
		while (m_rows_end >= 0 && !m_rows[m_rows_end].w0 && !m_rows[m_rows_end].w1)
			m_rows_end--;
	}
}

template <typename T>
static void sort_transfers(T *t, int count)
{
	for (int i = 1; i < count; i++)
	{
		const T e = t[i];
		int j = i;
		for ( ; j > 0 && t[j - 1].slot > e.slot; j--)
			t[j] = t[j - 1];
		t[j] = e;
	}
}

void roland_xv_device::decode_transfers()
{
	m_transfer_count = 0;
	m_transfers_stale = false;
	bool open = false, first = false;
	u16 read = 0, write = 0;
	u32 fetch = 0, ready = 0;
	for (int n = 0; n < RECORDS; n++)
	{
		const u32 word = m_space[SPACE_RECORDS + n] & 0xf'ffff;
		if ((word & 0xf'c000) == 0xf'c000)
		{
			write = IBUS_STAGING | ((word >> 7) & 0x7f);
			read = IBUS_STAGING | (word & 0x7f);
			open = first = true;
			fetch = ready = std::max(fetch, ready) + 1;
		}
		else if (open && word)
		{
			const u16 slot = std::max(fetch, ready);
			if (BIT(word, 19))
			{
				m_transfers[m_transfer_count++] = transfer{ slot, XFER_WRITE, write, word & 0x7'ffff };
				write = IBUS_STAGING | ((write + 1) & 0x7f);
			}
			else
			{
				m_transfers[m_transfer_count++] = transfer{ slot, XFER_READ, u16(n), word & 0x7'ffff };
				m_transfers[m_transfer_count++] = transfer{ u16(slot + 4), XFER_RETURN, read, u32(n) };
				read = IBUS_STAGING | ((read + 1) & 0x7f);
			}
			fetch = slot + 2;
			ready = slot + (first ? 3 : 2);
			first = false;
		}
		else
			fetch++;
	}
	sort_transfers(m_transfers, m_transfer_count);
}

s32 roland_xv_device::cell_r(u16 address) const
{
	if (address < IBUS_STATIC)
		return util::sext(m_iram[ring_index(address)], 24);
	if (address < IBUS_MIX)
		return util::sext(m_iram[address], 24);
	if (address < IBUS_BANK)
		return m_bus[address - IBUS_MIX];
	if (address < IBUS_BANK_END)
		return bank_r(address - IBUS_BANK);
	return 0;
}

void roland_xv_device::cell_w(u16 address, s32 value)
{
	if (address < IBUS_STATIC)
		m_iram[ring_index(address)] = u32(value);
	else if (address < IBUS_MIX)
		m_iram[address] = u32(value);
	else if (address < IBUS_BANK)
		m_bus[address - IBUS_MIX] = value;
}

void roland_xv_device::swap_work()
{
	m_dsp->work_phase ^= 1;
	m_bus = m_work + (m_dsp->work_phase ? WORK_CELLS : 0);
	m_bus_other = m_work + (m_dsp->work_phase ? 0 : WORK_CELLS);
}

void roland_xv_device::return_w(u16 cell, s32 value, int order, bool later)
{
	if (cell >= IBUS_MIX && cell < IBUS_BANK)
	{
		s16 &previous = m_return_order[later ? 1 : 0][cell - IBUS_MIX];
		if (order < previous)
			return;
		previous = order;
		(later ? m_bus_other : m_bus)[cell - IBUS_MIX] = value;
	}
	else
		cell_w(cell, value);
}

void roland_xv_device::run_transfers()
{
	if (m_transfers_stale)
		decode_transfers();
	dsp_state &s = *m_dsp;
	if (!m_transfer_count && !s.tap_count)
		return;
	std::fill_n(&m_return_order[0][0], 2 * WORK_CELLS, -1);

	int taps = 0;
	for (u32 n = 0; n < s.tap_count; n++)
	{
		const u16 slot = m_tap_slot[n];
		const s32 whole = m_tap_delay[n] >> 4;
		m_tap_events[taps++] = transfer{ slot, XFER_TAP_FIRST, u16(n), u32(whole - (slot + 2) / DSP_ROW_BUDGET) };
		m_tap_events[taps++] = transfer{ slot, XFER_TAP_SECOND, u16(n), u32(whole + 1 - (slot + 5) / DSP_ROW_BUDGET) };
		m_tap_events[taps++] = transfer{ u16(slot + 4), XFER_TAP_FRACTION, u16(n), 0 };
		m_tap_events[taps++] = transfer{ u16(slot + 5), XFER_TAP_FIRST_RETURN, u16(n), 0 };
		m_tap_events[taps++] = transfer{ u16(slot + 7), XFER_TAP_SECOND_RETURN, u16(n), 0 };
	}
	sort_transfers(m_tap_events, taps);

	int fixed = 0, tap = 0;
	while (fixed < m_transfer_count || tap < taps)
	{
		const bool record = tap == taps || (fixed < m_transfer_count && m_transfers[fixed].slot <= m_tap_events[tap].slot);
		const transfer &t = record ? m_transfers[fixed++] : m_tap_events[tap++];
		switch (t.kind)
		{
		case XFER_WRITE:
			m_eram[eram_index(t.offset)] = cell_r(t.cell);
			break;
		case XFER_READ:
			m_fetched[t.cell] = m_eram[eram_index(t.offset)];
			break;
		case XFER_RETURN:
			return_w(t.cell, m_fetched[t.offset], 2 * (t.slot - 4), false);
			break;
		case XFER_TAP_FIRST:
		case XFER_TAP_SECOND:
			m_tap_fetched[t.cell][t.kind - XFER_TAP_FIRST] = m_eram[eram_index(t.offset)];
			break;
		default:
		{
			const int component = t.kind == XFER_TAP_FRACTION ? 2 : t.kind - XFER_TAP_FIRST_RETURN;
			s32 value = 0;
			if (t.slot != DSP_ROW_BUDGET - 1)
				value = component == 2 ? (m_tap_delay[t.cell] & 15) * 0x8'0000 : m_tap_fetched[t.cell][component];
			return_w(m_tap_cell[t.cell] + component, value, 2 * m_tap_slot[t.cell] + 1, t.slot >= DSP_ROW_BUDGET);
			break;
		}
		}
	}
	s.tap_count = 0;
}

void roland_xv_device::run_dsp(const s64 *mix, u64 claimed)
{
	dsp_state &s = *m_dsp;
	swap_work();
	for (int bus = 0; claimed; bus++, claimed >>= 1)
		if (BIT(claimed, 0))
			m_bus[bus] = clamp24(mix[bus]);
	run_transfers();
	for (u32 n = 0; n < s.late_count; n++)
		m_bus[m_late_cell[n] - IBUS_MIX] = 0;
	s.late_count = 0;
	service_parameters();

	if (m_recompiler)
		m_recompiler->run();
	else
		interpret();
	s.cursor--;
	m_ramp_phase++;
}

void roland_xv_device::interpret()
{
	m_dsp->steps = 0;
	interpret(0, -1);
}

void roland_xv_device::interpret(int pc, int pending)
{
	dsp_state &s = *m_dsp;
	while (pc < DSP_ROWS_MAPPED && s.steps < DSP_ROW_BUDGET)
	{
		if (pc > m_rows_end && pending < 0)
		{
			if (!s.last_hold)
			{
				s.flag = s.last;
				s.flag_carry = s.last_carry;
				s.last_hold = 1;
			}
			s.steps = std::min<u32>(DSP_ROW_BUDGET, s.steps + DSP_ROWS_MAPPED - pc);
			break;
		}
		const dsp_row &row = m_rows[pc];
		bool taken = false, commit = true, predicate = true;
		switch (row.kind)
		{
		case ROW_PREDICATED:
			commit = condition(row.condition);
			predicate = commit || !row.condition;
			break;
		case ROW_BRANCH:
			taken = !row.condition || condition(row.condition);
			break;
		case ROW_JUMP:
			taken = true;
			break;
		}
		if (taken && pending < 0 && row.inert && s.last_hold && pc + 1 < DSP_ROWS_MAPPED)
		{
			const dsp_row &slot = m_rows[pc + 1];
			const int target = row.kind == ROW_JUMP ? row.target : (pc + 1 + row.displacement) & (DSP_ROWS - 1);
			if (target == pc && slot.inert && slot.kind == ROW_PLAIN)
			{
				s.steps = DSP_ROW_BUDGET;
				break;
			}
		}
		if (!row.inert)
			execute(row, commit, predicate);
		else if (!s.last_hold)
		{
			s.flag = s.last;
			s.flag_carry = s.last_carry;
			s.last_hold = 1;
		}
		s.steps++;
		if (pc + 1 == DSP_ROWS_MAPPED)
			break;
		const int next = pending >= 0 ? pending : pc + 1;
		pending = -1;
		if (taken)
			pending = row.kind == ROW_JUMP ? row.target : (next + row.displacement) & (DSP_ROWS - 1);
		pc = next;
	}
	if (s.steps < DSP_ROW_BUDGET && !s.last_hold)
	{
		s.flag = s.last;
		s.flag_carry = s.last_carry;
		s.last_hold = 1;
	}
}

bool roland_xv_device::condition(int code) const
{
	const s32 f = m_dsp->flag;
	const bool carry = m_dsp->flag_carry;
	switch (code)
	{
	case 0x0: return false;
	case 0x1: return true;
	case 0x2: return f == 0;
	case 0x3: return f != 0;
	case 0x4: return carry;
	case 0x5: return !carry;
	case 0x6: return f >= 0x800'0000 || f < -0x800'0000;
	case 0x7: return f < 0x800'0000 && f >= -0x800'0000;
	case 0x8: return f >= 0;
	case 0x9: return f < 0;
	case 0xa: return f > 0;
	case 0xb: return f <= 0;
	case 0xc: return carry || f == 0;
	case 0xd: return !carry && f != 0;
	case 0xe: return carry && f != 0;
	default: return !carry || f == 0;
	}
}

void roland_xv_device::execute(const dsp_row &row, bool commit, bool predicate)
{
	dsp_state &s = *m_dsp;
	const dsp_fields &f = row.fields[predicate ? 0 : 1];
	const s32 aq = s.acc[0], bq = s.acc[1];
	const s32 a = aq >> 4, b = bq >> 4;
	const s64 product_entered = s.product;
	const u8 shift_entered = s.product_shift;
	const s32 p = s32(product_entered >> 4);
	const s32 r = s.latch[0], sl = s.latch[1], m = s.latch[2], c = s.latch[3];

	const auto access = [&](int mode, u16 address)
	{
		switch (mode)
		{
		case MEM_NONE:
			if (BIT(address, 9) && s.tap_count < TAPS)
			{
				m_tap_cell[s.tap_count] = address | 0x100;
				m_tap_delay[s.tap_count] = BIT(address, 8) ? b : a;
				m_tap_slot[s.tap_count] = s.steps;
				s.tap_count++;
			}
			break;
		case MEM_STORE_P:
			cell_w(address, clamp24(p));
			break;
		case MEM_STORE_A:
		case MEM_STORE_B:
			if (s.steps == DSP_ROW_BUDGET - 1 && address >= IBUS_MIX && address < IBUS_BANK)
			{
				if (s.late_count < 2)
					m_late_cell[s.late_count++] = address;
			}
			else
				cell_w(address, clamp24(mode == MEM_STORE_A ? a : b));
			break;
		default:
			s.latch[mode - MEM_READ_R] = cell_r(address);
			break;
		}
	};
	if (row.mode != MEM_NONE || row.address)
		access(row.mode, row.address);
	if (row.second)
	{
		if (predicate || row.predicate < PREDICATE_PARAMETER)
			access(row.mode2, row.address2);
		else if (row.mode2 >= MEM_READ_R)
			s.latch[row.mode2 - MEM_READ_R] = bank_r(row.argument);
		else if (row.predicate == PREDICATE_PARAMETER_STORE)
			access(row.mode2, row.address2);
	}

	s32 coefficient = 0;
	bool product = true;
	if (f.multiply)
		coefficient = f.cell_coefficient ? (c & 0xff) * 0x8000 : row.coefficient;
	else if (f.cell_coefficient)
		coefficient = c & ~0xff;
	else if (f.source >= SOURCE_M)
		coefficient = clamp24(a) & ~0xff;
	else if (f.source == SOURCE_S)
		coefficient = s32(s16(u16(c >> 8) | 0x8000)) * 256;
	else
		product = false;
	if (product)
	{
		s32 operand = 0;
		switch (f.source)
		{
		case SOURCE_R: operand = r; break;
		case SOURCE_S: operand = sl; break;
		case SOURCE_M: operand = m; break;
		case SOURCE_C: operand = c; break;
		case SOURCE_A: operand = clamp24(a); break;
		case SOURCE_B: operand = clamp24(b); break;
		case SOURCE_P: operand = clamp24(p); break;
		}
		s.product = (s64(operand) * coefficient) >> (19 - f.shift);
		s.product_shift = f.multiply && f.cell_coefficient ? 15 : 0;
	}

	if (row.hold)
	{
		if (!s.last_hold)
		{
			s.flag = s.last;
			s.flag_carry = s.last_carry;
			s.last_hold = 1;
		}
		return;
	}

	s64 left = 0;
	switch (f.left)
	{
	case LEFT_R: left = s64(r) * 16; break;
	case LEFT_S: left = s64(sl) * 16; break;
	case LEFT_M: left = s64(m) * 16; break;
	case LEFT_A: left = clamp28(aq); break;
	case LEFT_B: left = clamp28(bq); break;
	case LEFT_MAG_A: left = std::abs(clamp28(aq)); break;
	case LEFT_MAG_B: left = std::abs(clamp28(bq)); break;
	}
	s64 right = 0;
	switch (f.right)
	{
	case RIGHT_A: right = clamp28(aq); break;
	case RIGHT_B: right = clamp28(bq); break;
	case RIGHT_R: right = s64(r) * 16; break;
	case RIGHT_K23: case RIGHT_K19: case RIGHT_K15: right = f.immediate; break;
	case RIGHT_P: right = clamp28(product_entered) >> shift_entered; break;
	}

	s64 result;
	bool carry = false;
	bool overflow = false;
	if (!row.special)
	{
		result = (f.negate_left ? -left : left) + (f.negate_right ? -right : right);
		constexpr u64 mask = 0x1fff'ffff;
		const u64 carry_left = (u64(left) ^ (f.negate_left ? mask : 0)) & mask;
		const u64 carry_right = (u64(right) ^ (f.negate_right ? mask : 0)) & mask;
		carry = carry_left + carry_right + (f.negate_left ? 1 : 0) + (f.negate_right ? 1 : 0) > mask;
	}
	else
	{
		s64 special = left;
		if (f.left >= LEFT_MAG_A && (f.left == LEFT_MAG_A ? aq : bq) < 0)
			special--;
		if (f.negate_left)
			special = ~special;
		if (row.predicate == PREDICATE_SHIFT)
		{
			const u32 count = (row.argument & 0xf) ? (row.argument & 0xf) : u32((f.negate_right ? ~right : right) >> 4) & 0xf;
			result = special * (s64(1) << count);
			overflow = result != clamp28(result);
			carry = ((u64(special) & 0xfff'ffff) << count) > 0xfff'ffff;
			result = row.wrap ? util::sext(result, 28) : clamp28(result);
		}
		else
		{
			const s32 l = s32(special >> 4), rv = s32(right >> 4);
			const s32 rs = f.negate_right ? ~rv : rv;
			const s64 difference = (f.negate_left ? -left : left) - right;
			const s64 comparison = row.wrap ? difference : util::sext(difference, 28);
			s32 v = l;
			bool bypass = false;
			switch (row.argument & 7)
			{
			case SPECIAL_MAX: bypass = comparison < 0; break;
			case SPECIAL_MIN: bypass = comparison >= 0; break;
			case SPECIAL_FEEDBACK:
			{
				const s64 wide = special * 2;
				overflow = !row.wrap && wide != clamp28(wide);
				carry = BIT(wide, 28);
				const s32 shifted = row.wrap ? s32(wide >> 4) : clamp24(wide >> 4);
				const u32 input = u32(shifted >> 1);
				v = (shifted & ~1) | ((input ^ (input >> 1) ^ (input >> 6) ^ (input >> 23)) & 1) | ((special == 0) ? 1 : 0);
				break;
			}
			case SPECIAL_NORMALIZE:
			{
				const u32 magnitude = u32(l < 0 ? ~l : l) & 0x7f'ffff;
				v = 0;
				while (v < 15 && !(magnitude & (0x40'0000 >> v)))
					v++;
				break;
			}
			case SPECIAL_AND: v = l & rs; break;
			case SPECIAL_OR: v = l | rs; break;
			case SPECIAL_XOR: v = l ^ rs; break;
			case SPECIAL_COPY: break;
			}
			result = bypass ? right : s64(v) * 16;
		}
	}
	result = util::sext(result, 29);
	const s32 raw = s32(overflow ? util::sext(result ^ (s64(1) << 28), 29) : result);
	if (f.clamp && !row.wrap)
		result = clamp28(result);
	if (commit || !row.gated)
		s.acc[row.to_b ? 1 : 0] = s32(row.wrap ? util::sext(result, 28) : result);
	if (!s.last_hold)
	{
		s.flag = s.last;
		s.flag_carry = s.last_carry;
	}
	s.last = raw;
	s.last_carry = carry;
	s.last_hold = row.gated && !commit && row.condition;
}
