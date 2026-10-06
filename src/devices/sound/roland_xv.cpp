// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Roland XV tone generator (TC223C660CF-503, RA08-503)

    The XP's successor: 64 voices a chip, two chips in an XV-5080.  The
    host sees a 16-bit register window: the address spaces reached through
    it (the DSP program rows, the scalars, the wide records and the
    per-voice longs), the FIFO transfer engine onto the chip's own wave and
    sample memory, the object-indexed voice file and the interrupt path.

    A voice reads the XP's 8-bit sample format, or a 16-bit linear one,
    at a 28-bit address - a memory bank in bits 27:25, a 1 Mi-sample
    page in it and a 20-bit index the loop points share - and steps it by
    a linear 18-bit pitch (0x10000 = one sample an output sample),
    scaled by word 0x76/77 from its first loop crossing on, decodes twelve
    samples ahead into a sixteen-cell cache it interpolates from, runs its
    cutoff, resonance, amplitude and pitch through host-targeted ramps
    with a 4-bit rate code each, filters, scales by a Q15 amplitude and adds
    itself to up to six of the DSP's sixty-four mix cells at six send levels.

    A mix cell an enabled voice sends to starts each sample afresh; the
    others keep what the DSP left in them.  The DSP
    runs its program once an output sample: 768 issued three-word rows
    from the 896 it maps, with a multiplier, two accumulators, four read
    latches, delayed flags and a conditional family (predicates, special
    ALU operations, branches and jumps with one delay slot), over a
    512-cell ring that slides one cell a sample, 128 static cells, the
    mix cells, transport slots and staging cells in two work banks the
    samples take in turn, and a parameter bank of sixty-four coefficients
    the host ramps through the object path.  Wide records at 0x3000 move
    cells to and from a 2^19-cell external memory behind a cursor that
    falls once a sample, in service slots interleaved with the fractional
    taps a row of memory mode 0 with an address requests.  An I/O
    sequencer's table at 0x2000 names the transport slots the serial pins
    carry: the words sent to and taken
    from the other chip, which the chip given set_link() runs from its own
    stream, and the words of the five outputs, the stream's channels.

    The accumulators are 24.4 with a guard bit and are clipped to
    7fffff.f when fed back; products keep four bits below a cell's step
    and floor; stores clamp to 24 bits.

    TODO:
    - the filter's update order and precision with moving integrators
    - the source gain's odd codes; loop points, reverse play and the
      DPCM history at a loop or a relaunch; the cache's twentieth bit
    - the mix's clip point, at each deposit or at the sum
    - the multiplier's guard width beyond the clipped ports, predicate mode
      5's other role, ERAM arbitration when a tap, a fixed read and a fixed
      write share a cell, and the last records of the table
    - which work bank the host reaches; when the parameter
      ramps' event bit raises reason 0 and is taken back
    - the I/O sequencer's control words, its waits and the pins' timing
    - the bank widths the configuration words 0x12-0x19 presumably carry
    - paired structures 4-6, which the firmware never writes
    - reason 8

***************************************************************************/

#include "emu.h"
#include "roland_xv.h"


#define LOG_REGS    (1U << 1)
#define LOG_XFER    (1U << 2)
#define LOG_SPACE   (1U << 3)
#define LOG_OBJECT  (1U << 4)
#define LOG_IRQ     (1U << 5)
#define LOG_DSP     (1U << 6)

#define VERBOSE (LOG_GENERAL)
#include "logmacro.h"

namespace {

const s16 interp_weights[3][128] = {
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

const s32 ramp_rate[roland_xv_device::RATE_CODES] = {
	368, 184, 124, 92, 74, 62, 54, 46, 41, 37, 31, 27, 23, 21, 19, 15 };

const s32 ramp_acceleration[roland_xv_device::RATE_CODES] = {
	896, 768, 640, 512, 448, 384, 320, 256, 224, 192, 160, 128, 112, 96, 80, 64 };

s32 sext(u32 value, int bits) { return s32(value << (32 - bits)) >> (32 - bits); }

} // anonymous namespace

DEFINE_DEVICE_TYPE(ROLAND_XV, roland_xv_device, "roland_xv", "Roland XV")

roland_xv_device::roland_xv_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, ROLAND_XV, tag, owner, clock)
	, device_memory_interface(mconfig, *this)
	, device_sound_interface(mconfig, *this)
	, m_link(*this, finder_base::DUMMY_TAG)
	, m_wave_config("wave", ENDIANNESS_LITTLE, 16, 32, -1)
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

device_memory_interface::space_config_vector roland_xv_device::memory_space_config() const
{
	return space_config_vector { std::make_pair(AS_WAVE, &m_wave_config) };
}

// the state the rows touch sits near the recompiler's code when there is one
void *roland_xv_device::alloc_near(size_t bytes, size_t align)
{
	if (m_recompiler)
		return m_recompiler->alloc_near(bytes, align);
	m_heap.emplace_back(new u8[bytes + align]);
	return reinterpret_cast<void *>((uintptr_t(m_heap.back().get()) + align - 1) & ~uintptr_t(align - 1));
}

void roland_xv_device::device_start()
{
	space(AS_WAVE).specific(m_wave);
	m_stream = stream_alloc(0, OUTPUTS, sample_rate());
	m_space = std::make_unique<u32[]>(0x10000);
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
	save_pointer(NAME(m_space), 0x10000);
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
	save_item(STRUCT_MEMBER(m_voices, wrap_cells));
	save_item(STRUCT_MEMBER(m_voices, region));
	save_item(STRUCT_MEMBER(m_voices, scaled));
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
	std::fill(&m_object_regs[0][0], &m_object_regs[0][0] + OBJECTS * (OBJECT_END - OBJECT_BASE), 0);
	std::fill(&m_cache[0][0], &m_cache[0][0] + OBJECTS * CACHE_CELLS, 0);
	std::fill_n(m_space.get(), 0x10000, 0);
	m_address = 0;
	m_high_latch = 0;
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
	for (int n = 0; n < OBJECTS; n++)
	{
		m_voices[n] = voice();
		for (int word : { CUTOFF_RAMP, RESONANCE_RAMP, AMPLITUDE_RAMP, SEND_PORT_A, SEND_PORT_B })
			object_ref(n, word) = RAMP_HOLD | RAMP_REUSE;
		object_ref(n, PITCH_RAMP) = PITCH_HOLD | PITCH_REUSE;
	}
	for (auto &r : m_rows)
		r = dsp_row();
	m_rows_end = -1;
	std::fill_n(m_eram.get(), ERAM_CELLS, 0);
	std::fill_n(m_iram, IBUS_IRAM_END, 0);
	std::fill_n(m_work, 2 * WORK_CELLS, 0);
	hold_parameters();
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

// a linked chip runs at its master's rate, so the master's clock is passed on to it
void roland_xv_device::device_clock_changed()
{
	const u32 rate = sample_rate();
	m_stream->set_sample_rate(rate);
	const attotime sample = attotime::from_hz(rate);
	m_stream_timer->adjust(m_master ? attotime::never : sample, 0, sample);
	if (m_link)
		m_link->set_unscaled_clock(clock());
}

// a linked chip's frames are run from its master's stream; its own stream is silent
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

// the voices raise their interrupts from the stream, so it is brought up to
// date every sample
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

// one output sample: the voices onto the mix cells they claim, then the DSP over them
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
//  the I/O sequencer: its table at 0x2000 is read once a sample.  A
//  transmit, a strobe and a receive each take the word at their pointer
//  and step it on, wrapping in the 64 cells from 0x2c0; the n-th primary
//  receive takes the n-th word the other chip transmits, the auxiliary
//  receive takes the disconnected input's all-ones word, and the first
//  and second strobe of a kind are an output's left and right word.
//  Both chips' transmits are taken before either chip receives, and the
//  words land in the bank each chip has just run.
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
//  the window: a word is its high byte then its low byte, and the low byte
//  is where a write commits and a read takes its side effects
//-------------------------------------------------

u8 roland_xv_device::read(offs_t offset)
{
	const int word = (offset >> 1) & 0xff;
	if (word == DATA_HIGH || word == DATA_LOW)
		sync();
	const u16 data = word_peek(word);
	if (!BIT(offset, 0))
		return data >> 8;
	if (!machine().side_effects_disabled())
		word_taken(word);
	return data & 0xff;
}

void roland_xv_device::write(offs_t offset, u8 data)
{
	const int word = (offset >> 1) & 0xff;
	if (word >= OBJECT_BASE || (word >= RUN_MASK && word <= RUN_COMMIT) || word == DATA_LOW)
		sync();
	if (!BIT(offset, 0))
		m_regs[word] = (m_regs[word] & 0x00ff) | (data << 8);
	else
	{
		const u16 value = (m_regs[word] & 0xff00) | data;
		if (!BIT(word, 0))
			m_high_latch = value;
		word_w(word, value);
	}
}

u16 roland_xv_device::word_peek(int word)
{
	switch (word)
	{
	case DATA_HIGH:
		return space_r(m_address) >> 16;

	case DATA_LOW:
		return space_r(m_address) & 0xffff;

	case FIFO:
		if (m_scan_select >= 0)
			return m_scan[(m_scan_select + m_scan_read) & 15];
		if (m_led_select >= 0)
			return m_led[(m_led_select + m_scan_read) & 15];
		return m_fifo[m_fifo_read];

	case IRQ_MASK:
		return m_irq_pending & m_irq_enable;

	case 0x24:      // the SD-90's boot loader reads bit 13 here after every display byte
		return 0xffff;

	case SWITCH_INDEX:
		return m_switch_index;

	case STATUS:
		return 0;

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

void roland_xv_device::word_taken(int word)
{
	switch (word)
	{
	case DATA_LOW:
		LOGMASKED(LOG_SPACE, "%s: read %04x = %08x\n", machine().describe_context(), m_address, space_r(m_address));
		m_address = next_address(m_address);
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
		if (BIT(data, 6))
			hold_parameters();
		break;

	case DATA_LOW:
		space_w(m_address, (u32(m_high_latch) << 16) | data);
		LOGMASKED(LOG_SPACE, "%s: write %04x = %08x\n", machine().describe_context(), m_address, space_r(m_address));
		m_address = next_address(m_address);
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
					if ((changed >> (k * 4)) & 15)
						m_led_callback((w >> 1) * 8 + (w & 1) * 4 + k, (data >> (k * 4)) & 15);
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
			m_fifo_write = m_fifo_read = data & 0x3f;   // 0x200+n selects the word, as 0x240+n and 0x260+n do
		else if (data < 0x0200)
			m_fifo_write = m_fifo_read = data;
		else
			fifo_rewind();
		break;

	case COMMAND_STROBE:
		// bit 15 issues the command and reads back set until it is taken; the
		// host clears it by writing the word back, which is not a second one
		if (!BIT(data, 15))
			break;
		LOGMASKED(LOG_XFER, "%s: command %04x (strobe %04x)\n", machine().describe_context(), m_fifo[0], data);
		// under mode 0 each word is a byte for the display on the chip's own
		// LCD pins, its bit 8 the RS line, and bits 13:8 of the strobe say how
		// many; the modes the transfer engine and the streams set take it elsewhere
		if ((m_regs[MODE] & 0xffc0) == 0)
			for (int i = 0, count = std::max(1, (data >> 8) & 0x3f); i < count; i++)
				m_lcd_callback(BIT(m_fifo[i], 8), m_fifo[i] & 0xff);
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
			if (BIT(data, reason) && m_irq_waiting[reason])
			{
				const int voice = std::countr_zero(m_irq_waiting[reason]);
				m_irq_waiting[reason] &= ~(u64(1) << voice);
				m_irq_voice[reason] = voice;
				m_irq_pending |= 1 << reason;
			}
		present_switch();
		update_irq();
		break;

	default:
		if (word >= OBJECT_BASE && word < OBJECT_END)
			object_w(object(), word, data);
		else if (word < 0x60)
			LOGMASKED(LOG_REGS, "%s: register %02x = %04x\n", machine().describe_context(), word, data);
		else
			LOGMASKED(LOG_GENERAL, "%s: unknown register %02x = %04x\n", machine().describe_context(), word, data);
		break;
	}
}


// the longs from word 0x70 to 0xbf and from 0xe0 to 0xfb are pairs: the even
// word only stages the latch every even word shares, and the odd word commits
// the two, the high word cut to its width
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
		m_voices[voice].filter_band = wrap24(object_long(voice, FILTER_BAND));
		break;

	case FILTER_LOW + 1:
		m_voices[voice].filter_low = wrap24(object_long(voice, FILTER_LOW));
		break;

	case PAIR_SMOOTH + 1:
		m_voices[voice].pair_smooth = wrap24(object_long(voice, PAIR_SMOOTH));
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
//  the run mask: word 0x0d bit 0 is voice 0, word 0x0a bit 15 voice 63.
//  A cleared bit stops its voice as it is written; a set bit waits for a
//  write of zero to word 0x0e, which takes all four words, and which reads
//  back with its busy bit clear.
//-------------------------------------------------

void roland_xv_device::run_mask_w(int word, u16 data)
{
	if (word == RUN_COMMIT)
	{
		m_regs[RUN_COMMIT] = data & 0x7f;
		if (!data)
		{
			m_run_mask = 0;
			for (int w = 0; w < 4; w++)
				m_run_mask |= u64(m_regs[RUN_MASK + 3 - w]) << (16 * w);
		}
		LOGMASKED(LOG_REGS, "%s: run mask sync %04x\n", machine().describe_context(), data);
		return;
	}
	m_run_mask &= ~(u64(u16(~data)) << (16 * (RUN_MASK + 3 - word)));
	LOGMASKED(LOG_REGS, "%s: run mask word %02x = %04x\n", machine().describe_context(), word, data);
}


//-------------------------------------------------
//  the spaces behind word 0x06.  The DSP's cells are 24 bits: the ring
//  and the static cells by their own address, the mix cells, transport
//  slots and staging cells in the work bank the last sample ran, and the parameter bank
//  with its word in both halves.  The counter steps a program row's three
//  words and then the next row's first: the fourth slot, the runtime
//  operand, is reached only by its own address, and whichever of the
//  third and fourth was written last is the operand the row runs with and
//  both read back.  Rows past the mapped ones take nothing and read the
//  last mapped row's operand.  Every write lands at once; the wide records
//  and the I/O table are decoded again at the next sample.
//-------------------------------------------------

u16 roland_xv_device::next_address(u16 address)
{
	if ((address & 0xf003) == (SPACE_PRAM | 2))
		return address + 2;
	return address + 1;
}

u32 roland_xv_device::space_r(u16 address) const
{
	if (address < IBUS_IRAM_END)
		return m_iram[address] & 0xffffff;
	if (address < IBUS_BANK)
		return u32(m_bus[address - IBUS_MIX]) & 0xffffff;
	if (address < IBUS_BANK_END)
	{
		const u16 word = u16(m_bank_next[address - IBUS_BANK] >> 8);
		return (u32(word) << 16) | word;
	}
	if ((address & 0xf000) == SPACE_PRAM)
	{
		const int n = (address >> 2) & (DSP_ROWS - 1);
		if (n >= DSP_ROWS_MAPPED)
			return m_rows[DSP_ROWS_MAPPED - 1].operand;
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
		m_iram[address] = data & 0xffffff;
	else if (address < IBUS_BANK)
		m_bus[address - IBUS_MIX] = wrap24(s32(data));
	else if (address < IBUS_BANK_END)
	{
		if (!BIT(m_regs[MODE], 6))
			m_bank[address - IBUS_BANK] = m_bank_next[address - IBUS_BANK] = s32(s16(data)) * 256;
	}
	else if (address >= SPACE_CACHE && address < SPACE_CACHE_END)
		m_cache[address & (OBJECTS - 1)][(address - SPACE_CACHE) / OBJECTS] = data & 0xfffff;
	else switch (address & 0xf000)
	{
	case SPACE_PRAM:
	{
		const int n = (address >> 2) & (DSP_ROWS - 1);
		if (n >= DSP_ROWS_MAPPED)
			break;
		m_space[address] = data & 0xffff;
		switch (address & 3)
		{
		case 0: m_rows[n].w0 = data; break;
		case 1: m_rows[n].w1 = data; break;
		default: m_rows[n].operand = data; break;
		}
		row_changed(n);
		break;
	}

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
//  the FIFO and the transfer engine.  Word 0x09 rewinds both pointers and
//  keeps the contents: the host writes it before pushing a write's data
//  and again before pulling a read's.  A read request fills the FIFO from
//  the wave space and a write request empties it there; both finish at
//  once, so the status word never shows either flag.
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
//  interrupts: a pending reason is raised while its enable is set, and
//  acknowledged by writing its bit back
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
//  the panel scanner: eight strobes against eight switch lines and eight
//  LED lines.  The switches are nibbles of the words the host reaches
//  through the FIFO after selecting 0x240+n in word 0x09, two words a
//  strobe: bit 0 is the level, high while the switch is open, bits 1 and
//  2 are set by a press and a release and cleared by the host writing the
//  word back.  A press raises reason 14 with the switch's number, strobe
//  times eight plus line, in word 0x1c, one switch per interrupt; a
//  release raises nothing.  The
//  LEDs are the words behind 0x260+n, again two a strobe, a brightness
//  nibble per line.
//-------------------------------------------------

TIMER_CALLBACK_MEMBER(roland_xv_device::scan_switches)
{
	const u64 now = m_switch_callback();
	const u64 changed = now ^ m_switch_state;
	if (!changed)
		return;
	for (int n = 0; n < 64; n++)
		if (BIT(changed, n))
		{
			const int shift = (n & 3) * 4;
			u16 &w = m_scan[n >> 2];
			w = (w & ~(1 << shift)) | ((BIT(now, n) ^ 1) << shift) | ((BIT(now, n) ? 2 : 4) << shift);
			if (BIT(now, n))
				m_switch_changed |= u64(1) << n;
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
//  ramps: a control word and a target, committed by the target's write,
//  walk a current once a sample by a Q8 slope the ramp caches; the parameter
//  bank's sixty-four share them with the voices' cutoff, resonance,
//  amplitude, pitch and two send ports
//-------------------------------------------------

u16 roland_xv_device::ramp(u16 current, u16 target, u16 &control, s32 &slope, int use, bool event_free, u8 threshold) const
{
	if ((control & RAMP_HOLD) == RAMP_HOLD)
		return current;
	const bool curve = control & RAMP_CURVE;
	if (!curve && !(control & RAMP_REUSE))
	{
		const s32 product = (s32(target) - s32(current)) * ramp_rate[control & 15];
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
	s32 next = s32(current) + (slope >> 8) + ((slope & 0xff) > threshold);
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
		slope = std::max(-0x80000, slope - ramp_acceleration[control & 15]);
	return u16(std::clamp(next, 0, 0xffff));
}

void roland_xv_device::service_coefficient(int n, int i)
{
	const int command = CUTOFF_RAMP + 2 * i;
	u16 control = object_word(n, command);
	if ((control & RAMP_HOLD) == RAMP_HOLD)
		return;
	const int reason = IRQ_CUTOFF_LANDED + i;
	s32 slope = sext(object_long(n, CUTOFF_SLOPE + 2 * i), 20);
	object_ref(n, CUTOFF + i) = ramp(object_word(n, CUTOFF + i), object_word(n, command + 1), control, slope, USE_COEFFICIENT, !BIT(m_irq_pending, reason), m_threshold);
	object_ref(n, command) = control;
	set_long(n, CUTOFF_SLOPE + 2 * i, slope & 0xfffff);
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
	s32 slope = sext(object_long(n, SEND_SLOPE_A + 2 * i), 20);
	object_ref(n, level) = ramp(object_word(n, level), object_word(n, command + 1), control, slope, USE_SEND, !BIT(m_irq_pending, reason), m_threshold);
	object_ref(n, command) = control;
	set_long(n, SEND_SLOPE_A + 2 * i, slope & 0xfffff);
	if ((control & (RAMP_HOLD | RAMP_EVENT)) == (RAMP_HOLD | RAMP_EVENT))
		raise_irq(reason, n);
}

void roland_xv_device::service_pitch(int n)
{
	u16 control = object_word(n, PITCH_RAMP);
	if ((control & (PITCH_REUSE | PITCH_HOLD)) == (PITCH_REUSE | PITCH_HOLD))
		return;
	const s32 current = object_long(n, PITCH_STEP) & 0x3fffff;
	const s32 target = ((control & 7) << 16) | object_word(n, PITCH_RAMP + 1);
	s32 slope = sext(object_long(n, PITCH_SLOPE), 22);
	if (!(control & PITCH_REUSE))
	{
		const s32 product = (target - current) * ramp_rate[(control >> 3) & 15];
		slope = (product >> 7) | ((product & 0x7f) ? 1 : 0);
		control |= PITCH_REUSE;
		set_long(n, PITCH_SLOPE, slope & 0x3fffff);
	}
	if (!(control & PITCH_HOLD))
	{
		s32 next = current + (slope >> 8) + ((slope & 0xff) > m_threshold);
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
		set_long(n, PITCH_STEP, std::clamp(next, 0, 0x3fffff));
	}
	object_ref(n, PITCH_RAMP) = control;
}

// the parameter bank: a row reads a parameter's value from before this sample's
// step until the slot the parameter's index names, and the stepped one from it
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
		s32 slope = sext(object_long(p, PARAMETER_SLOPE), 20);
		const u16 next = ramp(u16(m_bank[p] >> 8), object_word(p, BLOCK_CONTROL + 1), control, slope, USE_PARAMETER, false, m_threshold);
		m_bank_next[p] = s32(s16(next)) * 256;
		object_ref(p, BLOCK_CONTROL) = control;
		set_long(p, PARAMETER_SLOPE, slope & 0xfffff);
	}
}

// word 0x02 bit 6, and reset: every parameter zero and settled
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
//  the wave reader: word 0x60 bits 13:12 pick the XP's 8-bit format (1),
//  two samples a cell with the low byte first and each 1 Mi-sample page's
//  first 32 Ki its exponent-nibble table, or a 16-bit one (0), a signed
//  cell a sample, which the accumulator takes rather than sums.  Bits 27:25
//  of the address are the bank: the internal set, the SR-JV80 slots (a
//  byte-wide device, one sample a cell), the four SRX slots and the two
//  SIMMs; the loop points keep only their 20-bit index.  Word 0x60 bit 7 gives the loop points a
//  sub-sample fraction each, the two bytes of word 0x74/75, which the
//  forward loop takes.  Bits 11:10 are the loop mode; the end (word 0x84)
//  is where a loop rewinds or a ping-pong turns in either direction, the
//  loop start (word 0x82) the rewind target and the other turn, and a
//  voice with no loop holds at its end
//-------------------------------------------------

u32 roland_xv_device::cell_of(u32 sample, bool wide)
{
	const int bank = (sample >> 25) & 7;
	if (wide || bank == BANK_BYTE_WIDE)
		return sample & ADDRESS_MASK;
	return (sample & ADDRESS_MASK & ~0x1ffffff) | ((sample & ADDRESS_MASK) >> 1 & 0x1ffffff);
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

// the exponent word a sample's group reads, as the voice shows it at word 0x63
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

// the start address written is where the fetch begins; the phase and the
// decoder's accumulator start from zero, the cache's producer where word 0x61 puts it
void roland_xv_device::launch(int n)
{
	voice &v = m_voices[n];
	v.fetching = true;
	v.backward = BIT(object_word(n, VOICE_CONTROL2), 5);
	v.region = REGION_BEFORE;
	v.scaled = false;
	v.wrap_cells = 0;
	set_long(n, PHASE, 0);
	object_ref(n, VOICE_CONTROL2) &= 0x1fff;
	object_ref(n, ACCUMULATOR) = 0;
}

u32 roland_xv_device::loop_fraction(int n, bool at_loop) const
{
	if (!BIT(object_word(n, VOICE_CONTROL), 7))
		return 0;
	const u16 both = object_word(n, LOOP_FRACTION + 1);
	return u32(at_loop ? (both >> 8) : (both & 0xff)) << 8;
}

roland_xv_device::address_step roland_xv_device::advance(int n, address_step s, u32 phase) const
{
	const u32 loop = object_long(n, LOOP_START) & PAGE_MASK;
	const u32 end = object_long(n, END) & PAGE_MASK;
	const u32 index = s.address & PAGE_MASK;
	const int mode = (object_word(n, VOICE_CONTROL) >> 10) & 3;

	if (!s.backward)
	{
		const bool past_end = index > end || (index == end && phase >= loop_fraction(n, false));
		if (!past_end)
			return { in_page(s.address, index + 1), false, false };
		switch (mode)
		{
		case LOOP_FORWARD: return { in_page(s.address, loop), false, true };
		case LOOP_ALTERNATE: return { s.address, true, false };
		default: return { s.address, false, false };
		}
	}

	if (mode == LOOP_ALTERNATE)
		return index <= loop ? address_step{ s.address, false, false } : address_step{ in_page(s.address, index - 1), true, false };
	if (index > end)
		return { in_page(s.address, index - 1), true, false };
	return mode == LOOP_FORWARD ? address_step{ in_page(s.address, loop), true, false } : address_step{ s.address, true, false };
}

// the region a fetched sample lies in, and what word 0x60 makes of the crossing: bit 6
// picks the crossing that adopts the second pitch scale, bits 9:8 the one that finishes
void roland_xv_device::cross(int n, u32 address)
{
	voice &v = m_voices[n];
	const u32 index = address & PAGE_MASK;
	u8 region;
	if (index == (object_long(n, END) & PAGE_MASK))
		region = REGION_END;
	else if (index == (object_long(n, LOOP_START) & PAGE_MASK))
		region = REGION_LOOP;
	else
		return;
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
//  the cache: the decoder runs twelve samples ahead of the phase into
//  sixteen cells, fetching to the next group of four, its producer in word
//  0x61 bits 12:8 and its accumulator, nineteen bits, in bits 15:13 and
//  word 0x62; the phase, word 0x70/71, keeps sixteen fraction bits and the
//  consumer's five.  A voice past the end of a one-shot holds its last sample
//-------------------------------------------------

void roland_xv_device::fill(int n, int consumer)
{
	voice &v = m_voices[n];
	const u16 control = object_word(n, VOICE_CONTROL);
	const bool wide = !BIT(control, 12);
	const int mode = (control >> 10) & 3;
	const bool fractions = BIT(control, 7);
	u16 &flags = object_ref(n, VOICE_CONTROL2);
	int producer = (flags >> 8) & 31;
	if (((producer - consumer - 1) & 31) >= CACHE_AHEAD)
		return;
	u32 acc = ((flags & 0xe000) << 3) | object_word(n, ACCUMULATOR);
	address_step s{ object_long(n, START) & ADDRESS_MASK, v.backward, false };
	while (((producer - consumer - 1) & 31) < CACHE_AHEAD)
		for (int count = 4 - (s.address & 3); count; count--)
		{
			bool wrapped = false;
			if (v.fetching)
			{
				const wave_cell c = cell_at(s.address, wide);
				acc = (wide ? u32(c.mantissa) : acc + delta_of(c)) & 0x7ffff;
				if (!wide)
					object_ref(n, EXPONENTS) = exponent_word(s.address);
				cross(n, s.address);
				if (mode == LOOP_NONE && v.region == REGION_END)
					v.fetching = false;
				else
				{
					const address_step next = advance(n, s, 0xffff);
					wrapped = next.wrapped && fractions;
					s = next;
				}
			}
			const int cell = producer & (CACHE_CELLS - 1);
			m_cache[n][cell] = acc;
			v.wrap_cells = (v.wrap_cells & ~(1 << cell)) | (wrapped << cell);
			producer = (producer + 1) & 31;
		}
	flags = (flags & 0x00ff) | (producer << 8) | ((acc >> 3) & 0xe000);
	object_ref(n, ACCUMULATOR) = acc & 0xffff;
	set_long(n, START, s.address);
	v.backward = s.backward;
}

// the cache cell at the consumer and the three after it under the XP's
// interpolation weights, then the gain word 0x60 bits 3:0 give, 6 dB an even
// step from a half at code 0
s32 roland_xv_device::interpolate(int n, u32 phase, int gain) const
{
	const int consumer = phase >> 16;
	const int fraction = (phase >> 9) & 0x7f;
	s32 previous = wrap19(m_cache[n][consumer & (CACHE_CELLS - 1)]);
	s64 sum = s64(previous) * 4096;
	for (int i = 0; i < 3; i++)
	{
		const s32 next = wrap19(m_cache[n][(consumer + i + 1) & (CACHE_CELLS - 1)]);
		sum += s64(wrap19(next - previous)) * interp_weights[i][fraction];
		previous = next;
	}
	if (BIT(gain, 0))
		sum = sum * 181 / 128;
	return clamp24(sum >> (13 - (gain >> 1)));
}

// the sample at the phase; a voice of format 2 or 3 fetches nothing and
// sources nothing, whatever its cache holds.  A forward loop's fractions move
// the phase as it reaches the loop's last sample
s32 roland_xv_device::source(int n, int gain)
{
	voice &v = m_voices[n];
	u32 phase = object_long(n, PHASE) & PHASE_MASK;
	const int consumer = phase >> 16;
	s32 sample = 0;
	if (!BIT(object_word(n, VOICE_CONTROL), 13))
	{
		fill(n, consumer);
		sample = interpolate(n, phase, gain);
	}

	u32 step = object_long(n, PITCH_STEP) & 0x3fffff;
	if (v.scaled)
		step = (u64(step) * object_word(n, WAVE_SCALE + 1)) >> 15;
	s64 next = s64(phase) + step;
	for (s64 cell = consumer + 1; cell <= (next >> 16); cell++)
		if (BIT(v.wrap_cells, cell & (CACHE_CELLS - 1)))
		{
			v.wrap_cells &= ~(1 << (cell & (CACHE_CELLS - 1)));
			next += s32(loop_fraction(n, true)) - s32(loop_fraction(n, false));
		}
	set_long(n, PHASE, u32(next) & PHASE_MASK);
	return sample;
}


//-------------------------------------------------
//  the filter: the XP's state-variable structure, the cutoff and resonance
//  coefficients Q15 words, the type the output tap; types 4-7 add to the
//  input, OFF being type 7 with both coefficients at zero.  The two
//  integrators and the output read back at words 0xb0-0xb5
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
		const s32 damping = s32((s64(q) * band) >> 15);
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
		default: out = clamp24(sample + ((q * (s64(sample) - low - v.filter_band)) >> 14)); break;
		}
	}
	set_long(n, FILTER_BAND, u32(v.filter_band) & 0xffffff);
	set_long(n, FILTER_LOW, u32(v.filter_low) & 0xffffff);
	set_long(n, FILTER_OUTPUT, u32(out) & 0xffffff);
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

// the amplitude's product floors, and reads back at word 0xe4/e5
s32 roland_xv_device::amplitude(int n, s32 sample)
{
	const s32 out = clamp24((s64(sample) * object_word(n, AMPLITUDE)) >> 15);
	set_long(n, VOICE_OUTPUT, u32(out) & 0xffffff);
	return out;
}

// six sends, each a mix cell and an unsigned Q15 level whose product floors; a send
// claims its cell even at level zero, and one that follows a send to the same cell
// adds to the sum from before that send's deposit
void roland_xv_device::emit(int n, s32 output, s64 *mix, u64 &claimed) const
{
	int previous = -1;
	s64 base = 0;
	for (int slot = 0; slot < SENDS; slot++)
	{
		const int bus = object_word(n, SEND_BASE + slot * 2) & (BUSES - 1);
		const s64 level = object_word(n, SEND_BASE + slot * 2 + 1);
		const s64 old = mix[bus];
		mix[bus] = (bus == previous ? base : old) + ((output * level) >> 15);
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
//  paired structures: a running master whose word 0xc4 bits 11:8 name a
//  case takes the next voice's wave into its own path and the pair leaves
//  through the master's sends.  The partner's filter type is the master's
//  bits 7:4, the booster's gain the master's word 0xc3 and its clip level
//  the master's bits 15:12; the partner's word 0xc3 is the rate of a
//  one-pole the combined signal leaves as the difference from, whose state
//  is the partner's word 0xb4/b5.  The booster multiplies by word 0xc3
//  over 0x100 before its clip, sixteen times the level the firmware takes
//  back off the partner's amplitude on those two cases, and a ring
//  product of two voice words is shifted down by fifteen bits.
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
	// the pair stage is fitted to Roland's model, which the firmware's
	// gain code 0x8 on a pair's voices would put 24 dB over; silicon's
	// pairs are unmeasured, so the voices come in without it
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
//  the DSP.  A row is W0 (the ALU: left and right operands, their signs,
//  the destination, a wrap, a second memory operation in W2; the
//  multiplier's operand and whether it takes the C latch), W1 (a memory
//  operation with its ten-bit address, the coefficient scale, the
//  multiply request) and W2 (the coefficient, an immediate, or the second
//  memory operation).  W0 bit 15 makes W1's low bits a condition and a
//  predicate mode: a predicate on the row's fields, a branch with one
//  delay slot, or an absolute jump.  Every operand a row uses is the state it was
//  entered with.  The accumulators are 24.4 with a guard bit, and a row
//  sees the flags of the result two rows back.
//-------------------------------------------------

// a row's fields are decoded as its words land
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
		unsigned coefficient_mode = ((bus >> 13) & 7) | ((control & 1) << 3);
		unsigned source = (control >> 1) & 7;
		if (f && predicated)
		{
			const unsigned alternate = r.argument & 0xf;
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
		while (m_rows_end >= 0 && !m_rows[m_rows_end].w0 && !m_rows[m_rows_end].w1)
			m_rows_end--;
}

// the transfers keep their order within a slot
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

// the records' service slots: a header takes one, a transfer two and the
// first after a header three, though a zero record overlaps that third;
// a header points both cell pointers into the staging cells, and each
// steps on through them, a read's cell taking the word four slots later
void roland_xv_device::decode_transfers()
{
	m_transfer_count = 0;
	m_transfers_stale = false;
	bool open = false, first = false;
	u16 read = 0, write = 0;
	unsigned fetch = 0, ready = 0;
	for (int n = 0; n < RECORDS; n++)
	{
		const u32 word = m_space[SPACE_RECORDS + n] & 0xfffff;
		if ((word & 0xfc000) == 0xfc000)
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
				m_transfers[m_transfer_count++] = transfer{ slot, XFER_WRITE, write, word & 0x7ffff };
				write = IBUS_STAGING | ((write + 1) & 0x7f);
			}
			else
			{
				m_transfers[m_transfer_count++] = transfer{ slot, XFER_READ, u16(n), word & 0x7ffff };
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
		return wrap24(s32(m_iram[ring_index(address)]));
	if (address < IBUS_MIX)
		return wrap24(s32(m_iram[address]));
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

// the mix cells, transport slots and staging cells: the sample takes the work bank the one before last ran on
void roland_xv_device::swap_work()
{
	m_dsp->work_phase ^= 1;
	m_bus = m_work + (m_dsp->work_phase ? WORK_CELLS : 0);
	m_bus_other = m_work + (m_dsp->work_phase ? 0 : WORK_CELLS);
}

// a word coming back into a work cell; of two for one cell, the one asked for later wins,
// and one that comes back past the end of the sample lands in the next sample's bank
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

// the records and the last sample's taps through the ERAM port in slot
// order, the records first in a slot: a tap fetches both samples at its
// row's slot and returns the fraction, the first and the second sample 4,
// 5 and 7 slots on; a sample fetched past the end of its own sample is
// one cell later, and a return that lands on the last slot is zero
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
				value = component == 2 ? (m_tap_delay[t.cell] & 15) * 0x80000 : m_tap_fetched[t.cell][component];
			return_w(m_tap_cell[t.cell] + component, value, 2 * m_tap_slot[t.cell] + 1, t.slot >= DSP_ROW_BUDGET);
			break;
		}
		}
	}
	s.tap_count = 0;
}

// the sample: the other work bank, the voices' mix onto its mix cells, the
// transfers, then the rows from 0 until the budget is spent or the fetch
// leaves the mapped rows, then the cursor down
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

// the rows from pc on, pending the target of the branch whose delay slot pc is
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
	case 0x6: return f >= 0x8000000 || f < -0x8000000;
	case 0x7: return f < 0x8000000 && f >= -0x8000000;
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
			if (address && s.tap_count < TAPS)
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
		constexpr u64 mask = 0x1fffffff;
		const u64 carry_left = (u64(left) ^ (f.negate_left ? mask : 0)) & mask;
		const u64 carry_right = (u64(right) ^ (f.negate_right ? mask : 0)) & mask;
		carry = carry_left + carry_right + f.negate_left + f.negate_right > mask;
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
			const unsigned count = (row.argument & 0xf) ? (row.argument & 0xf) : unsigned((f.negate_right ? ~right : right) >> 4) & 0xf;
			result = special * (s64(1) << count);
			overflow = result != clamp28(result);
			carry = ((u64(special) & 0xfffffff) << count) > 0xfffffff;
			result = row.wrap ? wrap28(result) : clamp28(result);
		}
		else
		{
			const s32 l = s32(special >> 4), rv = s32(right >> 4);
			const s32 rs = f.negate_right ? ~rv : rv;
			const s64 difference = (f.negate_left ? -left : left) - right;
			const s64 comparison = row.wrap ? difference : wrap28(difference);
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
				v = (shifted & ~1) | ((input ^ (input >> 1) ^ (input >> 6) ^ (input >> 23)) & 1) | (special == 0);
				break;
			}
			case SPECIAL_NORMALIZE:
			{
				const u32 magnitude = u32(l < 0 ? ~l : l) & 0x7fffff;
				v = 0;
				while (v < 15 && !(magnitude & (0x400000 >> v)))
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
	result = wrap29(result);
	const s32 raw = s32(overflow ? wrap29(result ^ (s64(1) << 28)) : result);
	if (f.clamp && !row.wrap)
		result = clamp28(result);
	if (commit || !row.gated)
		s.acc[row.to_b ? 1 : 0] = s32(row.wrap ? wrap28(result) : result);
	if (!s.last_hold)
	{
		s.flag = s.last;
		s.flag_carry = s.last_carry;
	}
	s.last = raw;
	s.last_carry = carry;
	s.last_hold = row.gated && !commit && row.condition;
}
