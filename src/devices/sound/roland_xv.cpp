// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Roland XV tone generator (TC223C660CF-503, RA08-503)

    The XP's successor: 64 voices a chip, two chips in an XV-5080.  The
    host sees a 16-bit register window: the address spaces reached through
    it (the DSP program rows, the scalars, the wide records and the
    per-voice longs), the FIFO transfer engine onto the chip's own wave and
    sample memory, the object-indexed voice file and the interrupt path.

    A voice reads the XP's 8-bit sample format, or a 16-bit differential
    one, at a 28-bit address - a memory bank in bits 27:25, a 1 Mi-sample
    page in it and a 20-bit index the loop points share - and steps it by
    a linear 18-bit pitch (0x10000 = one sample an output sample),
    scaled by word 0x76/77 from its first loop crossing on, runs its
    cutoff, resonance, amplitude and pitch through host-targeted ramps
    with a 4-bit rate code each, filters, scales by a Q15 amplitude and adds
    itself to up to six of sixteen buses at six send levels.

    The buses land in the effect DSP's mix cells, one a bus, and the DSP
    runs its program once an output sample: up to 768 three-word rows with
    a multiplier, two accumulators, four read latches and a conditional
    family with one delay slot, over a 1024-cell ring that slides one cell
    a sample (IRAM), the mix, transport and ERAM staging cells that do not
    slide, and a bank of sixty-four coefficients the host writes through
    the object path.  Wide records at
    0x3000 move cells to and from a 2^19-cell external memory behind a
    cursor that falls once a sample, a mode-0 row with an address requests
    a fractional tap from it, and the four DAC pairs are the transport
    slots 0x2cc-0x2cf and 0x2dc-0x2df.  Two chips share a transport block:
    the one given set_link() runs the other from its own stream and moves
    seventeen words each way between samples.

    The arithmetic is a provisional policy: 32-bit accumulators and
    product with 23 fraction bits, every store, transfer, immediate add
    and magnitude clamped to 24 bits, products truncated toward zero.
    Whether the chip is wider or narrower, and where it clamps, is unread.

    TODO:
    - the ramp's granularity, and whether its table is samples or a divider
    - the filter's state width, rounding and saturation; the BPF and PKG taps
    - the DSP's widths, its clamp sites, the ERAM's service order, condition
      codes 4/5/e/f and multiplier selector e, which are unobserved
    - the 16-bit format's accumulator width, and the bank widths the
      configuration words 0x12-0x19 presumably carry
    - paired structures (word 0xc4 bits 11:8, word 0xc3)
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

// the samples a ramp takes to land, by rate code: 2 ms a code to 20 ms, then 4 ms
// a code to 40 ms, and 48 ms for 0xf
const u16 ramp_samples[16] = {
	88, 176, 265, 353, 441, 529, 617, 706, 794, 882, 1058, 1235, 1411, 1588, 1764, 2117 };

// what a muted ramp's slope loses each sample, by rate code, in the ramp's
// fraction: 3.5 Q15 steps a sample at code 0, halving every four codes
const s32 mute_fade[16] = {
	14336, 12288, 10240, 8192, 7168, 6144, 5120, 4096, 3584, 3072, 2560, 2048, 1792, 1536, 1280, 1024 };

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
	, m_master(nullptr)
{
}

device_memory_interface::space_config_vector roland_xv_device::memory_space_config() const
{
	return space_config_vector { std::make_pair(AS_WAVE, &m_wave_config) };
}

void roland_xv_device::device_start()
{
	space(AS_WAVE).specific(m_wave);
	m_stream = stream_alloc(0, OUTPUTS, SAMPLE_RATE);
	m_space = std::make_unique<u32[]>(0x10000);
	m_eram = std::make_unique<s32[]>(ERAM_CELLS);
	if (m_link)
		m_link->m_master = this;

	save_item(NAME(m_regs));
	save_item(NAME(m_object_regs));
	save_pointer(NAME(m_space), 0x10000);
	save_item(NAME(m_address));
	save_item(NAME(m_data_high));
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
	save_item(NAME(m_int_state));
	save_item(NAME(m_run_mask));
	save_item(STRUCT_MEMBER(m_voices, address));
	save_item(STRUCT_MEMBER(m_voices, phase));
	save_item(STRUCT_MEMBER(m_voices, predictor));
	save_item(STRUCT_MEMBER(m_voices, backward));
	save_item(STRUCT_MEMBER(m_voices, launch));
	save_item(STRUCT_MEMBER(m_voices, fetching));
	save_item(STRUCT_MEMBER(m_voices, was_running));
	save_item(STRUCT_MEMBER(m_voices, region));
	save_item(STRUCT_MEMBER(m_voices, finished));
	save_item(STRUCT_MEMBER(m_voices, scaled));
	save_item(STRUCT_MEMBER(m_voices, filter_low));
	save_item(STRUCT_MEMBER(m_voices, filter_band));
	save_item(STRUCT_MEMBER(m_voices, ramp_current));
	save_item(STRUCT_MEMBER(m_voices, ramp_target));
	save_item(STRUCT_MEMBER(m_voices, ramp_position));
	save_item(STRUCT_MEMBER(m_voices, ramp_step));
	save_item(STRUCT_MEMBER(m_voices, ramp_remaining));
	save_item(STRUCT_MEMBER(m_voices, ramp_fade));
	save_item(STRUCT_MEMBER(m_voices, ramp_armed));
	save_item(STRUCT_MEMBER(m_voices, send_slot));
	save_item(STRUCT_MEMBER(m_rows, w0));
	save_item(STRUCT_MEMBER(m_rows, w1));
	save_item(STRUCT_MEMBER(m_rows, operand));
	save_pointer(NAME(m_eram), ERAM_CELLS);
	save_item(NAME(m_bus));
	save_item(NAME(m_bank));
	save_item(NAME(m_cursor));
	save_item(NAME(m_acc));
	save_item(NAME(m_product));
	save_item(NAME(m_latch));
	save_item(NAME(m_flag_value));
	save_item(NAME(m_flag_raw));
	save_item(NAME(m_last_value));
	save_item(NAME(m_last_raw));
	save_item(NAME(m_last_hold));
	save_item(STRUCT_MEMBER(m_taps, cell));
	save_item(STRUCT_MEMBER(m_taps, delay));
	save_item(NAME(m_tap_count));
}

void roland_xv_device::device_post_load()
{
	m_rows_end = -1;
	for (int n = 0; n < DSP_ROWS; n++)
		decode_row(n);
	m_transfers_stale = true;
}

void roland_xv_device::device_reset()
{
	std::fill(std::begin(m_regs), std::end(m_regs), 0);
	std::fill(&m_object_regs[0][0], &m_object_regs[0][0] + OBJECTS * (OBJECT_END - OBJECT_BASE), 0);
	std::fill_n(m_space.get(), 0x10000, 0);
	m_address = 0;
	m_data_high = 0;
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
	m_run_mask = 0;
	for (auto &v : m_voices)
		v = voice();
	for (auto &r : m_rows)
		r = dsp_row();
	m_rows_end = -1;
	std::fill_n(m_eram.get(), ERAM_CELLS, 0);
	std::fill(std::begin(m_bus), std::end(m_bus), 0);
	std::fill(std::begin(m_bank), std::end(m_bank), 0);
	m_cursor = 0;
	m_acc[0] = m_acc[1] = 0;
	m_product = 0;
	std::fill(std::begin(m_latch), std::end(m_latch), 0);
	m_flag_value = 0;
	m_flag_raw = 0;
	m_last_value = 0;
	m_last_raw = 0;
	m_last_hold = true;
	m_transfer_count = 0;
	m_transfers_stale = true;
	m_tap_count = 0;
	m_logged = 0;
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
			if (m_link)
				exchange();
		}
		for (int pair = 0; pair < DAC_PAIRS; pair++)
		{
			stream.put_int_clamp(2 * pair, i, m_master ? 0 : m_bus[DAC_L + pair - IBUS_MIX], 1 << DSP_FRACTION_BITS);
			stream.put_int_clamp(2 * pair + 1, i, m_master ? 0 : m_bus[DAC_R + pair - IBUS_MIX], 1 << DSP_FRACTION_BITS);
		}
	}
}

void roland_xv_device::sync()
{
	if (m_master)
		m_master->m_stream->update();
	else
		m_stream->update();
}

// one output sample: the voices onto the mix cells with the four guard bits the
// program expects, then the DSP over them
void roland_xv_device::frame()
{
	s32 buses[BUSES] = { 0 };
	for (int n = 0; n < OBJECTS; n++)
		run_voice(n, buses);
	for (int bus = 0; bus < BUSES; bus++)
		m_bus[bus] = clamp24(s64(buses[bus]) * 2);
	run_dsp();
}

// the seventeen words each way, one sample late
void roland_xv_device::exchange()
{
	for (int n = 0; n < LINK_WORDS_L; n++)
	{
		m_link->m_bus[LINK_OUT_L + n - IBUS_MIX] = m_bus[LINK_OUT_L + n - IBUS_MIX];
		m_bus[LINK_IN_L + n - IBUS_MIX] = m_link->m_bus[LINK_IN_L + n - IBUS_MIX];
	}
	for (int n = 0; n < LINK_WORDS_R; n++)
	{
		m_link->m_bus[LINK_OUT_R + n - IBUS_MIX] = m_bus[LINK_OUT_R + n - IBUS_MIX];
		m_bus[LINK_IN_R + n - IBUS_MIX] = m_link->m_bus[LINK_IN_R + n - IBUS_MIX];
	}
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
		word_w(word, (m_regs[word] & 0xff00) | data);
}

u16 roland_xv_device::word_peek(int word)
{
	switch (word)
	{
	case DATA_HIGH:
		return m_space[m_address] >> 16;

	case DATA_LOW:
		return m_space[m_address] & 0xffff;

	case FIFO:
		if (m_scan_select >= 0)
			return m_scan[(m_scan_select + m_scan_read) & 7];
		if (m_led_select >= 0)
			return m_led[(m_led_select + m_scan_read) & 15];
		return m_fifo[m_fifo_read];

	case IRQ_MASK:
		return m_irq_pending;

	case 0x24:      // the SD-90's boot loader wants to read bit 13 here
		return 0xffff;

	case SWITCH_INDEX:
		return m_switch_index;

	case STATUS:
		return 0;

	default:
		if (word >= IRQ_VOICE && word < IRQ_VOICE + IRQ_REASONS)
			return m_irq_voice[word - IRQ_VOICE];
		if (word == CUTOFF || word == RESONANCE || word == AMPLITUDE)
			return m_voices[object()].ramp_current[word == CUTOFF ? RAMP_CUTOFF : word == RESONANCE ? RAMP_RESONANCE : RAMP_AMPLITUDE];
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
		LOGMASKED(LOG_SPACE, "%s: read %04x = %08x\n", machine().describe_context(), m_address, m_space[m_address]);
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
		break;

	case DATA_HIGH:
		m_data_high = data;
		break;

	case DATA_LOW:
		space_w(m_address, (u32(m_data_high) << 16) | data);
		LOGMASKED(LOG_SPACE, "%s: write %04x = %08x\n", machine().describe_context(), m_address, m_space[m_address]);
		m_address = next_address(m_address);
		m_data_high = 0;
		break;

	case ADDRESS:
		m_address = data;
		break;

	case FIFO:
		if (m_scan_select >= 0)
		{
			const int w = (m_scan_select + m_scan_write) & 7;
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
		if ((data & 0xfff8) == 0x0240)
			m_scan_select = data & 7;
		else if ((data & 0xfff0) == 0x0260)
			m_led_select = data & 15;
		else if ((data & 0xfff0) == 0x0200)
			m_fifo_write = m_fifo_read = data & 15;   // 0x200+n selects the word, as 0x240+n and 0x260+n do
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


void roland_xv_device::object_w(int voice, int word, u16 data)
{
	m_object_regs[voice][word - OBJECT_BASE] = data;
	const u32 value = object_long(voice, word & ~1);
	if (word >= SEND_BASE && BIT(word, 0))
		for (int port = 0; port < 2; port++)
			if (m_voices[voice].send_slot[port] == (word - SEND_BASE) >> 1)
				seed_ramp(voice, RAMP_SEND_A + port, data);
	switch (word)
	{
	case START + 1:
		m_voices[voice].launch = true;
		break;

	case PITCH_STEP + 1:
		seed_ramp(voice, RAMP_PITCH, value & 0x3ffff);
		break;

	case FILTER_BAND + 1:
		m_voices[voice].filter_band = wrap24(value);
		break;

	case FILTER_LOW + 1:
		m_voices[voice].filter_low = wrap24(value);
		break;

	case CUTOFF:
		seed_ramp(voice, RAMP_CUTOFF, data);
		break;

	case RESONANCE:
		seed_ramp(voice, RAMP_RESONANCE, data);
		break;

	case AMPLITUDE:
		seed_ramp(voice, RAMP_AMPLITUDE, data);
		break;

	case CUTOFF_RAMP + 1:
		start_ramp(voice, RAMP_CUTOFF, value);
		break;

	case RESONANCE_RAMP + 1:
		start_ramp(voice, RAMP_RESONANCE, value);
		break;

	case AMPLITUDE_RAMP + 1:
		start_ramp(voice, RAMP_AMPLITUDE, value);
		LOGMASKED(LOG_OBJECT, "%s: object %03x amplitude %08x\n", machine().describe_context(), m_regs[MODE], value);
		return;

	case PITCH_RAMP + 1:
		start_ramp(voice, RAMP_PITCH, value);
		break;

	case PITCH_INCREMENT + 1:
		increment_ramp(voice, RAMP_PITCH, value);
		break;

	case CUTOFF_INCREMENT + 1:
		increment_ramp(voice, RAMP_CUTOFF, value);
		break;

	case RESONANCE_INCREMENT + 1:
		increment_ramp(voice, RAMP_RESONANCE, value);
		break;

	case AMPLITUDE_INCREMENT + 1:
		increment_ramp(voice, RAMP_AMPLITUDE, value);
		break;

	case SEND_PORT_A + 1:
		send_port_w(voice, RAMP_SEND_A, value);
		break;

	case SEND_PORT_B + 1:
		send_port_w(voice, RAMP_SEND_B, value);
		break;

	case BLOCK_CONTROL + 1:
		m_bank[voice] = s32(s16(data)) << (DSP_FRACTION_BITS - 15);
		LOGMASKED(LOG_OBJECT, "%s: parameter %02x = %08x\n", machine().describe_context(), voice, value);
		return;
	}
	LOGMASKED(LOG_OBJECT, "%s: object %03x word %02x = %04x\n", machine().describe_context(), m_regs[MODE], word, data);
}


//-------------------------------------------------
//  the run mask: word 0x0d bit 0 is voice 0, word 0x0a bit 15 voice 63.
//  A written bit takes effect at once; a cleared one waits for the
//  commit, a write of word 0x0e, which reads back with its busy bit clear.
//-------------------------------------------------

void roland_xv_device::run_mask_w(int word, u16 data)
{
	u64 written = 0;
	for (int w = 0; w < 4; w++)
		written |= u64(m_regs[RUN_MASK + 3 - w]) << (16 * w);
	if (word == RUN_COMMIT)
	{
		m_run_mask = written;
		m_regs[RUN_COMMIT] = data & 0x7f;
		LOGMASKED(LOG_REGS, "%s: run mask commit %016llx\n", machine().describe_context(), (unsigned long long)m_run_mask);
	}
	else
	{
		m_run_mask |= written;
		LOGMASKED(LOG_REGS, "%s: run mask word %02x = %04x\n", machine().describe_context(), word, data);
	}
}


//-------------------------------------------------
//  the spaces behind word 0x06.  The counter steps a program row's three
//  words and then the next row's first: the fourth slot, the runtime
//  operand, is reached only by its own address, and whichever of the
//  third and fourth was written last is the operand the row runs with.
//  Every write lands at once; the wide records are decoded again at the
//  next sample.
//-------------------------------------------------

u16 roland_xv_device::next_address(u16 address)
{
	if ((address & 0xf003) == (SPACE_PRAM | 2))
		return address + 2;
	return address + 1;
}

void roland_xv_device::space_w(u16 address, u32 data)
{
	m_space[address] = data;
	switch (address & 0xf000)
	{
	case SPACE_PRAM:
	{
		const int n = (address >> 2) & (DSP_ROWS - 1);
		switch (address & 3)
		{
		case 0: m_rows[n].w0 = data; break;
		case 1: m_rows[n].w1 = data; break;
		default: m_rows[n].operand = data; break;
		}
		decode_row(n);
		break;
	}

	case SPACE_RECORDS:
		if (address < SPACE_RECORDS + RECORDS)
			m_transfers_stale = true;
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
//  word back.  A change raises reason 14 with the switch's number, strobe
//  times eight plus line, in word 0x1c, one switch per interrupt.  The
//  LEDs are the words behind 0x260+n, again two a strobe, a brightness
//  nibble per line.  Only four strobes are modelled here.
//-------------------------------------------------

TIMER_CALLBACK_MEMBER(roland_xv_device::scan_switches)
{
	const u32 now = m_switch_callback();
	const u32 changed = now ^ m_switch_state;
	if (!changed)
		return;
	for (int n = 0; n < 32; n++)
		if (BIT(changed, n))
		{
			const int shift = (n & 3) * 4;
			u16 &w = m_scan[n >> 2];
			w = (w & ~(1 << shift)) | ((BIT(now, n) ^ 1) << shift) | ((BIT(now, n) ? 2 : 4) << shift);
			m_switch_changed |= 1 << n;
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
	m_switch_changed &= ~(1 << n);
	m_switch_index = n;
	m_irq_pending |= 1 << IRQ_SWITCH;
}


//-------------------------------------------------
//  ramps: a target long carries a 4-bit rate code (bits 19:16; 22:19 on
//  the pitch) and a landing time of 2 ms a code to 20 ms, 4 ms a code to
//  40 ms, then 48 ms; the pitch's bit 23 and the others' bit 21 arm the
//  arrival interrupt, the pitch's bit 24 and the others' bit 22 keep the
//  slope in flight, and bit 20 on a cutoff, resonance or amplitude long fades
//  the value to zero on a parabola whose deceleration the code sets.  The
//  current registers seed the ramps and read back.  The two send ports are
//  two more ramps of the same shape, each aimed by bits 25:23 at one send
//  slot whose level register is the ramp's current; their bit 20 is a
//  faster fade, and a landing arms reason 6 or 7.
//-------------------------------------------------

void roland_xv_device::set_current(int n, int kind, s32 value)
{
	voice &v = m_voices[n];
	v.ramp_current[kind] = value;
	if (kind >= RAMP_SEND_A)
		m_object_regs[n][SEND_BASE + v.send_slot[kind - RAMP_SEND_A] * 2 + 1 - OBJECT_BASE] = u16(value);
}

void roland_xv_device::send_port_w(int n, int kind, u32 value)
{
	voice &v = m_voices[n];
	const int slot = (value >> 23) & 7;
	if (slot >= SENDS)
		return;
	v.send_slot[kind - RAMP_SEND_A] = slot;
	v.ramp_current[kind] = object_word(n, SEND_BASE + slot * 2 + 1);
	v.ramp_position[kind] = v.ramp_current[kind] << RAMP_FRACTION_BITS;
	start_ramp(n, kind, value);
}

// a seed moves the value a ramp is walking and leaves its slope alone
void roland_xv_device::seed_ramp(int n, int kind, s32 value)
{
	voice &v = m_voices[n];
	v.ramp_current[kind] = value;
	v.ramp_position[kind] = value << RAMP_FRACTION_BITS;
	v.ramp_remaining[kind] = v.ramp_remaining[kind] ? steps_to_target(n, kind) : 0;
}

// the samples the slope a voice is already running on needs to reach its target
u16 roland_xv_device::steps_to_target(int n, int kind) const
{
	const voice &v = m_voices[n];
	const s32 distance = v.ramp_target[kind] - v.ramp_current[kind];
	if (!v.ramp_step[kind] || (distance < 0) != (v.ramp_step[kind] < 0))
		return 0;
	const s64 steps = ((s64(distance) << RAMP_FRACTION_BITS) / v.ramp_step[kind]) + 1;
	return u16(std::clamp<s64>(steps, 1, 0xffff));
}

void roland_xv_device::start_ramp(int n, int kind, u32 value)
{
	voice &v = m_voices[n];
	const bool pitch = kind == RAMP_PITCH;
	const int rate = pitch ? (value >> 19) & 0xf : (value >> 16) & 0xf;
	const bool restart = !BIT(value, pitch ? 24 : 22);
	v.ramp_target[kind] = pitch ? value & 0x3ffff : value & 0xffff;
	v.ramp_armed[kind] = pitch ? BIT(value, 23) : BIT(value, 21);
	v.ramp_fade[kind] = 0;
	if (!pitch && BIT(value, 20))
	{
		v.ramp_fade[kind] = kind >= RAMP_SEND_A ? 1 : mute_fade[rate];
		v.ramp_target[kind] = 0;
		v.ramp_remaining[kind] = 0;
		return;
	}
	if (v.ramp_target[kind] == v.ramp_current[kind])
	{
		v.ramp_position[kind] = v.ramp_current[kind] << RAMP_FRACTION_BITS;
		v.ramp_step[kind] = 0;
		v.ramp_remaining[kind] = 1;
		return;
	}
	if (!restart)
	{
		v.ramp_remaining[kind] = v.ramp_remaining[kind] ? steps_to_target(n, kind) : 0;
		return;
	}
	const u16 samples = ramp_samples[rate];
	v.ramp_position[kind] = v.ramp_current[kind] << RAMP_FRACTION_BITS;
	v.ramp_step[kind] = ((v.ramp_target[kind] - v.ramp_current[kind]) << RAMP_FRACTION_BITS) / samples;
	v.ramp_remaining[kind] = samples;
}

// words 0x7c/7d and 0xa0-0xa5 set a ramp's per-sample increment outright, in the
// ramp's own units; a restart takes it back from the rate code
void roland_xv_device::increment_ramp(int n, int kind, u32 value)
{
	voice &v = m_voices[n];
	v.ramp_step[kind] = s32(std::clamp<s64>(s64(s32(value)) << RAMP_FRACTION_BITS,
			std::numeric_limits<s32>::min(), std::numeric_limits<s32>::max()));
	v.ramp_remaining[kind] = steps_to_target(n, kind);
}

void roland_xv_device::service_ramp(int n, int kind)
{
	voice &v = m_voices[n];
	if (v.ramp_fade[kind])
	{
		v.ramp_position[kind] += v.ramp_step[kind];
		v.ramp_step[kind] -= kind >= RAMP_SEND_A ? v.ramp_position[kind] >> 4 : v.ramp_fade[kind];
		if (v.ramp_position[kind] > 0)
		{
			set_current(n, kind, v.ramp_position[kind] >> RAMP_FRACTION_BITS);
			return;
		}
		v.ramp_fade[kind] = 0;
	}
	else if (!v.ramp_remaining[kind])
		return;
	else if (--v.ramp_remaining[kind])
	{
		v.ramp_position[kind] += v.ramp_step[kind];
		set_current(n, kind, v.ramp_position[kind] >> RAMP_FRACTION_BITS);
		return;
	}
	set_current(n, kind, v.ramp_target[kind]);
	v.ramp_position[kind] = v.ramp_target[kind] << RAMP_FRACTION_BITS;
	v.ramp_step[kind] = 0;
	if (!v.ramp_armed[kind])
		return;
	switch (kind)
	{
	case RAMP_CUTOFF: raise_irq(IRQ_CUTOFF_LANDED, n); break;
	case RAMP_RESONANCE: raise_irq(IRQ_RESONANCE_LANDED, n); break;
	case RAMP_AMPLITUDE: raise_irq(IRQ_AMPLITUDE_LANDED, n); break;
	case RAMP_PITCH: raise_irq(IRQ_PITCH_LANDED, n); break;
	case RAMP_SEND_A: raise_irq(IRQ_SEND_A_LANDED, n); break;
	case RAMP_SEND_B: raise_irq(IRQ_SEND_B_LANDED, n); break;
	}
}


//-------------------------------------------------
//  the wave reader: word 0x60 bits 13:12 pick the XP's 8-bit format (1),
//  two samples a cell with the low byte first and each 1 Mi-sample page's
//  first 32 Ki its exponent-nibble table, or a 16-bit differential one
//  (0), a signed cell a sample.  Bits 27:25 of the address are the bank:
//  the internal set, the SR-JV80 slots (a byte-wide device, one sample a
//  cell), the four SRX slots and the two SIMMs; the loop points keep only
//  their 20-bit index.  Word 0x60 bit 7 gives the loop points a
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
	return (sample & ADDRESS_MASK & ~0x1ffffff) | ((sample & 0x1ffffff) >> 1);
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

s32 roland_xv_device::tap(s32 weight, wave_cell c)
{
	const s32 p = (weight * c.mantissa) & ~3;
	return c.exponent <= 10 ? p >> (10 - c.exponent) : p << (c.exponent - 10);
}

void roland_xv_device::launch(int n)
{
	voice &v = m_voices[n];
	v.launch = false;
	v.fetching = true;
	v.address = object_long(n, START) & ADDRESS_MASK;
	v.phase = 0;
	v.predictor = 0;
	v.backward = BIT(object_word(n, VOICE_CONTROL2), 5);
	v.region = REGION_BEFORE;
	v.finished = false;
	v.scaled = false;
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
	if (index == (object_long(n, END) & PAGE_MASK))
		v.region = REGION_END;
	else if (index == (object_long(n, LOOP_START) & PAGE_MASK))
		v.region = REGION_LOOP;
	else
		return;
	const u16 control = object_word(n, VOICE_CONTROL);
	const int condition = (control >> 8) & 3;
	if (v.region == (BIT(control, 6) ? REGION_LOOP : REGION_END))
		v.scaled = true;
	if (!v.finished && ((v.region == REGION_END && condition == END_AT_END) || (v.region == REGION_LOOP && condition == END_INSIDE_LOOP)))
	{
		v.finished = true;
		raise_irq(IRQ_FINISHED, n);
	}
}


//-------------------------------------------------
//  the filter: the XP's state-variable structure, the cutoff and resonance
//  coefficients Q15 words, the type nibble the output tap; OFF is the
//  high tap with both coefficients at zero
//-------------------------------------------------

s32 roland_xv_device::filter(int n, s32 sample)
{
	voice &v = m_voices[n];
	const int type = object_word(n, FILTER_TYPE) & 0xf;
	if (type >= FILTER_TYPES)
		return sample;

	const s32 f = v.ramp_current[RAMP_CUTOFF];
	const s32 q = v.ramp_current[RAMP_RESONANCE];
	const s32 band = v.filter_band;

	if (type >= FILTER_HIGH_POLE)
	{
		const s32 high = clamp24(s64(sample) - band);
		v.filter_band = clamp24(band + (s64(f) * high) / (1 << 15));
		const s32 tap = type == FILTER_HIGH_POLE ? band : high;
		return clamp24((type == FILTER_HIGH_POLE ? high : band) + (s64(q) * tap) / (1 << 15));
	}

	const s32 damping = type < FILTER_LOW_SHELF ? s32((s64(q) * band) / (1 << 15)) : band;
	const s32 low = clamp24(v.filter_low + (s64(f) * band) / (1 << 15));
	const s32 high = clamp24(s64(sample) - (damping + low));
	v.filter_low = low;
	v.filter_band = clamp24(band + (s64(f) * high) / (1 << 15));

	switch (type)
	{
	case FILTER_LPF: return low;
	case FILTER_BPF: return v.filter_band;
	case FILTER_HPF: return high;
	case FILTER_PKG: return clamp24(s64(low) - high);
	case FILTER_NOTCH: return clamp24(s64(low) + high);
	}
	const s32 tap = type == FILTER_LOW_SHELF ? low : type == FILTER_PEAK ? v.filter_band : high;
	return clamp24(sample + 2 * ((s64(q) * tap) / (1 << 15)));
}

void roland_xv_device::run_voice(int n, s32 *buses)
{
	voice &v = m_voices[n];
	const bool run = running(n);
	if (!run)
	{
		v.was_running = false;
		return;
	}
	if (v.launch || !v.was_running)
		launch(n);
	v.was_running = true;

	for (int kind = 0; kind < RAMPS; kind++)
		service_ramp(n, kind);

	const u16 control = object_word(n, VOICE_CONTROL);
	if (BIT(control, 13))
		return;
	const bool wide = !BIT(control, 12);
	const int mode = (control >> 10) & 3;

	address_step s{ v.address, v.backward, false };
	s32 sum = 4 * v.predictor;
	for (int i = 0; i < 3; i++)
	{
		sum += tap(interp_weights[i][v.phase >> 9], cell_at(s.address, wide));
		s = advance(n, s, v.phase);
	}
	s32 sample = wrap20(sum) >> (3 - std::min(3, (control >> 1) & 3));

	if (v.fetching)
	{
		u32 step = v.ramp_current[RAMP_PITCH] & 0x3ffff;
		if (v.scaled)
			step = (u64(step) * object_word(n, WAVE_SCALE + 1)) >> 15;
		const u32 accumulated = u32(v.phase) + step;
		u32 phase = accumulated & 0xffff;
		u32 carry = accumulated >> 16;
		address_step current{ v.address, v.backward, false };
		while (carry)
		{
			carry--;
			const s32 predictor = v.predictor + delta_of(cell_at(current.address, wide));
			v.predictor = wide ? wrap16(predictor) : wrap18(predictor);
			cross(n, current.address);
			if (mode == LOOP_NONE && v.region == REGION_END)
			{
				v.fetching = false;
				break;
			}
			const address_step next = advance(n, current, phase);
			if (next.wrapped)
			{
				const u64 end = object_long(n, END) & PAGE_MASK;
				const u64 over = (u64(current.address & PAGE_MASK) - end) * 0x10000 + phase - loop_fraction(n, false);
				const u32 total = u32(std::min<u64>(over, 0xffff)) + loop_fraction(n, true);
				phase = total & 0xffff;
				carry += total >> 16;
			}
			current = next;
		}
		v.phase = u16(phase);
		v.address = current.address;
		v.backward = current.backward;
	}

	sample = filter(n, sample);
	const s32 output = clamp24((s64(sample) * v.ramp_current[RAMP_AMPLITUDE]) / (1 << 15));

	for (int slot = 0; slot < SENDS; slot++)
	{
		const int bus = object_word(n, SEND_BASE + slot * 2) & 0x3f;
		const s32 level = object_word(n, SEND_BASE + slot * 2 + 1);
		if (bus < BUSES && level)
			buses[bus] += s32((s64(output) * level) / (1 << 15));
	}
}


//-------------------------------------------------
//  the DSP.  A row is W0 (the ALU: left and right operands, their signs,
//  the destination, a wrap, a second memory operation in W2, the
//  conditional family; the multiplier's operand and whether it takes the
//  C latch), W1 (a memory operation with its ten-bit address, the
//  coefficient shift, the multiply request) and W2 (the coefficient, an
//  immediate, or the second memory operation).  Every operand a row uses
//  is the state it was entered with.
//-------------------------------------------------

void roland_xv_device::decode_row(int n)
{
	dsp_row &r = m_rows[n];
	u16 control = r.w0;
	u16 bus = r.w1;
	r.conditional = BIT(r.w0, 15);
	r.branch = r.conditional && BIT(r.w0, 14);
	r.condition = 0;
	r.displacement = 0;
	if (r.conditional)
	{
		control = r.w0 & 0x3fff;
		r.condition = r.w1 & 0xf;
		r.displacement = s8((r.w1 >> 4) & 0xff);
		bus = r.w1 & 0xe000;
		if (BIT(r.w1, 12) && !BIT(r.w1, 15))
			control |= 0x4000;
	}
	r.multiply = BIT(bus, 15);
	r.shift = (0x4210 >> (4 * ((bus >> 13) & 3))) & 0xf;
	r.mode = (bus >> 10) & 7;
	r.address = bus & 0x3ff;
	r.mode2 = (r.operand >> 10) & 7;
	r.address2 = r.operand & 0x3ff;
	r.second = BIT(control, 14) && !r.multiply && r.mode2 != MEM_NONE;
	r.cell_coefficient = BIT(control, 0);
	r.source = (control >> 1) & 7;
	r.left = (control >> 8) & 7;
	r.right = (control >> 4) & 7;
	r.negate_left = BIT(control, 11);
	r.negate_right = BIT(control, 7);
	r.to_b = BIT(control, 12);
	r.wrap = BIT(control, 13);
	r.hold = r.left == LEFT_ZERO && r.right == RIGHT_ZERO;
	const bool immediate = r.right == RIGHT_K23 || r.right == RIGHT_K19 || r.right == RIGHT_K15;
	r.clamp = immediate && (r.left != LEFT_ZERO || r.negate_right);
	if (!r.multiply && r.right == RIGHT_K23)
		r.immediate = r.operand;
	else if (!r.multiply && r.right == RIGHT_K19)
		r.immediate = s32(r.operand) << 4;
	else
		r.immediate = s32(s16(r.operand)) << (DSP_FRACTION_BITS - 15 + r.shift);

	const bool nop = !r.w0 && !r.w1;
	if (!nop && n > m_rows_end)
		m_rows_end = n;
	else if (nop && n == m_rows_end)
		while (m_rows_end >= 0 && !m_rows[m_rows_end].w0 && !m_rows[m_rows_end].w1)
			m_rows_end--;
}

// a block is a header and the records after it up to the next header; a zero
// word is not a record, and a record before any header is nothing
void roland_xv_device::decode_transfers()
{
	m_transfer_count = 0;
	m_transfers_stale = false;
	bool open = false;
	u16 write_base = 0, read_base = 0;
	for (int n = 0; n < RECORDS; n++)
	{
		const u32 word = m_space[SPACE_RECORDS + n] & 0xfffff;
		if ((word & 0xfc000) == 0xfc000)
		{
			write_base = IBUS_STAGING + ((word >> 7) & 0x7f);
			read_base = IBUS_STAGING + (word & 0x7f);
			open = true;
		}
		else if (open && word)
		{
			const bool write = BIT(word, 19);
			m_transfers[m_transfer_count++] = transfer{ write, write ? write_base++ : read_base++, word & 0x7ffff };
		}
	}
}

s32 roland_xv_device::cell_r(u16 address) const
{
	if (address < IBUS_MIX)
		return wrap24(s32(m_space[ring_index(address)]));
	if (address < IBUS_BANK)
		return m_bus[address - IBUS_MIX];
	if (address < IBUS_BANK_END)
		return m_bank[address - IBUS_BANK];
	return 0;
}

void roland_xv_device::cell_w(u16 address, s32 value)
{
	if (address < IBUS_MIX)
		m_space[ring_index(address)] = u32(value);
	else if (address < IBUS_BANK)
		m_bus[address - IBUS_MIX] = value;
}

void roland_xv_device::log_once(int what, const char *text)
{
	if (BIT(m_logged, what))
		return;
	m_logged |= 1 << what;
	LOGMASKED(LOG_DSP, "%s: %s\n", machine().describe_context(), text);
}

// the sample: the read records into their cells, the rows from 0 to the halt,
// the write records out of theirs, the cursor down, the taps deposited
void roland_xv_device::run_dsp()
{
	if (m_transfers_stale)
		decode_transfers();
	for (int n = 0; n < m_transfer_count; n++)
		if (!m_transfers[n].write)
			cell_w(m_transfers[n].cell, m_eram[eram_index(m_transfers[n].offset)]);

	int pc = 0;
	for (int steps = 0; pc <= m_rows_end && steps < DSP_ROW_BUDGET; steps++)
	{
		const dsp_row &row = m_rows[pc];
		if (!row.conditional)
		{
			execute(row, true);
			pc++;
			continue;
		}
		const bool taken = condition(row.condition);
		execute(row, row.branch || taken);
		if (!row.branch)
		{
			pc++;
			continue;
		}
		if (pc + 1 >= DSP_ROWS)
			break;
		const dsp_row &slot = m_rows[pc + 1];
		if (slot.branch)
			log_once(0, "branch in a delay slot");
		execute(slot, !slot.conditional || slot.branch || condition(slot.condition));
		steps++;
		const int target = pc + 1 + row.displacement;
		if (taken && target == pc)
			break;
		pc = taken ? target & (DSP_ROWS - 1) : pc + 2;
	}

	for (int n = 0; n < m_transfer_count; n++)
		if (m_transfers[n].write)
			m_eram[eram_index(m_transfers[n].offset)] = cell_r(m_transfers[n].cell);
	m_cursor--;
	for (int n = 0; n < m_tap_count; n++)
	{
		const s32 whole = m_taps[n].delay >> 4;
		cell_w(m_taps[n].cell, m_eram[eram_index(whole)]);
		cell_w(m_taps[n].cell + 1, m_eram[eram_index(whole + 1)]);
		cell_w(m_taps[n].cell + 2, (m_taps[n].delay & 15) << (DSP_FRACTION_BITS - 4));
	}
	m_tap_count = 0;
}

bool roland_xv_device::condition(int code)
{
	switch (code)
	{
	case 0x0: return false;
	case 0x1: return true;
	case 0x2: return m_flag_value == 0;
	case 0x3: return m_flag_value != 0;
	case 0x6: return m_flag_raw >= (1 << DSP_FRACTION_BITS) || m_flag_raw <= -(1 << DSP_FRACTION_BITS);
	case 0x7: return m_flag_raw < (1 << DSP_FRACTION_BITS) && m_flag_raw > -(1 << DSP_FRACTION_BITS);
	case 0x8: case 0xc: return m_flag_value >= 0;
	case 0x9: case 0xd: return m_flag_value < 0;
	case 0xa: return m_flag_value > 0;
	case 0xb: return m_flag_value <= 0;
	default:
		log_once(1, "unobserved condition code");
		return false;
	}
}

void roland_xv_device::execute(const dsp_row &row, bool commit)
{
	const s32 a = m_acc[0], b = m_acc[1], p = m_product;
	const s32 r = m_latch[0], sl = m_latch[1], m = m_latch[2], c = m_latch[3];
	const s32 k = row.immediate;

	s64 left = 0;
	switch (row.left)
	{
	case LEFT_R: left = r; break;
	case LEFT_S: left = sl; break;
	case LEFT_M: left = m; break;
	case LEFT_A: left = a; break;
	case LEFT_B: left = b; break;
	case LEFT_MAG_A: left = clamp24(std::abs(s64(a))); break;
	case LEFT_MAG_B: left = clamp24(std::abs(s64(b))); break;
	}
	s64 right = 0;
	switch (row.right)
	{
	case RIGHT_A: right = a; break;
	case RIGHT_B: right = b; break;
	case RIGHT_R: right = r; break;
	case RIGHT_K23: case RIGHT_K19: case RIGHT_K15: right = k; break;
	case RIGHT_P: right = p; break;
	}

	for (int op = 0; op < (row.second ? 2 : 1); op++)
	{
		const int mode = op ? row.mode2 : row.mode;
		const u16 address = op ? row.address2 : row.address;
		switch (mode)
		{
		case MEM_NONE:
			if (address && m_tap_count < TAPS)
				m_taps[m_tap_count++] = tap_request{ u16(address | 0x100), BIT(address, 8) ? b : a };
			break;
		case MEM_STORE_P: cell_w(address, clamp24(p)); break;
		case MEM_STORE_A: cell_w(address, clamp24(a)); break;
		case MEM_STORE_B: cell_w(address, clamp24(b)); break;
		default: m_latch[mode - MEM_READ_R] = cell_r(address); break;
		}
	}

	s64 operand = 0;
	switch (row.source)
	{
	case SOURCE_R: operand = r; break;
	case SOURCE_S: operand = sl; break;
	case SOURCE_M: operand = m; break;
	case SOURCE_C: operand = c; break;
	case SOURCE_A: operand = a; break;
	case SOURCE_B: operand = b; break;
	case SOURCE_P: operand = p; break;
	default: log_once(2, "unobserved multiplier selector"); break;
	}
	bool product = true;
	s64 coefficient = 0;
	if (row.multiply)
		coefficient = s32(s16(row.operand)) << (DSP_FRACTION_BITS - 15);
	else if (row.cell_coefficient)
		coefficient = c;
	else if (row.source == SOURCE_M || row.source == SOURCE_A || row.source == SOURCE_B)
		coefficient = a;
	else if (row.source == SOURCE_S)
		coefficient = s64(c) - (1 << DSP_FRACTION_BITS);
	else
		product = false;
	if (product)
	{
		const s64 full = operand * coefficient;
		const int shift = DSP_FRACTION_BITS - row.shift;
		m_product = s32((full + ((full >> 63) & ((s64(1) << shift) - 1))) >> shift);
	}

	const s32 destination = row.to_b ? b : a;
	s64 result = destination;
	if (!row.hold)
		result = (row.negate_left ? -left : left) + (row.negate_right ? -right : right);
	const s64 raw = result;
	if (row.wrap)
		result = wrap24(s32(result));
	else if (row.clamp)
		result = clamp24(result);
	const s32 value = s32(result);
	if (commit)
		m_acc[row.to_b ? 1 : 0] = value;

	if (!m_last_hold)
	{
		m_flag_value = m_last_value;
		m_flag_raw = m_last_raw;
	}
	m_last_value = value;
	m_last_raw = raw;
	m_last_hold = row.hold;
}
