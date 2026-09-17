// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Roland XV tone generator (TC223C660CF-503, RA08-503)

    The XP's successor: 64 voices a chip, two chips in an XV-5080.  The
    host sees a 16-bit register window: the address spaces reached through
    it (the DSP program rows, the scalars, the wide records and the
    per-voice longs), the FIFO transfer engine onto the chip's own wave and
    sample memory, the object-indexed voice file and the interrupt path.

    A voice reads the XP's sample format at a 25-bit sample address, steps
    it by a linear 18-bit pitch (0x10000 = one sample an output sample),
    scaled by word 0x76/77 from its first loop crossing on, runs its
    cutoff, feedback, level and pitch through host-targeted ramps
    with a 4-bit rate code each, filters, scales by a Q15 level and adds
    itself to up to six buses at six send levels.  The effect DSP is not
    here yet: every output pair is summed onto the stream and the chorus
    and reverb sends (buses 6 and 7) go nowhere.

    TODO:
    - the ramp's granularity, and whether its table is samples or a divider
    - the filter's state width, rounding and saturation; the BPF and PKG taps
    - the DSP, the effect buses and the serial link
    - expansion-board and sample-RAM wave formats
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

#define VERBOSE (LOG_GENERAL | LOG_XFER | LOG_REGS | LOG_IRQ | LOG_OBJECT | LOG_SPACE)
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

DEFINE_DEVICE_TYPE(ROLAND_XV, roland_xv_device, "roland_xv", "Roland XV tone generator")

roland_xv_device::roland_xv_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, ROLAND_XV, tag, owner, clock)
	, device_memory_interface(mconfig, *this)
	, device_sound_interface(mconfig, *this)
	, m_wave_config("wave", ENDIANNESS_LITTLE, 16, 32, -1)
	, m_int_callback(*this)
	, m_stream(nullptr)
{
}

device_memory_interface::space_config_vector roland_xv_device::memory_space_config() const
{
	return space_config_vector { std::make_pair(AS_WAVE, &m_wave_config) };
}

void roland_xv_device::device_start()
{
	space(AS_WAVE).specific(m_wave);
	m_stream = stream_alloc(0, 2, SAMPLE_RATE);
	m_space = std::make_unique<u32[]>(0x10000);

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
	m_run_mask = 0;
	for (auto &v : m_voices)
		v = voice();
}

void roland_xv_device::sound_stream_update(sound_stream &stream)
{
	for (int i = 0; i < stream.samples(); i++)
	{
		s32 buses[BUSES] = { 0 };
		for (int n = 0; n < OBJECTS; n++)
			run_voice(n, buses);
		s32 left = 0, right = 0;
		for (int pair = 0; pair < 16; pair += 2)
			if (pair != BUS_CHORUS)
			{
				left += buses[pair];
				right += buses[pair + 1];
			}
		stream.put_int_clamp(0, i, left, 1 << OUTPUT_BITS);
		stream.put_int_clamp(1, i, right, 1 << OUTPUT_BITS);
	}
}

//-------------------------------------------------
//  the window: a word is its high byte then its low byte, and the low byte
//  is where a write commits and a read takes its side effects
//-------------------------------------------------

u8 roland_xv_device::read(offs_t offset)
{
	const int word = (offset >> 1) & 0xff;
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
	if (word >= OBJECT_BASE || (word >= RUN_MASK && word <= RUN_COMMIT))
		m_stream->update();
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
		return m_fifo[m_fifo_read];

	case IRQ_MASK:
		return m_irq_pending;

	case STATUS:
		return 0;

	default:
		if (word >= IRQ_VOICE && word < IRQ_VOICE + IRQ_REASONS)
			return m_irq_voice[word - IRQ_VOICE];
		if (word == CUTOFF || word == FEEDBACK || word == LEVEL)
			return m_voices[object()].ramp_current[word == CUTOFF ? RAMP_CUTOFF : word == FEEDBACK ? RAMP_FEEDBACK : RAMP_LEVEL];
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
		m_address++;
		break;

	case FIFO:
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
		m_space[m_address] = (u32(m_data_high) << 16) | data;
		LOGMASKED(LOG_SPACE, "%s: write %04x = %08x\n", machine().describe_context(), m_address, m_space[m_address]);
		m_address++;
		m_data_high = 0;
		break;

	case ADDRESS:
		m_address = data;
		break;

	case FIFO:
		fifo_push(data);
		break;

	case FIFO_CONTROL:
		fifo_rewind();
		break;

	case COMMAND_STROBE:
		LOGMASKED(LOG_XFER, "%s: command %04x (strobe %04x)\n", machine().describe_context(), m_fifo[0], data);
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

	case CUTOFF:
		seed_ramp(voice, RAMP_CUTOFF, data);
		break;

	case FEEDBACK:
		seed_ramp(voice, RAMP_FEEDBACK, data);
		break;

	case LEVEL:
		seed_ramp(voice, RAMP_LEVEL, data);
		break;

	case CUTOFF_RAMP + 1:
		start_ramp(voice, RAMP_CUTOFF, value);
		break;

	case FEEDBACK_RAMP + 1:
		start_ramp(voice, RAMP_FEEDBACK, value);
		break;

	case LEVEL_RAMP + 1:
		start_ramp(voice, RAMP_LEVEL, value);
		LOGMASKED(LOG_OBJECT, "%s: object %03x level %08x\n", machine().describe_context(), m_regs[MODE], value);
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

	case FEEDBACK_INCREMENT + 1:
		increment_ramp(voice, RAMP_FEEDBACK, value);
		break;

	case LEVEL_INCREMENT + 1:
		increment_ramp(voice, RAMP_LEVEL, value);
		break;

	case SEND_PORT_A + 1:
		send_port_w(voice, RAMP_SEND_A, value);
		break;

	case SEND_PORT_B + 1:
		send_port_w(voice, RAMP_SEND_B, value);
		break;

	case BLOCK_CONTROL + 1:
		LOGMASKED(LOG_OBJECT, "%s: object %03x control %08x\n", machine().describe_context(), m_regs[MODE], value);
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
//  ramps: a target long carries a 4-bit rate code (bits 19:16; 22:19 on
//  the pitch) and a landing time of 2 ms a code to 20 ms, 4 ms a code to
//  40 ms, then 48 ms; the pitch's bit 23 and the others' bit 21 arm the
//  arrival interrupt, the pitch's bit 24 and the others' bit 22 keep the
//  slope in flight, and bit 20 on a cutoff, feedback or level long fades
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
	case RAMP_FEEDBACK: raise_irq(IRQ_FEEDBACK_LANDED, n); break;
	case RAMP_LEVEL: raise_irq(IRQ_LEVEL_LANDED, n); break;
	case RAMP_PITCH: raise_irq(IRQ_PITCH_LANDED, n); break;
	case RAMP_SEND_A: raise_irq(IRQ_SEND_A_LANDED, n); break;
	case RAMP_SEND_B: raise_irq(IRQ_SEND_B_LANDED, n); break;
	}
}


//-------------------------------------------------
//  the wave reader: the XP's sample format at a 25-bit sample address,
//  two samples a cell, the low byte first; each 1 MB region's first 32 kB
//  is its exponent-nibble table.  Word 0x60 bit 7 gives the loop points a
//  sub-sample fraction each, the two bytes of word 0x74/75, which the
//  forward loop takes.  Bits 11:10 are the loop mode; the end (word 0x84)
//  is where a loop rewinds or a ping-pong turns in either direction, the
//  loop start (word 0x82) the rewind target and the other turn, and a
//  voice with no loop holds at its end
//-------------------------------------------------

u8 roland_xv_device::sample_byte(u32 sample)
{
	const u16 cell = m_wave.read_word(sample >> 1);
	return BIT(sample, 0) ? cell >> 8 : cell & 0xff;
}

roland_xv_device::wave_cell roland_xv_device::cell_at(u32 address)
{
	const u8 byte = sample_byte(address);
	const u8 shifts = sample_byte((address & ~0xfffff) | ((address & 0xfffff) >> 5));
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
	v.address = object_long(n, START) & 0x1ffffff;
	v.phase = 0;
	v.predictor = 0;
	v.backward = BIT(object_word(n, VOICE_CONTROL2), 5);
	v.region = REGION_BEFORE;
	v.finished = false;
	v.scaled = false;
	v.filter_low = 0;
	v.filter_band = 0;
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
	const u32 loop = object_long(n, LOOP_START) & 0x1ffffff;
	const u32 end = object_long(n, END) & 0x1ffffff;
	const int mode = (object_word(n, VOICE_CONTROL) >> 10) & 3;

	if (!s.backward)
	{
		const bool past_end = s.address > end || (s.address == end && phase >= loop_fraction(n, false));
		if (!past_end)
			return { s.address + 1, false, false };
		switch (mode)
		{
		case LOOP_FORWARD: return { loop, false, true };
		case LOOP_ALTERNATE: return { s.address, true, false };
		default: return { s.address, false, false };
		}
	}

	if (mode == LOOP_ALTERNATE)
		return s.address <= loop ? address_step{ s.address, false, false } : address_step{ s.address - 1, true, false };
	if (s.address > end)
		return { s.address - 1, true, false };
	return mode == LOOP_FORWARD ? address_step{ loop, true, false } : address_step{ s.address, true, false };
}

// the region a fetched sample lies in, and what word 0x60 makes of the crossing: bit 6
// picks the crossing that adopts the second pitch scale, bits 9:8 the one that finishes
void roland_xv_device::cross(int n, u32 address)
{
	voice &v = m_voices[n];
	if (address == (object_long(n, END) & 0x1ffffff))
		v.region = REGION_END;
	else if (address == (object_long(n, LOOP_START) & 0x1ffffff))
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
//  the filter: the XP's state-variable structure, the cutoff and feedback
//  coefficients Q15 words, the type nibble the output tap; OFF is the
//  high tap with both coefficients at zero
//-------------------------------------------------

s32 roland_xv_device::filter(int n, s32 sample)
{
	voice &v = m_voices[n];
	const int type = object_word(n, FILTER_TYPE) & 0xf;
	const s32 f = v.ramp_current[RAMP_CUTOFF];
	const s32 q = v.ramp_current[RAMP_FEEDBACK];
	s32 low = clamp24(v.filter_low + (s64(f) * v.filter_band) / (1 << 15));
	const s32 high = clamp24(sample - (s32((s64(q) * v.filter_band) / (1 << 15)) + low));
	const s32 band = clamp24(v.filter_band + (s64(f) * high) / (1 << 15));
	v.filter_low = low;
	v.filter_band = band;
	switch (type)
	{
	case FILTER_LPF: return low;
	case FILTER_BPF: return band;
	case FILTER_PKG: return clamp24(s64(high) - low);
	default: return high;
	}
}


//-------------------------------------------------
//  the voice
//-------------------------------------------------

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
	if (!BIT(control, 12) || BIT(control, 13))
		return;
	const int mode = (control >> 10) & 3;

	address_step s{ v.address, v.backward, false };
	s32 sum = 4 * v.predictor;
	for (int i = 0; i < 3; i++)
	{
		sum += tap(interp_weights[i][v.phase >> 9], cell_at(s.address));
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
			v.predictor = wrap18(v.predictor + delta_of(cell_at(current.address)));
			cross(n, current.address);
			if (mode == LOOP_NONE && v.region == REGION_END)
			{
				v.fetching = false;
				break;
			}
			const address_step next = advance(n, current, phase);
			if (next.wrapped)
			{
				const u64 end = object_long(n, END) & 0x1ffffff;
				const u64 over = (u64(current.address) - end) * 0x10000 + phase - loop_fraction(n, false);
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
	const s32 output = clamp24((s64(sample) * v.ramp_current[RAMP_LEVEL]) / (1 << 15));

	for (int slot = 0; slot < SENDS; slot++)
	{
		const int bus = object_word(n, SEND_BASE + slot * 2) & 0x3f;
		const s32 level = object_word(n, SEND_BASE + slot * 2 + 1);
		if (bus < BUSES && level)
			buses[bus] += s32((s64(output) * level) / (1 << 15));
	}
}
