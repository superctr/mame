// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Roland EP (Fujitsu MB87731A), the JD-800's and JD-990's PCM sample
    player.

    TODO:
    - the chip's own output stage, and its unity pitch in samples a second
    - the gate word, and the registers the firmware writes at boot

***************************************************************************/

#include "emu.h"
#include "roland_ep.h"

#define LOG_REGS  (1U << 1)

#define VERBOSE (0)
#include "logmacro.h"

namespace {

// the resampling kernel: three weights on the deltas after the sample, by
// the phase's top seven bits, in 1/4096
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

} // anonymous namespace

DEFINE_DEVICE_TYPE(ROLAND_EP, roland_ep_device, "roland_ep", "Roland EP PCM sample player")

roland_ep_device::roland_ep_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, ROLAND_EP, tag, owner, clock)
	, device_memory_interface(mconfig, *this)
	, device_sound_interface(mconfig, *this)
	, m_wave_config("wave", ENDIANNESS_LITTLE, 8, 24, 0)
	, m_stream(nullptr)
{
}

device_memory_interface::space_config_vector roland_ep_device::memory_space_config() const
{
	return space_config_vector { std::make_pair(AS_WAVE, &m_wave_config) };
}

void roland_ep_device::device_start()
{
	space(AS_WAVE).specific(m_wave);
	m_stream = stream_alloc(0, VOICES, SAMPLE_RATE);

	save_item(NAME(m_regs));
	save_item(NAME(m_key_mask));
	save_item(STRUCT_MEMBER(m_voices, regs));
	save_item(STRUCT_MEMBER(m_voices, address));
	save_item(STRUCT_MEMBER(m_voices, phase));
	save_item(STRUCT_MEMBER(m_voices, predictor));
	save_item(STRUCT_MEMBER(m_voices, backward));
	save_item(STRUCT_MEMBER(m_voices, running));
}

void roland_ep_device::device_reset()
{
	std::fill(std::begin(m_regs), std::end(m_regs), 0);
	m_key_mask = 0;
	for (voice &v : m_voices)
		v = voice();
}


//-------------------------------------------------
//  the window: the globals, the read-through and, behind the select, a
//  voice's own words
//-------------------------------------------------

u16 roland_ep_device::read(offs_t offset)
{
	const int word = offset & (WORDS - 1);
	if (word == READ_DATA)
	{
		const u32 address = ((u32(m_regs[READ_HIGH]) << 16) | m_regs[READ_LOW]) >> 8;
		return m_wave.read_byte(address & ADDRESS_MASK);
	}
	return 0;
}

void roland_ep_device::write(offs_t offset, u16 data)
{
	const int word = offset & (WORDS - 1);
	LOGMASKED(LOG_REGS, "%s: word %02X = %04X (voice %d)\n", machine().describe_context(), word, data, m_regs[SELECT] & (VOICES - 1));

	switch (word)
	{
	case READ_LOW:
	case READ_HIGH:
	case SELECT:
	case CONFIG:
	case VOICES_IN_USE:
	case RESET:
		m_regs[word] = data;
		return;

	case KEY_LOW:
	case KEY_HIGH:
		m_stream->update();
		key_w(word, data);
		return;

	default:
		m_stream->update();
		m_voices[m_regs[SELECT] & (VOICES - 1)].regs[word] = data;
		return;
	}
}

void roland_ep_device::voice_pitch_w(int voice, u16 data)
{
	m_stream->update();
	m_voices[voice & (VOICES - 1)].regs[PITCH] = data;
}

void roland_ep_device::key_w(int word, u16 data)
{
	m_regs[word] = data;
	const u32 mask = (u32(m_regs[KEY_HIGH]) << 16) | m_regs[KEY_LOW];
	const u32 rising = mask & ~m_key_mask;
	m_key_mask = mask;
	for (int n = 0; n < VOICES; n++)
	{
		if (BIT(rising, n))
			launch(n);
		else if (!BIT(mask, n))
			m_voices[n].running = false;
	}
}


//-------------------------------------------------
//  the wave: a pair holds the sample address times 4096 with the
//  overflowed top nibble in the wave-select word; the deltas from the
//  data area, their exponents from the region's nibble table
//-------------------------------------------------

u32 roland_ep_device::address_of(const voice &v, int high) const
{
	const u32 page = (v.regs[WAVE] >> 4) & 0xf;
	return (page << 20) | (u32(v.regs[high + 1]) << 4) | (v.regs[high] >> 12);
}

void roland_ep_device::advance(const voice &v, u32 &address, bool &backward, wave_cell &c)
{
	if (backward && address <= loop_of(v))
		backward = false;

	if (backward)
	{
		c = cell_at((address - 1) & ADDRESS_MASK);
		address = (address - 1) & ADDRESS_MASK;
	}
	else
	{
		c = cell_at(address);
		address = (address + 1) & ADDRESS_MASK;
		if (address > end_of(v))
		{
			if (BIT(v.regs[WAVE], 2))
				backward = true;
			else
				address = loop_of(v);
		}
	}
}

roland_ep_device::wave_cell roland_ep_device::cell_at(u32 address)
{
	const u8 byte = m_wave.read_byte(address);
	const u8 shifts = m_wave.read_byte((address & ~PAGE_MASK) | ((address & PAGE_MASK) >> 5));
	const int exponent = BIT(address, 4) ? (shifts >> 4) : (shifts & 0x0f);
	return wave_cell{ exponent > 10 ? 0 : s8(byte), exponent };
}

s32 roland_ep_device::tap(s32 weight, wave_cell c)
{
	const s32 p = (weight * c.mantissa) & ~3;
	return p >> (10 - c.exponent);
}

void roland_ep_device::launch(int n)
{
	voice &v = m_voices[n];
	v.address = address_of(v, START_HIGH);
	v.phase = (v.regs[START_HIGH] & 0xfff) << 4;
	v.predictor = 0;
	v.backward = false;
	v.running = true;
}

s32 roland_ep_device::run_voice(int n)
{
	voice &v = m_voices[n];
	if (!v.running || !BIT(v.regs[GATE], 15))
		return 0;

	u32 address = v.address;
	bool backward = v.backward;
	wave_cell c;
	s32 sum = 4 * v.predictor;
	for (int i = 0; i < 3; i++)
	{
		advance(v, address, backward, c);
		sum += tap(interp_weights[i][v.phase >> 9], c);
	}
	const s32 sample = wrap20(sum >> 2);

	const u32 accumulated = u32(v.phase) + (u32(v.regs[PITCH]) << 2);
	v.phase = accumulated & 0xffff;
	for (u32 carry = accumulated >> 16; carry; carry--)
	{
		advance(v, v.address, v.backward, c);
		v.predictor = wrap20(v.predictor + (c.mantissa << c.exponent));
	}
	return sample;
}

void roland_ep_device::sound_stream_update(sound_stream &stream)
{
	for (int i = 0; i < stream.samples(); i++)
		for (int n = 0; n < VOICES; n++)
			stream.put_int_clamp(n, i, run_voice(n), 1 << (SAMPLE_BITS - 1));
}
