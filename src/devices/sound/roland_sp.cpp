// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Roland SP-AG / SP-IP (Fujitsu MB87422 / MB87423A) sampler pair, the
    S-750, S-770 and S-760's sample players: 32 voices over the wave DRAM,
    the host's own port into that DRAM, and one output per voice to the TVF.

    Per voice, three register blocks behind data ports: a write or read of
    the voice's address selects the register, and the value goes through
    the block's port.  A voice plays forward, in reverse, or alternating
    between its loop points; it raises a loop event when it first crosses
    its loop boundary and an end event at its end address, unless C2's bit
    31 or 30 masks them, through a queue the host reads at C002 and
    acknowledges at C016.

    The interpolation is the GP's, XP's and XV's kernel (four taps, as
    the current sample and three differences after it), assumed for this
    family too.  Voices 4 and 20 record the left and right input words
    into the wave memory instead of playing while C004 bits 0 and 1 are
    set; C01C reads the input words in turn, C000 bits 7 and 6 marking
    left and right.  Not emulated: the C6 registers.

***************************************************************************/

#include "emu.h"
#include "roland_sp.h"

#define VERBOSE (0)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(ROLAND_SP, roland_sp_device, "roland_sp", "Roland SP-AG/SP-IP sampler")

namespace {

enum : u8 {
	BLOCK_NONE = 0,
	BLOCK_C2,
	BLOCK_C4,
	BLOCK_C6
};

// the resampling kernel: three weights on the deltas after the sample, by
// the phase's top seven bits, in 1/4096
constexpr s16 interp_weights[3][128] = {
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

// C4 block words
enum : int {
	C4_END = 0, C4_PITCH = 2, C4_LOOP_TO = 3, C4_LOOP_AT = 5
};

}

roland_sp_device::roland_sp_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, ROLAND_SP, tag, owner, clock),
	device_sound_interface(mconfig, *this),
	m_int_cb(*this),
	m_stream(nullptr),
	m_int_timer(nullptr),
	m_key{ 0, 0 },
	m_sel_block(BLOCK_NONE), m_sel_voice(0), m_sel_word(0),
	m_c2_latch(0),
	m_event_head(0), m_event_count(0), m_int_state(false),
	m_host_addr(0), m_host_latch(0),
	m_record_inputs(false), m_record_in{ 0, 0 }, m_level_channel(0)
{
	std::fill(std::begin(m_ctrl), std::end(m_ctrl), 0);
	std::fill(std::begin(m_blocks), std::end(m_blocks), nullptr);
	std::fill(std::begin(m_events), std::end(m_events), 0);
}

void roland_sp_device::device_start()
{
	m_stream = stream_alloc(m_record_inputs ? 2 : 0, VOICES, clock(), m_record_inputs ? STREAM_SYNCHRONOUS : STREAM_DEFAULT_FLAGS);
	m_int_timer = timer_alloc(FUNC(roland_sp_device::update_int), this);

	for(int i = 0; i != VOICES; i++) {
		save_item(NAME(m_voices[i].c4), i);
		save_item(NAME(m_voices[i].c6), i);
		save_item(NAME(m_voices[i].pos), i);
		save_item(NAME(m_voices[i].flags), i);
		save_item(NAME(m_voices[i].on), i);
		save_item(NAME(m_voices[i].looped), i);
		save_item(NAME(m_voices[i].ended), i);
	}
	save_item(NAME(m_key));
	save_item(NAME(m_ctrl));
	save_item(NAME(m_sel_block));
	save_item(NAME(m_sel_voice));
	save_item(NAME(m_sel_word));
	save_item(NAME(m_c2_latch));
	save_item(NAME(m_events));
	save_item(NAME(m_event_head));
	save_item(NAME(m_event_count));
	save_item(NAME(m_int_state));
	save_item(NAME(m_host_addr));
	save_item(NAME(m_host_latch));
	save_item(NAME(m_record_in));
	save_item(NAME(m_level_channel));
}

void roland_sp_device::device_clock_changed()
{
	m_stream->set_sample_rate(clock());
}

void roland_sp_device::device_reset()
{
	for(voice &v : m_voices)
		v = voice();
	m_key[0] = m_key[1] = 0;
	m_sel_block = BLOCK_NONE;
	m_event_head = m_event_count = 0;
	m_int_state = false;
	m_int_cb(0);
}

void roland_sp_device::set_wave_block(unsigned block, u16 *words)
{
	if(m_stream)
		m_stream->update();
	m_blocks[block % WAVE_BLOCKS] = words;
}

u16 roland_sp_device::host_read()
{
	u16 const data = m_host_latch;
	m_host_addr++;
	m_host_latch = wave_word(m_host_addr);
	return data;
}

void roland_sp_device::host_write(u16 data, u16 mem_mask)
{
	m_stream->update();
	if(u16 *const w = wave_ptr(m_host_addr))
		COMBINE_DATA(w);
	m_host_addr++;
}

u32 roland_sp_device::c2_read(const voice &v) const
{
	return (v.flags & ~0x07ffffffU) | (u32(v.pos >> (POS_FRACTION - 3)) & 0x07ffffff);
}

void roland_sp_device::c2_write(voice &v, u32 data)
{
	v.flags = data & ~0x07ffffffU;
	v.pos = s64(data & 0x07ffffff) << (POS_FRACTION - 3);
}

void roland_sp_device::key_w(int base, u16 data)
{
	m_stream->update();
	u16 const rise = data & ~m_key[base >> 4];
	m_key[base >> 4] = data;
	for(int i = 0; i != 16; i++) {
		voice &v = m_voices[base + i];
		v.on = BIT(data, i);
		if(BIT(rise, i)) {
			v.looped = v.ended = false;
			LOG("key on %d at %06x pitch %04x mode %02x flags %02x\n", base + i, u32(v.pos >> POS_FRACTION), v.c4[C4_PITCH], v.c4[C4_END + 1] >> 8, v.flags >> 24);
		}
	}
}

void roland_sp_device::event(int voice, u8 kind)
{
	if(m_event_count == EVENTS) {
		LOG("event queue full\n");
		return;
	}
	m_events[(m_event_head + m_event_count++) % EVENTS] = kind | voice;
	m_int_timer->adjust(attotime::zero);
}

TIMER_CALLBACK_MEMBER(roland_sp_device::update_int)
{
	bool const state = m_event_count != 0;
	if(state != m_int_state) {
		m_int_state = state;
		m_int_cb(state);
	}
}

u16 roland_sp_device::read(offs_t offset, u16 mem_mask)
{
	offset &= 0x3ff;
	if(offset >= 0x100) {
		if(!machine().side_effects_disabled()) {
			m_stream->update();
			m_sel_block = BLOCK_C2 + (offset >> 8) - 1;
			m_sel_voice = (offset >> 3) & 0x1f;
			m_sel_word = offset & 7;
			if(m_sel_block == BLOCK_C2)
				m_c2_latch = c2_read(m_voices[m_sel_voice]);
		}
		return 0;
	}

	switch(offset) {
	case 0x00:
		if(!machine().side_effects_disabled())
			m_level_channel ^= 1;
		return m_level_channel ? 0x0082 : 0x0042;
	case 0x01:
		if(!machine().side_effects_disabled())
			m_stream->update();
		return m_event_count ? m_events[m_event_head] : 0;
	case 0x05:
		if(machine().side_effects_disabled())
			return m_host_latch;
		return host_read();
	case 0x08:
		return m_c2_latch;
	case 0x09:
		return m_c2_latch >> 16;
	case 0x0a:
		return m_sel_block == BLOCK_C4 ? m_voices[m_sel_voice].c4[m_sel_word] : 0;
	case 0x0b:
		if(!machine().side_effects_disabled() && m_event_count) {
			m_event_head = (m_event_head + 1) % EVENTS;
			m_event_count--;
			m_int_state = false;
			m_int_cb(0);
			m_int_timer->adjust(attotime::zero);
		}
		return 0;
	case 0x0d:
		return m_sel_block == BLOCK_C6 ? m_voices[m_sel_voice].c6[m_sel_word] : 0;
	case 0x0e:
		return u16(m_record_in[m_level_channel] >> 11);
	default:
		if(!machine().side_effects_disabled())
			LOG("read %04x\n", 0xc000 + offset * 2);
		return m_ctrl[offset & 0xf];
	}
}

void roland_sp_device::write(offs_t offset, u16 data, u16 mem_mask)
{
	offset &= 0x3ff;
	if(offset >= 0x100) {
		m_sel_block = BLOCK_C2 + (offset >> 8) - 1;
		m_sel_voice = (offset >> 3) & 0x1f;
		m_sel_word = offset & 7;
		return;
	}

	switch(offset) {
	case 0x00:
		key_w(0, data);
		break;
	case 0x01:
		key_w(16, data);
		break;
	case 0x04:
		host_write(data, mem_mask);
		break;
	case 0x06:
		m_host_addr = (m_host_addr & 0x0000ffff) | (data << 16);
		m_host_latch = wave_word(m_host_addr);
		break;
	case 0x07:
		m_host_addr = (m_host_addr & 0xffff0000) | data;
		m_host_latch = wave_word(m_host_addr);
		break;
	case 0x08:
		m_c2_latch = (m_c2_latch & 0xffff0000) | data;
		break;
	case 0x09:
		m_c2_latch = (m_c2_latch & 0x0000ffff) | (data << 16);
		if(m_sel_block == BLOCK_C2) {
			m_stream->update();
			c2_write(m_voices[m_sel_voice], m_c2_latch);
		}
		break;
	case 0x0a:
		if(m_sel_block == BLOCK_C4) {
			m_stream->update();
			m_voices[m_sel_voice].c4[m_sel_word] = data;
		}
		break;
	case 0x0d:
		if(m_sel_block == BLOCK_C6) {
			m_stream->update();
			m_voices[m_sel_voice].c6[m_sel_word] = data;
		}
		break;
	default:
		LOG("write %04x = %04x\n", 0xc000 + offset * 2, data);
		m_ctrl[offset & 0xf] = data;
		break;
	}
}

s32 roland_sp_device::step(int n, voice &v)
{
	bool const backward = (c4_long(v, C4_END) & MODE_REVERSE) || (v.flags & C2_BACKWARD);
	s64 adr = v.pos >> POS_FRACTION;
	int phase = v.pos & ((1 << POS_FRACTION) - 1);
	int dir = 1;
	if(backward && phase) {
		adr++;
		phase = (1 << POS_FRACTION) - phase;
		dir = -1;
	} else if(backward)
		dir = -1;
	phase >>= POS_FRACTION - 7;

	s32 prev = s16(wave_word(adr));
	s32 sum = 4 * prev;
	for(int i = 0; i != 3; i++) {
		s32 const next = s16(wave_word(adr + dir * (i + 1)));
		sum += ((interp_weights[i][phase] * (next - prev)) & ~3) >> 10;
		prev = next;
	}
	advance(n, v);
	return sum >> 2;
}

void roland_sp_device::advance(int n, voice &v)
{
	u32 const end = c4_long(v, C4_END);
	s64 const loop_to = s64(c4_long(v, C4_LOOP_TO)) << (POS_FRACTION - 8);
	s64 const loop_at = s64(c4_long(v, C4_LOOP_AT)) << (POS_FRACTION - 8);
	s64 const pitch = v.c4[C4_PITCH];

	auto looped = [&]() {
		if(!v.looped && !(v.flags & C2_NO_LOOP_EVENT)) {
			v.looped = true;
			event(n, EVENT_LOOP);
		}
	};

	if(end & MODE_REVERSE) {
		v.pos -= pitch;
		if(v.pos <= loop_at) {
			looped();
			if(end & MODE_ONE_SHOT)
				v.on = false;
			else
				v.pos += loop_to - loop_at;
		}
	} else if(v.flags & C2_BACKWARD) {
		v.pos -= pitch;
		if(v.pos <= loop_to) {
			v.flags &= ~C2_BACKWARD;
			v.pos = 2 * loop_to - v.pos;
		}
	} else {
		v.pos += pitch;
		if(v.pos > loop_at) {
			looped();
			if(end & MODE_ONE_SHOT)
				v.on = false;
			else if(end & MODE_ALTERNATE) {
				v.flags |= C2_BACKWARD;
				v.pos = 2 * loop_at - v.pos;
			} else
				v.pos -= loop_at - loop_to;
		}
		if(!(v.flags & C2_NO_END_EVENT) && !v.ended && v.pos >= s64(end & 0x00ffffff) << POS_FRACTION) {
			v.ended = true;
			event(n, EVENT_END);
		}
	}
}

bool roland_sp_device::recording(int n) const
{
	return BIT(m_ctrl[2], 0) && (n == RECORD_VOICES[0] || n == RECORD_VOICES[1]);
}

void roland_sp_device::record_w(offs_t offset, u32 data)
{
	if(offset > 1)
		return;
	m_record_in[offset] = util::sext(data, 24);
	if(offset == 1)
		record();
}

void roland_sp_device::record()
{
	for(int c = 0; c != 2; c++) {
		int const n = RECORD_VOICES[c];
		voice &v = m_voices[n];
		if(!v.on || !recording(n))
			continue;
		if(u16 *const w = wave_ptr(v.pos >> POS_FRACTION))
			*w = u16(m_record_in[c] >> 8);
		advance(n, v);
	}
}

void roland_sp_device::sound_stream_update(sound_stream &stream)
{
	if(m_record_inputs)
		for(int i = 0; i != stream.samples(); i++) {
			for(int c = 0; c != 2; c++)
				m_record_in[c] = std::clamp(s32(stream.get(c, i) * 0x800000), -0x800000, 0x7fffff);
			record();
		}
	for(int n = 0; n != VOICES; n++) {
		voice &v = m_voices[n];
		for(int i = 0; i != stream.samples(); i++)
			stream.put_int(n, i, v.on && !recording(n) ? step(n, v) : 0, 65536);
	}
}
