// license:BSD-3-Clause
// copyright-holders:superctr
// thanks-to: giulioz
/***************************************************************************

    Roland SP-AG / SP-IP (Fujitsu MB87422 / MB87423A) sampler pair, the
    S-750, S-770 and S-760's sample players: 32 voices over the wave DRAM,
    the host's own port into that DRAM, and one output per voice to the TVF.

    Per voice, three register blocks behind data ports: a write or read of
    the voice's address selects the register, and the value goes through
    the block's port.  A voice plays forward, in reverse, or alternating
    between its loop points; it raises a loop event when it first crosses
    its loop boundary and an end event at its end address, unless its state
    word's bit 31 or 30 masks them, through a queue the host reads at C002 and
    acknowledges at C016.

    Each frame a voice reads four words from its position and adds three
    weighted differences from the first to it, the weights chosen by the
    fraction's top seven bits; the window and the weights are the same in
    either direction.  The result is scaled by the voice's level: C604 is
    the target, C602 the current level, which each frame blends toward the
    target and then takes it.  Voices 20 and 4 record the frame's first and
    second input words, through a five-tap filter, into the wave memory
    instead of playing while C004 bit 0 is set and their C604 level is
    zero; C01C reads the input words in turn, C000 bit 7 set while it
    reads voice 20's and bit 6 while it reads voice 4's.

***************************************************************************/

#include "emu.h"
#include "roland_sp.h"

#define VERBOSE (0)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(ROLAND_SP, roland_sp_device, "roland_sp", "Roland SP-AG/SP-IP sampler")

namespace {

// the global registers, by word offset from C000
enum : offs_t
{
	REG_KEY_LOW = 0x00, REG_STATUS = 0x00,
	REG_KEY_HIGH = 0x01, REG_EVENT = 0x01,
	REG_RECORD = 0x02,
	REG_UNKNOWN_06 = 0x03,
	REG_WAVE_STORE = 0x04, REG_WAVE_DATA = 0x05,
	REG_WAVE_ADDR_HIGH = 0x06, REG_WAVE_ADDR_LOW = 0x07,
	REG_STATE_LOW = 0x08, REG_STATE_HIGH = 0x09,
	REG_PLAYBACK = 0x0a,
	REG_ACK = 0x0b,
	REG_UNKNOWN_18 = 0x0c,
	REG_LEVEL = 0x0d,
	REG_INPUT = 0x0e, REG_INPUT_TAP = 0x0f
};

enum : u16 { RECORD_WRITE = 0x0001, RECORD_AUX = 0x0002 };

// the playback block's words: end, pitch, loop start and loop end, the 32-bit ones low half first
enum : int
{
	PLAY_END_LOW = 0, PLAY_END_HIGH, PLAY_PITCH, PLAY_LOOP_START_LOW, PLAY_LOOP_START_HIGH, PLAY_LOOP_END_LOW, PLAY_LOOP_END_HIGH,
	PLAY_UNKNOWN_E
};

// the level block's words, C600-C606
enum : int { LEVEL_UNKNOWN_0 = 0, LEVEL_LOW, LEVEL_HIGH, LEVEL_UNKNOWN_6 };

// the resampling kernel: the weights of the three words after the first, on
// their differences from it, by the phase's top seven bits, in 1/8192
constexpr s16 interp_weights[3][128] = {
	{
		5350, 5348, 5346, 5343, 5339, 5334, 5328, 5322, 5315, 5307, 5298, 5289, 5279, 5268, 5256, 5243,
		5230, 5216, 5201, 5186, 5169, 5152, 5135, 5116, 5097, 5077, 5057, 5035, 5013, 4991, 4967, 4943,
		4919, 4894, 4868, 4841, 4814, 4786, 4758, 4729, 4700, 4670, 4639, 4608, 4576, 4544, 4512, 4479,
		4445, 4411, 4377, 4342, 4306, 4271, 4235, 4198, 4161, 4124, 4086, 4049, 4010, 3972, 3933, 3894,
		3855, 3815, 3775, 3736, 3695, 3655, 3614, 3574, 3533, 3492, 3451, 3410, 3368, 3327, 3286, 3244,
		3203, 3161, 3120, 3078, 3037, 2995, 2954, 2912, 2871, 2830, 2789, 2748, 2707, 2666, 2625, 2585,
		2544, 2504, 2464, 2424, 2385, 2345, 2306, 2267, 2228, 2190, 2151, 2113, 2076, 2038, 2001, 1964,
		1928, 1891, 1855, 1820, 1785, 1750, 1715, 1681, 1647, 1613, 1580, 1547, 1515, 1483, 1451, 1419,
	},
	{
		1451, 1483, 1515, 1547, 1580, 1613, 1647, 1681, 1715, 1750, 1785, 1820, 1855, 1891, 1928, 1964,
		2001, 2038, 2076, 2113, 2151, 2190, 2228, 2267, 2306, 2345, 2385, 2424, 2464, 2504, 2544, 2585,
		2625, 2666, 2707, 2748, 2789, 2830, 2871, 2912, 2954, 2995, 3037, 3078, 3120, 3161, 3203, 3244,
		3286, 3327, 3368, 3410, 3451, 3492, 3533, 3574, 3614, 3655, 3695, 3736, 3775, 3815, 3855, 3894,
		3933, 3972, 4010, 4049, 4086, 4124, 4161, 4198, 4235, 4271, 4306, 4342, 4377, 4411, 4445, 4479,
		4512, 4544, 4576, 4608, 4639, 4670, 4700, 4729, 4758, 4786, 4814, 4841, 4868, 4894, 4919, 4943,
		4967, 4991, 5013, 5035, 5057, 5077, 5097, 5116, 5135, 5152, 5169, 5186, 5201, 5216, 5230, 5243,
		5256, 5268, 5279, 5289, 5298, 5307, 5315, 5322, 5328, 5334, 5339, 5343, 5346, 5348, 5350, 5351,
	},
	{
		   0,    1,    2,    2,    3,    4,    5,    6,    6,    7,    8,    9,   10,   11,   12,   13,
		  15,   16,   17,   18,   20,   21,   23,   25,   26,   28,   30,   32,   34,   36,   39,   41,
		  44,   46,   49,   52,   55,   58,   61,   65,   68,   72,   76,   80,   84,   88,   93,   98,
		 102,  107,  113,  118,  124,  130,  136,  142,  148,  155,  162,  169,  177,  184,  192,  200,
		 209,  217,  226,  236,  245,  255,  265,  275,  286,  297,  308,  319,  331,  343,  356,  369,
		 382,  395,  409,  423,  437,  452,  467,  483,  499,  515,  531,  548,  566,  583,  601,  620,
		 639,  658,  678,  697,  718,  739,  760,  781,  803,  826,  848,  871,  895,  919,  943,  968,
		 993, 1019, 1045, 1071, 1098, 1125, 1153, 1181, 1209, 1238, 1267, 1297, 1327, 1357, 1388, 1419,
	},
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
	m_state_port(0),
	m_event_head(0), m_event_count(0), m_int_state(false),
	m_wave_addr(0), m_wave_latch(0),
	m_record_inputs(false), m_record_in{ 0, 0 }, m_record_got(0), m_record_history{ { 0 }, { 0 } }, m_level_channel(0)
{
	std::fill(std::begin(m_regs), std::end(m_regs), 0);
	std::fill(std::begin(m_blocks), std::end(m_blocks), nullptr);
	std::fill(std::begin(m_events), std::end(m_events), 0);
}

void roland_sp_device::device_start()
{
	m_stream = stream_alloc(m_record_inputs ? 2 : 0, VOICES, clock(), m_record_inputs ? STREAM_SYNCHRONOUS : STREAM_DEFAULT_FLAGS);
	m_int_timer = timer_alloc(FUNC(roland_sp_device::update_int), this);

	for (int i = 0; i != VOICES; i++)
	{
		save_item(NAME(m_voices[i].end), i);
		save_item(NAME(m_voices[i].pitch), i);
		save_item(NAME(m_voices[i].loop_start), i);
		save_item(NAME(m_voices[i].loop_end), i);
		save_item(NAME(m_voices[i].level), i);
		save_item(NAME(m_voices[i].pos), i);
		save_item(NAME(m_voices[i].state), i);
		save_item(NAME(m_voices[i].on), i);
		save_item(NAME(m_voices[i].looped), i);
		save_item(NAME(m_voices[i].ended), i);
	}
	save_item(NAME(m_key));
	save_item(NAME(m_regs));
	save_item(NAME(m_sel_block));
	save_item(NAME(m_sel_voice));
	save_item(NAME(m_sel_word));
	save_item(NAME(m_state_port));
	save_item(NAME(m_events));
	save_item(NAME(m_event_head));
	save_item(NAME(m_event_count));
	save_item(NAME(m_int_state));
	save_item(NAME(m_wave_addr));
	save_item(NAME(m_wave_latch));
	save_item(NAME(m_record_in));
	save_item(NAME(m_level_channel));
}

void roland_sp_device::device_clock_changed()
{
	m_stream->set_sample_rate(clock());
}

void roland_sp_device::device_reset()
{
	for (voice &v : m_voices)
		v = voice();
	m_key[0] = m_key[1] = 0;
	m_sel_block = BLOCK_NONE;
	m_event_head = m_event_count = 0;
	m_int_state = false;
	m_int_cb(0);
}

void roland_sp_device::set_wave_block(unsigned block, u16 *words)
{
	if (m_stream)
		m_stream->update();
	m_blocks[block % WAVE_BLOCKS] = words;
}

u16 roland_sp_device::wave_data_r()
{
	u16 const data = m_wave_latch;
	m_wave_addr++;
	m_wave_latch = wave_word(m_wave_addr);
	return data;
}

void roland_sp_device::wave_store(u16 data, u16 mem_mask)
{
	m_stream->update();
	if (u16 *const w = wave_ptr(m_wave_addr))
		COMBINE_DATA(w);
}

void roland_sp_device::wave_data_w(u16 data, u16 mem_mask)
{
	wave_store(data, mem_mask);
	m_wave_addr++;
}

u32 roland_sp_device::state_r(const voice &v) const
{
	return (v.state & STATE_FLAGS) | (u32(v.pos >> (POS_FRACTION - 3)) & ~STATE_FLAGS);
}

void roland_sp_device::state_w(voice &v, u32 data)
{
	v.state = data & STATE_FLAGS;
	v.pos = s64(data & ~STATE_FLAGS) << (POS_FRACTION - 3);
}

u16 roland_sp_device::playback_r(const voice &v, int word) const
{
	switch (word)
	{
	case PLAY_END_LOW:      return v.end;
	case PLAY_END_HIGH:     return v.end >> 16;
	case PLAY_PITCH:        return v.pitch;
	case PLAY_LOOP_START_LOW:  return v.loop_start;
	case PLAY_LOOP_START_HIGH: return v.loop_start >> 16;
	case PLAY_LOOP_END_LOW:  return v.loop_end;
	case PLAY_LOOP_END_HIGH: return v.loop_end >> 16;
	case PLAY_UNKNOWN_E:    return 0; // C40E: writes change nothing audible
	default:                return 0;
	}
}

void roland_sp_device::playback_w(voice &v, int word, u16 data)
{
	switch (word)
	{
	case PLAY_END_LOW:      v.end = (v.end & 0xffff0000) | data; break;
	case PLAY_END_HIGH:     v.end = (v.end & 0x0000ffff) | (u32(data) << 16); break;
	case PLAY_PITCH:        v.pitch = data; break;
	case PLAY_LOOP_START_LOW:  v.loop_start = (v.loop_start & 0xffff0000) | data; break;
	case PLAY_LOOP_START_HIGH: v.loop_start = (v.loop_start & 0x0000ffff) | (u32(data) << 16); break;
	case PLAY_LOOP_END_LOW:  v.loop_end = (v.loop_end & 0xffff0000) | data; break;
	case PLAY_LOOP_END_HIGH: v.loop_end = (v.loop_end & 0x0000ffff) | (u32(data) << 16); break;
	case PLAY_UNKNOWN_E:    break; // C40E: writes change nothing audible
	default:                LOG("playback word %d = %04x\n", word, data); break;
	}
}

u16 roland_sp_device::level_r(const voice &v, int word) const
{
	switch (word)
	{
	case LEVEL_UNKNOWN_0: return 0; // C600: writes change nothing audible
	case LEVEL_LOW:       return v.level; // C602, the current level, reads back
	case LEVEL_HIGH:      return v.level >> 16; // C604, the target level
	case LEVEL_UNKNOWN_6: return 0; // C606: writes change nothing audible
	default:              return 0;
	}
}

void roland_sp_device::level_w(voice &v, int word, u16 data)
{
	switch (word)
	{
	case LEVEL_UNKNOWN_0: break; // C600: writes change nothing audible
	case LEVEL_LOW:       v.level = (v.level & 0xffff0000) | data; break; // C602, the current level
	case LEVEL_HIGH:      v.level = (v.level & 0x0000ffff) | (u32(data) << 16); break; // C604, the target level
	case LEVEL_UNKNOWN_6: break; // C606: writes change nothing audible
	default:              LOG("level word %d = %04x\n", word, data); break;
	}
}

void roland_sp_device::key_w(int base, u16 data)
{
	m_stream->update();
	u16 const rise = data & ~m_key[base >> 4];
	m_key[base >> 4] = data;
	for (int i = 0; i != 16; i++)
	{
		voice &v = m_voices[base + i];
		v.on = BIT(data, i);
		if (BIT(rise, i))
		{
			v.looped = v.ended = false;
			LOG("key on %d at %06x pitch %04x mode %02x flags %02x\n", base + i, u32(v.pos >> POS_FRACTION), v.pitch, v.end >> 24, v.state >> 24);
		}
	}
}

void roland_sp_device::event(int voice, u8 kind)
{
	if (m_event_count == EVENTS)
	{
		LOG("event queue full\n");
		return;
	}
	m_events[(m_event_head + m_event_count++) % EVENTS] = kind | voice;
	m_int_timer->adjust(attotime::zero);
}

TIMER_CALLBACK_MEMBER(roland_sp_device::update_int)
{
	bool const state = m_event_count != 0;
	if (state != m_int_state)
	{
		m_int_state = state;
		m_int_cb(state);
	}
}

u16 roland_sp_device::read(offs_t offset, u16 mem_mask)
{
	offset &= 0x3ff; // C800-CFFF mirror C000-C7FF
	if (offset >= 0x100)
	{
		if (!machine().side_effects_disabled())
		{
			m_stream->update();
			m_sel_block = BLOCK_STATE + (offset >> 8) - 1;
			m_sel_voice = (offset >> 3) & 0x1f;
			m_sel_word = offset & 7;
			if (m_sel_block == BLOCK_STATE)
				m_state_port = state_r(m_voices[m_sel_voice]);
		}
		return 0;
	}

	switch (offset)
	{
	case REG_STATUS: // bit 7 set while C01C reads voice 20's word, bit 6 voice 4's; bit 1 a completed wave store
		if (!machine().side_effects_disabled())
			m_level_channel ^= 1;
		return m_level_channel ? 0x0082 : 0x0042;
	case REG_EVENT: // the oldest event: the voice in bits 0-4, bit 5 set for the end event
		if (!machine().side_effects_disabled())
			m_stream->update();
		return m_event_count ? m_events[m_event_head] : 0;
	case REG_RECORD: // write only
		return m_regs[REG_RECORD];
	case REG_WAVE_STORE: // reads FFFF
		return 0xffff;
	case REG_WAVE_DATA: // the fetched word, then the next word fetched
		if (machine().side_effects_disabled())
			return m_wave_latch;
		return wave_data_r();
	case REG_WAVE_ADDR_HIGH: // not readable
		return 0;
	case REG_WAVE_ADDR_LOW: // not readable
		return 0;
	case REG_STATE_LOW:
		return m_state_port;
	case REG_STATE_HIGH:
		return m_state_port >> 16;
	case REG_PLAYBACK:
		return m_sel_block == BLOCK_PLAYBACK ? playback_r(m_voices[m_sel_voice], m_sel_word) : 0;
	case REG_ACK: // drops the oldest event
		if (!machine().side_effects_disabled() && m_event_count)
		{
			m_event_head = (m_event_head + 1) % EVENTS;
			m_event_count--;
			m_int_state = false;
			m_int_cb(0);
			m_int_timer->adjust(attotime::zero);
		}
		return 0;
	case REG_LEVEL:
		return m_sel_block == BLOCK_LEVEL ? level_r(m_voices[m_sel_voice], m_sel_word) : 0;
	case REG_INPUT: // the recorders' input words in turn, the top 13 bits
		return u16(m_record_in[m_level_channel ^ 1] >> 11);
	case REG_INPUT_TAP: // write only
		return m_regs[REG_INPUT_TAP];
	default:
		if (!machine().side_effects_disabled())
			LOG("read %04x\n", 0xc000 + offset * 2);
		return m_regs[offset & 0xf];
	}
}

void roland_sp_device::write(offs_t offset, u16 data, u16 mem_mask)
{
	offset &= 0x3ff; // C800-CFFF mirror C000-C7FF
	if (offset >= 0x100)
	{
		m_sel_block = BLOCK_STATE + (offset >> 8) - 1;
		m_sel_voice = (offset >> 3) & 0x1f;
		m_sel_word = offset & 7;
		return;
	}

	switch (offset)
	{
	case REG_KEY_LOW: // voices 0-15 keyed, a voice starting on a rising bit
		key_w(0, data);
		break;
	case REG_KEY_HIGH: // voices 16-31 keyed
		key_w(16, data);
		break;
	case REG_RECORD: // bit 0 lets voices 20 and 4 record, bit 1 takes serial input 2
		m_regs[REG_RECORD] = data;
		break;
	case REG_UNKNOWN_06: // 0060 during set-up, 0064 after; playback needs 0064
		m_regs[REG_UNKNOWN_06] = data;
		break;
	case REG_WAVE_STORE: // stores at the address, which does not move
		wave_store(data, mem_mask);
		break;
	case REG_WAVE_DATA: // advances, then stores; a byte access advances per byte and leaves the other lane undriven
		m_wave_addr++;
		wave_store(data, mem_mask);
		break;
	case REG_WAVE_ADDR_HIGH: // the wave port's address, bits 16-23; not readable
		m_wave_addr = (m_wave_addr & 0x0000ffff) | (data << 16);
		m_wave_latch = wave_word(m_wave_addr);
		break;
	case REG_WAVE_ADDR_LOW: // the wave port's address, bits 0-15; not readable
		m_wave_addr = (m_wave_addr & 0xffff0000) | data;
		m_wave_latch = wave_word(m_wave_addr);
		break;
	case REG_STATE_LOW:
		m_state_port = (m_state_port & 0xffff0000) | data;
		break;
	case REG_STATE_HIGH:
		m_state_port = (m_state_port & 0x0000ffff) | (data << 16);
		if (m_sel_block == BLOCK_STATE)
		{
			m_stream->update();
			state_w(m_voices[m_sel_voice], m_state_port);
		}
		break;
	case REG_PLAYBACK:
		if (m_sel_block == BLOCK_PLAYBACK)
		{
			m_stream->update();
			playback_w(m_voices[m_sel_voice], m_sel_word, data);
		}
		break;
	case REG_ACK: // written zero at set-up
		m_regs[REG_ACK] = data;
		break;
	case REG_UNKNOWN_18: // no firmware uses it; writes change nothing audible
		m_regs[REG_UNKNOWN_18] = data;
		break;
	case REG_LEVEL:
		if (m_sel_block == BLOCK_LEVEL)
		{
			m_stream->update();
			level_w(m_voices[m_sel_voice], m_sel_word, data);
		}
		break;
	case REG_INPUT: // the input filter's outer taps, signed bytes: the second and fourth high, the first and fifth low
		m_regs[REG_INPUT] = data;
		break;
	case REG_INPUT_TAP: // the input filter's centre tap, a signed low byte
		m_regs[REG_INPUT_TAP] = data;
		break;
	default:
		LOG("write %04x = %04x\n", 0xc000 + offset * 2, data);
		m_regs[offset & 0xf] = data;
		break;
	}
}

s32 roland_sp_device::step(int n, voice &v)
{
	s64 const adr = v.pos >> POS_FRACTION;
	int const phase = (v.pos >> (POS_FRACTION - 7)) & 0x7f;
	s32 const first = s16(wave_word(adr));
	s32 sum = 0;
	for (int i = 0; i != 3; i++)
		sum += (interp_weights[i][phase] * (s16(wave_word(adr + i + 1)) - first)) >> 12;
	s32 const sample = first + (sum >> 1);

	u32 const target = (v.level >> 16) & 0x1fff;
	u32 const level = ((3 * (target << 2) + (v.level & 0x7fff)) >> 2) >> 2;
	v.level = (v.level & 0xffff0000) | (target << 2);
	advance(n, v);
	return s32((4 * s64(sample) * level) >> 14);
}

void roland_sp_device::advance(int n, voice &v)
{
	u32 const end = v.end;
	s64 const loop_start = s64(v.loop_start) << (POS_FRACTION - 8);
	s64 const loop_end = s64(v.loop_end) << (POS_FRACTION - 8);
	s64 const pitch = v.pitch;

	auto looped = [&]()
	{
		if (!v.looped && !(v.state & STATE_NO_LOOP_EVENT))
		{
			v.looped = true;
			event(n, EVENT_LOOP);
		}
	};

	if (end & MODE_REVERSE)
	{
		v.pos -= pitch;
		if (v.pos <= loop_end)
		{
			looped();
			v.pos += loop_start - loop_end;
		}
	}
	else if (v.state & STATE_BACKWARD)
	{
		v.pos -= pitch;
		if (v.pos <= loop_start)
		{
			v.state &= ~STATE_BACKWARD;
			v.pos = 2 * loop_start - v.pos;
		}
	}
	else
	{
		v.pos += pitch;
		if (v.pos > loop_end)
		{
			looped();
			if (end & MODE_ALTERNATE)
			{
				v.state |= STATE_BACKWARD;
				v.pos = 2 * loop_end - v.pos;
			}
			else
				v.pos -= loop_end - loop_start;
		}
		if (!(v.state & STATE_NO_END_EVENT) && !v.ended && v.pos >= s64(end & END_ADDRESS) << POS_FRACTION)
		{
			v.ended = true;
			event(n, EVENT_END);
		}
	}
}

bool roland_sp_device::recording(int n) const
{
	return (m_regs[REG_RECORD] & RECORD_WRITE) && (n == RECORD_VOICES[0] || n == RECORD_VOICES[1]) && !(m_voices[n].level >> 16);
}

void roland_sp_device::record_w(offs_t offset, u32 data)
{
	if (offset > 1 || (m_record_inputs && !(m_regs[REG_RECORD] & RECORD_AUX)))
		return;
	m_record_in[offset] = util::sext(data, 24);
	m_record_got |= 1 << offset;
	if (m_record_got == 3)
	{
		m_record_got = 0;
		record();
	}
}

s16 roland_sp_device::filtered(int c)
{
	s16 *const h = m_record_history[c];
	std::copy(h + 1, h + 5, h);
	h[4] = s16(m_record_in[c] >> 8);
	s32 const a = s8(m_regs[REG_INPUT] >> 8), b = s8(m_regs[REG_INPUT]), m = s8(m_regs[REG_INPUT_TAP]);
	return s16(((b * h[0]) >> 6) + ((a * h[1]) >> 6) + ((m * h[2]) >> 6) + ((a * h[3]) >> 6) + ((b * h[4]) >> 6));
}

void roland_sp_device::record()
{
	for (int c = 0; c != 2; c++)
	{
		s16 const word = filtered(c);
		int const n = RECORD_VOICES[c];
		voice &v = m_voices[n];
		if (!v.on || !recording(n))
			continue;
		if (u16 *const w = wave_ptr(v.pos >> POS_FRACTION))
			*w = u16(word);
		advance(n, v);
	}
}

void roland_sp_device::sound_stream_update(sound_stream &stream)
{
	if (m_record_inputs && !(m_regs[REG_RECORD] & RECORD_AUX))
		for (int i = 0; i != stream.samples(); i++)
		{
			for (int c = 0; c != 2; c++)
				m_record_in[c] = std::clamp(s32(stream.get(c, i) * 0x800000), -0x800000, 0x7fffff);
			record();
		}
	for (int n = 0; n != VOICES; n++)
	{
		voice &v = m_voices[n];
		for (int i = 0; i != stream.samples(); i++)
			stream.put_int(n, i, v.on && !recording(n) ? step(n, v) : 0, 65536);
	}
}
