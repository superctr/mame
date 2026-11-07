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
    its loop boundary and an end event at its end address, through a queue
    the host reads at C002 and acknowledges at C016.

    Not emulated: sampling, the interpolation method (linear here), the
    C6 registers.

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
	m_wave_size(0x200000),
	m_wave_words(0),
	m_wave_ptr(nullptr),
	m_key{ 0, 0 },
	m_sel_block(BLOCK_NONE), m_sel_voice(0), m_sel_word(0),
	m_c2_latch(0),
	m_event_head(0), m_event_count(0), m_int_state(false),
	m_host_bank(0), m_host_addr(0), m_host_latch(0)
{
	std::fill(std::begin(m_ctrl), std::end(m_ctrl), 0);
	std::fill(std::begin(m_events), std::end(m_events), 0);
}

void roland_sp_device::device_start()
{
	m_wave_words = m_wave_size / 2;
	memory_share *share = machine().memory().share_alloc(*this, "wave", 16, m_wave_size, ENDIANNESS_LITTLE);
	m_wave_ptr = reinterpret_cast<u16 *>(share->ptr());
	m_stream = stream_alloc(0, VOICES, clock());
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
	save_item(NAME(m_host_bank));
	save_item(NAME(m_host_addr));
	save_item(NAME(m_host_latch));
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

u16 roland_sp_device::host_read()
{
	u16 const data = m_host_latch;
	m_host_addr++;
	m_host_latch = m_host_bank == 1 ? m_wave_ptr[m_host_addr & (m_wave_words - 1)] : 0xffff;
	return data;
}

void roland_sp_device::host_write(u16 data, u16 mem_mask)
{
	m_stream->update();
	if(m_host_bank == 1)
		COMBINE_DATA(&m_wave_ptr[m_host_addr & (m_wave_words - 1)]);
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
		return 0x0002;
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
		break;
	case 0x07:
		m_host_addr = (m_host_addr & 0xffff0000) | data;
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
	s64 const adr = v.pos >> POS_FRACTION;
	s32 const frac = v.pos & ((1 << POS_FRACTION) - 1);
	s32 const s0 = s16(wave_word(adr));
	s32 const s1 = s16(wave_word(adr + 1));
	s32 const out = s0 + (((s1 - s0) * frac) >> POS_FRACTION);

	u32 const end = c4_long(v, C4_END);
	s64 const loop_to = s64(c4_long(v, C4_LOOP_TO)) << (POS_FRACTION - 8);
	s64 const loop_at = s64(c4_long(v, C4_LOOP_AT)) << (POS_FRACTION - 8);
	s64 const pitch = v.c4[C4_PITCH];

	auto looped = [&]() {
		if(!v.looped) {
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
		if(v.pos >= loop_at) {
			looped();
			if(end & MODE_ONE_SHOT)
				v.on = false;
			else if(end & MODE_ALTERNATE) {
				v.flags |= C2_BACKWARD;
				v.pos = 2 * loop_at - v.pos;
			} else
				v.pos -= loop_at - loop_to;
		}
		if(!(v.flags & C2_NO_END) && !v.ended && v.pos >= s64(end & 0x00ffffff) << POS_FRACTION) {
			v.ended = true;
			event(n, EVENT_END);
		}
	}
	return out;
}

void roland_sp_device::sound_stream_update(sound_stream &stream)
{
	for(int n = 0; n != VOICES; n++) {
		voice &v = m_voices[n];
		for(int i = 0; i != stream.samples(); i++)
			stream.put_int(n, i, v.on && !(v.flags & C2_IDLE) ? step(n, v) : 0, 32768);
	}
}
