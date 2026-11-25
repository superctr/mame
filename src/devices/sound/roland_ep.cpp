// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Roland EP (Fujitsu MB87731A), the JD-800's and JD-990's PCM sample
    player.

    TODO:
    - the chip's own output stage, and its unity pitch in samples a second
    - the interrupt, taken as raised once a launch at the voice's first
      pass through its end
    - the gate word's bits 13-12, taken as a gain in eighths of the
      interpolator's sum (only 3 is ever written), and the registers the
      firmware writes at boot

***************************************************************************/

#include "emu.h"
#include "roland_ep.h"

#include "roland_pcm_interp.h"

#include <algorithm>
#include <bit>

#define LOG_REGS  (1U << 1)

#define VERBOSE (0)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(ROLAND_EP, roland_ep_device, "roland_ep", "Roland EP")

roland_ep_device::roland_ep_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, ROLAND_EP, tag, owner, clock)
	, device_memory_interface(mconfig, *this)
	, device_sound_interface(mconfig, *this)
	, m_wave_config("wave", ENDIANNESS_LITTLE, 8, 24, 0)
	, m_stream(nullptr)
	, m_irq_cb(*this)
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
	save_item(NAME(m_irq_pending));
	save_item(STRUCT_MEMBER(m_voices, regs));
	save_item(STRUCT_MEMBER(m_voices, address));
	save_item(STRUCT_MEMBER(m_voices, phase));
	save_item(STRUCT_MEMBER(m_voices, predictor));
	save_item(STRUCT_MEMBER(m_voices, backward));
	save_item(STRUCT_MEMBER(m_voices, running));
	save_item(STRUCT_MEMBER(m_voices, wrapped));
}

void roland_ep_device::device_reset()
{
	std::fill(std::begin(m_regs), std::end(m_regs), 0);
	m_key_mask = 0;
	m_irq_pending = 0;
	for (voice &v : m_voices)
		v = voice();
	m_irq_cb(CLEAR_LINE);
}

u16 roland_ep_device::read(offs_t offset)
{
	const int word = offset & (WORDS - 1);
	if (word == READ_DATA)
	{
		const u32 address = ((u32(m_regs[READ_HIGH]) << 16) | m_regs[READ_LOW]) >> 8;
		return m_wave.read_byte(address & ADDRESS_MASK);
	}
	if (word == IRQ_VOICE)
	{
		if (!machine().side_effects_disabled())
			m_stream->update();
		if (!m_irq_pending)
			return 0;
		const int n = std::countr_zero(m_irq_pending);
		if (!machine().side_effects_disabled())
		{
			m_irq_pending &= ~(1U << n);
			irq_update();
		}
		return n;
	}
	return 0;
}

void roland_ep_device::irq_update()
{
	m_irq_cb(m_irq_pending ? ASSERT_LINE : CLEAR_LINE);
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

void roland_ep_device::voice_restart(int n, u16 start_high, u16 start_low)
{
	m_stream->update();
	voice &v = m_voices[n & (VOICES - 1)];
	v.address = (u32((v.regs[WAVE] >> 4) & 0xf) << 20) | (u32(start_low) << 4) | (start_high >> 12);
	v.phase = (start_high & 0xfff) << 4;
	v.predictor = 0;
	v.backward = false;
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

u32 roland_ep_device::address_of(const voice &v, int high) const
{
	const u32 page = (v.regs[WAVE] >> 4) & 0xf;
	return (page << 20) | (u32(v.regs[high + 1]) << 4) | (v.regs[high] >> 12);
}

bool roland_ep_device::advance(const voice &v, u32 &address, bool &backward, wave_cell &c)
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
			return true;
		}
	}
	return false;
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
	return (c.exponent > 10) ? 0 : (p >> (10 - c.exponent));
}

void roland_ep_device::launch(int n)
{
	voice &v = m_voices[n];
	v.address = address_of(v, START_HIGH);
	v.phase = (v.regs[START_HIGH] & 0xfff) << 4;
	v.predictor = 0;
	v.backward = false;
	v.running = true;
	v.wrapped = false;
	if (BIT(m_irq_pending, n))
	{
		m_irq_pending &= ~(1U << n);
		irq_update();
	}
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
		sum += tap(roland_pcm_interp::WEIGHTS[i][v.phase >> 9], c);
	}
	const s32 sample = wrap20((sum * BIT(v.regs[GATE], 12, 2)) >> 3);

	const u32 accumulated = u32(v.phase) + (u32(v.regs[PITCH]) << 2);
	v.phase = accumulated & 0xffff;
	for (u32 carry = accumulated >> 16; carry; carry--)
	{
		if (advance(v, v.address, v.backward, c) && !v.wrapped)
		{
			v.wrapped = true;
			m_irq_pending |= 1U << n;
			irq_update();
		}
		v.predictor = wrap20(v.predictor + (c.mantissa << c.exponent));
	}
	return sample;
}

void roland_ep_device::sound_stream_update(sound_stream &stream)
{
	for (int i = 0; i < stream.samples(); i++)
	{
		for (int n = 0; n < VOICES; n++)
			stream.put_int(n, i, run_voice(n), 1 << 17);
	}
}
