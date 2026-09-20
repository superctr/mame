// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Roland TVF (Fujitsu MB87424A), the JD-800's and JD-990's filter and
    amplifier, thirty-two channels behind the EP.

    TODO:
    - the arithmetic widths and rounding are provisional
    - the other command values, the four zero pairs, and the readback's layout

***************************************************************************/

#include "emu.h"
#include "roland_tvf.h"

#define LOG_REGS  (1U << 1)

#define VERBOSE (0)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(ROLAND_TVF, roland_tvf_device, "roland_tvf", "Roland TVF")

roland_tvf_device::roland_tvf_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, ROLAND_TVF, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
	, m_stream(nullptr)
{
}

void roland_tvf_device::device_start()
{
	m_stream = stream_alloc(VOICES, VOICES, SAMPLE_RATE);

	save_item(NAME(m_regs));
	save_item(STRUCT_MEMBER(m_voices, regs));
	for (int i = 0; i < VOICES; i++)
	{
		save_item(NAME(m_voices[i].cutoff.value), i);
		save_item(NAME(m_voices[i].cutoff.start), i);
		save_item(NAME(m_voices[i].cutoff.target), i);
		save_item(NAME(m_voices[i].cutoff.remaining), i);
		save_item(NAME(m_voices[i].cutoff.length), i);
		save_item(NAME(m_voices[i].amplitude.value), i);
		save_item(NAME(m_voices[i].amplitude.start), i);
		save_item(NAME(m_voices[i].amplitude.target), i);
		save_item(NAME(m_voices[i].amplitude.remaining), i);
		save_item(NAME(m_voices[i].amplitude.length), i);
	}
	save_item(STRUCT_MEMBER(m_voices, low));
	save_item(STRUCT_MEMBER(m_voices, band));
}

void roland_tvf_device::device_reset()
{
	std::fill(std::begin(m_regs), std::end(m_regs), 0);
	for (voice &v : m_voices)
		v = voice();
}


//-------------------------------------------------
//  the window: the readback pair reads the selected channel's damping
//  and mode back in the chip's own byte order; everything else is a
//  channel's word behind the select, or a global
//-------------------------------------------------

u16 roland_tvf_device::read(offs_t offset)
{
	const int word = offset & (WORDS - 1);
	const voice &v = m_voices[m_regs[READBACK_SELECT] & (VOICES - 1)];
	switch (word)
	{
	case READBACK_LOW: return ((v.regs[DAMPING] & 0xff) << 8) | ((v.regs[FLAGS] >> 8) & 0x06);
	case READBACK_HIGH: return (v.regs[DAMPING] >> 8) & 0x1f;
	default: return 0;
	}
}

void roland_tvf_device::write(offs_t offset, u16 data)
{
	const int word = offset & (WORDS - 1);
	LOGMASKED(LOG_REGS, "%s: word %02X = %04X (voice %d)\n", machine().describe_context(), word, data, m_regs[SELECT] & (VOICES - 1));

	switch (word)
	{
	case KEY_HIGH:
	case KEY_LOW:
	case VOICES_IN_USE:
	case COMMAND:
	case SELECT:
	case READBACK_SELECT:
		m_regs[word] = data;
		return;

	default:
		m_stream->update();
		voice_w(m_regs[SELECT] & (VOICES - 1), word, data);
		return;
	}
}

void roland_tvf_device::voice_w(int n, int word, u16 data)
{
	voice &v = m_voices[n];
	v.regs[word] = data;
	switch (word)
	{
	case CUTOFF_INITIAL:
		v.cutoff.set(data, 0);
		break;

	case CUTOFF:
		v.cutoff.set(data, RAMP_SAMPLES);
		break;

	case AMPLITUDE:
		if (BIT(data, 15) || m_regs[COMMAND] == 0x0008)
			v.amplitude.set(data & 0x7fff, 0);
		else
			v.amplitude.set(data, m_regs[COMMAND] == 0x0007 ? 32 : RAMP_SAMPLES);
		break;
	}
}


//-------------------------------------------------
//  the channels
//-------------------------------------------------

void roland_tvf_device::ramp::set(u16 data, int samples)
{
	target = s32(data) << (COEFFICIENT_FRACTION_BITS - 14);
	start = value;
	length = samples;
	remaining = value == target ? 0 : samples;
	if (!remaining)
		value = target;
}

void roland_tvf_device::ramp::advance()
{
	if (remaining)
	{
		--remaining;
		value = start + s64(target - start) * (length - remaining) / length;
	}
}

void roland_tvf_device::service(voice &v)
{
	v.cutoff.advance();
	v.amplitude.advance();
}

s64 roland_tvf_device::rounded_shift(s64 value, int bits)
{
	const s64 half = s64(1) << (bits - 1);
	return value < 0 ? -((-value + half) >> bits) : (value + half) >> bits;
}

s32 roland_tvf_device::filter(voice &v, s64 sample) const
{
	const s32 f = v.cutoff.value;
	const s32 low = saturate(v.low + rounded_shift(s64(f) * v.band, COEFFICIENT_FRACTION_BITS));
	const s32 high = saturate(sample - low - rounded_shift(s64(v.regs[DAMPING]) * v.band, 12));
	v.low = low;
	v.band = saturate(v.band + rounded_shift(s64(f) * high, COEFFICIENT_FRACTION_BITS));
	switch (mode_of(v))
	{
	case MODE_HPF: return high;
	case MODE_BPF: return v.band;
	default: return low;
	}
}

// the pair's two channels through one of the structures; the flags of the
// first channel say which
std::pair<s32, s32> roland_tvf_device::pair(int n, s32 first, s32 second)
{
	voice &a = m_voices[n];
	voice &b = m_voices[n + 1];
	service(a);
	service(b);

	switch (structure_of(a))
	{
	case PAIR_SUM_THEN_FILTERS:
		return { 0, amplify(b, filter(b, filter(a, s64(first) + second))) };

	case PAIR_RING_THEN_FILTERS:
	{
		const s64 ring = rounded_shift(6 * s64(amplify(a, first)) * second, SAMPLE_FRACTION_BITS) + (mixes_second(a) ? second : 0);
		return { 0, amplify(b, filter(b, filter(a, ring))) };
	}

	case PAIR_FILTERS_THEN_RING:
	{
		const s32 filtered = filter(b, second);
		const s64 ring = rounded_shift(6 * s64(amplify(a, filter(a, first))) * filtered, SAMPLE_FRACTION_BITS) + (mixes_second(a) ? filtered : 0);
		return { 0, amplify(b, ring) };
	}

	default:
		return { amplify(a, filter(a, first)), amplify(b, filter(b, second)) };
	}
}

void roland_tvf_device::sound_stream_update(sound_stream &stream)
{
	for (int i = 0; i < stream.samples(); i++)
	{
		for (int n = 0; n < VOICES; n += 2)
		{
			const s32 input_a = s32(std::clamp(stream.get(n, i), -1.0f, 1.0f) * SAMPLE_ONE);
			const s32 input_b = s32(std::clamp(stream.get(n + 1, i), -1.0f, 1.0f) * SAMPLE_ONE);
			const auto [first, second] = pair(n, input_a, input_b);
			stream.put_int(n, i, first, SAMPLE_ONE);
			stream.put_int(n + 1, i, second, SAMPLE_ONE);
		}
	}
}
