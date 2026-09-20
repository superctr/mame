// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Roland TVF (Fujitsu MB87424A), the JD-800's and JD-990's filter and
    amplifier, thirty-two channels behind the EP.

    TODO:
    - the filter, the ring modulator and the amplifier run in floating
      point here, where the chip is fixed point: its state widths, its
      rounding and where it saturates are all unread
    - the damping word's scale
    - the command register, the four zero pairs, and the readback's layout

***************************************************************************/

#include "emu.h"
#include "roland_tvf.h"

#define LOG_REGS  (1U << 1)

#define VERBOSE (0)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(ROLAND_TVF, roland_tvf_device, "roland_tvf", "Roland TVF filter and amplifier")

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
	save_item(STRUCT_MEMBER(m_voices, cutoff));
	save_item(STRUCT_MEMBER(m_voices, cutoff_step));
	save_item(STRUCT_MEMBER(m_voices, cutoff_remaining));
	save_item(STRUCT_MEMBER(m_voices, amplitude));
	save_item(STRUCT_MEMBER(m_voices, amplitude_step));
	save_item(STRUCT_MEMBER(m_voices, amplitude_remaining));
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
		v.cutoff = data / 16384.0f;
		v.cutoff_remaining = 0;
		break;

	case CUTOFF:
		v.cutoff_step = (data / 16384.0f - v.cutoff) / RAMP_SAMPLES;
		v.cutoff_remaining = RAMP_SAMPLES;
		break;

	case AMPLITUDE:
		if (BIT(data, 15))
		{
			v.amplitude = (data & 0x7fff) / 4096.0f;
			v.amplitude_remaining = 0;
		}
		else
		{
			v.amplitude_step = (data / 4096.0f - v.amplitude) / RAMP_SAMPLES;
			v.amplitude_remaining = RAMP_SAMPLES;
		}
		break;
	}
}


//-------------------------------------------------
//  the channels
//-------------------------------------------------

void roland_tvf_device::service(voice &v)
{
	if (v.cutoff_remaining)
	{
		v.cutoff += v.cutoff_step;
		v.cutoff_remaining--;
	}
	if (v.amplitude_remaining)
	{
		v.amplitude += v.amplitude_step;
		v.amplitude_remaining--;
	}
}

float roland_tvf_device::filter(voice &v, float sample) const
{
	const float f = v.cutoff;
	const float q = v.regs[DAMPING] / 4096.0f;
	const float low = std::clamp(v.low + f * v.band, -4.0f, 4.0f);
	const float high = sample - low - q * v.band;
	v.low = low;
	v.band = std::clamp(v.band + f * high, -4.0f, 4.0f);
	switch (mode_of(v))
	{
	case MODE_HPF: return high;
	case MODE_BPF: return v.band;
	default: return low;
	}
}

// the pair's two channels through one of the structures; the flags of the
// first channel say which
std::pair<float, float> roland_tvf_device::pair(int n, float first, float second)
{
	voice &a = m_voices[n];
	voice &b = m_voices[n + 1];
	service(a);
	service(b);

	switch (structure_of(a))
	{
	case PAIR_SUM_THEN_FILTERS:
		return { 0, amplify(b, filter(b, filter(a, first + second))) };

	case PAIR_RING_THEN_FILTERS:
	{
		const float ring = amplify(a, first) * second + (mixes_second(a) ? second : 0.0f);
		return { 0, amplify(b, filter(b, filter(a, ring))) };
	}

	case PAIR_FILTERS_THEN_RING:
	{
		const float filtered = filter(b, second);
		const float ring = amplify(a, filter(a, first)) * filtered + (mixes_second(a) ? filtered : 0.0f);
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
			const auto [first, second] = pair(n, stream.get(n, i), stream.get(n + 1, i));
			stream.put(n, i, first * 0.25f);
			stream.put(n + 1, i, second * 0.25f);
		}
	}
}
