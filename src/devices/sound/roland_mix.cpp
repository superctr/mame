// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Roland MIX (Toshiba TC23SC100AF-502), the output mixer and assigner of
    the S-770 and S-750.

    Each of 32 voices has one word: a 6-bit gain to the right and to the
    left of the stereo pair, and a 4-bit assign that sends the voice at
    full level to one of the six individual outputs, or to none.  With the
    stereo pair turned off its two outputs take two more assigns.  The
    host selects a voice by writing its select register and then writes
    the word to the data register.  The serial output carries the stereo
    pair, or with control bit 7 the serial input's words.

    Not emulated: the meaning of the remaining control bits.

***************************************************************************/

#include "emu.h"
#include "roland_mix.h"

DEFINE_DEVICE_TYPE(ROLAND_MIX, roland_mix_device, "roland_mix", "Roland MIX")

namespace {

enum : u16 { CONTROL_STEREO_OFF = 0x0040, CONTROL_THROUGH = 0x0080, CONTROL_RUN = 0x0100 };

constexpr s32 FULL_SCALE = 0x7ffff;

}

roland_mix_device::roland_mix_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, ROLAND_MIX, tag, owner, clock),
	device_sound_interface(mconfig, *this),
	m_serial_cb(*this),
	m_stream(nullptr)
{
}

void roland_mix_device::device_start()
{
	m_stream = stream_alloc(VOICES + 2, OUTPUTS + 2, clock());

	save_item(NAME(m_regs));
	save_item(NAME(m_voice));
	save_item(NAME(m_select));
}

void roland_mix_device::device_reset()
{
	std::fill(std::begin(m_regs), std::end(m_regs), 0);
	std::fill(std::begin(m_voice), std::end(m_voice), 0x0001);
	m_select = 0;
}

void roland_mix_device::device_clock_changed()
{
	m_stream->set_sample_rate(clock());
}

u16 roland_mix_device::read(offs_t offset)
{
	return m_regs[offset & (WORDS - 1)];
}

void roland_mix_device::write(offs_t offset, u16 data)
{
	offset &= WORDS - 1;
	m_stream->update();
	m_regs[offset] = data;
	if (offset >= REG_SELECT)
		m_select = offset - REG_SELECT;
	else if (offset == REG_DATA)
		m_voice[m_select] = data;
}

void roland_mix_device::sound_stream_update(sound_stream &stream)
{
	u16 const control = m_regs[REG_CONTROL];
	if (!(control & CONTROL_RUN))
		return;
	bool const stereo = !(control & CONTROL_STEREO_OFF);

	for (int i = 0; i != stream.samples(); i++)
	{
		s64 out[OUTPUTS] = { 0 };
		for (int v = 0; v != VOICES; v++)
		{
			u16 const word = m_voice[v];
			s32 const in = s32(std::clamp(stream.get(v, i), -1.0f, 1.0f) * FULL_SCALE);
			if (!in)
				continue;
			if (stereo)
			{
				out[OUTPUT_LEFT] += s64(in) * BIT(word, 4, 6);
				out[OUTPUT_RIGHT] += s64(in) * BIT(word, 10, 6);
			}
			int const assign = BIT(word, 1, 3);
			if (!BIT(word, 0) && (assign >= OUTPUT_INDIVIDUAL || !stereo))
				out[assign] += s64(in) << 6;
		}
		s32 word[OUTPUTS];
		for (int o = 0; o != OUTPUTS; o++)
		{
			word[o] = s32(std::clamp<s64>(out[o] >> 6, -FULL_SCALE - 1, FULL_SCALE));
			stream.put_int(o, i, word[o], FULL_SCALE + 1);
		}
		s32 serial[2];
		for (int c = 0; c != 2; c++)
		{
			serial[c] = (control & CONTROL_THROUGH)
					? std::clamp(s32(stream.get(INPUT_SERIAL + c, i) * 0x800000), -0x800000, 0x7fffff)
					: word[OUTPUT_LEFT + c] << 4;
			stream.put_int(OUTPUT_SERIAL + c, i, serial[c], 0x800000);
		}
		m_serial_cb(0, u32(serial[1]) & 0xffffff);
		m_serial_cb(1, u32(serial[0]) & 0xffffff);
	}
}
