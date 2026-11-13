// license:BSD-3-Clause
// copyright-holders:superctr
// thanks-to: giulioz
/***************************************************************************

    Roland MEQ (Toshiba TC24SC220AF-007), the mixer, EQ and effects DSP of
    the JD-800, S-760 and SP-700.

    A fixed program of up to 400 steps runs once a sample: each step
    reads one data word and writes another, multiplies the input the step
    before chose by its coefficient and adds a word read, a result or
    nothing.  24 voice words come in one a strobe; results leave through
    latches on three serial ports, exposed here as eight words a port in
    step order.

    Two serial input ports arrive as stream inputs after the voices, left
    then right: a port's first read in a sample takes the left word, its
    second the right.

    Not emulated: the clock-pin field.

***************************************************************************/

#include "emu.h"
#include "roland_meq.h"

#define VERBOSE (0)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(ROLAND_MEQ, roland_meq_device, "roland_meq", "Roland MEQ")

namespace {

enum : u16
{
	LO_SERIAL_LATCH = 0x0600, LO_RAMP = 0x0800, LO_R2 = 0x0100, LO_SATURATE = 0x0080,
	LO_STROBE = 0x0002, LO_READ_Q = 0x0001
};

enum : int { ADD_Q = 0, ADD_R2 = 1, ADD_RESULT = 2, ADD_NONE = 3 };
enum : int { SRC_VOICE = 0, SRC_READ = 1, SRC_R2 = 2, SRC_RESULT = 3, SRC_HALF = 4, SRC_SERIAL = 5 };
enum : int { MODE_DYNAMIC = 1, MODE_SIGN = 2 };

constexpr int input_of(u16 lo) { return (lo >> 2) & 7; }
constexpr int mode_of(u16 lo) { return (lo >> 12) & 3; }

constexpr s32 wrap24(s64 v) { return s32(util::sext(u32(v), 24)); }
constexpr s32 clamp24(s64 v) { return s32(std::clamp<s64>(v, -0x800000, 0x7fffff)); }

}

roland_meq_device::roland_meq_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock) :
	device_t(mconfig, ROLAND_MEQ, tag, owner, clock),
	device_sound_interface(mconfig, *this),
	m_port3_cb(*this),
	m_synchronous(false),
	m_serial_right_first{ false, false },
	m_stream(nullptr),
	m_first_voice(20),
	m_latch_steps{ nullptr, nullptr, nullptr }
{
}

void roland_meq_device::device_start()
{
	for (int port = 0; port != PORTS; port++)
	{
		std::fill(std::begin(m_latch_of_step[port]), std::end(m_latch_of_step[port]), -1);
		if (m_latch_steps[port])
			for (int n = 0; n != LATCHES; n++)
				if (m_latch_steps[port][n] < STEPS)
					m_latch_of_step[port][m_latch_steps[port][n]] = n;
	}

	m_stream = stream_alloc(VOICES + SERIAL_INPUTS * 2, PORTS * LATCHES, clock(), m_synchronous ? STREAM_SYNCHRONOUS : STREAM_DEFAULT_FLAGS);

	save_item(NAME(m_program));
	save_item(NAME(m_coef));
	save_item(NAME(m_data));
	save_item(NAME(m_latch));
	save_item(NAME(m_config));
	save_item(NAME(m_byte_hi));
	save_item(NAME(m_read_l));
	save_item(NAME(m_read_q));
	save_item(NAME(m_voice));
	save_item(NAME(m_voice_word));
	save_item(NAME(m_result));
	save_item(NAME(m_stored));
	save_item(NAME(m_r2));
	save_item(NAME(m_r2_before));
	save_item(NAME(m_port));
}

void roland_meq_device::device_clock_changed()
{
	m_stream->set_sample_rate(clock());
}

void roland_meq_device::device_reset()
{
	std::fill(std::begin(m_program), std::end(m_program), 0x80800000);
	std::fill(std::begin(m_coef), std::end(m_coef), 0);
	std::fill(std::begin(m_data), std::end(m_data), 0);
	m_latch = 0;
	m_config[0] = m_config[1] = 0;
	std::fill(std::begin(m_byte_hi), std::end(m_byte_hi), 0);
	m_read_l = m_read_q = m_voice = m_voice_word = 0;
	m_result = 0;
	m_stored = m_r2 = m_r2_before = 0;
	for (auto &port : m_port)
		std::fill(std::begin(port), std::end(port), 0);
}

u16 roland_meq_device::read(offs_t offset)
{
	return BIT(offset, 0) ? m_latch >> 16 : m_latch;
}

void roland_meq_device::write(offs_t offset, u16 data)
{
	switch (offset & 7)
	{
	case 0:
		m_latch = (m_latch & 0xffff0000) | data;
		break;
	case 1:
		m_latch = (m_latch & 0x0000ffff) | (data << 16);
		break;
	case 2:
		data &= 0x7ff;
		if (data < 0x200)
			m_latch = data < STEPS ? m_program[data] : 0xffffffff;
		else if (data < 0x400)
			m_latch = data - 0x200 < STEPS ? m_coef[data - 0x200] : 0xffffffff;
		else if (data < 0x600)
			m_latch = m_data[data & (DATA_WORDS - 1)] & 0xffffff;
		else
			m_latch = (data << 16) | data;
		break;
	case 3:
		m_stream->update();
		if (data < 0x200)
		{
			if (data < STEPS)
				m_program[data] = m_latch;
		}
		else if (data < 0x400)
		{
			if (data - 0x200 < STEPS)
				m_coef[data - 0x200] = m_latch;
		}
		else if (data < 0x600)
			m_data[data & (DATA_WORDS - 1)] = util::sext(m_latch, 24);
		break;
	case 4:
	case 5:
		m_stream->update();
		m_config[offset & 1] = data;
		break;
	default:
		LOG("write %x = %04x\n", offset, data);
		break;
	}
}

offs_t roland_meq_device::byte_register(offs_t offset)
{
	offs_t const word = (offset >> 1) & 7;
	return BIT(word, 1) ? word : word ^ 1;
}

u8 roland_meq_device::read8(offs_t offset)
{
	u16 const word = read(byte_register(offset));
	return BIT(offset, 0) ? word : word >> 8;
}

void roland_meq_device::write8(offs_t offset, u8 data)
{
	if (!BIT(offset, 0))
		m_byte_hi[(offset >> 1) & 7] = data;
	else
		write(byte_register(offset), (m_byte_hi[(offset >> 1) & 7] << 8) | data);
}

s64 roland_meq_device::multiply(s32 x, s32 mantissa, u16 coef)
{
	static constexpr int SHIFTS[4] = { 0, 1, 2, 4 };
	s64 const n = (s64(x) * mantissa) << SHIFTS[(coef >> 12) & 3];
	int const s = BIT(coef, 14) ? 22 : 11;
	if (BIT(coef, 15))
		return (n >> s) + (n < 0 ? 1 : 0);
	if (n < 0)
		return (n * 8 + (s64(3) << s)) >> (s + 3);
	return (n + (s64(1) << (s - 1))) >> s;
}

void roland_meq_device::run(const s32 *voices, const s32 (*serial)[2])
{
	int const length = std::min((m_config[0] & 0x1ff) + 3, int(STEPS));
	int latched[PORTS] = { 0, 0, 0 };
	int serial_reads[SERIAL_INPUTS] = { 0, 0 };
	int strobes = 0;
	for (int port = 0; port != PORTS; port++)
		if (m_latch_steps[port])
			std::fill(std::begin(m_port[port]), std::end(m_port[port]), 0);

	for (int m = 0; m != length; m++)
	{
		u16 const hi = m_program[m] >> 16;
		u16 const lo = m_program[m];
		u16 const prev = m_program[m ? m - 1 : length - 1];
		u16 const prev2 = m_program[m > 1 ? m - 2 : m + length - 2];
		bool const writes = !BIT(hi, 15);
		bool const reads = !BIT(hi, 7);
		int const addend = (lo >> 5) & 3;
		u16 const coef = m_coef[m];

		if (input_of(prev) == SRC_VOICE && (input_of(prev2) != SRC_VOICE || (prev & LO_STROBE)))
			m_voice = m_voice_word;
		if (lo & LO_STROBE)
			m_voice_word = voices[(strobes++ + m_first_voice) % VOICES];

		s64 f = (lo & LO_SATURATE) ? clamp24(m_result) : m_result;
		if (prev & LO_RAMP)
		{
			u32 const target = u32(m_read_q) & 0xffffff;
			u32 const t = (u32(f) << 4) & 0xffffff;
			bool const coarse = ((prev >> 5) & 3) == ADD_NONE;
			bool const falling = coarse || BIT(target, 11);
			if (t < 0x800000 && (falling ? t > (target & 0x7ff000) : t < (target & 0x7ff000)))
				f = coarse ? (t & 0xfff000) : t;
			else
				f = target & 0xfff000;
		}
		s32 const formatted = wrap24(f);

		s32 x;
		switch (input_of(prev))
		{
		case SRC_VOICE: x = m_voice; break;
		case SRC_READ: x = m_read_l; break;
		case SRC_R2: x = m_r2_before; break;
		case SRC_RESULT: x = m_stored; break;
		case SRC_HALF: x = 0x400000; break;
		default:
			{
				int const port = input_of(prev) - SRC_SERIAL;
				x = port < SERIAL_INPUTS ? serial[port][serial_reads[port]++ & 1] : 0;
			}
			break;
		}

		s32 mantissa;
		if (mode_of(prev) & MODE_DYNAMIC)
			mantissa = util::sext(u32(m_stored) >> 12, 12);
		else if (mode_of(prev) == MODE_SIGN)
			mantissa = (coef & 0x7ff) - (x < 0 ? 0x800 : 0);
		else
			mantissa = util::sext(coef, 12);
		s64 const product = multiply(x, mantissa, coef);

		if (reads && (lo & LO_READ_Q))
			m_read_q = m_data[hi & 0x7f];

		s64 a;
		switch (addend)
		{
		case ADD_NONE: a = 0; break;
		case ADD_RESULT: a = formatted; break;
		case ADD_R2: a = m_r2; break;
		default: a = (lo & LO_RAMP) ? util::sext(m_read_q, 12) : m_read_q; break;
		}
		s64 const result = a + product;

		m_r2_before = m_r2;
		if (lo & LO_R2)
			m_r2 = formatted;

		if (writes)
			m_data[(hi >> 8) & 0x7f] = m_stored;

		if (lo & LO_SERIAL_LATCH)
		{
			int const port = ((lo & LO_SERIAL_LATCH) >> 9) - 1;
			if (port == 2)
				m_port3_cb(latched[port], u32(m_stored) & 0xffffff);
			if (m_latch_steps[port])
			{
				if (m_latch_of_step[port][m] >= 0)
					m_port[port][m_latch_of_step[port][m]] = m_stored;
				latched[port]++;
			}
			else if (latched[port] < LATCHES)
				m_port[port][latched[port]++] = m_stored;
		}

		if (reads && !(lo & LO_READ_Q))
			m_read_l = m_data[hi & 0x7f];

		m_stored = formatted;
		m_result = result;
	}
}

void roland_meq_device::sound_stream_update(sound_stream &stream)
{
	s32 voices[VOICES];
	s32 serial[SERIAL_INPUTS][2];
	for (int i = 0; i != stream.samples(); i++)
	{
		for (int v = 0; v != VOICES; v++)
			voices[v] = s32(std::clamp(stream.get(v, i), -1.0f, 1.0f) * 0x7fffff) & ~0x1f;
		for (int p = 0; p != SERIAL_INPUTS; p++)
		{
			s32 in[2];
			for (int c = 0; c != 2; c++)
				in[c] = s32(std::clamp(stream.get(INPUT_SERIAL + p * 2 + c, i), -1.0f, 1.0f) * 0x7fffff);
			serial[p][0] = in[m_serial_right_first[p] ? 1 : 0];
			serial[p][1] = in[m_serial_right_first[p] ? 0 : 1];
		}
		run(voices, serial);
		for (int p = 0; p != PORTS; p++)
			for (int l = 0; l != LATCHES; l++)
				stream.put_int(p * LATCHES + l, i, m_port[p][l], 0x800000);
	}
}
