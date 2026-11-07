// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Roland MEQ (Toshiba TC24SC220AF-007), the mixer, EQ and effects DSP of
    the JD-800, S-760 and SP-700.

    A fixed program of up to 400 steps runs once a sample: each step
    reads one data word and writes another, multiplies an operand by its
    coefficient and adds one of the last two results or the word read.
    24 voice words come in through their own steps; results leave through
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

enum : u16 {
	LO_SERIAL_LATCH = 0x0600, LO_SMOOTH = 0x0800, LO_DATA_COEF = 0x1000, LO_ABS = 0x2000,
	LO_R2 = 0x0100, LO_SATURATE = 0x0080, LO_SERIAL_IN = 0x0010,
	LO_TRANSFER = 0x0008, LO_TRANSFER_R1 = 0x0004, LO_OPERAND2 = 0x0001
};

enum : int { ADD_READ = 0, ADD_R2 = 1, ADD_R1 = 2, ADD_NONE = 3 };

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
	m_slot_table(nullptr),
	m_slots_per_voice(0)
{
}

void roland_meq_device::device_start()
{
	std::fill(std::begin(m_slot_of_step), std::end(m_slot_of_step), -1);
	if(m_slot_table)
		for(int v = 0; v != VOICES; v++)
			for(unsigned i = 0; i != m_slots_per_voice; i++)
				if(m_slot_table[v * m_slots_per_voice + i] < STEPS)
					m_slot_of_step[m_slot_table[v * m_slots_per_voice + i]] = v;

	m_stream = stream_alloc(VOICES + SERIAL_INPUTS * 2, PORTS * LATCHES, clock(), m_synchronous ? STREAM_SYNCHRONOUS : STREAM_DEFAULT_FLAGS);

	save_item(NAME(m_program));
	save_item(NAME(m_coef));
	save_item(NAME(m_data));
	save_item(NAME(m_latch));
	save_item(NAME(m_config));
	save_item(NAME(m_byte_hi));
	save_item(NAME(m_operand_l));
	save_item(NAME(m_operand_p));
	save_item(NAME(m_r1));
	save_item(NAME(m_r2));
	save_item(NAME(m_pipe));
	save_item(NAME(m_prev_read));
	save_item(NAME(m_pending_coef));
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
	m_operand_l = m_operand_p = m_r1 = m_r2 = 0;
	m_pipe[0] = m_pipe[1] = 0;
	m_prev_read = 0;
	m_pending_coef = -1;
	for(auto &port : m_port)
		std::fill(std::begin(port), std::end(port), 0);
}

u16 roland_meq_device::read(offs_t offset)
{
	switch(offset & 7) {
	case 0:
		return m_latch;
	case 1:
		return m_latch >> 16;
	default:
		return 0;
	}
}

void roland_meq_device::write(offs_t offset, u16 data)
{
	switch(offset & 7) {
	case 0:
		m_latch = (m_latch & 0xffff0000) | data;
		break;
	case 1:
		m_latch = (m_latch & 0x0000ffff) | (data << 16);
		break;
	case 2:
		if(data < 0x200)
			m_latch = data < STEPS ? m_program[data] : 0;
		else if(data < 0x400)
			m_latch = data - 0x200 < STEPS ? m_coef[data - 0x200] : 0;
		else
			m_latch = m_data[data & (DATA_WORDS - 1)] & 0xffffff;
		break;
	case 3:
		m_stream->update();
		if(data < 0x200) {
			if(data < STEPS)
				m_program[data] = m_latch;
		} else if(data < 0x400) {
			if(data - 0x200 < STEPS)
				m_coef[data - 0x200] = m_latch & 0x3fff;
		} else
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

u8 roland_meq_device::read8(offs_t offset)
{
	u16 const word = read(offset >> 1);
	return BIT(offset, 0) ? word : word >> 8;
}

void roland_meq_device::write8(offs_t offset, u8 data)
{
	if(!BIT(offset, 0))
		m_byte_hi[(offset >> 1) & 7] = data;
	else
		write(offset >> 1, (m_byte_hi[(offset >> 1) & 7] << 8) | data);
}

s32 roland_meq_device::multiply(s32 x, u16 coef) const
{
	static constexpr int SHIFTS[4] = { 0, 1, 2, 4 };
	s64 const m = util::sext(coef, 12);
	return s32(((s64(x) * m) << SHIFTS[(coef >> 12) & 3]) >> 11);
}

void roland_meq_device::run(const s32 *voices, const s32 (*serial)[2])
{
	int const length = std::min((m_config[0] & 0x1ff) + 3, int(STEPS));
	int latched[PORTS] = { 0, 0, 0 };
	int serial_reads[SERIAL_INPUTS] = { 0, 0 };

	for(int m = 0; m != length; m++) {
		u16 const hi = m_program[m] >> 16;
		u16 const lo = m_program[m];
		bool const writes = !BIT(hi, 15);
		bool const reads = !BIT(hi, 7);
		int const addend = (lo >> 5) & 3;

		s32 const written = m_pipe[0];
		if(writes)
			m_data[(hi >> 8) & 0x7f] = written;
		s32 const rd = reads ? m_data[hi & 0x7f] : 0;

		s32 x;
		if(lo & LO_OPERAND2) {
			int const v = addend == ADD_READ ? m_slot_of_step[m] : -1;
			x = v >= 0 ? voices[v] : m_operand_p;
		} else
			x = m_operand_l;

		s32 a;
		switch(addend) {
		case ADD_NONE: a = 0; break;
		case ADD_R1: a = m_r1; break;
		case ADD_R2: a = m_r2; break;
		default:
			if(lo & LO_OPERAND2)
				a = rd;
			else
				a = (lo & LO_R2) ? m_prev_read : m_r1;
			break;
		}

		if(lo & LO_SMOOTH) {
			a = x;
			x = rd - x;
		}

		u16 coef = m_coef[m];
		if(m_pending_coef >= 0) {
			coef = (coef & 0x3000) | m_pending_coef;
			m_pending_coef = -1;
		}
		if(lo & LO_DATA_COEF)
			m_pending_coef = (m_r1 >> 12) & 0xfff;

		s64 const sum = s64(a) + multiply(x, coef);
		s32 const result = ((lo & LO_SATURATE) && !(lo & LO_R2)) ? clamp24(sum) : wrap24(sum);

		if(lo & LO_SERIAL_LATCH) {
			int const port = ((lo & LO_SERIAL_LATCH) >> 9) - 1;
			if(port == 2)
				m_port3_cb(latched[port], u32(written) & 0xffffff);
			if(latched[port] < LATCHES)
				m_port[port][latched[port]++] = written;
		}

		m_pipe[0] = m_pipe[1];
		m_pipe[1] = result;
		if(reads)
			m_prev_read = rd;

		bool loaded = false;
		if(lo & LO_SERIAL_IN) {
			int const port = ((lo >> 2) & 3) - 1;
			if(port >= 0 && port < SERIAL_INPUTS)
				m_operand_l = m_operand_p = serial[port][serial_reads[port]++ & 1];
			else
				m_operand_l = m_operand_p = 0;
			loaded = true;
		} else if(reads && (addend != ADD_READ || !(lo & LO_OPERAND2))) {
			m_operand_l = m_operand_p = (lo & LO_ABS) ? std::abs(rd) : rd;
			loaded = true;
		}
		if((lo & LO_TRANSFER) && !(lo & LO_SERIAL_IN)) {
			if(lo & LO_TRANSFER_R1) {
				m_operand_p = m_r1;
				if(!loaded)
					m_operand_l = m_r1;
			} else {
				m_operand_p = m_r2;
				if(lo & LO_DATA_COEF)
					m_operand_l = m_r2;
			}
		}
		if(lo & LO_R2)
			m_r2 = m_r1;
		m_r1 = result;
	}
}

void roland_meq_device::sound_stream_update(sound_stream &stream)
{
	s32 voices[VOICES];
	s32 serial[SERIAL_INPUTS][2];
	for(int i = 0; i != stream.samples(); i++) {
		for(int v = 0; v != VOICES; v++)
			voices[v] = s32(std::clamp(stream.get(v, i), -1.0f, 1.0f) * 0x7fffff) & ~0x1f;
		for(int p = 0; p != SERIAL_INPUTS; p++) {
			s32 in[2];
			for(int c = 0; c != 2; c++)
				in[c] = s32(std::clamp(stream.get(INPUT_SERIAL + p * 2 + c, i), -1.0f, 1.0f) * 0x7fffff);
			serial[p][0] = in[m_serial_right_first[p] ? 1 : 0];
			serial[p][1] = in[m_serial_right_first[p] ? 0 : 1];
		}
		run(voices, serial);
		for(int p = 0; p != PORTS; p++)
			for(int l = 0; l != LATCHES; l++)
				stream.put_int(p * LATCHES + l, i, m_port[p][l], 0x800000);
	}
}
