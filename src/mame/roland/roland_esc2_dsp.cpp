// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Roland ESC2 (MB8AA4181) DSP

    Two units, each running one program once per sample frame.  The host
    sees both through one window: program memory, parameter targets, delay
    descriptors, a block of direct words per unit, a field port that patches
    program operands, and a port into the external sample memory.

***************************************************************************/

#include "emu.h"
#include "roland_esc2_dsp.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
#include <numbers>

#define LOG_UNSUPPORTED (1U << 1)

#define VERBOSE (LOG_UNSUPPORTED)
#include "logmacro.h"

#define LOGUNSUPPORTED(...) LOGMASKED(LOG_UNSUPPORTED, __VA_ARGS__)


DEFINE_DEVICE_TYPE(MB8AA4181_DSP, mb8aa4181_dsp_device, "mb8aa4181_dsp", "Roland ESC2 DSP")

namespace {

constexpr offs_t FRAME_COUNT = 0x048 / 4;
constexpr offs_t MEMORY_ADDRESS = 0x050 / 4;
constexpr offs_t SWITCH_REQUEST[2] = { 0x054 / 4, 0x05c / 4 };
constexpr offs_t FLAGS = 0x600 / 4;
constexpr offs_t NOTIFY_STATUS = 0x640 / 4;
constexpr offs_t TARGET_STATUS = 0x648 / 4;
constexpr offs_t SWITCH_STATUS[2] = { 0x658 / 4, 0x654 / 4 };
constexpr offs_t MEMORY_DATA = 0x800 / 4;
constexpr offs_t FIELD_DESCRIPTOR = 0x900 / 4;
constexpr offs_t FIELD_DATA = 0x904 / 4;
constexpr offs_t DIRECT_BASE = 0x1000 / 4;
constexpr offs_t UNIT_BASE = 0x40000 / 4;
constexpr offs_t UNIT_STRIDE = 0x20000 / 4;
constexpr offs_t UNIT_TARGETS = 0xc000 / 4;
constexpr offs_t UNIT_DRP = 0xe000 / 4;
constexpr offs_t UNIT_PRG = 0x10000 / 4;

constexpr u32 SWITCH_GO = 1 << 30;
constexpr u32 TARGET_NOTIFY = 1 << 28;
constexpr u32 FIELD_NUMERIC = 1 << 30;

constexpr int NO_TRANSFER = -1;
constexpr int RETURN = -2;

double wrap(double x)
{
	return std::fmod(std::fmod(x + 1.0, 2.0) + 2.0, 2.0) - 1.0;
}

double noise(double x)
{
	s64 word = s64(x * 16777216.0);
	if (!word)
		word = 1;
	else
		word = (word << 1) | (((word >> 23) ^ (word >> 21) ^ 1) & 1);
	word &= 0x1ffffff;
	if (word & 0x1000000)
		word -= 0x2000000;
	return double(word) / 16777216.0;
}

} // anonymous namespace


mb8aa4181_dsp_device::mb8aa4181_dsp_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, MB8AA4181_DSP, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
	, m_irq_cb(*this)
	, m_stream(nullptr)
{
}

void mb8aa4181_dsp_device::device_start()
{
	std::fill_n(&m_layout[0][0], 256 * (MAX_OPS + 1), 0);
	for (int i = 0; i < 5; i++)
		m_layout[8 + i][0] = "ABCDE"[i];
	for (int count = 2; count <= 5; count++)
	{
		int shape[5] = { 0, 0, 0, 0, 0 };
		int tag = 1 << (count + 2);
		for (;;)
		{
			int ab = 0, d = 0;
			for (int i = 0; i < count; i++)
			{
				ab += shape[i] < 2;
				d += shape[i] == 3;
			}
			if (ab <= 2 || (ab == 3 && d))
			{
				const int code = tag >= 0xc8 ? tag + 8 : tag;
				for (int i = 0; i < count; i++)
					m_layout[code][i] = "ABCDE"[shape[i]];
				tag++;
			}
			int i = count - 1;
			while (i >= 0 && shape[i] == 4)
				i--;
			if (i < 0)
				break;
			shape[i]++;
			for (int j = i + 1; j < count; j++)
				shape[j] = shape[i];
		}
	}

	m_unit = std::make_unique<unit_state[]>(UNITS);
	m_memory = std::make_unique<u32[]>(MEMORY_CELLS);
	std::fill_n(m_memory.get(), MEMORY_CELLS, 0);

	m_stream = stream_alloc(0, 2, clock() / 256);
	m_frame_timer = timer_alloc(FUNC(mb8aa4181_dsp_device::frame), this);
	for (unsigned i = 0; i < UNITS; i++)
		for (unsigned j = 0; j < 16; j++)
			m_notify_timer[i][j] = timer_alloc(FUNC(mb8aa4181_dsp_device::notify), this);
	m_packet_clock = owner()->clock();

	for (unsigned i = 0; i < UNITS; i++)
	{
		unit_state &u = m_unit[i];
		u.index.resize(PRG_HALFWORDS);
		save_item(NAME(u.prg), i);
		save_item(NAME(u.direct), i);
		save_item(NAME(u.target), i);
		save_item(NAME(u.drp), i);
		save_item(NAME(u.r), i);
		save_item(NAME(u.sel), i);
		save_item(NAME(u.shortmem), i);
		save_item(NAME(u.local), i);
		save_item(NAME(u.bus), i);
		save_item(NAME(u.slot), i);
		save_item(NAME(u.requested), i);
		save_item(NAME(u.slot_valid), i);
		save_item(NAME(u.requested_valid), i);
		save_item(NAME(u.slot_base), i);
		save_item(NAME(u.sample), i);
		save_item(NAME(u.origin), i);
		save_item(NAME(u.arith.last), i);
		save_item(NAME(u.arith.visible), i);
		save_item(NAME(u.arith.has_last), i);
		save_item(NAME(u.arith.has_visible), i);
		save_item(NAME(u.comp.last), i);
		save_item(NAME(u.comp.visible), i);
		save_item(NAME(u.comp.has_last), i);
		save_item(NAME(u.comp.has_visible), i);
		save_item(NAME(u.func.last), i);
		save_item(NAME(u.func.visible), i);
		save_item(NAME(u.func.has_last), i);
		save_item(NAME(u.func.has_visible), i);
		save_item(NAME(u.arith_published), i);
		save_item(NAME(u.has_arith_published), i);
		save_item(NAME(u.comparison_published), i);
		save_item(NAME(u.arrival), i);
		save_item(NAME(u.arrival_last), i);
		save_item(NAME(u.arrival_visible), i);
		save_item(NAME(u.target_queued), i);
		save_item(NAME(u.target_pending), i);
		save_item(NAME(u.notify_pending), i);
	}
	save_pointer(NAME(m_memory), MEMORY_CELLS);
	save_item(NAME(m_shared));
	save_item(NAME(m_port));
	save_item(NAME(m_field));
	save_item(NAME(m_flags));
	save_item(NAME(m_frame_start));
	save_item(NAME(m_frames));
	save_item(NAME(m_switch_queued));
	save_item(NAME(m_switch_done));
}

void mb8aa4181_dsp_device::device_reset()
{
	m_regs.clear();
	std::fill_n(m_shared, 0x100, 0.0);
	std::fill_n(m_port, 0x100, 0.0);
	m_field = 0;
	m_flags = 0;
	m_frames = 0;
	m_switch_queued = m_switch_done = 0;
	m_output_read = m_output_write = 0;
	m_last[0] = m_last[1] = 0.0f;

	for (unsigned i = 0; i < UNITS; i++)
	{
		unit_state &u = m_unit[i];
		std::fill_n(u.prg, PRG_WORDS, 0);
		std::fill_n(u.direct, DIRECT_WORDS, 0);
		std::fill_n(&u.target[0][0], TARGETS * 4, 0);
		std::fill_n(&u.drp[0][0], DRP_ENTRIES * 2, 0);
		std::fill_n(u.r, 8, 0.0);
		std::fill_n(u.sel, 4, 0.0);
		std::fill_n(u.shortmem, SHORT_WORDS, 0.0);
		std::fill_n(u.local, SHORT_WORDS, 0.0);
		std::fill_n(u.bus, BUS_WORDS, 0.0);
		std::fill_n(u.slot, SLOTS, 0.0);
		std::fill_n(u.requested, SLOTS, 0.0);
		std::fill_n(u.slot_valid, SLOTS, 0);
		std::fill_n(u.requested_valid, SLOTS, 0);
		std::fill_n(u.slot_base, SLOTS / 4, -1);
		std::fill_n(u.sample, SAMPLE_OWNERS * 16, 0.0f);
		u.origin = 0;
		u.arith.reset();
		u.comp.reset();
		u.func.reset();
		u.arith_published = 0.0;
		u.has_arith_published = false;
		u.comparison_published = false;
		u.arrival = u.arrival_last = u.arrival_visible = false;
		u.depth = 0;
		u.dirty = true;
		u.drp_dirty = true;
		u.samples.clear();
		u.direct_log.clear();
		u.packets_run = 0;
		u.pending.clear();
		std::fill_n(u.target_queued, TARGETS / 32, 0);
		std::fill_n(u.target_pending, TARGETS / 32, 0);
		u.notify_pending = 0;
		irq_update(i);
		m_irq_cb[IRQ_SWITCH0 + i](CLEAR_LINE);
	}

	const attotime period = attotime::from_hz(clock() / 256);
	m_frame_timer->adjust(period, 0, period);
}

void mb8aa4181_dsp_device::device_post_load()
{
	for (unsigned i = 0; i < UNITS; i++)
	{
		m_unit[i].dirty = true;
		m_unit[i].drp_dirty = true;
		m_unit[i].samples.clear();
		m_unit[i].direct_log.clear();
		m_unit[i].pending.clear();
		for (unsigned slot = 0; slot < SLOTS; slot++)
			if (m_unit[i].requested_valid[slot])
				m_unit[i].pending.push_back(slot);
	}
}


//  Host window

void mb8aa4181_dsp_device::irq_update(unsigned unit)
{
	unit_state &u = m_unit[unit];
	m_irq_cb[IRQ_NOTIFY0 + unit](u.notify_pending ? ASSERT_LINE : CLEAR_LINE);
	m_irq_cb[IRQ_TARGET0 + unit](target_next(unit) >= 0 ? ASSERT_LINE : CLEAR_LINE);
}

int mb8aa4181_dsp_device::target_next(unsigned unit) const
{
	const unit_state &u = m_unit[unit];
	for (unsigned i = 0; i < TARGETS / 32; i++)
		if (u.target_pending[i])
			return i * 32 + std::countr_zero(u.target_pending[i]);
	return -1;
}

u32 mb8aa4181_dsp_device::read(offs_t offset, u32 mem_mask)
{
	if (offset == FRAME_COUNT)
		return m_frames;
	if (offset == NOTIFY_STATUS || offset == NOTIFY_STATUS + 1)
	{
		const u16 pending = m_unit[offset - NOTIFY_STATUS].notify_pending;
		return pending ? u32(std::countr_zero(pending)) << 24 : 0;
	}
	if (offset == TARGET_STATUS || offset == TARGET_STATUS + 1)
	{
		const int index = target_next(offset - TARGET_STATUS);
		return index >= 0 ? u32(index) << 4 : 0;
	}
	for (unsigned unit = 0; unit < UNITS; unit++)
		if (offset == SWITCH_STATUS[unit])
			return BIT(m_switch_done, unit);
	if (offset == MEMORY_DATA)
	{
		const auto it = m_regs.find(MEMORY_ADDRESS);
		return m_memory[((it != m_regs.end() ? it->second : 0) >> 2) & (MEMORY_CELLS - 1)];
	}
	if (offset == FIELD_DATA)
		return field_read();
	return host_read(offset);
}

void mb8aa4181_dsp_device::write(offs_t offset, u32 data, u32 mem_mask)
{
	if (offset == 0x0c / 4)
		return;
	if (offset == NOTIFY_STATUS || offset == NOTIFY_STATUS + 1)
	{
		const unsigned unit = offset - NOTIFY_STATUS;
		unit_state &u = m_unit[unit];
		if (u.notify_pending)
			u.notify_pending &= u.notify_pending - 1;
		irq_update(unit);
		return;
	}
	if (offset == TARGET_STATUS || offset == TARGET_STATUS + 1)
	{
		const unsigned unit = offset - TARGET_STATUS;
		const int index = target_next(unit);
		if (index >= 0)
			m_unit[unit].target_pending[index >> 5] &= ~(1U << (index & 31));
		irq_update(unit);
		return;
	}
	for (unsigned unit = 0; unit < UNITS; unit++)
	{
		if (offset == SWITCH_STATUS[unit])
		{
			m_switch_done &= ~(1 << unit);
			m_irq_cb[IRQ_SWITCH0 + unit](CLEAR_LINE);
			return;
		}
	}
	if (offset == MEMORY_DATA)
	{
		const auto it = m_regs.find(MEMORY_ADDRESS);
		COMBINE_DATA(&m_memory[((it != m_regs.end() ? it->second : 0) >> 2) & (MEMORY_CELLS - 1)]);
		return;
	}
	if (offset == FIELD_DATA)
	{
		field_write(data);
		return;
	}
	if (offset == FIELD_DESCRIPTOR)
		COMBINE_DATA(&m_field);

	host_write(offset, data, mem_mask);

	for (unsigned unit = 0; unit < UNITS; unit++)
		if (offset == SWITCH_REQUEST[unit] && (data & mem_mask & SWITCH_GO))
			m_switch_queued |= 1 << unit;
}

u32 mb8aa4181_dsp_device::host_read(offs_t offset) const
{
	if (offset >= DIRECT_BASE && offset < DIRECT_BASE + UNITS * DIRECT_WORDS)
	{
		const unit_state &u = m_unit[(offset - DIRECT_BASE) / DIRECT_WORDS];
		const unsigned index = offset & (DIRECT_WORDS - 1);
		if (!u.direct_log.empty())
		{
			const u64 elapsed = (machine().time() - m_frame_start).as_ticks(m_packet_clock);
			for (const direct_write &w : u.direct_log)
				if (w.index == index && w.packet > elapsed)
					return w.before;
		}
		return u.direct[index];
	}
	if (offset >= UNIT_BASE && offset < UNIT_BASE + UNITS * UNIT_STRIDE)
	{
		const unit_state &u = m_unit[(offset - UNIT_BASE) / UNIT_STRIDE];
		const offs_t local = (offset - UNIT_BASE) % UNIT_STRIDE;
		if (local >= UNIT_PRG)
			return u.prg[local - UNIT_PRG];
		if (local >= UNIT_DRP)
			return u.drp[(local - UNIT_DRP) >> 1][local & 1];
		if (local >= UNIT_TARGETS)
			return u.target[(local - UNIT_TARGETS) >> 2][local & 3];
	}
	const auto it = m_regs.find(offset);
	return it != m_regs.end() ? it->second : 0;
}

void mb8aa4181_dsp_device::host_write(offs_t offset, u32 data, u32 mem_mask)
{
	if (offset >= DIRECT_BASE && offset < DIRECT_BASE + UNITS * DIRECT_WORDS)
	{
		COMBINE_DATA(&m_unit[(offset - DIRECT_BASE) / DIRECT_WORDS].direct[offset & (DIRECT_WORDS - 1)]);
		return;
	}
	if (offset >= UNIT_BASE && offset < UNIT_BASE + UNITS * UNIT_STRIDE)
	{
		const unsigned unit = (offset - UNIT_BASE) / UNIT_STRIDE;
		unit_state &u = m_unit[unit];
		const offs_t local = (offset - UNIT_BASE) % UNIT_STRIDE;
		if (local >= UNIT_PRG)
		{
			COMBINE_DATA(&u.prg[local - UNIT_PRG]);
			u.dirty = true;
			return;
		}
		if (local >= UNIT_DRP)
		{
			COMBINE_DATA(&u.drp[(local - UNIT_DRP) >> 1][local & 1]);
			u.drp_dirty = true;
			return;
		}
		if (local >= UNIT_TARGETS)
		{
			const unsigned index = (local - UNIT_TARGETS) >> 2;
			u32 *const target = u.target[index];
			COMBINE_DATA(&target[local & 3]);
			if ((local & 3) == 1)
			{
				target[2] = target[1];
				if (target[0] & TARGET_NOTIFY)
					u.target_queued[index >> 5] |= 1U << (index & 31);
			}
			return;
		}
	}
	COMBINE_DATA(&m_regs[offset]);
	if (offset == FLAGS)
		m_flags = m_regs[offset];
}

u32 mb8aa4181_dsp_device::native_number(u32 data, unsigned bits)
{
	const double value = std::bit_cast<float>(data);
	const double magnitude = std::fabs(value);
	unsigned scale = 0;
	while (scale < 3 && magnitude >= std::ldexp(1.0, 4 * scale - 3))
		scale++;
	const u64 limit = (u64(1) << (bits - 3)) - 1;
	const u64 m = std::min<u64>(limit, u64(std::ldexp(magnitude, bits - 4 * scale)));
	return (value < 0.0 ? u32(1) << (bits - 1) : 0) | (scale << (bits - 3)) | u32(m);
}

void mb8aa4181_dsp_device::field_write(u32 data)
{
	const unsigned width = ((m_field >> 28) & 3) + 2;
	u32 value = data;
	if (m_field & FIELD_NUMERIC)
		value = native_number(data, 8 * width) << (32 - 8 * width);
	const offs_t base = m_field & 0xfffff;
	for (unsigned i = 0; i < width; i++)
	{
		const offs_t address = base + i;
		const unsigned shift = 8 * (3 - (address & 3));
		host_write(address >> 2, u32((value >> (24 - 8 * i)) & 0xff) << shift, u32(0xff) << shift);
	}
}

u32 mb8aa4181_dsp_device::field_read() const
{
	const unsigned width = ((m_field >> 28) & 3) + 2;
	const offs_t base = m_field & 0xfffff;
	u32 value = 0;
	for (unsigned i = 0; i < width; i++)
	{
		const offs_t address = base + i;
		value |= ((host_read(address >> 2) >> (8 * (3 - (address & 3)))) & 0xff) << (24 - 8 * i);
	}
	return value;
}


//  Program decoding

u16 mb8aa4181_dsp_device::halfword(const unit_state &u, u32 pc) const
{
	const u32 word = u.prg[(pc >> 2) & (PRG_WORDS - 1)];
	return (pc & 2) ? u16(word) : u16(word >> 16);
}

void mb8aa4181_dsp_device::decode(unit_state &u)
{
	u.packets.clear();
	std::fill(u.index.begin(), u.index.end(), -2);
	std::vector<unsigned> zeros;
	unsigned h = 0;
	while (h < PRG_HALFWORDS)
	{
		const u16 header = halfword(u, h * 2);
		const unsigned length = header & 0x1f;
		if (!header || (header & 0x20) || !length || (header < 0x800 && header != 1) || h + length > PRG_HALFWORDS)
		{
			zeros.push_back(h);
			h++;
			continue;
		}

		packet p;
		p.pc = h * 2;
		p.length = length;
		p.count = 0;
		p.selectors = 0;
		p.extension_length = 0;
		if (header != 1)
		{
			const char *const shape = m_layout[header >> 8];
			unsigned count = strlen(shape);
			unsigned extra = (header >> 6) & 3;
			unsigned aux = std::count(shape, shape + count, 'A');
			if (!count || 1 + count + extra + aux > length)
			{
				zeros.push_back(h);
				h++;
				continue;
			}
			unsigned a = 0;
			for (unsigned i = 0; i < count + extra; i++)
			{
				operation &op = p.ops[i];
				op.kind = i < count ? u8(strchr("ABCDE", shape[i]) - "ABCDE") : KIND_E;
				op.word = halfword(u, (h + 1 + i) * 2);
				op.aux = op.kind == KIND_A ? halfword(u, (h + 1 + count + extra + a++) * 2) : 0;
			}
			p.count = count + extra;
			p.selectors = 0;
			for (unsigned i = 1 + count + extra; i < length; i++)
			{
				const u16 word = halfword(u, (h + i) * 2);
				p.extension[p.extension_length++] = word >> 8;
				p.extension[p.extension_length++] = word;
			}
			for (unsigned i = 0; i < p.count; i++)
			{
				classify(p, i, p.ops[i]);
				if (p.ops[i].type == OP_D_SELECT)
					p.selectors |= 1 << i;
			}
		}
		const int index = u.packets.size();
		u.index[h] = index;
		for (unsigned z : zeros)
			u.index[z] = index;
		zeros.clear();
		u.packets.push_back(p);
		h += length;
	}
	for (unsigned z : zeros)
		u.index[z] = -1;
	for (size_t i = 0; i < u.packets.size(); i++)
		u.packets[i].next = i + 1 < u.packets.size() ? int(i + 1) : -1;
	u.first = u.packets.empty() ? -1 : 0;
	u.dirty = false;
}

void mb8aa4181_dsp_device::rebuild_descriptors(unit_state &u)
{
	std::fill_n(u.descriptor, SLOTS, -1);
	for (unsigned i = 0; i < DRP_ENTRIES; i++)
	{
		if (u.drp[i][0] & 0x7f800000)
			continue;
		u.descriptor[(u.drp[i][1] >> 16) & (SLOTS - 1)] = i;
	}
	u.reads.clear();
	for (unsigned slot = 0; slot < SLOTS; slot++)
		if (u.descriptor[slot] >= 0 && !(u.drp[u.descriptor[slot]][0] & 0x80000000))
			u.reads.push_back(slot);
	u.drp_dirty = false;
}


//  Operands

u32 mb8aa4181_dsp_device::raw(const packet &p, unsigned offset, unsigned width)
{
	u32 value = 0;
	for (unsigned i = 0; i < width; i++)
		value = (value << 8) | (offset + i < p.extension_length ? p.extension[offset + i] : 0);
	return value;
}

double mb8aa4181_dsp_device::native_value(u32 value, unsigned bits)
{
	const unsigned scale = (value >> (bits - 3)) & 3;
	const double magnitude = std::ldexp(double(value & ((u32(1) << (bits - 3)) - 1)), 4 * scale - bits);
	return BIT(value, bits - 1) ? -magnitude : magnitude;
}

double mb8aa4181_dsp_device::number(const packet &p, unsigned offset, unsigned width)
{
	return native_value(raw(p, offset, width), 8 * width);
}

u32 mb8aa4181_dsp_device::token_cell(const unit_state &u, double token) const
{
	return u32(s64(std::floor(token * 16384.0)) + s64(std::floor(u.sel[1])));
}

double mb8aa4181_dsp_device::read_operand(unsigned unit, u16 address) const
{
	const unit_state &u = m_unit[unit];
	switch (address >> 12)
	{
	case 0x0: case 0x1:
		return shortmem(u, address);
	case 0x2:
		if (address < 0x2200)
			return u.bus[(u.origin + address) & (BUS_WORDS - 1)];
		break;
	case 0x4:
		if (address < 0x4200)
		{
			const unsigned slot = address & (SLOTS - 1);
			if (u.slot_valid[slot])
				return u.slot[slot];
			if (u.descriptor[slot] >= 0)
				return u.buffered[slot];
			return 0.0;
		}
		break;
	case 0x6:
		if (address < 0x6100)
			return m_port[address & 0xff];
		break;
	case 0xa: case 0xb:
		if (address < 0xa100)
			return std::bit_cast<float>(u.direct[address & 0xff]);
		if ((address & 0xf00) == 0x100)
			return m_shared[address & 0xff];
		break;
	case 0xc:
		if (address < 0xc000 + TARGETS)
			return std::bit_cast<float>(u.target[address & (TARGETS - 1)][2]);
		break;
	case 0xe:
		switch (address)
		{
		case 0xe000: return 0.0;
		case 0xe013: return 1.0 / 16384.0;
		case 0xe018: return 1.0 / 512.0;
		case 0xe020: return 0.5;
		case 0xe021: return 1.0;
		}
		break;
	}
	if (!m_unsupported.count(0x10000000 | address))
		LOGUNSUPPORTED("unit %u: read of %04x\n", unit, address);
	const_cast<mb8aa4181_dsp_device *>(this)->m_unsupported[0x10000000 | address] = true;
	return 0.0;
}

void mb8aa4181_dsp_device::write_operand(unsigned unit, u16 address, double value)
{
	unit_state &u = m_unit[unit];
	switch (address >> 12)
	{
	case 0x0: case 0x1:
		u.shortmem[address & (SHORT_WORDS - 1)] = value;
		return;
	case 0x2:
		if (address < 0x2200)
		{
			u.bus[(u.origin + address) & (BUS_WORDS - 1)] = value;
			return;
		}
		break;
	case 0x4:
		if (address < 0x4200)
		{
			const unsigned slot = address & (SLOTS - 1);
			const int descriptor = u.descriptor[slot];
			if (descriptor >= 0 && (u.drp[descriptor][0] & 0x80000000))
				set_cell(u.origin + (u.drp[descriptor][0] & 0x7fffff), value);
			else if (descriptor < 0 && u.slot_base[slot >> 2] >= 0)
				set_cell(u.origin + u.slot_base[slot >> 2], value);
			return;
		}
		break;
	case 0x6:
		if (address < 0x6100)
		{
			m_port[address & 0xff] = value;
			return;
		}
		break;
	case 0xa: case 0xb:
		if (address < 0xa100)
		{
			direct_store(u, address & 0xff, value);
			return;
		}
		if ((address & 0xf00) == 0x100)
		{
			m_shared[address & 0xff] = value;
			return;
		}
		break;
	case 0xe:
		if ((address & 0xff00) == 0xe800)
			return;
		break;
	}
	if (!m_unsupported.count(0x20000000 | address))
		LOGUNSUPPORTED("unit %u: write of %04x\n", unit, address);
	m_unsupported[0x20000000 | address] = true;
}

bool mb8aa4181_dsp_device::condition(const unit_state &u, u16 word) const
{
	const unsigned source = (word >> 12) & 3;
	const unsigned relation = (word >> 9) & 3;
	const bool complement = BIT(word, 8);
	if (source == 3)
		return BIT(m_flags, (word >> 9) & 7) != complement;
	if (relation == 2)
		return source == 1 ? u.arrival_visible != complement : false;

	double value;
	if (source == 0 && u.has_arith_published)
		value = u.arith_published;
	else
	{
		const history &h = source == 0 ? u.arith : source == 1 ? u.comp : u.func;
		if (!h.has_visible)
			return false;
		value = h.visible;
	}
	if (source == 2 && relation == 1)
		return complement ? value > 0.0 : value < 0.0;
	const bool result = relation == 3 ? value > 0.0 : relation == 0 ? value == 0.0 : value < 0.0;
	return result != complement;
}

void mb8aa4181_dsp_device::unsupported(unsigned unit, const packet &p, const operation &op)
{
	const u32 key = (unit << 24) | (op.kind << 16) | op.word;
	if (!m_unsupported.count(key))
		LOGUNSUPPORTED("unit %u: unsupported %c:%04x at %04x\n", unit, "ABCDE"[op.kind], op.word, p.pc);
	m_unsupported[key] = true;
}


//  Execution

mb8aa4181_dsp_device::source mb8aa4181_dsp_device::make_source(const packet &p, unsigned code, bool subtract)
{
	if (code & 0x30)
		return { number(p, code & 15, ((code >> 4) & 3) + 1), 0, 2 };
	return { 0.0, u8(code & 7), u8((code & 8) && subtract) };
}

void mb8aa4181_dsp_device::classify(const packet &p, unsigned k, operation &op)
{
	const u16 w = op.word;
	const unsigned f = (w >> 10) & 3;
	op.d = (w >> 12) & 7;
	op.x = (w >> 6) & 7;
	op.y = w & 7;
	op.flags = BIT(w, 9) ? F_SIGMA : 0;
	op.type = OP_UNSUPPORTED;
	op.q = 0.0;
	op.value = 0;
	op.drop_true = op.drop_false = op.named = 0;

	switch (op.kind)
	{
	case KIND_A:
		if (!f)
			break;
		op.type = OP_A;
		op.s = (w & 63) ? make_source(p, w & 63, false) : source{ 1.0, 0, 2 };
		if (f & 2)
		{
			const unsigned width = (op.aux >> 10) & 3, code = (op.aux >> 6) & 15;
			op.y = (op.aux >> 12) & 7;
			op.flags |= F_SECOND;
			if (width)
			{
				op.q = number(p, code, width + 1);
				op.flags |= F_SECOND_IMMEDIATE;
			}
			else if (code >= 8)
				op.value = code & 7;
			else if (!code)
				op.flags |= F_SECOND_ALONE;
			else
			{
				op.type = OP_UNSUPPORTED;
				break;
			}
			if (BIT(op.aux, 15))
				op.flags |= F_NEGATE;
		}
		if (f & 1)
		{
			op.t = make_source(p, op.aux & 63, true);
			op.flags |= F_ADDEND;
		}
		if (BIT(w, 15))
			op.flags |= F_HALVE;
		break;

	case KIND_B:
		op.s = make_source(p, w & 63, true);
		if (BIT(w, 15))
		{
			op.type = OP_B_SUM;
			op.y = (w >> 9) & 7;
		}
		else if (!f && op.x == 2 && (w & 0x30) == 0x20)
			op.type = OP_B_SQUARE;
		else if (!f && op.x == 2 && !(w & 0x30))
			op.type = OP_B_NEWTON;
		else if (!f && op.x == 1)
		{
			op.type = OP_B_MOVE;
			op.flags |= F_MOVE;
		}
		else if (!f && op.x == 4)
		{
			op.type = OP_B_DOUBLE;
			op.t = make_source(p, w & 63, false);
		}
		else if (f == 1)
			op.type = BIT(w, 9) ? OP_B_MAC : OP_B_MUL;
		else if (f == 2)
			op.type = OP_B_F2;
		else if (f == 3)
			op.type = OP_B_F3;
		break;

	case KIND_C:
	{
		const unsigned function = (w >> 3) & 7;
		if ((w & 0xfffc) == 0xff70 || (w & 0x8ffe) == 0x8f74 || w == 0xcf64)
			op.type = OP_NONE;
		else if (w == 0xbf7c)
			op.type = OP_C_ARRIVAL_CLEAR;
		else if ((w & 0x8fff) == 0x8f6a)
			op.type = OP_C_ADVANCE;
		else if ((w & 0x8fff) == 0x8f6b)
			op.type = OP_C_CLAMP;
		else if ((w & 0x8ffe) == 0x8f80)
		{
			op.type = OP_C_BIT;
			op.y = w & 1;
		}
		else if ((w & 0x8ff8) == 0x8f40)
			op.type = OP_C_FRACTION;
		else if (!BIT(w, 15) && !BIT(w, 9) && f < 3)
		{
			op.type = OP_C_LOGIC;
			op.s = make_source(p, w & 63, false);
			op.value = f;
		}
		else if ((w & 0x8c00) == 0x8000 || (w & 0x8c00) == 0x8400)
		{
			op.type = (w & 0x400) ? OP_C_MAX : OP_C_MIN;
			op.s = make_source(p, w & 63, false);
		}
		else if ((w & 0x8fc0) == 0x8e00)
		{
			op.type = OP_C_COMPARE;
			op.s = make_source(p, w & 63, true);
		}
		else if ((w & 0x8fc0) == 0x8e40)
		{
			op.type = OP_C_SET;
			op.s = make_source(p, w & 63, true);
		}
		else if ((w & 0x8fc0) == 0x8e80)
		{
			op.type = OP_C_RECIPROCAL;
			op.s = make_source(p, w & 63, false);
		}
		else if ((w & 0x8fc0) == 0x8f00)
		{
			switch (function)
			{
			case 0: case 1: op.type = OP_C_SEED; break;
			case 3: op.type = OP_C_EXPONENT; break;
			case 4: op.type = OP_C_LOGARITHM; break;
			case 5: op.type = OP_C_SIGN; break;
			case 7: op.type = OP_C_FLOOR; break;
			}
		}
		else if ((w & 0x8ff8) == 0x8f48)
			op.type = OP_C_FLOOR14;
		else if ((w & 0x8ff8) == 0x8f50)
			op.type = OP_C_FRACTION14;
		else if ((w & 0x8ffc) == 0x8f58)
		{
			op.type = OP_C_SELECTOR;
			op.y = w & 3;
		}
		else if ((w & 0x8ffc) == 0x8f6c)
		{
			op.type = OP_C_ADD_SELECTOR;
			op.y = w & 3;
		}
		else if ((w & 0x8f00) == 0x0f00)
		{
			op.type = OP_C_ADD;
			op.s = make_source(p, w & 63, false);
		}
		else if (!BIT(w, 15) && f == 3)
		{
			op.type = OP_C_F3;
			op.s = make_source(p, w & 63, true);
		}
		else if ((w & 0x8fc0) == 0x8ec0)
		{
			if (function < 7)
				op.type = OP_C_UNARY0 + function;
		}
		else if ((w & 0x8e00) == 0x8c00 && ((w & 0x3f) == 0x28 || (w & 0x3f) == 0x30))
		{
			op.type = OP_C_SCALE;
			op.q = (w & 0x3f) == 0x28 ? 1.0 / 256.0 : 1.0 / 65536.0;
		}
		else if ((w & 0x8e38) == 0x8c08)
			op.type = OP_C_SHIFT;
		else if ((w & 0x8e38) == 0x8c10 || (w & 0x8e38) == 0x8c20)
		{
			op.type = OP_C_SCALE;
			op.q = std::ldexp(1.0, (w & 0x8e38) == 0x8c10 ? int(op.y) : -int(op.y));
			if (w == 0xfc90 || w == 0xfcd0)
				op.flags |= F_FUNCTION;
		}
		break;
	}

	case KIND_D:
	{
		const unsigned low = w & 0xff;
		if (w >= 0x4000)
		{
			if ((w & 0xff00) == 0x4000)
			{
				const unsigned code = w & 63;
				op.y = (w >> 6) & 3;
				if (code & 0x30)
				{
					op.type = OP_D_ADDRESS;
					op.q = double(raw(p, code & 15, ((code >> 4) & 3) + 1));
				}
				else
				{
					op.type = OP_D_ADDRESS_REGISTER;
					op.x = code & 7;
				}
			}
			break;
		}
		if (w >> 8)
			op.flags |= F_CONDITIONAL;
		if (low >= 0x10 && low < 0x80)
		{
			u8 others[MAX_OPS];
			unsigned m = 0;
			for (unsigned j = 0; j < p.count; j++)
				if (j != k)
					others[m++] = j;
			const unsigned high = (low >> 4) & 15, lower = low & 15;
			if (high > m || lower > m)
				break;
			op.type = OP_D_SELECT;
			op.drop_true = lower ? 1 << others[lower - 1] : 0;
			op.drop_false = high ? 1 << others[high - 1] : 0;
			op.named = op.drop_true | op.drop_false;
		}
		else if (low >= 0x80 && low < 0xc0)
		{
			const u32 offset = raw(p, w & 15, 2);
			op.type = (low & 0xf0) == 0xa0 ? OP_D_CALL : OP_D_JUMP;
			op.value = ((low & 0xf0) == 0x90 ? p.pc + 2 * p.length + 2 * offset : 2 * offset) & 0xffff;
		}
		else if (low >= 0xc0 && low < 0xd0)
		{
			op.type = OP_D_NOTIFY;
			op.y = low & 15;
		}
		else if (low >= 0xd0 && low < 0xd8)
			op.type = OP_D_CLEAR;
		else if (low == 0xe7)
			op.type = OP_D_COUNTER;
		else if (low == 0xf8)
			op.type = OP_D_RETURN;
		else if (low == 0xe8 || low == 0xe9 || low == 0xf1)
			op.type = OP_NONE;
		break;
	}

	case KIND_E:
		if (BIT(w, 15))
			op.flags |= F_STORE;
		if ((w & 0x0fe0) == 0x0fe0)
		{
			op.type = OP_E_OPERAND;
			op.value = raw(p, w & 15, 2);
		}
		else if ((w & 0x8ff8) == 0x89b0)
		{
			const unsigned kind = (w >> 12) & 7;
			if (op.y)
			{
				op.type = OP_E_PUBLISH;
				op.value = kind == 3 ? 2 : kind == 4 ? 3 : op.y == 1 ? 1 : op.y == 2 ? 2 : 0;
			}
		}
		else if ((w & 0x8ff8) == 0x89b8)
			op.type = OP_E_INDIRECT;
		else if ((w & 0x8ff0) == 0x0a90)
		{
			op.type = OP_E_DELAY;
			op.value = w & 15;
		}
		else if ((w & 0x8ff0) == 0x0a80)
		{
			op.type = OP_E_TOKEN;
			op.value = w & 15;
		}
		else if ((w & 0x8ff0) == 0x08b0)
		{
			op.type = OP_E_SAMPLE;
			op.value = w & 15;
		}
		else if ((w & 0x8ff0) == 0x09b0)
		{
			op.type = OP_E_REQUEST;
			op.value = w & 15;
		}
		else if ((w & 0x0fe0) == 0x06c0)
		{
			op.type = OP_E_LOCAL;
			op.value = w & 31;
		}
		else if ((w & 0x0f30) == 0x0200)
		{
			op.type = OP_E_SHORT;
			op.y = (w >> 6) & 3;
			op.value = w & 15;
		}
		else if ((w & 0x0fd0) == 0x0d80)
		{
			op.type = (w & 0x20) ? OP_E_PARAMETER : OP_E_DIRECT;
			op.value = w & 15;
		}
		else if ((w & 0x8ff0) == 0x0ea0)
		{
			op.type = OP_E_TARGET;
			op.value = w & 15;
		}
		break;
	}
}

int mb8aa4181_dsp_device::step(unsigned unitnum, const packet &p, bool &call)
{
	unit_state &u = m_unit[unitnum];
	double r[8];
	std::copy_n(u.r, 8, r);
	const unsigned n = p.count;

	u8 dropped = 0;
	if (p.selectors)
	{
		u8 drop[MAX_OPS];
		for (unsigned k = 0; k < n; k++)
			if (BIT(p.selectors, k))
			{
				const operation &op = p.ops[k];
				drop[k] = (!(op.flags & F_CONDITIONAL) || condition(u, op.word)) ? op.drop_true : op.drop_false;
			}
		for (;;)
		{
			u8 update = 0;
			for (unsigned k = 0; k < n; k++)
				if (BIT(p.selectors, k))
					update |= BIT(dropped, k) ? p.ops[k].named : drop[k];
			if (!(update & ~dropped))
				break;
			dropped |= update;
		}
	}

	struct write { u8 reg; double value; };
	struct store { u8 type; u32 address; double value; };
	write loads[MAX_OPS], sets[MAX_OPS * 2];
	store stores[MAX_OPS];
	write selector_sets[MAX_OPS];
	store requests[MAX_OPS];
	unsigned nloads = 0, nsets = 0, nstores = 0, nselectors = 0, nrequests = 0;
	double results[MAX_OPS];
	u8 has_result = 0;
	double arithmetic[2] = { 0.0, 0.0 };
	u8 arithmetic_kind[2] = { 0, 0 };
	u16 arithmetic_word[2] = { 0, 0 };
	unsigned narithmetic = 0;
	bool has_first_arith = false;
	double first_arith = 0.0;
	bool has_compare = false, has_c_arith = false, tested = false, has_function = false;
	double compare = 0.0, c_arith = 0.0, function = 0.0;
	bool has_comparison = false;
	double comparison = 0.0;
	struct publication { u8 condition; u8 source; };
	publication publications[MAX_OPS];
	unsigned npublications = 0;
	int transfer = NO_TRANSFER;
	call = false;

	enum : u8 { STORE_OPERAND, STORE_SHORT, STORE_LOCAL, STORE_DIRECT, STORE_CELL };
	enum : u8 { REQUEST_DELAY, REQUEST_TOKEN, REQUEST_SAMPLE };

	for (unsigned k = 0; k < n; k++)
	{
		if (BIT(dropped, k))
			continue;
		const operation &op = p.ops[k];
		const unsigned d = op.d, x = op.x, y = op.y;
		const double sigma = (op.flags & F_SIGMA) ? -1.0 : 1.0;
		bool has = true, is_test = false, is_function = false;
		double value = 0.0;
		int target = d;

		switch (op.type)
		{
		case OP_NONE:
		case OP_D_SELECT:
			has = false;
			break;

		case OP_A:
			value = sigma * r[x] * get(op.s, r);
			if (op.flags & F_SECOND)
			{
				double second = r[y];
				if (op.flags & F_SECOND_IMMEDIATE)
					second *= op.q;
				else if (!(op.flags & F_SECOND_ALONE))
					second *= r[op.value];
				value += (op.flags & F_NEGATE) ? -second : second;
			}
			if (op.flags & F_ADDEND)
				value += get(op.t, r);
			if (op.flags & F_HALVE)
				value *= 0.5;
			break;

		case OP_B_SUM: value = r[y] + r[x] + get(op.s, r); break;
		case OP_B_SQUARE: value = r[y] * r[y] / 4.0; break;
		case OP_B_NEWTON: value = r[d] * (2.0 - r[y] * r[d]); break;
		case OP_B_MOVE: value = get(op.s, r); break;
		case OP_B_DOUBLE: value = 2.0 * r[d] + get(op.t, r); break;
		case OP_B_MUL: value = r[x] * get(op.s, r); break;
		case OP_B_MAC: value = r[d] + r[x] * get(op.s, r); break;
		case OP_B_F2: value = sigma * r[d] * r[x] + get(op.s, r); break;
		case OP_B_F3: value = sigma * r[x] + get(op.s, r); break;

		case OP_C_ARRIVAL_CLEAR:
			u.arrival = false;
			has = false;
			break;
		case OP_C_ADVANCE:
			value = r[d] + r[7] / 16.0;
			break;
		case OP_C_CLAMP:
		{
			const double rate = r[(d + 1) & 7], endpoint = r[(d + 2) & 7];
			if (rate == 0.0)
			{
				unsupported(unitnum, p, op);
				has = false;
				break;
			}
			const bool arrived = (r[d] - endpoint) * (rate > 0.0 ? 1.0 : -1.0) > 0.0;
			u.arrival = arrived;
			value = arrived ? endpoint : r[d];
			break;
		}
		case OP_C_BIT:
			value = double((s64(r[d]) >> y) & 1);
			is_function = true;
			target = -1;
			break;
		case OP_C_FRACTION: value = r[y] - std::floor(r[y]); break;
		case OP_C_LOGIC:
		{
			const u64 left = u64(s64(std::trunc(r[x] * 8388608.0))) & 0xffffffff;
			const u64 right = u64(s64(std::trunc(get(op.s, r) * 8388608.0))) & 0xffffffff;
			const u64 result = op.value == 0 ? left & right : op.value == 1 ? left | right : left ^ right;
			value = double(result) / 8388608.0;
			break;
		}
		case OP_C_MIN: value = std::min(sigma * r[x], get(op.s, r)); break;
		case OP_C_MAX: value = std::max(sigma * r[x], get(op.s, r)); break;
		case OP_C_COMPARE:
			value = r[d] + get(op.s, r);
			is_test = true;
			target = -1;
			has_compare = true;
			compare = value;
			break;
		case OP_C_SET: value = get(op.s, r); break;
		case OP_C_RECIPROCAL: value = 1.0 / get(op.s, r); break;
		case OP_C_SEED: value = 1.0 / r[y]; is_function = true; break;
		case OP_C_EXPONENT:
		{
			const double k2 = std::floor(r[y] / std::numbers::ln2);
			sets[nsets++] = { u8(d), std::ldexp(1.0, int(k2)) };
			sets[nsets++] = { u8(y), r[y] - k2 * std::numbers::ln2 };
			has = false;
			break;
		}
		case OP_C_LOGARITHM:
		{
			has = false;
			const double v = r[y];
			if (v <= 0.0)
			{
				unsupported(unitnum, p, op);
				break;
			}
			const double k2 = std::floor(std::log2(v / (std::numbers::sqrt2 / 2.0)));
			const double m = v / std::ldexp(1.0, int(k2));
			if (m == 1.0)
			{
				unsupported(unitnum, p, op);
				break;
			}
			sets[nsets++] = { u8(d), k2 };
			sets[nsets++] = { u8(y), (m + 1.0) / (m - 1.0) };
			break;
		}
		case OP_C_SIGN: value = double((r[y] > 0.0) - (r[y] < 0.0)); is_function = true; break;
		case OP_C_FLOOR: value = std::floor(r[y]); is_function = true; break;
		case OP_C_FLOOR14: value = std::floor(r[y] * 16384.0) / 16384.0; break;
		case OP_C_FRACTION14:
		{
			const double v = r[y] * 16384.0;
			value = v - std::floor(v);
			break;
		}
		case OP_C_SELECTOR: value = u.sel[y] / 16384.0; break;
		case OP_C_ADD_SELECTOR: value = r[d] + u.sel[y] / 16384.0; break;
		case OP_C_ADD:
			value = r[d] + get(op.s, r);
			is_test = true;
			break;
		case OP_C_F3:
		{
			const double s = get(op.s, r);
			value = sigma * r[x] + s;
			has_c_arith = true;
			c_arith = std::fabs(r[x]) + s;
			break;
		}
		case OP_C_UNARY0: value = r[d] + r[y]; is_test = true; break;
		case OP_C_UNARY1: value = r[d] - r[y]; is_test = true; has_compare = true; compare = value; break;
		case OP_C_UNARY2: value = std::fabs(r[y]); break;
		case OP_C_UNARY3: value = std::clamp(r[y], -1.0, 1.0); break;
		case OP_C_UNARY4: value = wrap(r[y]); is_function = true; break;
		case OP_C_UNARY5: value = 1.0 - 2.0 * std::fabs(wrap(r[y] - 0.5)); is_function = true; break;
		case OP_C_UNARY6: value = noise(r[y]); is_function = true; break;
		case OP_C_SCALE:
			value = r[x] * op.q;
			is_function = op.flags & F_FUNCTION;
			break;
		case OP_C_SHIFT:
		{
			const double e = r[y];
			if (e != std::trunc(e) || std::fabs(e) > 1024.0)
			{
				unsupported(unitnum, p, op);
				has = false;
				break;
			}
			value = std::ldexp(std::trunc(std::ldexp(r[x], 32)), int(e) - 32);
			break;
		}

		case OP_D_ADDRESS:
			selector_sets[nselectors++] = { u8(y), op.q };
			has = false;
			break;
		case OP_D_ADDRESS_REGISTER:
			selector_sets[nselectors++] = { u8(y), r[x] * 16384.0 };
			has = false;
			break;
		case OP_D_JUMP:
		case OP_D_CALL:
			has = false;
			if (!(op.flags & F_CONDITIONAL) || condition(u, op.word))
			{
				transfer = op.value;
				call = op.type == OP_D_CALL;
			}
			break;
		case OP_D_NOTIFY:
			has = false;
			if (!(op.flags & F_CONDITIONAL) || condition(u, op.word))
			{
				emu_timer *const timer = m_notify_timer[unitnum][y];
				if (timer->enabled())
				{
					u.notify_pending |= 1 << y;
					irq_update(unitnum);
				}
				else
					timer->adjust(attotime::from_ticks(u.packets_run, m_packet_clock), (unitnum << 4) | y);
			}
			break;
		case OP_D_CLEAR:
			has = false;
			if (!(op.flags & F_CONDITIONAL) || condition(u, op.word))
				sets[nsets++] = { u8(y), 0.0 };
			break;
		case OP_D_COUNTER:
			has = false;
			if (!(op.flags & F_CONDITIONAL) || condition(u, op.word))
			{
				value = r[7] - 1.0 / 16384.0;
				has = is_test = true;
				target = 7;
			}
			break;
		case OP_D_RETURN:
			has = false;
			if (!(op.flags & F_CONDITIONAL) || condition(u, op.word))
				transfer = RETURN;
			break;

		case OP_E_OPERAND:
			has = false;
			if (op.flags & F_STORE)
				stores[nstores++] = { STORE_OPERAND, op.value, r[d] };
			else
				loads[nloads++] = { u8(d), read_operand(unitnum, op.value) };
			break;
		case OP_E_PUBLISH:
			has = false;
			publications[npublications++] = { u8(op.value), u8(y - 1) };
			break;
		case OP_E_INDIRECT:
			has = false;
			stores[nstores++] = { STORE_CELL, token_cell(u, r[y]), r[d] };
			break;
		case OP_E_DELAY:
			has = false;
			requests[nrequests++] = { REQUEST_DELAY, op.value, r[d] };
			break;
		case OP_E_TOKEN:
			has = false;
			requests[nrequests++] = { REQUEST_TOKEN, op.value, r[d] };
			break;
		case OP_E_SAMPLE:
			has = false;
			loads[nloads++] = { u8(d), u.sample[((u32(u.sel[3]) & (SAMPLE_OWNERS - 1)) << 4) | op.value] };
			break;
		case OP_E_REQUEST:
			has = false;
			requests[nrequests++] = { REQUEST_SAMPLE, op.value, r[d] };
			break;
		case OP_E_LOCAL:
		{
			has = false;
			const u32 address = u32(u.sel[3]) + op.value;
			if (op.flags & F_STORE)
				stores[nstores++] = { STORE_LOCAL, address, r[d] };
			else
				loads[nloads++] = { u8(d), u.local[address & (SHORT_WORDS - 1)] };
			break;
		}
		case OP_E_SHORT:
		{
			has = false;
			const u32 address = u32(u.sel[y]) + op.value;
			if (op.flags & F_STORE)
				stores[nstores++] = { STORE_SHORT, address, r[d] };
			else
				loads[nloads++] = { u8(d), shortmem(u, address) };
			break;
		}
		case OP_E_DIRECT:
		case OP_E_PARAMETER:
		{
			has = false;
			const u32 address = (op.type == OP_E_PARAMETER ? u32(u.sel[2]) : 0) + op.value;
			if (op.flags & F_STORE)
				stores[nstores++] = { STORE_DIRECT, address, r[d] };
			else
				loads[nloads++] = { u8(d), std::bit_cast<float>(u.direct[address & (DIRECT_WORDS - 1)]) };
			break;
		}
		case OP_E_TARGET:
			has = false;
			loads[nloads++] = { u8(d), std::bit_cast<float>(u.target[(u32(u.sel[2]) + op.value) & (TARGETS - 1)][2]) };
			break;

		default:
			unsupported(unitnum, p, op);
			has = false;
			break;
		}

		if (!has)
			continue;
		results[k] = value;
		has_result |= 1 << k;
		if (target >= 0)
			sets[nsets++] = { u8(target), value };
		if (is_test)
		{
			has_comparison = true;
			comparison = value;
			tested = true;
		}
		if (is_function)
		{
			has_function = true;
			function = value;
		}
		if (op.kind == KIND_A || op.kind == KIND_B)
		{
			if (narithmetic < 2)
			{
				arithmetic[narithmetic] = value;
				arithmetic_kind[narithmetic] = op.kind;
				arithmetic_word[narithmetic] = op.word;
			}
			narithmetic++;
			if (!(op.flags & F_MOVE) && !has_first_arith)
			{
				has_first_arith = true;
				first_arith = value;
			}
		}
	}

	for (unsigned i = 0; i < nstores; i++)
	{
		const store &s = stores[i];
		switch (s.type)
		{
		case STORE_OPERAND: write_operand(unitnum, s.address, s.value); break;
		case STORE_SHORT: u.shortmem[s.address & (SHORT_WORDS - 1)] = s.value; break;
		case STORE_LOCAL: u.local[s.address & (SHORT_WORDS - 1)] = s.value; break;
		case STORE_DIRECT: direct_store(u, s.address & (DIRECT_WORDS - 1), s.value); break;
		case STORE_CELL: set_cell(s.address, s.value); break;
		}
	}
	for (unsigned i = 0; i < nselectors; i++)
		u.sel[selector_sets[i].reg] = selector_sets[i].value;
	for (unsigned i = 0; i < nrequests; i++)
	{
		const store &q = requests[i];
		const double position = q.value * 16384.0;
		if (q.type == REQUEST_DELAY)
		{
			const unsigned slot = (u32(u.sel[1]) + q.address) & (SLOTS - 1);
			const double whole = std::trunc(position);
			const u32 address = u.origin + u32(s64(whole)) + 2;
			u.slot_base[slot >> 2] = s32(u.sel[0]);
			u.slot[slot] = cell(address);
			u.slot[(slot + 1) & (SLOTS - 1)] = cell(address + 1);
			u.slot[(slot + 2) & (SLOTS - 1)] = position - whole;
			u.slot_valid[slot] = u.slot_valid[(slot + 1) & (SLOTS - 1)] = u.slot_valid[(slot + 2) & (SLOTS - 1)] = 1;
		}
		else if (q.type == REQUEST_TOKEN)
		{
			const unsigned slot = (u32(u.sel[0]) + q.address) & (SLOTS - 1);
			const u32 address = token_cell(u, q.value);
			const double values[3] = { cell(address), cell(address + 1), position - std::floor(position) };
			for (unsigned j = 0; j < 3; j++)
			{
				const unsigned target = (slot + j) & (SLOTS - 1);
				u.requested[target] = values[j];
				if (!u.requested_valid[target])
				{
					u.requested_valid[target] = 1;
					u.pending.push_back(target);
				}
			}
		}
		else
		{
			const s64 address = s64(std::floor(q.value * 65536.0));
			u.samples.push_back({ ((u32(u.sel[3]) & (SAMPLE_OWNERS - 1)) << 4) | q.address, cell(u32(address >> 2)) });
		}
	}
	for (unsigned i = 0; i < nloads; i++)
		u.r[loads[i].reg] = loads[i].value;
	for (unsigned i = 0; i < nsets; i++)
		u.r[sets[i].reg] = sets[i].value;

	bool has_c_published = false, has_f_published = false;
	double c_published = 0.0, f_published = 0.0;
	if (npublications)
	{
		u8 survivors[MAX_OPS];
		unsigned nsurvivors = 0;
		for (unsigned k = 0; k < n; k++)
			if (!BIT(dropped, k))
				survivors[nsurvivors++] = k;
		for (unsigned i = 0; i < npublications; i++)
		{
			const publication &pub = publications[i];
			if (pub.source >= nsurvivors || !BIT(has_result, survivors[pub.source]))
				continue;
			const double v = results[survivors[pub.source]];
			switch (pub.condition)
			{
			case 1: u.arith_published = v; u.has_arith_published = true; break;
			case 2: c_published = v; has_c_published = true; break;
			case 3: f_published = v; has_f_published = true; break;
			}
		}
	}

	if (has_c_published)
	{
		u.comparison_published = true;
		has_comparison = true;
		comparison = c_published;
	}
	else if (has_compare)
	{
		u.comparison_published = false;
		has_comparison = true;
		comparison = compare;
	}
	else if (u.comparison_published)
	{
	}
	else if (narithmetic > 1 && !(arithmetic_kind[0] == KIND_B && BIT(arithmetic_word[1], 9)))
	{
		has_comparison = true;
		comparison = arithmetic[1];
	}
	else if (has_c_arith && !tested)
	{
		has_comparison = true;
		comparison = c_arith;
	}

	u.comp.push(has_comparison, comparison);
	u.arith.push(has_first_arith, first_arith);
	u.func.push(has_f_published || has_function, has_f_published ? f_published : function);
	u.arrival_visible = u.arrival_last;
	u.arrival_last = u.arrival;

	return transfer;
}

void mb8aa4181_dsp_device::direct_store(unit_state &u, unsigned index, double value)
{
	u.direct_log.push_back({ u16(index), u.packets_run, u.direct[index] });
	u.direct[index] = std::bit_cast<u32>(float(value));
}

TIMER_CALLBACK_MEMBER(mb8aa4181_dsp_device::notify)
{
	const unsigned unit = param >> 4;
	m_unit[unit].notify_pending |= 1 << (param & 15);
	irq_update(unit);
}

void mb8aa4181_dsp_device::begin_frame(unit_state &u)
{
	u.direct_log.clear();
	u.packets_run = 0;
	if (u.dirty)
		decode(u);
	if (u.drp_dirty)
		rebuild_descriptors(u);
	for (u16 slot : u.reads)
		u.buffered[slot] = cell(u.origin + (u.drp[u.descriptor[slot]][0] & 0x7fffff) + 2);
	for (u16 slot : u.pending)
	{
		u.slot[slot] = u.requested[slot];
		u.slot_valid[slot] = 1;
		u.requested_valid[slot] = 0;
	}
	u.pending.clear();
}

void mb8aa4181_dsp_device::end_frame(unit_state &u)
{
	u.origin = (u.origin - 1) & (MEMORY_CELLS - 1);
	for (const sample_request &s : u.samples)
		u.sample[s.owner] = s.value;
	u.samples.clear();
}

void mb8aa4181_dsp_device::run_frame(unsigned unit)
{
	unit_state &u = m_unit[unit];
	begin_frame(u);
	u.depth = 0;
	int index = u.first;
	bool pending = false, pending_call = false;
	int pending_target = NO_TRANSFER;
	for (unsigned count = 0; index >= 0 && count < FRAME_PACKETS; count++)
	{
		const packet &p = u.packets[index];
		bool call;
		const int transfer = step(unit, p, call);
		u.packets_run++;
		if (!pending)
		{
			index = p.next;
			if (transfer != NO_TRANSFER)
			{
				pending = true;
				pending_target = transfer;
				pending_call = call;
			}
			continue;
		}

		pending = false;
		if (pending_target == RETURN)
		{
			index = u.depth ? int(u.stack[--u.depth]) : -1;
			continue;
		}
		if (pending_call && u.depth < STACK_DEPTH && p.next >= 0)
			u.stack[u.depth++] = p.next;
		index = u.index[(pending_target >> 1) & (PRG_HALFWORDS - 1)];
		if (index == -2)
		{
			const u32 key = 0x30000000 | (unit << 16) | pending_target;
			if (!m_unsupported.count(key))
				LOGUNSUPPORTED("unit %u: transfer into a packet at %04x\n", unit, pending_target);
			m_unsupported[key] = true;
			index = -1;
		}
	}
	end_frame(u);
}

TIMER_CALLBACK_MEMBER(mb8aa4181_dsp_device::frame)
{
	if (!(m_frames & 0xff))
		m_stream->update();

	m_frame_start = machine().time();
	run_frame(1);
	run_frame(0);
	m_frames++;

	const u32 next = (m_output_write + 1) % OUTPUT_BUFFER;
	if (next != m_output_read)
	{
		m_output[m_output_write][0] = float(m_port[0]);
		m_output[m_output_write][1] = float(m_port[1]);
		m_output_write = next;
	}

	for (unsigned unit = 0; unit < UNITS; unit++)
	{
		unit_state &u = m_unit[unit];
		for (unsigned i = 0; i < TARGETS / 32; i++)
		{
			u.target_pending[i] |= u.target_queued[i];
			u.target_queued[i] = 0;
		}
		irq_update(unit);
		if (BIT(m_switch_queued, unit))
		{
			m_switch_queued &= ~(1 << unit);
			m_switch_done |= 1 << unit;
			m_irq_cb[IRQ_SWITCH0 + unit](ASSERT_LINE);
		}
	}
}

void mb8aa4181_dsp_device::sound_stream_update(sound_stream &stream)
{
	for (int i = 0; i < stream.samples(); i++)
	{
		if (m_output_read != m_output_write)
		{
			m_last[0] = m_output[m_output_read][0];
			m_last[1] = m_output[m_output_read][1];
			m_output_read = (m_output_read + 1) % OUTPUT_BUFFER;
		}
		stream.put(0, i, m_last[0]);
		stream.put(1, i, m_last[1]);
	}
}
