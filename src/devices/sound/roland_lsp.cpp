// license:BSD-3-Clause
// copyright-holders:superctr
// thanks-to: giulioz
/*
 *  Roland LSP (Fujitsu MB87837)
 *
 *  Effect processor of the Boss ME-6 and, as the insertion-effect DSP, of the Roland
 *  SC-88Pro and successors. A 384-instruction program runs once per sample -
 *  the clock is 2 x 384 x Fs and the program is the sample period, so the chip
 *  is a CPU with a fixed schedule rather than a streaming sound device: the host PCM
 *  chip writes the serial inputs, runs one pass and reads the outputs back.
 *
 *  Host window: sixteen byte registers, a 24-bit data latch committed by the write of
 *  the address low byte, a read latch loaded by the read address, and a configuration
 *  word that starts and halts the program. The address space is 128 words of internal
 *  RAM below the 384 program words, and the firmware patches individual program words
 *  while the program runs - the coefficient bytes are the effect's parameters and there
 *  are no parameter registers.
 *
 *  Datapath: two 24-bit accumulators over the internal RAM, which is a delay line whose
 *  pointer steps back once per sample, so an instruction's 7-bit offset is a fixed
 *  delay. A store writes the accumulator as it stood three slots earlier, saturating or
 *  merely narrowed depending on the store field, and it lands before the same slot reads
 *  its operand; the sums themselves run on the whole 32-bit accumulator. The multiply by a
 *  coefficient register takes the register's top byte or, eight bits further down, its
 *  next byte, so a pair of them is a sixteen-bit multiply. That low half, and the form of
 *  the special opcodes that multiplies a second coefficient, take their operand from the
 *  slot that ran before them, and every special slot also feeds its accumulator.
 *
 *  Long delays live in an external DRAM that steps back the same way; an access carries
 *  no address of its own, but assembles one from the three-bit external RAM field of the
 *  six slots following an opener, the first five of which open nothing. A read lands in
 *  the read register twelve slots after its opener, before that slot runs; a write's
 *  address reaches the write latch eight slots after, and the next store into the write
 *  latch performs the write, which one program in the corpus puts a slot late. A word is
 *  sixteen bits, four nibbles on a four-bit bus: a block float of the sample's top
 *  twenty bits, a 14-bit mantissa and a 2-bit exponent counting two bits a step, so loud
 *  material keeps fourteen significant bits (set_eram_block_float(false) keeps the
 *  twenty linear).
 *
 *  Everything one slot hands a later one is clocked by the slots as they run: the
 *  transactions, the operands handed on, a jump's follower and slot 0x12's skip. A jump
 *  special sets a condition latch, and the slot that runs after it jumps once it has run
 *  if the latch is set. The program counter is nine bits, the 128 addresses below the
 *  program fetching nothing, and a sample is 384 slots of it whatever it does. Stereo is
 *  by slot: the first 192 of a sample handle one channel and the rest the other.
 *
 *  In the SC-88Pro, the XP is the pump, so the device is halted as far as the scheduler
 *  is concerned and execute_run() serves the debugger; run_once() is one sample.
 *
 *  TODO:
 *  - the external RAM's word, which no capture yet tells from the linear twenty bits
 *  - when a write takes its data: the store into the write latch, or a fixed commit a slot
 *    or two after the window
 *  - which word carries slot 0x12's skip once a jump intervenes
 *  - what the fetch finds below the program store
 *  - the DRAM row/column walk, modelled here as a flat ring
 *  - what raises INT, and the busy bit's timing
 *  - the serial word width the configuration word selects
 *  - whether the program can write the program area
 */
#include "emu.h"
#include "roland_lsp.h"

#include "roland_lspd.h"

#include <algorithm>

#define LOG_HOST    (1U << 1)
#define LOG_CONFIG  (1U << 2)

#define VERBOSE (LOG_GENERAL)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(ROLAND_LSP, roland_lsp_device, "roland_lsp", "Roland LSP")

namespace {

// offsets 1 to 4 are not memory reads but a power-of-two operand, so the coefficient is an immediate
constexpr s32 IMMEDIATE[5] = { 0, 1 << 7, 1 << 12, 1 << 17, 1 << 22 };

constexpr bool is_immediate(int offset) { return offset >= 1 && offset <= 4; }

} // anonymous namespace

roland_lsp_device::roland_lsp_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: cpu_device(mconfig, ROLAND_LSP, tag, owner, clock)
	, m_program_config("program", ENDIANNESS_BIG, 32, 9, -2, address_map_constructor(FUNC(roland_lsp_device::program_map), this))
{
}

void roland_lsp_device::program_map(address_map &map)
{
	map(0x000, 0x1ff).rw(FUNC(roland_lsp_device::internal_r), FUNC(roland_lsp_device::internal_w));
}

device_memory_interface::space_config_vector roland_lsp_device::memory_space_config() const
{
	return space_config_vector { std::make_pair(AS_PROGRAM, &m_program_config) };
}

std::unique_ptr<util::disasm_interface> roland_lsp_device::create_disassembler()
{
	return std::make_unique<roland_lsp_disassembler>();
}

void roland_lsp_device::device_start()
{
	m_eram = make_unique_clear<s32[]>(ERAM_SIZE);
	set_icountptr(m_icount);

	state_add(STATE_GENPC, "GENPC", m_pc).noshow();
	state_add(STATE_GENPCBASE, "CURPC", m_pc).noshow();
	state_add(0, "PC", m_pc);
	state_add(1, "ACCA", m_acc[0]);
	state_add(2, "ACCB", m_acc[1]);
	state_add(3, "BUF", m_buffer_pos);
	state_add(4, "ERAM", m_eram_pos);
	state_add(5, "MUL0", m_multiplier[0]);
	state_add(6, "MUL1", m_multiplier[1]);
	state_add(7, "TAP", m_tap);

	save_pointer(NAME(m_eram), ERAM_SIZE);
	save_item(NAME(m_program));
	save_item(NAME(m_iram));
	save_item(NAME(m_pc));
	save_item(NAME(m_slot));
	save_item(NAME(m_configuration));
	save_item(NAME(m_running));
	save_item(NAME(m_halted));
	save_item(NAME(m_acc));
	save_item(NAME(m_history));
	save_item(NAME(m_hand_on.from));
	save_item(NAME(m_hand_on.offset));
	save_item(NAME(m_after_low));
	save_item(NAME(m_run));
	save_item(NAME(m_handed));
	save_item(NAME(m_multiplier));
	save_item(NAME(m_eram_cmd));
	save_item(NAME(m_eram_read));
	save_item(NAME(m_eram_latch));
	save_item(NAME(m_eram_address));
	save_item(NAME(m_tap));
	save_item(NAME(m_buffer_pos));
	save_item(NAME(m_eram_pos));
	save_item(NAME(m_jump));
	save_item(NAME(m_follow));
	save_item(NAME(m_serial_in));
	save_item(NAME(m_serial_out));
	save_item(NAME(m_audio_in));
	save_item(NAME(m_audio_out));
	save_item(NAME(m_host_data));
	save_item(NAME(m_host_read));
	save_item(NAME(m_host_address));
}

void roland_lsp_device::device_reset()
{
	std::fill(std::begin(m_program), std::end(m_program), 0);
	std::fill(std::begin(m_iram), std::end(m_iram), 0);
	std::fill_n(m_eram.get(), ERAM_SIZE, 0);
	std::fill(std::begin(m_decoded), std::end(m_decoded), EMPTY);

	m_icount = 0;
	m_pc = PROGRAM_BASE;
	m_slot = 0;
	m_configuration = 0;
	m_running = false;
	m_halted = true;

	m_acc[0] = m_acc[1] = 0;
	std::fill(&m_history[0][0], &m_history[0][0] + 6, 0);
	m_hand_on = { FROM_MEMORY, 0 };
	m_after_low = false;
	m_run = 0;
	m_handed = 0;
	m_multiplier[0] = m_multiplier[1] = 0;
	std::fill(std::begin(m_eram_cmd), std::end(m_eram_cmd), 0);
	m_eram_read = 0;
	m_eram_latch = 0;
	m_eram_address = 0;
	m_tap = 0;
	m_buffer_pos = 0;
	m_eram_pos = 0;
	m_jump = false;
	m_follow = -1;

	m_serial_in[0] = m_serial_in[1] = 0;
	m_serial_out[0] = m_serial_out[1] = 0;
	m_audio_in = 0;
	m_audio_out = 0;

	m_host_data = 0;
	m_host_read = 0;
	m_host_address = 0;
}

void roland_lsp_device::device_post_load()
{
	for (int n = 0; n < PROGRAM_SIZE; n++)
		m_decoded[n] = decode(m_program[n]);
}

roland_lsp_device::instruction roland_lsp_device::decode(u32 word)
{
	instruction s;
	s.nop = !word;
	s.opcode = (word >> 21) & 7;
	s.store = (word >> 19) & 3;
	s.eram_cmd = (word >> 16) & 7;
	s.offset = (word >> 8) & 0x7f;
	s.coefficient = word & 0xff;
	s.shift = BIT(word, 15) ? 5 : 7;
	return s;
}

u32 roland_lsp_device::internal_r(offs_t address)
{
	return (address < PROGRAM_BASE) ? (m_iram[address] & 0xffffff) : m_program[address - PROGRAM_BASE];
}

void roland_lsp_device::internal_w(offs_t address, u32 data)
{
	if (address < PROGRAM_BASE)
		m_iram[address] = narrow(data);
	else
	{
		m_program[address - PROGRAM_BASE] = data & 0xffffff;
		m_decoded[address - PROGRAM_BASE] = decode(data & 0xffffff);
	}
}

void roland_lsp_device::configure(u16 word)
{
	LOGMASKED(LOG_CONFIG, "configure %04x\n", word);

	m_configuration = word;
	if (BIT(word, 12))
		m_running = false;
	else if (!m_running)
	{
		m_running = true;
		m_pc = PROGRAM_BASE;
		m_slot = 0;
	}
}

u8 roland_lsp_device::host_r(offs_t offset)
{
	switch (offset & 0x0f)
	{
	case HOST_ADDRESS_LOW:  return m_host_read & 0xff;
	case HOST_ADDRESS_HIGH: return (m_host_read >> 8) & 0xff;
	case HOST_DATA_LOW:     return (m_host_read >> 16) & 0xff;
	case HOST_DATA_MID:     return 0;
	}
	return 0;
}

void roland_lsp_device::host_w(offs_t offset, u8 data)
{
	switch (offset & 0x0f)
	{
	case HOST_ADDRESS_LOW:
		m_host_address = (m_host_address & 0xff00) | data;
		LOGMASKED(LOG_HOST, "write %03x = %06x\n", m_host_address & 0x1ff, m_host_data);
		internal_w(m_host_address & 0x1ff, m_host_data);
		break;

	case HOST_ADDRESS_HIGH:
		m_host_address = (m_host_address & 0x00ff) | (u16(data) << 8);
		break;

	case HOST_DATA_LOW:
		m_host_data = (m_host_data & 0xffff00) | data;
		break;

	case HOST_DATA_MID:
		m_host_data = (m_host_data & 0xff00ff) | (u32(data) << 8);
		break;

	case HOST_DATA_HIGH:
		m_host_data = (m_host_data & 0x00ffff) | (u32(data) << 16);
		break;

	case HOST_CONFIGURE:
		configure(m_host_data & 0xffff);
		break;

	case HOST_READ_LOW:
		m_host_address = (m_host_address & 0xff00) | data;
		m_host_read = internal_r(m_host_address & 0x1ff);
		break;

	case HOST_READ_HIGH:
		m_host_address = (m_host_address & 0x00ff) | (u16(data) << 8);
		break;
	}
}

void roland_lsp_device::eram_clock(u8 command)
{
	bool locked = false;
	for (int n = 0; n < 5; n++)
		locked |= eram_opened(n) != 0;

	std::copy_backward(std::begin(m_eram_cmd), std::end(m_eram_cmd) - 1, std::end(m_eram_cmd));
	m_eram_cmd[0] = command | (locked ? 0 : (((command >> 1) & 3) << 3));
}

u16 roland_lsp_device::eram_base(int kind, int back) const
{
	if (kind == 2)
		return ((m_eram_cmd[back - 1] & 7) == 2) ? 1 : 0;

	u16 base = 0;
	for (int digit = 0; digit < 5; digit++)
		base += (m_eram_cmd[back - 1 - digit] & 7) << (digit * 3);
	if (BIT(m_eram_cmd[back - 6], 0))
		base += 0x8000;
	return base;
}

s32 roland_lsp_device::eram_word(s32 value) const
{
	if (!m_eram_block_float)
		return value;
	const s32 magnitude = value ^ (value >> 31);
	const int exponent = (magnitude >= 0x2000) + (magnitude >= 0x8000) + (magnitude >= 0x20000);
	return value & ~((1 << (2 * exponent)) - 1);
}

s32 roland_lsp_device::source(const instruction &s) const
{
	if (!s.store)
		return 0;
	const s32 value = m_history[s.store == 2][2];
	return (s.store == 3) ? narrow(value) : saturate(value);
}

roland_lsp_device::operand_ref roland_lsp_device::resolve(const instruction &s)
{
	if (s.low_half())
	{
		if (!m_after_low)
			m_run = s.offset;
		return m_hand_on;
	}
	if (s.second_operand())
		return m_hand_on;

	const u8 bit_set = (m_after_low && (s.mac() || s.high_half())) ? m_run : s.offset;
	return is_immediate(bit_set) ? operand_ref{ FROM_CONSTANT, bit_set } : operand_ref{ FROM_MEMORY, s.offset };
}

s32 roland_lsp_device::operand(const operand_ref &o) const
{
	switch (o.from)
	{
	case FROM_CONSTANT: return IMMEDIATE[o.offset];
	case FROM_SPECIAL:  return m_handed;
	default:            return m_iram[iram_address(o.offset)];
	}
}

void roland_lsp_device::mac(const instruction &s, const operand_ref &o)
{
	if (s.store)
		m_iram[iram_address(s.offset)] = source(s);

	const s32 product = s32((s64(operand(o)) * s8(s.coefficient)) >> s.shift);

	switch (s.opcode)
	{
	case OP_MAC_A: m_acc[0] = add(m_acc[0], product); break;
	case OP_SET_A: m_acc[0] = product; break;
	case OP_MAC_B: m_acc[1] = add(m_acc[1], product); break;
	case OP_SET_B: m_acc[1] = product; break;
	case OP_ABS:   m_acc[0] = BIT(product, 24) ? s32(0U - u32(product)) : product; break;
	default:       accumulate(m_acc[s.opcode & 1], product, s.replace()); break;
	}
}

void roland_lsp_device::multiply(const instruction &s, const operand_ref &o)
{
	const u8 code = s.coefficient;
	const bool low = BIT(code, 6);
	s32 multiplicand;

	if (!(code & 3))
		multiplicand = low ? s32(code) : s32(s8(code));
	else
	{
		const s32 reg = m_multiplier[BIT(code, 1)];
		multiplicand = low ? ((reg >> 8) & 0xff) : (reg >> 16);
	}

	s32 value;
	if (low)
	{
		value = operand(o);
		if (s.store)
			m_iram[iram_address(s.offset)] = source(s);
	}
	else
	{
		if (s.store)
			m_iram[iram_address(s.offset)] = source(s);
		value = operand(o);
	}

	s32 product = s32((s64(value) * multiplicand) >> (s.shift + (low ? 8 : 0)));
	if (BIT(code, 2))
		product = s32(0U - u32(product));

	accumulate(m_acc[BIT(code, 4)], product, BIT(code, 3));
}

void roland_lsp_device::special(const instruction &s, const operand_ref &o)
{
	const int slot = s.slot();
	s32 &acc = m_acc[s.opcode & 1];

	if (s.second_operand())
	{
		accumulate(acc, s32((s64(operand(o)) * s.coefficient) >> (8 + s.shift)), s.replace());
		return;
	}

	s32 value = source(s);
	switch (slot)
	{
	case SLOT_JUMP_NEGATIVE: m_jump = value < 0; break;
	case SLOT_JUMP_POSITIVE: m_jump = value >= 0; break;
	case SLOT_JUMP:          m_jump = true; break;
	case SLOT_ERAM_WRITE:    m_eram_latch = value; break;

	case SLOT_TAP:
		if (s.store == 3)
		{
			const s32 raw = m_history[0][2];
			m_tap = u16((raw >> 10) & 0xffff);
			m_multiplier[0] = (raw & 0x3ff) << 13;
		}
		break;

	case SLOT_MULTIPLIER:
	case SLOT_MULTIPLIER + 1:
		m_multiplier[slot - SLOT_MULTIPLIER] = value;
		break;

	case SLOT_STORE_76:
		m_iram[iram_address(0x76)] = value;
		break;

	case SLOT_AUDIO_OUT:
		m_audio_out = value;
		m_iram[iram_address(0x78)] = value;
		if (m_slot < PROGRAM_SIZE / 2)
			m_serial_out[1] = value;
		break;

	case SLOT_ERAM_READ:
	case SLOT_ERAM_READ + 1:
	case SLOT_ERAM_READ + 2:
	case SLOT_ERAM_READ + 3:
		value = m_eram_read;
		m_iram[iram_address(0x60 + slot)] = value;
		break;

	case SLOT_AUDIO_IN:
	case SLOT_AUDIO_IN + 1:
		if (m_slot >= PROGRAM_SIZE / 2)
			m_audio_in = m_serial_in[0];
		value = m_audio_in;
		m_iram[iram_address(0x60 + slot)] = value;
		break;
	}

	m_handed = value;
	accumulate(acc, s32((s64(value) * s8(s.coefficient)) >> s.shift), s.replace());
}

void roland_lsp_device::step()
{
	const int index = m_pc - PROGRAM_BASE;
	const instruction &s = (index >= 0) ? m_decoded[index] : EMPTY;

	if (!m_slot)
	{
		m_audio_in = m_serial_in[1];
		m_follow = -1;
	}
	const int follow = m_follow;
	m_follow = -1;

	eram_clock(s.eram_cmd);
	const int reading = eram_opened(12);
	if (reading == 1 || reading == 2)
		m_eram_read = m_eram[u16(m_eram_pos + eram_base(reading, 12) + ((reading == 2) ? m_tap : 0))] << 4;
	if (eram_opened(8) == 3)
		m_eram_address = eram_base(3, 8);

	int skip = -1;
	if (index >= 0 && follow < 0)
	{
		if (index + 3 < PROGRAM_SIZE && m_decoded[index + 3].skip())
			skip = index + 3;
		else if (index < 3 && s.skip())
			skip = index;
	}

	const operand_ref o = resolve(s);
	if (s.special())
		special(s, o);
	else if (s.opcode == OP_MUL)
		multiply(s, o);
	else if (!s.nop)
		mac(s, o);

	for (int n = 0; n < 2; n++)
	{
		m_history[n][2] = m_history[n][1];
		m_history[n][1] = m_history[n][0];
		m_history[n][0] = m_acc[n];
	}

	if (s.write_latch())
	{
		const s32 word = eram_word(m_eram_latch >> 4);
		m_eram[u16(m_eram_pos + m_eram_address)] = word;
		m_eram_read = word << 4;
	}

	u16 next = (m_pc + 1) & ADDRESS_MASK;
	if (skip >= 0)
	{
		for (int n = 0; n < 2; n++)
			m_history[n][2] = m_history[n][1] = m_history[n][0];
		next = (skip + 3 <= PROGRAM_SIZE / 2) ? PROGRAM_BASE + PROGRAM_SIZE / 2 : 0;
	}
	else if (follow >= 0 && m_jump)
	{
		m_jump = false;
		next = follow;
	}

	if (s.nop)
		m_hand_on = { FROM_MEMORY, 0 };
	else if (s.mac() || s.high_half())
		m_hand_on = o;
	else if (s.special() && !s.second_operand())
	{
		m_hand_on = { FROM_SPECIAL, 0 };
		if (s.jump())
			m_follow = s.coefficient << 1;
	}
	m_after_low = s.low_half();

	if (++m_slot >= PROGRAM_SIZE)
	{
		finish_sample();
		m_pc = PROGRAM_BASE;
		m_slot = 0;
	}
	else
		m_pc = next;
}

void roland_lsp_device::finish_sample()
{
	m_serial_out[0] = m_audio_out;
	m_buffer_pos = (m_buffer_pos - 1) & 0x7f;
	m_eram_pos--;
}

void roland_lsp_device::execute_run()
{
	while (m_icount > 0)
	{
		if (m_halted || !m_running)
		{
			m_icount = 0;
			return;
		}

		debugger_instruction_hook(m_pc);
		step();
		m_icount--;
	}
}

void roland_lsp_device::run_once()
{
	m_halted = false;
	for (int n = 0; m_running && n < PROGRAM_SIZE; n++)
	{
		m_icount = 1;
		execute_run();
		if (!m_slot)
			break;
	}
	m_halted = true;
}
