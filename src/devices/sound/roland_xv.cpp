// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Roland XV tone generator (TC223C660CF-503, RA08-503)

    The XP's successor: 64 voices a chip, two chips in an XV-5080.  Only
    the host interface is here: the 16-bit register window, the address
    spaces reached through it (the DSP program rows, the scalars, the wide
    records and the per-voice longs), the FIFO transfer engine onto the
    chip's own wave and sample memory, the object-indexed register file
    and the interrupt path.  It plays nothing.

***************************************************************************/

#include "emu.h"
#include "roland_xv.h"

#define LOG_REGS    (1U << 1)
#define LOG_XFER    (1U << 2)
#define LOG_SPACE   (1U << 3)
#define LOG_OBJECT  (1U << 4)
#define LOG_IRQ     (1U << 5)

#define VERBOSE (LOG_GENERAL | LOG_XFER | LOG_REGS | LOG_IRQ | LOG_OBJECT)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(ROLAND_XV, roland_xv_device, "roland_xv", "Roland XV tone generator")

roland_xv_device::roland_xv_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, ROLAND_XV, tag, owner, clock)
	, device_memory_interface(mconfig, *this)
	, device_sound_interface(mconfig, *this)
	, m_wave_config("wave", ENDIANNESS_LITTLE, 16, 32, -1)
	, m_int_callback(*this)
	, m_stream(nullptr)
{
}

device_memory_interface::space_config_vector roland_xv_device::memory_space_config() const
{
	return space_config_vector { std::make_pair(AS_WAVE, &m_wave_config) };
}

void roland_xv_device::device_start()
{
	space(AS_WAVE).specific(m_wave);
	m_stream = stream_alloc(0, 2, 44100);
	m_space = std::make_unique<u32[]>(0x10000);

	save_item(NAME(m_regs));
	save_item(NAME(m_object_regs));
	save_pointer(NAME(m_space), 0x10000);
	save_item(NAME(m_address));
	save_item(NAME(m_data_high));
	save_item(NAME(m_fifo));
	save_item(NAME(m_fifo_write));
	save_item(NAME(m_fifo_read));
	save_item(NAME(m_irq_enable));
	save_item(NAME(m_irq_pending));
	save_item(NAME(m_irq_voice));
	save_item(NAME(m_int_state));
}

void roland_xv_device::device_reset()
{
	std::fill(std::begin(m_regs), std::end(m_regs), 0);
	std::fill(&m_object_regs[0][0], &m_object_regs[0][0] + OBJECTS * (OBJECT_END - OBJECT_BASE), 0);
	std::fill_n(m_space.get(), 0x10000, 0);
	m_address = 0;
	m_data_high = 0;
	std::fill(std::begin(m_fifo), std::end(m_fifo), 0);
	fifo_rewind();
	m_irq_enable = 0;
	m_irq_pending = 0;
	std::fill(std::begin(m_irq_voice), std::end(m_irq_voice), 0);
	m_int_state = false;
	m_int_callback(0);
}

void roland_xv_device::sound_stream_update(sound_stream &stream)
{
}


//-------------------------------------------------
//  the window: a word is its high byte then its low byte, and the low byte
//  is where a write commits and a read takes its side effects
//-------------------------------------------------

u8 roland_xv_device::read(offs_t offset)
{
	const int word = (offset >> 1) & 0xff;
	const u16 data = word_peek(word);
	if (!BIT(offset, 0))
		return data >> 8;
	if (!machine().side_effects_disabled())
		word_taken(word);
	return data & 0xff;
}

void roland_xv_device::write(offs_t offset, u8 data)
{
	const int word = (offset >> 1) & 0xff;
	if (!BIT(offset, 0))
		m_regs[word] = (m_regs[word] & 0x00ff) | (data << 8);
	else
		word_w(word, (m_regs[word] & 0xff00) | data);
}

u16 roland_xv_device::word_peek(int word)
{
	switch (word)
	{
	case DATA_HIGH:
		return m_space[m_address] >> 16;

	case DATA_LOW:
		return m_space[m_address] & 0xffff;

	case FIFO:
		return m_fifo[m_fifo_read];

	case IRQ_MASK:
		return m_irq_pending;

	case STATUS:
		return 0;

	default:
		if (word > IRQ_VOICE && word < IRQ_VOICE + IRQ_REASONS)
			return m_irq_voice[word - IRQ_VOICE];
		if (word >= OBJECT_BASE && word < OBJECT_END)
			return m_object_regs[object()][word - OBJECT_BASE];
		return m_regs[word];
	}
}

void roland_xv_device::word_taken(int word)
{
	switch (word)
	{
	case DATA_LOW:
		LOGMASKED(LOG_SPACE, "%s: read %04x = %08x\n", machine().describe_context(), m_address, m_space[m_address]);
		m_address++;
		break;

	case FIFO:
		m_fifo_read = (m_fifo_read + 1) % FIFO_DEPTH;
		break;
	}
}

void roland_xv_device::word_w(int word, u16 data)
{
	m_regs[word] = data;
	switch (word)
	{
	case MODE:
		LOGMASKED(LOG_REGS, "%s: mode %04x\n", machine().describe_context(), data);
		break;

	case DATA_HIGH:
		m_data_high = data;
		break;

	case DATA_LOW:
		m_space[m_address] = (u32(m_data_high) << 16) | data;
		LOGMASKED(LOG_SPACE, "%s: write %04x = %08x\n", machine().describe_context(), m_address, m_space[m_address]);
		m_address++;
		m_data_high = 0;
		break;

	case ADDRESS:
		m_address = data;
		break;

	case FIFO:
		fifo_push(data);
		break;

	case FIFO_CONTROL:
		fifo_rewind();
		break;

	case COMMAND_STROBE:
		LOGMASKED(LOG_XFER, "%s: command %04x (strobe %04x)\n", machine().describe_context(), m_fifo[0], data);
		fifo_rewind();
		break;

	case XFER_COMMAND:
		if (BIT(data, 0))
			transfer_write();
		break;

	case READ_GO:
		if (BIT(data, 0))
			transfer_read();
		break;

	case IRQ_MASK:
		LOGMASKED(LOG_IRQ, "%s: interrupt mask %04x\n", machine().describe_context(), data);
		m_irq_enable = data;
		update_irq();
		break;

	case IRQ_ACK:
		m_irq_pending &= ~data;
		update_irq();
		break;

	default:
		if (word >= OBJECT_BASE && word < OBJECT_END)
		{
			m_object_regs[object()][word - OBJECT_BASE] = data;
			if (word == LEVEL_RAMP + 1)
				LOGMASKED(LOG_OBJECT, "%s: object %02x level %04x%04x\n", machine().describe_context(), object(), m_object_regs[object()][LEVEL_RAMP - OBJECT_BASE], data);
			else if (word == BLOCK_CONTROL + 1)
				LOGMASKED(LOG_OBJECT, "%s: object %02x control %04x%04x\n", machine().describe_context(), object(), m_object_regs[object()][BLOCK_CONTROL - OBJECT_BASE], data);
			else
				LOGMASKED(LOG_OBJECT, "%s: object %02x word %02x = %04x\n", machine().describe_context(), object(), word, data);
		}
		else if (word < 0x60)
			LOGMASKED(LOG_REGS, "%s: register %02x = %04x\n", machine().describe_context(), word, data);
		else
			LOGMASKED(LOG_GENERAL, "%s: unknown register %02x = %04x\n", machine().describe_context(), word, data);
		break;
	}
}


//-------------------------------------------------
//  the FIFO and the transfer engine.  Word 0x09 rewinds both pointers and
//  keeps the contents: the host writes it before pushing a write's data
//  and again before pulling a read's.  A read request fills the FIFO from
//  the wave space and a write request empties it there; both finish at
//  once, so the status word never shows either flag.
//-------------------------------------------------

void roland_xv_device::fifo_rewind()
{
	m_fifo_write = 0;
	m_fifo_read = 0;
}

void roland_xv_device::fifo_push(u16 data)
{
	m_fifo[m_fifo_write] = data;
	m_fifo_write = (m_fifo_write + 1) % FIFO_DEPTH;
}

void roland_xv_device::transfer_read()
{
	const u32 address = (u32(m_regs[READ_ADDRESS]) << 16) | m_regs[READ_ADDRESS + 1];
	const u32 length = (u32(m_regs[READ_LENGTH]) << 16) | m_regs[READ_LENGTH + 1];
	fifo_rewind();
	for (u32 i = 0; i < length && i < FIFO_DEPTH; i++)
		fifo_push(m_wave.read_word(address + i));
	LOGMASKED(LOG_XFER, "%s: read %08x x %x: %04x %04x %04x %04x\n", machine().describe_context(), address, length, m_fifo[0], m_fifo[1], m_fifo[2], m_fifo[3]);
}

void roland_xv_device::transfer_write()
{
	const u32 address = (u32(m_regs[WRITE_ADDRESS]) << 16) | m_regs[WRITE_ADDRESS + 1];
	const u32 length = (u32(m_regs[WRITE_LENGTH]) << 16) | m_regs[WRITE_LENGTH + 1];
	LOGMASKED(LOG_XFER, "%s: write %08x x %x: %04x %04x %04x %04x\n", machine().describe_context(), address, length, m_fifo[0], m_fifo[1], m_fifo[2], m_fifo[3]);
	for (u32 i = 0; i < length && i < FIFO_DEPTH; i++)
		m_wave.write_word(address + i, m_fifo[i]);
	fifo_rewind();
}


//-------------------------------------------------
//  interrupts: a pending reason is raised while its enable is set, and
//  acknowledged by writing its bit back
//-------------------------------------------------

void roland_xv_device::update_irq()
{
	const bool state = (m_irq_pending & m_irq_enable) != 0;
	if (state != m_int_state)
	{
		m_int_state = state;
		m_int_callback(state ? 1 : 0);
	}
}
