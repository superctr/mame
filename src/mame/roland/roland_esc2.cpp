// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Roland ESC2 (MB8AA4181)

    Cortex-M3 with on-chip memories, a serial-flash interface, multi-function
    serial channels, a dual timer, two DSP units and the audio serial ports.
    The boot ROM is undumped: reset copies the boot region of the serial
    flash into on-chip memory as the ROM's serial-flash boot would, and the
    entries of its API table the firmware relies on are high-level.

***************************************************************************/

#include "emu.h"
#include "roland_esc2.h"

#define LOG_UNMAPPED (1U << 1)
#define LOG_BOOTROM  (1U << 2)

#define VERBOSE (LOG_UNMAPPED | LOG_BOOTROM)
#include "logmacro.h"

#define LOGUNMAPPED(...) LOGMASKED(LOG_UNMAPPED, __VA_ARGS__)
#define LOGBOOTROM(...)  LOGMASKED(LOG_BOOTROM, __VA_ARGS__)


DEFINE_DEVICE_TYPE(MB8AA4181_MFS, mb8aa4181_mfs_device, "mb8aa4181_mfs", "Roland ESC2 multi-function serial channel")
DEFINE_DEVICE_TYPE(MB8AA4181, mb8aa4181_device, "mb8aa4181", "Roland ESC2 (MB8AA4181)")

namespace {

constexpr u32 IRAM_BASE = 0x01000000;
constexpr u32 IRAM_SIZE = 0x30000;
constexpr u32 BOOTROM_API = 0x3000;
constexpr u32 BOOTROM_STUBS = 0x1000;
constexpr u32 BOOTROM_ENTRIES = 0x100;

constexpr unsigned IRQ_DUALTIMER = 26;
constexpr unsigned IRQ_MFS = 64;
constexpr unsigned IRQ_ADC = 128;

} // anonymous namespace


//**************************************************************************
//  Multi-function serial channel
//**************************************************************************

mb8aa4181_mfs_device::mb8aa4181_mfs_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, MB8AA4181_MFS, tag, owner, clock)
	, device_serial_interface(mconfig, *this)
	, m_txd_cb(*this)
	, m_sot_cb(*this)
	, m_sin_cb(*this, 0xff)
	, m_rx_irq_cb(*this)
	, m_tx_irq_cb(*this)
	, m_status_irq_cb(*this)
{
}

void mb8aa4181_mfs_device::device_start()
{
	m_csio_timer = timer_alloc(FUNC(mb8aa4181_mfs_device::csio_done), this);

	save_item(NAME(m_scr_smr));
	save_item(NAME(m_ssr_escr));
	save_item(NAME(m_rdr));
	save_item(NAME(m_tdr));
	save_item(NAME(m_bgr));
	save_item(NAME(m_fcr));
	save_item(NAME(m_rx_threshold));
	save_item(NAME(m_tx_fifo));
	save_item(NAME(m_rx_fifo));
	save_item(NAME(m_tx_head));
	save_item(NAME(m_tx_count));
	save_item(NAME(m_rx_head));
	save_item(NAME(m_rx_count));
	save_item(NAME(m_tx_busy));
	save_item(NAME(m_tdr_full));
	save_item(NAME(m_rxd));
	save_item(NAME(m_csio_data));
	save_item(NAME(m_ibsr));
	save_item(NAME(m_i2c_address));
	save_item(NAME(m_i2c_active));
}

void mb8aa4181_mfs_device::device_reset()
{
	m_scr_smr = 0;
	m_ssr_escr = SSR_TDRE | SSR_TBI;
	m_rdr = 0;
	m_tdr = 0;
	m_bgr = 0;
	m_fcr = FCR_FDRQ;
	m_rx_threshold = 0;
	m_tx_head = m_tx_count = 0;
	m_rx_head = m_rx_count = 0;
	m_tx_busy = false;
	m_tdr_full = false;
	m_rxd = 1;
	m_csio_data = 0;
	m_ibsr = 0;
	m_i2c_address = 0;
	m_i2c_active = false;
	m_csio_timer->adjust(attotime::never);
	m_regs.clear();

	receive_register_reset();
	transmit_register_reset();
	update_frame();
	m_txd_cb(1);
	update_irq();
}

bool mb8aa4181_mfs_device::csio_mode() const
{
	return (m_scr_smr & SMR_MD) == 0x40;
}

bool mb8aa4181_mfs_device::tx_fifo_enabled() const
{
	return m_fcr & ((m_fcr & FCR_FSEL) ? FCR_FE2 : FCR_FE1);
}

bool mb8aa4181_mfs_device::rx_fifo_enabled() const
{
	return m_fcr & ((m_fcr & FCR_FSEL) ? FCR_FE1 : FCR_FE2);
}

void mb8aa4181_mfs_device::update_frame()
{
	const int bits = std::array<int, 8>{ 8, 5, 6, 7, 9, 8, 8, 8 }[m_ssr_escr & ESCR_L];
	set_data_frame(1, bits, PARITY_NONE, (m_scr_smr & SMR_SBL) ? STOP_BITS_2 : STOP_BITS_1);
}

void mb8aa4181_mfs_device::update_rate()
{
	if (m_bgr & 0x7fff)
		set_rate(clock(), (m_bgr & 0x7fff) + 1);
}

bool mb8aa4181_mfs_device::i2c_mode() const
{
	return (m_scr_smr & SMR_MD) == 0x80;
}

void mb8aa4181_mfs_device::update_irq()
{
	if (i2c_mode())
	{
		const u16 ibcr = m_scr_smr;
		const bool status = ((ibcr & IBCR_INTE) && (ibcr & (IBCR_INT | IBCR_BER))) || ((ibcr & IBCR_CNDE) && (m_ibsr & (IBSR_RSC | IBSR_SPC)));
		const bool rx = BIT(m_scr_smr, 3) && m_rx_count;
		const bool tx = (BIT(m_scr_smr, 2) && (m_ssr_escr & SSR_TDRE)) || ((m_fcr & FCR_FTIE) && (m_fcr & FCR_FDRQ));
		m_rx_irq_cb(rx ? ASSERT_LINE : CLEAR_LINE);
		m_tx_irq_cb(tx ? ASSERT_LINE : CLEAR_LINE);
		m_status_irq_cb(status ? ASSERT_LINE : CLEAR_LINE);
		return;
	}
	const u16 ssr = m_ssr_escr;
	const bool rx = (m_scr_smr & SCR_RIE) && (ssr & (SSR_RDRF | SSR_PE | SSR_FRE | SSR_ORE));
	const bool tx = ((m_scr_smr & SCR_TIE) && (ssr & SSR_TDRE)) || ((m_scr_smr & SCR_TBIE) && (ssr & SSR_TBI))
			|| ((m_fcr & FCR_FTIE) && (m_fcr & FCR_FDRQ));
	m_rx_irq_cb(rx ? ASSERT_LINE : CLEAR_LINE);
	m_tx_irq_cb(tx ? ASSERT_LINE : CLEAR_LINE);
}

void mb8aa4181_mfs_device::fifo_reset(bool fifo2)
{
	const bool tx = bool(m_fcr & FCR_FSEL) == fifo2;
	if (tx)
	{
		m_tx_head = m_tx_count = 0;
		m_fcr |= FCR_FDRQ;
	}
	else
	{
		m_rx_head = m_rx_count = 0;
		if (rx_fifo_enabled())
			m_ssr_escr &= ~SSR_RDRF;
	}
}

void mb8aa4181_mfs_device::start_tx()
{
	if (m_tx_busy || !(m_scr_smr & SCR_TXE))
		return;

	u8 data;
	if (m_tdr_full)
	{
		data = m_tdr;
		m_tdr_full = false;
	}
	else if (tx_fifo_enabled() && m_tx_count)
	{
		data = m_tx_fifo[m_tx_head];
		m_tx_head = (m_tx_head + 1) % FIFO_SIZE;
		m_tx_count--;
		if (!m_tx_count)
			m_fcr |= FCR_FDRQ;
	}
	else
		return;

	m_tx_busy = true;
	m_ssr_escr = (m_ssr_escr & ~SSR_TBI) | SSR_TDRE;
	if (csio_mode())
	{
		m_csio_data = data;
		m_csio_timer->adjust(attotime::from_ticks(8 * ((m_bgr & 0x7fff) + 1), clock()));
	}
	else
		transmit_register_setup(data);
	update_irq();
}

TIMER_CALLBACK_MEMBER(mb8aa4181_mfs_device::csio_done)
{
	m_sot_cb(m_csio_data);
	const u8 in = m_sin_cb();
	if (m_scr_smr & SCR_RXE)
	{
		if (rx_fifo_enabled() && m_rx_count < FIFO_SIZE)
		{
			m_rx_fifo[(m_rx_head + m_rx_count) % FIFO_SIZE] = in;
			m_rx_count++;
			m_ssr_escr |= SSR_RDRF;
		}
		else if (!rx_fifo_enabled())
		{
			if (m_ssr_escr & SSR_RDRF)
				m_ssr_escr |= SSR_ORE;
			m_rdr = in;
			m_ssr_escr |= SSR_RDRF;
		}
	}
	tra_complete();
}

void mb8aa4181_mfs_device::tra_callback()
{
	m_txd_cb(transmit_register_get_data_bit());
}

void mb8aa4181_mfs_device::tra_complete()
{
	m_tx_busy = false;
	start_tx();
	if (!m_tx_busy)
		m_ssr_escr |= SSR_TBI;
	update_irq();
}

void mb8aa4181_mfs_device::rcv_complete()
{
	receive_register_extract();
	const u8 data = get_received_char();
	if (!(m_scr_smr & SCR_RXE))
		return;

	if (is_receive_parity_error())
		m_ssr_escr |= SSR_PE;
	else if (is_receive_framing_error())
		m_ssr_escr |= SSR_FRE;
	else if (rx_fifo_enabled())
	{
		if (m_rx_count == FIFO_SIZE)
			m_ssr_escr |= SSR_ORE;
		else
		{
			m_rx_fifo[(m_rx_head + m_rx_count) % FIFO_SIZE] = data;
			m_rx_count++;
			m_ssr_escr |= SSR_RDRF;
		}
	}
	else if (m_ssr_escr & SSR_RDRF)
		m_ssr_escr |= SSR_ORE;
	else
	{
		m_rdr = data;
		m_ssr_escr |= SSR_RDRF;
	}

	if (m_ssr_escr & (SSR_PE | SSR_FRE | SSR_ORE))
		m_fcr &= (m_fcr & FCR_FSEL) ? ~FCR_FE1 : ~FCR_FE2;
	update_irq();
}

void mb8aa4181_mfs_device::rxd_w(int state)
{
	m_rxd = state;
	if (m_scr_smr & SCR_RXE)
		device_serial_interface::rx_w(state);
}

u32 mb8aa4181_mfs_device::read(offs_t offset, u32 mem_mask)
{
	if (i2c_mode())
		return i2c_read(offset);
	switch (offset)
	{
	case 0x00 / 4:
		return m_scr_smr & ~SCR_UPCL;

	case 0x04 / 4:
		return m_ssr_escr & ~SSR_REC;

	case 0x08 / 4:
		if (rx_fifo_enabled())
		{
			if (!m_rx_count)
				return m_rdr;
			m_rdr = m_rx_fifo[m_rx_head];
			if (!machine().side_effects_disabled())
			{
				m_rx_head = (m_rx_head + 1) % FIFO_SIZE;
				m_rx_count--;
				if (!m_rx_count)
				{
					m_ssr_escr &= ~SSR_RDRF;
					update_irq();
				}
			}
			return m_rdr;
		}
		if (!machine().side_effects_disabled())
		{
			m_ssr_escr &= ~SSR_RDRF;
			update_irq();
		}
		return m_rdr;

	case 0x0c / 4:
		return m_bgr;

	case 0x14 / 4:
		return m_fcr & ~(FCR_FCL1 | FCR_FCL2);

	case 0x18 / 4:
	{
		const u8 tx = m_tx_count;
		const u8 rx = m_rx_count;
		return (m_fcr & FCR_FSEL) ? (u32(tx) << 8 | rx) : (u32(rx) << 8 | tx);
	}

	default:
	{
		const auto it = m_regs.find(offset);
		return it != m_regs.end() ? it->second : 0;
	}
	}
}

void mb8aa4181_mfs_device::write(offs_t offset, u32 data, u32 mem_mask)
{
	if (i2c_mode() || (offset == 0 && ACCESSING_BITS_0_7 && (data & SMR_MD) == 0x80))
	{
		i2c_write(offset, data, mem_mask);
		return;
	}
	switch (offset)
	{
	case 0x00 / 4:
	{
		const u16 old = m_scr_smr;
		COMBINE_DATA(&m_scr_smr);
		if ((old ^ m_scr_smr) & SMR_SBL)
			update_frame();
		if ((m_scr_smr & ~old) & SCR_RXE)
			restart_rx();
		if (m_scr_smr & SCR_UPCL)
		{
			m_scr_smr &= ~SCR_UPCL;
			m_ssr_escr = (m_ssr_escr & 0xff) | SSR_TDRE | SSR_TBI;
			m_tdr_full = false;
			m_tx_busy = false;
			receive_register_reset();
			transmit_register_reset();
			m_txd_cb(1);
		}
		start_tx();
		update_irq();
		break;
	}

	case 0x04 / 4:
		if (ACCESSING_BITS_8_15 && (data & SSR_REC))
			m_ssr_escr &= ~(SSR_PE | SSR_FRE | SSR_ORE);
		if (ACCESSING_BITS_0_7 && u8(data) != u8(m_ssr_escr))
		{
			m_ssr_escr = (m_ssr_escr & 0xff00) | (data & 0xff);
			update_frame();
		}
		update_irq();
		break;

	case 0x08 / 4:
		if (tx_fifo_enabled())
		{
			if (m_tx_count < FIFO_SIZE)
			{
				m_tx_fifo[(m_tx_head + m_tx_count) % FIFO_SIZE] = data;
				m_tx_count++;
			}
			if (m_tx_count == FIFO_SIZE)
				m_fcr &= ~FCR_FDRQ;
		}
		else
		{
			m_tdr = data;
			m_tdr_full = true;
			m_ssr_escr &= ~(SSR_TDRE | SSR_TBI);
		}
		start_tx();
		update_irq();
		break;

	case 0x0c / 4:
		COMBINE_DATA(&m_bgr);
		update_rate();
		break;

	case 0x14 / 4:
	{
		const u16 old = m_fcr;
		u16 value = m_fcr;
		COMBINE_DATA(&value);
		m_fcr = (value & ~(FCR_FCL1 | FCR_FCL2 | FCR_FDRQ)) | (old & FCR_FDRQ);
		if (ACCESSING_BITS_8_15 && !(value & FCR_FDRQ) && m_tx_count)
			m_fcr &= ~FCR_FDRQ;
		if (value & FCR_FCL1)
			fifo_reset(false);
		if (value & FCR_FCL2)
			fifo_reset(true);
		start_tx();
		update_irq();
		break;
	}

	case 0x18 / 4:
		m_rx_threshold = BIT(data, (m_fcr & FCR_FSEL) ? 0 : 8, 8);
		break;

	default:
		COMBINE_DATA(&m_regs[offset]);
		break;
	}
}

u32 mb8aa4181_mfs_device::i2c_read(offs_t offset)
{
	switch (offset)
	{
	case 0x00 / 4:
		return (m_scr_smr & ~IBCR_ACT) | (m_i2c_active ? IBCR_ACT : 0);
	case 0x04 / 4:
		return (m_ssr_escr & 0xff00) | m_ibsr;
	case 0x08 / 4:
		if (m_rx_count)
		{
			const u8 data = m_rx_fifo[m_rx_head];
			if (!machine().side_effects_disabled())
			{
				m_rx_head = (m_rx_head + 1) % FIFO_SIZE;
				m_rx_count--;
			}
			return data;
		}
		return m_rdr;
	case 0x0c / 4:
		return m_bgr;
	case 0x10 / 4:
		return m_i2c_address;
	case 0x14 / 4:
		return m_fcr & ~(FCR_FCL1 | FCR_FCL2);
	case 0x18 / 4:
		return (m_fcr & FCR_FSEL) ? (u32(m_tx_count) << 8 | m_rx_count) : (u32(m_rx_count) << 8 | m_tx_count);
	default:
	{
		const auto it = m_regs.find(offset);
		return it != m_regs.end() ? it->second : 0;
	}
	}
}

void mb8aa4181_mfs_device::i2c_start()
{
	u8 address;
	if (m_tx_count)
	{
		address = m_tx_fifo[m_tx_head];
		m_tx_head = (m_tx_head + 1) % FIFO_SIZE;
		m_tx_count--;
		if (!m_tx_count)
			m_fcr |= FCR_FDRQ;
	}
	else
		address = m_tdr;

	m_i2c_active = true;
	m_ibsr = (m_ibsr & ~(IBSR_RSA | IBSR_AL)) | IBSR_BB | IBSR_FBT | IBSR_RACK;
	if (!BIT(address, 0))
		m_ibsr |= IBSR_TRX;
	else
		m_ibsr &= ~IBSR_TRX;
	m_scr_smr |= IBCR_INT;
}

void mb8aa4181_mfs_device::i2c_write(offs_t offset, u32 data, u32 mem_mask)
{
	switch (offset)
	{
	case 0x00 / 4:
	{
		const u16 old = m_scr_smr;
		u16 value = m_scr_smr;
		COMBINE_DATA(&value);
		m_scr_smr = (value & ~(IBCR_BER | IBCR_INT | IBCR_ACT)) | (old & (IBCR_BER | IBCR_INT));
		if (ACCESSING_BITS_8_15 && !(value & IBCR_INT))
			m_scr_smr &= ~(IBCR_INT | IBCR_BER);
		if (!i2c_mode())
			break;
		if ((m_scr_smr & ~old & IBCR_MSS) && BIT(m_i2c_address, 15) && !(m_ibsr & IBSR_BB))
			i2c_start();
		else if ((old & ~m_scr_smr & IBCR_MSS) && m_i2c_active)
		{
			m_i2c_active = false;
			m_ibsr = (m_ibsr & ~(IBSR_BB | IBSR_FBT | IBSR_TRX)) | IBSR_SPC;
			m_scr_smr &= ~IBCR_INT;
		}
		else if (ACCESSING_BITS_8_15 && (value & IBCR_ACT) && m_i2c_active && (m_scr_smr & IBCR_MSS))
		{
			m_ibsr |= IBSR_RSC;
			i2c_start();
		}
		break;
	}

	case 0x04 / 4:
		if (ACCESSING_BITS_0_7)
			m_ibsr &= data | ~(IBSR_RSC | IBSR_SPC | IBSR_AL);
		if (ACCESSING_BITS_8_15 && (data & SSR_REC))
			m_ssr_escr &= ~SSR_ORE;
		break;

	case 0x08 / 4:
		m_tdr = data & 0xff;
		if (m_tx_count < FIFO_SIZE)
		{
			m_tx_fifo[(m_tx_head + m_tx_count) % FIFO_SIZE] = data;
			m_tx_count++;
		}
		if (m_tx_count == FIFO_SIZE)
			m_fcr &= ~FCR_FDRQ;
		break;

	case 0x0c / 4:
		COMBINE_DATA(&m_bgr);
		break;

	case 0x10 / 4:
		COMBINE_DATA(&m_i2c_address);
		if (!BIT(m_i2c_address, 15))
		{
			m_i2c_active = false;
			m_ibsr = 0;
			m_scr_smr &= ~(IBCR_MSS | IBCR_INT | IBCR_BER);
		}
		break;

	case 0x14 / 4:
	{
		u16 value = m_fcr;
		COMBINE_DATA(&value);
		const u16 old = m_fcr;
		m_fcr = (value & ~(FCR_FCL1 | FCR_FCL2 | FCR_FDRQ)) | (old & FCR_FDRQ);
		if (ACCESSING_BITS_8_15 && !(value & FCR_FDRQ) && m_tx_count)
			m_fcr &= ~FCR_FDRQ;
		if (value & FCR_FCL1)
			fifo_reset(false);
		if (value & FCR_FCL2)
			fifo_reset(true);
		break;
	}

	case 0x18 / 4:
		m_rx_threshold = BIT(data, (m_fcr & FCR_FSEL) ? 0 : 8, 8);
		break;

	default:
		COMBINE_DATA(&m_regs[offset]);
		break;
	}
	update_irq();
}

void mb8aa4181_mfs_device::restart_rx()
{
	receive_register_reset();
	device_serial_interface::rx_w(1);
	if (!m_rxd)
		device_serial_interface::rx_w(0);
}

void mb8aa4181_mfs_device::rom_mode(u32 mode)
{
	m_scr_smr = (m_scr_smr & 0xff00) | SMR_SOE;
	m_fcr |= FCR_FE1 | FCR_FE2;
	update_frame();
}

void mb8aa4181_mfs_device::rom_baud(u32 baud)
{
	if (baud)
		m_bgr = std::clamp<u32>((clock() + baud / 2) / baud, 1, 0x8000) - 1;
	const u16 old = m_scr_smr;
	m_scr_smr |= SCR_RIE | SCR_TIE | SCR_TBIE | SCR_RXE | SCR_TXE;
	update_frame();
	update_rate();
	if ((m_scr_smr & ~old) & SCR_RXE)
		restart_rx();
	update_irq();
}


//**************************************************************************
//  The chip
//**************************************************************************

mb8aa4181_device::mb8aa4181_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: cortex_m3_device(mconfig, MB8AA4181, tag, owner, clock, address_map_constructor(FUNC(mb8aa4181_device::internal_map), this))
	, m_flash(*this, finder_base::DUMMY_TAG)
	, m_mfs(*this, "mfs%u", 0U)
	, m_iram(*this, "iram", IRAM_SIZE, ENDIANNESS_LITTLE)
	, m_gpio_out_cb(*this)
	, m_gpio_in_cb(*this, 0)
	, m_adc_in_cb(*this, 0)
{
	set_num_irq(160);
	set_vtor_reset(IRAM_BASE);
}

void mb8aa4181_device::device_add_mconfig(machine_config &config)
{
	for (unsigned i = 0; i < 8; i++)
	{
		MB8AA4181_MFS(config, m_mfs[i], DERIVED_CLOCK(1, 2));
		m_mfs[i]->rx_irq_cb().set([this, i] (int state) { set_irq_line(IRQ_MFS + 4 * i, state); });
		m_mfs[i]->tx_irq_cb().set([this, i] (int state) { set_irq_line(IRQ_MFS + 4 * i + 1, state); });
		m_mfs[i]->status_irq_cb().set([this, i] (int state) { set_irq_line(IRQ_MFS + 4 * i + 2, state); });
	}
}

void mb8aa4181_device::internal_map(address_map &map)
{
	map(0x00000000, 0x5fffffff).rw(FUNC(mb8aa4181_device::unmapped_r), FUNC(mb8aa4181_device::unmapped_w));
	map(0x01000000, 0x0102ffff).ram().share(m_iram);
	map(0x02000000, 0x0200ffff).r(FUNC(mb8aa4181_device::bootrom_r));
	map(0x03000000, 0x03ffffff).r(FUNC(mb8aa4181_device::flash_r));
	map(0x20000000, 0x2000ffff).ram();
	map(0x40008000, 0x4000a0ff).ram();
	map(0x40002000, 0x4000200b).rw(FUNC(mb8aa4181_device::dmaflag_r), FUNC(mb8aa4181_device::dmaflag_w));
	map(0x40017040, 0x40017047).r(FUNC(mb8aa4181_device::timebase_r));
	map(0x40025000, 0x40025fff).rw(FUNC(mb8aa4181_device::dualtimer_r), FUNC(mb8aa4181_device::dualtimer_w));
	for (unsigned i = 0; i < 8; i++)
		map(0x40028000 + 0x400 * i, 0x400283ff + 0x400 * i).rw(m_mfs[i], FUNC(mb8aa4181_mfs_device::read), FUNC(mb8aa4181_mfs_device::write));
	map(0x4002c000, 0x4002c03f).rw(FUNC(mb8aa4181_device::adc_r), FUNC(mb8aa4181_device::adc_w));
	map(0x4002d000, 0x4002d03f).rw(FUNC(mb8aa4181_device::gpio_r), FUNC(mb8aa4181_device::gpio_w));
	map(0x40100000, 0x4017ffff).rw(FUNC(mb8aa4181_device::dsp_r), FUNC(mb8aa4181_device::dsp_w));
	map(0x40045000, 0x40045fff).ram();
}

void mb8aa4181_device::device_start()
{
	cortex_m3_device::device_start();

	for (int i = 0; i < 2; i++)
		m_timer[i] = timer_alloc(FUNC(mb8aa4181_device::dualtimer_expired), this);
	m_adc_timer = timer_alloc(FUNC(mb8aa4181_device::adc_done), this);

	save_item(NAME(m_gpio_out));
	save_item(NAME(m_dma_flags));
	save_item(NAME(m_adc_ctrl));
	save_item(NAME(m_adc_config));
	save_item(NAME(m_adc_status));
	save_item(NAME(m_adc_data));
	save_item(NAME(m_timer_load));
	save_item(NAME(m_timer_bgload));
	save_item(NAME(m_timer_ctrl));
	save_item(NAME(m_timer_int));
}

void mb8aa4181_device::device_reset()
{
	const u32 length = std::min<u32>(m_flash[0x0c / 4], IRAM_SIZE);
	for (u32 i = 0; i < length; i += 4)
		m_iram[i / 4] = m_flash[(0x30 + i) / 4];

	m_regs.clear();
	m_dsp.clear();
	m_dma_flags = 0;
	m_adc_ctrl = m_adc_config = m_adc_status = 0;
	std::fill_n(m_adc_data, 8, 0);
	m_adc_timer->adjust(attotime::never);
	for (int i = 0; i < 8; i++)
	{
		m_gpio_out[i] = 0;
		m_gpio_out_cb[i](0);
	}
	for (int i = 0; i < 2; i++)
	{
		m_timer_load[i] = 0;
		m_timer_bgload[i] = 0;
		m_timer_ctrl[i] = 0x20;
		m_timer_int[i] = false;
		m_timer[i]->adjust(attotime::never);
	}

	cortex_m3_device::device_reset();
	for (int i = 0; i < 2; i++)
		dualtimer_irq(i);
}

u32 mb8aa4181_device::bootrom_call(u32 entry)
{
	const u32 r0 = state_int(ARMV7M_R0);
	const u32 r1 = state_int(ARMV7M_R1);
	switch (entry)
	{
	case 0x27c:
		if (r0 < 8)
			m_mfs[r0]->rom_mode(r1);
		return 0;

	case 0x280:
		return 0;

	case 0x284:
		if (r0 < 8)
			m_mfs[r0]->rom_baud(r1);
		return 0;

	case 0x134:
		rom_dma(r1);
		return 0;

	default:
		LOGBOOTROM("%s: boot ROM API %03x (%08x %08x %08x %08x)\n", machine().describe_context(), entry,
				r0, r1, state_int(ARMV7M_R2), state_int(ARMV7M_R3));
		return 0;
	}
}

u32 mb8aa4181_device::bootrom_r(offs_t offset, u32 mem_mask)
{
	const u32 address = offset * 4;
	if (address >= BOOTROM_API && address < BOOTROM_API + BOOTROM_ENTRIES * 4)
		return 0x02000000 + BOOTROM_STUBS + (address - BOOTROM_API) + 1;
	if (address >= BOOTROM_STUBS && address < BOOTROM_STUBS + BOOTROM_ENTRIES * 4)
	{
		u32 result = 0;
		if (!machine().side_effects_disabled() && ACCESSING_BITS_0_15)
			result = bootrom_call(address - BOOTROM_STUBS);
		return 0x47702000 | (result & 0xff);
	}
	if (!machine().side_effects_disabled())
		LOGBOOTROM("%s: boot ROM read %08x & %08x\n", machine().describe_context(), 0x02000000 + address, mem_mask);
	return 0;
}

u32 mb8aa4181_device::flash_r(offs_t offset)
{
	return m_flash[offset & (m_flash.length() - 1)];
}


//  time base

u32 mb8aa4181_device::timebase_r(offs_t offset)
{
	const u64 ticks = machine().time().as_ticks(clock());
	return offset ? u32(ticks >> 32) : u32(ticks);
}


//  ADC

TIMER_CALLBACK_MEMBER(mb8aa4181_device::adc_done)
{
	for (int i = 0; i < 8; i++)
		m_adc_data[i] = m_adc_in_cb[i]() & 0xfff;
	m_adc_ctrl &= ~0x400;
	m_adc_status = 1;
	set_irq_line(IRQ_ADC, ASSERT_LINE);
}

u32 mb8aa4181_device::adc_r(offs_t offset)
{
	switch (offset)
	{
	case 0: return m_adc_ctrl;
	case 1: return m_adc_config;
	case 2: return m_adc_status;
	case 4: case 5: case 6: case 7: case 8: case 9: case 10: case 11: return m_adc_data[offset - 4];
	default: return 0;
	}
}

void mb8aa4181_device::adc_w(offs_t offset, u32 data, u32 mem_mask)
{
	switch (offset)
	{
	case 0:
		COMBINE_DATA(&m_adc_ctrl);
		if (BIT(m_adc_ctrl, 10) && BIT(m_adc_ctrl, 8) && m_adc_timer->remaining().is_never())
			m_adc_timer->adjust(attotime::from_usec(8));
		break;
	case 1:
		COMBINE_DATA(&m_adc_config);
		break;
	case 2:
		m_adc_status = 0;
		set_irq_line(IRQ_ADC, CLEAR_LINE);
		break;
	}
}


//  DMA

void mb8aa4181_device::rom_dma(u32 desc)
{
	address_space &space = this->space(AS_PROGRAM);
	for (int guard = 0; desc && (desc & ~0x1ff) != 0x40012600 && guard < 0x10000; guard++)
	{
		u32 src = space.read_dword(desc);
		u32 dst = space.read_dword(desc + 4);
		const u32 next = space.read_dword(desc + 8);
		const u32 control = space.read_dword(desc + 12);
		const unsigned swidth = 1 << std::min(BIT(control, 18, 3), 2U);
		const unsigned dwidth = 1 << std::min(BIT(control, 21, 3), 2U);
		const u32 bytes = BIT(control, 0, 12) * swidth;
		for (u32 i = 0; i < bytes; i += std::min(swidth, dwidth))
		{
			const unsigned width = std::min(swidth, dwidth);
			const u32 data = width == 4 ? space.read_dword(src) : width == 2 ? space.read_word(src) : space.read_byte(src);
			if (width == 4)
				space.write_dword(dst, data);
			else if (width == 2)
				space.write_word(dst, data);
			else
				space.write_byte(dst, data);
			if (BIT(control, 26))
				src += width;
			if (BIT(control, 27))
				dst += width;
		}
		desc = next;
	}
	m_dma_flags |= 1 << 5;
}

u32 mb8aa4181_device::dmaflag_r(offs_t offset)
{
	return offset == 1 ? m_dma_flags : 0;
}

void mb8aa4181_device::dmaflag_w(offs_t offset, u32 data, u32 mem_mask)
{
	if (offset == 2)
		m_dma_flags &= ~(data & mem_mask);
}


//  DSP window

u32 mb8aa4181_device::dsp_r(offs_t offset, u32 mem_mask)
{
	if (offset == 0x48 / 4)
		return u32(machine().time().as_ticks(clock() / 3250));
	const auto it = m_dsp.find(offset);
	return it != m_dsp.end() ? it->second : 0;
}

void mb8aa4181_device::dsp_w(offs_t offset, u32 data, u32 mem_mask)
{
	if (offset == 0x0c / 4)
		return;
	COMBINE_DATA(&m_dsp[offset]);
}


//  GPIO

u32 mb8aa4181_device::gpio_r(offs_t offset)
{
	const unsigned port = offset >> 1;
	if (BIT(offset, 0))
		return machine().side_effects_disabled() ? 0 : m_gpio_in_cb[port]();
	return m_gpio_out[port];
}

void mb8aa4181_device::gpio_w(offs_t offset, u32 data, u32 mem_mask)
{
	const unsigned port = offset >> 1;
	if (BIT(offset, 0))
		return;
	const u32 old = m_gpio_out[port];
	COMBINE_DATA(&m_gpio_out[port]);
	if (m_gpio_out[port] != old)
		m_gpio_out_cb[port](m_gpio_out[port]);
}


//  dual timer

attotime mb8aa4181_device::dualtimer_period(int which) const
{
	static constexpr unsigned PRESCALE[4] = { 1, 16, 256, 256 };
	return attotime::from_ticks(PRESCALE[BIT(m_timer_ctrl[which], 2, 2)], clock() / 2);
}

void mb8aa4181_device::dualtimer_start(int which, u32 count)
{
	if (!BIT(m_timer_ctrl[which], 7))
	{
		m_timer[which]->adjust(attotime::never);
		return;
	}
	if (!BIT(m_timer_ctrl[which], 1))
		count &= 0xffff;
	m_timer[which]->adjust(dualtimer_period(which) * (u64(count) + 1), which);
}

u32 mb8aa4181_device::dualtimer_value(int which) const
{
	if (m_timer[which]->remaining().is_never())
		return 0;
	const attotime period = dualtimer_period(which);
	const u64 ticks = m_timer[which]->remaining().as_ticks(period.as_hz());
	return ticks ? u32(ticks - 1) : 0;
}

void mb8aa4181_device::dualtimer_irq(int which)
{
	set_irq_line(IRQ_DUALTIMER + which, (m_timer_int[which] && BIT(m_timer_ctrl[which], 5)) ? ASSERT_LINE : CLEAR_LINE);
}

TIMER_CALLBACK_MEMBER(mb8aa4181_device::dualtimer_expired)
{
	const int which = param;
	m_timer_int[which] = true;
	dualtimer_irq(which);
	if (BIT(m_timer_ctrl[which], 0))
	{
		m_timer[which]->adjust(attotime::never);
		return;
	}
	dualtimer_start(which, BIT(m_timer_ctrl[which], 6) ? m_timer_bgload[which] : 0xffffffff);
}

u32 mb8aa4181_device::dualtimer_r(offs_t offset)
{
	const int which = BIT(offset, 3);
	switch (offset & 7)
	{
	case 0: return m_timer_load[which];
	case 1: return dualtimer_value(which);
	case 2: return m_timer_ctrl[which];
	case 4: return m_timer_int[which];
	case 5: return m_timer_int[which] && BIT(m_timer_ctrl[which], 5);
	case 6: return m_timer_bgload[which];
	default: return 0;
	}
}

void mb8aa4181_device::dualtimer_w(offs_t offset, u32 data, u32 mem_mask)
{
	if (offset >= 0x10)
		return;
	const int which = BIT(offset, 3);
	switch (offset & 7)
	{
	case 0:
		m_timer_load[which] = m_timer_bgload[which] = data;
		dualtimer_start(which, data);
		break;

	case 2:
	{
		const u32 old = m_timer_ctrl[which];
		m_timer_ctrl[which] = data & 0xef;
		if (BIT(data, 7) != BIT(old, 7))
			dualtimer_start(which, BIT(data, 7) ? m_timer_load[which] : 0);
		dualtimer_irq(which);
		break;
	}

	case 3:
		m_timer_int[which] = false;
		dualtimer_irq(which);
		break;

	case 6:
		m_timer_bgload[which] = data;
		break;
	}
}


//  everything not yet modelled

u32 mb8aa4181_device::unmapped_r(offs_t offset, u32 mem_mask)
{
	const auto it = m_regs.find(offset);
	u32 data = it != m_regs.end() ? it->second : 0;
	if (offset == 0x4002dff4 / 4)
	{
		const auto req = m_regs.find(0x4002dff8 / 4);
		data = req != m_regs.end() ? BIT(req->second, 0) : 0;
	}
	if (!machine().side_effects_disabled() && m_logged[offset]++ < 16)
		LOGUNMAPPED("%s: read %08x & %08x = %08x\n", machine().describe_context(), offset * 4, mem_mask, data);
	return data;
}

void mb8aa4181_device::unmapped_w(offs_t offset, u32 data, u32 mem_mask)
{
	if (m_logged[offset | 0x80000000]++ < 16)
		LOGUNMAPPED("%s: write %08x = %08x & %08x\n", machine().describe_context(), offset * 4, data, mem_mask);
	u32 &reg = m_regs[offset];
	COMBINE_DATA(&reg);
}
