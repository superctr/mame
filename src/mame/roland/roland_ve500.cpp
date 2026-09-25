// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    BOSS VE-500 Vocal Performer

    An ESC2 (MB8AA4181) booting from a 4 MiB serial flash, with SDRAM on
    its external bus.

***************************************************************************/

#include "emu.h"

#include "roland_esc2.h"

#include "machine/generic_spi_flash.h"
#include "bus/midi/midi.h"
#include "diserial.h"

#include "speaker.h"


namespace {

class ve500_state : public driver_device, public device_serial_interface
{
public:
	ve500_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, device_serial_interface(mconfig, *this)
		, m_maincpu(*this, "maincpu")
		, m_spiflash(*this, "spiflash")
		, m_usbout(*this, "usbout")
	{
	}

	void ve500(machine_config &config) ATTR_COLD;

private:
	required_device<mb8aa4181_device> m_maincpu;
	required_device<generic_spi_flash_device> m_spiflash;
	required_device<midi_port_device> m_usbout;

	u8 m_usb_message[3] = { };
	u8 m_usb_count = 0;
	u8 m_usb_status = 0;
	bool m_usb_sysex = false;
	u8 m_usb_tx[256] = { };
	u8 m_usb_tx_head = 0;
	u16 m_usb_tx_count = 0;

	void mem_map(address_map &map) ATTR_COLD;

	void usb_packet(u8 cin);
	void usb_host_byte(u8 data);
	void usb_device_packet(u32 packet);

	virtual void rcv_complete() override;
	virtual void tra_callback() override;
	virtual void tra_complete() override;

	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;
};

void ve500_state::mem_map(address_map &map)
{
	map(0x60000000, 0x61ffffff).ram();
}

void ve500_state::machine_start()
{
	m_spiflash->set_rom_ptr(memregion("flash")->base());
	m_spiflash->set_rom_size(memregion("flash")->bytes());

	set_data_frame(1, 8, PARITY_NONE, STOP_BITS_1);
	set_rate(31250);

	save_item(NAME(m_usb_message));
	save_item(NAME(m_usb_count));
	save_item(NAME(m_usb_status));
	save_item(NAME(m_usb_sysex));
	save_item(NAME(m_usb_tx));
	save_item(NAME(m_usb_tx_head));
	save_item(NAME(m_usb_tx_count));
}

void ve500_state::usb_packet(u8 cin)
{
	m_maincpu->usb_rx_w(cin | m_usb_message[0] << 8 | m_usb_message[1] << 16 | m_usb_message[2] << 24);
	std::fill_n(m_usb_message, 3, 0);
	m_usb_count = 0;
}

void ve500_state::usb_host_byte(u8 data)
{
	if (data >= 0xf8)
	{
		m_maincpu->usb_rx_w(0x0f | data << 8);
		return;
	}
	if (data == 0xf7 && m_usb_sysex)
	{
		m_usb_message[m_usb_count++] = data;
		m_usb_sysex = false;
		usb_packet(4 + m_usb_count);
		return;
	}
	if (data & 0x80)
	{
		std::fill_n(m_usb_message, 3, 0);
		m_usb_message[0] = data;
		m_usb_count = 1;
		m_usb_sysex = data == 0xf0;
		m_usb_status = data < 0xf0 ? data : 0;
		if (data == 0xf6)
			usb_packet(5);
		return;
	}
	if (m_usb_sysex)
	{
		m_usb_message[m_usb_count++] = data;
		if (m_usb_count == 3)
			usb_packet(4);
		return;
	}
	if (!m_usb_count)
	{
		if (!m_usb_status)
			return;
		m_usb_message[0] = m_usb_status;
		m_usb_count = 1;
	}
	m_usb_message[m_usb_count++] = data;
	const u8 status = m_usb_message[0];
	const unsigned length = (status & 0xe0) == 0xc0 || status == 0xf1 || status == 0xf3 ? 2 : 3;
	if (m_usb_count == length)
		usb_packet(status < 0xf0 ? status >> 4 : length);
}

void ve500_state::usb_device_packet(u32 packet)
{
	static constexpr u8 LENGTH[16] = { 0, 0, 2, 3, 3, 1, 2, 3, 3, 3, 3, 3, 2, 2, 3, 1 };
	for (unsigned i = 0; i < LENGTH[packet & 0x0f] && m_usb_tx_count < std::size(m_usb_tx); i++)
		m_usb_tx[u8(m_usb_tx_head + m_usb_tx_count++)] = packet >> (8 * (i + 1));
	if (is_transmit_register_empty() && m_usb_tx_count)
		tra_complete();
}

void ve500_state::rcv_complete()
{
	receive_register_extract();
	usb_host_byte(get_received_char());
}

void ve500_state::tra_callback()
{
	m_usbout->write_txd(transmit_register_get_data_bit());
}

void ve500_state::tra_complete()
{
	if (m_usb_tx_count)
	{
		transmit_register_setup(m_usb_tx[m_usb_tx_head++]);
		m_usb_tx_count--;
	}
}

void ve500_state::machine_reset()
{
	m_maincpu->exint_w<7>(ASSERT_LINE);
	m_maincpu->usb_host_w(ASSERT_LINE);
}

static INPUT_PORTS_START(ve500)
INPUT_PORTS_END

void ve500_state::ve500(machine_config &config)
{
	MB8AA4181(config, m_maincpu, 156'000'000);
	m_maincpu->set_addrmap(AS_PROGRAM, &ve500_state::mem_map);
	m_maincpu->set_flash_tag("flash");
	m_maincpu->sfi_cs_cb().set(m_spiflash, FUNC(generic_spi_flash_device::cs_w));
	m_maincpu->sfi_tx_cb().set(m_spiflash, FUNC(generic_spi_flash_device::write));
	m_maincpu->sfi_rx_cb().set(m_spiflash, FUNC(generic_spi_flash_device::read));

	SPEAKER(config, "speaker", 2).front();
	mb8aa4181_dsp_device &dsp = *m_maincpu->subdevice<mb8aa4181_dsp_device>("dsp");
	dsp.set_clock(24.576_MHz_XTAL);
	dsp.add_route(0, "speaker", 1.0, 0);
	dsp.add_route(1, "speaker", 1.0, 1);

	GENERIC_SPI_FLASH(config, m_spiflash);

	m_maincpu->usb_tx_cb().set(FUNC(ve500_state::usb_device_packet));
	MIDI_PORT(config, "usbin", midiin_slot, "midiin").rxd_handler().set(FUNC(ve500_state::rx_w));
	MIDI_PORT(config, m_usbout, midiout_slot, "midiout");
}

ROM_START(ve500)
	ROM_REGION32_LE(0x400000, "flash", 0)
	// the 1.10 update image
	ROM_LOAD("ve500_110.bin", 0, 0x400000, BAD_DUMP CRC(f23c7976) SHA1(9a15b09141dd96e738bb314c67ee5d009874064e))
ROM_END

} // anonymous namespace


SYST(2015, ve500, 0, 0, ve500, ve500, ve500_state, empty_init, "Boss", "VE-500 Vocal Performer", MACHINE_NOT_WORKING | MACHINE_NO_SOUND)
