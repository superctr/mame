// license:BSD-3-Clause
// copyright-holders:superctr
/***************************************************************************

    Roland SYSTEM-1 Variable Synthesizer

    Two ESC2 (MB8AA4181) chips.  The master boots from the 4 MiB serial
    flash, boots the slave over I2C and sends it its application over an
    internal USB link; it reads the panel through a multiplexed ADC and
    takes MIDI.  The slave scans the keyboard and runs the oscillators,
    whose outputs its DSP hands to the master's through the audio ports.

***************************************************************************/

#include "emu.h"

#include "roland_esc2.h"

#include "machine/generic_spi_flash.h"
#include "bus/midi/midi.h"

#include "speaker.h"


namespace {

class system1_state : public driver_device
{
public:
	system1_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_subcpu(*this, "subcpu")
		, m_spiflash(*this, "spiflash")
		, m_control(*this, "CTRL%u", 0U)
	{
	}

	void system1(machine_config &config) ATTR_COLD;

private:
	required_device<mb8aa4181_device> m_maincpu;
	required_device<mb8aa4181_device> m_subcpu;
	required_device<generic_spi_flash_device> m_spiflash;
	required_ioport_array<64> m_control;

	u32 m_port0 = 0;

	void mem_map(address_map &map) ATTR_COLD;
	template <unsigned N> u16 control_r() { return (m_control[((BIT(m_port0, 19) << 2) | (m_port0 & 3)) * 8 + N]->read() << 4) | 8; }
	void sub_map(address_map &map) ATTR_COLD;

	virtual void machine_start() override ATTR_COLD;
};

void system1_state::mem_map(address_map &map)
{
	map(0x60000000, 0x61ffffff).ram();
}

void system1_state::sub_map(address_map &map)
{
	map(0x60000000, 0x61ffffff).ram();
}

void system1_state::machine_start()
{
	save_item(NAME(m_port0));
	m_spiflash->set_rom_ptr(memregion("flash")->base());
	m_spiflash->set_rom_size(memregion("flash")->bytes());
}

static INPUT_PORTS_START(system1)
	PORT_START("CTRL0")
	PORT_ADJUSTER(128, "Pitch Env Decay") PORT_MINMAX(0, 255)
	PORT_START("CTRL1")
	PORT_ADJUSTER(255, "Amp Sustain") PORT_MINMAX(0, 255)
	PORT_START("CTRL2")
	PORT_ADJUSTER(128, "Tempo") PORT_MINMAX(0, 255)
	PORT_START("CTRL3")
	PORT_ADJUSTER(128, "LFO Filter Depth") PORT_MINMAX(0, 255)
	PORT_START("CTRL4")
	PORT_ADJUSTER(255, "Filter Cutoff") PORT_MINMAX(0, 255)
	PORT_START("CTRL5")
	PORT_ADJUSTER(128, "Control 5.0") PORT_MINMAX(0, 255)
	PORT_START("CTRL6")
	PORT_ADJUSTER(128, "Control 6.0") PORT_MINMAX(0, 255)
	PORT_START("CTRL7")
	PORT_ADJUSTER(160, "Volume") PORT_MINMAX(0, 255)
	PORT_START("CTRL8")
	PORT_ADJUSTER(128, "Filter Env Decay") PORT_MINMAX(0, 255)
	PORT_START("CTRL9")
	PORT_ADJUSTER(128, "Control 1.1") PORT_MINMAX(0, 255)
	PORT_START("CTRL10")
	PORT_ADJUSTER(0, "OSC2 Mod") PORT_MINMAX(0, 255)
	PORT_START("CTRL11")
	PORT_ADJUSTER(0, "Portamento") PORT_MINMAX(0, 255)
	PORT_START("CTRL12")
	PORT_ADJUSTER(0, "Mixer OSC2") PORT_MINMAX(0, 255)
	PORT_START("CTRL13")
	PORT_ADJUSTER(0, "Reverb") PORT_MINMAX(0, 255)
	PORT_START("CTRL14")
	PORT_ADJUSTER(128, "Control 6.1") PORT_MINMAX(0, 255)
	PORT_START("CTRL15")
	PORT_ADJUSTER(128, "Control 7.1") PORT_MINMAX(0, 255)
	PORT_START("CTRL16")
	PORT_ADJUSTER(0, "Filter Env Attack") PORT_MINMAX(0, 255)
	PORT_START("CTRL17")
	PORT_ADJUSTER(40, "Amp Release") PORT_MINMAX(0, 255)
	PORT_START("CTRL18")
	PORT_ADJUSTER(128, "OSC2 Color") PORT_MINMAX(0, 255)
	PORT_START("CTRL19")
	PORT_ADJUSTER(128, "LFO Amp Depth") PORT_MINMAX(0, 255)
	PORT_START("CTRL20")
	PORT_ADJUSTER(128, "Pitch Env Depth") PORT_MINMAX(0, 255)
	PORT_START("CTRL21")
	PORT_ADJUSTER(0, "Delay") PORT_MINMAX(0, 255)
	PORT_START("CTRL22")
	PORT_ADJUSTER(128, "Control 6.2") PORT_MINMAX(0, 255)
	PORT_START("CTRL23")
	PORT_ADJUSTER(128, "Control 7.2") PORT_MINMAX(0, 255)
	PORT_START("CTRL24")
	PORT_ADJUSTER(0, "Pitch Env Attack") PORT_MINMAX(0, 255)
	PORT_START("CTRL25")
	PORT_ADJUSTER(255, "Mixer OSC1") PORT_MINMAX(0, 255)
	PORT_START("CTRL26")
	PORT_ADJUSTER(0, "OSC1 Cross Mod") PORT_MINMAX(0, 255)
	PORT_START("CTRL27")
	PORT_ADJUSTER(128, "LFO Pitch Depth") PORT_MINMAX(0, 255)
	PORT_START("CTRL28")
	PORT_ADJUSTER(0, "Filter Resonance") PORT_MINMAX(0, 255)
	PORT_START("CTRL29")
	PORT_ADJUSTER(128, "Delay Time") PORT_MINMAX(0, 255)
	PORT_START("CTRL30")
	PORT_ADJUSTER(128, "Control 6.3") PORT_MINMAX(0, 255)
	PORT_START("CTRL31")
	PORT_ADJUSTER(128, "Control 7.3") PORT_MINMAX(0, 255)
	PORT_START("CTRL32")
	PORT_ADJUSTER(128, "Amp Decay") PORT_MINMAX(0, 255)
	PORT_START("CTRL33")
	PORT_ADJUSTER(128, "OSC2 Wave") PORT_MINMAX(0, 255)
	PORT_START("CTRL34")
	PORT_ADJUSTER(128, "OSC1 Range") PORT_MINMAX(0, 255)
	PORT_START("CTRL35")
	PORT_ADJUSTER(128, "LFO Wave") PORT_MINMAX(0, 255)
	PORT_START("CTRL36")
	PORT_ADJUSTER(0, "Crusher") PORT_MINMAX(0, 255)
	PORT_START("CTRL37")
	PORT_ADJUSTER(0, "Filter HPF") PORT_MINMAX(0, 255)
	PORT_START("CTRL38")
	PORT_ADJUSTER(128, "Control 6.4") PORT_MINMAX(0, 255)
	PORT_START("CTRL39")
	PORT_ADJUSTER(128, "Control 7.4") PORT_MINMAX(0, 255)
	PORT_START("CTRL40")
	PORT_ADJUSTER(0, "Amp Attack") PORT_MINMAX(0, 255)
	PORT_START("CTRL41")
	PORT_ADJUSTER(128, "OSC2 Range") PORT_MINMAX(0, 255)
	PORT_START("CTRL42")
	PORT_ADJUSTER(0, "OSC1 Mod") PORT_MINMAX(0, 255)
	PORT_START("CTRL43")
	PORT_ADJUSTER(0, "LFO Fade") PORT_MINMAX(0, 255)
	PORT_START("CTRL44")
	PORT_ADJUSTER(128, "Amp Tone") PORT_MINMAX(0, 255)
	PORT_START("CTRL45")
	PORT_ADJUSTER(128, "Control 5.5") PORT_MINMAX(0, 255)
	PORT_START("CTRL46")
	PORT_ADJUSTER(128, "Control 6.5") PORT_MINMAX(0, 255)
	PORT_START("CTRL47")
	PORT_ADJUSTER(128, "Control 7.5") PORT_MINMAX(0, 255)
	PORT_START("CTRL48")
	PORT_ADJUSTER(255, "Filter Env Sustain") PORT_MINMAX(0, 255)
	PORT_START("CTRL49")
	PORT_ADJUSTER(128, "OSC2 Fine Tune") PORT_MINMAX(0, 255)
	PORT_START("CTRL50")
	PORT_ADJUSTER(128, "OSC1 Wave") PORT_MINMAX(0, 255)
	PORT_START("CTRL51")
	PORT_ADJUSTER(128, "Control 3.6") PORT_MINMAX(0, 255)
	PORT_START("CTRL52")
	PORT_ADJUSTER(128, "Filter Env Depth") PORT_MINMAX(0, 255)
	PORT_START("CTRL53")
	PORT_ADJUSTER(0, "Mixer Noise") PORT_MINMAX(0, 255)
	PORT_START("CTRL54")
	PORT_ADJUSTER(128, "Control 6.6") PORT_MINMAX(0, 255)
	PORT_START("CTRL55")
	PORT_ADJUSTER(128, "Control 7.6") PORT_MINMAX(0, 255)
	PORT_START("CTRL56")
	PORT_ADJUSTER(40, "Filter Env Release") PORT_MINMAX(0, 255)
	PORT_START("CTRL57")
	PORT_ADJUSTER(0, "Mixer Sub OSC") PORT_MINMAX(0, 255)
	PORT_START("CTRL58")
	PORT_ADJUSTER(128, "OSC1 Color") PORT_MINMAX(0, 255)
	PORT_START("CTRL59")
	PORT_ADJUSTER(128, "LFO Rate") PORT_MINMAX(0, 255)
	PORT_START("CTRL60")
	PORT_ADJUSTER(128, "Filter Key Follow") PORT_MINMAX(0, 255)
	PORT_START("CTRL61")
	PORT_ADJUSTER(128, "Control 5.7") PORT_MINMAX(0, 255)
	PORT_START("CTRL62")
	PORT_ADJUSTER(128, "Control 6.7") PORT_MINMAX(0, 255)
	PORT_START("CTRL63")
	PORT_ADJUSTER(128, "Control 7.7") PORT_MINMAX(0, 255)
INPUT_PORTS_END

void system1_state::system1(machine_config &config)
{
	MB8AA4181(config, m_maincpu, 156'000'000);
	m_maincpu->set_addrmap(AS_PROGRAM, &system1_state::mem_map);
	m_maincpu->set_flash_tag("flash");
	m_maincpu->sfi_cs_cb().set(m_spiflash, FUNC(generic_spi_flash_device::cs_w));
	m_maincpu->sfi_tx_cb().set(m_spiflash, FUNC(generic_spi_flash_device::write));
	m_maincpu->sfi_rx_cb().set(m_spiflash, FUNC(generic_spi_flash_device::read));
	m_maincpu->gpio_out_cb<0>().set([this] (u32 data) { m_port0 = data; });
	m_maincpu->adc_in_cb<0>().set(FUNC(system1_state::control_r<0>));
	m_maincpu->adc_in_cb<1>().set(FUNC(system1_state::control_r<1>));
	m_maincpu->adc_in_cb<2>().set(FUNC(system1_state::control_r<2>));
	m_maincpu->adc_in_cb<3>().set(FUNC(system1_state::control_r<3>));
	m_maincpu->adc_in_cb<4>().set(FUNC(system1_state::control_r<4>));
	m_maincpu->adc_in_cb<5>().set(FUNC(system1_state::control_r<5>));
	m_maincpu->adc_in_cb<6>().set(FUNC(system1_state::control_r<6>));
	m_maincpu->adc_in_cb<7>().set(FUNC(system1_state::control_r<7>));
	m_maincpu->set_i2c_bus<7>(m_subcpu);
	m_maincpu->set_usb_link(m_subcpu, true);

	MB8AA4181(config, m_subcpu, 156'000'000);
	m_subcpu->set_addrmap(AS_PROGRAM, &system1_state::sub_map);
	m_subcpu->set_flash_tag("flash");
	m_subcpu->set_boot_mode(3);
	m_subcpu->set_i2c_bus<7>(m_maincpu);
	m_subcpu->set_usb_link(m_maincpu, false);

	SPEAKER(config, "speaker", 2).front();
	mb8aa4181_dsp_device &dsp = *m_maincpu->subdevice<mb8aa4181_dsp_device>("dsp");
	dsp.set_clock(22.5792_MHz_XTAL);
	dsp.add_route(0, "speaker", 1.0, 0);
	dsp.add_route(1, "speaker", 1.0, 1);
	mb8aa4181_dsp_device &subdsp = *m_subcpu->subdevice<mb8aa4181_dsp_device>("dsp");
	subdsp.set_clock(22.5792_MHz_XTAL);
	subdsp.set_port_link(dsp);
	dsp.set_port_link(subdsp);

	GENERIC_SPI_FLASH(config, m_spiflash);

	MIDI_PORT(config, "mdin", midiin_slot, "midiin").rxd_handler().set(m_maincpu, FUNC(mb8aa4181_device::rxd_w<6>));
	m_maincpu->txd_cb<6>().set("mdout", FUNC(midi_port_device::write_txd));
	MIDI_PORT(config, "mdout", midiout_slot, "midiout");
}

ROM_START(system1)
	ROM_REGION32_LE(0x400000, "flash", 0)
	// the 1.30 update image
	ROM_LOAD("system1_130.bin", 0, 0x400000, BAD_DUMP CRC(d72123de) SHA1(a76bd01482f4032bfae5ee562ed067db6a509b65))
ROM_END

} // anonymous namespace


SYST(2014, system1, 0, 0, system1, system1, system1_state, empty_init, "Roland", "SYSTEM-1 Variable Synthesizer", MACHINE_NOT_WORKING | MACHINE_IMPERFECT_SOUND)
