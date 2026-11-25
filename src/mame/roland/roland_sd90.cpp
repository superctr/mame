// license:BSD-3-Clause
// copyright-holders:superctr
/****************************************************************************

    Edirol SD-80 and SD-90 Studio Canvas

    SD-80 main board:
    - IC2 HD6417706 SH-3, EXTAL 12 MHz from the USB block, 96 MHz internal
    - IC3 LH28F160BJE-BTL80 program flash, 2 MB
    - IC4/IC6 LC3816161ET-70 SDRAM, 4 MB
    - IC19/IC27 TC223C660CF-503 XV tone generators, 16.9344 MHz
    - IC26/IC30 GM71C18163CJ-6 effect DRAM, one beside each XV
    - IC28/IC29 uPD23C128040ALGY wave mask ROMs, 16 MB each
    - IC17 M37641M8-137FP USB controller, 24 MHz
    - IC32/IC34 PCM1716E DACs, IC33 TC9271FS digital output
    - LCD board IC7 RCM2072M-B, 20 x 2 characters

    SD-90: the SD-80's platform with an LH28F320BF flash, the MR3 effect
    processor as the output mixer, audio inputs and a 128 x 64 graphic LCD.

    TODO:
    - the USB controller: no USB MIDI or audio
    - the SD-90's analog and digital inputs, the digital outputs

****************************************************************************/

#include "emu.h"

#include "roland_uipc.h"
#include "wavecard.h"

#include "bus/midi/midiinport.h"
#include "bus/midi/midioutport.h"
#include "cpu/sh/sh3comn.h"
#include "cpu/sh/sh4.h"
#include "machine/intelfsh.h"
#include "sound/roland_mr.h"
#include "sound/roland_xv.h"
#include "video/hd44780.h"
#include "video/st7565.h"

#include "emupal.h"
#include "screen.h"
#include "speaker.h"

#include <algorithm>
#include <iterator>

#define LOG_UIPC    (1U << 1)
#define LOG_RESET   (1U << 2)

#define VERBOSE (0)
#include "logmacro.h"


namespace {

class sd_state : public driver_device
{
public:
	void init_waverom() ATTR_COLD;

protected:
	enum
	{
		LED_GS, LED_MIDI, LED_SYSTEM, LED_NATIVE, LED_USB, LED_EFFECTS, LED_GM2, LED_PREVIEW, LED_INST_DRUM,
		LED_BYPASS, LED_EDIT, LED_SELECT, LED_TRACK_DOWN, LED_DIGITAL_IN, LED_LINE_IN, LED_MIC_GUITAR, LEDS
	};

	sd_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_xv(*this, "xv%u", 0U)
		, m_flash(*this, "flash")
		, m_out(*this, "out%u", 1U)
		, m_tg_view(*this, "tg_view")
		, m_leds(*this, "led_%u", 0U)
	{
	}

	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

	void common(machine_config &config) ATTR_COLD;
	void xv_wave_map(address_map &map) ATTR_COLD;
	void lcd_palette(palette_device &palette) const ATTR_COLD;

	static bool port_pin(u64 data, int bit) { return BIT(data, 16 + 2 * bit, 2) != 1 || BIT(data, bit); }
	bool tg_reset_w(int state);

	required_device<sh7709_device> m_maincpu;
	required_device_array<roland_xv_device, 2> m_xv;
	required_device<intelfsh16_device> m_flash;
	required_device_array<speaker_device, 2> m_out;
	memory_view m_tg_view;
	output_finder<LEDS> m_leds;

	bool m_tg_reset = true;
};

class sd80_state : public sd_state
{
public:
	sd80_state(const machine_config &mconfig, device_type type, const char *tag)
		: sd_state(mconfig, type, tag)
		, m_uipc(*this, "uipc")
		, m_lcd(*this, "lcd")
		, m_mdout2(*this, "mdout2")
		, m_dial(*this, "DIAL")
		, m_midi_sw(*this, "MIDI_SW")
	{
	}

	void sd80(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	void sd80_map(address_map &map) ATTR_COLD;
	void sd80_io_map(address_map &map) ATTR_COLD;

	void port_c_w(u64 data);
	void port_d_w(u64 data);
	void update_dac();
	void led_w(offs_t offset, u8 data);
	void out2_update();
	u64 scp_r();
	bool enca() const { return m_encoder_phase != 1 && m_encoder_phase != 2; }
	TIMER_CALLBACK_MEMBER(step_encoder);

	required_device<roland_uipc_device> m_uipc;
	required_device<hd44780_device> m_lcd;
	required_device<midi_port_device> m_mdout2;
	required_ioport m_dial;
	required_ioport m_midi_sw;

	emu_timer *m_encoder_timer = nullptr;
	u8 m_dial_last = 0;
	s8 m_dial_pending = 0;
	u8 m_encoder_phase = 0;
	u8 m_in1_rxd = 1;
	u8 m_scif_txd = 1;
	bool m_dac_reset = true;
	bool m_amute = true;
};

class sd90_state : public sd_state
{
public:
	sd90_state(const machine_config &mconfig, device_type type, const char *tag)
		: sd_state(mconfig, type, tag)
		, m_mr3(*this, "mr3")
		, m_glcd(*this, "glcd")
		, m_panel(*this, "PANEL%u", 0U)
		, m_knob(*this, "V%u", 1U)
		, m_hacks(*this, "HACKS")
	{
	}

	void sd90(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	void sd90_map(address_map &map) ATTR_COLD;
	void sd90_io_map(address_map &map) ATTR_COLD;

	void port_e_w(u64 data);
	u8 usb_r(offs_t offset);
	void usb_w(offs_t offset, u8 data);
	void led_w(offs_t offset, u8 data);
	u64 knob_a_r();
	u64 knob_b_r();
	void knob_clear_w(u64 data);
	void update_knobs();
	TIMER_CALLBACK_MEMBER(step_knobs);

	required_device<roland_mr_device> m_mr3;
	required_device<st7565_device> m_glcd;
	required_ioport_array<2> m_panel;
	required_ioport_array<3> m_knob;
	required_ioport m_hacks;

	emu_timer *m_knob_timer = nullptr;
	u8 m_usb_state = 0;
	u8 m_usb_reg[0x10]{};
	u8 m_knob_last[3]{};
	s8 m_knob_pending[3]{};
	bool m_knob_latched[3]{};
	bool m_knob_up[3]{};
};


void sd_state::init_waverom()
{
	roland_srx_slot_device::descramble_rom(memregion("waverom")->base(), memregion("waverom")->bytes());
}

void sd_state::machine_start()
{
	save_item(NAME(m_tg_reset));
}

void sd_state::machine_reset()
{
	m_tg_reset = true;
	m_tg_view.select(1);
}

bool sd_state::tg_reset_w(int state)
{
	if (bool(state) == m_tg_reset)
		return false;

	LOGMASKED(LOG_RESET, "%s: tone generator reset %s\n", machine().describe_context(), state ? "asserted" : "released");
	m_tg_reset = state;
	m_tg_view.select(state ? 1 : 0);
	for (auto &xv : m_xv)
		xv->reset();
	return true;
}

void sd_state::lcd_palette(palette_device &palette) const
{
	palette.set_pen_color(0, rgb_t(0x20, 0x50, 0xd0));
	palette.set_pen_color(1, rgb_t(0xe8, 0xf0, 0xff));
}

void sd_state::xv_wave_map(address_map &map)
{
	map.unmap_value_high();
	map(0x00000000, 0x00ffffff).mirror(0x01000000).rom().region("waverom", 0);
}

void sd_state::common(machine_config &config)
{
	SH7709(config, m_maincpu, 24_MHz_XTAL / 2 * 8, ENDIANNESS_BIG);

	SPEAKER(config, m_out[0], 2).front();
	SPEAKER(config, m_out[1], 2).front();

	ROLAND_XV(config, m_xv[0], 16.9344_MHz_XTAL);
	m_xv[0]->set_addrmap(roland_xv_device::AS_WAVE, &sd_state::xv_wave_map);
	m_xv[0]->int_callback().set_inputline(m_maincpu, 0);
	m_xv[0]->set_link(m_xv[1]);

	ROLAND_XV(config, m_xv[1], 16.9344_MHz_XTAL);
	m_xv[1]->set_addrmap(roland_xv_device::AS_WAVE, &sd_state::xv_wave_map);
	m_xv[1]->int_callback().set_inputline(m_maincpu, 1);

	PALETTE(config, "palette", FUNC(sd_state::lcd_palette), 2);
}


void sd80_state::machine_start()
{
	sd_state::machine_start();

	m_encoder_timer = timer_alloc(FUNC(sd80_state::step_encoder), this);
	m_encoder_timer->adjust(attotime::from_hz(1000), 0, attotime::from_hz(1000));

	save_item(NAME(m_dial_last));
	save_item(NAME(m_dial_pending));
	save_item(NAME(m_encoder_phase));
	save_item(NAME(m_in1_rxd));
	save_item(NAME(m_scif_txd));
	save_item(NAME(m_dac_reset));
	save_item(NAME(m_amute));
}

void sd80_state::machine_reset()
{
	sd_state::machine_reset();

	m_dac_reset = true;
	m_amute = true;
	update_dac();
}

void sd80_state::port_c_w(u64 data)
{
	tg_reset_w(port_pin(data, 4));
	const bool dac_reset = port_pin(data, 5);
	if (dac_reset != m_dac_reset)
	{
		LOGMASKED(LOG_RESET, "%s: DAC reset %s\n", machine().describe_context(), dac_reset ? "asserted" : "released");
		m_dac_reset = dac_reset;
		update_dac();
	}
}

void sd80_state::port_d_w(u64 data)
{
	const bool amute = port_pin(data, 6);
	if (amute != m_amute)
	{
		LOGMASKED(LOG_RESET, "%s: analog mute %s\n", machine().describe_context(), amute ? "on" : "off");
		m_amute = amute;
		update_dac();
	}
}

void sd80_state::update_dac()
{
	const float gain = (m_dac_reset || m_amute) ? 0.0f : 1.0f;
	for (auto &out : m_out)
		for (int ch = 0; ch < 2; ch++)
			out->set_input_gain(ch, gain);
}

void sd80_state::led_w(offs_t offset, u8 data)
{
	constexpr u8 LED_MAP[3][3] = {
		{ LED_GS,     LED_MIDI,    LED_SYSTEM },
		{ LED_NATIVE, LED_USB,     LED_EFFECTS },
		{ LED_GM2,    LED_PREVIEW, LED_INST_DRUM } };
	const u32 strobe = offset >> 3, line = offset & 7;
	if (strobe < 3 && line < 3)
		m_leds[LED_MAP[strobe][line]] = data;
}

u64 sd80_state::scp_r()
{
	const bool encb = m_encoder_phase < 2;
	return 0xcf | (enca() ? 0x20 : 0x00) | (encb ? 0x10 : 0x00);
}

TIMER_CALLBACK_MEMBER(sd80_state::step_encoder)
{
	const u8 now = m_dial->read();
	m_dial_pending += s8(now - m_dial_last);
	m_dial_last = now;

	if (!m_dial_pending)
		return;

	const int step = (m_dial_pending > 0) ? 1 : -1;
	const bool was = enca();
	m_encoder_phase = (m_encoder_phase + step) & 3;
	if (!m_encoder_phase)
		m_dial_pending -= step;

	if (was != enca())
		m_maincpu->set_input_line(5, enca() ? CLEAR_LINE : ASSERT_LINE);
}

void sd80_state::out2_update()
{
	m_mdout2->write_txd(BIT(m_midi_sw->read(), 0) ? m_in1_rxd : m_scif_txd);
}

void sd80_state::sd80_map(address_map &map)
{
	map(0x00000000, 0x001fffff).rw(m_flash, FUNC(intelfsh16_device::read), FUNC(intelfsh16_device::write));
	map(0x08000000, 0x083fffff).ram();
	map(0x14000000, 0x17ffffff).view(m_tg_view);
	m_tg_view[0](0x14000000, 0x140001ff).rw(m_xv[0], FUNC(roland_xv_device::read), FUNC(roland_xv_device::write));
	m_tg_view[0](0x15000000, 0x150001ff).rw(m_xv[1], FUNC(roland_xv_device::read), FUNC(roland_xv_device::write));
	m_tg_view[1](0x14000000, 0x17ffffff).noprw();
	map(0x18000000, 0x18000003).lrw8(
			NAME([this] (offs_t offset) { return m_uipc->tx_r(offset >> 1); }),
			NAME([this] (offs_t offset, u8 data) { m_uipc->tx_w(offset >> 1, data); }));
	map(0x19000000, 0x19000003).lrw8(
			NAME([this] (offs_t offset) { return m_uipc->rx_r(offset >> 1); }),
			NAME([this] (offs_t offset, u8 data) { m_uipc->rx_w(offset >> 1, data); }));
}

void sd80_state::sd80_io_map(address_map &map)
{
	map(SH3_PORT_C, SH3_PORT_C + 7).w(FUNC(sd80_state::port_c_w));
	map(SH3_PORT_D, SH3_PORT_D + 7).w(FUNC(sd80_state::port_d_w));
	map(SH3_PORT_SC, SH3_PORT_SC + 7).r(FUNC(sd80_state::scp_r));
}

void sd80_state::sd80(machine_config &config)
{
	common(config);
	m_maincpu->set_addrmap(AS_PROGRAM, &sd80_state::sd80_map);
	m_maincpu->set_addrmap(AS_IO, &sd80_state::sd80_io_map);

	SHARP_LH28F160BJE(config, m_flash);

	ROLAND_UIPC(config, m_uipc);
	m_uipc->set_announcement({ { 0x01, 0xaa }, { 0x01, 0x00 } });

	m_xv[0]->add_route(0, m_out[0], 1.0, 0);
	m_xv[0]->add_route(1, m_out[0], 1.0, 1);
	m_xv[0]->add_route(2, m_out[1], 1.0, 0);
	m_xv[0]->add_route(3, m_out[1], 1.0, 1);

	screen_device &screen(SCREEN(config, "screen"));
	screen.set_lcd();
	screen.set_refresh_hz(60);
	screen.set_screen_update(m_lcd, FUNC(hd44780_device::screen_update));
	screen.set_size(6 * 20, 9 * 2);
	screen.set_visarea_full();
	screen.set_palette("palette");

	HD44780(config, m_lcd, 270'000);
	m_lcd->set_lcd_size(2, 20);
	m_xv[0]->lcd_callback().set(m_lcd, FUNC(hd44780_device::write));

	m_xv[0]->switch_callback().set_ioport("PANEL");
	m_xv[0]->led_callback().set(FUNC(sd80_state::led_w));

	midi_port_device &mdin1(MIDI_PORT(config, "mdin1", midiin_slot, "midiin"));
	mdin1.rxd_handler().set([this] (int state) { m_maincpu->sci_rxd_w(state); m_in1_rxd = state; out2_update(); });
	midi_port_device &mdout1(MIDI_PORT(config, "mdout1", midiout_slot, "midiout"));
	m_maincpu->sci_txd_handler().set(mdout1, FUNC(midi_port_device::write_txd));

	midi_port_device &mdin2(MIDI_PORT(config, "mdin2", midiin_slot, "midiin"));
	mdin2.rxd_handler().set(m_maincpu, FUNC(sh7709_device::scif_rxd_w));
	MIDI_PORT(config, m_mdout2, midiout_slot, "midiout");
	m_maincpu->scif_txd_handler().set([this] (int state) { m_scif_txd = state; out2_update(); });
}


void sd90_state::machine_start()
{
	sd_state::machine_start();

	m_knob_timer = timer_alloc(FUNC(sd90_state::step_knobs), this);
	m_knob_timer->adjust(attotime::from_hz(1000), 0, attotime::from_hz(1000));

	save_item(NAME(m_usb_state));
	save_item(NAME(m_usb_reg));
	save_item(NAME(m_knob_last));
	save_item(NAME(m_knob_pending));
	save_item(NAME(m_knob_latched));
	save_item(NAME(m_knob_up));
}

void sd90_state::machine_reset()
{
	sd_state::machine_reset();

	m_usb_state = 0;

	// HACK: the USB controller is not emulated, so a blank flash is given the
	// settings record that starts the machine in MIDI mode
	if (BIT(m_hacks->read(), 0))
	{
		constexpr u8 MIDI_STARTUP[0x28] = {
				'R', 'o', 'l', 'a', 'n', 'd', ' ', 'D', 'M', 'Z', '0', '7', 0x44, 0xbb, 0x50, 0xaf,
				0x42, 0x04, 0x28, 0x5e, 0x7f, 0x3c, 0x00, 0x00,
				'R', 'o', 'l', 'a', 'n', 'd', ' ', 'D', 'M', 'Z', '0', '7', 0x44, 0xbb, 0x50, 0xaf };
		u8 *const params = m_flash->base() + 0x8000;
		if (std::all_of(params, params + 0x8000, [] (u8 b) { return b == 0xff; }))
			std::copy(std::begin(MIDI_STARTUP), std::end(MIDI_STARTUP), params + 0x2000);
	}
}

void sd90_state::port_e_w(u64 data)
{
	if (tg_reset_w(port_pin(data, 2)))
		m_mr3->reset();
}

void sd90_state::led_w(offs_t offset, u8 data)
{
	constexpr s8 LED_MAP[32] = {
		LED_PREVIEW, -1, LED_EFFECTS, LED_INST_DRUM, -1, -1, -1, -1,
		-1, -1, -1, -1, -1, -1, -1, -1,
		LED_SYSTEM, LED_BYPASS, LED_EDIT, LED_SELECT, -1, -1, -1, -1,
		LED_TRACK_DOWN, LED_DIGITAL_IN, LED_LINE_IN, LED_MIC_GUITAR, -1, -1, -1, -1 };
	if (offset < 32 && LED_MAP[offset] >= 0)
		m_leds[LED_MAP[offset]] = data;
}

u64 sd90_state::knob_a_r()
{
	u8 data = 0xff;
	for (int i = 0; i < 3; i++)
		if (m_knob_latched[i])
			data &= ~(2 << i);
	return data;
}

u64 sd90_state::knob_b_r()
{
	u8 data = 0xff;
	for (int i = 0; i < 3; i++)
		if (!m_knob_up[i])
			data &= ~(2 << i);
	return data;
}

void sd90_state::knob_clear_w(u64 data)
{
	for (int i = 0; i < 3; i++)
		if (BIT(data, 16 + 2 * (5 + i), 2) == 1 && !BIT(data, 5 + i))
			m_knob_latched[i] = false;
	update_knobs();
}

void sd90_state::update_knobs()
{
	const u8 a = knob_a_r();
	m_maincpu->pint_w<1>(BIT(a, 1));
	m_maincpu->pint_w<2>(BIT(a, 2));
	m_maincpu->pint_w<3>(BIT(a, 3));
}

TIMER_CALLBACK_MEMBER(sd90_state::step_knobs)
{
	for (int i = 0; i < 3; i++)
	{
		const u8 now = m_knob[i]->read();
		m_knob_pending[i] += s8(now - m_knob_last[i]);
		m_knob_last[i] = now;
		if (!m_knob_pending[i] || m_knob_latched[i])
			continue;

		m_knob_up[i] = m_knob_pending[i] > 0;
		m_knob_pending[i] += m_knob_up[i] ? -1 : 1;
		m_knob_latched[i] = true;
	}
	update_knobs();
}

u8 sd90_state::usb_r(offs_t offset)
{
	switch (offset)
	{
	case 0x08: return 1;
	case 0x0a: return 0;
	case 0x0d: return m_usb_state;
	case 0x10: return (m_usb_state == 3) ? 1 : 0;
	default:   return (offset < 0x10) ? m_usb_reg[offset] : 0;
	}
}

void sd90_state::usb_w(offs_t offset, u8 data)
{
	if (offset < 0x10)
	{
		if (offset != 0x0a && offset != 0x0d)
			LOGMASKED(LOG_UIPC, "%s: usb write %x = %02x\n", machine().describe_context(), offset, data);
		m_usb_reg[offset] = data;
	}
	if (offset == 0x08 && data == 0 && m_usb_state < 5)
		m_usb_state++;
}

void sd90_state::sd90_map(address_map &map)
{
	map(0x00000000, 0x003fffff).rw(m_flash, FUNC(intelfsh16_device::read), FUNC(intelfsh16_device::write));
	map(0x08000000, 0x083fffff).ram();
	map(0x14000000, 0x1800003f).view(m_tg_view);
	m_tg_view[0](0x14000000, 0x140001ff).rw(m_xv[0], FUNC(roland_xv_device::read), FUNC(roland_xv_device::write));
	m_tg_view[0](0x15000000, 0x150001ff).rw(m_xv[1], FUNC(roland_xv_device::read), FUNC(roland_xv_device::write));
	m_tg_view[0](0x18000000, 0x1800003f).m(m_mr3, FUNC(roland_mr_device::map));
	m_tg_view[1](0x14000000, 0x1800003f).noprw();
	map(0x18800000, 0x1880003f).rw(FUNC(sd90_state::usb_r), FUNC(sd90_state::usb_w));
}

void sd90_state::sd90_io_map(address_map &map)
{
	map(SH3_PORT_C, SH3_PORT_C + 7).rw(FUNC(sd90_state::knob_a_r), FUNC(sd90_state::knob_clear_w));
	map(SH3_PORT_E, SH3_PORT_E + 7).w(FUNC(sd90_state::port_e_w));
	map(SH3_PORT_L, SH3_PORT_L + 7).r(FUNC(sd90_state::knob_b_r));
}

void sd90_state::sd90(machine_config &config)
{
	common(config);
	m_maincpu->set_clock(100'000'000);
	m_maincpu->set_addrmap(AS_PROGRAM, &sd90_state::sd90_map);
	m_maincpu->set_addrmap(AS_IO, &sd90_state::sd90_io_map);

	SHARP_LH28F320BF(config, m_flash);

	screen_device &screen(SCREEN(config, "screen"));
	screen.set_lcd();
	screen.set_refresh_hz(60);
	screen.set_screen_update(m_glcd, FUNC(st7565_device::screen_update));
	screen.set_size(128, 64);
	screen.set_visarea_full();
	screen.set_palette("palette");

	ST7565(config, m_glcd, 0);
	m_glcd->set_panel(128, 64, 131, -1);
	m_xv[0]->lcd_callback().set(m_glcd, FUNC(st7565_device::write));
	m_xv[0]->switch_callback().set([this] () { return u64(m_panel[0]->read()) | (u64(m_panel[1]->read()) << 32); });
	m_xv[0]->led_callback().set(FUNC(sd90_state::led_w));

	ROLAND_MR(config, m_mr3, 16.9344_MHz_XTAL);
	m_xv[0]->add_route(0, m_mr3, 0.5, 0);
	m_xv[0]->add_route(1, m_mr3, 0.5, 1);
	m_mr3->add_route(0, m_out[0], 1.0, 0);
	m_mr3->add_route(1, m_out[0], 1.0, 1);
	m_mr3->add_route(2, m_out[1], 1.0, 0);
	m_mr3->add_route(3, m_out[1], 1.0, 1);

	midi_port_device &mdin1(MIDI_PORT(config, "mdin1", midiin_slot, "midiin"));
	mdin1.rxd_handler().set(m_maincpu, FUNC(sh7709_device::irda_rxd_w));
	midi_port_device &mdout1(MIDI_PORT(config, "mdout1", midiout_slot, "midiout"));
	m_maincpu->irda_txd_handler().set(mdout1, FUNC(midi_port_device::write_txd));

	midi_port_device &mdin2(MIDI_PORT(config, "mdin2", midiin_slot, "midiin"));
	mdin2.rxd_handler().set(m_maincpu, FUNC(sh7709_device::scif_rxd_w));
	midi_port_device &mdout2(MIDI_PORT(config, "mdout2", midiout_slot, "midiout"));
	m_maincpu->scif_txd_handler().set(mdout2, FUNC(midi_port_device::write_txd));
}


INPUT_PORTS_START(sd80)
	PORT_START("PANEL")
	PORT_BIT(0x00000002, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Cursor Left") PORT_CODE(KEYCODE_LEFT)
	PORT_BIT(0x00000008, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Inst/Drum") PORT_CODE(KEYCODE_I)
	PORT_BIT(0x00000100, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Enter") PORT_CODE(KEYCODE_ENTER)
	PORT_BIT(0x00000200, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Cursor Right") PORT_CODE(KEYCODE_RIGHT)
	PORT_BIT(0x00000400, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Part Down") PORT_CODE(KEYCODE_OPENBRACE)
	PORT_BIT(0x00000800, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Effects") PORT_CODE(KEYCODE_E)
	PORT_BIT(0x00010000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Preview") PORT_CODE(KEYCODE_SPACE)
	PORT_BIT(0x00020000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Shift") PORT_CODE(KEYCODE_LSHIFT)
	PORT_BIT(0x00040000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Part Up") PORT_CODE(KEYCODE_CLOSEBRACE)
	PORT_BIT(0x00080000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("System") PORT_CODE(KEYCODE_S)

	PORT_START("DIAL")
	PORT_BIT(0xff, 0x00, IPT_DIAL) PORT_NAME("Value") PORT_SENSITIVITY(25) PORT_KEYDELTA(2)

	PORT_START("MIDI_SW")
	PORT_CONFNAME(0x01, 0x00, "MIDI OUT 2")
	PORT_CONFSETTING(   0x00, "OUT 2")
	PORT_CONFSETTING(   0x01, "IN 1 THRU")
INPUT_PORTS_END

INPUT_PORTS_START(sd90)
	PORT_START("PANEL0")
	PORT_BIT(0x00000001, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Enter") PORT_CODE(KEYCODE_ENTER)
	PORT_BIT(0x00000002, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Exit") PORT_CODE(KEYCODE_BACKSPACE)
	PORT_BIT(0x00000004, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Part >") PORT_CODE(KEYCODE_CLOSEBRACE)
	PORT_BIT(0x00000008, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Part <") PORT_CODE(KEYCODE_OPENBRACE)
	PORT_BIT(0x00000100, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Preview") PORT_CODE(KEYCODE_SPACE)
	PORT_BIT(0x00000200, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Shift") PORT_CODE(KEYCODE_LSHIFT)
	PORT_BIT(0x00000400, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Effects") PORT_CODE(KEYCODE_E)
	PORT_BIT(0x00000800, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Inst/Drum Set") PORT_CODE(KEYCODE_I)
	PORT_BIT(0x00040000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Page Down") PORT_CODE(KEYCODE_PGDN)
	PORT_BIT(0x00080000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Page Up") PORT_CODE(KEYCODE_PGUP)
	PORT_BIT(0x01000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("System") PORT_CODE(KEYCODE_S)
	PORT_BIT(0x02000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("AFX Bypass") PORT_CODE(KEYCODE_B)
	PORT_BIT(0x04000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("AFX Edit") PORT_CODE(KEYCODE_D)
	PORT_BIT(0x08000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("AFX Select") PORT_CODE(KEYCODE_A)

	PORT_START("PANEL1")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Track Down") PORT_CODE(KEYCODE_4)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Digital In") PORT_CODE(KEYCODE_3)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Line In") PORT_CODE(KEYCODE_2)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Mic/Guitar") PORT_CODE(KEYCODE_1)

	PORT_START("V1")
	PORT_BIT(0xff, 0x00, IPT_DIAL) PORT_NAME("V1") PORT_SENSITIVITY(25) PORT_KEYDELTA(2) PORT_CODE_DEC(KEYCODE_Z) PORT_CODE_INC(KEYCODE_X)

	PORT_START("V2")
	PORT_BIT(0xff, 0x00, IPT_DIAL) PORT_PLAYER(2) PORT_NAME("V2") PORT_SENSITIVITY(25) PORT_KEYDELTA(2) PORT_CODE_DEC(KEYCODE_C) PORT_CODE_INC(KEYCODE_V)

	PORT_START("V3")
	PORT_BIT(0xff, 0x00, IPT_DIAL) PORT_PLAYER(3) PORT_NAME("V3") PORT_SENSITIVITY(25) PORT_KEYDELTA(2) PORT_CODE_DEC(KEYCODE_N) PORT_CODE_INC(KEYCODE_M)

	PORT_START("HACKS")
	PORT_CONFNAME(0x01, 0x01, "Start a blank flash in MIDI mode (hack)")
	PORT_CONFSETTING(   0x00, DEF_STR(No))
	PORT_CONFSETTING(   0x01, DEF_STR(Yes))
INPUT_PORTS_END


#define ROM_WAVEROM \
	ROM_REGION16_LE(0x2000000, "waverom", ROMREGION_ERASE00) \
	ROM_LOAD("upd23c128040algy525mjh.ic28", 0x0000000, 0x1000000, CRC(bb778083) SHA1(57cd6709274329a1e109f22564d2b706832ecdb0)) \
	ROM_LOAD("upd23c128040algy526mkh.ic29", 0x1000000, 0x1000000, CRC(dbbdc726) SHA1(68e3d39248bfde38b8e2cc509f7972feac066760))

ROM_START(sd80)
	ROM_REGION16_BE(0x200000, "flash", 0)
	ROM_LOAD16_WORD_SWAP("sd80_v1.01.ic3", 0x000000, 0x200000, CRC(e0cfce2e) SHA1(8fcf6df19cf0c32bc07e464fdc77fd31a02e6c32))

	ROM_WAVEROM
ROM_END

ROM_START(sd90)
	ROM_REGION16_BE(0x400000, "flash", ROMREGION_ERASEFF)
	ROM_LOAD("sd90_v1.03.bin", 0x000000, 0x300000, CRC(e44fb343) SHA1(447cc7edabedfe3fb4cb7c280f44eb6eca50d7fe))

	ROM_WAVEROM
ROM_END

} // anonymous namespace


//    YEAR  NAME  PARENT  COMPAT  MACHINE  INPUT  CLASS       INIT          COMPANY   FULLNAME               FLAGS
SYST( 2001, sd90, 0,      0,      sd90,    sd90,  sd90_state, init_waverom, "Edirol", "Studio Canvas SD-90", MACHINE_NOT_WORKING )
SYST( 2002, sd80, 0,      0,      sd80,    sd80,  sd80_state, init_waverom, "Edirol", "Studio Canvas SD-80", 0 )
