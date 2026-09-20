// license:BSD-3-Clause
// copyright-holders:superctr
/****************************************************************************

    Roland JD-990 Super JD synthesizer module.

    Main board (service notes):
    - IC9 H8/570 (R15199852), 20 MHz crystal, mode 3: expanded maximum,
      16-bit bus, on-chip ROM disabled
    - IC6 R10-0001 gate array: the chip selects, the panel scan
    - IC31 program ROM, 256K x 16; IC30/IC32 LC36256 battery-backed RAM
    - IC22 MB87731A "EP" PCM chip, IC25 MB87424A "TVF", IC27/IC28 TC6088AF
      "CSP" effect processors, IC26 HG62E11R46FB "IFCS" output stage,
      PCM61P DAC and eight sample-and-holds
    - IC8 SED1335F0B LCD controller with IC7 SRM2064, a graphic LCD
    - IC10 HG62E11R24FS RAM card interface

    The CPU's ISP sub-processor scans the panel, reads the encoder and
    keeps the firmware's tick and active-sensing timers; its program is in
    the part, so the driver models what it does.  So far: the tick, at a
    guessed 1 ms; the display transfer, in which the CPU sets the
    controller's cursor, puts a work-RAM source address in DR7 and a byte
    count in DR10 and clears DR6H bit 0, and the ISP streams the bytes
    into the controller; the mailbox byte the ISP answers with ISF0; the
    panel scan, one column a tick into work RAM with the column number
    in DR5H and ISF3 on a change; and the encoder, its steps summed into
    DR31H with ISF10; and the active-sensing timer, which counts in DR4H
    while DR6H bit 4 is clear (the CPU clears it on every active-sensing
    byte) and raises ISF8 when the count runs out.  The tick is 1 ms: the
    tone delay parameter, whose table the owner's manual gives in
    seconds, comes out at those seconds with it.  The timeout is a guess
    at 300 ms.

    FXM is the ISP's too: for a tone with FXM the CPU leaves, per voice,
    a colour code (1, 2, 3, 4, 6, 8, 10, 14 for colours 1-8) and two
    pitch ratios, the tone's pitch raised and lowered by the depth in
    0.06 semitone steps, in work RAM at 08:FB78, 08:FFD0 and 08:FFA0,
    writes the EP's pitch once and never again, and the ISP alternates
    the EP's pitch register between the two ratios every colour-code
    loops of its service loop, deferring while the CPU holds the bus.
    The loop period is a guess at 100 us; the manual's "small values
    metallic, large values gritty" fits a modulator from 5 kHz down to
    360 Hz. The TVF channels feed two CSPs in series, followed by the
    IFCS output demultiplexer. Bus alignment and output levels are provisional.

    The panel is 32 switches on five columns of the ISP's scan plus the
    VALUE knob's push switch; the names come from pressing each position
    in emulation against the owner's manual.  PERFORM has not answered
    on any position and three positions are unplaced.

    The wave ROMs are stored with the EP's address and data lines
    permuted, the SC-55's map per 1 MB region (ep/docs/wave_rom_lines.md);
    the driver undoes it at start, and the three chips form one 6 MB
    module at address 0 of the EP's wave space; the card and expansion
    slots sit at 0x600000 and above and are empty here.

    The 1 MB map as the firmware uses it: the program ROM in pages 0-7,
    except that the upper half of page 0 is RAM (the image is blank there
    and the firmware keeps the same variables at 0:9F48 and 8:9F48); the
    64 KB battery RAM in page 8; the EP and TVF windows in the lower half
    of page 0E, with the RAM's upper half again above them (the voice
    service clears flags in its voice records through that page); the
    two CSPs, the LCD controller (0F:A000 data, 0F:A002 command) and the
    CPU's own top-page mirror in page 0F.

****************************************************************************/

#include "emu.h"

#include "bus/midi/midi.h"
#include "cpu/h8500/h8570.h"
#include "machine/nvram.h"
#include "sound/roland_ep.h"
#include "sound/roland_csp.h"
#include "sound/roland_tvf.h"
#include "video/sed1330.h"

#include "emupal.h"
#include "screen.h"
#include "speaker.h"

#define LOG_EP  (1U << 1)
#define LOG_CSP (1U << 2)
#define LOG_ISP (1U << 3)

#define VERBOSE (LOG_EP | LOG_CSP)
#include "logmacro.h"


class jd990_sound_device;
DECLARE_DEVICE_TYPE(JD990_SOUND, jd990_sound_device)

class jd990_sound_device : public device_t, public device_sound_interface
{
public:
	jd990_sound_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
		: device_t(mconfig, JD990_SOUND, tag, owner, clock)
		, device_sound_interface(mconfig, *this)
		, m_csp(*this, "^csp%u", 1U)
	{
	}

	void update() { m_stream->update(); }

protected:
	virtual void device_start() override
	{
		m_stream = stream_alloc(24, 8, 44100);
	}

	virtual void sound_stream_update(sound_stream &stream) override
	{
		static constexpr unsigned slots[] = { 4, 7, 18, 21, 24, 27, 30, 1 };
		for (int sample = 0; sample < stream.samples(); sample++)
		{
			for (unsigned channel = 0; channel < 24; channel++)
				m_csp[0]->sc_w((channel + 4) % 24, s32(std::clamp(stream.get(channel, sample) * 3145728.0f, -8388608.0f, 8388607.0f)));
			m_csp[0]->run_once(768);
			for (unsigned channel = 0; channel < 32; channel++)
				m_csp[1]->ser_w(channel, m_csp[0]->ser_r(channel));
			m_csp[1]->run_once(768);
			for (unsigned channel = 0; channel < 8; channel++)
				stream.put_int_clamp(channel, sample, m_csp[1]->ser_r(slots[channel]), 8388608);
		}
	}

private:
	required_device_array<roland_csp_device, 2> m_csp;
	sound_stream *m_stream = nullptr;
};

DEFINE_DEVICE_TYPE(JD990_SOUND, jd990_sound_device, "jd990_sound", "JD-990 audio bus and output interface")

namespace {

class roland_jd990_state : public driver_device
{
public:
	roland_jd990_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_lcdc(*this, "lcdc")
		, m_ep(*this, "ep")
		, m_tvf(*this, "tvf")
		, m_csp(*this, "csp%u", 1U)
		, m_sound(*this, "ifcs")
		, m_waverom(*this, "waverom")
		, m_keys(*this, "KEY%u", 0U)
		, m_encoder(*this, "ENCODER")
	{
	}

	void jd990(machine_config &config);

protected:
	virtual void machine_start() override ATTR_COLD;

private:
	void isp_reset_w(int state);
	void isp_dr_w(offs_t offset, u8 data);
	TIMER_CALLBACK_MEMBER(isp_tick);
	TIMER_CALLBACK_MEMBER(isp_fxm);

	void mem_map(address_map &map) ATTR_COLD;
	void lcdc_map(address_map &map) ATTR_COLD;
	void lcd_palette(palette_device &palette) const ATTR_COLD;

	u16 page_e_r(offs_t offset, u16 mem_mask);
	void page_e_w(offs_t offset, u16 data, u16 mem_mask);
	template <int Chip> u8 csp_r(offs_t offset);
	template <int Chip> void csp_w(offs_t offset, u8 data);

	required_device<h8570_device> m_maincpu;
	required_device<sed1330_device> m_lcdc;
	required_device<roland_ep_device> m_ep;
	required_device<roland_tvf_device> m_tvf;
	required_device_array<roland_csp_device, 2> m_csp;
	required_device<jd990_sound_device> m_sound;
	required_region_ptr<u8> m_waverom;
	required_ioport_array<8> m_keys;
	required_ioport m_encoder;

	static constexpr int FXM_VOICES = 24;
	static constexpr attotime FXM_PERIOD = attotime::from_usec(100);

	emu_timer *m_isp_tick = nullptr;
	emu_timer *m_isp_fxm = nullptr;
	u8 m_isp_control = 0;
	u8 m_scan_column = 0;
	u8 m_scan_state[8] = {};
	u8 m_encoder_last = 0;
	u16 m_sense_ticks = 0;
	bool m_sense_counting = false;
	u8 m_fxm_count[FXM_VOICES] = {};
	u8 m_fxm_phase[FXM_VOICES] = {};
	std::unique_ptr<u8[]> m_wave;
};

void roland_jd990_state::machine_start()
{
	static const u8 addr_lines[20] = { 2, 0, 3, 4, 1, 9, 13, 10, 18, 17, 6, 15, 11, 16, 8, 5, 12, 7, 14, 19 };
	static const u8 data_lines[8] = { 2, 0, 4, 5, 7, 6, 3, 1 };
	const u32 size = m_waverom.bytes();
	m_wave = std::make_unique<u8[]>(size);
	for (u32 region = 0; region < size; region += 0x100000)
	{
		for (u32 i = 0; i < 0x100000; i++)
		{
			u32 src = 0;
			for (int bit = 0; bit < 20; bit++)
				if (BIT(i, bit))
					src |= 1 << addr_lines[bit];
			const u8 raw = m_waverom[region + src];
			u8 data = 0;
			for (int bit = 0; bit < 8; bit++)
				if (BIT(raw, data_lines[bit]))
					data |= 1 << bit;
			m_wave[region + i] = data;
		}
	}
	m_ep->space(roland_ep_device::AS_WAVE).install_rom(0, size - 1, m_wave.get());

	m_isp_tick = timer_alloc(FUNC(roland_jd990_state::isp_tick), this);
	m_isp_fxm = timer_alloc(FUNC(roland_jd990_state::isp_fxm), this);
	save_item(NAME(m_isp_control));
	save_item(NAME(m_scan_column));
	save_item(NAME(m_scan_state));
	save_item(NAME(m_encoder_last));
	save_item(NAME(m_sense_ticks));
	save_item(NAME(m_sense_counting));
	save_item(NAME(m_fxm_count));
	save_item(NAME(m_fxm_phase));
}

void roland_jd990_state::isp_dr_w(offs_t offset, u8 data)
{
	LOGMASKED(LOG_ISP, "%s: DR%d%c = %02X\n", machine().describe_context(), offset >> 1, BIT(offset, 0) ? 'L' : 'H', data);
	if (offset != 0x0c)
		return;

	const bool start = BIT(m_isp_control, 0) && !BIT(data, 0);
	m_isp_control = data;
	m_sense_counting = !BIT(data, 4);
	m_sense_ticks = 0;
	if (!start)
		return;

	address_space &space = m_maincpu->space(AS_PROGRAM);
	const u16 src = (m_maincpu->dr_r(0x0e) << 8) | m_maincpu->dr_r(0x0f);
	const u16 count = (m_maincpu->dr_r(0x14) << 8) | m_maincpu->dr_r(0x15);
	m_lcdc->command_w(0x42);
	for (u16 i = 0; i < count; i++)
		m_lcdc->data_w(space.read_byte(0x80000 | u16(src + i)));
}

void roland_jd990_state::isp_reset_w(int state)
{
	if (state)
	{
		m_isp_tick->adjust(attotime::never);
		m_isp_fxm->adjust(attotime::never);
	}
	else
	{
		m_isp_tick->adjust(attotime::from_msec(1), 0, attotime::from_msec(1));
		m_isp_fxm->adjust(FXM_PERIOD, 0, FXM_PERIOD);
	}
}

TIMER_CALLBACK_MEMBER(roland_jd990_state::isp_fxm)
{
	if (BIT(m_maincpu->isp_icf(), 0))
		return;

	address_space &space = m_maincpu->space(AS_PROGRAM);
	const u32 keyed = (u32(m_maincpu->dr_r(0x36)) << 24) | (u32(m_maincpu->dr_r(0x37)) << 16) | (u32(m_maincpu->dr_r(0x34)) << 8) | m_maincpu->dr_r(0x35);
	for (int v = 0; v < FXM_VOICES; v++)
	{
		const u8 code = space.read_word(0x8fb78 + 2 * v) >> 8;
		if (!code || !BIT(keyed, v))
			continue;
		if (++m_fxm_count[v] < code)
			continue;
		m_fxm_count[v] = 0;
		m_fxm_phase[v] ^= 1;
		m_ep->voice_pitch_w(v, space.read_word((m_fxm_phase[v] ? 0x8ffa0 : 0x8ffd0) + 2 * v));
	}
}

TIMER_CALLBACK_MEMBER(roland_jd990_state::isp_tick)
{
	address_space &space = m_maincpu->space(AS_PROGRAM);

	m_maincpu->isp_raise(9);
	if (space.read_byte(0x87e85))
		m_maincpu->isp_raise(0);

	const u8 column = m_scan_column;
	m_scan_column = (m_scan_column + 1) & 7;
	const u8 keys = m_keys[column]->read();
	if (keys != m_scan_state[column])
	{
		m_scan_state[column] = keys;
		space.write_byte(0x8ff90 + column, keys);
		m_maincpu->dr_w(0x0a, column);
		m_maincpu->isp_raise(3);
	}

	if (m_sense_counting)
	{
		m_sense_ticks++;
		m_maincpu->dr_w(0x08, m_sense_ticks >> 1);
		if (m_sense_ticks == 300)
		{
			m_sense_counting = false;
			m_maincpu->isp_raise(8);
		}
	}

	const u8 encoder = m_encoder->read();
	const s8 delta = s8(encoder - m_encoder_last);
	m_encoder_last = encoder;
	if (delta)
	{
		m_maincpu->dr_w(0x3e, m_maincpu->dr_r(0x3e) + delta);
		m_maincpu->isp_raise(10);
	}
}

u16 roland_jd990_state::page_e_r(offs_t offset, u16 mem_mask)
{
	if (!machine().side_effects_disabled())
		LOGMASKED(LOG_EP, "%s: page 0E read %04X (mask %04X)\n", machine().describe_context(), offset << 1, mem_mask);
	return 0;
}

void roland_jd990_state::page_e_w(offs_t offset, u16 data, u16 mem_mask)
{
	LOGMASKED(LOG_EP, "%s: page 0E write %04X = %04X (mask %04X)\n", machine().describe_context(), offset << 1, data, mem_mask);
}

template <int Chip>
u8 roland_jd990_state::csp_r(offs_t offset)
{
	if (!machine().side_effects_disabled())
	{
		m_sound->update();
		LOGMASKED(LOG_CSP, "%s: CSP%d read %04X\n", machine().describe_context(), Chip + 1, offset);
	}
	return m_csp[Chip]->host_r(offset);
}

template <int Chip>
void roland_jd990_state::csp_w(offs_t offset, u8 data)
{
	m_sound->update();
	LOGMASKED(LOG_CSP, "%s: CSP%d write %04X = %02X\n", machine().describe_context(), Chip + 1, offset, data);
	m_csp[Chip]->host_w(offset, data);
}

void roland_jd990_state::mem_map(address_map &map)
{
	map(0x00000, 0x7ffff).rom().region("progrom", 0);
	map(0x08000, 0x0ffff).mirror(0x80000).ram().share("nvram_hi");
	map(0x80000, 0x87fff).ram().share("nvram_lo");
	map(0xe0000, 0xe7fff).rw(FUNC(roland_jd990_state::page_e_r), FUNC(roland_jd990_state::page_e_w));
	map(0xe0000, 0xe007f).rw(m_ep, FUNC(roland_ep_device::read), FUNC(roland_ep_device::write));
	map(0xe4000, 0xe407f).rw(m_tvf, FUNC(roland_tvf_device::read), FUNC(roland_tvf_device::write));
	map(0xe8000, 0xeffff).ram().share("nvram_hi");
	map(0xf0000, 0xf3fff).rw(FUNC(roland_jd990_state::csp_r<0>), FUNC(roland_jd990_state::csp_w<0>));
	map(0xf4000, 0xf7fff).rw(FUNC(roland_jd990_state::csp_r<1>), FUNC(roland_jd990_state::csp_w<1>));
	map(0xfa000, 0xfa000).rw(m_lcdc, FUNC(sed1330_device::data_r), FUNC(sed1330_device::data_w));
	map(0xfa002, 0xfa002).rw(m_lcdc, FUNC(sed1330_device::status_r), FUNC(sed1330_device::command_w));
}

void roland_jd990_state::lcdc_map(address_map &map)
{
	map(0x0000, 0x1fff).mirror(0xe000).ram();
}

void roland_jd990_state::lcd_palette(palette_device &palette) const
{
	palette.set_pen_color(0, rgb_t(131, 136, 139));
	palette.set_pen_color(1, rgb_t(0, 0, 0));
}

static INPUT_PORTS_START(jd990)
	PORT_START("KEY0")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Cursor Up")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Cursor Left")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Rhythm")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("SW 0/3")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("SW 0/4")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Patch")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("SW 0/6")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("SW 0/7")

	PORT_START("KEY1")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Undo")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("System Setup")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Effects On/Off")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("SW 1/3")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("F4")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("F5")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("F6")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Exit")

	PORT_START("KEY2")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("User Int")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("User Card")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Preset A")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Preset B")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Utility")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("F1")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("F2")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("F3")

	PORT_START("KEY3")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Tone Switch 1")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Tone Switch 2")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Tone Switch 3")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Tone Switch 4")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Dec")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Inc")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("SW 3/6")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("SW 3/7")

	PORT_START("KEY4")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Tone Select 1")
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Tone Select 2")
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Tone Select 3")
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Tone Select 4")
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Cursor Right")
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("SW 4/5")
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("ROM Play")
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Cursor Down")

	PORT_START("KEY5")
	PORT_BIT(0xff, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("KEY6")
	PORT_BIT(0xff, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("KEY7")
	PORT_BIT(0xff, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("ENCODER")
	PORT_BIT(0xff, 0x00, IPT_DIAL) PORT_SENSITIVITY(25) PORT_KEYDELTA(1) PORT_NAME("Value")
INPUT_PORTS_END

void roland_jd990_state::jd990(machine_config &config)
{
	H8570(config, m_maincpu, 20_MHz_XTAL);
	m_maincpu->set_mode(3);
	m_maincpu->set_addrmap(AS_PROGRAM, &roland_jd990_state::mem_map);
	m_maincpu->isp_reset_cb().set(FUNC(roland_jd990_state::isp_reset_w));
	m_maincpu->isp_dr_write_cb().set(FUNC(roland_jd990_state::isp_dr_w));

	NVRAM(config, "nvram_lo", nvram_device::DEFAULT_ALL_0);
	NVRAM(config, "nvram_hi", nvram_device::DEFAULT_ALL_0);

	screen_device &screen(SCREEN(config, "screen"));
	screen.set_lcd();
	screen.set_refresh_hz(60);
	screen.set_screen_update("lcdc", FUNC(sed1330_device::screen_update));
	screen.set_size(320, 80);
	screen.set_visarea_full();
	screen.set_palette("palette");

	PALETTE(config, "palette", FUNC(roland_jd990_state::lcd_palette), 2);

	SED1330(config, m_lcdc, 20_MHz_XTAL / 2);
	m_lcdc->set_screen("screen");
	m_lcdc->set_addrmap(0, &roland_jd990_state::lcdc_map);

	SPEAKER(config, "speaker", 2).front();

	ROLAND_EP(config, m_ep, 44100);
	ROLAND_TVF(config, m_tvf, 44100);
	for (int n = 0; n < roland_ep_device::VOICES; n++)
		m_ep->add_route(n, m_tvf, 1.0, n);
	for (auto &csp : m_csp)
		ROLAND_CSP(config, csp, 67.7376_MHz_XTAL);
	JD990_SOUND(config, m_sound, 0);
	for (int n = 0; n < 24; n++)
		m_tvf->add_route(n, m_sound, 1.0, n);
	for (int pair = 0; pair < 4; pair++)
	{
		m_sound->add_route(pair * 2, "speaker", 1.0, 0);
		m_sound->add_route(pair * 2 + 1, "speaker", 1.0, 1);
	}

	midi_port_device &mdin(MIDI_PORT(config, "mdin", midiin_slot, "midiin"));
	mdin.rxd_handler().set(m_maincpu, FUNC(h8570_device::sci_rx_w<0>));

	midi_port_device &mdout(MIDI_PORT(config, "mdout", midiout_slot, "midiout"));
	m_maincpu->write_sci_tx<0>().set(mdout, FUNC(midi_port_device::write_txd));
}

ROM_START(jd990)
	ROM_REGION16_BE(0x80000, "progrom", 0)
	ROM_LOAD16_WORD_SWAP("jd990_v1.05.ic31", 0x00000, 0x80000, CRC(2555146f) SHA1(9e74c6e8b22b26e4427830487ca5074bdd2bd352))

	ROM_REGION(0x600000, "waverom", 0)
	ROM_LOAD("wa_r15209393.bin", 0x000000, 0x200000, CRC(ac55642d) SHA1(ca25811273ebfdb1b02472ccccf1dd040c747d04))
	ROM_LOAD("wb_r15209394.bin", 0x200000, 0x200000, CRC(63a2f96e) SHA1(14dd6be63718713d515c66bd57813c8766d9e5d7))
	ROM_LOAD("wc_r15209395.bin", 0x400000, 0x200000, CRC(2b5e790c) SHA1(38749fb6bd074355cc922a061d9c47ad1d69ff6b))
ROM_END

} // anonymous namespace


SYST(1993, jd990, 0, 0, jd990, jd990, roland_jd990_state, empty_init, "Roland", "JD-990", MACHINE_NOT_WORKING | MACHINE_IMPERFECT_SOUND)
