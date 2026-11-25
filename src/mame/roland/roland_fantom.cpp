// license:BSD-3-Clause
// copyright-holders:superctr
/****************************************************************************

    Roland Fantom (FA-76)

    Main board:
    - IC3 HD6417709AF133 SH-3, 16.5 MHz x 8 = 132 MHz
    - IC5 TC58FVB321FT-10 program flash, 4 MB
    - IC4/IC7 HY57V641620HGT-P SDRAM, 16 MB
    - IC12 BR24C08F serial EEPROM, 8 kbit
    - IC13 M66273FP LCD controller; LM320191 LCD, 320 x 240
    - IC24 S1L50282F32H000 key scan, 24 MHz
    - IC25 FDC37C78 floppy disk controller
    - IC31 TC223C660CF-503 XV tone generator, 16.9344 MHz
    - IC37 GM71V18163CT-6 effect DRAM
    - IC48/IC51 uPD23C128040LGY wave mask ROMs, 16 MB each
    - IC59/IC62 AK4393-VF-E2 DACs, IC68 TC9271FS digital output
    - CN7 SR-JV80 socket, CN10/CN11 SRX sockets

    76-note keyboard, D-Beam, 3.5" floppy drive, one MIDI IN and OUT.

    TODO:
    - the performance mode plays about 20 dB above the GM mode and clips;
      what the firmware writes to the XV there is unchecked on a real unit
    - the key scan's time stamp unit, so the key velocity, is a guess

****************************************************************************/

#include "emu.h"

#include "wavecard.h"

#include "bus/midi/midiinport.h"
#include "bus/midi/midioutport.h"
#include "cpu/sh/sh3comn.h"
#include "cpu/sh/sh4.h"
#include "imagedev/floppy.h"
#include "machine/i2cmem.h"
#include "machine/intelfsh.h"
#include "machine/upd765.h"
#include "sound/roland_xv.h"
#include "video/m66273.h"

#include "emupal.h"
#include "screen.h"
#include "speaker.h"

#include <algorithm>


namespace {

class fantom_keyscan_device : public device_t
{
public:
	static constexpr feature_type imperfect_features() { return feature::CONTROLS; }

	fantom_keyscan_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

	auto int_handler() { return m_int_cb.bind(); }

	u16 read(offs_t offset);
	void key_w(int key, int state, const attotime &travel);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	static constexpr u32 KEYS = 88;
	static constexpr u32 TIME_BITS = 14;
	static constexpr u32 FIFO_SIZE = 64;
	static constexpr u32 PENDING = 256;

	void contact_w(int key, int contact, int state);
	void push(u8 status, u8 key);
	void pend(const attotime &when, int key, int contact, int state);
	void update_int();
	void arm_travel();
	u16 stamp() const;
	TIMER_CALLBACK_MEMBER(wrap);
	TIMER_CALLBACK_MEMBER(travel);

	devcb_write_line m_int_cb;
	emu_timer *m_wrap_timer;
	emu_timer *m_travel_timer;

	u8 m_fifo_status[FIFO_SIZE];
	u8 m_fifo_key[FIFO_SIZE];
	u16 m_fifo_time[FIFO_SIZE];
	u8 m_fifo_head;
	u8 m_fifo_count;
	attotime m_pending_when[PENDING];
	u8 m_pending_key[PENDING];
	u8 m_pending_contact[PENDING];
	u8 m_pending_state[PENDING];
	u16 m_pending_count;
	u8 m_latch_status;
	u8 m_latch_key;
	u16 m_latch_time;
	u8 m_contacts[KEYS];
	u8 m_int;
};

} // anonymous namespace

DEFINE_DEVICE_TYPE_PRIVATE(FANTOM_KEYSCAN, fantom_keyscan_device, fantom_keyscan_device, "fantom_keyscan", "Roland Fantom key scan")

namespace {

fantom_keyscan_device::fantom_keyscan_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, FANTOM_KEYSCAN, tag, owner, clock)
	, m_int_cb(*this)
	, m_wrap_timer(nullptr)
	, m_travel_timer(nullptr)
	, m_fifo_head(0)
	, m_fifo_count(0)
	, m_pending_count(0)
	, m_latch_status(0)
	, m_latch_key(0)
	, m_latch_time(0)
	, m_int(0)
{
}

void fantom_keyscan_device::device_start()
{
	m_wrap_timer = timer_alloc(FUNC(fantom_keyscan_device::wrap), this);
	m_travel_timer = timer_alloc(FUNC(fantom_keyscan_device::travel), this);

	save_item(NAME(m_fifo_status));
	save_item(NAME(m_fifo_key));
	save_item(NAME(m_fifo_time));
	save_item(NAME(m_fifo_head));
	save_item(NAME(m_fifo_count));
	save_item(NAME(m_pending_when));
	save_item(NAME(m_pending_key));
	save_item(NAME(m_pending_contact));
	save_item(NAME(m_pending_state));
	save_item(NAME(m_pending_count));
	save_item(NAME(m_latch_status));
	save_item(NAME(m_latch_key));
	save_item(NAME(m_latch_time));
	save_item(NAME(m_contacts));
	save_item(NAME(m_int));
}

void fantom_keyscan_device::device_reset()
{
	std::fill(std::begin(m_fifo_status), std::end(m_fifo_status), 0);
	std::fill(std::begin(m_fifo_key), std::end(m_fifo_key), 0);
	std::fill(std::begin(m_fifo_time), std::end(m_fifo_time), 0);
	m_fifo_head = 0;
	m_fifo_count = 0;
	std::fill(std::begin(m_pending_when), std::end(m_pending_when), attotime::zero);
	std::fill(std::begin(m_pending_key), std::end(m_pending_key), 0);
	std::fill(std::begin(m_pending_contact), std::end(m_pending_contact), 0);
	std::fill(std::begin(m_pending_state), std::end(m_pending_state), 0);
	m_pending_count = 0;
	m_latch_status = 0;
	m_latch_key = 0;
	m_latch_time = 0;
	std::fill(std::begin(m_contacts), std::end(m_contacts), 0);
	m_int = 0;
	m_int_cb(0);

	const attotime period = clocks_to_attotime(1 << TIME_BITS);
	m_wrap_timer->adjust(period, 0, period);
	m_travel_timer->adjust(attotime::never);
}

u16 fantom_keyscan_device::stamp() const
{
	return attotime_to_clocks(machine().time()) & util::make_bitmask<u16>(TIME_BITS);
}

void fantom_keyscan_device::push(u8 status, u8 key)
{
	if (m_fifo_count == FIFO_SIZE)
		return;
	const u32 at = (m_fifo_head + m_fifo_count) % FIFO_SIZE;
	m_fifo_status[at] = status;
	m_fifo_key[at] = key;
	m_fifo_time[at] = stamp();
	m_fifo_count++;
	update_int();
}

void fantom_keyscan_device::update_int()
{
	const u8 state = m_fifo_count ? 1 : 0;
	if (state != m_int)
	{
		m_int = state;
		m_int_cb(state);
	}
}

TIMER_CALLBACK_MEMBER(fantom_keyscan_device::wrap)
{
	push(0x04, 0);
}

void fantom_keyscan_device::contact_w(int key, int contact, int state)
{
	const u8 mask = 1 << contact;
	const u8 contacts = (m_contacts[key] & ~mask) | (state ? mask : 0);
	if (contacts == m_contacts[key])
		return;
	m_contacts[key] = contacts;
	push(0x40 | (contacts << 4), key);
}

void fantom_keyscan_device::pend(const attotime &when, int key, int contact, int state)
{
	if (m_pending_count == PENDING)
		return;
	m_pending_when[m_pending_count] = when;
	m_pending_key[m_pending_count] = key;
	m_pending_contact[m_pending_count] = contact;
	m_pending_state[m_pending_count] = state;
	m_pending_count++;
}

void fantom_keyscan_device::key_w(int key, int state, const attotime &travel)
{
	if (key < 0 || key >= int(KEYS))
		return;

	const attotime now = machine().time();
	attotime at = now;
	for (u32 n = 0; n < m_pending_count; n++)
		if (m_pending_key[n] == key && m_pending_when[n] > at)
			at = m_pending_when[n];

	if (at == now)
		contact_w(key, state ? 0 : 1, state);
	else
		pend(at, key, state ? 0 : 1, state ? 1 : 0);
	pend(at + travel, key, state ? 1 : 0, state ? 1 : 0);
	arm_travel();
}

void fantom_keyscan_device::arm_travel()
{
	attotime next = attotime::never;
	for (u32 n = 0; n < m_pending_count; n++)
		if (m_pending_when[n] < next)
			next = m_pending_when[n];
	m_travel_timer->adjust(next.is_never() ? attotime::never : next - machine().time());
}

TIMER_CALLBACK_MEMBER(fantom_keyscan_device::travel)
{
	const attotime now = machine().time();
	u32 kept = 0;
	for (u32 n = 0; n < m_pending_count; n++)
	{
		if (m_pending_when[n] <= now)
			contact_w(m_pending_key[n], m_pending_contact[n], m_pending_state[n]);
		else
		{
			m_pending_when[kept] = m_pending_when[n];
			m_pending_key[kept] = m_pending_key[n];
			m_pending_contact[kept] = m_pending_contact[n];
			m_pending_state[kept] = m_pending_state[n];
			kept++;
		}
	}
	m_pending_count = kept;
	arm_travel();
}

u16 fantom_keyscan_device::read(offs_t offset)
{
	if (offset)
		return m_latch_time;

	const bool empty = !m_fifo_count;
	const u8 status = empty ? 0 : m_fifo_status[m_fifo_head];
	const u8 key = empty ? 0 : m_fifo_key[m_fifo_head];
	if (!machine().side_effects_disabled())
	{
		m_latch_status = status;
		m_latch_key = key;
		m_latch_time = empty ? 0 : m_fifo_time[m_fifo_head];
		if (!empty)
		{
			m_fifo_head = (m_fifo_head + 1) % FIFO_SIZE;
			m_fifo_count--;
		}
		update_int();
	}
	return (status << 8) | key;
}


class fantom_state : public driver_device
{
public:
	fantom_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_flash(*this, "flash")
		, m_fdc(*this, "fdc")
		, m_xv(*this, "xv")
		, m_exp(*this, "expa")
		, m_srx(*this, "exp%c", 'b')
		, m_eeprom(*this, "eeprom")
		, m_keyscan(*this, "keyscan")
		, m_lcdc(*this, "lcdc")
		, m_velocity(*this, "VELOCITY")
		, m_encoder(*this, "ENCODER")
		, m_analog(*this, "AN%u", 0U)
		, m_leds(*this, "led_%u", 0U)
	{
	}

	void fantom(machine_config &config) ATTR_COLD;

	void init_waverom() ATTR_COLD;

	DECLARE_INPUT_CHANGED_MEMBER(key_changed);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	enum { LED_BEAT_A, LED_BEAT_B, LED_REC, LED_RTC1, LED_RTC2, LED_RTC3, LED_RTC4, LED_ARPEGGIO, LED_RHYTHM, LED_PLAY, LED_DBEAM, LEDS };

	void fantom_map(address_map &map) ATTR_COLD;
	void fantom_io_map(address_map &map) ATTR_COLD;
	void xv_wave_map(address_map &map) ATTR_COLD;

	u64 portc_r();
	void portc_w(u64 data);
	void portd_w(u64 data);
	u64 porte_r();
	void porte_w(u64 data);
	u64 portf_r();
	void porth_w(u64 data);
	void fdc_drq_w(int state);
	void fdc_dack();
	TIMER_CALLBACK_MEMBER(fdc_dack_end);
	u8 fdc_data_r();
	void fdc_data_w(u8 data);
	void led_w(offs_t offset, u8 data);
	template <int N> u16 adc_r();

	TIMER_CALLBACK_MEMBER(encoder_tick);

	u32 screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	void lcd_palette(palette_device &palette) const ATTR_COLD;

	required_device<sh7709_device> m_maincpu;
	required_device<tc58fvb321_device> m_flash;
	required_device<smc37c78_device> m_fdc;
	required_device<roland_xv_device> m_xv;
	required_device<roland_srjv80_slot_device> m_exp;
	required_device_array<roland_srx_slot_device, 2> m_srx;
	required_device<i2c_24c08_device> m_eeprom;
	required_device<fantom_keyscan_device> m_keyscan;
	required_device<m66273_device> m_lcdc;
	required_ioport m_velocity;
	required_ioport m_encoder;
	required_ioport_array<11> m_analog;
	output_finder<LEDS> m_leds;

	emu_timer *m_encoder_timer = nullptr;
	emu_timer *m_fdc_dack_timer = nullptr;
	u8 m_encoder_last = 0;
	s32 m_encoder_pending = 0;
	u8 m_encoder_phase = 0;
	u8 m_adsel = 0;
	u8 m_lcdenb = 0;
	u8 m_fdc_drq = 0;
};


void fantom_state::init_waverom()
{
	roland_srx_slot_device::descramble_rom(memregion("waverom")->base(), memregion("waverom")->bytes());
}

void fantom_state::machine_start()
{
	m_encoder_timer = timer_alloc(FUNC(fantom_state::encoder_tick), this);
	m_fdc_dack_timer = timer_alloc(FUNC(fantom_state::fdc_dack_end), this);

	save_item(NAME(m_encoder_last));
	save_item(NAME(m_encoder_pending));
	save_item(NAME(m_encoder_phase));
	save_item(NAME(m_adsel));
	save_item(NAME(m_lcdenb));
	save_item(NAME(m_fdc_drq));
}

void fantom_state::machine_reset()
{
	m_encoder_last = m_encoder->read();
	m_encoder_pending = 0;
	m_encoder_phase = 0;
	m_encoder_timer->adjust(attotime::from_msec(2), 0, attotime::from_msec(2));
}

void fantom_state::lcd_palette(palette_device &palette) const
{
	for (int i = 0; i < m66273_device::LEVELS; i++)
	{
		const int level = 0xff * i / (m66273_device::LEVELS - 1);
		palette.set_pen_color(i, rgb_t(level, level, level));
	}
}

u32 fantom_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	if (!m_lcdenb)
	{
		bitmap.fill(0, cliprect);
		return 0;
	}
	return m_lcdc->screen_update(screen, bitmap, cliprect);
}

u64 fantom_state::portc_r()
{
	return 0x3f | (m_eeprom->read_sda() << 7);
}

void fantom_state::portc_w(u64 data)
{
	const u16 pccr = data >> 16;
	const bool sda_out = BIT(pccr, 14, 2) == 1;
	const bool scl_out = BIT(pccr, 12, 2) == 1;
	m_eeprom->write_sda(sda_out ? BIT(data, 7) : 1);
	m_eeprom->write_scl(scl_out ? BIT(data, 6) : 1);
}

void fantom_state::portd_w(u64 data)
{
	m_adsel = BIT(data, 7);
}

TIMER_CALLBACK_MEMBER(fantom_state::encoder_tick)
{
	const u8 dial = m_encoder->read();
	m_encoder_pending += s8(dial - m_encoder_last);
	m_encoder_last = dial;

	if (m_encoder_pending > 0)
	{
		m_encoder_phase = (m_encoder_phase + 1) & 3;
		if (m_encoder_phase == 0)
			m_encoder_pending--;
	}
	else if (m_encoder_pending < 0)
	{
		m_encoder_phase = (m_encoder_phase - 1) & 3;
		if (m_encoder_phase == 0)
			m_encoder_pending++;
	}
}

u64 fantom_state::porte_r()
{
	return 0x02;
}

void fantom_state::porte_w(u64 data)
{
	m_flash->vid_w(BIT(data, 3));
}

u64 fantom_state::portf_r()
{
	constexpr u8 PHASES[4] = { 0x3, 0x1, 0x0, 0x2 };
	return 0xfc | PHASES[m_encoder_phase];
}

template <int N>
u16 fantom_state::adc_r()
{
	switch (N)
	{
	case 5: return m_analog[m_adsel ? 6 : 5]->read();
	case 6: return m_analog[m_adsel ? 8 : 7]->read();
	case 7: return m_analog[m_adsel ? 10 : 9]->read();
	default: return m_analog[N]->read();
	}
}

void fantom_state::fdc_drq_w(int state)
{
	m_fdc_drq = state;
	m_fdc_dack_timer->adjust(attotime::never);
	m_maincpu->dreq_w<0>(state);
}

TIMER_CALLBACK_MEMBER(fantom_state::fdc_dack_end)
{
	m_maincpu->dreq_w<0>(m_fdc_drq);
}

void fantom_state::fdc_dack()
{
	if (m_fdc_drq)
	{
		m_maincpu->dreq_w<0>(0);
		m_fdc_dack_timer->adjust(attotime::from_usec(1));
	}
}

u8 fantom_state::fdc_data_r()
{
	if (!m_fdc_drq || machine().side_effects_disabled())
		return m_fdc->fifo_r();
	const u8 data = m_fdc->dma_r();
	fdc_dack();
	return data;
}

void fantom_state::fdc_data_w(u8 data)
{
	if (!m_fdc_drq)
		m_fdc->fifo_w(data);
	else
	{
		m_fdc->dma_w(data);
		fdc_dack();
	}
}

void fantom_state::porth_w(u64 data)
{
	m_fdc->tc_w(BIT(data, 7));
}

void fantom_state::led_w(offs_t offset, u8 data)
{
	constexpr s8 LED_MAP[4][3] = {
		{ LED_BEAT_A, LED_RTC2, LED_ARPEGGIO },
		{ LED_BEAT_B, LED_RTC3, LED_RHYTHM },
		{ LED_REC, LED_RTC4, LED_PLAY },
		{ -1, LED_RTC1, LED_DBEAM } };
	const u32 strobe = offset >> 3, line = offset & 7;
	if (strobe < 4 && line < 3 && LED_MAP[strobe][line] >= 0)
		m_leds[LED_MAP[strobe][line]] = data;
}

INPUT_CHANGED_MEMBER(fantom_state::key_changed)
{
	constexpr u32 TRAVEL_UNITS[8] = { 760, 480, 320, 200, 136, 80, 16, 16 };
	const attotime travel = m_keyscan->clocks_to_attotime(TRAVEL_UNITS[m_velocity->read() & 7]);
	m_keyscan->key_w(param, newval, travel);
}

void fantom_state::fantom_map(address_map &map)
{
	map(0x00000000, 0x003fffff).rw(m_flash, FUNC(tc58fvb321_device::read), FUNC(tc58fvb321_device::write));
	map(0x0c000000, 0x0cffffff).ram();
	map(0x10000000, 0x10000003).r(m_keyscan, FUNC(fantom_keyscan_device::read));
	map(0x14000000, 0x14000007).m(m_fdc, FUNC(smc37c78_device::map));
	map(0x14000005, 0x14000005).rw(FUNC(fantom_state::fdc_data_r), FUNC(fantom_state::fdc_data_w));
	map(0x18000000, 0x180001ff).mirror(0x7e00).rw(m_xv, FUNC(roland_xv_device::read), FUNC(roland_xv_device::write));
	map(0x18008000, 0x1800ffff).rw(m_lcdc, FUNC(m66273_device::read), FUNC(m66273_device::write));
}

void fantom_state::fantom_io_map(address_map &map)
{
	map(SH3_PORT_C, SH3_PORT_C + 7).rw(FUNC(fantom_state::portc_r), FUNC(fantom_state::portc_w));
	map(SH3_PORT_D, SH3_PORT_D + 7).w(FUNC(fantom_state::portd_w));
	map(SH3_PORT_E, SH3_PORT_E + 7).rw(FUNC(fantom_state::porte_r), FUNC(fantom_state::porte_w));
	map(SH3_PORT_F, SH3_PORT_F + 7).r(FUNC(fantom_state::portf_r));
	map(SH3_PORT_H, SH3_PORT_H + 7).w(FUNC(fantom_state::porth_w));
}

void fantom_state::xv_wave_map(address_map &map)
{
	map.unmap_value_high();
	map(0x00000000, 0x00ffffff).mirror(0x01000000).rom().region("waverom", 0);
	map(0x02000000, 0x027fffff).umask16(0x00ff).r(m_exp, FUNC(roland_srjv80_slot_device::read));
	map(0x04000000, 0x04ffffff).r(m_srx[0], FUNC(roland_srx_slot_device::read16));
	map(0x07000000, 0x07ffffff).r(m_srx[1], FUNC(roland_srx_slot_device::read16));
}

void fantom_state::fantom(machine_config &config)
{
	SH7709(config, m_maincpu, 16.5_MHz_XTAL * 8, ENDIANNESS_BIG);
	m_maincpu->set_addrmap(AS_PROGRAM, &fantom_state::fantom_map);
	m_maincpu->set_addrmap(AS_IO, &fantom_state::fantom_io_map);
	m_maincpu->read_adc<0>().set(FUNC(fantom_state::adc_r<0>));
	m_maincpu->read_adc<1>().set(FUNC(fantom_state::adc_r<1>));
	m_maincpu->read_adc<2>().set(FUNC(fantom_state::adc_r<2>));
	m_maincpu->read_adc<3>().set(FUNC(fantom_state::adc_r<3>));
	m_maincpu->read_adc<4>().set(FUNC(fantom_state::adc_r<4>));
	m_maincpu->read_adc<5>().set(FUNC(fantom_state::adc_r<5>));
	m_maincpu->read_adc<6>().set(FUNC(fantom_state::adc_r<6>));
	m_maincpu->read_adc<7>().set(FUNC(fantom_state::adc_r<7>));

	TC58FVB321(config, m_flash);

	I2C_24C08(config, m_eeprom);

	FANTOM_KEYSCAN(config, m_keyscan, 10'000);
	m_keyscan->int_handler().set_inputline(m_maincpu, 2);

	SMC37C78(config, m_fdc, 24_MHz_XTAL);
	m_fdc->intrq_wr_callback().set_inputline(m_maincpu, 3);
	m_fdc->drq_wr_callback().set(FUNC(fantom_state::fdc_drq_w));
	FLOPPY_CONNECTOR(config, "fdc:1", "35hd", FLOPPY_35_HD, true, floppy_image_device::default_pc_floppy_formats);

	screen_device &screen(SCREEN(config, "screen").set_lcd());
	screen.set_refresh_hz(62.5);
	screen.set_size(320, 240);
	screen.set_visarea_full();
	screen.set_screen_update(FUNC(fantom_state::screen_update));
	screen.set_palette("palette");
	PALETTE(config, "palette", FUNC(fantom_state::lcd_palette), m66273_device::LEVELS);

	M66273(config, m_lcdc, 24_MHz_XTAL / 2);
	m_lcdc->set_screen("screen");
	m_lcdc->cse_callback().set([this] (int state) { m_maincpu->dreq_w<1>(!state); });
	m_lcdc->lcdenb_callback().set([this] (int state) { m_lcdenb = state; });

	ROLAND_SRJV80_SLOT(config, m_exp, 0).set_image_names("xp-a", "xp-a").set_open(0xffff);
	for (int slot = 0; slot < 2; slot++)
		ROLAND_SRX_SLOT(config, m_srx[slot], 0).set_image_names(util::string_format("xp-%c", 'b' + slot), util::string_format("xp-%c", 'b' + slot)).set_open(0xffff);

	SPEAKER(config, "outa", 2).front();
	SPEAKER(config, "outb", 2).front();

	ROLAND_XV(config, m_xv, 16.9344_MHz_XTAL);
	m_xv->set_addrmap(roland_xv_device::AS_WAVE, &fantom_state::xv_wave_map);
	m_xv->int_callback().set_inputline(m_maincpu, 0);
	m_xv->switch_callback().set_ioport("PANEL");
	m_xv->led_callback().set(FUNC(fantom_state::led_w));
	m_xv->add_route(0, "outa", 1.0, 0);
	m_xv->add_route(1, "outa", 1.0, 1);
	m_xv->add_route(2, "outb", 1.0, 0);
	m_xv->add_route(3, "outb", 1.0, 1);

	m_maincpu->sci_txd_handler().set([this] (int state) { m_maincpu->adtrg_w(!state); });

	midi_port_device &mdin(MIDI_PORT(config, "mdin", midiin_slot, "midiin"));
	mdin.rxd_handler().set(m_maincpu, FUNC(sh7709_device::irda_rxd_w));
	midi_port_device &mdout(MIDI_PORT(config, "mdout", midiout_slot, "midiout"));
	m_maincpu->irda_txd_handler().set(mdout, FUNC(midi_port_device::write_txd));
}


INPUT_PORTS_START(fantom)
	PORT_START("PANEL")
	PORT_BIT(0x00000001, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Realtime Control 4")
	PORT_BIT(0x00000002, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Stop/Play") PORT_CODE(KEYCODE_SPACE)
	PORT_BIT(0x00000004, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Rec") PORT_CODE(KEYCODE_R)
	PORT_BIT(0x00000008, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Mode") PORT_CODE(KEYCODE_F1)
	PORT_BIT(0x00000010, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Menu") PORT_CODE(KEYCODE_F2)
	PORT_BIT(0x00000020, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("List") PORT_CODE(KEYCODE_F3)
	PORT_BIT(0x00000040, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Favorite 1")
	PORT_BIT(0x00000080, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Favorite 5")
	PORT_BIT(0x00000100, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Realtime Control 3")
	PORT_BIT(0x00000200, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Fwd")
	PORT_BIT(0x00000400, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Rhythm")
	PORT_BIT(0x00000800, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Dec") PORT_CODE(KEYCODE_MINUS)
	PORT_BIT(0x00001000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Cursor Left") PORT_CODE(KEYCODE_LEFT)
	PORT_BIT(0x00002000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Shift") PORT_CODE(KEYCODE_LSHIFT)
	PORT_BIT(0x00004000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Favorite 2")
	PORT_BIT(0x00008000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Favorite 6")
	PORT_BIT(0x00010000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Realtime Control 2")
	PORT_BIT(0x00020000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Bwd")
	PORT_BIT(0x00040000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Arpeggio")
	PORT_BIT(0x00080000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Cursor Up") PORT_CODE(KEYCODE_UP)
	PORT_BIT(0x00100000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Cursor Down") PORT_CODE(KEYCODE_DOWN)
	PORT_BIT(0x00200000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Jump") PORT_CODE(KEYCODE_F4)
	PORT_BIT(0x00400000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Favorite 3")
	PORT_BIT(0x00800000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Favorite 7")
	PORT_BIT(0x01000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Realtime Control 1")
	PORT_BIT(0x02000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Reset")
	PORT_BIT(0x04000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("D Beam")
	PORT_BIT(0x08000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Inc") PORT_CODE(KEYCODE_EQUALS)
	PORT_BIT(0x10000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Cursor Right") PORT_CODE(KEYCODE_RIGHT)
	PORT_BIT(0x20000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Exit") PORT_CODE(KEYCODE_ESC)
	PORT_BIT(0x40000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Favorite 4")
	PORT_BIT(0x80000000, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_NAME("Favorite 8")

	PORT_START("VELOCITY")
	PORT_CONFNAME(0x07, 0x03, "Key velocity")
	PORT_CONFSETTING(0x00, "pp")
	PORT_CONFSETTING(0x01, "p")
	PORT_CONFSETTING(0x02, "mp")
	PORT_CONFSETTING(0x03, "mf")
	PORT_CONFSETTING(0x04, "f")
	PORT_CONFSETTING(0x05, "ff")
	PORT_CONFSETTING(0x06, "fff")

	PORT_START("ENCODER")
	PORT_BIT(0xff, 0x00, IPT_DIAL) PORT_NAME("Value") PORT_SENSITIVITY(25) PORT_KEYDELTA(2) PORT_CODE_DEC(KEYCODE_PGDN) PORT_CODE_INC(KEYCODE_PGUP)

	PORT_START("AN0")
	PORT_BIT(0x3ff, 0x200, IPT_PADDLE) PORT_NAME("Pitch Bend") PORT_MINMAX(0x000, 0x3ff) PORT_SENSITIVITY(50) PORT_KEYDELTA(32)

	PORT_START("AN1")
	PORT_BIT(0x3ff, 0x000, IPT_PEDAL) PORT_NAME("Modulation") PORT_MINMAX(0x000, 0x3ff) PORT_SENSITIVITY(50) PORT_KEYDELTA(32)

	PORT_START("AN2")
	PORT_BIT(0x3ff, 0x000, IPT_PEDAL3) PORT_NAME("D Beam") PORT_MINMAX(0x000, 0x3ff) PORT_SENSITIVITY(50) PORT_KEYDELTA(32)

	PORT_START("AN3")
	PORT_BIT(0x3ff, 0x000, IPT_PEDAL2) PORT_NAME("Aftertouch") PORT_MINMAX(0x000, 0x3ff) PORT_SENSITIVITY(50) PORT_KEYDELTA(32)

	PORT_START("AN4")
	PORT_BIT(0x3ff, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("Hold Pedal") PORT_CODE(KEYCODE_TAB)

	PORT_START("AN5")
	PORT_BIT(0x3ff, 0x3ff, IPT_PADDLE) PORT_NAME("Control Pedal 1") PORT_MINMAX(0x000, 0x3ff) PORT_SENSITIVITY(50) PORT_KEYDELTA(32) PORT_CENTERDELTA(0) PORT_PLAYER(6)

	PORT_START("AN6")
	PORT_BIT(0x3ff, 0x3ff, IPT_PADDLE) PORT_NAME("Control Pedal 2") PORT_MINMAX(0x000, 0x3ff) PORT_SENSITIVITY(50) PORT_KEYDELTA(32) PORT_CENTERDELTA(0) PORT_PLAYER(7)

	PORT_START("AN7")
	PORT_BIT(0x3ff, 0x200, IPT_PADDLE) PORT_NAME("Realtime Control Knob 1") PORT_MINMAX(0x000, 0x3ff) PORT_SENSITIVITY(50) PORT_KEYDELTA(32) PORT_CENTERDELTA(0) PORT_PLAYER(2)

	PORT_START("AN8")
	PORT_BIT(0x3ff, 0x200, IPT_PADDLE) PORT_NAME("Realtime Control Knob 2") PORT_MINMAX(0x000, 0x3ff) PORT_SENSITIVITY(50) PORT_KEYDELTA(32) PORT_CENTERDELTA(0) PORT_PLAYER(3)

	PORT_START("AN9")
	PORT_BIT(0x3ff, 0x200, IPT_PADDLE) PORT_NAME("Realtime Control Knob 3") PORT_MINMAX(0x000, 0x3ff) PORT_SENSITIVITY(50) PORT_KEYDELTA(32) PORT_CENTERDELTA(0) PORT_PLAYER(4)

	PORT_START("AN10")
	PORT_BIT(0x3ff, 0x200, IPT_PADDLE) PORT_NAME("Realtime Control Knob 4") PORT_MINMAX(0x000, 0x3ff) PORT_SENSITIVITY(50) PORT_KEYDELTA(32) PORT_CENTERDELTA(0) PORT_PLAYER(5)

#define FANTOM_KEY(port, bit, key, name, code) \
	PORT_BIT(1 << (bit), IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(name) code PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(fantom_state::key_changed), key)

	PORT_START("KEY0")
	FANTOM_KEY(0, 0,  0, "E1", )
	FANTOM_KEY(0, 1,  1, "F1", )
	FANTOM_KEY(0, 2,  2, "F#1", )
	FANTOM_KEY(0, 3,  3, "G1", )
	FANTOM_KEY(0, 4,  4, "G#1", )
	FANTOM_KEY(0, 5,  5, "A1", )
	FANTOM_KEY(0, 6,  6, "A#1", )
	FANTOM_KEY(0, 7,  7, "B1", )

	PORT_START("KEY1")
	FANTOM_KEY(1, 0,  8, "C2", )
	FANTOM_KEY(1, 1,  9, "C#2", )
	FANTOM_KEY(1, 2, 10, "D2", )
	FANTOM_KEY(1, 3, 11, "D#2", )
	FANTOM_KEY(1, 4, 12, "E2", )
	FANTOM_KEY(1, 5, 13, "F2", )
	FANTOM_KEY(1, 6, 14, "F#2", )
	FANTOM_KEY(1, 7, 15, "G2", )

	PORT_START("KEY2")
	FANTOM_KEY(2, 0, 16, "G#2", )
	FANTOM_KEY(2, 1, 17, "A2", )
	FANTOM_KEY(2, 2, 18, "A#2", )
	FANTOM_KEY(2, 3, 19, "B2", )
	FANTOM_KEY(2, 4, 20, "C3", PORT_CODE(KEYCODE_Z))
	FANTOM_KEY(2, 5, 21, "C#3", PORT_CODE(KEYCODE_S))
	FANTOM_KEY(2, 6, 22, "D3", PORT_CODE(KEYCODE_X))
	FANTOM_KEY(2, 7, 23, "D#3", PORT_CODE(KEYCODE_D))

	PORT_START("KEY3")
	FANTOM_KEY(3, 0, 24, "E3", PORT_CODE(KEYCODE_C))
	FANTOM_KEY(3, 1, 25, "F3", PORT_CODE(KEYCODE_V))
	FANTOM_KEY(3, 2, 26, "F#3", PORT_CODE(KEYCODE_G))
	FANTOM_KEY(3, 3, 27, "G3", PORT_CODE(KEYCODE_B))
	FANTOM_KEY(3, 4, 28, "G#3", PORT_CODE(KEYCODE_H))
	FANTOM_KEY(3, 5, 29, "A3", PORT_CODE(KEYCODE_N))
	FANTOM_KEY(3, 6, 30, "A#3", PORT_CODE(KEYCODE_J))
	FANTOM_KEY(3, 7, 31, "B3", PORT_CODE(KEYCODE_M))

	PORT_START("KEY4")
	FANTOM_KEY(4, 0, 32, "C4", PORT_CODE(KEYCODE_Q))
	FANTOM_KEY(4, 1, 33, "C#4", PORT_CODE(KEYCODE_2))
	FANTOM_KEY(4, 2, 34, "D4", PORT_CODE(KEYCODE_W))
	FANTOM_KEY(4, 3, 35, "D#4", PORT_CODE(KEYCODE_3))
	FANTOM_KEY(4, 4, 36, "E4", PORT_CODE(KEYCODE_E))
	FANTOM_KEY(4, 5, 37, "F4", PORT_CODE(KEYCODE_R))
	FANTOM_KEY(4, 6, 38, "F#4", PORT_CODE(KEYCODE_5))
	FANTOM_KEY(4, 7, 39, "G4", PORT_CODE(KEYCODE_T))

	PORT_START("KEY5")
	FANTOM_KEY(5, 0, 40, "G#4", PORT_CODE(KEYCODE_6))
	FANTOM_KEY(5, 1, 41, "A4", PORT_CODE(KEYCODE_Y))
	FANTOM_KEY(5, 2, 42, "A#4", PORT_CODE(KEYCODE_7))
	FANTOM_KEY(5, 3, 43, "B4", PORT_CODE(KEYCODE_U))
	FANTOM_KEY(5, 4, 44, "C5", PORT_CODE(KEYCODE_I))
	FANTOM_KEY(5, 5, 45, "C#5", PORT_CODE(KEYCODE_9))
	FANTOM_KEY(5, 6, 46, "D5", PORT_CODE(KEYCODE_O))
	FANTOM_KEY(5, 7, 47, "D#5", PORT_CODE(KEYCODE_0))

	PORT_START("KEY6")
	FANTOM_KEY(6, 0, 48, "E5", PORT_CODE(KEYCODE_P))
	FANTOM_KEY(6, 1, 49, "F5", )
	FANTOM_KEY(6, 2, 50, "F#5", )
	FANTOM_KEY(6, 3, 51, "G5", )
	FANTOM_KEY(6, 4, 52, "G#5", )
	FANTOM_KEY(6, 5, 53, "A5", )
	FANTOM_KEY(6, 6, 54, "A#5", )
	FANTOM_KEY(6, 7, 55, "B5", )

	PORT_START("KEY7")
	FANTOM_KEY(7, 0, 56, "C6", )
	FANTOM_KEY(7, 1, 57, "C#6", )
	FANTOM_KEY(7, 2, 58, "D6", )
	FANTOM_KEY(7, 3, 59, "D#6", )
	FANTOM_KEY(7, 4, 60, "E6", )
	FANTOM_KEY(7, 5, 61, "F6", )
	FANTOM_KEY(7, 6, 62, "F#6", )
	FANTOM_KEY(7, 7, 63, "G6", )

	PORT_START("KEY8")
	FANTOM_KEY(8, 0, 64, "G#6", )
	FANTOM_KEY(8, 1, 65, "A6", )
	FANTOM_KEY(8, 2, 66, "A#6", )
	FANTOM_KEY(8, 3, 67, "B6", )
	FANTOM_KEY(8, 4, 68, "C7", )
	FANTOM_KEY(8, 5, 69, "C#7", )
	FANTOM_KEY(8, 6, 70, "D7", )
	FANTOM_KEY(8, 7, 71, "D#7", )

	PORT_START("KEY9")
	FANTOM_KEY(9, 0, 72, "E7", )
	FANTOM_KEY(9, 1, 73, "F7", )
	FANTOM_KEY(9, 2, 74, "F#7", )
	FANTOM_KEY(9, 3, 75, "G7", )
	PORT_BIT(0xf0, IP_ACTIVE_HIGH, IPT_UNUSED)

#undef FANTOM_KEY
INPUT_PORTS_END


ROM_START(fantom)
	ROM_REGION16_BE(0x400000, "flash", 0)
	ROM_DEFAULT_BIOS("v120")
	ROM_SYSTEM_BIOS(0, "v120", "Version 1.20")
	ROMX_LOAD("fantom_v1.20.ic5", 0x000000, 0x400000, CRC(318821c0) SHA1(0feaab22c4a7987a25c2cbbb0965072a2c015053), ROM_BIOS(0))
	ROM_SYSTEM_BIOS(1, "v118", "Version 1.18")
	ROMX_LOAD("fantom_v1.18.ic5", 0x000000, 0x400000, CRC(f9fae390) SHA1(86f6924893d1496b491fc124173d349694c30c1b), ROM_BIOS(1))

	ROM_REGION16_LE(0x2000000, "waverom", ROMREGION_ERASE00)
	ROM_LOAD("upd23c128040lgy-849.ic48", 0x0000000, 0x1000000, CRC(528aad5b) SHA1(5ced38ba29f9e6188ac1767933dc2bd9eac172fd))
	ROM_LOAD("upd23c128040lgy-850.ic51", 0x1000000, 0x1000000, CRC(17163f60) SHA1(4f360f09fe4e9185cf4a52df6aeb1e72c477fcff))
ROM_END

} // anonymous namespace


//    YEAR  NAME    PARENT  COMPAT  MACHINE  INPUT   CLASS         INIT          COMPANY   FULLNAME          FLAGS
SYST( 2001, fantom, 0,      0,      fantom,  fantom, fantom_state, init_waverom, "Roland", "Fantom (FA-76)", MACHINE_NOT_WORKING )
