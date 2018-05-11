// license:BSD-3-Clause
// copyright-holders:Valley Bell
/***************************************************************************

  Capcom QSound DL-1425 (HLE)
  ===========================

  Driver by Valley Bell, ported by ctr

  based on disassembled DSP code...

  TODO:
  - structure variables

  Links:
  https://siliconpr0n.org/map/capcom/dl-1425

***************************************************************************/

#include "emu.h"
#include "qsoundhle.h"

// device type definition
DEFINE_DEVICE_TYPE(QSOUND_HLE, qsound_hle_device, "qsound_hle", "QSound (HLE)")

// DSP internal ROM region
ROM_START( qsound_hle )
	ROM_REGION( 0x2000, "dsp", 0 )
	// removing WORD_SWAP from original definition
	ROM_LOAD16_WORD( "dl-1425.bin", 0x0000, 0x2000, CRC(d6cf5ef5) SHA1(555f50fe5cdf127619da7d854c03f4a244a0c501) )
	ROM_IGNORE( 0x4000 )
ROM_END


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  qsound_hle_device - constructor
//-------------------------------------------------

qsound_hle_device::qsound_hle_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, QSOUND_HLE, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
	, device_rom_interface(mconfig, *this, 24)
	, m_stream(nullptr)
	, m_dataLatch(0)
{
}


//-------------------------------------------------
//  rom_bank_updated - the rom bank has changed
//-------------------------------------------------

void qsound_hle_device::rom_bank_updated()
{
	m_stream->update();
}

// ---- routine offsets ----
// The routine to be executed can be selected externally by writing to OFS_ROUTINE.
//
// initialization routines
#define DSPRT_INIT1		0x0288	// default initialization routine
#define DSPRT_INIT2		0x061A	// alternate initialization routine
// refresh filter data, then start update
#define DSPRT_DO_TEST1	0x000C	// test routine - low-frequency sawtooth
#define DSPRT_DO_TEST2	0x000F	// test routine - high-frequency sawtooth
#define DSPRT_DO_UPD1	0x0039
#define DSPRT_DO_UPD2	0x004F
// update routines
#define DSPRT_TEST		0x0018	// test routine, outputs a sawtooth wave, doesn't do any additonal processing
#define DSPRT_UPDATE1	0x0314	// default update routine
#define DSPRT_UPDATE2	0x06B2	// alternate update routine


// ---- RAM offsets ----
#define OFS_CHANNEL_MEM		0x000	// 000-07F, 16 channels, 8 bytes per channel
#define OFS_CH_BANK			0x000
#define OFS_CH_CUR_ADDR		0x001
#define OFS_CH_RATE			0x002
#define OFS_CH_FRAC			0x003
#define OFS_CH_LOOP_LEN		0x004
#define OFS_CH_END_ADDR		0x005
#define OFS_CH_VOLUME		0x006
#define OFS_PAN_POS			0x080	// 080-092, 16+3 channels, offset into DSP ROM pan table for left speaker
#define OFS_ADPCM_PAN_POS	0x090	// 090-092

#define OFS_REVERB_FB		0x093	// Reverb feedback volume
// 094-0B9 unused
#define OFS_CH_REVERB		0x0BA	// 0BA-0C9, 16 channels

#define OFS_ADPCM_START_ADDR 0x0CA	// 0CA-0D5, 3 "channels"
#define OFS_ADPCM_END_ADDR	0x0CB
#define OFS_ADPCM_BANK		0x0CC
#define OFS_ADPCM_VOLUME	0x0CD

#define OFS_ADPCM_FLAG		0x0D6	// 0D6-0D8, 3 channels, non-zero value starts ADPCM playback

#define OFS_REVERB_BUF_END	0x0D9	// subtract with OFS_Mx_DELAY_BUFFER for reverb delay length

#define OFS_L_FIR_POS		0x0DA	// Left filter A select
#define OFS_L_FIR_B_POS		0x0DB	// Left filter B select
#define OFS_R_FIR_POS		0x0DC	// right filter A select
#define OFS_R_FIR_B_POS		0x0DD	// right filter B select

#define OFS_L_WET_DELAY		0x0DE	// Left filtered phase delay
#define OFS_L_DRY_DELAY		0x0DF	// Left raw phase delay
#define OFS_R_WET_DELAY		0x0E0	// Right filtered phase delay
#define OFS_R_DRY_DELAY		0x0E1	// Right raw phase delay

#define OFS_DELAY_REFRESH	0x0E2	// set to 1 to request recalculation of wet/dry delay
#define OFS_ROUTINE			0x0E3	// offset of DSP ROM sample processing routine

#define OFS_L_WET_VOL		0x0E4	// volume for left speaker filtered part
#define OFS_L_DRY_VOL		0x0E5	// volume for left speaker unfiltered part
#define OFS_R_WET_VOL		0x0E6	// volume for right speaker filtered part
#define OFS_R_DRY_VOL		0x0E7	// volume for right speaker unfiltered part

// **** End of Z80 communication registers ***

#define OFS_ADPCM_CUR_VOL	0x0E8	// 0E8-0EA, 3 channels
#define OFS_ADPCM_SIGNAL	0x0EB	// 0EB-0EE, 3 channels
#define OFS_ADPCM_CUR_ADDR	0x0F1	// 0F1-0F3, 3 channels
#define OFS_ADPCM_STEP		0x0F4	// 0F4-0F6, 3 channels

#define OFS_PAN_RPOS		0x0F7	// 0F7-109, 16+3 channels, offset cache for right speaker (== RAM[OFS_PAN_POS]+2*98)
#define OFS_CH_SMPLDATA		0x10A	// 10A-11C, 16+3 channels, caches (sampleData * channelVolume)
#define OFS_ADPCM_SMPLDATA	0x11A	// 11A-11C, 3 channels, noise generator?

#define OFS_RAW_OUT			0x11E	// raw / non filtered output

#define OFS_REVERB_BUF_POS	0x11F

#define OFS_REV_OUTBUF_POS	0x120	// position of some delay buffer for the reverb output
#define OFS_REVERB_OUTPUT	0x121
#define OFS_REVERB_LASTSAMP	0x122	// Last value from the filter buffer, used to smooth the reverb output

#define OFS_L_FIR_BUF_POS	0x123	// ring buffer position
#define OFS_R_FIR_BUF_POS	0x124

#define OFS_L_WET_BUF_W		0x125	// write pointer
#define OFS_L_WET_BUF_R		0x126	// read pointer
#define OFS_L_DRY_BUF_W		0x127	// write pointer
#define OFS_L_DRY_BUF_R		0x128	// read pointer

#define OFS_R_WET_BUF_W		0x129	// write pointer
#define OFS_R_WET_BUF_R		0x12A	// read pointer
#define OFS_R_DRY_BUF_W		0x12B	// write pointer
#define OFS_R_DRY_BUF_R		0x12C	// read pointer

#define OFS_L_WET_BUF		0x12D	// Delay line for left filtered output
#define OFS_L_DRY_BUF		0x160	// Delay line for left unfiltered output
#define OFS_R_WET_BUF		0x193	// Delay line for left filtered output
#define OFS_R_DRY_BUF		0x1C6	// Delay line for left unfiltered output

#define OFS_M1_L_FIR_BUF	0x1F9	// tapped delay line for the left channel FIR filter
#define OFS_M1_R_FIR_BUF	0x257	// tapped delay line for the right channel FIR filter
#define OFS_M1_L_FIR_TBL	0x2B5	// coefficients for the left channel FIR filter
#define OFS_M1_R_FIR_TBL	0x314	// coefficients for the right channel FIR filter
#define OFS_M1_REV_OUTBUF	0x373	// Reverb output buffer. Only written to, never read ?
#define OFS_M1_DELAY_BUFFER	0x554

// for mode 2, the "unfiltered" output actually goes through another filter.
#define OFS_M2_L_FIR_BUF_POS 0x1F9	// ring buffer position,
#define OFS_M2_R_FIR_BUF_POS 0x1FA	// used for the second filter
#define OFS_M2_L_FIR_BUF	0x1FB
#define OFS_M2_R_FIR_BUF	0x227
#define OFS_M2_L_FIR_B_BUF	0x253
#define OFS_M2_R_FIR_B_BUF	0x27E
#define OFS_M2_L_FIR_TBL	0x2A9
#define OFS_M2_L_FIR_B_TBL	0x2D6
#define OFS_M2_R_FIR_TBL	0x302
#define OFS_M2_R_FIR_B_TBL	0x32F
#define OFS_M2_REV_OUTBUF	0x35B
#define OFS_M2_DELAY_BUFFER	0x53C

#define UPDATE_TEST 0
#define UPDATE_1 1
#define UPDATE_2 2
#define UPDATE_STEP 3
#define UPDATE_DELAY 4

// ---- lookup tables ----
#define PANTBL_LEFT_OUTPUT	0
#define PANTBL_LEFT_FILTER	1
#define PANTBL_RIGHT_OUTPUT	2
#define PANTBL_RIGHT_FILTER	3

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void qsound_hle_device::device_start()
{
	m_stream = stream_alloc(0, 2, clock() / 2 / 1248); // DSP program uses 1248 machine cycles per iteration

	// copy tables from the DSP rom
	uint16_t *dsp_rom = (uint16_t*)memregion("dsp")->base();
	memcpy(m_pan_tables, &dsp_rom[0x110], sizeof(m_pan_tables));
	memcpy(m_lut_09d2, &dsp_rom[0x9d2], sizeof(m_lut_09d2));
	memcpy(m_adpcm_shift, &dsp_rom[0x9dc], sizeof(m_adpcm_shift));
	memcpy(m_filter_lut_mode1, &dsp_rom[0xd53], sizeof(m_filter_lut_mode1));
	memcpy(m_filter_lut_mode2, &dsp_rom[0xf73], sizeof(m_filter_lut_mode2));

	// state save
	save_item(NAME(m_ram));
	save_item(NAME(m_testInc));
	save_item(NAME(m_testOut));
	save_item(NAME(m_dataLatch));
	save_item(NAME(m_busyState));
	save_item(NAME(m_out));
}

//-------------------------------------------------
//  rom_region - return a pointer to the device's
//  internal ROM region
//-------------------------------------------------

const tiny_rom_entry *qsound_hle_device::device_rom_region() const
{
	return ROM_NAME( qsound_hle );
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void qsound_hle_device::device_reset()
{
	memset(m_ram, 0x00, sizeof(uint16_t)*0x800);
	m_testInc = 0;
	m_testOut = 0;
	m_dataLatch = 0x0000;
	m_busyState = 1;
	m_out[0] = m_out[1] = 0;
	m_dspRoutine = DSPRT_INIT1;
	m_dspRtStep = 0;
	m_updateFunc = UPDATE_STEP;
	
	dsp_init1();
}


//-------------------------------------------------
//  sound_stream_update - handle a stream update
//-------------------------------------------------

void qsound_hle_device::sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples)
{
	// Clear the buffers
	memset(outputs[0], 0, samples * sizeof(*outputs[0]));
	memset(outputs[1], 0, samples * sizeof(*outputs[1]));

	for (int i = 0; i < samples; i ++)
	{
		dsp_run_update();
		outputs[0][i] = m_out[0];
		outputs[1][i] = m_out[1];
	}
}

WRITE8_MEMBER(qsound_hle_device::qsound_w)
{
	switch (offset)
	{
		case 0:
			m_dataLatch = (m_dataLatch & 0x00ff) | (data << 8);
			break;

		case 1:
			m_dataLatch = (m_dataLatch & 0xff00) | data;
			break;

		case 2:
			m_stream->update();
			write_data(data, m_dataLatch);
			break;

		default:
			logerror("%s: qsound_w %d = %02x\n", machine().describe_context(), offset, data);
			break;
	}
}

READ8_MEMBER(qsound_hle_device::qsound_r)
{
	m_stream->update();
	// ready bit (0x00 = busy, 0x80 == ready)
	return m_busyState ? 0x00 : 0x80;
	//return 0x80;
}

void qsound_hle_device::write_data(uint8_t address, uint16_t data)
{
	m_ram[address] = data;
	m_busyState = 1;
}

/*
	Quick memory map of the QSound ROM. (offsets are words)
	0x0110: Pan tables (98 values per table, though only 64 are used)
		left channel unfiltered volume	- 0x0110
		left channel filtered volume	- 0x0172
		right channel unfiltered volume	- 0x01d4
		right channel filtered volume	- 0x0236
	0x09dc: Unknown table used in filter calculations.
		16 or 24 values, first 8 used for negative indices.
	0x0d53: FIR coefficients for "mode 1" filter
		95 coefficients, 5 filter configurations
	0x0f73: FIR coefficients for "mode 2" filter
*/

int16_t* qsound_hle_device::get_filter_table_1(uint16_t offset)
{
	size_t tblIdx;
	
	if (offset < 0x0D53)
		return NULL;
	offset -= 0x0D53;
	if ((offset % 95) > 0)
		return NULL;
	tblIdx = offset / 95;
	if (tblIdx >= 5)
		return NULL;
	return (int16_t*)&m_filter_lut_mode1[tblIdx];	// return beginning of one of the tables
}

int16_t* qsound_hle_device::get_filter_table_2(uint16_t offset)
{
	if (offset < 0x0F73)
		return NULL;
	offset -= 0x0F73;
	if (offset >= 95)
		return NULL;
	return (int16_t*)&m_filter_lut_mode2[offset];	// return pointer to table data
}

int16_t qsound_hle_device::dsp_get_sample(uint16_t bank, uint16_t ofs)
{
	uint32_t romOfs;
	uint8_t smplData;
	/*
	if (! m_romMask)
		return 0;	// no ROM loaded
	if (! (bank & 0x8000))
		return 0;	// ignore attempts to read from DSP program ROM
	*/
	bank &= 0x7FFF;
	romOfs = (bank << 16) | (ofs << 0);
	
	smplData = read_byte(romOfs);
	
	return (int16_t)((smplData << 8) | (smplData << 0));	// MAME currently expands the 8 bit ROM data to 16 bits this way.
}

void qsound_hle_device::INC_MODULO(uint16_t* reg, uint16_t rb, uint16_t re)
{
	if (*reg == re)
		*reg = rb;
	else
		(*reg) ++;
	return;
}

int32_t qsound_hle_device::DSP_ROUND(int32_t value)
{
	return (value + 0x8000) & ~0xFFFF;
}

void qsound_hle_device::dsp_do_update_step()
{
	do
	{
		switch(m_ram[OFS_ROUTINE])
		{
		case DSPRT_INIT1:
			dsp_init1();
			break;
		case DSPRT_INIT2:
			dsp_init2();
			break;
		case DSPRT_DO_TEST1:	// ROM: 000C
			m_testInc = 0x0001;
			m_ram[OFS_ROUTINE] = DSPRT_TEST;
			break;
		case DSPRT_DO_TEST2:	// ROM: 000F
			m_testInc = 0x0400;
			m_ram[OFS_ROUTINE] = DSPRT_TEST;
			break;
		case DSPRT_DO_UPD1:
			dsp_copy_filter_data_1();
			break;
		case DSPRT_DO_UPD2:
			dsp_copy_filter_data_2();
			break;
		case DSPRT_TEST:
			m_updateFunc = UPDATE_TEST;
			break;
		case DSPRT_UPDATE1:
			m_updateFunc = UPDATE_1;
			break;
		case DSPRT_UPDATE2:
			m_updateFunc = UPDATE_2;
			break;
		default:	// handle invalid routines
			m_ram[OFS_ROUTINE] = DSPRT_INIT1;
			break;
		}
	} while(m_updateFunc == UPDATE_STEP);
	
	m_dspRtStep = 0;
	dsp_run_update();
	return;
}

void qsound_hle_device::dsp_update_delay()
{
	if (m_dspRtStep > 0)
		m_dspRtStep --;
	else
		m_updateFunc = UPDATE_STEP;
	
	return;
}

void qsound_hle_device::dsp_update_test()	// ROM: 0018
{
	if (m_dspRtStep == 0)
	{
		m_testOut += m_testInc;	// saturation is disabled here
		
		// ---- Note: The DSP program processes external commands here. ---
		m_busyState = 0;
	}
	
	m_out[0] = m_testOut;
	m_out[1] = m_testOut;
	
	m_dspRtStep ++;
	if (m_dspRtStep >= 2)
	{
		m_dspRtStep -= 2;
		m_updateFunc = UPDATE_STEP;
	}
	return;
}

void qsound_hle_device::dsp_copy_filter_data_1()	// ROM: 0039
{
	const int16_t* filterTbl;
	
	filterTbl = get_filter_table_1(m_ram[OFS_L_FIR_POS]);
	if (filterTbl != NULL)
		memcpy(&m_ram[OFS_M1_L_FIR_TBL], filterTbl, 95 * sizeof(int16_t));
	
	filterTbl = get_filter_table_1(m_ram[OFS_R_FIR_POS]);
	if (filterTbl != NULL)
		memcpy(&m_ram[OFS_M1_R_FIR_TBL], filterTbl, 95 * sizeof(int16_t));
	
	m_ram[OFS_ROUTINE] = DSPRT_UPDATE1;
	return;
}

void qsound_hle_device::dsp_copy_filter_data_2()	// ROM: 004F
{
	const int16_t* filterTbl;
	
	filterTbl = get_filter_table_2(m_ram[OFS_L_FIR_POS]);
	if (filterTbl != NULL)
		memcpy(&m_ram[OFS_M2_L_FIR_TBL], filterTbl, 45 * sizeof(int16_t));
	filterTbl = get_filter_table_2(m_ram[OFS_L_FIR_B_POS]);
	if (filterTbl != NULL)
		memcpy(&m_ram[OFS_M2_L_FIR_B_TBL], filterTbl, 44 * sizeof(int16_t));
	
	filterTbl = get_filter_table_2(m_ram[OFS_R_FIR_POS]);
	if (filterTbl != NULL)
		memcpy(&m_ram[OFS_M2_R_FIR_TBL], filterTbl, 45 * sizeof(int16_t));
	filterTbl = get_filter_table_2(m_ram[OFS_R_FIR_B_POS]);
	if (filterTbl != NULL)
		memcpy(&m_ram[OFS_M2_R_FIR_B_TBL], filterTbl, 44 * sizeof(int16_t));
	
	m_ram[OFS_ROUTINE] = DSPRT_UPDATE2;
	return;
}

void qsound_hle_device::dsp_init1()	// ROM: 0288
{
	uint8_t curChn;
	
	memset(m_ram, 0x00, sizeof(m_ram));
	m_ram[OFS_ROUTINE] = DSPRT_UPDATE1;
	
	for (curChn = 0; curChn < 19; curChn ++)
		m_ram[OFS_PAN_POS + curChn] = 0x0120;	// pan = centre
	m_ram[OFS_L_FIR_BUF_POS] = OFS_M1_L_FIR_BUF;
	m_ram[OFS_R_FIR_BUF_POS] = OFS_M1_R_FIR_BUF;
	m_ram[OFS_REV_OUTBUF_POS] = OFS_M1_REV_OUTBUF;
	m_ram[OFS_REVERB_BUF_POS] = OFS_M1_DELAY_BUFFER;
	m_ram[OFS_REVERB_BUF_END] = OFS_M1_DELAY_BUFFER + 6;
	m_ram[OFS_L_WET_BUF_W] = OFS_L_WET_BUF;
	m_ram[OFS_L_DRY_BUF_W] = OFS_L_DRY_BUF;
	m_ram[OFS_R_WET_BUF_W] = OFS_R_WET_BUF;
	m_ram[OFS_R_DRY_BUF_W] = OFS_R_DRY_BUF;
	
	m_ram[OFS_L_WET_DELAY] = 0x0000;
	m_ram[OFS_L_DRY_DELAY] = 0x002E;
	m_ram[OFS_R_WET_DELAY] = 0x0000;
	m_ram[OFS_R_DRY_DELAY] = 0x0030;
	dsp_delay_recalc(OFS_DELAY_REFRESH);
	m_ram[OFS_L_WET_VOL] = 0x3FFF;
	m_ram[OFS_L_DRY_VOL] = 0x3FFF;
	m_ram[OFS_R_WET_VOL] = 0x3FFF;
	m_ram[OFS_R_DRY_VOL] = 0x3FFF;
	for (curChn = 0; curChn < 16; curChn ++)
		m_ram[OFS_CH_BANK + curChn * 0x08] = 0x8000;
	
	m_ram[OFS_ADPCM_BANK] = 0x8000;
	m_ram[OFS_ADPCM_BANK+4] = 0x8000;
	m_ram[OFS_ADPCM_BANK+8] = 0x8000;
	m_ram[OFS_L_FIR_POS] = 0x0DB2;
	m_ram[OFS_R_FIR_POS] = 0x0E11;
	
	dsp_copy_filter_data_1();
	m_dspRtStep = 4;
	m_updateFunc = UPDATE_DELAY;	// simulate the DSP being busy for 4 samples
	return;
}

void qsound_hle_device::dsp_init2()	// ROM: 061A
{
	uint8_t curChn;
	
	memset(m_ram, 0x00, sizeof(m_ram));
	m_ram[OFS_ROUTINE] = DSPRT_UPDATE2;
	
	for (curChn = 0; curChn < 19; curChn ++)
		m_ram[OFS_PAN_POS + curChn] = 0x0120;	// pan = centre
	m_ram[OFS_L_FIR_BUF_POS] = OFS_M2_L_FIR_BUF;
	m_ram[OFS_R_FIR_BUF_POS] = OFS_M2_R_FIR_BUF;
	m_ram[OFS_M2_L_FIR_BUF_POS] = OFS_M2_L_FIR_B_BUF;
	m_ram[OFS_M2_R_FIR_BUF_POS] = OFS_M2_R_FIR_B_BUF;
	m_ram[OFS_REV_OUTBUF_POS] = OFS_M2_REV_OUTBUF;
	m_ram[OFS_REVERB_BUF_POS] = OFS_M2_DELAY_BUFFER;
	m_ram[OFS_REVERB_BUF_END] = OFS_M2_DELAY_BUFFER + 6;
	m_ram[OFS_L_WET_BUF_W] = OFS_L_WET_BUF;
	m_ram[OFS_L_DRY_BUF_W] = OFS_L_DRY_BUF;
	m_ram[OFS_R_WET_BUF_W] = OFS_R_WET_BUF;
	m_ram[OFS_R_DRY_BUF_W] = OFS_R_DRY_BUF;
	
	m_ram[OFS_L_WET_DELAY] = 0x0001;
	m_ram[OFS_L_DRY_DELAY] = 0x0000;
	m_ram[OFS_R_WET_DELAY] = 0x0000;
	m_ram[OFS_R_DRY_DELAY] = 0x0000;
	dsp_delay_recalc(OFS_DELAY_REFRESH);
	m_ram[OFS_L_WET_VOL] = 0x3FFF;
	m_ram[OFS_L_DRY_VOL] = 0x3FFF;
	m_ram[OFS_R_WET_VOL] = 0x3FFF;
	m_ram[OFS_R_DRY_VOL] = 0x3FFF;
	for (curChn = 0; curChn < 16; curChn ++)
		m_ram[OFS_CH_BANK + curChn * 0x08] = 0x8000;
	
	m_ram[OFS_ADPCM_BANK] = 0x8000;
	m_ram[OFS_ADPCM_BANK+4] = 0x8000;
	m_ram[OFS_ADPCM_BANK+8] = 0x8000;
	m_ram[OFS_L_FIR_POS] = 0x0F73;
	m_ram[OFS_L_FIR_B_POS] = 0x0FA4;
	m_ram[OFS_R_FIR_POS] = 0x0F73;
	m_ram[OFS_R_FIR_B_POS] = 0x0FA4;
	
	dsp_copy_filter_data_2();
	m_dspRtStep = 4;
	m_updateFunc = UPDATE_DELAY;	// simulate the DSP being busy for 4 samples
	return;
}

void qsound_hle_device::dsp_delay_recalc(uint16_t refreshFlagAddr)	// ROM: 05DD / 099B
{
	// Note: Subroutines 05DD and 099B are identical, except for a varying amount of NOPs at the end of the routine.
	uint8_t curFilter;
	uint16_t temp;
	
	for (curFilter = 0; curFilter < 4; curFilter ++)
	{
		temp = m_ram[OFS_L_WET_BUF_W + curFilter * 2] - m_ram[OFS_L_WET_DELAY + curFilter];
		if (temp < 301 + curFilter * 51)
			temp += 51;
		m_ram[OFS_L_WET_BUF_R + curFilter * 2] = temp;
	}
	
	m_ram[refreshFlagAddr] = 0;
	return;
}

void qsound_hle_device::dsp_update_1()	// ROM: 0314
{
	if (m_dspRtStep < 3)
		dsp_update_adpcm_top(m_dspRtStep - 0);
	else
		dsp_update_adpcm_bottom(m_dspRtStep - 3);
	dsp_sample_calc_1();
	
	m_dspRtStep ++;
	if (m_dspRtStep >= 6)
	{
		m_dspRtStep -= 6;
		m_updateFunc = UPDATE_STEP;
	}
	return;
}

void qsound_hle_device::dsp_update_2()	// ROM: 06B2
{
	// ADPCM processing is identical in both functions
	if (m_dspRtStep < 3)
		dsp_update_adpcm_top(m_dspRtStep - 0);
	else
		dsp_update_adpcm_bottom(m_dspRtStep - 3);
	dsp_sample_calc_2();
	
	m_dspRtStep ++;
	if (m_dspRtStep >= 6)
	{
		m_dspRtStep -= 6;
		m_updateFunc = UPDATE_STEP;
	}
	return;
}

// Handles 4-bit ADPCM. First function for top nibble, second function for bottom.

void qsound_hle_device::dsp_update_adpcm_top(uint16_t i1)	// ROM: 0314/036F/03CA
{
	uint16_t i4 = i1 * 4;
	int16_t x, y, z;
	int32_t a1;
	int32_t p;
	
	if (m_ram[OFS_ADPCM_CUR_ADDR + i1] == m_ram[OFS_ADPCM_END_ADDR + i4])
		m_ram[OFS_ADPCM_CUR_VOL + i1] = 0;
	// ROM 0322/037D/03D8
	if (m_ram[OFS_ADPCM_FLAG + i1] != 0)
	{
		m_ram[OFS_ADPCM_SMPLDATA + i1] = 0;
		m_ram[OFS_ADPCM_FLAG + i1] = 0;
		m_ram[OFS_ADPCM_SIGNAL + i1] = 0x000A;
		m_ram[OFS_ADPCM_CUR_VOL + i1] = m_ram[OFS_ADPCM_VOLUME + i4];
		m_ram[OFS_ADPCM_CUR_ADDR + i1] = m_ram[OFS_ADPCM_START_ADDR + i4];
	}
	// ROM: 0333/038E/03E9
	//m_ram[OFS_ADPCM_CUR_ADDR + i1] = m_ram[OFS_ADPCM_CUR_ADDR + i1];
	x = dsp_get_sample(m_ram[OFS_ADPCM_BANK + i4], m_ram[OFS_ADPCM_CUR_ADDR + i1]);
	m_ram[OFS_ADPCM_STEP + i1] = x >> 12;	// top nibble of x, sign-extended
	p = (int16_t)m_ram[OFS_ADPCM_STEP + i1] * (int16_t)m_ram[OFS_ADPCM_SIGNAL + i1];
	z = (int16_t)m_ram[OFS_ADPCM_SIGNAL + i1] >> 1;
	// ROM 034A/03A5/0400
	if (p <= 0)
		z = -z;
	a1 = m_ram[OFS_ADPCM_SMPLDATA + i1] + p;
	x = m_ram[OFS_ADPCM_CUR_VOL + i1];
	y = a1 + z;
	p = x * y;
	m_ram[OFS_ADPCM_SMPLDATA + i1] = p >> 16;
	// ROM: 0354/03AF/040A
	x = m_adpcm_shift[0x08 + (int16_t)m_ram[OFS_ADPCM_STEP + i1]];
	p = x * (int16_t)m_ram[OFS_ADPCM_SIGNAL + i1];
	y = (p << 10) >> 16;
	if (y <= 1)
		y = 1;
	else if (y >= 2000)
		y = 2000;
	m_ram[OFS_ADPCM_SIGNAL + i1] = y;
	
	return;
}

void qsound_hle_device::dsp_update_adpcm_bottom(uint16_t i1)	// ROM: 0425/0470/04B4
{
	uint16_t i4 = i1 * 4;
	int16_t x, y, z;
	int32_t a1;
	int32_t p;
	
	x = dsp_get_sample(m_ram[OFS_ADPCM_BANK + i4], m_ram[OFS_ADPCM_CUR_ADDR + i1]);
	m_ram[OFS_ADPCM_CUR_ADDR + i1] ++;
	x = (x >> 8) & 0x0F;
	m_ram[OFS_ADPCM_STEP + i1] = x;
	p = x * (int16_t)m_ram[OFS_ADPCM_SIGNAL + i1];
	z = (int16_t)m_ram[OFS_ADPCM_SIGNAL + i1] >> 1;
	// ROM 0447/0492/04DD
	if (p <= 0)
		z = -z;
	a1 = m_ram[OFS_ADPCM_SMPLDATA + i1] + p;
	x = m_ram[OFS_ADPCM_CUR_VOL + i1];
	y = a1 + z;
	p = x * y;
	m_ram[OFS_ADPCM_SMPLDATA + i1] = p >> 16;
	// ROM: 0451/049C/04E7
	x = m_adpcm_shift[0x08 + (int16_t)m_ram[OFS_ADPCM_STEP + i1]];
	p = x * (int16_t)m_ram[OFS_ADPCM_SIGNAL + i1];
	y = (p << 10) >> 16;
	if (y <= 1)
		y = 1;
	else if (y >= 2000)
		y = 2000;
	m_ram[OFS_ADPCM_SIGNAL + i1] = y;
	
	return;
}

void qsound_hle_device::dsp_sample_calc_1()	// ROM: 0504
{
	int16_t x, y;
	int32_t a0, a1;
	int32_t p;
	uint8_t curChn;
	uint16_t curBank;
	uint16_t chnBase;
	uint16_t tmpOfs, tmpOf2;
	
	// ROM: 050A
	curBank = m_ram[0x0078 + OFS_CH_BANK];
	for (curChn = 0, chnBase = OFS_CHANNEL_MEM; curChn < 16; curChn ++, chnBase += 0x08)
	{
		int64_t addr;
		
		p = (int16_t)m_ram[chnBase + OFS_CH_VOLUME] * dsp_get_sample(curBank, m_ram[chnBase + OFS_CH_CUR_ADDR]);
		m_ram[OFS_CH_SMPLDATA + curChn] = p >> 14;	// p*4 >> 16
		
		curBank = m_ram[chnBase + OFS_CH_BANK];
		addr = m_ram[chnBase + OFS_CH_RATE] << 4;
		addr += ((int16_t)m_ram[chnBase + OFS_CH_CUR_ADDR] << 16) | (m_ram[chnBase + OFS_CH_FRAC] << 0);
		if ((addr >> 16) >= (int16_t)m_ram[chnBase + OFS_CH_END_ADDR])
			addr -= (m_ram[chnBase + OFS_CH_LOOP_LEN] << 16);
		// The DSP's a0/a1 registers are 36 bits. The result is clamped when writing it back to RAM.
		if (addr > 0x7FFFFFFFLL)
			addr = 0x7FFFFFFFLL;
		else if (addr < -0x80000000LL)
			addr = -0x80000000LL;
		m_ram[chnBase + OFS_CH_FRAC] = (addr >> 0) & 0xFFFF;
		m_ram[chnBase + OFS_CH_CUR_ADDR] = (addr >> 16) & 0xFFFF;
	}
	
	// ROM: 051E
	a1 = 0;
	for (curChn = 0; curChn < 16; curChn ++)
	{
		p = (int16_t)m_ram[OFS_CH_SMPLDATA + curChn] * (int16_t)m_ram[OFS_CH_REVERB + curChn];
		a1 += (p * 4);
	}
	tmpOfs = m_ram[OFS_REVERB_BUF_POS] & 0x7FF;
	a0 = (int16_t)m_ram[tmpOfs];
	y = m_ram[OFS_REVERB_LASTSAMP];
	m_ram[OFS_REVERB_LASTSAMP] = a0;
	a0 = (a0 + y)>>1; // get the mean from the last two samples in the reverb buffer
	y = a0; // using moving average filter
	x = (int16_t)m_ram[OFS_REVERB_FB]; // actually this is the feedback volume
	m_ram[OFS_REVERB_OUTPUT] = y;
	p = x * y;					// reverb feedback
	a1 += (p * 4);				// sum with the reverb contributions for this sample
	m_ram[tmpOfs] = a1 >> 16;
	INC_MODULO(&tmpOfs, OFS_M1_DELAY_BUFFER, m_ram[OFS_REVERB_BUF_END]);
	m_ram[OFS_REVERB_BUF_POS] = tmpOfs;
	
	// ROM: 0538
	tmpOfs = m_ram[OFS_REV_OUTBUF_POS] & 0x7FF;
	m_ram[tmpOfs] = a0;
	INC_MODULO(&tmpOfs, OFS_M1_REV_OUTBUF, OFS_M1_DELAY_BUFFER - 1);
	m_ram[OFS_REV_OUTBUF_POS] = tmpOfs;
	
	// ---- Note: The DSP program processes external commands here. ---
	m_busyState = 0;
	
	// ---- left channel ----
	// ROM: 055E
	y = 0; p = 0; a0 = 0; a1 = 0;
	for (curChn = 0; curChn < 19; curChn ++)
	{
		uint16_t panPos;
		
		panPos = m_ram[OFS_PAN_POS + curChn];
		a0 -= p*4;
		p = x * y;
		y = m_ram[OFS_CH_SMPLDATA + curChn];
		x = (int16_t)m_pan_tables[PANTBL_LEFT_OUTPUT][panPos - 0x110];
		
		a1 -= p*4;
		p = x * y;
		y = m_ram[OFS_CH_SMPLDATA + curChn];
		x = (int16_t)m_pan_tables[PANTBL_LEFT_FILTER][panPos - 0x110];
		
		m_ram[OFS_PAN_RPOS + curChn] = panPos + 98 * 2;
	}
	a0 -= p*4;	// a0 is the unfiltered sample output
	p = x * y;
	a1 -= p*4;	// a1 is the sample to input to the filter
	a0 += (int16_t)m_ram[OFS_REVERB_OUTPUT] << 16;
	m_ram[OFS_RAW_OUT] = a0 >> 16;
	
	// ROM: 0572
	tmpOf2 = OFS_M1_L_FIR_TBL;
	tmpOfs = m_ram[OFS_L_FIR_BUF_POS] & 0x7FF;
	a0 = 0; p = 0;
	x = (int16_t)m_ram[tmpOf2];	tmpOf2 ++;
	y = (int16_t)m_ram[tmpOfs];	INC_MODULO(&tmpOfs, OFS_M1_L_FIR_BUF, OFS_M1_R_FIR_BUF - 1);
	for (curChn = 0; curChn < 93; curChn ++)
	{
		a0 -= p*4;
		p = x * y;
		x = (int16_t)m_ram[tmpOf2];	tmpOf2 ++;
		y = (int16_t)m_ram[tmpOfs];	INC_MODULO(&tmpOfs, OFS_M1_L_FIR_BUF, OFS_M1_R_FIR_BUF - 1);
	}
	a0 -= p*4;
	p = x * y;
	x = (int16_t)m_ram[tmpOf2];	tmpOf2 ++;
	y = a1 >> 16;	// the current sample
	a0 -= p*4;
	p = x * y;
	m_ram[tmpOfs] = y;	INC_MODULO(&tmpOfs, OFS_M1_L_FIR_BUF, OFS_M1_R_FIR_BUF - 1);
	m_ram[OFS_L_FIR_BUF_POS] = tmpOfs;
	
	// ROM: 0582
	a0 -= p*4;
	x = (int16_t)m_ram[OFS_L_WET_VOL];
	
	// insert the filtered sample into a delay line
	tmpOfs = m_ram[OFS_L_WET_BUF_W] & 0x7FF;
	m_ram[tmpOfs] = a0 >> 16;	INC_MODULO(&tmpOfs, OFS_L_WET_BUF, OFS_L_DRY_BUF - 1);
	m_ram[OFS_L_WET_BUF_W] = tmpOfs;
	
	tmpOfs = m_ram[OFS_L_WET_BUF_R] & 0x7FF;
	y = (int16_t)m_ram[tmpOfs];	INC_MODULO(&tmpOfs, OFS_L_WET_BUF, OFS_L_DRY_BUF - 1);
	p = x * y;
	x = (int16_t)m_ram[OFS_L_DRY_VOL];
	a0 = p*4;
	y = (int16_t)m_ram[OFS_RAW_OUT];
	m_ram[OFS_L_WET_BUF_R] = tmpOfs;
	
	tmpOfs = m_ram[OFS_L_DRY_BUF_W] & 0x7FF;
	m_ram[tmpOfs] = y;	INC_MODULO(&tmpOfs, OFS_L_DRY_BUF, OFS_R_WET_BUF - 1);
	m_ram[OFS_L_DRY_BUF_W] = tmpOfs;
	
	tmpOfs = m_ram[OFS_L_DRY_BUF_R] & 0x7FF;
	y = m_ram[tmpOfs];	INC_MODULO(&tmpOfs, OFS_L_DRY_BUF, OFS_R_WET_BUF - 1);
	m_ram[OFS_L_DRY_BUF_R] = tmpOfs;
	p = x * y;
	y = (int16_t)m_ram[tmpOfs];
	x = (int16_t)m_lut_09d2[0];
	a0 += p*4;
	p = x * y;
	a0 = DSP_ROUND(a0);
	m_out[0] = a0 >> 16;
	
	// ---- right channel ----
	// ROM: 059D
	y = 0; p = 0; a0 = 0; a1 = 0;
	for (curChn = 0; curChn < 19; curChn ++)
	{
		uint16_t panPos;
		
		//panPos = m_ram[OFS_PAN_POS + curChn] + 98 * 2;
		panPos = m_ram[OFS_PAN_RPOS + curChn];
		a0 -= p*4;
		p = x * y;
		y = (int16_t)m_ram[OFS_CH_SMPLDATA + curChn];
		x = (int16_t)m_pan_tables[PANTBL_RIGHT_OUTPUT][panPos - 0x110 - 98 * 2];
		
		a1 -= p*4;
		p = x * y;
		y = (int16_t)m_ram[OFS_CH_SMPLDATA + curChn];
		x = (int16_t)m_pan_tables[PANTBL_RIGHT_FILTER][panPos - 0x110 - 98 * 2];
	}
	a0 -= p*4;
	p = x * y;
	a1 -= p*4;
	a1 += (int16_t)m_ram[OFS_REVERB_OUTPUT] << 16;
	m_ram[OFS_RAW_OUT] = a0 >> 16;
	
	// ROM: 05A9
	tmpOf2 = OFS_M1_R_FIR_TBL;
	tmpOfs = m_ram[OFS_R_FIR_BUF_POS] & 0x7FF;
	a0 = 0; p = 0;
	x = (int16_t)m_ram[tmpOf2];	tmpOf2 ++;
	y = (int16_t)m_ram[tmpOfs];	INC_MODULO(&tmpOfs, OFS_M1_R_FIR_BUF, OFS_M1_L_FIR_TBL - 1);
	for (curChn = 0; curChn < 93; curChn ++)
	{
		a0 -= p*4;
		p = x * y;
		x = (int16_t)m_ram[tmpOf2];	tmpOf2 ++;
		y = (int16_t)m_ram[tmpOfs];	INC_MODULO(&tmpOfs, OFS_M1_R_FIR_BUF, OFS_M1_L_FIR_TBL - 1);
	}
	a0 -= p*4;
	p = x * y;
	x = (int16_t)m_ram[tmpOf2];	tmpOf2 ++;
	y = a1 >> 16;
	a0 -= p*4;
	p = x * y;
	m_ram[tmpOfs] = y;	INC_MODULO(&tmpOfs, OFS_M1_R_FIR_BUF, OFS_M1_L_FIR_TBL - 1);
	m_ram[OFS_R_FIR_BUF_POS] = tmpOfs;
	
	// ROM: 05BA
	a0 -= p*4;
	x = (int16_t)m_ram[OFS_R_WET_VOL];
	tmpOfs = m_ram[OFS_R_WET_BUF_W] & 0x7FF;
	m_ram[tmpOfs] = a0 >> 16;	INC_MODULO(&tmpOfs, OFS_R_WET_BUF, OFS_R_DRY_BUF - 1);
	m_ram[OFS_R_WET_BUF_W] = tmpOfs;
	
	tmpOfs = m_ram[OFS_R_WET_BUF_R] & 0x7FF;
	y = (int16_t)m_ram[tmpOfs];	INC_MODULO(&tmpOfs, OFS_R_WET_BUF, OFS_R_DRY_BUF - 1);
	p = x * y;
	x = (int16_t)m_ram[OFS_R_DRY_VOL];
	a0 = p*4;
	y = (int16_t)m_ram[OFS_RAW_OUT];
	m_ram[OFS_R_WET_BUF_R] = tmpOfs;
	
	tmpOfs = m_ram[OFS_R_DRY_BUF_W] & 0x7FF;
	m_ram[tmpOfs] = y;	INC_MODULO(&tmpOfs, OFS_R_DRY_BUF, OFS_M1_L_FIR_BUF - 1);
	m_ram[OFS_R_DRY_BUF_W] = tmpOfs;
	
	tmpOfs = m_ram[OFS_R_DRY_BUF_R] & 0x7FF;
	y = m_ram[tmpOfs];	INC_MODULO(&tmpOfs, OFS_R_DRY_BUF, OFS_M1_L_FIR_BUF - 1);
	m_ram[OFS_R_DRY_BUF_R] = tmpOfs;
	p = x * y;
	y = (int16_t)m_ram[tmpOfs];
	x = (int16_t)m_lut_09d2[0];
	a0 += p*4;
	p = x * y;
	a0 = DSP_ROUND(a0);
	m_out[1] = a0 >> 16;
	
	if (m_ram[OFS_DELAY_REFRESH])
		dsp_delay_recalc(OFS_DELAY_REFRESH);
	
	return;
}

void qsound_hle_device::dsp_sample_calc_2()	// ROM: 08A2
{
	int16_t x, y;
	int32_t a0, a1;
	int32_t p;
	uint8_t curChn;
	uint16_t curBank;
	uint16_t chnBase;
	uint16_t tmpOfs, tmpOf2;
	
	// ROM: 08A8
	curBank = m_ram[0x0078 + OFS_CH_BANK];
	for (curChn = 0, chnBase = OFS_CHANNEL_MEM; curChn < 16; curChn ++, chnBase += 0x08)
	{
		int64_t addr;
		
		p = (int16_t)m_ram[chnBase + OFS_CH_VOLUME] * dsp_get_sample(curBank, m_ram[chnBase + OFS_CH_CUR_ADDR]);
		m_ram[OFS_CH_SMPLDATA + curChn] = p >> 14;	// p*4 >> 16
		
		curBank = m_ram[chnBase + OFS_CH_BANK];
		addr = m_ram[chnBase + OFS_CH_RATE] << 4;
		addr += ((int16_t)m_ram[chnBase + OFS_CH_CUR_ADDR] << 16) | (m_ram[chnBase + OFS_CH_FRAC] << 0);
		if ((addr >> 16) >= (int16_t)m_ram[chnBase + OFS_CH_END_ADDR])
			addr -= (m_ram[chnBase + OFS_CH_LOOP_LEN] << 16);
		// The DSP's a0/a1 registers are 36 bits. The result is clamped when writing it back to RAM.
		if (addr > 0x7FFFFFFFLL)
			addr = 0x7FFFFFFFLL;
		else if (addr < -0x80000000LL)
			addr = -0x80000000LL;
		m_ram[chnBase + OFS_CH_FRAC] = (addr >> 0) & 0xFFFF;
		m_ram[chnBase + OFS_CH_CUR_ADDR] = (addr >> 16) & 0xFFFF;
	}
	
	// ROM: 08BC
	a1 = 0;
	for (curChn = 0; curChn < 16; curChn ++)
	{
		p = (int16_t)m_ram[OFS_CH_SMPLDATA + curChn] * (int16_t)m_ram[OFS_CH_REVERB + curChn];
		a1 += (p * 4);
	}
	tmpOfs = m_ram[OFS_REVERB_BUF_POS] & 0x7FF;
	a0 = (int16_t)m_ram[tmpOfs];
	y = m_ram[OFS_REVERB_LASTSAMP];
	m_ram[OFS_REVERB_LASTSAMP] = a0;
	a0 = (a0 + y)>>1; // get the mean from the last two samples in the reverb buffer
	y = a0;
	x = (int16_t)m_ram[OFS_REVERB_FB];
	m_ram[OFS_REVERB_OUTPUT] = y;
	p = x * y;					// reverb feedback
	a1 += (p * 4);				// sum with the reverb contributions for this sample
	m_ram[tmpOfs] = a1 >> 16;
	INC_MODULO(&tmpOfs, OFS_M2_DELAY_BUFFER, m_ram[OFS_REVERB_BUF_END]);
	m_ram[OFS_REVERB_BUF_POS] = tmpOfs;
	
	// ROM: 08D6
	tmpOfs = m_ram[OFS_REV_OUTBUF_POS] & 0x7FF;
	m_ram[tmpOfs] = a0;
	INC_MODULO(&tmpOfs, OFS_M2_REV_OUTBUF, OFS_M2_DELAY_BUFFER - 1);
	m_ram[OFS_REV_OUTBUF_POS] = tmpOfs;
	
	// ---- Note: The DSP program processes external commands here. ---
	m_busyState = 0;
	
	// ---- left channel ----
	// ROM: 08FC
	y = 0; p = 0; a0 = 0; a1 = 0;
	for (curChn = 0; curChn < 19; curChn ++)
	{
		uint16_t panPos;
		
		panPos = m_ram[OFS_PAN_POS + curChn];
		a0 -= p*4;
		p = x * y;
		y = m_ram[OFS_CH_SMPLDATA + curChn];
		x = (int16_t)m_pan_tables[PANTBL_LEFT_OUTPUT][panPos - 0x110];
		
		a1 -= p*4;
		p = x * y;
		y = m_ram[OFS_CH_SMPLDATA + curChn];
		x = (int16_t)m_pan_tables[PANTBL_LEFT_FILTER][panPos - 0x110];
		
		m_ram[OFS_PAN_RPOS + curChn] = panPos + 98 * 2;
	}
	a0 -= p*4;
	p = x * y;
	a1 -= p*4;
	a0 += (int16_t)m_ram[OFS_REVERB_OUTPUT] << 16;
	m_ram[OFS_RAW_OUT] = a0 >> 16;
	
	// ROM: 0910
	tmpOf2 = OFS_M2_L_FIR_TBL;
	tmpOfs = m_ram[OFS_L_FIR_BUF_POS] & 0x7FF;
	a0 = 0; p = 0;
	x = (int16_t)m_ram[tmpOf2];	tmpOf2 ++;
	y = (int16_t)m_ram[tmpOfs];	INC_MODULO(&tmpOfs, OFS_M2_L_FIR_BUF, OFS_M2_R_FIR_BUF - 1);
	for (curChn = 0; curChn < 43; curChn ++)
	{
		a0 -= p*4;
		p = x * y;
		x = (int16_t)m_ram[tmpOf2];	tmpOf2 ++;
		y = (int16_t)m_ram[tmpOfs];	INC_MODULO(&tmpOfs, OFS_M2_L_FIR_BUF, OFS_M2_R_FIR_BUF - 1);
	}
	a0 -= p*4;
	p = x * y;
	x = (int16_t)m_ram[tmpOf2];	tmpOf2 ++;
	y = a1 >> 16;
	a0 -= p*4;
	p = x * y;
	a0 -= p*4;
	m_ram[tmpOfs] = y;	INC_MODULO(&tmpOfs, OFS_M2_L_FIR_BUF, OFS_M2_R_FIR_BUF - 1);
	m_ram[OFS_L_FIR_BUF_POS] = tmpOfs;
	
	// ROM: 0921
	tmpOfs = m_ram[OFS_M2_L_FIR_BUF_POS] & 0x7FF;
	a1 = 0; p = 0;
	x = (int16_t)m_ram[tmpOf2];	tmpOf2 ++;
	y = (int16_t)m_ram[tmpOfs];	INC_MODULO(&tmpOfs, OFS_M2_L_FIR_B_BUF, OFS_M2_R_FIR_B_BUF - 1);
	for (curChn = 0; curChn < 42; curChn ++)
	{
		a1 -= p*4;
		p = x * y;
		x = (int16_t)m_ram[tmpOf2];	tmpOf2 ++;
		y = (int16_t)m_ram[tmpOfs];	INC_MODULO(&tmpOfs, OFS_M2_L_FIR_B_BUF, OFS_M2_R_FIR_B_BUF - 1);
	}
	a1 -= p*4;
	p = x * y;
	x = (int16_t)m_ram[tmpOf2];	tmpOf2 ++;
	y = (int16_t)m_ram[OFS_RAW_OUT];
	a1 -= p*4;
	p = x * y;
	m_ram[tmpOfs] = y;	INC_MODULO(&tmpOfs, OFS_M2_L_FIR_B_BUF, OFS_M2_R_FIR_B_BUF - 1);
	m_ram[OFS_M2_L_FIR_BUF_POS] = tmpOfs;
	
	// ROM: 0930
	a1 -= p*4;
	x = (int16_t)m_ram[OFS_L_WET_VOL];
	tmpOfs = m_ram[OFS_L_WET_BUF_W] & 0x7FF;
	m_ram[tmpOfs] = a0 >> 16;	INC_MODULO(&tmpOfs, OFS_L_WET_BUF, OFS_L_DRY_BUF - 1);
	m_ram[OFS_L_WET_BUF_W] = tmpOfs;
	
	tmpOfs = m_ram[OFS_L_WET_BUF_R] & 0x7FF;
	y = (int16_t)m_ram[tmpOfs];	INC_MODULO(&tmpOfs, OFS_L_WET_BUF, OFS_L_DRY_BUF - 1);
	p = x * y;
	x = (int16_t)m_ram[OFS_L_DRY_VOL];
	a0 = p*4;
	y = (int16_t)m_ram[OFS_RAW_OUT];
	m_ram[OFS_L_WET_BUF_R] = tmpOfs;
	
	tmpOfs = m_ram[OFS_L_DRY_BUF_W] & 0x7FF;
	m_ram[tmpOfs] = a1 >> 16;	INC_MODULO(&tmpOfs, OFS_L_DRY_BUF, OFS_R_WET_BUF - 1);
	m_ram[OFS_L_DRY_BUF_W] = tmpOfs;
	
	tmpOfs = m_ram[OFS_L_DRY_BUF_R] & 0x7FF;
	y = m_ram[tmpOfs];	INC_MODULO(&tmpOfs, OFS_L_DRY_BUF, OFS_R_WET_BUF - 1);
	m_ram[OFS_L_DRY_BUF_R] = tmpOfs;
	p = x * y;
	y = (int16_t)m_ram[tmpOfs];
	x = (int16_t)m_lut_09d2[0];
	a0 += p*4;
	p = x * y;
	a0 = DSP_ROUND(a0);
	m_out[0] = a0 >> 16;
	
	// ---- right channel ----
	// ROM: 094B
	y = 0; p = 0; a0 = 0; a1 = 0;
	for (curChn = 0; curChn < 19; curChn ++)
	{
		uint16_t panPos;
		
		//panPos = m_ram[OFS_PAN_POS + curChn] + 98 * 2;
		panPos = m_ram[OFS_PAN_RPOS + curChn];
		a0 -= p*4;
		p = x * y;
		y = (int16_t)m_ram[OFS_CH_SMPLDATA + curChn];
		x = (int16_t)m_pan_tables[PANTBL_RIGHT_OUTPUT][panPos - 0x110 - 98 * 2];
		
		a1 -= p*4;
		p = x * y;
		y = (int16_t)m_ram[OFS_CH_SMPLDATA + curChn];
		x = (int16_t)m_pan_tables[PANTBL_RIGHT_FILTER][panPos - 0x110 - 98 * 2];
	}
	a0 -= p*4;
	p = x * y;
	a1 -= p*4;
	a1 += (int16_t)m_ram[OFS_REVERB_OUTPUT] << 16;
	m_ram[OFS_RAW_OUT] = a0 >> 16;
	
	// ROM: 0957
	tmpOf2 = OFS_M2_R_FIR_TBL;
	tmpOfs = m_ram[OFS_R_FIR_BUF_POS] & 0x7FF;
	a0 = 0; p = 0;
	x = (int16_t)m_ram[tmpOf2];	tmpOf2 ++;
	y = (int16_t)m_ram[tmpOfs];	INC_MODULO(&tmpOfs, OFS_M2_R_FIR_BUF, OFS_M2_L_FIR_B_BUF - 1);
	for (curChn = 0; curChn < 93; curChn ++)
	{
		a0 -= p*4;
		p = x * y;
		x = (int16_t)m_ram[tmpOf2];	tmpOf2 ++;
		y = (int16_t)m_ram[tmpOfs];	INC_MODULO(&tmpOfs, OFS_M2_R_FIR_BUF, OFS_M2_L_FIR_B_BUF - 1);
	}
	a0 -= p*4;
	p = x * y;
	x = (int16_t)m_ram[tmpOf2];	tmpOf2 ++;
	y = a1 >> 16;
	a0 -= p*4;
	p = x * y;
	a0 -= p*4;
	m_ram[tmpOfs] = y;	INC_MODULO(&tmpOfs, OFS_M2_R_FIR_BUF, OFS_M2_L_FIR_B_BUF - 1);
	m_ram[OFS_R_FIR_BUF_POS] = tmpOfs;
	
	// ROM: 0969
	tmpOfs = m_ram[OFS_M2_R_FIR_BUF_POS] & 0x7FF;
	a1 = 0; p = 0;
	x = (int16_t)m_ram[tmpOf2];	tmpOf2 ++;
	y = (int16_t)m_ram[tmpOfs];	INC_MODULO(&tmpOfs, OFS_M2_R_FIR_B_BUF, 0x02A9 - 1);
	for (curChn = 0; curChn < 42; curChn ++)
	{
		a1 -= p*4;
		p = x * y;
		x = (int16_t)m_ram[tmpOf2];	tmpOf2 ++;
		y = (int16_t)m_ram[tmpOfs];	INC_MODULO(&tmpOfs, OFS_M2_R_FIR_B_BUF, 0x02A9 - 1);
	}
	a1 -= p*4;
	p = x * y;
	x = (int16_t)m_ram[tmpOf2];	tmpOf2 ++;
	y = (int16_t)m_ram[OFS_RAW_OUT];
	a1 -= p*4;
	p = x * y;
	m_ram[tmpOfs] = y;	INC_MODULO(&tmpOfs, OFS_M2_R_FIR_B_BUF, 0x02A9 - 1);
	m_ram[OFS_M2_R_FIR_BUF_POS] = tmpOfs;
	
	// ROM: 0978
	a1 -= p*4;
	x = (int16_t)m_ram[OFS_R_WET_VOL];
	tmpOfs = m_ram[OFS_R_WET_BUF_W] & 0x7FF;
	m_ram[tmpOfs] = a0 >> 16;	INC_MODULO(&tmpOfs, OFS_R_WET_BUF, OFS_R_DRY_BUF - 1);
	m_ram[OFS_R_WET_BUF_W] = tmpOfs;
	
	tmpOfs = m_ram[OFS_R_WET_BUF_R] & 0x7FF;
	y = (int16_t)m_ram[tmpOfs];	INC_MODULO(&tmpOfs, OFS_R_WET_BUF, OFS_R_DRY_BUF - 1);
	p = x * y;
	x = (int16_t)m_ram[OFS_R_DRY_VOL];
	a0 = p*4;
	y = (int16_t)m_ram[OFS_RAW_OUT];
	m_ram[OFS_R_WET_BUF_R] = tmpOfs;
	
	tmpOfs = m_ram[OFS_R_DRY_BUF_W] & 0x7FF;
	m_ram[tmpOfs] = a1 >> 16;	INC_MODULO(&tmpOfs, OFS_R_DRY_BUF, OFS_M1_L_FIR_BUF - 1);
	m_ram[OFS_R_DRY_BUF_W] = tmpOfs;
	
	tmpOfs = m_ram[OFS_R_DRY_BUF_R] & 0x7FF;
	y = m_ram[tmpOfs];	INC_MODULO(&tmpOfs, OFS_R_DRY_BUF, OFS_M1_L_FIR_BUF - 1);
	m_ram[OFS_R_DRY_BUF_R] = tmpOfs;
	p = x * y;
	y = (int16_t)m_ram[tmpOfs];
	x = (int16_t)m_lut_09d2[0];
	a0 += p*4;
	p = x * y;
	a0 = DSP_ROUND(a0);
	m_out[1] = a0 >> 16;
	
	if (m_ram[OFS_DELAY_REFRESH])
		dsp_delay_recalc(OFS_DELAY_REFRESH);
	
	return;
}

void qsound_hle_device::dsp_run_update()
{
	switch(m_updateFunc)
	{
		case UPDATE_TEST:
			return dsp_update_test();
		case UPDATE_1:
			return dsp_update_1();
		case UPDATE_2:
			return dsp_update_2();
		case UPDATE_STEP:
			return dsp_do_update_step();
		case UPDATE_DELAY:
			return dsp_update_delay();
	}
}
