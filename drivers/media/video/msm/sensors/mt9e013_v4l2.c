/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "msm_sensor.h"
#define SENSOR_NAME "mt9e013"
#define PLATFORM_DRIVER_NAME "msm_camera_mt9e013"
#define mt9e013_obj mt9e013_##obj

#define LITEON  0
#define CRESYN  1
#define SUNNY   2
#define SUNNY1  3
#define LITEON1 4
static int vendor;
static int is_otp;

DEFINE_MUTEX(mt9e013_mut);
static struct msm_sensor_ctrl_t mt9e013_s_ctrl;

static struct msm_camera_i2c_reg_conf mt9e013_groupon_settings[] = {
	{0x0104, 0x01},
};

static struct msm_camera_i2c_reg_conf mt9e013_groupoff_settings[] = {
	{0x0104, 0x00},
};

static struct msm_camera_i2c_reg_conf mt9e013_prev_settings[] = {
	/*Output Size (1632x1224) */
	{0x0344, 0x0008},	/*X_ADDR_START */
	{0x0348, 0x0CC9},	/*X_ADDR_END */
	{0x0346, 0x0008},	/*Y_ADDR_START */
	{0x034A, 0x0999},	/*Y_ADDR_END */
	{0x034C, 0x0660},	/*X_OUTPUT_SIZE */
	{0x034E, 0x04C8},	/*Y_OUTPUT_SIZE */
	{0x306E, 0xFCB0},	/*DATAPATH_SELECT */
	{0x3040, 0x04C3},	/*READ_MODE */
	{0x3178, 0x0000},	/*ANALOG_CONTROL5 */
	{0x3ED0, 0x1E24},	/*DAC_LD_4_5 */
	{0x0400, 0x0002},	/*SCALING_MODE */
	{0x0404, 0x0010},	/*SCALE_M */
	/*Timing configuration */
	{0x0342, 0x1018},	/*LINE_LENGTH_PCK */
	{0x0340, 0x055B},	/*FRAME_LENGTH_LINES */
	{0x0202, 0x0557},	/*COARSE_INTEGRATION_TIME */
	{0x3014, 0x0846},	/*FINE_INTEGRATION_TIME_ */
	{0x3010, 0x0130},	/*FINE_CORRECTION */
};

static struct msm_camera_i2c_reg_conf mt9e013_snap_settings[] = {
	/*Output Size (3264x2448) */
	{0x0344, 0x0000},	/*X_ADDR_START */
	{0x0348, 0x0CCF},	/*X_ADDR_END */
	{0x0346, 0x0000},	/*Y_ADDR_START */
	{0x034A, 0x099F},	/*Y_ADDR_END */
	{0x034C, 0x0CD0},	/*X_OUTPUT_SIZE */
	{0x034E, 0x09A0},	/*Y_OUTPUT_SIZE */
	{0x306E, 0xFC80},	/*DATAPATH_SELECT */
	{0x3040, 0x0041},	/*READ_MODE */
	{0x3178, 0x0000},	/*ANALOG_CONTROL5 */
	{0x3ED0, 0x1E24},	/*DAC_LD_4_5 */
	{0x0400, 0x0000},	/*SCALING_MODE */
	{0x0404, 0x0010},	/*SCALE_M */
	/*Timing configuration */
	{0x0342, 0x13F8},	/*LINE_LENGTH_PCK */
	{0x0340, 0x0A2F},	/*FRAME_LENGTH_LINES */
	{0x0202, 0x0A1F},	/*COARSE_INTEGRATION_TIME */
	{0x3014, 0x03F6},	/*FINE_INTEGRATION_TIME_ */
	{0x3010, 0x0078},	/*FINE_CORRECTION */
};

static struct msm_camera_i2c_reg_conf mt9e013_hfr60_settings[] = {
	{0x0300, 0x0005},	/*VT_PIX_CLK_DIV */
	{0x0302, 0x0001},	/*VT_SYS_CLK_DIV */
	{0x0304, 0x0002},	/*PRE_PLL_CLK_DIV */
	{0x0306, 0x0029},	/*PLL_MULTIPLIER */
	{0x0308, 0x000A},	/*OP_PIX_CLK_DIV */
	{0x030A, 0x0001},	/*OP_SYS_CLK_DIV */
	{0x0344, 0x0008},	/*X_ADDR_START */
	{0x0348, 0x0685},	/*X_ADDR_END */
	{0x0346, 0x013a},	/*Y_ADDR_START */
	{0x034A, 0x055B},	/*Y_ADDR_END */
	{0x034C, 0x0340},	/*X_OUTPUT_SIZE */
	{0x034E, 0x0212},	/*Y_OUTPUT_SIZE */
	{0x306E, 0xFC80},	/*DATAPATH_SELECT */
	{0x3040, 0x00C3},	/*READ_MODE */
	{0x3178, 0x0000},	/*ANALOG_CONTROL5 */
	{0x3ED0, 0x1E24},	/*DAC_LD_4_5 */
	{0x0400, 0x0000},	/*SCALING_MODE */
	{0x0404, 0x0010},	/*SCALE_M */
	/*Timing configuration */
	{0x0342, 0x0970},	/*LINE_LENGTH_PCK */
	{0x0340, 0x02A1},	/*FRAME_LENGTH_LINES */
	{0x0202, 0x02A1},	/*COARSE_INTEGRATION_TIME */
	{0x3014, 0x03F6},	/*FINE_INTEGRATION_TIME_ */
	{0x3010, 0x0078},	/*FINE_CORRECTION */
};

static struct msm_camera_i2c_reg_conf mt9e013_hfr90_settings[] = {
	{0x0300, 0x0005},	/*VT_PIX_CLK_DIV */
	{0x0302, 0x0001},	/*VT_SYS_CLK_DIV */
	{0x0304, 0x0002},	/*PRE_PLL_CLK_DIV */
	{0x0306, 0x003D},	/*PLL_MULTIPLIER */
	{0x0308, 0x000A},	/*OP_PIX_CLK_DIV */
	{0x030A, 0x0001},	/*OP_SYS_CLK_DIV */
	{0x0344, 0x0008},	/*X_ADDR_START */
	{0x0348, 0x0685},	/*X_ADDR_END */
	{0x0346, 0x013a},	/*Y_ADDR_START */
	{0x034A, 0x055B},	/*Y_ADDR_END */
	{0x034C, 0x0340},	/*X_OUTPUT_SIZE */
	{0x034E, 0x0212},	/*Y_OUTPUT_SIZE */
	{0x306E, 0xFC80},	/*DATAPATH_SELECT */
	{0x3040, 0x00C3},	/*READ_MODE */
	{0x3178, 0x0000},	/*ANALOG_CONTROL5 */
	{0x3ED0, 0x1E24},	/*DAC_LD_4_5 */
	{0x0400, 0x0000},	/*SCALING_MODE */
	{0x0404, 0x0010},	/*SCALE_M */
	/*Timing configuration */
	{0x0342, 0x0970},	/*LINE_LENGTH_PCK */
	{0x0340, 0x02A1},	/*FRAME_LENGTH_LINES */
	{0x0202, 0x02A1},	/*COARSE_INTEGRATION_TIME */
	{0x3014, 0x03F6},	/*FINE_INTEGRATION_TIME_ */
	{0x3010, 0x0078},	/*FINE_CORRECTION */
};

static struct msm_camera_i2c_reg_conf mt9e013_hfr120_settings[] = {
	{0x0300, 0x0005},	/*VT_PIX_CLK_DIV */
	{0x0302, 0x0001},	/*VT_SYS_CLK_DIV */
	{0x0304, 0x0002},	/*PRE_PLL_CLK_DIV */
	{0x0306, 0x0052},	/*PLL_MULTIPLIER */
	{0x0308, 0x000A},	/*OP_PIX_CLK_DIV */
	{0x030A, 0x0001},	/*OP_SYS_CLK_DIV */
	{0x0344, 0x0008},	/*X_ADDR_START */
	{0x0348, 0x0685},	/*X_ADDR_END */
	{0x0346, 0x013a},	/*Y_ADDR_START */
	{0x034A, 0x055B},	/*Y_ADDR_END */
	{0x034C, 0x0340},	/*X_OUTPUT_SIZE */
	{0x034E, 0x0212},	/*Y_OUTPUT_SIZE */
	{0x306E, 0xFC80},	/*DATAPATH_SELECT */
	{0x3040, 0x00C3},	/*READ_MODE */
	{0x3178, 0x0000},	/*ANALOG_CONTROL5 */
	{0x3ED0, 0x1E24},	/*DAC_LD_4_5 */
	{0x0400, 0x0000},	/*SCALING_MODE */
	{0x0404, 0x0010},	/*SCALE_M */
	/*Timing configuration */
	{0x0342, 0x0970},	/*LINE_LENGTH_PCK */
	{0x0340, 0x02A1},	/*FRAME_LENGTH_LINES */
	{0x0202, 0x02A1},	/*COARSE_INTEGRATION_TIME */
	{0x3014, 0x03F6},	/*FINE_INTEGRATION_TIME_ */
	{0x3010, 0x0078},	/*FINE_CORRECTION */
};

static struct msm_camera_i2c_reg_conf mt9e013_recommend_settings[] = {
	/*Disable embedded data */

	{0x3064, 0x7800},	/*SMIA_TEST */
	/*configure 2-lane MIPI */
	{0x31AE, 0x0202},	/*SERIAL_FORMAT */
	{0x31B8, 0x0E3F},	/*MIPI_TIMING_2 */
	/*set data to RAW10 format */
	{0x0112, 0x0A0A},	/*CCP_DATA_FORMAT */
	{0x30F0, 0x8000},	/*VCM CONTROL */

	{0x3044, 0x0590},
	{0x306E, 0xFC80},
	{0x30B2, 0xC000},
	{0x30D6, 0x0800},
	{0x316C, 0xB42F},
	{0x316E, 0x869A},
	{0x3170, 0x210E},
	{0x317A, 0x010E},
	{0x31E0, 0x1FB9},
	{0x31E6, 0x07FC},
	{0x37C0, 0x0000},
	{0x37C2, 0x0000},
	{0x37C4, 0x0000},
	{0x37C6, 0x0000},
	{0x3E00, 0x0011},
	{0x3E02, 0x8801},
	{0x3E04, 0x2801},
	{0x3E06, 0x8449},
	{0x3E08, 0x6841},
	{0x3E0A, 0x400C},
	{0x3E0C, 0x1001},
	{0x3E0E, 0x2603},
	{0x3E10, 0x4B41},
	{0x3E12, 0x4B24},
	{0x3E14, 0xA3CF},
	{0x3E16, 0x8802},
	{0x3E18, 0x84FF},
	{0x3E1A, 0x8601},
	{0x3E1C, 0x8401},
	{0x3E1E, 0x840A},
	{0x3E20, 0xFF00},
	{0x3E22, 0x8401},
	{0x3E24, 0x00FF},
	{0x3E26, 0x0088},
	{0x3E28, 0x2E8A},
	{0x3E30, 0x0000},
	{0x3E32, 0x8801},
	{0x3E34, 0x4029},
	{0x3E36, 0x00FF},
	{0x3E38, 0x8469},
	{0x3E3A, 0x00FF},
	{0x3E3C, 0x2801},
	{0x3E3E, 0x3E2A},
	{0x3E40, 0x1C01},
	{0x3E42, 0xFF84},
	{0x3E44, 0x8401},
	{0x3E46, 0x0C01},
	{0x3E48, 0x8401},
	{0x3E4A, 0x00FF},
	{0x3E4C, 0x8402},
	{0x3E4E, 0x8984},
	{0x3E50, 0x6628},
	{0x3E52, 0x8340},
	{0x3E54, 0x00FF},
	{0x3E56, 0x4A42},
	{0x3E58, 0x2703},
	{0x3E5A, 0x6752},
	{0x3E5C, 0x3F2A},
	{0x3E5E, 0x846A},
	{0x3E60, 0x4C01},
	{0x3E62, 0x8401},
	{0x3E66, 0x3901},
	{0x3E90, 0x2C01},
	{0x3E98, 0x2B02},
	{0x3E92, 0x2A04},
	{0x3E94, 0x2509},
	{0x3E96, 0x0000},
	{0x3E9A, 0x2905},
	{0x3E9C, 0x00FF},
	{0x3ECC, 0x00EB},
	{0x3ED0, 0x1E24},
	{0x3ED4, 0xAFC4},
	{0x3ED6, 0x909B},
	{0x3EE0, 0x2424},
	{0x3EE2, 0x9797},
	{0x3EE4, 0xC100},
	{0x3EE6, 0x0540},

	{0x0300, 0x0004},	/*VT_PIX_CLK_DIV */
	{0x0302, 0x0001},	/*VT_SYS_CLK_DIV */
	{0x0304, 0x0002},	/*PRE_PLL_CLK_DIV */
	{0x0306, 0x003A},	/*PLL_MULTIPLIER */
	{0x0308, 0x000A},	/*OP_PIX_CLK_DIV */
	{0x030A, 0x0001},	/*OP_SYS_CLK_DIV */
};

static struct msm_camera_i2c_reg_conf mt9e013_recommend_settings2[] = {
	/*Disable embedded data */
	{0x3064, 0x7800},	/*SMIA_TEST */
	/*configure 2-lane MIPI */
	{0x31AE, 0x0202},	/*SERIAL_FORMAT */
	{0x31B8, 0x0E3F},	/*MIPI_TIMING_2 */
	/*set data to RAW10 format */
	{0x0112, 0x0A0A},	/*CCP_DATA_FORMAT */
	{0x30F0, 0x8000},	/*VCM CONTROL */

	{0x3044, 0x0590},
	{0x306E, 0xFC80},
	{0x30B2, 0xC000},
	{0x30D6, 0x0800},
	{0x316C, 0xB42A},
	{0x316E, 0x869A},
	{0x3170, 0x210E},
	{0x317A, 0x010E},
	{0x31E0, 0x1FB9},
	{0x31E6, 0x07FC},
	{0x37C0, 0x0000},
	{0x37C2, 0x0000},
	{0x37C4, 0x0000},
	{0x37C6, 0x0000},
	{0x3E00, 0x0011},
	{0x3E02, 0x8801},
	{0x3E04, 0x2801},
	{0x3E06, 0x8449},
	{0x3E08, 0x6841},
	{0x3E0A, 0x400C},
	{0x3E0C, 0x1001},
	{0x3E0E, 0x2603},
	{0x3E10, 0x4B41},
	{0x3E12, 0x4B24},
	{0x3E14, 0xA3CF},
	{0x3E16, 0x8802},
	{0x3E18, 0x8401},
	{0x3E1A, 0x8601},
	{0x3E1C, 0x8401},
	{0x3E1E, 0x840A},
	{0x3E20, 0xFF00},
	{0x3E22, 0x8401},
	{0x3E24, 0x00FF},
	{0x3E26, 0x0088},
	{0x3E28, 0x2E8A},
	{0x3E30, 0x0000},
	{0x3E32, 0x00FF},
	{0x3E34, 0x4029},
	{0x3E36, 0x00FF},
	{0x3E38, 0x8469},
	{0x3E3A, 0x00FF},
	{0x3E3C, 0x2801},
	{0x3E3E, 0x3E2A},
	{0x3E40, 0x1C01},
	{0x3E42, 0xFF84},
	{0x3E44, 0x8401},
	{0x3E46, 0x0C01},
	{0x3E48, 0x8401},
	{0x3E4A, 0x00FF},
	{0x3E4C, 0x8402},
	{0x3E4E, 0x8984},
	{0x3E50, 0x6628},
	{0x3E52, 0x8340},
	{0x3E54, 0x00FF},
	{0x3E56, 0x4A42},
	{0x3E58, 0x2703},
	{0x3E5A, 0x6752},
	{0x3E5C, 0x3F2A},
	{0x3E5E, 0x846A},
	{0x3E60, 0x4C01},
	{0x3E62, 0x8401},
	{0x3E66, 0x3901},
	{0x3E90, 0x2C01},
	{0x3E98, 0x2B02},
	{0x3E92, 0x2A04},
	{0x3E94, 0x2509},
	{0x3E96, 0xF000},
	{0x3E9A, 0x2905},
	{0x3E9C, 0x00FF},
	{0x3ECC, 0x00EB},
	{0x3ED0, 0x1E24},
	{0x3ED4, 0xFAA4},
	{0x3ED6, 0x909B},
	{0x3EE0, 0x2424},
	{0x3EE2, 0x9797},
	{0x3EE4, 0xC100},
	{0x3EE6, 0x0540},

	{0x0300, 0x0004},	/*VT_PIX_CLK_DIV */
	{0x0302, 0x0001},	/*VT_SYS_CLK_DIV */
	{0x0304, 0x0002},	/*PRE_PLL_CLK_DIV */
	{0x0306, 0x003A},	/*PLL_MULTIPLIER */
	{0x0308, 0x000A},	/*OP_PIX_CLK_DIV */
	{0x030A, 0x0001},	/*OP_SYS_CLK_DIV */
};

struct msm_camera_i2c_reg_conf lc_settings[] = {
	{0x3600, 0x0290},
	{0x3602, 0x628D},
	{0x3604, 0x36F0},
	{0x3606, 0xCB2C},
	{0x3608, 0x9F70},
	{0x360A, 0x02D0},
	{0x360C, 0x852E},
	{0x360E, 0x2AF0},
	{0x3610, 0x5F4E},
	{0x3612, 0x9210},
	{0x3614, 0x0390},
	{0x3616, 0x13CE},
	{0x3618, 0x280F},
	{0x361A, 0x93AE},
	{0x361C, 0x846F},
	{0x361E, 0x0350},
	{0x3620, 0xA6CE},
	{0x3622, 0x40D0},
	{0x3624, 0x58CE},
	{0x3626, 0xB450},
	{0x3640, 0x8E4D},
	{0x3642, 0x81CE},
	{0x3644, 0x826F},
	{0x3646, 0x08EE},
	{0x3648, 0x676F},
	{0x364A, 0x832D},
	{0x364C, 0x006E},
	{0x364E, 0x916E},
	{0x3650, 0xD28E},
	{0x3652, 0x2FEF},
	{0x3654, 0x43CC},
	{0x3656, 0x29AE},
	{0x3658, 0xA6EC},
	{0x365A, 0xEA6E},
	{0x365C, 0x1BAD},
	{0x365E, 0x2FCD},
	{0x3660, 0x8F8E},
	{0x3662, 0xE46C},
	{0x3664, 0x3E4E},
	{0x3666, 0x208B},
	{0x3680, 0x0A11},
	{0x3682, 0x042F},
	{0x3684, 0x83F3},
	{0x3686, 0xE88F},
	{0x3688, 0x2B53},
	{0x368A, 0x1111},
	{0x368C, 0x5108},
	{0x368E, 0xEA52},
	{0x3690, 0x904F},
	{0x3692, 0x1713},
	{0x3694, 0x6B10},
	{0x3696, 0x0B6F},
	{0x3698, 0xC5D2},
	{0x369A, 0xA6AF},
	{0x369C, 0x0493},
	{0x369E, 0x0471},
	{0x36A0, 0x6F0D},
	{0x36A2, 0x8453},
	{0x36A4, 0xCCEF},
	{0x36A6, 0x2CF3},
	{0x36C0, 0x840D},
	{0x36C2, 0x1A4E},
	{0x36C4, 0x0731},
	{0x36C6, 0xFDAE},
	{0x36C8, 0xBD11},
	{0x36CA, 0x262C},
	{0x36CC, 0xA04E},
	{0x36CE, 0x1F70},
	{0x36D0, 0x2FAF},
	{0x36D2, 0x86F1},
	{0x36D4, 0xA8AE},
	{0x36D6, 0x998F},
	{0x36D8, 0x20F0},
	{0x36DA, 0x1A30},
	{0x36DC, 0xA7CF},
	{0x36DE, 0x972E},
	{0x36E0, 0x660E},
	{0x36E2, 0x050F},
	{0x36E4, 0x9D70},
	{0x36E6, 0x734F},
	{0x3700, 0xCFD1},
	{0x3702, 0xA910},
	{0x3704, 0x4F93},
	{0x3706, 0x10F1},
	{0x3708, 0x8613},
	{0x370A, 0xC131},
	{0x370C, 0x41AE},
	{0x370E, 0x3833},
	{0x3710, 0xB470},
	{0x3712, 0xD252},
	{0x3714, 0xBA91},
	{0x3716, 0xCB70},
	{0x3718, 0x2F53},
	{0x371A, 0x21B1},
	{0x371C, 0xDE12},
	{0x371E, 0xC2D1},
	{0x3720, 0x458E},
	{0x3722, 0x4753},
	{0x3724, 0xF310},
	{0x3726, 0xE132},
	{0x3782, 0x0614},
	{0x3784, 0x04FC},
	{0x37C0, 0xA0AB},
	{0x37C2, 0xD00A},
	{0x37C4, 0xC16A},
	{0x37C6, 0x80AB},
	{0x3780, 0x8000},
};

struct msm_camera_i2c_reg_conf otp_settings[] = {
	{0x3600, 0x0000},
	{0x3602, 0x0000},
	{0x3604, 0x0000},
	{0x3606, 0x0000},
	{0x3608, 0x0000},
	{0x360A, 0x0000},
	{0x360C, 0x0000},
	{0x360E, 0x0000},
	{0x3610, 0x0000},
	{0x3612, 0x0000},
	{0x3614, 0x0000},
	{0x3616, 0x0000},
	{0x3618, 0x0000},
	{0x361A, 0x0000},
	{0x361C, 0x0000},
	{0x361E, 0x0000},
	{0x3620, 0x0000},
	{0x3622, 0x0000},
	{0x3624, 0x0000},
	{0x3626, 0x0000},
	{0x3640, 0x0000},
	{0x3642, 0x0000},
	{0x3644, 0x0000},
	{0x3646, 0x0000},
	{0x3648, 0x0000},
	{0x364A, 0x0000},
	{0x364C, 0x0000},
	{0x364E, 0x0000},
	{0x3650, 0x0000},
	{0x3652, 0x0000},
	{0x3654, 0x0000},
	{0x3656, 0x0000},
	{0x3658, 0x0000},
	{0x365A, 0x0000},
	{0x365C, 0x0000},
	{0x365E, 0x0000},
	{0x3660, 0x0000},
	{0x3662, 0x0000},
	{0x3664, 0x0000},
	{0x3666, 0x0000},
	{0x3680, 0x0000},
	{0x3682, 0x0000},
	{0x3684, 0x0000},
	{0x3686, 0x0000},
	{0x3688, 0x0000},
	{0x368A, 0x0000},
	{0x368C, 0x0000},
	{0x368E, 0x0000},
	{0x3690, 0x0000},
	{0x3692, 0x0000},
	{0x3694, 0x0000},
	{0x3696, 0x0000},
	{0x3698, 0x0000},
	{0x369A, 0x0000},
	{0x369C, 0x0000},
	{0x369E, 0x0000},
	{0x36A0, 0x0000},
	{0x36A2, 0x0000},
	{0x36A4, 0x0000},
	{0x36A6, 0x0000},
	{0x36C0, 0x0000},
	{0x36C2, 0x0000},
	{0x36C4, 0x0000},
	{0x36C6, 0x0000},
	{0x36C8, 0x0000},
	{0x36CA, 0x0000},
	{0x36CC, 0x0000},
	{0x36CE, 0x0000},
	{0x36D0, 0x0000},
	{0x36D2, 0x0000},
	{0x36D4, 0x0000},
	{0x36D6, 0x0000},
	{0x36D8, 0x0000},
	{0x36DA, 0x0000},
	{0x36DC, 0x0000},
	{0x36DE, 0x0000},
	{0x36E0, 0x0000},
	{0x36E2, 0x0000},
	{0x36E4, 0x0000},
	{0x36E6, 0x0000},
	{0x3700, 0x0000},
	{0x3702, 0x0000},
	{0x3704, 0x0000},
	{0x3706, 0x0000},
	{0x3708, 0x0000},
	{0x370A, 0x0000},
	{0x370C, 0x0000},
	{0x370E, 0x0000},
	{0x3710, 0x0000},
	{0x3712, 0x0000},
	{0x3714, 0x0000},
	{0x3716, 0x0000},
	{0x3718, 0x0000},
	{0x371A, 0x0000},
	{0x371C, 0x0000},
	{0x371E, 0x0000},
	{0x3720, 0x0000},
	{0x3722, 0x0000},
	{0x3724, 0x0000},
	{0x3726, 0x0000},
	{0x3782, 0x0000},
	{0x3784, 0x0000},
	{0x37C0, 0x0000},
	{0x37C2, 0x0000},
	{0x37C4, 0x0000},
	{0x37C6, 0x0000},
};

static struct v4l2_subdev_info mt9e013_subdev_info[] = {
	{
	 .code = V4L2_MBUS_FMT_SBGGR10_1X10,
	 .colorspace = V4L2_COLORSPACE_JPEG,
	 .fmt = 1,
	 .order = 0,
	 },
	/* more can be supported, to be added later */
};

static struct msm_camera_i2c_conf_array mt9e013_init_conf[] = {
	{&mt9e013_recommend_settings[0],
	 ARRAY_SIZE(mt9e013_recommend_settings), 0, MSM_CAMERA_I2C_WORD_DATA}
};

static struct msm_camera_i2c_conf_array mt9e013_confs[] = {
	{&mt9e013_snap_settings[0],
	 ARRAY_SIZE(mt9e013_snap_settings), 0, MSM_CAMERA_I2C_WORD_DATA},
	{&mt9e013_prev_settings[0],
	 ARRAY_SIZE(mt9e013_prev_settings), 0, MSM_CAMERA_I2C_WORD_DATA},
	{&mt9e013_hfr60_settings[0],
	 ARRAY_SIZE(mt9e013_hfr60_settings), 0, MSM_CAMERA_I2C_WORD_DATA},
	{&mt9e013_hfr90_settings[0],
	 ARRAY_SIZE(mt9e013_hfr90_settings), 0, MSM_CAMERA_I2C_WORD_DATA},
	{&mt9e013_hfr120_settings[0],
	 ARRAY_SIZE(mt9e013_hfr120_settings), 0, MSM_CAMERA_I2C_WORD_DATA},
};

static struct msm_sensor_output_info_t mt9e013_dimensions[] = {
	{
	 .x_output = 0xCD0,
	 .y_output = 0x9A0,
	 .line_length_pclk = 0x13F8,
	 .frame_length_lines = 0xA2F,
	 .vt_pixel_clk = 174000000,
	 .op_pixel_clk = 174000000,
	 .binning_factor = 1,
	 },
	{
	 .x_output = 0x660,
	 .y_output = 0x4C8,
	 .line_length_pclk = 0x1018,
	 .frame_length_lines = 0x55B,
	 .vt_pixel_clk = 174000000,
	 .op_pixel_clk = 174000000,
	 .binning_factor = 1,
	 },
	{
	 .x_output = 0x340,
	 .y_output = 0x212,
	 .line_length_pclk = 0x970,
	 .frame_length_lines = 0x2A1,
	 .vt_pixel_clk = 98400000,
	 .op_pixel_clk = 98400000,
	 .binning_factor = 1,
	 },
	{
	 .x_output = 0x340,
	 .y_output = 0x212,
	 .line_length_pclk = 0x970,
	 .frame_length_lines = 0x2A1,
	 .vt_pixel_clk = 146400000,
	 .op_pixel_clk = 146400000,
	 .binning_factor = 1,
	 },
	{
	 .x_output = 0x340,
	 .y_output = 0x212,
	 .line_length_pclk = 0x970,
	 .frame_length_lines = 0x2A1,
	 .vt_pixel_clk = 196800000,
	 .op_pixel_clk = 196800000,
	 .binning_factor = 1,
	 },
};

static struct msm_camera_csi_params mt9e013_csi_params = {
	.data_format = CSI_10BIT,
	.lane_cnt = 2,
	.lane_assign = 0xe4,
	.dpcm_scheme = 0,
	.settle_cnt = 0x18,
};

static struct msm_camera_csi_params *mt9e013_csi_params_array[] = {
	&mt9e013_csi_params,
	&mt9e013_csi_params,
	&mt9e013_csi_params,
	&mt9e013_csi_params,
	&mt9e013_csi_params,
};

static struct msm_sensor_output_reg_addr_t mt9e013_reg_addr = {
	.x_output = 0x34C,
	.y_output = 0x34E,
	.line_length_pclk = 0x342,
	.frame_length_lines = 0x340,
};

static struct msm_sensor_id_info_t mt9e013_id_info = {
	.sensor_id_reg_addr = 0x0,
	.sensor_id = 0x4B00,
};

static struct msm_sensor_exp_gain_info_t mt9e013_exp_gain_info = {
	.coarse_int_time_addr = 0x202,
	.global_gain_addr = 0x305E,
	.vert_offset = 0,
};

static int32_t mt9e013_write_exp_gain(struct msm_sensor_ctrl_t *s_ctrl,
				      uint16_t gain, uint32_t line)
{
	uint32_t fl_lines;
	fl_lines =
	    (s_ctrl->curr_frame_length_lines * s_ctrl->fps_divider) / Q10;

	s_ctrl->func_tbl->sensor_group_hold_on(s_ctrl);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			     s_ctrl->sensor_exp_gain_info->global_gain_addr,
			     gain | 0x1000, MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			     s_ctrl->sensor_exp_gain_info->coarse_int_time_addr,
			     line, MSM_CAMERA_I2C_WORD_DATA);
	s_ctrl->func_tbl->sensor_group_hold_off(s_ctrl);
	return 0;
}

static int32_t mt9e013_write_exp_snapshot_gain
    (struct msm_sensor_ctrl_t *s_ctrl, uint16_t gain, uint32_t line) {
	uint32_t fl_lines;
	fl_lines =
	    (s_ctrl->curr_frame_length_lines * s_ctrl->fps_divider) / Q10;

	s_ctrl->func_tbl->sensor_group_hold_on(s_ctrl);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			     s_ctrl->sensor_exp_gain_info->global_gain_addr,
			     gain | 0x1000, MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			     s_ctrl->sensor_exp_gain_info->coarse_int_time_addr,
			     line, MSM_CAMERA_I2C_WORD_DATA);
	s_ctrl->func_tbl->sensor_group_hold_off(s_ctrl);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			     0x301A, (0x065C | 0x2), MSM_CAMERA_I2C_WORD_DATA);

	return 0;
}

static void mt9e013_start_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			     0x301A, 0x8250, MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			     0x301A, 0x8650, MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			     0x301A, 0x8658, MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			     0x0104, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			     0x301A, 0x065C, MSM_CAMERA_I2C_WORD_DATA);
}

static void mt9e013_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			     0x301A, 0x0058, MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			     0x301A, 0x0050, MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			     0x0104, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
}

static void mt9e013_read_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
	int i;
	uint16_t reg = 0;
	int rc = 0;
	int slot = 0x3400;

	pr_info("mt9e013 liteon read otprom\n");
read_otp:
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			     0x301A, 0x0020, MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			     0x301A, 0x0610, MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			     0x3134, 0xCD95, MSM_CAMERA_I2C_WORD_DATA);
	/* Record Type */
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			     0x304C, slot, MSM_CAMERA_I2C_WORD_DATA);
	/* Read Command */
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			     0x304A, 0x0010, MSM_CAMERA_I2C_WORD_DATA);
	msleep(10);
	for (i = 0; i < 10; i++) {
		rc = msm_camera_i2c_read(s_ctrl->sensor_i2c_client,
					 0x304A, &reg,
					 MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0) {
			pr_info("mt9e013_i2c_read otp failed!\n");
			i = 10;
			break;
		}
		pr_info("mt9e013 0x304A:0x%04x\n", reg);
		if (reg & 0x0020) {
			/* read end */
			if (reg & 0x0040)
				pr_info("mt9e013 otp read ok\n");
			else
				i = 10;
			break;
		}
		msleep(10);
	}
	if (i < 10) {
		for (i = 0; i <= 0xD2; i += 2) {
			rc = msm_camera_i2c_read(s_ctrl->sensor_i2c_client,
						 0x3800 + i, &reg,
						 MSM_CAMERA_I2C_WORD_DATA);
			if (rc < 0) {
				pr_info("mt9e013 otp failed!\n");
				break;
			}
			msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
					     otp_settings[i / 2].reg_addr, reg,
					     MSM_CAMERA_I2C_WORD_DATA);
			otp_settings[i / 2].reg_data = reg;
		}
		is_otp = 1;
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
				     0x3780, 0x8000, MSM_CAMERA_I2C_WORD_DATA);
		pr_info("mt9e013 (0x%04x 0x%04x) (0x%04x 0x%04x)",
			otp_settings[0].reg_addr,
			otp_settings[0].reg_data, otp_settings[105].reg_addr,
			otp_settings[105].reg_data);
	} else {
		if (slot > 0x3000) {
			slot -= 0x100;
			pr_info("mt9e013 otp try slot:0x%x\n", slot);
			goto read_otp;
		}
		/* OTP failed, never read it */
		otp_settings[0].reg_data = 0xFFFF;
	}
}

static void mt9e013_read_otp_sunny(struct msm_sensor_ctrl_t *s_ctrl)
{
	int i;
	uint16_t reg = 0;
	uint16_t reg1 = 0;
	int rc = 0;
	int slot = 0x3500;
	pr_info("mt9e013 sunny read otprom\n");
read_otps:
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			     0x301A, 0x0020, MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			     0x301A, 0x0610, MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			     0x3134, 0xCD95, MSM_CAMERA_I2C_WORD_DATA);
	/* Record Type */
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			     0x304C, slot, MSM_CAMERA_I2C_WORD_DATA);
	/* Read Command */
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			     0x304A, 0x0010, MSM_CAMERA_I2C_WORD_DATA);
	msleep(10);
	for (i = 0; i < 10; i++) {
		rc = msm_camera_i2c_read(s_ctrl->sensor_i2c_client,
					 0x304A, &reg,
					 MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0) {
			pr_info("mt9e013_i2c_read otp failed!\n");
			i = 10;
			break;
		}
		pr_info("mt9e013 0x304A:0x%04x\n", reg);
		if (reg & 0x0020) {
			/* read end */
			if (reg & 0x0040)
				pr_info("mt9e013 otp read ok\n");
			else {
				i = 10;
				break;
			}
			/* verify checksum */
			msm_camera_i2c_read(s_ctrl->sensor_i2c_client,
					    0x38EC, &reg,
					    MSM_CAMERA_I2C_WORD_DATA);
			msm_camera_i2c_read(s_ctrl->sensor_i2c_client, 0x38EE,
					    &reg1, MSM_CAMERA_I2C_WORD_DATA);
			if ((reg == 0xFFFF) || (reg1 == 0xFFFF)) {
				/* read otp success */
				i = 0;
				break;
			} else if ((reg == 0xF0F0) || (reg1 == 0xFFFF)) {
				/* read otp failed */
				i = 10;
				break;
			} else {
				if (slot > 0x3000) {
					slot -= 0x100;
					pr_info
					    ("mt9e013 sunny otp try slot:0x%x\n",
					     slot);
					goto read_otps;
				} else {
					i = 10;
					break;
				}
			}
		}
		msleep(10);
	}
	if (i < 10) {
		for (i = 0; i <= 0xD2; i += 2) {
			rc = msm_camera_i2c_read(s_ctrl->sensor_i2c_client,
						 0x3810 + i, &reg,
						 MSM_CAMERA_I2C_WORD_DATA);
			if (rc < 0) {
				pr_info("mt9e013 otp failed!\n");
				break;
			}
			msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
					     otp_settings[i / 2].reg_addr,
					     reg, MSM_CAMERA_I2C_WORD_DATA);
			otp_settings[i / 2].reg_data = reg;
		}
		is_otp = 1;
		msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
				     0x3780, 0x8000, MSM_CAMERA_I2C_WORD_DATA);
		pr_info("mt9e013 (0x%04x 0x%04x) (0x%04x 0x%04x)",
			otp_settings[0].reg_addr,
			otp_settings[0].reg_data, otp_settings[0].reg_addr,
			otp_settings[105].reg_data);
	} else {
		if (slot > 0x3000) {
			slot -= 0x100;
			pr_info("mt9e013 otp try slot:0x%x\n", slot);
			goto read_otps;
		}
		/* OTP failed, never read it */
		otp_settings[0].reg_data = 0xFFFF;
	}
}

static char *sensor_names[5] = { "mt9e013", "mt9e013",
	"mt9e0132", "mt9e0133", "mt9e0134"
};

static int32_t mt9e013_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{

	int32_t rc = 0;
	uint16_t chipid = 0;
	uint16_t chipver = 0;

	rc = msm_camera_i2c_read(s_ctrl->sensor_i2c_client,
				 s_ctrl->sensor_id_info->sensor_id_reg_addr,
				 &chipid, MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s: %s: read id failed\n", __func__,
		       s_ctrl->sensordata->sensor_name);
		return rc;
	}

	CDBG("mt9e013_sensor id: 0x%04x\n", chipid);
	if (chipid != 0x4b00) {
		pr_err("mt9e013_match_id chip id doesnot match 0x%04x\n",
		       chipid);
		return -ENODEV;
	}
	msm_camera_i2c_read(s_ctrl->sensor_i2c_client,
			    0x0002, &chipver, MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_read(s_ctrl->sensor_i2c_client,
			    0x301A, &chipid, MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_write(s_ctrl->sensor_i2c_client,
			     0x301A, chipid | 1 << 8, MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_read(s_ctrl->sensor_i2c_client,
			    0x301A, &chipid, MSM_CAMERA_I2C_WORD_DATA);
	msm_camera_i2c_read(s_ctrl->sensor_i2c_client,
			    0x3026, &chipid, MSM_CAMERA_I2C_WORD_DATA);

	vendor = chipid & 0xf;
	if ((chipver & 0xf000) == 0x1000) {	/* A8141 */
		if (vendor == SUNNY)
			vendor = SUNNY1;
		if (vendor == LITEON)
			vendor = LITEON1;
		memcpy(mt9e013_recommend_settings, mt9e013_recommend_settings2,
		       ARRAY_SIZE(mt9e013_recommend_settings));
	}
	if ((vendor == SUNNY) || (vendor == SUNNY1))
		mt9e013_read_otp_sunny(s_ctrl);
	else
		mt9e013_read_otp(s_ctrl);

	pr_info("mt9e013 gpi:%x ver:%x vendor:%d", chipid, chipver, vendor);
	s_ctrl->sensordata->sensor_name = sensor_names[vendor];
	return 0;
}

static const struct i2c_device_id mt9e013_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t) &mt9e013_s_ctrl},
	{}
};

static struct i2c_driver mt9e013_i2c_driver = {
	.id_table = mt9e013_i2c_id,
	.probe = msm_sensor_i2c_probe,
	.driver = {
		   .name = SENSOR_NAME,
		   },
};

static struct msm_camera_i2c_client mt9e013_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static int __init msm_sensor_init_module(void)
{
	return i2c_add_driver(&mt9e013_i2c_driver);
}

static struct v4l2_subdev_core_ops mt9e013_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops mt9e013_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops mt9e013_subdev_ops = {
	.core = &mt9e013_subdev_core_ops,
	.video = &mt9e013_subdev_video_ops,
};

static struct msm_sensor_fn_t mt9e013_func_tbl = {
	.sensor_start_stream = mt9e013_start_stream,
	.sensor_stop_stream = mt9e013_stop_stream,
	.sensor_group_hold_on = msm_sensor_group_hold_on,
	.sensor_group_hold_off = msm_sensor_group_hold_off,
	.sensor_set_fps = msm_sensor_set_fps,
	.sensor_write_exp_gain = mt9e013_write_exp_gain,
	.sensor_write_snapshot_exp_gain = mt9e013_write_exp_snapshot_gain,
	.sensor_csi_setting = msm_sensor_setting2,
	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	.sensor_mode_init = msm_sensor_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
	.sensor_match_id = mt9e013_match_id,
};

static struct msm_sensor_reg_t mt9e013_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.group_hold_on_conf = mt9e013_groupon_settings,
	.group_hold_on_conf_size = ARRAY_SIZE(mt9e013_groupon_settings),
	.group_hold_off_conf = mt9e013_groupoff_settings,
	.group_hold_off_conf_size = ARRAY_SIZE(mt9e013_groupoff_settings),
	.init_settings = &mt9e013_init_conf[0],
	.init_size = ARRAY_SIZE(mt9e013_init_conf),
	.mode_settings = &mt9e013_confs[0],
	.output_settings = &mt9e013_dimensions[0],
	.num_conf = ARRAY_SIZE(mt9e013_confs),
};

static struct msm_sensor_ctrl_t mt9e013_s_ctrl = {
	.msm_sensor_reg = &mt9e013_regs,
	.sensor_i2c_client = &mt9e013_sensor_i2c_client,
	.sensor_i2c_addr = 0x6C,
	.sensor_output_reg_addr = &mt9e013_reg_addr,
	.sensor_id_info = &mt9e013_id_info,
	.sensor_exp_gain_info = &mt9e013_exp_gain_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	.csic_params = &mt9e013_csi_params_array[0],
	.msm_sensor_mutex = &mt9e013_mut,
	.sensor_i2c_driver = &mt9e013_i2c_driver,
	.sensor_v4l2_subdev_info = mt9e013_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(mt9e013_subdev_info),
	.sensor_v4l2_subdev_ops = &mt9e013_subdev_ops,
	.func_tbl = &mt9e013_func_tbl,
	.clk_rate = MSM_SENSOR_MCLK_24HZ,
};

module_init(msm_sensor_init_module);
MODULE_DESCRIPTION("Aptina 8MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");
