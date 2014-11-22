/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
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
#define SENSOR_NAME "imx105"
#define PLATFORM_DRIVER_NAME "msm_camera_imx105"
#define imx105_obj imx105_##obj

DEFINE_MUTEX(imx105_mut);
static struct msm_sensor_ctrl_t imx105_s_ctrl;

static struct msm_camera_i2c_reg_conf imx105_start_settings[] = {
	{0x0100, 0x01},
};

static struct msm_camera_i2c_reg_conf imx105_stop_settings[] = {
	{0x0100, 0x00},
};

static struct msm_camera_i2c_reg_conf imx105_groupon_settings[] = {
	{0x104, 0x01},
};

static struct msm_camera_i2c_reg_conf imx105_groupoff_settings[] = {
	{0x104, 0x00},
};

static struct msm_camera_i2c_reg_conf imx105_prev_settings[] = {
	{0x0340, 0x0004},
	{0x0341, 0x00F2},
	{0x0342, 0x000D},
	{0x0343, 0x00D0},
	{0x0344, 0x0000},
	{0x0345, 0x0004},
	{0x0346, 0x0000},
	{0x0347, 0x0024},
	{0x0348, 0x000C},
	{0x0349, 0x00D3},
	{0x034A, 0x0009},
	{0x034B, 0x00C3},
	{0x034C, 0x0006},
	{0x034D, 0x0068},
	{0x034E, 0x0004},
	{0x034F, 0x00D0},
	{0x0381, 0x0001},
	{0x0383, 0x0003},
	{0x0385, 0x0001},
	{0x0387, 0x0003},
	{0x3033, 0x0000},
	{0x3048, 0x0001},
	{0x304C, 0x006F},
	{0x304D, 0x0003},
	{0x306A, 0x00D2},
	{0x309B, 0x0028},
	{0x309E, 0x0000},
	{0x30AA, 0x0002},
	{0x30D5, 0x0009},
	{0x30D6, 0x0001},
	{0x30D7, 0x0001},
	{0x30DE, 0x0002},
	{0x3102, 0x0008},
	{0x3103, 0x0022},
	{0x3104, 0x0020},
	{0x3105, 0x0000},
	{0x3106, 0x0087},
	{0x3107, 0x0000},
	{0x315C, 0x00A5},
	{0x315D, 0x00A4},
	{0x316E, 0x00A6},
	{0x316F, 0x00A5},
	{0x3318, 0x0072},
	{0x0202, 0x0004},
	{0x0203, 0x00ED}
};

static struct msm_camera_i2c_reg_conf imx105_snap_settings[] = {
	{0x0340, 0x0009},
	{0x0341, 0x00E6},
	{0x0342, 0x000D},
	{0x0343, 0x00D0},
	{0x0346, 0x0000},
	{0x0347, 0x0024},
	{0x034A, 0x0009},
	{0x034B, 0x00C3},
	{0x034C, 0x000C},
	{0x034D, 0x00D0},
	{0x034E, 0x0009},
	{0x034F, 0x00A0},
	{0x0381, 0x0001},
	{0x0383, 0x0001},
	{0x0385, 0x0001},
	{0x0387, 0x0001},
	{0x3033, 0x0000},
	{0x3048, 0x0000},
	{0x304C, 0x006F},
	{0x304D, 0x0003},
	{0x306A, 0x00D2},
	{0x309B, 0x0020},
	{0x309E, 0x0000},
	{0x30AA, 0x0002},
	{0x30D5, 0x0000},
	{0x30D6, 0x0085},
	{0x30D7, 0x002A},
	{0x30DE, 0x0000},
	{0x3102, 0x0008},
	{0x3103, 0x0022},
	{0x3104, 0x0020},
	{0x3105, 0x0000},
	{0x3106, 0x0087},
	{0x3107, 0x0000},
	{0x315C, 0x00A5},
	{0x315D, 0x00A4},
	{0x316E, 0x00A6},
	{0x316F, 0x00A5},
	{0x3318, 0x0062},
	{0x0202, 0x0009},
	{0x0203, 0x00E1}
};

/* FullHD register settings */
static struct msm_camera_i2c_reg_conf imx105_fullhd_settings[] = {
	{0x0340, 0x0005},
	{0x0341, 0x00F0},
	{0x0342, 0x000D},
	{0x0343, 0x00D0},
	{0x0344, 0x0001},
	{0x0345, 0x004C},
	{0x0346, 0x0002},
	{0x0347, 0x0012},
	{0x0348, 0x000B},
	{0x0349, 0x008B},
	{0x034A, 0x0007},
	{0x034B, 0x00D5},
	{0x034C, 0x000A},
	{0x034D, 0x0040},
	{0x034E, 0x0005},
	{0x034F, 0x00C4},
	{0x0381, 0x0001},
	{0x0383, 0x0001},
	{0x0385, 0x0001},
	{0x0387, 0x0001},
	{0x3033, 0x0000},
	{0x3048, 0x0000},
	{0x304C, 0x006F},
	{0x304D, 0x0003},
	{0x306A, 0x00D2},
	{0x309B, 0x0020},
	{0x309E, 0x0000},
	{0x30AA, 0x0002},
	{0x30D5, 0x0000},
	{0x30D6, 0x0085},
	{0x30D7, 0x002A},
	{0x30DE, 0x0000},
	{0x3102, 0x0008},
	{0x3103, 0x0022},
	{0x3104, 0x0020},
	{0x3105, 0x0000},
	{0x3106, 0x0087},
	{0x3107, 0x0000},
	{0x315C, 0x00A5},
	{0x315D, 0x00A4},
	{0x316E, 0x00A6},
	{0x316F, 0x00A5},
	{0x3318, 0x0062},
	{0x0202, 0x0005},
	{0x0203, 0x00EB}
};

static struct msm_camera_i2c_reg_conf imx105_recommend_settings[] = {
	{0x0103, 0x0001},
	{0x0305, 0x0001},
	{0x0307, 0x001C},
	{0x303C, 0x004B},
	{0x3031, 0x0010},
	{0x3064, 0x0012},
	{0x3087, 0x0057},
	{0x308A, 0x0035},
	{0x3091, 0x0041},
	{0x3098, 0x0003},
	{0x3099, 0x00C0},
	{0x309A, 0x00A3},
	{0x309C, 0x0034},
	{0x30AB, 0x0001},
	{0x30AD, 0x0008},
	{0x30F3, 0x0003},
	{0x3116, 0x0031},
	{0x3117, 0x0038},
	{0x3138, 0x0028},
	{0x3137, 0x0014},
	{0x3139, 0x002E},
	{0x314D, 0x002A},
	{0x3343, 0x0004},
	{0x3032, 0x0040},
	{0x30D0, 0x007F},
	{0x30CF, 0x000E},
	{0x3640, 0x0010},
	{0x3620, 0x0000},
	{0x3408, 0x0000},
	{0x340A, 0x0000},
	{0x340C, 0x0001},
	{0x3081, 0x0048},	/*REGSELIVCM[0:1] */
	{0x3400, 0x0000},	/* VCMDAMP bit 1-- OFF, STBVCM bit 0-- NO STANDBY */
/*	{0x3401, 0x000C}, */	/*VCM_SYNC - vcmcode change at sync*/
	{0x3401, 0x0002},	/*vcm sync, the max current is 80mA */
	{0x3404, 0x0017},	/*VCMIIRA[0:7] 23d */
	{0x3405, 0x0000}	/* Damping Control mode: VCM Filter Mode */
};

static struct v4l2_subdev_info imx105_subdev_info[] = {
	{
	 .code = V4L2_MBUS_FMT_SBGGR10_1X10,
	 .colorspace = V4L2_COLORSPACE_JPEG,
	 .fmt = 1,
	 .order = 0,
	 },
	/* more can be supported, to be added later */
};

static struct msm_camera_i2c_conf_array imx105_init_conf[] = {
	{&imx105_recommend_settings[0],
	 ARRAY_SIZE(imx105_recommend_settings), 0, MSM_CAMERA_I2C_BYTE_DATA}
};

static struct msm_camera_i2c_conf_array imx105_confs[] = {
	{&imx105_snap_settings[0],
	 ARRAY_SIZE(imx105_snap_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&imx105_prev_settings[0],
	 ARRAY_SIZE(imx105_prev_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&imx105_fullhd_settings[0],
	 ARRAY_SIZE(imx105_fullhd_settings), 0, MSM_CAMERA_I2C_BYTE_DATA},

};

static struct msm_sensor_output_info_t imx105_dimensions[] = {
	{
	 .x_output = 3280,
	 .y_output = 2464,
	 .line_length_pclk = 0xDD0,
	 .frame_length_lines = 0x9E6,
	 .vt_pixel_clk = 174000000,
	 .op_pixel_clk = 174000000,
	 .binning_factor = 0,
	 },
	{
	 .x_output = 1640,
	 .y_output = 1232,
	 .line_length_pclk = 0xDD0,
	 .frame_length_lines = 0x4F2,
	 .vt_pixel_clk = 134300000,
	 .op_pixel_clk = 174000000,
	 .binning_factor = 1,
	 },
	{
	 .x_output = 2624,
	 .y_output = 1476,
	 .line_length_pclk = 0xDD0,
	 .frame_length_lines = 0x5F0,
	 .vt_pixel_clk = 161000000,
	 .op_pixel_clk = 174000000,
	 .binning_factor = 1,
	 },

};

static struct msm_camera_csi_params imx105_csic_params = {
	.data_format = CSI_10BIT,
	.lane_cnt = 2,
	.lane_assign = 0xe4,
	.dpcm_scheme = 0,
	.settle_cnt = 0x14,
};

static struct msm_camera_csi_params *imx105_csic_params_array[] = {
	&imx105_csic_params,
	&imx105_csic_params,
	&imx105_csic_params,
};

static struct msm_camera_csid_vc_cfg imx105_cid_cfg[] = {
	{0, CSI_RAW10, CSI_DECODE_10BIT},
	{1, CSI_EMBED_DATA, CSI_DECODE_8BIT},
	{2, CSI_RESERVED_DATA_0, CSI_DECODE_8BIT},
};

static struct msm_camera_csi2_params imx105_csi_params = {
	.csid_params = {
			.lane_cnt = 2,
			.lut_params = {
				       .num_cid = ARRAY_SIZE(imx105_cid_cfg),
				       .vc_cfg = imx105_cid_cfg,
				       },
			},
	.csiphy_params = {
			  .lane_cnt = 2,
			  .settle_cnt = 0x14,
			  },
};

static struct msm_camera_csi2_params *imx105_csi_params_array[] = {
	&imx105_csi_params,
	&imx105_csi_params,
	&imx105_csi_params,
};

static struct msm_sensor_output_reg_addr_t imx105_reg_addr = {
	.x_output = 0x34C,
	.y_output = 0x34E,
	.line_length_pclk = 0x342,
	.frame_length_lines = 0x340,
};

static struct msm_sensor_id_info_t imx105_id_info = {
	.sensor_id_reg_addr = 0x0,
	.sensor_id = 0x0105,
};

static struct msm_sensor_exp_gain_info_t imx105_exp_gain_info = {
	.coarse_int_time_addr = 0x202,
	.global_gain_addr = 0x204,
	.vert_offset = 4,
};

static const struct i2c_device_id imx105_i2c_id[] = {
	{SENSOR_NAME, (kernel_ulong_t) &imx105_s_ctrl},
	{}
};

static struct i2c_driver imx105_i2c_driver = {
	.id_table = imx105_i2c_id,
	.probe = msm_sensor_i2c_probe,
	.driver = {
		   .name = SENSOR_NAME,
		   },
};

static struct msm_camera_i2c_client imx105_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static int __init msm_sensor_init_module(void)
{
	return i2c_add_driver(&imx105_i2c_driver);
}

static struct v4l2_subdev_core_ops imx105_subdev_core_ops = {
	.ioctl = msm_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops imx105_subdev_video_ops = {
	.enum_mbus_fmt = msm_sensor_v4l2_enum_fmt,
};

static struct v4l2_subdev_ops imx105_subdev_ops = {
	.core = &imx105_subdev_core_ops,
	.video = &imx105_subdev_video_ops,
};

static char *sensor_names[5] = {
	"imx105", "imx105", "imx105s", "imx105p", "imx105f"
};

static int32_t imx105_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{

	int32_t rc = 0;
	int32_t i;
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

	pr_info("imx105_sensor id: 0x%04x\n", chipid);
	if (chipid != 0x0105) {
		pr_err("imx105_match_id chip id doesnot match 0x%04x\n",
		       chipid);
		return -ENODEV;
	}

	for (i = 4; i >= 0; i--) {
		rc = msm_camera_i2c_read(s_ctrl->sensor_i2c_client,
					 0x34C9, &chipid,
					 MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0)
			pr_info("imx105 select otp bank error rc:%d\n", rc);

		msm_camera_i2c_read(s_ctrl->sensor_i2c_client,
				    0x3500 + i * 8, &chipid,
				    MSM_CAMERA_I2C_WORD_DATA);

		if ((chipid >= 0x0801) && (chipid <= 0x0804)) {
			chipver = chipid & 0x000f;
			pr_info("imx105 vendor:%d from:0x%04x @ %04x", chipver,
				chipid, 0x3500 + i * 8);
		} else if (chipid == 0x0811) {
			chipver = 1;
		} else {
			chipver = 0;
		}
		if (chipver != 0)
			break;
	}
	pr_info("%s vendor:%d from:0x%04x @ %04x", sensor_names[chipver],
		chipver, chipid, 0x3500 + i * 8);
	s_ctrl->sensordata->sensor_name = sensor_names[chipver];
	return 0;
}

static struct msm_sensor_fn_t imx105_func_tbl = {
	.sensor_start_stream = msm_sensor_start_stream,
	.sensor_stop_stream = msm_sensor_stop_stream,
	.sensor_group_hold_on = msm_sensor_group_hold_on,
	.sensor_group_hold_off = msm_sensor_group_hold_off,
	.sensor_set_fps = msm_sensor_set_fps,
	.sensor_write_exp_gain = msm_sensor_write_exp_gain1,
	.sensor_write_snapshot_exp_gain = msm_sensor_write_exp_gain1,
	.sensor_setting = msm_sensor_setting,
	.sensor_csi_setting = msm_sensor_setting1,
	.sensor_set_sensor_mode = msm_sensor_set_sensor_mode,
	.sensor_mode_init = msm_sensor_mode_init,
	.sensor_get_output_info = msm_sensor_get_output_info,
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_adjust_frame_lines = msm_sensor_adjust_frame_lines,
	.sensor_get_csi_params = msm_sensor_get_csi_params,
	.sensor_match_id = imx105_match_id,
};

static struct msm_sensor_reg_t imx105_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.start_stream_conf = imx105_start_settings,
	.start_stream_conf_size = ARRAY_SIZE(imx105_start_settings),
	.stop_stream_conf = imx105_stop_settings,
	.stop_stream_conf_size = ARRAY_SIZE(imx105_stop_settings),
	.group_hold_on_conf = imx105_groupon_settings,
	.group_hold_on_conf_size = ARRAY_SIZE(imx105_groupon_settings),
	.group_hold_off_conf = imx105_groupoff_settings,
	.group_hold_off_conf_size = ARRAY_SIZE(imx105_groupoff_settings),
	.init_settings = &imx105_init_conf[0],
	.init_size = ARRAY_SIZE(imx105_init_conf),
	.mode_settings = &imx105_confs[0],
	.output_settings = &imx105_dimensions[0],
	.num_conf = ARRAY_SIZE(imx105_confs),
};

static struct msm_sensor_ctrl_t imx105_s_ctrl = {
	.msm_sensor_reg = &imx105_regs,
	.sensor_i2c_client = &imx105_sensor_i2c_client,
	.sensor_i2c_addr = 0x20,
	.sensor_output_reg_addr = &imx105_reg_addr,
	.sensor_id_info = &imx105_id_info,
	.sensor_exp_gain_info = &imx105_exp_gain_info,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	.csic_params = &imx105_csic_params_array[0],
	.csi_params = &imx105_csi_params_array[0],
	.msm_sensor_mutex = &imx105_mut,
	.sensor_i2c_driver = &imx105_i2c_driver,
	.sensor_v4l2_subdev_info = imx105_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(imx105_subdev_info),
	.sensor_v4l2_subdev_ops = &imx105_subdev_ops,
	.func_tbl = &imx105_func_tbl,
	.clk_rate = MSM_SENSOR_MCLK_24HZ,
};

module_init(msm_sensor_init_module);
MODULE_DESCRIPTION("Sony 8MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");
