/*  Date: 2012/06/25 15:48:00
 *  Revision: 1.0
 */

/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/uaccess.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/input/bmc055.h>

#define LOG_LEVEL_E 3
#define LOG_LEVEL_N 5
#define LOG_LEVEL_I 6
#define LOG_LEVEL_D 7

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_I
#endif

#ifndef MODULE_TAG
#define MODULE_TAG "<>"
#endif

#if (LOG_LEVEL >= LOG_LEVEL_E)
#define PERR(fmt, args...) \
	printk(KERN_INFO "\n" "[E]" KERN_ERR MODULE_TAG \
	"<%s><%d>" fmt "\n", __func__, __LINE__, ##args)
#else
#define PERR(fmt, args...)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_N)
#define PNOTICE(fmt, args...) \
	printk(KERN_INFO "\n" "[N]" KERN_NOTICE MODULE_TAG \
	"<%s><%d>" fmt "\n", __func__, __LINE__, ##args)
#else
#define PNOTICE(fmt, args...)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_I)
#define PINFO(fmt, args...) printk(KERN_INFO "\n" "[I]" KERN_INFO MODULE_TAG \
	"<%s><%d>" fmt "\n", __func__, __LINE__, ##args)
#else
#define PINFO(fmt, args...)
#endif

#if (LOG_LEVEL >= LOG_LEVEL_D)
#define PDEBUG(fmt, args...) printk(KERN_INFO "\n" "[D]" KERN_DEBUG MODULE_TAG \
	"<%s><%d>" fmt "\n", __func__, __LINE__, ##args)
#else
#define PDEBUG(fmt, args...)
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bmc055_early_suspend(struct early_suspend *h);
static void bmc055_late_resume(struct early_suspend *h);
#endif

/*user defined Structures*/
struct bmm050_mdata {
	signed short datax;
	signed short datay;
	signed short dataz;
	unsigned short resistance;
};

struct bmm050_offset {
	signed short datax;
	signed short datay;
	signed short dataz;
};

struct bmm050 {
	signed char dig_x1;
	signed char dig_y1;

	signed char dig_x2;
	signed char dig_y2;

	unsigned short dig_z1;
	signed short dig_z2;
	signed short dig_z3;
	signed short dig_z4;

	unsigned char dig_xy1;
	signed char dig_xy2;

	unsigned short dig_xyz1;
};

struct op_mode_map {
	char *op_mode_name;
	long op_mode;
};

struct bmm050_data {
	struct bmm050 device;
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;
	atomic_t delay;
	struct bmm050_mdata value;
	u8 enable;
	u8 opened;
	s8 op_mode;
	u8 rept_xy;
	u8 rept_z;
};

struct bma255acc {
	s16 x, y, z;
};

struct bma255_data {
	atomic_t delay;
	atomic_t enable;
	atomic_t range;
	unsigned char mode;
	struct input_dev *input;
	struct bma255acc value;
	struct delayed_work work;
};

struct bmc055_data {
	struct i2c_client *bmc055_client;
	struct mutex bmc055_mutex;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	struct bma255_data acc;
	struct bmm050_data mag;
};

/* macro for define sys attribute */
#define BMC055_ATTR(_prefix, _name, _mode, _show, _store) \
		static struct device_attribute \
		bmc055_##_prefix##_##_name##_attr = \
		__ATTR(_name, _mode, _show, _store)

/*-----------------bmm050_driver-------------------------------*/
/* i2c operation for API */
static void bmm_delay(u32 msec);
static char bmm_i2c_read(struct i2c_client *client, u8 reg_addr,
			 u8 *data, u8 len);
static char bmm_i2c_write(struct i2c_client *client, u8 reg_addr,
			  u8 *data, u8 len);
static void bmm_dump_reg(struct i2c_client *client);
static int bmm_wakeup(struct i2c_client *client);
static int bmm_check_chip_id(struct i2c_client *client);

static const struct op_mode_map op_mode_maps[] = {
	{"normal", BMM050_NORMAL_MODE},
	{"forced", BMM050_FORCED_MODE},
	{"suspend", BMM050_SUSPEND_MODE},
	{"sleep", BMM050_SLEEP_MODE},
};

static int bmm050_init_trim_registers(struct bmm050_data *pbmm050_data)
{
	int comres = 0;
	unsigned char a_data_u8r[2];
	struct bmm050 *p_bmm050 = &pbmm050_data->device;
	struct bmc055_data *bmc055_data = container_of(pbmm050_data,
						       struct bmc055_data, mag);

	comres = bmm_i2c_read(bmc055_data->bmc055_client,
			      BMM050_DIG_X1, (unsigned char *)&p_bmm050->dig_x1,
			      1);
	comres |=
	    bmm_i2c_read(bmc055_data->bmc055_client, BMM050_DIG_Y1,
			 (unsigned char *)&p_bmm050->dig_y1, 1);
	comres |=
	    bmm_i2c_read(bmc055_data->bmc055_client, BMM050_DIG_X2,
			 (unsigned char *)&p_bmm050->dig_x2, 1);
	comres |=
	    bmm_i2c_read(bmc055_data->bmc055_client, BMM050_DIG_Y2,
			 (unsigned char *)&p_bmm050->dig_y2, 1);
	comres |=
	    bmm_i2c_read(bmc055_data->bmc055_client, BMM050_DIG_XY1,
			 (unsigned char *)&p_bmm050->dig_xy1, 1);
	comres |=
	    bmm_i2c_read(bmc055_data->bmc055_client, BMM050_DIG_XY2,
			 (unsigned char *)&p_bmm050->dig_xy2, 1);

	/* shorts can not be recasted into (unsigned char*)
	 * due to possible mixup between trim data
	 * arrangement and memory arrangement */

	comres |= bmm_i2c_read(bmc055_data->bmc055_client,
			       BMM050_DIG_Z1_LSB, a_data_u8r, 2);
	p_bmm050->dig_z1 = (BMM050_U16) ((((BMM050_U16) ((unsigned char)
							 a_data_u8r[1])) <<
					  SHIFT_LEFT_8_POSITION) |
					 a_data_u8r[0]);

	comres |= bmm_i2c_read(bmc055_data->bmc055_client,
			       BMM050_DIG_Z2_LSB, a_data_u8r, 2);
	p_bmm050->dig_z2 = (BMM050_S16) ((((BMM050_S16) ((signed char)
							 a_data_u8r[1])) <<
					  SHIFT_LEFT_8_POSITION) |
					 a_data_u8r[0]);

	comres |= bmm_i2c_read(bmc055_data->bmc055_client,
			       BMM050_DIG_Z3_LSB, a_data_u8r, 2);
	p_bmm050->dig_z3 = (BMM050_S16) ((((BMM050_S16) ((signed char)
							 a_data_u8r[1])) <<
					  SHIFT_LEFT_8_POSITION) |
					 a_data_u8r[0]);

	comres |= bmm_i2c_read(bmc055_data->bmc055_client,
			       BMM050_DIG_Z4_LSB, a_data_u8r, 2);
	p_bmm050->dig_z4 = (BMM050_S16) ((((BMM050_S16) ((signed char)
							 a_data_u8r[1])) <<
					  SHIFT_LEFT_8_POSITION) |
					 a_data_u8r[0]);

	comres |= bmm_i2c_read(bmc055_data->bmc055_client,
			       BMM050_DIG_XYZ1_LSB, a_data_u8r, 2);
	a_data_u8r[1] = BMM050_GET_BITSLICE(a_data_u8r[1], BMM050_DIG_XYZ1_MSB);
	p_bmm050->dig_xyz1 = (BMM050_U16) ((((BMM050_U16)
					     ((unsigned char)a_data_u8r[1])) <<
					    SHIFT_LEFT_8_POSITION) |
					   a_data_u8r[0]);
	return comres;
}

static int bmm050_get_powermode(struct i2c_client *i2c_client,
				unsigned char *mode)
{
	int comres = 0;
	unsigned char v_data_u8r;

	if (i2c_client == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		comres = bmm_i2c_read(i2c_client,
				      BMM050_POWER_CNTL_PCB__REG,
				      &v_data_u8r, 1);
		*mode = BMM050_GET_BITSLICE(v_data_u8r, BMM050_POWER_CNTL_PCB);
	}
	return comres;
}

static int bmm050_set_powermode(struct i2c_client *i2c_client,
				unsigned char mode)
{
	int comres = 0;
	unsigned char v_data_u8r;

	if (i2c_client == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		comres = bmm_i2c_read(i2c_client,
				      BMM050_POWER_CNTL_PCB__REG,
				      &v_data_u8r, 1);
		v_data_u8r = BMM050_SET_BITSLICE(v_data_u8r,
						 BMM050_POWER_CNTL_PCB, mode);
		comres |= bmm_i2c_write(i2c_client,
					BMM050_POWER_CNTL_PCB__REG,
					&v_data_u8r, 1);
	}
	return comres;
}

static int bmm050_get_functional_state(struct i2c_client *i2c_client,
				       unsigned char *functional_state)
{
	int comres = 0;
	unsigned char v_data_u8r;

	if (i2c_client == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		comres = bmm_i2c_read(i2c_client,
				      BMM050_CNTL_OPMODE__REG, &v_data_u8r, 1);
		*functional_state =
		    BMM050_GET_BITSLICE(v_data_u8r, BMM050_CNTL_OPMODE);
	}
	return comres;
}

static int bmm050_set_functional_state(struct i2c_client *i2c_client,
				       unsigned char functional_state)
{
	int comres = 0;
	unsigned char v_data1_u8r;

	if (i2c_client == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		switch (functional_state) {
		case BMM050_NORMAL_MODE:
			comres = bmm050_get_powermode(i2c_client, &v_data1_u8r);
			if (v_data1_u8r == BMM050_OFF) {
				comres =
				    bmm050_set_powermode(i2c_client, BMM050_ON);
				bmm_delay(BMM050_DELAY_SUSPEND_SLEEP);
			}
			{
				comres |= bmm_i2c_read(i2c_client,
						       BMM050_CNTL_OPMODE__REG,
						       &v_data1_u8r, 1);
				v_data1_u8r = BMM050_SET_BITSLICE(v_data1_u8r,
								  BMM050_CNTL_OPMODE,
								  BMM050_NORMAL_MODE);
				comres |= bmm_i2c_write(i2c_client,
							BMM050_CNTL_OPMODE__REG,
							&v_data1_u8r, 1);
			}
			break;
		case BMM050_SUSPEND_MODE:
			comres = bmm050_set_powermode(i2c_client, BMM050_OFF);
			break;
		case BMM050_FORCED_MODE:
			comres = bmm050_get_powermode(i2c_client, &v_data1_u8r);
			if (v_data1_u8r == BMM050_OFF) {
				comres =
				    bmm050_set_powermode(i2c_client, BMM050_ON);
				bmm_delay(BMM050_DELAY_SUSPEND_SLEEP);
			}
			comres |= bmm_i2c_read(i2c_client,
					       BMM050_CNTL_OPMODE__REG,
					       &v_data1_u8r, 1);
			v_data1_u8r = BMM050_SET_BITSLICE(v_data1_u8r,
							  BMM050_CNTL_OPMODE,
							  BMM050_ON);
			comres |=
			    bmm_i2c_read(i2c_client, BMM050_CNTL_OPMODE__REG,
					 &v_data1_u8r, 1);
			break;
		case BMM050_SLEEP_MODE:
			bmm050_get_powermode(i2c_client, &v_data1_u8r);
			if (v_data1_u8r == BMM050_OFF) {
				comres =
				    bmm050_set_powermode(i2c_client, BMM050_ON);
				bmm_delay(BMM050_DELAY_SUSPEND_SLEEP);
			}
			comres |= bmm_i2c_read(i2c_client,
					       BMM050_CNTL_OPMODE__REG,
					       &v_data1_u8r, 1);
			v_data1_u8r = BMM050_SET_BITSLICE(v_data1_u8r,
							  BMM050_CNTL_OPMODE,
							  BMM050_SLEEP_MODE);
			comres |= bmm_i2c_read(i2c_client,
					       BMM050_CNTL_OPMODE__REG,
					       &v_data1_u8r, 1);
			break;
		default:
			comres = E_BMM050_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}

static int bmm050_get_repetitions_XY(struct i2c_client *i2c_client,
				     unsigned char *no_repetitions_xy)
{
	int comres = 0;
	unsigned char v_data_u8r;
	if (i2c_client == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		comres = bmm_i2c_read(i2c_client,
				      BMM050_NO_REPETITIONS_XY, &v_data_u8r, 1);
		*no_repetitions_xy = v_data_u8r;
	}
	return comres;
}

static int bmm050_set_repetitions_XY(struct i2c_client *i2c_client,
				     unsigned char no_repetitions_xy)
{
	int comres = 0;
	unsigned char v_data_u8r;
	if (i2c_client == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		v_data_u8r = no_repetitions_xy;
		comres = bmm_i2c_read(i2c_client,
				      BMM050_NO_REPETITIONS_XY, &v_data_u8r, 1);
	}
	return comres;
}

static int bmm050_get_repetitions_Z(struct i2c_client *i2c_client,
				    unsigned char *no_repetitions_z)
{
	int comres = 0;
	unsigned char v_data_u8r;
	if (i2c_client == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		comres = bmm_i2c_read(i2c_client,
				      BMM050_NO_REPETITIONS_Z, &v_data_u8r, 1);
		*no_repetitions_z = v_data_u8r;
	}
	return comres;
}

static int bmm050_set_repetitions_Z(struct i2c_client *i2c_client,
				    unsigned char no_repetitions_z)
{
	int comres = 0;
	unsigned char v_data_u8r;
	if (i2c_client == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		v_data_u8r = no_repetitions_z;
		comres = bmm_i2c_write(i2c_client,
				       BMM050_NO_REPETITIONS_Z, &v_data_u8r, 1);
	}
	return comres;
}

BMM050_S16 bmm050_compensate_X(struct bmm050 *p_bmm050,
			       BMM050_S16 mdata_x, BMM050_U16 data_R)
{
	BMM050_S16 inter_retval;
	if (mdata_x != BMM050_FLIP_OVERFLOW_ADCVAL	/* no overflow */
	    ) {
		inter_retval = ((BMM050_S16) (((BMM050_U16)
					       ((((BMM050_S32) p_bmm050->
						  dig_xyz1) << 14) / (data_R !=
								      0 ? data_R
								      :
								      p_bmm050->
								      dig_xyz1)))
					      - ((BMM050_U16) 0x4000)));
		inter_retval =
		    ((BMM050_S16)
		     ((((BMM050_S32) mdata_x) *
		       ((((((((BMM050_S32) p_bmm050->dig_xy2) *
			     ((((BMM050_S32) inter_retval) *
			       ((BMM050_S32) inter_retval)) >> 7)) +
			    (((BMM050_S32) inter_retval) *
			     ((BMM050_S32) (((BMM050_S16) p_bmm050->dig_xy1)
					    << 7)))) >> 9) +
			  ((BMM050_S32) 0x100000)) *
			 ((BMM050_S32)
			  (((BMM050_S16) p_bmm050->dig_x2) +
			   ((BMM050_S16) 0xA0)))) >> 12)) >> 13)) +
		    (((BMM050_S16) p_bmm050->dig_x1) << 3);
	} else {
		/* overflow */
		inter_retval = BMM050_OVERFLOW_OUTPUT;
	}
	return inter_retval;
}

BMM050_S16 bmm050_compensate_Y(struct bmm050 *p_bmm050,
			       BMM050_S16 mdata_y, BMM050_U16 data_R)
{
	BMM050_S16 inter_retval;
	if (mdata_y != BMM050_FLIP_OVERFLOW_ADCVAL	/* no overflow */
	    ) {
		inter_retval = ((BMM050_S16) (((BMM050_U16) ((((BMM050_S32)
							       p_bmm050->
							       dig_xyz1) << 14)
							     / (data_R !=
								0 ? data_R :
								p_bmm050->
								dig_xyz1))) -
					      ((BMM050_U16) 0x4000)));
		inter_retval =
		    ((BMM050_S16) ((((BMM050_S32) mdata_y) * ((((((((BMM050_S32)
								    p_bmm050->
								    dig_xy2) *
								   ((((BMM050_S32) inter_retval) * ((BMM050_S32) inter_retval)) >> 7)) + (((BMM050_S32) inter_retval) * ((BMM050_S32) (((BMM050_S16)
																							p_bmm050->
																							dig_xy1)
																						       <<
																						       7))))
								 >> 9) +
								((BMM050_S32)
								 0x100000)) *
							       ((BMM050_S32)
								(((BMM050_S16)
								  p_bmm050->
								  dig_y2)
								 +
								 ((BMM050_S16)
								  0xA0))))
							      >> 12)) >> 13)) +
		    (((BMM050_S16) p_bmm050->dig_y1) << 3);
	} else {
		/* overflow */
		inter_retval = BMM050_OVERFLOW_OUTPUT;
	}
	return inter_retval;
}

BMM050_S16 bmm050_compensate_Z(struct bmm050 *p_bmm050,
			       BMM050_S16 mdata_z, BMM050_U16 data_R)
{
	BMM050_S32 retval;
	if ((mdata_z != BMM050_HALL_OVERFLOW_ADCVAL)	/* no overflow */
	    ) {
		retval = (((((BMM050_S32) (mdata_z - p_bmm050->dig_z4)) << 15) -
			   ((((BMM050_S32) p_bmm050->dig_z3) *
			     ((BMM050_S32) (((BMM050_S16) data_R) -
					    ((BMM050_S16)
					     p_bmm050->dig_xyz1)))) >> 2)) /
			  (p_bmm050->dig_z2 + ((BMM050_S16) (((((BMM050_S32)
								p_bmm050->
								dig_z1) *
							       ((((BMM050_S16)
								  data_R) <<
								 1))) +
							      (1 << 15)) >>
							     16))));
		/* saturate result to +/- 2 mT */
		if (retval > BMM050_POSITIVE_SATURATION_Z) {
			retval = BMM050_POSITIVE_SATURATION_Z;
		} else {
			if (retval < BMM050_NEGATIVE_SATURATION_Z)
				retval = BMM050_NEGATIVE_SATURATION_Z;
		}
	} else {
		/* overflow */
		retval = BMM050_OVERFLOW_OUTPUT;
	}
	return (BMM050_S16) retval;
}

static int bmm050_read_mdataXYZ(struct bmm050_data *client_data,
				struct bmm050_mdata *mdata)
{
	int comres = 0;
	unsigned char a_data_u8r[8];
	struct bmc055_data *bmc055_data = container_of(client_data,
						       struct bmc055_data, mag);
	struct i2c_client *i2c_client = bmc055_data->bmc055_client;
	struct bmm050 *pbmm050 = &bmc055_data->mag.device;

	struct {
		BMM050_S16 raw_dataX;
		BMM050_S16 raw_dataY;
		BMM050_S16 raw_dataZ;
		BMM050_U16 raw_dataR;
	} raw_dataXYZ;

	if (i2c_client == BMM050_NULL) {
		comres = E_BMM050_NULL_PTR;
	} else {
		comres = bmm_i2c_read(i2c_client,
				      BMM050_DATAX_LSB, a_data_u8r, 8);

		/* Reading data for X axis */
		a_data_u8r[0] = BMM050_GET_BITSLICE(a_data_u8r[0],
						    BMM050_DATAX_LSB_VALUEX);
		raw_dataXYZ.raw_dataX = (BMM050_S16) ((((BMM050_S16)
							((signed char)
							 a_data_u8r[1])) <<
						       SHIFT_LEFT_5_POSITION) |
						      a_data_u8r[0]);

		/* Reading data for Y axis */
		a_data_u8r[2] = BMM050_GET_BITSLICE(a_data_u8r[2],
						    BMM050_DATAY_LSB_VALUEY);
		raw_dataXYZ.raw_dataY = (BMM050_S16) ((((BMM050_S16)
							((signed char)
							 a_data_u8r[3])) <<
						       SHIFT_LEFT_5_POSITION) |
						      a_data_u8r[2]);

		/* Reading data for Z axis */
		a_data_u8r[4] = BMM050_GET_BITSLICE(a_data_u8r[4],
						    BMM050_DATAZ_LSB_VALUEZ);
		raw_dataXYZ.raw_dataZ = (BMM050_S16) ((((BMM050_S16)
							((signed char)
							 a_data_u8r[5])) <<
						       SHIFT_LEFT_7_POSITION) |
						      a_data_u8r[4]);

		/* Reading data for Resistance */
		a_data_u8r[6] = BMM050_GET_BITSLICE(a_data_u8r[6],
						    BMM050_R_LSB_VALUE);
		raw_dataXYZ.raw_dataR = (BMM050_U16) ((((BMM050_U16)
							a_data_u8r[7]) <<
						       SHIFT_LEFT_6_POSITION) |
						      a_data_u8r[6]);

		/* Compensation for X axis */
		mdata->datax = bmm050_compensate_X(pbmm050,
						   raw_dataXYZ.raw_dataX,
						   raw_dataXYZ.raw_dataR);

		/* Compensation for Y axis */
		mdata->datay = bmm050_compensate_Y(pbmm050,
						   raw_dataXYZ.raw_dataY,
						   raw_dataXYZ.raw_dataR);

		/* Compensation for Z axis */
		mdata->dataz = bmm050_compensate_Z(pbmm050,
						   raw_dataXYZ.raw_dataZ,
						   raw_dataXYZ.raw_dataR);

		if ((mdata->datax == BMM050_OVERFLOW_OUTPUT) ||
		    (mdata->datay == BMM050_OVERFLOW_OUTPUT) ||
		    (mdata->dataz == BMM050_OVERFLOW_OUTPUT))
			comres = -5;	/* return -EIO if overflow */
	}
	return comres;
}

/* static int bmm050_init(struct bmm050 *bmm050) */
static int bmm050_init(struct bmm050_data *pbmm050_data)
{
	int comres = 0;
	unsigned char a_data_u8r[2];
	struct bmc055_data *bmc055_data = container_of(pbmm050_data,
						       struct bmc055_data, mag);

	/* set device from suspend into sleep mode */
	bmm050_set_powermode(bmc055_data->bmc055_client, BMM050_ON);

	/* wait two millisecond for bmc to settle */
	bmm_delay(BMM050_DELAY_SETTLING_TIME);

	/*Read CHIP_ID and REv. info */
	comres = bmm_i2c_read(bmc055_data->bmc055_client,
			      BMM050_CHIP_ID, a_data_u8r, 2);

	/* Function to initialise trim values */
	bmm050_init_trim_registers(pbmm050_data);
	bmm050_set_powermode(bmc055_data->bmc055_client, BMM050_OFF);

	return comres;
}

static int bmm_setpresets_data(struct bmm050_data *data, unsigned long period)
{
	unsigned long tmp;

	period = max(period, 10000000UL);

	if (period >= 160000000)	/* 160ms */
		tmp = 160000000;
	else if (period >= 80000000)	/* 80ms */
		tmp = 80000000;
	else if (period >= 40000000)	/* 40ms */
		tmp = 40000000;
	else if (period >= 20000000)	/* 20ms(50Hz), most common usage */
		tmp = 20000000;
	else if (period >= 10000000)	/* 10ms(100Hz) */
		tmp = 10000000;
	else if (period >= 5000000)	/* 5ms */
		tmp = 5000000;

	atomic_set(&data->delay, tmp);
	switch (tmp) {
	case 5000000:
		data->rept_xy = 2;
		data->rept_z = 4;
		break;
	case 10000000:
		data->rept_xy = 4;
		data->rept_z = 14;
		break;
	case 20000000:
		data->rept_xy = 7;
		data->rept_z = 26;
		break;
	case 40000000:
		data->rept_xy = 18;
		data->rept_z = 57;
		break;
	case 80000000:
	case 160000000:
		data->rept_xy = 23;
		data->rept_z = 82;
		break;
	default:
		BUG();
	}

	return 0;
}

static int bmm_setpresets_reg(struct bmm050_data *data)
{
	int err = 0;
	struct bmc055_data *bmc055_data = container_of(data,
						       struct bmc055_data, mag);

	err = bmm050_set_repetitions_XY(bmc055_data->bmc055_client,
					data->rept_xy);
	if (err)
		return err;

	mdelay(BMM_I2C_WRITE_DELAY_TIME);
	err = bmm050_set_repetitions_Z(bmc055_data->bmc055_client,
				       data->rept_z);
	if (err)
		return err;

	mdelay(BMM_I2C_WRITE_DELAY_TIME);

	return 0;
}

static int bmm_check_chip_id(struct i2c_client *client)
{
	int err = 0;
	u8 chip_id = 0;

	bmm_i2c_read(client, BMM_REG_NAME(CHIP_ID), &chip_id, 1);
	PINFO("read chip id result: %#x", chip_id);

	if ((chip_id & 0xff) != SENSOR_CHIP_ID_BMM)
		err = -1;

	return err;
}

static void bmm_scale_output(struct bmm050_mdata *value,
			     int *xmag, int *ymag, int *zmag)
{
	/* The bmm050 have a fixed measure range of +/-10Ga */
	/* 13bits for X and Y, 15bits for Z */
	*xmag = value->datax / 2;
	*ymag = value->datay / 2;
	*zmag = value->dataz / 2;
}

static void bmm_delay(u32 msec)
{
	mdelay(msec);
}

static void bmm_dump_reg(struct i2c_client *client)
{
	int i;
	u8 dbg_buf[64];
	u8 dbg_buf_str[64 * 3 + 1] = "";

	for (i = 0; i < BYTES_PER_LINE; i++) {
		dbg_buf[i] = i;
		sprintf(dbg_buf_str + i * 3, "%02x%c",
			dbg_buf[i],
			(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	printk(KERN_DEBUG "%s\n", dbg_buf_str);

	bmm_i2c_read(client, BMM_REG_NAME(CHIP_ID), dbg_buf, 19);
	for (i = 0; i < 19; i++) {
		sprintf(dbg_buf_str + i * 3, "%02x%c",
			dbg_buf[i],
			(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	printk(KERN_DEBUG "%s\n", dbg_buf_str);
}

static int bmm_wakeup(struct i2c_client *client)
{
	int err = 0;
	int try_times = BMM_MAX_RETRY_WAKEUP;
	const u8 value = 0x01;
	u8 dummy;

	PINFO("waking up the chip...");

	while (try_times) {
		err = bmm_i2c_write(client,
				    BMM_REG_NAME(POWER_CNTL), (u8 *) &value,
				    1);
		mdelay(BMM_I2C_WRITE_DELAY_TIME);
		dummy = 0;
		err = bmm_i2c_read(client, BMM_REG_NAME(POWER_CNTL), &dummy, 1);
		if (value == dummy)
			break;

		try_times--;
	}

	PINFO("wake up result: %s, tried times: %d",
	      (try_times > 0) ? "succeed" : "fail",
	      BMM_MAX_RETRY_WAKEUP - try_times + 1);

	err = (try_times > 0) ? 0 : -1;

	return err;
}

/*	i2c read routine for API*/
static char bmm_i2c_read(struct i2c_client *client, u8 reg_addr,
			 u8 *data, u8 len)
{
	s32 dummy;
	if ((NULL == client) || !len)
		return -1;

	if (len == 1) {
		dummy = i2c_smbus_read_byte_data(client, reg_addr);
		if (dummy < 0) {
			PERR("i2c bus read error");
			return -1;
		}
		*data = (u8) (dummy & 0xff);

	} else if (len > 1) {
		dummy = i2c_smbus_read_i2c_block_data(client, reg_addr,
						      len, data);
		if (dummy < 0) {
			PERR("i2c bus block read error");
			return -1;
		}
	}
	return 0;
}

/*	i2c write routine for */
static char bmm_i2c_write(struct i2c_client *client, u8 reg_addr,
			  u8 *data, u8 len)
{
	s32 dummy;

	if (NULL == client)
		return -1;

	while (0 != len--) {
		dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
		reg_addr++;
		data++;
		if (dummy < 0) {
			PERR("error writing i2c bus");
			return -1;
		}
	}
	return 0;
}

/* this function exists for optimization of speed,
 * because it is frequently called */
static inline int bmm_set_forced_mode(struct i2c_client *client)
{
	int err = 0;

	/* FORCED_MODE */
	const u8 value = 0x02;
	err = bmm_i2c_write(client, BMM_REG_NAME(CONTROL), (u8 *) &value, 1);

	return err;
}

static void bmm_work_func(struct work_struct *work)
{
	int err = 0;
	struct bmm050_data *client_data =
	    container_of((struct delayed_work *)work,
			 struct bmm050_data, work);
	struct bmc055_data *bmc055_data = container_of(client_data,
						       struct bmc055_data, mag);
	struct bmc055_platform_data *pdata =
	    bmc055_data->bmc055_client->dev.platform_data;

	unsigned long delay_ms = atomic_read(&client_data->delay) / 1000000;
	int xmag, ymag, zmag;

	mutex_lock(&bmc055_data->bmc055_mutex);

	err = bmm050_read_mdataXYZ(client_data, &client_data->value);
	if (err) {
		PERR("bmm050: data invalid, err=%d\n", err);
	} else {
		bmm_scale_output(&client_data->value, &xmag, &ymag, &zmag);

		input_report_abs(client_data->input, pdata->mag_xcode,
				 xmag * pdata->mag_xdir);
		input_report_abs(client_data->input, pdata->mag_ycode,
				 ymag * pdata->mag_ydir);
		input_report_abs(client_data->input, pdata->mag_zcode,
				 zmag * pdata->mag_zdir);

		input_always_sync(client_data->input);

		PDEBUG("bmm work func, %d, %d, %d, delay=%ld\n",
		       client_data->value.datax,
		       client_data->value.datay,
		       client_data->value.dataz, delay_ms);
	}

	if (BMM050_NORMAL_MODE != client_data->op_mode)
		bmm_set_forced_mode(bmc055_data->bmc055_client);

	schedule_delayed_work(&client_data->work, msecs_to_jiffies(delay_ms));
	mutex_unlock(&bmc055_data->bmc055_mutex);
}

static ssize_t bmm_show_op_mode(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	u8 op_mode = 0xff;
	u8 power_mode;
	struct input_dev *input = to_input_dev(dev);
	struct bmm050_data *client_data = input_get_drvdata(input);
	struct bmc055_data *bmc055_data = container_of(client_data,
						       struct bmc055_data, mag);

	mutex_lock(&bmc055_data->bmc055_mutex);
	bmm050_get_powermode(bmc055_data->bmc055_client, &power_mode);
	if (power_mode) {
		bmm050_get_functional_state(bmc055_data->bmc055_client,
					    &op_mode);
	} else {
		op_mode = BMM050_SUSPEND_MODE;
	}
	mutex_unlock(&bmc055_data->bmc055_mutex);

	PDEBUG("op_mode: %d", op_mode);

	ret = sprintf(buf, "%d\n", op_mode);

	return ret;
}

static inline int bmm_get_op_mode_idx(u8 op_mode)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(op_mode_maps); i++) {
		if (op_mode_maps[i].op_mode == op_mode)
			break;
	}

	if (i < ARRAY_SIZE(op_mode_maps))
		return i;
	else
		return -1;
}

static ssize_t bmm_store_op_mode(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	int err = 0;
	int i;
	long op_mode;
	struct input_dev *input = to_input_dev(dev);
	struct bmm050_data *client_data = input_get_drvdata(input);
	struct bmc055_data *bmc055_data = container_of(client_data,
						       struct bmc055_data, mag);

	err = strict_strtoul(buf, 10, &op_mode);
	if (err)
		return err;

	mutex_lock(&bmc055_data->bmc055_mutex);

	i = bmm_get_op_mode_idx(op_mode);

	if (i != -1) {
		if (op_mode != client_data->op_mode) {
			if (BMM050_FORCED_MODE == op_mode) {
				/* special treat of forced mode
				 * for optimization */
				err =
				    bmm_set_forced_mode(bmc055_data->
							bmc055_client);
				PINFO("force_mode:%d, err=%d\n", (int)op_mode,
				      err);
			} else {
				err =
				    bmm050_set_functional_state(bmc055_data->
								bmc055_client,
								op_mode);
				PINFO("normal_mode:%d, err=%d\n", (int)op_mode,
				      err);
			}

			if (!err) {
				if (BMM050_FORCED_MODE == op_mode)
					client_data->op_mode =
					    BMM_OP_MODE_UNKNOWN;
				else
					client_data->op_mode = op_mode;
			}
		}
	} else {
		err = -EINVAL;
	}

	mutex_unlock(&bmc055_data->bmc055_mutex);

	if (err)
		return err;
	else
		return count;
}

static ssize_t bmm_show_rept_xy(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmm050_data *client_data = input_get_drvdata(input);
	struct bmc055_data *bmc055_data = container_of(client_data,
						       struct bmc055_data, mag);

	int err;
	u8 power_mode;

	mutex_lock(&bmc055_data->bmc055_mutex);
	bmm050_get_powermode(bmc055_data->bmc055_client, &power_mode);
	if (power_mode) {
		err = bmm050_get_repetitions_XY(bmc055_data->bmc055_client,
						&data);
	} else {
		err = -EIO;
	}

	mutex_unlock(&bmc055_data->bmc055_mutex);

	if (err)
		return err;

	return sprintf(buf, "%d\n", data);
}

static ssize_t bmm_store_rept_xy(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	unsigned long tmp = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmm050_data *client_data = input_get_drvdata(input);
	struct bmc055_data *bmc055_data = container_of(client_data,
						       struct bmc055_data, mag);

	int err;
	u8 data;
	u8 power_mode;

	err = strict_strtoul(buf, 10, &tmp);
	if (err)
		return err;

	if (tmp > 255)
		return -EINVAL;

	data = (unsigned char)tmp;

	mutex_lock(&bmc055_data->bmc055_mutex);
	bmm050_get_powermode(bmc055_data->bmc055_client, &power_mode);
	if (power_mode) {
		err = bmm050_set_repetitions_XY(bmc055_data->bmc055_client,
						data);
		if (!err) {
			mdelay(BMM_I2C_WRITE_DELAY_TIME);
			client_data->rept_xy = data;
		}
	} else {
		err = -EIO;
	}
	mutex_unlock(&bmc055_data->bmc055_mutex);

	if (err)
		return err;

	return count;
}

static ssize_t bmm_show_rept_z(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmm050_data *client_data = input_get_drvdata(input);
	struct bmc055_data *bmc055_data = container_of(client_data,
						       struct bmc055_data, mag);

	int err;
	u8 power_mode;

	mutex_lock(&bmc055_data->bmc055_mutex);
	bmm050_get_powermode(bmc055_data->bmc055_client, &power_mode);
	if (power_mode) {
		err = bmm050_get_repetitions_Z(bmc055_data->bmc055_client,
					       &data);
	} else {
		err = -EIO;
	}
	mutex_unlock(&bmc055_data->bmc055_mutex);

	if (err)
		return err;

	return sprintf(buf, "%d\n", data);
}

static ssize_t bmm_store_rept_z(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long tmp = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmm050_data *client_data = input_get_drvdata(input);
	struct bmc055_data *bmc055_data = container_of(client_data,
						       struct bmc055_data, mag);

	int err;
	u8 data;
	u8 power_mode;

	err = strict_strtoul(buf, 10, &tmp);
	if (err)
		return err;

	if (tmp > 255)
		return -EINVAL;

	data = (unsigned char)tmp;

	mutex_lock(&bmc055_data->bmc055_mutex);
	bmm050_get_powermode(bmc055_data->bmc055_client, &power_mode);
	if (power_mode) {
		err = bmm050_set_repetitions_Z(bmc055_data->bmc055_client,
					       data);
		if (!err) {
			mdelay(BMM_I2C_WRITE_DELAY_TIME);
			client_data->rept_z = data;
		}
	} else {
		err = -EIO;
	}
	mutex_unlock(&bmc055_data->bmc055_mutex);

	if (err)
		return err;

	return count;
}

static ssize_t bmm_show_value(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmm050_data *client_data = input_get_drvdata(input);
	struct bmc055_data *bmc055_data = container_of(client_data,
						       struct bmc055_data, mag);

	int count;

	mutex_lock(&bmc055_data->bmc055_mutex);
	bmm050_read_mdataXYZ(client_data, &client_data->value);
	mutex_unlock(&bmc055_data->bmc055_mutex);

	count = sprintf(buf, "%hd %hd %hd\n",
			client_data->value.datax,
			client_data->value.datay, client_data->value.dataz);

	return count;
}

static ssize_t bmm_show_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmm050_data *client_data = input_get_drvdata(input);
	struct bmc055_data *bmc055_data = container_of(client_data,
						       struct bmc055_data, mag);
	int err;

	mutex_lock(&bmc055_data->bmc055_mutex);
	err = sprintf(buf, "%d\n", client_data->enable);
	mutex_unlock(&bmc055_data->bmc055_mutex);
	return err;
}

static ssize_t bmm_store_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmm050_data *client_data = input_get_drvdata(input);
	struct bmc055_data *bmc055_data = container_of(client_data,
						       struct bmc055_data, mag);
	unsigned long delay_ms = atomic_read(&client_data->delay) / 1000000;

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	data = data ? 1 : 0;
	mutex_lock(&bmc055_data->bmc055_mutex);
	if (data != client_data->enable) {
		if (data) {
			if (BMM050_NORMAL_MODE != client_data->op_mode)
				bmm_set_forced_mode(bmc055_data->bmc055_client);

			schedule_delayed_work(&client_data->work,
					      msecs_to_jiffies(delay_ms));
		} else {
			cancel_delayed_work(&client_data->work);
		}

		client_data->enable = data;
	}
	mutex_unlock(&bmc055_data->bmc055_mutex);

	return count;
}

static ssize_t bmm_show_delay(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmm050_data *client_data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", atomic_read(&client_data->delay));

}

static ssize_t bmm_store_delay(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmm050_data *client_data = input_get_drvdata(input);
	struct bmc055_data *bmc055_data = container_of(client_data,
						       struct bmc055_data, mag);

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	if (data <= 0) {
		err = -EINVAL;
		return err;
	}

	if (data < BMM_DELAY_MIN)
		data = BMM_DELAY_MIN;

	mutex_lock(&bmc055_data->bmc055_mutex);
	bmm_setpresets_data(client_data, data);
	err = bmm_setpresets_reg(client_data);
	mutex_unlock(&bmc055_data->bmc055_mutex);

	if (err)
		return -EIO;

	return count;
}

BMC055_ATTR(mag, op_mode, S_IRUGO | S_IWUSR,
	    bmm_show_op_mode, bmm_store_op_mode);
BMC055_ATTR(mag, rept_xy, S_IRUGO | S_IWUSR,
	    bmm_show_rept_xy, bmm_store_rept_xy);
BMC055_ATTR(mag, rept_z, S_IRUGO | S_IWUSR, bmm_show_rept_z, bmm_store_rept_z);
BMC055_ATTR(mag, value, S_IRUGO, bmm_show_value, NULL);
BMC055_ATTR(mag, enable, S_IRUGO | S_IWUSR, bmm_show_enable, bmm_store_enable);
BMC055_ATTR(mag, poll_delay, S_IRUGO | S_IWUSR,
	    bmm_show_delay, bmm_store_delay);

static struct attribute *bmm_attributes[] = {
	&bmc055_mag_op_mode_attr.attr,
	&bmc055_mag_rept_xy_attr.attr,
	&bmc055_mag_rept_z_attr.attr,
	&bmc055_mag_value_attr.attr,
	&bmc055_mag_enable_attr.attr,
	&bmc055_mag_poll_delay_attr.attr,
	NULL
};

static struct attribute_group bmm_attribute_group = {
	.attrs = bmm_attributes
};

static int open_device(struct bmm050_data *client_data)
{
	int err = 0;
	struct bmc055_data *bmc055_data = container_of(client_data,
						       struct bmc055_data, mag);

	err = bmm050_set_functional_state(bmc055_data->bmc055_client,
					  BMM050_SLEEP_MODE);
	if (err) {
		PERR("fail to set state of %s", "bmm050");
		return -EIO;
	}

	/* init the basic registers */
	err = bmm_setpresets_reg(client_data);
	if (err) {
		bmm050_set_functional_state(bmc055_data->bmc055_client,
					    BMM050_SUSPEND_MODE);
		PERR("fail to init registers of %s", "bmm050");
		return -EIO;
	}

	client_data->op_mode = BMM050_SLEEP_MODE;
	client_data->opened = true;

	return err;
}

static int bmm_open(struct input_dev *dev)
{
	int err = 0;
	struct bmm050_data *client_data = input_get_drvdata(dev);
	struct bmc055_data *bmc055_data = container_of(client_data,
						       struct bmc055_data, mag);

	mutex_lock(&bmc055_data->bmc055_mutex);
	err = open_device(client_data);
	mutex_unlock(&bmc055_data->bmc055_mutex);

	return err;
}

static void bmm_close(struct input_dev *dev)
{
	struct bmm050_data *client_data = input_get_drvdata(dev);
	struct bmc055_data *bmc055_data = container_of(client_data,
						       struct bmc055_data, mag);

	mutex_lock(&bmc055_data->bmc055_mutex);
	bmm050_set_functional_state(bmc055_data->bmc055_client,
				    BMM050_SUSPEND_MODE);
	client_data->op_mode = BMM050_SUSPEND_MODE;
	client_data->opened = false;
	mutex_unlock(&bmc055_data->bmc055_mutex);
}

static int bmm_input_init(struct bmm050_data *client_data)
{
	struct input_dev *dev;
	int err = 0;
	struct bmc055_data *bmc055_data = container_of(client_data,
						       struct bmc055_data, mag);
	struct bmc055_platform_data *pdata =
	    bmc055_data->bmc055_client->dev.platform_data;

	dev = input_allocate_device();
	if (NULL == dev)
		return -ENOMEM;

	dev->name = "compass";
	dev->id.bustype = BUS_I2C;
	dev->open = bmm_open;
	dev->close = bmm_close;

	input_set_capability(dev, EV_ABS, pdata->mag_xcode);
	input_set_abs_params(dev, pdata->mag_xcode,
			     MAG_VALUE_MIN, MAG_VALUE_MAX, 1, 0);

	input_set_capability(dev, EV_ABS, pdata->mag_ycode);
	input_set_abs_params(dev, pdata->mag_ycode,
			     MAG_VALUE_MIN, MAG_VALUE_MAX, 1, 0);

	input_set_capability(dev, EV_ABS, pdata->mag_zcode);
	input_set_abs_params(dev, pdata->mag_zcode,
			     MAG_VALUE_MIN, MAG_VALUE_MAX, 1, 0);

	input_set_drvdata(dev, client_data);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}

	dev->phys = kobject_get_path(&dev->dev.kobj, GFP_KERNEL);
	if (dev->phys == NULL) {
		dev_err(&dev->dev, "fail to get mag input sysfs path.");
		input_unregister_device(dev);
		return -ENOMEM;
	}

	client_data->input = dev;

	return 0;
}

static void bmm_input_destroy(struct bmm050_data *client_data)
{
	struct input_dev *dev = client_data->input;

	kfree(dev->phys);
	input_unregister_device(dev);
}

int bmm_local_init(struct bmc055_data *pbmc055)
{
	int err = 0;
	int dummy;
	struct i2c_client *client = pbmc055->bmc055_client;
	struct bmm050_data *client_data = &pbmc055->mag;
	struct bmc055_platform_data *pdata = client->dev.platform_data;

	PINFO("function entrance");

	/* wake up the chip */
	dummy = bmm_wakeup(client);
	if (dummy < 0) {
		PERR("Cannot wake up %s, I2C xfer error", "bmm050");
		err = -EIO;
		goto exit_err_clean;
	}

	PINFO("register dump after waking up");
	bmm_dump_reg(client);
	/* check chip id */
	err = bmm_check_chip_id(client);
	if (!err) {
		PNOTICE("Bosch Sensortec Device %s detected", "bmm050");
	} else {
		PERR("Bosch Sensortec Device not found, chip id mismatch");
		err = -ENODEV;
		goto exit_err_clean;
	}

	client_data->client = client;
	client_data->enable = 0;
	/* now it's power on which is considered as resuming from suspend */
	bmm_setpresets_data(client_data, pdata->mag_period);

	/* input device init */
	err = bmm_input_init(client_data);
	if (err < 0)
		goto exit_err_clean;

	/* sysfs node creation */
	err = sysfs_create_group(&client_data->input->dev.kobj,
				 &bmm_attribute_group);
	if (err < 0)
		goto exit_err_sysfs;

	/* workqueue init */
	INIT_DELAYED_WORK(&client_data->work, bmm_work_func);

	/* h/w init */
	bmm050_init(client_data);

	/* bmm_dump_reg(client); */

	PDEBUG("trimming_reg x1: %d y1: %d x2: %d y2: %d xy1: %d xy2: %d",
	       client_data->device.dig_x1,
	       client_data->device.dig_y1,
	       client_data->device.dig_x2,
	       client_data->device.dig_y2,
	       client_data->device.dig_xy1, client_data->device.dig_xy2);

	PDEBUG("trimming_reg z1: %d z2: %d z3: %d z4: %d xyz1: %d",
	       client_data->device.dig_z1,
	       client_data->device.dig_z2,
	       client_data->device.dig_z3,
	       client_data->device.dig_z4, client_data->device.dig_xyz1);

	PNOTICE("sensor %s probed successfully", "bmm050");

	PDEBUG("i2c_client: %p client_data: %p i2c_device: %p input: %p",
	       client, client_data, &client->dev, client_data->input);

	return 0;

exit_err_sysfs:
	if (err)
		bmm_input_destroy(client_data);

exit_err_clean:
	return err;
}

#ifdef CONFIG_BMM050_HAS_EARLYSUSPEND
static int bmm_pre_suspend(struct bmm050_data *client_data)
{
	int err = 0;
	struct bmc055_data *bmc055_data = container_of(client_data,
						       struct bmc055_data, mag);
	PDEBUG("function entrance");

	mutex_lock(&bmc055_data->bmc055_mutex);
	if (client_data->enable) {
		cancel_delayed_work(&client_data->work);
		PDEBUG("cancel work");
	}
	mutex_unlock(&bmc055_data->bmc055_mutex);

	return err;
}

static int bmm_post_resume(struct bmm050_data *client_data)
{
	int err = 0;
	struct bmc055_data *bmc055_data = container_of(client_data,
						       struct bmc055_data, mag);
	unsigned long delay_ms = atomic_read(&client_data->delay) / 1000000;

	mutex_lock(&bmc055_data->bmc055_mutex);
	if (client_data->enable) {
		schedule_delayed_work(&client_data->work,
				      msecs_to_jiffies(delay_ms));
	}
	mutex_unlock(&bmc055_data->bmc055_mutex);

	return err;
}

void bmm_early_suspend(struct bmm050_data *client_data)
{
	int err = 0;
	u8 power_mode;
	struct bmc055_data *bmc055_data = container_of(client_data,
						       struct bmc055_data, mag);

	PDEBUG("function entrance");

	mutex_lock(&bmc055_data->bmc055_mutex);
	bmm050_get_powermode(bmc055_data->bmc055_client, &power_mode);
	if (power_mode) {
		err = bmm_pre_suspend(client_data);
		err = bmm050_set_functional_state(bmc055_data->bmc055_client,
						  BMM050_SUSPEND_MODE);
	}
	mutex_unlock(&bmc055_data->bmc055_mutex);

}

void bmm_late_resume(struct bmm050_data *client_data)
{
	int err = 0;
	struct bmc055_data *bmc055_data = container_of(client_data,
						       struct bmc055_data, mag);

	PDEBUG("function entrance");

	mutex_lock(&bmc055_data->bmc055_mutex);

	/* err = bmm_restore_hw_cfg(bmc055_client, client_data); */
	/* post resume operation */
	bmm_post_resume(client_data);

	mutex_unlock(&bmc055_data->bmc055_mutex);
}
#else
void bmm_early_suspend(struct bmm050_data *client_data)
{

}

void bmm_late_resume(struct bmm050_data *client_data)
{

}
#endif

int bmm_remove(struct bmm050_data *client_data)
{
	int err = 0;
	struct bmc055_data *bmc055_data = container_of(client_data,
						       struct bmc055_data, mag);
	if (NULL != client_data) {

		mutex_lock(&bmc055_data->bmc055_mutex);
		if (BMM050_NORMAL_MODE == client_data->op_mode) {
			cancel_delayed_work(&client_data->work);
			PDEBUG("cancel work");
		}
		err = bmm050_set_functional_state(bmc055_data->bmc055_client,
						  BMM050_SUSPEND_MODE);
		mutex_unlock(&bmc055_data->bmc055_mutex);
		mdelay(BMM_I2C_WRITE_DELAY_TIME);

		sysfs_remove_group(&client_data->input->dev.kobj,
				   &bmm_attribute_group);
		bmm_input_destroy(client_data);
	}

	return err;
}

/*------------------------bma255_driver--------------------------*/
#define BMA255_NVM_SIZE  27	/* 26 regs and 1 crc */

static const u8 bma255_valid_range[] = {
	bma255_RANGE_2G,
	bma255_RANGE_4G,
	bma255_RANGE_8G,
	bma255_RANGE_16G,
};

static const u8 bma255_valid_bw[] = {
	bma255_BW_7_81HZ,
	bma255_BW_15_63HZ,
	bma255_BW_31_25HZ,
	bma255_BW_62_50HZ,
	bma255_BW_125HZ,
	bma255_BW_250HZ,
	bma255_BW_500HZ,
	bma255_BW_1000HZ,
};

static int bma255_smbus_read_byte(struct i2c_client *client,
				  unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_read_byte_data(client, reg_addr);
	if (dummy < 0)
		return -1;
	*data = dummy & 0x000000ff;

	return 0;
}

static int bma255_smbus_write_byte(struct i2c_client *client,
				   unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
	if (dummy < 0)
		return -1;
	return 0;
}

static int bma255_smbus_read_byte_block(struct i2c_client *client,
					unsigned char reg_addr,
					unsigned char *data, unsigned char len)
{
	s32 dummy;
	dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
	if (dummy < 0)
		return -1;
	return 0;
}

static int bma255_check_crc8(unsigned char *mem_addr, unsigned char block_size)
{
	unsigned char crc_reg = 0xFF;
	unsigned char mem_byte;
	unsigned char bit_no;
	unsigned char crc_read = mem_addr[block_size - 1];

	block_size -= 1;
	while (block_size) {
		mem_byte = *mem_addr;
		for (bit_no = 0; bit_no < 8; bit_no++) {
			if ((crc_reg ^ mem_byte) & 0x80) {
				crc_reg = (crc_reg << 1) ^ 0x11D;
			} else {
				crc_reg <<= 1;
			}
			mem_byte <<= 1;
		}
		block_size--;
		mem_addr++;
	}

	crc_reg = ~crc_reg;
	return (crc_reg == crc_read) ? 0 : -1;
}

static u8 bma255_get_bw_code(unsigned long period)
{
	switch (period) {
	case 160000000:
		return bma255_BW_7_81HZ;
	case 80000000:
		return bma255_BW_15_63HZ;
	case 40000000:
		return bma255_BW_31_25HZ;
	case 20000000:
		return bma255_BW_62_50HZ;
	case 10000000:
		return bma255_BW_125HZ;
	case 5000000:
		return bma255_BW_250HZ;
	case 2500000:
		return bma255_BW_500HZ;
	case 1250000:
		return bma255_BW_1000HZ;
	default:
		BUG();
	}
}

static u8 bma255_get_range_code(unsigned long scale)
{
	switch (scale) {
	case 2000:
		return bma255_RANGE_2G;
	case 4000:
		return bma255_RANGE_4G;
	case 8000:
		return bma255_RANGE_8G;
	case 16000:
		return bma255_RANGE_16G;
	default:
		BUG();
	}
}

static unsigned long bma255_adjust_period(unsigned long period)
{
	period = max(period, 10000000UL);

/* The bandwidth of the bma255 hardware is not the same serial with Apps
   and BMM050, so we need to adjust it */
	if (period >= 160000000)	/* 128ms(7.81Hz) to 160ms */
		return 160000000;
	else if (period >= 80000000)	/* 64ms(15.63Hz) to 80ms */
		return 80000000;
	else if (period >= 40000000)	/* 32ms(31.25Hz) to 40ms */
		return 40000000;
	else if (period >= 20000000)	/* 16ms(62.5Hz) to 20ms */
		return 20000000;
	else if (period >= 10000000)	/* 8ms(125Hz) to 10ms */
		return 10000000;
	else if (period >= 5000000)	/* 4ms(250Hz) to 5ms */
		return 5000000;
	else if (period >= 2500000)	/* 2ms(500Hz) to 2.5ms */
		return 2500000;
	else if (period >= 1250000)	/* 1ms(1000Hz) to 1.25ms */
		return 1250000;

	return 1250000;		/* out of range return as fast as we can */
}

static unsigned long bma255_adjust_scale(unsigned long scale)
{
	if (scale == 0)		/* auto scale */
		return 2000;
	else if (scale <= 2000)	/* 2G */
		return 2000;
	else if (scale <= 4000)	/* 4G */
		return 4000;
	else if (scale <= 8000)	/* 8G */
		return 8000;
	else if (scale <= 16000)	/* 16G */
		return 16000;

	return 16000;		/* out of range, return as large as we can */
}

static unsigned long bma255_get_sensitivity(unsigned long scale)
{
	/* to be compatible with HAL factor */
	switch (scale) {
	case 2000:
		return 1;
	case 4000:
		return 2;
	case 8000:
		return 4;
	case 16000:
		return 8;
	default:
		BUG();
	}
}

static int bma255_scale_output(s16 output, unsigned long scale)
{
	return (int)bma255_get_sensitivity(scale) * output;
}

static int bma255_set_mode(struct i2c_client *client, unsigned char mode)
{
	int comres = 0;
	unsigned char data1 = 0;

	if (client == NULL) {
		comres = -1;
	} else {
		if (mode < 3) {
			comres = bma255_smbus_read_byte(client,
							bma255_EN_LOW_POWER__REG,
							&data1);
			switch (mode) {
			case bma255_MODE_NORMAL:
				data1 = bma255_SET_BITSLICE(data1,
							    bma255_EN_LOW_POWER,
							    0);
				data1 =
				    bma255_SET_BITSLICE(data1,
							bma255_EN_SUSPEND, 0);
				break;
			case bma255_MODE_LOWPOWER:
				data1 = bma255_SET_BITSLICE(data1,
							    bma255_EN_LOW_POWER,
							    1);
				data1 =
				    bma255_SET_BITSLICE(data1,
							bma255_EN_SUSPEND, 0);
				break;
			case bma255_MODE_SUSPEND:
				data1 = bma255_SET_BITSLICE(data1,
							    bma255_EN_LOW_POWER,
							    0);
				data1 =
				    bma255_SET_BITSLICE(data1,
							bma255_EN_SUSPEND, 1);
				break;
			default:
				break;
			}

			comres += bma255_smbus_write_byte(client,
							  bma255_EN_LOW_POWER__REG,
							  &data1);
		} else {
			comres = -1;
		}
	}

	return comres;
}

static int bma255_get_mode(struct i2c_client *client, unsigned char *mode)
{
	int comres = 0;

	if (client == NULL) {
		comres = -1;
	} else {
		comres = bma255_smbus_read_byte(client,
						bma255_EN_LOW_POWER__REG, mode);
		*mode = (*mode) >> 6;
	}

	return comres;
}

static int bma255_set_range(struct i2c_client *client, unsigned char range)
{
	int comres = 0;
	unsigned char data1 = 0;
	int i;

	if (client == NULL) {
		comres = -1;
	} else {
		for (i = 0; i < ARRAY_SIZE(bma255_valid_range); i++) {
			if (bma255_valid_range[i] == range)
				break;
		}

		if (ARRAY_SIZE(bma255_valid_range) > i) {
			comres = bma255_smbus_read_byte(client,
							bma255_RANGE_SEL_REG,
							&data1);

			data1 = bma255_SET_BITSLICE(data1,
						    bma255_RANGE_SEL, range);

			comres += bma255_smbus_write_byte(client,
							  bma255_RANGE_SEL_REG,
							  &data1);
		} else {
			comres = -EINVAL;
		}
	}

	return comres;
}

static int bma255_get_range(struct i2c_client *client, unsigned char *range)
{
	int comres = 0;
	unsigned char data = 0;

	if (client == NULL) {
		comres = -1;
	} else {
		comres = bma255_smbus_read_byte(client, bma255_RANGE_SEL__REG,
						&data);
		data = bma255_GET_BITSLICE(data, bma255_RANGE_SEL);
		*range = data;
	}

	return comres;
}

static int bma255_set_bandwidth(struct i2c_client *client, unsigned char bw)
{
	int comres = 0;
	unsigned char data = 0;
	int i = 0;

	if (client == NULL) {
		comres = -1;
	} else {

		for (i = 0; i < ARRAY_SIZE(bma255_valid_bw); i++) {
			if (bma255_valid_bw[i] == bw)
				break;
		}

		if (ARRAY_SIZE(bma255_valid_bw) > i) {
			comres = bma255_smbus_read_byte(client,
							bma255_BANDWIDTH__REG,
							&data);
			data = bma255_SET_BITSLICE(data, bma255_BANDWIDTH, bw);
			comres += bma255_smbus_write_byte(client,
							  bma255_BANDWIDTH__REG,
							  &data);
		} else {
			comres = -EINVAL;
		}
	}

	return comres;
}

static int bma255_get_bandwidth(struct i2c_client *client, unsigned char *bw)
{
	int comres = 0;
	unsigned char data = 0;

	if (client == NULL) {
		comres = -1;
	} else {
		comres = bma255_smbus_read_byte(client, bma255_BANDWIDTH__REG,
						&data);
		data = bma255_GET_BITSLICE(data, bma255_BANDWIDTH);
		if (data < bma255_BW_7_81HZ)
			*bw = bma255_BW_7_81HZ;
		else if (data > bma255_BW_1000HZ)
			*bw = bma255_BW_1000HZ;
		else
			*bw = data;
	}

	return comres;
}

static int bma255_read_accel_xyz(struct i2c_client *client,
				 struct bma255acc *acc)
{
	int comres;
	unsigned char data[6];
	if (client == NULL) {
		comres = -1;
	} else {
		comres = bma255_smbus_read_byte_block(client,
						      bma255_ACC_X_LSB__REG,
						      data, 6);

		acc->x = bma255_GET_BITSLICE(data[0], bma255_ACC_X_LSB)
		    | (bma255_GET_BITSLICE(data[1], bma255_ACC_X_MSB)
		       << bma255_ACC_X_LSB__LEN);
		acc->y = bma255_GET_BITSLICE(data[2], bma255_ACC_Y_LSB)
		    | (bma255_GET_BITSLICE(data[3], bma255_ACC_Y_MSB)
		       << bma255_ACC_Y_LSB__LEN);
		acc->z = bma255_GET_BITSLICE(data[4], bma255_ACC_Z_LSB)
		    | (bma255_GET_BITSLICE(data[5], bma255_ACC_Z_MSB)
		       << bma255_ACC_Z_LSB__LEN);
	}

	return comres;
}

static void bma255_work_func(struct work_struct *work)
{
	struct bma255_data *bma255 = container_of((struct delayed_work *)work,
						  struct bma255_data, work);
	struct bma255acc acc;
	struct bmc055_data *bmc055_data = container_of(bma255,
						       struct bmc055_data, acc);
	struct bmc055_platform_data *pdata =
	    bmc055_data->bmc055_client->dev.platform_data;

	int err;
	unsigned long delay_ms = atomic_read(&bma255->delay) / 1000000;
	int xmg, ymg, zmg;

	mutex_lock(&bmc055_data->bmc055_mutex);
	err = bma255_read_accel_xyz(bmc055_data->bmc055_client, &acc);
	if (err < 0) {
		PERR("bma255: data invalid, err=%d", err);
	} else {
		acc.x /= 16;	/* trim the last 4 dummy bits */
		acc.y /= 16;
		acc.z /= 16;

		xmg = bma255_scale_output(acc.x, atomic_read(&bma255->range));
		ymg = bma255_scale_output(acc.y, atomic_read(&bma255->range));
		zmg = bma255_scale_output(acc.z, atomic_read(&bma255->range));

		input_report_abs(bma255->input, pdata->acc_xcode,
				 xmg * pdata->acc_xdir);
		input_report_abs(bma255->input, pdata->acc_ycode,
				 ymg * pdata->acc_ydir);
		input_report_abs(bma255->input, pdata->acc_zcode,
				 zmg * pdata->acc_zdir);
		input_always_sync(bma255->input);

		bma255->value = acc;
	}
	schedule_delayed_work(&bma255->work, msecs_to_jiffies(delay_ms));
	mutex_unlock(&bmc055_data->bmc055_mutex);
}

static ssize_t bma255_range_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	unsigned char data, range_code;
	unsigned long range;
	int error;
	struct input_dev *input = to_input_dev(dev);
	struct bma255_data *bma255 = input_get_drvdata(input);
	struct bmc055_data *bmc055_data = container_of(bma255,
						       struct bmc055_data, acc);

	mutex_lock(&bmc055_data->bmc055_mutex);
	range = atomic_read(&bma255->range);
	range_code = bma255_get_range_code(range);
	error = bma255_get_range(bmc055_data->bmc055_client, &data);
	mutex_unlock(&bmc055_data->bmc055_mutex);

	if (error < 0)
		return sprintf(buf, "Read error\n");

	if (range_code == data)
		return sprintf(buf, "%ld\n", range);
	else
		return sprintf(buf, "%d != %d\n", range_code, data);
}

static ssize_t bma255_range_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct input_dev *input = to_input_dev(dev);
	struct bma255_data *bma255 = input_get_drvdata(input);
	struct bmc055_data *bmc055_data = container_of(bma255,
						       struct bmc055_data, acc);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	mutex_lock(&bmc055_data->bmc055_mutex);
	atomic_set(&bma255->range, bma255_adjust_scale(data));
	data = bma255_get_range_code(bma255_adjust_scale(data));
	error =
	    bma255_set_range(bmc055_data->bmc055_client, (unsigned char)data);
	mutex_unlock(&bmc055_data->bmc055_mutex);

	if (error < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma255_bandwidth_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	unsigned char data;
	int error;
	struct input_dev *input = to_input_dev(dev);
	struct bma255_data *bma255 = input_get_drvdata(input);
	struct bmc055_data *bmc055_data = container_of(bma255,
						       struct bmc055_data, acc);

	mutex_lock(&bmc055_data->bmc055_mutex);
	error = bma255_get_bandwidth(bmc055_data->bmc055_client, &data);
	mutex_unlock(&bmc055_data->bmc055_mutex);

	if (error < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t bma255_bandwidth_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct input_dev *input = to_input_dev(dev);
	struct bma255_data *bma255 = input_get_drvdata(input);
	struct bmc055_data *bmc055_data = container_of(bma255,
						       struct bmc055_data, acc);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	mutex_lock(&bmc055_data->bmc055_mutex);
	error =
	    bma255_set_bandwidth(bmc055_data->bmc055_client,
				 (unsigned char)data);
	mutex_unlock(&bmc055_data->bmc055_mutex);

	if (error < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma255_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	int error;
	struct input_dev *input = to_input_dev(dev);
	struct bma255_data *bma255 = input_get_drvdata(input);
	struct bmc055_data *bmc055_data = container_of(bma255,
						       struct bmc055_data, acc);
	mutex_lock(&bmc055_data->bmc055_mutex);
	error = bma255_get_mode(bmc055_data->bmc055_client, &data);
	mutex_unlock(&bmc055_data->bmc055_mutex);

	if (error)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);
}

static ssize_t bma255_mode_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	unsigned long data = 0;
	int error;
	struct input_dev *input = to_input_dev(dev);
	struct bma255_data *bma255 = input_get_drvdata(input);
	struct bmc055_data *bmc055_data = container_of(bma255,
						       struct bmc055_data, acc);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	mutex_lock(&bmc055_data->bmc055_mutex);
	error =
	    bma255_set_mode(bmc055_data->bmc055_client, (unsigned char)data);
	mutex_unlock(&bmc055_data->bmc055_mutex);

	if (error < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma255_value_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int error;
	struct input_dev *input = to_input_dev(dev);
	struct bma255_data *bma255 = input_get_drvdata(input);
	struct bmc055_data *bmc055_data = container_of(bma255,
						       struct bmc055_data, acc);

	mutex_lock(&bmc055_data->bmc055_mutex);
	error = sprintf(buf, "%d %d %d\n", bma255->value.x, bma255->value.y,
			bma255->value.z);
	mutex_unlock(&bmc055_data->bmc055_mutex);

	return error;
}

static ssize_t bma255_delay_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma255_data *bma255 = input_get_drvdata(input);

	return sprintf(buf, "%d\n", atomic_read(&bma255->delay));
}

static ssize_t bma255_delay_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct input_dev *input = to_input_dev(dev);
	struct bma255_data *bma255 = input_get_drvdata(input);
	struct bmc055_data *bmc055_data = container_of(bma255,
						       struct bmc055_data, acc);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (data <= 0)
		return -EINVAL;

	if (data < bma255_MIN_DELAY)
		data = bma255_MIN_DELAY;

	mutex_lock(&bmc055_data->bmc055_mutex);
	data = bma255_adjust_period(data);
	error = bma255_set_bandwidth(bmc055_data->bmc055_client,
				     bma255_get_bw_code(data));
	mutex_unlock(&bmc055_data->bmc055_mutex);

	if (error < 0)
		return error;

	atomic_set(&bma255->delay, (unsigned int)data);
	return count;
}

static ssize_t bma255_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma255_data *bma255 = input_get_drvdata(input);

	return sprintf(buf, "%d\n", atomic_read(&bma255->enable));
}

static void bma255_set_enable(struct device *dev, int enable)
{
	struct input_dev *input = to_input_dev(dev);
	struct bma255_data *bma255 = input_get_drvdata(input);
	struct bmc055_data *bmc055_data = container_of(bma255,
						       struct bmc055_data, acc);
	int pre_enable = atomic_read(&bma255->enable);
	unsigned long delay_ms = atomic_read(&bma255->delay) / 1000000;

	mutex_lock(&bmc055_data->bmc055_mutex);
	if (enable) {
		if (pre_enable == 0) {
			schedule_delayed_work(&bma255->work,
					      msecs_to_jiffies(delay_ms));
			atomic_set(&bma255->enable, 1);
			bma255_set_mode(bmc055_data->bmc055_client,
					bma255_MODE_NORMAL);
		}

	} else {
		if (pre_enable == 1) {
			cancel_delayed_work(&bma255->work);
			atomic_set(&bma255->enable, 0);
			bma255_set_mode(bmc055_data->bmc055_client,
					bma255_MODE_SUSPEND);
		}
	}
	mutex_unlock(&bmc055_data->bmc055_mutex);
}

static ssize_t bma255_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if ((data == 0) || (data == 1))
		bma255_set_enable(dev, data);

	return count;
}

static ssize_t bma255_nvm_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	unsigned char cmd_data;
	unsigned char regs[BMA255_NVM_SIZE] = { 0 };
	struct input_dev *input = to_input_dev(dev);
	struct bma255_data *bma255 = input_get_drvdata(input);
	struct bmc055_data *bmc055_data = container_of(bma255,
						       struct bmc055_data, acc);
	struct i2c_client *client = bmc055_data->bmc055_client;
	unsigned char dbg_buf_str[100], i;

	mutex_lock(&bmc055_data->bmc055_mutex);

	/* since we are never in deep suspend mode, so we need not to touch
	   0x11 reg to set the mode and can enter the extended mode directly */
	cmd_data = 0xAA;
	bma255_smbus_write_byte(client, 0x35, &cmd_data);
	bma255_smbus_write_byte(client, 0x35, &cmd_data);

	/* since we did this action in probe, no more set here */
	/* bma255_smbus_read_byte(client, 0x05, &cmd_data); */
	/* cmd_data = (cmd_data & 0x1F); */
	/* bma255_smbus_write_byte(client, 0x05, &cmd_data); */

	for (i = 0; i < BMA255_NVM_SIZE; i++)
		bma255_smbus_read_byte(client, i, &regs[i]);

	/* exit extended mode */
	cmd_data = 0x0A;
	bma255_smbus_write_byte(client, 0x35, &cmd_data);

	mutex_unlock(&bmc055_data->bmc055_mutex);

	memset(dbg_buf_str, 0, 100);
	for (i = 0; i < BMA255_NVM_SIZE; i++) {
		sprintf(dbg_buf_str + i * 3, "%02x%c", regs[i],
			(i == BMA255_NVM_SIZE - 1) ? '\0' : ':');
	}
	PINFO("nvm data read:\n%s", dbg_buf_str);

	if (!bma255_check_crc8(regs, BMA255_NVM_SIZE)) {
		strcpy(buf, dbg_buf_str);
		return strlen(dbg_buf_str) + 1;
	} else {
		PERR("CRC check failed");
		strcpy(buf, dbg_buf_str);
		return -EIO;
	}

}

static int bma255_nvm_test_write(struct i2c_client *client,
				 unsigned char *restore)
{
	int err = 0;
	unsigned char dbg_buf_str[100], i, j;
	unsigned char regs[BMA255_NVM_SIZE] = { 0 };

	for (i = 0; i < BMA255_NVM_SIZE; i++)
		bma255_smbus_read_byte(client, i, &regs[i]);

	if (!bma255_check_crc8(regs, BMA255_NVM_SIZE)) {
		PINFO("Good G Sensor, Do not write the NVM");
		return -EPERM;
	} else {
		PINFO("G Sensor NVM destroied, try to restore");
		for (i = 0; i < BMA255_NVM_SIZE; i++) {
			sprintf(dbg_buf_str + i * 3, "%02x%c", restore[i],
				(i == BMA255_NVM_SIZE - 1) ? '\0' : ':');
		}
		PINFO("data to write:\n%s", dbg_buf_str);

		for (j = 0; j < 5; j++) {
			err = 0;
			for (i = 0; i < BMA255_NVM_SIZE; i++) {
				mdelay(2);
				err += bma255_smbus_write_byte(client, i,
							       &restore[i]);
			}
			PINFO("write cycle %d, err=%d", j, err);
			memset(regs, 0, BMA255_NVM_SIZE);
			for (i = 0; i < BMA255_NVM_SIZE; i++)
				bma255_smbus_read_byte(client, i, &regs[i]);

			for (i = 0; i < BMA255_NVM_SIZE; i++) {
				sprintf(dbg_buf_str + i * 3, "%02x%c", regs[i],
					(i ==
					 BMA255_NVM_SIZE - 1) ? '\0' : ':');
			}
			PINFO("data read back:\n%s", dbg_buf_str);

			if ((!err) && (!memcmp(restore, regs, BMA255_NVM_SIZE)))
				return 0;
		}
		return -EIO;
	}
}

static ssize_t bma255_nvm_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int err = 0;
	unsigned int tmp;
	unsigned char i, cmd_data;
	unsigned char restore[BMA255_NVM_SIZE] = { 0 };
	struct input_dev *input = to_input_dev(dev);
	struct bma255_data *bma255 = input_get_drvdata(input);
	struct bmc055_data *bmc055_data = container_of(bma255,
						       struct bmc055_data, acc);
	struct i2c_client *client = bmc055_data->bmc055_client;

	if (strlen(buf) < (BMA255_NVM_SIZE * 3 - 1)) {
		PINFO("invalid restore data lenth %d", strlen(buf));
		return -EINVAL;
	}
	for (i = 0; i < BMA255_NVM_SIZE; i++) {
		if (i == (BMA255_NVM_SIZE - 1))
			sscanf(buf + i * 3, "%02x", &tmp);
		else
			sscanf(buf + i * 3, "%02x:", &tmp);

		restore[i] = (unsigned char)tmp;
	}

	if (!bma255_check_crc8(restore, BMA255_NVM_SIZE)) {
		mutex_lock(&bmc055_data->bmc055_mutex);

		/* since we are never in deep suspend mode, so we need not to touch
		   0x11 reg to set the mode and can enter the extended mode directly */
		cmd_data = 0xAA;
		bma255_smbus_write_byte(client, 0x35, &cmd_data);
		bma255_smbus_write_byte(client, 0x35, &cmd_data);

		/* bma255_smbus_read_byte(client, 0x05, &cmd_data); */
		/* cmd_data = (cmd_data & 0x1F); */
		/* bma255_smbus_write_byte(client, 0x05, &cmd_data); */

		err = bma255_nvm_test_write(client, restore);

		/* exit extended mode */
		cmd_data = 0x0A;
		bma255_smbus_write_byte(client, 0x35, &cmd_data);

		mutex_unlock(&bmc055_data->bmc055_mutex);

		if (err < 0) {
			PERR("write fail: %d", err);
			return err;
		} else {
			return count;
		}
	} else {
		PERR("wrong data:%d, CRC error", count);
		return -EINVAL;
	}
}

BMC055_ATTR(acc, range, S_IRUGO | S_IWUSR | S_IWGRP | S_IWOTH,
	    bma255_range_show, bma255_range_store);
BMC055_ATTR(acc, bandwidth, S_IRUGO | S_IWUSR | S_IWGRP | S_IWOTH,
	    bma255_bandwidth_show, bma255_bandwidth_store);
BMC055_ATTR(acc, mode, S_IRUGO | S_IWUSR | S_IWGRP | S_IWOTH,
	    bma255_mode_show, bma255_mode_store);
BMC055_ATTR(acc, value, S_IRUGO, bma255_value_show, NULL);
BMC055_ATTR(acc, poll_delay, S_IRUGO | S_IWUSR | S_IWGRP | S_IWOTH,
	    bma255_delay_show, bma255_delay_store);
BMC055_ATTR(acc, enable, S_IRUGO | S_IWUSR | S_IWGRP | S_IWOTH,
	    bma255_enable_show, bma255_enable_store);
BMC055_ATTR(acc, nvm, S_IRUGO | S_IWUSR | S_IWGRP | S_IWOTH,
	    bma255_nvm_show, bma255_nvm_store);

static struct attribute *bma255_attributes[] = {
	&bmc055_acc_range_attr.attr,
	&bmc055_acc_bandwidth_attr.attr,
	&bmc055_acc_mode_attr.attr,
	&bmc055_acc_value_attr.attr,
	&bmc055_acc_poll_delay_attr.attr,
	&bmc055_acc_enable_attr.attr,
	&bmc055_acc_nvm_attr.attr,
	NULL
};

static struct attribute_group bma255_attribute_group = {
	.attrs = bma255_attributes
};

static void bma255_input_delete(struct bma255_data *bma255)
{
	struct input_dev *dev = bma255->input;

	kfree(dev->phys);
	input_unregister_device(dev);
}

static int bma255_input_init(struct i2c_client *client,
			     struct bma255_data *bma255)
{
	struct input_dev *dev;
	int err;
	struct bmc055_platform_data *pdata;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	pdata = client->dev.platform_data;

	dev->name = "accelerometer";
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, pdata->acc_xcode);
	input_set_abs_params(dev, pdata->acc_xcode, BMA255_AMIN, BMA255_AMAX, 1,
			     0);

	input_set_capability(dev, EV_ABS, pdata->acc_ycode);
	input_set_abs_params(dev, pdata->acc_ycode, BMA255_AMIN, BMA255_AMAX, 1,
			     0);

	input_set_capability(dev, EV_ABS, pdata->acc_zcode);
	input_set_abs_params(dev, pdata->acc_zcode, BMA255_AMIN, BMA255_AMAX, 1,
			     0);

	input_set_drvdata(dev, bma255);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}

	dev->phys = kobject_get_path(&dev->dev.kobj, GFP_KERNEL);
	if (dev->phys == NULL) {
		dev_err(&dev->dev, "fail to get acc input sysfs path.");
		input_unregister_device(dev);
		return -ENOMEM;
	}

	bma255->input = dev;

	return 0;
}

static int bma255_scan_device(struct i2c_client *client)
{
	unsigned char i, ori_addr, tempvalue, find_device = 0;
	int err = 0;

	ori_addr = client->addr;
	for (i = 0x11; i <= 0x1F; i++) {
		client->addr = i;
		tempvalue = 0;
		err = bma255_smbus_read_byte(client,
					     bma255_CHIP_ID_REG, &tempvalue);
		PINFO("address 0x%02x: err=%d, id=0x%02x", i, err, tempvalue);
		if ((tempvalue == bma255_CHIP_ID) && (!err)) {
			PNOTICE("Address 0x%02x hit", i);

#if 0
			err = bma255_set_mode(client, bma255_MODE_NORMAL);
			if (err < 0) {
				PERR("device 0x%2x set first normal mode failed", i);
				continue;
			}
			msleep(2);
#endif

			tempvalue = 0xAA;
			err = bma255_smbus_write_byte(client, 0x35, &tempvalue);
			err +=
			    bma255_smbus_write_byte(client, 0x35, &tempvalue);

			err += bma255_smbus_read_byte(client, 0x05, &tempvalue);
			PINFO("err=%d, Reg_05=0x%02x", err, tempvalue);
			tempvalue = (tempvalue & 0x1F);
			err +=
			    bma255_smbus_write_byte(client, 0x05, &tempvalue);
			PINFO("err=%d", err);

			tempvalue = 0x0A;
			client->addr = ori_addr;
			err +=
			    bma255_smbus_write_byte(client, 0x35, &tempvalue);
			PINFO("err=%d", err);
			if (!err) {
				find_device = 1;
				break;
			}
		}
	}

	client->addr = ori_addr;
	if (find_device) {
		tempvalue = 0;
		err =
		    bma255_smbus_read_byte(client, bma255_CHIP_ID_REG,
					   &tempvalue);
		PINFO("address 0x%02x: err=%d, id=0x%02x", ori_addr, err,
		      tempvalue);
		if ((tempvalue == bma255_CHIP_ID) && (!err))
			return 0;
		else
			return -1;
	} else {
		return -1;
	}
}

int bma255_local_init(struct bmc055_data *pbmc055)
{
	int err = 0;
	int tempvalue;
	struct i2c_client *client = pbmc055->bmc055_client;
	struct bma255_data *data = &pbmc055->acc;
	struct bmc055_platform_data *pdata = client->dev.platform_data;
	unsigned char cmd_data;

	/* read chip id */
	cmd_data = 0;
	err = bma255_smbus_read_byte(client, bma255_CHIP_ID_REG, &cmd_data);

	if (cmd_data == bma255_CHIP_ID) {
		printk(KERN_INFO "Bosch Sensortec Device detected!\n"
		       "%s registered I2C driver!\n", "bma255");
	} else {
		printk(KERN_INFO
		       "Bosch Sensortec Device predefined-address not found,"
		       "i2c error=%d, id=0x%02x \n", err, cmd_data);

		printk(KERN_INFO "Now let's try to scan the different address"
		       " to detect BMA255:\n");

		err = bma255_scan_device(client);
		if (err < 0) {
			PERR("scan bma255 and set the right addr faild");
			err = -ENXIO;
			goto exit;
		}
		PNOTICE("scan bma255 and set the right addr successful");
	}

	INIT_DELAYED_WORK(&data->work, bma255_work_func);

	tempvalue = bma255_adjust_period(pdata->acc_period);
	atomic_set(&data->delay, tempvalue);
	err = bma255_set_bandwidth(client, bma255_get_bw_code(tempvalue));
	if (err < 0)
		goto exit;

	tempvalue = bma255_adjust_scale(pdata->acc_scale);
	atomic_set(&data->range, tempvalue);
	err = bma255_set_range(client, bma255_get_range_code(tempvalue));
	if (err < 0)
		goto exit;

	atomic_set(&data->enable, 0);
	err = bma255_input_init(client, data);
	if (err < 0)
		goto exit;

	err = sysfs_create_group(&data->input->dev.kobj,
				 &bma255_attribute_group);
	if (err < 0)
		goto error_sysfs;

	return 0;

error_sysfs:
	bma255_input_delete(data);
exit:
	return err;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void bma255_early_suspend(struct bma255_data *data)
{
	struct bmc055_data *bmc055_data = container_of(data,
						       struct bmc055_data, acc);

	mutex_lock(&bmc055_data->bmc055_mutex);
	if (atomic_read(&data->enable) == 1) {
		bma255_set_mode(bmc055_data->bmc055_client,
				bma255_MODE_SUSPEND);
		cancel_delayed_work(&data->work);
	}
	mutex_unlock(&bmc055_data->bmc055_mutex);
}

void bma255_late_resume(struct bma255_data *data)
{
	unsigned long delay_ms = atomic_read(&data->delay) / 1000000;
	struct bmc055_data *bmc055_data = container_of(data,
						       struct bmc055_data, acc);

	mutex_lock(&bmc055_data->bmc055_mutex);
	if (atomic_read(&data->enable) == 1) {
		bma255_set_mode(bmc055_data->bmc055_client, bma255_MODE_NORMAL);
		schedule_delayed_work(&data->work, msecs_to_jiffies(delay_ms));
	}
	mutex_unlock(&bmc055_data->bmc055_mutex);
}
#endif

int bma255_remove(struct bma255_data *data)
{
	bma255_set_enable(&data->input->dev, 0);

	sysfs_remove_group(&data->input->dev.kobj, &bma255_attribute_group);
	bma255_input_delete(data);
	return 0;
}

/*-------------chip driver for bmc055/056--------------------------*/
static int bmc055_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err = 0;
	struct bmc055_data *data;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_INFO "bmm050: i2c_check_functionality error\n");
		err = -1;
		goto exit;
	}

	data = kzalloc(sizeof(struct bmc055_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, data);
	data->bmc055_client = client;
	mutex_init(&data->bmc055_mutex);

	err = bma255_local_init(data);
	if (err < 0)
		goto kfree_exit;

	err = bmm_local_init(data);
	if (err < 0)
		goto kfree_exit_acc;

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = bmc055_early_suspend;
	data->early_suspend.resume = bmc055_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

	return 0;

kfree_exit_acc:		/* free memory for acc sensor's input */
	bma255_remove(&data->acc);

kfree_exit:
	if (err) {
		if (data != NULL) {
			kfree(data);
			data = NULL;
		}
	}
exit:
	return err;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bmc055_early_suspend(struct early_suspend *h)
{
	struct bmc055_data *data =
	    container_of(h, struct bmc055_data, early_suspend);

	bma255_early_suspend(&data->acc);
	bmm_early_suspend(&data->mag);
}

static void bmc055_late_resume(struct early_suspend *h)
{
	struct bmc055_data *data =
	    container_of(h, struct bmc055_data, early_suspend);

	bma255_late_resume(&data->acc);
	bmm_late_resume(&data->mag);
}
#endif

static int bmc055_remove(struct i2c_client *client)
{
	struct bmc055_data *data = i2c_get_clientdata(client);

	bma255_remove(&data->acc);
	bmm_remove(&data->mag);
	kfree(data);
	return 0;
}

static const struct i2c_device_id bmc055_id[] = {
	{"bmc055", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, bmc055_id);

static struct i2c_driver bmc055_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "bmc055",
		   },
	.id_table = bmc055_id,
	.probe = bmc055_probe,
	.remove = bmc055_remove,
};

static int __init bmc055_init(void)
{
	return i2c_add_driver(&bmc055_driver);
}

static void __exit bmc055_exit(void)
{
	i2c_del_driver(&bmc055_driver);
}

MODULE_AUTHOR("Tracy Tao <deliang.tao@bosch-sensortec.com>");
MODULE_DESCRIPTION("bmc055 driver");
MODULE_LICENSE("GPL");

module_init(bmc055_init);
module_exit(bmc055_exit);
