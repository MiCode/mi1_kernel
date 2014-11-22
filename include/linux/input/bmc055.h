/* include/linux/input/bmc055.h
 *
 * Copyright (C) 2012-2014 XiaoMi, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef LINUX_INPUT_BMC055_H
#define LINUX_INPUT_BMC055_H


/*------------Following for BMM050-----------------------------------------*/
#define SENSOR_CHIP_ID_BMM (0x32)

#define BMM_REG_NAME(name) BMM050_##name
#define BMM_VAL_NAME(name) BMM050_##name
#define BMM_CALL_API(name) bmm050_##name

#define BMM_I2C_WRITE_DELAY_TIME 1

#define BMM_DEFAULT_REPETITION_XY BMM_VAL_NAME(REGULAR_REPXY)
#define BMM_DEFAULT_REPETITION_Z BMM_VAL_NAME(REGULAR_REPZ)
#define BMM_DEFAULT_ODR BMM_VAL_NAME(REGULAR_DR)
/* generic */
#define BMM_MAX_RETRY_I2C_XFER (100)
#define BMM_MAX_RETRY_WAKEUP (5)
#define BMM_MAX_RETRY_WAIT_DRDY (100)

#define BMM_DELAY_MIN (1)
#define BMM_DELAY_DEFAULT (200)

#define MAG_VALUE_MAX (10000)
#define MAG_VALUE_MIN (-10000)

#define BYTES_PER_LINE (16)

#define BMM_SELF_TEST 1
#define BMM_ADV_TEST 2

#define BMM_OP_MODE_UNKNOWN (-1)


#define BMM050_U16 unsigned short
#define BMM050_S16 signed short
#define BMM050_S32 signed int


#define BMM050_BUS_WR_RETURN_TYPE char
#define BMM050_BUS_WR_PARAM_TYPES\
	unsigned char, unsigned char, unsigned char *, unsigned char
#define BMM050_BUS_WR_PARAM_ORDER\
	(device_addr, register_addr, register_data, wr_len)
#define BMM050_BUS_WRITE_FUNC(\
		device_addr, register_addr, register_data, wr_len)\
	bus_write(device_addr, register_addr, register_data, wr_len)

#define BMM050_BUS_RD_RETURN_TYPE char

#define BMM050_BUS_RD_PARAM_TYPES\
	unsigned char, unsigned char, unsigned char *, unsigned char

#define BMM050_BUS_RD_PARAM_ORDER (device_addr, register_addr, register_data)

#define BMM050_BUS_READ_FUNC(device_addr, register_addr, register_data, rd_len)\
	bus_read(device_addr, register_addr, register_data, rd_len)


#define BMM050_DELAY_RETURN_TYPE void

#define BMM050_DELAY_PARAM_TYPES unsigned int

#define BMM050_DELAY_FUNC(delay_in_msec)\
	delay_func(delay_in_msec)

#define BMM050_DELAY_POWEROFF_SUSPEND      1
#define BMM050_DELAY_SUSPEND_SLEEP         2
#define BMM050_DELAY_SLEEP_ACTIVE          1
#define BMM050_DELAY_ACTIVE_SLEEP          1
#define BMM050_DELAY_SLEEP_SUSPEND         1
#define BMM050_DELAY_ACTIVE_SUSPEND        1
#define BMM050_DELAY_SLEEP_POWEROFF        1
#define BMM050_DELAY_ACTIVE_POWEROFF       1
#define BMM050_DELAY_SETTLING_TIME         2


#define BMM050_RETURN_FUNCTION_TYPE        char
#define BMM050_I2C_ADDRESS                 0x10

/*General Info datas*/
#define BMM050_SOFT_RESET7_ON              1
#define BMM050_SOFT_RESET1_ON              1
#define BMM050_SOFT_RESET7_OFF             0
#define BMM050_SOFT_RESET1_OFF             0
#define BMM050_DELAY_SOFTRESET             1

/* Fixed Data Registers */
#define BMM050_CHIP_ID                     0x40
#define BMM050_REVISION_ID                 0x41

/* Data Registers */
#define BMM050_DATAX_LSB                   0x42
#define BMM050_DATAX_MSB                   0x43
#define BMM050_DATAY_LSB                   0x44
#define BMM050_DATAY_MSB                   0x45
#define BMM050_DATAZ_LSB                   0x46
#define BMM050_DATAZ_MSB                   0x47
#define BMM050_R_LSB                       0x48
#define BMM050_R_MSB                       0x49

/* Status Registers */
#define BMM050_INT_STAT                    0x4A

/* Control Registers */
#define BMM050_POWER_CNTL                  0x4B
#define BMM050_CONTROL                     0x4C
#define BMM050_INT_CNTL                    0x4D
#define BMM050_SENS_CNTL                   0x4E
#define BMM050_LOW_THRES                   0x4F
#define BMM050_HIGH_THRES                  0x50
#define BMM050_NO_REPETITIONS_XY           0x51
#define BMM050_NO_REPETITIONS_Z            0x52

/* Trim Extended Registers */
#define BMM050_DIG_X1                      0x5D
#define BMM050_DIG_Y1                      0x5E
#define BMM050_DIG_Z4_LSB                  0x62
#define BMM050_DIG_Z4_MSB                  0x63
#define BMM050_DIG_X2                      0x64
#define BMM050_DIG_Y2                      0x65
#define BMM050_DIG_Z2_LSB                  0x68
#define BMM050_DIG_Z2_MSB                  0x69
#define BMM050_DIG_Z1_LSB                  0x6A
#define BMM050_DIG_Z1_MSB                  0x6B
#define BMM050_DIG_XYZ1_LSB                0x6C
#define BMM050_DIG_XYZ1_MSB                0x6D
#define BMM050_DIG_Z3_LSB                  0x6E
#define BMM050_DIG_Z3_MSB                  0x6F
#define BMM050_DIG_XY2                     0x70
#define BMM050_DIG_XY1                     0x71


/* Data X LSB Regsiter */
#define BMM050_DATAX_LSB_VALUEX__POS        3
#define BMM050_DATAX_LSB_VALUEX__LEN        5
#define BMM050_DATAX_LSB_VALUEX__MSK        0xF8
#define BMM050_DATAX_LSB_VALUEX__REG        BMM050_DATAX_LSB

#define BMM050_DATAX_LSB_TESTX__POS         0
#define BMM050_DATAX_LSB_TESTX__LEN         1
#define BMM050_DATAX_LSB_TESTX__MSK         0x01
#define BMM050_DATAX_LSB_TESTX__REG         BMM050_DATAX_LSB

/* Data Y LSB Regsiter */
#define BMM050_DATAY_LSB_VALUEY__POS        3
#define BMM050_DATAY_LSB_VALUEY__LEN        5
#define BMM050_DATAY_LSB_VALUEY__MSK        0xF8
#define BMM050_DATAY_LSB_VALUEY__REG        BMM050_DATAY_LSB

#define BMM050_DATAY_LSB_TESTY__POS         0
#define BMM050_DATAY_LSB_TESTY__LEN         1
#define BMM050_DATAY_LSB_TESTY__MSK         0x01
#define BMM050_DATAY_LSB_TESTY__REG         BMM050_DATAY_LSB

/* Data Z LSB Regsiter */
#define BMM050_DATAZ_LSB_VALUEZ__POS        1
#define BMM050_DATAZ_LSB_VALUEZ__LEN        7
#define BMM050_DATAZ_LSB_VALUEZ__MSK        0xFE
#define BMM050_DATAZ_LSB_VALUEZ__REG        BMM050_DATAZ_LSB

#define BMM050_DATAZ_LSB_TESTZ__POS         0
#define BMM050_DATAZ_LSB_TESTZ__LEN         1
#define BMM050_DATAZ_LSB_TESTZ__MSK         0x01
#define BMM050_DATAZ_LSB_TESTZ__REG         BMM050_DATAZ_LSB

/* Hall Resistance LSB Regsiter */
#define BMM050_R_LSB_VALUE__POS             2
#define BMM050_R_LSB_VALUE__LEN             6
#define BMM050_R_LSB_VALUE__MSK             0xFC
#define BMM050_R_LSB_VALUE__REG             BMM050_R_LSB

#define BMM050_DATA_RDYSTAT__POS            0
#define BMM050_DATA_RDYSTAT__LEN            1
#define BMM050_DATA_RDYSTAT__MSK            0x01
#define BMM050_DATA_RDYSTAT__REG            BMM050_R_LSB

/* Interupt Status Register */
#define BMM050_INT_STAT_DOR__POS            7
#define BMM050_INT_STAT_DOR__LEN            1
#define BMM050_INT_STAT_DOR__MSK            0x80
#define BMM050_INT_STAT_DOR__REG            BMM050_INT_STAT

#define BMM050_INT_STAT_OVRFLOW__POS        6
#define BMM050_INT_STAT_OVRFLOW__LEN        1
#define BMM050_INT_STAT_OVRFLOW__MSK        0x40
#define BMM050_INT_STAT_OVRFLOW__REG        BMM050_INT_STAT

#define BMM050_INT_STAT_HIGH_THZ__POS       5
#define BMM050_INT_STAT_HIGH_THZ__LEN       1
#define BMM050_INT_STAT_HIGH_THZ__MSK       0x20
#define BMM050_INT_STAT_HIGH_THZ__REG       BMM050_INT_STAT

#define BMM050_INT_STAT_HIGH_THY__POS       4
#define BMM050_INT_STAT_HIGH_THY__LEN       1
#define BMM050_INT_STAT_HIGH_THY__MSK       0x10
#define BMM050_INT_STAT_HIGH_THY__REG       BMM050_INT_STAT

#define BMM050_INT_STAT_HIGH_THX__POS       3
#define BMM050_INT_STAT_HIGH_THX__LEN       1
#define BMM050_INT_STAT_HIGH_THX__MSK       0x08
#define BMM050_INT_STAT_HIGH_THX__REG       BMM050_INT_STAT

#define BMM050_INT_STAT_LOW_THZ__POS        2
#define BMM050_INT_STAT_LOW_THZ__LEN        1
#define BMM050_INT_STAT_LOW_THZ__MSK        0x04
#define BMM050_INT_STAT_LOW_THZ__REG        BMM050_INT_STAT

#define BMM050_INT_STAT_LOW_THY__POS        1
#define BMM050_INT_STAT_LOW_THY__LEN        1
#define BMM050_INT_STAT_LOW_THY__MSK        0x02
#define BMM050_INT_STAT_LOW_THY__REG        BMM050_INT_STAT

#define BMM050_INT_STAT_LOW_THX__POS        0
#define BMM050_INT_STAT_LOW_THX__LEN        1
#define BMM050_INT_STAT_LOW_THX__MSK        0x01
#define BMM050_INT_STAT_LOW_THX__REG        BMM050_INT_STAT

/* Power Control Register */
#define BMM050_POWER_CNTL_SRST7__POS       7
#define BMM050_POWER_CNTL_SRST7__LEN       1
#define BMM050_POWER_CNTL_SRST7__MSK       0x80
#define BMM050_POWER_CNTL_SRST7__REG       BMM050_POWER_CNTL

#define BMM050_POWER_CNTL_SPI3_EN__POS     2
#define BMM050_POWER_CNTL_SPI3_EN__LEN     1
#define BMM050_POWER_CNTL_SPI3_EN__MSK     0x04
#define BMM050_POWER_CNTL_SPI3_EN__REG     BMM050_POWER_CNTL

#define BMM050_POWER_CNTL_SRST1__POS       1
#define BMM050_POWER_CNTL_SRST1__LEN       1
#define BMM050_POWER_CNTL_SRST1__MSK       0x02
#define BMM050_POWER_CNTL_SRST1__REG       BMM050_POWER_CNTL

#define BMM050_POWER_CNTL_PCB__POS         0
#define BMM050_POWER_CNTL_PCB__LEN         1
#define BMM050_POWER_CNTL_PCB__MSK         0x01
#define BMM050_POWER_CNTL_PCB__REG         BMM050_POWER_CNTL

/* Control Register */
#define BMM050_CNTL_ADV_ST__POS            6
#define BMM050_CNTL_ADV_ST__LEN            2
#define BMM050_CNTL_ADV_ST__MSK            0xC0
#define BMM050_CNTL_ADV_ST__REG            BMM050_CONTROL

#define BMM050_CNTL_DR__POS                3
#define BMM050_CNTL_DR__LEN                3
#define BMM050_CNTL_DR__MSK                0x38
#define BMM050_CNTL_DR__REG                BMM050_CONTROL

#define BMM050_CNTL_OPMODE__POS            1
#define BMM050_CNTL_OPMODE__LEN            2
#define BMM050_CNTL_OPMODE__MSK            0x06
#define BMM050_CNTL_OPMODE__REG            BMM050_CONTROL

#define BMM050_CNTL_S_TEST__POS            0
#define BMM050_CNTL_S_TEST__LEN            1
#define BMM050_CNTL_S_TEST__MSK            0x01
#define BMM050_CNTL_S_TEST__REG            BMM050_CONTROL

/* Interupt Control Register */
#define BMM050_INT_CNTL_DOR_EN__POS            7
#define BMM050_INT_CNTL_DOR_EN__LEN            1
#define BMM050_INT_CNTL_DOR_EN__MSK            0x80
#define BMM050_INT_CNTL_DOR_EN__REG            BMM050_INT_CNTL

#define BMM050_INT_CNTL_OVRFLOW_EN__POS        6
#define BMM050_INT_CNTL_OVRFLOW_EN__LEN        1
#define BMM050_INT_CNTL_OVRFLOW_EN__MSK        0x40
#define BMM050_INT_CNTL_OVRFLOW_EN__REG        BMM050_INT_CNTL

#define BMM050_INT_CNTL_HIGH_THZ_EN__POS       5
#define BMM050_INT_CNTL_HIGH_THZ_EN__LEN       1
#define BMM050_INT_CNTL_HIGH_THZ_EN__MSK       0x20
#define BMM050_INT_CNTL_HIGH_THZ_EN__REG       BMM050_INT_CNTL

#define BMM050_INT_CNTL_HIGH_THY_EN__POS       4
#define BMM050_INT_CNTL_HIGH_THY_EN__LEN       1
#define BMM050_INT_CNTL_HIGH_THY_EN__MSK       0x10
#define BMM050_INT_CNTL_HIGH_THY_EN__REG       BMM050_INT_CNTL

#define BMM050_INT_CNTL_HIGH_THX_EN__POS       3
#define BMM050_INT_CNTL_HIGH_THX_EN__LEN       1
#define BMM050_INT_CNTL_HIGH_THX_EN__MSK       0x08
#define BMM050_INT_CNTL_HIGH_THX_EN__REG       BMM050_INT_CNTL

#define BMM050_INT_CNTL_LOW_THZ_EN__POS        2
#define BMM050_INT_CNTL_LOW_THZ_EN__LEN        1
#define BMM050_INT_CNTL_LOW_THZ_EN__MSK        0x04
#define BMM050_INT_CNTL_LOW_THZ_EN__REG        BMM050_INT_CNTL

#define BMM050_INT_CNTL_LOW_THY_EN__POS        1
#define BMM050_INT_CNTL_LOW_THY_EN__LEN        1
#define BMM050_INT_CNTL_LOW_THY_EN__MSK        0x02
#define BMM050_INT_CNTL_LOW_THY_EN__REG        BMM050_INT_CNTL

#define BMM050_INT_CNTL_LOW_THX_EN__POS        0
#define BMM050_INT_CNTL_LOW_THX_EN__LEN        1
#define BMM050_INT_CNTL_LOW_THX_EN__MSK        0x01
#define BMM050_INT_CNTL_LOW_THX_EN__REG        BMM050_INT_CNTL

/* Sensor Control Register */
#define BMM050_SENS_CNTL_DRDY_EN__POS          7
#define BMM050_SENS_CNTL_DRDY_EN__LEN          1
#define BMM050_SENS_CNTL_DRDY_EN__MSK          0x80
#define BMM050_SENS_CNTL_DRDY_EN__REG          BMM050_SENS_CNTL

#define BMM050_SENS_CNTL_IE__POS               6
#define BMM050_SENS_CNTL_IE__LEN               1
#define BMM050_SENS_CNTL_IE__MSK               0x40
#define BMM050_SENS_CNTL_IE__REG               BMM050_SENS_CNTL

#define BMM050_SENS_CNTL_CHANNELZ__POS         5
#define BMM050_SENS_CNTL_CHANNELZ__LEN         1
#define BMM050_SENS_CNTL_CHANNELZ__MSK         0x20
#define BMM050_SENS_CNTL_CHANNELZ__REG         BMM050_SENS_CNTL

#define BMM050_SENS_CNTL_CHANNELY__POS         4
#define BMM050_SENS_CNTL_CHANNELY__LEN         1
#define BMM050_SENS_CNTL_CHANNELY__MSK         0x10
#define BMM050_SENS_CNTL_CHANNELY__REG         BMM050_SENS_CNTL

#define BMM050_SENS_CNTL_CHANNELX__POS         3
#define BMM050_SENS_CNTL_CHANNELX__LEN         1
#define BMM050_SENS_CNTL_CHANNELX__MSK         0x08
#define BMM050_SENS_CNTL_CHANNELX__REG         BMM050_SENS_CNTL

#define BMM050_SENS_CNTL_DR_POLARITY__POS      2
#define BMM050_SENS_CNTL_DR_POLARITY__LEN      1
#define BMM050_SENS_CNTL_DR_POLARITY__MSK      0x04
#define BMM050_SENS_CNTL_DR_POLARITY__REG      BMM050_SENS_CNTL

#define BMM050_SENS_CNTL_INTERRUPT_LATCH__POS            1
#define BMM050_SENS_CNTL_INTERRUPT_LATCH__LEN            1
#define BMM050_SENS_CNTL_INTERRUPT_LATCH__MSK            0x02
#define BMM050_SENS_CNTL_INTERRUPT_LATCH__REG            BMM050_SENS_CNTL

#define BMM050_SENS_CNTL_INTERRUPT_POLARITY__POS         0
#define BMM050_SENS_CNTL_INTERRUPT_POLARITY__LEN         1
#define BMM050_SENS_CNTL_INTERRUPT_POLARITY__MSK         0x01
#define BMM050_SENS_CNTL_INTERRUPT_POLARITY__REG         BMM050_SENS_CNTL

/* Register 6D */
#define BMM050_DIG_XYZ1_MSB__POS         0
#define BMM050_DIG_XYZ1_MSB__LEN         7
#define BMM050_DIG_XYZ1_MSB__MSK         0x7F
#define BMM050_DIG_XYZ1_MSB__REG         BMM050_DIG_XYZ1_MSB


#define BMM050_X_AXIS               0
#define BMM050_Y_AXIS               1
#define BMM050_Z_AXIS               2
#define BMM050_RESISTANCE           3
#define BMM050_X                    1
#define BMM050_Y                    2
#define BMM050_Z                    4
#define BMM050_XYZ                  7

/* Constants */
#define BMM050_NULL                             0
#define BMM050_INTPIN_DISABLE                   1
#define BMM050_INTPIN_ENABLE                    0
#define BMM050_DISABLE                          0
#define BMM050_ENABLE                           1
#define BMM050_CHANNEL_DISABLE                  1
#define BMM050_CHANNEL_ENABLE                   0
#define BMM050_INTPIN_LATCH_ENABLE              1
#define BMM050_INTPIN_LATCH_DISABLE             0
#define BMM050_OFF                              0
#define BMM050_ON                               1

#define BMM050_NORMAL_MODE                      0x00
#define BMM050_FORCED_MODE                      0x01
#define BMM050_SUSPEND_MODE                     0x02
#define BMM050_SLEEP_MODE                       0x03

#define BMM050_ADVANCED_SELFTEST_OFF            0
#define BMM050_ADVANCED_SELFTEST_NEGATIVE       2
#define BMM050_ADVANCED_SELFTEST_POSITIVE       3

#define BMM050_NEGATIVE_SATURATION_Z            -32767
#define BMM050_POSITIVE_SATURATION_Z            32767

#define BMM050_SPI_RD_MASK                      0x80
#define BMM050_READ_SET                         0x01

#define E_BMM050_NULL_PTR                       ((char)-127)
#define E_BMM050_COMM_RES                       ((char)-1)
#define E_BMM050_OUT_OF_RANGE                   ((char)-2)
#define E_BMM050_UNDEFINED_MODE                 0


/*Shifting Constants*/
#define SHIFT_RIGHT_1_POSITION                  1
#define SHIFT_RIGHT_2_POSITION                  2
#define SHIFT_RIGHT_3_POSITION                  3
#define SHIFT_RIGHT_4_POSITION                  4
#define SHIFT_RIGHT_5_POSITION                  5
#define SHIFT_RIGHT_6_POSITION                  6
#define SHIFT_RIGHT_7_POSITION                  7
#define SHIFT_RIGHT_8_POSITION                  8

#define SHIFT_LEFT_1_POSITION                   1
#define SHIFT_LEFT_2_POSITION                   2
#define SHIFT_LEFT_3_POSITION                   3
#define SHIFT_LEFT_4_POSITION                   4
#define SHIFT_LEFT_5_POSITION                   5
#define SHIFT_LEFT_6_POSITION                   6
#define SHIFT_LEFT_7_POSITION                   7
#define SHIFT_LEFT_8_POSITION                   8

/* Conversion factors*/
#define BMM050_CONVFACTOR_LSB_UT                6

/* get bit slice  */
#define BMM050_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)

/* Set bit slice */
#define BMM050_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

/* compensated output value returned if sensor had overflow */
#define BMM050_OVERFLOW_OUTPUT       -32768
/* Flipcore overflow ADC value */
#define BMM050_FLIP_OVERFLOW_ADCVAL  -4096
/* Hall overflow 1 ADC value */
#define BMM050_HALL_OVERFLOW_ADCVAL  -16384


#define BMM050_PRESETMODE_LOWPOWER                  1
#define BMM050_PRESETMODE_REGULAR                   2
#define BMM050_PRESETMODE_HIGHACCURACY              3

/* PRESET MODES - DATA RATES */
#define BMM050_LOWPOWER_DR                       BMM050_DR_10HZ
#define BMM050_REGULAR_DR                        BMM050_DR_10HZ
#define BMM050_HIGHACCURACY_DR                   BMM050_DR_20HZ

/* PRESET MODES - REPETITIONS-XY RATES */
#define BMM050_LOWPOWER_REPXY                     2
#define BMM050_REGULAR_REPXY                      5
#define BMM050_HIGHACCURACY_REPXY                40

/* PRESET MODES - REPETITIONS-Z RATES */
#define BMM050_LOWPOWER_REPZ                      4
#define BMM050_REGULAR_REPZ                      13
#define BMM050_HIGHACCURACY_REPZ                 89

/* Data Rates */

#define BMM050_DR_10HZ                     0
#define BMM050_DR_02HZ                     1
#define BMM050_DR_06HZ                     2
#define BMM050_DR_08HZ                     3
#define BMM050_DR_15HZ                     4
#define BMM050_DR_20HZ                     5
#define BMM050_DR_25HZ                     6
#define BMM050_DR_30HZ                     7


/*------------Following for BMA255-----------------------------------------*/
#define GRAVITY_EARTH                   9806550
#define ABSMIN_2G                       (-GRAVITY_EARTH * 2)
#define ABSMAX_2G                       (GRAVITY_EARTH * 2)
#define BMA255_AMIN                     -16000
#define BMA255_AMAX                     16000
#define SLOPE_THRESHOLD_VALUE		32
#define SLOPE_DURATION_VALUE		1
#define INTERRUPT_LATCH_MODE		13
#define INTERRUPT_ENABLE		1
#define INTERRUPT_DISABLE		0
#define MAP_SLOPE_INTERRUPT		2
#define SLOPE_X_INDEX		5
#define SLOPE_Y_INDEX		6
#define SLOPE_Z_INDEX		7

#define bma255_MIN_DELAY		1
#define bma255_DEFAULT_DELAY	200000000

#define bma255_CHIP_ID			0xfa
#define bma255_RANGE_SET		0
#define bma255_BW_SET			4


/*
 *
 *      register definitions
 *
 */

#define bma255_CHIP_ID_REG                      0x00
#define bma255_VERSION_REG                      0x01
#define bma255_X_AXIS_LSB_REG                   0x02
#define bma255_X_AXIS_MSB_REG                   0x03
#define bma255_Y_AXIS_LSB_REG                   0x04
#define bma255_Y_AXIS_MSB_REG                   0x05
#define bma255_Z_AXIS_LSB_REG                   0x06
#define bma255_Z_AXIS_MSB_REG                   0x07
#define bma255_TEMP_RD_REG                      0x08
#define bma255_STATUS1_REG                      0x09
#define bma255_STATUS2_REG                      0x0A
#define bma255_STATUS_TAP_SLOPE_REG             0x0B
#define bma255_STATUS_ORIENT_HIGH_REG           0x0C
#define bma255_RANGE_SEL_REG                    0x0F
#define bma255_BW_SEL_REG                       0x10
#define bma255_MODE_CTRL_REG                    0x11
#define bma255_LOW_NOISE_CTRL_REG               0x12
#define bma255_DATA_CTRL_REG                    0x13
#define bma255_RESET_REG                        0x14
#define bma255_INT_ENABLE1_REG                  0x16
#define bma255_INT_ENABLE2_REG                  0x17
#define bma255_INT1_PAD_SEL_REG                 0x19
#define bma255_INT_DATA_SEL_REG                 0x1A
#define bma255_INT2_PAD_SEL_REG                 0x1B
#define bma255_INT_SRC_REG                      0x1E
#define bma255_INT_SET_REG                      0x20
#define bma255_INT_CTRL_REG                     0x21
#define bma255_LOW_DURN_REG                     0x22
#define bma255_LOW_THRES_REG                    0x23
#define bma255_LOW_HIGH_HYST_REG                0x24
#define bma255_HIGH_DURN_REG                    0x25
#define bma255_HIGH_THRES_REG                   0x26
#define bma255_SLOPE_DURN_REG                   0x27
#define bma255_SLOPE_THRES_REG                  0x28
#define bma255_TAP_PARAM_REG                    0x2A
#define bma255_TAP_THRES_REG                    0x2B
#define bma255_ORIENT_PARAM_REG                 0x2C
#define bma255_THETA_BLOCK_REG                  0x2D
#define bma255_THETA_FLAT_REG                   0x2E
#define bma255_FLAT_HOLD_TIME_REG               0x2F
#define bma255_STATUS_LOW_POWER_REG             0x31
#define bma255_SELF_TEST_REG                    0x32
#define bma255_EEPROM_CTRL_REG                  0x33
#define bma255_SERIAL_CTRL_REG                  0x34
#define bma255_CTRL_UNLOCK_REG                  0x35
#define bma255_OFFSET_CTRL_REG                  0x36
#define bma255_OFFSET_PARAMS_REG                0x37
#define bma255_OFFSET_FILT_X_REG                0x38
#define bma255_OFFSET_FILT_Y_REG                0x39
#define bma255_OFFSET_FILT_Z_REG                0x3A
#define bma255_OFFSET_UNFILT_X_REG              0x3B
#define bma255_OFFSET_UNFILT_Y_REG              0x3C
#define bma255_OFFSET_UNFILT_Z_REG              0x3D
#define bma255_SPARE_0_REG                      0x3E
#define bma255_SPARE_1_REG                      0x3F




#define bma255_ACC_X_LSB__POS           0
#define bma255_ACC_X_LSB__LEN           8
#define bma255_ACC_X_LSB__MSK           0xFF
#define bma255_ACC_X_LSB__REG           bma255_X_AXIS_LSB_REG

#define bma255_ACC_X_MSB__POS           0
#define bma255_ACC_X_MSB__LEN           8
#define bma255_ACC_X_MSB__MSK           0xFF
#define bma255_ACC_X_MSB__REG           bma255_X_AXIS_MSB_REG

#define bma255_ACC_Y_LSB__POS           0
#define bma255_ACC_Y_LSB__LEN           8
#define bma255_ACC_Y_LSB__MSK           0xFF
#define bma255_ACC_Y_LSB__REG           bma255_Y_AXIS_LSB_REG

#define bma255_ACC_Y_MSB__POS           0
#define bma255_ACC_Y_MSB__LEN           8
#define bma255_ACC_Y_MSB__MSK           0xFF
#define bma255_ACC_Y_MSB__REG           bma255_Y_AXIS_MSB_REG

#define bma255_ACC_Z_LSB__POS           0
#define bma255_ACC_Z_LSB__LEN           8
#define bma255_ACC_Z_LSB__MSK           0xFF
#define bma255_ACC_Z_LSB__REG           bma255_Z_AXIS_LSB_REG

#define bma255_ACC_Z_MSB__POS           0
#define bma255_ACC_Z_MSB__LEN           8
#define bma255_ACC_Z_MSB__MSK           0xFF
#define bma255_ACC_Z_MSB__REG           bma255_Z_AXIS_MSB_REG

#define bma255_RANGE_SEL__POS             0
#define bma255_RANGE_SEL__LEN             4
#define bma255_RANGE_SEL__MSK             0x0F
#define bma255_RANGE_SEL__REG             bma255_RANGE_SEL_REG

#define bma255_BANDWIDTH__POS             0
#define bma255_BANDWIDTH__LEN             5
#define bma255_BANDWIDTH__MSK             0x1F
#define bma255_BANDWIDTH__REG             bma255_BW_SEL_REG

#define bma255_EN_LOW_POWER__POS          6
#define bma255_EN_LOW_POWER__LEN          1
#define bma255_EN_LOW_POWER__MSK          0x40
#define bma255_EN_LOW_POWER__REG          bma255_MODE_CTRL_REG

#define bma255_EN_SUSPEND__POS            7
#define bma255_EN_SUSPEND__LEN            1
#define bma255_EN_SUSPEND__MSK            0x80
#define bma255_EN_SUSPEND__REG            bma255_MODE_CTRL_REG


#define bma255_UNLOCK_EE_WRITE_SETTING__POS     0
#define bma255_UNLOCK_EE_WRITE_SETTING__LEN     1
#define bma255_UNLOCK_EE_WRITE_SETTING__MSK     0x01
#define bma255_UNLOCK_EE_WRITE_SETTING__REG     bma255_EEPROM_CTRL_REG

#define bma255_START_EE_WRITE_SETTING__POS      1
#define bma255_START_EE_WRITE_SETTING__LEN      1
#define bma255_START_EE_WRITE_SETTING__MSK      0x02
#define bma255_START_EE_WRITE_SETTING__REG      bma255_EEPROM_CTRL_REG

#define bma255_EE_WRITE_SETTING_S__POS          2
#define bma255_EE_WRITE_SETTING_S__LEN          1
#define bma255_EE_WRITE_SETTING_S__MSK          0x04
#define bma255_EE_WRITE_SETTING_S__REG          bma255_EEPROM_CTRL_REG

#define bma255_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)


#define bma255_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))


/* range and bandwidth */

#define bma255_RANGE_2G                 3
#define bma255_RANGE_4G                 5
#define bma255_RANGE_8G                 8
#define bma255_RANGE_16G                12


#define bma255_BW_7_81HZ        0x08
#define bma255_BW_15_63HZ       0x09
#define bma255_BW_31_25HZ       0x0A
#define bma255_BW_62_50HZ       0x0B
#define bma255_BW_125HZ         0x0C
#define bma255_BW_250HZ         0x0D
#define bma255_BW_500HZ         0x0E
#define bma255_BW_1000HZ        0x0F

/* mode settings */

#define bma255_MODE_NORMAL      0
#define bma255_MODE_LOWPOWER    1
#define bma255_MODE_SUSPEND     2


#define bma255_EN_SELF_TEST__POS                0
#define bma255_EN_SELF_TEST__LEN                2
#define bma255_EN_SELF_TEST__MSK                0x03
#define bma255_EN_SELF_TEST__REG                bma255_SELF_TEST_REG

#define bma255_NEG_SELF_TEST__POS               2
#define bma255_NEG_SELF_TEST__LEN               1
#define bma255_NEG_SELF_TEST__MSK               0x04
#define bma255_NEG_SELF_TEST__REG               bma255_SELF_TEST_REG

#define bma255_EN_FAST_COMP__POS                5
#define bma255_EN_FAST_COMP__LEN                2
#define bma255_EN_FAST_COMP__MSK                0x60
#define bma255_EN_FAST_COMP__REG                bma255_OFFSET_CTRL_REG

#define bma255_FAST_COMP_RDY_S__POS             4
#define bma255_FAST_COMP_RDY_S__LEN             1
#define bma255_FAST_COMP_RDY_S__MSK             0x10
#define bma255_FAST_COMP_RDY_S__REG             bma255_OFFSET_CTRL_REG

#define bma255_COMP_TARGET_OFFSET_X__POS        1
#define bma255_COMP_TARGET_OFFSET_X__LEN        2
#define bma255_COMP_TARGET_OFFSET_X__MSK        0x06
#define bma255_COMP_TARGET_OFFSET_X__REG        bma255_OFFSET_PARAMS_REG

#define bma255_COMP_TARGET_OFFSET_Y__POS        3
#define bma255_COMP_TARGET_OFFSET_Y__LEN        2
#define bma255_COMP_TARGET_OFFSET_Y__MSK        0x18
#define bma255_COMP_TARGET_OFFSET_Y__REG        bma255_OFFSET_PARAMS_REG

#define bma255_COMP_TARGET_OFFSET_Z__POS        5
#define bma255_COMP_TARGET_OFFSET_Z__LEN        2
#define bma255_COMP_TARGET_OFFSET_Z__MSK        0x60
#define bma255_COMP_TARGET_OFFSET_Z__REG        bma255_OFFSET_PARAMS_REG



struct bmc055_platform_data {
	/* for magnetometer sensor */
	int mag_xdir, mag_ydir, mag_zdir; /* can be +1 or -1 */
	int mag_xcode, mag_ycode, mag_zcode; /* can be ABS_? */
	unsigned long mag_period; /* default sample period */
	unsigned long mag_scale; /* 2/4/8/12Ga, 0 mean auto scale */
	u16 mag_lowthres; /* threshold for switch scale value */
	u16 mag_highthres; /* threshold for switch scale value */
	/* for accelerometer sensor */
	int acc_xdir, acc_ydir, acc_zdir; /* can be +1 or -1 */
	int acc_xcode, acc_ycode, acc_zcode; /* can be ABS_? */
	unsigned long acc_period; /* default sample period */
	unsigned long acc_scale; /* 2/4/8/16G, 0 mean auto scale */
	u16 acc_lowthres; /* threshold for switch scale value */
	u16 acc_highthres; /* threshold for switch scale value */
};

#endif
