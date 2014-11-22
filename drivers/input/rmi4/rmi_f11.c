/*
 * Copyright (c) 2011 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/rmi.h>
#include "rmi_driver.h"

#define RESUME_REZERO (defined(CONFIG_RMI4_RESUME_REZERO) && defined(CONFIG_PM))
#if RESUME_REZERO
#include <linux/delay.h>
#define DEFAULT_REZERO_WAIT_MS	40
#endif

#ifndef MT_TOOL_MAX
#define MT_TOOL_MAX MT_TOOL_PEN
#endif

#define F11_MAX_NUM_OF_SENSORS		8
#define F11_MAX_NUM_OF_FINGERS		10
#define F11_MAX_NUM_OF_TOUCH_SHAPES	16

#define F11_REL_POS_MIN		-128
#define F11_REL_POS_MAX		127

#define FINGER_STATE_MASK	0x03
#define GET_FINGER_STATE(f_states, i) \
	((f_states[i / 4] >> (2 * (i % 4))) & FINGER_STATE_MASK)

#define F11_CTRL_SENSOR_MAX_X_POS_OFFSET	6
#define F11_CTRL_SENSOR_MAX_Y_POS_OFFSET	8

#define F11_CEIL(x, y) (((x) + ((y)-1)) / (y))
#define INBOX(x, y, box) (x >= box.x && x < (box.x + box.width) \
			&& y >= box.y && y < (box.y + box.height))

#define DEFAULT_XY_MAX 9999
#define DEFAULT_MAX_ABS_MT_PRESSURE 255
#define DEFAULT_MAX_ABS_MT_TOUCH 15
#define DEFAULT_MAX_ABS_MT_ORIENTATION 1
#define DEFAULT_MIN_ABS_MT_TRACKING_ID 0
#define DEFAULT_MAX_ABS_MT_TRACKING_ID 9
#define MAX_NAME_LENGTH 256

static ssize_t f11_flip_show(struct device *dev,
				   struct device_attribute *attr, char *buf);

static ssize_t f11_flip_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count);

static ssize_t f11_clip_show(struct device *dev,
				   struct device_attribute *attr, char *buf);

static ssize_t f11_clip_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count);

static ssize_t f11_offset_show(struct device *dev,
				     struct device_attribute *attr, char *buf);

static ssize_t f11_offset_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count);

static ssize_t f11_swap_show(struct device *dev,
				   struct device_attribute *attr, char *buf);

static ssize_t f11_swap_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count);

static ssize_t f11_relreport_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static ssize_t f11_relreport_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count);

static ssize_t f11_lcdPos_show(struct device *dev,
				   struct device_attribute *attr, char *buf);

static ssize_t f11_lcdPos_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count);

static ssize_t f11_maxPos_show(struct device *dev,
				     struct device_attribute *attr, char *buf);

static ssize_t f11_rezero_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count);

#if RESUME_REZERO
static ssize_t f11_rezeroOnResume_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static ssize_t f11_rezeroOnResume_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count);
static ssize_t f11_rezeroWait_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static ssize_t f11_rezeroWait_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count);
#endif

static ssize_t f11_logReport_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count);


static void rmi_f11_free_memory(struct rmi_function_container *fc);

static int rmi_f11_initialize(struct rmi_function_container *fc);

static int rmi_f11_create_sysfs(struct rmi_function_container *fc);

static int rmi_f11_config(struct rmi_function_container *fc);

static int rmi_f11_reset(struct rmi_function_container *fc);

static int rmi_f11_register_devices(struct rmi_function_container *fc);

static void rmi_f11_free_devices(struct rmi_function_container *fc);


static struct device_attribute attrs[] = {
	__ATTR(flip, RMI_RW_ATTR, f11_flip_show, f11_flip_store),
	__ATTR(clip, RMI_RW_ATTR, f11_clip_show, f11_clip_store),
	__ATTR(offset, RMI_RW_ATTR, f11_offset_show, f11_offset_store),
	__ATTR(swap, RMI_RW_ATTR, f11_swap_show, f11_swap_store),
	__ATTR(relreport, RMI_RW_ATTR, f11_relreport_show, f11_relreport_store),
	__ATTR(lcdPos, RMI_RW_ATTR, f11_lcdPos_show, f11_lcdPos_store),
	__ATTR(maxPos, RMI_RO_ATTR, f11_maxPos_show, rmi_store_error),
#if RESUME_REZERO
	__ATTR(rezeroOnResume, RMI_RW_ATTR, f11_rezeroOnResume_show,
		f11_rezeroOnResume_store),
	__ATTR(rezeroWait, RMI_RW_ATTR, f11_rezeroWait_show,
		f11_rezeroWait_store),
#endif
	__ATTR(rezero, RMI_WO_ATTR, rmi_show_error, f11_rezero_store),
	__ATTR(logReport, RMI_WO_ATTR, rmi_show_error, f11_logReport_store)
};


union f11_2d_commands {
	struct {
		u8 rezero:1;
	};
	u8 reg;
};

struct f11_2d_device_query {
	union {
		struct {
			u8 nbr_of_sensors:3;
			u8 has_query9:1;
			u8 has_query11:1;
		};
		u8 f11_2d_query0;
	};
};

union f11_2d_query9 {
	struct {
		u8 has_pen:1;
		u8 has_proximity:1;
		u8 has_palm_det_sensitivity:1;
		u8 has_suppress_on_palm_detect:1;
		u8 has_two_pen_thresholds:1;
		u8 has_contact_geometry:1;
	};
	u8 reg;
};

struct f11_2d_sensor_query {
	union {
		struct {
			/* query1 */
			u8 number_of_fingers:3;
			u8 has_rel:1;
			u8 has_abs:1;
			u8 has_gestures:1;
			u8 has_sensitivity_adjust:1;
			u8 configurable:1;
			/* query2 */
			u8 num_of_x_electrodes:7;
			u8 query2_reserved:1;
			/* query3 */
			u8 num_of_y_electrodes:7;
			u8 query3_reserved:1;
			/* query4 */
			u8 max_electrodes:7;
			u8 query4_reserved:1;
		};
		u8 f11_2d_query1__4[4];
	};

	union {
		struct {
			u8 abs_data_size:2;
			u8 has_anchored_finger:1;
			u8 has_adj_hyst:1;
			u8 has_dribble:1;
			u8 has_bending_correction:1;
		};
		u8 f11_2d_query5;
	};

	u8 f11_2d_query6;

	union {
		struct {
			u8 has_single_tap:1;
			u8 has_tap_n_hold:1;
			u8 has_double_tap:1;
			u8 has_early_tap:1;
			u8 has_flick:1;
			u8 has_press:1;
			u8 has_pinch:1;
			u8 padding:1;

			u8 has_palm_det:1;
			u8 has_rotate:1;
			u8 has_touch_shapes:1;
			u8 has_scroll_zones:1;
			u8 has_individual_scroll_zones:1;
			u8 has_multi_finger_scroll:1;
		};
		u8 f11_2d_query7__8[2];
	};

	union f11_2d_query9 query9;

	union {
		struct {
			u8 nbr_touch_shapes:5;
		};
		u8 f11_2d_query10;
	};

	union {
		struct {
			u8 has_z_tuning:1;
			u8 has_pos_interpolation_tuning:1;
			u8 has_w_tuning:1;
			u8 has_pitch_info:1;
			u8 has_default_finger_width:1;
			u8 has_segmentation_aggressiveness:1;
			u8 has_tx_rw_clip:1;
			u8 has_drumming_correction:1;
		};
		u8 f11_2d_query11;
	};
};

struct f11_2d_data_1_5 {
	u8 x_msb;
	u8 y_msb;
	u8 x_lsb:4;
	u8 y_lsb:4;
	u8 w_x:4;
	u8 w_y:4;
	u8 z;
};

struct f11_2d_data_6_7 {
	s8 delta_x;
	s8 delta_y;
};

struct f11_2d_data_8 {
	u8 single_tap:1;
	u8 tap_and_hold:1;
	u8 double_tap:1;
	u8 early_tap:1;
	u8 flick:1;
	u8 press:1;
	u8 pinch:1;
};

struct f11_2d_data_9 {
	u8 palm_detect:1;
	u8 rotate:1;
	u8 shape:1;
	u8 scrollzone:1;
	u8 multi_finger_scroll:1;
	u8 finger_count:3;
};

struct f11_2d_data_10 {
	u8 pinch_motion;
};

struct f11_2d_data_10_12 {
	u8 x_flick_dist;
	u8 y_flick_dist;
	u8 flick_time;
};

struct f11_2d_data_11_12 {
	u8 motion;
	u8 finger_separation;
};

struct f11_2d_data_13 {
	u8 shape_n;
};

struct f11_2d_data_14_15 {
	u8 horizontal;
	u8 vertical;
};

struct f11_2d_data_14_17 {
	u8 x_low;
	u8 y_right;
	u8 x_upper;
	u8 y_left;
};

struct f11_2d_data {
	u8				*f_state;
	const struct f11_2d_data_1_5	*abs_pos;
	const struct f11_2d_data_6_7	*rel_pos;
	const struct f11_2d_data_8	*gest_1;
	const struct f11_2d_data_9	*gest_2;
	const struct f11_2d_data_10	*pinch;
	const struct f11_2d_data_10_12	*flick;
	const struct f11_2d_data_11_12	*rotate;
	const struct f11_2d_data_13	*shapes;
	const struct f11_2d_data_14_15	*multi_scroll;
	const struct f11_2d_data_14_17	*scroll_zones;
};

struct f11_2d_sensor {
	struct rmi_f11_virtualbutton_map *virtualbutton_map;
	struct rmi_f11_2d_axis_alignment axis_align;
	struct f11_2d_sensor_query sens_query;
	struct f11_2d_data data;
	bool log_report;
	u16 max_x;
	u16 max_y;
	u8 nbr_fingers;
	u8 finger_tracker[F11_MAX_NUM_OF_FINGERS];
	u16 x_tracker[F11_MAX_NUM_OF_FINGERS];
	u16 y_tracker[F11_MAX_NUM_OF_FINGERS];
	unsigned long jiffies_tracker[F11_MAX_NUM_OF_FINGERS];
	u8 *data_pkt;
	int pkt_size;
	u8 sensor_index;
	char input_name[MAX_NAME_LENGTH];
	char input_phys[MAX_NAME_LENGTH];
	struct input_dev *input;
	struct input_dev *mouse_input;
};

struct f11_data {
	struct f11_2d_device_query dev_query;
#if	RESUME_REZERO
	u16 rezero_wait_ms;
	bool rezero_on_resume;
#endif
#ifdef CONFIG_RMI4_VIRTUAL_BUTTON
	struct rmi_function_container *fc;
	struct kobject *virtual_button_dir;
	char virtual_button_attr_name[MAX_NAME_LENGTH];
	struct kobj_attribute virtual_button_attr;
#endif
	struct f11_2d_sensor sensors[F11_MAX_NUM_OF_SENSORS];
};

enum finger_state_values {
	F11_NO_FINGER	= 0x00,
	F11_PRESENT	= 0x01,
	F11_INACCURATE	= 0x02,
	F11_RESERVED	= 0x03
};

/** F11_INACCURATE state is overloaded to indicate pen present. */
#ifdef	CONFIG_RMI4_F11_PEN
#define F11_PEN F11_INACCURATE

static int get_tool_type(struct f11_2d_sensor *sensor, u8 finger_state)
{
	if (sensor->sens_query.query9.has_pen && finger_state == F11_PEN)
		return MT_TOOL_PEN;
	return MT_TOOL_FINGER;
}
#endif

static void rmi_f11_rel_pos_report(struct f11_2d_sensor *sensor, u8 n_finger)
{
	struct f11_2d_data *data = &sensor->data;
	struct rmi_f11_2d_axis_alignment *axis_align = &sensor->axis_align;
	s8 x, y;
	s8 temp;

	x = data->rel_pos[n_finger].delta_x;
	y = data->rel_pos[n_finger].delta_y;

	x = min(F11_REL_POS_MAX, max(F11_REL_POS_MIN, (int)x));
	y = min(F11_REL_POS_MAX, max(F11_REL_POS_MIN, (int)y));

	if (axis_align->swap_axes) {
		temp = x;
		x = y;
		y = temp;
	}
	if (axis_align->flip_x)
		x = min(F11_REL_POS_MAX, -x);
	if (axis_align->flip_y)
		y = min(F11_REL_POS_MAX, -y);

	if (x || y) {
		if (sensor->log_report)
			pr_info("%s: f_state[%d]: - x:%d y:%d\n", __func__, n_finger, x, y);
		input_report_rel(sensor->mouse_input, REL_X, x);
		input_report_rel(sensor->mouse_input, REL_Y, y);
	}
	input_sync(sensor->mouse_input);
}

static void rmi_f11_abs_pos_report(struct f11_2d_sensor *sensor,
					u8 finger_state, u8 n_finger)
{
	struct f11_2d_data *data = &sensor->data;
	struct rmi_f11_virtualbutton_map *button_map = sensor->virtualbutton_map;
	struct rmi_f11_2d_axis_alignment *axis_align = &sensor->axis_align;
	int prev_state = sensor->finger_tracker[n_finger];
	int x, y, max_x, max_y, z;
	int w_x, w_y, w_max, w_min, orient;
	int temp;

	if (prev_state && !finger_state) {
		/* this is a release */
		x = y = z = w_max = w_min = orient = 0;
	} else if (!prev_state && !finger_state) {
		/* nothing to report */
		return;
	} else {
		x = ((data->abs_pos[n_finger].x_msb << 4) |
			data->abs_pos[n_finger].x_lsb);
		y = ((data->abs_pos[n_finger].y_msb << 4) |
			data->abs_pos[n_finger].y_lsb);
		z = max((int)data->abs_pos[n_finger].z, 1);
		w_x = max((int)data->abs_pos[n_finger].w_x, 1);
		w_y = max((int)data->abs_pos[n_finger].w_y, 1);
		w_max = max(w_x, w_y);
		w_min = min(w_x, w_y);
		max_x = sensor->max_x;
		max_y = sensor->max_y;

		if (axis_align->swap_axes) {
			temp = x;
			x = y;
			y = temp;
			temp = w_x;
			w_x = w_y;
			w_y = temp;
			temp = max_x;
			max_x = max_y;
			max_y = temp;
		}

		orient = w_x > w_y ? 1 : 0;

		if (axis_align->flip_x)
			x = max(max_x - x, 0);

		if (axis_align->flip_y)
			y = max(max_y - y, 0);

		/*
		** here checking if X offset or y offset are specified is
		**  redundant.  We just add the offsets or, clip the values
		**
		** note: offsets need to be done before clipping occurs,
		** or we could get funny values that are outside
		** clipping boundaries.
		*/
		x += axis_align->offset_X;
		y += axis_align->offset_Y;
		x =  max(axis_align->clip_X_low, x);
		y =  max(axis_align->clip_Y_low, y);
		if (axis_align->clip_X_high)
			x = min(axis_align->clip_X_high, x);
		if (axis_align->clip_Y_high)
			y =  min(axis_align->clip_Y_high, y);

		/* ignore finger outside of lcd and keypad region */
		if (!(x >= axis_align->lcd_X_low &&
		      x <= (axis_align->lcd_X_high ? : max_x) &&
		      y >= axis_align->lcd_Y_low &&
		      y <= (axis_align->lcd_Y_high ? : max_y)) &&
		    !(0 != button_map &&
		      x >= button_map->pad_X_low &&
		      x <= button_map->pad_X_high &&
		      y >= button_map->pad_Y_low &&
		      y <= button_map->pad_Y_high)) {
			 /* force to release state */
			finger_state = 0;
		}

		/* apply jitter filter algorithm */
		if (finger_state == 0) {
			/* this is a release */
			x = y = z = w_max = w_min = orient = 0;
		} else if (prev_state == 0) { /* initial finger touch */
			/* record the position and jiffies */
			sensor->x_tracker[n_finger] = x;
			sensor->y_tracker[n_finger] = y;
			sensor->jiffies_tracker[n_finger] = jiffies;
		} else { /* the rest report until finger lift */
			unsigned long landed_jiffies;
			int delta_x, delta_y, threshold;

			landed_jiffies  = sensor->jiffies_tracker[n_finger];
			landed_jiffies += axis_align->landing_jiffies;

			 /* no significant movement yet */
			if ((prev_state & 0x80) == 0) {
				/* use the big threshold for landing period */
				if (time_before(jiffies, landed_jiffies))
					threshold = axis_align->landing_threshold;
				else /* use the middle jitter threshold */
					threshold = axis_align->staying_threshold;
			} else { /* use the small threshold during movement */
				threshold = axis_align->moving_threshold;
			}

			delta_x = x - sensor->x_tracker[n_finger];
			delta_y = y - sensor->y_tracker[n_finger];

			/* report the recorded position if the change is small */
			if (delta_x * delta_x + delta_y * delta_y <= threshold * threshold) {
				x = sensor->x_tracker[n_finger];
				y = sensor->y_tracker[n_finger];
				finger_state |= (prev_state & 0x80);
			} else {/* save new location and set moving flag */
				sensor->x_tracker[n_finger] = x;
				sensor->y_tracker[n_finger] = y;
				finger_state |= 0x80;
			}
		}
	}

	if (sensor->log_report) {
		pr_info("%s: f_state[%d]:%d - x:%d y:%d z:%d w_max:%d w_min:%d\n",
			__func__, n_finger, finger_state, x, y, z, w_max, w_min);
	}


	if (finger_state != 0) {
#ifdef ABS_MT_PRESSURE
		input_report_abs(sensor->input, ABS_MT_PRESSURE, z);
#endif
		input_report_abs(sensor->input, ABS_MT_WIDTH_MAJOR, w_max);
		input_report_abs(sensor->input, ABS_MT_WIDTH_MINOR, w_min);
		input_report_abs(sensor->input, ABS_MT_ORIENTATION, orient);
		input_report_abs(sensor->input, ABS_MT_POSITION_X, x);
		input_report_abs(sensor->input, ABS_MT_POSITION_Y, y);
		input_report_abs(sensor->input, ABS_MT_TRACKING_ID, n_finger);
#ifdef	CONFIG_RMI4_F11_PEN
		if (sensor->sens_query.query9.has_pen) {
			input_report_abs(sensor->input, ABS_MT_TOOL_TYPE,
					 get_tool_type(sensor, finger_state));
		}
#endif
	}

	/* MT sync between fingers */
	input_mt_sync(sensor->input);
	sensor->finger_tracker[n_finger] = finger_state;
}

static void rmi_f11_finger_handler(struct f11_2d_sensor *sensor)
{
	const u8 *f_state = sensor->data.f_state;
	u8 finger_state;
	u8 i;

	for (i = 0; i < sensor->nbr_fingers; i++) {
		/* Possible of having 4 fingers per f_statet register */
		finger_state = GET_FINGER_STATE(f_state, i);

		if (finger_state == F11_RESERVED) {
			pr_err("%s: Invalid finger state[%d]:0x%02x.", __func__,
					i, finger_state);
			continue;
		}

		if (sensor->data.abs_pos)
			rmi_f11_abs_pos_report(sensor, finger_state, i);

		if (sensor->data.rel_pos)
			rmi_f11_rel_pos_report(sensor, i);
	}
	input_sync(sensor->input);
}

static int f11_2d_construct_data(struct f11_2d_sensor *sensor)
{
	struct f11_2d_sensor_query *query = &sensor->sens_query;
	struct f11_2d_data *data = &sensor->data;
	int i;

	sensor->nbr_fingers = (query->number_of_fingers == 5 ? 10 :
				query->number_of_fingers + 1);

	sensor->pkt_size = F11_CEIL(sensor->nbr_fingers, 4);

	if (query->has_abs)
		sensor->pkt_size += (sensor->nbr_fingers * 5);

	if (query->has_rel)
		sensor->pkt_size +=  (sensor->nbr_fingers * 2);

	/* Check if F11_2D_Query7 is non-zero */
	if (query->f11_2d_query7__8[0])
		sensor->pkt_size += sizeof(u8);

	/* Check if F11_2D_Query7 or F11_2D_Query8 is non-zero */
	if (query->f11_2d_query7__8[0] || query->f11_2d_query7__8[1])
		sensor->pkt_size += sizeof(u8);

	if (query->has_flick)
		sensor->pkt_size += 3;
	else {
		if (query->has_pinch)
			sensor->pkt_size += 1;
		if (query->has_rotate)
			sensor->pkt_size += 2;
	}

	if (query->has_touch_shapes)
		sensor->pkt_size += F11_CEIL(query->nbr_touch_shapes + 1, 8);

	if (query->has_scroll_zones || query->has_multi_finger_scroll)
		sensor->pkt_size += 2;

	if (query->has_scroll_zones && query->has_individual_scroll_zones)
		sensor->pkt_size += 2;

	if (query->query9.has_contact_geometry)
		sensor->pkt_size += (sensor->nbr_fingers * 10);

	if (query->has_bending_correction)
		sensor->pkt_size += 1;

	sensor->data_pkt = kzalloc(sensor->pkt_size, GFP_KERNEL);
	if (!sensor->data_pkt)
		return -ENOMEM;

	data->f_state = sensor->data_pkt;
	i = F11_CEIL(sensor->nbr_fingers, 4);

	if (query->has_abs) {
		data->abs_pos = (struct f11_2d_data_1_5 *)
				&sensor->data_pkt[i];
		i += (sensor->nbr_fingers * 5);
	}

	if (query->has_rel) {
		data->rel_pos = (struct f11_2d_data_6_7 *)
				&sensor->data_pkt[i];
		i += (sensor->nbr_fingers * 2);
	}

	if (query->f11_2d_query7__8[0]) {
		data->gest_1 = (struct f11_2d_data_8 *)&sensor->data_pkt[i];
		i++;
	}

	if (query->f11_2d_query7__8[0] || query->f11_2d_query7__8[1]) {
		data->gest_2 = (struct f11_2d_data_9 *)&sensor->data_pkt[i];
		i++;
	}

	if (query->has_pinch) {
		data->pinch = (struct f11_2d_data_10 *)&sensor->data_pkt[i];
		i++;
	}

	if (query->has_flick) {
		if (query->has_pinch) {
			data->flick = (struct f11_2d_data_10_12 *)data->pinch;
			i += 2;
		} else {
			data->flick = (struct f11_2d_data_10_12 *)
					&sensor->data_pkt[i];
			i += 3;
		}
	}

	if (query->has_rotate) {
		if (query->has_flick) {
			data->rotate = (struct f11_2d_data_11_12 *)
					((char *)data->flick + 1);
		} else {
			data->rotate = (struct f11_2d_data_11_12 *)
					&sensor->data_pkt[i];
			i += 2;
		}
	}

	if (query->has_touch_shapes) {
		data->shapes = (struct f11_2d_data_13 *)&sensor->data_pkt[i];
		i += F11_CEIL(query->nbr_touch_shapes + 1, 8);
	}

	if (query->has_scroll_zones) {
		data->scroll_zones = (struct f11_2d_data_14_17 *)&sensor->data_pkt[i];
		if (query->has_individual_scroll_zones)
			i += 4;
		else
			i += 2;
	}

	if (query->has_multi_finger_scroll) {
		if (data->scroll_zones) {
			data->multi_scroll = (struct f11_2d_data_14_15 *)data->scroll_zones;
		} else {
			data->multi_scroll = (struct f11_2d_data_14_15 *)&sensor->data_pkt[i];
			i += 2;
		}
	}

	return 0;
}

static int rmi_f11_get_query_parameters(struct rmi_device *rmi_dev,
			struct f11_2d_device_query *device_query,
			struct f11_2d_sensor_query *query, u16 query_base_addr)
{
	int query_size;
	int rc;

	rc = rmi_read_block(rmi_dev, query_base_addr, query->f11_2d_query1__4,
					sizeof(query->f11_2d_query1__4));
	if (rc < 0)
		return rc;
	query_size = rc;

	if (query->has_abs) {
		rc = rmi_read(rmi_dev, query_base_addr + query_size,
					&query->f11_2d_query5);
		if (rc < 0)
			return rc;
		query_size++;
	}

	if (query->has_rel) {
		rc = rmi_read(rmi_dev, query_base_addr + query_size,
					&query->f11_2d_query6);
		if (rc < 0)
			return rc;
		query_size++;
	}

	if (query->has_gestures) {
		rc = rmi_read_block(rmi_dev, query_base_addr + query_size,
					query->f11_2d_query7__8,
					sizeof(query->f11_2d_query7__8));
		if (rc < 0)
			return rc;
		query_size += sizeof(query->f11_2d_query7__8);
	}

	if (device_query->has_query9) {
		rc = rmi_read(rmi_dev, query_base_addr + query_size,
				  &query->query9.reg);
		if (rc < 0)
			return rc;
		query_size++;
	}

	if (query->has_touch_shapes) {
		rc = rmi_read(rmi_dev, query_base_addr + query_size,
					&query->f11_2d_query10);
		if (rc < 0)
			return rc;
		query_size++;
	}

	if (device_query->has_query11) {
		rc = rmi_read(rmi_dev, query_base_addr + query_size,
					&query->f11_2d_query11);
		if (rc < 0)
			return rc;
		query_size++;
	}

	return query_size;
}

/* This operation is done in a number of places, so we have a handy routine
 * for it.
 */
static void f11_set_abs_params(struct rmi_function_container *fc, int index)
{
	struct f11_data *instance_data =  fc->data;
	struct f11_2d_sensor *sensor = &instance_data->sensors[index];
	struct input_dev *input = sensor->input;
	int device_x_max = sensor->max_x;
	int device_y_max = sensor->max_y;
	int x_min, x_max, y_min, y_max;

	if (sensor->axis_align.swap_axes) {
		int temp = device_x_max;
		device_x_max = device_y_max;
		device_y_max = temp;
	}

	/* Use the max X and max Y read from the device, or the lcd values */
	x_min = sensor->axis_align.lcd_X_low;
	if (sensor->axis_align.lcd_X_high)
		x_max = sensor->axis_align.lcd_X_high;
	else
		x_max = device_x_max;

	y_min = sensor->axis_align.lcd_Y_low;
	if (sensor->axis_align.lcd_Y_high)
		y_max = sensor->axis_align.lcd_Y_high;
	else
		y_max = device_y_max;

	dev_dbg(&fc->dev, "Set ranges X=[%d..%d] Y=[%d..%d].",
			x_min, x_max, y_min, y_max);

#ifdef ABS_MT_PRESSURE
	input_set_abs_params(input, ABS_MT_PRESSURE, 0,
			DEFAULT_MAX_ABS_MT_PRESSURE, 0, 0);
#endif
	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR,
			0, DEFAULT_MAX_ABS_MT_TOUCH, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MINOR,
			0, DEFAULT_MAX_ABS_MT_TOUCH, 0, 0);
	input_set_abs_params(input, ABS_MT_ORIENTATION,
			0, DEFAULT_MAX_ABS_MT_ORIENTATION, 0, 0);
	input_set_abs_params(input, ABS_MT_TRACKING_ID,
			DEFAULT_MIN_ABS_MT_TRACKING_ID,
			DEFAULT_MAX_ABS_MT_TRACKING_ID, 0, 0);
	/* TODO get max_x_pos (and y) from control registers. */
	input_set_abs_params(input, ABS_MT_POSITION_X,
			x_min, x_max, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y,
			y_min, y_max, 0, 0);
#ifdef	CONFIG_RMI4_F11_PEN
	if (sensor->sens_query.query9.has_pen)
		input_set_abs_params(input, ABS_MT_TOOL_TYPE,
				     0, MT_TOOL_MAX, 0, 0);
#endif
}

static int rmi_f11_init(struct rmi_function_container *fc)
{
	int rc;

	rc = rmi_f11_initialize(fc);
	if (rc < 0)
		goto err_free_data;

	rc = rmi_f11_register_devices(fc);
	if (rc < 0)
		goto err_free_data;

	rc = rmi_f11_create_sysfs(fc);
	if (rc < 0)
		goto err_free_data;

	return 0;

err_free_data:
	rmi_f11_free_memory(fc);

	return rc;
}

static void rmi_f11_free_memory(struct rmi_function_container *fc)
{
	struct f11_data *f11 = fc->data;
	int i;

	if (f11) {
		for (i = 0; i < f11->dev_query.nbr_of_sensors + 1; i++)
			kfree(f11->sensors[i].data_pkt);
		kfree(f11);
		fc->data = NULL;
	}
}


static int rmi_f11_initialize(struct rmi_function_container *fc)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct f11_data *f11;
	u16 query_offset;
	u16 query_base_addr;
	u16 control_base_addr;
	u16 max_x_pos, max_y_pos;
	int rc;
	int i;
	struct rmi_device_platform_data *pdata = to_rmi_platform_data(rmi_dev);

	dev_dbg(&fc->dev, "Initializing F11 values for %s.\n",
		 pdata->sensor_name);

	/*
	** init instance data, fill in values and create any sysfs files
	*/
	f11 = kzalloc(sizeof(struct f11_data), GFP_KERNEL);
	if (!f11)
		return -ENOMEM;

	fc->data = f11;
#if	RESUME_REZERO
	f11->rezero_on_resume = true;
	f11->rezero_wait_ms = DEFAULT_REZERO_WAIT_MS;
#endif

	query_base_addr = fc->fd.query_base_addr;
	control_base_addr = fc->fd.control_base_addr;

	rc = rmi_read(rmi_dev, query_base_addr, &f11->dev_query.f11_2d_query0);
	if (rc < 0)
		return rc;

	query_offset = (query_base_addr + 1);
	/* Increase with one since number of sensors is zero based */
	for (i = 0; i < (f11->dev_query.nbr_of_sensors + 1); i++) {
		f11->sensors[i].sensor_index = i;

		rc = rmi_f11_get_query_parameters(rmi_dev,
					&f11->dev_query,
					&f11->sensors[i].sens_query,
					query_offset);
		if (rc < 0)
			return rc;
		query_offset += rc;

		f11->sensors[i].virtualbutton_map = pdata->virtualbutton_map;
		f11->sensors[i].axis_align = pdata->axis_align;

		rc = rmi_read_block(rmi_dev,
			control_base_addr + F11_CTRL_SENSOR_MAX_X_POS_OFFSET,
			(u8 *)&max_x_pos, sizeof(max_x_pos));
		if (rc < 0)
			return rc;

		rc = rmi_read_block(rmi_dev,
			control_base_addr + F11_CTRL_SENSOR_MAX_Y_POS_OFFSET,
			(u8 *)&max_y_pos, sizeof(max_y_pos));
		if (rc < 0)
			return rc;

		f11->sensors[i].max_x = max_x_pos;
		f11->sensors[i].max_y = max_y_pos;

		rc = f11_2d_construct_data(&f11->sensors[i]);
		if (rc < 0)
			return rc;
	}

	return 0;
}

static int rmi_f11_register_devices(struct rmi_function_container *fc)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct f11_data *f11 = fc->data;
	struct input_dev *input_dev;
	struct input_dev *input_dev_mouse;
	int sensors_itertd = 0;
	int i;
	int rc;

	for (i = 0; i < (f11->dev_query.nbr_of_sensors + 1); i++) {
		sensors_itertd = i;
		input_dev = input_allocate_device();
		if (!input_dev) {
			rc = -ENOMEM;
			goto error_unregister;
		}

		f11->sensors[i].input = input_dev;
		/* TODO how to modify the dev name and
		* phys name for input device */
		sprintf(f11->sensors[i].input_name, "%sfn%02x",
			dev_name(&rmi_dev->dev), fc->fd.function_number);
		input_dev->name = f11->sensors[i].input_name;
		sprintf(f11->sensors[i].input_phys, "%s/input0",
			input_dev->name);
		input_dev->phys = f11->sensors[i].input_phys;
		input_dev->dev.parent = &rmi_dev->dev;
		input_set_drvdata(input_dev, f11);

		set_bit(EV_SYN, input_dev->evbit);
		set_bit(EV_ABS, input_dev->evbit);
		set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

		f11_set_abs_params(fc, i);

		rc = input_register_device(input_dev);
		if (rc < 0) {
			input_free_device(input_dev);
			f11->sensors[i].input = NULL;
			goto error_unregister;
		}

		if (f11->sensors[i].sens_query.has_rel) {
			/*create input device for mouse events  */
			input_dev_mouse = input_allocate_device();
			if (!input_dev_mouse) {
				rc = -ENOMEM;
				goto error_unregister;
			}

			f11->sensors[i].mouse_input = input_dev_mouse;
			input_dev_mouse->name = "rmi_mouse";
			input_dev_mouse->phys = "rmi_f11/input0";

			input_dev_mouse->id.vendor  = 0x18d1;
			input_dev_mouse->id.product = 0x0210;
			input_dev_mouse->id.version = 0x0100;

			set_bit(EV_REL, input_dev_mouse->evbit);
			set_bit(REL_X, input_dev_mouse->relbit);
			set_bit(REL_Y, input_dev_mouse->relbit);

			rc = input_register_device(input_dev_mouse);
			if (rc < 0) {
				input_free_device(input_dev_mouse);
				f11->sensors[i].mouse_input = NULL;
				goto error_unregister;
			}
		}

	}

	return 0;

error_unregister:
	for (; sensors_itertd >= 0; sensors_itertd--) {
		if (f11->sensors[sensors_itertd].input) {
			if (f11->sensors[sensors_itertd].mouse_input) {
				input_unregister_device(
				   f11->sensors[sensors_itertd].mouse_input);
				f11->sensors[sensors_itertd].mouse_input = NULL;
			}
			input_unregister_device(f11->sensors[i].input);
			f11->sensors[i].input = NULL;
		}
	}

	return rc;
}

static void rmi_f11_free_devices(struct rmi_function_container *fc)
{
	struct f11_data *f11 = fc->data;
	int i;

	for (i = 0; i < (f11->dev_query.nbr_of_sensors + 1); i++) {
		if (f11->sensors[i].input)
			input_unregister_device(f11->sensors[i].input);
		if (f11->sensors[i].sens_query.has_rel &&
				f11->sensors[i].mouse_input)
			input_unregister_device(f11->sensors[i].mouse_input);
	}
}

#ifdef CONFIG_RMI4_VIRTUAL_BUTTON
static ssize_t f11_virtual_button_show(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	int i, count = 0;
	struct f11_data *f11;
	struct rmi_f11_virtualbutton_map *virtualbutton_map;

	f11 = container_of(attr, struct f11_data, virtual_button_attr);
	virtualbutton_map = f11->sensors[0].virtualbutton_map;

	for (i = 0; virtualbutton_map && i < virtualbutton_map->buttons; i++) {
		int width  = virtualbutton_map->map[i].width;
		int height = virtualbutton_map->map[i].height;
		int midx   = virtualbutton_map->map[i].x + width / 2;
		int midy   = virtualbutton_map->map[i].y + height / 2;

		count += snprintf(buf + count, PAGE_SIZE - count,
				"0x%02x:%d:%d:%d:%d:%d:", EV_KEY,
				virtualbutton_map->map[i].code,
				midx, midy, width, height);
	}

	count -= 1; /* remove the last colon */
	count += snprintf(buf+count, PAGE_SIZE-count, "\n");
	return count;
}
#endif

static int rmi_f11_create_sysfs(struct rmi_function_container *fc)
{
	int attr_count = 0;
	int rc;

	dev_dbg(&fc->dev, "Creating sysfs files.\n");
	/* Set up sysfs device attributes. */
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		if (sysfs_create_file
		    (&fc->dev.kobj, &attrs[attr_count].attr) < 0) {
			dev_err(&fc->dev,
				"Failed to create sysfs file for %s.",
				attrs[attr_count].attr.name);
			rc = -ENODEV;
			goto err_remove_sysfs;
		}
	}
#ifdef CONFIG_RMI4_VIRTUAL_BUTTON
	{
		struct f11_data *f11 = fc->data;
		f11->fc = fc;

		f11->virtual_button_dir = kobject_create_and_add("board_properties", NULL);
		if (f11->virtual_button_dir == NULL) {
			dev_err(&fc->dev, "Fail to create board_properties entry\n");
			rc = -ENOMEM;
			goto err_remove_sysfs;
		}

		sysfs_attr_init(&f11->virtual_button_attr.attr);
		sprintf(f11->virtual_button_attr_name, "virtualkeys.%s", f11->sensors[0].input_name);
		f11->virtual_button_attr.attr.name = f11->virtual_button_attr_name;
		f11->virtual_button_attr.attr.mode = RMI_RO_ATTR;
		f11->virtual_button_attr.show      = f11_virtual_button_show;

		rc = sysfs_create_file(f11->virtual_button_dir, &f11->virtual_button_attr.attr);
		if (rc < 0) {
			dev_err(&fc->dev, "Fail to create virtualkeys entry\n");
			kobject_put(f11->virtual_button_dir);
			goto err_remove_sysfs;
		}
	}
#endif

	return 0;

err_remove_sysfs:
	for (attr_count--; attr_count >= 0; attr_count--)
		sysfs_remove_file(&fc->dev.kobj,
						  &attrs[attr_count].attr);
	return rc;
}

static int rmi_f11_config(struct rmi_function_container *fc)
{
	/* we do nothing here */
	return 0;
}

static int rmi_f11_reset(struct rmi_function_container *fc)
{
	/* we do nothing here */
	return 0;
}

int rmi_f11_attention(struct rmi_function_container *fc, u8 *irq_bits)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct f11_data *f11 = fc->data;
	u16 data_base_addr = fc->fd.data_base_addr;
	int data_base_addr_offset = 0;
	int error;
	int i;

	for (i = 0; i < f11->dev_query.nbr_of_sensors + 1; i++) {
		error = rmi_read_block(rmi_dev,
				data_base_addr + data_base_addr_offset,
				f11->sensors[i].data_pkt,
				f11->sensors[i].pkt_size);
		if (error < 0)
			return error;

		rmi_f11_finger_handler(&f11->sensors[i]);
		data_base_addr_offset += f11->sensors[i].pkt_size;
	}

	return 0;
}

static int rmi_f11_suspend(struct rmi_function_container *fc)
{
	struct f11_data *f11 = fc->data;
	int i;

	for (i = 0; i < f11->dev_query.nbr_of_sensors + 1; i++) {
		memset(f11->sensors[i].finger_tracker, 0,
			sizeof(f11->sensors[i].finger_tracker));
	}

	return 0;
}

#if RESUME_REZERO
static int rmi_f11_resume(struct rmi_function_container *fc)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct f11_data *data = fc->data;
	/* Command register always reads as 0, so we can just use a local. */
	union f11_2d_commands commands = {};
	int retval = 0;

	dev_dbg(&fc->dev, "Resuming...\n");
	if (!data->rezero_on_resume)
		return 0;

	if (data->rezero_wait_ms)
		mdelay(data->rezero_wait_ms);

	commands.rezero = 1;
	retval = rmi_write_block(rmi_dev, fc->fd.command_base_addr,
			&commands.reg, sizeof(commands.reg));
	if (retval < 0) {
		dev_err(&rmi_dev->dev, "%s: failed to issue rezero command, error = %d.",
			__func__, retval);
		return retval;
	}

	return retval;
}
#endif /* RESUME_REZERO */

static void rmi_f11_remove(struct rmi_function_container *fc)
{
	int attr_count = 0;

#ifdef CONFIG_RMI4_VIRTUAL_BUTTON
	struct f11_data *f11 = fc->data;
	kobject_put(f11->virtual_button_dir);
#endif

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		sysfs_remove_file(&fc->dev.kobj,
				  &attrs[attr_count].attr);
	}

	rmi_f11_free_devices(fc);

	rmi_f11_free_memory(fc);

}

static struct rmi_function_handler function_handler = {
	.func = 0x11,
	.init = rmi_f11_init,
	.config = rmi_f11_config,
	.reset = rmi_f11_reset,
	.attention = rmi_f11_attention,
	.remove = rmi_f11_remove,
	.suspend = rmi_f11_suspend,
#if	RESUME_REZERO
	.resume = rmi_f11_resume
#endif
};

static int __init rmi_f11_module_init(void)
{
	int error;

	error = rmi_register_function_driver(&function_handler);
	if (error < 0) {
		pr_err("%s: register failed!\n", __func__);
		return error;
	}

	return 0;
}

static void __exit rmi_f11_module_exit(void)
{
	rmi_unregister_function_driver(&function_handler);
}

static ssize_t f11_maxPos_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct rmi_function_container *fc;
	struct f11_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u %u\n",
			data->sensors[0].max_x, data->sensors[0].max_y);
}

static ssize_t f11_flip_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct rmi_function_container *fc;
	struct f11_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u %u\n",
			data->sensors[0].axis_align.flip_x,
			data->sensors[0].axis_align.flip_y);
}

static ssize_t f11_flip_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;
	unsigned int new_X, new_Y;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;


	if (sscanf(buf, "%u %u", &new_X, &new_Y) != 2)
		return -EINVAL;
	if (new_X < 0 || new_X > 1 || new_Y < 0 || new_Y > 1)
		return -EINVAL;
	instance_data->sensors[0].axis_align.flip_x = new_X;
	instance_data->sensors[0].axis_align.flip_y = new_Y;

	return count;
}

static ssize_t f11_swap_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			instance_data->sensors[0].axis_align.swap_axes);
}

static ssize_t f11_swap_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;
	unsigned int newSwap;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;


	if (sscanf(buf, "%u", &newSwap) != 1)
		return -EINVAL;
	if (newSwap < 0 || newSwap > 1)
		return -EINVAL;
	instance_data->sensors[0].axis_align.swap_axes = newSwap;

	f11_set_abs_params(fc, 0);

	return count;
}

static ssize_t f11_relreport_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			instance_data->
			sensors[0].axis_align.rel_report_enabled);
}

static ssize_t f11_relreport_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t count)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;
	unsigned int new_value;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;


	if (sscanf(buf, "%u", &new_value) != 1)
		return -EINVAL;
	if (new_value < 0 || new_value > 1)
		return -EINVAL;
	instance_data->sensors[0].axis_align.rel_report_enabled = new_value;

	return count;
}

static ssize_t f11_offset_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%d %d\n",
			instance_data->sensors[0].axis_align.offset_X,
			instance_data->sensors[0].axis_align.offset_Y);
}

static ssize_t f11_offset_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;
	int new_X, new_Y;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;


	if (sscanf(buf, "%d %d", &new_X, &new_Y) != 2)
		return -EINVAL;
	instance_data->sensors[0].axis_align.offset_X = new_X;
	instance_data->sensors[0].axis_align.offset_Y = new_Y;

	return count;
}

static ssize_t f11_clip_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{

	struct rmi_function_container *fc;
	struct f11_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u %u %u %u\n",
			instance_data->sensors[0].axis_align.clip_X_low,
			instance_data->sensors[0].axis_align.clip_X_high,
			instance_data->sensors[0].axis_align.clip_Y_low,
			instance_data->sensors[0].axis_align.clip_Y_high);
}

static ssize_t f11_clip_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;
	unsigned int new_X_low, new_X_high, new_Y_low, new_Y_high;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	if (sscanf(buf, "%u %u %u %u",
		   &new_X_low, &new_X_high, &new_Y_low, &new_Y_high) != 4)
		return -EINVAL;
	if (new_X_low < 0 || new_X_low >= new_X_high || new_Y_low < 0
	    || new_Y_low >= new_Y_high)
		return -EINVAL;
	instance_data->sensors[0].axis_align.clip_X_low = new_X_low;
	instance_data->sensors[0].axis_align.clip_X_high = new_X_high;
	instance_data->sensors[0].axis_align.clip_Y_low = new_Y_low;
	instance_data->sensors[0].axis_align.clip_Y_high = new_Y_high;

	return count;
}

static ssize_t f11_lcdPos_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{

	struct rmi_function_container *fc;
	struct f11_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%d %d %d %d\n",
			instance_data->sensors[0].axis_align.lcd_X_low,
			instance_data->sensors[0].axis_align.lcd_X_high,
			instance_data->sensors[0].axis_align.lcd_Y_low,
			instance_data->sensors[0].axis_align.lcd_Y_high);
}

static ssize_t f11_lcdPos_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;
	int new_X_low, new_X_high, new_Y_low, new_Y_high;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	if (sscanf(buf, "%d %d %d %d",
		   &new_X_low, &new_X_high, &new_Y_low, &new_Y_high) != 4)
		return -EINVAL;
	if (new_X_low >= new_X_high || new_Y_low >= new_Y_high)
		return -EINVAL;
	instance_data->sensors[0].axis_align.lcd_X_low = new_X_low;
	instance_data->sensors[0].axis_align.lcd_X_high = new_X_high;
	instance_data->sensors[0].axis_align.lcd_Y_low = new_Y_low;
	instance_data->sensors[0].axis_align.lcd_Y_high = new_Y_high;

	/*
	** for now, we assume this is sensor index 0
	*/
	f11_set_abs_params(fc, 0);

	return count;
}

static ssize_t f11_rezero_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct rmi_function_container *fc = NULL;
	unsigned int rezero;
	int retval = 0;
	/* Command register always reads as 0, so we can just use a local. */
	union f11_2d_commands commands = {};

	fc = to_rmi_function_container(dev);

	if (sscanf(buf, "%u", &rezero) != 1)
		return -EINVAL;
	if (rezero < 0 || rezero > 1)
		return -EINVAL;

	/* Per spec, 0 has no effect, so we skip it entirely. */
	if (rezero) {
		commands.rezero = 1;
		retval = rmi_write_block(fc->rmi_dev, fc->fd.command_base_addr,
				&commands.reg, sizeof(commands.reg));
		if (retval < 0) {
			dev_err(dev, "%s: failed to issue rezero command, error = %d.",
				__func__, retval);
			return retval;
		}
	}

	return count;
}

#if RESUME_REZERO
static ssize_t f11_rezeroOnResume_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct rmi_function_container *fc = NULL;
	unsigned int newValue;
	struct f11_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	if (sscanf(buf, "%u", &newValue) != 1)
		return -EINVAL;
	if (newValue < 0 || newValue > 1) {
		dev_err(dev, "rezeroOnResume must be either 1 or 0.\n");
		return -EINVAL;
	}

	instance_data->rezero_on_resume = (newValue != 0);

	return count;
}

static ssize_t f11_rezeroOnResume_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			instance_data->rezero_on_resume);
}

static ssize_t f11_rezeroWait_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct rmi_function_container *fc = NULL;
	unsigned int newValue;
	struct f11_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	if (sscanf(buf, "%u", &newValue) != 1)
		return -EINVAL;
	if (newValue < 0) {
		dev_err(dev, "rezeroWait must be 0 or greater.\n");
		return -EINVAL;
	}

	instance_data->rezero_wait_ms = newValue;

	return count;
}

static ssize_t f11_rezeroWait_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			instance_data->rezero_wait_ms);
}
#endif

static ssize_t f11_logReport_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,
				    size_t count)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;
	int log_report;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	if (sscanf(buf, "%d", &log_report) != 1)
		return -EINVAL;

	instance_data->sensors[0].log_report = (log_report != 0);
	return count;
}

module_init(rmi_f11_module_init);
module_exit(rmi_f11_module_exit);

MODULE_AUTHOR("Christopher Heiny <cheiny@synaptics.com");
MODULE_DESCRIPTION("RMI F11 module");
MODULE_LICENSE("GPL");
MODULE_VERSION(RMI_DRIVER_VERSION);
