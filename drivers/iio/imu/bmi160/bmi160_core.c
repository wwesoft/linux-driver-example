// SPDX-License-Identifier: GPL-2.0
/*
 * BMI160 - Bosch IMU (accel, gyro plus external magnetometer)
 *
 * Copyright (c) 2016, Intel Corporation.
 * Copyright (c) 2019, Martin Kelly.
 *
 * IIO core driver for BMI160, with support for I2C/SPI busses
 *
 * TODO: magnetometer, hardware FIFO
 */
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/of_irq.h>

#include <linux/iio/iio.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>

#include "bmi160.h"

#define BMI160_IRQ_NAME			"bmi160_event"
#define BMI160_REG_CHIP_ID	0x00
#define BMI160_CHIP_ID_VAL	0xD1

#define BMI160_REG_PMU_STATUS	0x03

/* X axis data low byte address, the rest can be obtained using axis offset */
#define BMI160_REG_DATA_MAGN_XOUT_L	0x04
#define BMI160_REG_DATA_GYRO_XOUT_L	0x0C
#define BMI160_REG_DATA_ACCEL_XOUT_L	0x12

#define BMI160_REG_STATUS			0x1B
#define BMI160_FOC_RDY				BIT(3)

#define BMI160_ACCEL_REG_INT_STATUS_0	0x1E
#define BMI160_ANYM_INT 			BIT(2)

#define BMI160_ACCEL_REG_INT_STATUS_1	0x1D
#define BMI160_NOMOT_INT 			BIT(7)
#define BMI160_DRDY_INT 			BIT(4)

#define BMI160_ACCEL_REG_INT_STATUS_2	0x1E
#define BMI160_ANYM_SIGN 			BIT(3)
#define BMI160_ANYM_Z 				BIT(2)
#define BMI160_ANYM_Y 				BIT(1)
#define BMI160_ANYM_X 				BIT(0)

#define BMI160_REG_ACCEL_CONFIG		0x40
#define BMI160_ACCEL_CONFIG_ODR_MASK	GENMASK(3, 0)
#define BMI160_ACCEL_CONFIG_BWP_MASK	GENMASK(6, 4)
#define BMI160_ACCEL_CONFIG_BWP_SHIFT	4

#define BMI160_REG_ACCEL_RANGE		0x41
#define BMI160_ACCEL_RANGE_2G		0x03
#define BMI160_ACCEL_RANGE_4G		0x05
#define BMI160_ACCEL_RANGE_8G		0x08
#define BMI160_ACCEL_RANGE_16G		0x0C

#define BMI160_REG_GYRO_CONFIG		0x42
#define BMI160_GYRO_CONFIG_ODR_MASK	GENMASK(3, 0)
#define BMI160_GYRO_CONFIG_BWP_MASK	GENMASK(5, 4)

#define BMI160_REG_GYRO_RANGE		0x43
#define BMI160_GYRO_RANGE_2000DPS	0x00
#define BMI160_GYRO_RANGE_1000DPS	0x01
#define BMI160_GYRO_RANGE_500DPS	0x02
#define BMI160_GYRO_RANGE_250DPS	0x03
#define BMI160_GYRO_RANGE_125DPS	0x04

#define BMI160_REG_CMD			0x7E
#define BMI160_CMD_START_FOC	0x03
#define BMI160_CMD_ACCEL_PM_SUSPEND	0x10
#define BMI160_CMD_ACCEL_PM_NORMAL	0x11
#define BMI160_CMD_ACCEL_PM_LOW_POWER	0x12
#define BMI160_CMD_GYRO_PM_SUSPEND	0x14
#define BMI160_CMD_GYRO_PM_NORMAL	0x15
#define BMI160_CMD_GYRO_PM_FAST_STARTUP	0x17
#define BMI160_CMD_INTRESET		0xB1
#define BMI160_CMD_SOFTRESET		0xB6

#define BMI160_REG_INT_EN_0		0x50
#define BMI160_ANYMOTION_X_INT_EN		BIT(0)
#define BMI160_ANYMOTION_Y_INT_EN		BIT(1)
#define BMI160_ANYMOTION_Z_INT_EN		BIT(2)

#define BMI160_REG_INT_EN_1		0x51
#define BMI160_DRDY_INT_EN		BIT(4)

#define BMI160_REG_INT_EN_2		0x52
#define BMI160_NOMOTION_X_INT_EN		BIT(0)
#define BMI160_NOMOTION_Y_INT_EN		BIT(1)
#define BMI160_NOMOTION_Z_INT_EN		BIT(2)

#define BMI160_REG_INT_OUT_CTRL		0x53
#define BMI160_INT_OUT_CTRL_MASK	0x0f
#define BMI160_INT1_OUT_CTRL_SHIFT	0
#define BMI160_INT2_OUT_CTRL_SHIFT	4
#define BMI160_EDGE_TRIGGERED		BIT(0)
#define BMI160_ACTIVE_HIGH		BIT(1)
#define BMI160_OPEN_DRAIN		BIT(2)
#define BMI160_OUTPUT_EN		BIT(3)

#define BMI160_REG_INT_LATCH		0x54
#define BMI160_INT_LATCH_MASK		0x0f
#define BMI160_INT_LATCH_10		0x06
#define BMI160_INT_LATCH_20		0x07
#define BMI160_INT_LATCH_40		0x08
#define BMI160_INT_LATCH_80		0x09
#define BMI160_INT_LATCH_160		0x0A
#define BMI160_INT_LATCH_320		0x0B
#define BMI160_INT_LATCH_640		0x0C
#define BMI160_INT_LATCH_1280		0x0D
#define BMI160_INT_LATCH_2560		0x0E
#define BMI160_INT_LATCH_EVER		0x0F


#define BMI160_REG_INT_MAP0		0x55
#define BMI160_REG_INT_MAP2		0x57
#define BMI160_INT_MAP_NOMOTION_EN	BIT(3)
#define BMI160_INT_MAP_ANYMOTION_EN	BIT(2)

#define BMI160_REG_INT_MAP1		0x56
#define BMI160_INT1_MAP_DRDY_EN		BIT(7)
#define BMI160_INT2_MAP_DRDY_EN		BIT(3)

#define BMI160_REG_INT_DATA_1	0x59
#define BMI160_INT_MOTION_SRC	BIT(7)
/* Configuration register for Motion interrupt  */
#define BMI160_REG_INT_DURATION		0x5F
#define BMI160_ANYMOTION_DUR_MASK	0x03
#define BMI160_ANYMOTION_DUR_SHIFT	0x00
#define BMI160_NOMOTION_DUR_MASK	0x3F
#define BMI160_NOMOTION_DUR_SHIFT	0x02

#define BMI160_REG_INT_ANYMOT_THRES		0x60
#define BMI160_REG_INT_NOMOT_THRES		0x61
#define BMI160_REG_INT_MOTION_CFG		0x62
#define BMI160_NOMOTION_SEL		BIT(0)
#define BMI160_SIGMOTION_SEL		BIT(1)


/* Configuration register for Fast Offset Compensation  */
#define BMI160_REG_FOC_CONF		0x69
#define BMI160_FOC_GYR_EN_MASK	BIT(6)
#define BMI160_FOC_ACC_X_SHIFT	4
#define BMI160_FOC_ACC_Y_SHIFT	2
#define BMI160_FOC_ACC_Z_SHIFT	0
#define BMI160_FOC_ACC_MASK	0x03

/* register Offset Compensation  */
#define BMI160_REG_OFFSET		0x77
#define BMI160_GYR_OFF_EN_MASK	BIT(7)
#define BMI160_ACC_OFF_EN_MASK	BIT(6)


#define BMI160_REG_DUMMY		0x7F

/* Slope duration in terms of number of samples */
#define BMI160_ACCEL_DEF_SLOPE_DURATION		1
/* in terms of multiples of g's/LSB, based on range */
#define BMI160_ACCEL_DEF_SLOPE_THRESHOLD	1


#define BMI160_NORMAL_WRITE_USLEEP	5
#define BMI160_SUSPENDED_WRITE_USLEEP	450

#define BMI160_ACCEL_PMU_MIN_USLEEP	3800
#define BMI160_GYRO_PMU_MIN_USLEEP	80000
#define BMI160_SOFTRESET_USLEEP		1000


#define BMI160_CHANNEL(_type, _axis, _index) {			\
	.type = _type,						\
	.modified = 1,						\
	.channel2 = IIO_MOD_##_axis,		\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |  \
		BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _index,				\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 16,					\
		.storagebits = 16,				\
		.endianness = IIO_LE,			\
	},									\
}

#define BMI160_EVENT_CHANNEL(_type, _axis, _index) {			\
	.type = _type,						\
	.modified = 1,						\
	.channel2 = IIO_MOD_##_axis,		\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |  \
		BIT(IIO_CHAN_INFO_SAMP_FREQ) |						\
		BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY),	\
	.scan_index = _index,				\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 16,					\
		.storagebits = 16,				\
		.endianness = IIO_LE,			\
	},									\
	.event_spec = bmi160_events,	\
	.num_event_specs = ARRAY_SIZE(bmi160_events)			\
}

/* scan indexes follow DATA register order */
enum bmi160_scan_axis {
	BMI160_SCAN_EXT_MAGN_X = 0,
	BMI160_SCAN_EXT_MAGN_Y,
	BMI160_SCAN_EXT_MAGN_Z,
	BMI160_SCAN_RHALL,
	BMI160_SCAN_GYRO_X,
	BMI160_SCAN_GYRO_Y,
	BMI160_SCAN_GYRO_Z,
	BMI160_SCAN_ACCEL_X,
	BMI160_SCAN_ACCEL_Y,
	BMI160_SCAN_ACCEL_Z,
	BMI160_SCAN_TIMESTAMP,
};

enum bmi160_sensor_type {
	BMI160_ACCEL	= 0,
	BMI160_GYRO,
	BMI160_EXT_MAGN,
	BMI160_NUM_SENSORS /* must be last */
};

enum bmi160_int_pin {
	BMI160_PIN_INT1,
	BMI160_PIN_INT2
};

enum bmi160_accel_interrupt_id {
	BMI160_ACCEL_INT_DATA_READY,
	BMI160_ACCEL_INT_ANY_MOTION,
	BMI160_ACCEL_INT_NO_MOTION,
	BMI160_ACCEL_INTERRUPTS,
};

enum bmi160_accel_interrupt_type {
	BMI160_DATA_DRIVEN_INTERRUPT,
	BMI160_PHYSICAL_INTERRUPT
};

enum bmi160_accel_trigger_id {
	BMI160_ACCEL_TRIGGER_DATA_READY,
	BMI160_ACCEL_TRIGGER_ANY_MOTION,
	BMI160_ACCEL_TRIGGER_NO_MOTION,
	BMI160_ACCEL_TRIGGERS,
};

const struct regmap_config bmi160_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};
EXPORT_SYMBOL(bmi160_regmap_config);

struct bmi160_accel_interrupt {
	const struct bmi160_accel_interrupt_info *info;
	atomic_t users;
};

struct bmi160_accel_trigger {
	struct bmi160_data *data;
	struct iio_trigger *indio_trig;
	int (*setup)(struct bmi160_accel_trigger *t, bool state);
	int intr;
	bool enabled;
};

struct bmi160_data {
	struct regmap *regmap;
	struct iio_trigger *trig;
	struct bmi160_accel_interrupt interrupts[BMI160_ACCEL_INTERRUPTS];
	struct bmi160_accel_trigger triggers[BMI160_ACCEL_TRIGGERS];
	struct mutex mutex;
	u8 intr_pin;
	u8 anymot_mustbe_updated;
	u8 nomot_mustbe_updated;
	u32 slope_anymot_dur;
	u32 slope_anymot_thres;
	u32 slope_nomot_dur;
	u32 slope_nomot_thres;
	int ev_nomot_enable_state;
	int ev_anymot_enable_state;
	int64_t timestamp;
};

struct bmi160_regs {
	u8 data; /* LSB byte register for X-axis */
	u8 config;
	u8 config_odr_mask;
	u8 config_bwp_mask;
	u8 range;
	u8 pmu_cmd_normal;
	u8 pmu_cmd_suspend;
};

static struct bmi160_regs bmi160_regs[] = {
	[BMI160_ACCEL] = {
		.data	= BMI160_REG_DATA_ACCEL_XOUT_L,
		.config	= BMI160_REG_ACCEL_CONFIG,
		.config_odr_mask = BMI160_ACCEL_CONFIG_ODR_MASK,
		.config_bwp_mask = BMI160_ACCEL_CONFIG_BWP_MASK,
		.range	= BMI160_REG_ACCEL_RANGE,
		.pmu_cmd_normal = BMI160_CMD_ACCEL_PM_NORMAL,
		.pmu_cmd_suspend = BMI160_CMD_ACCEL_PM_SUSPEND,
	},
	[BMI160_GYRO] = {
		.data	= BMI160_REG_DATA_GYRO_XOUT_L,
		.config	= BMI160_REG_GYRO_CONFIG,
		.config_odr_mask = BMI160_GYRO_CONFIG_ODR_MASK,
		.config_bwp_mask = BMI160_GYRO_CONFIG_BWP_MASK,
		.range	= BMI160_REG_GYRO_RANGE,
		.pmu_cmd_normal = BMI160_CMD_GYRO_PM_NORMAL,
		.pmu_cmd_suspend = BMI160_CMD_GYRO_PM_SUSPEND,
	},
};

static unsigned long bmi160_pmu_time[] = {
	[BMI160_ACCEL] = BMI160_ACCEL_PMU_MIN_USLEEP,
	[BMI160_GYRO] = BMI160_GYRO_PMU_MIN_USLEEP,
};

struct bmi160_scale {
	u8 bits;
	int uscale;
};

struct bmi160_odr {
	u8 bits;
	int odr;
	int uodr;
};

struct bmi160_bwp {
	u8 bits;
	int bwp;
	int ubwp;
};

struct bmi160_bwp_by_odr {
	const struct bmi160_bwp * bwps;
	int num;
	u8 bits;
};

struct bmi160_bwp_info {
	const struct bmi160_bwp_by_odr * bwp_odr;
	int num;
};

static const struct bmi160_scale bmi160_accel_scale[] = {
	{ BMI160_ACCEL_RANGE_2G, 598},
	{ BMI160_ACCEL_RANGE_4G, 1197},
	{ BMI160_ACCEL_RANGE_8G, 2394},
	{ BMI160_ACCEL_RANGE_16G, 4788},
};

static const struct bmi160_scale bmi160_gyro_scale[] = {
	{ BMI160_GYRO_RANGE_2000DPS, 1065},
	{ BMI160_GYRO_RANGE_1000DPS, 532},
	{ BMI160_GYRO_RANGE_500DPS, 266},
	{ BMI160_GYRO_RANGE_250DPS, 133},
	{ BMI160_GYRO_RANGE_125DPS, 66},
};

struct bmi160_scale_item {
	const struct bmi160_scale *tbl;
	int num;
};

static const struct  bmi160_scale_item bmi160_scale_table[] = {
	[BMI160_ACCEL] = {
		.tbl	= bmi160_accel_scale,
		.num	= ARRAY_SIZE(bmi160_accel_scale),
	},
	[BMI160_GYRO] = {
		.tbl	= bmi160_gyro_scale,
		.num	= ARRAY_SIZE(bmi160_gyro_scale),
	},
};

static const struct bmi160_odr bmi160_accel_odr[] = {
	{0x01, 0, 781250},
	{0x02, 1, 562500},
	{0x03, 3, 125000},
	{0x04, 6, 250000},
	{0x05, 12, 500000},
	{0x06, 25, 0},
	{0x07, 50, 0},
	{0x08, 100, 0},
	{0x09, 200, 0},
	{0x0A, 400, 0},
	{0x0B, 800, 0},
	{0x0C, 1600, 0},
};

static const struct bmi160_odr bmi160_gyro_odr[] = {
	{0x06, 25, 0},
	{0x07, 50, 0},
	{0x08, 100, 0},
	{0x09, 200, 0},
	{0x0A, 400, 0},
	{0x0B, 800, 0},
	{0x0C, 1600, 0},
	{0x0D, 3200, 0},
};

struct bmi160_odr_item {
	const struct bmi160_odr *tbl;
	int num;
};

static const struct  bmi160_odr_item bmi160_odr_table[] = {
	[BMI160_ACCEL] = {
		.tbl	= bmi160_accel_odr,
		.num	= ARRAY_SIZE(bmi160_accel_odr),
	},
	[BMI160_GYRO] = {
		.tbl	= bmi160_gyro_odr,
		.num	= ARRAY_SIZE(bmi160_gyro_odr),
	},
};

/****** PWP ***************************/

static const struct bmi160_bwp bmi160_bwp_1600[] = {
	{0x02, 684, 0},
	{0x01, 324, 0},
	{0x00, 162, 0},
};

static const struct bmi160_bwp bmi160_bwp_800[] = {
	{0x02, 324, 0},
	{0x01, 162, 0},
	{0x00, 80, 0},
};

static const struct bmi160_bwp bmi160_bwp_400[] = {
	{0x02, 162, 0},
	{0x01, 80, 0},
	{0x00, 40, 500000},
};

static const struct bmi160_bwp bmi160_bwp_200[] = {
	{0x02, 80, 0},
	{0x01, 40, 500000},
	{0x00, 20, 250000},
};

static const struct bmi160_bwp bmi160_bwp_100[] = {
	{0x02, 40, 500000},
	{0x01, 20, 250000},
	{0x00, 10, 120000},
};

static const struct bmi160_bwp bmi160_bwp_50[] = {
	{0x02, 20, 250000},
	{0x01, 10, 120000},
	{0x00, 5, 60000},
};


static const struct bmi160_bwp_by_odr bmi160_bwps_tbl[] = {
	{
		.bwps = bmi160_bwp_50,
		.num = ARRAY_SIZE(bmi160_bwp_50),
		.bits = 0x07
	},
	{
		.bwps = bmi160_bwp_100,
		.num = ARRAY_SIZE(bmi160_bwp_100),
		.bits = 0x08
	},
	{
		.bwps = bmi160_bwp_200,
		.num = ARRAY_SIZE(bmi160_bwp_200),
		.bits = 0x09
	},
	{
		.bwps = bmi160_bwp_400,
		.num = ARRAY_SIZE(bmi160_bwp_400),
		.bits = 0x0A
	},
	{
		.bwps = bmi160_bwp_800,
		.num = ARRAY_SIZE(bmi160_bwp_800),
		.bits = 0x0B
	},
	{
		.bwps = bmi160_bwp_1600,
		.num = ARRAY_SIZE(bmi160_bwp_1600),
		.bits = 0x0C
	},
};

static const struct bmi160_bwp_info bmi160_bwp_infos = {
	.bwp_odr = bmi160_bwps_tbl,
	.num = ARRAY_SIZE(bmi160_bwps_tbl)
};
/***** IIO Events  *******************/
static const struct iio_event_spec bmi160_events[] = {
	{
		.type = IIO_EV_TYPE_ROC,
		.dir = IIO_EV_DIR_RISING,
		.mask_shared_by_dir = BIT(IIO_EV_INFO_VALUE) |
				 BIT(IIO_EV_INFO_ENABLE) |
				 BIT(IIO_EV_INFO_PERIOD)
	}, {
		.type = IIO_EV_TYPE_ROC,
		.dir = IIO_EV_DIR_FALLING,
		.mask_shared_by_dir = BIT(IIO_EV_INFO_VALUE) |
				 BIT(IIO_EV_INFO_ENABLE) |
				 BIT(IIO_EV_INFO_PERIOD)
	}
};


static const struct iio_chan_spec bmi160_channels[] = {
	BMI160_EVENT_CHANNEL(IIO_ACCEL, X, BMI160_SCAN_ACCEL_X),
	BMI160_EVENT_CHANNEL(IIO_ACCEL, Y, BMI160_SCAN_ACCEL_Y),
	BMI160_EVENT_CHANNEL(IIO_ACCEL, Z, BMI160_SCAN_ACCEL_Z),
	BMI160_CHANNEL(IIO_ANGL_VEL, X, BMI160_SCAN_GYRO_X),
	BMI160_CHANNEL(IIO_ANGL_VEL, Y, BMI160_SCAN_GYRO_Y),
	BMI160_CHANNEL(IIO_ANGL_VEL, Z, BMI160_SCAN_GYRO_Z),
	IIO_CHAN_SOFT_TIMESTAMP(BMI160_SCAN_TIMESTAMP),
};

static enum bmi160_sensor_type bmi160_to_sensor(enum iio_chan_type iio_type)
{
	switch (iio_type) {
	case IIO_ACCEL:
		return BMI160_ACCEL;
	case IIO_ANGL_VEL:
		return BMI160_GYRO;
	default:
		return -EINVAL;
	}
}

static
int bmi160_set_mode(struct bmi160_data *data, enum bmi160_sensor_type t,
		    bool mode)
{
	int ret;
	u8 cmd;

	if (mode)
		cmd = bmi160_regs[t].pmu_cmd_normal;
	else
		cmd = bmi160_regs[t].pmu_cmd_suspend;

	ret = regmap_write(data->regmap, BMI160_REG_CMD, cmd);
	if (ret)
		return ret;

	usleep_range(bmi160_pmu_time[t], bmi160_pmu_time[t] + 1000);

	return 0;
}

static
int bmi160_set_scale(struct bmi160_data *data, enum bmi160_sensor_type t,
		     int uscale)
{
	int i;

	for (i = 0; i < bmi160_scale_table[t].num; i++)
		if (bmi160_scale_table[t].tbl[i].uscale == uscale)
			break;

	if (i == bmi160_scale_table[t].num)
		return -EINVAL;

	return regmap_write(data->regmap, bmi160_regs[t].range,
			    bmi160_scale_table[t].tbl[i].bits);
}

static
int bmi160_get_scale(struct bmi160_data *data, enum bmi160_sensor_type t,
		     int *uscale)
{
	int i, ret, val;

	ret = regmap_read(data->regmap, bmi160_regs[t].range, &val);
	if (ret)
		return ret;

	for (i = 0; i < bmi160_scale_table[t].num; i++)
		if (bmi160_scale_table[t].tbl[i].bits == val) {
			*uscale = bmi160_scale_table[t].tbl[i].uscale;
			return 0;
		}

	return -EINVAL;
}

static int bmi160_get_data(struct bmi160_data *data, int chan_type,
			   int axis, int *val)
{
	u8 reg;
	int ret;
	__le16 sample;
	enum bmi160_sensor_type t = bmi160_to_sensor(chan_type);

	reg = bmi160_regs[t].data + (axis - IIO_MOD_X) * sizeof(sample);

	ret = regmap_bulk_read(data->regmap, reg, &sample, sizeof(sample));
	if (ret)
		return ret;

	*val = sign_extend32(le16_to_cpu(sample), 15);

	return 0;
}

static
int bmi160_set_odr(struct bmi160_data *data, enum bmi160_sensor_type t,
		   int odr, int uodr)
{
	int i;

	for (i = 0; i < bmi160_odr_table[t].num; i++)
		if (bmi160_odr_table[t].tbl[i].odr == odr &&
		    bmi160_odr_table[t].tbl[i].uodr == uodr)
			break;

	if (i >= bmi160_odr_table[t].num)
		return -EINVAL;

	return regmap_update_bits(data->regmap,
				  bmi160_regs[t].config,
				  bmi160_regs[t].config_odr_mask,
				  bmi160_odr_table[t].tbl[i].bits);
}

static int bmi160_get_odr(struct bmi160_data *data, enum bmi160_sensor_type t,
			  int *odr, int *uodr)
{
	int i, val, ret;

	ret = regmap_read(data->regmap, bmi160_regs[t].config, &val);
	if (ret)
		return ret;

	val &= bmi160_regs[t].config_odr_mask;

	for (i = 0; i < bmi160_odr_table[t].num; i++)
		if (val == bmi160_odr_table[t].tbl[i].bits)
			break;

	if (i >= bmi160_odr_table[t].num)
		return -EINVAL;

	*odr = bmi160_odr_table[t].tbl[i].odr;
	*uodr = bmi160_odr_table[t].tbl[i].uodr;

	return 0;
}

static int bmi160_accel_set_bwp(struct bmi160_data *data, int bwp, int ubwp)
{
	int i, num, ret;
	unsigned int val, odr;

	ret = regmap_read(data->regmap, BMI160_REG_ACCEL_CONFIG, &val);
	if (ret)
		return ret;
	odr = val & BMI160_ACCEL_CONFIG_ODR_MASK;

	for (i = 0; i < bmi160_bwp_infos.num; i++)
	{
		if (bmi160_bwp_infos.bwp_odr[i].bits == odr)
			break;
	}
	if (i >= bmi160_bwp_infos.num)
		return -EINVAL;

	const struct bmi160_bwp * bwp_item = bmi160_bwp_infos.bwp_odr[i].bwps;
	num = bmi160_bwp_infos.bwp_odr[i].num;
	
	for (i = 0; i < num; i++)
	{
		if (bwp_item[i].bwp == bwp &&
			bwp_item[i].ubwp == ubwp)
			break;
	}
	if (i >= num)
		return -EINVAL;

	val = (val & ~BMI160_ACCEL_CONFIG_BWP_MASK) | ((bwp_item[i].bits << BMI160_ACCEL_CONFIG_BWP_SHIFT) & BMI160_ACCEL_CONFIG_BWP_MASK);
	
	ret = regmap_write(data->regmap, BMI160_REG_ACCEL_CONFIG, val);

	if (ret)
		return ret;

	usleep_range(BMI160_NORMAL_WRITE_USLEEP, BMI160_NORMAL_WRITE_USLEEP + 1000);

	return 0;

}

static int bmi160_accel_get_bwp(struct bmi160_data *data, int *bwp, int *ubwp)
{
	int i, num, ret;
	unsigned int val, odr;

	ret = regmap_read(data->regmap, BMI160_REG_ACCEL_CONFIG, &val);
	if (ret)
		return ret;
	odr = val & BMI160_ACCEL_CONFIG_ODR_MASK;

	for (i = 0; i < bmi160_bwp_infos.num; i++)
	{
		if (bmi160_bwp_infos.bwp_odr[i].bits == odr)
			break;
	}
	if (i >= bmi160_bwp_infos.num)
		return -EINVAL;

	const struct bmi160_bwp * bwp_item = bmi160_bwp_infos.bwp_odr[i].bwps;
	num = bmi160_bwp_infos.bwp_odr[i].num;
	val = (val & BMI160_ACCEL_CONFIG_BWP_MASK) >> BMI160_ACCEL_CONFIG_BWP_SHIFT;
	
	for (i = 0; i < num; i++)
	{
		if (bwp_item[i].bits == val)
			break;
	}
	if (i >= num)
		return -EINVAL;

	*bwp = bwp_item[i].bwp;
	*ubwp =	bwp_item[i].ubwp;
	return 0;
}

static int bmi160_write_conf_reg(struct regmap *regmap, unsigned int reg,
				 unsigned int mask, unsigned int bits,
				 unsigned int write_usleep)
{
	int ret;
	unsigned int val;

	ret = regmap_read(regmap, reg, &val);
	if (ret)
		return ret;

	val = (val & ~mask) | bits;

	ret = regmap_write(regmap, reg, val);
	if (ret)
		return ret;

	/*
	 * We need to wait after writing before we can write again. See the
	 * datasheet, page 93.
	 */
	usleep_range(write_usleep, write_usleep + 1000);

	return 0;
}

static int bmi160_anymotion_update_slope(struct bmi160_data *data)
{
	struct device *dev = regmap_get_device(data->regmap);
	int ret;

	ret = bmi160_write_conf_reg(data->regmap, BMI160_REG_INT_DURATION,
				     BMI160_ANYMOTION_DUR_MASK, data->slope_anymot_dur & BMI160_ANYMOTION_DUR_MASK,
				     BMI160_NORMAL_WRITE_USLEEP);
	if (ret)
		return ret;

	ret = regmap_write(data->regmap, BMI160_REG_INT_ANYMOT_THRES, data->slope_anymot_thres);
	if (ret)
		return ret;

	usleep_range(BMI160_NORMAL_WRITE_USLEEP, BMI160_NORMAL_WRITE_USLEEP + 1000);

	data->anymot_mustbe_updated = 0;
	return 0;
}

static int bmi160_nomotion_update_slope(struct bmi160_data *data)
{
	struct device *dev = regmap_get_device(data->regmap);
	int ret;
	ret = bmi160_write_conf_reg(data->regmap, BMI160_REG_INT_DURATION,
				     BMI160_NOMOTION_DUR_MASK << BMI160_NOMOTION_DUR_SHIFT, 
					 (data->slope_nomot_dur & BMI160_NOMOTION_DUR_MASK) << BMI160_NOMOTION_DUR_SHIFT,
				     BMI160_NORMAL_WRITE_USLEEP);
	if (ret)
		return ret;

	ret = regmap_write(data->regmap, BMI160_REG_INT_NOMOT_THRES, data->slope_nomot_thres);
	if (ret)
		return ret;

	usleep_range(BMI160_NORMAL_WRITE_USLEEP, BMI160_NORMAL_WRITE_USLEEP + 1000);

	data->nomot_mustbe_updated = 0;
	return 0;
}

static int bmi160_accel_any_motion_setup(struct bmi160_accel_trigger *t,
					 bool state)
{
	if (state)
		return bmi160_anymotion_update_slope(t->data);

	return 0;
}

static int bmi160_accel_no_motion_setup(struct bmi160_accel_trigger *t,
					 bool state)
{
	if (state)
		return bmi160_nomotion_update_slope(t->data);

	return 0;
}

static irqreturn_t bmi160_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct bmi160_data *data = iio_priv(indio_dev);
	__le16 buf[16];
	/* 3 sens x 3 axis x __le16 + 3 x __le16 pad + 4 x __le16 tstamp */
	int i, ret, j = 0, base = BMI160_REG_DATA_MAGN_XOUT_L;
	__le16 sample;

	mutex_lock(&data->mutex);
	for_each_set_bit(i, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		ret = regmap_bulk_read(data->regmap, base + i * sizeof(sample),
				       &sample, sizeof(sample));
		if (ret)
		{
			mutex_unlock(&data->mutex);
			goto done;
		}
		buf[j++] = sample;
	}
	mutex_unlock(&data->mutex);

	iio_push_to_buffers_with_timestamp(indio_dev, buf, pf->timestamp);
done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static int bmi160_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	int ret;
	struct bmi160_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&data->mutex);
		ret = bmi160_get_data(data, chan->type, chan->channel2, val);
		mutex_unlock(&data->mutex);
		if (ret)
			return ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		mutex_lock(&data->mutex);
		ret = bmi160_get_scale(data,
				       bmi160_to_sensor(chan->type), val2);
		mutex_unlock(&data->mutex);
		return ret ? ret : IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SAMP_FREQ:
		mutex_lock(&data->mutex);
		ret = bmi160_get_odr(data, bmi160_to_sensor(chan->type),
				     val, val2);
		mutex_unlock(&data->mutex);
		return ret ? ret : IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		if (chan->type != IIO_ACCEL)
			return -EINVAL;
		mutex_lock(&data->mutex);
		ret = bmi160_accel_get_bwp(data, val, val2);
		mutex_unlock(&data->mutex);
		return ret ? ret : IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bmi160_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	int ret;
	struct bmi160_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		mutex_lock(&data->mutex);
		ret = bmi160_set_scale(data,
					bmi160_to_sensor(chan->type), val2);
		mutex_unlock(&data->mutex);
		return ret;
	case IIO_CHAN_INFO_SAMP_FREQ:
		mutex_lock(&data->mutex);
		ret = bmi160_set_odr(data, bmi160_to_sensor(chan->type),
				      val, val2);
		mutex_unlock(&data->mutex);
		return ret;
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		if (chan->type != IIO_ACCEL)
			return -EINVAL;
		mutex_lock(&data->mutex);
		ret = bmi160_accel_set_bwp(data, val, val2);
		mutex_unlock(&data->mutex);
		return ret;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct bmi160_accel_interrupt_info {
	u8 map1_reg;
	u8 map1_bitmask;
	u8 map2_reg;
	u8 map2_bitmask;
	u8 en_reg;
	u8 en_bitmask;
	u8 type;
} bmi160_accel_interrupts[BMI160_ACCEL_INTERRUPTS] = {
	{ /* data ready interrupt */
		.map1_reg = BMI160_REG_INT_MAP1,
		.map1_bitmask = BMI160_INT1_MAP_DRDY_EN,
		.map2_reg = BMI160_REG_INT_MAP1,
		.map2_bitmask = BMI160_INT2_MAP_DRDY_EN,
		.en_reg = BMI160_REG_INT_EN_1,
		.en_bitmask = BMI160_DRDY_INT_EN,
		.type = BMI160_DATA_DRIVEN_INTERRUPT
	},
	{  /* any motion interrupt */
		.map1_reg = BMI160_REG_INT_MAP0,
		.map1_bitmask = BMI160_INT_MAP_ANYMOTION_EN,
		.map2_reg = BMI160_REG_INT_MAP2,
		.map2_bitmask = BMI160_INT_MAP_ANYMOTION_EN,
		.en_reg = BMI160_REG_INT_EN_0,
		.en_bitmask =  BMI160_ANYMOTION_X_INT_EN |
			BMI160_ANYMOTION_Y_INT_EN |
			BMI160_ANYMOTION_Z_INT_EN,
		.type = BMI160_PHYSICAL_INTERRUPT
	},
	{ /* no motion interrupt */
		.map1_reg = BMI160_REG_INT_MAP0,
		.map1_bitmask = BMI160_INT_MAP_NOMOTION_EN,
		.map2_reg = BMI160_REG_INT_MAP2,
		.map2_bitmask = BMI160_INT_MAP_NOMOTION_EN,
		.en_reg = BMI160_REG_INT_EN_2,
		.en_bitmask = BMI160_NOMOTION_X_INT_EN |
			BMI160_NOMOTION_Y_INT_EN |
			BMI160_NOMOTION_Z_INT_EN,
		.type = BMI160_PHYSICAL_INTERRUPT
	},
};

static int bmi160_get_map_mask(u8 pin, const struct bmi160_accel_interrupt_info *info, 
			u8 * reg_map, u8 * bit_map) 
{
	if (pin == BMI160_PIN_INT1) {
		*reg_map = info->map1_reg;
		*bit_map = info->map1_bitmask;
	}
	else if (pin == BMI160_PIN_INT2) {
		*reg_map = info->map2_reg;
		*bit_map = info->map2_bitmask;
	}
	else {
		return -EINVAL;
	}
	return 0;
}

static int bmi160_accel_set_interrupt(struct bmi160_data *data, int i,
				      bool state)
{
	struct device *dev = regmap_get_device(data->regmap);
	struct bmi160_accel_interrupt *intr = &data->interrupts[i];
	const struct bmi160_accel_interrupt_info *info = intr->info;
	int ret;
	u8 map_reg, map_bitmask;
	u8 map_bit;
	u8 int_type = info->type;

	ret = bmi160_get_map_mask(data->intr_pin, info, &map_reg, &map_bitmask);
	if (ret < 0) {
		dev_err(dev, "Interrupt pin number is not set\n");
		return -EINVAL;
	}

	if (state) {
		if (atomic_inc_return(&intr->users) > 1)
			return 0;
	} else {
		if (atomic_dec_return(&intr->users) > 0)
			return 0;
	}

	/* map the interrupt to the appropriate pins */
	map_bit = state ? map_bitmask : 0;
	/* interrupt pin output must be disabled if physical interrupt*/
	if (state && (int_type == BMI160_PHYSICAL_INTERRUPT))
	{
		for (i = 0; i < BMI160_ACCEL_INTERRUPTS; i++)
		{
			intr = &data->interrupts[i];
			if (intr->info->type == BMI160_DATA_DRIVEN_INTERRUPT)
			{
				if (atomic_read(&intr->users) > 0)
				{
					map_bit = 0;
					break;
				}
			}
		}
	}

	ret = bmi160_write_conf_reg(data->regmap, map_reg,
				     map_bitmask, 
					 map_bit,
				     BMI160_NORMAL_WRITE_USLEEP);

	if (ret < 0) {
		dev_err(dev, "Error updating reg_int_map\n");
		return ret;
	}

	/* check if other interrupt pin must be disabled */ 
	if (int_type == BMI160_DATA_DRIVEN_INTERRUPT)
	{
		for (i = 0; i < BMI160_ACCEL_INTERRUPTS; i++)
		{
			intr = &data->interrupts[i];
			if (intr->info->type == BMI160_PHYSICAL_INTERRUPT)
			{
				if (atomic_read(&intr->users) > 0)
				{
					ret = bmi160_get_map_mask(data->intr_pin, intr->info, 
										&map_reg, &map_bitmask);
					if (ret == 0) {	
						/* disable or enable other interrupt pin */
						ret = bmi160_write_conf_reg(data->regmap, map_reg,
				    			map_bitmask, state ? 0 : map_bitmask,
				    			BMI160_NORMAL_WRITE_USLEEP);
						if (ret < 0) {
							dev_err(dev, "Error updating other reg_int_map\n");
						}
					}					
				}
			}
		}
	}

	/* enable/disable the interrupt */
	ret = bmi160_write_conf_reg(data->regmap, info->en_reg,
				     info->en_bitmask, 
					 (state ? info->en_bitmask : 0),
				     BMI160_NORMAL_WRITE_USLEEP);

	if (ret < 0) {
		dev_err(dev, "Error updating reg_int_en\n");
		return ret;
	}

	return 0;
}

static int bmi160_accel_read_event(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan,
				   enum iio_event_type type,
				   enum iio_event_direction dir,
				   enum iio_event_info info,
				   int *val, int *val2)
{
	struct bmi160_data *data = iio_priv(indio_dev);
	*val2 = 0;

	if (type != IIO_EV_TYPE_ROC)
		return -EINVAL;

	switch (dir) {
	case IIO_EV_DIR_RISING:
		switch (info) {
		case IIO_EV_INFO_VALUE:
			*val = data->slope_anymot_thres;
			break;
		case IIO_EV_INFO_PERIOD:
			*val = data->slope_anymot_dur;
			break;
		default:
			return -EINVAL;
		}
		break;
	case IIO_EV_DIR_FALLING:
		switch (info) {
		case IIO_EV_INFO_VALUE:
			*val = data->slope_nomot_thres;
			break;
		case IIO_EV_INFO_PERIOD:
			*val = data->slope_nomot_dur;
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}



	return IIO_VAL_INT;
}

static int bmi160_accel_write_event(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir,
				    enum iio_event_info info,
				    int val, int val2)
{
	struct bmi160_data *data = iio_priv(indio_dev);

	if (type != IIO_EV_TYPE_ROC)
		return -EINVAL;

	switch (dir) {
	case IIO_EV_DIR_RISING:
		if (data->ev_anymot_enable_state)
			return -EBUSY;
		switch (info) {
		case IIO_EV_INFO_VALUE:
			data->slope_anymot_thres = val & 0xff;
			break;
		case IIO_EV_INFO_PERIOD:
			data->slope_anymot_dur = val & BMI160_ANYMOTION_DUR_MASK;
			break;
		default:
			return -EINVAL;
		}
		data->anymot_mustbe_updated = 1;
		break;
	case IIO_EV_DIR_FALLING:
		if (data->ev_nomot_enable_state)
			return -EBUSY;
		switch (info) {
		case IIO_EV_INFO_VALUE:
			data->slope_nomot_thres = val & 0xff;
			break;
		case IIO_EV_INFO_PERIOD:
			data->slope_nomot_dur = val & BMI160_NOMOTION_DUR_MASK;
			break;
		default:
			return -EINVAL;
		}
		data->nomot_mustbe_updated = 1;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bmi160_accel_read_event_config(struct iio_dev *indio_dev,
					  const struct iio_chan_spec *chan,
					  enum iio_event_type type,
					  enum iio_event_direction dir)
{
	struct bmi160_data *data = iio_priv(indio_dev);
	int state;
		
	if (type != IIO_EV_TYPE_ROC)
		return -EINVAL;

	switch (dir) {
	case IIO_EV_DIR_RISING:
		state = data->ev_anymot_enable_state;
		break;
	case IIO_EV_DIR_FALLING:
		state = data->ev_nomot_enable_state;
		break;
	default:
		return -EINVAL;
	}
	return state;
}

static int bmi160_accel_write_event_config(struct iio_dev *indio_dev,
					   const struct iio_chan_spec *chan,
					   enum iio_event_type type,
					   enum iio_event_direction dir,
					   int state)
{
	struct bmi160_data *data = iio_priv(indio_dev);
	int ret;
	int intr;
	int *pre_state;

	if (type != IIO_EV_TYPE_ROC)
		return -EINVAL;

	switch (dir) {
	case IIO_EV_DIR_RISING:
		pre_state = &data->ev_anymot_enable_state;
		intr = BMI160_ACCEL_INT_ANY_MOTION;
		break;
	case IIO_EV_DIR_FALLING:
		pre_state = &data->ev_nomot_enable_state;
		intr = BMI160_ACCEL_INT_NO_MOTION;
		break;
	default:
		return -EINVAL;
	}

	if (state == *pre_state)
		return 0;

	mutex_lock(&data->mutex);

	if (state) {
		if (dir == IIO_EV_DIR_RISING) {
			if (data->anymot_mustbe_updated) {
				ret = bmi160_anymotion_update_slope(data);
			}
		} else {
			if (data->nomot_mustbe_updated) {
				ret = bmi160_nomotion_update_slope(data);
			}
		}
	}
	
	if (ret == 0 )
		ret = bmi160_accel_set_interrupt(data, intr, (bool)state);

	if (ret < 0) {
		mutex_unlock(&data->mutex);
		return ret;
	}

	*pre_state = state;
	mutex_unlock(&data->mutex);

	return 0;
}

static ssize_t bmi160_filter_frequency_available(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int ret, i, num;
	unsigned int val;
	int len=0;

	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bmi160_data *data = iio_priv(indio_dev);

	mutex_lock(&data->mutex);

	ret = regmap_read(data->regmap, BMI160_REG_ACCEL_CONFIG, &val);
	mutex_unlock(&data->mutex);
	if (ret)
		return ret;

	val &= BMI160_ACCEL_CONFIG_ODR_MASK;

	for (i = 0; i < bmi160_bwp_infos.num; i++)
	{
		if (bmi160_bwp_infos.bwp_odr[i].bits == val)
			break;
	}
	if (i >= bmi160_bwp_infos.num)
		return -EINVAL;

	const struct bmi160_bwp * bwp_item = bmi160_bwp_infos.bwp_odr[i].bwps;
	num = bmi160_bwp_infos.bwp_odr[i].num;
	
	for (i = 0; i < num; i++)
	{
		len += sprintf(buf+len,"%d.%d ", bwp_item[i].bwp, bwp_item[i].ubwp);
	}
	return len;

}

static ssize_t bmi160_gyro_get_offset_calib(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int ret;
	unsigned int val;
	bool enable;

	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bmi160_data *data = iio_priv(indio_dev);

	mutex_lock(&data->mutex);
	ret = regmap_read(data->regmap, BMI160_REG_FOC_CONF, &val);
	mutex_unlock(&data->mutex);
	if (ret)
		return ret;

	enable = val & BMI160_FOC_GYR_EN_MASK;

	return sprintf(buf,"%d\n", enable);
}

static ssize_t bmi160_gyro_set_offset_calib(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	int ret=0;
	unsigned int val=0;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bmi160_data *data = iio_priv(indio_dev);

	if (sysfs_streq(buf, "1") || sysfs_streq(buf, "on"))
		val=BMI160_FOC_GYR_EN_MASK;
	else if (sysfs_streq(buf, "0") || sysfs_streq(buf, "off"))
		val=0;
	else 
		return -EINVAL;

	mutex_lock(&data->mutex);
	ret = bmi160_write_conf_reg(data->regmap, BMI160_REG_FOC_CONF,
				     BMI160_FOC_GYR_EN_MASK, val,
				     BMI160_NORMAL_WRITE_USLEEP);

	mutex_unlock(&data->mutex);

	return ret ? ret : len;
}

static ssize_t bmi160_accel_get_offset_calib(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int ret;
	unsigned int val;

	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bmi160_data *data = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);

	mutex_lock(&data->mutex);
	ret = regmap_read(data->regmap, BMI160_REG_FOC_CONF, &val);
	mutex_unlock(&data->mutex);

	if (ret)
		return ret;

	val >>= (u32)this_attr->address;
	val &= BMI160_FOC_ACC_MASK;

	char *str;
	switch (val) {
	case 0:
		str = "disable";
	break;
	case 1:
		str= "+1g";
	break;
	case 2:
		str = "-1g";
	break;
	case 3:
		str = "0g";
	break;
	}

	return sprintf(buf,str);
}

static ssize_t bmi160_accel_set_offset_calib(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	int ret=0;
	unsigned int val=0;
	unsigned int mask;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bmi160_data *data = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);

	if (sysfs_streq(buf, "0") || sysfs_streq(buf, "disable"))
		val=0;
	else if (sysfs_streq(buf, "+1g"))
		val=1;
	else if (sysfs_streq(buf, "-1g"))
		val=2;
	else if (sysfs_streq(buf, "0g"))
		val=3;
	else
		return -EINVAL;
	
	val <<= (u32)this_attr->address;
	mask = BMI160_FOC_ACC_MASK << (u32)this_attr->address;

	if (!ret)
	{
		mutex_lock(&data->mutex);
		ret = bmi160_write_conf_reg(data->regmap, BMI160_REG_FOC_CONF,
				     mask, val,
				     BMI160_NORMAL_WRITE_USLEEP);
		mutex_unlock(&data->mutex);
	}

	return ret ? ret : len;
}

static ssize_t bmi160_get_offset_enable(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int ret;
	unsigned int val;
	bool enable;

	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bmi160_data *data = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);

	mutex_lock(&data->mutex);
	ret = regmap_read(data->regmap, BMI160_REG_OFFSET, &val);
	mutex_unlock(&data->mutex);
	if (ret)
		return ret;

	
	enable = val & (u32)this_attr->address;

	return sprintf(buf,"%d\n", enable);
}

static ssize_t bmi160_set_offset_enable(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	int ret=0;
	unsigned int val=0;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bmi160_data *data = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);

	if (sysfs_streq(buf, "1") || sysfs_streq(buf, "on"))
		val=(u32)this_attr->address;
	else if (sysfs_streq(buf, "0") || sysfs_streq(buf, "off"))
		val=0;
	else 
		return -EINVAL;

	mutex_lock(&data->mutex);
	ret = bmi160_write_conf_reg(data->regmap, BMI160_REG_OFFSET,
				     (u32)this_attr->address, val,
				     BMI160_NORMAL_WRITE_USLEEP);
	mutex_unlock(&data->mutex);

	return ret ? ret : len;
}


static ssize_t bmi160_foc_start(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	int ret=0;
	int timeout = 0;
	unsigned int val=0;
	int i, base;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bmi160_data *data = iio_priv(indio_dev);

	mutex_lock(&data->mutex);
	ret = regmap_write(data->regmap, BMI160_REG_CMD, BMI160_CMD_START_FOC);
	if (ret)
	{
		mutex_unlock(&data->mutex);
		return ret;
	}
	usleep_range(BMI160_NORMAL_WRITE_USLEEP, BMI160_NORMAL_WRITE_USLEEP + 1000);
	mutex_unlock(&data->mutex);

	msleep(25);
	while (ret == 0 && (val & BMI160_FOC_RDY) == 0 && timeout < 11)
	{
		mutex_lock(&data->mutex);
		ret = regmap_read(data->regmap, BMI160_REG_STATUS, &val);
		mutex_unlock(&data->mutex);
		msleep(25);
		timeout++;
	}
	
	if (ret)
		return ret;

	if ((val & BMI160_FOC_RDY) == 0)
		return -ETIME;

	base = BMI160_REG_DATA_MAGN_XOUT_L;
	__le16 sample;

	mutex_lock(&data->mutex);
	for_each_set_bit(i, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		ret = regmap_bulk_read(data->regmap, base + i * sizeof(sample),
				       &sample, sizeof(sample));
		if (ret)
			break;
	}
	mutex_unlock(&data->mutex);
	return ret ? ret : len;
}

static ssize_t bmi160_get_int_motion_src(struct device *dev,
				 struct device_attribute *attr, char *buf) {
	int ret;
	unsigned int val;
	bool pre_filtered;

	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bmi160_data *data = iio_priv(indio_dev);

	mutex_lock(&data->mutex);
	ret = regmap_read(data->regmap, BMI160_REG_INT_DATA_1, &val);
	mutex_unlock(&data->mutex);
	if (ret)
		return ret;

	
	pre_filtered = val & BMI160_INT_MOTION_SRC;

	return sprintf(buf,"%d\n", pre_filtered);
}


static ssize_t bmi160_set_int_motion_src(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len) {
	int ret=0;
	unsigned int val=0;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bmi160_data *data = iio_priv(indio_dev);

	if (sysfs_streq(buf, "1") || sysfs_streq(buf, "on"))
		val=BMI160_INT_MOTION_SRC;
	else if (sysfs_streq(buf, "0") || sysfs_streq(buf, "off"))
		val=0;
	else 
		return -EINVAL;

	mutex_lock(&data->mutex);
	ret = regmap_write(data->regmap, BMI160_REG_INT_DATA_1, val);
	if (ret)
	{
		mutex_unlock(&data->mutex);
		return ret;
	}
	usleep_range(BMI160_NORMAL_WRITE_USLEEP, BMI160_NORMAL_WRITE_USLEEP + 1000);
	mutex_unlock(&data->mutex);
	return len;
}


static
IIO_CONST_ATTR(in_accel_sampling_frequency_available,
	       "0.78125 1.5625 3.125 6.25 12.5 25 50 100 200 400 800 1600");
static
IIO_CONST_ATTR(in_anglvel_sampling_frequency_available,
	       "25 50 100 200 400 800 1600 3200");
static
IIO_CONST_ATTR(in_accel_scale_available,
	       "0.000598 0.001197 0.002394 0.004788");
static
IIO_CONST_ATTR(in_anglvel_scale_available,
	       "0.001065 0.000532 0.000266 0.000133 0.000066");
static
IIO_CONST_ATTR(in_accel_calib_available,
	       "disabled +1g -1g 0g");

static 
IIO_DEVICE_ATTR(in_accel_filter_frequency_available, S_IRUGO,
		    bmi160_filter_frequency_available, NULL, 0);

static 
IIO_DEVICE_ATTR(gyro_calib_config, S_IRUGO | S_IWUSR,
		    bmi160_gyro_get_offset_calib, bmi160_gyro_set_offset_calib, 0);
static 
IIO_DEVICE_ATTR(accel_x_calib_config, S_IRUGO | S_IWUSR,
		    bmi160_accel_get_offset_calib, bmi160_accel_set_offset_calib, BMI160_FOC_ACC_X_SHIFT);
static
IIO_DEVICE_ATTR(accel_y_calib_config, S_IRUGO | S_IWUSR,
		    bmi160_accel_get_offset_calib, bmi160_accel_set_offset_calib, BMI160_FOC_ACC_Y_SHIFT);
static
IIO_DEVICE_ATTR(accel_z_calib_config, S_IRUGO | S_IWUSR,
		    bmi160_accel_get_offset_calib, bmi160_accel_set_offset_calib, BMI160_FOC_ACC_Z_SHIFT);

static
IIO_DEVICE_ATTR(accel_offset_enable, S_IRUGO | S_IWUSR,
		    bmi160_get_offset_enable, bmi160_set_offset_enable, BMI160_ACC_OFF_EN_MASK);
static
IIO_DEVICE_ATTR(gyro_offset_enable, S_IRUGO | S_IWUSR,
		    bmi160_get_offset_enable, bmi160_set_offset_enable, BMI160_GYR_OFF_EN_MASK);
static
IIO_DEVICE_ATTR(foc_start, S_IWUSR,
		    NULL, bmi160_foc_start, 0); 

static
IIO_DEVICE_ATTR(accel_int_motion_src, S_IRUGO | S_IWUSR,
		    bmi160_get_int_motion_src, bmi160_set_int_motion_src, 0);


static struct attribute *bmi160_attrs[] = {
	&iio_const_attr_in_accel_sampling_frequency_available.dev_attr.attr,
	&iio_const_attr_in_anglvel_sampling_frequency_available.dev_attr.attr,
	&iio_const_attr_in_accel_scale_available.dev_attr.attr,
	&iio_const_attr_in_anglvel_scale_available.dev_attr.attr,
	&iio_const_attr_in_accel_calib_available.dev_attr.attr,
	&iio_dev_attr_in_accel_filter_frequency_available.dev_attr.attr,
	&iio_dev_attr_gyro_calib_config.dev_attr.attr,
	&iio_dev_attr_accel_x_calib_config.dev_attr.attr,
	&iio_dev_attr_accel_y_calib_config.dev_attr.attr,
	&iio_dev_attr_accel_z_calib_config.dev_attr.attr,
	&iio_dev_attr_accel_offset_enable.dev_attr.attr,
	&iio_dev_attr_gyro_offset_enable.dev_attr.attr,
	&iio_dev_attr_foc_start.dev_attr.attr,
	&iio_dev_attr_accel_int_motion_src.dev_attr.attr,
	NULL
};

static const struct attribute_group bmi160_attrs_group = {
	.attrs = bmi160_attrs,
};

static const struct iio_info bmi160_info = {
	.read_raw = bmi160_read_raw,
	.write_raw = bmi160_write_raw,
	.attrs = &bmi160_attrs_group,
	.read_event_value	= bmi160_accel_read_event,
	.write_event_value	= bmi160_accel_write_event,
	.write_event_config	= bmi160_accel_write_event_config,
	.read_event_config	= bmi160_accel_read_event_config,
	.driver_module		= THIS_MODULE,
};

static const char *bmi160_match_acpi_device(struct device *dev)
{
	const struct acpi_device_id *id;

	id = acpi_match_device(dev->driver->acpi_match_table, dev);
	if (!id)
		return NULL;

	return dev_name(dev);
}


static void bmi160_accel_interrupts_setup(struct iio_dev *indio_dev,
					  struct bmi160_data *data)
{
	int i;

	for (i = 0; i < BMI160_ACCEL_INTERRUPTS; i++)
		data->interrupts[i].info = &bmi160_accel_interrupts[i];
}

static int bmi160_int_reset(struct regmap *regmap,
								unsigned long write_usleep)
{
	int ret;
	ret = regmap_write(regmap, BMI160_REG_CMD, BMI160_CMD_INTRESET);
	if (ret)
		return ret;
	usleep_range(write_usleep, write_usleep + 1000);

	return ret;
}

static int bmi160_latch_set(struct regmap *regmap,
								unsigned long write_usleep) {
	int ret;
	struct device *dev = regmap_get_device(regmap);

	ret = regmap_write(regmap, BMI160_REG_INT_LATCH, 
							0);
	if (ret)
		return ret;

	usleep_range(write_usleep, write_usleep + 1000);

	return 0;
}

static int bmi160_config_pin(struct regmap *regmap, enum bmi160_int_pin pin,
			     bool open_drain, u8 irq_mask,
			     unsigned long write_usleep)
{
	int ret;
	struct device *dev = regmap_get_device(regmap);
	u8 int_out_ctrl_shift;
	u8 int_out_ctrl_mask;
	u8 int_out_ctrl_bits;
	const char *pin_name;

	switch (pin) {
	case BMI160_PIN_INT1:
		int_out_ctrl_shift = BMI160_INT1_OUT_CTRL_SHIFT;
		break;
	case BMI160_PIN_INT2:
		int_out_ctrl_shift = BMI160_INT2_OUT_CTRL_SHIFT;
		break;
	}
	int_out_ctrl_mask = BMI160_INT_OUT_CTRL_MASK << int_out_ctrl_shift;

	/*
	 * Enable the requested pin with the right settings:
	 * - Push-pull/open-drain
	 * - Active low/high
	 * - Edge/level triggered
	 */
	int_out_ctrl_bits = BMI160_OUTPUT_EN;
	if (open_drain)
		/* Default is push-pull. */
		int_out_ctrl_bits |= BMI160_OPEN_DRAIN;
	int_out_ctrl_bits |= irq_mask;
	int_out_ctrl_bits <<= int_out_ctrl_shift;

	ret = bmi160_write_conf_reg(regmap, BMI160_REG_INT_OUT_CTRL,
				    int_out_ctrl_mask, int_out_ctrl_bits,
				    write_usleep);
	if (ret)
		return ret;

	/* !M1 Set the pin to output mode with latching. */
	ret = bmi160_latch_set(regmap, write_usleep);
	
	if (ret) {
		switch (pin) {
		case BMI160_PIN_INT1:
			pin_name = "INT1";
			break;
		case BMI160_PIN_INT2:
			pin_name = "INT2";
			break;
		}
		dev_err(dev, "Failed to configure %s IRQ pin", pin_name);
	}

	return ret;
}

static int bmi160_get_irq(struct device_node *of_node, enum bmi160_int_pin *pin)
{
	int irq;

	/* Use INT1 if possible, otherwise fall back to INT2. */
	irq = of_irq_get_byname(of_node, "INT1");
	if (irq > 0) {
		*pin = BMI160_PIN_INT1;
		return irq;
	}

	irq = of_irq_get_byname(of_node, "INT2");
	if (irq > 0)
		*pin = BMI160_PIN_INT2;

	return irq;
}

static int bmi160_config_device_irq(struct iio_dev *indio_dev, int irq_type,
				    enum bmi160_int_pin pin)
{
	bool open_drain;
	u8 irq_mask;
	struct bmi160_data *data = iio_priv(indio_dev);
	struct device *dev = regmap_get_device(data->regmap);

	/* Level-triggered, active-low is the default if we set all zeroes. */
	if (irq_type == IRQF_TRIGGER_RISING)
		irq_mask = BMI160_ACTIVE_HIGH | BMI160_EDGE_TRIGGERED;
	else if (irq_type == IRQF_TRIGGER_FALLING)
		irq_mask = BMI160_EDGE_TRIGGERED;
	else if (irq_type == IRQF_TRIGGER_HIGH)
		irq_mask = BMI160_ACTIVE_HIGH;
	else if (irq_type == IRQF_TRIGGER_LOW)
		irq_mask = 0;
	else {
		dev_err(&indio_dev->dev,
			"Invalid interrupt type 0x%x specified\n", irq_type);
		return -EINVAL;
	}

	open_drain = of_property_read_bool(dev->of_node, "drive-open-drain");

	return bmi160_config_pin(data->regmap, pin, open_drain, irq_mask,
				 BMI160_NORMAL_WRITE_USLEEP);
}

static int bmi160_setup_irq(struct iio_dev *indio_dev, int irq,
			    enum bmi160_int_pin pin)
{
	struct irq_data *desc;
	u32 irq_type;
	int ret;

	desc = irq_get_irq_data(irq);
	if (!desc) {
		dev_err(&indio_dev->dev, "Could not find IRQ %d\n", irq);
		return -EINVAL;
	}

	irq_type = irqd_get_trigger_type(desc);

	ret = bmi160_config_device_irq(indio_dev, irq_type, pin);
	if (ret)
		return ret;

	return bmi160_probe_trigger(indio_dev, irq, irq_type);
}

static int bmi160_chip_init(struct bmi160_data *data, bool use_spi)
{
	int ret;
	unsigned int val;
	struct device *dev = regmap_get_device(data->regmap);

	ret = regmap_write(data->regmap, BMI160_REG_CMD, BMI160_CMD_SOFTRESET);
	if (ret)
		return ret;

	usleep_range(BMI160_SOFTRESET_USLEEP, BMI160_SOFTRESET_USLEEP + 1);

	/*
	 * CS rising edge is needed before starting SPI, so do a dummy read
	 * See Section 3.2.1, page 86 of the datasheet
	 */
	if (use_spi) {
		ret = regmap_read(data->regmap, BMI160_REG_DUMMY, &val);
		if (ret)
			return ret;
	}

	ret = regmap_read(data->regmap, BMI160_REG_CHIP_ID, &val);
	if (ret) {
		dev_err(dev, "Error reading chip id\n");
		return ret;
	}
	if (val != BMI160_CHIP_ID_VAL) {
		dev_err(dev, "Wrong chip id, got %x expected %x\n",
			val, BMI160_CHIP_ID_VAL);
		return -ENODEV;
	}

	ret = bmi160_set_mode(data, BMI160_ACCEL, true);
	if (ret)
		return ret;

	ret = bmi160_set_mode(data, BMI160_GYRO, true);
	if (ret)
		return ret;

	/* Read slope duration and thresholds */
	/*  Read motion config registers  */
	ret = regmap_read(data->regmap, BMI160_REG_INT_DURATION, &val);
	if (ret) {
		dev_err(dev, "Error duration register\n");
		return ret;
	}
	data->slope_anymot_dur = val & BMI160_ANYMOTION_DUR_MASK;
	data->slope_nomot_dur = (val >> BMI160_NOMOTION_DUR_SHIFT) & BMI160_NOMOTION_DUR_MASK;

	ret = regmap_read(data->regmap, BMI160_REG_INT_ANYMOT_THRES, &val);
	if (ret) {
		dev_err(dev, "Error any motion threshold register\n");
		return ret;
	}
	data->slope_anymot_thres = val;

	ret = regmap_read(data->regmap, BMI160_REG_INT_NOMOT_THRES, &val);
	if (ret) {
		dev_err(dev, "Error no motion threshold register\n");
		return ret;
	}
	data->slope_nomot_thres = val;

	data->nomot_mustbe_updated = 0;
	data->anymot_mustbe_updated = 0;

	/*  configure no/any motion  */
	ret = bmi160_write_conf_reg(data->regmap, BMI160_REG_INT_MOTION_CFG,
				     BMI160_NOMOTION_SEL | BMI160_SIGMOTION_SEL, 
					 BMI160_NOMOTION_SEL,
				     BMI160_NORMAL_WRITE_USLEEP);

	return 0;
}

static int bmi160_accel_trig_try_reen(struct iio_trigger *trig)
{
	struct bmi160_accel_trigger *t = iio_trigger_get_drvdata(trig);
	struct bmi160_data *data = t->data;
	struct device *dev = regmap_get_device(data->regmap);
	int ret;

	/* new data interrupts don't need ack */
	if (t == &t->data->triggers[BMI160_ACCEL_TRIGGER_DATA_READY])
		return 0;

	mutex_lock(&data->mutex);
	/* clear any latched interrupt */
	ret = bmi160_int_reset(data->regmap, BMI160_NORMAL_WRITE_USLEEP);
	mutex_unlock(&data->mutex);
	if (ret < 0) {
		dev_err(dev, "Error writing reg_int_rst_latch\n");
		return ret;
	}

	return 0;
}

static int bmi160_accel_trigger_set_state(struct iio_trigger *trig,
					     bool state)
{
	struct bmi160_accel_trigger *t = iio_trigger_get_drvdata(trig);
	struct bmi160_data *data = t->data;
	int ret;

	mutex_lock(&data->mutex);

	if (t->enabled == state) {
		mutex_unlock(&data->mutex);
		return 0;
	}

	if (t->setup) {
		ret = t->setup(t, state);
		if (ret < 0) {
			mutex_unlock(&data->mutex);
			return ret;
		}
	}

	ret = bmi160_accel_set_interrupt(data, t->intr, state);
	if (ret < 0) {
		mutex_unlock(&data->mutex);
		return ret;
	}

	t->enabled = state;

	mutex_unlock(&data->mutex);

	return ret;
}

static const struct iio_trigger_ops bmi160_trigger_ops = {
	.set_trigger_state = bmi160_accel_trigger_set_state,
	//.try_reenable = bmi160_accel_trig_try_reen,
};


/*  roc event   ********************* */
static int bmi160_handle_roc_event(struct iio_dev *indio_dev)
{
	struct bmi160_data *data = iio_priv(indio_dev);
	struct device *dev = regmap_get_device(data->regmap);
	int dir;
	int ret, intv=0;
	unsigned int val;

	ret = regmap_read(data->regmap, BMI160_ACCEL_REG_INT_STATUS_0, &val);
	if (ret < 0) {
		dev_err(dev, "Error reading reg_int_status_0\n");
		return ret;
	}

	if (val & BMI160_ANYM_INT)
	{
		ret = regmap_read(data->regmap, BMI160_ACCEL_REG_INT_STATUS_2, &val);
		if (ret < 0) {
			dev_err(dev, "Error reading reg_int_status_0\n");
			return ret;
		}
		intv |= BMI160_ANYM_INT;

		if (val & BMI160_ANYM_X)
		{
			iio_push_event(indio_dev,
			       IIO_MOD_EVENT_CODE(IIO_ACCEL,
						  0,
						  IIO_MOD_X,
						  IIO_EV_TYPE_ROC,
						  IIO_EV_DIR_RISING),
			       data->timestamp);
		}
		
		if (val & BMI160_ANYM_Y)
		{
			iio_push_event(indio_dev,
			       IIO_MOD_EVENT_CODE(IIO_ACCEL,
						  0,
						  IIO_MOD_Y,
						  IIO_EV_TYPE_ROC,
						  IIO_EV_DIR_RISING),
			       data->timestamp);
		}
		
		if (val & BMI160_ANYM_Z)
		{
			iio_push_event(indio_dev,
			       IIO_MOD_EVENT_CODE(IIO_ACCEL,
						  0,
						  IIO_MOD_Z,
						  IIO_EV_TYPE_ROC,
						  IIO_EV_DIR_RISING),
			       data->timestamp);
		}

	}

	ret = regmap_read(data->regmap, BMI160_ACCEL_REG_INT_STATUS_1, &val);
	if (ret < 0) {
		dev_err(dev, "Error reading reg_int_status_1\n");
		return ret;
	}

	if (val & BMI160_NOMOT_INT)
	{
		intv |= BMI160_NOMOT_INT;
		iio_push_event(indio_dev,
		       IIO_MOD_EVENT_CODE(IIO_ACCEL,
					  0,
					  IIO_MOD_X_AND_Y_AND_Z,
					  IIO_EV_TYPE_ROC,
					  IIO_EV_DIR_FALLING),
		       data->timestamp);
	}

	return intv;
}


/* irq */
static irqreturn_t bmi160_irq_thread_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct bmi160_data *data = iio_priv(indio_dev);
	struct device *dev = regmap_get_device(data->regmap);
	bool ack = false;
	int ret;

	mutex_lock(&data->mutex);

	if (data->ev_nomot_enable_state || data->ev_anymot_enable_state) {
		ret = bmi160_handle_roc_event(indio_dev);
		if (ret > 0)
			ack = true;
	}

	if (ack) {
		ret = IRQ_HANDLED;
	} else {
		ret = IRQ_NONE;
	}

	mutex_unlock(&data->mutex);

	return ret;
}

static irqreturn_t bmi160_irq_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct bmi160_data *data = iio_priv(indio_dev);
	bool ack = false;
	int i;

	data->timestamp = iio_get_time_ns(indio_dev);

	for (i = 0; i < BMI160_ACCEL_TRIGGERS; i++) {
		if (data->triggers[i].enabled) {
			iio_trigger_poll(data->triggers[i].indio_trig);
			ack = true;
			break;
		}
	}

	if (data->ev_nomot_enable_state || data->ev_anymot_enable_state)
		return IRQ_WAKE_THREAD;

	if (ack)
		return IRQ_HANDLED;

	return IRQ_NONE;
}

static const struct {
	int intr;
	const char *name;
	int (*setup)(struct bmi160_accel_trigger *t, bool state);
} bmi160_accel_triggers[BMI160_ACCEL_TRIGGERS] = {
	{
		.intr = 0,
		.name = "%s-dev%d",
	},
	{
		.intr = 1,
		.name = "%s-any-motion-dev%d",
		.setup = bmi160_accel_any_motion_setup,
	},
	{
		.intr = 2,
		.name = "%s-no-motion-dev%d",
		.setup = bmi160_accel_no_motion_setup,
	},
};
static void bmi160_unregister_triggers(struct bmi160_data *data,
					     int from)
{
	int i;

	for (i = from; i >= 0; i--) {
		if (data->triggers[i].indio_trig) {
			iio_trigger_unregister(data->triggers[i].indio_trig);
			data->triggers[i].indio_trig = NULL;
		}
	}
}

int bmi160_probe_trigger(struct iio_dev *indio_dev, int irq, u32 irq_type)
{
	struct bmi160_data *data = iio_priv(indio_dev);
	struct device *dev = regmap_get_device(data->regmap);
	int i,ret;

	for (i = 0; i < BMI160_ACCEL_TRIGGERS; i++) {
		struct bmi160_accel_trigger *t = &data->triggers[i];

		t->indio_trig = devm_iio_trigger_alloc(dev,
						bmi160_accel_triggers[i].name,
					    indio_dev->name, indio_dev->id);

		if (t->indio_trig == NULL) {
			ret = -ENOMEM;
			break;
		}

		t->indio_trig->dev.parent = dev;
		t->indio_trig->ops = &bmi160_trigger_ops;
		t->intr = bmi160_accel_triggers[i].intr;
		t->data = data;
		t->setup = bmi160_accel_triggers[i].setup;
		t->enabled = false;
		iio_trigger_set_drvdata(t->indio_trig, t);

		ret = devm_iio_trigger_register(dev, t->indio_trig);
		if (ret)
			break;
	}

	if (ret)
	{
		bmi160_unregister_triggers(data, i - 1);
	}
	else {
		ret = devm_request_threaded_irq(
						dev, irq,
						bmi160_irq_handler,
						bmi160_irq_thread_handler,
						irq_type,
						BMI160_IRQ_NAME,
						indio_dev);
	}

	return ret;
}

static void bmi160_chip_uninit(void *data)
{
	struct bmi160_data *bmi_data = data;

	bmi160_set_mode(bmi_data, BMI160_GYRO, false);
	bmi160_set_mode(bmi_data, BMI160_ACCEL, false);
}

int bmi160_core_probe(struct device *dev, struct regmap *regmap,
		      const char *name, bool use_spi)
{
	struct iio_dev *indio_dev;
	struct bmi160_data *data;
	int irq;
	enum bmi160_int_pin int_pin;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);
	data->regmap = regmap;

	ret = bmi160_chip_init(data, use_spi);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, bmi160_chip_uninit, data);
	if (ret)
		return ret;

	if (!name && ACPI_HANDLE(dev))
		name = bmi160_match_acpi_device(dev);

	mutex_init(&data->mutex);

	indio_dev->dev.parent = dev;
	indio_dev->channels = bmi160_channels;
	indio_dev->num_channels = ARRAY_SIZE(bmi160_channels);
	indio_dev->name = name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &bmi160_info;

	ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
					      iio_pollfunc_store_time,
					      bmi160_trigger_handler, NULL);
	if (ret < 0) {
		dev_err(dev, "Failed: iio triggered buffer setup\n");
		return ret;
	}

	irq = bmi160_get_irq(dev->of_node, &int_pin);
	if (irq > 0) {
		data->intr_pin = int_pin;

		ret = bmi160_setup_irq(indio_dev, irq, int_pin);
		if (ret)
			dev_err(&indio_dev->dev, "Failed to setup IRQ %d\n",
				irq);
		else {
			bmi160_accel_interrupts_setup(indio_dev, data);
		}
	} else {
		dev_info(&indio_dev->dev, "Not setting up IRQ trigger\n");
	}

	return devm_iio_device_register(dev, indio_dev);

}
EXPORT_SYMBOL_GPL(bmi160_core_probe);

MODULE_AUTHOR("Daniel Baluta <daniel.baluta@intel.com");
MODULE_DESCRIPTION("Bosch BMI160 driver");
MODULE_LICENSE("GPL v2");
