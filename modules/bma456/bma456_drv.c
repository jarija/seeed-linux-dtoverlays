// SPDX-License-Identifier: GPL-2.0-only
/*
 * bma456.c - IIO driver for Bosch BMA456 triaxial acceleration sensor
 *
 * Copyright 2020 Zhangqun Ming <north_sea@qq.com>
 *
 * SPI is not supported by driver
 * BMA456: 7-bit I2C slave address 0x19
 */

#include "bma456.h"


#define GRAVITY_EARTH      9807	//(9.80665f)

static const int scale_table[]= {598, 1197, 2394, 4789};
static const int bw_table[] = { // in mHz
	781, 1563, 3125, 6250, 12500, 
	25000, 50000, 100000, 200000, 
	400000, 800000, 1600000
};

/* power_mode */
static uint8_t bma456_chip_enable(struct bma456_data *data, bool enable)
{
	uint16_t rslt = 0;

	return rslt;
}

static int bma456_get_power_mode(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan)
{
	DBG_FUNC("bma456_get_power_mode()");
	return 1;
}

static int bma456_set_power_mode(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan, unsigned int mode)
{
	DBG_FUNC("bma456_set_power_mode():%d", mode);
	return 0;
}

static const struct iio_mount_matrix *
	bma456_accel_get_mount_matrix(const struct iio_dev *indio_dev,
			const struct iio_chan_spec *chan)
{
	struct bma456_data *data = iio_priv(indio_dev);
	return &data->orientation;
}

static const char * const bma456_power_modes[] = { 
	"low_noise", "low_power" 
};
static const struct iio_enum bma456_power_mode_enum = {
	.items = bma456_power_modes,
	.num_items = ARRAY_SIZE(bma456_power_modes),
	.get = bma456_get_power_mode,
	.set = bma456_set_power_mode,
};
static const struct iio_chan_spec_ext_info bma456_ext_info[] = {
	IIO_ENUM("power_mode", true, &bma456_power_mode_enum),
	IIO_ENUM_AVAILABLE("power_mode", &bma456_power_mode_enum),
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_DIR, bma456_accel_get_mount_matrix),
	{ }
};


/* iio_chan */
static const struct iio_chan_spec bma456_channels[] = {
	BMA456_ACC_CHANNEL(X, 16),
	BMA456_ACC_CHANNEL(Y, 16),
	BMA456_ACC_CHANNEL(Z, 16),
	BMA456_TEMP_CHANNEL,
	IIO_CHAN_SOFT_TIMESTAMP(4),

	{// STEPS
		.type = IIO_STEPS,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) |
				BIT(IIO_CHAN_INFO_ENABLE),
	},
};


/* iio_info */
static ssize_t in_accel_filter_low_pass_3db_frequency_available_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	int i = 0;
	size_t len = 0;

	for (i=0; i<ARRAY_SIZE(bw_table); i++) {
		len += scnprintf(buf+len, PAGE_SIZE-len, "%d ", bw_table[i]);
	}
	len += scnprintf(buf+len, PAGE_SIZE-len, "\n");

	return len;
}

static ssize_t in_accel_scale_available_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i = 0;
	size_t len = 0;

	for (i=0; i<ARRAY_SIZE(scale_table); i++) {
		len += scnprintf(buf+len, PAGE_SIZE-len, "0.%06d ", scale_table[i]);
	}
	len += scnprintf(buf+len, PAGE_SIZE-len, "\n");

	return len;
}

static IIO_DEVICE_ATTR_RO(in_accel_filter_low_pass_3db_frequency_available, 0);
static IIO_DEVICE_ATTR_RO(in_accel_scale_available, 0);
static struct attribute *bma456_attributes[] = {
	&iio_dev_attr_in_accel_filter_low_pass_3db_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_accel_scale_available.dev_attr.attr,
	NULL,
};
static const struct attribute_group bma456_attrs_group = {
	.attrs = bma456_attributes,
};

static int bma456_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int *val, int *val2,
		long mask)
{
	struct bma456_data *data = iio_priv(indio_dev);
	uint16_t rslt;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		DBG_FUNC("IIO_CHAN_INFO_RAW idx=%d", chan->scan_index);
		if (chan->scan_index == TEMP) {
			int32_t temp;
			rslt = bma4_get_temperature(&temp, BMA4_DEG, &data->bma);
			*val = temp;
		}
		else {
			int32_t val32 = 0;
			uint8_t value[2] = { 0 };
			int axis = chan->scan_index;
			if ((AXIS_X <= axis) && (axis <= AXIS_Z)) {
				axis -= AXIS_X;
				rslt = bma4_read_regs(BMA4_DATA_8_ADDR + axis*2, value, 2, &data->bma);
				if (rslt != BMA4_OK) {
					*val = 0;
					return -EINVAL;
				}

				val32 = value[1];
				val32 <<= 8;
				val32 += value[0];
				*val = sign_extend32(val32 >> chan->scan_type.shift, chan->scan_type.realbits - 1);
			}
		}
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		DBG_FUNC("IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY");
		*val = bw_table[data->accel.odr - BMA4_OUTPUT_DATA_RATE_0_78HZ];
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		DBG_FUNC("IIO_CHAN_INFO_SCALE type=%d", chan->type);
		switch (chan->type) {
		case IIO_ACCEL:
			*val = 0;
			*val2 = scale_table[data->accel.range - BMA4_ACCEL_RANGE_2G];
			return IIO_VAL_INT_PLUS_MICRO;
		case IIO_TEMP:
			*val = 0;
			*val2 = BMA4_SCALE_TEMP;
			return IIO_VAL_INT_PLUS_MICRO;
		break;
		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_OFFSET:
		DBG_FUNC("IIO_CHAN_INFO_OFFSET");
		if (IIO_TEMP == chan->type) {
			*val = BMA4_OFFSET_TEMP; /* 0 LSB @ 23 degree C */
		}
		else {
			int8_t value[1] = { 0 };
			int axis = chan->scan_index;
			if ((AXIS_X <= axis) && (axis <= AXIS_Z)) {
				axis -= AXIS_X;
				rslt = bma4_read_regs(BMA4_OFFSET_0_ADDR + axis, value, 1, &data->bma);
				if (rslt != BMA4_OK) {
					*val = 0;
					return -EINVAL;
				}

				*val = value[0];
			}
		}
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_ENABLE:
		DBG_FUNC("IIO_CHAN_INFO_ENABLE");
		if (IIO_STEPS == chan->type) {
			*val = data->step_enable;
		}
		else {
			// to do:
			*val = 1;
		}
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_PROCESSED:
		if (IIO_STEPS == chan->type) {
			*val = data->step_count;
		}
		return IIO_VAL_INT;

	default:
		DBG_FUNC("default mask=%d\n", (int)mask);
		return -EINVAL;
	}

	return -EINVAL;
}

static int bma456_write_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	struct bma456_data *data = iio_priv(indio_dev);
	int32_t i = 0;
	uint16_t rslt;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		DBG_FUNC("IIO_CHAN_INFO_SCALE:%d, %d", val, val2);
		for (i=0; i<ARRAY_SIZE(scale_table); i++) {
			if (val2 == scale_table[i])
				break;
		}
		if (i >= ARRAY_SIZE(scale_table))
			return -EINVAL;
		data->accel.range = BMA4_ACCEL_RANGE_2G + i;
		rslt = bma4_set_accel_config(&data->accel, &data->bma);
		if (BMA4_OK != rslt) {
			dev_err(&data->client->dev, "bma4_set_accel_config\n");
			return -EINVAL;
		}
		return rslt;

	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		DBG_FUNC("IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:%d, %d", val, val2);
		for (i=0; i<ARRAY_SIZE(bw_table); i++) {
			if (val == bw_table[i])
				break;
		}
		if (i >= ARRAY_SIZE(bw_table))
			return -EINVAL;
		data->accel.odr = BMA4_OUTPUT_DATA_RATE_0_78HZ + i;
		rslt = bma4_set_accel_config(&data->accel, &data->bma);
		if (BMA4_OK != rslt) {
			dev_err(&data->client->dev, "bma4_set_accel_config\n");
			return -EINVAL;
		}
		return rslt;

	case IIO_CHAN_INFO_OFFSET:
		DBG_FUNC("IIO_CHAN_INFO_OFFSET:%d, %d", chan->type, chan->scan_index);
		if (IIO_ACCEL == chan->type) {
			int8_t value[1] = { 0 };
			int axis = chan->scan_index;
			if ((AXIS_X <= axis) && (axis <= AXIS_Z)) {
				axis -= AXIS_X;
				value[0] = (int8_t)val;
				rslt = bma4_write_regs(BMA4_OFFSET_0_ADDR + axis, value, 1, &data->bma);
				if (rslt != BMA4_OK) {
					return -EINVAL;
				}
				return rslt;
			}
		}
		return -EINVAL;

	case IIO_CHAN_INFO_ENABLE:
		DBG_FUNC("IIO_CHAN_INFO_ENABLE");
		if (IIO_STEPS == chan->type) {
			int enable = !!(val);
			if (enable == data->step_enable)
				return 0;
			data->step_enable = enable;
			if (data->step_enable) {
				data->step_count = 0;
				bma456_reset_step_counter(&data->bma);
				bma456_feature_enable(BMA456_STEP_CNTR, BMA4_ENABLE, &data->bma);
			}
			else {
				bma456_feature_enable(BMA456_STEP_CNTR, BMA4_DISABLE, &data->bma);
			}
		}
		return 0;

	default:
		DBG_FUNC("mask=%d, %d, %d\n", (int)mask, val, val2);
		return -EINVAL;
	}
}

static const struct iio_info bma456_info = {
	.attrs			= &bma456_attrs_group,
	.read_raw		= bma456_read_raw,
	.write_raw		= bma456_write_raw,
};


static irqreturn_t bma456_irq(int irq, void *handle)
{
	uint8_t rslt;
	uint16_t int_status;
	struct bma456_data *data = (struct bma456_data *)handle;

	/* Read interrupt status */
	rslt = bma456_read_int_status(&int_status, &data->bma);
	bma4_error_codes_print_result("bma456_read_int_status", rslt);
	if (rslt != BMA4_OK)
		return IRQ_HANDLED;

	if (int_status & BMA456_ANY_MOT_INT) {
		//DBG_FUNC("BMA456_ANY_MOT_INT");
	}
	if (int_status & BMA456_NO_MOT_INT) {
		//DBG_FUNC("BMA456_NO_MOT_INT");
	}
	if (int_status & BMA456_SINGLE_TAP_INT) {
		DBG_FUNC("BMA456_SINGLE_TAP_INT");
	}
	if (int_status & BMA456_DOUBLE_TAP_INT) {
		DBG_FUNC("BMA456_DOUBLE_TAP_INT");
	}
	if (int_status & BMA456_ACTIVITY_INT) {
		uint8_t activity_output = 0;

		rslt = bma456_activity_output(&activity_output, &data->bma);
		bma4_error_codes_print_result("bma456_activity_output status", rslt);
		if (rslt != BMA4_OK)
			return IRQ_HANDLED;
		switch (activity_output)
        {
		case BMA456_USER_STATIONARY:
			DBG_FUNC("BMA456_USER_STATIONARY");
		break;
		
		case BMA456_USER_WALKING:
			DBG_FUNC("BMA456_USER_WALKING");
		break;
		
		case BMA456_USER_RUNNING:
			DBG_FUNC("BMA456_USER_RUNNING");
		break;

		case BMA456_STATE_INVALID:
			DBG_FUNC("BMA456_STATE_INVALID");
		break;

		default:
		break;
		}
	}
	if (int_status & BMA456_STEP_CNTR_INT) {
		uint32_t step_out = 0;
		rslt = bma456_step_counter_output(&step_out, &data->bma);
		bma4_error_codes_print_result("bma456_step_counter_output status", rslt);
		DBG_FUNC("step_out=%d", step_out);
	}
	if (int_status & BMA4_ACCEL_DATA_RDY_INT) {
		struct bma4_accel sens_data = { 0 };
		rslt = bma4_read_accel_xyz(&sens_data, &data->bma);
      	if (rslt == BMA4_OK) {
			//DBG_FUNC("x=%d,y=%d,z=%d", sens_data.x, sens_data.y, sens_data.z);
		}
	}

	return IRQ_HANDLED;
}

/* work_cb_irq: scan bma456's interrupt. */
static void work_cb_irq(struct work_struct *work)
{
	struct bma456_data *data = container_of((struct delayed_work*)work, 
		struct bma456_data, work_irq);
	int msec = 1000000 / bw_table[data->accel.odr - BMA4_OUTPUT_DATA_RATE_0_78HZ];

	bma456_irq(data->client->irq, data);
	schedule_delayed_work(&data->work_irq, msecs_to_jiffies(msec));

	return;
}

/* apis */
#if 1
static BMA4_INTF_RET_TYPE bma_i2c_read(uint8_t reg_addr, 
			uint8_t *read_data, uint32_t len, void *intf_ptr)
{
	struct bma456_data *data = (struct bma456_data *)intf_ptr;

	mutex_lock(&data->mutex);
	i2c_smbus_read_i2c_block_data(data->client, reg_addr, len, read_data);
	mutex_unlock(&data->mutex);
	return 0;
}

static BMA4_INTF_RET_TYPE bma_i2c_write(uint8_t reg_addr, 
			const uint8_t *write_data, uint32_t len, void *intf_ptr)
{
	struct bma456_data *data = (struct bma456_data *)intf_ptr;

	mutex_lock(&data->mutex);
	i2c_smbus_write_i2c_block_data(data->client, reg_addr, len, write_data);
	mutex_unlock(&data->mutex);
	return 0;
}

static void bma_delay_us(uint32_t period, void *intf_ptr)
{
	uint32_t ms = period/1000;
	uint32_t us = period%1000;

	if (ms)
		msleep(ms);
	if (us)
		udelay(us);
}

static void set_accel_config(struct bma4_dev *bma)
{
	int8_t rslt = 0;
	struct bma4_accel_config accel_conf = { 0 };
	/* Accelerometer configuration settings */
	/* Output data Rate */
	accel_conf.odr = BMA4_OUTPUT_DATA_RATE_200HZ;

	/* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G) */
	accel_conf.range = BMA4_ACCEL_RANGE_2G;

	/* The bandwidth parameter is used to configure the number of sensor samples that are averaged
		if it is set to 2, then 2^(bandwidth parameter) samples
		are averaged, resulting in 4 averaged samples
		Note1 : For more information, refer the datasheet.
		Note2 : A higher number of averaged samples will result in a less noisier signal, but
		this has an adverse effect on the power consumed.
	*/
	accel_conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4;

	/* Enable the filter performance mode where averaging of samples
		will be done based on above set bandwidth and ODR.
		There are two modes
		0 -> Averaging samples (Default)
		1 -> No averaging
		For more info on No Averaging mode refer datasheet.
	*/
	accel_conf.perf_mode = BMA4_CIC_AVG_MODE;

	/* Set the accel configurations */
	rslt = bma4_set_accel_config(&accel_conf, bma);
	bma4_error_codes_print_result("bma4_set_accel_enable status", rslt);
}


/*! Structure to define any/no-motion configurations */
struct bma456_any_no_mot_config any_no_mot = { 0 };

/*!
    @brief This internal API is used to get any/no-motion configurations.
*/
int8_t get_any_no_mot_config(struct bma4_dev *bma)
{
	/* Variable to store the status of API */
	int8_t rslt;

	/* Getting any-motion configuration to get default configuration */
	rslt = bma456_get_any_mot_config(&any_no_mot, bma);
	bma4_error_codes_print_result("bma456_get_any_mot_config status", rslt);

	if (rslt == BMA4_OK)
	{
		/*
			Set the slope threshold:
			Interrupt will be generated if the slope of all the axis exceeds the threshold (1 bit = 0.48mG)
		*/
		any_no_mot.threshold = 10;

		/*
			Set the duration for any-motion interrupt:
			Duration defines the number of consecutive data points for which threshold condition must be true(1
			bit =
			20ms)
		*/
		any_no_mot.duration = 4;

		/* Enabling X, Y, and Z axis for Any-motion feature */
		any_no_mot.axes_en = BMA456_EN_ALL_AXIS;

		/* Like threshold and duration, we can also change the config of int_bhvr and slope */

		/* Set the threshold and duration configuration */
		rslt = bma456_set_any_mot_config(&any_no_mot, bma);
		bma4_error_codes_print_result("bma456_set_any_mot_config status", rslt);

		if (rslt == BMA4_OK)
		{
			/* Getting no-motion configuration to get default configuration */
			rslt = bma456_get_no_mot_config(&any_no_mot, bma);
			bma4_error_codes_print_result("bma456_get_no_mot_config status", rslt);

			if (rslt == BMA4_OK)
			{
				/*
					Set the slope threshold:
					Interrupt will be generated if the slope of all the axis exceeds the threshold (1 bit = 0.48mG)
				*/
				any_no_mot.threshold = 10;

				/*
					Set the duration for no-motion interrupt:
					Duration defines the number of consecutive data points for which threshold condition must be
					true(1 bit = 20ms)
				*/
				any_no_mot.duration = 4;

				/* Enabling X, Y, and Z axis for no-motion feature */
				any_no_mot.axes_en = BMA456_EN_ALL_AXIS;

				/* Like threshold and duration, we can also change the config of int_bhvr */

				/* Set the threshold and duration configuration */
				rslt = bma456_set_no_mot_config(&any_no_mot, bma);
				bma4_error_codes_print_result("bma456_set_no_mot_config status", rslt);
			}
		}
	}

	return rslt;
}
#endif

/* probe */
static int bma456_probe(struct i2c_client *client, 
				const struct i2c_device_id *id)
{
	struct bma456_data *data;
	struct iio_dev *indio_dev;
	int ret = 0;
	int8_t rslt = 0;

	DBG_FUNC("");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "check I2C_FUNC_I2C\n");
		return -EOPNOTSUPP;
	}

	/* iio_dev: alloc */
	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;
	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;
	mutex_init(&data->mutex);

	/* i2c */
	rslt = bma4_interface_selection(&data->bma);
	bma4_error_codes_print_result("bma4_interface_selection status", rslt);
	if (rslt != BMA4_OK)
		return -ENODEV;
	data->bma.intf_ptr = (void *)data;
	data->bma.bus_read = bma_i2c_read;
	data->bma.bus_write = bma_i2c_write;
	data->bma.delay_us = bma_delay_us;

	rslt = bma4_soft_reset(&data->bma);
	bma4_error_codes_print_result("bma4_soft_reset status", rslt);
	if (rslt != BMA4_OK)
		return -ENODEV;
	//bma_delay_us(150, NULL);
	msleep(150);

	/* Sensor initialization */
	rslt = bma456_init(&data->bma);
	bma4_error_codes_print_result("bma456_init status", rslt);
	if (rslt != BMA4_OK)
		return -ENODEV;
	DBG_FUNC("chip_id:0x%x", data->bma.chip_id);

	/* Upload the configuration file to enable the features of the sensor. */
	rslt = bma456_write_config_file(&data->bma);
	bma4_error_codes_print_result("bma456_write_config status", rslt);

	/* Enable the accelerometer */
	rslt = bma4_set_accel_enable(BMA4_ENABLE, &data->bma);
	bma4_error_codes_print_result("bma4_set_accel_enable status", rslt);

	set_accel_config(&data->bma);
	/* Map the interrupt 1 for any/no-motion step activity and step counter, single double tap*/
	rslt = bma456_map_interrupt(BMA4_INTR1_MAP, 
		(BMA456_ACTIVITY_INT | BMA456_STEP_CNTR_INT | BMA456_ANY_MOT_INT | \
			BMA456_NO_MOT_INT | BMA456_SINGLE_TAP_INT | BMA456_DOUBLE_TAP_INT), 
		BMA4_ENABLE, &data->bma);
	bma4_error_codes_print_result("bma456_map1_interrupt status", rslt);

	/* Mapping data ready interrupt with interrupt pin 2 to get interrupt status once getting new accel data */
	rslt = bma456_map_interrupt(BMA4_INTR2_MAP, BMA4_DATA_RDY_INT, BMA4_ENABLE, &data->bma);
	bma4_error_codes_print_result("bma456_map2_interrupt status", rslt);

	/* Setting watermark level 1, the output step resolution is 20 steps.
	Eg: 1 means, 1 * 20 = 20. Every 20 steps once output triggers
	*/
	rslt = bma456_step_counter_set_watermark(1, &data->bma);
	bma4_error_codes_print_result("bma456_step_counter_set_watermark status", rslt);

	if (rslt == BMA4_OK)
	{
		/* Enabling step detector, tap features */
		rslt = bma456_feature_enable(BMA456_STEP_ACT | BMA456_STEP_CNTR | \
					BMA456_SINGLE_TAP | BMA456_DOUBLE_TAP, BMA4_ENABLE, &data->bma);
		bma4_error_codes_print_result("bma456_feature_enable status", rslt);
	}

	/* Get any-motion and no-motion configurations */
	rslt = get_any_no_mot_config(&data->bma);
	bma4_error_codes_print_result("get_any_no_mot_config status", rslt);

	bma456_single_tap_set_sensitivity(0x00, &data->bma);
	bma456_double_tap_set_sensitivity(0x00, &data->bma);

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq, 
					NULL,
					bma456_irq,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					dev_name(&client->dev),
					data);
		if (ret) {
			dev_err(&client->dev, "irq %d busy?\n", client->irq);
			goto err_chip_disable;
		}
	}
	else {
		INIT_DELAYED_WORK(&data->work_irq, work_cb_irq);
		schedule_delayed_work(&data->work_irq, 0);
	}

	/* iio_dev: register */
	ret = iio_read_mount_matrix(&client->dev, "mount-matrix", &data->orientation);
	if (ret < 0)
		return ret;

	indio_dev->dev.parent = &client->dev;
	indio_dev->channels = bma456_channels;
	indio_dev->num_channels = ARRAY_SIZE(bma456_channels);
	indio_dev->name = id->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &bma456_info;
	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(&client->dev, "unable to register iio device:%d\n", ret);
		goto err_chip_disable;
	}

	return 0;

err_chip_disable:
	bma456_chip_enable(data, false);

	return ret;
}

static int bma456_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct bma456_data *data = iio_priv(indio_dev);

	if (!data->client->irq) {
		cancel_delayed_work_sync(&data->work_irq);
	}
	bma456_chip_enable(data, false);
	iio_device_unregister(indio_dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int bma456_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct bma456_data *data = iio_priv(indio_dev);

	DBG_FUNC();
	return bma456_chip_enable(data, false);
}

static int bma456_resume(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct bma456_data *data = iio_priv(indio_dev);

	DBG_FUNC();
	return bma456_chip_enable(data, true);
}

static SIMPLE_DEV_PM_OPS(bma456_pm_ops, bma456_suspend, bma456_resume);
#define BMA456_PM_OPS (&bma456_pm_ops)
#else
#define BMA456_PM_OPS NULL
#endif

static const struct i2c_device_id bma456_ids[] = {
	{ "bma456", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bma456_ids);

static const struct of_device_id bma456_of_match[] = {
	{ .compatible = "bosch,bma456", },
	{ }
};
MODULE_DEVICE_TABLE(of, bma456_of_match);

static struct i2c_driver bma456_driver = {
	.driver = {
		.name	= "bma456",
		.pm	= BMA456_PM_OPS,
		.of_match_table = bma456_of_match,
	},
	.probe		= bma456_probe,
	.remove		= bma456_remove,
	.id_table	= bma456_ids,
};
module_i2c_driver(bma456_driver);

MODULE_AUTHOR("Zhangqun Ming <north_sea@qq.com>");
MODULE_AUTHOR("Seeed, Inc.");
MODULE_DESCRIPTION("Bosch BMA456 triaxial acceleration sensor");
MODULE_LICENSE("GPL v2");
