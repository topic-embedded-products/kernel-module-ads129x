/*
 * ads129x series SPI ADC driver
 *
 * Copyright (C) 2014 Topic Embedded Products
 *
 * Licensed under the GPL-2.
 */

#define DEBUG

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include <linux/of_device.h>
#include <linux/of_address.h>

struct ads129x_state {
	struct spi_device *spi;
};

#define ADS129x_V_CHAN(index)						\
	{								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.channel = index,					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
		.address = index,					\
		.scan_index = index,					\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = 24,					\
			.storagebits = 24,				\
			.endianness = IIO_BE,				\
		},							\
	}


static const struct iio_chan_spec ads129x_channels[] = {
	{
		.type = IIO_PROXIMITY, /* Actually, status bits for channels */
		.indexed = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.address = -1,
		.scan_index = -1,
		.scan_type = {
			.sign = 'u',
			.realbits = 24,
			.storagebits = 24,
		},
	},
	ADS129x_V_CHAN(0),
	ADS129x_V_CHAN(1),
	ADS129x_V_CHAN(2),
	ADS129x_V_CHAN(3),
	ADS129x_V_CHAN(4),
	ADS129x_V_CHAN(5),
	ADS129x_V_CHAN(6),
	ADS129x_V_CHAN(7),
};


/**
 * ads129x_update_scan_mode() setup the spi transfer buffer for the new scan mask
 **/
static int ads129x_update_scan_mode(struct iio_dev *indio_dev,
	const unsigned long *active_scan_mask)
{
	struct ads129x_state *st = iio_priv(indio_dev);

	pr_debug("%s\n", __func__);

	return -ENODEV;
}

/**
 * ads129x_trigger_handler() bh of trigger launched polling to ring buffer
 **/
static irqreturn_t ads129x_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ads129x_state *st = iio_priv(indio_dev);

	pr_debug("%s\n", __func__);

	return IRQ_HANDLED;
}

static int ads129x_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct ads129x_state *st = iio_priv(indio_dev);
	pr_debug("%s (%d)\n", __func__, m);
	return -ENODEV;
}

static const struct iio_info ads129x_info = {
	.read_raw = &ads129x_read_raw,
	.update_scan_mode = ads129x_update_scan_mode,
	.driver_module = THIS_MODULE,
};

static int ads129x_probe(struct spi_device *spi)
{
	struct ads129x_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;
	st = iio_priv(indio_dev);
	spi_set_drvdata(spi, indio_dev);
	st->spi = spi;

	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->dev.parent = &spi->dev;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ads129x_channels;
	indio_dev->num_channels = ARRAY_SIZE(ads129x_channels);
	indio_dev->info = &ads129x_info;

	/* Setup default message */
	ret = iio_triggered_buffer_setup(indio_dev, NULL,
			&ads129x_trigger_handler, NULL);
	if (ret)
		goto error_disable_reg;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_cleanup_ring;

	return 0;

error_cleanup_ring:
error_disable_reg:
	return ret;
}

static int ads129x_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ads129x_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	return 0;
}

static const struct spi_device_id ads129x_id[] = {
	{"ads129x", 8}, /* We only really support the 8-channel version */
	{"ads1298", 8},
	{}
};
MODULE_DEVICE_TABLE(spi, ads129x_id);

static struct spi_driver ads129x_driver = {
	.driver = {
		.name	= "ads129x",
		.owner	= THIS_MODULE,
	},
	.probe		= ads129x_probe,
	.remove		= ads129x_remove,
	.id_table	= ads129x_id,
};
module_spi_driver(ads129x_driver);

MODULE_AUTHOR("Mike Looijmans <mike.looijmans@topic.nl>");
MODULE_DESCRIPTION("TI ADS129x ADC");
MODULE_LICENSE("GPL v2");
