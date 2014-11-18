/*
 * ads129x series SPI ADC driver
 *
 * Copyright (C) 2014 Topic Embedded Products
 *
 * Licensed under the GPL-2.
 */
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <asm/uaccess.h>

#include <linux/fs.h>
#include <linux/cdev.h>

#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/of_address.h>

#include "ads129xio.h"

#define SPI_BUFF_SIZE 64
/* SPI clock for reading data. Spec sheet says 50ns period, so 20MHz */
#define SPI_BUS_SPEED_FAST	20000000
/* SPI clock for reading/writing registers. To allow the ads to
 * decode the message, reduce the clock speed during this type
 * of transfer */
#define SPI_BUS_SPEED_SLOW	2048000

#define MAX_ADS_CHIPS		3

#define ACQ_BUF_SIZE		27


struct ads129x_chip{
	struct spi_device *spi;
	char *rx_buff;
	char *tx_buff;
	struct spi_message msg;
	struct spi_transfer transfer;
};

struct ads129x_dev {
	struct cdev dev;
	dev_t dev_no;
	struct class *cl;
	struct semaphore fop_sem;

	struct mutex mutex;

	struct ads129x_chip chip[MAX_ADS_CHIPS];
	int num_ads_chips;
	wait_queue_head_t wait_queue;
	int rx_count;
	int irq;

	volatile int sample_req;

	int gpio_initialized;
};

static struct ads129x_dev ads_inst;


struct gpio ads129x_gpio_pwdn = { .label = "pwdn-gpio", .flags = GPIOF_OUT_INIT_LOW };
struct gpio ads129x_gpio_reset = { .label = "reset-gpio", .flags = GPIOF_OUT_INIT_LOW };
struct gpio ads129x_gpio_start = { .label = "start-gpio", .flags =GPIOF_OUT_INIT_LOW };

static void ads129x_spi_handler(void *arg)
{
	struct ads129x_dev *ads = arg;

	ads->rx_count++;
	pr_debug("%s rx_count=%d\n", __func__, ads->rx_count);
	if (ads->rx_count == ads->num_ads_chips)
		wake_up_interruptible(&ads->wait_queue);
}

static irqreturn_t ads129x_irq_handler(int irq, void *id)
{
	int i;

	if (!ads_inst.sample_req)
		return IRQ_HANDLED;

	pr_debug("%s\n", __func__);
	for (i=0; i<ads_inst.num_ads_chips; i++) {
		spi_async(ads_inst.chip[i].spi, &ads_inst.chip[i].msg);
	}
	return IRQ_HANDLED;
}

static int ads129x_init_io_from_dt(struct device *dev, struct gpio *pgpio)
{
	int ret;
	struct device_node *np = dev->of_node;

	pgpio->gpio = of_get_named_gpio(np, pgpio->label, 0);
	pr_debug("%s: %d\n", pgpio->label, pgpio->gpio);
	if (!gpio_is_valid(pgpio->gpio)) {
		dev_err(dev, "Invalid gpio\n");
		return -EINVAL;
	} else {
		ret = devm_gpio_request_one(dev, pgpio->gpio, pgpio->flags, pgpio->label);
		if (ret < 0) {
			dev_err(dev, "Error requesting gpio %s:%d.\n",
				pgpio->label, pgpio->gpio);
			return ret;
		}
	}

	return 0;
};

static int ads129x_init_gpio_pins(struct ads129x_dev *ads, struct device *dev)
{
	int ret;

	if (ads->gpio_initialized == 0) {
		ret = ads129x_init_io_from_dt(dev, &ads129x_gpio_pwdn);
		if (ret < 0)
			goto error_exit;
		ret = ads129x_init_io_from_dt(dev, &ads129x_gpio_reset);
		if (ret < 0)
			goto error_exit;
		ret = ads129x_init_io_from_dt(dev, &ads129x_gpio_start);
		if (ret < 0)
			goto error_exit;

		ads->irq = irq_of_parse_and_map(of_get_child_by_name(dev->of_node, "rdy-irq"), 0);
		if (ads->irq < 0) {
			dev_err(dev, "IRQ resource missing\n");
			ret = -EINVAL;
			goto error_exit;
		} else
			pr_debug("IRQ: %d\n", ads->irq);
		ret = request_irq(ads->irq, ads129x_irq_handler, 0, "ads129x-drdy", NULL);
		if (ret) {
			dev_err(dev, "Cannot claim IRQ %d\n", ads->irq);
			ret = -EINVAL;
			goto error_exit;
		}
		ads->gpio_initialized = 1;
	}
	return ads->gpio_initialized;

error_exit:
	ads->gpio_initialized = ret;
	return ret;
};


static int ads_send_byte(struct ads129x_chip *dev, int data)
{
	int status;
	struct spi_message msg = { };
	struct spi_transfer transfer = { };

	pr_debug("%s(%#x)\n", __func__, data);

	spi_message_init(&msg);
	dev->tx_buff[0] = data;
	transfer.tx_buf = dev->tx_buff;
	transfer.rx_buf = dev->rx_buff;
	transfer.len = 1;
	transfer.speed_hz = SPI_BUS_SPEED_SLOW;
	spi_message_add_tail(&transfer, &msg);
	status = spi_sync(dev->spi, &msg);

	return status;
};

static int ads_read_registers(struct ads129x_chip *dev, u8 reg, u8 size)
{
	struct spi_message msg = { };
	struct spi_transfer transfer = { };
	int ret;

	spi_message_init(&msg);
	dev->tx_buff[0] = ADS1298_RREG | reg;
	dev->tx_buff[1] = size - 1; /* datasheet: # of registers minus one */
	memset(dev->rx_buff + 2, 0, size);
	memset(dev->tx_buff + 2, 0, size);
	transfer.speed_hz = SPI_BUS_SPEED_SLOW;
	transfer.tx_buf = dev->tx_buff;
	transfer.rx_buf = dev->rx_buff;
	transfer.len = size + 2;
	transfer.delay_usecs = 2; /* Delay 4 clocks after each message */
	transfer.cs_change = 1; /* Always toggle CS line after transfer */
	spi_message_add_tail(&transfer, &msg);
	ret = spi_sync(dev->spi, &msg);
	return ret;
}


static int ads_send_RREG(struct ads129x_chip *chip, char __user * buf, u8 reg, u8 size)
{
	int status;

	status = ads_read_registers(chip, reg, size);
	if (unlikely(status))
		return status;
	status = copy_to_user(buf, chip->rx_buff + 2, size);
	return status;
}

static int ads_send_WREG(struct ads129x_chip *dev, char __user * buf, u8 reg, u8 size)
{
	int status;
	struct spi_message msg = { };
	struct spi_transfer transfer = { };

	spi_message_init(&msg);
	dev->tx_buff[0] = ADS1298_WREG | reg;
	dev->tx_buff[1] = size-1;
	status = copy_from_user(dev->tx_buff + 2, buf, size);
	if (unlikely(status))
		return status;
	transfer.speed_hz = SPI_BUS_SPEED_SLOW;
	transfer.tx_buf = dev->tx_buff;
	transfer.rx_buf = dev->rx_buff;
	transfer.len = size + 2;
	transfer.delay_usecs = 2; /* Delay 4 clocks after each message */
	transfer.cs_change = 1; /* Always toggle CS line after transfer */
	spi_message_add_tail(&transfer, &msg);
	status = spi_sync(dev->spi, &msg);

	pr_debug("%s(%#x,%d) %#x .. -> %d\n",
		__func__, reg, size, dev->tx_buff[2], status);

	return status;
}

static int ads_set_register(struct ads129x_chip *dev, u8 reg, u8 value)
{
	struct spi_message msg = { };
	struct spi_transfer transfer = { };

	spi_message_init(&msg);
	dev->tx_buff[0] = ADS1298_WREG | reg;
	dev->tx_buff[1] = 0;
	dev->tx_buff[2] = value;
	transfer.tx_buf = dev->tx_buff;
	transfer.rx_buf = dev->rx_buff;
	transfer.len = 3;
	transfer.speed_hz = SPI_BUS_SPEED_SLOW;
	transfer.delay_usecs = 2; /* Delay 4 clocks after each message */
	transfer.cs_change = 1; /* Always toggle CS line after transfer */
	spi_message_add_tail(&transfer, &msg);
	pr_debug("%s(%#x) %#x\n", __func__, reg, dev->tx_buff[2]);
	return spi_sync(dev->spi, &msg);
}

static loff_t ads_cdev_llseek(struct file *filp, loff_t off, int whence)
{
	pr_debug("%s\n", __func__);
	return 0;
};

static ssize_t ads_cdev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	int i, status, bytes_send=0;
	struct ads129x_dev *ads = filp->private_data;

	if (*f_pos > 0)
		return 0;

	if (unlikely(down_interruptible(&ads->fop_sem)))
		return -ERESTARTSYS;

	for (i = 0; i < ads->num_ads_chips; i++) {
		memset(ads->chip[i].tx_buff, 0, ACQ_BUF_SIZE);
		ads->chip[i].transfer.tx_buf = ads->chip[i].tx_buff;
		ads->chip[i].transfer.rx_buf = ads->chip[i].rx_buff;
		ads->chip[i].transfer.len = ACQ_BUF_SIZE;
		ads->chip[i].transfer.speed_hz = SPI_BUS_SPEED_FAST;
		ads->chip[i].transfer.cs_change = 1;
		spi_message_init(&ads->chip[i].msg);
		spi_message_add_tail(&ads->chip[i].transfer, &ads->chip[i].msg);
		ads->chip[i].msg.context = ads;
		ads->chip[i].msg.complete = ads129x_spi_handler;
		pr_debug("Setting chip #%d in DATAC\n", i);
		ads_send_byte(&ads->chip[i], ADS1298_RDATAC);
	}

	gpio_set_value(ads129x_gpio_start.gpio, 1);
	up(&ads->fop_sem);

	while (bytes_send <= (count - (ads->num_ads_chips * ACQ_BUF_SIZE))) {
		ads->rx_count = 0;
		ads->sample_req = 1;
		status = wait_event_interruptible_timeout(ads->wait_queue, ads->rx_count == ads->num_ads_chips, HZ*1);
		ads->sample_req = 0;
		if (unlikely(status <= 0))
		{
			if (status == 0)
				status = -ETIMEDOUT; /* 1s without any data -> report timeout*/
			goto read_error;
		}
		for (i=0; i<ads->num_ads_chips; i++) {
			if (unlikely(copy_to_user(&buf[bytes_send], ads->chip[i].rx_buff, ACQ_BUF_SIZE)))
			{
				status = -EFAULT;
				goto read_error;
			}
			bytes_send += ACQ_BUF_SIZE;
		}
	};
	*f_pos += bytes_send;
	return bytes_send;

read_error:
	return status;
};

static ssize_t ads_cdev_write(struct file *filp, const char __user *buf, size_t count, loff_t *pos)
{
	int status = -ENODEV;
	int i;
	struct ads129x_dev *dev = filp->private_data;
	struct spi_message msg;
	struct spi_transfer transfer;

	pr_debug("%s\n", __func__);

	if (unlikely(down_interruptible(&dev->fop_sem)))
		return -ERESTARTSYS;

	if (count > SPI_BUFF_SIZE)
		count = SPI_BUFF_SIZE;

	for (i = 0; i < dev->num_ads_chips; i++) {
		if (unlikely(copy_from_user(dev->chip[i].tx_buff, buf, count)))
		{
			status = -EFAULT;
		}
		else
		{
			spi_message_init(&msg);
			transfer.tx_buf = dev->chip[i].tx_buff;
			transfer.rx_buf = dev->chip[i].rx_buff;
			transfer.len = count;
			transfer.speed_hz = SPI_BUS_SPEED_SLOW;
			spi_message_add_tail(&transfer, &msg);
			status = spi_sync(dev->chip[i].spi, &msg);
			if (status == 0)
			{
				status = count;
			}
		}
	}

	up(&dev->fop_sem);
	return status;
};

static long ads_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int status = -ENODEV;
	int i;
	int cmd_nr = _IOC_NR(cmd) & 0x7F;
	int cmd_size = _IOC_SIZE(cmd) & ADS1298_SIZE_MASK;

	int single_dev = (_IOC_NR(cmd) & ADS1298_SINGLE_DEV_CTRL) ? 1 : 0;
	int dev_id =  (_IOC_SIZE(cmd) & 0xE0) >> 5;

	struct ads129x_dev *dev = filp->private_data;

	if (dev_id >= dev->num_ads_chips) {
		printk(KERN_WARNING "Device %d does not exist.\n", dev_id);
		return -ENODEV;
	}

	if (_IOC_TYPE(cmd) != ADS1298_IOC_MAGIC)
		return -ENOTTY;

	if (dev == NULL)
	{
		printk(KERN_WARNING "ads_ioctl without device?\n");
		return -ENODEV;
	}

	if (unlikely(down_interruptible(&dev->fop_sem)))
		return -ERESTARTSYS;

	if (cmd_nr < ADS1298_RREG)
	{
		if ((cmd_nr & ADS1298_CMD_MASK) == ADS1298_SDATAC)
			gpio_set_value(ads129x_gpio_start.gpio, 0);
		if (single_dev)
			status = ads_send_byte(&dev->chip[dev_id], cmd_nr);
		else {
			for (i = (dev->num_ads_chips-1); i >= 0; i--)
				status = ads_send_byte(&dev->chip[i], cmd_nr);
		}
		if ((cmd_nr & ADS1298_CMD_MASK) == ADS1298_RDATAC)
			gpio_set_value(ads129x_gpio_start.gpio, 1);
	}
	else
	{
		if (gpio_get_value(ads129x_gpio_start.gpio))
		{
			printk(KERN_WARNING "Cannot change registers during aquisition, send SDATAC first\n");
			status = -EBUSY;
		}
		else
		{
			switch (cmd_nr & (ADS1298_RREG | ADS1298_WREG))
			{
				case ADS1298_RREG:
					if (single_dev)
						status = ads_send_RREG(&dev->chip[dev_id], (char __user *)arg, cmd_nr, cmd_size);
					else
						status = ads_send_RREG(&dev->chip[0], (char __user *)arg, cmd_nr, cmd_size);
					break;
				case ADS1298_WREG:
					if (single_dev)
						status = ads_send_WREG(&dev->chip[dev_id], (char __user *)arg, cmd_nr, cmd_size);
					else
						for (i = (dev->num_ads_chips-1); i >= 0; i--)
							status = ads_send_WREG(&dev->chip[i], (char __user *)arg, cmd_nr, cmd_size);
					break;
				default:
					status = -ENOTTY;
					break;
			}
		}
	}
	up(&dev->fop_sem);
	return status;
};

static void ads_power(int mode)
{
	gpio_set_value(ads129x_gpio_reset.gpio, mode);
	gpio_set_value(ads129x_gpio_pwdn.gpio, mode);
}

static int ads_cdev_open(struct inode *inode, struct file *filp)
{
	int status;
	struct ads129x_dev *ads = container_of(inode->i_cdev, struct ads129x_dev, dev);
	int i;
	int retries = 3;

	pr_debug("%s\n", __func__);
	if (down_interruptible(&ads->fop_sem))
		return -ERESTARTSYS;

	filp->private_data = ads;

	gpio_set_value(ads129x_gpio_start.gpio, 0);
	ads_power(1); /* Power up ADS chip(s) */
	do {
		status = 0;
		msleep(20);	/* Wait for power on, or just delay on retry */
		/* Reset pulse */
		gpio_set_value(ads129x_gpio_reset.gpio, 0);
		udelay(10);
		gpio_set_value(ads129x_gpio_reset.gpio, 1);
		udelay(10);	// Wait for reset (>18 clks)

		for (i=0; i< ads->num_ads_chips; i++) {
			ads_send_byte(&ads->chip[i], ADS1298_SDATAC);
			/* Check if chip is okay */
			ads_read_registers(&ads->chip[i], 0x00, 1);
			if (unlikely(ads->chip[i].rx_buff[2] != 0x92))
			{
				printk(KERN_WARNING "Bad chip[%d] ID, 0x%x is not an ads1298\n", i, ads->chip[i].rx_buff[2]);
				status = -ENODEV;
			}
		}
	} while ((status != 0) && (--retries));

	if (status < 0) {
		/* power off the device */
		ads_power(0);
	} else {
		for (i=0; i< ads->num_ads_chips; i++) {
			/* Set high-resolution mode (not low power)
			* Enable multiple-readback mode (disable daisy-chain)
			* 1kHz sampling freq */
			ads_set_register(&ads->chip[i], 0x01, 0b11000101);
			/* Enable internal VREF buffer, VREF=4V */
			ads_set_register(&ads->chip[i], 0x03, 0b11100000);
		}
	}

	up(&ads->fop_sem);
	return status;
};

static int ads_cdev_release(struct inode *inode, struct file *filp)
{
	struct ads129x_dev *dev = filp->private_data;
	int i;

	pr_debug("%s\n", __func__);
	if (down_interruptible(&dev->fop_sem))
		return -ERESTARTSYS;
	gpio_set_value(ads129x_gpio_start.gpio, 0);
	for (i=0; i < dev->num_ads_chips; i++) {
		ads_set_register(&dev->chip[i], 0x03, 0b11100000);
		ads_send_byte(&dev->chip[i], ADS1298_STANDBY);
	}
	ads_power(0);
	up(&dev->fop_sem);
	return 0;
};

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.llseek = ads_cdev_llseek,
	.read = ads_cdev_read,
	.write = ads_cdev_write,
	.unlocked_ioctl = ads_cdev_ioctl,
	.open = ads_cdev_open,
	.release = ads_cdev_release,
};


static int ads129x_init_cdev(struct ads129x_dev *ads)
{
	int ret;

	ret = alloc_chrdev_region( &ads->dev_no /*&devt*/ , 0, 1,"ads129x_cdev");
	if (ret < 0) {
		printk(KERN_ERR "alloc_chrdev_region failed!\n");
		goto error_exit;
	}
	ads->cl = class_create(THIS_MODULE, "ads129x_cdev");
	if (ads->cl == NULL) {
		printk(KERN_ERR "Could not create class!\n");
		goto cleanup_region;
	}

	if (device_create(ads->cl, NULL, ads->dev_no, NULL, "ads129x_cdev") == NULL) {
		printk(KERN_ERR "Could not create character device!\n");
		goto cleanup_class;
	}

	cdev_init(&ads->dev, &fops );
	ret = cdev_add(&ads->dev, ads->dev_no, 1);
	if (ret < 0) {
		printk(KERN_ERR "Could not add character device!\n");
		goto cleanup_dev;
	}

	return 0;

cleanup_dev:
	device_destroy(ads->cl, ads->dev_no);
cleanup_class:
	class_destroy(ads->cl);
cleanup_region:
	unregister_chrdev_region(ads->dev_no, 1);
error_exit:
	return (ret < 0) ? ret : -1;
};

static int ads129x_get_free_chip_pointer(struct ads129x_chip **c)
{
	int i;
	for (i=0; i < MAX_ADS_CHIPS; i++) {
		if (ads_inst.chip[i].spi == NULL) {
			*c = &ads_inst.chip[i];
			return 0;
		}
	}
	return -ENOMEM;
}

static int ads129x_probe(struct spi_device *spi)
{
	int ret;
	struct ads129x_chip *chip;

	mutex_lock(&ads_inst.mutex);

	pr_debug("%s\n", __func__);

	ret = ads129x_get_free_chip_pointer(&chip);
	if (ret < 0) {
		printk(KERN_ERR "Error, no ads_chip structure available!\n");
		goto error_exit;
	}
	chip->spi = spi;
	chip->rx_buff = devm_kzalloc(&spi->dev, SPI_BUFF_SIZE, GFP_KERNEL | GFP_DMA);
	if (chip->rx_buff == NULL) {
		ret = -ENOMEM;
		goto error_exit;
	}

	chip->tx_buff = devm_kzalloc(&spi->dev, SPI_BUFF_SIZE, GFP_KERNEL | GFP_DMA);
	if (chip->tx_buff == NULL) {
		ret = -ENOMEM;
		goto error_exit;
	}

	ret = ads129x_init_gpio_pins(&ads_inst, &spi->dev);
	if (ret < 0)
		goto error_exit;
	ads_inst.num_ads_chips++;
	printk(KERN_DEBUG "ADS Chip registered, %d chips total.\n", ads_inst.num_ads_chips);

	mutex_unlock(&ads_inst.mutex);
	return 0;

error_exit:
	mutex_unlock(&ads_inst.mutex);
	return ret;
}

static int ads129x_remove(struct spi_device *spi)
{
	mutex_lock(&ads_inst.mutex);
	ads_inst.num_ads_chips--;
	mutex_unlock(&ads_inst.mutex);
	pr_debug("ADS Remove: %d chips left!\n", ads_inst.num_ads_chips);
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


static int __init ads129x_init(void)
{
	int i, ret;

	/* init mutex */
	mutex_init(&ads_inst.mutex);
	/* init semaphore(s) */
	sema_init(&ads_inst.fop_sem, 1);

	ads_inst.gpio_initialized = 0;
	ads_inst.sample_req = 0;
	ads_inst.num_ads_chips = 0;

	for (i=0; i < MAX_ADS_CHIPS; i++)
		ads_inst.chip[i].spi = NULL;

	ret = ads129x_init_cdev(&ads_inst);
	if (ret < 0)
		goto error_exit;

	init_waitqueue_head(&ads_inst.wait_queue);

        return spi_register_driver(&ads129x_driver);

error_exit:
	return ret;
}
subsys_initcall(ads129x_init);

static void __exit ads129x_exit(void)
{
	free_irq(ads_inst.irq, NULL);
	cdev_del(&ads_inst.dev);
	device_destroy(ads_inst.cl, ads_inst.dev_no);
	class_destroy(ads_inst.cl);
	unregister_chrdev_region(ads_inst.dev_no, 1);
	spi_unregister_driver(&ads129x_driver);
}
module_exit(ads129x_exit);

MODULE_AUTHOR("Mike Looijmans <mike.looijmans@topic.nl>");
MODULE_DESCRIPTION("TI ADS129x ADC");
MODULE_LICENSE("GPL v2");
