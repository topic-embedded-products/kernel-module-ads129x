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
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include <linux/fs.h>
#include <linux/cdev.h>

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

#define NUM_ADS_CHIPS		2

struct ads129x_dev {
	struct cdev dev;
	dev_t dev_no;
	struct class *cl;
	struct semaphore fop_sem;

	// Spi	
	struct spi_device *spi;
	char *rx_buff;
	char *tx_buff;
};

static struct ads129x_dev ads_inst;


struct gpio ads129x_gpio_clksel = { .label = "clksel-gpio", .flags = GPIOF_OUT_INIT_LOW };
struct gpio ads129x_gpio_drdy = { .label = "drdy-gpio", .flags = GPIOF_IN };
struct gpio ads129x_gpio_pwdn = { .label = "pwdn-gpio", .flags = GPIOF_OUT_INIT_LOW };
struct gpio ads129x_gpio_reset = { .label = "reset-gpio", .flags = GPIOF_OUT_INIT_LOW };
struct gpio ads129x_gpio_start = { .label = "start-gpio", .flags =GPIOF_OUT_INIT_LOW };


static irqreturn_t ads129x_irq_handler(int irq, void *id)
{
	gpio_set_value(ads129x_gpio_clksel.gpio, gpio_get_value(ads129x_gpio_clksel.gpio) ? 0 : 1 );
        
	return IRQ_HANDLED;
}


static int ads129x_init_io_from_dt(struct device *dev, struct gpio *pgpio){
	int ret;
	int irq;
	struct device_node *np = dev->of_node;
	
	pgpio->gpio = of_get_named_gpio(np, pgpio->label, 0);
	printk("%s: %d\n", pgpio->label, pgpio->gpio);
	if(!gpio_is_valid(pgpio->gpio)){
		dev_err(dev, "Invalid gpio\n");
		return -EINVAL;
	}else{
		ret = devm_gpio_request_one(dev, pgpio->gpio, pgpio->flags, pgpio->label);
		if(ret < 0){
			printk("Error requesting gpio!\n");
			return ret;
		}
		if(pgpio->flags == GPIOF_IN){
			irq = gpio_to_irq(pgpio->gpio);
			printk(" - %s-int = %d\n", pgpio->label, irq);
			devm_request_irq(dev, irq, ads129x_irq_handler, 0, "ads192x irq handler", NULL);
			irq_set_irq_type(irq, IRQ_TYPE_EDGE_RISING);
		}
	}

	return 0;
};

static int ads129x_init_gpio_pins(struct device *dev){
	int ret;

	ret = ads129x_init_io_from_dt(dev, &ads129x_gpio_clksel);
	if(ret < 0){ return ret; }
	ret = ads129x_init_io_from_dt(dev, &ads129x_gpio_drdy);
	if(ret < 0){ return ret; }
	ret = ads129x_init_io_from_dt(dev, &ads129x_gpio_pwdn); 
	if(ret < 0){ return ret; }
	ret = ads129x_init_io_from_dt(dev, &ads129x_gpio_reset);
	if(ret < 0){ return ret; }
	ret = ads129x_init_io_from_dt(dev, &ads129x_gpio_start);
	if(ret < 0){ return ret; }

	return 0;
};


// -----

static int ads_send_byte(struct ads129x_dev *dev, int data){
	int status;
	struct spi_message msg;
	struct spi_transfer transfer;
	
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

static int ads_read_registers(struct ads129x_dev *dev, u8 reg, u8 size)
{
        struct spi_message msg;
        struct spi_transfer transfer;

	spi_message_init(&msg);
	dev->tx_buff[0] = ADS1298_RREG | reg;
	dev->tx_buff[1] = size - 1; /* datasheet: # of registers minus one */
	memset(dev->rx_buff + 2, 0, size);
	memset(dev->tx_buff + 2, 0, size);
	transfer.speed_hz = SPI_BUS_SPEED_SLOW;
	transfer.tx_buf = dev->tx_buff;
	transfer.rx_buf = dev->rx_buff;
	transfer.len = size + 2;
	spi_message_add_tail(&transfer, &msg);
	return spi_sync(dev->spi, &msg);
}


static int ads_send_RREG(struct ads129x_dev *dev, char __user * buf, u8 reg, u8 size)
{
	int status;

	status = ads_read_registers(dev, reg, size);

	if (likely(status == 0))
	{
		status = copy_to_user(buf, dev->rx_buff + 2, size);
	}

	return status;
}

static int ads_send_WREG(struct ads129x_dev *dev, char __user * buf, u8 reg, u8 size)
{
	int status;
        struct spi_message msg;
        struct spi_transfer transfer;

	spi_message_init(&msg);
	dev->tx_buff[0] = ADS1298_WREG | reg;
	dev->tx_buff[1] = size-1;

	status = copy_from_user(dev->tx_buff + 2, buf, size);
	if (likely(status == 0))
	{
		transfer.speed_hz = SPI_BUS_SPEED_SLOW;
		transfer.tx_buf = dev->tx_buff;
		transfer.rx_buf = dev->rx_buff;
		transfer.len = size + 2;
		spi_message_add_tail(&transfer, &msg);
		status = spi_sync(dev->spi, &msg);
	}

	return status;
}

static int ads_set_register(struct ads129x_dev *dev, u8 reg, u8 value)
{
        struct spi_message msg;
        struct spi_transfer transfer;

	spi_message_init(&msg);
	dev->tx_buff[0] = ADS1298_WREG | reg;
	dev->tx_buff[1] = 0;
	dev->tx_buff[2] = value;
	transfer.tx_buf = dev->tx_buff;
	transfer.rx_buf = dev->rx_buff;
	transfer.len = 3;
	transfer.speed_hz = SPI_BUS_SPEED_SLOW;
	spi_message_add_tail(&transfer, &msg);
	return spi_sync(dev->spi, &msg);
}

// -----

static loff_t ads_cdev_llseek(struct file *filp, loff_t off, int whence){
	printk(" # llseek\n");
	return 0;
};

static ssize_t ads_cdev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos){
};

static ssize_t ads_cdev_write(struct file *filp, const char __user *buf, size_t count, loff_t *pos){
	int status;
	struct ads129x_dev *dev = filp->private_data;
	struct spi_message msg;
	struct spi_transfer transfer;

	if (unlikely(down_interruptible(&dev->fop_sem)))
		return -ERESTARTSYS;

	if (count > SPI_BUFF_SIZE)
		count = SPI_BUFF_SIZE;

	if (unlikely(copy_from_user(dev->tx_buff, buf, count)))
	{
		status = -EFAULT;
	}
	else
	{
		spi_message_init(&msg);
		transfer.tx_buf = dev->tx_buff;
		transfer.rx_buf = dev->rx_buff;
		transfer.len = count;
		transfer.speed_hz = SPI_BUS_SPEED_SLOW;
		spi_message_add_tail(&transfer, &msg);
		status = spi_sync(dev->spi, &msg);
		if (status == 0)
		{
			status = count;
		}
	}

	up(&dev->fop_sem);
	return status;
};

static long ads_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg){
	int status;
	int cmd_nr = _IOC_NR(cmd);
	struct ads129x_dev *dev = filp->private_data;

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
		// Single byte command without response
		status = ads_send_byte(dev, cmd_nr);
	}
	else
	{
		if (false /*dev->mode == MODE_RUNNING*/)
		{
			printk(KERN_WARNING "Cannot change registers during aquisition, send SDATAC first\n");
			status = -EBUSY;
		}
		else
		{
			switch (cmd_nr & (ADS1298_RREG | ADS1298_WREG))
			{
				case ADS1298_RREG:
					status = ads_send_RREG(dev, (char __user *)arg, cmd_nr, _IOC_SIZE(cmd));
					break;
				case ADS1298_WREG:
					status = ads_send_WREG(dev, (char __user *)arg, cmd_nr, _IOC_SIZE(cmd));
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

static int ads_cdev_open(struct inode *inode, struct file *filp){
	int status = 0;

	struct ads129x_dev *ads = container_of(inode->i_cdev, struct ads129x_dev, dev);

	if (down_interruptible(&ads->fop_sem)){
		return -ERESTARTSYS;	
	}

	filp->private_data = ads;

	// Power up ADS chip(s)
	gpio_set_value(ads129x_gpio_pwdn.gpio, 1);
	gpio_set_value(ads129x_gpio_reset.gpio, 1);

	msleep(1);	// Wait for power on

	// Reset pulse
	gpio_set_value(ads129x_gpio_reset.gpio, 0);
	udelay(1);
	gpio_set_value(ads129x_gpio_reset.gpio, 1);

	udelay(10);	// Wait for reset (>18 clks)

	ads_send_byte(ads, ADS1298_SDATAC);

	/* Check if chip is okay */
	ads_read_registers(ads, 0x00, 1);
	if (unlikely(ads->rx_buff[2] != 0x92))
	{
		printk(KERN_WARNING "Bad chip ID, 0x%x is not a ads1298\n", ads->rx_buff[2]);
		status = -ENODEV;
	}
	else
	{
		/* Set high-resolution mode (not low power)
		* Enable multiple-readback mode (disable daisy-chain)
		* 1kHz sampling freq */
		ads_set_register(ads, 0x01, 0b11000101);
		/* Enable internal VREF buffer, VREF=4V */
		ads_set_register(ads, 0x03, 0b11100000);
	}

	if (status < 0)
	{
		// dev->mode = MODE_IDLE;
		/* power off the device */
		gpio_set_value(ads129x_gpio_pwdn.gpio, 0);
		gpio_set_value(ads129x_gpio_reset.gpio, 0);
	}

	up(&ads->fop_sem);

	return status;

};

static int ads_cdev_release(struct inode *inode, struct file *filp){
        printk(" # release\n");
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


static int ads129x_init_cdev(struct ads129x_dev *ads){
	int ret;

	ret = alloc_chrdev_region( &ads->dev_no /*&devt*/ , 0, 1,"ads129x_cdev");
	if (ret < 0) {
		printk("alloc_chrdev_region failed!\n");
		goto error_exit; 
	}
	ads->cl = class_create(THIS_MODULE, "ads129x_cdev");
	if(ads->cl == NULL){
		printk("Could not create class!\n");
		goto cleanup_region;
	}
	
	if(device_create(ads->cl, NULL, ads->dev_no, NULL, "ads129x_cdev") == NULL){
		printk("Could not create character device!\n");
		goto cleanup_class;
	}

	cdev_init(&ads->dev, &fops );
	ret = cdev_add(&ads->dev, ads->dev_no, 1);
	if(ret < 0){
		printk("Could not add character device!\n");
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

static int ads129x_probe(struct spi_device *spi)
{
	int ret;
	
	/* init semaphore(s) */
	sema_init(&ads_inst.fop_sem, 1);
	ads_inst.rx_buff = devm_kzalloc(&spi->dev, SPI_BUFF_SIZE, GFP_KERNEL | GFP_DMA);
	if(ads_inst.rx_buff == NULL){
		ret = -ENOMEM;
		goto error_exit;
	}

        ads_inst.tx_buff = devm_kzalloc(&spi->dev, SPI_BUFF_SIZE, GFP_KERNEL | GFP_DMA);
        if(ads_inst.tx_buff == NULL){
                ret = -ENOMEM;
                goto error_exit;
        }


	ads_inst.spi = spi;
	spi_set_drvdata(spi, &ads_inst);

	ret = ads129x_init_gpio_pins(&spi->dev);
	//if(ret < 0){ TODO }

	ret = ads129x_init_cdev(&ads_inst);
	if(ret < 0){
		// TODO: cleanup gpio
		return ret;
	}
	
	return 0;

error_exit:
	return ret;
}

static int ads129x_remove(struct spi_device *spi)
{
	cdev_del(&ads_inst.dev);
	device_destroy(ads_inst.cl, ads_inst.dev_no);
	class_destroy(ads_inst.cl);

	unregister_chrdev_region(ads_inst.dev_no, 1);

	// TODO: Cleanup

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
