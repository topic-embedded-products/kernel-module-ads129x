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


struct ads_cdev{
	struct class *cl;
	dev_t dev_no;
	struct cdev dev;
};

struct ads129x_dev {
	struct ads_cdev cdev;
};

static struct ads129x_dev ads_inst;

struct ads129x_state {
	struct spi_device *spi;
};

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

static loff_t ads_cdev_llseek(struct file *filp, loff_t off, int whence){
	printk(" # llseek\n");
	return 0;
};

static ssize_t ads_cdev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos){
        printk(" # read\n");
	return 0;
};

static ssize_t ads_cdev_write(struct file *filp, const char __user *buf, size_t count, loff_t *pos){
        printk("# write\n");
	return 0;
};

static long ads_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg){
        printk(" # ioctl\n");
	return 0;
};

static int ads_cdev_open(struct inode *inode, struct file *filp){
        printk(" # open\n");
	return 0;
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

	ret = alloc_chrdev_region( &ads->cdev.dev_no /*&devt*/ , 0, 1,"ads129x_cdev");
	if (ret < 0) {
		printk("alloc_chrdev_region failed!\n");
		goto error_exit; 
	}
	ads->cdev.cl = class_create(THIS_MODULE, "ads129x_cdev");
	if(ads->cdev.cl == NULL){
		printk("Could not create class!\n");
		goto cleanup_region;
	}
	
	if(device_create(ads->cdev.cl, NULL, ads->cdev.dev_no, NULL, "ads129x_cdev") == NULL){
		printk("Could not create character device!\n");
		goto cleanup_class;
	}

	cdev_init(&ads->cdev.dev, &fops );
	ret = cdev_add(&ads->cdev.dev, ads->cdev.dev_no, 1);
	if(ret < 0){
		printk("Could not add character device!\n");
		goto cleanup_dev;
	}

	return 0;

cleanup_dev:
	device_destroy(ads->cdev.cl, ads->cdev.dev_no);
cleanup_class:
	class_destroy(ads->cdev.cl);
cleanup_region:
	unregister_chrdev_region(ads->cdev.dev_no, 1);
error_exit:
	return (ret < 0) ? ret : -1;
};

static int ads129x_probe(struct spi_device *spi)
{
//	struct ads129x_state *st;
	int ret;

	printk("-- ads129x probe(..)!\n");
	
	ret = ads129x_init_gpio_pins(&spi->dev);

	//if(ret < 0){ TODO }

	ret = ads129x_init_cdev(&ads_inst);
	if(ret < 0){
		return ret;
	}

	printk("Writing test data... ( ");
	//ret = spi_write(spi, "Spi Test Data", 13);

	printk("%d )\n", ret);

	return ret;
}

static int ads129x_remove(struct spi_device *spi)
{
	cdev_del(&ads_inst.cdev.dev);
	device_destroy(ads_inst.cdev.cl, ads_inst.cdev.dev_no);
	class_destroy(ads_inst.cdev.cl);

	unregister_chrdev_region(ads_inst.cdev.dev_no, 1);

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
