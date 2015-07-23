/*
 * ads129x series SPI ADC driver
 *
 * Copyright (C) 2014 Topic Embedded Products
 *
 * Licensed under the GPL-2.
 */

#include <fcntl.h>
#include <sys/ioctl.h>
#include "libads129x.h"
#include "../ads129xio.h"

int ads129x_start_acquisition(int ads129x_fd)
{
	return ioctl(ads129x_fd, ADS1298_IOCRDATAC, 0);
};

int ads129x_stop_acquisition(int ads129x_fd)
{
	return ioctl(ads129x_fd, ADS1298_IOCSDATAC, 0);
};

/* 1kHz, internal test, no lead-off */
static char reg_set_1[] = { 0xC5, 0x10, 0xE0, 0x00 };
/* Gain = 6, mux is analog input, no lead-off detection */
static char reg_set_2[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static char reg_set_2_tst[] = { 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00 };
/* GPIO, PACE, RESP disabled. WCT enabled */
static char reg_set_3[] = { 0x0F, 0x00, 0x00, 0x00, 0x0D, 0xD4 };

int ads129x_set_default_mode(int ads129x_fd)
{
	int ret;
	ret = ioctl(ads129x_fd, _IOC(0, ADS1298_IOC_MAGIC, ADS1298_WREG | 0x01, 4), reg_set_1);
	ret = ioctl(ads129x_fd, _IOC(0, ADS1298_IOC_MAGIC, ADS1298_WREG | 0x05, 13), reg_set_2);
	ret = ioctl(ads129x_fd, _IOC(0, ADS1298_IOC_MAGIC, ADS1298_WREG | 0x14, 6), reg_set_3);
	return ret;
};

int ads129x_set_test_mode(int ads129x_fd)
{
	int ret;
	ret = ioctl(ads129x_fd, _IOC(0, ADS1298_IOC_MAGIC, ADS1298_WREG | 0x01, 4), reg_set_1);
	ret = ioctl(ads129x_fd, _IOC(0, ADS1298_IOC_MAGIC, ADS1298_WREG | 0x05, 13), reg_set_2_tst);
	ret = ioctl(ads129x_fd, _IOC(0, ADS1298_IOC_MAGIC, ADS1298_WREG | 0x14, 6), reg_set_3);
	return ret;
};

int ads129x_set_registers(int ads129x_fd, int offset, char* data, int count)
{
	return ioctl(ads129x_fd, _IOC(0, ADS1298_IOC_MAGIC, ADS1298_WREG | offset, count), data);
}
