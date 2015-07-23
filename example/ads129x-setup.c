/*
 * ads129x series SPI ADC driver
 *
 * Copyright (C) 2014 Topic Embedded Products
 *
 * Licensed under the GPL-2.
 */

#include <stdlib.h>
#include <fcntl.h>      /* open */
#include <unistd.h>     /* exit */
#include <stdio.h>
#include <sys/ioctl.h>  /* ioctl */
#include <getopt.h>

#include "libads129x.h"

static const char default_file_name[] = "/dev/ads129x_cdev";

static void usage(const char* name)
{
	fprintf(stderr, "usage: %s [-d device] [-h] [-n|-t] [-s] [-r] ...\n"
		" -d    Device node file (default %s).\n"
		" -t    Set standard test mode.\n"
		" -n    Set normal mode.\n"
		" -s    Start aquisition (after setting options).\n"
		" -h    Stop aquisition (before setting options).\n"
		" -r    Read binary data from device.\n"
		" -f x  Set sample frequency (x=250, 500, 1000, ..., 16000)\n"
		" ...   Other arguments ignored for now, not implemented yet.\n",
		name, default_file_name);
}

static int exit_perror(const char* context)
{
	perror(context);
	return 1;
}


int main(int argc, char** argv)
{
	int opt;
	const char* device_name = default_file_name;
	int mode = 0;
	int stop = 0;
	int start = 0;
	int freq = 0;
	int fd;
	while ((opt = getopt(argc, argv, "d:f:hnrst")) != -1)
	{
		switch (opt)
		{
			case 'd':
				device_name = optarg;
				break;
			case 'f':
				freq = atoi(optarg);
				break;
			case 'n':
			case 't':
				mode = opt;
				break;
			case 'h':
				stop = 1;
				break;
			case 'r':
			case 's':
				start = opt;
				break;
			default:
				usage(argv[0]);
				return 1;
		}
	}
	fd = open(device_name, O_RDWR);
	if (fd == -1)
		return exit_perror(device_name);
	if (stop)
		if (ads129x_stop_acquisition(fd) < 0)
			return exit_perror("stop_acquisition");
	switch (mode)
	{
		case 'n':
			if (ads129x_set_default_mode(fd) < 0)
				return exit_perror("set_default_mode");
			break;
		case 't':
			if (ads129x_set_test_mode(fd) < 0)
				return exit_perror("set_test_mode");
			break;
	}
	if (freq)
	{
		unsigned char freq_code = 0xC5;
		unsigned char divider = 8;
		freq /= 250;
		if (freq <= 1)
			freq_code = 0x46;
		else
		{
			do {
				freq >>= 1;
				--divider;
			} while (freq && divider);
			freq_code = 0xC0 | divider;
		}
		if (ads129x_set_registers(fd, 1, &freq_code, 1) < 0)
			return exit_perror("ads129x_set_registers(frequency)");
	}
	if (start)
	{
		if (ads129x_start_acquisition(fd) < 0)
			return exit_perror("start_acquisition");
		if (start == 'r')
		{
			const size_t buffer_size = 1024;
			void* buffer = malloc(buffer_size);
			for (;;)
			{
				ssize_t bytes = read(fd, buffer, buffer_size);
				if (bytes == 0)
					break;
				if (bytes < 0)
					return exit_perror("read");
				if (write(1, buffer, bytes) < 0)
					return exit_perror("write");
			}
			free(buffer);
			return 0;
		}
	}
	read(0, &opt, 1); /* Wait for input */
	close(fd); /* Close will turn off power to the chips */
	return 0;
}
