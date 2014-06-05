#include <stdlib.h>
#include <fcntl.h>      /* open */
#include <unistd.h>     /* exit */
#include <stdio.h>
#include <sys/ioctl.h>  /* ioctl */
#include <getopt.h>

#include "../ads129xio.h"

static const char default_file_name[] = "/dev/ads129x_cdev";
static int ads129x_fd = -1;

/*
int ads129x::write_reg(uint8_t reg, const char* value){
	if(ads129x_fd < 0){
		return -1;
	}
	return ioctl(ads129x_fd, _IOC(0, ADS1298_IOC_MAGIC, ADS1298_WREG | reg, 1), value);
};

int ads129x::read_reg(uint8_t reg, char *value){
	if(ads129x_fd < 0){
		return -1;
	}
	return ioctl(ads129x_fd, _IOC(0, ADS1298_IOC_MAGIC, ADS1298_RREG | reg, 1), value);
};
*/

static int start_acquisition(int ads129x_fd)
{
	return ioctl(ads129x_fd, ADS1298_IOCRDATAC, NULL);
};

static int stop_acquisition(int ads129x_fd)
{
	return ioctl(ads129x_fd, ADS1298_IOCSDATAC, NULL);
};

static char reg_set_1[] = { 0xC5, 0x00, 0xE0, 0x00 };
static char reg_set_2[] = { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 };
static char reg_set_2_tst[] = { 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00 };
static char reg_set_3[] = { 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00 };

static int set_default_mode(int ads129x_fd)
{
	int ret;
	ret = ioctl(ads129x_fd, _IOC(0, ADS1298_IOC_MAGIC, ADS1298_WREG | 0x01, 4), reg_set_1);
	ret = ioctl(ads129x_fd, _IOC(0, ADS1298_IOC_MAGIC, ADS1298_WREG | 0x05, 13), reg_set_2);
	ret = ioctl(ads129x_fd, _IOC(0, ADS1298_IOC_MAGIC, ADS1298_WREG | 0x14, 6), reg_set_3);
	return ret;
};

static int set_test_mode(int ads129x_fd)
{
	int ret;
	ret = ioctl(ads129x_fd, _IOC(0, ADS1298_IOC_MAGIC, ADS1298_WREG | 0x01, 4), reg_set_1);
	ret = ioctl(ads129x_fd, _IOC(0, ADS1298_IOC_MAGIC, ADS1298_WREG | 0x05, 13), reg_set_2_tst);
	ret = ioctl(ads129x_fd, _IOC(0, ADS1298_IOC_MAGIC, ADS1298_WREG | 0x14, 6), reg_set_3);
	return ret;
};

static void usage(const char* name)
{
	fprintf(stderr, "usage: %s [-d device] [-h] [-n|-t] [-s] [-r] ...\n"
		" -d    Device node file (default %s).\n"
		" -t    Set standard test mode.\n"
		" -n    Set normal mode.\n"
		" -s    Start aquisition (after setting options).\n"
		" -h    Stop aquisition (before setting options).\n"
		" -r    Read binary data from device.\n"
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
	int fd;
	while ((opt = getopt(argc, argv, "d:hnrst")) != -1)
	{
		switch (opt)
		{
			case 'd':
				device_name = optarg;
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
		if (stop_acquisition(fd) < 0)
			return exit_perror("stop_acquisition");
	switch (mode)
	{
		case 'n':
			if (set_default_mode(fd) < 0)
				return exit_perror("set_default_mode");
			break;
		case 't':
			if (set_test_mode(fd) < 0)
				return exit_perror("set_test_mode");
			break;
	}
	if (start)
	{
		if (start_acquisition(fd) < 0)
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
