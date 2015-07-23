/*
 * ads129x series SPI ADC driver
 *
 * Copyright (C) 2014 Topic Embedded Products
 *
 * Licensed under the GPL-2.
 */

#pragma once

int ads129x_start_acquisition(int ads129x_fd);
int ads129x_stop_acquisition(int ads129x_fd);
int ads129x_set_default_mode(int ads129x_fd);
int ads129x_set_test_mode(int ads129x_fd);
int ads129x_set_registers(int ads129x_fd, int offset, char* data, int count);
