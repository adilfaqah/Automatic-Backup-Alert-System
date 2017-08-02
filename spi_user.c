/*
CSE 438 - Embedded Systems Programming
Project: 4
Description: This is the user space program for the Automatic Back-up Alert System driver.

Author: Adil Faqah
Date: 7th April 2016
*/
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <unistd.h>
#include "spidev_abas_defs.h"

void main(void)
{
	int fd = 0;
	unsigned int val = 0;

	fd = open("/dev/spidev1.0", O_RDWR);

	printf(KBLU"Automatic Back-up Alert System Userspace Test"RESET);
	
	//Try to set the minimum threshold values to less than 2 (outside hardware capability).
	val = 1;
	printf(KYEL"\n\n  Trying to set minimum threshold value to: %d."RESET, val);
	if(ioctl(fd, SPI_SET_MIN_THRESH, &val) < 0)
		printf(KRED"\n > Unable to set minimum threshold value."RESET);
	else
		printf(KGRN"\n > Minimum threshold value succesfully updated."RESET);

	//Try to set the minimum threshold values to a value greater than the maximum threshold value.
	val = 300;
	printf(KYEL"\n\n  Trying to set minimum threshold value to: %d."RESET, val);
	if(ioctl(fd, SPI_SET_MIN_THRESH, &val) < 0)
		printf(KRED"\n > Unable to set minimum threshold value."RESET);
	else
		printf(KGRN"\n > Minimum threshold value succesfully updated."RESET);

	//Try to set the minimum threshold value to a reasonable value.
	val = 10;
	printf(KYEL"\n\n  Trying to set minimum threshold value to: %d."RESET, val);
	if(ioctl(fd, SPI_SET_MIN_THRESH, &val) < 0)
		printf(KRED"\n > Unable to set minimum threshold value."RESET);
	else
		printf(KGRN"\n > Minimum threshold value succesfully updated."RESET);

	//Try to set the maximum threshold values to greater than 400 (outside hardware capability).
	val = 500;
	printf(KYEL"\n\n  Trying to set maximum threshold value to: %d."RESET, val);
	if(ioctl(fd, SPI_SET_MAX_THRESH, &val) < 0)
		printf(KRED"\n > Unable to set maximum threshold value."RESET);
	else
		printf(KGRN"\n > Maximum threshold value succesfully updated."RESET);

	//Try to set the maximum threshold values to a value lesser than the minimum threshold value.
	val = 5;
	printf(KYEL"\n\n  Trying to set maximum threshold value to: %d."RESET, val);
	if(ioctl(fd, SPI_SET_MAX_THRESH, &val) < 0)
		printf(KRED"\n > Unable to set maximum threshold value."RESET);
	else
		printf(KGRN"\n > Maximum threshold value succesfully updated."RESET);

	//Try to set the maximum threshold value to a reasonable value.
	val = 100;
	printf(KYEL"\n\n  Trying to set maximum threshold value to: %d."RESET, val);
	if(ioctl(fd, SPI_SET_MAX_THRESH, &val) < 0)
		printf(KRED"\n > Unable to set maximum threshold value.\n"RESET);
	else
		printf(KGRN"\n > Maximum threshold value succesfully updated.\n"RESET);

	close(fd);
}