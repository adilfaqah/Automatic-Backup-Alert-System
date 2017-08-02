#ifndef spidev_abas_defs_h
#define spidev_abas_defs_h

#include <linux/ioctl.h>

#define MAGIC_NUMBER 'k'
#define SPI_SET_MIN_THRESH _IO(MAGIC_NUMBER, 6)
#define SPI_SET_MAX_THRESH _IO(MAGIC_NUMBER, 7)

//Preprocessor directives to change the color of console output text.
#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define RESET "\033[0m"

#endif