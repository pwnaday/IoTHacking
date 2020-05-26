#ifndef __UART_H__
#define __UART_H__
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <termios.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#define __BAUD_RATE	    57600
#define	__READ_BLOCK	    0x0
#define __PACKET_SIZE	    CS8
#define __PARITY	    0x0
#define __RX_BUFFER_SIZE    0x80
#define __TX_BUFFER_SIZE    0x80
typedef struct termios tty_t;
typedef unsigned char uint8_t;

static const char* dev_name = "/dev/ttyUSB0";
extern int uart_fd; 
int set_rx_ack();
int set_tx_ack();
int  config_uart_io (int fd, int rate, int parity);
void config_blocking (int fd, int flag);
void uart_puts(const int8_t* str);
void uart_put (void* data, size_t size);
ssize_t uart_read(void* buf, size_t length);
int uart_init(const int8_t* serial_port, int baud_rate, int to_block);

#endif

