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
#include <pthread.h>
#include <signal.h>
#include <poll.h>
#include <sys/stat.h>
#include <sys/types.h>

#define __BAUD_RATE	    57600
#define	__READ_BLOCK	    0x0
#define __PACKET_SIZE	    CS8
#define __PARITY	    0x0
#define __RX_BUFFER_SIZE    0x80
#define __TX_BUFFER_SIZE    0x80
typedef struct termios tty_t;
typedef unsigned char uint8_t;
struct serial_console {
    int         fd;
    int         state;
    int         active;
    int8_t      rxbuf[__RX_BUFFER_SIZE];
    uint32_t    start;
    uint32_t    end;
    pthread_t   rx_thread;
    tty_t	tty;
};
typedef struct serial_console serial_t;
static const char* dev_name = "/dev/ttyUSB0";
extern int uart_fd; 
extern int set_ack();
extern int  config_uart_io (int fd, int rate, int parity);
extern void config_blocking (int fd, int flag);
extern void uart_puts(const int8_t* str);
extern void uart_put (void* data, size_t size);
extern ssize_t uart_read(void* buf, size_t length);
extern int uart_init(const int8_t* serial_port, int baud_rate, int to_block);

#endif

