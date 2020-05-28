/*
 *  Basic serial console device driver code for communicating over USB UART. Async I/O works but is a bit sketchy.
 *  Version: 1.0
 *  Date   : 05/25/2020
 *  Author : @pwnaday
 *
 */
#ifndef __UART_C__
#define __UART_C__
#include <pthread.h>
#include "UART.h"

int8_t rxbuf[__RX_BUFFER_SIZE]; /* Data from device */
int8_t txbuf[__TX_BUFFER_SIZE]; /* Data to device   */

int uart_fd            = -1;
volatile int buflen    = 0;
volatile int ack_en    = 1;

/* Synchronization vars */
static pthread_mutex_t   mtx;
static pthread_cond_t    cv;

/* Sets up Serial device */
int config_uart_io (int fd, int rate, int parity)
{
    tty_t tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
	fprintf(stderr, "[tcgetattr] error %x recieved\n", errno);
	return -1;
    }
    cfsetospeed(&tty, rate); 
    cfsetispeed(&tty, rate);
    tty.c_cflag     = (tty.c_cflag & ~CSIZE) | CS8; /* CS8 encoding */
    tty.c_iflag     &= ~IGNBRK | ~ECHO;

    tty.c_lflag     = 0;
    tty.c_oflag     = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;

    /* tty attributes */
    tty.c_cflag    |= (CLOCAL | CREAD);
    tty.c_cflag    |= parity;
    tty.c_iflag    &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag    &= ~CSTOPB;
    tty.c_cflag    &= ~CRTSCTS;
    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
	fprintf(stderr, "[tcsetattr] error %x recieved\n", errno);
	return -1;
    }
    return 0;
}

/* Configures serial device blocking */
void config_blocking(int fd, int flag)
{
    tty_t tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
	fprintf(stderr, "[tcgetattr] error %x recieved\n", errno);
	return;
    }
    tty.c_cc[VMIN]  = flag ? 1 : 0;
    tty.c_cc[VTIME] = 5;
    if (tcsetattr (fd, TCSANOW, &tty) != 0) {
	fprintf(stderr, "[tcsetattr] error %x recieved\n", errno);
    }
    return;
}

/* writes a string to device */
void uart_puts(const int8_t* str)
{
    write(uart_fd, str, strlen(str));
    return;
}

/* more generic helper function to send data to device in packets of size size */
void uart_put(void* data, size_t size)
{
    write(uart_fd, data, size);
}

/* read device data */
ssize_t uart_read(void* buf, size_t length)
{
    return read(uart_fd, buf, length);
}

/* initialize serial device */
int uart_init(const int8_t* serial_port, int baud_rate, int to_block)
{
    uart_fd = open(serial_port, O_RDWR | O_NOCTTY | O_SYNC);
    if (uart_fd < 0) {
	fprintf(stderr, "[open] error %x recieved. Port: %s -> %s\n", errno, serial_port, strerror(errno));
	return -1;
    }
    config_uart_io(uart_fd, baud_rate, __PARITY);
    config_blocking(uart_fd, to_block);
    printf("Sucessfully opened serial Port %s at baud rate %d\n", serial_port, baud_rate);
    return 1;
}

/* set ack flag for async  I/O */
int set_ack()
{
    pthread_mutex_lock(&mtx);
    ack_en = 1;
    pthread_cond_signal(&cv);
    pthread_mutex_unlock(&mtx);
    return 0;
}

/* keyboard thread */
volatile int8_t c;
static void* tx_data(void*arg) {
    while (1) {
	pthread_mutex_lock(&mtx);
	c = getchar();
	if (buflen >= __TX_BUFFER_SIZE) {
	    int length = 0;
	    while (length < buflen) {
		uart_puts(txbuf + length);
		length += __TX_BUFFER_SIZE;
		buflen -= length;
	    }
	    uart_puts(txbuf + length);
	}
	if (c == '\n' || c == '\r' || c == '\0') {
	    txbuf[buflen] = c;
	    uart_puts(txbuf);
	    memset((void*)txbuf, 0, __TX_BUFFER_SIZE);
	    buflen = 0;
	} else {
	    txbuf[buflen++] = c;
	}

	pthread_mutex_unlock(&mtx);
    }
}
    
/* RX handler */
static void rx_data() {
    bzero(rxbuf, __RX_BUFFER_SIZE);
    uart_read(rxbuf, __RX_BUFFER_SIZE);
}

/* driver code */
int main()
{
    if (!uart_init(dev_name, B57600, __READ_BLOCK)) {
	return -1;
    }
    pthread_t tid1;
    pthread_t tid2;
    pthread_attr_t attr;
    pthread_mutex_init(&mtx, NULL);                              /* RX/TX mutex          */
    pthread_cond_init(&cv, NULL);                                /* Condition variable   */
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE); /* threads are joinable */
    pthread_create(&tid1, NULL, tx_data, (void*)txbuf); 
    for (;;) {                                                   /* spin nicely          */
	rx_data();
	printf("%s", rxbuf);
    }
    return 0;
}
#endif
