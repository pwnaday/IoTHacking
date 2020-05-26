#ifndef __UART_C__
#define __UART_C__
#include <pthread.h>
#include "UART.h"

int8_t rxbuf[__RX_BUFFER_SIZE];
int8_t txbuf[__TX_BUFFER_SIZE];
int uart_fd = -1;
volatile int ack_en = 0;
static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  cv  = PTHREAD_COND_INITIALIZER;

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
    tty.c_cflag     = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag     &= ~IGNBRK;

    tty.c_lflag     = 0;
    tty.c_oflag     = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;

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

void uart_puts(const int8_t* str)
{
    write(uart_fd, str, strlen(str));
    return;
}

void uart_put(void* data, size_t size)
{
    write(uart_fd, data, size);
}

ssize_t uart_read(void* buf, size_t length)
{
    return read(uart_fd, buf, length);
}

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

int set_ack()
{
    pthread_mutex_lock(&mtx);
    ack_en = 1;
    pthread_cond_signal(&cv);
    pthread_mutex_unlock(&mtx);
    return 0;
}
static void rx_data()
{
    bzero(rxbuf, __RX_BUFFER_SIZE);
    uart_read(rxbuf, __RX_BUFFER_SIZE);
}
static void tx_data()
{
    bzero(txbuf, __TX_BUFFER_SIZE);
    int buflen = 0;
    while ((txbuf[buflen++] = getchar()) != '\n');
}
    
static void *rx_tx_thread(void *arg) {
    //while (1) {
	pthread_mutex_lock(&mtx);
	while (!ack_en) {
	    pthread_cond_wait(&cv, &mtx);
	}
	bzero(rxbuf, __RX_BUFFER_SIZE);
	bzero(txbuf, __TX_BUFFER_SIZE);
	uart_read(rxbuf, __RX_BUFFER_SIZE);
	ack_en = 0;
	pthread_mutex_unlock(&mtx);
    //}
    return NULL;
}


static void *uart_thread(void *arg)
{
	pthread_mutex_lock(&mtx);
	pthread_cond_signal(&cv);
	uart_puts((const int8_t*)txbuf);
	uart_read(rxbuf, __RX_BUFFER_SIZE);
	pthread_mutex_unlock(&mtx);
    return NULL;
}


int main()
{
    if (!uart_init(dev_name, B57600, __READ_BLOCK)) {
	return -1;
    }
    //pthread_t tid1;
    pthread_t tid2;
    //pthread_create(&tid1, NULL, rx_tx_thread, NULL);
    pthread_create(&tid2, NULL, uart_thread, NULL);
    //printf("Joining thread 1...\n");
    //pthread_join(tid1, NULL);
    //printf("Joining thread 2...\n");
    pthread_join(tid2, NULL);
    //printf("Running...\n");
    for (;;) {
	//printf("%s\n", rxbuf);
	rx_data();
	printf("%s", rxbuf);
	tx_data();
    }
    return 0;
}
#endif
