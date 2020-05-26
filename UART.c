    #ifndef __UART_C__
    #define __UART_C__
    #include <pthread.h>
    #include "UART.h"
    int uart_fd = -1;
    static pthread_mutex_t tx_mtx = PTHREAD_MUTEX_INITIALIZER;
    static pthread_mutex_t rx_mtx = PTHREAD_MUTEX_INITIALIZER;
    static pthread_cond_t  tx_cv  = PTHREAD_COND_INITIALIZER;
    static pthread_cond_t  rx_cv  = PTHREAD_COND_INITIALIZER;

    volatile int rx_en = 0;
    volatile int tx_en = 0;
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
	set_tx_ack();
	write(uart_fd, str, strlen(str));
	return;
    }
    void uart_put(void* data, size_t size)
    {
	set_tx_ack();
	write(uart_fd, data, size);
    }
    ssize_t uart_read(void* buf, size_t length)
    {
	set_rx_ack();
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
    int set_rx_ack()
    {
	pthread_mutex_lock(&rx_mtx);
	rx_en = 1;
	pthread_cond_signal(&rx_cv);
	pthread_mutex_unlock(&rx_mtx);
	return 0;
    }
    int set_tx_ack()
    {
	pthread_mutex_lock(&tx_mtx);
	tx_en = 1;
	pthread_cond_signal (&tx_cv);
	pthread_mutex_unlock(&tx_mtx);
	return 0;
    }

    int8_t rxbuf[__RX_BUFFER_SIZE];
    static void *rx_thread(void *arg) {
	//while (!rx_en)
	//	pthread_cond_wait(&rx_cv, &rx_mtx);
	//bzero(rxbuf, __RX_BUFFER_SIZE);
	uart_read(rxbuf, __RX_BUFFER_SIZE);
	rx_en = 0;
	//if (rxbuf != NULL && *rxbuf != '\0' && *rxbuf != '\n') {
	//	printf("%c", *rxbuf);
	//}
	printf("Recieved: %s\n", rxbuf);
	return NULL;
    }

    int8_t txbuf[__TX_BUFFER_SIZE];
    static void *tx_thread(void *arg) {
	pthread_mutex_lock(&tx_mtx);
	while (!tx_en)
	    pthread_cond_wait(&tx_cv, &tx_mtx);
	bzero(txbuf, __TX_BUFFER_SIZE);
	int buf_len = 0;
	while ((txbuf[buf_len++] = getchar()) != '\n' && buf_len < __TX_BUFFER_SIZE);
	uart_puts((const int8_t*) txbuf);
	tx_en = 0;
	printf("Sent: %s\n", txbuf);
	pthread_mutex_unlock(&tx_mtx);
	return NULL;
    }
    static void *uart_thread(void *arg)
    {
	while (1) {
	    set_rx_ack();
	    while (!rx_en)
	      pthread_cond_wait(&rx_cv, &rx_mtx);
	    if (strlen(rxbuf) != 0) {
		pthread_cond_signal(&rx_cv);
	    }
	    //bzero(rxbuf, __RX_BUFFER_SIZE);

	//	uart_read(rxbuf, __RX_BUFFER_SIZE);
	//	rx_en = 0;
	//	pthread_mutex_unlock(&rx_mtx);
	//	break;
	//}
	//while (1) {
	    //set_tx_ack();
	    //set_rx_ack();
	    //pthread_mutex_lock(&rx_mtx);
	    //if (1) {
	    //}
	       // pthread_mutex_unlock(&rx_mtx);
	    //pthread_mutex_lock(&tx_mtx);
	    //if (strlen(txbuf) != 0) {
	    //    pthread_cond_signal(&tx_cv);
	    //}
	    //pthread_mutex_unlock(&tx_mtx);
	}
	return NULL;
    }


    int main()
    {
	if (!uart_init(dev_name, B57600, __READ_BLOCK)) {
	    return -1;
	}
	pthread_t tid1;
	//pthread_t tid2;
	pthread_t tid3;
	pthread_create(&tid1, NULL, rx_thread, NULL);
	//pthread_create(&tid2, NULL, tx_thread, NULL);
	pthread_create(&tid3, NULL, uart_thread, NULL);
	printf("Joining thread 1...\n");
	pthread_join(tid1, NULL);
	//pthread_join(tid2, NULL);
	printf("Joining thread 3...\n");
	pthread_join(tid3, NULL);
	printf("Running...\n");
	//while (1) {
	//	printf("Sent: %s\n", txbuf);
	//	printf("Recieved: %s\n", rxbuf);
	//}
	//int8_t rxbuf[__RX_BUFFER_SIZE];
	//int8_t txbuf[__TX_BUFFER_SIZE];
	int buf_len = 0;
	for (;;) {
	    //bzero(rxbuf, __RX_BUFFER_SIZE);
	    //bzero(txbuf, __TX_BUFFER_SIZE);
	    //uart_read(rxbuf, __RX_BUFFER_SIZE);
	    printf("%s\n", rxbuf);
	    //while ((txbuf[buf_len++] = getchar()) != '\n' && buf_len < __TX_BUFFER_SIZE);
	    //uart_puts((const int8_t*) txbuf);
	    //usleep(1000 * 1000);
	}
	return 0;
    }
#endif
