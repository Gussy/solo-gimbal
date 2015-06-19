#ifndef UART_H_
#define UART_H_

void setup_serial_port();
int read_serial_port(unsigned char *data, unsigned int max_size);
int send_serial_port(unsigned char *data, unsigned int size);

#endif /* UART_H_ */
