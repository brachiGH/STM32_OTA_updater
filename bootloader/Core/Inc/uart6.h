/*
 * uart2.h
 *
 *  Created on: Aug 28, 2025
 *      Author: benhih
 */

#ifndef INC_UART2_H_
#define INC_UART2_H_

#define UART_MAX_SEND_BUFF_SIZE 512
#define UART_MAX_RECV_BUFF_SIZE 1024
#define UART_MAX_CMD_BUFF_SIZE 3

void uart6_rxtx_init(void);
void uart6_write(int ch);
char uart6_read();

#endif /* INC_UART2_H_ */
