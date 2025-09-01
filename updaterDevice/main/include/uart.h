#ifndef UART_H
#define UART_H

#ifdef __cplusplus
extern "C" {
#endif

void uart_init(QueueHandle_t *uart_queue);

#ifdef __cplusplus
}
#endif

#endif // UART_H