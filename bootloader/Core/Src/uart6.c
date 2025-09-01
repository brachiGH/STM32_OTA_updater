/*
 * uart6.c
 *
 *  Created on: Aug 28, 2025
 *      Author: benhih
 */


#include "uart6.h"

#include "main.h"
#include "stdio.h"

#define GPIOCEN (1U << 2)
#define UART6EN (1U << 5)

#define CR1_RE (1U << 2)
#define CR1_TE (1U << 3)
#define CR1_UE (1U << 13)
#define CR1_PCE (1U << 10)
#define CR1_M (1U << 12)
#define SR_RXNE (1U << 5)
#define SR_TXE (1U << 7)

#define DEFAULT_SYS_FREQ 16000000u
#define APB2_CLK DEFAULT_SYS_FREQ
#define UART_BAUDRATE 115200u

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate);
static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate);

void uart6_rxtx_init(void)
{
  /**************Configure uart gpio pin***************/
  /*Enable clock access to gpioc */
  RCC->AHB1ENR |= GPIOCEN;

  /*Set PC6 mode to alternate function mode*/
  GPIOC->MODER &= ~(1U << 12);
  GPIOC->MODER |= (1U << 13);

  /*Set PC7 mode to alternate function mode*/
  GPIOC->MODER &= ~(1U << 14);
  GPIOC->MODER |= (1U << 15);

  /*Set PC6 alternate function type to UART_RX (AF08) */
  GPIOC->AFR[0] &= ~(1 << 24);
  GPIOC->AFR[0] &= ~(1 << 25);
  GPIOC->AFR[0] &= ~(1 << 26);
  GPIOC->AFR[0] |= (1 << 27);

  /*Set PC7 alternate function type to UART_TX (AF08)*/
  GPIOC->AFR[0] &= ~(1U << 28);
  GPIOC->AFR[0] &= ~(1U << 29);
  GPIOC->AFR[0] &= ~(1U << 30);
  GPIOC->AFR[0] |= (1U << 31);

  /**************Configure uart module***************/
  /*Enable clock access to uart6 */
  RCC->APB2ENR |= UART6EN;

  /*Configure baudrate*/
  uart_set_baudrate(USART6, APB2_CLK, UART_BAUDRATE);

  /*Configure the transfer direction*/
  USART6->CR1 |= (CR1_TE | CR1_RE | CR1_M | CR1_PCE);

  /*Enable uart module*/
  USART6->CR1 |= CR1_UE;
}

char uart6_read()
{
  /*Make sure the receive data register is not empty*/
  while (!(USART6->SR & SR_RXNE))
  {
  }
  /*Read data*/
  return USART6->DR;
}

void uart6_write(int ch)
{
  /*Make sure the transmit data register is empty*/
  while (!(USART6->SR & SR_TXE))
  {
  }

  /*Write to transmit data register*/
  USART6->DR = (ch & 0xFF);
}

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate)
{
  USARTx->BRR = compute_uart_bd(PeriphClk, BaudRate);
}

static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate)
{
  return (PeriphClk + (BaudRate / 2U)) / BaudRate;
}
