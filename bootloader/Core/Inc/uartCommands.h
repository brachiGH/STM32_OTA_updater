#ifndef _UART_COMMANDS_H_
#define _UART_COMMANDS_H_

static const uint8_t __SYN = 0x34;
static const uint8_t __ACK = 0x43;
static const uint8_t __NAK = 0X23;

static const uint8_t _CMD_PING = 0x49;
static const uint8_t _CMD_GET_LASTEST_VERSION_ID = 0x50;
static const uint8_t _CMD_GET_UPDATE = 0x51;
static const uint8_t _CMD_LOG = 0x52;

#endif