#ifndef _COMMANDS_H
#define _COMMANDS_H

#include <stdint.h>

#define CMD_TX_BUF_SIZE 3

static const uint8_t __SYN = 0x34;
static const uint8_t __ACK = 0X43;
static const uint8_t __NAK = 0X23;

static const uint8_t _CMD_PING = 0x49;
static const uint8_t _CMD_GET_LASTEST_VERSION_ID = 0x50;
static const uint8_t _CMD_GET_UPDATE = 0x51;
static const uint8_t _CMD_LOG = 0x52;

#endif