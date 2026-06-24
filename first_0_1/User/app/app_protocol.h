#ifndef __APP_PROTOCOL_H
#define __APP_PROTOCOL_H

#include "stm32f10x.h"

#define PROTO_BUF_SIZE   64

typedef enum
{
    PKT_TRASH = 0,
    PKT_HEARTBEAT,
    PKT_ACK,
    PKT_STATUS,
    PKT_UNKNOWN
} PktType_t;

typedef struct
{
    PktType_t type;
    int16_t   x;
    int16_t   y;
    uint8_t   raw[PROTO_BUF_SIZE];
    uint8_t   len;
} K230Packet_t;

void App_Protocol_Init(void);
void App_Protocol_ParseByte(uint8_t ch);
uint8_t App_Protocol_PacketReady(void);
K230Packet_t *App_Protocol_GetPacket(void);
void App_Protocol_SendAck(void);
void App_Protocol_SendStatus(const char *state, float pitch, float roll,
                             float front, float left, float right);
void App_Protocol_SendString(const char *str);

#endif
