#include "app_protocol.h"
#include "bsp_usart.h"
#include <string.h>

static K230Packet_t g_packet;
static volatile uint8_t g_packetReady = 0;

static uint8_t g_rxBuf[PROTO_BUF_SIZE];
static uint8_t g_rxIdx = 0;

/* Manual TRASH,x,y parser — no sscanf dependency */
static void ParseTrash(char *str, int16_t *x, int16_t *y)
{
    char *p = str + 6; /* skip "TRASH," */
    int32_t val;
    int8_t sign;

    /* Parse x */
    sign = 1;
    if (*p == '-') { sign = -1; p++; }
    val = 0;
    while (*p >= '0' && *p <= '9') { val = val * 10 + (*p - '0'); p++; }
    *x = (int16_t)(sign * val);

    if (*p == ',') p++;

    /* Parse y */
    sign = 1;
    if (*p == '-') { sign = -1; p++; }
    val = 0;
    while (*p >= '0' && *p <= '9') { val = val * 10 + (*p - '0'); p++; }
    *y = (int16_t)(sign * val);
}

/* Lightweight float-to-int for telemetry: keep 1 decimal digit */
static int32_t FloatToInt1(float f)
{
    return (int32_t)(f * 10.0f);
}

static void BuildStatus(char *buf, const char *state, float pitch, float roll,
                        float front, float left, float right)
{
    int32_t ip, ir, if_, il, ir2;
    uint8_t idx = 0;
    const char *s;

    /* "STAT," */
    buf[idx++] = 'S'; buf[idx++] = 'T'; buf[idx++] = 'A'; buf[idx++] = 'T';
    buf[idx++] = ',';

    /* state string */
    s = state;
    while (*s) buf[idx++] = *s++;
    buf[idx++] = ',';

    /* pitch * 10 */
    ip = FloatToInt1(pitch);
    if (ip < 0) { buf[idx++] = '-'; ip = -ip; }
    buf[idx++] = '0' + (ip / 10); buf[idx++] = '.';
    buf[idx++] = '0' + (ip % 10);
    buf[idx++] = ',';

    /* roll * 10 */
    ir = FloatToInt1(roll);
    if (ir < 0) { buf[idx++] = '-'; ir = -ir; }
    buf[idx++] = '0' + (ir / 10); buf[idx++] = '.';
    buf[idx++] = '0' + (ir % 10);
    buf[idx++] = ',';

    /* front (integer cm) */
    if_ = (int32_t)front;
    if (if_ < 0) { buf[idx++] = '-'; if_ = -if_; }
    if (if_ >= 1000) { buf[idx++] = '9'; buf[idx++] = '9'; buf[idx++] = '9'; }
    else if (if_ >= 100) { buf[idx++] = '0' + (if_ / 100); if_ %= 100; buf[idx++] = '0' + (if_ / 10); buf[idx++] = '0' + (if_ % 10); }
    else { buf[idx++] = '0' + (if_ / 10); buf[idx++] = '0' + (if_ % 10); }
    buf[idx++] = ',';

    /* left */
    il = (int32_t)left;
    if (il < 0) { buf[idx++] = '-'; il = -il; }
    if (il >= 1000) { buf[idx++] = '9'; buf[idx++] = '9'; buf[idx++] = '9'; }
    else if (il >= 100) { buf[idx++] = '0' + (il / 100); il %= 100; buf[idx++] = '0' + (il / 10); buf[idx++] = '0' + (il % 10); }
    else { buf[idx++] = '0' + (il / 10); buf[idx++] = '0' + (il % 10); }
    buf[idx++] = ',';

    /* right */
    ir2 = (int32_t)right;
    if (ir2 < 0) { buf[idx++] = '-'; ir2 = -ir2; }
    if (ir2 >= 1000) { buf[idx++] = '9'; buf[idx++] = '9'; buf[idx++] = '9'; }
    else if (ir2 >= 100) { buf[idx++] = '0' + (ir2 / 100); ir2 %= 100; buf[idx++] = '0' + (ir2 / 10); buf[idx++] = '0' + (ir2 % 10); }
    else { buf[idx++] = '0' + (ir2 / 10); buf[idx++] = '0' + (ir2 % 10); }

    buf[idx++] = '\n';
    buf[idx] = '\0';
}

void App_Protocol_Init(void)
{
    memset(&g_packet, 0, sizeof(g_packet));
    g_packetReady = 0;
    g_rxIdx = 0;
}

void App_Protocol_ParseByte(uint8_t ch)
{
    if (ch == '\n' || ch == '\r')
    {
        if (g_rxIdx > 0)
        {
            g_rxBuf[g_rxIdx] = '\0';
            memcpy((void *)g_packet.raw, g_rxBuf, g_rxIdx + 1);
            g_packet.len = g_rxIdx;
            g_rxIdx = 0;

            if (strncmp((char *)g_packet.raw, "TRASH,", 6) == 0)
            {
                ParseTrash((char *)g_packet.raw, &g_packet.x, &g_packet.y);

                /* Validate coordinates: the K230 frame is 640x640, so a valid
                 * target must be within [0,639] on both axes. Reject out-of-range
                 * / malformed frames instead of generating a bogus target. */
                if (g_packet.x >= 0 && g_packet.x <= PROTO_IMG_MAX_X &&
                    g_packet.y >= 0 && g_packet.y <= PROTO_IMG_MAX_Y)
                {
                    g_packet.type = PKT_TRASH;
                    g_packetReady = 1;
                }
            }
            else if (strncmp((char *)g_packet.raw, "HB", 2) == 0)
            {
                g_packet.type = PKT_HEARTBEAT;
                g_packetReady = 1;
            }
            else if (strncmp((char *)g_packet.raw, "ACK", 3) == 0)
            {
                g_packet.type = PKT_ACK;
                g_packetReady = 1;
            }
        }
    }
    else
    {
        if (g_rxIdx < PROTO_BUF_SIZE - 1)
        {
            g_rxBuf[g_rxIdx++] = ch;
        }
    }
}

uint8_t App_Protocol_PacketReady(void)
{
    return g_packetReady;
}

K230Packet_t *App_Protocol_GetPacket(void)
{
    g_packetReady = 0;
    return &g_packet;
}

void App_Protocol_SendAck(void)
{
    BSP_USART_SendString("ACK\n");
}

void App_Protocol_SendStatus(const char *state, float pitch, float roll,
                             float front, float left, float right)
{
    char buf[80];
    BuildStatus(buf, state, pitch, roll, front, left, right);
    BSP_USART_SendString(buf);
}

void App_Protocol_SendString(const char *str)
{
    BSP_USART_SendString((char *)str);
}
