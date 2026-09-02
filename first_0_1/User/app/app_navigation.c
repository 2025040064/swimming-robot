#include "app_navigation.h"
#include "algorithm/algo_filter.h"
#include "bsp_qmc5883l.h"
#include "bsp_systick.h"
#include <string.h>

#define NAV_GPS_LINE_MAX       96U
#define NAV_GPS_FIX_TIMEOUT_MS 3000UL

typedef struct
{
    const char *ptr;
    uint8_t len;
} NavNMEAField_t;

static NavWaypoint_t g_home;
static NavWaypoint_t g_dest;
static NavWaypoint_t g_current;
static NavCompass_t g_compass;
static uint8_t g_homeSet = 0;

static char g_gpsLine[NAV_GPS_LINE_MAX];
static uint8_t g_gpsLineLen = 0;
static uint8_t g_gpsFixValid = 0;
static uint8_t g_gpsFixSeen = 0;
static uint32_t g_lastFixTick = 0;

/* Fast sin approximation (radians, range [-pi, pi]) - polynomial. */
static float fast_sin(float x)
{
    float x2 = x * x;
    return x * (1.0f - x2 * (0.16666667f - x2 * 0.00833333f));
}

/* Fast cos via sin. */
static float fast_cos(float x)
{
    return fast_sin(1.57079633f - x);
}

static uint8_t Nav_HexNibble(char ch)
{
    if ((ch >= '0') && (ch <= '9')) return (uint8_t)(ch - '0');
    if ((ch >= 'A') && (ch <= 'F')) return (uint8_t)(ch - 'A' + 10);
    if ((ch >= 'a') && (ch <= 'f')) return (uint8_t)(ch - 'a' + 10);
    return 0xFFU;
}

static uint8_t Nav_ChecksumValid(const char *nmea)
{
    const char *p;
    uint8_t checksum = 0;
    uint8_t high;
    uint8_t low;

    if ((nmea == 0) || (nmea[0] != '$'))
        return 0;

    p = nmea + 1;
    while ((*p != '\0') && (*p != '*'))
    {
        checksum ^= (uint8_t)*p;
        p++;
    }
    if ((*p != '*') || (p[1] == '\0') || (p[2] == '\0') || (p[3] != '\0'))
        return 0;

    high = Nav_HexNibble(p[1]);
    low = Nav_HexNibble(p[2]);
    if ((high == 0xFFU) || (low == 0xFFU))
        return 0;
    return (checksum == (uint8_t)((high << 4) | low)) ? 1U : 0U;
}

static uint8_t Nav_SplitFields(const char *nmea, NavNMEAField_t *fields, uint8_t maxFields)
{
    const char *p;
    const char *start;
    uint8_t count = 0;

    if ((nmea == 0) || (fields == 0) || (maxFields == 0U))
        return 0;

    start = nmea + 1;
    p = start;
    while ((*p != '\0') && (*p != '*'))
    {
        if (*p == ',')
        {
            if (count >= maxFields)
                return count;
            fields[count].ptr = start;
            fields[count].len = (uint8_t)(p - start);
            count++;
            start = p + 1;
        }
        p++;
    }

    if (count < maxFields)
    {
        fields[count].ptr = start;
        fields[count].len = (uint8_t)(p - start);
        count++;
    }
    return count;
}

static uint8_t Nav_ParseCoordinate(const NavNMEAField_t *field, float *coordinate)
{
    float raw = 0.0f;
    float fraction = 0.1f;
    float minutes;
    uint16_t degrees;
    uint8_t i;
    uint8_t decimalSeen = 0;
    uint8_t digitCount = 0;

    if ((field == 0) || (coordinate == 0) || (field->len < 4U))
        return 0;

    for (i = 0; i < field->len; i++)
    {
        char ch = field->ptr[i];
        if ((ch >= '0') && (ch <= '9'))
        {
            if (decimalSeen)
            {
                raw += (float)(ch - '0') * fraction;
                fraction *= 0.1f;
            }
            else
            {
                raw = raw * 10.0f + (float)(ch - '0');
            }
            digitCount++;
        }
        else if ((ch == '.') && (decimalSeen == 0U))
        {
            decimalSeen = 1;
        }
        else
        {
            return 0;
        }
    }

    if (digitCount < 4U)
        return 0;
    degrees = (uint16_t)(raw / 100.0f);
    minutes = raw - (float)degrees * 100.0f;
    if (minutes >= 60.0f)
        return 0;

    *coordinate = (float)degrees + minutes / 60.0f;
    return 1;
}

static uint8_t Nav_IsRMC(const NavNMEAField_t *field)
{
    if ((field == 0) || (field->len != 5U))
        return 0;
    return (field->ptr[2] == 'R') && (field->ptr[3] == 'M') &&
           (field->ptr[4] == 'C');
}

void App_Nav_Init(void)
{
    memset(&g_home, 0, sizeof(g_home));
    memset(&g_dest, 0, sizeof(g_dest));
    memset(&g_current, 0, sizeof(g_current));
    g_compass.heading = 0.0f;
    g_compass.valid = 0;
    g_homeSet = 0;
    g_gpsLineLen = 0;
    g_gpsFixValid = 0;
    g_gpsFixSeen = 0;
    g_lastFixTick = 0;
}

void App_Nav_SetHome(float lat, float lng)
{
    g_home.lat = lat;
    g_home.lng = lng;
    g_homeSet = 1;
}

void App_Nav_SetDestination(float lat, float lng)
{
    g_dest.lat = lat;
    g_dest.lng = lng;
}

float App_Nav_GetHeadingToTarget(float curLat, float curLng)
{
    float dLng;
    float y;
    float x;
    float rad = 0.0174533f;
    float toRad = 57.29578f;
    float heading;

    dLng = (g_dest.lng - curLng) * rad;
    y = fast_sin(dLng) * fast_cos(g_dest.lat * rad);
    x = fast_cos(curLat * rad) * fast_sin(g_dest.lat * rad)
      - fast_sin(curLat * rad) * fast_cos(g_dest.lat * rad) * fast_cos(dLng);

    heading = fast_atan2(y, x) * toRad;
    if (heading < 0.0f) heading += 360.0f;
    return heading;
}

void App_Nav_UpdateGPS(float lat, float lng)
{
    g_current.lat = lat;
    g_current.lng = lng;
    g_gpsFixValid = 1;
    g_gpsFixSeen = 1;
    g_lastFixTick = BSP_GetTick();
}

void App_Nav_UpdateCompass(float heading)
{
    g_compass.heading = heading;
    g_compass.valid = 1;
}

void App_Nav_GPS_Parse(const char *nmea)
{
    NavNMEAField_t fields[7];
    float lat;
    float lng;
    uint8_t count;

    if (!Nav_ChecksumValid(nmea))
        return;

    count = Nav_SplitFields(nmea, fields, 7);
    if ((count < 7U) || !Nav_IsRMC(&fields[0]))
        return;

    if ((fields[2].len != 1U) || (fields[2].ptr[0] != 'A'))
    {
        g_gpsFixValid = 0;
        return;
    }
    if (!Nav_ParseCoordinate(&fields[3], &lat) || !Nav_ParseCoordinate(&fields[5], &lng))
        return;
    if ((lat > 90.0f) || (lng > 180.0f))
        return;
    if ((fields[4].len != 1U) || (fields[6].len != 1U))
        return;

    if (fields[4].ptr[0] == 'S') lat = -lat;
    else if (fields[4].ptr[0] != 'N') return;
    if (fields[6].ptr[0] == 'W') lng = -lng;
    else if (fields[6].ptr[0] != 'E') return;

    App_Nav_UpdateGPS(lat, lng);
}

void App_Nav_GPS_FeedByte(uint8_t ch)
{
    if (ch == '$')
    {
        g_gpsLineLen = 0;
        g_gpsLine[g_gpsLineLen++] = (char)ch;
        return;
    }

    if (g_gpsLineLen == 0U)
        return;
    if (ch == '\r')
        return;
    if (ch == '\n')
    {
        g_gpsLine[g_gpsLineLen] = '\0';
        App_Nav_GPS_Parse(g_gpsLine);
        g_gpsLineLen = 0;
        return;
    }
    if (g_gpsLineLen >= (NAV_GPS_LINE_MAX - 1U))
    {
        g_gpsLineLen = 0;
        return;
    }

    g_gpsLine[g_gpsLineLen++] = (char)ch;
}

uint8_t App_Nav_GPS_IsFixValid(void)
{
    if ((g_gpsFixValid == 0U) || (g_gpsFixSeen == 0U))
        return 0;
    if ((BSP_GetTick() - g_lastFixTick) >= NAV_GPS_FIX_TIMEOUT_MS)
        return 0;
    return 1;
}

uint32_t App_Nav_GPS_GetAgeMs(void)
{
    if (g_gpsFixSeen == 0U)
        return 0xFFFFFFFFUL;
    return BSP_GetTick() - g_lastFixTick;
}

uint8_t App_Nav_QMC5883_Read(int16_t *mag)
{
    uint8_t result;

    if (mag == 0)
        return QMC5883L_ERR_PARAM;

    result = BSP_QMC5883L_ReadRaw(mag);
    if (result != QMC5883L_OK)
    {
        mag[0] = 0;
        mag[1] = 0;
        mag[2] = 0;
        g_compass.valid = 0;
    }
    return result;
}
