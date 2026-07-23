#include "app_navigation.h"
#include "algorithm/algo_filter.h"
#include <string.h>

static NavWaypoint_t g_home;
static NavWaypoint_t g_dest;
static NavWaypoint_t g_current;
static NavCompass_t g_compass;
static uint8_t g_homeSet = 0;

/* Fast sin approximation (radians, range [-pi, pi]) — polynomial */
static float fast_sin(float x)
{
    float x2 = x * x;
    return x * (1.0f - x2 * (0.16666667f - x2 * 0.00833333f));
}

/* Fast cos via sin */
static float fast_cos(float x)
{
    return fast_sin(1.57079633f - x);
}

void App_Nav_Init(void)
{
    memset(&g_home, 0, sizeof(g_home));
    memset(&g_dest, 0, sizeof(g_dest));
    memset(&g_current, 0, sizeof(g_current));
    g_compass.heading = 0.0f;
    g_compass.valid = 0;
    g_homeSet = 0;
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
    float dLng, y, x;
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
}

void App_Nav_UpdateCompass(float heading)
{
    g_compass.heading = heading;
    g_compass.valid = 1;
}

void App_Nav_GPS_Parse(const char *nmea)
{
    (void)nmea;
}

void App_Nav_QMC5883_Read(int16_t *mag)
{
    (void)mag;
}
