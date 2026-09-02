#ifndef __APP_NAVIGATION_H
#define __APP_NAVIGATION_H

#include "stm32f10x.h"

/* Navigation waypoint (for future GPS expansion) */
typedef struct
{
    float lat;
    float lng;
} NavWaypoint_t;

/* Compass interface placeholder */
typedef struct
{
    float heading;
    uint8_t valid;
} NavCompass_t;

void App_Nav_Init(void);
void App_Nav_SetHome(float lat, float lng);
void App_Nav_SetDestination(float lat, float lng);
float App_Nav_GetHeadingToTarget(float curLat, float curLng);
void App_Nav_UpdateGPS(float lat, float lng);
void App_Nav_UpdateCompass(float heading);

/* GPS receives NMEA through USART2; only checksum-verified valid RMC fixes update position. */
void App_Nav_GPS_FeedByte(uint8_t ch);
void App_Nav_GPS_Parse(const char *nmea);
uint8_t App_Nav_GPS_IsFixValid(void);
uint32_t App_Nav_GPS_GetAgeMs(void);

/* Raw QMC5883L data needs installation calibration before it is a true heading. */
uint8_t App_Nav_QMC5883_Read(int16_t *mag);

#endif
