#ifndef __BSP_ULTRASONIC_H
#define __BSP_ULTRASONIC_H

#include "stm32f10x.h"

#define US_FRONT    0
#define US_LEFT     1
#define US_RIGHT    2

/* AJ-SR04M manual: 20 cm blind zone and 8 m maximum range. */
#define US_MIN_DIST_CM       20.0f
#define US_MAX_DIST_CM       800.0f
#define US_NO_ECHO           999.0f

/* AJ-SR04M manual: distance_cm = Echo_high_time_us / 58. */
#define US_ECHO_US_PER_CM    58.0f

/* AJ-SR04M mode 1: keep Trig low briefly, then high for at least 10 us. */
#define US_TRIG_LOW_US       2u
#define US_TRIG_PULSE_US     10u

/* A reading older than this is considered invalid / stale. */
#define US_STALE_MS          2000

/* 8 m takes 46.4 ms at the manual's 58 us/cm conversion. The 60 ms
 * round-robin slot keeps each 50 ms measurement window separate. */
#define US_TRIG_INTERVAL_MS  60
#define US_ECHO_TIMEOUT_MS   50

/* Front: Trig=PA6, Echo=PA7 (EXTI7) */
#define US_F_TRIG_PORT   GPIOA
#define US_F_TRIG_PIN    GPIO_Pin_6
#define US_F_ECHO_PORT   GPIOA
#define US_F_ECHO_PIN    GPIO_Pin_7

/* Left: Trig=PB5, Echo=PB6 (EXTI6) */
#define US_L_TRIG_PORT   GPIOB
#define US_L_TRIG_PIN    GPIO_Pin_5
#define US_L_ECHO_PORT   GPIOB
#define US_L_ECHO_PIN    GPIO_Pin_6

/* Right: Trig=PB3, Echo=PB4 (EXTI4). PB3/PB4 require JTAG to be disabled. */
#define US_R_TRIG_PORT   GPIOB
#define US_R_TRIG_PIN    GPIO_Pin_3
#define US_R_ECHO_PORT   GPIOB
#define US_R_ECHO_PIN    GPIO_Pin_4

void BSP_Ultrasonic_Init(void);
void BSP_Ultrasonic_Update(void);
/* Returns a current valid distance in cm, otherwise US_NO_ECHO. */
float BSP_Ultrasonic_GetDistance(uint8_t sensor);
float BSP_Ultrasonic_GetFront(void);
float BSP_Ultrasonic_GetLeft(void);
float BSP_Ultrasonic_GetRight(void);

/* 1 = the sensor has a fresh (non-stale) reading; 0 = no echo / timed out / stale */
uint8_t  BSP_Ultrasonic_IsValid(uint8_t sensor);
/* Milliseconds since the last successful echo (0xFFFFFFFF if never measured) */
uint32_t BSP_Ultrasonic_GetAgeMs(uint8_t sensor);

#endif
