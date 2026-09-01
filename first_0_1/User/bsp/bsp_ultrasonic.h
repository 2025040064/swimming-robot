#ifndef __BSP_ULTRASONIC_H
#define __BSP_ULTRASONIC_H

#include "stm32f10x.h"

#define US_FRONT    0
#define US_LEFT     1
#define US_RIGHT    2

/* Distance limits (cm). 999.0f means "no echo / timeout" — NOT "clear ahead". */
#define US_MIN_DIST_CM       2.0f
#define US_MAX_DIST_CM       400.0f
#define US_NO_ECHO           999.0f

/* A reading older than this is considered invalid / stale. */
#define US_STALE_MS          2000

/* Round-robin trigger interval (ms) and per-sensor echo timeout (ms).
 * Max echo ~400cm => ~23.5ms round trip, so 30ms is a safe timeout. */
#define US_TRIG_INTERVAL_MS  60
#define US_ECHO_TIMEOUT_MS   30

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

/* Right: Trig=PB7, Echo=PB8 (EXTI8) */
#define US_R_TRIG_PORT   GPIOB
#define US_R_TRIG_PIN    GPIO_Pin_7
#define US_R_ECHO_PORT   GPIOB
#define US_R_ECHO_PIN    GPIO_Pin_8

void BSP_Ultrasonic_Init(void);
void BSP_Ultrasonic_Update(void);
float BSP_Ultrasonic_GetDistance(uint8_t sensor);
float BSP_Ultrasonic_GetFront(void);
float BSP_Ultrasonic_GetLeft(void);
float BSP_Ultrasonic_GetRight(void);

/* 1 = the sensor has a fresh (non-stale) reading; 0 = no echo / timed out / stale */
uint8_t  BSP_Ultrasonic_IsValid(uint8_t sensor);
/* Milliseconds since the last successful echo (0xFFFFFFFF if never measured) */
uint32_t BSP_Ultrasonic_GetAgeMs(uint8_t sensor);

#endif
