#ifndef __BSP_ULTRASONIC_H
#define __BSP_ULTRASONIC_H

#include "stm32f10x.h"

#define US_FRONT    0
#define US_LEFT     1
#define US_RIGHT    2

/* Front: Trig=PA6, Echo=PA7 */
#define US_F_TRIG_PORT   GPIOA
#define US_F_TRIG_PIN    GPIO_Pin_6
#define US_F_ECHO_PORT   GPIOA
#define US_F_ECHO_PIN    GPIO_Pin_7

/* Left: Trig=PB5, Echo=PB6 */
#define US_L_TRIG_PORT   GPIOB
#define US_L_TRIG_PIN    GPIO_Pin_5
#define US_L_ECHO_PORT   GPIOB
#define US_L_ECHO_PIN    GPIO_Pin_6

/* Right: Trig=PB7, Echo=PB8 */
#define US_R_TRIG_PORT   GPIOB
#define US_R_TRIG_PIN    GPIO_Pin_7
#define US_R_ECHO_PORT   GPIOB
#define US_R_ECHO_PIN    GPIO_Pin_8

void BSP_Ultrasonic_Init(void);
float BSP_Ultrasonic_GetDistance(uint8_t sensor);
void BSP_Ultrasonic_Update(void);
float BSP_Ultrasonic_GetFront(void);
float BSP_Ultrasonic_GetLeft(void);
float BSP_Ultrasonic_GetRight(void);

#endif
