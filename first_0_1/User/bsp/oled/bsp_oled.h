#ifndef BSP_OLED_H
#define BSP_OLED_H

#include "stm32f10x.h"

/* Assumed module: SSD1306 128x64, four-pin I2C. Controller is unconfirmed.
 * SCK=PB10, SDA=PB11, VDD=3.3V, GND=common ground.
 * These pins no longer carry recovery motor PWM. */
#define OLED_GPIO_PORT GPIOB
#define OLED_GPIO_CLK  RCC_APB2Periph_GPIOB
#define OLED_SCK_PIN   GPIO_Pin_10
#define OLED_SDA_PIN   GPIO_Pin_11
#define OLED_I2C_DELAY_US 10U
#define OLED_RETRY_DELAY_US 50U
#define OLED_COLUMN_OFFSET 0U
#define OLED_FRAME_MS 250U
#define OLED_TEXT_COLUMNS 21U

uint8_t BSP_OLED_Init(void);
uint8_t BSP_OLED_IsPresent(void);
uint8_t BSP_OLED_IsBusy(void);
void BSP_OLED_Clear(void);
void BSP_OLED_Text(uint8_t row, const char *text);
void BSP_OLED_Present(void);
void BSP_OLED_Task(void);

#endif
