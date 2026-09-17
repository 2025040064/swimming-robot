#ifndef BSP_OLED_H
#define BSP_OLED_H

#include "stm32f10x.h"

/* Assumed module: SSD1306 128x64, four-pin I2C. Controller is unconfirmed.
 * SCL=PB8, SDA=PB9, VDD=3.3V, GND=common ground.
 * The OLED shares remapped hardware I2C1 with the MPU6050. */
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
