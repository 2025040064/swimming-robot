/**
 * Battery voltage monitoring via ADC1 on PA8.
 *
 * Hardware: voltage divider R1=10k (top), R2=4.7k (bottom)
 *   V_adc = V_bat * R2 / (R1 + R2)
 *   V_bat = V_adc * (R1 + R2) / R2
 *
 * For 2S LiPo (6.0–8.4 V):
 *   - Full (8.4V) → ADC pin 2.69V → 3331 counts
 *   - Empty (6.0V) → ADC pin 1.92V → 2380 counts
 *
 * Resolution: ~2.5mV per LSB at battery level
 */

#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "stm32f10x.h"

/* Voltage divider: R1 (top, to battery+) and R2 (bottom, to GND) */
#define ADC_DIVIDER_R1      10000U
#define ADC_DIVIDER_R2      4700U
#define ADC_DIVIDER_RATIO   ((float)(ADC_DIVIDER_R1 + ADC_DIVIDER_R2) / (float)ADC_DIVIDER_R2)

/* Battery thresholds (mV) for a 2S LiPo */
#define BAT_FULL_MV          8400
#define BAT_LOW_MV           7000    /* <7.0V → low-battery warning */
#define BAT_CRITICAL_MV      6400    /* <6.4V → must return to base */

/* ADC reference: STM32 Vref+ = VDDA = 3.3V, 12-bit resolution */
#define ADC_VREF_MV          3300.0f
#define ADC_MAX_COUNTS       4095

void BSP_ADC_Init(void);
uint16_t BSP_ADC_GetBatteryRaw(void);
float BSP_ADC_GetBatteryVoltage(void);
uint8_t BSP_ADC_GetBatteryPercent(void);

#endif
