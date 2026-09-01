/**
 * Battery ADC module — DISABLED (needs hardware rework).
 *
 * The original code configured PA8 as ADC input with ADC_Channel_8.
 * On STM32F103C8T6, PA8 has no ADC function: ADC_Channel_8 maps to PB0,
 * which is already used by the conveyor motor direction pin (TB2_BIN1).
 * The "battery voltage" was therefore reading a motor control level.
 *
 * Every ADC-capable pin (PA0~PA7, PB0, PB1) is currently occupied, so there
 * is no free ADC pin without rewiring. Until the hardware is changed, all
 * functions are no-ops returning safe sentinels.
 *
 * To restore battery monitoring later:
 *   - Move a GPIO function off one of PA0~PA7/PB0/PB1 (e.g. TB1_STBY from
 *     PA4 to PA8), then
 *   - Wire the divider midpoint to that pin (e.g. PA4 = ADC1_IN4) and
 *   - Re-enable the ADC config below with the correct channel.
 */

#include "bsp_adc.h"

void BSP_ADC_Init(void)
{
    /* Disabled: no valid ADC pin available without hardware rework. */
}

uint16_t BSP_ADC_GetBatteryRaw(void)
{
    return 0;
}

float BSP_ADC_GetBatteryVoltage(void)
{
    return 0.0f;   /* mV */
}

uint8_t BSP_ADC_GetBatteryPercent(void)
{
    return 0;
}
