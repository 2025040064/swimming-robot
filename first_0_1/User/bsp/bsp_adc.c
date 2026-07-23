/**
 * Battery ADC module: single-shot conversion on PA8 (ADC12_IN8).
 * Uses ADC1 in independent mode, software-triggered, no DMA.
 * Each call to BSP_ADC_GetBatteryRaw() does a blocking conversion.
 * This is acceptable because we read battery only once per second.
 */

#include "bsp_adc.h"

#define BAT_ADC              ADC1
#define BAT_ADC_CLK          RCC_APB2Periph_ADC1
#define BAT_ADC_CH           ADC_Channel_8
#define BAT_ADC_GPIO_CLK     RCC_APB2Periph_GPIOA
#define BAT_ADC_PORT         GPIOA
#define BAT_ADC_PIN          GPIO_Pin_8

void BSP_ADC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    RCC_APB2PeriphClockCmd(BAT_ADC_CLK | BAT_ADC_GPIO_CLK, ENABLE);

    /* PA8: analog input */
    GPIO_InitStructure.GPIO_Pin  = BAT_ADC_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(BAT_ADC_PORT, &GPIO_InitStructure);

    /* ADC1: independent, software trigger, 12-bit, single conversion */
    ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode       = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel       = 1;
    ADC_Init(BAT_ADC, &ADC_InitStructure);

    /* Calibrate */
    ADC_Cmd(BAT_ADC, ENABLE);
    ADC_ResetCalibration(BAT_ADC);
    while (ADC_GetResetCalibrationStatus(BAT_ADC));
    ADC_StartCalibration(BAT_ADC);
    while (ADC_GetCalibrationStatus(BAT_ADC));
}

uint16_t BSP_ADC_GetBatteryRaw(void)
{
    ADC_RegularChannelConfig(BAT_ADC, BAT_ADC_CH, 1, ADC_SampleTime_55Cycles5);
    ADC_SoftwareStartConvCmd(BAT_ADC, ENABLE);
    while (!ADC_GetFlagStatus(BAT_ADC, ADC_FLAG_EOC));
    return ADC_GetConversionValue(BAT_ADC);
}

float BSP_ADC_GetBatteryVoltage(void)
{
    uint16_t raw = BSP_ADC_GetBatteryRaw();
    float vPin  = (float)raw * ADC_VREF_MV / (float)ADC_MAX_COUNTS;
    return vPin * ADC_DIVIDER_RATIO;  /* mV */
}

uint8_t BSP_ADC_GetBatteryPercent(void)
{
    float mv = BSP_ADC_GetBatteryVoltage();
    int32_t pct;

    if (mv >= (float)BAT_FULL_MV)
        return 100;
    if (mv <= (float)BAT_CRITICAL_MV)
        return 0;

    pct = (int32_t)((mv - (float)BAT_CRITICAL_MV)
          * 100.0f / (float)(BAT_FULL_MV - BAT_CRITICAL_MV));

    return (uint8_t)((pct > 100) ? 100 : (pct < 0 ? 0 : pct));
}
