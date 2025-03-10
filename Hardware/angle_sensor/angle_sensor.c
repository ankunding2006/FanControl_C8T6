#include "angle_sensor.h"
#include "math.h"
#include <stddef.h>
#include "delay.h"

/* 私有宏定义 */
#define ADC_MIN 820  // ADC最小值
#define ADC_MID 2420 // ADC中间值（0度位置）
#define ADC_MAX 4020 // ADC最大值

/**
 * @brief  角度传感器初始化
 * @retval AngleSensorStatus_TypeDef 初始化状态
 */
AngleSensorStatus_TypeDef ANGLE_SENSOR_Init(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
    ADC_Cmd(ADC1, ENABLE);

    // ADC??
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1))
        ;
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1))
        ;

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    return ANGLE_SENSOR_OK;
}

/**
 * @brief  获取角度值
 * @retval float: 当前角度值(度)
 */
float ANGLE_SENSOR_GetAngle(void)
{
    static uint16_t adc_buffer[8];
    uint32_t sum = 0;

    for (int i = 0; i < 8; i++)
    {
        while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
            ;
        adc_buffer[i] = ADC_GetConversionValue(ADC1);
        sum += adc_buffer[i];
    }

    uint16_t avg_adc = sum / 8;

    float actual_angle;
    if (avg_adc <= ADC_MID)
    {

        actual_angle = (avg_adc - ADC_MID) * 90.0f / (ADC_MID - ADC_MIN);
    }
    else
    {

        actual_angle = (avg_adc - ADC_MID) * 90.0f / (ADC_MAX - ADC_MID);
    }

    if (actual_angle > 90.0f)
        actual_angle = 90.0f;
    if (actual_angle < -90.0f)
        actual_angle = -90.0f;

    if (fabs(actual_angle) < 0.5f)
        actual_angle = 0.0f;

    return actual_angle * 1.5 + 90;
}
