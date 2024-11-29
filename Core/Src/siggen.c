/*
 * siggen.c
 *
 *  Created on: Nov 11, 2024
 *      Author: Wojciech Weyna
 */


#include "main.h"

#include "scope.h"
#include "vdda.h"

#include <assert.h>
#include <math.h>

#include "stm32g4xx_ll_dac.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_gpio.h"

extern DAC_HandleTypeDef hdac1;
extern DMA_HandleTypeDef hdma_dac1_ch1;
extern TIM_HandleTypeDef htim6;

extern DAC_HandleTypeDef hdac2;
extern DMA_HandleTypeDef hdma_dac2_ch1;
extern TIM_HandleTypeDef htim7;

uint32_t siggen_vdda_voltage;

#define PI 3.141592
#define MAX_DAC_VALUE (4095)   // 0xFFF, maximum 12 bit value

#define MAX_WAVEFROM_SAMPLES_DAC1  1000
uint16_t waveform_samples_DAC1[MAX_WAVEFROM_SAMPLES_DAC1];

#define MAX_WAVEFROM_SAMPLES_DAC2  200
uint16_t waveform_samples_DAC2[MAX_WAVEFROM_SAMPLES_DAC2];

void generate_wave(uint16_t* waveform_samples, uint32_t samples_count)
{
    const double max_amplitude = 2.0;       // amplitude of sin wave
    const double amplitude_factor = 0.93;   // this will limit the amplitude
    const double wave_to_dac = MAX_DAC_VALUE / max_amplitude;  // sin() to 16 bit DAC converting factor

    for (int i = 0; i < samples_count; ++i)
    {
        const double y = sin(i * 2 * PI / samples_count) * amplitude_factor + 1;
        const uint16_t dac_value = round(y * wave_to_dac);
        waveform_samples[i] = dac_value;
    }
}

void generate_wave_DAC1()
{
    generate_wave(waveform_samples_DAC1, MAX_WAVEFROM_SAMPLES_DAC1);
}

void generate_wave_DAC2()
{
    generate_wave(waveform_samples_DAC2, MAX_WAVEFROM_SAMPLES_DAC2);
}

uint32_t timer_prescaler;                   /* Time base prescaler to have timebase aligned on minimum frequency possible */
uint32_t timer_reload;                      /* Timer reload value in function of timer prescaler to achieve time base period */

void Activate_DAC(void);
void TIM_PrescalerReloadCalculation(void);

void SigGen_Sinus_DAC1()
{
    siggen_vdda_voltage = VDDA_GetVoltage();

    DAC_ChannelConfTypeDef dacConfig;
    HAL_StatusTypeDef dacCalibrationStatus = HAL_DACEx_SelfCalibrate(&hdac1, &dacConfig, DAC_CHANNEL_1);
    UNUSED(dacConfig);
    UNUSED(dacCalibrationStatus);

//
//    HAL_StatusTypeDef dacTrimmingStatus = HAL_DACEx_SetUserTrimming(&hdac1, &dacConfig, DAC_CHANNEL_1, 31);
//    UNUSED(dacTrimmingStatus);

    generate_wave_DAC1();
    HAL_TIM_Base_Start(&htim6);
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)waveform_samples_DAC1, MAX_WAVEFROM_SAMPLES_DAC1, DAC_ALIGN_12B_R);
}

void SigGen_Sinus_DAC2()
{
    siggen_vdda_voltage = VDDA_GetVoltage();

    DAC_ChannelConfTypeDef dacConfig;
    HAL_StatusTypeDef dacCalibrationStatus = HAL_DACEx_SelfCalibrate(&hdac2, &dacConfig, DAC_CHANNEL_1);
    UNUSED(dacConfig);
    UNUSED(dacCalibrationStatus);

//
//    HAL_StatusTypeDef dacTrimmingStatus = HAL_DACEx_SetUserTrimming(&hdac1, &dacConfig, DAC_CHANNEL_1, 31);
//    UNUSED(dacTrimmingStatus);

    generate_wave_DAC2();
    HAL_TIM_Base_Start(&htim7);
    HAL_DAC_Start_DMA(&hdac2, DAC_CHANNEL_1, (uint32_t*)waveform_samples_DAC2, MAX_WAVEFROM_SAMPLES_DAC2, DAC_ALIGN_12B_R);
}

/*
  * @brief  DMA transfer error callback
  * @note   This function is executed when the transfer error interrupt
  *         is generated during DMA transfer
  * @retval None
  */
void DacDmaTransferError_Callback()
{
  /* Error detected during DMA transfer */
  assert(0);
}

/**
  * @brief  DAC under-run interruption callback
  * @note   This function is executed when DAC channel under-run error occurs.
  * @retval None
  */
void DacUnderrunError_Callback(void)
{
  /* Note: Disable DAC interruption that caused this error before entering in */
  /*       infinite loop below.                                               */

  /* Disable interruption DAC channel1 under-run */
  LL_DAC_DisableIT_DMAUDR1(DAC1);

  /* Error from ADC */
  assert(0);
}
