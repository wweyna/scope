/*
 * vdda.c
 *
 *  Created on: Nov 11, 2024
 *      Author: Wojciech Weyna
 */


#include "main.h"
#include "vdda.h"

#include <assert.h>
#include <math.h>

extern ADC_HandleTypeDef hadc3;

uint16_t vdda_vrefint_value;
uint16_t vdda_voltage;

uint16_t VDDA_GetVoltage()
{
    if (vdda_voltage == 0)
    {
        HAL_ADCEx_Calibration_Start (&hadc3, ADC_SINGLE_ENDED);

        // Safe delay between ADC end of calibration and ADC enable.
        int wait_loop_index = LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 100;
        while(wait_loop_index != 0)
        {
          wait_loop_index--;
        }

        HAL_ADC_Start (&hadc3);
        HAL_ADC_PollForConversion (&hadc3, 0xFFFF);
        vdda_vrefint_value = HAL_ADC_GetValue (&hadc3);
        vdda_voltage = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(vdda_vrefint_value, ADC_RESOLUTION_12B);
        HAL_ADC_Stop (&hadc3);
    }
    return vdda_voltage;
}
