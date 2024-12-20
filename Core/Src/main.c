/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../Scope/scope.h"
#include "../Scope/siggen.h"

#define USE_FULL_ASSERT
#include <assert.h>

#include <math.h>
#include <string.h>

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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

__IO uint32_t BspButtonState = BUTTON_RELEASED;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void
SystemClock_Config (void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define CMD_RX_BUFFER_SIZE 1
uint8_t cmd_rx_buffer[CMD_RX_BUFFER_SIZE + 1];

#define SCOPE_PWM_ARR_START_VALUE 10000
#define SCOPE_PWM_FREQ_DIV 1

int arrStart = SCOPE_PWM_ARR_START_VALUE;
int freq_div = SCOPE_PWM_FREQ_DIV;

void
PWM_Next_Frequency ()
{
    freq_div *= 10;

    if (freq_div >= SCOPE_PWM_ARR_START_VALUE / 10)
    {
        arrStart = SCOPE_PWM_ARR_START_VALUE;
        freq_div = SCOPE_PWM_FREQ_DIV;
    }

    TIM1->ARR = arrStart / freq_div - 1;
    TIM1->CCR1 = arrStart / freq_div / 2;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int
main (void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init ();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config ();

    /* USER CODE BEGIN SysInit */
    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init ();
    MX_DMA_Init ();
    MX_ADC3_Init ();
    MX_ADC1_Init ();
    MX_LPUART1_UART_Init ();
    MX_TIM1_Init ();
    MX_DAC1_Init ();
    MX_TIM6_Init ();
    MX_DAC2_Init ();
    MX_TIM7_Init ();
    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* Initialize led */
    BSP_LED_Init (LED_GREEN);

    /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
    BSP_PB_Init (BUTTON_USER, BUTTON_MODE_EXTI);

    /* USER CODE BEGIN BSP */

    /* -- Sample board code to send message over COM1 port ---- */

    /* -- Sample board code to switch on led ---- */
    BSP_LED_On (LED_GREEN);

    /* USER CODE END BSP */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    // Start analog signal generators
    SigGen_Sinus_DAC1 ();
    SigGen_Sinus_DAC2 ();

    // Start digital signal generator
    HAL_TIM_PWM_Start (&htim1, TIM_CHANNEL_1);

    // Start oscilloscope - start continuous ADC sampling
    Scope_Run ();

    HAL_UART_Receive_IT (&hlpuart1, cmd_rx_buffer, CMD_RX_BUFFER_SIZE);

    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        if (BspButtonState == BUTTON_PRESSED)
        {
            BspButtonState = BUTTON_RELEASED;
            BSP_LED_Toggle (LED_GREEN);
            HAL_Delay (30);
            PWM_Next_Frequency ();
            //HAL_UART_Transmit(&hlpuart1, "Hello\r\n", 7, 10);
        }

        Scope_TransmitFrame ();

    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void
SystemClock_Config (void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct =
    { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct =
    { 0 };

    /** Configure the main internal regulator output voltage
     */
    HAL_PWREx_ControlVoltageScaling (PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN = 15;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig (&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler ();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                    | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    {
        Error_Handler ();
    }
}

/* USER CODE BEGIN 4 */
void
HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
    if (huart == &hlpuart1)
    {
        BSP_LED_Toggle (LED_GREEN);
        Scope_OnCommandReceived (cmd_rx_buffer[0]);
        memset (cmd_rx_buffer, (uint8_t) 0, CMD_RX_BUFFER_SIZE);
        HAL_UART_Receive_IT (&hlpuart1, cmd_rx_buffer, CMD_RX_BUFFER_SIZE);
    }
}
/* USER CODE END 4 */

/**
 * @brief BSP Push Button callback
 * @param Button Specifies the pressed button
 * @retval None
 */
void
BSP_PB_Callback (Button_TypeDef Button)
{
    if (Button == BUTTON_USER)
    {
        BspButtonState = BUTTON_PRESSED;
    }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void
Error_Handler (void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq ();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void
assert_failed (uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
