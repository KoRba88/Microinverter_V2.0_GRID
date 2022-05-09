/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_opamp.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STATUS1_Pin GPIO_PIN_13
#define STATUS1_GPIO_Port GPIOC
#define VAC_BF_RELAY_Pin GPIO_PIN_0
#define VAC_BF_RELAY_GPIO_Port GPIOF
#define VAC_ADC1_IN1_Pin GPIO_PIN_0
#define VAC_ADC1_IN1_GPIO_Port GPIOA
#define VAC_ADC1_IN2_Pin GPIO_PIN_1
#define VAC_ADC1_IN2_GPIO_Port GPIOA
#define HVDC_ADC1_IN3_Pin GPIO_PIN_2
#define HVDC_ADC1_IN3_GPIO_Port GPIOA
#define HVDC_ADC1_IN4_Pin GPIO_PIN_3
#define HVDC_ADC1_IN4_GPIO_Port GPIOA
#define IAC_PGA2_VOUT_Pin GPIO_PIN_6
#define IAC_PGA2_VOUT_GPIO_Port GPIOA
#define IAC_ADC2_IN4_Pin GPIO_PIN_7
#define IAC_ADC2_IN4_GPIO_Port GPIOA
#define IAC_ADC2_IN5_Pin GPIO_PIN_4
#define IAC_ADC2_IN5_GPIO_Port GPIOC
#define IAC_PGA2_VIN_COMP4_INP_Pin GPIO_PIN_0
#define IAC_PGA2_VIN_COMP4_INP_GPIO_Port GPIOB
#define IAC_COMP1_INP_Pin GPIO_PIN_1
#define IAC_COMP1_INP_GPIO_Port GPIOB
#define SHUNT_NTC_ADC1_IN11_Pin GPIO_PIN_12
#define SHUNT_NTC_ADC1_IN11_GPIO_Port GPIOB
#define TMON_SHUNT_NTC_Pin GPIO_PIN_13
#define TMON_SHUNT_NTC_GPIO_Port GPIOB
#define IAC_REF_Pin GPIO_PIN_14
#define IAC_REF_GPIO_Port GPIOB
#define VINMON_Pin GPIO_PIN_15
#define VINMON_GPIO_Port GPIOB
#define OC_OUT_Pin GPIO_PIN_9
#define OC_OUT_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define BB(reg) ((uint32_t *)(PERIPH_BB_BASE + ((uint32_t)&(reg) - PERIPH_BASE) * 32U))

#define DMA1_CLEAR_IT (BB(DMA1->IFCR)[1])

#define SPI_RX_BUFFER_SIZE 3
#define SPI_TX_BUFFER_SIZE 3
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
