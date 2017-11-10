/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define SPI1_CLK_Pin GPIO_PIN_2
#define SPI1_CLK_GPIO_Port GPIOE
#define SPI1_MISO_Pin GPIO_PIN_5
#define SPI1_MISO_GPIO_Port GPIOE
#define SPI1_MOSI_Pin GPIO_PIN_6
#define SPI1_MOSI_GPIO_Port GPIOE
#define LASER_ENABLE_Pin GPIO_PIN_2
#define LASER_ENABLE_GPIO_Port GPIOF
#define TEMP1_Pin GPIO_PIN_3
#define TEMP1_GPIO_Port GPIOF
#define TEMP2_Pin GPIO_PIN_4
#define TEMP2_GPIO_Port GPIOF
#define SPI2_CLK_Pin GPIO_PIN_7
#define SPI2_CLK_GPIO_Port GPIOF
#define SPI2_MISO_Pin GPIO_PIN_8
#define SPI2_MISO_GPIO_Port GPIOF
#define SPI2_MOSI_Pin GPIO_PIN_9
#define SPI2_MOSI_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define VPROGRAM_BIAS_Pin GPIO_PIN_4
#define VPROGRAM_BIAS_GPIO_Port GPIOA
#define GREEN_Pin GPIO_PIN_5
#define GREEN_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define SPI1_CS_TPL_A_Pin GPIO_PIN_13
#define SPI1_CS_TPL_A_GPIO_Port GPIOF
#define TDC_INT_RX1_Pin GPIO_PIN_15
#define TDC_INT_RX1_GPIO_Port GPIOF
#define TDC_INT_RX1_EXTI_IRQn EXTI15_10_IRQn
#define SPI1_CS_TPL_B_Pin GPIO_PIN_9
#define SPI1_CS_TPL_B_GPIO_Port GPIOE
#define SPI2_CS_RX1_Pin GPIO_PIN_11
#define SPI2_CS_RX1_GPIO_Port GPIOE
#define TDC_INT_Pin GPIO_PIN_13
#define TDC_INT_GPIO_Port GPIOE
#define TDC_INT_EXTI_IRQn EXTI15_10_IRQn
#define SPI2_CS_RX2_Pin GPIO_PIN_14
#define SPI2_CS_RX2_GPIO_Port GPIOE
#define BUTTON_ENABLE_Pin GPIO_PIN_10
#define BUTTON_ENABLE_GPIO_Port GPIOB
#define BUTTON_CONFIG_Pin GPIO_PIN_11
#define BUTTON_CONFIG_GPIO_Port GPIOB
#define BUTTON_CONFIG_EXTI_IRQn EXTI15_10_IRQn
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define RED_Pin GPIO_PIN_14
#define RED_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define SPI1_CS_TX_Pin GPIO_PIN_14
#define SPI1_CS_TX_GPIO_Port GPIOD
#define SPI1_CS_RX0_Pin GPIO_PIN_15
#define SPI1_CS_RX0_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define TDC_CLK_Pin GPIO_PIN_9
#define TDC_CLK_GPIO_Port GPIOC
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define START_Pin GPIO_PIN_0
#define START_GPIO_Port GPIOD
#define CALIBRATION_Pin GPIO_PIN_1
#define CALIBRATION_GPIO_Port GPIOD
#define TPROB_CALIBRATION_ENABLE2_Pin GPIO_PIN_3
#define TPROB_CALIBRATION_ENABLE2_GPIO_Port GPIOD
#define _5K_GAIN_SETTING2_Pin GPIO_PIN_4
#define _5K_GAIN_SETTING2_GPIO_Port GPIOD
#define TPROB_CALIBRATION_ENABLE1_Pin GPIO_PIN_6
#define TPROB_CALIBRATION_ENABLE1_GPIO_Port GPIOD
#define _5K_GAIN_SETTING1_Pin GPIO_PIN_7
#define _5K_GAIN_SETTING1_GPIO_Port GPIOD
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define TDC_INT_RX2_Pin GPIO_PIN_14
#define TDC_INT_RX2_GPIO_Port GPIOG
#define TDC_INT_RX2_EXTI_IRQn EXTI15_10_IRQn
#define SW0_Pin GPIO_PIN_3
#define SW0_GPIO_Port GPIOB
#define BLUE_Pin GPIO_PIN_7
#define BLUE_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#include "stm32f7xx_hal.h"

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
