/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "lwip.h"

/* USER CODE BEGIN Includes */
#include "utils.h"
#include "tcp_client.h"
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "lwip/tcp.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

DAC_HandleTypeDef hdac;

SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC3_Init(void);
static void MX_DAC_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM10_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void spi_set_cs_high(void);
void couple_spi_cs(void);
void tdc_tpl_config(void);
static void tdcs_enable(void);
static void laser_enable(void);
static void laser_disable(void);
static void populate_TX_RX0(void);
static void populate_RX1(void);
static void populate_RX2(void);
void compute_distance(int);
void send_measures_tcp(void);
void prepare_measure(char *, int, char *, double);


static err_t tcp_alive_connected(void *arg, struct tcp_pcb *tpcb, err_t err);
static err_t tcp_measure_connected(void *arg, struct tcp_pcb *tpcb, err_t err);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/*couple spi with cs pin*/

struct SPI_Struct spiTX;
struct SPI_Struct spiRX0;
struct SPI_Struct spiRX1;
struct SPI_Struct spiRX2;
struct SPI_Struct tplA;
struct SPI_Struct tplB;

struct TDC_reg TX;
struct TDC_reg RX0;
struct TDC_reg RX1;
struct TDC_reg RX2;

struct measure distance;

int finish_measurement = 0; // 1 if measurement is finished

int alive_conn = 1;
int measure_conn = 0;

struct tcp_pcb *alive_pcb;
struct tcp_pcb *measure_pcb;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
		case BUTTON_CONFIG_Pin : 
				{
					//se sono in config mode segnalo a labview che mi assento per misurare
					if (config_mode)
					{
						tcp_write(alive_pcb, "conn=1&conf=0", strlen("conn=1&conf=0"), 1);
						tcp_close(alive_pcb);
						alive_conn = 0;
					}
					else 
					{
						alive_conn = 1;
					}

					HAL_GPIO_TogglePin(BLUE_GPIO_Port, BLUE_Pin); 
					HAL_GPIO_TogglePin(RED_GPIO_Port, RED_Pin);
					config_mode = !config_mode;
					break;
				}
				
		case TDC_INT_Pin : 
				{
					populate_TX_RX0();
					break;
				}					
		
		case TDC_INT_RX1_Pin:
				{
					populate_RX1();
				  break;
				}
		
		case TDC_INT_RX2_Pin: 
				{		
					populate_RX2();
					break;
				}		
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (config_mode)
	{
		HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);
		
		if (alive_conn)
		{
			char * mode;
			sprintf(mode, "%d", config_mode);
			char * config = (char *) malloc(1 + strlen("conn=1") + strlen("&conf=")+ strlen(mode) );
			strcpy(config, "conn=1");
			strcat(config, "&conf=");
			strcat(config, mode);
			
			tcp_send(config);
		}
	}
	else 
	{
		HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET);
	}
}


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC3_Init();
  MX_DAC_Init();
  MX_SPI4_Init();
  MX_SPI5_Init();
  MX_LWIP_Init();
  MX_TIM10_Init();

  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim10);
	
	HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);
			
			
	//set all spi cs high
	spi_set_cs_high();
	
	//set spi channel and cs pin
	couple_spi_cs();
	
	//initial config of tdcs and obtain initial reg values
	//tdc_tpl_config();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		if (!config_mode)
		{
			for (int i = 0; i < num_of_points; i++)
			{
				//enable tdcs register for measurement and laser driver
				tdcs_enable();
				laser_enable();
				
				//set start signal
				HAL_GPIO_WritePin(START_GPIO_Port, START_Pin, GPIO_PIN_SET);
				//delay 10 ns
				HAL_Delay(10);
				//reset start signal
				HAL_GPIO_WritePin(START_GPIO_Port, START_Pin, GPIO_PIN_RESET);
				
				//wait until first interrupt comes
				while(RX0.empty);
				
				//disable laser driver (for security reasons)
				laser_disable();
				
				compute_distance(0);
				if (!RX1.empty)
					compute_distance(1);
				if (!RX2.empty)
					compute_distance(2);
			}
			//tell to micro to send data
			//send_measures_tcp();
			//enable config mode at the end of measurements
			config_mode = 1;
		}
		else
		{
			MX_LWIP_Process();
		}
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_PLLI2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 160;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLP_DIV2;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 4;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
  PeriphClkInitStruct.PLLI2SDivQ = 1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_PLLI2SCLK, RCC_MCODIV_5);

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC3 init function */
static void MX_ADC3_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI4 init function */
static void MX_SPI4_Init(void)
{

  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI5 init function */
static void MX_SPI5_Init(void)
{

  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi5.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 7;
  hspi5.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi5.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 99999;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 39999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PD8   ------> USART3_TX
     PD9   ------> USART3_RX
     PC9   ------> RCC_MCO_2
     PA8   ------> RCC_MCO_1
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LASER_ENABLE_Pin|SPI1_CS_TPL_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SPI1_CS_TPL_B_Pin|SPI2_CS_RX1_Pin|SPI2_CS_RX2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUTTON_ENABLE_Pin|RED_Pin|BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SPI1_CS_TX_Pin|SPI1_CS_RX0_Pin|START_Pin|CALIBRATION_Pin 
                          |TPROB_CALIBRATION_ENABLE2_Pin|_5K_GAIN_SETTING2_Pin|TPROB_CALIBRATION_ENABLE1_Pin|_5K_GAIN_SETTING1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LASER_ENABLE_Pin SPI1_CS_TPL_A_Pin */
  GPIO_InitStruct.Pin = LASER_ENABLE_Pin|SPI1_CS_TPL_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : GREEN_Pin */
  GPIO_InitStruct.Pin = GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TDC_INT_RX1_Pin */
  GPIO_InitStruct.Pin = TDC_INT_RX1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TDC_INT_RX1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS_TPL_B_Pin SPI2_CS_RX1_Pin SPI2_CS_RX2_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_TPL_B_Pin|SPI2_CS_RX1_Pin|SPI2_CS_RX2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : TDC_INT_Pin */
  GPIO_InitStruct.Pin = TDC_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TDC_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_ENABLE_Pin RED_Pin BLUE_Pin */
  GPIO_InitStruct.Pin = BUTTON_ENABLE_Pin|RED_Pin|BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_CONFIG_Pin */
  GPIO_InitStruct.Pin = BUTTON_CONFIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BUTTON_CONFIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS_TX_Pin SPI1_CS_RX0_Pin START_Pin CALIBRATION_Pin 
                           TPROB_CALIBRATION_ENABLE2_Pin _5K_GAIN_SETTING2_Pin TPROB_CALIBRATION_ENABLE1_Pin _5K_GAIN_SETTING1_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_TX_Pin|SPI1_CS_RX0_Pin|START_Pin|CALIBRATION_Pin 
                          |TPROB_CALIBRATION_ENABLE2_Pin|_5K_GAIN_SETTING2_Pin|TPROB_CALIBRATION_ENABLE1_Pin|_5K_GAIN_SETTING1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TDC_CLK_Pin */
  GPIO_InitStruct.Pin = TDC_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(TDC_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TDC_INT_RX2_Pin */
  GPIO_InitStruct.Pin = TDC_INT_RX2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TDC_INT_RX2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */


/* Start measurement phase */
void start_measuring()
{
	config_mode = 0;
}

void stop_measuring()
{
	config_mode = 1;
}


/**
Set to high all CS pins of spi channels
*/
void spi_set_cs_high()
{
	HAL_GPIO_WritePin(SPI1_CS_TX_GPIO_Port, SPI1_CS_TX_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI1_CS_RX0_GPIO_Port, SPI1_CS_RX0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI2_CS_RX1_GPIO_Port, SPI2_CS_RX1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI2_CS_RX2_GPIO_Port, SPI2_CS_RX2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI1_CS_TPL_A_GPIO_Port, SPI1_CS_TPL_A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI1_CS_TPL_B_GPIO_Port, SPI1_CS_TPL_B_Pin, GPIO_PIN_SET);
}

/**
Coupling in one struct CS pin with relative spi channel
*/
void couple_spi_cs()
{
	spiTX.spi = &hspi4;
	spiTX.pin_port = SPI1_CS_TX_GPIO_Port;
	spiTX.pin = SPI1_CS_TX_Pin;
	
	spiRX0.spi = &hspi4;
	spiRX0.pin_port = SPI1_CS_RX0_GPIO_Port;
	spiRX0.pin = SPI1_CS_RX0_Pin;
	
	spiRX1.spi = &hspi5;
	spiRX1.pin_port = SPI2_CS_RX1_GPIO_Port;
	spiRX1.pin = SPI2_CS_RX1_Pin;
	
	spiRX2.spi = &hspi5;
	spiRX2.pin_port = SPI2_CS_RX2_GPIO_Port;
	spiRX2.pin = SPI2_CS_RX2_Pin;
	
	tplA.spi = &hspi4;
	tplA.pin_port = SPI1_CS_TPL_A_GPIO_Port;
	tplA.pin = SPI1_CS_TPL_A_Pin;
	
	tplB.spi = &hspi4;
	tplB.pin_port = SPI1_CS_TPL_B_GPIO_Port;
	tplB.pin = SPI1_CS_TPL_B_Pin;
}

/**
* Reset local register variable and
* enable start_measurement on tdcs
*/
void tdcs_enable()
{
	tdc_enable_measure(&spiTX);
	tdc_enable_measure(&spiRX0);
	tdc_enable_measure(&spiRX1);
	tdc_enable_measure(&spiRX2);
	utils_init_regs(&TX);
	utils_init_regs(&RX0);
	utils_init_regs(&RX1);
	utils_init_regs(&RX2);
}

/**
* Initial config of register and obtain reg values at startup
*/
void tdc_tpl_config()
{
	tdc_init(&spiTX);
	tdc_init(&spiRX0);
	tdc_init(&spiRX1);
	tdc_init(&spiRX2);
	
	tpl_init(&tplA, 0);
	tpl_init(&tplA, 1);
}


/**
* Enable laser driver
*/
void laser_enable()
{
	HAL_GPIO_WritePin(LASER_ENABLE_GPIO_Port, LASER_ENABLE_Pin, GPIO_PIN_SET);
}

/**
* Disable laser driver
*/
void laser_disable()
{
	HAL_GPIO_WritePin(LASER_ENABLE_GPIO_Port, LASER_ENABLE_Pin, GPIO_PIN_RESET);
}

/**
* Once first interrupt TDC_INT has come
* retrive data from TX and RX0 registers
*/
void populate_TX_RX0()
{	
	tdc_obtain_measure(&spiTX, &TX);
	tdc_obtain_measure(&spiRX0, &RX0);
	TX.empty = 0;
	RX0.empty = 0;
}
	
/**
* Once second interrupt TDC_RX1_INT has come
* retrive data from RX1 registers
*/
void populate_RX1()
{
	tdc_obtain_measure(&spiRX1, &RX1);
	RX1.empty = 0;
}
	
/**
* Once second interrupt TDC_RX2_INT has come
* retrive data from RX2 registers
*/
void populate_RX2()
{
	tdc_obtain_measure(&spiRX2, &RX2);
	RX2.empty = 0;
}

/**
* Compute distance of i-th tdc
*/
void compute_distance(int i)
{
	double calCount, normLSB, tofInit, tofSec;
	
	calCount = (TX.calibration2 - TX.calibration1) / (TX.calibration2periods - 1);
	normLSB  = TX.clock_period / calCount;
	tofInit = TX.time * normLSB;
	
	//tdc_TX & tdc_RX0 data ready
	if (i == 0)
	{
		calCount = (RX0.calibration2 - RX0.calibration1) / (RX0.calibration2periods - 1);
		normLSB  = RX0.clock_period / calCount;
		tofSec = RX0.time * normLSB;
		
		distance.distance0 = tofSec - tofInit;
		distance.time = RX0.timestamp;
	}
	
	//tdc_RX1 data ready
	if (i == 1)
	{
		calCount = (RX1.calibration2 - RX1.calibration1) / (RX1.calibration2periods - 1);
		normLSB  = RX1.clock_period / calCount;
		tofSec = RX1.time * normLSB;
		
		distance.distance1 = tofSec - tofInit;
		distance.time = RX1.timestamp;
	}
	
	//tdc_RX2 data ready
	if (i == 2)
	{
		calCount = (RX2.calibration2 - RX2.calibration1) / (RX2.calibration2periods - 1);
		normLSB  = RX2.clock_period / calCount;
		tofSec = RX2.time * normLSB;
		
		distance.distance2 = tofSec - tofInit;
		distance.time = RX2.timestamp;
	}
}

void send_measures_tcp()
{
	prepare_measure("time=", distance.time, "dist=", distance.distance0);
	prepare_measure("time=", distance.time, "dist=", distance.distance1);
	prepare_measure("time=", distance.time, "dist=", distance.distance2);
}


void prepare_measure(char * t, int time, char* d, double distance)
{
	char * dist;
	char * tim;
	sprintf(tim, "%d", time);
	sprintf(dist, "%f", distance);
	char * data_to_send = (char *) malloc(1 + strlen(tim)+ strlen(dist)+ strlen(t)+ strlen(d) );
	strcpy(data_to_send, t);
	strcat(data_to_send, tim);
	strcat(data_to_send, d);
	strcat(data_to_send, dist);
	
	//send back data to host
	tcp_send(data_to_send);
	
	free(data_to_send);
}


/*TCP ALIVE AND MEASURE SEND FUNCTIONS*/
void tcp_alive_connect()
{
  ip_addr_t DestIPaddr;
	ip_addr_t LocalIPaddr;
   
  /* create new tcp pcb */
  alive_pcb = tcp_new();

  if (alive_pcb != NULL)
  {
		IP4_ADDR( &LocalIPaddr, LOCAL_IP_ADDR0, LOCAL_IP_ADDR1, LOCAL_IP_ADDR2, LOCAL_IP_ADDR3 );
    IP4_ADDR( &DestIPaddr,  DEST_IP_ADDR0,  DEST_IP_ADDR1,  DEST_IP_ADDR2,  DEST_IP_ADDR3 );
    
		/* bind tcp connection with local parameter */
		tcp_bind(alive_pcb, &LocalIPaddr, ALIVE_MICRO_PORT);

		err_t err;
		
    /* connect to destination address/port */
    err = tcp_connect(alive_pcb,&DestIPaddr,ALIVE_SERVER_PORT,tcp_alive_connected);
		
		if (err != ERR_OK){
			HAL_GPIO_TogglePin(RED_GPIO_Port, RED_Pin);
			tcp_close(alive_pcb);
		}
  }
  else
  {
    /* deallocate the pcb */
    memp_free(MEMP_TCP_PCB, alive_pcb);
  }
}

static err_t tcp_alive_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
	alive_conn = 1;
	return ERR_OK;
}
	
static err_t tcp_measure_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
	measure_conn = 1;
	return ERR_OK;
}




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
