/**
******************************************************************************
* File Name          : cgi_handlers.c
* Description        : Manage interactions between http server and firmware
* Author						 : Davide Citterio
* Date							 : 30/10/17
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip/debug.h"
#include "lwip/tcp.h"
#include "httpd.h"
#include "cgi_handlers.h"
#include "spi.h"
#include "const.h"
#include "tcp_client.h"

#include <string.h>
#include <stdlib.h>

SPI_HandleTypeDef spi;

DAC_HandleTypeDef vbias;

ADC_HandleTypeDef temp1;
ADC_HandleTypeDef temp2;

struct SPI_Struct spiTPLa;
struct SPI_Struct spiTPLb;

int ADC1_not_configured = 1;
int ADC2_not_configured = 1;
int SPI_not_configured = 1;
int DAC_not_configured = 1;

int config_mode = 1;
int num_of_points = 0;

extern int config_mode;

static void SPI_Init(void);
void ADC_temp1_config(void);
void ADC_temp2_config(void);
void DAC_vbias_Init(void);
void httpd_ssi_init(void);
void httpd_cgi_init(void);
void send_back_data(char *, char *);

const char * TPL_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);


/* Html request for "/leds.cgi" will start LEDS_CGI_Handler */
//const tCGI TDC_CGI={"/tdc.cgi", TDC_CGI_Handler};
const tCGI TPL_CGI={"/tpl.cgi", TPL_CGI_Handler};

/* Cgi call table, only one CGI used */
tCGI CGI_TAB[1];


const char * TPL_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
  uint32_t i=0;
  
  /* We have only one SSI handler iIndex = 0 */
  if (iIndex==0)
  { 
    /* Check cgi parameter : example GET /leds.cgi?led=2&led=4 */
    for (i=0; i<iNumParams; i++)
    {
			if (strcmp(pcParam[i] , "start_measure")==0)
			{
				 num_of_points = atoi(pcValue[i]);
				 config_mode = 0;
				 return "ok";
			}	
		
      if (strcmp(pcParam[i] , "tpl_voutn")==0)   
      {
				/*
				if (SPI_not_configured ==1)       
				{
					SPI_Init();
					SPI_not_configured=0;
				}
				
				HAL_GPIO_WritePin(SPI1_CS_TPL_A_GPIO_Port, SPI1_CS_TPL_A_Pin, GPIO_PIN_RESET);
				//Write Wiper Register A address
        HAL_SPI_Transmit(&spi, (uint8_t * ) TPL_A_WIPER_ADDR, 8, 1000);
				//Write Wiper Register A data
				HAL_SPI_Transmit(&spi, (uint8_t * ) pcValue[i], 8, 1000);
				//Copy Wiper Register A to NV Register A
				HAL_SPI_Transmit(&spi, (uint8_t * ) TPL_A_COPY_WIPER_NV, 8, 1000);
				HAL_GPIO_WritePin(SPI1_CS_TPL_A_GPIO_Port, SPI1_CS_TPL_A_Pin, GPIO_PIN_SET);
				*/
				send_back_data("tpl_voutn", pcValue[i]);
				return "ok";
      }
			
			if (strcmp(pcParam[i] , "tpl_laser")==0)   
      {
				if (SPI_not_configured ==1)       
				{
					SPI_Init();
					SPI_not_configured=0;
				}
			
				HAL_GPIO_WritePin(SPI1_CS_TPL_A_GPIO_Port, SPI1_CS_TPL_A_Pin, GPIO_PIN_RESET);
				//Write Wiper Register B address
        HAL_SPI_Transmit(&spi, (uint8_t * ) TPL_B_WIPER_ADDR, 8, 1000);
				//Write Wiper Register B data
				HAL_SPI_Transmit(&spi, (uint8_t * ) pcValue[i], 8, 1000);
				//Copy Wiper Register B to NV Register B
				HAL_SPI_Transmit(&spi, (uint8_t * ) TPL_B_COPY_WIPER_NV, 8, 1000);
				HAL_GPIO_WritePin(SPI1_CS_TPL_A_GPIO_Port, SPI1_CS_TPL_A_Pin, GPIO_PIN_SET);
				
				send_back_data("tpl_laser", pcValue[i]);
				return "ok";
      }
			
			if (strcmp(pcParam[i] , "tpl_vout1")==0)   
      {
				if (SPI_not_configured ==1)       
				{
					SPI_Init();
					SPI_not_configured=0;
				}
		
				HAL_GPIO_WritePin(SPI1_CS_TPL_B_GPIO_Port, SPI1_CS_TPL_B_Pin, GPIO_PIN_RESET);
				//Write Wiper Register A address
        HAL_SPI_Transmit(&spi, (uint8_t * ) TPL_A_WIPER_ADDR, 8, 1000);
				//Write Wiper Register A data
				HAL_SPI_Transmit(&spi, (uint8_t * ) pcValue[i], 8, 1000);
				//Copy Wiper Register A to NV Register A
				HAL_SPI_Transmit(&spi, (uint8_t * ) TPL_A_COPY_WIPER_NV, 8, 1000);
				HAL_GPIO_WritePin(SPI1_CS_TPL_B_GPIO_Port, SPI1_CS_TPL_B_Pin, GPIO_PIN_SET);
				
				send_back_data("tpl_vout1", pcValue[i]);
				return "ok";
      }
			
			if (strcmp(pcParam[i] , "tpl_vout2")==0)   
      {
				/* configure ADC if not yet configured */
			 if (SPI_not_configured ==1)       
			 {
					SPI_Init();
					SPI_not_configured=0;
			 }
			 
				HAL_GPIO_WritePin(SPI1_CS_TPL_B_GPIO_Port, SPI1_CS_TPL_B_Pin, GPIO_PIN_RESET);
				//Write Wiper Register B address
        HAL_SPI_Transmit(&spi, (uint8_t * ) TPL_B_WIPER_ADDR, 8, 1000);
				//Write Wiper Register B data
				HAL_SPI_Transmit(&spi, (uint8_t * ) pcValue[i], 8, 1000);
				//Copy Wiper Register B to NV Register B
				HAL_SPI_Transmit(&spi, (uint8_t * ) TPL_B_COPY_WIPER_NV, 8, 1000);
				HAL_GPIO_WritePin(SPI1_CS_TPL_B_GPIO_Port, SPI1_CS_TPL_B_Pin, GPIO_PIN_SET);
			 
				send_back_data("tpl_vout2", pcValue[i]);
			 return "ok";
      }
			
			if (strcmp(pcParam[i] , "vbias")==0)   
      {
			 // configure ADC if not yet configured 
			 if (DAC_not_configured ==1)       
			 {
					DAC_vbias_Init();
					DAC_not_configured=0;
			 }
			 
			 //to-do set pcValue[i] to vbias
			 HAL_DAC_Start(&vbias, DAC_CHANNEL_1);
			 HAL_DAC_SetValue(&vbias, DAC_CHANNEL_1, DAC_ALIGN_8B_R, 2048);
			 
			 send_back_data("vbias", pcValue[i]);
			 return "ok";
      }
			
			if (strcmp(pcParam[i] , "temp1")==0)
			{
				uint32_t ADCtemp1 = 0;        

			 /* configure ADC if not yet configured */
			 if (ADC1_not_configured ==1)       
			 {
					ADC_temp1_config();
					ADC1_not_configured=0;
			 }
			 
			 HAL_ADC_Start(&temp1);
			 HAL_ADC_PollForConversion(&temp1, 10);
			 /* get ADC conversion value */
			 ADCtemp1 =  HAL_ADC_GetValue(&temp1);
			 
			 /* convert to Voltage,  step = 0.8 mV */
			 ADCtemp1 = (uint32_t)(ADCtemp1 * 205);
			 
			 send_back_data("temp1", (char *)ADCtemp1);
			 return "ok";
			}
				
			if (strcmp(pcParam[i] , "temp2")==0)
			{
				uint32_t ADCtemp2 = 0;        

			 /* configure ADC if not yet configured */
			 if (ADC2_not_configured ==1)       
			 {
					ADC_temp2_config();
					ADC2_not_configured=0;
			 }
			 
			 HAL_ADC_Start(&temp2);
			 HAL_ADC_PollForConversion(&temp2, 10);
			 /* get ADC conversion value */
			 ADCtemp2 =  HAL_ADC_GetValue(&temp2);
			 
			 /* convert to Voltage,  step = 0.8 mV */
			 ADCtemp2 = (uint32_t)(ADCtemp2 * 0.8); 
			 
			 send_back_data("temp2", (char *) ADCtemp2);
			 return "ok";
		 }	
			
		 if (strcmp(pcParam[i] , "alive")==0)
		 {
			 tcp_send("micro_conn=1");
		 }	
    }
		
  }
  /* uri to send after cgi call*/
  return "ok";  
}


/* SPI4 init function */
static void SPI_Init(void)
{

  /* SPI4 parameter configuration*/
  spi.Instance = SPI4;
  spi.Init.Mode = SPI_MODE_MASTER;
  spi.Init.Direction = SPI_DIRECTION_2LINES;
  spi.Init.DataSize = SPI_DATASIZE_8BIT;
  spi.Init.CLKPolarity = SPI_POLARITY_HIGH;
  spi.Init.CLKPhase = SPI_PHASE_2EDGE;
  spi.Init.NSS = SPI_NSS_SOFT;
  spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
  spi.Init.TIMode = SPI_TIMODE_DISABLE;
  spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  spi.Init.CRCPolynomial = 7;
  spi.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  spi.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&spi) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TEMP1 init function */
static void ADC_temp1_config(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  temp1.Instance = ADC3;
  temp1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  temp1.Init.Resolution = ADC_RESOLUTION_12B;
  temp1.Init.ScanConvMode = DISABLE;
  temp1.Init.ContinuousConvMode = DISABLE;
  temp1.Init.DiscontinuousConvMode = DISABLE;
  temp1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  temp1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  temp1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  temp1.Init.NbrOfConversion = 1;
  temp1.Init.DMAContinuousRequests = DISABLE;
  temp1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&temp1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&temp1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TEMP2 init function */
static void ADC_temp2_config(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  temp2.Instance = ADC3;
  temp2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  temp2.Init.Resolution = ADC_RESOLUTION_12B;
  temp2.Init.ScanConvMode = DISABLE;
  temp2.Init.ContinuousConvMode = DISABLE;
  temp2.Init.DiscontinuousConvMode = DISABLE;
  temp2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  temp2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  temp2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  temp2.Init.NbrOfConversion = 1;
  temp2.Init.DMAContinuousRequests = DISABLE;
  temp2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&temp1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&temp2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


/* DAC init function */
void DAC_vbias_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  vbias.Instance = DAC;
  if (HAL_DAC_Init(&vbias) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&vbias, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* prepare string to send to host */
void send_back_data(char * ref, char * value)
{
	char * data_to_send = (char *) malloc(1 + strlen(ref) +strlen("=")+ strlen(value));
	strcpy(data_to_send, ref);
	strcat(data_to_send, "=");
	strcat(data_to_send, value);

	//send back data to host
	tcp_send(data_to_send);
}


/**
  * @brief  Http webserver Init
  */
void http_server_init(void)
{
  /* Httpd Init */
  httpd_init();
	
  /* configure CGI handlers (LEDs control CGI) */
  CGI_TAB[0] = TPL_CGI;
  http_set_cgi_handlers(CGI_TAB, 1);
}
