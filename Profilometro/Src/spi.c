/****************************************************************************
  * File Name          : tdc.h
  * Description        : Header communication with tdc7201
	* Author						 : Davide Citterio
	* Date							 : 26/1/17
  ******************************************************************************
*/
	
#include "spi.h"
#include "stm32f7xx_hal.h"
#include <stdint.h>
#include "const.h"
#include "tcp_client.h"
#include <string.h>
#include <stdlib.h>

/*
* Reset all tdc registers and prepare for spi communication
*/
void tdc_init(struct SPI_Struct * spi)
{
	// set calibration2period register
	HAL_GPIO_WritePin(spi->pin_port, spi->pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spi->spi, (uint8_t *)CONFIG2_addr, 8, TIMEOUT);
	HAL_SPI_Transmit(spi->spi, (uint8_t *)CONFIG2_data, 8, TIMEOUT);
	HAL_GPIO_WritePin(spi->pin_port, spi->pin, GPIO_PIN_SET);
}

void tpl_init(struct SPI_Struct * spi, int i)
{
	//TPLA
	if (i == 0){
		// set default thresholds
		HAL_GPIO_WritePin(spi->pin_port, spi->pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(spi->spi, (uint8_t *)TPL_A_WIPER_ADDR, 8, TIMEOUT);
		HAL_SPI_Transmit(spi->spi, (uint8_t *)TPL_VOUTN_THRESHOLD, 8, TIMEOUT);
		HAL_SPI_Transmit(spi->spi, (uint8_t *)TPL_A_COPY_WIPER_NV, 8, TIMEOUT);
		HAL_GPIO_WritePin(spi->pin_port, spi->pin, GPIO_PIN_SET);
		tcp_send_default("tpl_voutn=", (char *)TPL_VOUTN_THRESHOLD);
		
		// set default thresholds
		HAL_GPIO_WritePin(spi->pin_port, spi->pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(spi->spi, (uint8_t *)TPL_B_WIPER_ADDR, 8, TIMEOUT);
		HAL_SPI_Transmit(spi->spi, (uint8_t *)TPL_LASER_THRESHOLD, 8, TIMEOUT);
		HAL_SPI_Transmit(spi->spi, (uint8_t *)TPL_B_COPY_WIPER_NV, 8, TIMEOUT);
		HAL_GPIO_WritePin(spi->pin_port, spi->pin, GPIO_PIN_SET);
		tcp_send_default("tpl_laser=", (char *)TPL_LASER_THRESHOLD);
	}
	
	//TPLB
	if (i == 1){
		// set default thresholds
		HAL_GPIO_WritePin(spi->pin_port, spi->pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(spi->spi, (uint8_t *)TPL_A_WIPER_ADDR, 8, TIMEOUT);
		HAL_SPI_Transmit(spi->spi, (uint8_t *)TPL_VOUT1_THRESHOLD, 8, TIMEOUT);
		HAL_SPI_Transmit(spi->spi, (uint8_t *)TPL_A_COPY_WIPER_NV, 8, TIMEOUT);
		HAL_GPIO_WritePin(spi->pin_port, spi->pin, GPIO_PIN_SET);
		tcp_send_default("tpl_vout1=", (char *)TPL_VOUT1_THRESHOLD);
		
		// set default thresholds
		HAL_GPIO_WritePin(spi->pin_port, spi->pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(spi->spi, (uint8_t *)TPL_B_WIPER_ADDR, 8, TIMEOUT);
		HAL_SPI_Transmit(spi->spi, (uint8_t *)TPL_VOUT2_THRESHOLD, 8, TIMEOUT);
		HAL_SPI_Transmit(spi->spi, (uint8_t *)TPL_B_COPY_WIPER_NV, 8, TIMEOUT);
		HAL_GPIO_WritePin(spi->pin_port, spi->pin, GPIO_PIN_SET);
		tcp_send_default("tpl_vout2=", (char *)TPL_VOUT2_THRESHOLD);
	}
}

void tcp_send_default(char * c, char * d)
{
	char * data_to_send;
	data_to_send=malloc(strlen(c)+1+strlen(d));
	strcpy(data_to_send, c);
	strcat(data_to_send, d);
	
	//send back data to host
	tcp_send(data_to_send);
	
	free(data_to_send);
}
	
/*
* Set register enable_measure
*/
void tdc_enable_measure(struct SPI_Struct * spi)
{
	HAL_GPIO_WritePin(spi->pin_port, spi->pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spi->spi, (uint8_t *)CONFIG1_addr, 8, TIMEOUT);
	HAL_SPI_Transmit(spi->spi, (uint8_t *)CONFIG1_data, 8, TIMEOUT);
	HAL_GPIO_WritePin(spi->pin_port, spi->pin, GPIO_PIN_SET);
}
	
/*
* Obtain distance by querying data from tdcs and calculating
*/
	void tdc_obtain_measure(struct SPI_Struct * spi, struct TDC_reg * reg)
{
	HAL_GPIO_WritePin(spi->pin_port, spi->pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(spi->spi, (uint8_t *) TIME_reg, &reg->time, (uint16_t)24, TIMEOUT);
	HAL_SPI_TransmitReceive(spi->spi, (uint8_t *) CALIBRATION1, &reg->calibration1 , (uint16_t)24, TIMEOUT);
	HAL_SPI_TransmitReceive(spi->spi, (uint8_t *) CALIBRATION2, &reg->calibration2, (uint16_t)24, TIMEOUT);
	reg->clock_period = CLOCK_PERIOD;
	reg->calibration2periods = CALIBRATION2PERIODS; 
	HAL_GPIO_WritePin(spi->pin_port, spi->pin, GPIO_PIN_SET);
}
	
/*
* Set treshold of TPL0202
*/
void tpl_set_th(struct SPI_Struct * spi, uint8_t treshold)
{
	HAL_GPIO_WritePin(spi->pin_port, spi->pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spi->spi, &treshold, 16, TIMEOUT);
	HAL_GPIO_WritePin(spi->pin_port, spi->pin, GPIO_PIN_SET);
}
