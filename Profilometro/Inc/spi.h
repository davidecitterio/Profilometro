/**
******************************************************************************
* File Name          : tdc.h
* Description        : Header communication with tdc7201
* Author						 : Davide Citterio
* Date							 : 26/10/17
******************************************************************************
*/
#include "main.h"

struct SPI_Struct{
	SPI_HandleTypeDef * spi;
	GPIO_TypeDef * pin_port;
	uint16_t pin;
};

struct TDC_reg{
	uint8_t time;
	uint8_t clock_period;
	uint8_t calibration2;
	uint8_t calibration1;
	uint8_t calibration2periods;
	int timestamp;
	int empty;
};

struct measure{
	double distance0;
	double distance1;
	double distance2;
	int time;
};


void tdc_init(struct SPI_Struct * spi);
void tpl_init(struct SPI_Struct * spi, int i);
void tcp_send_default(char *, char *);
void tdc_enable_measure(struct SPI_Struct * spi);
void tdc_obtain_measure(struct SPI_Struct * spi, struct TDC_reg * reg);

void tpl_set_th(struct SPI_Struct * spi, uint8_t treshold);
