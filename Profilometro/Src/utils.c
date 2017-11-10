/**
******************************************************************************
* File Name          : utils.c
* Description        : Contains some utils function
* Author						 : Davide Citterio
* Date							 : 27/10/17
******************************************************************************
*/

#include "utils.h"

void utils_init_regs(struct TDC_reg * REG)
{
	struct TDC_reg new_reg;
	
	new_reg.calibration1 = 0;
	new_reg.calibration2 = 0;
	new_reg.calibration2periods = 0;
	new_reg.clock_period = 0;
	new_reg.time = 0;
	new_reg.empty = 1;
	
	*REG = new_reg;
}
