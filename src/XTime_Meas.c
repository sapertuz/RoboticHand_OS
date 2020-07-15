/*
 * XTime_Meas.cpp
 *
 *  Created on: 19 de dez de 2017
 *      Author: sapmc
 */
#include "XTime_Meas.h"

uint32_t micros(){
	XTime x_time;
	XTime_GetTime(&x_time);
	uint32_t time_f = (uint32_t)(((uint32_t)x_time)/((uint32_t)COUNTS_PER_US));
	return (time_f);
}

uint32_t millis(){
	XTime x_time;
	XTime_GetTime(&x_time);
	uint32_t time_f = (uint32_t)(((uint32_t)x_time)/((uint32_t)COUNTS_PER_MS));
	return (time_f);
}

XTime getClockCount(){
	XTime x_time;
	XTime_GetTime(&x_time);
	return (x_time);
}
