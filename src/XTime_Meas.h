/*
 * time_measuring.h
 *
 *  Created on: 18 de dez de 2017
 *      Author: sapmc
 */

#ifndef XTIME_MEAS_H
#define XTIME_MEAS_H

#include "xtime_l.h"

#define COUNTS_PER_MS COUNTS_PER_SECOND/1000
#define COUNTS_PER_US COUNTS_PER_SECOND/1000000

	uint32_t micros();

	uint32_t millis();
	
	XTime getClockCount();

#endif /* SRC_TIME_MEASURING_H_ */
