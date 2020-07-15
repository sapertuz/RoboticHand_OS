/*
 * definitions.h
 *
 *  Created on: 25 de fev de 2019
 *      Author: sapmc
 */

#ifndef SRC_DEFINITIONS_H_
#define SRC_DEFINITIONS_H_

#define M_FLOAT
//#define M_DOUBLE

#ifdef M_FLOAT
typedef float _real;
#define MAX_VALUE FLT_MAX
#else
#ifdef M_DOUBLE
typedef double _real;
#define MAX_VALUE DBL_MAX
#endif
#endif

#endif /* SRC_DEFINITIONS_H_ */
