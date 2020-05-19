#ifndef HANDDRIVER_H
#define HANDDRIVER_H

/***************************** Include Files ********************************/

#include <stdio.h>

#include "SensorMod.hpp"
#include "PID_v1.h"
#include "MotorDriver.hpp"

#include "XTime_Meas.h"
#include "xuartps.h"

#include "definitions.h"
#include "xil_io.h"
#include "xadcps.h"

/**
 * PL modules uio drivers locations
 * @{
 */
#define addr_motorMod 	0x43C10000 // motor mod
#define addr_sensorMod  0x43C20000 // sensor mod
/*@}*/


#define A0 XADCPS_CH_AUX_MIN + 1
#define A1 XADCPS_CH_AUX_MIN + 9
#define A2 XADCPS_CH_AUX_MIN + 6
#define A3 XADCPS_CH_AUX_MIN +15
#define A4 XADCPS_CH_AUX_MIN + 5
#define A5 XADCPS_CH_AUX_MIN +13

#define A6 XADCPS_CH_AUX_MIN +12
#define A8 XADCPS_CH_AUX_MIN
#define A10 XADCPS_CH_AUX_MIN+ 8

#define Pot	A2

/**
 * @name Motor Driver index
 * @{
 */
#define pwmCh_M 	1
#define pwmCh_I2 	2
#define pwmCh_A		5
#define pwmCh_C 	4
#define pwmCh_I1 	3
#define pwmCh_P1	6
#define pwmCh_P2 	0
/*@}*/

/**
 * @name DOF and Current Analog Channels
 * @{
 */
#define anCurr_P1	A6 // A4
#define anCurr_C	A10// A1
#define anCurr_I1	A8 // A0

#define anPos_P1	A2 // A2 3.3v
#define anPos_I1	A3 // A3 3.3v
#define anPos_C		A5 // A5 3.3v
#define anPos_A		A4 // A6 3.3v
#define anPos_M		A1 // A8 3.3v
#define anPos_P2	A0 // A10 3.3v
/*@}*/

/**
 * @name Indexes for the Sensor Module and Controllers DOF.
 * @{
 */
#define i_I1 	pwmCh_I1
#define i_I2 	pwmCh_I2
#define i_P1 	pwmCh_P1
#define i_P2	pwmCh_P2
#define i_A		pwmCh_A
#define i_C		pwmCh_C
#define i_M		pwmCh_M
#define Nmodules 	7
/*@}*/

/**
 * @name Some constants for data conversion
 * @{
 */
#define pi 		3.1416
#define upPos	60000
#define lowPos	22000
#define RAD_ADC 4.21e-5	// rad/ADC -> // (pi/2)/(upPos-lowPos)
#define PWM_V	1023.0/8.0	// PWM/V
#define DEG_RAD	180/pi	// Degrees/Radians
#define mA_ADC	0.108   // miliAmpers/ADCraw
#define DEG_ADC	0.016	// Deg/ADCraw
/*@}*/

/**
 * @name Parameters for the Impedance Controllers
 * @{
 */
#define dt	5	// Filter and Control Period in milliSeconds
#define M 	0.528404343800586
#define K 	88.6173474407601
#define B 	9.58011226419688
#define P 	0.4	// Proportional P gain
#define outMax	4.0	// Maximum output P controller
#define outMin	-4.0	// Minimum output P controller
/*@}*/

/**
 * @name Parameters for the filters
 * @{
 */
#define R 		4e-3
#define nQ 		10000
#define a		-0.9450
#define b 		0.0275
#define cenCurr	22500.0
/*@}*/

/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/

/************************** Driver Class Definition ************************/

class handDriver{
	public:
		handDriver();
		int initialize();

		void linkSensors(_real *, _real *);
		void linkSenFilters(_real *, _real *, _real *);
		void linkControl(_real *, _real *, _real *);

		void compute();
		void shutdown();
		void update_sensors();
		void init_impOut();

	/************************** Variable Definitions ****************************/
	private:


	/**
	 * Controllers Input/Output Variables
	 * @{
	 */
	_real posIn_f[Nmodules];	// Define Input Variables for filters
	_real currIn_f[Nmodules];
	//_real voltIn_f[Nmodules];
	_real cenCurr_f[Nmodules];   // Define Center Current variable
	_real posStart_f[Nmodules]; // Defin Star position of position sensors

	_real pos_filt[Nmodules];	// Define Output Variables for filters
	_real vel_filt[Nmodules];
	_real curr_filt[Nmodules];

	_real voltOut[Nmodules];	// Define Variables to connect PID controller

	_real imp_sp[Nmodules];		// Variables to connect Impedance controller
	_real impOut[Nmodules];
	/*@}*/

	XAdcPs XAdcInst;      /* XADC driver instance */
	XAdcPs *XAdcInstPtr = &XAdcInst;

	/**
	 * Create handlers for MotorDriver
	 * {
	 */
	motorDriver *motorsHandler_p;
	/*}*/

	/**
	 * Create handlers for sensorMod
	 * {
	 */
	sensorMod *sensorHandler_p;
	/*}*/

	/**
	 * Create Proportional controllers objects
	 * {
	 */
	PID pidControllers[Nmodules];
	/*}*/
};
/*****************************************************************************/

#endif // HANDDRIVER_H
