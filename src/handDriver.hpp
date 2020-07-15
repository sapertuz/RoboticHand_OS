#ifndef HANDDRIVER_H
#define HANDDRIVER_H

/***************************** Include Files ********************************/

#include <stdio.h>

#include "SensorMod.hpp"
#include "PID_v1.h"
#include "Impedance_v1.hpp"
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
#define DT		5		// Filter and Control Period in milliSeconds
#define M 		8/6		//8	//0.528404343800586
#define K		5/6		//5	//88.6173474407601
#define B		30/6	//30//9.58011226419688
#define P 		0.1		// Proportional P gain
#define outMax	6.0		// Maximum output P controller
#define outMin	-3.0	// Minimum output P controller
/*@}*/

/**
 * @name Parameters for the filters
 * @{
 */
#define R 		1e-1
#define nQ 		200
#define a		-0.9450
#define b 		0.0275
#define cenCurr	22500.0
/*@}*/

/**
 * @name Control Type Definitions
 * @{
 */
#define UNOPERATIONAL	0
#define MANUAL_MOTORS	1
#define P_ONLY			2
#define FULL_IMPEDANCE	3
/*@}*/

/**
 * @name Impedance Fingers Control Activation
 * @{
 */
// 						   7  6  5  4  3  2  1  0
// 							 P1  A  C I1 I2  M P2
#define NO_FINGERS 	0x00// 0  0  0  0  0  0  0  0
#define ALL_FINGERS	0x7F// 0  1  1  1  1  1  1  1
#define TRI_FINGERS	0x58// 0  1  0  1  1  0  0  0
#define INX_FINGERS	0x08// 0  0  0  0  1  0  0  0
/*@}*/

/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/

/************************** Driver Class Definition ************************/

class handDriver{
	public:
		handDriver();
		int initialize();

		void init_controllers();
		void set_control(uint8_t);

		void calibrateCurrentS();
		void calibratePositionS(_real *);

		void getSensors(_real *, _real *);
		void getSenFilters(_real *, _real *, _real *);
		void getControl(_real *, _real *, _real *);

		void compute();
		void shutdown();
		void update_sensors();
		void init_impOut();

		void set_impFingers(uint8_t);
		int set_currSP(_real *);
		int set_posSP(_real *);
		int set_motorV(_real *);

		void printConfig();

	/************************** Variable Definitions ****************************/
	private:

	/**
	 * Controllers Input/Output Variables
	 * @{
	 */
	_real posIn_f[Nmodules] =
	{0, 0, 0, 0, 0, 0, 0};		// Define Input Variables for filters
	_real currIn_f[Nmodules] =
	{0, 0, 0, 0, 0, 0, 0};

				// Define Center Current variable
	_real cenCurr_f[Nmodules] = 
		{0, 0, 0, 0, 0, 0, 0};
				// Define Star position of position sensors
	_real posStart_f[Nmodules] = {
		65535-16500,	// P2 0
		9000,	// M  1
		0,		// I2 2
		7900,	// I1 3
		10000,	// C  4
		8900,	// A  5
		8900,	// P1 6
	};

	_real pos_filt[Nmodules] =
	{0, 0, 0, 0, 0, 0, 0};		// Define Output Variables for filters
	_real vel_filt[Nmodules] =
	{0, 0, 0, 0, 0, 0, 0};
	_real curr_filt[Nmodules] =
	{0, 0, 0, 0, 0, 0, 0};

	_real voltOut[Nmodules] =
	{0, 0, 0, 0, 0, 0, 0};		// Define Variables to connect PID controller
	_real imp_sp[Nmodules] =
	{0, 0, 0, 0, 0, 0, 0};			// Variables to connect Impedance controller
	_real impOut[Nmodules] =
	{0, 0, 0, 0, 0, 0, 0};
	/*@}*/

	uint16_t imp_index=0;

	XAdcPs XAdcInst;      			// XADC driver instance
	XAdcPs *XAdcInstPtr = &XAdcInst;
	XAdcPs_Config *ConfigPtr;

	motorDriver motorsHandler;		// Create handlers for MotorDriver
	sensorMod sensorHandler; 		// Create handlers for sensorMod
	PID pidControllers[Nmodules];	// Create Proportional controllers objects
	Impedance ImpControllers[Nmodules]; // Create Impedance controllers objects

	/* Control Type
	 * 0 - Control off
	 * 1 - Control manual
	 * 2 - Control PID position
	 * 3 - Control Impedance + PID
	*/
	uint8_t control_t = 0;
	
};
/*****************************************************************************/

#endif // HANDDRIVER_H
