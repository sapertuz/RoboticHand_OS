#ifndef IMPEDANCE_V1_H
#define IMPEDANCE_V1_H

#include "XTime_Meas.h"
#include "definitions.h"

	//Constants used in some of the functions below
#define AUTOMATIC	1
#define MANUAL		0
#define DIRECT		0
#define REVERSE		1
#define LIMITED		1
#define NONLIMITED	0

class Impedance
{

public:

	//commonly used functions **************************************************************************
	Impedance();
	
	void Configure(_real*, _real*, _real*, _real*, _real*,	// * Constructor.  links the Impedance controller to the Input,
		_real, _real, _real, int);//   Output, and Setpoint.  Initial tuning parameters M B and K*/
	void Restart();

	void SetMode(int Mode);					// * Sets Impedance Controller to either Manual (0) or Auto (non-0)

	bool Compute();							// * Performs the Impedance Controller action calculation.
											//   it should be called every time loop() cycles. ON/OFF and
											//   calculation frequency can be set using SetMode
											//   SetSampleTime respectively

	//available but not commonly used functions ********************************************************
	void SetTunings(_real, _real,			// * While most users will set the tunings once in the
		_real);         					//   constructor, this function gives the user the option
											//   of changing tunings during runtime for Adaptive control

	void SetControllerDirection(int);		// * Sets the Direction, or "Action" of the controller. DIRECT
											//   means the output will increase when error is positive. REVERSE
											//   means the opposite.  it's very unlikely that this will be needed
											//   once it is set in the constructor.
	
	void SetSampleTime(int);				// * Sets the frequency, in Milliseconds, with which 
											//   the PID calculation is performed.  default is 100

	//Display functions ****************************************************************
	_real GetM();							//	These functions query the Impedance Controller for interal values.
	_real GetB();
	_real GetK();
	_real Get_xp();
	int GetMode();							
	int GetDirection();						

private:
	void Initialize();

	_real dispM;				// * we'll hold on to the tuning parameters in user-entered
	_real dispB;				//   format for display purposes
	_real dispK;				//

	_real myM;                  // * M - Inertia Component
	_real myB;                  // * B - Energy dissipation term from the viscous damper
	_real myK;                  // * K - Stiffness of the equivalent system

	int controllerDirection;
	
	_real x;
	_real xp;
	_real xpp;

	_real *myInput;              // * Pointers to the Input, Output, and Setpoint variables
	_real *myOutput;
	_real *mySetpoint;

	_real *my_x;
	_real *my_xp;

	unsigned long lastTime;
	_real last_xpp, last_xp, last_x;

	unsigned long SampleTime;
	_real SampleTimeInSec;
	_real outMin, outMax;
	bool inAuto;
};
#endif

