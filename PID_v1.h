#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.2.1

#include "XTime_Meas.h"
#include "definitions.h"

class PID
{


  public:

  //Constants used in some of the functions below
  #define AUTOMATIC	1
  #define MANUAL	0
  #define DIRECT  0
  #define REVERSE  1
  #define P_ON_M 0
  #define P_ON_E 1

  //commonly used functions **************************************************************************
    PID();
    void Configure(_real*, _real*, _real*,
        _real, _real, _real, int, int);

    PID(_real*, _real*, _real*,        // * constructor.  links the PID to the Input, Output, and 
        _real, _real, _real, int, int);//   Setpoint.  Initial tuning parameters are also set here.
                                          //   (overload for specifying proportional mode)

    PID(_real*, _real*, _real*,        // * constructor.  links the PID to the Input, Output, and 
        _real, _real, _real, int);     //   Setpoint.  Initial tuning parameters are also set here


    void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

    bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetOutputLimits(_real, _real); // * clamps the output to a specific range. 0-255 by default, but
										                      //   it's likely the user will want to change this depending on
										                      //   the application
	


  //available but not commonly used functions ********************************************************
    void SetTunings(_real, _real,       // * While most users will set the tunings once in the 
                    _real);         	    //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
    void SetTunings(_real, _real,       // * overload for specifying proportional mode
                    _real, int);         	  

	void SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
    void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which 
                                          //   the PID calculation is performed.  default is 100


  //Display functions ****************************************************************
	_real GetKp();						  // These functions query the pid for interal values.
	_real GetKi();						  //  they were created mainly for the pid front-end,
	_real GetKd();						  // where it's important to know what is actually 
	int GetMode();						  //  inside the PID.
	int GetDirection();					  //

  private:
	void Initialize();
	
	_real dispKp;				// * we'll hold on to the tuning parameters in user-entered 
	_real dispKi;				//   format for display purposes
	_real dispKd;				//
    
	_real kp;                  // * (P)roportional Tuning Parameter
  _real ki;                  // * (I)ntegral Tuning Parameter
  _real kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;
	int pOn;

  _real *myInput;              // * Pointers to the Input, Output, and Setpoint variables
  _real *myOutput;             //   This creates a hard link between the variables and the 
  _real *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                //   what these values are.  with pointers we'll just know.
			  
	unsigned long lastTime;
	_real outputSum, lastInput;

	unsigned long SampleTime;
	_real outMin, outMax;
	bool inAuto, pOnE;
};
#endif

