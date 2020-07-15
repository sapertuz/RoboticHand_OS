

/***************************** Include Files *******************************/
#include "MotorDriver.h"

void setMotor(u32 BaseAddress, u8 pwmPin, double pid_Out){

	int32_t pwm_Out = PWM_V * pid_Out;
	if (pwm_Out > 1023){
		pwm_Out = 1023;
	}else{
		if (pwm_Out < -1023){
			pwm_Out = -1023;
		}
	}
	u32 pwm_AXI = (pwm_Out << 4) + (pwmPin);
	Xil_Out32( BaseAddress , pwm_AXI );

}


/************************** Function Definitions ***************************/
