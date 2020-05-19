
/***************************** Include Files *******************************/
#include "MotorDriver.hpp"
#include "xil_io.h"

/************************** Function Definitions ***************************/ 
motorDriver::motorDriver(const uint32_t _addr){
	this->virtAddr =  _addr;
}

void motorDriver::setMotor(uint8_t pwmPin, double pid_Out){
	int32_t pwm_Out = (int32_t)(MOTORDRIVER_PWM_V * pid_Out);
	if (pwm_Out > MAX_DUTY){
		pwm_Out = MAX_DUTY;
	}else{
		if (pwm_Out < -MAX_DUTY){
			pwm_Out = -MAX_DUTY;
		}
	}
	uint32_t pwm_AXI = ((uint32_t)pwm_Out << PWM_OUT_LSB) | ((uint32_t)pwmPin & PWMPIN_MASK);
	write32(pwm_AXI);

}

void motorDriver::disableMotors(){
	for (uint8_t i=0; i<NMOTORS; i++){
		setMotor(i, (uint32_t)0);
	}
}

void motorDriver::write32(const uint32_t writeval){
	Xil_Out32(this->virtAddr, writeval);
}
