#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

/***************************** Include Files ********************************/
#include <stdio.h>

#define MOTORDRIVER_S00AXI_SLV_REG0_OFFSET 0
#define MOTORDRIVER_S00_AXI_SLV_REG1_OFFSET 4
#define MOTORDRIVER_S00_AXI_SLV_REG2_OFFSET 8
#define MOTORDRIVER_S00_AXI_SLV_REG3_OFFSET 12

#define NMOTORS 7
#define MAX_DUTY 1023
#define MOTORDRIVER_PWM_V 127.875			// PWM/V ->		(ant 1023/8)
#define PWM_OUT_LSB 4
#define PWMPIN_MASK 0xF

class motorDriver{
    public:
        // Creates UIO handler for /dev/uio* device
        motorDriver();

        void setAddress(const uint32_t _addr);
        
        void setMotor(uint8_t pwmPin, double pid_Out);
        void disableMotors();

    private:
        uint32_t virtAddr;

        void write32(const uint32_t writeval);
};


#endif // MOTORDRIVER_H
