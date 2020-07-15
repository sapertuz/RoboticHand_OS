#ifndef SENSORMOD_H
#define SENSORMOD_H

/****************** Include Files ********************/

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include "XTime_Meas.h"

#define SENMOD_SIZE		0x10000

#define STATUS_REG 			0
#define OFFSET_MOD 			0
#define OFFSET_DT 			4
#define OFFSET_R 			8
#define OFFSET_CEN_CURR 	12
#define OFFSET_Q0 			16
#define OFFSET_Q12 			20
#define OFFSET_Q3 			24
#define OFFSET_b01 			28
#define OFFSET_a1 			32
#define OFFSET_DEG_ADC 		36
#define OFFSET_mA_ADC 		40

#define OFFSET_POS_START 	44

#define OFFSET_POS 			48
#define OFFSET_VEL_CURR		52
#define OFFSET_CURR_VOLT 	56
#define Nmodules			7

//#define STATUS_READ     0x01
#define RESET_MASK 		0x10
#define READY_MASK		0xFFFF0000
//#define ISREADY			(1<<Nmodules)-1
#define MASK_MOD_IN		0xFFFFFFE0
#define MASK_READY		0xFFFF0000
#define ISREADY			0x7F0000

/**************************** Type Definitions *****************************/
typedef union sensorMod_union_t {
	        uint32_t 	*_uint32;
	        float 		*_float32;
        }sensorMod_data_t;

/************************** Driver Class Definition ************************/

class sensorMod{
public:
	//***********************************************  Constructor
	sensorMod();
	
	//***********************************************  Configuration
	void setAddress(const uint32_t addr);
	void setDataMonitor(float* posIn, float* currIn, float* voltIn, 
			float* pos_filt, float* vel_filt, float* curr_filt);

	//***********************************************  Calculating Functions
	void start();
	void wait();

	//***********************************************  Configuration Functions
	void set_butter(const float *ext_ab);
	void set_centerCurr(const float ext_centerCurr, uint32_t mod);
	void set_positionStart(const float ext_positionStart, uint32_t mod);
	void set_Kalman(const float ext_dt, const float ext_R, const float ext_Q);
	void set_2ADC(const float DEG_ADC, const float mA_ADC);
	
	//***********************************************  Variable Functions
	float get_centerCurr(uint32_t mod);
	float get_positionStart(uint32_t mod);
	float get_dt();
	float get_R();
	float* get_Q();
	float* get_ab();
	float get_mA_ADC();
	float get_DEG_ADC();

	//********************************************** Get data
	uint32_t isReady();
	void get_modFilt_Data(uint32_t mod, float* mod_data);

	void printConfig();
	void printFiltData();

private:
	uint32_t virtAddr;

	sensorMod_data_t filt_pos, filt_vel, filt_curr;
	sensorMod_data_t posIn, currIn, voltIn;

	sensorMod_data_t center_current;
	sensorMod_data_t position_start;
	sensorMod_data_t dt, R, Q;
	sensorMod_data_t ab;
	sensorMod_data_t DEG_ADC, mA_ADC;

	uint32_t lastTime;
	uint32_t SampleTime;


	void get_filteredData();
	uint32_t read32(const uint32_t offset);
	void write32 (const uint32_t offset, const uint32_t writeval); 
};

#endif // SENSORMOD_H
