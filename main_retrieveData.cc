
/***************************** Include Files ********************************/

#include <stdio.h>
//#include <iostream>
#include <math.h>

//#include <iomanip>

#include "handDriver.hpp"

#include "XTime_Meas.h"
#include "xuartps.h"
//#include "keyboard_utils.h"

#include "definitions.h"
#include "xil_io.h"

/************************** Constant Definitions ****************************/

#define XIL_GPIO_1_OFFSET		0x0
#define XIL_GPIO_1_TRI_OFFSET	0x4
#define XIL_GPIO_2_OFFSET		0x8
#define XIL_GPIO_2_TRI_OFFSET	0xC

/**
 * PL modules uio drivers locations
 * @{
 */
#define addr_sw 		0x41210000 // switches
#define addr_leds 		0x41200000 // shields leds
/*@}*/

/**
 * @name Constants for user menu
 * @{
 */
#define print_dt 50
#define menu_columnMax 6
#define menu_columnMin 0
#define menu_spMax 0
#define menu_spMin 200
const uint32_t menu_indexColumn[Nmodules] = {
i_P1,
i_P2,
i_I1,
i_I2,
i_C,
i_A,
i_M
};
/*@}*/

#define finger i_I1 // finger that will be moved

/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/

/************************** Function Prototypes *****************************/

void print_data();
void wait();
void float2xil_printf(_real*, uint16_t, int32_t*, int32_t*, int16_t);

/************************** Variable Definitions ****************************/

uint64_t lastTime = 0;
bool menu_flag = false;
uint16_t column = 0; // Current Columnn in menu

bool flag_P2 = false, control_flag = false;
uint8_t wave_select = 0;
//_real voltP2 = 0.0;

uint16_t control = MANUAL_MOTORS;
int32_t whole[Nmodules], hundreds[Nmodules];
_real volt = 3;


handDriver robotHanDler;

/*@}*/

/**
 * Controllers Input/Output Variables
 * @{
 */
_real posIn_f[Nmodules] =
		{0, 0, 0, 0, 0, 0, 0};	// Define Input Variables for filters
_real currIn_f[Nmodules] =
		{0, 0, 0, 0, 0, 0, 0};

_real pos_filt[Nmodules] =
		{0, 0, 0, 0, 0, 0, 0};	// Define Output Variables for filters
_real vel_filt[Nmodules] =
		{0, 0, 0, 0, 0, 0, 0};
_real curr_filt[Nmodules] =
		{0, 0, 0, 0, 0, 0, 0};

_real voltOut[Nmodules] =
		{0, 0, 0, 0, 0, 0, 0};	// Define Variables to connect PID controller

_real imp_sp[Nmodules] =
		{0, 0, 0, 0, 0, 0, 0};	// Variables to connect Impedance controller
_real impOut[Nmodules] =
		{0, 0, 0, 0, 0, 0, 0};
/*@}*/

_real volt_manual[Nmodules] = {0, 0, 0, 0, 0, 0, 0};
_real sine_wave, step_wave, t = 10;
_real delta = 3*2*pi/4;
_real step;
/*****************************************************************************/

int main()
{
	xil_printf("\n\n\rRaw Data Retreiving %s %s", __DATE__, __TIME__);

	for (uint16_t nMod=0; nMod<Nmodules; nMod++){
		voltOut[nMod] = 0.0;
	}

	//robotHanDler.printConfig();
	robotHanDler.set_control(control);
	robotHanDler.set_impFingers(INX_FINGERS);

	xil_printf("\n\r---------------------------------------------");
	//wait();

	while(true){
		if (control_flag == true){
			control_flag = false;
			robotHanDler.shutdown();
			robotHanDler.set_control(control);
		}

		// Hand Execute
		robotHanDler.compute();
		robotHanDler.getSensors(posIn_f, currIn_f);
		robotHanDler.getSenFilters(pos_filt, vel_filt, curr_filt);

		// COntrol Action
		switch (control)
		{
		case MANUAL_MOTORS: // Manual Control
			robotHanDler.set_motorV(volt_manual);
			break;

		case FULL_IMPEDANCE: // Impedance controller
			robotHanDler.set_currSP(imp_sp);
			break;

		default:
			break;
		}

		robotHanDler.getControl(voltOut, imp_sp, impOut);

		print_data();

		if (menu_flag)
			break;

		usleep(5000);
	}

	robotHanDler.shutdown();

	xil_printf("\nEnd");

	return 0;
}

void print_data(){
	uint32_t now = millis() - lastTime;

	//uint32_t w = 9;
	if (print_dt < now){

	_real sp_dt = print_dt*.001;
	_real curr_max = 50;
	_real n = 8;
	_real steps = 8;
	_real m = curr_max/(2*2*n);
	lastTime = millis();
	if (t <= 2*2*n){
		sine_wave = curr_max/2 + (curr_max/2)*sin(t*pi/n + delta);
		if (m * t > (curr_max/steps)*step){
			step = step+1;
			step_wave = (curr_max/steps)*step;
		}
		t = t + sp_dt;
	}else{
		sine_wave = 0.0;
		step_wave = (curr_max/steps);
	}

	if (control == FULL_IMPEDANCE){
		switch(wave_select){
		case 0: imp_sp[finger] = 0.0; break;
		case 1: imp_sp[finger] = sine_wave; break;
		case 2: imp_sp[finger] = step_wave; break;
		default: break;
		}
	}

	if (XUartPs_IsReceiveData(XPAR_PS7_UART_0_BASEADDR)){
		u8 data = XUartPs_ReadReg(XPAR_PS7_UART_0_BASEADDR, XUARTPS_FIFO_OFFSET);
		switch (data){
		case 'w': // extension
			if (control == MANUAL_MOTORS){
				volt_manual[finger] =-3.0;wave_select = 0;
			}
			break; // flexion
		case 's':
			if (control == MANUAL_MOTORS){
				volt_manual[finger] = 3.0;wave_select = 0;
			}
			break;

		case 'd': // start sine wave
			if (control == FULL_IMPEDANCE){
				t = 0; wave_select = 1;
			}
			break;
		case 'f': // start step wave
			if (control == FULL_IMPEDANCE){
				t = 0; wave_select = 2; step = 1;
			}
			break;

		case 'e':
			t = 4;
			break;
		case 'q': menu_flag = true; break;


		case '0': 	control_flag = true;
					control = UNOPERATIONAL;
					wave_select = 0;
					imp_sp[finger] = 0.0;
					step = 1;
					t = 20;
					break;
		case '1': 	control_flag = true;
					control = MANUAL_MOTORS;
					wave_select = 0;
					imp_sp[finger] = 0.0;
					step = 1;
					t = 20;
					break;
		case '3': 	control_flag = true;
					control = FULL_IMPEDANCE;
					t = 20;
					step = 1;
					wave_select = 0;
					break;

		default:
			break;
		}
	}else{
		volt_manual[finger] = 0.0;
	}

		float2xil_printf(pos_filt,Nmodules,whole,hundreds,100);
			xil_printf("\r\e[A\n%6d.%-2d,",whole[finger],hundreds[finger]);
		float2xil_printf(impOut,Nmodules,whole,hundreds,100);
			xil_printf("%6d.%-2d,",whole[finger],hundreds[finger]);
		float2xil_printf(curr_filt,Nmodules,whole,hundreds,100);
			xil_printf("%6d.%-2d,",whole[finger],hundreds[finger]);
		float2xil_printf(imp_sp,Nmodules,whole,hundreds,100);
			xil_printf("%6d.%-2d,",whole[finger],hundreds[finger]);
		float2xil_printf(voltOut,Nmodules,whole,hundreds,100);
			xil_printf("%6d.%-2d|",whole[finger],hundreds[finger]);

		switch (control){
		case UNOPERATIONAL:
			xil_printf("UNOPERATIONAL ");
			break;
		case MANUAL_MOTORS:
			xil_printf("MANUAL_MOTORS ");
			break;
		case FULL_IMPEDANCE:
			xil_printf("FULL_IMPEDANCE");
			break;
		default: break;
		}

		switch (wave_select){
		case 0:
			xil_printf("|    ");
			break;
		case 1:
			xil_printf("|SINE");
			break;
		case 2:
			xil_printf("|STEP");
			break;
		default: break;
		}
	}
}

void wait(){
	while(not(XUartPs_IsReceiveData(XPAR_PS7_UART_0_BASEADDR))){
		usleep(10000);
	}
}

void float2xil_printf(_real* _fval, uint16_t _size,
		int32_t* _whole, int32_t* _decimal, int16_t digits){
	volatile int32_t whole, decimal;
    for (int8_t i=0; i<_size;i++){
    	whole = _fval[i];
		decimal = _real(_fval[i]-(_real)whole)*100;
		decimal = abs(decimal);
		_whole[i] = whole;
    	_decimal[i] = decimal;
	}
}
