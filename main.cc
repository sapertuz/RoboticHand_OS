
/***************************** Include Files ********************************/

#include <stdio.h>
//#include <iostream>
//#include <iomanip>

#include "SensorMod.hpp"
#include "PID_v1.h"
#include "MotorDriver.hpp"

#include "XTime_Meas.h"
#include "xuartps.h"
//#include "keyboard_utils.h"

#include "definitions.h"
#include "xil_io.h"
#include "xadcps.h"

/************************** Constant Definitions ****************************/

#define XIL_SIZE				0x10000

#define XIL_GPIO_1_OFFSET		0x0
#define XIL_GPIO_1_TRI_OFFSET	0x4
#define XIL_GPIO_2_OFFSET		0x8
#define XIL_GPIO_2_TRI_OFFSET	0xC


/**
 * PL modules uio drivers locations
 * @{
 */
#define addr_motorMod 	0x43C10000 // motor mod
#define addr_sensorMod  0x43C20000 // sensor mod
#define addr_sw 		0x41210000 // switches
#define addr_leds 		0x41200000 // shields leds
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
#define R 	4e-3
#define nQ 	10000
#define a	-0.9450
#define b 	0.0275
#define cenCurr	22500.0
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

/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/

/************************** Function Prototypes *****************************/

void print_data();
void print_header();
void float2xil_printf(_real*, uint16_t, int32_t*, int32_t*, int16_t);

void update_sensors(XAdcPs* );

void init_impOut(XAdcPs* );

/************************** Variable Definitions ****************************/

/**
 * Menu variables
 * @{
 */
uint64_t lastTime = 0;
char menu_text0[] = "    |   ***   |         |         |         |         |         |         |";
char menu_text1[] = "    |         |   ***   |         |         |         |         |         |";
char menu_text2[] = "    |         |         |   ***   |         |         |         |         |";
char menu_text3[] = "    |         |         |         |   ***   |         |         |         |";
char menu_text4[] = "    |         |         |         |         |   ***   |         |         |";
char menu_text5[] = "    |         |         |         |         |         |   ***   |         |";
char menu_text6[] = "    |         |         |         |         |         |         |   ***   |";
char menu_textDefault[] = "    |         |         |         |         |         |         |         |";
char *menu_text;
bool menu_flag = false;
uint16_t column = 0; // Current Columnn in menu

bool flag_P2 = false;
_real voltP2 = 0.0;

int32_t whole[Nmodules], hundreds[Nmodules];

/*@}*/

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

static XAdcPs XAdcInst;      /* XADC driver instance */

/*****************************************************************************/
using namespace std;
int main()
{
    xil_printf("\033[3J Hand Control PS-PL test for standalone");

	/*
	 * Initialize the XAdc driver.
	 * {
	 */
	XAdcPs_Config *ConfigPtr;
	XAdcPs *XAdcInstPtr = &XAdcInst;
	int Status;

	ConfigPtr = XAdcPs_LookupConfig(XPAR_XADCPS_0_DEVICE_ID);
	if (ConfigPtr == NULL) {
		return XST_FAILURE;
	}
	XAdcPs_CfgInitialize(XAdcInstPtr, ConfigPtr,
				ConfigPtr->BaseAddress);
	/*
	 * Self Test the XADC/ADC device
	 */
	Status = XAdcPs_SelfTest(XAdcInstPtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	/*
	 * Enable the Channel Sequencer before configuring the Sequence
	 * registers.
	 */
	XAdcPs_SetSequencerMode(XAdcInstPtr, XADCPS_SEQ_MODE_CONTINPASS);
	/*}*/

    /**
	 * Create handlers for MotorDriver and set
	 * {
	 */
    motorDriver motorsHandler(addr_motorMod);

	Xil_Out32(addr_sw + XIL_GPIO_1_TRI_OFFSET, 0x00); // set as output io26-41
	Xil_Out32(addr_leds + XIL_GPIO_2_TRI_OFFSET, 0x00); // set as output led

	unsigned data = Xil_In32(addr_sw + XIL_GPIO_2_OFFSET);
	Xil_Out32(addr_leds + XIL_GPIO_2_OFFSET, data);
	motorsHandler.disableMotors();
	/*}*/

	/**
	 * Create handlers for sensorMod and Initialize
	 * {
	 */
	sensorMod sensorHandler(addr_sensorMod,
			posIn_f, currIn_f, voltOut,
    		pos_filt, vel_filt, curr_filt);
	// Initialize SensorMod
	const _real ab[2] = {a, b};
	sensorHandler.set_Kalman(dt, R, nQ);
    sensorHandler.set_butter(ab);
    sensorHandler.set_2ADC(DEG_ADC, mA_ADC);
    // Calibrate Current Sensors
	for (uint32_t j = 0; j < 100; j++){
		update_sensors(XAdcInstPtr);
		cenCurr_f[i_P1] += currIn_f[i_P1]*0.01;
		cenCurr_f[i_C] += currIn_f[i_C]*0.01;
		cenCurr_f[i_I1] += currIn_f[i_I1]*0.01;
		usleep(10000);
	}
	cenCurr_f[i_P2]=0.0;cenCurr_f[i_I2]=0.0;
	cenCurr_f[i_M]=0.0;cenCurr_f[i_A] = 0.0;
	for (uint32_t i = 0; i < Nmodules; i++)
		sensorHandler.set_centerCurr(cenCurr_f[i],i);
	// Calibrate start value of position sensors
	posStart_f[i_P1] = 10200; posStart_f[i_P2] = 13700;
	posStart_f[i_I1] = 7900; posStart_f[i_C] = 10500;
	posStart_f[i_A] = 8900; posStart_f[i_M] = 10200;
	for (uint32_t i = 0; i < Nmodules; i++)
		sensorHandler.set_positionStart(posStart_f[i],i);

    sensorHandler.printConfig();
    /*}*/

	/**
	 * Create Proportional controllers objects and Initialize
	 * {
	 */
	PID pidControllers[Nmodules];
	for(uint32_t nMod=0; nMod<Nmodules; nMod++){
		pidControllers[nMod].Configure(&pos_filt[nMod], &voltOut[nMod], &impOut[nMod], 
			P, 0.0, 0.0, P_ON_E, REVERSE);
		pidControllers[nMod].SetSampleTime(dt);
		pidControllers[nMod].SetOutputLimits(outMin, outMax);
	}
	/*}*/


	/**
	 * Start SensorMod Handler and Controllers
	 * {
	 */
	for(uint32_t nMod=0; nMod<Nmodules; nMod++){
		pidControllers[nMod].SetMode(AUTOMATIC);		
	}
	init_impOut(XAdcInstPtr); // start sp for position control
	/*}*/

	// Init Variables
	for (uint32_t j=0; j<Nmodules; j++){
		voltOut[j]=	0;
		posIn_f[j]=	0;
		currIn_f[j]=0;
	}

	//xil_printf("\n");
	print_header();
	
	while(true){
		// Sensors Update and filtering
		update_sensors(XAdcInstPtr);
		sensorHandler.start();

		// COntrol Action
		/*
		for (uint32_t j=0; j<Nmodules; j++){
			pidControllers[j].Compute();
		}
		*/
		// Control Execution

		for (uint32_t nMod=0; nMod<Nmodules; nMod++){
			if(nMod != pwmCh_P2){
				motorsHandler.setMotor(nMod, voltOut[nMod]);
			}else{
				motorsHandler.setMotor(nMod, voltP2);
			}
		}


		print_data();

		if (menu_flag)
			break;

		usleep(1000);
	}

	motorsHandler.disableMotors();
	/*
	delete &sensorHandler;
	delete &gpioSw_handler;
	delete &gpioLED_handler;
	delete &motorsHandler;
	*/
	xil_printf("\n \033[1;31m End \033[0m\n");

    return 0;
}

/*********************************** Function  ***********************************/
void update_sensors(XAdcPs* XAdcInstPtr){
	currIn_f[i_P1]= (_real)XAdcPs_GetAdcData(XAdcInstPtr,anCurr_P1);
 	currIn_f[i_C]= 	(_real)XAdcPs_GetAdcData(XAdcInstPtr,anCurr_C);
 	currIn_f[i_I1]= (_real)XAdcPs_GetAdcData(XAdcInstPtr,anCurr_I1);

	posIn_f[i_P1]=	(_real)XAdcPs_GetAdcData(XAdcInstPtr,anPos_P1);// 3.3v
	posIn_f[i_I1]=	(_real)XAdcPs_GetAdcData(XAdcInstPtr,anPos_I1);// 3.3v
	posIn_f[i_C]=	(_real)XAdcPs_GetAdcData(XAdcInstPtr,anPos_C); // 3.3v
	posIn_f[i_A]=	(_real)XAdcPs_GetAdcData(XAdcInstPtr,anPos_A); // 3.3v
	posIn_f[i_M]=	(_real)XAdcPs_GetAdcData(XAdcInstPtr,anPos_M); // 3.3v
	posIn_f[i_P2]=	(_real)XAdcPs_GetAdcData(XAdcInstPtr,anPos_P2);// 3.3v
}

void init_impOut(XAdcPs* XAdcInstPtr){
	impOut[i_P1]=	(_real)XAdcPs_GetAdcData(XAdcInstPtr,anPos_P1)*DEG_ADC; // 3.3v
	impOut[i_I1]=	(_real)XAdcPs_GetAdcData(XAdcInstPtr,anPos_I1)*DEG_ADC; // 3.3v
	impOut[i_C]=	(_real)XAdcPs_GetAdcData(XAdcInstPtr,anPos_C) *DEG_ADC; // 3.3v
	impOut[i_A]=	(_real)XAdcPs_GetAdcData(XAdcInstPtr,anPos_A) *DEG_ADC; // 3.3v
	impOut[i_M]=	(_real)XAdcPs_GetAdcData(XAdcInstPtr,anPos_M) *DEG_ADC; // 3.3v
	impOut[i_P2]=	(_real)XAdcPs_GetAdcData(XAdcInstPtr,anPos_P2)*DEG_ADC; // 3.3v
}


void print_header(){
	printf  ("    |                             Robotic Hand                            |" );
	printf("\n    | Thumb1  |  Thumb2 |  Index1 |  Index2 |  Middle |   Ring  | Little  |" );
	printf("\n    -----------------------------------------------------------------------" );
	printf("\n\n\n\n\n");
}

void print_data(){
	uint32_t now = millis() - lastTime;
	//uint32_t w = 9;
	if (print_dt < now){
		//cout.setf(ios::fixed, ios::floatfield);
    	//cout << setprecision(2);
		// A key was pressed
        if (XUartPs_IsReceiveData(XPAR_PS7_UART_0_BASEADDR)){
    	//if (0){
        	u8 data = XUartPs_ReadReg(XPAR_PS7_UART_0_BASEADDR, XUARTPS_FIFO_OFFSET);
            switch (data){
                case 'd':column++; break;
                case 'a':column--; break;
                case 'w':
					if (menu_indexColumn[(uint32_t)column] != i_P2){
						impOut[menu_indexColumn[(uint32_t)column]]++; 

						// teste @{
						voltOut[menu_indexColumn[(uint32_t)column]] =-2.0;
						//}
					}else{
						voltP2 = -4.0;
						flag_P2 = true;
					}
					break;
                case 's':
					if (menu_indexColumn[(uint32_t)column] != i_P2){
						impOut[menu_indexColumn[(uint32_t)column]]--; 

						// teste @{
						voltOut[menu_indexColumn[(uint32_t)column]] = 2.0;
						//}
					}else{
						voltP2 = 4.0;
						flag_P2 = true;
					}
					break;
                case 'q':menu_flag = true; break;
				case 'i': // Init Point
					impOut[pwmCh_P1] = 44.2;
					impOut[pwmCh_I1] = 56.13;
					impOut[pwmCh_C] = 48.8;
					impOut[pwmCh_A] = 63;
					impOut[pwmCh_M] = 85;
					break;
                
                case 'c': // Grab bottle
					impOut[pwmCh_P1] = 92.2;
					impOut[pwmCh_I1] = 96.13;
					impOut[pwmCh_C] = 98.80;
					impOut[pwmCh_A] = 130.95;
					impOut[pwmCh_M] = 130;
					break;
                
                case 'b': // Grab ballb
					impOut[pwmCh_P1] = 72;
					impOut[pwmCh_I1] = 116;
					impOut[pwmCh_C] = 101;
					impOut[pwmCh_A] = 119;
					impOut[pwmCh_M] = 125;
					break;

				default:
                    break;
            }
                
            if (column > menu_columnMax) column = menu_columnMax;
            if (column < menu_columnMin) column = menu_columnMin;
        }else{
			voltP2 = 0.0;
			flag_P2 = false;

			// teste @{
			for (uint8_t j=0; j<Nmodules; j++){
				voltOut[j]=	0;
			}
			//}
		}
        // wich column is selected
        switch (column){
            case 0: menu_text = menu_text0; break;
            case 1: menu_text = menu_text1; break;
            case 2: menu_text = menu_text2; break;
            case 3: menu_text = menu_text3; break;
            case 4: menu_text = menu_text4; break;
            case 5: menu_text = menu_text5; break;
            case 6: menu_text = menu_text6; break;
        
            default: menu_text = menu_textDefault; break;
        }

        
        //cout << "\r\e[A\e[A\e[A\e[A" << menu_text << endl;
        xil_printf("\r\e[A\e[A\e[A\e[A%s\n", menu_text);
        
        float2xil_printf(posIn_f,Nmodules,whole,hundreds,100);
			xil_printf("\rRawP|%6d.%2d|",whole[i_P1],hundreds[i_P1]);
			xil_printf("%6d.%2d|",whole[i_P2],hundreds[i_P2]);
			xil_printf("%6d.%2d|",whole[i_I1],hundreds[i_I1]);
			xil_printf("%6d.%2d|",whole[i_I2],hundreds[i_I2]);
			xil_printf("%6d.%2d|",whole[i_C],hundreds[i_C]);
			xil_printf("%6d.%2d|",whole[i_A],hundreds[i_A]);
			xil_printf("%6d.%2d|",whole[i_M],hundreds[i_M]);
			xil_printf("\n");

        float2xil_printf(pos_filt,Nmodules,whole,hundreds,100);
			xil_printf("\rPos |%6d.%2d|",whole[i_P1],hundreds[i_P1]);
			xil_printf("%6d.%2d|",whole[i_P2],hundreds[i_P2]);
			xil_printf("%6d.%2d|",whole[i_I1],hundreds[i_I1]);
			xil_printf("%6d.%2d|",whole[i_I2],hundreds[i_I2]);
			xil_printf("%6d.%2d|",whole[i_C],hundreds[i_C]);
			xil_printf("%6d.%2d|",whole[i_A],hundreds[i_A]);
			xil_printf("%6d.%2d|",whole[i_M],hundreds[i_M]);
			xil_printf("\n");
/*
        float2xil_printf(voltOut,Nmodules,whole,hundreds,100);
			xil_printf("\rVolt|%6d.%2d|",whole[i_P1],hundreds[i_P1]);
			xil_printf("%6d.%2d|",whole[i_P2],hundreds[i_P2]);
			xil_printf("%6d.%2d|",whole[i_I1],hundreds[i_I1]);
			xil_printf("%6d.%2d|",whole[i_I2],hundreds[i_I2]);
			xil_printf("%6d.%2d|",whole[i_C],hundreds[i_C]);
			xil_printf("%6d.%2d|",whole[i_A],hundreds[i_A]);
			xil_printf("%6d.%2d|",whole[i_M],hundreds[i_M]);
			xil_printf("\n");
*/
		float2xil_printf(currIn_f,Nmodules,whole,hundreds,100);
			xil_printf("\rRawC|%6d.%2d|",whole[i_P1],hundreds[i_P1]);

			//xil_printf("         |");
			xil_printf("%6d.%2d|",whole[i_P2],hundreds[i_P2]);

			xil_printf("%6d.%2d|",whole[i_I1],hundreds[i_I1]);

			//xil_printf("         |");
			xil_printf("%6d.%2d|",whole[i_I2],hundreds[i_I2]);

			xil_printf("%6d.%2d|",whole[i_C],hundreds[i_C]);

			//xil_printf("         |");
			//xil_printf("         |");
			xil_printf("%6d.%2d|",whole[i_A],hundreds[i_A]);
			xil_printf("%6d.%2d|",whole[i_M],hundreds[i_M]);

			xil_printf("\n");

		float2xil_printf(curr_filt,Nmodules,whole,hundreds,100);
			xil_printf("\rCurr|%6d.%2d|",whole[i_P1],hundreds[i_P1]);

			xil_printf("         |");
			xil_printf("%6d.%2d|",whole[i_I1],hundreds[i_I1]);

			_real curr_test = (currIn_f[i_I1] - cenCurr_f[i_I1])*mA_ADC;
			int32_t entero, resto;
			float2xil_printf(&curr_test, 1, &entero, &resto, 100);
			xil_printf("%6d.%2d|",entero,resto);

			//xil_printf("         |");
			xil_printf("%6d.%2d|",whole[i_C],hundreds[i_C]);
			xil_printf("         |");
			xil_printf("         |");

		lastTime = millis();
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
