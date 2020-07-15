
/***************************** Include Files ********************************/

#include <stdio.h>
//#include <iostream>
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

/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/

/************************** Function Prototypes *****************************/

void print_data();
void print_header();
void float2xil_printf(_real*, uint16_t, int32_t*, int32_t*, int16_t);

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

bool flag_P2 = false, control_flag = false;
//_real voltP2 = 0.0;

int32_t whole[Nmodules], hundreds[Nmodules];
uint16_t control = MANUAL_MOTORS;

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

_real pos_sp[Nmodules] =
		{0, 0, 0, 0, 0, 0, 0};
_real volt_sp[Nmodules] =
		{0, 0, 0, 0, 0, 0, 0};


/*****************************************************************************/
using namespace std;
int main()
{
	printf("\n\n\n\n\n");
	xil_printf("\n\033[3J\r Hand Control PS-PL test for standalone");
	xil_printf("\n\rData Retreiving %s %s", __DATE__, __TIME__);

	// Switches and LEDs initialize
	Xil_Out32(addr_sw + XIL_GPIO_1_TRI_OFFSET, 0x00); // set as output io26-41
	Xil_Out32(addr_leds + XIL_GPIO_2_TRI_OFFSET, 0x00); // set as output led

	unsigned data = Xil_In32(addr_sw + XIL_GPIO_2_OFFSET);
	Xil_Out32(addr_leds + XIL_GPIO_2_OFFSET, data);

	for (uint16_t nMod=0; nMod<Nmodules; nMod++){
		voltOut[nMod] = 0.0;
	}

	robotHanDler.printConfig();
	robotHanDler.set_control(control);

	xil_printf("\n");
	print_header();
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
			robotHanDler.set_motorV(volt_sp);
			break;

		case P_ONLY: // P controller
			robotHanDler.set_posSP(pos_sp);
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

		usleep(1000);
	}

	robotHanDler.shutdown();

	xil_printf("\n \033[1;31m End \033[0m\n");

    return 0;
}

/*********************************** Function  ***********************************/
void print_header(){
	printf("    |                             Robotic Hand                            |\n" );
	printf("    | Thumb1  |  Thumb2 |  Index1 |  Index2 |  Middle |   Ring  | Little  |\n" );
	printf("    -----------------------------------------------------------------------\n" );
	//printf("\n\n\n\n\n");
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
						pos_sp[menu_indexColumn[(uint32_t)column]]++;
						volt_sp[menu_indexColumn[(uint32_t)column]] =-3.0;

					break;
                case 's':
                		pos_sp[menu_indexColumn[(uint32_t)column]]--;
                		volt_sp[menu_indexColumn[(uint32_t)column]] = 3.0;

					break;
                case 'q':menu_flag = true; break;

                case '0': control_flag = true; control = UNOPERATIONAL;
                			imp_sp[6] = 0.0; imp_sp[3] = 0.0; imp_sp[4] = 0.0;break;
                case '1': control_flag = true; control = MANUAL_MOTORS;
                			imp_sp[6] = 0.0; imp_sp[3] = 0.0; imp_sp[4] = 0.0;break;
                case '2': control_flag = true; control = P_ONLY;
                			imp_sp[6] = 0.0; imp_sp[3] = 0.0; imp_sp[4] = 0.0;
                			for (uint16_t i=0; i<Nmodules; i++){
                				pos_sp[i] = impOut[i];
                			}
                			break;
                case '3': control_flag = true; control = FULL_IMPEDANCE; break;

                case 'i': // initial position
                	if (control == P_ONLY){
                	pos_sp[6] = 29;	// P1 6
                	pos_sp[0] = 15;	// P2 0
                	pos_sp[3] = 16;	// I1 3
					pos_sp[4] = 10;	// C  4
                	pos_sp[5] = 15;	// A  5
					pos_sp[1] = 15;	// M  1
                	}
					break;

                case 'p': // pinch position
                	if (control == P_ONLY){
                	pos_sp[6] = 140;	// P1 6
                	pos_sp[0] = 51;		// P2 0
                	pos_sp[3] = 190;	// I1 3
					pos_sp[4] = 20;		// C  4
                	pos_sp[5] = 15;		// A  5
					pos_sp[1] = 26;		// M  1
                	}
					break;

                case 'o': // sphere position
                	if (control == P_ONLY){
					pos_sp[6] = 27;		// P1 6
                	pos_sp[0] = 129;	// P2 0
                	pos_sp[3] = 115;		// I1 3
					pos_sp[4] = 84;		// C  4
                	pos_sp[5] = 83;		// A  5
					pos_sp[1] = 161;	// M  1
                	}
					break;

                case 'P': // Disc Grasp
                	if (control == P_ONLY){
					pos_sp[6] = 155;	// P1 6
                	pos_sp[0] = 146;	// P2 0
                	pos_sp[3] = 97;		// I1 3
					pos_sp[4] = 67;		// C  4
                	pos_sp[5] = 64;		// A  5
					pos_sp[1] = 83;		// M  1
                	}
					break;

                case 'j': // S cillinder
                	if (control == P_ONLY){
					pos_sp[6] = 120;	// P1 6
                	pos_sp[0] = 122;	// P2 0
                	pos_sp[3] = 154;	// I1 3
					pos_sp[4] = 159;	// C  4
                	pos_sp[5] = 154;	// A  5
					pos_sp[1] = 213;	// M  1
                	}
                	break;

                case 'l': // Big cillinder
                	if (control == P_ONLY){
					pos_sp[6] = 25;		// P1 6
                	pos_sp[0] = 122;	// P2 0
                	pos_sp[3] = 67;		// I1 3
					pos_sp[4] = 83;		// C  4
                	pos_sp[5] = 94;		// A  5
					pos_sp[1] = 129;	// M  1
                	}
                	break;

                case 'k': // Medium cillinder
                	if (control == P_ONLY){
                	pos_sp[6] = 24;		// P1 6
                	pos_sp[0] = 142;	// P2 0
                	pos_sp[3] = 120;	// I1 3
					pos_sp[4] = 118;	// C  4
                	pos_sp[5] = 128;	// A  5
					pos_sp[1] = 148;	// M  1
                	}
                	if (control == FULL_IMPEDANCE){
					imp_sp[6] = 50;		// P1 6
					imp_sp[3] = 60;		// I1 3
					imp_sp[4] = 50;		// C  4
                	}
					break;

                case 'O': // sphere presicion
                	if (control == P_ONLY){
                	pos_sp[6] = 37;		// P1 6
                	pos_sp[0] = 130;	// P2 0
                	pos_sp[3] = 92;		// I1 3
					pos_sp[4] = 64;		// C  4
                	pos_sp[5] = 71;		// A  5
					pos_sp[1] = 107;	// M  1
                	}
					break;

                case 'h': // Tripod
                	if (control == P_ONLY){
                	pos_sp[6] = 70;		// P1 6
                	pos_sp[0] = 111;	// P2 0
                	pos_sp[3] = 139;	// I1 3
					pos_sp[4] = 154;	// C  4
                	pos_sp[5] = 15;		// A  5
					pos_sp[1] = 26;		// M  1
                	}
					break;

                case 'b': // Tool 1
                	if (control == P_ONLY){
                	pos_sp[6] = 89;		// P1 6
                	pos_sp[0] = 126;	// P2 0
                	pos_sp[3] = 123;	// I1 3
					pos_sp[4] = 105;	// C  4
                	pos_sp[5] = 130;	// A  5
					pos_sp[1] = 144;	// M  1
                	}
					break;

                case 'n': // Tool 2
                	if (control == P_ONLY){
                	pos_sp[6] = 83;		// P1 6
                	pos_sp[0] = 112;	// P2 0
                	pos_sp[3] = 145;	// I1 3
					pos_sp[4] = 140;	// C  4
                	pos_sp[5] = 24;		// A  5
					pos_sp[1] = 29;		// M  1
                	}
					break;

                case 'm': // Tool 3
                	if (control == P_ONLY){
                	pos_sp[6] = 89;		// P1 6
                	pos_sp[0] = 97;		// P2 0
                	pos_sp[3] = 126;	// I1 3
					pos_sp[4] = 29;		// C  4
                	pos_sp[5] = 25;		// A  5
					pos_sp[1] = 29;		// M  1
                	}
					break;

				default:
                    break;
            }
                
            if (column > menu_columnMax) column = menu_columnMax;
            if (column < menu_columnMin) column = menu_columnMin;
        }else{
			// teste @{
			for (uint8_t j=0; j<Nmodules; j++){
				volt_sp[j]=	0;
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
        //xil_printf("\r\e[A\e[A\e[A\e[A\e[A%s", menu_text);
        xil_printf("%s", menu_text);
		switch (control){
		case UNOPERATIONAL:
			xil_printf(" UNOPERATIONAL ");
			break;
		case MANUAL_MOTORS:
			xil_printf(" MANUAL_MOTORS ");
			break;
		case P_ONLY:
			xil_printf(" P_ONLY        ");
			break;
		case FULL_IMPEDANCE:
			xil_printf(" FULL_IMPEDANCE");
			break;
		default: break;
		}
		xil_printf("\r\n");
        
        float2xil_printf(impOut,Nmodules,whole,hundreds,100);
			xil_printf("setP|%6d.%-2d|",whole[i_P1],hundreds[i_P1]);
			xil_printf("%6d.%-2d|",whole[i_P2],hundreds[i_P2]);
			xil_printf("%6d.%-2d|",whole[i_I1],hundreds[i_I1]);
			xil_printf("%6d.%-2d|",whole[i_I2],hundreds[i_I2]);
			xil_printf("%6d.%-2d|",whole[i_C],hundreds[i_C]);
			xil_printf("%6d.%-2d|",whole[i_A],hundreds[i_A]);
			xil_printf("%6d.%-2d|",whole[i_M],hundreds[i_M]);
			xil_printf("\r\n");

        float2xil_printf(pos_filt,Nmodules,whole,hundreds,100);
			xil_printf("Pos |%6d.%-2d|",whole[i_P1],hundreds[i_P1]);
			xil_printf("%6d.%-2d|",whole[i_P2],hundreds[i_P2]);
			xil_printf("%6d.%-2d|",whole[i_I1],hundreds[i_I1]);
			xil_printf("%6d.%-2d|",whole[i_I2],hundreds[i_I2]);
			xil_printf("%6d.%-2d|",whole[i_C],hundreds[i_C]);
			xil_printf("%6d.%-2d|",whole[i_A],hundreds[i_A]);
			xil_printf("%6d.%-2d|",whole[i_M],hundreds[i_M]);
			xil_printf("\r\n");

        float2xil_printf(voltOut,Nmodules,whole,hundreds,100);
			xil_printf("Volt|%6d.%-2d|",whole[i_P1],hundreds[i_P1]);
			xil_printf("%6d.%-2d|",whole[i_P2],hundreds[i_P2]);
			xil_printf("%6d.%-2d|",whole[i_I1],hundreds[i_I1]);
			xil_printf("%6d.%-2d|",whole[i_I2],hundreds[i_I2]);
			xil_printf("%6d.%-2d|",whole[i_C],hundreds[i_C]);
			xil_printf("%6d.%-2d|",whole[i_A],hundreds[i_A]);
			xil_printf("%6d.%-2d|",whole[i_M],hundreds[i_M]);
			xil_printf("\r\n");

		float2xil_printf(curr_filt,Nmodules,whole,hundreds,100);
			xil_printf("Curr|%6d.%-2d|",whole[i_P1],hundreds[i_P1]);
			xil_printf("         |");
			xil_printf("%6d.%-2d|",whole[i_I1],hundreds[i_I1]);
			xil_printf("         |");
			xil_printf("%6d.%-2d|",whole[i_C],hundreds[i_C]);
			xil_printf("         |");
			xil_printf("         |");
			xil_printf("\r\n");

		float2xil_printf(imp_sp,Nmodules,whole,hundreds,100);
			xil_printf("setC|%6d.%-2d|",whole[i_P1],hundreds[i_P1]);
			xil_printf("         |");
			xil_printf("%6d.%-2d|",whole[i_I1],hundreds[i_I1]);
			xil_printf("         |");
			xil_printf("%6d.%-2d|",whole[i_C],hundreds[i_C]);
			xil_printf("         |");
			xil_printf("         |");
			xil_printf("\r\n");

		xil_printf("\e[A\e[A\e[A\e[A\e[A\e[A");

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
