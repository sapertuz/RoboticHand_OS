/***************************** Include Files *******************************/
#include "handDriver.hpp"

/************************** Function Definitions ***************************/ 

handDriver::handDriver()
{
	initialize();
}

int handDriver::initialize()
{
	/*
	 * Initialize the XAdc driver.
	 * {
	 */
	XAdcPs_Config *ConfigPtr;
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
    
	motorDriver motorsHandler(addr_motorMod);
    motorsHandler.disableMotors();
    motorsHandler_p = &motorsHandler;
	
	/**
	 * Initialize sensorMod
	 * {
	 */
	// Initialize SensorMod
    sensorMod sensorHandler(addr_sensorMod,
							posIn_f, currIn_f, voltOut,
							pos_filt, vel_filt, curr_filt);
	const _real ab[2] = {a, b};
	sensorHandler.set_Kalman(dt, R, nQ);
    sensorHandler.set_butter(ab);
    sensorHandler.set_2ADC(DEG_ADC, mA_ADC);
    // Calibrate Current Sensors
	for (uint32_t j = 0; j < 100; j++){
		update_sensors();
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
    sensorHandler_p = &sensorHandler;
    /*}*/

	/**
	 * Initialize PID
	 * {
	 */
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
	init_impOut(); // start sp for position control
	/*}*/

	// Init Variables
	for (uint32_t j=0; j<Nmodules; j++){
		voltOut[j]=	0;
		posIn_f[j]=	0;
		currIn_f[j]=0;
	}

	return XST_SUCCESS;

}


void handDriver::linkSensors(_real *_posIn_f,_real *_currIn_f)
{
	_posIn_f 	= posIn_f;
	_currIn_f 	= currIn_f ;
}

void handDriver::linkSenFilters(_real *_pos_filt, _real *_vel_filt, _real *_curr_filt)
{
	_pos_filt	= pos_filt;
	_vel_filt	= vel_filt;
	_curr_filt	= curr_filt;
}

void handDriver::linkControl(_real *_voltOut, _real *_imp_sp,_real *_impOut)
{
	_voltOut	= voltOut;
	_imp_sp		= imp_sp;
	_impOut		= impOut;
}

void handDriver::compute(){

    // Sensors Update and filtering
    update_sensors();
    sensorHandler_p->start();

    // COntrol Action
    /*
    for (uint32_t j=0; j<Nmodules; j++){
        pidControllers[j].Compute();
    }
    */
    // Control Execution

    for (uint32_t nMod=0; nMod<Nmodules; nMod++){
        //if(nMod != pwmCh_P2){
            motorsHandler_p->setMotor(nMod, voltOut[nMod]);
        //}else{
        //    motorsHandler_p->setMotor(nMod, voltP2);
        //}
    }

}

void handDriver::shutdown(){
	motorsHandler_p->disableMotors();
}


void handDriver::update_sensors(){
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

void handDriver::init_impOut(){
	impOut[i_P1]=	(_real)XAdcPs_GetAdcData(XAdcInstPtr,anPos_P1)*DEG_ADC; // 3.3v
	impOut[i_I1]=	(_real)XAdcPs_GetAdcData(XAdcInstPtr,anPos_I1)*DEG_ADC; // 3.3v
	impOut[i_C]=	(_real)XAdcPs_GetAdcData(XAdcInstPtr,anPos_C) *DEG_ADC; // 3.3v
	impOut[i_A]=	(_real)XAdcPs_GetAdcData(XAdcInstPtr,anPos_A) *DEG_ADC; // 3.3v
	impOut[i_M]=	(_real)XAdcPs_GetAdcData(XAdcInstPtr,anPos_M) *DEG_ADC; // 3.3v
	impOut[i_P2]=	(_real)XAdcPs_GetAdcData(XAdcInstPtr,anPos_P2)*DEG_ADC; // 3.3v
}
