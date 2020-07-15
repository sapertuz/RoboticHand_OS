/***************************** Include Files *******************************/
#include "handDriver.hpp"

/************************** Function Definitions ***************************/ 

handDriver::handDriver()
{
	set_control(0); // set control off
	initialize();
}

int handDriver::initialize()
{
	/*
	 * Initialize the XAdc driver.
	 * {
	 */
	int Status;

	ConfigPtr = XAdcPs_LookupConfig(XPAR_XADCPS_0_DEVICE_ID);
	if (ConfigPtr == NULL) {
		return XST_FAILURE;
	}
	XAdcPs_CfgInitialize(this->XAdcInstPtr, ConfigPtr,
				ConfigPtr->BaseAddress);
	/*
	 * Self Test the XADC/ADC device
	 */
	Status = XAdcPs_SelfTest(this->XAdcInstPtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	/*
	 * Enable the Channel Sequencer before configuring the Sequence
	 * registers.
	 */
	XAdcPs_SetSequencerMode(this->XAdcInstPtr, XADCPS_SEQ_MODE_CONTINPASS);
	/*}*/
    
	//motorDriver motorsHandler;
	this->motorsHandler.setAddress(addr_motorMod);
	this->motorsHandler.disableMotors();
    //motorsHandler_p = &motorsHandler;
	
	/**
	 * Initialize sensorMod
	 * {
	 */
	// Initialize SensorMod
    //sensorMod sensorHandler;
    this->sensorHandler.setAddress(addr_sensorMod);
	this->sensorHandler.setDataMonitor(posIn_f, currIn_f, voltOut,
								pos_filt, vel_filt, curr_filt);
	const _real ab[2] = {a, b};
	this->sensorHandler.set_Kalman(DT, R, nQ);
	this->sensorHandler.set_butter(ab);
    this->sensorHandler.set_2ADC(DEG_ADC, mA_ADC);
	this->calibrateCurrentS();
	this->calibratePositionS(posStart_f);
	/*}*/

	for (uint16_t i=0; i<10; i++ ){
	    // Sensors Update and filtering
		this->update_sensors();
		this->sensorHandler.start();
		usleep(5000);
	}

	/**
	 * Initialize Impedance
	 * {
	 */
	for(uint32_t nMod=0; nMod<Nmodules; nMod++){
		this->ImpControllers[nMod].Configure(&this->curr_filt[nMod],
			&this->pos_filt[nMod], &this->vel_filt[nMod],
			&this->impOut[nMod], &this->imp_sp[nMod], M, B, K, DIRECT);
		this->ImpControllers[nMod].SetSampleTime(DT*10);
	}
	/*}*/

	/**
	 * Initialize PID
	 * {
	 */
	for(uint32_t nMod=0; nMod<Nmodules; nMod++){
		this->pidControllers[nMod].Configure(
				&this->pos_filt[nMod], &this->voltOut[nMod], &this->impOut[nMod],
				P, 0.0, 0.0, P_ON_E, DIRECT);
		this->pidControllers[nMod].SetSampleTime(DT*10);
		this->pidControllers[nMod].SetOutputLimits(outMin, outMax);
	}
	/*}*/

	this->init_controllers();

	this->imp_index = TRI_FINGERS;

	return XST_SUCCESS;
}

void handDriver::init_controllers(){
	/**
	 * Start SensorMod Handler and Controllers
	 * {
	 */
	for(uint32_t nMod=0; nMod<Nmodules; nMod++){
		this->ImpControllers[nMod].SetMode(AUTOMATIC);
		this->pidControllers[nMod].SetMode(AUTOMATIC);
	}
	this->init_impOut(); // start sp for position control
	/*}*/

	// Init Variables
	for (uint32_t j=0; j<Nmodules; j++){
		this->voltOut[j]= 0;
		this->imp_sp[j] = 0;
	}

}

void handDriver::set_control(uint8_t _control_t)
{
	control_t = _control_t;
	this->shutdown();
	for(uint32_t nMod=0; nMod<Nmodules; nMod++){
		this->ImpControllers[nMod].Restart();
		this->imp_sp[nMod] = 0.0;
	}
}

void handDriver::calibrateCurrentS()
{
	uint16_t nData = 10; // 100 Data for calibrate
	uint16_t calibrate_delay = 10000; // 10 ms between each measure
	uint32_t cenP1_acum=0, cenC_acum=0, cenI1_acum=0;
	this->shutdown();

	// Calibrate Current Sensors
	for (uint32_t j = 0; j < nData; j++){
		//this->update_sensors();
		cenP1_acum += XAdcPs_GetAdcData(XAdcInstPtr,anCurr_P1);
		cenC_acum +=  XAdcPs_GetAdcData(XAdcInstPtr,anCurr_C);
		cenI1_acum += XAdcPs_GetAdcData(XAdcInstPtr,anCurr_I1);
		usleep(calibrate_delay);
	}

	this->cenCurr_f[i_P1] += (_real)cenP1_acum/(_real)nData;
	this->cenCurr_f[i_C] +=  (_real)cenC_acum /(_real)nData; 
	this->cenCurr_f[i_I1] += (_real)cenI1_acum/(_real)nData; 

	for (uint32_t i = 0; i < Nmodules; i++)
		this->sensorHandler.set_centerCurr(cenCurr_f[i],i);
}

void handDriver::calibratePositionS(_real * poStart){
	// Calibrate start value of position sensors
	for (uint32_t i = 0; i < Nmodules; i++)
		this->sensorHandler.set_positionStart(poStart[i],i);    
}

void handDriver::getSensors(_real *_posIn_f,_real *_currIn_f)
{
	for (uint16_t i=0; i<Nmodules; i++){
		_posIn_f[i] =	this->posIn_f[i];
		_currIn_f[i] =	this->currIn_f[i];
	}
}

void handDriver::getSenFilters(_real *_pos_filt, _real *_vel_filt, _real *_curr_filt)
{
	for (uint16_t i=0; i<Nmodules; i++){
		_pos_filt[i]  = this->pos_filt[i] ;
		_vel_filt[i]  = this->vel_filt[i] ;
		_curr_filt[i] = this->curr_filt[i];
	}
}

void handDriver::getControl(_real *_voltOut, _real *_imp_sp,_real *_impOut)
{
	for (uint16_t i=0; i<Nmodules; i++){
		_voltOut[i] = this->voltOut[i];
		_imp_sp[i]  = this->imp_sp[i] ;
		_impOut[i]  = this->impOut[i] ;
	}
}

void handDriver::compute(){

    // Sensors Update and filtering
	this->update_sensors();
	this->sensorHandler.start();

    // COntrol Action
    switch (control_t)
	{
	case UNOPERATIONAL:
		this->shutdown();
		break;

	case MANUAL_MOTORS: // Manual Control
		for (uint32_t nMod=0; nMod<Nmodules; nMod++){
			this->motorsHandler.setMotor(nMod, voltOut[nMod]);
		}
		break;
	
	case P_ONLY: // P controller
		for (uint32_t nMod=0; nMod<Nmodules; nMod++){
			this->pidControllers[nMod].Compute();
        	this->motorsHandler.setMotor(nMod, voltOut[nMod]);
		}
		break;

	case FULL_IMPEDANCE: // Impedance controller
		for (uint32_t nMod=0; nMod<Nmodules; nMod++){
			uint16_t flag = (this->imp_index >> nMod) & 1;
			if (flag){
				this->ImpControllers[nMod].Compute();
				this->pidControllers[nMod].Compute();
				this->motorsHandler.setMotor(nMod, voltOut[nMod]);
			}
		}
		break;

	default:
		this->shutdown();
		break;
	}
}

void handDriver::shutdown(){
	motorsHandler.disableMotors();
	for (uint32_t nMod=0; nMod<Nmodules; nMod++)
		voltOut[nMod] = 0.0;
}

void handDriver::update_sensors(){
	this->currIn_f[i_P1]= 	(_real)XAdcPs_GetAdcData(this->XAdcInstPtr,anCurr_P1);
	this->currIn_f[i_C]=	(_real)XAdcPs_GetAdcData(this->XAdcInstPtr,anCurr_C);
	this->currIn_f[i_I1]= 	(_real)XAdcPs_GetAdcData(this->XAdcInstPtr,anCurr_I1);

	this->posIn_f[i_P1]=	(_real)XAdcPs_GetAdcData(this->XAdcInstPtr,anPos_P1);// 3.3v
	this->posIn_f[i_I1]=	(_real)XAdcPs_GetAdcData(this->XAdcInstPtr,anPos_I1);// 3.3v
	this->posIn_f[i_C]=		(_real)XAdcPs_GetAdcData(this->XAdcInstPtr,anPos_C); // 3.3v
	this->posIn_f[i_A]=		(_real)XAdcPs_GetAdcData(this->XAdcInstPtr,anPos_A); // 3.3v
	this->posIn_f[i_M]=		(_real)XAdcPs_GetAdcData(this->XAdcInstPtr,anPos_M); // 3.3v
	this->posIn_f[i_P2]=	(_real)(65535 -XAdcPs_GetAdcData(this->XAdcInstPtr,anPos_P2));// 3.3v
}

void handDriver::init_impOut(){
	this->shutdown();

	this->impOut[i_P1]=	(_real)XAdcPs_GetAdcData(this->XAdcInstPtr,anPos_P1)*DEG_ADC; // 3.3v
	this->impOut[i_I1]=	(_real)XAdcPs_GetAdcData(this->XAdcInstPtr,anPos_I1)*DEG_ADC; // 3.3v
	this->impOut[i_C]=	(_real)XAdcPs_GetAdcData(this->XAdcInstPtr,anPos_C) *DEG_ADC; // 3.3v
	this->impOut[i_A]=	(_real)XAdcPs_GetAdcData(this->XAdcInstPtr,anPos_A) *DEG_ADC; // 3.3v
	this->impOut[i_M]=	(_real)XAdcPs_GetAdcData(this->XAdcInstPtr,anPos_M) *DEG_ADC; // 3.3v
	this->impOut[i_P2]=	(_real)(65535 - XAdcPs_GetAdcData(this->XAdcInstPtr,anPos_P2))*DEG_ADC; // 3.3v

	for (uint16_t nMod=0; nMod<Nmodules; nMod++){
		this->ImpControllers[nMod].Restart();
	}
}

void handDriver::set_impFingers(uint8_t impFingers){
	this->imp_index = impFingers;
}

int handDriver::set_currSP(_real *currSP){
	if (this->control_t == FULL_IMPEDANCE)
	{
		for(uint16_t nMod=0; nMod<Nmodules; nMod++){
			this->imp_sp[nMod] = currSP[nMod]; 
		}	
		return XST_SUCCESS;
	}else
	{
		return XST_FAILURE;
	}	
}

int handDriver::set_posSP(_real *posSP){
	if (this->control_t == P_ONLY)
	{
		for(uint16_t nMod=0; nMod<Nmodules; nMod++){
			this->impOut[nMod] = posSP[nMod]; 
		}	
		return XST_SUCCESS;
	}else
	{
		return XST_FAILURE;
	}
}

int handDriver::set_motorV(_real *volt){
	if (this->control_t == MANUAL_MOTORS)
	{
		for(uint16_t nMod=0; nMod<Nmodules; nMod++){
			this->voltOut[nMod] = volt[nMod]; 
		}
		return XST_SUCCESS;
	}else
	{
		return XST_FAILURE;
	}
}

void handDriver::printConfig(){
	this->sensorHandler.printConfig();
}
