#include "SensorMod.hpp"
#include "xil_io.h"

//***********************************************  Constructor

// Constructor (without allocation)
sensorMod::sensorMod(const uint32_t _addr,
		float* posIn, float* currIn, float* voltIn,
		float* pos_filt, float* vel_filt, float* curr_filt){
    
	this->virtAddr = _addr;


	/**
	 * Reset SensorMod
	 * }
	 */
	write32(STATUS_REG, RESET_MASK);
	/*}*/

	/**
	 * Lookup SensorMod Config
	 * }
	 */
    // dt
    uint32_t *dt = (uint32_t *)malloc(sizeof(uint32_t *));
    *dt = read32(OFFSET_DT);
    this->dt._uint32 = dt;
    // R
    uint32_t *R = (uint32_t *)malloc(sizeof(uint32_t *));
    *R = read32(OFFSET_R);
    this->R._uint32 = R;
    // Q
    uint32_t* Q = (uint32_t*)malloc(4*3);
    Q[0] = read32(OFFSET_Q0);
    Q[1] = read32(OFFSET_Q12);
    Q[2] = read32(OFFSET_Q3);
    this->Q._uint32 = Q;
    // ab
    uint32_t* ab = (uint32_t*)malloc(4*2);
    ab[0] = read32(OFFSET_a1);
    ab[1] = read32(OFFSET_b01);
    this->ab._uint32 = ab;
	// DEG_ADC
    uint32_t *DEG_ADC = (uint32_t *)malloc(sizeof(uint32_t *));
    *DEG_ADC = read32(OFFSET_DEG_ADC);
    this->DEG_ADC._uint32 = DEG_ADC;
	// mA_ADC
    uint32_t *mA_ADC = (uint32_t *)malloc(sizeof(uint32_t *));
    *mA_ADC = read32(OFFSET_mA_ADC);
    this->mA_ADC._uint32 = mA_ADC;
    // Center Current
	uint32_t *cenCurr = (uint32_t *)malloc(4*Nmodules);
	for (uint32_t mod=0; mod<Nmodules; mod++){
		write32(OFFSET_MOD, mod+1);
		cenCurr[mod] = read32(OFFSET_CEN_CURR);
	}
	this->center_current._uint32 = cenCurr;
    // Position Start
	uint32_t *posStart = (uint32_t *)malloc(4*Nmodules);
	for (uint32_t mod=0; mod<Nmodules; mod++){
		write32(OFFSET_MOD, mod+1);
		posStart[mod] = read32(OFFSET_POS_START);
	}
	this->position_start._uint32 = posStart;
	/*}*/
    
	// Filter;
    this->filt_pos._float32 = (float *)pos_filt;
	this->filt_vel._float32 = (float *)vel_filt;
	this->filt_curr._float32 = (float *)curr_filt;
	// Input
	this->posIn._float32 = (float *)posIn;
	this->currIn._float32 = (float *)currIn;
	this->voltIn._float32 = (float *)voltIn;

	this->SampleTime = this->dt._float32[0]*1000;
	this->lastTime = (uint64_t)millis();
}


//***********************************************  Calculating Functions
void sensorMod::start(){
	uint32_t data_n = 0;
	uint32_t now = (uint64_t)millis();
	uint32_t timeChange = (now - this->lastTime);
	if (timeChange >= this->SampleTime){
		for(uint32_t mod=1; mod <= Nmodules; mod++){
			this->posIn._uint32[data_n] = (this->posIn._uint32[data_n] & MASK_MOD_IN) | mod;
			write32(OFFSET_POS, this->posIn._uint32[data_n]);

			this->currIn._uint32[data_n] = (this->currIn._uint32[data_n] & MASK_MOD_IN) | mod;
			write32(OFFSET_VEL_CURR, this->currIn._uint32[data_n]);

			this->voltIn._uint32[data_n] = (this->voltIn._uint32[data_n] & MASK_MOD_IN) | mod;
			write32(OFFSET_CURR_VOLT, this->voltIn._uint32[data_n]);

			data_n++;
		}
    	wait();

		this->lastTime = now;
	}
}

uint32_t sensorMod::isReady(){
    uint16_t status = read32(STATUS_REG);
    if ( (status & READY_MASK) == ISREADY ){
    	get_filteredData();
    	return 1;
    }else{
    	return 0;
    }
}

void sensorMod::wait(){
	uint32_t flag = 0;
	while (flag != 1){
		flag = isReady();
	}
}

void sensorMod::get_modFilt_Data(uint32_t mod, float* mod_data){
    mod_data[0] = this->filt_pos._float32[mod];
    mod_data[1] = this->filt_vel._float32[mod];
    mod_data[2] = this->filt_curr._float32[mod];
}

//***********************************************  Configuration Functions
void sensorMod::set_centerCurr(const float ext_centerCurr, uint32_t mod){
	uint32_t xil_mod = mod+1;
	this->center_current._float32[mod] = ext_centerCurr;
	uint32_t axi_data = this->center_current._uint32[mod];
	axi_data = (axi_data & MASK_MOD_IN) | xil_mod;
	write32(OFFSET_CEN_CURR, axi_data);

	write32(OFFSET_MOD,xil_mod);
	axi_data = read32(OFFSET_CEN_CURR);
    this->center_current._uint32[mod] = axi_data;
}

void sensorMod::set_positionStart(const float ext_positionStart, uint32_t mod){
	uint32_t xil_mod = mod+1;
	this->position_start._float32[mod] = ext_positionStart;
	uint32_t axi_data = this->position_start._uint32[mod];
	axi_data = (axi_data & MASK_MOD_IN) | xil_mod;
	write32(OFFSET_POS_START, axi_data);

	write32(OFFSET_MOD,xil_mod);
	axi_data = read32(OFFSET_POS_START);
	this->position_start._uint32[mod] = axi_data;
}

void sensorMod::set_Kalman(const float ext_dt, const float ext_R, const float ext_nQ){
	this->SampleTime = ext_dt;
	float my_dt = (float)ext_dt*(float)0.001;

	this->dt._float32[0] = my_dt; // dt is in milliseconds
    write32(OFFSET_DT, this->dt._uint32[0]);
    this->dt._uint32[0] = read32(OFFSET_DT);

    this->R._float32[0] = ext_R;
    write32(OFFSET_R, this->R._uint32[0]);
    this->R._uint32[0] = read32(OFFSET_R);

    // (nQ*dt^4)/4
    this->Q._float32[0] = (ext_nQ*my_dt*my_dt*my_dt*my_dt)/4.0f;
    write32(OFFSET_Q0, this->Q._uint32[0]);
    this->Q._uint32[0] = read32(OFFSET_Q0);

    // (nQ*dt^3)/2
    this->Q._float32[1] = (ext_nQ*my_dt*my_dt*my_dt)/2.0f;
    write32(OFFSET_Q12, this->Q._uint32[1]);
    this->Q._uint32[1] = read32(OFFSET_Q12);

    // nQ*dt^2
    this->Q._float32[2] = ext_nQ*my_dt*my_dt;
    write32(OFFSET_Q3, this->Q._uint32[2]);
    this->Q._uint32[2] = read32(OFFSET_Q3);

}

void sensorMod::set_butter(const float *ab){
    this->ab._float32[0] = ab[0];
    write32(OFFSET_a1, this->ab._uint32[0]);
    this->ab._uint32[0] = read32(OFFSET_a1);
    this->ab._float32[1] = ab[1];
    write32(OFFSET_b01, this->ab._uint32[1]);
    this->ab._uint32[1] = read32(OFFSET_b01);
}

void sensorMod::set_2ADC(const float _DEG_ADC, const float _mA_ADC){
	this->mA_ADC._float32[0] = _mA_ADC;
    write32(OFFSET_mA_ADC, this->mA_ADC._uint32[0]);
    this->mA_ADC._uint32[0] = read32(OFFSET_mA_ADC);
	this->DEG_ADC._float32[0] = _DEG_ADC;
    write32(OFFSET_DEG_ADC, this->DEG_ADC._uint32[0]);
    this->DEG_ADC._uint32[0] = read32(OFFSET_DEG_ADC);
}
	

//***********************************************  Variable Functions
float sensorMod::get_centerCurr(uint32_t mod){return this->center_current._float32[mod];}
float sensorMod::get_positionStart(uint32_t mod){return this->position_start._float32[mod];}
float sensorMod::get_dt(){return this->dt._float32[0];};
float sensorMod::get_R(){return this->R._float32[0];};
float* sensorMod::get_Q(){return this->Q._float32;};
float* sensorMod::get_ab(){return this->ab._float32;};
float sensorMod::get_mA_ADC(){return this->mA_ADC._float32[0];};
float sensorMod::get_DEG_ADC(){return this->DEG_ADC._float32[0];};

void sensorMod::printConfig(){
	float dt, R, *Q, *ab;
	float mA_ADC, DEG_ADC;
	float cenCurr[Nmodules], posStart[Nmodules];

	dt = get_dt();
	R = get_R();
	Q = get_Q();
	ab = get_ab();
	mA_ADC = get_mA_ADC();
	DEG_ADC = get_DEG_ADC();
    for (uint32_t mod=0; mod<Nmodules; mod++){
		cenCurr[mod] = get_centerCurr(mod);
		posStart[mod] = get_positionStart(mod);
	}

	printf("\n\nSensor Module Configuration Set \n");
	printf("\tdt = %.4e | %lu millis \n", dt, this->SampleTime);
	printf("\tmA_ADC = %.4e | DEG_ADC = %.4e\n", mA_ADC, DEG_ADC);
	printf("\tR = %.4e \n", R);
	printf("\tQ = { %.4e , %.4e , %.4e } \n", Q[0], Q[1], Q[2]);
	printf("\t  | CenterCurr | PositionStart \n");
	for (uint32_t i=0; i<Nmodules; i++){
		printf("\t%lu | %.4e | %.4e \n", i, cenCurr[i],posStart[i]);
	}
	printf("\tab = { %.10e , %.10e } \n", ab[0], ab[1] );
}

void sensorMod::printFiltData(){

	float *sensor_filt = (float *)malloc(4*3);
	printf("\n\nFiltered Data\n");
	printf("----------------------------------------\n");
	printf( "Mod | position | velocity | current \n");
	for (uint32_t mod=0; mod<Nmodules; mod++){
		get_modFilt_Data(mod, sensor_filt);
		printf(" %lu  | %f | %f | %f \n", mod, sensor_filt[0], sensor_filt[1], sensor_filt[2]);
	}
}

//***********************************************  Private Functions

void sensorMod::get_filteredData(){
	uint32_t xil_mod=0;
	for (uint32_t mod=0; mod<Nmodules; mod++){
		xil_mod++;
		write32(OFFSET_MOD, xil_mod);
		this->filt_pos._uint32[mod] = read32(OFFSET_POS);
		this->filt_vel._uint32[mod] = read32(OFFSET_VEL_CURR);
		this->filt_curr._uint32[mod] = read32(OFFSET_CURR_VOLT);
	}
}

// Read data from uio device
uint32_t sensorMod::read32(const uint32_t offset){	
	uint32_t read_result = Xil_In32(this->virtAddr + offset);
	return read_result;
}

// Write data to  uio device
void 	sensorMod::write32 (const uint32_t offset, const uint32_t writeval){
	 Xil_Out32(this->virtAddr + offset, writeval);
}
