/**
 *****************************************************************************
 * @title   ROV2016_REG main
 * @author  H.Line
 * @date    26 Feb 2016
 * @brief   Main file for ROV regulator
 ********************************
 */
/* Include------------------------------------------------------------------------------*/

/* Include .h files from other modules here.*/
#include "stm32f30x.h"

/* Global variables --------------------------------------------------------------------*/

/* Macro--------------------------------------------------------------------------------*/
#define DEBUG_MODE

/* Private Function Prototypes ----------------------------------------------------------*/
static void CalcThrust();
static void matrix_multiply(int32_t* pSrc1, int32_t* pSrc2, int32_t* pDst, uint8_t src1_rows,
		uint8_t src1_columns, uint8_t src2_rows, uint8_t src2_columns);



/* Private variables -------------------------------------------------------------------*/
static int16_t setPReg[3] 	= {0}; 	/*Set point for roll, pitch and depth(x 1/1000)*/
static int32_t setPAcc[6] 	= {0};/*Set point for surge, swai and yaw(x 1/1000)*/
static int16_t SensDat[6]	= {0}; /*Data from sensor node(x 1/1000)*/
static int32_t uP[8] 		= {0};/*Thruster gain in uNewton*/
static int32_t mP[8] 		= {0};/*Thruster gain in milliNewton*/


/* Regulator variables -----------------------------------------------------------------*/
static uint16_t Ts 			= 100;			// 	Time step for regulator
static int16_t Err[6] 		= {0}; 			//	Error, from set point to sensor value
static int16_t ErrOld[6] 	= {0}; 			//	Last Error, from set point to sensor value
static int32_t KpReg[6]		= {0};			//  Gain from regulator (x 1/1000 N)
static int32_t KPmax[6] 	= {0};			//  Maximum axis gain 	(x 1/1000 N)
static int16_t Pmax 		= 0;			//  Maximum thruster gain 	(x 1/1000 N)


static int16_t Phi = 0;						//	Proportional value for height control
static int16_t Ppi = 0;						//	Proportional value for pitch control
static int16_t Pro = 0;						//	Proportional value for roll control

static int16_t Ihi = 0;						//	Integral value for height control
static int16_t Ipi = 0;						//	Integral value for pitch control
static int16_t Iro = 0;						//	Integral value for roll control

static int16_t Dhi = 0;						//	Derivative value for height control
static int16_t Dpi = 0;						//	Derivative value for pitch control
static int16_t Dro = 0;						//	Derivative value for roll control

static int16_t Kp_t = 0;					//Parameter for regulator
static int16_t Ti_t = 0;					//Parameter for regulator
static int16_t Td_t = 0;					//Parameter for regulator

static int16_t Kp_r = 0;					//Parameter for regulator
static int16_t Ti_r = 0;					//Parameter for regulator
static int16_t Td_r = 0;					//Parameter for regulator



// Thruster force matrix calculation: PP(8x1) = ABR(8*6) * KP(6x1)
static int32_t ABR [8][6] 	={
		{0,0,250,1667,-1563,0},
		{0,0,250,1667,1563,0},
		{0,0,250,-1667,1563,0},
		{0,0,250,-1667,-1563,0},
		{354,-354,0,0,0,-1407},
		{354,354,0,0,0,-1407},
		{354,-354,0,0,0,1407},
		{354,354,0,0,0,1407}
};

static int32_t ABmax[6] = {2830, 2830, 4000, 600 , 640, 710};

static int32_t Kp[6][1] 	= {{0},{0},{0},{0},{0},{0}};
static int16_t PP[8][1] 	= {{0},{0},{0},{0},{0},{0},{0},{0}};
static int16_t sizeABR[2] 	= {8,6};
static int16_t sizeKP[2] 	= {6,1};
static int16_t CalcSum 		= 0;

/* Function definitions ----------------------------------------------------------------*/

extern void REG_regulateThrusters(int16_t* PReg, int16_t* PAcc, int16_t* Sens){

	setPReg[3] = PReg[0];		//roll
	setPReg[4] = PReg[1];		//pitch
	setPReg[2] = PReg[2];		//depth

	setPAcc[0] = PAcc[0];		//X
	setPAcc[1] = PAcc[1];		//Y
	setPAcc[5] = PAcc[2];		//HEADING


	SensDat[3] = Sens[0];		//roll
	SensDat[4] = Sens[1];		//pitch
	SensDat[2] = Sens[2];		//depth

	CalcThrust();
}

static void CalcThrust(){
	/*l
	 * @brief  Calculate the thruster gain with PID regulator
	 */

	// 1: Calculate error
	ErrOld[2] = Err[2];
	ErrOld[3] = Err[3];
	ErrOld[4] = Err[4];

	Err[2] = SensDat[2] - setPReg[2];
	Err[3] = SensDat[3] - setPReg[3];
	Err[4] = SensDat[4] - setPReg[4];


	// 2: Calculate thruster gain from PID regulator

	// Depth
	Phi  = Kp_t*Err[2];														// Proportional part
	Ihi += Kp_t*(Ts/Ti_t)*Err[2]/1000;										// Integral part
	Dhi  = Kp_t*Ts*(ErrOld[2]-Err[2])*1000;									// Diff. part
	KpReg[2]  = (int32_t) (Phi + Ihi + Dhi)/10;								// Sum all three, mN/mNm

	// Roll
	Pro  = Kp_r*Err[3];															// Proportional part
	Iro += Kp_r*(Ts/Ti_r)*Err[3]/1000;											// Integral part
	Dro  = Kp_t*Ts*(ErrOld[3]-Err[3])/1000;										// Diff. part
	KpReg[3]  = (int32_t) (Pro + Iro + Dro)/10;									// Sum all three, mN/mNm
	// Pitch
	Ppi = Kp_r*Err[4];															// Proportional part
	Ipi += Kp_r*(Ts/Ti_r)*Err[4]/1000;											// Integral part
	Dpi  = Kp_t*Ts*(ErrOld[4]-Err[4])/1000;										// Diff. part
	KpReg[4]  = (int32_t) (Ppi + Ipi + Dpi)/10;											// Sum all three, mN/mNm


	// 3: Sum up regulator output + setPAcc, and scale. (mN/mNm)

	volatile uint8_t i;
	for (i=0; i<6; i++){
		Kp[i][0] = (int32_t) KpReg[i] + setPAcc[i];
		if(Kp[i][0]> KPmax[i]) Kp[i][0] = KPmax[i];
	}



	// 4: Update values in PP
	// mP(6x1) = ABR(8*6) * KP(6x1);

	matrix_multiply(ABR,Kp, uP, 8, 6, 6, 1);


	for (i=0; i<8; i++){
		mP[i] = uP[i]/1000;
		if(mP[i] > Pmax) mP[i] = Pmax;
	}
}



extern int16_t* REG_getThrusterValues(void){
	return mP;
}




/*Set regulator parameters*/
extern void REG_init(int16_t timeStamp, int16_t sKp_t, int16_t sTi_t, int16_t sTd_t,
		int16_t sKp_r, int16_t sTi_r, int16_t sTd_r,  int16_t maxThrust){

	volatile uint8_t i;

	Ts = timeStamp;
	//Translational regulator parameters
	Kp_t = sKp_t;
	Ti_t = sTi_t;
	Td_t = sTd_t;
	//Rotational regulator parameters
	Kp_r = sKp_r;
	Ti_r = sTi_r;
	Td_r = sTd_r;

	Pmax = maxThrust * 1000;

	for (i=0; i<6; i++){
		KPmax[i] = ABmax[i]*maxThrust;											//Max thrust in axis

	}
}

static void matrix_multiply(int32_t* pSrc1, int32_t* pSrc2, int32_t* pDst, uint8_t src1_rows,
		uint8_t src1_columns, uint8_t src2_rows, uint8_t src2_columns){

	if(src1_columns != src2_rows){
		return;
	}

	int32_t* p1 = pSrc1;
	int32_t* p2 = pSrc2;
	int32_t* p3 = pDst;

	int32_t sum=0, n1, n2;


	uint8_t dst_rows = src1_rows;
	uint8_t dst_columns = src2_columns;

	uint8_t i, j, k;

	for(i=0; i<src1_rows; i++){
		for(j=0; j<src2_columns; j++) {
			for(k=0; k<src1_columns; k++){
				n1 = *p1;
				n2 = *p2;
				sum += n1*n2;
				p1 ++;
				p2 += src2_columns;
			}
			// Reset to prev position.
			p1 --;
			p2 -= src2_columns;
			p1 = p1 - src1_columns + 1;
			p2 = p2 - (src2_rows-1)*src2_columns + 1;

			// Last iteration.
			if(j == src2_columns-1) p2 = pSrc2;

			*p3 = sum;
			p3 ++;
			sum = 0;
		}
		p1 = pSrc1 + (i+1)*src1_columns;
		p2 = pSrc2;
		p3 = pDst + (i+1)*dst_columns;
	}

}

/**
 * @brief  	Calculates tilt compensated heading.
 * 			Button/axes mapping:
 * 				0. Left trigger:			Depth -
 * 				1. Right trigger:			Depth +
 * 				2. Left stick L/R:			Sway
 * 				3. Left stick fwd/back:		Surge
 * 				4. Right stick L/R:			Roll
 * 				5. Right stick fwd/back:	Pitch
 * @param  	Pointer to controller CAN-messages.
 * @retval 	int16_t array containing controller data.
 */
static int16_t* REG_readController(uint8_t* controller_package1, uint8_t* controller_package2){
	int16_t controller_data[6];

	/* Left trigger */
	controller_data[0] = ( (int16_t)*(controller_package1 + 5) << 8 ) | *(controller_package1 + 4);

	/* Right trigger */
	controller_data[1] = ( (int16_t)*(controller_package2 + 3) << 8) | *(controller_package2 + 2);

	/* Left stick left/right*/
	controller_data[2] = ( (int16_t)*(controller_package1 + 1) << 8) | *controller_package1;

	/* Left stick fwd/back */
	controller_data[3] = ( (int16_t)*(controller_package1 + 3) << 8) | *(controller_package1 + 2);

	/* Right stick left/right */
	controller_data[4] = ( (int16_t)*(controller_package1 + 7) << 8) | *(controller_package1 + 6);

	/* Right stick fwd/back */
	controller_data[5] = ( (int16_t)*(controller_package2 + 1) << 8) | *controller_package2;

	return controller_data;
}
