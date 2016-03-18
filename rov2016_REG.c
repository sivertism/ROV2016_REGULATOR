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
#include "stm32f30x_usart.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include "float.h"

/* Global variables --------------------------------------------------------------------*/

/* Private Function Prototypes ----------------------------------------------------------*/
static void UpdPar();
static void CalcThrust();



/* Private variables -------------------------------------------------------------------*/
static int16_t setPReg[] 	= {0,0,0}; 					/*Set point for roll, pitch and depth(x 1/1000)-------------*/
static int16_t setPAcc[] 	= {0,0,0};					/*Set point for surge, swai and yaw(x 1/1000)---------------*/
static int16_t P[] 			= {0,0,0,0,0,0,0,0}; 		/*Thruster gain in milliNewton------------------------------*/
static int16_t SensDat[]	= {0,0,0,0,0,0}; 			/*Data from sensor node(x 1/1000)---------------------------*/


/* Regulator variables -------------------------------------------------------------------*/
static uint16_t Ts 			= 100;											// 	Time step for regulator
static int16_t Err[6] 		= {0}; 											//	Error, from set point to sensor value
static int16_t ErrOld[6] 	= {0}; 											//	Last Error, from set point to sensor value
static int16_t KpU[6]		= {0};											//  Gain from regulator (x 1/1000)
static int16_t KPmax[6] 	= {19800,19800,28000,4480,4480,4970};			//  Maximum axis gain 	(x 1/1000)

static int16_t Phi = 0;														//	Proportional value for height control
static int16_t Ppi = 0;														//	Proportional value for pitch control
static int16_t Pro = 0;														//	Proportional value for roll control

static int16_t Ihi = 0;														//	Integral value for height control
static int16_t Ipi = 0;														//	Integral value for pitch control
static int16_t Iro = 0;														//	Integral value for roll control

static int16_t Dhi = 0;														//	Derivative value for height control
static int16_t Dpi = 0;														//	Derivative value for pitch control
static int16_t Dro = 0;														//	Derivative value for roll control

static int16_t Uhi = 0;														//	Gain value for height control
static int16_t Upi = 0;														//	Gain value for pitch control
static int16_t Uro = 0;														//	Gain value for roll control

static int8_t Kp_t = 8;														//Parameter for regulator
static int8_t Ti_t = 3;														//Parameter for regulator
static int8_t Td_t = 3;														//Parameter for regulator

static int16_t Kp_r = 0;													//Parameter for regulator
static int16_t Ti_r = 9999;													//Parameter for regulator
static int16_t Td_r = 0;													//Parameter for regulator



// Thruster force matrix calculation: PP(8x1) = ABR(8*6) * KP(6x1)
static int16_t ABR [8][6] 	={{0,0,250,1670,-1563,0},{0,0,250,1670,1560,0},{0,0,250,-1670,1563,0},{0,0,250,-1670,-1563,0},{350,-350,0,0,0,-1560},{350,350,0,0,0,-1560},{350,-350,0,0,0,1560},{350,350,0,0,0,1560}};
static int16_t KP[6][1] 	= {{0},{0},{0},{0},{0},{0}};
static int16_t PP[8][1] 	= {{0},{0},{0},{0},{0},{0},{0},{0}};
static int16_t sizeABR[2] 	= {8,6};
static int16_t sizeKP[2] 	= {6,1};
static int16_t CalcSum 		= 0;

//TEST PARAMETER
static int16_t TEST1[2][3] = {{1, 2, 3},{4,5,6}};
static int16_t TEST2[3][2] = {{1,2},{3,4},{5,6}};
static int16_t TEST3[2][2] = {0};
static int16_t TES1 = 0;
static int16_t TES2 = 0;

extern void regulateThrusters(void){
	CalcThrust();
	int32_t test = sizeABR[0];
	/*printf(" Thrusterpådrag: %d ", PP[0][0]);
	printf("%d ", PP[1][0]);
	printf("%d ", PP[2][0]);
	printf("%d ", PP[3][0]);
	printf("%d ", PP[4][0]);
	printf("%d ", PP[5][0]);
	printf("%d ", PP[6][0]);
	printf("%d ", PP[7][0]);

*/
	/*printf("Sum = %d ", TEST3[0][0]);
	printf("Sum = %d ", TEST3[0][1]);
	printf("Sum = %d ", TEST3[1][0]);
	printf("Sum = %d ", TEST3[1][1]);
	*/
}


/* Function definitions ----------------------------------------------------------------*/


static void CalcThrust(){
	/*l
	 * @brief  Calculate the thruster gain with PID regulator
	 * @param
	 * @retval No return
	 */

	// 1: Calculate error
	ErrOld[2] = Err[2];
	ErrOld[3] = Err[3];
	ErrOld[4] = Err[4];

	Err[2] = SensDat[2] - setPReg[2];
	Err[3] = SensDat[0] - setPReg[3];
	Err[4] = SensDat[1] - setPReg[4];


	// 2: Calculate thruster gain from PID regulator

	// Depth
	Phi  = Kp_t*Err[2];														// Proportional part
	//Ihi += Kp_t*(Ts/Ti_t)*Err[2]/1000;									// Integral part
	Dhi  = Kp_t*Ts*(ErrOld[2]-Err[2])/1000;									// Diff. part
	KpU[2]  = Phi + Ihi + Dhi;												// Sum all three, mN/mNm

	// Roll
	Pro  = Kp_r*Err[3];														// Proportional part
	//Iro += Kp_r*(Ts/Ti_r)*Err[4];											// Integral part
	Dro  = Kp_t*Ts*(ErrOld[3]-Err[3])/1000;									// Diff. part
	KpU[3]  = Pro + Iro + Dro;												// Sum all three, mN/mNm

	// Pitch
	Ppi = Kp_r*Err[4];														// Proportional part
	//Ipi += Kp_r*(Ts/Ti_r)*Err[4];											// Integral part
	Dpi  = Kp_t*Ts*(ErrOld[4]-Err[4])/1000;									// Diff. part
	KpU[4]  = Ppi + Ipi + Dpi;												// Sum all three, mN/mNm





	// 3: Sum up regulator output + setPAcc, and scale.

	volatile uint8_t i;
	for (i=0; i<8; i++){
		KpU[i] = setPAcc[i]+ KpU[i];
	}




	// 4: Update values in PP
	// PP(6x1) = ABR(8*6) * KP(6x1);

	KP [0][0] = 350;
	KP [1][0] = 0;
	KP [2][0] = 200;
	KP [3][0] = 0;
	KP [4][0] = 235;
	KP [5][0] = 0;



	printf(" Starter aa regne \n");
	TES1 = 0;
	TES2 = 0;
	for (int i = 0; i < 2; i++){
		for (int j = 0; j < 2; j++){
			CalcSum = 0;
			for (int k = 0; k < sizeABR[1]; k++){
				TES1 = TEST1[i][k];
				TES2 = TEST2[k][j];
				CalcSum += TES1*TES2;
				//CalcSum += *ABR[i, k] * *KP[k, j];
			}
			printf("\n Sum = %d ", CalcSum);
			//*PP[i, j] = CalcSum;
		//TEST3[i,j] = CalcSum;
		}
	}
	printf(" Regnet ferdig \n");


}

extern int16_t getThrusterValues(void){
	return P;
}

extern void UpdateTopsideVar(int16_t* PReg, int16_t* PAcc){

	volatile uint8_t i;
	for (i=0; i<3; i++){
		setPReg[i] = PReg[i];
	}
	for (i=0; i<3; i++){
		setPAcc[i] = PAcc[i];
	}
}

extern void UpdateSensorVar(int16_t* Sens){
	SensDat[3] = Sens[0];													//pitch
	SensDat[4] = Sens[1];													//roll
	SensDat[2] = Sens[2];													//depth
}

extern void setTimestamp(uint16_t* Tst){									//Set time stamp
	Ts = Tst;
}

/*Set regulator parameters*/
//Translational
extern void setTransRegparam(int16_t sKp_t, int16_t sTi_t, int16_t sTd_t){
	Kp_t = sKp_t;
	Ti_t = sTi_t;
	Td_t = sTd_t;
}

//Rotational
extern void setRotRegparam(int16_t sKp_r, int16_t sTi_r, int16_t sTd_r){

	Kp_r = sKp_r;
	Ti_r = sTi_r;
	Td_r = sTd_r;
}
