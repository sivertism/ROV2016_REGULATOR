#include "stm32f30x.h"
#include "stm32f30x_usart.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include "rov2016_REG.h"
#include "rov2016_UART.h"
#include "def_global_vars.h"
#include "init.h"

int main(void)
{

	uint16_t Tst = 100;	//Time stamp value for regulator
	USART2_init();

	setUpReg(100, 240, 990, 10, 4, 990, 10, 20);

	static int16_t PReg[] 	= {310,0,0}; 					/*Set point for roll, pitch and depth	(mRad, mRad, mm)*/
	static int16_t Sens[]   = {0,0,0};						/*Sensor data for roll, pitch and depth (mRad, mRad, mm)*/
	static int16_t PAcc[]   = {0,0,0};						/*Set point for surge, sway and yaw		(mm,mm, mRad)*/
	static int16_t Thrust[8][1] = {0,0,0,0,0,0,0,0};		/*Variable for thruster gain			(mN)*/

	uint32_t i;												//counter variable for hold function
	while(1){
		i=16000000;
		while(i-->0);
		//updateCan


		regulateThrusters(PReg, PAcc, Sens);				//Regulate Thruster values
		//Thrust = getThrusterValues();						//Get thruster values

	}
}

