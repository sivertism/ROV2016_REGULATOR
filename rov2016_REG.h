/**
  **************************************************************************************
  * @file    rov2016_REG.h
  * @author  Hartvik Line, Olav H. Karstensen
  * @version V01
  * @date    07-Mars-2016
  * @brief   This file contains macros and extern function definitions for
  			 rov2016_REG.c.
  **************************************************************************************
  */

/* Macro -------------------------------------------------------------------------------*/
/* Put #define's here.

/* ID list, RANGE = [0...0x7FF] --------------------------------------------------------*/

/* Exported function prototypes --------------------------------------------------------*/

/* Declare global functions here, preferrably with comments to explain the functions.
 */



// Macro -------------------------------------------------------------------------------


extern void REG_regulateThrusters(int16_t* PReg, int16_t* PAcc, int16_t* Sens);
/* Call this method to start the regulator. Call this method every Ts.
 * PReg 				Set point for roll, pitch and depth			(mRad, mRad, mm)
 * Sens					Sensor data for roll, pitch and depth 		(mRad, mRad, mm)
 * PAcc					Set acceleration for surge, sway and yaw	(mN,mN, mNM)
 * */



/*Send updated variables from top side and sensor before calling regulateThrusters.*/



extern int16_t* REG_getThrusterValues(void);
/*Get updated thruster values from regulator*/


extern void REG_init(int16_t timeStamp, int16_t sKp_t, int16_t sTi_t, int16_t sTd_t,
		int16_t sKp_r, int16_t sTi_r, int16_t sTd_r, int16_t maxThrust);
/*Set regulator parameters
 * timeStamp 	= time stamp in milli seconds
 * sKp_t 		= translational KP value 	(*10)
 * sTi_t 		= translational TI value 	(*10)
 * sTd_t 		= translational TD value 	(*10)
 * sKp_r 		= rotational KP value 		(*10)
 * sTi_r 		= rotational TI value		(*10)
 * sTd_r 		= rotational TD value		(*10)
 * maxThrust 	= maximal thruster values, in Newton
 *
 * */

