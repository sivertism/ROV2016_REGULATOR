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

/* Call this method to start the regulator. Call this method every Ts. */
extern void regulateThrusters(void);

/*Send updated variables from top side and sensor before calling regulateThrusters.*/
extern void UpdateTopsideVar(int16_t* PReg, int16_t* PAcc);
extern void UpdateSensorVar(int16_t* Sens);


/*Set point for the time interval of the regulator, in milliseconds
*Initial the Ts = 100 mS
*/
extern void setTimestamp(uint16_t* Tst);

/*Get updatet thruster values from regulator*/
extern int16_t getThrusterValues(void);


/*Set regulator parameters*/
/*Translational*/
extern void setTransRegparam(int16_t sKp_t, int16_t sTi_t, int16_t sTd_t);	//Initial parameters: (8.5,2.8,2.8)

/*Rotational*/
extern void setRotRegparam(int16_t sKp_r, int16_t sTi_r, int16_t sTd_r);	//Initial parameters: (0 9990 0)

