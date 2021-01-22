/*
 * User_Defines.h
 *
 *  Created on: 04 May 2017
 *      Author: Sonja
 */

#ifndef USER_DEFINES_H_
#define USER_DEFINES_H_

#include "User_Prototypes.h"


// Local definitions
#define I2C_SLAVE_ADDR        0x08
#define I2C_NUMBYTES          3                 //ekstra 1 vir crc byte
#define I2C_EEPROM_HIGH_ADDR  0x09
#define I2C_EEPROM_LOW_ADDR   0x91

#define BQEnable GpioDataRegs.GPADAT.bit.GPIO3
#define led2 GpioDataRegs.GPADAT.bit.GPIO4
#define led1 GpioDataRegs.GPADAT.bit.GPIO5
#define LPwr_Out_Ctrl_1 GpioDataRegs.GPADAT.bit.GPIO6            //BMS N-type MfET output 1
#define LPwr_Out_Ctrl_2 GpioDataRegs.GPADAT.bit.GPIO7            //BMS N-type MfET output 2

#define BQ_Alert GpioDataRegs.GPADAT.bit.GPIO9                   //BQ pin GPIO9

//Do HRCAP for GPIO11

#define Temperature_Control GpioDataRegs.GPADAT.bit.GPIO12
#define Ctrl_HPwr_48V_O_1 GpioDataRegs.GPADAT.bit.GPIO13         //Ctrl_HPwr_48V_O_1
#define Aux_Control GpioDataRegs.GPADAT.bit.GPIO15               //Ctrl_HPwr_48V_O_2

#define Ctrl_LPwr_48V_O_2 GpioDataRegs.GPADAT.bit.GPIO19         //Ctrl_LPwr_48V_O_2
#define ContactorOut GpioDataRegs.GPADAT.bit.GPIO20              //Ctrl_LPwr_48V_O_1

#define Ctrl_Pre_Charge GpioDataRegs.GPADAT.bit.GPIO21                //Ctrl_Pre_Charge

#define Key_switch_1 GpioDataRegs.GPADAT.bit.GPIO22              //Key_switch_Position_1
#define Key_switch_2 GpioDataRegs.GPADAT.bit.GPIO24              //Key_switch_Position_2

#define CHG_J1772_Ctrl GpioDataRegs.GPADAT.bit.GPIO25            //Type 2 charger control pin

#define BQOn GpioDataRegs.GPADAT.bit.GPIO26
#define CANEnable GpioDataRegs.GPADAT.bit.GPIO27

#define Ctrl_LPwr_48V_O_3 GpioDataRegs.GPBDAT.bit.GPIO39         //Ctrl_LPwr_48V_O_3

#define led3 GpioDataRegs.GPBDAT.bit.GPIO40                      //LED Current Sense PCB
#define CSControl GpioDataRegs.GPBDAT.bit.GPIO44

#endif /* USER_DEFINES_H_ */
