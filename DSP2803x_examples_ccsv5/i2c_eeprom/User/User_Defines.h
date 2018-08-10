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
#define Aux_Control2 GpioDataRegs.GPADAT.bit.GPIO6
#define Aux_Control GpioDataRegs.GPADAT.bit.GPIO15
#define KeyDrive GpioDataRegs.GPADAT.bit.GPIO19
#define ContactorOut GpioDataRegs.GPADAT.bit.GPIO20
#define Fan_Control GpioDataRegs.GPADAT.bit.GPIO19
#define PreCharge GpioDataRegs.GPADAT.bit.GPIO21
#define KeySwitch GpioDataRegs.GPADAT.bit.GPIO24
#define BQOn GpioDataRegs.GPADAT.bit.GPIO26
#define CANEnable GpioDataRegs.GPADAT.bit.GPIO27
#define led3 GpioDataRegs.GPBDAT.bit.GPIO40
#define CSControl GpioDataRegs.GPBDAT.bit.GPIO44

#endif /* USER_DEFINES_H_ */
