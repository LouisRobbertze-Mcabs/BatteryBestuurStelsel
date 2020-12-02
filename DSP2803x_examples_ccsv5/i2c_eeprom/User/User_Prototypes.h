/*
 * User_Prototypes.h
 *
 *  Created on: 04 May 2017
 *      Author: Sonja
 */

#ifndef USER_PROTOTYPES_H_
#define USER_PROTOTYPES_H_

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "math.h"
#include <stdio.h>
#include <float.h>
#include <string.h>
#include "User_CAN.h"
#include "User_ADC.h"
#include "User_I2C.h"
#include "User_BQ.h"
#include "User_Interrupts.h"

union bits32
{
    Uint32 asUint;
    float32 asFloat;
};

void Initialise_BMS(void);
void Init_Gpio(void);

void Toggle_LED(void);
void Read_Cell_Voltages(void);
void Process_Voltages(void);
void Read_Temperatures(void);
void Process_Temperatures(void);
Uint32 ChgCalculator(float Voltage, float Current);
void Calculate_Current(void);
void Read_System_Status(void);
void Process_BQ_System_Status(void);

void Calculate_SOH(void);

void Calculate_SOC();
long interpolate_segment(long x0, long y0, long x1, long y1, long x);
long interpolate_table_1d(struct table_1d *table, long x);



void Calibrate_Current(void);

void Balance(int period, float reference);

unsigned char CRC8(unsigned char *ptr, unsigned char len,unsigned char key);

void Battery_Status(void);
void Battery_Error(void);

float Charging_Animation(float real_SOC);

//#pragma CODE_SECTION(DELAY_US, "ramfuncs");


#endif /* USER_PROTOTYPES_H_ */
