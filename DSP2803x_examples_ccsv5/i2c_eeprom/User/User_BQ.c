/*
 * User_BQ.c
 *
 *  Created on: 04 May 2017
 *      Author: Sonja
 */

#include "User_BQ.h"

void Shut_D_BQ(void)                        //bq turn off sequence
{
    I2CA_WriteData(0x04,0x00);
    I2CA_WriteData(0x04,0x02);
    I2CA_WriteData(0x04,0x01);
    I2CA_WriteData(0x04,0x02);
}

void  Bq76940_Init(void)
{
    Uint16 temp;
    Uint16 temp2;
    int OV;
    int UV;
    Uint16 Reset;


    BQEnable = 1;                                           //turn on BQ chip
    while(counter_2Hz != 1);                                //toets delay

    Reset = I2CA_ReadData(&I2cMsgIn1,0x00, 1);

    if(Reset != 00)
        I2CA_WriteData(0x00,(char)Reset);

    I2CA_WriteData(0x04,0x18);                                  //Sit metings aan

    I2CA_WriteData(0x01,0x00);
    I2CA_WriteData(0x02,0x00);
    I2CA_WriteData(0x03,0x00);

    I2CA_WriteData(0x08,0x08);                                  //protect 3

    I2CA_WriteData(0x0B,0x19);

    I2CA_WriteData(0x05,0x03);                                  //turn on outputs (CHG+DSG)

    //lees adc gain en offset
    temp = I2CA_ReadData(&I2cMsgIn1,0x50, 1);

    temp2 = I2CA_ReadData(&I2cMsgIn1,0x59, 1);

    temp2 = temp2 >> 5;
    temp2= temp2 & 0x07;

    temp = temp<<1;
    temp = temp & 0x0C;

    temp = temp2 | temp;

    ADCgain = ((float)temp + 365)*0.000001;

   temp = (I2CA_ReadData(&I2cMsgIn1,0x51, 1));

    // If a positive value, return it
    if ((temp & 0x80) == 0)
    {
        ADCoffset = temp * 0.001;
    }
    else                                                                // Otherwise perform the 2's complement math on the value
    {
        ADCoffset = ((~(temp - 0x01)) & 0xFF) * -0.001;
    }
//   ADCoffset = ((I2CA_ReadData(&I2cMsgIn1,0x51, 1))) * 0.001;

    //Over voltage = 3.7 V
    OV = (3.65-ADCoffset)/ADCgain;
    OV = (OV>>4) & 0xFF;

    I2CA_WriteData(0x09, (char)OV);                             //Stel OV_trip op

    //Under voltage = 2.4 V
    UV = (2.45-ADCoffset)/ADCgain;                              //should probably be 2.55 and do low power afterwards/shutdown
    UV = (UV>>4) & 0xFF;

    I2CA_WriteData(0x0A,(char)UV);                              //Stel UV_trip op

    BQEnable = 0;                                               //pull low to allow BQ to measure temp

    EALLOW;

    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;     //BQenable
    GpioCtrlRegs.GPADIR.bit.GPIO3 = 0;      //Bq enable as input
    EDIS;
}
