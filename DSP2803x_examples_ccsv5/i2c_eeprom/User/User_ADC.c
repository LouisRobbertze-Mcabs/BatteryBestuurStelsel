/*
 * User_ADC.c
 *
 *  Created on: 04 May 2017
 *      Author: Bartho Horn
 */

#include "User_ADC.h"

void configADC(void)
{
	EALLOW;
	AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;    //ADCINT1 trips after AdcResults latch
	AdcRegs.INTSEL1N2.bit.INT1E     = 1;    //Enabled ADCINT1
	AdcRegs.INTSEL1N2.bit.INT1CONT  = 0;    //Disable ADCINT1 Continuous mode
	AdcRegs.INTSEL1N2.bit.INT1SEL   = 1;    //setup EOC1 to trigger ADCINT1 to fire		//1
	AdcRegs.ADCSOC0CTL.bit.CHSEL    = 0x0B; //0     set SOC0 channel select to ADCINB3(dummy sample for rev0 errata workaround)
	AdcRegs.ADCSOC1CTL.bit.CHSEL    = 0x0B; //0     set SOC1 channel select to ADCINB3
	AdcRegs.ADCSOC0CTL.bit.TRIGSEL  = 0x03; //  CPU Timer 0  // hierdie was 0x03
	AdcRegs.ADCSOC1CTL.bit.TRIGSEL  = 0x03; //					// hierdie was 0x03
	AdcRegs.ADCSOC0CTL.bit.ACQPS    = 20;   //set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
	AdcRegs.ADCSOC1CTL.bit.ACQPS    = 20;   //set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)

	//channel B4  -  Aux measure
	AdcRegs.ADCSOC2CTL.bit.CHSEL    = 0x0C;
	AdcRegs.ADCSOC2CTL.bit.TRIGSEL  = 0x02;
	AdcRegs.ADCSOC2CTL.bit.ACQPS    = 20;

	//Temp measure
	//channel A7 - SOC0 - Cell0
	AdcRegs.ADCSOC0CTL.bit.CHSEL    = 0x07;
	AdcRegs.ADCSOC0CTL.bit.TRIGSEL  = 0x01;
	AdcRegs.ADCSOC0CTL.bit.ACQPS    = 20;

	//channel A6 - SOC3 - Cell1
	AdcRegs.ADCSOC3CTL.bit.CHSEL    = 0x06;
	AdcRegs.ADCSOC3CTL.bit.TRIGSEL  = 0x01;
	AdcRegs.ADCSOC3CTL.bit.ACQPS    = 20;

	//channel A5 - SOC4 - Cell2
	AdcRegs.ADCSOC4CTL.bit.CHSEL    = 0x05;
	AdcRegs.ADCSOC4CTL.bit.TRIGSEL  = 0x01;
	AdcRegs.ADCSOC4CTL.bit.ACQPS    = 20;

	//channel A4 - SOC5 - Cell3
	AdcRegs.ADCSOC5CTL.bit.CHSEL    = 0x04;
	AdcRegs.ADCSOC5CTL.bit.TRIGSEL  = 0x01;
	AdcRegs.ADCSOC5CTL.bit.ACQPS    = 20;

	//channel A3 - SOC6 - Cell5
	AdcRegs.ADCSOC6CTL.bit.CHSEL    = 0x03;
	AdcRegs.ADCSOC6CTL.bit.TRIGSEL  = 0x01;
	AdcRegs.ADCSOC6CTL.bit.ACQPS    = 20;

	//channel A2 - SOC7 - Cell6
	AdcRegs.ADCSOC7CTL.bit.CHSEL    = 0x02;
	AdcRegs.ADCSOC7CTL.bit.TRIGSEL  = 0x01;
	AdcRegs.ADCSOC7CTL.bit.ACQPS    = 20;

	//channel A1 - SOC8 - Cell7
	AdcRegs.ADCSOC8CTL.bit.CHSEL    = 0x01;
	AdcRegs.ADCSOC8CTL.bit.TRIGSEL  = 0x01;
	AdcRegs.ADCSOC8CTL.bit.ACQPS    = 20;

	//channel A0 - SOC9 - Cell8
	AdcRegs.ADCSOC9CTL.bit.CHSEL    = 0x0;
	AdcRegs.ADCSOC9CTL.bit.TRIGSEL  = 0x01;
	AdcRegs.ADCSOC9CTL.bit.ACQPS    = 20;

	//channel B0 - SOC10 - Cell10									//possibly also use for BMS model detection
	AdcRegs.ADCSOC10CTL.bit.CHSEL    = 0x08;
	AdcRegs.ADCSOC10CTL.bit.TRIGSEL  = 0x01;
	AdcRegs.ADCSOC10CTL.bit.ACQPS    = 20;

	//channel B1 - SOC11 - Cell11
	AdcRegs.ADCSOC11CTL.bit.CHSEL    = 0x09;
	AdcRegs.ADCSOC11CTL.bit.TRIGSEL  = 0x01;
	AdcRegs.ADCSOC11CTL.bit.ACQPS    = 20;

	//channel B2 - SOC12 - Cell12
	AdcRegs.ADCSOC12CTL.bit.CHSEL    = 0x0A;
	AdcRegs.ADCSOC12CTL.bit.TRIGSEL  = 0x01;
	AdcRegs.ADCSOC12CTL.bit.ACQPS    = 20;

	//channel B5 - SOC13 - Cell13
	AdcRegs.ADCSOC13CTL.bit.CHSEL    = 0x0D;
	AdcRegs.ADCSOC13CTL.bit.TRIGSEL  = 0x01;
	AdcRegs.ADCSOC13CTL.bit.ACQPS    = 20;

	//channel B6 - SOC14 - Cell14
	AdcRegs.ADCSOC14CTL.bit.CHSEL    = 0x0E;
	AdcRegs.ADCSOC14CTL.bit.TRIGSEL  = 0x01;
	AdcRegs.ADCSOC14CTL.bit.ACQPS    = 20;

	//channel B7 - SOC15 - Cell15
	AdcRegs.ADCSOC15CTL.bit.CHSEL    = 0x0F;
	AdcRegs.ADCSOC15CTL.bit.TRIGSEL  = 0x01;
	AdcRegs.ADCSOC15CTL.bit.ACQPS    = 20;

	EDIS;
}

void Reset_ADC(void)
{
	I2CA_WriteData(0x04,0x8);
	I2CA_WriteData(0x04,0x18);

	I2CA_WriteData(0x05,0x03);			//testing
}

void Reset_MCU(int flag)
{
    static int delay = 0;

    if(ECanaRegs.CANES.bit.EP == 1)
        delay++;
    else
        delay=0;

    if(flag == 1 && delay > 300 && ECanaRegs.CANES.bit.EP == 1 && Key_switch_2 == 0)   //5 min intervals
        for(;;){}
}
