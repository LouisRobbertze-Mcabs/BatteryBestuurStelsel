/*
 * User_Interrupts.c
 *
 *  Created on: 04 May 2017
 *      Author: Sonja
 */

#include "User_Interrupts.h"

__interrupt void  adc_isr(void)
{
	//Sit dit dalk deur 'n laag deurlaat filter y(k) = y(k - 1) + a[x(k) - y(k - 1)] met a = 1 - e^WcTs

	static float Filter_100HZ;
	static float Filter_100HZ_past = 0;
	static float current_p;
	//  int toets_stroom = 0;


	test_current = current_p + (0.00314*(AdcResult.ADCRESULT1-current_p));     //   0.00314-1Hz     //  0.01249 - 4 Hz      //0.27-100Hz
	current_p=test_current;


	Filter_100HZ = Filter_100HZ_past + (Ifilter*(AdcResult.ADCRESULT1-Filter_100HZ_past));     //   0.00314-1Hz     //  0.01249 - 4 Hz      //0.27-100Hz
	Filter_100HZ_past=Filter_100HZ;

	//   Filter_100HZ = (test_current-2109)* 0.122;

	if(Filter_100HZ > Imax || Filter_100HZ < Imin)                       ////////////////////////////////////////////////
	{
		//sit uittree af
		ContactorOut = 0;       //turn off contactor
		flagCurrent = 1;
	} 																	////////////////////////////////////////////////


	Current_Sum = Current_Sum + AdcResult.ADCRESULT1;
	Current_Counter++;


	AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;       //Clear ADCINT1 flag reinitialize for next SOC
	PieCtrlRegs.PIEACK.bit.ACK10 = 1;   // Acknowledge interrupt to PIE

	led2 = 0x01^led2;                   //toggle led
}

__interrupt void cpu_timer0_isr(void)
{
	static float Temperatures_resistance_temp[14];

	counter_2Hz++;

	//temp meetings
	//SOC15 - Outside
	Temperatures_resistance[13] = Temperatures_resistance_temp[13] + (0.466*(((AdcResult.ADCRESULT15))-Temperatures_resistance_temp[13]));
	Temperatures_resistance_temp[13] = Temperatures_resistance[13];

	//Cells:
	//SOC14 - Cell0
	Temperatures_resistance[0] = Temperatures_resistance_temp[0] + (0.466*(((AdcResult.ADCRESULT14))-Temperatures_resistance_temp[0]));
	Temperatures_resistance_temp[0] = Temperatures_resistance[0];
	//SOC13 - Cell1
	Temperatures_resistance[1] = Temperatures_resistance_temp[1] + (0.466*(((AdcResult.ADCRESULT13))-Temperatures_resistance_temp[1]));
	Temperatures_resistance_temp[1] = Temperatures_resistance[1];
	//SOC12 - Cell2
	Temperatures_resistance[2] = Temperatures_resistance_temp[2] + (0.466*(((AdcResult.ADCRESULT12))-Temperatures_resistance_temp[2]));
	Temperatures_resistance_temp[2] = Temperatures_resistance[2];
	//SOC11 - Cell3
	Temperatures_resistance[3] = Temperatures_resistance_temp[3] + (0.466*(((AdcResult.ADCRESULT11))-Temperatures_resistance_temp[3]));
	Temperatures_resistance_temp[3] = Temperatures_resistance[3];
	//SOC10 - Cell4
	Temperatures_resistance[4] = Temperatures_resistance_temp[4] + (0.466*(((AdcResult.ADCRESULT10))-Temperatures_resistance_temp[4]));
	Temperatures_resistance_temp[4] = Temperatures_resistance[4];
	//SOC9 - Cell6
	Temperatures_resistance[5] = Temperatures_resistance_temp[5] + (0.466*(((AdcResult.ADCRESULT9))-Temperatures_resistance_temp[5]));
	Temperatures_resistance_temp[5] = Temperatures_resistance[5];
	//SOC8 - Cell7
	Temperatures_resistance[6] = Temperatures_resistance_temp[6] + (0.466*(((AdcResult.ADCRESULT8))-Temperatures_resistance_temp[6]));
	Temperatures_resistance_temp[6] = Temperatures_resistance[6];
	//SOC7 - Cell8
	Temperatures_resistance[7] = Temperatures_resistance_temp[7] + (0.466*(((AdcResult.ADCRESULT7))-Temperatures_resistance_temp[7]));
	Temperatures_resistance_temp[7] = Temperatures_resistance[7];
	//SOC6 - Cell9
	Temperatures_resistance[8] = Temperatures_resistance_temp[8] + (0.466*(((AdcResult.ADCRESULT6))-Temperatures_resistance_temp[8]));
	Temperatures_resistance_temp[8] = Temperatures_resistance[8];
	//SOC5 - Cell11
	Temperatures_resistance[9] = Temperatures_resistance_temp[9] + (0.466*(((AdcResult.ADCRESULT5))-Temperatures_resistance_temp[9]));
	Temperatures_resistance_temp[9] = Temperatures_resistance[9];
	//SOC5 - Cell12
	Temperatures_resistance[10] = Temperatures_resistance_temp[10] + (0.466*(((AdcResult.ADCRESULT4))-Temperatures_resistance_temp[10]));
	Temperatures_resistance_temp[10] = Temperatures_resistance[10];
	//SOC5 - Cell13
	Temperatures_resistance[11] = Temperatures_resistance_temp[11] + (0.466*(((AdcResult.ADCRESULT3))-Temperatures_resistance_temp[11]));
	Temperatures_resistance_temp[11] = Temperatures_resistance[11];
	//SOC5 - Cell14
	Temperatures_resistance[12] = Temperatures_resistance_temp[12] + (0.466*(((AdcResult.ADCRESULT0))-Temperatures_resistance_temp[12]));
	Temperatures_resistance_temp[12] = Temperatures_resistance[12];

	CpuTimer0.InterruptCount++;
	PieCtrlRegs.PIEACK.bit.ACK1 = 1/* PIEACK_GROUP1*/;
}

__interrupt void cpu_timer1_isr(void)
{
	//check status of all flags as well as the key switch
	counter_50Hz++;
	static float Aux_Voltage_temp = 0;

	//Deurlaat filter y(k) = y(k - 1) + a[x(k) - y(k - 1)] met a = 1 - e^WcTs

	//adc/4096 *3.3* 10.51/10.51      12.2/2.2


	Auxilliary_Voltage = Aux_Voltage_temp + (0.0609*(((AdcResult.ADCRESULT2)* 0.00442)-Aux_Voltage_temp));					//maak miskien gebruik van die config leer
	Aux_Voltage_temp = Auxilliary_Voltage;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////// testing
	if(KeySwitch == 1)  //keyswitch == 1
	{
		//  led3 = 1;       //turn on red led

		//binne die keydrive if
		if((flagDischarged == 0) && (flagCurrent == 0)  && (flagTemp == 0) && (Charger_status == 0))
		{

			ContactorOut = 1;           //turn on contactor
		}

	}
	else if((KeySwitch == 0) && (Charger_status == 0)) //keyswitch == 0
	{
		flagCurrent = 0;

		ContactorOut = 0;       //turn off contactor

		//  led3 = 0;       //turn off red led
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	CpuTimer1.InterruptCount++;
	EDIS;
}

__interrupt void cpu_timer2_isr(void)
{
	EALLOW;

	CpuTimer2.InterruptCount++;
	EDIS;
}

__interrupt void i2c_int1a_isr(void)     // I2C-A
{
	Uint16 IntSource;

	// Read interrupt source
	IntSource = I2caRegs.I2CISRC.all;

	// Interrupt source = stop condition detected
	if(IntSource == I2C_SCD_ISRC)
	{
		// If completed message was writing data, reset msg to inactive state
		if (CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_WRITE_BUSY)
		{
			CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_INACTIVE;
			//I2cMsgIn1.MsgStatus = I2C_MSGSTAT_SEND_NOSTOP;
		}
		else
		{
			// If a message receives a NACK during the address setup portion of the
			// EEPROM read, the code further below included in the register access ready
			// interrupt source code will generate a stop condition. After the stop
			// condition is received (here), set the message status to try again.
			// User may want to limit the number of retries before generating an error.
			if(CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP_BUSY)
			{
				CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_RESTART;
			}
			// If completed message was reading EEPROM data, reset msg to inactive state
			// and read data from FIFO.
			else if (CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_READ_BUSY)
			{
				CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_INACTIVE;

				DataOut = I2caRegs.I2CDRR;
				DataOut2 = I2caRegs.I2CDRR;
			}
		}
	}  // end of stop condition detected
	else if(IntSource == 2)												//no acknowledge condition
	{
		// Generate some error due to invalid interrupt source
		CurrentMsgPtr->MsgStatus = 0xFF;
	}
	else
	{
		// Generate some error due to invalid interrupt source
		//__asm("   ESTOP0");
		CurrentMsgPtr->MsgStatus = 0xFF;
	}
	// Enable future I2C (PIE Group 8) interrupts
	PieCtrlRegs.PIEACK.bit.ACK8 = 1;
}

__interrupt void can_rx_isr(void)
{
	if (ECanaRegs.CANRMP.bit.RMP1 == 1)
	{
		CANSlaveReception();                        // Handle the received message
	}
	else if (ECanaRegs.CANRMP.bit.RMP2 == 1)
	{
		CANChargerReception();
	}
	else if(ECanaRegs.CANRMP.bit.RMP3 == 1)
	{
		CANSlaveConfig();
	}

	ECanaRegs.CANRMP.all = 0xFFFFFFFF;          // Reset receive mailbox flags
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;         // Acknowledge this interrupt to receive more interrupts from group 9
}

__interrupt void can_tx_isr(void)
{
	/*if (ECanaRegs.CANTA.all == 0x00000001)
    {
        ECanaRegs.CANTA.all = 0xFFFFFFFF;           // Reset tranmission flags
    }*/


	ECanaRegs.CANTA.all = 0xFFFFFFFF;           // Reset tranmission flags
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;         // Acknowledge this interrupt to receive more interrupts from group 9
}

