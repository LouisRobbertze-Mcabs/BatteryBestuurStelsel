/*
 * User_Interrupts.c
 *
 *  Created on: 04 May 2017
 *      Author: Bartho Horn
 */

#include "User_Interrupts.h"

__interrupt void  adc_isr(void)
{
	//Sit dit dalk deur 'n laag deurlaat filter y(k) = y(k - 1) + a[x(k) - y(k - 1)] met a = 1 - e^(-WcTs)

	static long int Filter_SC;
	static long int Filter_SC_past = 0;
	static float current_p;

	static long trip_timer;

	test_current = current_p + (0.00314*(AdcResult.ADCRESULT1-current_p));     		//   0.00314-1Hz     //  0.01249 - 4 Hz      //0.27-100Hz
	current_p=test_current;

	////////////
	//insert limits here
	//4095 = maksimum current -> 250??
	//3686 -> 4.5V = 200 A
	//3359 -> 4.1V = 160 A
	//3031 -> 3.7V = 120 A
	//2048 -> 2.5V= 0 A
	//1065 -> 1.3V = -120
	//410 -> 0.5V = -200A
	//0 -> 0V = -250A

	Filter_SC = (100*Filter_SC_past + (27*(AdcResult.ADCRESULT1-Filter_SC_past)))/100;	   //   0.00314-1Hz     //  0.01249 - 4 Hz      //0.27-100Hz
	Filter_SC_past=Filter_SC;

	//Short circuit fault - 100 Hz cut-off
	if(Filter_SC > Imax || Filter_SC < Imin)
	{
		ContactorOut = 0;       														//turn off contactor
		flagCurrent = 1;
	}
	else if(Filter_SC <= 3031 && Filter_SC > 2048)										//current between 0A and 120 A
	{
		trip_timer = interpolate_table_1d(&trip2_table, Filter_SC);						//linear cooling -- straight line
		trip_counter = trip_counter - (1200000/trip_timer);
	}
	else if(Filter_SC <= 2048 && Filter_SC > 1065)										//current between 0A and -120 A
	{
		trip_timer = interpolate_table_1d(&trip2_table, (4096-Filter_SC));				//linear cooling -- straight line
		trip_counter = trip_counter - (1200000/trip_timer);
	}
	else if(Filter_SC > 3031)															//current larger than 120 A
	{
		trip_timer = interpolate_table_1d(&trip_table, Filter_SC);						//Non-linear heating
		trip_counter = trip_counter + (1200000/trip_timer);
		/*		if( Filter_SC> testvariable2)
			testvariable2 = Filter_SC;
		testvariable++;*/
	}
	else																				//current smaller than -120 A
	{
		trip_timer = interpolate_table_1d(&trip_table, (4096-Filter_SC));				//Non-linear heating
		trip_counter = trip_counter + (1200000/trip_timer);
	}

	//if(trip_counter > testvariable)
	//testvariable = trip_counter;


	if(trip_counter > 1200000)
	{
		ContactorOut = 0;       														//turn off contactor
		flagCurrent = 1;
	}

	if(trip_counter < 0)																//counter out of bounds
		trip_counter = 0;

	//timecounter++;

	AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;       //Clear ADCINT1 flag reinitialize for next SOC
	PieCtrlRegs.PIEACK.bit.ACK10 = 1;   		// Acknowledge interrupt to PIE
}

__interrupt void cpu_timer0_isr(void)
{
	counter_2Hz++;
	CpuTimer0.InterruptCount++;
	PieCtrlRegs.PIEACK.bit.ACK1 = 1/* PIEACK_GROUP1*/;
}

__interrupt void cpu_timer1_isr(void)
{
	//check status of all flags as well as the key switch
	static float Aux_Voltage_temp = 0;

	//Deurlaat filter y(k) = y(k - 1) + a[x(k) - y(k - 1)] met a = 1 - e^-WcTs
	//adc/4096 *3.3* 10.51/10.51      12.2/2.2
	//a = 0.015 ~ 0.1Hz, a = 0.12 ~ 1Hz, a = 0.47 ~ 5Hz

	Auxilliary_Voltage = Aux_Voltage_temp + (0.47*(((AdcResult.ADCRESULT2)* 0.00442)-Aux_Voltage_temp));					//50hz sny af op 0.1hz
	Aux_Voltage_temp = Auxilliary_Voltage;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////// testing
	if(KeySwitch == 1 && System_State == 1)  //keyswitch == 1
	{
		//binne die keydrive if
		if((flagDischarged == 0) && (flagCurrent == 0)  && (flagTemp == 0) && (Charger_status == 0))
		{
			ContactorOut = 1;           //turn on contactor
		}
		else
		{
			//Maybe this should just always show????
			ContactorOut = 0;           //turn off contactor
			//led3 = 1;
		}
	}
	else if((KeySwitch == 0) && (Charger_status == 0)) //keyswitch == 0
	{
		flagCurrent = 0;
		ContactorOut = 0;       //turn off contactor
		//led3 = 0;       		//turn off red led
	}

	if(flagCurrent == 1)
		led3 = 1;

	if(flagDischarged == 1 || flagDischarged == 2)
		led2 = 1;

	if(flagTemp == 1)
	{
		led3 = 1;
		led2 = 1;
	}

	EALLOW;
	CpuTimer1.InterruptCount++;
	EDIS;
}

__interrupt void cpu_timer2_isr(void)
{
	EALLOW;
	//counter
	if(KeySwitch == 1 && flagCurrent == 0)
		testcounter++;
	CpuTimer2.InterruptCount++;
	EDIS;
}

__interrupt void i2c_int1a_isr(void)     														// I2C-A
{
	Uint16 IntSource;																			//reset all during whatchdog restart???

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
	//was receive successful? maybe use CANES.All < 3

	if (ECanaRegs.CANRMP.bit.RMP1 == 1)
	{
		//CANSlaveReception();                    // Handle the received message
	}
	else if (ECanaRegs.CANRMP.bit.RMP2 == 1)
	{
		//CANChargerReception();
		CAN_Charger_dataL = ECanaMboxes.MBOX2.MDL.all;                // Data taken out of direct mailbox
		CAN_Charger_dataH = ECanaMboxes.MBOX2.MDH.all;                // Data taken out of direct mailbox
	}


	ECanaRegs.CANRMP.all = 0xFFFFFFFF;          // Reset receive mailbox flags
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;     // Acknowledge this interrupt to receive more interrupts from group 9
}

__interrupt void can_tx_isr(void)
{

	ECanaRegs.CANTA.all = 0xFFFFFFFF;           // Reset tranmission flags
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;     // Acknowledge this interrupt to receive more interrupts from group 9
}
