/*
 * User_Functions.c
 *
 *  Created on: 04 May 2017
 *      Author: Sonja
 */

#include "User_Defines.h"

void Initialise_BMS(void)
{
	flagCurrent = 0;

	InitSysCtrl();
	InitI2CGpio();
	Init_Gpio();
	InitAdc();

	DINT;

	InitPieCtrl();

	IER = 0x0000;
	IFR = 0x0000;

	InitPieVectTable();

	EALLOW; // This is needed to write to EALLOW protected registers
	PieVectTable.I2CINT1A = &i2c_int1a_isr;
	PieVectTable.TINT0 = &cpu_timer0_isr;
	PieVectTable.TINT1 = &cpu_timer1_isr;
	PieVectTable.TINT2 = &cpu_timer2_isr;
	PieVectTable.ADCINT1 = &adc_isr;
	EDIS;   // This is needed to disable write to EALLOW protected registers

	I2CA_Init();
	InitCpuTimers();

	ConfigCpuTimer(&CpuTimer0, 60, 500000); //2 hz
	ConfigCpuTimer(&CpuTimer1, 60, 20000);  //50 hz
	ConfigCpuTimer(&CpuTimer2, 60, 500);    //2 Khz         //500

	CpuTimer0Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0
	CpuTimer1Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0
	CpuTimer2Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0

	// Enable ADC interrupt 10 in the PIE: Group 10 interrupt 1
	PieCtrlRegs.PIEIER10.bit.INTx1 = 1;
	IER |= M_INT10;

	// Enable CPU interrupt 1 in the PIE: Group 1 interrupt 7
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;          //clock
	IER |= M_INT1;
	IER |= M_INT13;
	IER |= M_INT14;

	// Enable I2C interrupt 1 in the PIE: Group 8 interrupt 1
	PieCtrlRegs.PIEIER8.bit.INTx1 = 1;
	// Enable I2C INT8 which is connected to PIE group 8
	IER |= M_INT8;

	EnableInterrupts();

	EINT;
	ERTM;   // Enable Global realtime interrupt DBGM

	CAN_Init();
	configADC();
	Bq76940_Init();
	// Shut_D_BQ();

	Calibrate_Current();

	// Enable the watchdog
	EALLOW;
	SysCtrlRegs.WDCR = 0x002F;
	EDIS;
	//watchdog timer>>>>>>>>
	//DisableDog();
}

void Init_Gpio(void)
{
	EALLOW;

	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;     //BQenable
	GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;      //Bq enable

	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;     //led2
	GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;      //led2

	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;     //led1
	GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;      //led1

	GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;     //BT reset
	GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;      //BT reset
	BTReset = 0;                            //keep BT in reset

	GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;    //12 Aux drive
	GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;     // 12 Aux drive (verander miskien)
	Aux_Control = 1;

	//	GpioCtrlRegs.GPAPUD.bit.GPIO19 = 1;    // Enable pull-up for GPIO19
	GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;    //KeyDrive
	GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;     //(Output) key drive (raak nou fan control)
	Fan_Control = 0; 						//turn off fan for now

	GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;    //contactor output
	GpioCtrlRegs.GPADIR.bit.GPIO20 = 1;     // contactor output
	ContactorOut = 0;                       //turn off contactor

	GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0;    //precharge resistor
	GpioCtrlRegs.GPADIR.bit.GPIO21 = 1;     // precharge resistor
	PreCharge = 1;                          //turn on precharge resistor

	GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0;    //key switch
	GpioCtrlRegs.GPADIR.bit.GPIO24 = 0;     //key switch

	GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;    //BQ on
	GpioCtrlRegs.GPADIR.bit.GPIO26 = 0;     //BQ on (input)

	GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;    //CANenable
	GpioCtrlRegs.GPADIR.bit.GPIO27 = 1;     //CANenable (output)

	GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 0;    //led3
	GpioCtrlRegs.GPBDIR.bit.GPIO40 = 1;     //led3

	GpioCtrlRegs.GPBMUX1.bit.GPIO44 = 0;    //CScontrol
	GpioCtrlRegs.GPBDIR.bit.GPIO44 = 1;     //CSControl

	EDIS;

	CSControl = 0;  //turn CScontrol on for current measurement

	//turn off contactor
	ContactorOut = 0;
	//turn off PreCharge
	PreCharge = 0;
}

void Toggle_LED(void)
{
	GpioDataRegs.GPATOGGLE.bit.GPIO5 = 1;
	//GpioDataRegs.GPATOGGLE.bit.GPIO19 = 1;
}

void  Read_Cell_Voltages(void)
{
	// Read data from EEPROM section //
	int i;
	Voltage_total = 0;
	//reset values
	//Voltage_low = 10;
	float Voltages_backup5 = Voltages[5];
	float Voltages_backup10 = Voltages[10];

	float temp_V = 0;
	float temp_Voltage_high;
	temp_Voltage_high = 0;
	float temp_Voltage_low;
	temp_Voltage_low = 10;


	for(i = 0; i<15; i++)
	{
		temp_V = I2CA_ReadData(&I2cMsgIn1,0x0C+(i*0x02), 2);
		temp_V = (ADCgain * temp_V) + ADCoffset;

		if((i == 5) && ((Cell_B1 & 0x10)>>4 == 1))
			Voltages[i] = Voltages_backup5;
		else if(i == 5)
			Voltages[i] = temp_V;

		if((i == 10) && ((Cell_B2&0x10)>>4 == 1))
			Voltages[i] = Voltages_backup10;
		else if (i==10)
			Voltages[i] = temp_V;

		if(i != 5 && i != 10)
			Voltages[i] = temp_V;

		Voltage_total = Voltage_total +  Voltages[i];

		if(temp_Voltage_high<Voltages[i])
			temp_Voltage_high = Voltages[i];

		if(temp_Voltage_low>Voltages[i])
			temp_Voltage_low = Voltages[i];
	}

	Voltage_high = temp_Voltage_high;
	Voltage_low = temp_Voltage_low;
}

void Process_Voltages(void)
{
	//	static int delay = 0;
	if(Voltage_high > Vmax)         //3.6
	{
		balance = 1;            //start balancing
		flagCharged = 1;        //charged flag to to stop charging
		ContactorOut = 0;
	}

	if(Voltage_low > Vmin && Auxilliary_Voltage < Vauxmin && Auxilliary_Voltage > 8)
	{
		Auxilliary_counter = 0;			//turn on aux supply
		Aux_Control = 1;
	}
	else if(Auxilliary_counter > AuxChargeTime || Auxilliary_Voltage < 8)
	{
		Aux_Control = 0;										//turn off aux supply
	}

	Auxilliary_counter++;


	if(Voltage_low < Vmin && Voltage_low > Vcritical && Charger_status == 0)
	{
		Aux_Control = 0;
		flagDischarged = 1;
		led3 = 1;               //turn on red led
		ContactorOut = 0;       //turn off contactor            //turn off output
	}
	else if(Voltage_low < Vcritical && Charger_status == 0)
	{
		Aux_Control = 0;
		flagDischarged = 2;
		led3 = 1;               //turn on red led
		PreCharge = 0;
	}

	if(Voltage_high<Vchargedflagreset )
		flagCharged = 0;

	if(Voltage_low>Vdischargedflagreset )
	{
		flagDischarged = 0;
		led3 = 0;               //turn off red led
	}

}

void Calculate_Current(void)
{
	Current = (test_current-Current_CAL)* 0.122;                   //2090    maal, moenie deel nie!!!!     0.0982--200/2048
}

void Read_System_Status(void)
{
	system_status = I2CA_ReadData(&I2cMsgIn1,0x00, 1);

}

void Process_System_Status(void)
{
	if(system_status != 00)
	{
		I2CA_WriteData(0x00, 0x3F);
	}
}

void Read_Temperatures(void)
{
	int i;
	int flag = 0;
	float Vts;
	float Rts;
	float temperature_avg=0;
	float temp_Temperature_high = 0;
	float temp_Temperature_low = 71;

	float temp_T = 0;

	//cells 0-3
	for(i = 0; i<4; i++)
	{
		Vts = (Temperatures_resistance[i]) * 0.00080566;
		Rts = (33000/Vts) - 10000;
		Temperatures[i] = (1/((log(Rts/10000))/4000+0.003356))-273;
	}

	//cells 5-8
	for(i = 5; i<9; i++)
	{
		Vts = (Temperatures_resistance[i-1]) * 0.00080566;
		Rts = (33000/Vts) - 10000;
		Temperatures[i] = (1/((log(Rts/10000))/4000+0.003356))-273;
	}

	//cells 10-14
	for(i = 10; i<15; i++)
	{
		Vts = (Temperatures_resistance[i-2]) * 0.00080566;
		Rts = (33000/Vts) - 10000;
		Temperatures[i] = (1/((log(Rts/10000))/4000+0.003356))-273;
	}

	//cell5,cell10
	temp_T = I2CA_ReadData(&I2cMsgIn1, 0x2E, 2);
	Vts = temp_T*ADCgain;
	Rts = (10000*Vts)/(3.3-Vts);
	Temperatures[4] = (1/((log(Rts/10000))/4000+0.003356))-273;

	temp_T = I2CA_ReadData(&I2cMsgIn1, 0x30, 2);
	Vts = temp_T*ADCgain;
	Rts = (10000*Vts)/(3.3-Vts);
	Temperatures[9] = (1/((log(Rts/10000))/4000+0.003356))-273;


	//Outside
	Vts = (Temperatures_resistance[13]) * 0.00080566;
	Rts = (33000/Vts) - 10000;
	Temperatures[15] = (1/((log(Rts/10000))/4000+0.003356))-273;

	for(i = 0; i<15; i++)
	{
		temperature_avg = temperature_avg+Temperatures[i];

		if(temp_Temperature_high<Temperatures[i])
			temp_Temperature_high = Temperatures[i];

		if(temp_Temperature_low>Temperatures[i])
			temp_Temperature_low = Temperatures[i];
	}
	Temperature_avg = temperature_avg*0.0667;


	//	if(temp_Temperature_high>70)					//old system filter
	//	{
	//		Temperature_avg = Temperatures[4];
	//		Temperature_high = Temperatures[9];
	//		Temperatures[15] = 0;

	if(Temperatures[4]> Tmax || Temperatures[4]<Tmin)
	{
		flag = 1;
	}

	if(Temperatures[9]> Tmax || Temperatures[9]<Tmin)
	{
		flag = 1;
	}
	/*	}
	else											//system normal
	{
		Temperature_avg = temperature_avg;
		Temperature_high = temp_Temperature_high;
		Temperature_low = temp_Temperature_low;

		if(Temperature_high> Tmax || Temperature_low<Tmin)
		{
			flagTemp = 1;
		}

		if((Temperature_avg - Temperatures[15])> 5)
		{
			Fan_Control = 1;
		}
		else
			Fan_Control = 0;
	}
	 */
	if(flag == 1)
		flagTemp = 1;
	else if(flag == 0)
		flagTemp = 0;
}

void Balance(int period, float reference)
{
	static float count = 0;
	float Cell_B_Voltage = reference;
	int i;

	if(balance == 1)
	{
		if(count ==0)       // 4 siklusse met onewe selle
		{
			//reset all balancing
			I2CA_WriteData(0x01,0x00);
			I2CA_WriteData(0x02,0x00);
			I2CA_WriteData(0x03,0x00);
			Cell_B1 = 0;
			Cell_B2 = 0;
			Cell_B3 = 0;

			//setup balancing
			for(i = 0; i<5; i+=2)
			{
				if(Voltages[i] > Cell_B_Voltage)
				{
					Cell_B1 = (Cell_B1 | (0x01 << i));
				}
			}

			for(i = 5; i<10; i+=2)
			{
				if(Voltages[i] > Cell_B_Voltage)
				{

					Cell_B2 = (Cell_B2 | (0x01 << (i-5)));
				}
			}

			for(i = 10; i<15; i+=2)
			{
				if(Voltages[i] > Cell_B_Voltage)
				{

					Cell_B3 = (Cell_B3 | (0x01 << (i-10)));
				}
			}

			if(Cell_B1==0x0 && Cell_B2==0x0 && Cell_B3==0x0)
			{
				count = period;
			}
			else
			{
				I2CA_WriteData(0x01,Cell_B1);
				I2CA_WriteData(0x02,Cell_B2);
				I2CA_WriteData(0x03,Cell_B3);
				count++;
			}

		}
		else if(count == period) // 4 siklusse met ewe selle
		{
			//reset all balancing
			I2CA_WriteData(0x01,0x00);
			I2CA_WriteData(0x02,0x00);
			I2CA_WriteData(0x03,0x00);
			Cell_B1 = 0;
			Cell_B2 = 0;
			Cell_B3 = 0;

			//setup balancing
			for(i = 1; i<5; i+=2)
			{
				if(Voltages[i] > Cell_B_Voltage)
				{
					Cell_B1 = (Cell_B1 | (0x01 << i));
				}
			}

			for(i = 6; i<10; i+=2)
			{
				if(Voltages[i] > Cell_B_Voltage)
				{

					Cell_B2 = (Cell_B2 | (0x01 << (i-5)));
				}
			}

			for(i = 11; i<15; i+=2)
			{
				if(Voltages[i] > Cell_B_Voltage)
				{

					Cell_B3 = (Cell_B3 | (0x01 << (i-10)));
				}
			}

			if(Cell_B1==0x0 && Cell_B2==0x0 && Cell_B3==0x0)
			{
				count = period*2;
			}
			else
			{
				I2CA_WriteData(0x01,Cell_B1);
				I2CA_WriteData(0x02,Cell_B2);
				I2CA_WriteData(0x03,Cell_B3);
				count++;
			}
		}
		else if(count == period*2)
		{
			//speel maar so bietjie
			balance = 0;

			for(i = 0; i<15; i++)
			{
				if(Voltages[i] > Cell_B_Voltage)
				{
					balance++;
				}
			}

			if(balance !=0)
			{
				balance = 1;
			}
			else if(balance ==0)                //sit miskien else hier             hierdie is toets fase
			{
				balance = 0;
			}

			count = 0;
			I2CA_WriteData(0x01,0x00);
			I2CA_WriteData(0x02,0x00);
			I2CA_WriteData(0x03,0x00);
			Cell_B1 = 0;
			Cell_B2 = 0;
			Cell_B3 = 0;
		}
		else
		{
			count++;
		}
	}
	else
	{
		I2CA_WriteData(0x01,0x00);
		I2CA_WriteData(0x02,0x00);
		I2CA_WriteData(0x03,0x00);
		Cell_B1 = 0;
		Cell_B2 = 0;
		Cell_B3 = 0;
	}
}

unsigned char CRC8(unsigned char *ptr, unsigned char len,unsigned char key)
{
	unsigned char i;
	unsigned char crc=0;
	while(len--!=0)
	{
		for(i=0x80; i!=0; i/=2)
		{
			if((crc & 0x80) != 0)
			{
				crc *= 2;
				crc ^= key;
			}
			else
				crc *= 2;

			if((*ptr & i)!=0)
				crc ^= key;
		}
		ptr++;
	}
	return(crc);
}

Uint32 ChgCalculator(float Voltage, float Current)
{
	Uint32 answer= 0;
	Uint16 temp1 = 0;

	Voltage = Voltage*10;
	Current = Current*10;

	temp1 = (Uint16)Current;

	answer = answer | (temp1&0xFF);
	answer = (answer<<8) | (temp1>>8);

	temp1 = (Uint16)Voltage;

	answer = answer<<8 | (temp1&0xFF);
	answer = (answer<<8) | (temp1>>8);

	return answer;
}

/**
 * Returns the interpolated y-value.
 * Saturates to y0 or y1 if x outside interval [x0, x1].
 */
float interpolate_segment(float x0, float y0, float x1, float y1, float x)
{
	float t;

	if (x <= y0) { return x0; }
	if (x >= y1) { return x1; }

	t =  (x-y0);
	t /= (y1-y0);

	return x0 + t*(x1-x0);
}
/******************************************************************************/

float interpolate_table_1d(struct table_1d *table, float x)
/* 1D Table lookup with interpolation */
{
	Uint16 segment;

	/* Check input bounds and saturate if out-of-bounds */
	if (x > (table->y_values[table->x_length-1])) {
		/* x-value too large, saturate to max y-value */
		return table->x_values[table->x_length-1];
	}
	else if (x < (table->y_values[0])) {
		/* x-value too small, saturate to min y-value */
		return table->x_values[0];
	}

	/* Find the segment that holds x */
	for (segment = 0; segment<(table->x_length-1); segment++)
	{
		if ((table->y_values[segment]   <= x) &&
				(table->y_values[segment+1] >= x))
		{
			/* Found the correct segment */
			/* Interpolate */
			return interpolate_segment(table->x_values[segment],   /* x0 */
					table->y_values[segment],   /* y0 */
					table->x_values[segment+1], /* x1 */
					table->y_values[segment+1], /* y1 */
					x);                         /* x  */
		}
	}

	/* Something with the data was wrong if we get here */
	/* Saturate to the max value */
	return table->x_values[table->x_length-1];
}
/******************************************************************************/

void Calculate_SOC()
{
	float SOCv;
	static float SOCc;
//	static Uint16 t=1000;							//time since current activity
	float Wsoc;										//weighting parameter

	SOCv = interpolate_table_1d(&sine_table, Voltage_low);

	SOC_t++;

	if(Current>4 || Current<-4)
	{
		SOC_t = 0;
	}
	SOCc = SOC - Current*0.00000185;				//coulomb counter      Ampere sec -> Ampere huur						1/150A*3600s


	Wsoc = exp(-SOC_t*0.000463);						//halfuur inplaas van 2 ure?	0.000833		     0.002778 - 30 min

	SOC = Wsoc*SOCc + (1-Wsoc)*SOCv;

	if(SOC>1)
		SOC = 1;
	else if(SOC<0)
		SOC = 0;
}

void Calibrate_Current()
{
	// Reset the watchdog counter
	ServiceDog();

	Current_Sum = 0;
	Current_Counter = 0;

	while(Current_Counter <= 500);

	Current_CAL = Current_Sum * 0.002;
	PreCharge = 1;                          //turn on precharge resistor
	// Reset the watchdog counter
	ServiceDog();
}
