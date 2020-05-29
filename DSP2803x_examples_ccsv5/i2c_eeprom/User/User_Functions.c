/*
 * User_Functions.c
 *
 *  Created on: 04 May 2017
 *      Author: Bartho Horn
 */

#include "User_Defines.h"

void Initialise_BMS(void)
{
    flagCurrent = 0;
    System_State = 0;

    Temperatures[5] = 0;
    Temperatures_resistance[4] = 0;

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

    ConfigCpuTimer(&CpuTimer0, 60, 500000); //2 hz - 500
    ConfigCpuTimer(&CpuTimer1, 60, 20000);  //50 hz
    ConfigCpuTimer(&CpuTimer2, 60, 500);    //2 khz

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
    //Shut_D_BQ();

    //Calibrate_Current();

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
    led2 = 0;

    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;     //led1
    GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;      //led1

    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;     //BT reset
    GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;      //BT reset
    Aux_Control2 = 0;                            //keep BT in reset

    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;    //12 Aux drive
    GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;     // 12 Aux drive (verander miskien)
    Aux_Control = 0;

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
    //	ContactorOut = 0;
    //turn off PreCharge
    //PreCharge = 0;
}

void Toggle_LED(void)
{
    GpioDataRegs.GPATOGGLE.bit.GPIO5 = 1;

    System_State = 1; 							//State is good, not state in while loop

    //GpioDataRegs.GPATOGGLE.bit.GPIO6 = 1;
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
    static float Voltage_low_filter_temp = 0;
    static float Another_temp = 0;

    float temp_V = 0;
    float temp_Voltage_total = 0;
    float temp_Voltage_high;
    temp_Voltage_high = 0;
    float temp_Voltage_high_nr = 0;

    float temp_Voltage_low;
    temp_Voltage_low = 10;
    float temp_Voltage_low_nr = 0;

    float temp_Voltage_avg = 0;

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

        temp_Voltage_total = temp_Voltage_total +  Voltages[i];

        if(temp_Voltage_high<Voltages[i])
        {
            temp_Voltage_high = Voltages[i];
            temp_Voltage_high_nr = i + 1;
        }

        if(temp_Voltage_low>Voltages[i])
        {
            temp_Voltage_low = Voltages[i];
            temp_Voltage_low_nr = i + 1;
        }

        temp_Voltage_avg =  temp_Voltage_avg + Voltages[i];
    }
    Voltage_total = temp_Voltage_total;
    Voltage_avg = temp_Voltage_avg*0.0667;
    Voltage_high = temp_Voltage_high;
    Voltage_low_cell = temp_Voltage_low_nr;

    Voltage_high_cell = temp_Voltage_high_nr;
    //Voltage_low = temp_Voltage_low;

    // add voltage low filter. 3 second cut off period
    Another_temp = Voltage_low_filter_temp + (0.878*((temp_Voltage_low)-Voltage_low_filter_temp));
    Voltage_low = Another_temp;
    Voltage_low_filter_temp = Voltage_low;

    //ServiceDog();
}

void Process_Voltages(void)
{
    //	static int delay = 0;
    if(Voltage_high > Vmax)     //3.65
    {
        balance = 1;            //start balancing
        flagCharged = 1;        //charged flag to to stop charging
        ContactorOut = 0;																						//kan hierdie wees wat die contactor oopmaak
    }

    if(Voltage_low > Vmin && Auxilliary_Voltage < Vauxmin && Auxilliary_Voltage > 7 && Aux_Control == 0)
    {
        Auxilliary_counter = 0;															//turn on aux supply
        Aux_Control = 1;
    }
    else if(Auxilliary_counter > AuxChargeTime || Auxilliary_Voltage < 7)
    {
        Aux_Control = 0;																//turn off aux supply
    }

    Auxilliary_counter++;

/////////////////////////////////////////////////////////////////////////////////////////
    if(Current < 80)                                                                //check current to reduce false trips
    {

        if(Voltage_low < Vmin && Voltage_low > Vcritical && Charger_status == 0)
        {
            //Aux_Control = 0;
            flagDischarged = 1;
            //		led3 = 1;               //turn on red led
            //ContactorOut = 0;       //turn off contactor
            PreCharge = 1;
        }
        else if(Voltage_low < Vcritical && Charger_status == 0)						//Maybe make this a bit smaller --> 2.7 or even 2.6??
        {
            Aux_Control = 0;
            flagDischarged = 2;
            //		led3 = 1;               //turn on red led
            PreCharge = 0;
            ContactorOut = 0;       //turn off contactor
            Aux_Control2 = 0;
        }
    }
    else
    {
        if(Voltage_low < 2.5 && Voltage_low > Vcritical && Charger_status == 0)
        {
            //Aux_Control = 0;
            flagDischarged = 1;
            //      led3 = 1;               //turn on red led
            ContactorOut = 0;       //turn off contactor
            PreCharge = 1;
        }
    }
/////////////////////////////////////////////////////////////////////////////////////

    if(Voltage_high<Vchargedflagreset )
        flagCharged = 0;

    if(Voltage_low>Vdischargedflagreset )
    {
        flagDischarged = 0;
        //		led3 = 0;               //turn off red led
        PreCharge = 1;
        Aux_Control2 = 1;
    }
}

void Calculate_SOH(void)
{
    int i;
    int temp_SOC_max_cell = 0;
    float r_avg = 0;

    dI = fabs(Current - Current_old);

    if(dI>10)
    {
        resistance = 0;

        for(i = 0; i<15; i++)
        {
            resistance_temp = fabs(Voltages[i]-Voltages_old[i]) / dI;
            r_avg = resistance_temp + r_avg;

            if(resistance_temp>resistance)
            {
                resistance = resistance_temp;
                temp_SOC_max_cell = i +1;
            }
        }
        r_avg = r_avg*0.066667;
        SOH_avg = r_avg;
        SOH_max_cell = temp_SOC_max_cell ;
        SOH_max = resistance;
    }


    //1 + ((0.001 - resistance)/0.001);

    for(i = 0; i<15; i++)
    {
        Voltages_old[i] = Voltages[i];
    }

    Current_old = Current;
}

void Calculate_Current(void)
{
    Current = ((test_current)-Current_CAL )* 0.122;                   //2095    maal, moenie deel nie!!!!     0.0982--200/2048          /*Current_CAL/*

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
    static float Temperatures_resistance_temp[14] = { 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000 };

    //temp meetings
    //SOC15 - Outside
    Temperatures_resistance[13] = Temperatures_resistance_temp[13] + (0.02*(((AdcResult.ADCRESULT15))-Temperatures_resistance_temp[13]));
    Temperatures_resistance_temp[13] = Temperatures_resistance[13];

    //Cells:
    //SOC0 - Cell0
    Temperatures_resistance[0] = Temperatures_resistance_temp[0] + (0.02*(((AdcResult.ADCRESULT0))-Temperatures_resistance_temp[0]));
    Temperatures_resistance_temp[0] = Temperatures_resistance[0];
    //SOC3 - Cell1
    Temperatures_resistance[1] = Temperatures_resistance_temp[1] + (0.02*(((AdcResult.ADCRESULT3))-Temperatures_resistance_temp[1]));
    Temperatures_resistance_temp[1] = Temperatures_resistance[1];
    //SOC4 - Cell2
    Temperatures_resistance[2] = Temperatures_resistance_temp[2] + (0.02*(((AdcResult.ADCRESULT4))-Temperatures_resistance_temp[2]));
    Temperatures_resistance_temp[2] = Temperatures_resistance[2];
    //SOC5 - Cell3
    Temperatures_resistance[3] = Temperatures_resistance_temp[3] + (0.02*(((AdcResult.ADCRESULT5))-Temperatures_resistance_temp[3]));
    Temperatures_resistance_temp[3] = Temperatures_resistance[3];
    //SOC6 - Cell5
    Temperatures_resistance[4] = Temperatures_resistance_temp[4] + (0.02*(((AdcResult.ADCRESULT6))-Temperatures_resistance_temp[4]));		//possibly use for model detection
    Temperatures_resistance_temp[4] = Temperatures_resistance[4];
    //SOC7 - Cell6
    Temperatures_resistance[5] = Temperatures_resistance_temp[5] + (0.02*(((AdcResult.ADCRESULT7))-Temperatures_resistance_temp[5]));
    Temperatures_resistance_temp[5] = Temperatures_resistance[5];
    //SOC8 - Cell7
    Temperatures_resistance[6] = Temperatures_resistance_temp[6] + (0.02*(((AdcResult.ADCRESULT8))-Temperatures_resistance_temp[6]));
    Temperatures_resistance_temp[6] = Temperatures_resistance[6];
    //SOC9 - Cell8
    Temperatures_resistance[7] = Temperatures_resistance_temp[7] + (0.02*(((AdcResult.ADCRESULT9))-Temperatures_resistance_temp[7]));
    Temperatures_resistance_temp[7] = Temperatures_resistance[7];
    //SOC10 - Cell10
    Temperatures_resistance[8] = Temperatures_resistance_temp[8] + (0.02*(((AdcResult.ADCRESULT10))-Temperatures_resistance_temp[8]));
    Temperatures_resistance_temp[8] = Temperatures_resistance[8];
    //SOC11 - Cell11
    Temperatures_resistance[9] = Temperatures_resistance_temp[9] + (0.02*(((AdcResult.ADCRESULT11))-Temperatures_resistance_temp[9]));
    Temperatures_resistance_temp[9] = Temperatures_resistance[9];
    //SOC12 - Cell12
    Temperatures_resistance[10] = Temperatures_resistance_temp[10] + (0.02*(((AdcResult.ADCRESULT12))-Temperatures_resistance_temp[10]));
    Temperatures_resistance_temp[10] = Temperatures_resistance[10];
    //SOC13 - Cell13
    Temperatures_resistance[11] = Temperatures_resistance_temp[11] + (0.02*(((AdcResult.ADCRESULT13))-Temperatures_resistance_temp[11]));
    Temperatures_resistance_temp[11] = Temperatures_resistance[11];
    //SOC14 - Cell14
    Temperatures_resistance[12] = Temperatures_resistance_temp[12] + (0.02*(((AdcResult.ADCRESULT14))-Temperatures_resistance_temp[12]));
    Temperatures_resistance_temp[12] = Temperatures_resistance[12];

    int i;
    int flag = 0;
    float Vts;
    float Rts;
    float temperature_avg=0;
    float temp_Temperature_high = 0;
    float temp_Temperature_low = 71;
    float temp_high_cell = 0;
    float temp_low_cell = 0;

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
        temperature_avg = temperature_avg+Temperatures[i];					//calculate avg temperature

        if(temp_Temperature_high<Temperatures[i])							//calculate highest temperature
        {
            temp_Temperature_high = Temperatures[i];
            temp_high_cell = i ;

        }

        if(temp_Temperature_low>Temperatures[i])							//calculate lowest temperature
        {
            temp_Temperature_low = Temperatures[i];
            temp_low_cell = i;
        }

    }
    temperature_avg = temperature_avg*0.0667;

    if(Temperatures_resistance[8]<50)										//old bms version       (tipies 20)
    {
        Temperature_avg = (Temperatures[4]+Temperatures[9])/2;

        if(Temperatures[4]>Temperatures[9])
        {
            Temperature_high = Temperatures[4];
            Temperature_low = Temperatures[9];
            Temperature_high_cell = 5;
            Temperature_low_cell = 10;
        }
        else
        {
            Temperature_high = Temperatures[9];
            Temperature_low = Temperatures[4];
            Temperature_high_cell = 10;
            Temperature_low_cell = 5;
        }

        if((Temperatures[4]> Tmax || Temperatures[4]<Tmin) || (Temperatures[9]> Tmax || Temperatures[9]<Tmin))
            flag = 1;
    }
    else																	//new bms version
    {
        //led3 = 1;

        Temperature_avg = temperature_avg;
        Temperature_high = temp_Temperature_high;
        Temperature_high_cell = temp_high_cell;
        Temperature_low =temp_Temperature_low;
        Temperature_low_cell = temp_low_cell;

        if(Temperature_high> Tmax || Temperature_low<Tmin)
        {
            //flag = 1;
        }


        /*	if((Temperature_avg - Temperatures[15])> 4 && Temperatures[15]<50 && Temperature_avg>10&& Voltage_low > Vmin && balance == 0 && temptimer ==0)				//4.5 en 25sit net aan bo 25 grade celsius
		{
			Fan_Control = 1;
			temptimer = 1;
		}

		if(temptimer > 0)
			temptimer++;
		if (temptimer > 180)
		{
			temptimer = 0;
			Fan_Control = 0;
		}*/




        /*	if((Temperature_avg - Temperatures[15])> 4.5 && Temperatures[15]<50 && Temperature_avg>10&& Voltage_low > Vmin && balance == 0)				//4.5 en 25sit net aan bo 25 grade celsius
		{
			Fan_Control = 1;
			temptimer
		}
		else if(GpioDataRegs.GPADAT.bit.GPIO19 == 1 && (Temperature_avg - Temperatures[15])> 3 && Temperature_avg>8 && Voltage_low > Vmin && balance == 0)//bly aan solank 3.5 en 23
		{
			Fan_Control = 1;
		}
		else
			Fan_Control = 0;*/			//0
    }

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
long interpolate_segment(long x0, long y0, long x1, long y1, long x)
{
    long t;

    if (x <= y0) { return x0; }
    if (x >= y1) { return x1; }

    t =  (x-y0);
    //	t /= (y1-y0);

    t *= (x1-x0);

    return x0 + t/(y1-y0);

    //	return x0 + t*(x1-x0);
}
/******************************************************************************/

long interpolate_table_1d(struct table_1d *table, long x)
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
    //	float SOCv;
    //	static float SOCc;
    //	static Uint16 t=1000;							//time since current activity
    //	float Wsoc;										//weighting parameter

    SOCv = (float)(interpolate_table_1d(&sine_table, (long)(Voltage_low*1000)));

    SOC_t++;

    if(Current>3 || Current<-3 || Charger_status == 1)
    {
        SOC_t = 0;
    }
    SOCc = SOC - (Current*0.000164);				//coulomb counter      Ampere sec -> Ampere huur  	0.000185			1/150A*3600s
                                                       //                                                0.000164            1/170*2*3600s
    if(SOC_t > 5400)								//delay of 90 min maybe do 60 min?
        Wsoc = 0;
    else
        Wsoc = 1;

    SOC = Wsoc*SOCc + (1-Wsoc)*SOCv;

    if(SOC>100)
        SOC = 100;
    else if(SOC<0.01)
        SOC = 0.01;
}

void Calibrate_Current_charger()
{
    //Reset the watchdog counter
    ServiceDog();
    float error;

    static float old_ChargerCurrent;
    float ChargerCurrent_di;

    static float old_Current;
    float Current_di;

    ChargerCurrent_di = ChargerCurrent - old_ChargerCurrent;
    Current_di = Current - old_Current;

    if(ChargerCurrent > 24 && Aux_Control == 0 && ChargerCurrent_di<1 && Current_di<1)					//15 charger busy charging and aux charger turned off
    {
        error = (Current + ChargerCurrent-0.1)/Current;												//as percentage
        Current_CAL = Current_CAL -0.02* error * Current_CAL;											//maybe add slow filter to dampen the fault?

        if(Current_CAL>2200)																			//set maximum limit
            Current_CAL = 2200;
        if(Current_CAL<2000)																			//set minimum limit
            Current_CAL = 2000;
    }

    old_ChargerCurrent = ChargerCurrent;
    old_Current = Current;
}

void Battery_Status(void)
{
    BMS_Status = 0;

    if(balance == 1)
        BMS_Status = BMS_Status + 1;
    if(flagCharged == 1)
        BMS_Status = BMS_Status + 2;
    if(flagCurrent == 1)
        BMS_Status = BMS_Status + 4;
    if(flagVoltage == 1)
        BMS_Status = BMS_Status + 8;
    if(flagTemp == 1)
        BMS_Status = BMS_Status + 16;
    if(flagDischarged == 1)
        BMS_Status = BMS_Status + 32;
    if(Charger_status == 1)
        BMS_Status = BMS_Status + 64;
    if(KeySwitch == 1)
        BMS_Status = BMS_Status + 128;
    //Extra flags are a possibility
}

float Charging_Animation(float real_SOC)
{
    static float previous_SOC = 0;

    previous_SOC = previous_SOC + 12;

    if(previous_SOC <= real_SOC - 12)			//bottom animation
        previous_SOC = real_SOC - 12;
    else if(previous_SOC > 101 )				//top animation
        previous_SOC = real_SOC - 12;

    return previous_SOC;
}
