/*
 * User_Functions.c
 *
 *  Created on: 04 May 2017
 *      Author: Bartho Horn
 */

#include "User_Defines.h"
#include <stdio.h>
#include <string.h>

void Initialise_BMS(void)
{
    Initial_Capacity = 150;
    flagCurrent = 0;
    //    System_State = 0;

    //memcpy(&RamfuncsRunStart, &RamfuncsLoadStart,(Uint32)(&RamfuncsLoadEnd-RamfuncsLoadStart));

    InitSysCtrl();
    InitI2CGpio();
    InitAdc();
    Init_Gpio();

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


    NMT_State = 0x7F;                                       //Enter NMT pre-operational state from initialisation state

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

    GpioCtrlRegs.GPAPUD.bit.GPIO3  = 1;     //Disable pull-up for GPIO3
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;     //BQenable
    GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;      //Bq enable

    GpioCtrlRegs.GPAPUD.bit.GPIO4  = 1;     //Disable pull-up for GPIO4
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;     //led2
    GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;      //led2
    led2 = 0;

    GpioCtrlRegs.GPAPUD.bit.GPIO5  = 1;     //Disable pull-up for GPIO5
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;     //led1
    GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;      //led1

    GpioCtrlRegs.GPAPUD.bit.GPIO6  = 1;     //Disable pull-up for GPIO6
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;     //12V Secondary Auxiliary Supply Ctrl
    GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;      //12V Secondary Auxiliary Supply Ctrl
    LPwr_Out_Ctrl_1 = 0;                    //12V Secondary Auxiliary Supply Ctrl - off

    GpioCtrlRegs.GPAPUD.bit.GPIO7  = 1;     //Disable pull-up for GPIO7
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;     //Extra Mfet output
    GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;      //Extra Mfet output

    GpioCtrlRegs.GPAPUD.bit.GPIO9  = 1;     //Disable pull-up for GPIO9
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;     //BQ Alert pin
    GpioCtrlRegs.GPADIR.bit.GPIO9 = 0;      //MCU input

    GpioCtrlRegs.GPAPUD.bit.GPIO12  = 1;    //Disable pull-up for GPIO12
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;    //Temperature_Control
    GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;     //Temperature_Control
    Temperature_Control = 0;                //Switch on Temperature measurements

    GpioCtrlRegs.GPAPUD.bit.GPIO13  = 1;    //Disable pull-up for GPIO13
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0;    //High power 48V ctrl 1
    GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;     //High power 48V ctrl 1
    Ctrl_HPwr_48V_O_1 = 1;                  //High power 48V ctrl 1 - dink hierdie moet 1 wees

    GpioCtrlRegs.GPAPUD.bit.GPIO15  = 1;    //Disable pull-up for GPIO15
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;    //12 Aux drive
    GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;     // 12 Aux drive (verander miskien)


    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 1;     //Disable pull-up for GPIO19
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;    //Ctrl_LPwr_48V_O_2
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;     //Ctrl_LPwr_48V_O_2
    Ctrl_LPwr_48V_O_2 = 0; 					//turn off for now

    GpioCtrlRegs.GPAPUD.bit.GPIO20  = 1;    //Disable pull-up for GPIO20
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;    //contactor output
    GpioCtrlRegs.GPADIR.bit.GPIO20 = 1;     // contactor output

    GpioCtrlRegs.GPAPUD.bit.GPIO21  = 1;    //Disable pull-up for GPIO21
    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0;    //precharge resistor
    GpioCtrlRegs.GPADIR.bit.GPIO21 = 1;     //precharge resistor

    GpioCtrlRegs.GPAPUD.bit.GPIO22  = 1;    //Disable pull-up for GPIO22
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;    //key 1 switch
    GpioCtrlRegs.GPADIR.bit.GPIO22 = 0;     //key 1 switch

    GpioCtrlRegs.GPAPUD.bit.GPIO24  = 1;    //Disable pull-up for GPIO24
    GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0;    //key 2 switch
    GpioCtrlRegs.GPADIR.bit.GPIO24 = 0;     //key 2 switch

    GpioCtrlRegs.GPAPUD.bit.GPIO25  = 1;    //Disable pull-up for GPIO25
    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;    //Type 2 charger control
    GpioCtrlRegs.GPADIR.bit.GPIO25 = 1;     //Type 2 charger control
    CHG_J1772_Ctrl = 0;                     //Turn off vehicle charging

    GpioCtrlRegs.GPAPUD.bit.GPIO26  = 1;    //Disable pull-up for GPIO26
    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;    //BQ on
    GpioCtrlRegs.GPADIR.bit.GPIO26 = 0;     //BQ on (input)

    GpioCtrlRegs.GPAPUD.bit.GPIO27  = 1;    //Disable pull-up for GPIO27
    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;    //CANenable
    GpioCtrlRegs.GPADIR.bit.GPIO27 = 1;     //CANenable (output)

    GpioCtrlRegs.GPBPUD.bit.GPIO39  = 1;    //Disable pull-up for GPIO39
    GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 0;    //Ctrl_LPwr_48V_O_3
    GpioCtrlRegs.GPBDIR.bit.GPIO39 = 1;     //Ctrl_LPwr_48V_O_3
    Ctrl_LPwr_48V_O_3 = 0;                  //turn off for now

    GpioCtrlRegs.GPBPUD.bit.GPIO40  = 1;    //Disable pull-up for GPIO40
    GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 0;    //led3
    GpioCtrlRegs.GPBDIR.bit.GPIO40 = 1;     //led3

    GpioCtrlRegs.GPBPUD.bit.GPIO44  = 1;    //Disable pull-up for GPIO44
    GpioCtrlRegs.GPBMUX1.bit.GPIO44 = 0;    //CScontrol
    GpioCtrlRegs.GPBDIR.bit.GPIO44 = 1;     //CSControl

    EDIS;

    Contactor_Off();                        //turn off contactor
    Pre_Charge_On();                        //turn on precharge resistor

    CSControl = 1;                          //turn CScontrol on for current measurement
    Aux_Supply_12V_Off();

}

void Toggle_LED(void)
{
    GpioDataRegs.GPATOGGLE.bit.GPIO5 = 1;                   //led1
    GpioDataRegs.GPATOGGLE.bit.GPIO4 = 1;                   //led2

    //Ctrl_LPwr_48V_O_3 = 1;          //GPIO39
    //GpioDataRegs.GPBTOGGLE.bit.GPIO39 = 1;                working

    //Ctrl_LPwr_48V_O_2          //GPIO19
    //GpioDataRegs.GPATOGGLE.bit.GPIO19 = 1;                //working
    //Ctrl_LPwr_48V_O_2 = 1;

    //Ctrl_LPwr_48V_O_1 -  Dink dis die geval
    //GpioDataRegs.GPATOGGLE.bit.GPIO20 = 1;                working

    //Ctrl_HPwr_48V_O_2 - Aux_Control
    //GpioDataRegs.GPATOGGLE.bit.GPIO15 = 1;                working

    //Ctrl_HPwr_48V_O_1
    //GpioDataRegs.GPATOGGLE.bit.GPIO13 = 1;                working

    //    System_State = 1; 							//State is good, not state in while loop

    //GpioDataRegs.GPATOGGLE.bit.GPIO6 = 1;
    //GpioDataRegs.GPATOGGLE.bit.GPIO19 = 1;
}

void Status_LED(void)
{
      if(flagCurrent == 1)
          led3 = 1;
      else
          led3 = 0;

      if(flagDischarged != 0)
          led2 = 1;
      else
          led2 = 0;

      if(flagTemp_Discharge == 1)
      {
          led3 = 1;
          led2 = 1;
      }
}

void  Read_Cell_Voltages(void)
{
    // Read data from EEPROM section //
    int i;

    //Voltage_low = 10;
    float Voltages_backup5 = Voltages[5];
    float Voltages_backup10 = Voltages[10];

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
    Voltage_low = temp_Voltage_low;
}

void Process_Voltages(void)
{
    //High Voltage Error
    if(Voltage_high > Vmax)
    {
        balance = 1;                                            //start balancing
        flagCharged = 1;    //verander na overVoltage?                                    //charged flag to to stop charging
        Contactor_Off();										//kan hierdie wees wat die contactor oopmaak
    }

    //Low Voltage Error
    if(Voltage_low < Vmin && Voltage_low > Vcritical && Charger_status == 0)
    {
        flagDischarged = 1;

    }
    else if(Voltage_low < Vcritical && Charger_status == 0)
    {
        Aux_Supply_12V_Off();
        flagDischarged = 2;
        Contactor_Off();                                                                //turn off contactor
//        LPwr_Out_Ctrl_1 = 0;                                                            //Control Fusebox secondary regulator

        //Ctrl_HPwr_48V_O_1 = 0                                                         //switch off 48V supply when in critical mode
    }

    //Auxiliary control
    if(Key_switch_1 == 1 && Voltage_low > Vmin && Aux_Control == 0)
    {
        Aux_Supply_12V_On();
    }
    else
    {
        Aux_Supply_12V_Off();															//turn off aux supply
    }

    if(Voltage_high<Vchargedflagreset )
        flagCharged = 0;

    if(Voltage_low>Vdischargedflagreset )
    {
        flagDischarged = 0;
        //LPwr_Out_Ctrl_1 = 1;
    }

    if(Voltage_high<3.4)
    {
        SOP_charge = 50;
    }
    else
        SOP_charge = 0;                             //maybe add in PI controller for reference
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

void Process_BQ_System_Status(void)
{
    if(BQ_Alert == 1)                             //Error on flags
    {
        system_status = 0x2C & (I2CA_ReadData(&I2cMsgIn1,0x00, 1));     //only Xready, UV and OV

        if((system_status&0x20) == 0x20)           //Device_Xready flag
        {
            I2CA_WriteData(0x00, 0xBF);            //reset all possible flags
            //add to BQ_Error_counter
            I2CA_WriteData(0x05,0x03);             //Turn on outputs
        }
        else if((system_status&0x8) == 0x8)        //under voltage error
        {
            if(flagDischarged < 2)                 //Voltage is within bounds - maybe add something for the charger as well?
            {
                I2CA_WriteData(0x00, 0xBF);        //reset all possible flags
                I2CA_WriteData(0x05,0x03);         //Turn on outputs
            }
        }
        else if((system_status&0x4) == 0x4)        //Over Voltage error
        {
            if(flagCharged == 0)                   //Voltage is within bounds again
            {
                I2CA_WriteData(0x00, 0xBF);        //reset all possible flags
                I2CA_WriteData(0x05,0x03);         //Turn on outputs
            }
        }
    }
}

void Read_Temperatures(void)
{
    static float Temperatures_resistance_temp[5] = {1000, 1000, 1000, 1000, 1000};

    //SOC0 -- New modules: Module 3 Temp 2
    Temperatures_resistance[0] = Temperatures_resistance_temp[0] + (0.2*(((AdcResult.ADCRESULT0))-Temperatures_resistance_temp[0]));
    Temperatures_resistance_temp[0] = Temperatures_resistance[0];

    //SOC3 -- New modules: Module 2 Temp 2
    Temperatures_resistance[1] = Temperatures_resistance_temp[1] + (0.2*(((AdcResult.ADCRESULT3))-Temperatures_resistance_temp[1]));
    Temperatures_resistance_temp[1] = Temperatures_resistance[1];

    //SOC4 -- New modules: Module 1 Temp 2
    Temperatures_resistance[2] = Temperatures_resistance_temp[2] + (0.2*(((AdcResult.ADCRESULT4))-Temperatures_resistance_temp[2]));
    Temperatures_resistance_temp[2] = Temperatures_resistance[2];

    //SOC5 -- New modules: Current Sense Board
    Temperatures_resistance[3] = Temperatures_resistance_temp[3] + (0.2*(((AdcResult.ADCRESULT5))-Temperatures_resistance_temp[3]));
    Temperatures_resistance_temp[3] = Temperatures_resistance[3];

    //SOC6 -- New modules: Main BMS Board
    Temperatures_resistance[4] = Temperatures_resistance_temp[4] + (0.2*(((AdcResult.ADCRESULT6))-Temperatures_resistance_temp[4]));		//possibly use for model detection
    Temperatures_resistance_temp[4] = Temperatures_resistance[4];


    int i;
    float Vts;
    float Rts;
    float temperature_avg=0;
    float temp_Temperature_high = 0;
    float temp_Temperature_low = 71;
    int temp_high_cell = 0;
    int temp_low_cell = 0;

    float temp_T = 0;

    //Module3 Temperature sense2
    Vts = (Temperatures_resistance[0]) * 0.00080566;
    Rts = (10000*Vts)/(3.3-Vts);
    //Temperatures_Module[2][1] = (1/((log(Rts/10000))/4000+0.003356))-273;

    //Module2 Temperature sense2
    Vts = (Temperatures_resistance[1]) * 0.00080566;
    Rts = (10000*Vts)/(3.3-Vts);
    //Temperatures_Module[1][1] = (1/((log(Rts/10000))/4000+0.003356))-273;

    //Module1 Temperature sense2
    Vts = (Temperatures_resistance[2]) * 0.00080566;
    Rts = (10000*Vts)/(3.3-Vts);
    //Temperatures_Module[0][1] = (1/((log(Rts/10000))/4000+0.003356))-273;

    //Current Sensor Temperature
    Vts = (Temperatures_resistance[3]) * 0.00080566;
    Rts = (10000*Vts)/(3.3-Vts);
    Temperatures_CS = (1/((log(Rts/10000))/4000+0.003356))-273;

    //BMS Temperature
    Vts = (Temperatures_resistance[4]) * 0.00080566;
    Rts = (10000*Vts)/(3.3-Vts);
    Temperatures_BMS = (1/((log(Rts/10000))/3980+0.003356))-273;                        //maybe 3980 instead of 4000

    //BQ_temp1,BQ_temp2 and BQ_temp3
    temp_T = I2CA_ReadData(&I2cMsgIn1, 0x2C, 2);    //TS1
    Vts = temp_T*ADCgain;
    Rts = (10000*Vts)/(3.3-Vts);
    Temperatures_Module[0] = (1/((log(Rts/10000))/3435+0.003356))-273;

    temp_T = I2CA_ReadData(&I2cMsgIn1, 0x2E, 2);    //TS2
    Vts = temp_T*ADCgain;
    Rts = (10000*Vts)/(3.3-Vts);
    Temperatures_Module[1] = (1/((log(Rts/10000))/3435+0.003356))-273;

    temp_T = I2CA_ReadData(&I2cMsgIn1, 0x30, 2);    //TS3
    Vts = temp_T*ADCgain;
    Rts = (10000*Vts)/(3.3-Vts);
    Temperatures_Module[2] = (1/((log(Rts/10000))/3435+0.003356))-273;               //3435 (new thermistors)   //Thermistor: 1/T = 1/To + 1/B*ln(R/Ro)

    //Calculate avg cell temperature
    for(i=0; i<3; i++)
    {
        temperature_avg = temperature_avg + Temperatures_Module[i];

        if(temp_Temperature_high < Temperatures_Module[i])							//calculate highest temperature
        {
            temp_Temperature_high = (Temperatures_Module[i]);
            temp_high_cell = i;
        }

        if(temp_Temperature_low > Temperatures_Module[i])							//calculate lowest temperature
        {
            temp_Temperature_low = ((Temperatures_Module[i]));
            temp_low_cell = i;
        }
    }
    temperature_avg = temperature_avg/3;

    Temperature_avg = temperature_avg;
    Temperature_high = temp_Temperature_high;
    Temperature_high_cell = temp_high_cell;
    Temperature_low =temp_Temperature_low;
    Temperature_low_cell = temp_low_cell;
}

void Process_Temperatures(void)
{
    if(Temperature_high > 55 || Temperature_low < -10)
        flagTemp_Discharge = 1;
    else
        flagTemp_Discharge = 0;

    if(Temperature_high > 50 || Temperature_low < 0)
        flagTemp_Charge = 1;
    else
        flagTemp_Charge = 0;
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

    if(/*Current>3 || Current<-3*/ Key_switch_1 == 1 || Key_switch_2 == 1 || Charger_status == 1) //probably needs to go to whether the keyswitch is active or not
    {
        SOC_t = 0;
    }
    SOCc = SOC - (Current*(0.000185));		                //coulomb counter      Ampere sec -> Ampere huur  	0.000185	1/150A*3600s
                                                            //                                                  0.000164    1/170*2*3600s
    if(SOC_t > 5400)								        //delay of 90 min maybe do 60 min?
        Wsoc = 0;
    else
        Wsoc = 1;

    SOC = Wsoc*SOCc + (1-Wsoc)*SOCv;

    if(SOC>100)
        SOC = 100;
    else if(SOC<0.01)
        SOC = 0.01;
}

void Calibrate_Current()
{
    //Calibrate when key-switch(position 2,3) is deactivated, result in 48V and 12V supply delivers 0 Watts.
    float error;
    static Uint16 Calibrate_delay = 0;

    if(Key_switch_1==0 && Key_switch_2==0 && Charger_status == 0)		 /*Aux_Control == 0 && ContactorOut == 0*/  //Vehicle is off (Key-switch position 0)
    {
        if(Calibrate_delay > 10)                                                    //10 s delay for the vehicle/battery to do shut-off process
        {
            error = Current;
            Current_CAL = Current_CAL + (0.0001* error * Current_CAL);			    //maybe add slow filter to dampen the fault?

            if(Current_CAL>2200)													//set maximum limit
                Current_CAL = 2200;
            if(Current_CAL<2000)													//set minimum limit
                Current_CAL = 2000;
        }
        else
            Calibrate_delay++;
    }
    else
        Calibrate_delay=0;
}

void Battery_Status(void)
{
    //check this

    Uint16 BMS_Status_Temp = 0;

    if(GpioDataRegs.GPADAT.bit.GPIO20 == 1)                 //48V supply (High Current)
        BMS_Status_Temp = BMS_Status_Temp + 1;
    if(GpioDataRegs.GPADAT.bit.GPIO15 == 1)                 //12V supply (High Current)
        BMS_Status_Temp = BMS_Status_Temp + 2;
    if(GpioDataRegs.GPADAT.bit.GPIO6 == 1)                  //12V supply (low Current)    - check if inverse
        BMS_Status_Temp = BMS_Status_Temp + 4;
    if(GpioDataRegs.GPADAT.bit.GPIO21 == 1)                 //Pre-charge
        BMS_Status_Temp = BMS_Status_Temp + 8;
    if(balance == 1)                                        //cell balancing
        BMS_Status_Temp = BMS_Status_Temp + 16;
    if(Charging == 1)                                       //add charging flag
        BMS_Status_Temp = BMS_Status_Temp + 32;
    if(Charger_status == 1)                                 //add charger connected flag
        BMS_Status_Temp = BMS_Status_Temp + 64;
    if(GpioDataRegs.GPADAT.bit.GPIO22 == 1)                 //add keyswitch 1 flag        - still outstanding (Hardware changed)
        BMS_Status_Temp = BMS_Status_Temp + 128;
    if(GpioDataRegs.GPADAT.bit.GPIO24 == 1)                 //add keyswitch 2 flag
        BMS_Status_Temp = BMS_Status_Temp + 256;
    if(Cooling == 1)                                        //add cooling flag
        BMS_Status_Temp = BMS_Status_Temp + 512;
    if(Heating == 1)                                        //add heating flag
        BMS_Status_Temp = BMS_Status_Temp + 1024;

    BMS_Status = BMS_Status_Temp;                               //update setup
}

void Battery_Error(void)
{
    Uint16 BMS_Error_Temp = 0;

    if(flagVoltage == 1)                                    //voltage flag
        BMS_Error_Temp = BMS_Error_Temp + 1;
    if(flagDischarged == 2)                                 //voltage critical flag
        BMS_Error_Temp = BMS_Error_Temp + 2;
    if(flagCurrent == 1)                                    //over current flag
        BMS_Error_Temp = BMS_Error_Temp + 4;
    if(flag_Pre_Charge == 1)                                //pre-charge error
        BMS_Error_Temp = BMS_Error_Temp + 8;
    if(flagTemp_Discharge == 1)
        BMS_Error_Temp = BMS_Error_Temp + 16;
    if(flagTemp_Charge == 1)
        BMS_Error_Temp = BMS_Error_Temp + 32;


    //tempBMS
    //end of life

    if(flagDischarged == 1)
        BMS_Error_Temp = BMS_Error_Temp + 64;
    if(Charger_status == 1)
        BMS_Error_Temp = BMS_Error_Temp + 128;

    //Extra flags are a possibility

    BMS_Error = BMS_Error_Temp;                               //update setup
}

float Charging_Animation(float real_SOC)
{
    static int previous_SOC = 0;

    previous_SOC = previous_SOC + 13;

    if(previous_SOC < real_SOC - 13)            //bottom animation
        previous_SOC = real_SOC - 13;
    else if(previous_SOC > 102 )                //top animation
        previous_SOC = real_SOC - 13;

    if(previous_SOC<0)
        previous_SOC = previous_SOC + 12;

    return previous_SOC;
}

void OverCurrentFault(void)
{
    Contactor_Off();                            //Open contactor
    flagCurrent = 1;                            //Set Over Current flag
}

void Pre_Charge_On(void)
{
    GpioDataRegs.GPASET.bit.GPIO21 = 1;         //Turn on pre_charge
}

void Pre_Charge_Off(void)
{
    GpioDataRegs.GPACLEAR.bit.GPIO21 = 1;         //Turn off pre_charge
}

void Contactor_On(void)
{
    GpioDataRegs.GPASET.bit.GPIO20 = 1;         //Turn on Contactor
}

void Contactor_Off(void)
{
    GpioDataRegs.GPACLEAR.bit.GPIO20 = 1;         //Turn off Contactor
}

void Aux_Supply_12V_On(void)
{
    GpioDataRegs.GPASET.bit.GPIO15 = 1;         //Turn on 12V_Aux_Supply
}

void Aux_Supply_12V_Off(void)
{
    GpioDataRegs.GPACLEAR.bit.GPIO15 = 1;         //Turn on 12V_Aux_Supply
}
