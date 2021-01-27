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

    //static long int Filter_SC;
    static long Filter_SC_past = 0;
    static float current_p;

    static long trip_timer;

    test_current = current_p + (0.00314*(AdcResult.ADCRESULT1-current_p));          //   0.00314-1Hz     //  0.01249 - 4 Hz      //0.27-100Hz
    current_p=test_current;

    ////////////
    //insert limits here
    //4095 = maksimum current -> 250??
    //3686 -> 4.5V = 200 A          220   ~  4.7V = 3850
    //3359 -> 4.1V = 160 A          200   ~  4.5V = 3686
    //3031 -> 3.7V = 120 A          160   ~  4.1V = 3359
    //2048 -> 2.5V= 0 A             0 A   ~  2.5V = 2048
    //1065 -> 1.3V = -120           -160  ~  0.9V = 737
    //410 -> 0.5V = -200A           -200  ~  0.5V = 410
    //0 -> 0V = -250A               -220  ~  0.3V = 246

    Filter_SC = (100*Filter_SC_past + (27*(AdcResult.ADCRESULT1-Filter_SC_past)))/100;     //   0.00314-1Hz     //  0.01249 - 4 Hz      //0.27-100Hz
    Filter_SC_past=Filter_SC;

    //Short circuit fault - 100 Hz cut-off
    if(Filter_SC > Imax || Filter_SC < Imin)
    {
        OverCurrentFault();
    }
    else if(Filter_SC <= 3359 && Filter_SC > 2048)                                      //current between 0A and 160 A
    {
        trip_timer = interpolate_table_1d(&trip2_table, Filter_SC);                     //linear cooling -- straight line
        trip_counter = trip_counter - (1200000/trip_timer);
    }
    else if(Filter_SC <= 2048 && Filter_SC > 737)                                       //current between 0A and -160 A
    {
        trip_timer = interpolate_table_1d(&trip2_table, (4096-Filter_SC));              //linear cooling -- straight line
        trip_counter = trip_counter - (1200000/trip_timer);
    }
    else if(Filter_SC > 3359)                                                           //current larger than 160 A
    {
        trip_timer = interpolate_table_1d(&trip_table, Filter_SC);                      //Non-linear heating
        trip_counter = trip_counter + (1200000/trip_timer);
        /*      if( Filter_SC> testvariable2)
               testvariable2 = Filter_SC;
           testvariable++;*/
    }
    else                                                                                //current smaller than -160 A
    {
        trip_timer = interpolate_table_1d(&trip_table, (4096-Filter_SC));               //Non-linear heating
        trip_counter = trip_counter + (1200000/trip_timer);
    }

    //if(trip_counter > testvariable)
    //testvariable = trip_counter;

    if(trip_counter  > 1200000)
        OverCurrentFault();
    else if(trip_counter < 0)															//counter out of bounds
        trip_counter = 0;

    //do some series testing here!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    /*   if(trip_counter > 2000)
    {
        SOP_discharge = (((interpolate_table_1d(&trip3_table, trip_counter) - 2048) * 122)/1000 * (Uint16)Voltage_total)/100;
    }
    else if(trip_counter < 2000)
    {
        SOP_discharge = (((interpolate_table_1d(&trip3_table, 2000) - 2048) * 122)/1000 * (Uint16)Voltage_total)/100 ;
    }*/
    ////////////////////////////////////////////////////////////////////////////////////////////////////

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
    static long Pre_Charge_Measure_filter = 0;
    static long Pre_Charge_Measure_filter_temp = 0;
    /* static long Proximty_Measure_filter_temp = 0;
    static long Proximty_Measure_filter = 0;*/

    //Deurlaat filter y(k) = y(k - 1) + a[x(k) - y(k - 1)] met a = 1 - e^-WcTs
    //a = 0.015 ~ 0.1Hz, a = 0.12 ~ 1Hz, a = 0.47 ~ 5Hz

    Pre_Charge_Measure_filter = Pre_Charge_Measure_filter_temp + ((AdcResult.ADCRESULT13 - Pre_Charge_Measure_filter_temp)/2);      //50hz sny af op 0.1hz - 2.3V ADC = 48V
    Pre_Charge_Measure_filter_temp = Pre_Charge_Measure_filter;

    //Pre_Charge_Measure = AdcResult.ADCRESULT13;

    //adc: (1.051*3.3)/(0.051*4096) =  0.0166 ~ 167/10000
    Pre_Charge_Measure_filter = (Pre_Charge_Measure_filter * 167)/10000;
    Pre_Charge_Measure = (Uint16)Pre_Charge_Measure_filter;

    /*    Proximty_Measure_filter = Proximty_Measure_filter_temp + ((AdcResult.ADCRESULT11 - Proximty_Measure_filter_temp)/2);      //50hz sny af op 0.1hz - 2.3V ADC = 48V
    Proximty_Measure_filter_temp = Proximty_Measure_filter;

    Proximty_Measure_filter = (3300*Proximty_Measure_filter)/4096;               //50hz calculate f_cut-off - mV measurement
    Proximity_Measure = (int)Proximty_Measure_filter;
     */
    //adc: (2.7k*3.3)/(2.7k+330) =  2.94V (Not connected) OR (407*3.3)/(737) =  1.82V (Connected)
    //Actual: 2.8V -> Not Connected OR 2V -> Connected
    /*    if(Proximity_Measure<2400 && flagCharged == 0)     //2300                 //connected
        CHG_J1772_Ctrl = 1;                            //switch on
    else
        CHG_J1772_Ctrl = 0;                            //switch off
     */
    //Pilot_Measure not currently in used. Needs to be implemented to monitor higher current charging applications
    //Will require to measure 1kHz pwm duty cycle
    //Pilot_Measure = 3300*(Pilot_Measure_temp + (AdcResult.ADCRESULT12-Pilot_Measure_temp))/4096;                        //50hz calculate f_cut-off - mV measurement
    //Pilot_Measure = AdcResult.ADCRESULT12;
    //Pilot_Measure_temp = Pilot_Measure;

    if(Key_switch_2 == 1 && Charger_status == 0)
    {
        //binne die keydrive if
        if(flagDischarged == 0 && flagCurrent == 0  && flagTemp_Discharge == 0)
        {
            if(Pre_Charge_Measure > (0.8*Voltage_total))                                       //set to an appropriate value - maybe 0.8 is better
            {
                Contactor_On();
                Pre_Charge_Off();
                led2 = ~led2;
            }
            else
            {
                Pre_Charge_On();
            }
        }
        else
        {
            Contactor_Off();
        }
    }
    else if(Key_switch_2 == 0 && Charger_status == 0)       //Keyswitch = to zero
    {
        Contactor_Off();
        flagCurrent = 0;
    }

    EALLOW;
    CpuTimer1.InterruptCount++;
    EDIS;
}

__interrupt void cpu_timer2_isr(void)
{
    EALLOW;
    //counter
    if(Key_switch_2 == 1 && flagCurrent == 0)
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
        //CANSlaveReception();                                        //Handle the received message
        ECanaRegs.CANRMP.bit.RMP1 = 1;
    }
    else if (ECanaRegs.CANRMP.bit.RMP2 == 1)
    {
        //CANChargerReception();
        CAN_Charger_dataL = ECanaMboxes.MBOX2.MDL.all;                // Data taken out of direct mailbox
        CAN_Charger_dataH = ECanaMboxes.MBOX2.MDH.all;                // Data taken out of direct mailbox
        ECanaRegs.CANRMP.bit.RMP2 = 1;
    }
    else if (ECanaRegs.CANRMP.bit.RMP3 == 1)
    {
        //Add CANTRANSMIT here to deal with the answer if answer is short?
        //what to do with BMS state? execute state change?

        //CAN_Charger_dataL = ECanaMboxes.MBOX3.MDL.all;                // Data taken out of direct mailbox
        ECanaRegs.CANRMP.bit.RMP3 = 1;

    }
    else if (ECanaRegs.CANRMP.bit.RMP4 == 1)                          //NMT_MOSI
    {
        Uint16 NMT_Command = 0;
        Uint16 NMT_Command_Address = 0;

        NMT_Command = ECanaMboxes.MBOX4.MDL.all & 0xFF;                            //needs testing
        NMT_Command_Address = (ECanaMboxes.MBOX4.MDL.all >>8 ) & 0xFF;             //

        if(NMT_Command_Address == 0x1D || NMT_Command_Address == 0x00)             //should also listen to 0x0
        {
            switch(NMT_Command) {
            case 0x1 :
                NMT_State = 0x5;                   //Enter operational state
                break;
            case 0x2 :
                NMT_State = 0x4;                   //Enter stopped state
                break;
            case 0x80 :
                NMT_State = 0x7F;                  //Enter Pre-operational state
                break;
            case 0x81 :                            //Enter Reset the Device - (initialization sub-state)
                NMT_State = 0x0;
                while(NMT_State<0x100){;}          //reset using watchdog timer
                break;
            case 0x82 :                            //Reset the CAN bus - (initialization sub-state)
                NMT_State = 0x0;
                //add reset function
                break;
            }
        }
        //transmit heart-beat return or transmit it in the main loop????? Most probably
        CANTransmit(0x71D, 0x0, NMT_State, 0x1, 7); //Destination: 0x71C, Mailbox_high: 0, Mailbox_low: NMT_State, bytes: 1, Mailbox: 7,

        ECanaRegs.CANRMP.bit.RMP4 = 1;
    }
    else if (ECanaRegs.CANRMP.bit.RMP5 == 1)                          //PDO1_MOSI
    {
        static Uint16 PDO_Command = 0;

        PDO_Command = ECanaMboxes.MBOX5.MDL.all & 0xFF;

        if(NMT_State == 0x5)                                    //ensure system is in operational state
        {
            //bit 0 -> high PWR 48V output - initiate pre-charge
            if((PDO_Command & 0x1) == 1)
            {
                Pre_Charge_On();                                //follow Pre-charge pin - add contactor closing later
            }                                                   //Make use of timer to ensure Contactor closes after 2 seconds?
            else
            {
                Pre_Charge_Off();                               //follow Pre-charge pin
                Contactor_Off();                                //turn off contactor
            }

            //bit 1 -> high PWR 12V output - turn on 12V
            //Aux_Control = ((PDO_Command>>1 ) & 0x1);            //replace with Uax_Supply_12V_On();



            //bit 2 -> Low PWR 12V output - turn on 12V - usually ON but should maybe be inverse
            LPwr_Out_Ctrl_1 = ((PDO_Command>>2 ) & 0x1);

            //bit 3 -> Reset Flag - Current_flag = 0;
            if((PDO_Command>>3 & 0x1) == 1)
            {
                flagCurrent = 0;
            }

            CANTransmit(0x19C, (((Uint32)((SOP_charge<<8)|SOP_discharge))<<16) | (int16)(Voltage_total*Current), (((Uint32)BMS_Error)<<16) | (Uint32)BMS_Status, 0x8, 8); //Destination: 0x71C, Mailbox_high: 0, Mailbox_low: 0, bytes: 8, Mailbox: 8
        }
        //else return nothing

        ECanaRegs.CANRMP.bit.RMP5 = 1;
    }
    else if (ECanaRegs.CANRMP.bit.RMP6 == 1)                          //SDO_MOSI
    {
        Uint16 SDO_MOSI_Ctrl = 0;
        Uint16 SDO_MOSI_Index = 0;
        Uint32 SDO_MISO_Ctrl = 0;
        Uint32 SDO_MISO_Data = 0;

        SDO_MOSI_Ctrl = ECanaMboxes.MBOX6.MDL.all & 0xFF;
        SDO_MOSI_Index = (ECanaMboxes.MBOX6.MDL.all>>8) & 0xFFFF;

        if(NMT_State != 0x4 && SDO_MOSI_Ctrl == 0x42)                 //ensure system is not in stopped state
        {
            switch(SDO_MOSI_Index)
            {
            case 0x0900 :                                             //Voltage_total
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | ((int16)((Voltage_total*100)));
                break;
            case 0x0902 :                                             //Current
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Current*100);
                break;
                /* case 0x0904 :                                             //Power
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8  | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Current*Voltage_total);
                break;
            case 0x0906 :                                              //Voltage_low
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8  | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Voltage_low*1000);
                break;
            case 0x0908 :                                              //Voltage_low_cell
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8  | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Voltage_low_cell);
                break;
            case 0x090A :                                              //Voltage_high
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Voltage_high*1000);
                break;
            case 0x090C :                                              //Voltage_high_cell
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Voltage_high_cell);
                break;
            case 0x090E :                                              //Voltage_avg
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Voltage_avg*1000);
                break;
            case 0x0910 :                                               //Temperature_high
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Temperature_high*10);
                break;
            case 0x0912 :                                               //Temperature_high_cell
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Temperature_high_cell);
                break;
            case 0x0914 :                                              //Temperature_low
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Temperature_low*10);
                break;
            case 0x0916 :                                              //Temperature_low_cell
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Temperature_low_cell);
                break;
            case 0x0918 :                                               //Temp_avg
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Temperature_avg*10);
                break;
            case 0x091A :                                               //Aux_Voltage
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Auxilliary_Voltage*1000);
                break;
            case 0x091C :                                               //SOC
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(SOC);
                break;
            case 0x091E :                                               //Impedance     Review calculation
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(resistance*1000000);
                break;
            case 0x0920 :                                               //Maximum cell impedance      Review calculation
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(SOH_max*1000000);
                break;
            case 0x0922 :                                               //Maximum impedance cell number
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(SOH_max_cell);
                break;
            case 0x0924 :                                               //Cell 0
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Voltages[0]*1000);
                break;
            case 0x0926 :
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Voltages[1]*1000);
                break;
            case 0x0928 :
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Voltages[2]*1000);
                break;
            case 0x092A :
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Voltages[3]*1000);
                break;
            case 0x092C :
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Voltages[4]*1000);
                break;
            case 0x092E :
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Voltages[5]*1000);
                break;
            case 0x0930 :
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Voltages[6]*1000);
                break;
            case 0x0932 :
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Voltages[7]*1000);
                break;
            case 0x0934 :
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Voltages[8]*1000);
                break;
            case 0x0936 :
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Voltages[9]*1000);
                break;
            case 0x0938 :
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Voltages[10]*1000);
                break;
            case 0x093A :
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Voltages[11]*1000);
                break;
            case 0x093C :
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Voltages[12]*1000);
                break;
            case 0x093E :
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Voltages[13]*1000);
                break;
            case 0x0940 :
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Voltages[14]*1000);
                break;
            case 0x0942 :
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Temperatures[0]*10);
                break;
            case 0x0944 :
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Temperatures[1]*10);
                break;
            case 0x0946 :
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Temperatures[2]*10);
                break;
            case 0x0948 :
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Temperatures[3]*10);
                break;
            case 0x094A :
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Temperatures[4]*10);
                break;

            case 0x094B :
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = (Uint32)(BMS_Status);
                break;
            case 0x094E :
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = (Uint32)(BMS_Error);
                break;
            case 0x0950 :                                                                   //Battery SOH
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = (Uint32)(SOH);                                              //add SOH calculation....
                break;
            case 0x0952 :                                                                   //Peak current
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(I_maximum*100);
                break;
            case 0x0954 :                                                                   //Peak Voltage
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Voltage_maximum*1000);
                break;
            case 0x0956 :                                                                   //Peak Temperature
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Temperature_maximum*10);
                break;
            case 0x0958 :                                                                   //Minimum current
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(I_minimum*100);
                break;
            case 0x095A :                                                                   //Minimum Voltage
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Voltage_minimum*1000);
                break;
            case 0x095C :                                                                   //Minimum Temperature
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Temperature_minimum*10);
                break;
            case 0x095E :                                                                   //cycle tracker
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SDO_MISO_Data | (int16)(Cycles*10);
                break;
            case 0x0960 :
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = (Uint32)(Initial_Capacity);
                break;
            case 0x0962 :                                                                   //State of power discharge
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SOP_discharge;
                break;
            case 0x0964 :                                                                   //State of power charge
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = SOP_charge;
                break;*/
            default:
                //return error status
                SDO_MISO_Ctrl = ((Uint32)SDO_MOSI_Index)<<8 | 0x40;
                SDO_MISO_Data = 0x06020000;                                                 //Error: Object does not exist
            }
            CANTransmit(0x59C, SDO_MISO_Data, SDO_MISO_Ctrl, 0x8, 0x9); //Destination: 0x59C, Mailbox_high: 0, Mailbox_low: NMT_State, bytes: 8, Mailbox: 9,

        }

        ECanaRegs.CANRMP.bit.RMP6 = 1;
    }

    PieCtrlRegs.PIEACK.bit.ACK9 = 1;                // Acknowledge this interrupt to receive more interrupts from group 9
}

__interrupt void can_tx_isr(void)
{
    if(ECanaRegs.CANTA.bit.TA0 == 1)                //normal CAN transmit for CHG and speedometer
    {
        ECanaRegs.CANTA.bit.TA0 = 1;
    }
    else if(ECanaRegs.CANTA.bit.TA7 == 1)            //Heartbeat_MISO CAN transmit
    {
        ECanaRegs.CANTA.bit.TA7 = 1;
    }
    else if(ECanaRegs.CANTA.bit.TA8 == 1)            //PDO1_MISO CAN transmit
    {
        ECanaRegs.CANTA.bit.TA8 = 1;
    }
    else if(ECanaRegs.CANTA.bit.TA9 == 1)            //SDO_MISO CAN transmit
    {
        ECanaRegs.CANTA.bit.TA9 = 1;
    }

    PieCtrlRegs.PIEACK.bit.ACK9 = 1;                // Acknowledge this interrupt to receive more interrupts from group 9
}
