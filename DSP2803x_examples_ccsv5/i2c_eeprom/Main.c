//###########################################################################
// $TI Release: F2803x C/C++ Header Files and Peripheral Examples V127 $
// $Release Date: March 30, 2013 $

//To Do List:
//Add Temperature influence on SOC
//###########################################################################

#include "User\User_Defines.h"

void main(void)
{
    Initialise_BMS();

    for(;;)
    {
        if(counter_2Hz == 1)
        {
            ServiceDog();
        }
        if(counter_2Hz == 2)
        {
            Toggle_LED();
            Status_LED();

            Read_Cell_Voltages();
            Process_Voltages();

            Calculate_Current();

            Read_Temperatures();
            Process_Temperatures();

            ServiceDog();

            CANChargerReception(CAN_Charger_dataL, CAN_Charger_dataH);

            Balance(5,Vbalance);

            Battery_Status();
            Battery_Error();
            CAN_Output_All();

            ServiceDog();

            Calibrate_Current();

            //BQ check status
            Process_BQ_System_Status();
            Reset_BQ_ADC();

            Calculate_SOC();
            Calculate_SOH();

            counter_2Hz = 0;

            //add in issue function check
            Reset_MCU(1);
        }
    }
}
