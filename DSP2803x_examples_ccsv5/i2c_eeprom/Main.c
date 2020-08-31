//###########################################################################
// $TI Release: F2803x C/C++ Header Files and Peripheral Examples V127 $
// $Release Date: March 30, 2013 $

//To Do List:
//Check if CRC is correct, and BQ is responding
//Doen ekstra reset flag so that if new data is not returned contactor can't close / toets..
//Check CAN reset procedure to ensure always sending data -- Looks positive
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

			Read_Cell_Voltages();
			Calculate_Current();
			Process_Voltages();

			Read_Temperatures();

			ServiceDog();

			CANChargerReception(CAN_Charger_dataL, CAN_Charger_dataH);

			Balance(5,Vbalance);

			Battery_Status();
			CAN_Output_All();

			ServiceDog();

			Calibrate_Current_charger();

			Read_System_Status();
			Process_System_Status();

			Calculate_SOC();
			Calculate_SOH();

			counter_2Hz = 0;
			Reset_ADC();

			//add in issue function check
			Reset_MCU(1);
		}
	}
}
