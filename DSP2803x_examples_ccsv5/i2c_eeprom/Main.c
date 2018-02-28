//###########################################################################
// $TI Release: F2803x C/C++ Header Files and Peripheral Examples V127 $
// $Release Date: March 30, 2013 $
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

			ServiceDog();

			Read_Temperatures();
			Balance(5,Vbalance);

			Calculate_Current();
			Calibrate_Current_charger();

			Read_System_Status();
			Process_System_Status();

			Calculate_SOC();
			Calculate_SOH();

			counter_2Hz = 0;
			Reset_ADC();
		}
	}
}
