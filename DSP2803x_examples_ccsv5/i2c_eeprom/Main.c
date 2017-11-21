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
			Process_Voltages();
			Calculate_Current();

			ServiceDog();

			Calculate_SOC();

			Read_Temperatures();
			Balance(5,Vbalance);

			Read_System_Status();
			Process_System_Status();

			counter_2Hz = 0;
			Reset_ADC();
		}
	}
}
