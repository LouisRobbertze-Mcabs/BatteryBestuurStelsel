/*
 * User.ADC.h
 *
 *  Created on: 04 May 2017
 *      Author: Sonja
 */

#ifndef USER_USER_ADC_H_
#define USER_USER_ADC_H_

#include "User_Defines.h"

void configADC(void);
void Reset_ADC(void);
void Reset_MCU(int flag);
void Pre_Charge_Ctrl(void);

#endif /* USER_USER_ADC_H_ */
