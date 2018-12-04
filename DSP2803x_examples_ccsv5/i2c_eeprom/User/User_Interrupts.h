/*
 * User_Interrupts.h
 *
 *  Created on: 04 May 2017
 *      Author: Sonja
 */

#ifndef USER_USER_INTERRUPTS_H_
#define USER_USER_INTERRUPTS_H_

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "math.h"
#include <stdio.h>
#include <float.h>
#include <string.h>
#include "User_Defines.h"
#include "User_Prototypes.h"
#include "User_Globals.h"

__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void i2c_int1a_isr(void);
__interrupt void adc_isr(void);
//__interrupt void spiRxFifoIsr(void);
__interrupt void can_rx_isr(void);
__interrupt void can_tx_isr(void);

#endif /* USER_USER_INTERRUPTS_H_ */
