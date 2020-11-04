/*
 * User_CAN.h
 *
 *  Created on: 04 May 2017
 *      Author: Sonja
 */

#ifndef USER_USER_CAN_H_
#define USER_USER_CAN_H_

#include "User_Defines.h"

void CAN_Init(void);
void CANSetup(void);
void CANMailboxConfig(void);
void CANInterruptConfig(void);
void CANChargerReception(Uint32 RxDataL, Uint32 RxDataH);
void CANSlaveReception(void);
void CAN_Output_All(void);
void CANSlaveConfig(void);
void CANTransmit(Uint16 Destination, Uint32 TxDataH, Uint32 TxDataL, Uint16 Bytes);
float Charger_inputData_parse(Uint32 data);

#endif /* USER_USER_CAN_H_ */
