/*
 * User_I2C.c
 *
 *  Created on: 04 May 2017
 *      Author: Sonja
 */

#include "User_I2C.h"

void I2CA_Init(void)
{
	// Initialize I2C
	I2caRegs.I2CSAR = 0x08;         // Slave address - EEPROM control code              0x08
	I2caRegs.I2CPSC.all = 6;        // Prescaler - need 7-12 Mhz on module clk          8Mhz
	I2caRegs.I2CCLKL = 40;          // NOTE: must be non zero                           40
	I2caRegs.I2CCLKH = 40;          // NOTE: must be non zero                           40  Moet 95.238kHz tot gevolg hê..
	I2caRegs.I2CIER.all = 0x24;     // Enable SCD & ARDY interrupts     0x24
	I2caRegs.I2CMDR.all = 0x0020;   // Take I2C out of reset
	I2caRegs.I2CFFTX.all = 0x6000;  // Enable FIFO mode and TXFIFO
	I2caRegs.I2CFFRX.all = 0x2040;  // Enable RXFIFO, clear RXFFINT,
}


Uint16 I2CA_WriteData(unsigned char Register, unsigned char Data)
{
	unsigned char DataBuffer[4];

	while(I2caRegs.I2CMDR.bit.STP == 1);

	// Setup slave address
	I2caRegs.I2CSAR = 0x08;

	// Check if bus busy
	while(I2caRegs.I2CSTR.bit.BB == 1);

	CurrentMsgPtr = &I2cMsgOut1;

	// Setup number of bytes to send
	I2caRegs.I2CCNT = 3;

	DataBuffer[0] = (0x08) << 1;
	DataBuffer[1] = Register;
	DataBuffer[2] = Data;
	DataBuffer[3] = CRC8(DataBuffer, 3, 7); //bereken crc

	// Setup data to send
	I2caRegs.I2CDXR = Register;
	I2caRegs.I2CDXR = Data;
	I2caRegs.I2CDXR = DataBuffer[3];                            //send crc

	// Send start as master transmitter
	I2caRegs.I2CMDR.all = 0x6E20;

	I2cMsgOut1.MsgStatus = I2C_MSGSTAT_WRITE_BUSY;

	while(CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_WRITE_BUSY);

	if(CurrentMsgPtr->MsgStatus == 0xFF)									//check to see if write was succesfull
	{
		I2CA_WriteData(Register, Data);
	}
	ServiceDog();
	return I2C_SUCCESS;
}// end of write section

Uint16 I2CA_ReadData(struct I2CMSG *msg, unsigned char Register, Uint16 amount)
{
	while(I2caRegs.I2CMDR.bit.STP == 1);

	// Check if bus busy
	while(I2caRegs.I2CSTR.bit.BB == 1);

	// Setup slave address
	I2caRegs.I2CSAR = 0x08;

	// Setup number of bytes to send
	I2caRegs.I2CCNT = 1;

	// Setup data to send
	I2caRegs.I2CDXR = Register;

	I2caRegs.I2CMDR.all = 0x2E20;

	CurrentMsgPtr = &I2cMsgIn1;
	I2cMsgIn1.MsgStatus = I2C_MSGSTAT_SEND_NOSTOP_BUSY;

	while(CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP_BUSY);

	if(CurrentMsgPtr->MsgStatus == 0xFF)									//dink dit moet voor die laaste return kom though
	{
		I2CA_ReadData(msg, Register, amount);
	}

	if(msg->MsgStatus == I2C_MSGSTAT_RESTART)
	{
		while(I2caRegs.I2CMDR.bit.STP == 1);

		// Check if bus busy
		while(I2caRegs.I2CSTR.bit.BB == 1);

		// Setup slave address
		I2caRegs.I2CSAR = 0x08;

		I2caRegs.I2CCNT = 2;                    // Setup how many bytes to expect
		I2caRegs.I2CMDR.all = 0x2C20;           // Send restart as master receiver

		// Update current message pointer and message status
		CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_READ_BUSY;

		while(CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_READ_BUSY);

		if(CurrentMsgPtr->MsgStatus == 0xFF)									//dink dit moet voor die laaste return kom though
		{
			I2CA_ReadData(msg, Register, amount);
		}

		Received = DataOut;

		if(amount == 2)
		{
			Received = Received<<8;

			while(I2caRegs.I2CMDR.bit.STP == 1);

			// Setup slave address
			I2caRegs.I2CSAR = 0x08;

			I2caRegs.I2CCNT = 2;                    // Setup how many bytes to expect
			I2caRegs.I2CMDR.all = 0x2C20;           // Send restart as master receiver

			// Update current message pointer and message status
			CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_READ_BUSY;

			while(CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_READ_BUSY);

			if(CurrentMsgPtr->MsgStatus == 0xFF)									//dink dit moet voor die laaste return kom though
			{
				I2CA_ReadData(msg, Register, amount);
			}
			Received = Received | DataOut;

		}
		ServiceDog();
		return Received;
	}
	ServiceDog();
	return I2C_SUCCESS;
}
