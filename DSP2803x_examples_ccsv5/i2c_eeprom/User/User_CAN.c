/*
 * User_CAN.c
 *
 *  Created on: 04 May 2017
 *      Author: Bartho Horn
 */

#include "User_CAN.h"

void CAN_Init(void)
{
	CANSetup();
	CANMailboxConfig();
	CANInterruptConfig();
}

void CANSetup(void)
{
	EALLOW;

	// Configure CAN GPIO pins
	GpioCtrlRegs.GPAPUD.bit.GPIO30 = 0;             // Enable pull-up for GPIO30 (CANRXA)
	GpioCtrlRegs.GPAPUD.bit.GPIO31 = 0;             // Enable pull-up for GPIO31 (CANTXA)
	GpioCtrlRegs.GPAQSEL2.bit.GPIO30 = 3;           // Asynch qual for GPIO30 (CANRXA)
	GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 1;            // Configure GPIO30 for CANRXA operation
	GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 1;            // Configure GPIO31 for CANTXA operation
	GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;            // Set the enable signal to the CAN transciever as GPIO
	GpioCtrlRegs.GPADIR.bit.GPIO27 = 1;             // Set the enable signal to the CAN transciever as output
	GpioDataRegs.GPADAT.bit.GPIO27 = 1;             // Set the enable signal to the CAN transciever high

	// Configure eCAN RX and TX pins
	struct ECAN_REGS ECanaShadow;
	ECanaShadow.CANTIOC.all = ECanaRegs.CANTIOC.all;
	ECanaShadow.CANTIOC.bit.TXFUNC = 1;
	ECanaRegs.CANTIOC.all = ECanaShadow.CANTIOC.all;
	ECanaShadow.CANRIOC.all = ECanaRegs.CANRIOC.all;
	ECanaShadow.CANRIOC.bit.RXFUNC = 1;
	ECanaRegs.CANRIOC.all = ECanaShadow.CANRIOC.all;

	// Configure Master Control register
	ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
	ECanaShadow.CANMC.bit.SCB = 0;                  //  Martin Rademeyer //
	ECanaShadow.CANMC.bit.DBO = 1;                  //  Martin Rademeyer //     1 bartho
	ECanaShadow.CANMC.bit.ABO = 1; 					//Bartho testing Wake up on bus activity..
	ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

	// Initialise Message Control registers
	ECanaMboxes.MBOX0.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX1.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX2.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX3.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX4.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX5.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX6.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX7.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX8.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX9.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX10.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX11.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX12.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX13.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX14.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX15.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX16.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX17.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX18.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX19.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX20.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX21.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX22.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX23.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX24.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX25.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX26.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX27.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX28.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX29.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX30.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX31.MSGCTRL.all = 0x00000000;

	// Clear flag registers
	ECanaRegs.CANTA.all = 0xFFFFFFFF;
	ECanaRegs.CANRMP.all = 0xFFFFFFFF;
	ECanaRegs.CANGIF0.all = 0xFFFFFFFF;
	ECanaRegs.CANGIF1.all = 0xFFFFFFFF;

	// Configure bit-timing
	ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
	ECanaShadow.CANMC.bit.CCR = 1 ;
	ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
	do {ECanaShadow.CANES.all = ECanaRegs.CANES.all;}   // Wait until the CPU has been granted permission to change the configuration registers
	while (ECanaShadow.CANES.bit.CCE != 1 );            // Wait for CCE bit to be set..
	ECanaShadow.CANBTC.all = 0;
	ECanaShadow.CANBTC.bit.BRPREG = 3;
	//  ECanaShadow.CANBTC.bit.BRPREG = 15;                 //  Martin Rademeyer //
	//  ECanaShadow.CANBTC.bit.BRPREG = 1;                  //  Martin Rademeyer //
	ECanaShadow.CANBTC.bit.TSEG2REG = 2;                //  Martin Rademeyer //
	ECanaShadow.CANBTC.bit.TSEG1REG = 10;               //  Martin Rademeyer //
	ECanaShadow.CANBTC.bit.SAM = 0;                     //1
	ECanaRegs.CANBTC.all = ECanaShadow.CANBTC.all;

	// Finalise CAN configuration
	ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
	ECanaShadow.CANMC.bit.CCR = 0 ;
	ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
	do {ECanaShadow.CANES.all = ECanaRegs.CANES.all;}   // Wait until the CPU no longer has permission to change the configuration registers
	while(ECanaShadow.CANES.bit.CCE != 0 );             // Wait for CCE bit to be  cleared..
	ECanaRegs.CANME.all = 0;                            // Disable all Mailboxes - Required before writing the MSGIDs

	EDIS;
}

void CANMailboxConfig(void)
{
	ECanaRegs.CANGAM.all = 0x00000000;              // Global All-Pass Mask (Don't care: 1, Match: 0)

	ECanaRegs.CANMD.all = 0x0000000E;               // Message Direction (Rx: 1, Tx: 0)                                 //bartho    0x00000006

	// Tx Mailbox (0x00000001)
	ECanaMboxes.MBOX0.MSGCTRL.all = 0x00000004; // Transmit 4 bytes of data

	// Rx Mailbox (0x00000002)
	ECanaMboxes.MBOX1.MSGID.all = 0;                // Standard ID length, acceptance masks used, no remote frames
	ECanaMboxes.MBOX1.MSGID.bit.STDMSGID = NodeID;  // Current address loaded
	ECanaLAMRegs.LAM1.all = 0x00000000;             // Accept standard IDs with matching address

	ECanaMboxes.MBOX1.MSGCTRL.all = 0x00000005; // Receive 4 bytes of data

	// Rx Mailbox (0x00000003)
	ECanaMboxes.MBOX2.MSGID.all = 0;                // Standard ID length, acceptance masks used, no remote frames      //bartho
	ECanaMboxes.MBOX2.MSGID.bit.STDMSGID = 0x0611;  // Current address loaded                                           //bartho
	ECanaLAMRegs.LAM2.all = 0x00000000;             // Accept standard IDs with matching address                        //bartho
	ECanaMboxes.MBOX2.MSGCTRL.all = 0x00000008; // Receive 8 bytes of data                                              //bartho


	// Rx Mailbox (0x00000004)
	ECanaMboxes.MBOX3.MSGID.all = 0;                // Standard ID length, acceptance masks used, no remote frames      //bartho
	ECanaMboxes.MBOX3.MSGID.bit.STDMSGID = 0x011;  // Current address loaded                                           //bartho
	ECanaLAMRegs.LAM3.all = 0x00000000;             // Accept standard IDs with matching address                        //bartho
	ECanaMboxes.MBOX3.MSGCTRL.all = 0x00000005; // Receive 5 bytes of data                                              //bartho

	ECanaRegs.CANME.all = 0x0000000E;               // Enable Rx Mailbox                                                //bartho    0x00000006

	// The Tx Mailbox MSGID has to be set as required and then enabled
}

void CANInterruptConfig(void)
{
	DINT;                                           // Global interrupt disable

	EALLOW;
	ECanaRegs.CANGIM.all = 0x00000003;              // Enable ECAN0INT and ECAN0INT interrupt lines
	ECanaRegs.CANMIM.all = 0x0000000F;              // Allow interrupts for Mailbox 0 and 1 , 2,3
	ECanaRegs.CANMIL.all = 0x00000001;              // Mailbox 0 triggers ECAN1INT line, Mailbox 1 triggers ECAN0INT line       //1 bartho
	PieVectTable.ECAN0INTA = &can_rx_isr;           // Link Rx ISR function
	PieVectTable.ECAN1INTA = &can_tx_isr;           // Link Tx ISR function
	EDIS;

	PieCtrlRegs.PIEIER9.bit.INTx5 = 1;              // Enable Rx interrupt in PIE group 9
	PieCtrlRegs.PIEIER9.bit.INTx6 = 1;              // Enable Tx interrupt in PIE group 9

	IER |= M_INT9;                                  // Enable group 9 interrupts

	EINT;                                           // Global interrupt enable
}

void CANChargerReception(Uint32 RxDataL, Uint32 RxDataH)
{ //maybe setup CAN status structure to capture all the info required. -Charger active, battery charging,
	float ChgVoltage = 0;

	Uint16 ChgStatus = 0;
	Uint16 temp = 0;
	Uint16 temp2 = 0;

	static volatile float Current_max = 5;

	static volatile int delay = 0;  // miskien >> count 1 cycle from contactor closes till charger starts
	//   >> count 1 cycle from charger stops till contactor opens

	if(RxDataL != 0 || RxDataH != 0)
	{
		//Read Charger Voltage
		temp = RxDataL;
		temp2 = (temp & 0xFF) << 8;
		temp2 = ((temp &0xFF00)>>8) | temp2;
		ChgVoltage = (float)temp2*0.1;

		//Read Charger Current
		temp = (RxDataL& 0xFFFF0000)>>16;
		temp2 = (temp & 0xFF) << 8;
		temp2 = ((temp &0xFF00)>>8) | temp2;
		ChgCurrent = (float)temp2*0.1;

		//Read Charger Status
		ChgStatus = RxDataH & 0xFF;

		if(ChgStatus == 0 || ChgStatus == 0x08 /*|| ChgStatus == 0x10*/)                        //Charger ready to charge || battery voltage low flag || Comms issue
		{
			Charger_status = 1;														//0 - not plugged in, 1 -plugged in, 2 - plugged in and charging ?????											//charger connected
			if(flagCurrent == 0 && flagTemp == 0 && flagCharged == 0 && KeySwitch == 0 /*&& flagDischarged != 2*/)     //check flags to ensure charging is allowed
			{
				if(delay == 0)                                                              //sit miskien check in om met die charger Vbat te meet
				{
					ContactorOut = 1;                                                       //turn on contactor
					delay++;
				}
				else if(delay == 1)
				{
					Current_max =  Current_max + kp_multiplier*(kp_constant - Voltage_high);								//kp controller constant & kp multiplier

					if(Current_max <0)
						Current_max = 0;
					else if(Current_max > 25)
						Current_max = 25;


					if(Voltage_high> balancing_upper_level && Voltage_low < balancing_bottom_level)	//determine if balancing should start									//balancing upper level & balancing lower level
					{
						balance = 1;
					}
					else if(Voltage_high> balancing_upper_level && Voltage_low > balancing_bottom_level && Current > -2) //if cell high > 3.48 balance
					{
						flagCharged = 1;
						SOC = 100;						//Hierdie is toets fase om SOC by 100 te kry!
					}

					Charging_animation = 1;												//0 - not plugged in, 1 -plugged in
					CANTransmit(0x618, 0, ChgCalculator(52.5, Current_max), 8);             //charging started
					PreCharge = 1;                          								//turn on precharge resistor
				}
			}
			else																			//BMS flag high. Stop charging and disconnect blah blah
			{
				if(delay == 1)                                                              //sit miskien check in om met die charger Vbat te meet
				{
					CANTransmit(0x618,1,ChgCalculator(52.5, 0),8);                            //disconnect charger
					delay--;
				}
				else if(delay == 0)
				{                                                   //turn off contactor
					CANTransmit(0x618,1,ChgCalculator(52.5, 0),8);                            //disconnect charger
					if(flagCharged == 1)
						ContactorOut = 0;
					Charging_animation = 0;
				}
			}
		}
		else                                                                                //Charger flag set. typically power disconnected
		{
			if(delay == 1)
			{
				CANTransmit(0x618,1,ChgCalculator(52.5, 0),8);                                //disconnect charger
				delay--;
			}
			else if(delay == 0)
			{

				CANTransmit(0x618,1,ChgCalculator(52.5, 0),8);                              //disconnect charger
				ContactorOut = 0;                                                           //turn off contactor

				Charger_status = 0;															//add counter to monitor if charger is unplugged?
				Charging_animation = 0;
				Current_max = 5;															//speel rond om charge stabiel te kry
			}
		}

		ChargerVoltage = ChgVoltage;
		ChargerCurrent = ChgCurrent;

		CAN_Charger_dataL = 0;
		CAN_Charger_dataH = 0;
	}
}

void CANSlaveReception(void)
{
	Uint32 RxData = 0;
	//	Uint32 RxData2 = 0;
	union bits32 TxData;

	RxData = ECanaMboxes.MBOX1.MDH.all;             // Data taken out of direct mailbox
	//	RxData2 = ECanaMboxes.MBOX1.MDL.all;

	switch (RxData)
	{
	case 4: {TxData.asFloat=Voltage_total; CANTransmit(0, 4, TxData.asUint,5); break;}
	case 5: {TxData.asFloat=Current; CANTransmit(0, 5, TxData.asUint,5); break;}

	case 6: {TxData.asFloat=Voltage_low; CANTransmit(0, 6, TxData.asUint,5); break;}
	case 7: {TxData.asFloat=Voltage_low_cell; CANTransmit(0, 7, TxData.asUint,5); break;}

	case 8: {TxData.asFloat=Voltage_high; CANTransmit(0, 8, TxData.asUint,5); break;}
	case 9: {TxData.asFloat=Voltage_high_cell; CANTransmit(0, 9, TxData.asUint,5); break;}

	case 10: {TxData.asFloat=Voltage_avg; CANTransmit(0, 10, TxData.asUint,5); break;}

	case 11: {TxData.asFloat=Temperature_high; CANTransmit(0, 11, TxData.asUint,5); break;}
	case 12: {TxData.asFloat=Temperature_high_cell; CANTransmit(0, 12, TxData.asUint, 5); break;}

	case 13: {TxData.asFloat=Temperature_low; CANTransmit(0, 13, TxData.asUint,5); break;}
	case 14: {TxData.asFloat=Temperature_low_cell; CANTransmit(0, 14, TxData.asUint, 5); break;}

	case 15: {TxData.asFloat=Temperature_avg; CANTransmit(0, 15, TxData.asUint,5); break;}

	case 16: {TxData.asFloat=Auxilliary_Voltage; CANTransmit(0, 16, TxData.asUint, 5); break;}
	case 17: {TxData.asFloat=SOC*100; CANTransmit(0, 17, TxData.asUint, 5); break;}

	case 18: {TxData.asFloat= SOH_avg ; CANTransmit(0, 18, TxData.asUint,5); break;}					//r_avg
	case 19: {TxData.asFloat=SOH_max; CANTransmit(0, 19, TxData.asUint,5); break;}						//rmaks
	case 20: {TxData.asFloat=SOH_max_cell; CANTransmit(0, 20, TxData.asUint, 5); break;}				//rcell

	//cell voltage values
	case 21: {TxData.asFloat=Voltages[0]; CANTransmit(0, 21, TxData.asUint,5); break;}
	case 22: {TxData.asFloat=Voltages[1]; CANTransmit(0, 22, TxData.asUint,5); break;}
	case 23: {TxData.asFloat=Voltages[2]; CANTransmit(0, 23, TxData.asUint,5); break;}
	case 24: {TxData.asFloat=Voltages[3]; CANTransmit(0, 24, TxData.asUint,5); break;}
	case 25: {TxData.asFloat=Voltages[4]; CANTransmit(0, 25, TxData.asUint,5); break;}
	case 26: {TxData.asFloat=Voltages[5]; CANTransmit(0, 26, TxData.asUint,5); break;}
	case 27: {TxData.asFloat=Voltages[6]; CANTransmit(0, 27, TxData.asUint,5); break;}
	case 28: {TxData.asFloat=Voltages[7]; CANTransmit(0, 28, TxData.asUint,5); break;}
	case 29: {TxData.asFloat=Voltages[8]; CANTransmit(0, 29, TxData.asUint,5); break;}
	case 30: {TxData.asFloat=Voltages[9]; CANTransmit(0, 30, TxData.asUint,5); break;}
	case 31: {TxData.asFloat=Voltages[10]; CANTransmit(0, 31, TxData.asUint,5); break;}
	case 32: {TxData.asFloat=Voltages[11]; CANTransmit(0, 32, TxData.asUint,5); break;}
	case 33: {TxData.asFloat=Voltages[12]; CANTransmit(0, 33, TxData.asUint,5); break;}
	case 34: {TxData.asFloat=Voltages[13]; CANTransmit(0, 34, TxData.asUint,5); break;}
	case 35: {TxData.asFloat=Voltages[14]; CANTransmit(0, 35, TxData.asUint,5); break;}

	//cell Temperature values
	case 36: {TxData.asFloat=Temperatures[0]; CANTransmit(0, 36, TxData.asUint,5); break;}
	case 37: {TxData.asFloat=Temperatures[1]; CANTransmit(0, 37, TxData.asUint,5); break;}
	case 38: {TxData.asFloat=Temperatures[2]; CANTransmit(0, 38, TxData.asUint,5); break;}
	case 39: {TxData.asFloat=Temperatures[3]; CANTransmit(0, 39, TxData.asUint,5); break;}
	case 40: {TxData.asFloat=Temperatures[4]; CANTransmit(0, 40, TxData.asUint,5); break;}
	case 41: {TxData.asFloat=Temperatures[5]; CANTransmit(0, 41, TxData.asUint,5); break;}
	case 42: {TxData.asFloat=Temperatures[6]; CANTransmit(0, 42, TxData.asUint,5); break;}
	case 43: {TxData.asFloat=Temperatures[7]; CANTransmit(0, 43, TxData.asUint,5); break;}
	case 44: {TxData.asFloat=Temperatures[8]; CANTransmit(0, 44, TxData.asUint,5); break;}
	case 45: {TxData.asFloat=Temperatures[9]; CANTransmit(0, 45, TxData.asUint,5); break;}
	case 46: {TxData.asFloat=Temperatures[10]; CANTransmit(0, 46, TxData.asUint,5); break;}
	case 47: {TxData.asFloat=Temperatures[11]; CANTransmit(0, 47, TxData.asUint,5); break;}
	case 48: {TxData.asFloat=Temperatures[12]; CANTransmit(0, 48, TxData.asUint,5); break;}
	case 49: {TxData.asFloat=Temperatures[13]; CANTransmit(0, 49, TxData.asUint,5); break;}
	case 50: {TxData.asFloat=Temperatures[14]; CANTransmit(0, 50, TxData.asUint,5); break;}
	case 51: {TxData.asFloat=Temperatures[15]; CANTransmit(0, 51, TxData.asUint,5); break;}
	//case 52: {if(RxData2==0x8){Fan_Control = 1;}else if(RxData2==0x4){Fan_Control = 0;}; break;}
	}
}

void CAN_Output_All(void)
{
	Uint16 Acewell_Data = 0;
	//Uint32 RxData = 0;
	union bits32 TxData;
	int i;

	//Battery data
	TxData.asFloat=Voltage_total; CANTransmit(0x700, 4, TxData.asUint,5);
//	queue_insert(0x700, 4, TxData.asUint, 5, &CAN_queue); //insert into queue
	for(i=0;i<1500;i++){};

	TxData.asFloat=Current /*(float)test_current*/; CANTransmit(0x700, 5, TxData.asUint,5);
//	queue_insert(0x700, 5, TxData.asUint, 5, &CAN_queue); //insert into queue
	for(i=0;i<1500;i++){};

	TxData.asFloat=Voltage_low/* (float)testcounter*/; CANTransmit(0x700, 6, TxData.asUint,5);								///////////xcbxcb
//	queue_insert(0x700, 6, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Voltage_low_cell; CANTransmit(0x700, 7, TxData.asUint,5);
//	queue_insert(0x700, 7, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};

	TxData.asFloat=Voltage_high/*(float)testvariable*/; CANTransmit(0x700, 8, TxData.asUint,5);									///////////ljdfvbhsd
//	queue_insert(0x700, 8, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Voltage_high_cell; CANTransmit(0x700, 9, TxData.asUint,5);
//	queue_insert(0x700, 9, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};


	TxData.asFloat=Voltage_avg/*(float)testvariable2*/; CANTransmit(0x700, 10, TxData.asUint,5);							/////////////lksbhodfhx
//	queue_insert(0x700, 10, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};

	TxData.asFloat=Temperature_high /*(float)trip_counter*/; CANTransmit(0x700, 11, TxData.asUint,5); 						/////////dhjdghj
//	queue_insert(0x700, 11, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Temperature_high_cell; CANTransmit(0x700, 12, TxData.asUint, 5);
//	queue_insert(0x700, 12, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};

	TxData.asFloat=Temperature_low; CANTransmit(0x700, 13, TxData.asUint,5);
//	queue_insert(0x700, 13, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Temperature_low_cell; CANTransmit(0x700, 14, TxData.asUint, 5);
//	queue_insert(0x700, 14, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};

	TxData.asFloat=Temperature_avg; CANTransmit(0x700, 15, TxData.asUint,5);
//	queue_insert(0x700, 15, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};

	TxData.asFloat=Auxilliary_Voltage; CANTransmit(0x700, 16, TxData.asUint, 5);
//	queue_insert(0x700, 16, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=SOC; CANTransmit(0x700, 17, TxData.asUint, 5);
//	queue_insert(0x700, 17, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};

	TxData.asFloat= SOH_avg ; CANTransmit(0x700, 18, TxData.asUint,5);				//r_avg
//	queue_insert(0x700, 18, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=SOH_max; CANTransmit(0x700, 19, TxData.asUint,5);						//rmaks
//	queue_insert(0x700, 19, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=SOH_max_cell; CANTransmit(0x700, 20, TxData.asUint, 5);			//rcell
//	queue_insert(0x700, 20, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};

	//cell voltage values
	TxData.asFloat=Voltages[0]; CANTransmit(0x700, 21, TxData.asUint,5);
//	queue_insert(0x700, 21, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Voltages[1]; CANTransmit(0x700, 22, TxData.asUint,5);
//	queue_insert(0x700, 22, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Voltages[2]; CANTransmit(0x700, 23, TxData.asUint,5);
//	queue_insert(0x700, 23, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Voltages[3]; CANTransmit(0x700, 24, TxData.asUint,5);
//	queue_insert(0x700, 24, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Voltages[4]; CANTransmit(0x700, 25, TxData.asUint,5);
//	queue_insert(0x700, 25, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Voltages[5]; CANTransmit(0x700, 26, TxData.asUint,5);
//	queue_insert(0x700, 26, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Voltages[6]; CANTransmit(0x700, 27, TxData.asUint,5);
//	queue_insert(0x700, 27, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Voltages[7]; CANTransmit(0x700, 28, TxData.asUint,5);
//	queue_insert(0x700, 28, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Voltages[8]; CANTransmit(0x700, 29, TxData.asUint,5);
//	queue_insert(0x700, 29, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Voltages[9]; CANTransmit(0x700, 30, TxData.asUint,5);
//	queue_insert(0x700, 30, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Voltages[10]; CANTransmit(0x700, 31, TxData.asUint,5);
//	queue_insert(0x700, 31, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Voltages[11]; CANTransmit(0x700, 32, TxData.asUint,5);
//	queue_insert(0x700, 32, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Voltages[12]; CANTransmit(0x700, 33, TxData.asUint,5);
//	queue_insert(0x700, 33, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Voltages[13]; CANTransmit(0x700, 34, TxData.asUint,5);
//	queue_insert(0x700, 34, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Voltages[14]; CANTransmit(0x700, 35, TxData.asUint,5);
//	queue_insert(0x700, 35, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};

	//cell Temperature values
	TxData.asFloat=Temperatures[0]; CANTransmit(0x700, 36, TxData.asUint,5);
//	queue_insert(0x700, 36, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Temperatures[1]; CANTransmit(0x700, 37, TxData.asUint,5);
//	queue_insert(0x700, 37, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Temperatures[2]; CANTransmit(0x700, 38, TxData.asUint,5);
//	queue_insert(0x700, 38, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Temperatures[3]; CANTransmit(0x700, 39, TxData.asUint,5);
//	queue_insert(0x700, 39, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Temperatures[4]; CANTransmit(0x700, 40, TxData.asUint,5);
//	queue_insert(0x700, 40, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Temperatures[5]; CANTransmit(0x700, 41, TxData.asUint,5);
//	queue_insert(0x700, 41, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Temperatures[6]; CANTransmit(0x700, 42, TxData.asUint,5);
//	queue_insert(0x700, 42, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Temperatures[7]; CANTransmit(0x700, 43, TxData.asUint,5);
//	queue_insert(0x700, 43, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Temperatures[8]; CANTransmit(0x700, 44, TxData.asUint,5);
//	queue_insert(0x700, 44, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Temperatures[9]; CANTransmit(0x700, 45, TxData.asUint,5);
//	queue_insert(0x700, 45, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Temperatures[10]; CANTransmit(0x700, 46, TxData.asUint,5);
//	queue_insert(0x700, 46, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Temperatures[11]; CANTransmit(0x700, 47, TxData.asUint,5);
//	queue_insert(0x700, 47, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Temperatures[12]; CANTransmit(0x700, 48, TxData.asUint,5);
//	queue_insert(0x700, 48, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Temperatures[13]; CANTransmit(0x700, 49, TxData.asUint,5);
//	queue_insert(0x700, 49, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Temperatures[14]; CANTransmit(0x700, 50, TxData.asUint,5);
//	queue_insert(0x700, 50, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	TxData.asFloat=Temperatures[15]; CANTransmit(0x700, 51, TxData.asUint,5);
//	queue_insert(0x700, 51, TxData.asUint, 5, &CAN_queue);
	for(i=0;i<1500;i++){};
	CANTransmit(0x700, 52, BMS_Status,5);
	for(i=0;i<1500;i++){};
	//toets2 = ((int)(SOC*100)) & 0xFF;

	CANTransmit(0x718, 0x4, ((int)(Voltage_total*10)), 5); //Voltage
	for(i=0;i<1500;i++){};

//	queue_insert(0x718, 0x4, ((int)(Voltage_total*10))& 0xFFFF, 5, &CAN_queue);

	if(Charging_animation == 1)
		CANTransmit(0x718, 0x11, ((int)(Charging_Animation(SOC))), 5);
	else
		CANTransmit(0x718, 0x11, ((int)(SOC)), 5); //SOC
//	queue_insert(0x718, 0x11, ((int)(SOC*100)) & 0xFF, 5, &CAN_queue);
	for(i=0;i<1500;i++){};

	//sit system error, system charging, charge required
	//check flags for error messages

	Acewell_Data = ((Charger_status & 0x1)<<1);

	if (SOC<12)
		Acewell_Data = Acewell_Data + 1;

	if((flagDischarged >= 1) || (flagCurrent == 1)  || (flagTemp == 1))
		Acewell_Data = Acewell_Data + 4;

	CANTransmit(0x718, 0x88, Acewell_Data & 0xF, 5); //LEDS*/
	//queue_insert(0x718, 0x88, Acewell_Data & 0xF, 5, &CAN_queue);

}

void CANSlaveConfig(void)
{
	Uint32 RxDataNumber = 0;
	float RxDataValue = 0;
	union bits32 TxData;

	RxDataNumber = ECanaMboxes.MBOX3.MDH.all;             // Data taken out of direct mailbox
	TxData.asUint = ECanaMboxes.MBOX3.MDL.all;

	switch (RxDataNumber)
	{
	case 101: {if(TxData.asFloat<3.7 && TxData.asFloat>3.55){Vmax = TxData.asFloat;}break;}					//Vmax
	case 102: {if(RxDataValue<2.8 && RxDataValue>2.55){Vmin = RxDataValue;}break;}					//
	case 103: {if(RxDataValue<2.55 && RxDataValue>2.5){Vcritical = RxDataValue;}break;}
	case 104: {if(RxDataValue<3.65 && RxDataValue>3.4){Vcharge = RxDataValue;}break;}
	case 105: {if(RxDataValue<3.7 && RxDataValue>3.4){Vbalance = RxDataValue;}break;}

	case 106: {if(RxDataValue<4000 && RxDataValue>3000){Imax = RxDataValue;}break;}
	case 107: {if(RxDataValue<1000 && RxDataValue>100){Imin = RxDataValue;}break;}
	case 108: {if(RxDataValue<0.05 && RxDataValue>0.001){Ifilter = RxDataValue;}break;}			//   0.00314-1Hz     //  0.01249

	case 109: {if(RxDataValue<70 && RxDataValue>45){Tmax = RxDataValue;}break;}
	case 110: {if(RxDataValue<5&& RxDataValue>-5){Tmin = RxDataValue;}break;}

	case 111: {if(RxDataValue<12.8 && RxDataValue>12){Vauxmin = RxDataValue;}break;}
	case 112: {if(RxDataValue<5000 && RxDataValue>600){AuxChargeTime = RxDataValue;}break;}

	case 113: {if(RxDataValue<3.5 && RxDataValue>3.2){Vchargedflagreset = RxDataValue;}break;}
	case 114: {if(RxDataValue<3 && RxDataValue>2.6){Vdischargedflagreset = RxDataValue;}break;}

	//aux filter waarde

	//stroom meting offset waarde
	//stroom meting multiplier/skaal
	}
}

void CANTransmit(Uint16 Destination, Uint32 TxDataH, Uint32 TxDataL, Uint16 Bytes)      //destination, txdataH, txdataL,  bytes
{
	//struct ECAN_REGS ECanaShadow;

	if (ECanaRegs.CANES.all == 0 || ECanaRegs.CANES.all == 0x30000 || ECanaRegs.CANES.all == 0x30001)		//Maybe add more??     	CAN bus ready for action		/*|| ECanaRegs.CANES.all == 0x10000*/
	{
		//Start transmit
		ECanaRegs.CANME.all = 0x0000000E;                   	// Disable Tx Mailbox

		//Bytes
		ECanaMboxes.MBOX0.MSGCTRL.all = Bytes;              	// Transmit x bytes of data
		ECanaMboxes.MBOX0.MSGID.all = 0;                    	// Standard ID length, acceptance masks used, no remote frames
		ECanaMboxes.MBOX0.MSGID.bit.STDMSGID = Destination; 	// Load destination address   Destination

		ECanaMboxes.MBOX0.MDL.all = TxDataL;
		ECanaMboxes.MBOX0.MDH.all = TxDataH;

		ECanaRegs.CANME.all = 0x0000000F;                   	// Enable Tx Mailbox
		ECanaRegs.CANTRS.all = 0x00000001;                  	// Set transmit request
	}
	else 														//reset fault on CAN bus
	{
		ECanaRegs.CANES.all = 0xFFF0000;						//reset flags		0x1BB0000
		//add reset error warning, maybe to BMS flags
	}
/*	else if (ECanaRegs.CANES.bit.EP == 1)						//Warning, Error-passive state - Turn bus off
	{
		ECanaRegs.CANES.all = 0xFFFFFFFF;						//acknowledge all errors

		EALLOW;
		// Configure Master Control register
		ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
		ECanaShadow.CANMC.bit.PDR = 1; 							//request powerdown
		ECanaShadow.CANMC.bit.WUBA = 1;							//Bartho testing Wake up on bus activity..
		ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
		EDIS;
	}*/
}
