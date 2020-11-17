/*
 * User_Globals.c
 *
 *  Created on: 04 May 2017
 *      Author: Sonja
 */
#include "User_Globals.h"
//#include "User_Queue.h"

// Two bytes will be used for the outgoing address,
// thus only setup 2 bytes maximum
struct I2CMSG I2cMsgOut1={I2C_MSGSTAT_SEND_WITHSTOP,
                          I2C_SLAVE_ADDR,
                          I2C_NUMBYTES,
                          I2C_EEPROM_HIGH_ADDR,
                          I2C_EEPROM_LOW_ADDR,
                          0x00,                   // Msg Byte 1
                          0x00};                  // Msg Byte 2


struct I2CMSG I2cMsgIn1={ I2C_MSGSTAT_SEND_NOSTOP,
                          I2C_SLAVE_ADDR,
                          I2C_NUMBYTES,
                          I2C_EEPROM_HIGH_ADDR,
                          I2C_EEPROM_LOW_ADDR};

struct I2CMSG *CurrentMsgPtr;               // Used in interrupts

volatile Uint16 DataOut;
volatile Uint16 DataOut2;
volatile Uint16 Received;

struct BMS_STATUS_FLAGS;

/////////////////////////////////////////////
volatile Uint16 flagTemp_Discharge=0;
volatile Uint16 flagTemp_Charge=0;

volatile Uint16 flagCurrent=0;
volatile Uint16 flagVoltage=0;

volatile Uint16 flagCharged=0;											//*****
volatile Uint16 flagDischarged=0;
volatile Uint16 balance = 0;
volatile Uint16 flag_Pre_Charge = 0;
/////////////////////////////////////////////

volatile Uint16 counter_2Hz = 0;

float current_reference;

volatile Uint16 Charger_status = 0;
volatile Uint16 Charging = 0;
volatile Uint16 Heating = 0;
volatile Uint16 Cooling = 0;

volatile int system_status= 0;

volatile float ChargerVoltage=0;
volatile float ChargerCurrent=0;
volatile Uint16 ChargerDebug=0;
volatile float Current;
volatile int current_int;

volatile float Voltages[15];
volatile float Voltage_avg;
volatile float Voltage_high=0;
volatile float Voltage_high_cell;
volatile float Voltage_low=10;
volatile float Voltage_low_cell;
volatile float Temperatures[16];

volatile float Temperatures_Module_1[2];
volatile float Temperatures_Module_2[2];
volatile float Temperatures_Module_3[2];
volatile float Temperatures_CS;
volatile float Temperatures_BMS;

volatile float Temperature_avg;
volatile float Temperatures_resistance[5];
volatile float Temperature_high=0;
volatile float Temperature_low=50;
volatile float Temperature_high_cell;
volatile float Temperature_low_cell;

volatile float ADCgain;
volatile float ADCoffset;
volatile float I;
volatile float Voltage_total;
volatile float Ah= 0;
volatile int rus = 0;

Uint32 ref = 0;

volatile int CANcounter = 0;

volatile float test_current= 0;

volatile float test_blah[3];

int Cell_B1 = 0;
int Cell_B2 = 0;
int Cell_B3 = 0;

Uint16 NodeID = 1;
volatile Uint16 counter_50Hz = 0;

volatile float Auxilliary_Voltage = 13;
volatile Uint16 Auxilliary_counter = 0;


//defines:
volatile float Vmax = 3.65;
volatile float Vmin = 2.75;				//2.8
volatile float Vcritical = 2.65;		//2.7
volatile float Vcharge = 3.5;
volatile float Vbalance = 3.485; //4.8

volatile int Imin = 500;
volatile int Imax = 4050;//4050
volatile float Ifilter = 0.27;

volatile float Tmax = 50;
volatile float Tmin = 0;

volatile float SOC =100 ;//

volatile float Vauxmin = 13;
volatile float AuxChargeTime = 1800;

volatile float Vchargedflagreset = 3.35;
volatile float Vdischargedflagreset = 2.8;
volatile float kp_constant = 3.5;								//charger
volatile float kp_multiplier = 30;								//charger
volatile float balancing_upper_level = 3.49;
volatile float balancing_bottom_level = 3.48; //4.475

/* Declare variable using above structure and the function datapoints */
/* These coordinates correspond to the points illustrated in the above graph */
/////////////////////////////////////////////////////////////////////////////////////////////////
long sine_x[5] = {0, 10, 30, 60, 100};
long sine_y[5] = {2880, 3202, 3270, 3293, 3340};	//3349

struct table_1d sine_table = {
    5,      /* Number of data points */
    sine_x, /* Array of x-coordinates */
    sine_y  /* Array of y-coordinates */
};

long time_x[4] = {1200000, 10000, 1000, 20};		//10ms, 0.5s, 5s, 600s							{20, 1000, 10000, 1200000}
long I_y[4] = {3031,3359 ,3686 ,4050};			//  4050 = 245A, 3686=200A, 3359=160A, 3032=120A	{4050, 3686, 3359, 3031}

struct table_1d trip_table = {
    4,      /* Number of data points */
	time_x, /* Array of x-coordinates */
	I_y  /* Array of y-coordinates */
};

long time2_x[2] = {300000, 1200000}; //samples-> 300s  , 600s
long I2_y[2] = {2048, 3031};

struct table_1d trip2_table = {
    2,      /* Number of data points */
	time2_x, /* Array of x-coordinates */
	I2_y  /* Array of y-coordinates */
};

long time3_x[4] = {4050,3686,3359,3031  };         //10ms, 0.5s, 5s, 600s                          {20, 1000, 10000, 1200000}


long I3_y[4] =      {20,1000,10000,1200000   };    //  4050 = 245A, 3686=200A, 3359=160A, 3032=120A    {4050, 3686, 3359, 3031}

struct table_1d trip3_table = {
    4,      /* Number of data points */
    time3_x, /* Array of x-coordinates */
    I3_y  /* Array of y-coordinates */
};

volatile float Current_CAL = 2080;			//=2095

volatile float SOC_t;

float volatile Wsoc = 1;
float volatile SOCv = 0.5;
float volatile SOCc = 0;

float ChgCurrent = 0;

volatile float SOH_avg;
volatile float SOH_max;
volatile float SOH_max_cell;

volatile float Voltages_old[15];
volatile float Current_old;
volatile float resistance;
volatile float resistance_temp;
volatile float dI;

union bits32 TxData;

int toets;
int toets2;

int temptimer = 0;

volatile Uint32 CAN_Charger_dataL = 0;
volatile Uint32 CAN_Charger_dataH = 0;


volatile Uint32 System_State = 0;

volatile Uint16 BMS_Status = 0;
volatile Uint16 BMS_Error = 0;
volatile long trip_counter = 0;
volatile Uint32 testcounter = 0;

volatile long testvariable = 0;
volatile long testvariable2 = 0;

volatile long testtrip = 0;

volatile long timecounter = 0;
volatile long timecounterseconds = 0;

volatile float Charging_animation = 0;

volatile Uint32 NMT_State = 0x0;               //Initialization state

volatile Uint16 SOP_discharge = 50;
volatile Uint16 SOP_charge = 25;

volatile float I_maximum = 0;
volatile float I_minimum = 0;
volatile float Initial_Capacity = 80;
volatile float Temperature_maximum = 0;
volatile float Temperature_minimum = 0;
volatile float Voltage_maximum = 0;
volatile float Voltage_minimum = 0;
volatile float SOH = 100;
volatile Uint16 Cycles = 0;

volatile int Pre_Charge_Measure = 0;
