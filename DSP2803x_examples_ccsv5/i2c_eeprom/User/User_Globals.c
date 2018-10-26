/*
 * User_Globals.c
 *
 *  Created on: 04 May 2017
 *      Author: Sonja
 */
#include "User_Globals.h"
#include "User_Queue.h"


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

/////////////////////////////////////////////
volatile Uint16 flagTemp=0;
volatile Uint16 flagCurrent=0;
volatile Uint16 flagVoltage=0;

volatile Uint16 flagCharged=1;
volatile Uint16 flagDischarged=0;
volatile Uint16 balance = 0;
/////////////////////////////////////////////

volatile Uint16 counter_2Hz = 0;

float current_reference;

volatile Uint16 Charger_status = 0;
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
volatile float Temperature_avg;
volatile float Temperatures_resistance[14];
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
volatile float Vmin = 2.8;				//2.8
volatile float Vcritical = 2.7;		//2.7
volatile float Vcharge = 3.5;
volatile float Vbalance = 3.485; //4.8

volatile float Imin = 500;
volatile float Imax = 3800;
volatile float Ifilter = 0.27;

volatile float Tmax = 50;
volatile float Tmin = 0;

volatile float SOC;

volatile float Vauxmin = 12.3;
volatile float AuxChargeTime = 1800;

volatile float Vchargedflagreset = 3.35;
volatile float Vdischargedflagreset = 2.9;
volatile float kp_constant = 3.5;								//charger
volatile float kp_multiplier = 30;								//charger
volatile float balancing_upper_level = 3.49;
volatile float balancing_bottom_level = 3.48; //4.475

/* Declare variable using above structure and the function datapoints */
/* These coordinates correspond to the points illustrated in the above graph */
 float sine_x[5] = {0, 0.1, 0.3, 0.6, 1};
 float sine_y[5] = {2.88, 3.202, 3.27, 3.293, 3.349};

 struct table_1d sine_table = {
    5,      /* Number of data points */
    sine_x, /* Array of x-coordinates */
    sine_y  /* Array of y-coordinates */
};

 volatile float Current_CAL = 2080;			//=2095
 volatile Uint16 Current_Sum;
 volatile Uint16 Current_Counter;


 volatile Uint16 Toets_current_sum[10];


 float SOC_t;

 float Wsoc = 1;
 float SOCv = 0.5;
 float SOCc = 0;

 float ChgCurrent = 0;

 volatile float SOH_avg;
 volatile float SOH_max;
 volatile float SOH_max_cell;

 volatile float Voltages_old[15];
 volatile float Current_old;
 volatile float resistance;
 volatile float resistance_temp;
 volatile float dI;

struct queue_obj CAN_queue;					//initialise structure somewhere??

int queue_length = 0;

union bits32 TxData;

 int toets;
 int toets2;

 int temptimer = 0;
