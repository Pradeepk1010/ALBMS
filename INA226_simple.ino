//4XTemperature Sensor +2XINA237 CURRENT + ADS1115 BUS VOLTAGE +BMS CELL+SPEED+ DISPLAY
/*********
  ds18b20
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
  https://randomnerdtutorials.com/guide-for-ds18b20-temperature-sensor-with-arduino/
  Based on the Dallas Temperature Library example
*********/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <INA226.h>
#include <INA226S.h>
#include "ADS1X15.h"

#include <Arduino.h>
#include <stdint.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "UserInterface.h"

#include "LTC6806.h"
#include <SPI.h>
#include <SD.h>
#include <PWM.h>


#include <LiquidCrystal_I2C.h> //LCD
 #define L1_ORIGIN 0
 #define L2_ORIGIN 0
 #define L3_ORIGIN -4
 #define L4_ORIGIN -4
 #define DIPLAY_I2C_ADDR  0X27
 // Set the LCD address to 0x27 for a 16 chars and 4 line display
LiquidCrystal_I2C lcd(DIPLAY_I2C_ADDR, 16, 4);
float Speed=0.0;
float Current1=0.0;
float Current2=0.0;
float TemperaturesData[4];
float maxCellVal= -10.0;
float minCellVal=  10.0;
uint8_t MaxCellNum=0;
uint8_t MinCellNum=0;

/*Timer Related Parameters*/
#include <TimerOne.h>
uint32_t TdCounterms=0;
volatile bool FlagSDprinttd=false;
#define SDPrintTD_ms  10
volatile bool FlagLEDBlinktd = false;
#define LEDBlinkTD_ms  1
uint8_t FlagLedBlink=0;
#define DisplayPrint_ms  100
bool FlagDisplayPrint=false; 
/*Timer Related Parameters*/

/* Here are the variables for Initialization State*/
bool InitEnable=false; //This need to set true to Enter in Initialization loop
uint8_t Init_Step=0;  //State 0 is Ideal
int OVF_Status_Pin=0; //Status Of OVF pin will be read here
int IGNI_KET_Status_Pin=0; //Status of IGNITION KEY will be read here
bool InitializationDone=false;
//Enums for State
typedef enum {
        enIni_StateStartInitialization=0, //Step Id for Idle Situation--0
        enIni_StateTurnSWKeyOn,     //Turn On the SW Key -- 1
       // enIni_StateBallValveoff,  //Give a clock 500ms(Here)trigger here a separate thread
        enIni_StateAllPumpsON,      //Turn On All pumps -- 2
        enIni_StateCheckOVF,        //Check Overflow  -- 3
        enIni_StateTurnOnDCDCAux ,  //Turn DC-DC Auxiliary ON --4(A continuous High Signal is requiured)
        enIni_StateComplete,        // --5 , Here we can enter in to the Data Monitoring Sequence,
                                    //And Disable InitEnable to Stop , Set Init_Step to 0
}enInit_State;
/* Here are the variables for Initialization State*/

/* Here are the variables for ShutDown State*/
bool ShutEnable=false; //This need to set true to Enter in Initialization loop
uint8_t Shut_Step=0;  //State 0 is Ideal
//Enums for State
typedef enum {
        enShut_StateStartShutDown=0, //Step Id for Idle Situation--0
        enShut_StateAUXDCDCOff,      //Turn Off Auxiliary Supply --1
        enShut_StateMainPumpOff,     //Turn On All pumps -- 2
        enShut_SateTurnOnBallValve,  //Check Overflow  -- 3
        enShut_StateStoreData ,      //Store Data if need to store Something --4
        enShut_TurnSWKeyOFF,         //TurnOffSW Key --5
        enShut_StateComplete,        // --6 , Disable ShutEnable to Stop , Set Shut_Step to 0 
}enShut_State;
/* Here are the variables for ShutDown State*/


/* 
 *  Frequency Measurement Using Arduino
 *  Inpout is given to Pin 2
 *  Uses Interrupt to get pulse duration
 *  F = 1 / pulse duration
*/
#define MainPeriod 100 //Period of time for Speed Measurement

long previousMillis = 0; // will store last time of the cycle end
volatile unsigned long duration=0; // accumulates pulse width
volatile unsigned int pulsecount=0;
volatile unsigned long previousMicros=0;

/*PWM Parameters*/
int OutPWM1 = 11; //Timer 3
int OutPWM2 = 12; //Timer 4
//For testing
int freqInput = 100;          //10-10000 Hz (I wanted that range only)
int DUTY_PIN11_INIT_VAL = 0;      //1-65535
int DUTY_PIN12_INIT_VAL = 0;      //1-65535
uint8_t tempo=0;
/*PWM Parameters*/


// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
// MKRZero SD: SDCARD_SS_PIN
const int SDCS = 48;
 //CS for BMS Cell Monotoring is 53(Arduino Mega)
/*Define GPIO PINS*/
//PUMP GPIO ARDUINO MEGA
//PIN 2 IS DEDICATED TO SPEED MEASUREMENT
#define OUT_SW_KEY        4  //DO SW Key
#define IN_KEY_STATUS     5  //DI for KEY STATUS
#define IN_OVF            6  //DI For OVF STATUS
#define OUT_AUX_PIN      7   //DO For Auxialiary ON
#define PUMP5_GPIO        8  //DO/DI Spare 1 -->Initialized for SPEED TEST FOR TESTING ONLY
//#define PUMP6_GPIO   9  //DO/DI Spare 2
//#define PUMP7_GPIO   10 //DO/DI Spare 3
//#define PUMP8_GPIO   11  //Reserve for PWM Based O/P(pump)
//#define PUMP9_GPIO   A0  //DO/DI Spare 4
//#define PUMP10_GPIO  A1  //DO/DI Spare 5
//#define PUMP11_GPIO  42  //DO/DI Spare 6
/*Define GPIO PINS*/


#define ENABLED 1
#define DISABLED 0
#define CELL_CHANNELS 36

int8_t Error = 0; //Error Code For LTC6806
String dataStringFinal = "";
uint32_t DataCounter=0;
uint32_t FileCounterINT=0;
String FileCountStr="";
String LogFileNameStr="";
/**********************************************************
  Setup Variables
  The following variables can be modified to
  configure the software.

***********************************************************/
const uint8_t TOTAL_IC = 1;  //!<number of ICs in the daisy chain

//ADC Command Configurations
const uint8_t HIRNG = DISABLED;                  // See ltc6811_daisy.h for Options
const uint8_t ADC_CONVERSION_MODE = MD_NORMAL;   //MD_7KHZ_3KHZ; // See ltc6811_daisy.h for Options
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL;  // See ltc6811_daisy.h for Options
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL;    // See ltc6811_daisy.h for Options
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL;
const uint8_t ADOW_TIME = PRECHARGE_10MS;
const uint16_t MEASUREMENT_LOOP_TIME = 500;  //milliseconds(mS)

//Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 1333;  // Over voltage threshold ADC Code. LSB = 0.00015
const uint16_t UV_THRESHOLD = 133;   // Under voltage threshold ADC Code. LSB = 0.00015

//Loop Measurement Setup These Variables are ENABLED or DISABLED Remember ALL CAPS
const uint8_t WRITE_CONFIG = DISABLED;  // This is ENABLED or DISABLED
const uint8_t READ_CONFIG = DISABLED;   // This is ENABLED or DISABLED
const uint8_t MEASURE_CELL = ENABLED;   // This is ENABLED or DISABLED
const uint8_t MEASURE_AUX = DISABLED;   // This is ENABLED or DISABLED
const uint8_t RUN_OPENWIRE = DISABLED;  // This is ENABLED or DISABLED
/************************************
  END SETUP
*************************************/


/******************************************************
 *** Global Battery Variables received from 6806 commands
 These variables store the results from the LTC6806
 register reads and the array lengths must be based
 on the number of ICs on the stack
 ******************************************************/
int16_t cell_codes[TOTAL_IC][36];
/*!<
  The cell codes will be stored in the cell_codes[][12] array in the following format:

  |  cell_codes[0][0]| cell_codes[0][1] |  cell_codes[0][2]|    .....     |  cell_codes[0][11]|  cell_codes[1][0] | cell_codes[1][1]|  .....   |
  |------------------|------------------|------------------|--------------|-------------------|-------------------|-----------------|----------|
  |IC1 Cell 1        |IC1 Cell 2        |IC1 Cell 3        |    .....     |  IC1 Cell 12      |IC2 Cell 1         |IC2 Cell 2       | .....    |
****/

uint16_t aux_codes[TOTAL_IC][8];
/*!<
 The GPIO codes will be stored in the aux_codes[][8] array in the following format:

 |  aux_codes[0][0]| aux_codes[0][1] |  aux_codes[0][2]|  aux_codes[0][3]|  aux_codes[0][4]|  aux_codes[0][5]|  aux_codes[0][6]|  aux_codes[0][7]| aux_codes[1][0] |aux_codes[1][1]|  .....    |
 |-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|---------------|-----------|
 |IC1 Vref2        |IC1 GPIO1        |IC1 GPIO2        |IC1 GPIO3        |IC1 GPIO4        |IC1 GPIO5        |IC1 GPIO6        |IC1 RSVD         |IC2 Vref2        ||IC2 GPIO1     |  .....    |
*/

uint8_t tx_cfg[TOTAL_IC][6];
/*!<
  The tx_cfg[][6] stores the LTC6806 configuration data that is going to be written
  to the LTC6806 ICs on the daisy chain. The LTC6806 configuration data that will be
  written should be stored in blocks of 6 bytes. The array should have the following format:

 |  tx_cfg[0][0]| tx_cfg[0][1] |  tx_cfg[0][2]|  tx_cfg[0][3]|  tx_cfg[0][4]|  tx_cfg[0][5]| tx_cfg[1][0] |  tx_cfg[1][1]|  tx_cfg[1][2]|  .....    |
 |--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|-----------|
 |IC1 CFGR0     |IC1 CFGR1     |IC1 CFGR2     |IC1 CFGR3     |IC1 CFGR4     |IC1 CFGR5     |IC2 CFGR0     |IC2 CFGR1     | IC2 CFGR2    |  .....    |

*/

uint8_t rx_cfg[TOTAL_IC][8];
/*!<
  the rx_cfg[][8] array stores the data that is read back from a LTC6806-1 daisy chain.
  The configuration data for each IC  is stored in blocks of 8 bytes. Below is an table illustrating the array organization:

|rx_config[0][0]|rx_config[0][1]|rx_config[0][2]|rx_config[0][3]|rx_config[0][4]|rx_config[0][5]|rx_config[0][6]  |rx_config[0][7] |rx_config[1][0]|rx_config[1][1]|  .....    |
|---------------|---------------|---------------|---------------|---------------|---------------|-----------------|----------------|---------------|---------------|-----------|
|IC1 CFGR0      |IC1 CFGR1      |IC1 CFGR2      |IC1 CFGR3      |IC1 CFGR4      |IC1 CFGR5      |IC1 PEC High     |IC1 PEC Low     |IC2 CFGR0      |IC2 CFGR1      |  .....    |
*/

uint16_t stat_codes[TOTAL_IC][3];


uint8_t flags_uvov[TOTAL_IC][9];

uint8_t system_thsd[TOTAL_IC][1];

uint8_t system_muxfail[TOTAL_IC][1];

unsigned long system_open_wire[TOTAL_IC][2];




ADS1115 ADS(0x49); //BUS VOLTAGE READ ADDRESS 0x49
INA226 ina;
INA226S inas;
float INA_CurrentGain = 1.0;
float INA_CurrentOffset = 0;
float INAS_CurrentGain = 1.0;
float INAS_CurrentOffset = 0;

float Uncalvoltage;
float calvoltage;


// Data wire is conntec to the Arduino digital pin 4
#define ONE_WIRE_BUS 3 //for Temperature Read

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);
int deviceCount = 0;
float tempC;

void checkConfigs() {
  Serial.print("ModeS:                  ");
  switch (inas.getMode()) {
    case INA226S_MODE_POWER_DOWN: Serial.println("Power-Down"); break;
    case INA226S_MODE_SHUNT_TRIG: Serial.println("Shunt Voltage, Triggered"); break;
    case INA226S_MODE_BUS_TRIG: Serial.println("Bus Voltage, Triggered"); break;
    case INA226S_MODE_SHUNT_BUS_TRIG: Serial.println("Shunt and Bus, Triggered"); break;
    case INA226S_MODE_ADC_OFF: Serial.println("ADC Off"); break;
    case INA226S_MODE_SHUNT_CONT: Serial.println("Shunt Voltage, Continuous"); break;
    case INA226S_MODE_BUS_CONT: Serial.println("Bus Voltage, Continuous"); break;
    case INA226S_MODE_SHUNT_BUS_CONT: Serial.println("Shunt and Bus, Continuous"); break;
    default: Serial.println("unknown");
  }

  Serial.print("Samples averageS:       ");
  switch (inas.getAverages()) {
    case INA226S_AVERAGES_1: Serial.println("1 sample"); break;
    case INA226S_AVERAGES_4: Serial.println("4 samples"); break;
    case INA226S_AVERAGES_16: Serial.println("16 samples"); break;
    case INA226S_AVERAGES_64: Serial.println("64 samples"); break;
    case INA226S_AVERAGES_128: Serial.println("128 samples"); break;
    case INA226S_AVERAGES_256: Serial.println("256 samples"); break;
    case INA226S_AVERAGES_512: Serial.println("512 samples"); break;
    case INA226S_AVERAGES_1024: Serial.println("1024 samples"); break;
    default: Serial.println("unknown");
  }

  Serial.print("Bus conversion timeS:   ");
  switch (inas.getBusConversionTime()) {
    case INA226S_BUS_CONV_TIME_140US: Serial.println("140uS"); break;
    case INA226S_BUS_CONV_TIME_204US: Serial.println("204uS"); break;
    case INA226S_BUS_CONV_TIME_332US: Serial.println("332uS"); break;
    case INA226S_BUS_CONV_TIME_588US: Serial.println("558uS"); break;
    case INA226S_BUS_CONV_TIME_1100US: Serial.println("1.100ms"); break;
    case INA226S_BUS_CONV_TIME_2116US: Serial.println("2.116ms"); break;
    case INA226S_BUS_CONV_TIME_4156US: Serial.println("4.156ms"); break;
    case INA226S_BUS_CONV_TIME_8244US: Serial.println("8.244ms"); break;
    default: Serial.println("unknown");
  }

  Serial.print("Shunt conversion timeS: ");
  switch (inas.getShuntConversionTime()) {
    case INA226S_SHUNT_CONV_TIME_140US: Serial.println("140uS"); break;
    case INA226S_SHUNT_CONV_TIME_204US: Serial.println("204uS"); break;
    case INA226S_SHUNT_CONV_TIME_332US: Serial.println("332uS"); break;
    case INA226S_SHUNT_CONV_TIME_588US: Serial.println("558uS"); break;
    case INA226S_SHUNT_CONV_TIME_1100US: Serial.println("1.100ms"); break;
    case INA226S_SHUNT_CONV_TIME_2116US: Serial.println("2.116ms"); break;
    case INA226S_SHUNT_CONV_TIME_4156US: Serial.println("4.156ms"); break;
    case INA226S_SHUNT_CONV_TIME_8244US: Serial.println("8.244ms"); break;
    default: Serial.println("unknown");
  }

  Serial.print("Max possible currentS:  ");
  Serial.print(inas.getMaxPossibleCurrent());
  Serial.println(" A");

  Serial.print("Max currentS:           ");
  Serial.print(inas.getMaxCurrent());
  Serial.println(" A");

  Serial.print("Max shunt voltageS:     ");
  Serial.print(inas.getMaxShuntVoltage());
  Serial.println(" V");

  Serial.print("Max powerS:             ");
  Serial.print(inas.getMaxPower());
  Serial.println(" W");
}

void checkConfig() {
  Serial.print("Mode:                  ");
  switch (ina.getMode()) {
    case INA226_MODE_POWER_DOWN: Serial.println("Power-Down"); break;
    case INA226_MODE_SHUNT_TRIG: Serial.println("Shunt Voltage, Triggered"); break;
    case INA226_MODE_BUS_TRIG: Serial.println("Bus Voltage, Triggered"); break;
    case INA226_MODE_SHUNT_BUS_TRIG: Serial.println("Shunt and Bus, Triggered"); break;
    case INA226_MODE_ADC_OFF: Serial.println("ADC Off"); break;
    case INA226_MODE_SHUNT_CONT: Serial.println("Shunt Voltage, Continuous"); break;
    case INA226_MODE_BUS_CONT: Serial.println("Bus Voltage, Continuous"); break;
    case INA226_MODE_SHUNT_BUS_CONT: Serial.println("Shunt and Bus, Continuous"); break;
    default: Serial.println("unknown");
  }

  Serial.print("Samples average:       ");
  switch (ina.getAverages()) {
    case INA226_AVERAGES_1: Serial.println("1 sample"); break;
    case INA226_AVERAGES_4: Serial.println("4 samples"); break;
    case INA226_AVERAGES_16: Serial.println("16 samples"); break;
    case INA226_AVERAGES_64: Serial.println("64 samples"); break;
    case INA226_AVERAGES_128: Serial.println("128 samples"); break;
    case INA226_AVERAGES_256: Serial.println("256 samples"); break;
    case INA226_AVERAGES_512: Serial.println("512 samples"); break;
    case INA226_AVERAGES_1024: Serial.println("1024 samples"); break;
    default: Serial.println("unknown");
  }

  Serial.print("Bus conversion time:   ");
  switch (ina.getBusConversionTime()) {
    case INA226_BUS_CONV_TIME_140US: Serial.println("140uS"); break;
    case INA226_BUS_CONV_TIME_204US: Serial.println("204uS"); break;
    case INA226_BUS_CONV_TIME_332US: Serial.println("332uS"); break;
    case INA226_BUS_CONV_TIME_588US: Serial.println("558uS"); break;
    case INA226_BUS_CONV_TIME_1100US: Serial.println("1.100ms"); break;
    case INA226_BUS_CONV_TIME_2116US: Serial.println("2.116ms"); break;
    case INA226_BUS_CONV_TIME_4156US: Serial.println("4.156ms"); break;
    case INA226_BUS_CONV_TIME_8244US: Serial.println("8.244ms"); break;
    default: Serial.println("unknown");
  }

  Serial.print("Shunt conversion time: ");
  switch (ina.getShuntConversionTime()) {
    case INA226_SHUNT_CONV_TIME_140US: Serial.println("140uS"); break;
    case INA226_SHUNT_CONV_TIME_204US: Serial.println("204uS"); break;
    case INA226_SHUNT_CONV_TIME_332US: Serial.println("332uS"); break;
    case INA226_SHUNT_CONV_TIME_588US: Serial.println("558uS"); break;
    case INA226_SHUNT_CONV_TIME_1100US: Serial.println("1.100ms"); break;
    case INA226_SHUNT_CONV_TIME_2116US: Serial.println("2.116ms"); break;
    case INA226_SHUNT_CONV_TIME_4156US: Serial.println("4.156ms"); break;
    case INA226_SHUNT_CONV_TIME_8244US: Serial.println("8.244ms"); break;
    default: Serial.println("unknown");
  }

  Serial.print("Max possible current:  ");
  Serial.print(ina.getMaxPossibleCurrent());
  Serial.println(" A");

  Serial.print("Max current:           ");
  Serial.print(ina.getMaxCurrent());
  Serial.println(" A");

  Serial.print("Max shunt voltage:     ");
  Serial.print(ina.getMaxShuntVoltage());
  Serial.println(" V");

  Serial.print("Max power:             ");
  Serial.print(ina.getMaxPower());
  Serial.println(" W");
}
void setup(void) {
  // Start serial communication for debugging purposes
  Serial.begin(115200);
  //CONFIGURE DIGITAL PINS AS OUTPUT
   pinMode(OUT_SW_KEY,OUTPUT);pinMode(IN_KEY_STATUS,INPUT);pinMode(IN_OVF,INPUT);pinMode(OUT_AUX_PIN,OUTPUT);
   pinMode(PUMP5_GPIO,OUTPUT); 
   //pinMode(PUMP6_GPIO,OUTPUT);pinMode(PUMP7_GPIO,OUTPUT);pinMode(PUMP9_GPIO,OUTPUT);
   //pinMode(PUMP10_GPIO,OUTPUT);
  
   
  //CONFIGURE DIGITAL PINS AS OUTPUT
  //Serial.print(F_CPU / 1000000);
  Serial.print("Initializing SD card...");
  attachInterrupt(0, myinthandler, FALLING);
  // see if the card is present and can be initialized:
  if (!SD.begin(SDCS)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");

  lcd.begin();
  lcd.backlight();

  quikeval_SPI_connect();
  spi_enable(SPI_CLOCK_DIV16);  // This will set the Linduino to have a 1MHz Clock
  init_cfg();                   //initialize the 6806 configuration array to be written
  //pinMode(4, OUTPUT);
  //digitalWrite(4, HIGH);

//---------------Check if file Exist---------------------//
//Read File counter from File.
 File FileCounter = SD.open("COUNTER.txt");
// Check if the file is already available, if Yes read the file
  if (FileCounter) 
  {
    String CounterStr="";
     CounterStr = FileCounter.readStringUntil('\r');
     FileCounterINT = CounterStr.toInt();
     Serial.print("----Number of log files Saved(Existed): ");
     Serial.print(FileCounterINT , DEC);
     Serial.println("  ----");
    FileCounter.close();
    //Check File Counter Should be >0 && <100000
    if((FileCounterINT>=0)&&(FileCounterINT<9999999))
    {
     FileCounterINT++;
    }
    else
    {
      Serial.println("<- :(  -----File Counter Reset Done--- :( -->");
      FileCounterINT=1;
    }
    //Delete Existing File
    SD.remove("COUNTER.txt");
    //Save Updated Counter
    File FileCounter = SD.open("COUNTER.txt", FILE_WRITE);
    FileCountStr = String(FileCounterINT,DEC);
    if (FileCounter) {
    FileCounter.print(FileCountStr);
    FileCounter.print("\r");
    FileCounter.close();
    Serial.print("----Number of log files Saved(New): ");
    Serial.print(FileCounterINT,DEC);
    Serial.println("  ----");
  }
   // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening ||COUNTER.txt||");
   }
  }
  else
  { //if No Create a file Text as 1
    File FileCounter = SD.open("COUNTER.txt", FILE_WRITE);
    FileCountStr = String(1,DEC);
    if (FileCounter) {
    FileCounter.print(FileCountStr);
    FileCounter.print("\r");
    FileCounter.close();
    // print to the serial port too:
    FileCounterINT = 1;
    Serial.print("----Number of log files Saved(New): ");
    Serial.print(FileCounterINT,DEC);
    Serial.println("  ----");
  }
   // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening Counter.txt");
   }
  }
//---------------Check if file Exist---------------------//
  // Start up the library
  sensors.begin();
  // locate devices on the bus
  Serial.print("Locating Temperature devices...");
  Serial.print("Found ");
  deviceCount = sensors.getDeviceCount();
  Serial.print(deviceCount, DEC);
  Serial.println(" devices.");
  Serial.println("");


  Serial.println("Initialize INA226");
  Serial.println("-----------------------------------------------");

  // Default INA226 address is 0x45
  ina.begin();
  // Configure INA226
  ina.configure(INA226_AVERAGES_1, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
  ina.adc_config();
  // Calibrate INA226. Rshunt = 0.01 ohm, Max excepted current = 4A
  ina.calibrate(0.00075, 100);

  // Default INA226 address is 0x40
  inas.begin();
  // Configure INA226
  inas.configure(INA226S_AVERAGES_1, INA226S_BUS_CONV_TIME_1100US, INA226S_SHUNT_CONV_TIME_1100US, INA226S_MODE_SHUNT_BUS_CONT);
  inas.adc_config();
  // Calibrate INA226. Rshunt = 0.01 ohm, Max excepted current = 4A
  inas.calibrate(0.00075, 100);


  ADS.begin();
  ADS.setGain(0);      // 6.144 volt
  ADS.setDataRate(4);  // 0 = slow   4 = medium   7 = fast
  ADS.setMode(0);      // continuous mode
  ADS.readADC(0);      // first read to trigger
  ADS.requestADC_Differential_0_1();

  // Display configuration
  checkConfig();

  Timer1.initialize(10000); //Initialize timer with 1 second period
  Timer1.attachInterrupt(OnemsecFunction);

 InitTimersSafe();
 SetPinFrequencySafe(OutPWM1, freqInput );
 pwmWriteHR(OutPWM1, DUTY_PIN11_INIT_VAL );

/*Uncomment this Section to Init PWM @Pin 12*/
 //SetPinFrequencySafe(OutPWM2, freqInput ); 
 //pwmWriteHR(OutPWM2, DUTY_PIN12_INIT_VAL );
 /*Uncomment this Section to Init PWM @Pin 12*/
  Serial.println("-----------------------------------------------");
  Serial.println("--------Initialization Started------------");
  lcd.setCursor(L1_ORIGIN,0);
  lcd.print("                "); //Clear  Line 1
  lcd.setCursor(L1_ORIGIN,0);
  lcd.print("Initializing....");
  lcd.setCursor(L2_ORIGIN,1);
  lcd.print("                "); //Clear  Line 2
  lcd.setCursor(L3_ORIGIN,2);
  lcd.print("                "); //Clear  Line 3
  lcd.setCursor(L4_ORIGIN,3);
  lcd.print("                "); //Clear  Line 4
  InitEnable=true;
  Init_Step=0;
}

void loop(void) {

unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= MainPeriod) 
  {
    previousMillis = currentMillis;   
    // need to bufferize to avoid glitches
    unsigned long _duration = duration;
    unsigned long _pulsecount = pulsecount;
    duration = 0; // clear counters
    pulsecount = 0;
    float Freq = 1e6 / float(_duration);    //Duration is in uSecond so it is 1e6 / T

    Freq *= _pulsecount; // calculate F
    // output time and frequency data to RS232
    //Serial.print("Frequency: ");
    //Serial.print(Freq);
    //Serial.print("Hz     "); 
    Speed = (Freq/5.75);
    if((Speed>=0.0)&&(Speed<=80.0)){}
    else Speed=0;
   // Serial.print(Speed, 0);
   // Serial.println("  KM/h"); 
  }

  /*Check Digital Inputs*/
   OVF_Status_Pin = digitalRead(IN_OVF); //Read Overflow Status
   IGNI_KET_Status_Pin = digitalRead(IN_KEY_STATUS); //Read Ignition Status
   if((IGNI_KET_Status_Pin==0)&&(ShutEnable==false)&&(InitializationDone==true))
   {
     //Start Finishing and System Shut Down

     ShutEnable = true;
     Shut_Step=0;
  lcd.setCursor(L1_ORIGIN,0);
  lcd.print("                "); //Clear  Line 1
  lcd.setCursor(L1_ORIGIN,0);
  lcd.print("Shutting Down...");
  lcd.setCursor(L2_ORIGIN,1);
  lcd.print("                "); //Clear  Line 2
  lcd.setCursor(L3_ORIGIN,2);
  lcd.print("                "); //Clear  Line 3
  lcd.setCursor(L4_ORIGIN,3);
  lcd.print("                "); //Clear  Line 4
   }
   /*Check Digital Inputs*/

 if(FlagSDprinttd&&InitializationDone&&(ShutEnable==false))
  {
  FlagSDprinttd=false;
  sensors.requestTemperatures();
  /*Print Total Current*/
 // Serial.print("#  ");
 // Serial.print("TCURR01:");
  Current1 = ina.readCalibratedCurrent(INA_CurrentGain, INA_CurrentOffset);
  dataStringFinal = String(Current1, 5);
  dataStringFinal +=String(",");
  //Serial.print(ina.readCalibratedCurrent(INA_CurrentGain, INA_CurrentOffset), 5);
 // Serial.print(",");

  Current2 = inas.readCalibratedCurrent(INAS_CurrentGain, INAS_CurrentOffset);
  dataStringFinal += String(Current2, 5);
  dataStringFinal +=String(",");
 // Serial.print("TCURR02:");
  //Serial.print(inas.readCalibratedCurrent(INAS_CurrentGain, INAS_CurrentOffset), 5);
  //Serial.print(",");
  /*Print Total Current*/


  /*Print ADS1115 Voltage*/
  Uncalvoltage = ((ADS.getValue() * 0.1875) / 100);
  // Serial.print("Uncal Volt: ");
  // Serial.println(Uncalvoltage , 5);
  calvoltage = ((Uncalvoltage * 1.341273) + 0.044422);
  //Serial.print("BVOLT:");
  //Serial.print(calvoltage, 5);
  //Serial.print(",");
  dataStringFinal +=String(calvoltage, 5);
  dataStringFinal +=String(",");
  /*Print ADS1115 Voltage*/

  /*Print Temperature Data */
  // Call sensors.requestTemperatures() to issue a global temperature and Requests to all devices on the bus
  //Serial.print("TMP01:");
  // Why "byIndex"? You can have more than one IC on the same bus. 0 refers to the first IC on the wire
  //Serial.print(sensors.getTempCByIndex(0));
  //Serial.println("  $");
  //Serial.print(" - Fahrenheit temperature: ");
  //Serial.println(sensors.getTempFByIndex(0));


  for (int i = 0; i < deviceCount; i++) {
   // Serial.print("TEMP");
   // Serial.print(i + 1);
    //Serial.print(":");
    tempC = sensors.getTempCByIndex(i);
    TemperaturesData[i]= tempC;
    //Serial.print(tempC);
    //Serial.print(",");

    dataStringFinal +=String(tempC);
    dataStringFinal +=String(",");


    /*
    Serial.print((char)176);//shows degrees character
    Serial.print("C  |  ");
    Serial.print(DallasTemperature::toFahrenheit(tempC));
    Serial.print((char)176);//shows degrees character
    Serial.print("F");
    */
  }
  //Serial.println("  $");
  /*Print Temperature Data */
  wakeup_idle(TOTAL_IC);
  LTC6806_adcv(ADC_CONVERSION_MODE, CELL_CH_TO_CONVERT);
  delay(10);
  wakeup_idle(TOTAL_IC);
  Error = LTC6806_rdcv(0, TOTAL_IC, cell_codes);
  check_error(Error);
  //Serial.print("&  ");
  print_cells();
  dataStringFinal += String(Speed);
  //Serial.println("  &&");
  LogFileNameStr  = "D"; //Filename Not exceed 8 Characters(Limitations of concept of 8.3 file names )
  LogFileNameStr +=String(FileCounterINT,DEC);  
  LogFileNameStr +=".txt";
  //Serial.print("File Name--->  ");
  //Serial.println(LogFileNameStr);
  File dataFile = SD.open(LogFileNameStr, FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataStringFinal);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataStringFinal);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.print("error opening  ");
    Serial.println(LogFileNameStr);
  }
  DataCounter++;
  //Serial.print("Data Counter: ");
  //Serial.println(DataCounter);
  if(DataCounter>10000)
  { 
    SD.remove("COUNTER.txt");
    FileCounterINT++;
    DataCounter=0;
    File FileCounter1 = SD.open("COUNTER.txt", FILE_WRITE);
    FileCountStr = String(FileCounterINT,DEC);
    if (FileCounter1) {
    FileCounter1.print(FileCountStr);
    FileCounter1.print("\r");
    FileCounter1.close();
    //Serial.print("----Number of log files Saved(New): ");
    //Serial.print(FileCounterINT,DEC);
    //Serial.println("  ----");
    }
  }
    
   // UpdateDuty_Pin11(tempo);
   // tempo += 2;
   // if(tempo>100)tempo=0;
    
 }
 if(FlagDisplayPrint&&InitializationDone&&(ShutEnable==false))
 {
    FlagDisplayPrint=false;

    //Line 1 Print
    lcd.setCursor(L1_ORIGIN,0);
    lcd.print("                "); //Clear  Line 1
    lcd.setCursor(L1_ORIGIN,0);
    lcd.print(Speed, 0);
    lcd.print("KM/H");
    lcd.setCursor(L1_ORIGIN+9,0);
    lcd.print("BV:");
    lcd.print(calvoltage, 1);
    //Line 2 Print
    lcd.setCursor(L2_ORIGIN,1);
    lcd.print("                "); //Clear  Line 2
    lcd.setCursor(L2_ORIGIN,1);
    lcd.print("A:");
    lcd.print(Current1, 1);
    //lcd.print(-999.9, 1);
    lcd.setCursor(L2_ORIGIN+8,1);
    lcd.print("B:");
    lcd.print(Current2, 1);
    //lcd.print(-999.9, 1);
    //Line 3 Print
    lcd.setCursor(L3_ORIGIN,2);
    lcd.print("                "); //Clear  Line 3
    lcd.setCursor(L3_ORIGIN,2);
    lcd.print("CX");
    lcd.print(MaxCellNum);
    lcd.print(":");
    lcd.print(maxCellVal,2);
    lcd.setCursor(L3_ORIGIN+10,2);
    lcd.print("T1:");
    lcd.print(TemperaturesData[0], 0);
    //Line 4 Print
    lcd.setCursor(L4_ORIGIN,3);
    lcd.print("                "); //Clear  Line 3
    lcd.setCursor(L4_ORIGIN,3);
    lcd.print("CN");
    lcd.print(MinCellNum);
    lcd.print(":");
    lcd.print(minCellVal,2);
    lcd.setCursor(L4_ORIGIN+10,3);
    lcd.print("T2:");
    lcd.print(TemperaturesData[1], 0);
    maxCellVal= -10.0;
    minCellVal = 10.0;
 }
}
/*!***********************************
 \brief Initializes the configuration array
 **************************************/
void init_cfg() {
  for (int i = 0; i < TOTAL_IC; i++) {
    tx_cfg[i][0] = 0xFF;
    tx_cfg[i][1] = 0x40 | (HIRNG << 7) | ADOW_TIME;  //was 0x60
    tx_cfg[i][2] = 0x00;
    tx_cfg[i][3] = 0x00;
    tx_cfg[i][4] = 0x00;
    tx_cfg[i][5] = 0x00;
  }
}

/*!*********************************
  \brief Prints the main menu
***********************************/
void print_menu() {
  Serial.println(F("Please enter LTC6806 Command"));
  Serial.println(F("Write Configuration: 1            | Change Written Configuration: 10"));
  Serial.println(F("Read Configuration: 2             | Run Open Wire Test: 11"));
  Serial.println(F("Start Cell Voltage Conversion: 3  | Start CVST: 12"));
  Serial.println(F("Read Cell Voltages: 4             | Print Cells Hex: 13"));
  Serial.println(F("Start Aux Voltage Conversion: 5   | Set UV OV Thresholds: 14"));
  Serial.println(F("Read Aux Voltages: 6              | Start Continuous Monitor Mode: 15"));
  Serial.println(F("Start Stat Voltage Conversion: 7  | Print menu: 'm'"));
  Serial.println(F("Read Stat Register Data: 8        | "));
  Serial.println(F("loop cell voltages: 9             | "));
  Serial.println(F("                                 | "));
  Serial.println();
  Serial.println(F("Please enter command: "));
  Serial.println();
}

void continuous_monitor_mode() {
  byte mmd = 0;
  byte channels = 0;
  char input = 0;
  Serial.print(F("Select Cycle Time: \n 11.2mS : 1 \n 16.4mS : 2 \n 47.9mS : 3 \n"));
  mmd = (byte)read_int();
  Serial.println(mmd, DEC);
  Serial.print(F("Enter the Number of the First Connected C pin: "));
  channels = (byte)read_int();
  Serial.println(channels, DEC);
  for (int ic = 0; ic < TOTAL_IC; ic++) {
    tx_cfg[ic][2] = ((mmd & 0x3) << 6) | (channels & 0x37);
  }

  Serial.println(F("transmit 'm' to quit"));
  wakeup_sleep(TOTAL_IC);
  LTC6806_wrcfg(TOTAL_IC, tx_cfg);
  while (input != 'm') {
    if (Serial.available() > 0) {
      input = read_char();
    }
    delay(500);
  }
  for (int ic = 0; ic < TOTAL_IC; ic++) {
    tx_cfg[ic][2] = 0;
  }
  wakeup_sleep(TOTAL_IC);
  LTC6806_wrcfg(TOTAL_IC, tx_cfg);
  print_menu();
}

void set_uv_ov_thresholds() {
  float uv_threshold = 0;
  uint16_t uv_code = 0;
  float ov_threshold = 0;
  uint16_t ov_code = 0;

  Serial.print(F("Enter Under Voltage threshold: "));
  uv_threshold = read_float();
  Serial.println(uv_threshold, DEC);
  uv_code = int(uv_threshold / 0.0015);
  Serial.print("Written UV code: ");
  Serial.println(uv_code, DEC);

  Serial.print(F("Enter Over Voltage threshold: "));
  ov_threshold = read_float();
  Serial.println(ov_threshold, DEC);
  ov_code = int(ov_threshold / 0.0015);
  Serial.print("Written OV code: ");
  Serial.println(ov_code, DEC);
  for (int ic = 0; ic < TOTAL_IC; ic++) {
    tx_cfg[ic][3] = (uv_code >> 4) & 0xFF;
    tx_cfg[ic][4] = ((uv_code & 0x0F) << 4) | ((ov_code >> 8) & 0x0F);
    tx_cfg[ic][5] = ov_code & 0x00FF;
  }
}

/*!************************************************************
  \brief Prints cell coltage codes to the serial port
 *************************************************************/
void print_cells()
{

  float lsb;
  String dataString = ""; //Stores The complete String Here

  if (HIRNG == ENABLED)lsb = 0.003;
  else lsb =0.0015;

  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
   // Serial.print(F("IC"));
    //dataString += String(current_ic+1,DEC);
    
    //Serial.print(current_ic+1,DEC);
   // Serial.print(dataString);
    for (int i=0; i<36; i++)
    {
     // dataString += String(F(" C"));
      //Serial.print(F(" C"));
     // dataString += String(i+1,DEC);
      //Serial.print(i+1,DEC);
     // dataString += String(F(":"));
      //Serial.print(F(":"));
      dataString += String(cell_codes[current_ic][i]*lsb,4);
      //Serial.print(cell_codes[current_ic][i]*lsb,4);//*lsb,4);//
      dataString += String(F(","));
      //Serial.print(F(","));
     // if ((i%12==0)&&( i!= 0))
     // {
        //dataString += String("\n"); //@@@
        //Serial.println();
       // dataString += String("    ");
        //Serial.print("     ");
    //  }
    /*
     Serial.print("Index:");
     Serial.print(i);
     Serial.print("  ");
     Serial.println(cell_codes[current_ic][i]*0.0015,5);
     */
     if(maxCellVal < String(cell_codes[current_ic][i]*lsb,4).toFloat())
     {
      maxCellVal  = String(cell_codes[current_ic][i]*lsb,4).toFloat();
      MaxCellNum= (i+1);
     }
     if(minCellVal > String(cell_codes[current_ic][i]*lsb,4).toFloat())
     {
        minCellVal  = String(cell_codes[current_ic][i]*lsb,4).toFloat();
        MinCellNum= (i+1);
     }

    }
    //dataString += String("\n");
    //Serial.println();
  }
  //dataString += String("\n"); //@@@
  //Serial.println();
  dataStringFinal += dataString;
  //Serial.print(dataStringFinal);
}

void print_cells_hex() {

  float lsb;

  if (HIRNG == 1) lsb = 0.003;
  else lsb = 0.0015;

  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(F(" IC "));
    Serial.print(current_ic + 1, DEC);
    for (int i = 0; i < 36; i++) {
      Serial.print(F(" C"));
      Serial.print(i + 1, DEC);
      Serial.print(F(":"));
      Serial.print(cell_codes[current_ic][i], HEX);  //*lsb,4);//
      Serial.print(F(","));
      if (i % 12 == 0) {
        Serial.println();
      }
    }
    Serial.println();
  }
  Serial.println();
}

/*!****************************************************************************
  \brief Prints GPIO voltage codes and Vref2 voltage code onto the serial port
 *****************************************************************************/
void print_aux() {

  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(F(" IC "));
    Serial.print(current_ic + 1, DEC);
    for (int i = 1; i < 7; i++) {
      Serial.print(F(" GPIO-"));
      Serial.print(i, DEC);
      Serial.print(F(":"));
      Serial.print(aux_codes[current_ic][i] * 0.0015, 4);
      Serial.print(F(","));
    }
    Serial.print(F(" Vref2"));
    Serial.print(F(":"));
    Serial.print(aux_codes[current_ic][0] * 0.0015, 4);
    Serial.println();
  }
  Serial.println();
}

/*!****************************************************************************
  \brief Prints Status voltage codes and Vref2 voltage code onto the serial port
 *****************************************************************************/
void print_stat() {

  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(F(" IC "));
    Serial.print(current_ic + 1, DEC);
    Serial.print(F(" SOC:"));
    Serial.print(stat_codes[current_ic][0] * 0.0015 * 72, 4);
    Serial.print(F(","));
    Serial.print(F(" Itemp:"));
    Serial.print(stat_codes[current_ic][1] * 0.1875, 4);
    Serial.print(F(","));
    Serial.print(F(" V5:"));
    Serial.print(stat_codes[current_ic][2] * 0.001875, 4);
  }

  Serial.println();
}

/*!******************************************************************************
 \brief Prints the configuration data that is going to be written to the LTC6806
 to the serial port.
 ********************************************************************************/
void print_config() {
  int cfg_pec;
  Serial.println(F("Written Configuration: "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(F(" IC "));
    Serial.print(current_ic + 1, DEC);
    Serial.print(F(": "));
    Serial.print(F("0x"));
    serial_print_hex(tx_cfg[current_ic][0]);
    Serial.print(F(", 0x"));
    serial_print_hex(tx_cfg[current_ic][1]);
    Serial.print(F(", 0x"));
    serial_print_hex(tx_cfg[current_ic][2]);
    Serial.print(F(", 0x"));
    serial_print_hex(tx_cfg[current_ic][3]);
    Serial.print(F(", 0x"));
    serial_print_hex(tx_cfg[current_ic][4]);
    Serial.print(F(", 0x"));
    serial_print_hex(tx_cfg[current_ic][5]);
    Serial.print(F(", Calculated PEC: 0x"));
    cfg_pec = pec15_calc(6, &tx_cfg[current_ic][0]);
    serial_print_hex((uint8_t)(cfg_pec >> 8));
    Serial.print(F(", 0x"));
    serial_print_hex((uint8_t)(cfg_pec));
    Serial.println();
  }
  Serial.println();
}
/*!*****************************************************************
 \brief Prints the configuration data that was read back from the
 LTC6806 to the serial port.
 *******************************************************************/
void print_rxconfig() {
  Serial.println(F("Received Configuration "));
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    Serial.print(F(" IC "));
    Serial.print(current_ic + 1, DEC);
    Serial.print(F(": 0x"));
    serial_print_hex(rx_cfg[current_ic][0]);
    Serial.print(F(", 0x"));
    serial_print_hex(rx_cfg[current_ic][1]);
    Serial.print(F(", 0x"));
    serial_print_hex(rx_cfg[current_ic][2]);
    Serial.print(F(", 0x"));
    serial_print_hex(rx_cfg[current_ic][3]);
    Serial.print(F(", 0x"));
    serial_print_hex(rx_cfg[current_ic][4]);
    Serial.print(F(", 0x"));
    serial_print_hex(rx_cfg[current_ic][5]);
    Serial.print(F(", Received PEC: 0x"));
    serial_print_hex(rx_cfg[current_ic][6]);
    Serial.print(F(", 0x"));
    serial_print_hex(rx_cfg[current_ic][7]);
    Serial.println();
  }
  Serial.println();
}

void print_open() {
  for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
    if ((system_open_wire[current_ic][0] == 0) && (system_open_wire[current_ic][1] == 0)) {
      Serial.print("No Opens Detected on IC: ");
      Serial.print(current_ic + 1, DEC);
      Serial.println();
    }
    for (int cell = 0; cell < 32; cell++) {
      if ((system_open_wire[current_ic][0] & (1L << cell)) > 0) {
        Serial.print(F("There is an open wire on channel: "));
        Serial.println(cell, DEC);
      }
    }

    for (int cell = 32; cell <= 36; cell++) {
      if ((system_open_wire[current_ic][1] & (1L << (cell - 32))) > 0) {
        Serial.print(F("There is an open wire on channel: "));
        Serial.println(cell, DEC);
      }
    }
  }
}
void serial_print_hex(uint8_t data) {
  if (data < 16) {
    Serial.print("0");
    Serial.print((byte)data, HEX);
  } else
    Serial.print((byte)data, HEX);
}
void run_openwire(unsigned long open_wire_result[][2]) {
  int16_t OPENWIRE_THRESHOLD = -133;  //133;
  const uint8_t N_CHANNELS = CELL_CHANNELS;

  int16_t pullUp_cell_codes[TOTAL_IC][N_CHANNELS];
  int16_t pullDwn_cell_codes[TOTAL_IC][N_CHANNELS];
  int16_t openWire_delta[TOTAL_IC][N_CHANNELS];
  //long open_wire_result[TOTAL_IC];
  int8_t error;


  wakeup_sleep(TOTAL_IC);
  digitalWrite(4, LOW);
  LTC6806_adow(MD_NORMAL, PULL_UP_CURRENT, CELL_CH_ALL);
  delay(20);  //20
  digitalWrite(4, HIGH);
  wakeup_idle(TOTAL_IC);
  error = LTC6806_rdcv(0, TOTAL_IC, pullUp_cell_codes);  // Set to read back all cell voltage registers
  if (error == -1) {
    Serial.println(F("A PEC error was detected in the received data"));
  }

  /* Serial.println(F("pullup readings"));
  for(int cell=0;cell<N_CHANNELS;cell++)
   {
      Serial.println(pullUp_cell_codes[0][cell],DEC);
   }*/

  wakeup_sleep(TOTAL_IC);
  digitalWrite(4, LOW);
  LTC6806_adow(MD_NORMAL, PULL_DOWN_CURRENT, CELL_CH_ALL);
  delay(20);  //20
  digitalWrite(4, HIGH);
  wakeup_idle(TOTAL_IC);
  error = LTC6806_rdcv(0, TOTAL_IC, pullDwn_cell_codes);  // Set to read back all cell voltage registers
  /* Serial.println(F("pulldown readings"));
   for(int cell=0;cell<N_CHANNELS;cell++)
     {
        Serial.println(pullDwn_cell_codes[0][cell],DEC);
     }*/
  if (error == -1) {
    Serial.println(F("A PEC error was detected in the received data"));
  }

  for (int ic = 0; ic < TOTAL_IC; ic++) {
    open_wire_result[ic][0] = 0;
    open_wire_result[ic][1] = 0;
    for (int cell = 0; cell < N_CHANNELS; cell++) {
      openWire_delta[ic][cell] = pullUp_cell_codes[ic][cell] - pullDwn_cell_codes[ic][cell];
    }
    /* Serial.println(F("open wire delta"));
    for(int cell=0;cell<N_CHANNELS;cell++)
     {
        Serial.println(openWire_delta[0][cell],DEC);
     }*/
  }
  for (int ic = 0; ic < TOTAL_IC; ic++) {
    for (int cell = 0; cell < N_CHANNELS - 4; cell++) {

      if (openWire_delta[ic][cell + 1] < OPENWIRE_THRESHOLD) {
        open_wire_result[ic][0] |= (1L << cell + 1);
      }

      if ((pullUp_cell_codes[ic][cell + 1] > 1666) && (pullUp_cell_codes[ic][cell] < -666)) {
        open_wire_result[ic][0] |= (1L << cell + 1);
      }
    }
    for (int cell = N_CHANNELS - 5; cell < N_CHANNELS; cell++) {

      if (openWire_delta[ic][cell] < OPENWIRE_THRESHOLD) {
        open_wire_result[ic][1] |= (1L << (cell - 31));
      }
      if ((pullUp_cell_codes[ic][cell + 1] > 1666) && (pullUp_cell_codes[ic][cell] < -666)) {
        open_wire_result[ic][1] |= (1L << (cell - 31));
      }
    }
    if (pullUp_cell_codes[ic][0] < -1500) {
      open_wire_result[ic][0] |= 1L;
    }
    if (pullUp_cell_codes[ic][N_CHANNELS - 1] < -1500) {
      open_wire_result[ic][1] |= (1L << (N_CHANNELS - 32));
    }
  }
}


//Function to check error flag and print PEC error message
void check_error(int error) {
  if (error == -1) {
    Serial.println(F("A PEC error was detected in the received data"));
  }
}


// hex conversion constants
char hex_digits[16] = {
  '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
};

// global variables

char hex_to_byte_buffer[5] = {
  '0', 'x', '0', '0', '\0'
};  // buffer for ASCII hex to byte conversion
char byte_to_hex_buffer[3] = {
  '\0', '\0', '\0'
};
void read_config_data(uint8_t cfg_data[][6], uint8_t nIC) {
  //uint8_t cfg_data[6];
  Serial.println("Please enter two HEX characters for each configuration byte ");
  Serial.print("Enter Hex for CFGR0: ");
  cfg_data[nIC][0] = read_hex();
  Serial.print(cfg_data[nIC][0], HEX);
  Serial.println();
  Serial.print("Enter Hex for CFGR1: ");
  cfg_data[nIC][1] = read_hex();
  Serial.print(cfg_data[nIC][1], HEX);
  Serial.println();
  Serial.print("Enter Hex for CFGR2: ");
  cfg_data[nIC][2] = read_hex();
  Serial.print(cfg_data[nIC][2], HEX);
  Serial.println();
  Serial.print("Enter Hex for CFGR3: ");
  cfg_data[nIC][3] = read_hex();
  Serial.print(cfg_data[nIC][3], HEX);
  Serial.println();
  Serial.print("Enter Hex for CFGR4: ");
  cfg_data[nIC][4] = read_hex();
  Serial.print(cfg_data[nIC][4], HEX);
  Serial.println();
  Serial.print("Enter Hex for CFGR5: ");
  cfg_data[nIC][5] = read_hex();
  Serial.print(cfg_data[nIC][5], HEX);
  Serial.println();
  Serial.println("Configuration Received");
  Serial.println();
}
char read_hex()
// read 2 hex characters from the serial buffer and convert
// them to a byte
{
  byte data;
  hex_to_byte_buffer[2] = get_char();
  hex_to_byte_buffer[3] = get_char();
  get_char();
  get_char();
  data = strtol(hex_to_byte_buffer, NULL, 0);
  return (data);
}
char get_char()
// get the next character either from the serial port
// or the recording buffer
{

  // read a command from the serial port
  while (Serial.available() <= 0)
    ;
  return (Serial.read());
}

void OnemsecFunction(void){
  TdCounterms++;
  if((TdCounterms%SDPrintTD_ms)==0)
  {
    FlagSDprinttd=true;
    System_Initialization();
    System_ShutDown();
  }


  if((TdCounterms%LEDBlinkTD_ms)==0)
  {
    if(FlagLedBlink==0)
    {
     digitalWrite(PUMP5_GPIO,HIGH);
     FlagLedBlink=1;
    }
    else
    {
     digitalWrite(PUMP5_GPIO,LOW);
     FlagLedBlink=0;
    }
  }
  
  if((TdCounterms%DisplayPrint_ms)==0)
  {
    FlagDisplayPrint=true;
  }

  if(TdCounterms>=20000)TdCounterms=0;
}
void myinthandler() // interrupt handler
{
  unsigned long currentMicros = micros();
  duration += currentMicros - previousMicros;
  previousMicros = currentMicros;
  pulsecount++;
}
//Enter Duty In Percentage
void UpdateDuty_Pin11(uint8_t DUTY_PIN11_PERCENT)
{
  int DUTYVALUE= ((DUTY_PIN11_PERCENT*65535)/100);
  if(DUTYVALUE>65535)DUTYVALUE=65535;
  //Min 0  , Max 65535
 //Calculate Value from Percentage
  pwmWriteHR(OutPWM1, DUTYVALUE);
}

/*This is the function We need to call in while loop
*This function works for Initialization Loop
*Pass No value
*/
void System_Initialization(void)
{
 if(InitEnable==false)return;
 if(InitializationDone==true)return;
 if(Init_Step== enIni_StateStartInitialization)
 {
    Init_Step = enIni_StateTurnSWKeyOn;
    Serial.println("Initialization Started...Turning ON SW Key ||");
 }
 else if(Init_Step == enIni_StateTurnSWKeyOn)
 {
    digitalWrite(OUT_SW_KEY,HIGH);
    Init_Step = enIni_StateAllPumpsON;
    Serial.println("SW Key Turned ON...Turning ON All Pumps ||");
 }
 else if(Init_Step == enIni_StateAllPumpsON)
 {
    UpdateDuty_Pin11(100);
    Init_Step = enIni_StateCheckOVF;
    Serial.println("All Pumps Turned ON...Checking Over Flow ||");
 }
 else if(Init_Step == enIni_StateCheckOVF)
 {
   if(OVF_Status_Pin == 1)
   {
     Serial.println("Waiting For OverFlow to Happen...");
      //Do Nothing And Wait
   }
   else
   {
  //if Overflow Occur , Else Wait
    Init_Step = enIni_StateTurnOnDCDCAux;
    Serial.println("OverFlow Detected...Turning ON Auxiliary ||");
   }
 }
 else if(Init_Step == enIni_StateTurnOnDCDCAux)
 {
    digitalWrite(OUT_SW_KEY,HIGH);
    Init_Step = enIni_StateComplete;
    Serial.println("Auxiliary DC-DC ON...Finishing Initialization ||");
 }
 else if(Init_Step == enIni_StateComplete)
 {
     InitEnable = false;
     Init_Step = enIni_StateStartInitialization;
     InitializationDone=true;
     Serial.println("Initialization Complete...Done! ||");
 }
}

/*This is the function We need to call in while loop
*This function works for Shut Down and finishing System
*Pass No value
*/
void System_ShutDown(void)
{
 if(ShutEnable==false)return;
 if(Shut_Step== enShut_StateStartShutDown)
 {
    Shut_Step = enShut_StateAUXDCDCOff;
    Serial.println("Shut Down Started...Turning OFF DC-DC AUX||");
 }
 else if(Shut_Step == enShut_StateAUXDCDCOff)
 {
    digitalWrite(OUT_AUX_PIN,LOW);
    Shut_Step = enShut_StateMainPumpOff;
    Serial.println("DC-DC AUX Turned OFF...Turning OFF All Pumps ||");
 }
 else if(Shut_Step == enShut_StateMainPumpOff)
 {
    UpdateDuty_Pin11(0);
    Shut_Step = enShut_SateTurnOnBallValve;
    Serial.println("All Pumps Turned OFF...StateTurnOnValve ||");
 }
 else if(Shut_Step == enShut_SateTurnOnBallValve)
 {
    Shut_Step = enShut_StateStoreData;
    Serial.println("Ball Valve Turned ON(Not In Use)...Storing Data ||");
 }
 else if(Shut_Step == enShut_StateStoreData)
 {
    //Data Storage and logging can be Putting here
    Shut_Step = enShut_TurnSWKeyOFF;
    Serial.println("Data Logging and Storage Done...Turning OFF Key ||");
 }
 else if(Shut_Step == enShut_TurnSWKeyOFF)
 {
     Serial.println("Shutting Down...Finishing! ||");
     digitalWrite(OUT_SW_KEY,LOW);
     Shut_Step = enShut_StateComplete;
 }
 else if(Shut_Step == enShut_StateComplete)
 {
   //Should Not reach to This State as Power is turned off In last Step,
   //If reaches here Keep the state Engaged
    Shut_Step = enShut_StateComplete;
    Serial.println("System Waiting to shut down...Check Relay might be Faulty ||");
 }
}
