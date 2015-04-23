/* changes made to lcd layout to use abbreviations
   added morse code decoder using A6
   
*/
/* Changes made by W2ROW to allow for receive frequency CW offset by band
   changes to Ten_Tec code marked with W2ROW in comments
   offset_40m 40 meters requires a positive offset (currently set to 600 hertz)
   offset_20m 20 meters requires a negative offset (currently set to -600 hertz)
*/

/* W2ROW Updated TX_routine to fix bandwidth reset on transmit bug */

/*  Code for Production 3_2_15
<Patriot_507_Alpha_Rev01, Basic Software to operate a 2 band SSB/CW QRP Transceiver.
             See PROJECT PATRIOT SSB/CW QRP below>
 This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.
 
This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.
 
You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
//  https://groups.yahoo.com/neo/groups/TenTec507Patriot/info/
// !! Disclaimer !!  !! Disclaimer !!  !! Disclaimer !!  !! Disclaimer !!  !! Disclaimer !!
//  Attention ****  Ten-Tec Inc. is not responsile for any modification of Code 
//  below. If code modification is made, make a backup of the original code. 
//  If your new code does not work properly reload the factory code to start over again.
//  You are responsible for the code modifications you make yourself. And Ten-Tec Inc.
//  Assumes NO libility for code modification. Ten-Tec Inc. also cannot help you with any 
//  of your new code. There are several forums online to help with coding for the ChipKit UNO32.
//  If you have unexpected results after writing and programming of your modified code. 
//  Reload the factory code to see if the issues are still present. Before contacting Ten_Tec Inc.
//  Again Ten-Tec Inc. NOT RESPONSIBLE for modified code and cannot help with the rewriting of the 
//  factory code!
/*

/*********  PROJECT PATRIOT SSB/CW QRP  *****************************
  Program for the ChipKit Uno32
  This is a simple program to demonstrate a 2 band QRP Amateur Radio Transceiver
  Amateur Programmer Bill Curb (WA4CDM).
  This program will need to be cleaned up a bit and no doubt be made more efficient!
  Compiled using the MPIDE for the ChipKit Uno32.

  Prog for ad9834
  Serial timming setup for AD9834 DDS
  start > Fsync is high (1), Sclk taken high (1), Data is stable (0, or 1),
  Fsync is taken low (0), Sclk is taken low (0), then high (1), data changes
  Sclk starts again.
  Control Register D15, D14 = 00, D13(B28) = 1, D12(HLB) = X,
  Reset goes high to set the internal reg to 0 and sets the output to midscale.
  Reset is then taken low to enable output. 
 ***************************************************   
  This is real basic code to get things working. 
 *****************************************************************
 The pinout for the LCD is as follows: Also the LCD is setup up for 4 lines 20 charactors.
  * LCD RS pin to digital pin 26
  * LCD Enable pin to digital pin 27
  * LCD D4 pin to digital pin 28
  * LCD D5 pin to digital pin 29
  * LCD D6 pin to digital pin 30
  * LCD D7 pin to digital pin 31
  * LCD R/W pin to ground
  * 10K resistor:
  * ends to +5V and ground
  * wiper to LCD VO pin (pin 3)    analogWrite(Side_Tone, 127);
 *****************************************************************
    SELECT button steps from in 
    BW ( <Wide, green>, <Medium, yellow>, <Narrow, red> ).
    STEP ( <100 hz, green, <1Khz, yellow>, 10Khz, red> ).
    BND ( < 40M >, < 20M >, < , > ) OTHER has yet to be defined

  Default Band_width will be wide ( Green led lite ).
  When pressing the function button one of three leds will lite. 
  as explained above the select button will choose which setting will be used. 
  The Orange led in the Ten-Tec logo will flash to each step the STEP is set 
  too when tuning.  As it will also turn on when at the BAND edges.
  The TT logo led will also flash to indicate ALC. Input levels should be kept low enough
  to only flash this led on Peaks.  
  Default frequency on power up will be the calling frequency of the 
  40 meter band. 
  I.F. Frequency used is 9.0 mhz.
  DDS Range is: 
  40 meters will use HI side injection.
  9(I.F.) + 7(40m) = 16mhz.  9(I.F.) + 7.30 = 16.3 mhz.
  20 meters will use LO side injection.
  14(20m) - 9(I.F.) = 5mhz.  14.350(20m) - 9(I.F.) = 5.35 mhz.

  The Headphone jack can supply a headphone or speaker. The header pins(2) 
  if shorted will drive a speaker.
  Unshorted inserts 100 ohm resistors in series with the headphone to limit 
  the level to the headphones.

  The RIT knob will be at 0 offset in the Top Dead Center position. And will 
  go about -500 hz to +500 hz when turned to either extreme. Total range 
  about +/- 500 hz. This may change!

**************************************************************************************  

 Added an MCP23017 16-bit I/O Expander with Serial Interface to free up
 some I/O pins on the ChipKit Uno32 board.
 The parts of the 507 being controlled by this ic will be the Multi-purpose
 leds, the Select leds and the Wide/medium/Narrow control.
 5/1/2014 added a couple of routines to keep the filter wide on TX of SSB or CW
 Port B can be used by the user for external control.
 
 GPAO (21) Select Green led
 GPA1 (22) Select Yellow led
 GPA2 (23) Select Red led
 GPA3 (24) MP_A Green led
 GPA4 (25) MP_B Yellow led
 GPA5 (26) MP_C Red led
 GPA6 (27) Medium A8 BW_control
 GPA7 (28) Narrow A9 BW_control
 
 A mask function will be used to combine the various bits together.
*/
// various defines
#define SDATA_BIT                           11          // 
#define SCLK_BIT                            12          //  
#define FSYNC_BIT                           13          //  
#define RESET_BIT                           10          //  
#define FREQ_REGISTER_BIT                   9           //  
#define PHASE_REGISTER_BIT                  8
#define AD9834_FREQ0_REGISTER_SELECT_BIT    0x4000      //  
#define AD9834_FREQ1_REGISTER_SELECT_BIT    0x8000      //  
#define FREQ0_INIT_VALUE                    0x00000000  // 0x01320000

#define led                                 13
#define MUTE                                4
#define MIC_LINE_MUTE                       34

#define Side_Tone                           3           // 
                                                       
#define PTT_SSB                             22          // ptt input pulled high 
#define SSB_CW                              42          // control line for /SSB_CW switches output
                                                        // high for cw , low for ssb
#define TX_Dah                              33          // 
#define TX_Dit                              32          //  
#define TX_OUT                              38          //  
#define Band_End_Flash_led                  24          // // also this led will flash every 100/1khz/10khz is tuned
#define Band_Select                         41          // output for band select
#define Multi_Function_Button               5           //
#define Flash                               Band_End_Flash_led

#define Select_Button                       2           // 

#define Wide_BW                             0           // 
#define Medium_BW                           1           // 
#define Narrow_BW                           2           // 


#define Step_100_Hz                         0
#define Step_1000_hz                        1
#define Step_10000_hz                       2

#define  Other_1_user                       0           // 40 meters
#define  Other_2_user                       1           // 20 meters
#define  Other_3_user                       2           // anything you desire!
/**************************************************
 * WD9GYM modification for S meter on I2C LCD
 * Modified for base LCD code by W8CQD
 **************************************************/
// LCD Bars
byte meter_s1[8]  = {  B00000,  B00000,  B00000,  B00000,  B00000,  B00000,  B11000,  B11000, };
byte meter_s3[8]  = {  B00000,  B00000,  B00000,  B00000,  B00000,  B00011,  B11011,  B11011, };
byte meter_s5[8]  = {  B00000,  B00000,  B00000,  B00000,  B11000,  B11000,  B11000,  B11000, };
byte meter_s7[8]  = {  B00000,  B00000,  B00000,  B00011,  B11011,  B11011,  B11011,  B11011, };
byte meter_s9[8]  = {  B00000,  B00000,  B11000,  B11000,  B11000,  B11000,  B11000,  B11000, };
byte meter_s10[8] = {  B00000,  B00011,  B11011,  B11011,  B11011,  B11011,  B11011,  B11011, };
byte meter_s20[8] = {  B11000,  B11000,  B11000,  B11000,  B11000,  B11000,  B11000,  B11000, };
byte meter_s30[8] = {  B11011,  B11011,  B11011,  B11011,  B11011,  B11011,  B11011,  B11011, };

volatile unsigned long smeterTime, smeterLast, smeterInterval;
volatile unsigned long voltsTime, voltsLast, voltsInterval;

const int RitReadPin        = A0;  // pin that the sensor is attached to used for a rit routine later.
int RitReadValue            = 0;
int RitFreqOffset           = 0;
int old_RitFreqOffset       = 0;

const int SmeterReadPin     = A1;  // To give a realitive signal strength based on AGC voltage.
int SmeterReadValue         = 0;

const int BatteryReadPin    = A2;  // Reads 1/5 th or 0.20 of supply voltage.
int BatteryReadValue        = 0;

const int PowerOutReadPin   = A3;  // Reads RF out voltage at Antenna.
int PowerOutReadValue       = 0;

const int CodeReadPin       = A6;  // Can be used to decode CW. 
int CodeReadValue           = 0;

const int CWSpeedReadPin    = A7;  // To adjust CW speed for user written keyer.
int CWSpeedReadValue        = 0;            

//---------------- W2ROW  ------------------------------

int global_mode = 0;                  //W2ROW 0 for SSB,  1 for CW
int old_global_mode = 0;              //W2ROW Used to detect mode changes
const long offset_20m = -600;         //W2ROW CW offset for 20 meters (negative)
const long offset_40m = 600;          //W2ROW CW offset for 40 meters (positive)
long global_recv_offset = 0;          //W2ROW Computed receiver offset based on mode and band

#include "Wire.h"
#include <LiquidCrystal_I2C.h>

/*-----( Declare Constants )-----*/
/*-----( Declare objects )-----*/
// set the LCD address to 0x27 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

const char txt52[5]         = " ";
const char txt57[6]         = "FREQ:" ;
const char txt60[6]         = "STEP:";
const char txt62[3]         = "RX";
const char txt63[3]         = "TX";
const char txt64[4]         = "RIT";
const char txt65[5]         = "Band";
const char txt66[4]         = "20M";
const char txt67[4]         = "40M";
const char txt69[4]         = "   ";
const char txt70[5]         = "    ";
const char txt71[6]         = "     ";
const char txt72[10]        = "        ";
const char txt85[2]         = "W";
const char txt86[2]         = "M";
const char txt87[2]         = "N";
const char txt90[5]         = "S";
const char txt110[4]        = "BAT";
const char txt120[3]        = "BW";
const char txt130[5]        = "Mode";
const char txt132[3]        = "CW";
const char txt135[4]        = "SSB";
const char txt140[5]        = "WIDE";
const char txt150[7]        = "MEDIUM";
const char txt160[7]        = "NARROW";
const char txt170[7]        = "      ";
// some text variables for the display and terminal functions
#define bw 3
String bwtext[bw] = { "W", "M", "N" };
#define stp 3
String steptext[stp] = {"100", "1K ", "10K"};

String stringFREQ;
String stringREF;
String string_Frequency_Step;
String stringRIT;
String stringVolts;
String stringBW;

float floatVolts = 0.00;  //W8CQD

int TX_key;
int PTT_SSB_Key;
int old_PTT_SSB_Key;

int band_sel;                           // select band 40 or 20 meter
int band_set;
int bsm;  

int Step_Select_Button          = 0;
int Step_Select_Button1         = 0;
int Step_Multi_Function_Button  = 0;
int Step_Multi_Function_Button1 = 0;

int Selected_BW                 = 0;    // current Band width 
                                        // 0= wide, 1 = medium, 2= narrow
int Selected_Step               = 0;    // Current Step
int Selected_Other              = 0;    // To be used for anything

int old_bsm                     = 0;    //  this helps 5/13/14

int old_BatteryReadValue        = 0;

byte s = 0x00;                    // s = select
byte m = 0x00;                    // m = multi
byte b = 0x00;                    // b = bandwidth
byte t = 0x00;                    // s + m ored
byte old_b = 0x00;                // for the TX routine

//-----------------------------------------------------
// Encoder Stuff 
const int encoder0PinA          = 6; // reversed for 507
const int encoder0PinB          = 7; // reversed for 507

int val; 
int encoder0Pos                 = 0;
int encoder0PinALast            = LOW;
int n                           = LOW;

//-----------------------------------------------------
const long meter_40             = 16.285e6;        // IF + Band frequency, default for 40
                                                // HI side injection 40 meter 
                                                // range 16 > 16.3 mhz
const long meter_20             = 5.285e6;       // Band frequency - IF, LOW  default for 20
                                                // side injection 20 meter 
                                                // range 5 > 5.35 mhz
const long Reference            = 50.0e6;   // for ad9834 this may be 
                                                // tweaked in software to 
long frequency_TX;                                                // fine tune the Radio
long TX_frequency;
long RIT_frequency;
long RX_frequency;
long save_rec_frequency;
long Frequency_Step;
long old_Frequency_Step;
long frequency                  = 0;
long frequency_old              = 0;
long frequency_old_TX           = 0;
long frequency_tune             = 0;
long old_frequency_tune         = 0;
long frequency_default          = 0;
long fcalc;
long IF                         = 9.00e6;          //  I.F. Frequency
long TX_Frequency               = 0;

//-----------------------------------------------------
// Debug Stuff

unsigned long   loopCount       = 0;
unsigned long   lastLoopCount   = 0;
unsigned long   loopsPerSecond  = 0;
unsigned int    printCount      = 0;

unsigned long   loopStartTime   = 0;
unsigned long   loopElapsedTime = 0;
float           loopSpeed       = 0;

unsigned long LastFreqWriteTime = 0;

/**********************************************
      decoder stuff
**********************************************/
const int colums = 20; /// have to be 16 or 20
const int rows = 4;  /// have to be 2 or 4

int lcdindex = 0;
int line1[colums];
int line2[colums];

////////////////////////////////
// Define 8 specials letters  //
////////////////////////////////

byte U_umlaut[8] =   {B01010,B00000,B10001,B10001,B10001,B10001,B01110,B00000}; // 'Ü'  
byte O_umlaut[8] =   {B01010,B00000,B01110,B10001,B10001,B10001,B01110,B00000}; // 'Ö'  
byte A_umlaut[8] =   {B01010,B00000,B01110,B10001,B11111,B10001,B10001,B00000}; // 'Ä'    
byte AE_capital[8] = {B01111,B10100,B10100,B11110,B10100,B10100,B10111,B00000}; // 'Æ' 
byte OE_capital[8] = {B00001,B01110,B10011,B10101,B11001,B01110,B10000,B00000}; // 'Ø' 
byte fullblock[8] =  {B11111,B11111,B11111,B11111,B11111,B11111,B11111,B11111};  
byte AA_capital[8] = {B00100,B00000,B01110,B10001,B11111,B10001,B10001,B00000}; // 'Å'   
byte emtyblock[8] =  {B00000,B00000,B00000,B00000,B00000,B00000,B00000,B00000};  

int audioInPin = A6;
//int audioOutPin = 10;
int ledPin = 3;
boolean enableDecoder = false;
float magnitude ;
int magnitudelimit = 100;
int magnitudelimit_low = 100;
int realstate = LOW;
int realstatebefore = LOW;
int filteredstate = LOW;
int filteredstatebefore = LOW;


///////////////////////////////////////////////////////////
// The sampling frq will be 8928 on a 16 mhz             //
// without any prescaler etc                             //
// because we need the tone in the center of the bins    //
// you can set the tone to 496, 558, 744 or 992          //
// then n the number of samples which give the bandwidth //
// can be (8928 / tone) * 1 or 2 or 3 or 4 etc           //
// init is 8928/558 = 16 *4 = 64 samples                 //
// try to take n = 96 or 128 ;o)                         //
// 48 will give you a bandwidth around 186 hz            //
// 64 will give you a bandwidth around 140 hz            //
// 96 will give you a bandwidth around 94 hz             //
// 128 will give you a bandwidth around 70 hz            //
// BUT remember that high n take a lot of time           //
// so you have to find the compromice - i use 48         //
///////////////////////////////////////////////////////////

float coeff;
float Q1 = 0;
float Q2 = 0;
float sine;
float cosine;  
//float sampling_freq=8928.0;
float sampling_freq=24.760;
//float target_freq=558.0; /// adjust for your needs see above
float target_freq=744.0; /// adjust for your needs see above
float nc=48.0;  //// if you change  her please change next line also 
int testData[48];

//////////////////////////////
// Noise Blanker time which //
// shall be computed so     //
// this is initial          //
//////////////////////////////
int nbtime = 6;  /// ms noise blanker         

long starttimehigh;
long highduration;
long lasthighduration;
long hightimesavg;
long lowtimesavg;
long startttimelow;
long lowduration;
long laststarttime = 0;

char code[20];
int stop = LOW;
int wpm;
char lcdBuff[21] = "                    ";


/**********************************************
      decoder stuff
**********************************************/



void    serialDump();

//-----------------------------------------------------

void Default_frequency();
void AD9834_init();
void AD9834_reset();
void program_freq0(long freq);
void program_freq1(long freq1);  
void UpdateFreq(long freq);

void RX_Rit();             

//   void Frequency_up();                        
//   void Frequency_down(); 

                    
void TX_routine();
void RX_routine();
void Encoder();
void AD9834_reset_low();
void AD9834_reset_high();

void Change_Band();
void Step_Flash();
void RIT_Read();

void Multi_Function();          //
void Step_Selection();          // 
void Selection();               //
void Step_Multi_Function();     //

//----------------------------------------------------- 

void clock_data_to_ad9834(unsigned int data_word);

//-----------------------------------------------------

void setup() 
{
    // these pins are for the AD9834 control
    pinMode (SCLK_BIT,                 OUTPUT);    // clock
    pinMode (FSYNC_BIT,                OUTPUT);    // fsync
    pinMode (SDATA_BIT,                OUTPUT);    // data
    pinMode (RESET_BIT,                OUTPUT);    // reset
    pinMode (FREQ_REGISTER_BIT,        OUTPUT);    // freq register select

//---------------  Encoder ----------------------------
    pinMode (encoder0PinA,             INPUT);     // 
    pinMode (encoder0PinB,             INPUT);     // 

//-----------------------------------------------------
    pinMode (TX_Dit,                   INPUT);     // Dit Key line 
    pinMode (TX_Dah,                   INPUT);     // Dah Key line
    pinMode (TX_OUT,                   OUTPUT);    // control line for TX stuff
    pinMode (Band_End_Flash_led,       OUTPUT);    // line that flashes an led
    pinMode (PTT_SSB,                  INPUT);     // mic key has pull-up
    pinMode (SSB_CW,                   OUTPUT);    // control line for ssb cw switches
    
    pinMode (Multi_Function_Button,    INPUT);     // Choose from Band width, Step size, Other

    pinMode (Select_Button,            INPUT);     //  Selection from the above
 
    pinMode (Side_Tone,                OUTPUT);    // sidetone enable
    
    pinMode (Band_Select,              OUTPUT);
    
    pinMode (MUTE,                     OUTPUT);
    
    pinMode (MIC_LINE_MUTE,            OUTPUT);   // low on receive
       
    digitalWrite (Band_End_Flash_led,  LOW); //  not in 81324
    
    digitalWrite (MUTE,                LOW);
    
     
    BatteryReadValue = analogRead(BatteryReadPin);

    Default_Settings();
     // I2C stuff
     Wire.begin();                             // wake up I2C bus
     Wire.beginTransmission(0x20);  
     Wire.send(0x00);                          // IODIRA register
     Wire.send(0x00);                          // set all of port A to outputs
     Wire.endTransmission();
     Wire.beginTransmission(0x20);
     Wire.send(0x01);                          // IODIRB register
     Wire.send(0x00);                          // set all of port B to outputs
     Wire.endTransmission();
     
//-----------------------------------------------------
    // DDS
    AD9834_init();
    AD9834_reset();                            // low to high
//-----------------------------------------------------

    digitalWrite(TX_OUT,              LOW);      // turn off TX
    digitalWrite(SSB_CW,              LOW);      // keeps tx in ssb mode until high

//-----------------------------------------------------
    Frequency_Step = 100;   //  Can change this whatever step size one wants
    Selected_Step = Step_100_Hz; 
    DDS_Setup();        
    encoder0PinALast = digitalRead(encoder0PinA);   
    //attachInterrupt(encoder0PinA, Encoder, CHANGE);
    //attachInterrupt(encoder0PinB, Encoder, CHANGE);
    attachCoreTimerService(TimerOverFlow);//See function at the bottom of the file.

    Serial.begin(115200);
    Serial.println("Patriot Ready:");

    lcd.begin(20, 4);                          // 20 chars 4 lines
                                               // or change to suit ones 
                                               // lcd display 
    lcd.clear();                              // WD9GYM modification adjusted by W8CQD    
     lcd.print("WD9GYM");
    lcd.setCursor(0, 1);
    lcd.print("Ver_1_3 4/22/2015");
    delay(3000);
    lcd.clear();
    
    Display_Setup();
    
/**********************************************
      decoder stuff
**********************************************/
    
////////////////////////////////////
// The basic goertzel calculation //
////////////////////////////////////
  int	k;
  float	omega;
  k = (int) (0.5 + ((nc * target_freq) / sampling_freq));
  omega = (2.0 * PI * k) / nc;
  sine = sin(omega);
  cosine = cos(omega);
  coeff = 2.0 * cosine;
/**********************************************
      decoder stuff
**********************************************/

  
}   //    end of setup

//===================================================================
//===================================================================
 void Display_Setup()
{
 // RX
    lcd.setCursor(0, 0);
    lcd.print(txt62);      // RX
 
 // RIT
    lcd.setCursor(11, 0);
    lcd.print(txt64);      // RIT
  
 // TX
//    lcd.setCursor(0, 1);
//    lcd.print(txt63);       // TX 
 
 // BAND
 //   lcd.setCursor(12, 1);
 //   lcd.print(txt65);       // BAND
    // default band
    lcd.setCursor(11, 1);
    lcd.print(txt67);       // 40M   change this to txt66 for display of 20M
 
 // STEP
    lcd.setCursor(0, 1);
    lcd.print(txt90);       // STEP
 
 // BAT
//    lcd.setCursor(12, 1);
//    lcd.print(txt110);      // BAT  
 
 // BW
    lcd.setCursor(15, 1);
    lcd.print(txt120);      // BW
    // default BW
    lcd.setCursor(18, 1);
//    lcd.print(txt140);      // Wide
    lcd.print(bwtext[0]);
 
 // MODE
 //   lcd.setCursor(7, 1);
 //   lcd.print(txt130);      // MODE
    // default MODE
    lcd.setCursor(7, 1);
     lcd.print(txt69);
      lcd.setCursor(7, 1);
        lcd.print(txt135);  // SSB

  // VLT - Input Voltage
  lcd.setCursor(0, 2);
 // lcd.print(txt110);      // VLT
  lcd.print("V");      
        
}      // end of Display_Setup

//===================================================================

void Default_Settings()
{
   m = 0x08;                 //     

   s = 0x01;                 //
   
   bsm = 0;                  //  bsm = 0 is 40 meters bsm = 1 is 20 meters
   
   frequency_default = meter_40; // change this to meter_20 for 20 meter default
   Default_frequency();
   b = 0x00; // Hardware control of I.F. filter shape wide setting
   Selected_BW = Wide_BW; 
  
    digitalWrite (TX_OUT,               LOW);                                            
    digitalWrite (Band_End_Flash_led,   LOW);
    digitalWrite (Side_Tone,            LOW);    
    digitalWrite ( FREQ_REGISTER_BIT,   LOW);
    digitalWrite ( SSB_CW,              LOW);
    digitalWrite ( Band_Select,         LOW);
    digitalWrite ( MUTE,                HIGH);
    digitalWrite ( MIC_LINE_MUTE,       LOW);    //  receive mode
}

//-----------------------------------------------------------------
void DDS_Setup()
{
    digitalWrite(FSYNC_BIT,             HIGH);  // 
    digitalWrite(SCLK_BIT,              HIGH);  // 
}

//======================= Main Part =================================
void loop()    
{
       //  TX_routine(); 
         Encoder();
          TX_routine(); 
           RX_Rit();
            Multi_Function(); 
  
    loopCount++;
    loopElapsedTime    = millis() - loopStartTime;
//    if( 1000 <= loopElapsedTime ) {
//               serialDump();    // comment this out to remove the one second tick
//           }
    Splash_Volts(); //Display input volatage - W8CQD
  if (enableDecoder == false) {
      writeSmeter();  //Add call to S Meter display routine WD9GYM/W8CQD
  } else {
/**********************************************
      decoder stuff
**********************************************/
  ///////////////////////////////////// 
  // The basic where we get the tone //
  /////////////////////////////////////
  
  for (char index = 0; index < nc; index++)
  {
    testData[index] = analogRead(audioInPin);
  }
  for (char index = 0; index < nc; index++){
	  float Q0;
	  Q0 = coeff * Q1 - Q2 + (float) testData[index];
	  Q2 = Q1;
	  Q1 = Q0;	
  }
  float magnitudeSquared = (Q1*Q1)+(Q2*Q2)-Q1*Q2*coeff;  // we do only need the real part //
  magnitude = sqrt(magnitudeSquared);
  Q2 = 0;
  Q1 = 0;

//  Serial.print(magnitude); Serial.println();  //// here you can measure magnitude for setup..
  
  /////////////////////////////////////////////////////////// 
  // here we will try to set the magnitude limit automatic //
  ///////////////////////////////////////////////////////////
  
  if (magnitude > magnitudelimit_low){
    magnitudelimit = (magnitudelimit +((magnitude - magnitudelimit)/6));  /// moving average filter
  }
 
  if (magnitudelimit < magnitudelimit_low)
	magnitudelimit = magnitudelimit_low;
  
  ////////////////////////////////////
  // now we check for the magnitude //
  ////////////////////////////////////

  if(magnitude > magnitudelimit*0.6) // just to have some space up 
     realstate = HIGH; 
  else
    realstate = LOW; 
  
  ///////////////////////////////////////////////////// 
  // here we clean up the state with a noise blanker //
  /////////////////////////////////////////////////////
 
  if (realstate != realstatebefore){
	laststarttime = millis();
  }
  if ((millis()-laststarttime)> nbtime){
	if (realstate != filteredstate){
		filteredstate = realstate;
	}
  }
 
 ////////////////////////////////////////////////////////////
 // Then we do want to have some durations on high and low //
 ////////////////////////////////////////////////////////////
 
 if (filteredstate != filteredstatebefore){
	if (filteredstate == HIGH){
		starttimehigh = millis();
		lowduration = (millis() - startttimelow);
	}

	if (filteredstate == LOW){
		startttimelow = millis();
		highduration = (millis() - starttimehigh);
        if (highduration < (2*hightimesavg) || hightimesavg == 0){
			hightimesavg = (highduration+hightimesavg+hightimesavg)/3;     // now we know avg dit time ( rolling 3 avg)
		}
		if (highduration > (5*hightimesavg) ){
			hightimesavg = highduration+hightimesavg;     // if speed decrease fast ..
		}
	}
  }

 ///////////////////////////////////////////////////////////////
 // now we will check which kind of baud we have - dit or dah //
 // and what kind of pause we do have 1 - 3 or 7 pause        //
 // we think that hightimeavg = 1 bit                         //
 ///////////////////////////////////////////////////////////////
 
 if (filteredstate != filteredstatebefore){
  stop = LOW;
  if (filteredstate == LOW){  //// we did end a HIGH
   if (highduration < (hightimesavg*2) && highduration > (hightimesavg*0.6)){ /// 0.6 filter out false dits
	strcat(code,".");
//	Serial.print(".");
   }
   if (highduration > (hightimesavg*2) && highduration < (hightimesavg*6)){ 
	strcat(code,"-");
//	Serial.print("-");
	wpm = (wpm + (1200/((highduration)/3)))/2;  //// the most precise we can do ;o)
   
   }
  }
 
   if (filteredstate == HIGH){  //// we did end a LOW
   
   float lacktime = 1;
   if(wpm > 25)lacktime=1.0; ///  when high speeds we have to have a little more pause before new letter or new word 
   if(wpm > 30)lacktime=1.2;
   if(wpm > 35)lacktime=1.5;
   
   if (lowduration > (hightimesavg*(2*lacktime)) && lowduration < hightimesavg*(5*lacktime)){ // letter space
    docode();
	code[0] = '\0';
//	Serial.print("/");
   }
   if (lowduration >= hightimesavg*(5*lacktime)){ // word space
    docode();
	code[0] = '\0';
	printascii(32);
//	Serial.println();
   }
  }
 }
 
 //////////////////////////////
 // write if no more letters //
 //////////////////////////////

  if ((millis() - startttimelow) > (highduration * 6) && stop == LOW){
   docode();
   code[0] = '\0';
   stop = HIGH;
  }

 /////////////////////////////////////
 // we will turn on and off the LED //
 // and the speaker                 //
 /////////////////////////////////////
 
   if(filteredstate == HIGH){ 
//     digitalWrite(ledPin, HIGH);
//	 tone(audioOutPin,target_freq);
   }
   else{
//     digitalWrite(ledPin, LOW);
//	 noTone(audioOutPin);
   }
 
 //////////////////////////////////
 // the end of main loop clean up//
 /////////////////////////////////
 //updateinfolinelcd();
 realstatebefore = realstate;
 lasthighduration = highduration;
 filteredstatebefore = filteredstate;
  }
/**********************************************
      decoder stuff
**********************************************/


}    //  END LOOP



//-----------------------------------------------------
void  RX_Rit()
   {
    RIT_Read();
    frequency_tune  = frequency + RitFreqOffset; // RitFreqOffset is from Rit_Read();
    UpdateFreq(frequency_tune);
    splash_RX_freq();   // this only needs to be updated when encoder changed. 
   }

//------------------------------------------------------
void RIT_Read()
{
    int RitReadValueNew =0 ;
      RitReadValueNew = analogRead(RitReadPin);
      
        RitReadValue = (RitReadValueNew + (12 * RitReadValue)) / 13;//Lowpass filter possible display role if changed
          if(RitReadValue < 500) 
            RitFreqOffset = RitReadValue-500;
              else if(RitReadValue < 523) 
                RitFreqOffset = 0;//Deadband in middle of pot
    else 
        RitFreqOffset = RitReadValue - 523;
        
    splash_RIT();    //   comment out if display is not needed
}

//-----------------------------------------------------
void UpdateFreq(long freq)
{
    if (LastFreqWriteTime != 0)
      { if ((millis() - LastFreqWriteTime) < 100) return; }
        LastFreqWriteTime = millis();
          //if(freq == frequency_old) return;            //W2ROW
          if ((freq == frequency_old) && (global_mode == old_global_mode)) return;  //W2ROW  
            //program_freq0( freq );                     //W2ROW
             program_freq0( freq + global_recv_offset );  //W2ROW set offset
              frequency_old = freq;
              old_global_mode = global_mode;             //W2ROW
}

//--------------------------------------------------------------
void UpdateFreq1(long frequency_TX)
{
    if (LastFreqWriteTime != 0)
      { if ((millis() - LastFreqWriteTime) < 100) return; }
        LastFreqWriteTime = millis();
          if(frequency_TX == frequency_old_TX) return;
            program_freq1( frequency_TX );
              frequency_old_TX = frequency_TX;
}

//------------------------------------------------------------------
//##################################################################
//---------------------  TX Routine  -------------------------------
void TX_routine()      

//W2ROW all mode changes happen here global_mode will reflect those changes
//  as will global_recv_offset

{     //------------------  SSB Portion  ----------------------------
  
   PTT_SSB_Key = digitalRead( PTT_SSB );               // check to see if PTT is pressed, 
    if   ( PTT_SSB_Key == LOW  )                       // if pressed do the following
    {
 //////////////// W2ROW Moved out of loop ///////////////////     
      Splash_MODE();
        global_mode = 0;                                //W2ROW mode is SSB
        global_recv_offset = 0;                         //W2ROW
        TX_frequency = frequency;
        frequency_tune = TX_frequency;
        splash_TX_freq();
        
        old_b = b;                                 // save original b into old_b
        b = 0x00;                                // b is now set to wide filter setting
        
        Select_Multi_BW_Ored();                       // b is sent to port expander ic
 /////////////////// end moved out of loop //////////////////////       
      
      do
        {
          TX_Frequency = frequency;
            frequency_tune  = TX_Frequency;             // RitFreqOffset is from Rit_Read();
            digitalWrite ( FREQ_REGISTER_BIT,   HIGH);      // 
              UpdateFreq1(frequency_tune);
        
        //Splash_MODE();
        //global_mode = 0;                                //W2ROW mode is SSB
        //global_recv_offset = 0;                         //W2ROW
         // splash_TX_freq();
          
          //  old_b = b;                                 // save original b into old_b
          //    b = 0x00;                                // b is now set to wide filter setting
          
           //Select_Multi_BW_Ored();                       // b is sent to port expander ic
             
           digitalWrite ( MIC_LINE_MUTE, HIGH);        // turns Q35, Q16 off, unmutes mic/line
             digitalWrite (SSB_CW, HIGH);              // this causes the ALC line to connect
               digitalWrite(TX_OUT, HIGH);             // Truns on Q199 (pwr cntrl)(switched lo/dds)
                                                       // mutes audio to lm386
         PTT_SSB_Key = digitalRead(PTT_SSB);           // check to see if PTT is pressed  
        }     while (PTT_SSB_Key == LOW); 
       
         b = old_b;                                    // original b is now restored
          Select_Multi_BW_Ored();                      // original b is sent to port expander
          
           digitalWrite(TX_OUT, LOW);                  // turn off TX stuff
             digitalWrite ( FREQ_REGISTER_BIT,   LOW); // added 6/23/14  
               digitalWrite ( MIC_LINE_MUTE, LOW);     // turns Q36, Q16 on, mutes mic/line
   }  // End of SSB TX routine
                //---------------  CW Portion  --------------------------------
    TX_key = digitalRead(TX_Dit);                      //  Maybe put these on an interrupt!
     if ( TX_key == LOW)                               // was high   
    {
  //////////////// W2ROW Moved out of loop ///////////////////       
      Splash_MODE();  
        global_mode = 1;                               //W2ROW mode is CW
        if (bsm == 0)                                  //W2ROW 40 m
         global_recv_offset = offset_40m;              //W2ROW
        else                                           //W2ROW 20 m
         global_recv_offset = offset_20m;              //W2ROW
         
        TX_frequency = frequency;
        frequency_tune = TX_frequency;
        splash_TX_freq();
         
        old_b = b; 
        b = 0x00;                                  // b is now set to wide filter setting
       
        Select_Multi_BW_Ored();                       // b is sent to port expander ic 
     /////////////////// end moved out of loop //////////////////////    
        
       do
       {
                                          
            TX_Frequency = frequency;
            frequency_tune  = TX_Frequency;          // RitFreqOffset is from Rit_Read();
              digitalWrite ( FREQ_REGISTER_BIT,   HIGH);      //    
                UpdateFreq1(frequency_tune);
                
                   
        //Splash_MODE();  
       //global_mode = 1;                               //W2ROW mode is CW
        //if (bsm == 0)                                  //W2ROW 40 m
       //  global_recv_offset = offset_40m;              //W2ROW
        //else                                           //W2ROW 20 m
       //  global_recv_offset = offset_20m;              //W2ROW
        // splash_TX_freq();
          
         //   old_b = b; 
         //    b = 0x00;                                  // b is now set to wide filter setting
         //     Select_Multi_BW_Ored();                  // b is sent to port expander ic
               
         
        digitalWrite(TX_OUT, HIGH);                    // turns tx circuit on
          digitalWrite (SSB_CW, LOW);                  // enables the cw pull down circuit  
            digitalWrite(Side_Tone, HIGH);             // enables side-tone source
              TX_key = digitalRead(TX_Dit);            // reads dit key line
       }  while (TX_key == LOW);                       // key still down
         b = old_b;                                    // original b is now restored
           Select_Multi_BW_Ored();   
             for (int i=0; i <= 10e2; i++);            // delay
              digitalWrite(TX_OUT, LOW);               // trun off TX cw key is now high
                digitalWrite ( FREQ_REGISTER_BIT,   LOW);    // return to DDS register 0  not in other 
                       
       digitalWrite(Side_Tone, LOW);                   // side-tone off
       }
}                                                      // end  TX_routine()


//------------------------------------------------------------------------
//--------------------------- Encoder Routine ---------------------  

void Encoder()
{  
    n = digitalRead(encoder0PinA);
    if ( encoder0PinALast != n)
    {
    if ((encoder0PinALast == LOW) && (n == HIGH)) 
    {
        if (digitalRead(encoder0PinB) == LOW) //  Frequency_down
        {      //encoder0Pos--;
           frequency = frequency - Frequency_Step;
           Step_Flash();
           if ( bsm == 1 ) { Band_20_Limit(); }
           else if ( bsm == 0 ) { Band_40_Limit(); }
        } 
        else                                  //  Frequency_up
        {      //encoder0Pos++;
          frequency = frequency + Frequency_Step;
          Step_Flash();
          if ( bsm == 1 ) { Band_20_Limit(); }
          else if ( bsm == 0 ) { Band_40_Limit(); }
        }
    } 
    encoder0PinALast = n;
    }
}

//-------------------------------------------------------
//-------------------------------------------------------
void Change_Band()
{
    if ( bsm == 1 )                              //  select 40 or 20 meters 1 for 20 0 for 40
    { 
      digitalWrite(Band_Select, HIGH);
      Band_Set_40_20M();
    }
    else 
    { 
      digitalWrite(Band_Select, LOW);
      Band_Set_40_20M();
      IF *= -1;                               //  HI side injection
    }
}

//------------------ Band Select ------------------------------------
void Band_Set_40_20M()

//W2ROW all band changes happen here bsm will reflect those changes
//  as will global_recv_offset

{
  if ( old_bsm != bsm)                          //  this helps 5/13/14
  {
    if ( bsm == 1 )                             //  select 40 or 20 meters 1 for 20 0 for 40
    { 
        frequency_default = meter_20;
        Splash_Band(); 
        
        if (global_mode == 0)                    //W2ROW SSB
         global_recv_offset = 0;                 //W2ROW
        else                                     //W2ROW CW
         global_recv_offset = offset_20m;        //W2ROW
    }
    else 
    { 
        frequency_default = meter_40; 
        Splash_Band();
        IF *= -1;                                //  HI side injection
        
         if (global_mode == 0)                   //W2ROW SSB
         global_recv_offset = 0;                 //W2ROW
        else                                     //W2ROW CW
         global_recv_offset = offset_40m;        //W2ROW
    }
    Default_frequency();
}
    old_bsm = bsm;                              //  this helps 5/13/14
}

//--------------------Default Frequency-----------------------------
void Default_frequency()
{
    frequency = frequency_default;
    UpdateFreq(frequency);
    splash_RX_freq(); 
}   //  end   Default_frequency

//-----------------------------------------------------
//-----------------------------------------------------
 void  Band_40_Limit()
 {
   if ( frequency >= 16.3e6 )
    { 
       frequency = 16.3e6;
         stop_led_on();    
    }
    else if ( frequency <= 16.0e6 )  
    { 
        frequency = 16.0e6;
        stop_led_on();
    }
    else { stop_led_off(); }
 }
//-----------------------------------------------------  
  void  Band_20_Limit()
  {
    if ( frequency >= 5.35e6 )
    { 
       frequency = 5.35e6;
       stop_led_on();    
    }
    else if ( frequency <= 5.0e6 )  
    { 
        frequency = 5.0e6;
        stop_led_on();
    } 
    else { stop_led_off(); }
  }

//-----------------------------------------------------  
void Step_Flash()
{
    stop_led_on();
    for (int i=0; i <= 25e3; i++); // short delay 
    stop_led_off();   
}

//-----------------------------------------------------
void stop_led_on()  //  band edge and flash
{
    digitalWrite(Band_End_Flash_led, HIGH);
}

//-----------------------------------------------------
void stop_led_off()
{
    digitalWrite(Band_End_Flash_led, LOW);
}

//===================================================================
//===================================================================
void Multi_Function() //  pushbutton for BW, Step, Other
{
// look into a skip rtoutine for this
    Step_Multi_Function_Button = digitalRead(Multi_Function_Button);
    if (Step_Multi_Function_Button == HIGH) 
    {   
       while( digitalRead(Multi_Function_Button) == HIGH ){ }  // added for testing
        for (int i=0; i <= 150e3; i++); // short delay

        Step_Multi_Function_Button1 = Step_Multi_Function_Button1++;
        if (Step_Multi_Function_Button1 > 2 ) 
        { 
            Step_Multi_Function_Button1 = 0; 
        }
    }
    Step_Function();
}  // end Multi_Function()
//==================================================================
//================================================================== 
void Step_Function()
{
    switch ( Step_Multi_Function_Button1 )
    {
        case 0:
            m = 0x08;   // GPA3(24) Controls Function Green led
            Select_Multi_BW_Ored();
            Step_Select_Button1 = Selected_BW; // 
            Step_Select(); //
            Selection();
            for (int i=0; i <= 255; i++); // short delay
            break;   //

        case 1:
            m = 0x10;   // GPA4(25) Controls Function Yellow led
            Select_Multi_BW_Ored();
            Step_Select_Button1 = Selected_Step; //
            Step_Select(); //
            Selection();
            for (int i=0; i <= 255; i++); // short delay
            break;   //

        case 2: 
            m = 0x20;   // GPA5(26) Controls Function Red led
            Select_Multi_BW_Ored();
            Step_Select_Button1 = Selected_Other; //
            Step_Select(); //
            Selection();
            for (int i=0; i <= 255; i++); // short delay
            break;   //  
    }
}  // end Step_Function()

//===================================================================
/****************************************************
   decoder selection from PA3ANG to shut off s meter 
   and activate decoder
*****************************************************/
void  Selection()
{
    Step_Select_Button = digitalRead(Select_Button);
    if (Step_Select_Button == HIGH) 
    {   
       // Debounce start
       unsigned long time;
       unsigned long start_time;
       unsigned long long_time;
       long_time = millis();
       
       time = millis();
       while( digitalRead(Select_Button) == HIGH ){ 
         
         // function button is pressed longer then 0.5 seconds
         if ( (millis() - long_time) > 500 ) { 
           // flip logic boolean enableDecoder
             if (enableDecoder == true) {
                 enableDecoder = false;
                 lcd.setCursor(0, 3);
                 for (int i = 0; i < 20; i++) {
                      lcd.print(" ");
                 }
             } else {
                 enableDecoder = true;
             }
           // wait for button release
           while( digitalRead(Select_Button) == HIGH ){ 
           }   
           return;        
         } 
         start_time = time;
         while( (time - start_time) < 7) {
           time = millis();
         }
       } // Debounce end
        Step_Select_Button1 = Step_Select_Button1++;
        if (Step_Select_Button1 > 2 || (Step_Select_Button1 > 1 && Step_Multi_Function_Button1 == 2) ) 
        { 
            Step_Select_Button1 = 0; 
        }
    }
    Step_Select(); 
}

/*
void  Selection()
{
    Step_Select_Button = digitalRead(Select_Button);
    if (Step_Select_Button == HIGH) 
    {   
       while( digitalRead(Select_Button) == HIGH ){ }  // added for testing
        for (int i=0; i <= 150e3; i++); // short delay

        Step_Select_Button1 = Step_Select_Button1++;
        if (Step_Select_Button1 > 2 ) 
        { 
            Step_Select_Button1 = 0; 
        }
    }
    Step_Select(); 
}  // end Selection()
*/
/****************************************************
    end of
   decoder selection from PA3ANG to shut off s meter 
   and activate decoder
*****************************************************/
//-----------------------------------------------------------------------  
void Step_Select()
{
    switch ( Step_Select_Button1 )
    {
        case 0: //   Select_Green   
                   s = 0x01;  // GPA0(21) Controls Selection Green led 
               if (Step_Multi_Function_Button1 == 0)
                  {
                     b = 0x00; // Hardware control of I.F. filter shape
                     Selected_BW = Wide_BW;  // GPA7(28)LOW_GPA6(27)LOW wide
                  } 
               else if (Step_Multi_Function_Button1 == 1)
                  {
                       Frequency_Step = 100;   //  Can change this whatever step size one wants
                       Selected_Step = Step_100_Hz; 
                  } 
               else if (Step_Multi_Function_Button1 == 2)
                  {
                        bsm = 0;
                        Change_Band();
                        Encoder();
                        Selected_Other = Other_1_user; 
                 // Other_1();
                  } 
               for (int i=0; i <= 255; i++); // short delay  
               break;

        case 1: //   Select_Yellow  
                  s = 0x02;    //  GPA1(22) Controls Selection Green led
               if (Step_Multi_Function_Button1 == 0) 
                  {
                     b = 0x40; // Hardware control of I.F. filter shape
                     Selected_BW = Medium_BW;  //  GPA7(28)LOW_GPA6(27)HIGH medium
                  } 
               else if (Step_Multi_Function_Button1 == 1) 
                  {
                     Frequency_Step = 1e3;   //  Can change this whatever step size one wants
                     Selected_Step = Step_1000_hz; 
                  }
               else if (Step_Multi_Function_Button1 == 2) 
                  {
                     bsm = 1;
                     Change_Band();
                     Encoder();
                     Selected_Other = Other_2_user; 
                    
                 //   Other_2(); 
                  }
              for (int i=0; i <= 255; i++); // short delay   
              break; 

        case 2: //   Select_Red    
                 s = 0x04;    //  GPA2(23) Controls Selection Green led
              if (Step_Multi_Function_Button1 == 0) 
                  {
                    b = 0x80; // Hardware control of I.F. filter shape
                    Selected_BW = Narrow_BW;   //  GPA7(28)HIGH_GPA6(27)LOW narrow
                  } 
              else if (Step_Multi_Function_Button1 == 1) 
                  {
                     Frequency_Step = 10e3;    //  Can change this whatever step size one wants
                     Selected_Step = Step_10000_hz;
                  }
              else if (Step_Multi_Function_Button1 == 2) 
                  {
                     Selected_Other = Other_3_user; 
                     
                 //       Other_3(); 
                  }
              for (int i=0; i <= 255; i++); // short delay
              break;     
    }
        Select_Multi_BW_Ored();
        Splash_Step_Size();
        Splash_BW();
}  // end Step_Select()

//----------------------------------------------------------------------------------
void Select_Multi_BW_Ored()
{
    t = s | m | b ;    // or'ed bits
   
 Wire.beginTransmission(0x20);
 Wire.send(0x12); // GPIOA
 Wire.send(t); // port A  result of s, m, b
 Wire.endTransmission(); 
  
}  // end  Select_Multi_BW_Ored()

//-----------------------------------------------------------------------------
// ****************  Dont bother the code below  ******************************
// \/  \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/
//-----------------------------------------------------------------------------
void program_freq0(long frequency)
{
    AD9834_reset_high();  
    int flow,fhigh;
    fcalc = frequency*(268.435456e6 / Reference );    // 2^28 =
    flow = fcalc&0x3fff;              //  49.99975mhz  
    fhigh = (fcalc>>14)&0x3fff;
    digitalWrite(FSYNC_BIT, LOW);  //
    clock_data_to_ad9834(flow|AD9834_FREQ0_REGISTER_SELECT_BIT);
    clock_data_to_ad9834(fhigh|AD9834_FREQ0_REGISTER_SELECT_BIT);
    digitalWrite(FSYNC_BIT, HIGH);
    AD9834_reset_low();
}    // end   program_freq0

//------------------------------------------------------------------------------

void program_freq1(long frequency_TX)
{
    AD9834_reset_high(); 
    int flow,fhigh;
    fcalc = frequency*(268.435456e6 / Reference );    // 2^28 =
    flow = fcalc&0x3fff;              //  use for 49.99975mhz   
    fhigh = (fcalc>>14)&0x3fff;
    digitalWrite(FSYNC_BIT, LOW);  
    clock_data_to_ad9834(flow|AD9834_FREQ1_REGISTER_SELECT_BIT);
    clock_data_to_ad9834(fhigh|AD9834_FREQ1_REGISTER_SELECT_BIT);
    digitalWrite(FSYNC_BIT, HIGH);  
    AD9834_reset_low();
}  

//------------------------------------------------------------------------------
void clock_data_to_ad9834(unsigned int data_word)
{
    char bcount;
    unsigned int iData;
    iData=data_word;
    digitalWrite(SCLK_BIT, HIGH);  //portb.SCLK_BIT = 1;  
    // make sure clock high - only chnage data when high
    for(bcount=0;bcount<16;bcount++)
    {
        if((iData & 0x8000)) digitalWrite(SDATA_BIT, HIGH);  //portb.SDATA_BIT = 1; 
        // test and set data bits
        else  digitalWrite(SDATA_BIT, LOW);  
        digitalWrite(SCLK_BIT, LOW);  
        digitalWrite(SCLK_BIT, HIGH);     
        // set clock high - only change data when high
        iData = iData<<1; // shift the word 1 bit to the left
    }  // end for
}      // end  clock_data_to_ad9834

//-----------------------------------------------------------------------------
void AD9834_init()      // set up registers
{
    AD9834_reset_high(); 
    digitalWrite(FSYNC_BIT, LOW);
    clock_data_to_ad9834(0x2300);  // Reset goes high to 0 the registers and enable the output to mid scale.
    clock_data_to_ad9834((FREQ0_INIT_VALUE&0x3fff)|AD9834_FREQ0_REGISTER_SELECT_BIT);
    clock_data_to_ad9834(((FREQ0_INIT_VALUE>>14)&0x3fff)|AD9834_FREQ0_REGISTER_SELECT_BIT);
    clock_data_to_ad9834(0x2200); // reset goes low to enable the output.
    AD9834_reset_low();
    digitalWrite(FSYNC_BIT, HIGH);  
}  //  end   AD9834_init()

//----------------------------------------------------------------------------   
void AD9834_reset()
{
    digitalWrite(RESET_BIT, HIGH);  // hardware connection
    for (int i=0; i <= 2048; i++);  // small delay

    digitalWrite(RESET_BIT, LOW);   // hardware connection
}  // end AD9834_reset()

//-----------------------------------------------------------------------------
void AD9834_reset_low()
{
    digitalWrite(RESET_BIT, LOW);
}  // end AD9834_reset_low()

//..............................................................................     
void AD9834_reset_high()
{  
    digitalWrite(RESET_BIT, HIGH);
}  // end  AD9834_reset_high()

//^^^^^^^^^^^^^^^^^^^^^^^^^  DON'T BOTHER CODE ABOVE  ^^^^^^^^^^^^^^^^^^^^^^^^^ 
//=============================================================================

//------------------------Display Stuff below-----------------------------------

// All the code for display below seems to work well. 6-18-14

//------------------- Splash RIT -----------------------------------------------  
void splash_RIT()      // not used
{ 
    if ( old_RitFreqOffset != RitFreqOffset) // only if RIT changes
 { 
    lcd.setCursor(15, 0);
    lcd.print(txt70);                       // spaces
  
    lcd.setCursor(15, 0); 
    stringRIT = String( RitFreqOffset, DEC);
       
    lcd.print(stringRIT);
 }
    old_RitFreqOffset = RitFreqOffset;      // test for Rit change
}
//------------------------------------------------------------------------------
void splash_TX_freq()
{
/*  
      long TXD_frequency;  // ADDED 6-18-14 OK
      
      if ( bsm == 1 )                      // test for 20M
     { TXD_frequency = frequency_tune ; }
   
     else if ( bsm == 0 )                  // test for 40M
     { TXD_frequency = frequency_tune ; } 
  //---------------------------------------------------
   
     if ( TXD_frequency < 5.36e6 )
    { TXD_frequency = TXD_frequency + 9e6; }
   
     else if ( TXD_frequency > 15.95e6 )
    { TXD_frequency = TXD_frequency - 9e6; } 
  //--------------------------------------------------

    lcd.setCursor(3, 1);
    stringFREQ = String(TXD_frequency / 100, DEC);
    
    if (TXD_frequency > 10000000)
    {
    lcd.print("14."+ stringFREQ.substring(2,6));
    }
    else  
    {
    lcd.print(" 7."+ stringFREQ.substring(1,5));   
    }  
*/    
 }
//------------------------------------------------------------------------------
void splash_RX_freq()
{
      long RXD_frequency; // ADDED 6-18-14 OK
   
  if ( old_frequency_tune != frequency_tune )
   {
      if ( bsm == 1 )                      // test for 20M
       { RXD_frequency = frequency_tune ; }
     
     else if ( bsm == 0 )                 // test for 40M
     { RXD_frequency = frequency_tune ; }
   //-------------------------------------------
       
     if ( RXD_frequency < 5.36e6 )
     { RXD_frequency = RXD_frequency + 9e6; }
     
     else if ( RXD_frequency > 15.95e6 )
     { RXD_frequency = RXD_frequency - 9e6; }
  //--------------------------------------------
    lcd.setCursor(3, 0);
    lcd.print(txt72);                       // spaces
    
    lcd.setCursor(3, 0);
    stringFREQ = String(RXD_frequency/100 , DEC);
    if (RXD_frequency > 10000000)
    {
    lcd.print("14."+ stringFREQ.substring(2,6));
    }
    else  
    {
    lcd.print(" 7."+ stringFREQ.substring(1,5));   
    }  
   }
   old_frequency_tune = frequency_tune;
 }
 
//-----------------------------------------------------------------
void Splash_Band()
{
    if ( bsm == 1 )               // test for 20M
    {
        lcd.setCursor(11, 1);
        lcd.print(txt66);        // 20 meters
    }
    else                         
    {
        lcd.setCursor(11, 1);
        lcd.print(txt67);        // 40 meters
    } 
}   

//---------------------------------------------------------------------------------

void Splash_Step_Size()
{
    if ( old_Frequency_Step != Frequency_Step ) // 
 { 
    lcd.setCursor(2, 1);
    lcd.print(txt71 );                       // spaces
    
    lcd.setCursor(2, 1); 

 //   string_Frequency_Step = String(Frequency_Step, DEC);
//    lcd.print(string_Frequency_Step);
    if (Frequency_Step == 100) {
        lcd.print(steptext[0]);
    } else if (Frequency_Step == 1000) {
               lcd.print(steptext[1]);
    } else if (Frequency_Step == 10000) {
               lcd.print(steptext[2]);
    }
 }
    old_Frequency_Step = Frequency_Step;      // test for Rit change 
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
void Splash_BW()
{
  if ( old_b != b )
  {
 if ( b == 0x00 )
  { 
 //     lcd.setCursor(3, 2);
 //       lcd.print(txt170);
 //         lcd.setCursor(3, 2);
 //           lcd.print(txt140);  // wide
    lcd.setCursor(18, 1);
    lcd.print(bwtext[0]);

  }
 else if ( b == 0x40 )
  { 
//      lcd.setCursor(3, 2);
//        lcd.print(txt170);
//          lcd.setCursor(3, 2);
//            lcd.print(txt150);  // medium
    lcd.setCursor(18, 1);
    lcd.print(bwtext[1]);    // medium

  }
 else {
//      lcd.setCursor(3, 2);
//        lcd.print(txt170);
//          lcd.setCursor(3, 2);
//            lcd.print(txt160);  // narrow
    lcd.setCursor(18, 1);
    lcd.print(bwtext[2]);   // narrow

  }
 }
  old_b = b ;
}

//--------------------------------------------------------------------
void Splash_MODE()
{
  if ( old_PTT_SSB_Key != PTT_SSB_Key )
 { 
  if ( PTT_SSB_Key == LOW )
  {
    lcd.setCursor(7, 1);
     lcd.print(txt69);
      lcd.setCursor(7, 1);
        lcd.print(txt135);    // SSB
  }
  else                         
  {
    lcd.setCursor(7, 1);
     lcd.print(txt69);
       lcd.setCursor(7, 1);
        lcd.print(txt132);    // CW
  }
 }
   old_PTT_SSB_Key = PTT_SSB_Key;
}
//---------------------Display Battery Level----------------------------------------
//   Added by W8CQD
void Splash_Volts()
{
  voltsTime = millis(); // Grab Current Time
  voltsInterval = voltsTime - voltsLast; // Calculate interval between this and last event
  if (voltsInterval>30000 or voltsLast == 0) { // update every minute
    BatteryReadValue = analogRead(BatteryReadPin);
    floatVolts = (BatteryReadValue*1.7/100);
    lcd.setCursor(4, 2);
    lcd.print(txt71);
    lcd.setCursor(2, 2);
    lcd.print(floatVolts, 2);
    voltsLast = voltsTime; //set up for next event
  }
}   

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//stuff above is for testing using the Display Comment out if not needed  
//-----------------------------------------------------------------------------
uint32_t TimerOverFlow(uint32_t currentTime)
{
    return (currentTime + CORE_TICK_RATE*(1));//the Core Tick Rate is 1ms
}

//------------------ Debug data output ------------------------------

void    serialDump()
{
    loopStartTime   = millis();
    loopsPerSecond  = loopCount - lastLoopCount;
    loopSpeed       = (float)1e6 / loopsPerSecond;
    lastLoopCount   = loopCount;

    Serial.print    ( "uptime: " );
    Serial.print    ( ++printCount );
    Serial.println  ( " seconds" );

    Serial.print    ( "loops per second:    " );
    Serial.println  ( loopsPerSecond );
    Serial.print    ( "loop execution time: " );
    Serial.print    ( loopSpeed, 3 );
    Serial.println  ( " uS" );

    Serial.print    ( "Freq Rx: " );
    
    Serial.println  ( frequency_tune - IF );
    Serial.println  ( RX_frequency );
    
    Serial.print    ( "Freq Tx: " );
    
    Serial.println  ( frequency - IF );
    Serial.println  ( TX_frequency );
    
    Serial.print    ( "RIT: " );
    Serial.println  (  RitFreqOffset );
    
    Serial.print    ( "BW: " );
    Serial.println  ( );
 //   Serial.println  (  RitFreqOffset );
 
    Serial.print    ( "BAND: " );
    Serial.println  ( );
//    Serial.println  (  RitFreqOffset );

    Serial.print    ( "MODE: " );
    Serial.println  ( );
//    Serial.println  (  RitFreqOffset );

    Serial.print    ( "STEP: " );
    Serial.println  (  Frequency_Step );
    
    Serial.println  ();

} // end serialDump()
// ==========================================================================================================================================

void writeSmeter()                 // WD9GYM modification - adjusted by W8CQD
{
  int  level;          
  smeterTime = millis(); // grab current time
  smeterInterval = smeterTime - smeterLast; // calculate interval between this and last event

    if (smeterInterval > 200) // ignore checks less than 200mS after initial edge
  {

    SmeterReadValue = analogRead(SmeterReadPin);   // read value of signal 0 - 1023
    level = map(SmeterReadValue, 1023, 0,41,0);    // constrain the value into db 0 - 40

    lcd.setCursor(11, 2);                      // print the s meter on line 4
    lcd.print("S      ");                       // blank out bars

    if (level > 30) level = 40;
    if (level > 20 && level < 31) level = 30;
    if (level > 10 && level < 21) level = 20;
    if (level > 7 && level < 10) level = 9;
    if (level > 5 && level < 8) level = 7;
    if (level > 3 && level < 6) level = 5;
    if (level > 1 && level < 4) level = 3;
    if (level > 0 && level < 2) level = 1;
    switch( level ){                           // write each bar required
    case 40:                                 // do not put break between case statements
      lcd.createChar(4, meter_s30);         // let the code fall through
      lcd.setCursor(17, 2); 
      lcd.write( 4 );
    case 30: 
      lcd.createChar(4, meter_s30);
    case 20: 
      if( level == 20 ) lcd.createChar(4, meter_s20); 
      lcd.setCursor(16, 2); 
      lcd.write( 4 );
    case 10: 
      lcd.createChar(3, meter_s10);
    case 9: 
      if( level == 9 ) lcd.createChar(3, meter_s9); 
      lcd.setCursor(15, 2); 
      lcd.write( 3 );
    case 7: 
      lcd.createChar(2, meter_s7);
    case 5: 
      if( level == 5 ) lcd.createChar(2, meter_s5); 
      lcd.setCursor(14, 2); 
      lcd.write( 2 );
    case 3: 
      lcd.createChar(1, meter_s3);
    case 1: 
      if( level == 1 ) lcd.createChar(1, meter_s1); 
      lcd.setCursor(13, 2); 
      lcd.write( 1 );
    case 0:
    default: 
      break;
    }
    smeterLast = smeterTime; // set up for next event
  }

}
/**********************************************
      decoder stuff
**********************************************/

////////////////////////////////
// translate cw code to ascii //
////////////////////////////////

void docode(){

        if (strcmp(code,".-") == 0) printascii(65);
	if (strcmp(code,"-...") == 0) printascii(66);
	if (strcmp(code,"-.-.") == 0) printascii(67);
	if (strcmp(code,"-..") == 0) printascii(68);
	if (strcmp(code,".") == 0) printascii(69);
	if (strcmp(code,"..-.") == 0) printascii(70);
	if (strcmp(code,"--.") == 0) printascii(71);
	if (strcmp(code,"....") == 0) printascii(72);
	if (strcmp(code,"..") == 0) printascii(73);
	if (strcmp(code,".---") == 0) printascii(74);
	if (strcmp(code,"-.-") == 0) printascii(75);
	if (strcmp(code,".-..") == 0) printascii(76);
	if (strcmp(code,"--") == 0) printascii(77);
	if (strcmp(code,"-.") == 0) printascii(78);
	if (strcmp(code,"---") == 0) printascii(79);
	if (strcmp(code,".--.") == 0) printascii(80);
	if (strcmp(code,"--.-") == 0) printascii(81);
	if (strcmp(code,".-.") == 0) printascii(82);
	if (strcmp(code,"...") == 0) printascii(83);
	if (strcmp(code,"-") == 0) printascii(84);
	if (strcmp(code,"..-") == 0) printascii(85);
	if (strcmp(code,"...-") == 0) printascii(86);
	if (strcmp(code,".--") == 0) printascii(87);
	if (strcmp(code,"-..-") == 0) printascii(88);
	if (strcmp(code,"-.--") == 0) printascii(89);
	if (strcmp(code,"--..") == 0) printascii(90);

	if (strcmp(code,".----") == 0) printascii(49);
	if (strcmp(code,"..---") == 0) printascii(50);
	if (strcmp(code,"...--") == 0) printascii(51);
	if (strcmp(code,"....-") == 0) printascii(52);
	if (strcmp(code,".....") == 0) printascii(53);
	if (strcmp(code,"-....") == 0) printascii(54);
	if (strcmp(code,"--...") == 0) printascii(55);
	if (strcmp(code,"---..") == 0) printascii(56);
	if (strcmp(code,"----.") == 0) printascii(57);
	if (strcmp(code,"-----") == 0) printascii(48);

	if (strcmp(code,"..--..") == 0) printascii(63);
	if (strcmp(code,".-.-.-") == 0) printascii(46);
	if (strcmp(code,"--..--") == 0) printascii(44);
	if (strcmp(code,"-.-.--") == 0) printascii(33);
	if (strcmp(code,".--.-.") == 0) printascii(64);
	if (strcmp(code,"---...") == 0) printascii(58);
	if (strcmp(code,"-....-") == 0) printascii(45);
	if (strcmp(code,"-..-.") == 0) printascii(47);

	if (strcmp(code,"-.--.") == 0) printascii(40);
	if (strcmp(code,"-.--.-") == 0) printascii(41);
	if (strcmp(code,".-...") == 0) printascii(95);
	if (strcmp(code,"...-..-") == 0) printascii(36);
	if (strcmp(code,"...-.-") == 0) printascii(62);
	if (strcmp(code,".-.-.") == 0) printascii(60);
	if (strcmp(code,"...-.") == 0) printascii(126);
	//////////////////
	// The specials //
	//////////////////
	if (strcmp(code,".-.-") == 0) printascii(3);
	if (strcmp(code,"---.") == 0) printascii(4);
	if (strcmp(code,".--.-") == 0) printascii(6);

}

/////////////////////////////////////
// print the ascii code to the lcd //
// one a time so we can generate   //
// special letters                 //
/////////////////////////////////////
void printascii(int asciinumber){

int fail = 0;
/*
if (rows == 4 and colums == 16)fail = -4; /// to fix the library problem with 4*16 display http://forum.arduino.cc/index.php/topic,14604.0.html
 
 if (lcdindex > colums-1){
  lcdindex = 0;
  if (rows==4){
	  for (int i = 0; i <= colums-1 ; i++){
		lcd.setCursor(i,rows-3);
		lcd.write(line2[i]);
		line2[i]=line1[i];
	  }
   }
  for (int i = 0; i <= colums-1 ; i++){
    lcd.setCursor(i+fail,rows-2);
    lcd.write(line1[i]);
	lcd.setCursor(i+fail,rows-1);
    lcd.write(32);
  }
 }
*/ 

 lcd.setCursor(0, 3);
 for (int i = 0; i < 19; i++) {
      lcdBuff[i] = lcdBuff[i + 1];
      lcd.print(lcdBuff[i]);
 }
 
 lcdBuff[19] = asciinumber;
 lcd.print(lcdBuff[19]);
// Serial.println(lcdBuff);
/*
 line1[lcdindex]=asciinumber;
 lcd.setCursor(lcdindex+fail,rows-1);
 lcd.write(asciinumber);
 lcdindex += 1;
*/
}

void updateinfolinelcd(){
/////////////////////////////////////
// here we update the upper line   //
// with the speed.                 //
/////////////////////////////////////

  int place;
 
  if (rows == 4){
   place = colums/2;}
  else{
   place = 2;
  }
	if (wpm<10){
		lcd.setCursor((place)-2,0);
		lcd.print("0");
		lcd.setCursor((place)-1,0);
		lcd.print(wpm);
		lcd.setCursor((place),0);
		lcd.print(" WPM");
	}
	else{
		lcd.setCursor((place)-2,0);
		lcd.print(wpm);
		lcd.setCursor((place),0);
		lcd.print(" WPM ");
	}
}

