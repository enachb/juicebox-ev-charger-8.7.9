/*
EMotorWerks JuiceBox - an open-source 15kW EVSE

Micro-Controller: Arduino Pro Mini 5V, (based on a ATmega328P-PU microcontroller)

this version is matching V8.0-8.7 boards

Basic code structure:
Startup:
* initialize pins
* set output power level based on trimpot
* set duty cycle to 0

In endless loop:
* check for J1772 state
* check for EV & diode presence
* check for EV requesting power (state C)
* close relay to provide power (this is optional and code will work if no relay is present)
* run loop with power until non-C state detected or a button pressed
*     measure current and increment energy meter
*     display major params on the screen (this is optional and code will work if no LCD is present)

Created Jan 2013 by Electric Motor Werks, Inc. / Valery Miftakhov, Copyright 2013+

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, version 3 of the License

In a nutshell, it says if you wish to modify and distribute any derivation of this code, 
you must also distribute your modifications, and you MUST make the complete source code available.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details: http://www.gnu.org/licenses/
*/

//------------------------------ MAIN SWITCHES -----------------------------------
//#define DEBUG // this results in many additional printouts
// #define DEBUG2 // even more printouts
// #define DEBUGGFI

// the following results in much more frequent reporting of data by JuiceBox to EmotorWerks servers
// PLEASE DO NOT USE in your JuiceBox UNLESS AUTHORIZED BY EMotorWerks - this overloads our servers
// and slows down the system for everyone. JuiceBoxes that consistently report more frequently than 
// every ~1 minute will be permanently banned from our network
// #define DEBUG_WIFI 

#define AC1075
const int R_C=120; // this is the value of the shunting resistor. see datasheet for the right value. 
const int V_AC_threshold=164; // normally 164 (midpoint between 120V and 208V
const int V_AC_sensitivity=180; // normally 180 (empirical)
// #define JB_WiFi // is WiFi installed & we are using WiFlyHQ library?
//#define JB_WiFi_simple // is WiFi installed and we are just pushing data?
// #define JB_WiFi_control // is this JuiceBox controllable with WiFi (through HTTP responses)
// #define LCD_SGC // old version of the u144 LCD - used in some early JuiceBoxes
 #define PCB_83 // 8.3+ version of the PCB, includes 8.6, 8.7 versions
 #define VerStr "V8.7.9" // detailed exact version of firmware (thanks Gregg!)
 #define GFI // need to be uncommented for GFI functionality
// #define trim120current
// #define BuzzerIndication // indicate charging states via buzzer - only on V8.7 and higher
//------------------------------- END MAIN SWITCHES ------------------------------

`#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

// EEPROM handler
#include <EEPROM.h>
#include "EEPROM_VMcharger.h"

// WiFi library mega slon
#include <SoftwareSerial.h>
#include <WiFlyHQ.h>

//-------------------- WiFi UDP settings --------------------------------------------------------------------
const char UDPpacketEndSig[2]="\n"; // what is the signature of the packet end (should match the WiFly setting)

// need this to remap PWM frequency
#include <TimerOne.h>

// our LCD library for 4D systems display (http://www.4dsystems.com.au/prod.php?id=121)
// 4D Systems in its infinite wisdom decided to completely change the command set in its new 
// release of the LCDs so we (and other countless developers) had to completely rewrite our
// LCD libraries
#ifdef LCD_SGC
  #include <uLCD_144.h>
  uLCD_144 *myLCD;
#else
  #include <uLCD_144_SPE.h>
  uLCD_144_SPE *myLCD;
#endif

byte LCD_on=0; // this defines base vs. premium versions
byte REMOTE_ON=0; // this tells us if a remote is present or not

//------------------ current sensor calibration - only for AC1075 for now -----------------------------------------------
// the current sensing is done using a simple diode rectifier. As such, there is natural non-linearity to readings
// this lookup table takes care of that. Alternative approach is a precision rectifier using an op amp but 
// that adds a bit in parts and cost so taking a software approach here
// one entry per 0.1V in observed voltage on A1 pin. Since we have a 3.3v zener on that pin, need only 32-element array
// array contains current value in 0.1A units
// if the sensing was purely linear, we'd expect ~18A/volt but the forward drop on diode varies 
const unsigned int AC1075_calibration[32]={0,0,5,28,49,70,93,116,139,
                                           162,185,205,227,250,270,290,310,
                                           332,355,376,400,424,448,474,500,
                                           525,549,575,600,625,650,675};
//------------------ END current sensor calibration ---------------------------------------------------------------------

//---------------- savings consts for displays 
const int gascost=350; // in cents per gallon
const int mpg=25; // mpg of the gasoline car
const int ecost=12; // in cents per kWhr
const int whpermile=300; // energy efficiency of the ecar
int savingsPerKWH; // will be recalced later
//---------------- end of the energy constants

//---------------- pin-out constants ----------------
//---------------- analog inputs
const byte pin_pV=0; // pilot signal sensed through a 3-element divider 
const byte pin_V=1; // input voltage 
const byte pin_C=2; // AC current - as measured by the current transformer
const byte pin_throttle=3; // wired to a trimpot on a board
// pins A4 / A5 reserved for SPI comms to RTC chip
#ifdef trim120current
  const byte pin_throttle120=5; // when RTC is not used, this is an input used to set 120V target current (0-30A range)
#endif

//---------------- digital inputs / outputs
const byte pin_sRX=2; // SoftSerial RX - used for LCD or WiFi (default)
const byte pin_sTX=4; // SoftSerial TX - used for LCD or WiFi (default)
// GFI trip pin - goes high on GFI fault, driven by the specialized circuit based on LM1851 
// has to be pin 3 as only pin 2 and 3 are available for interrupts on Pro Mini
const byte pin_GFI=3; 
const byte pin_inRelay=5; 
const byte pin_ctrlBtn_C=6; // control button 1 ("C" on the remote, receiver pin 2)
const byte pin_ctrlBtn_D=8; // control button 2 ("D" on the remote, receiver pin 3)
const byte pin_PWM=9; // J pilot PWM pin

// pulling this pin high will trigger WPS application on the wifi module - on premium units, 
// this is also tied to one of the buttons of the remote so no Arduino action is needed
const byte pin_WPS=10; // ("B" on the remote, receiver pin 1) 

const byte pin_ctrlBtn_A=11; // control button 3 ("A" on the remote, receiver pin 0) 
const byte pin_GFItest=12; // pin wired to a GFCI-tripping relay - for the periodic testing of the GFCI circuit & stuck relay detection
#ifdef BuzzerIndication
  const byte pin_buzzer=13; // 2kHz buzzer for audio signal (8.7 boards have a bug in wiring of buzzer - drilling is needed so by default the buzzer is not installed)
#endif
//---------------- END PINOUTS -----------------------

//============= NON-VOLATILE INFO  =====
struct config_t {
  unsigned long energy; // total energy during lifetime, in kWHr - assuming 100kwhrs every day, we need long here
  byte day;
  byte hour;
  byte mins;
  // IDs for linking the JB to customer's account - a bunch of random ints
  unsigned int IDstamp[10]; 
  byte outC_240; // current setting for 240V
  byte outC_120; // current setting for 120V
  byte starttime[2]; 
  byte endtime[2]; 
} configuration;
//=======================================

//==================================== calibration constants etc
const float Aref=5.; // should be close
float pV_min=-12.;
float V_J1772_pin_=0; // global pilot voltage
const float divider_pV_R=100./27.; // 100k over 27k
float V_Ard_pin_0;
//===============================================================

//========== define J1772 states ===============================
// defaults
const float def_state_A_Vmin=10.5, def_state_A_Vmax=14; 
const float def_state_B_Vmin=7.5, def_state_B_Vmax=10.5; 
const float def_state_C_Vmin=4.5, def_state_C_Vmax=7.5; 
const float def_state_D_Vmin=1.5, def_state_D_Vmax=4.5; 
const float def_state_E_Vmin=-1.5, def_state_E_Vmax=1.5; 
const float def_state_F_Vmin=-14., def_state_F_Vmax=-10.; 
// now adjusted for the actual voltages
float state_A_Vmin, state_A_Vmax; 
float state_B_Vmin, state_B_Vmax; 
float state_C_Vmin, state_C_Vmax; 
float state_D_Vmin, state_D_Vmax; 
float state_E_Vmin, state_E_Vmax; 
float state_F_Vmin, state_F_Vmax; 
#define STATE_INVALID 255
#define STATE_A 0
#define STATE_B 1 
#define STATE_C 2
#define STATE_D 3
#define STATE_E 4
#define STATE_F 5
//=========== end definition of J1772 states ===================

// these should be global vars  -----------------------------
unsigned int duty=0, set_duty=0;
const unsigned int PWM_res=1024;
const unsigned int PWM_FULLON=1024;
const unsigned int MAXDUTY=970; // <97% to stay in AC charging zone for J1772 standard

const float maxC=60; // max rated current
float inV_AC=0; // this will be measured
const float nominal_outC_240V=30; // 30A by default from a 240VAC line
const float nominal_outC_120V=15; // 15A by default from a 120VAC line
float outC=nominal_outC_240V; 
float power=0;
float energy=0; // how much energy went through - in kWHrs 

char str[64]; // main temp str buffer - do not expand beyond 64 - may run out of memory
char tempstr[24]; // scratchpad for text operations

byte GFI_tripped=0;
byte GFI_trip_count=0;

byte cycleVar=0;
int state=-1, prev_state=-1;
int min2nextrun;
// ------------- end global vars ---------------------------

//------------- timing parameters --------------------------
unsigned long timer=0, timer0=0, timer_sec=0;
long clock_offset=0; // this has to be a signed value
unsigned long sec_up=0; // uptime since last reboot
const byte GFIblankingtime=100; // mask GFI trips for this many milliseconds from relay's closing - anti-noise
unsigned int delta=0;

// sensor timings
const byte meas_cycle_delay=100; // in ms

// how often to report on status
// report in every cycle if in DEBUG mode
#ifdef DEBUG_WIFI
  const int type1_reportMask=10; // in standby mode, every 10 seconds
  const int type2_reportMask=10; // in run mode, every 10 second
#else
  const int type1_reportMask=600; // in standby mode, every 10 minutes
  const int type2_reportMask=60; // in run mode, every 1 minute
#endif

// start and end times by weekday
const char *daysStr[7]={"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};
// initialize the clock - assume no RTC and that we are getting turned on at the hour
byte day=5, hour=12, mins=0; // default day is Sat, time is noon, 0 min
//------------ end timing params ---------------------------


#ifdef JB_WiFi_simple
  SoftwareSerial wifiSerial(pin_sRX, pin_sTX);
#endif

#ifdef JB_WiFi
  SoftwareSerial wifiSerial(pin_sRX, pin_sTX);
  WiFly wifly;
#endif


//-------------------------------------- BUZZER CODE -----------------------------------------------------
int tmr2cnt = 0;
int tmr2cnt2 = 0;
int tmr2th=0; // this defines frequency of beeps; 0 disables the beeps altogether

ISR(TIMER2_COMPA_vect) { //timer2 interrupt 8kHz toggles pin
  tmr2cnt++;
  tmr2cnt2++;
  
  if(tmr2cnt==8000) {
    tmr2cnt=0;
    sec_up++; // uptime
    wdt_reset(); // pat the dog 1/sec
  }

#ifdef BuzzerIndication
  // disable buzzer by this one
  if(tmr2th==0) {
    digitalWrite(pin_buzzer, LOW);
    return;
  }
  
  if(tmr2cnt2>tmr2th/2) {
    if((tmr2cnt2%16)==14) {
      digitalWrite(pin_buzzer,HIGH);
    }      
    if((tmr2cnt2%16)==15) {
      digitalWrite(pin_buzzer,LOW);
    }
  } else {
    digitalWrite(pin_buzzer,LOW);
  }
  
  if(tmr2cnt2>tmr2th) tmr2cnt2=0; // reset to zero on overflow
#endif
}
//-------------------------------------- END BUZZER CODE -------------------------------------------------


void setup() {
  wdt_disable();
  
  // set digital input pins
  pinMode(pin_GFI, INPUT);
  pinMode(pin_ctrlBtn_A, INPUT);
  pinMode(pin_ctrlBtn_C, INPUT);
  pinMode(pin_ctrlBtn_D, INPUT_PULLUP);

  // set digital output pins
  pinMode(pin_PWM, OUTPUT);
  pinMode(pin_inRelay, OUTPUT);
  // pinMode(pin_WPS, OUTPUT); // do NOT do this if there is a remote installed!
  pinMode(pin_GFItest, OUTPUT);
#ifdef BuzzerIndication
  pinMode(pin_buzzer, OUTPUT);
#endif

  //---------------------------------- set up timers
  cli();//stop interrupts

  // use Timer1 library to set PWM frequency 
  // 10-bit PWM resolution
  Timer1.initialize(1000); // 1kHz for J1772
  Timer1.pwm(pin_PWM, 0); 
  
  //set timer2 interrupt at 8kHz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 249;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);

  sei();
  //---------------------------------- end timer setup
  
  //================= initialize the display ===========================================
#ifdef LCD_SGC
  *myLCD=uLCD_144(9600);
#else
  *myLCD=uLCD_144_SPE(9600);
#endif
  //================= finish display init ==============================================

  // check if the display started / is present
  // if not present, we will assume this is the Base edition
  LCD_on=myLCD->isAlive();
  
#ifdef JB_WiFi_simple 
  wifiSerial.begin(9600);
#endif

  // the time settings only valid in the PREMIUM edition
  // load day/hour from the configuration (EEPROM)
  EEPROM_readAnything(0, configuration);
  // reset to defaults if this is a first run OR an 'A' button is pressed on power-up
  if(int(configuration.energy)<0 || isBtnPressed(pin_ctrlBtn_A)) {
    configuration.energy=0;
    // set defaults for time of use
    configuration.starttime[0]=configuration.starttime[1]=0; // starting at midnight
    configuration.endtime[0]=24; // no time of day by default
    configuration.endtime[1]=24; // no time of day by default
    configuration.day=day;
    configuration.hour=hour;
    configuration.mins=mins;
    configuration.outC_240=30;
    configuration.outC_120=15;
  } else {
    day=configuration.day;
    hour=configuration.hour;
    mins=configuration.mins; 
  }
  if(int(configuration.IDstamp[0])<0) {
    randomSeed(analogRead(6)+int(micros())); // this should be random enough
    for(byte iii=0; iii<10; iii++) {
      configuration.IDstamp[iii]=random(9999);
    }
  }
  
  day=limit(day, 0, 6);
  hour=limit(hour, 0, 23);
  mins=limit(mins, 0, 60);

  // will need to add pull of the true RTC time from a WiFi module here 
#ifdef JB_WiFi_simple
  // enter command mode, run 'get time'
#endif
    
  // set the clock offset; later in code, #of sec from midnight can be calculated as
  //     sec_up-clock_offset
  clock_offset=sec_up-long(day*24+hour)*3600-long(mins)*60; 
  
  if(LCD_on) { // this is a PREMIUM edition with LCD
    myLCD->setOpacity(1);
    myLCD->setMode(1); // reverse landscape

    printClrMsg(F("Thank You for\nchoosing \nJ.u.i.c.e B.o.x !!!"), 5000, 0, 0x3f, 0);
  }
  
  // auto-sense the remote
  // button D pin is set as INPUT_PULLUP - this means that if there is no remote, it will read '1'
  // if there IS a remote, AND button D is NOT pressed at startup (why would it?), 
  // this input should be at zero
  if(!digitalRead(pin_ctrlBtn_D)) REMOTE_ON=1; 
  
  EEPROM_writeAnything(0, configuration);

  //---------------------------- calibrate state boundaries ---------------------------------------------
  // first, need to record a minimum value of the wave - needed for pilot voltage measurement later on
  // set output pin to negative rail
  setPilot(0); // this should produce a steady -12V signal on a pilot pin
  pV_min=read_pV(); // this is supposed to be -12V

  // now calibrate the pilot voltage thresholds based on the actual voltage of positive rail 
  // calibration is done at every power-up
  setPilot(PWM_FULLON); // this should produce a steady +12V signal on a pilot pin
  float pVcal=read_pV();
#ifdef DEBUG
  sprintf(str, "pV: %d", int(pVcal*1000)); 
  printJBstr(0, 9, 2, 0x1f, 0, 0, str);      
#endif  

  // set default thresholds for pilot signal levels
  state_A_Vmin=def_state_A_Vmin; state_A_Vmax=def_state_A_Vmax; 
  state_B_Vmin=def_state_B_Vmin; state_B_Vmax=def_state_B_Vmax; 
  state_C_Vmin=def_state_C_Vmin; state_C_Vmax=def_state_C_Vmax; 
  state_D_Vmin=def_state_D_Vmin; state_D_Vmax=def_state_D_Vmax; 
  state_E_Vmin=-1.5, state_E_Vmax=1.5; 
  state_F_Vmin=-14., state_F_Vmax=-10.; 
  
  // recalibrate the pilot sensing code. helps fight any possible temperature / aging drifts
  // but only do it if it's not too far off - this will prevent recalibration in case the power 
  // cycles while the JuiceBox is plugged into the car
  // note that this will mean that the JuiceBox would not be able to recalibrate if the pilot is more than 
  // 10% off (unlikely with a precision 12V regulator used and R-R op amp)
  if(pVcal>def_state_B_Vmax) {  
    pVcal/=12.; // calibration constant
    // now adjust boundaries for top being not 12V
    state_A_Vmin=def_state_A_Vmin*pVcal;  state_A_Vmax=def_state_A_Vmax*pVcal; 
    state_B_Vmin=def_state_B_Vmin*pVcal;  state_B_Vmax=def_state_B_Vmax*pVcal; 
    state_C_Vmin=def_state_C_Vmin*pVcal;  state_C_Vmax=def_state_C_Vmax*pVcal; 
    state_D_Vmin=def_state_D_Vmin*pVcal;  state_D_Vmax=def_state_D_Vmax*pVcal; 
    state_E_Vmin=-1.5, state_E_Vmax=1.5; 
    state_F_Vmin=-14., state_F_Vmax=-10.; 
  }
  
  //-------------------- ONE-TIME: determine input / output AC voltage
  // this has to run before attaching interrupt to the GFI break pin
  // set the baseline 
  // V_Ard_pin_0=analogRead(pin_V)*Aref/1024.;
  V_Ard_pin_0=0; // DEBUG - override for now
  
  // now check for a stuck relay and measure input voltage 
#ifndef PCB_81
  GFI_tripped=0;
  // force the GFI pin
  digitalWrite(pin_GFItest, HIGH);
#ifdef GFI
  delay(100); 
  // by now, if the trip occurred, the GFI trip flag should be set
  if(GFI_tripped==1) {
    // we have a stuck relay, throw an error
    printErrorMsg(F("STUCK RELAY! \nContact us\nExiting..."), 30000);
    return; // break from loop() which will be called back a moment later
  }
#endif
  // turn on the main relay
  setRelay(HIGH);
  // wait for settling (RC on the pin is 0.1s so need to wait at least for 0.3s
  // but not too long or we will burn out the 10k resistor...
  delay(300);
#endif
  inV_AC=read_V();    
  digitalWrite(pin_GFItest, LOW);
  setRelay(LOW);  
  
  // attach interrupt on pin 3 (GFI)
#ifdef GFI
  attachInterrupt(1, GFI_break, RISING);
#endif

  // prep for calc of the savings
  getSavingsPerKWH(gascost, mpg, ecost, whpermile);
  
  // set watchdog - http://tushev.org/articles/arduino/item/46-arduino-and-watchdog-timer, http://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
  wdt_enable(WDTO_8S); // longest is 8S
  
  myclrScreen();

  // initialize in state A - EVSE ready
  setPilot(PWM_FULLON);
}


//============================================= MAIN LOOP ============================================
void loop() {
  // reset GFI trip status so we can retry after GFI timeout
  // GFI is checked in the end of this cycle - by that time, a few hundreds ms pass
  GFI_tripped=0; 
      
  // check if the car is there and requesting power
  prev_state=state;
  state=getState(); // this is a blocking call for <100ms
  
  // manage state changes
  if(state!=prev_state) {
    myclrScreen();
    timer=millis(); // start timer
    timer0=timer; // remember the start of charge
    
    if(state==STATE_C) {
      // entering charging state - check for diode
      setPilot(PWM_FULLON/2);
      if(read_pV()>-1.5) {
        state=STATE_F; // diode check failure!
      } 
      energy=0; // reset energy counter for this cycle  
    }

    if(prev_state==STATE_C) { // exiting state C - charging
      // store things in EEPROM so we can track total lifetime energy / savings and 
      // also are immune to short power interruptions
      configuration.energy+=energy; // keep track of the total energy transmitted through the EVSE
      configuration.day=dayOfWeek();
      configuration.hour=hourOfDay();
      configuration.mins=minsOfHour();
      EEPROM_writeAnything(0, configuration);
    }
  } // end state transition check


  //-------------------------------- process states
  if(state==STATE_A) {
    setRelay(LOW); // relay off
    setPilot(PWM_FULLON);
  }
    
  if(state==STATE_B) {
    setRelay(LOW); // relay off
    // check if we are ok to run - but ONLY if there is a remote control to allow override
    min2nextrun=timeToNextRun();
    if(min2nextrun>0 && !isBtnPressed(pin_ctrlBtn_C) && REMOTE_ON) { 
      sprintf(str, "Wait %d min    ", min2nextrun); 
      printJBstr(0, 12, 1, 0x1f, 0x3f, 0x1f, str);  
      setPilot(PWM_FULLON);
    } else {
      // clear part of screen
      printJBstr(0, 12, 1, 0x1f, 0x3f, 0x1f, F("               "));    
      setOutC();
      setPilot(duty);
    }
  }
    
  if(state==STATE_C) {
    setOutC();
    setPilot(duty);
    setRelay(HIGH); // relay on

    // process energy metering
    float outC_meas=read_C();
    power=outC_meas*inV_AC/1000; // in KW

    delta=int(millis()-timer);
    timer=millis();
    energy+=power*delta/1000/3600; 

    // print real-time stats
    printTime();    
    sprintf(str, "Power: %d.%01d KW  ", int(power), int(power*10)%10); 
        printJBstr(0, 2, 2, 0x1f, 0x3f, 0, str);   
    sprintf(str, "Time: %d min  ", int((timer-timer0)/1000)/60); 
        printJBstr(0, 3, 2, 0x1f, 0x3f, 0, str);   
    // also show energy cost in this one
    // use US average cost per http://www.eia.gov/electricity/monthly/epm_table_grapher.cfm?t=epmt_5_6_a - $0.12/kwhr
    sprintf(str, "%d.%01d KWH ($%d.%02d) ", int(energy), int(energy*10)%10, int(energy/8), int(energy/8*100)%100 ); 
        printJBstr(0, 5, 2, 0x1f, 0x3f, 0, str);   
    sprintf(str, "%dV, %dA (%d) ", int(inV_AC), int(outC_meas), int(outC)); 
        printJBstr(0, 7, 2, 0x1f, 0x3f, 0, str);   
        
    // print button menu
    printJBstr(0, 9, 2, 0, 0, 0x1f, F("A=outC+, D=outC- \nB=WPS")); 
    if(isBtnPressed(pin_ctrlBtn_A)) {
      if(inV_AC>160) configuration.outC_240++;
      else configuration.outC_120++;
    }
    if(isBtnPressed(pin_ctrlBtn_D)) {
      if(inV_AC>160) configuration.outC_240--;
      else configuration.outC_120--;
    }

    // send out a report to MotherShip via WiFi if on
#ifdef JB_WiFi_simple
    if( int(sec_up-timer_sec) > type2_reportMask ) {
      timer_sec=sec_up;
      sprintf(str, "V%d,L%d,E%d,A%d,P%d", int(inV_AC), int(configuration.energy+energy), int(energy*10), int(outC_meas*10), int(power*10));
      sendWiFiMsg(str);
    }
#endif    
  } // end state_C
  
  if(state==STATE_D) {
    // printClrMsg(F("Vehicle requested\nVENTILATED power!\nExiting..."), 1000, 0x1f, 0x3f, 0);
    setRelay(LOW); // relay off
  }
  
  if(state==STATE_E || state==STATE_F || state==STATE_INVALID) {
    printClrMsg(F("Abnormal State!"), 1000, 0x1f, 0x3f, 0);
    setRelay(LOW); // relay off
  }  
          
  // display standby
  if(state==STATE_A || state==STATE_B) {
    // load configuration - holding total energy and calibration constants
    // need to load here in loop() function as things will be written at end of each charge and we need to reload
    EEPROM_readAnything(0, configuration);
  
    // set the output current - can be changed by trimpot or remote without a restart
    // need this here so we have an echo on user input
    setOutC(); 
    
    int savings=int(configuration.energy*savingsPerKWH/100);
    
    printTime();
    
    if(LCD_on) {
      switch(cycleVar) {
        case 2:
          cycleVar=0;
        case 0:
          myLCD->printStr(0, 2, 2, 0x1f, 0x3f, 0, F("READY -   "));
          break;
        case 1:
          myLCD->printStr(0, 2, 2, 0x1f, 0x3f, 0, F("READY |   "));
          break;
        default: break;
      }
      cycleVar++;
  
      // output some key standby info - this is the default screen before charging commences
      sprintf(str, "Life: %d KWH", int(configuration.energy)); myLCD->printStr(0, 4, 2, 0, 0x3f, 0x1f, str);
      sprintf(str, "Saved: %d$", savings); myLCD->printStr(0, 5, 2, 0, 0x3f, 0x1f, str);
      sprintf(str, "Last: %d.%01d KWH", int(energy), int(energy*10)%10); myLCD->printStr(0, 7, 2, 0x1f, 0, 0x1f, str);
      sprintf(str, "Set: %dV, %dA    ", int(inV_AC), int(outC)); myLCD->printStr(0, 8, 2, 0x1f, 0x3f, 0x1f, str);
  #ifdef DEBUG2
      int reading=analogRead(pin_C);
      delay(8);
      reading+=analogRead(pin_C);
      sprintf(str, "A2:%d=%dA  ", int(reading*5./2/10.24), int(read_C()*10)); myLCD->printStr(0, 9, 2, 0x1f, 0x3f, 0x1f, str);
      delay(500); // before overwriting below
  #endif
  
      // print button menu
      printJBstr(0, 10, 2, 0, 0, 0x1f, F("A=MENU, B=WPS \nC=FORCE START")); 
      
      if(isBtnPressed(pin_ctrlBtn_A)) ctrlMenu();
  
    } else {
      
      // no LCD
      sprintf(str, "%dV, %dA", int(inV_AC), int(outC));
      Serial.println(str);
  #ifdef DEBUG
      // print ID - only in non-LCD mode so not to clutter anything
      for(int iii=0; iii<10; iii++) {
        Serial.print(configuration.IDstamp[iii]); // 10-50 digit ID - unique to each JuiceBox
      }
      Serial.println();
      sprintf(str, "    pilot=%d, inACpin=%d", int(V_J1772_pin_*1000), int(analogRead(pin_V)*Aref));
      Serial.println(str);
  #endif
  
    }
    
    // send out a report to MotherShip via WiFi if WiFi is enabled
  #ifdef JB_WiFi_simple
    if( int(sec_up-timer_sec) > type1_reportMask ) {
      timer_sec=sec_up; // reset start of timer
      sprintf(str, "V%d,L%d,S%d", int(inV_AC), configuration.energy, savings);
      sendWiFiMsg(str);
    }
  #endif
  
  }

  delay(meas_cycle_delay); // reasonable delay for screen refresh

#ifdef GFI
  // check GFI flag (if a trip is detected, this flag would be set via the special interrupt)
  if(GFI_tripped) {
    printClrMsg(F("GFI tripped!\nRetrying in 15 min..."), 300, 0x1f, 0x3f, 0);
    GFI_trip_count++; // allowed max of 4; if more than 4, need 2 user inputs to override
    if(GFI_trip_count>4) {
      // wait for user to unplug; since the user then will have to re-plug to re-energize the system, this can be considered 2 actions
      printClrMsg(F("4th GFI trip!\nUnplug / re-plug\nto resume"), 1000, 0x1f, 0x3f, 0);
      while(getState()!=STATE_A);
    } else {
#ifndef DEBUGGFI                 
      delaySecs(900); // 15 min
#endif
    }  
  }
#endif

} // end loop()


//=================================================== FUNCTIONS ==============================================
// interrupt - break on GFI trigger - breaks on RISING signal on pin 3 (transistor end of the relay)
void GFI_break() {
  // mask if within certain time from closing the main relay
  if(millis()>timer0 && millis()-timer0<GFIblankingtime) return; // first term protects from millis() overflow
  
  // check every 2mS for 20ms - if not a single additional trips, ignore
  for(byte i=0; i<10; i++) {
    delayMicroseconds(2000);
    if(digitalRead(pin_GFI)==HIGH) {
      GFI_tripped=1;
      // before tripping relay, kill pilot - maybe the onboard charger will drop current quickly and help save our relay contacts...
      setPilot(0); // -12V on pilot - ABNORMAL STATE. Compliant chargers should stop
      setRelay(LOW); // kill power NOW. generally, relay will take ~20-25ms to open
      break;
    }
  }
}

// operate output relay
void setRelay(byte state) {
  digitalWrite(pin_inRelay, state);
}


void setPilot(int _duty) {
  set_duty=_duty;
  Timer1.setPwmDuty(pin_PWM, _duty);
}


// set the pilot duty corresponding to the output current based on the pot or stored settings
// use default current setting if pin is grounded
void setOutC() {
  float minThrottle=0.05;
  float throttle=0;

  // different trimpot depending on voltage
  if(inV_AC==120) {
#ifdef trim120current  
    if(configuration.outC_120>0 && LCD_on) {
      outC=configuration.outC_120;
    } else {
      throttle=analogRead(pin_throttle120)/1024.;
      if(throttle>minThrottle) { // if something is set on the throttle pot, use that instead of the default outC
        outC=throttle*nominal_outC_120V*2; // full range is 2x of nominal
      }
    }
#else
    outC=min(nominal_outC_120V, outC);
#endif
  } else {
    // 208V+ setting
    if(configuration.outC_240>0 && LCD_on) {
      outC=configuration.outC_240;
    } else {    
      throttle=analogRead(pin_throttle)/1024.;
      if(throttle>minThrottle) { // if something is set on the throttle pot, use that instead of the default outC
        outC=throttle*maxC;
      }
    }
  }

  // per J1772 standard:
  // 1% duty = 0.6A until 85% duty cycle
  // after that, 1% = 2.5A up to 96%
  if(outC<51) {
    duty=PWM_res*outC/60.;
  } else {
    duty=PWM_res*(0.64+outC/250.);
  }
  
  if(duty>MAXDUTY) duty=MAXDUTY;
}


// this will block for ~200ms due to read_pV()
int getState() {
  byte mode=1; // PWM is on 
  if(set_duty==PWM_FULLON) mode=0; // PWM is off
  
  float pV=read_pV();
  
#ifdef DEBUG
//  sprintf(str, "raw pV=%d, ", int(pV*1000));
//  printJBstr(0, 10, 2, 0x1f, 0, 0, str);      
#endif

  // in mode=1, the state is measured while pilot is oscillating so need to recalc
  // pV=pV_min*(1-duty)+pV_max*duty
  // so pV_max=(pV-pV_min*(1-duty))/duty
  if(mode==1) pV=((pV-pV_min)*PWM_res+pV_min*duty)/duty;
  
#ifdef DEBUG
//  sprintf(str, "calc pV=%d", int(pV*1000));
//  printJBstr(0, 11, 2, 0x1f, 0, 0, str);      
#endif

  if(pV>state_A_Vmin && pV<=state_A_Vmax) return STATE_A;
  if(pV>state_B_Vmin && pV<=state_B_Vmax) return STATE_B;
  if(pV>state_C_Vmin && pV<=state_C_Vmax) return STATE_C;
  if(pV>state_D_Vmin && pV<=state_D_Vmax) return STATE_D;
  if(pV>state_E_Vmin && pV<=state_E_Vmax) return STATE_E;
  if(pV>state_F_Vmin && pV<=state_F_Vmax) return STATE_F;

  return STATE_INVALID;
}


// read the average pilot voltage - this is a BLOCKING CALL (200ms)
// time constant of the RC filter: 27k/2 * 3.3uF = ~0.04s - enough to smooth 1kHz signal
float read_pV() {
  // ensure settling of the signal before measurement
  delay(100); // this is ~2.5 time constants of the RC filter on this pin - measured value should be within 2% of its settled value
  int reading=analogRead(pin_pV); // this takes 100uS
  // for anti-noise, read 180 degree off the prev reading 
  // (integer number of milliseconds + 500 uS (half-PWM-period) - ADC conversion time)
  delayMicroseconds(2500); 
  reading+=analogRead(pin_pV);
  float V_Ard_pin=reading*Aref/1024./2;

  V_J1772_pin_=(2*V_Ard_pin-5)*divider_pV_R+V_Ard_pin;

  return V_J1772_pin_;
}

// read the average input AC voltage 
// this function should ONLY BE CALLED in setup()
// time constant of the RC filter: 27k * 3.3uF = ~0.09s - enough to smooth 60Hz signal
float read_V() {
  float V_AC=240; // default is 240
  
  float V_Ard_pin=analogRead(pin_V)*Aref/1024.;
  delay(8); // measure 180 degrees away by AC phase to smooth out any remaining ripple on A1 pin
  V_Ard_pin+=analogRead(pin_V)*Aref/1024.;
  V_Ard_pin/=2;
  
#ifdef PCB_81
  // for PCB versions before 8.3, 
  // THIS FEATURE IS IN BETA AND MAY NOT WORK ON THE FIRST VERSION OF THE BASE BOARDS WITHOUT TWEAKING FIRMWARE
  // specifically, you may need to tweak the voltage threshold between 120V and 240V input voltage 
  //   (line starting with 'if(V_Ard_pin >' below). (1) connect JuiceBox to 120V, measure the voltage on pin A1 of the Arduino
  //   (2) connect JuiceBox to 240V, measure the voltage on pin A1. Set the threshold to the voltage in the middle between 
  //   these two values
  // with 200k resistor from AC rectified line, we have 0.8mA peak primary current at 120VAC
  //     and 1.6mA at 240VAC
  // according to PC817X1 opto's datasheet, CTR is 80-160% at 5mA
  // typical curve suggests 80% of that at 2.5mA, 50% at 1mA
  // therefore, we have a secondary peak current of 0.3-0.6mA at 120VAC, 1-2mA at 240VAC
  // this corresponds to a secondary voltage drop: 0.3-0.6V or 1-2V per 1k of secondary resistance
  //               (actually clipped to 5V since we are using 5v supply)
  // also, need to take into account that we see the significant current only at the positive peak of AC wave
  // generally, for ~1/4 of the period for 120VAC and 1/3rd for 240VAC
  // finally, the average drop within over the drop time is ~1/2 of the peak drop
  // putting it all together, we expect average drop of 0.04-0.08V per 1k for 120VAC and 0.16-0.32V per 1k for 240VAC 
  //               (with some clipping starting at 3-5k - really becoming visible at 5-7k)  
  // Example: 4.7k secondary - 0.2-0.4V and 0.8-1.5V drops
  // Example: 10k secondary - 0.4-0.8V and 1.2-2.2V drops
  // Using 10k secondary, place mid-point at 1V drop, or 4V threshold
  // cap at 4.9V to prevent from defaulting to 120V in case when no PC817 installed at all
  if(V_Ard_pin > 3.5 && V_Ard_pin<4.9) V_AC=120;
#endif

#ifdef PCB_83
  // in 8.3 and later, the implementation changed to measurement using the GFCI current sensor
  // ~200x division factor for voltage (total gain of test loop is ~1.2e-2)
  //     (306 from RMS voltage on sensor = 680x on the opamp, 0.5x due to half-wave rectification, 0.9x for converting to average from RMS)
  //     (3.9e-5x from 0.39V/A sensor sensitivity with 390R shunt resistor, 0.0001A/V voltage-to-current conversion on a 10k resistor)
  //

  // if no GFI installed, cannot measure voltage - default to 240V 
#ifdef GFI
  V_AC=V_AC_sensitivity*(V_Ard_pin-V_Ard_pin_0);
#else
  V_AC=240;
#endif

#ifdef DEBUG
  sprintf(str, "V_AC: %d", int(V_AC));
  printJBstr(0, 9, 2, 0x1f, 0, 0, str);   
#endif
  
  if(V_AC < V_AC_threshold) { // midpoint between 120 and 208V
    V_AC=120;
  } else if(V_AC < 220) { // midpoint between 208V and 240V, allowing for sag of 4V
    V_AC=208;
  } else {
    V_AC=240; // default fall-back value
  }
  
#endif

  return V_AC; 
}

// read the AC current via the current transformer
// in the absense of the current transformer this will return zero
// RC constant defined by R11 C5 = 27k * 3.3uF = 90ms, or >5 line periods
float read_C() {
#ifdef AC1075
  const int Te=1000; // # of turns
#endif
#ifdef CT_8349-1500
  const int Te=1500; // # of turns
#endif
#ifdef CT_8420-1000
  // assume 8420-1000 current transformer (50A max current, 20-1000 Hz working range)
  const int Te=1018; // # of turns
#endif
#ifdef CT_3100
  // assume 3100 current transformer (75A max current, 20-1000 Hz working range)
  const int Te=3100; // # of turns
#endif

  // read the rectified voltage of the half-wave from the transformer
  // average between 2 readings 180 degree off each other
  int reading=analogRead(pin_C);
  delay(8);
  reading+=analogRead(pin_C);
  // this assumes an RC filter before Arduino pon with time constant >> line period and impedance >> R
  float V_C=reading*Aref/2/1024; 

#ifdef AC1075
  // use a table lookup
  int index=floor(V_C*10);
  float remainder=V_C*10-index;
  if(index>31) return 75.; // prevent from array overflow
  float coeff=float(AC1075_calibration[index+1]-AC1075_calibration[index]); // 0.1V step
  return float(AC1075_calibration[index]+remainder*coeff)/10.; // linear extrapolation
#else
  // use a crude linear approximation
  if(V_C>0.2) {
    V_C+=0.2+V_C/6; // assuming 0.2V diode drop
  } else V_C=0;

  // *2 for half-wave rectification, 1.11 for conversion of average into RMS
  // for AC1050-1075 this corresponds to ~18A/V
  return V_C*Te/R_C*2.22;  
#endif
}


//------------------------------ control MENUs -----------------------------------------
// this is generally called by pressing 'A' button on the remote
void ctrlMenu() {
  byte pos=0; // current position
  const byte state_MENU=0xFF;
  const byte state_EXIT=0x00;
  const byte state_OUTC=0x01;
  const byte state_PRINTID=0x02;
  const byte state_TIME=0x03;
  const byte state_TIMEOFCHARGE_W_S=0x04;
  const byte state_CAL_C=0x05;
  const byte state_CAL_V=0x06;
  const byte maxpos=4; // index of the last valid menu item

  const byte state_TIMEOFCHARGE_W_E=0x41;
  const byte state_TIMEOFCHARGE_WE_S=0x42;
  const byte state_TIMEOFCHARGE_WE_E=0x43;
  
  byte mstate=state_MENU;
  byte prev_mstate=99;
  byte btn=0xFF; // which button was pressed - this will be set to the pin number
  byte x=0; // target position / variable
  
  while(1) {
    if(mstate!=prev_mstate) {
      myclrScreen();
      btn=0xFF; // reset button state
    } else {
      // block for button if the state is the same 
      btn=waitForBtn();
    }
    prev_mstate=mstate;
    
    switch(mstate) {
      case state_MENU:
        // show menu
        pos=0;
        if(btn==pin_ctrlBtn_A) {
          if(x==0) {
            x=maxpos;
          } else {
            x--;
          }
        }
        if(btn==pin_ctrlBtn_D) x++;
        // by this time, pos=<number of menu entries>
        if(x>maxpos) x=0; // wrap around to top
        printJBstr(0, 2+pos, 2, 0, ( x==pos ? 0x3f : 0x1f ), 0, F("EXIT")); pos++;
        printJBstr(0, 2+pos, 2, 0, ( x==pos ? 0x3f : 0x1f ), 0, F("SET CURRENT")); pos++;
        printJBstr(0, 2+pos, 2, 0, ( x==pos ? 0x3f : 0x1f ), 0, F("PRINT ID")); pos++;
        printJBstr(0, 2+pos, 2, 0, ( x==pos ? 0x3f : 0x1f ), 0, F("SET CLOCK")); pos++;
        printJBstr(0, 2+pos, 2, 0, ( x==pos ? 0x3f : 0x1f ), 0, F("SET TIME OF USE")); pos++;
        if(btn==pin_ctrlBtn_C) mstate=x;
        break;
      
      case state_EXIT:
        return;
        break; 
      
      case state_OUTC:
        // set output curent 
        if(btn==pin_ctrlBtn_A) outC++;
        if(btn==pin_ctrlBtn_D) outC--;
        sprintf(str, "Setting for \noutV=%dV ", int(inV_AC));
        printJBstr(0, 2, 2, 0x1f, 0x3f, 0x1f, str);
        sprintf(str, "outC: %dA ", int(outC));
        printJBstr(0, 5, 2, 0x1f, 0x3f, 0, str);
        if(btn==pin_ctrlBtn_C) {
          if(inV_AC==120) {
            configuration.outC_120=outC;
          } else {
            configuration.outC_240=outC;
          }
          // need to write to config the new value
          EEPROM_writeAnything(0, configuration);

          mstate=state_MENU; // back to menu
        }
        break;      
      
      case state_PRINTID:
        // print JuiceBox ID if requested - required to associate the JuiceBox with online account
        // this is button D on the remote!
        for(int iii=0; iii<5; iii++) {
          sprintf(str, "%u %u", configuration.IDstamp[iii*2], configuration.IDstamp[iii*2+1]);
          printJBstr(0, 2+iii, 1, 0x1f, 0x3f, 0x1f, str);
        }  
        if(btn==pin_ctrlBtn_C) mstate=state_MENU;
        break;

      case state_TIME:
        // setting time
        day=dayOfWeek();
        hour=hourOfDay();
        mins=minsOfHour();
        // user setup of the day - 0=Monday
        printClrMsg(F("Set day:\n'A' to change,\n'C' to select"), 50, 0, 0x3f, 0);
        myLCD->printStr(0, 6, 2, 0x1f, 0x1f, 0, daysStr[day]); // print starting point
        while(1) {
          if(isBtnPressed(pin_ctrlBtn_A)) {
            day++;
            if(day>6) day=0;
            myLCD->printStr(0, 6, 2, 0x1f, 0x1f, 0, daysStr[day]);
            delay(100); // avoid double-button-press
          }
          if(isBtnPressed(pin_ctrlBtn_C)) {
            while(isBtnPressed(pin_ctrlBtn_C));
            break; // exit on timeout
          }
          delay(50);
        }      
        // user setup of the time - in 24 hour clock
        printClrMsg(F("Set time:\n'A' to change,\n'C' to select"), 50, 0, 0x3f, 0);
        sprintf(str, "Hour: %02d  ", hour); myLCD->printStr(0, 7, 2, 0x1f, 0x1f, 0, str); // print starting point
        while(1) {
          if(isBtnPressed(pin_ctrlBtn_A)) {
            hour++;
            if(hour>23) hour=0;
            sprintf(str, "Hour: %02d  ", hour); myLCD->printStr(0, 7, 2, 0x1f, 0x1f, 0, str); // print starting point
            delay(100); // avoid double-button-press
          }
          if(isBtnPressed(pin_ctrlBtn_C)) {
            while(isBtnPressed(pin_ctrlBtn_C));
            break; // exit on timeout
          }
          delay(50);
        }
        sprintf(str, "Minute: %02d  ", mins); myLCD->printStr(0, 7, 2, 0x1f, 0x1f, 0, str); // print starting point
        while(1) {
          if(isBtnPressed(pin_ctrlBtn_A)) {
            mins++;
            if(mins>59) mins=0; // 60 minutes in an hour
            sprintf(str, "Minute: %02d  ", mins); myLCD->printStr(0, 7, 2, 0x1f, 0x1f, 0, str); // print starting point
            delay(100); // avoid double-button-press
          }
          if(isBtnPressed(pin_ctrlBtn_C)) {
            break; // exit on timeout
          }
          delay(50);
        }        
        // reset the clock offset
        clock_offset=sec_up-long(day*24+hour)*3600-mins*60; 
        
        // store current date & time in EEPROM
        configuration.day=day;
        configuration.hour=hour;
        configuration.mins=mins;
        EEPROM_writeAnything(0, configuration);
        
        mstate=state_MENU;
        break;

      case state_TIMEOFCHARGE_W_S:
        // setting allowed time of charge
        myLCD->printStr(0, 6, 2, 0x1f, 0x3f, 0x1f, F("Set all to 0-24 to\ndisable timer"));
        // allow editing 2 types of days (weekday & weekend), start and end time for every type
        if(btn==pin_ctrlBtn_A && configuration.starttime[0]<23) configuration.starttime[0]++; // operate on Monday's time
        if(btn==pin_ctrlBtn_D && configuration.starttime[0]>0) configuration.starttime[0]--;
        sprintf(str, "Weekday:\nstart charge\n\n %02d:00 ", configuration.starttime[0]); 
        myLCD->printStr(0, 2, 2, 0, 0x3f, 0, str);
        if(btn==pin_ctrlBtn_C) mstate=state_TIMEOFCHARGE_W_E;
        break;
      case state_TIMEOFCHARGE_W_E:
        // setting allowed time of charge
        // allow editing 2 types of days (weekday & weekend), start and end time for every type
        if(btn==pin_ctrlBtn_A && configuration.endtime[0]<24) configuration.endtime[0]++; // operate on Monday's time, allow to go to 24th hour
        if(btn==pin_ctrlBtn_D && configuration.endtime[0]>0) configuration.endtime[0]--;
        sprintf(str, "Weekday:\nend charge\n\n %02d:00 ", configuration.endtime[0]); 
        myLCD->printStr(0, 2, 2, 0, 0x3f, 0, str);
        if(btn==pin_ctrlBtn_C) mstate=state_TIMEOFCHARGE_WE_S;
        break;
      case state_TIMEOFCHARGE_WE_S:
        // setting allowed time of charge
        // allow editing 2 types of days (weekday & weekend), start and end time for every type
        if(btn==pin_ctrlBtn_A && configuration.starttime[1]<23) configuration.starttime[1]++; // operate on Saturday's time
        if(btn==pin_ctrlBtn_D && configuration.starttime[1]>0) configuration.starttime[1]--; 
        sprintf(str, "WeekEND\nstart charge\n\n %02d:00 ", configuration.starttime[1]); 
        myLCD->printStr(0, 2, 2, 0, 0x3f, 0, str);
        if(btn==pin_ctrlBtn_C) mstate=state_TIMEOFCHARGE_WE_E;
        break;
      case state_TIMEOFCHARGE_WE_E:
        // setting allowed time of charge
        // allow editing 2 types of days (weekday & weekend), start and end time for every type
        if(btn==pin_ctrlBtn_A && configuration.endtime[1]<24) configuration.endtime[1]++; // operate on Saturday's time, allow to go to 24th hour
        if(btn==pin_ctrlBtn_D && configuration.endtime[1]>0) configuration.endtime[1]--; 
        sprintf(str, "WeekEND\nend charge\n\n %02d:00 ", configuration.endtime[1]); 
        myLCD->printStr(0, 2, 2, 0, 0x3f, 0, str);
        if(btn==pin_ctrlBtn_C) {
          // write out into the EEPROM config 
          EEPROM_writeAnything(0, configuration);    
          mstate=state_MENU;
        }
        break;

      default: 
        break;
    }
    
    // print button menu
    printJBstr(0, 9, 2, 0, 0, 0x1f, F("A=UP, D=DOWN \nB=WPS, C=SELECT"));    

    // delay(50); // some short delay between re-renderings
  }
}

// process remote button presses
// this is a BLOCKING call
byte waitForBtn() {
  while(1) {
    if(isBtnPressed(pin_ctrlBtn_A)) return pin_ctrlBtn_A;
    // no B button - reserved for WPS
    if(isBtnPressed(pin_ctrlBtn_C)) return pin_ctrlBtn_C;
    if(isBtnPressed(pin_ctrlBtn_D)) return pin_ctrlBtn_D;
    delay(10); // very small delay to not overload CPU too much
  }
}
//------------------------------ END control MENUs -------------------------------------


//------------------------------ printing help functions -------------------------
void printJBstr(byte col, byte row, byte font, byte c1, byte c2, byte c3, const __FlashStringHelper *fstr) {
  if(LCD_on) {
    myLCD->printStr(col, row, font, c1, c2, c3, fstr);
  } else {
    Serial.print("    ");
    Serial.println(fstr);
  }
}
void printJBstr(byte col, byte row, byte font, byte c1, byte c2, byte c3, const char *sstr) {
  if(LCD_on) {
    myLCD->printStr(col, row, font, c1, c2, c3, sstr);
  } else {
    Serial.print("    ");
    Serial.println(sstr);
  }
}
void printClrMsg(const __FlashStringHelper *fstr, const int del, const byte red, const byte green, const byte blue) {
  myclrScreen();
  printJBstr(0, 2, 2, red, green, blue, fstr);      
  delay(del);
}
void printClrMsg(const char *str, const int del, const byte red, const byte green, const byte blue) {
  myclrScreen();
  printJBstr(0, 2, 2, red, green, blue, str);      
  delay(del);
}
void printErrorMsg(const __FlashStringHelper *fstr, const int del) {
  printClrMsg(fstr, 30000, 0x1f, 0x3f, 0);
  // also send a message to server if WiFI is enabled
#ifdef JB_WiFi_simple
  sendWiFiMsg(fstr, 1);
#endif
}


// custom clear screen function. prints some header info
void myclrScreen() {
  if(LCD_on) {
    myLCD->clrScreen();
  } else {
    Serial.println("\n");
  }
  printTime();
}
// print time etc on the first line
void printTime() {
  // have to use tempstr here
  sprintf(tempstr, "%s %02d:%02d (%d) ", VerStr, hourOfDay(), minsOfHour(), state); 
  printJBstr(0, 0, 2, 0x1f, 0, 0x1f, tempstr);     
}


#ifdef JB_WiFi_simple
//==================== WIFI messaging functions ===============================================
void sendWiFiMsg(char *str) {
  // print out the packet
  // ID first
  for(int iii=0; iii<10; iii++) {
    wifiSerial.print(configuration.IDstamp[iii]); // 10-50 digit ID - unique to each JuiceBox
  }
  wifiSerial.print(":");
  // print data now
  wifiSerial.print(str);
  wifiSerial.print(":");
  wifiSerial.println(UDPpacketEndSig);
}
void sendWiFiMsg(const __FlashStringHelper *fstr, int dummy) {
  // print out the packet
  // ID first
  for(int iii=0; iii<10; iii++) {
    wifiSerial.print(configuration.IDstamp[iii]); // 10-50 digit ID - unique to each JuiceBox
  }
  wifiSerial.print(":");
  // print data now
  wifiSerial.print(fstr);
  wifiSerial.print(":");
  wifiSerial.println(UDPpacketEndSig);
}
//===================== END WiFi messaging functions ===========================================
#endif
//---------------------------- end printing help functions ------------------------

//---------------------------- input control functions ----------------------------
// this takes max of 50ms if the button is pressed
int isBtnPressed(int pin) {
  if(digitalRead(pin)==HIGH) {
    // check if noise
    for(int zz=0; zz<10; zz++) {
      if(digitalRead(pin)==LOW) return 0;
      delay(5);
    }
    return 1;
  } else {
    return 0;
  }
}

//---------------- timing functions -----------------------------------------------
// time in minutes to the next run
int timeToNextRun() {
  byte day=dayOfWeek();
  byte hour=hourOfDay();
  byte day_index=day<5?0:1;
  byte nextDay_index=day_index;
  if(day==4) nextDay_index=1; // weekend after Fri
  if(day==6) nextDay_index=0; // weekday after Sun
  
  if(hour<configuration.starttime[day_index]) return (configuration.starttime[day_index]-hour)*60;
  if(hour<configuration.endtime[day_index]) return -1; // here, hour is > starttime, so if hour is also < endtime, can go now
  return (configuration.starttime[nextDay_index]-hour+24)%24*60-minsOfHour(); // here, hour is > endtime, so calc to next day
}

// determine day of week
byte dayOfWeek() {
  return (byte)(((sec_up-clock_offset)/24/3600)%7);
}
// determine hour of day
byte hourOfDay() {
  return (byte)(((sec_up-clock_offset)/3600)%24);
}
// determine minutes of hour
byte minsOfHour() {
  return (byte)(((sec_up-clock_offset)/60)%60);
}

// need some RTC functions here...

//---------------- END timing functions -----------------------------------------------

byte limit(const byte value, const byte minimum, const byte maximum) {
  if(value<minimum) return minimum;
  if(value>maximum) return maximum;
  return value;
}

void getSavingsPerKWH(int gascost, int mpg, int ecost, int whpermile) {
  int gCostPerMile=gascost/mpg;
  int gCostPerKWH=gCostPerMile*1000/whpermile;
  
  savingsPerKWH=gCostPerKWH-ecost;
}

// long delays
void delaySecs(int secs) {
  for(int si=0; si<secs; si++) delay(1000);
}


