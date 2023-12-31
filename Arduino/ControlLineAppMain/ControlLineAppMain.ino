/*
 	 Name:           ControlLineAppMain.ino
 	 Description:    Geierwally's throttle and timer application for line controlled model planes
   Version:        V1.0.0
   CycleTime:      measured on toggle output 1ms
 	 Copyright:      Geierwally, 2023(c)
 */
#include "VoltageProtection.h"
#include "ServoControl.h"
#include "ButtonControl.h"
#include "LedControl.h"
#include "FlightTimer.h"
#include "GyroAccelSens.h"
//#define MEASURE_CYCLE_TIME

LedControl    statusLed_(3);                             /* status LED object for switching output LED */
ServoControl  throttleServo_(A0,9,2, &statusLed_);       /* servo object analog input A0, pwm output pin9, 2ms move delay */
ButtonControl userButton_(2, &statusLed_);               /* button object for input button connected at pin 2*/
FlightTimer   userFlightTimer(&statusLed_);              /* flight timer object for countdown flight time */
VoltageProtection voltageProtection_(A1,A2, &statusLed_);/* voltage protection object for controlling battery voltage of two cells */
GyroAccelSens gyroAccelSens_(0x68, &statusLed_);         /* gyroscope accelerometer object for measuring flight angle */
uint8_t servoteachActive = 0;                            /* value is != 0 if servo position teach is active, otherwise 0 */

#ifdef MEASURE_CYCLE_TIME
int cycleTimePin = 4;                                    /* toggle pin for measuring cycle time */
int cycleTimePinState = LOW;                             /* pin state for measuring cycle time */
#endif

/*********************************************************************
 * Function: void setup(void)
 *
 * Overview: initialize all objects
 ********************************************************************/ 
void setup()
{
  Serial.begin(9600);         /* initialize the serial console print */
  throttleServo_.Setup();     /* initialize the throttle servo */
  userButton_.Setup();        /* initialize the user button */
  statusLed_.Setup();         /* initialize the status LED */
  userFlightTimer.Setup();    /* initialize flight timer */
  voltageProtection_.Setup(); /* initialize voltage protectin instance */
  gyroAccelSens_.Setup();     /* initialize gyroscope accelerometer instance */
  Serial.println("control line application initialized");
#ifdef MEASURE_CYCLE_TIME
  cycleTimePinState = LOW;
  pinMode(cycleTimePin, OUTPUT);        
  digitalWrite(cycleTimePin,cycleTimePinState);
  AnalogReference(DEFAULT);
#endif  
}

/*********************************************************************
 * Function: void loop(void)
 *
 * Overview: the main loop of control line plane application
 ********************************************************************/ 
void loop() 
{
  uint8_t buttonPressState = userButton_.GetPressState();
  userButton_.Control();    /* read the user button */
  userFlightTimer.Control(buttonPressState,servoteachActive); /* execute flight timer task */
  statusLed_.Control();     /* switch status LED */
  servoteachActive = throttleServo_.Control(buttonPressState); /* control the throttle servo */
  voltageProtection_.Control(buttonPressState);                /* check battery voltage */
  gyroAccelSens_.Control(buttonPressState);                    /* measure flight angle and send to throttle control */
#ifdef MEASURE_CYCLE_TIME
  cycleTimePinState = (cycleTimePinState == LOW)? HIGH : LOW;            /* toggle outputpin for measuring cycle time */
  digitalWrite(cycleTimePin,cycleTimePinState);
#endif  
}
