/*
 	 Name:           ControlLineAppMain.ino
 	 Description:    Geierwally's throttle, additional functions and timer application for line controlled model planes
   Version:        V1.0.1
   CycleTime:      measured on toggle output and over timer function 6ms
 	 Copyright:      Geierwally, 2025(c)
 */
#include "VoltageProtection.h"
#include "ServoControl.h"
#include "ServoFunction.h"
#include "ButtonControl.h"
#include "LedControl.h"
#include "FlightTimer.h"
#include "GyroAccelSens.h"

LedControl    statusLed_(3);                             /* status LED object for switching output LED */
ButtonControl userButton_(2, &statusLed_);               /* button object for input button connected at pin 2*/
FlightTimer   userFlightTimer(&statusLed_);              /* flight timer object for countdown flight time */
VoltageProtection voltageProtection_(A1,A2, &statusLed_);/* voltage protection object for controlling battery voltage of two cells */
GyroAccelSens gyroAccelSens_(0x68, &statusLed_);         /* gyroscope accelerometer object for measuring flight angle */
ServoControl  throttleServo_(A0,9, &statusLed_,&gyroAccelSens_); /* servo object analog input A0, pwm output pin9 */
ServoFunction retractServo_(11, &statusLed_, &throttleServo_);   /* servo object for retract landing gear pwm output pin11 */
uint8_t servoTeachActive_ = 0;                            /* value is != 0 if servo position teach is active, otherwise 0 */
uint8_t gyroTeachActive_ = 0;                             /* value is != 0 if gyroscope teach is active, otherwise 0 */
uint8_t servoFunctionTeachActive_ = 0;                    /* value is != 0 if servo function teach is active, otherwise 0 */

#ifdef MEASURE_CYCLE_TIME
int cycleTimePin = 8;                                    /* toggle pin for measuring cycle time */
int cycleTimePinState = LOW;                             /* pin state for measuring cycle time */
#endif

/*********************************************************************
 * Function: void setup(void)
 *
 * Overview: initialize all objects
 ********************************************************************/ 
void setup()
{
  Serial.begin(115200);       /* initialize the serial console print */
  throttleServo_.Setup();     /* initialize the throttle servo */
  retractServo_.Setup();      /* initialize the retractable landing gear servo */
  userButton_.Setup();        /* initialize the user button */
  statusLed_.Setup();         /* initialize the status LED */
  userFlightTimer.Setup();    /* initialize flight timer */
  voltageProtection_.Setup(); /* initialize voltage protection instance */
  gyroAccelSens_.Setup();     /* initialize gyroscope accelerometer instance */
  analogReference(DEFAULT);
  Serial.println("control line application initialized");
#ifdef MEASURE_CYCLE_TIME
  cycleTimePinState = LOW;
  digitalWrite(cycleTimePin,cycleTimePinState);
#endif  
}

/*********************************************************************
 * Function: void loop(void)
 *
 * Overview: the main loop of control line plane application
 ********************************************************************/ 
void loop() 
{
  static bool prevFlightTimeElapsed = false;
  static bool flightTimeElapsed = false;
  #ifdef MEASURE_CYCLE_TIME
  static uint32_t cycleTimeMax = 0;
  uint32_t cycleTime = 0;
  TimerMs cycleTimer;
  #endif
 
  uint8_t buttonPressState = userButton_.GetPressState();
  userButton_.Control();                                      /* read the user button */
  statusLed_.Control();                                       /* switch status LED */
  servoTeachActive_ = throttleServo_.Control(buttonPressState, servoFunctionTeachActive_);/* control the throttle servo */
  voltageProtection_.Control(buttonPressState);               /* check battery voltage */
  gyroTeachActive_  = gyroAccelSens_.Control(buttonPressState);/* measure flight angle and send to throttle control */
  servoFunctionTeachActive_ = retractServo_.Control(buttonPressState); /* control the retractable landing gear servo */
  switch(gyroTeachActive_)
  {
    case (GyroAccelSens::GyroAccel_Teach + GyroAccelSens::gt_storeFlightPosHorizontal):
      throttleServo_.StoreGyroFlightPosHorizontal();
    break;
    case (GyroAccelSens::GyroAccel_Teach + GyroAccelSens::gt_storeFlightPosThrottle):
      throttleServo_.StoreGyroFlightThrottle();
    break;
    default:
    break;
  }
  flightTimeElapsed = userFlightTimer.Control(buttonPressState,gyroTeachActive_|servoTeachActive_|servoFunctionTeachActive_);/* execute flight timer task */
  if((flightTimeElapsed == true)&&(prevFlightTimeElapsed == false))
  {
    throttleServo_.SignalThrottleEndFlight(); /* signal throttle end of flight reached */
  }
  prevFlightTimeElapsed = flightTimeElapsed;
#ifdef MEASURE_CYCLE_TIME
  cycleTimePinState = (cycleTimePinState == LOW)? HIGH : LOW; /* toggle output pin for measuring cycle time */
  digitalWrite(cycleTimePin,cycleTimePinState);
  cycleTime = cycleTimer.GetRelativeTickCount();
  if(cycleTime > cycleTimeMax)
  {
    cycleTimeMax = cycleTime;
    Serial.println("cycle time max = " + String(cycleTimeMax));
  }
  cycleTimer.Start();
#endif  
}
