/*
 	 Name:           LedControl.h
 	 Description:    declaration of controlling status LED object
 	 Copyright:      Geierwally, 2025(c)
 */
#ifndef LED_CONTROL_h
#define LED_CONTROL_h

#include "Arduino.h"
#include "TimerMs.h"
//#define MEASURE_CYCLE_TIME /* toggle cycle time measurement over function 3 output */

class LedControl
{

  public:


/*********************************************************************
 * Method: LedControl(int ledPin)
 *
 * Overview: constructor
 * ledPin: the digital output pin where status LED is connected
 ********************************************************************/
	LedControl(int ledPin);

/*********************************************************************
 * Method: void Setup(void)
 *
 * Overview: initialize the LED control attributes
 ********************************************************************/
  void Setup();

 /*********************************************************************
 * Method: void FlashLed(uint8_t flashCounter, uint32_t flashTime)
 *
 * Overview: activate LED flash control, restart LED timer
 * flashCounter: count of flash pulses
 * flashTime: LED flash time in ms
 ********************************************************************/
  void FlashLed(uint8_t flashCounter, uint32_t flashTime, uint32_t breakTime);

 /*********************************************************************
 * Method: void Control(void)
 *
 * Overview: the LED control main function task
 * switch LED status
 ********************************************************************/
  void Control();

 /*********************************************************************
 * Method: void SwitchFunction(uint8_t functionID)
 *
 * Overview: switch digital function output on/off
 * @param: functionID the function ID to switch
 ********************************************************************/
  void SwitchFunction(uint8_t functionID);

 /*********************************************************************
 * Method: int GetFunction2State(void)
 *
 * Overview: returns the state of function 2 output
 * @return: the state of function 2 (0 or 1)
 ********************************************************************/
  uint8_t GetFunction2State(void);

/*********************************************************************
 * Method: void SetFunction3PinState(int state)
 *
 * Overview: write the state of function 3 output pin
 * @param: state the state to write to the function 3 output pin
 ********************************************************************/
  void SetFunction3PinState(int state); 

  private:
    enum functionPins
    {
      function3Pin = 8, /* digital output pin for function 3 */
      function4Pin = 12, /* digital output pin for function 4 */
      function5Pin = 13  /* digital output pin for function 5 */
    };

    TimerMs ledTimer_;          /* timer for switching LED states */
    uint32_t ledFlashTime_;     /* led flash time in ms */
    uint32_t ledBreakTime_;     /* led break time in ms */
    int ledPin_;                /* digital output pin for the LED */
    int ledPinState_;           /* last switch state of the LED */
    int function3PinState_;     /* digital output function 3 state  */
    int function4PinState_;     /* digital output function 4 state  */
    int function5PinState_;     /* digital output function 5 state  */
    uint8_t ledFlashCounter_;   /* counter for flashing led */   
    bool    ledlongFlash_;      /* 500 ms flash pulse if true, otherwise 50 ms */   
    uint8_t function2State_;    /* digital function 2 state (retractable landing gear) */      
};

#endif