/*
 	 Name:           LedControl.h
 	 Description:    declaration of controlling status LED object
 	 Copyright:      Geierwally, 2023(c)
 */
#ifndef LED_CONTROL_h
#define LED_CONTROL_h

#include "Arduino.h"
#include "TimerMs.h"

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


  private:

    TimerMs ledTimer_;          /* timer for switching LED states */
    uint32_t ledFlashTime_;     /* led flash time in ms */
    uint32_t ledBreakTime_;     /* led break time in ms */
    int ledPin_;                /* digital output pin for the LED */
    int ledPinState_;           /* last switch state of the LED */
    uint8_t ledFlashCounter_;   /* counte for flashing led */   
    bool    ledlongFlash_;      /* 500 ms flash pulse if true, otherwise 50 ms */         
};

#endif