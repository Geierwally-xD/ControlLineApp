/*
 	 Name:           LedControl.cpp
 	 Description:    definition of controlling status LED object
 	 Copyright:      Geierwally, 2023(c)
 */
#include "LedControl.h"

/*********************************************************************
 * Method: LedControl(int ledPin)
 *
 * Overview: constructor
 * ledPin: the digital output pin where status LED is connected
 ********************************************************************/
    LedControl::LedControl(int ledPin)
    {
      ledPin_ = ledPin;                     /* initialize digital output pin from constructor parameter */
      ledPinState_ = LOW;                   /* initialize last written LED state with LOW */
      ledFlashCounter_ = 0;
      ledFlashTime_ = 100;                  /* initialize flash time with 100 ms */
      ledBreakTime_ = 150;                  /* initialize flash time with 100 ms */
    }

     
/*********************************************************************
 * Method: void Setup(void)
 *
 * Overview: initialize the LED control attributes
 ********************************************************************/
    void LedControl::Setup()
    {
        ledPinState_ = LOW;
        pinMode(ledPin_, OUTPUT);        
        digitalWrite(ledPin_,ledPinState_);
        ledTimer_.Start();
        ledFlashCounter_ = 0;
        ledFlashTime_ = 100;
        ledBreakTime_ = 150; 
    }

 /*********************************************************************
 * Method: void Control(void)
 *
 * Overview: the LED control main function task
 * switch LED status
 ********************************************************************/
    void LedControl::Control() 
    {
      switch(ledPinState_)
      {
        case HIGH:
          if(ledTimer_.IsExpired(ledFlashTime_))
          {
            ledTimer_.Start();
            ledPinState_ = LOW;
            ledFlashCounter_ --;
          }
        break;
        case LOW:
          if(ledTimer_.IsExpired(ledBreakTime_))
          {
            if(ledFlashCounter_ >0)
            {
              ledTimer_.Start();
              ledPinState_ = HIGH;
            }
          }
        break;
      }
      digitalWrite(ledPin_,ledPinState_);
    }

 /*********************************************************************
 * Method: void FlashLed(uint8_t flashCounter, uint32_t flashTime)
 *
 * Overview: activate LED flash control, restart LED timer
 * flashCounter: count of flash pulses
 * flashTime: LED flash time in ms
 ********************************************************************/
    void LedControl::FlashLed(uint8_t flashCounter, uint32_t flashTime, uint32_t breakTime)
    {
      if((flashCounter > 0)&&(flashTime > 0)&&(breakTime > 0))
      {
        ledPinState_ = HIGH;
        ledTimer_.Start();
        ledFlashCounter_ = flashCounter;
        ledFlashTime_ = flashTime;
        ledBreakTime_ = breakTime;
      }
      else
      {
        ledPinState_ = LOW;
        ledTimer_.Start();
        ledFlashCounter_ = 0;
        ledFlashTime_ = 0;
        ledBreakTime_ = 0;
      }
    }       