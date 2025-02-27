/*
 	 Name:           LedControl.cpp
 	 Description:    definition of controlling status LED object
 	 Copyright:      Geierwally, 2025(c)
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
      function4PinState_ = LOW;             /* initialize function 4 output state off */
      function5PinState_ = LOW;             /* initialize function 5 output state off */
      ledFlashCounter_ = 0;
      ledFlashTime_ = 100;                  /* initialize flash time with 100 ms */
      ledBreakTime_ = 150;                  /* initialize flash time with 100 ms */
      function2State_ = 0;                  /* initialize function 2 output state off */ 
    }

     
/*********************************************************************
 * Method: void Setup(void)
 *
 * Overview: initialize the LED control attributes
 ********************************************************************/
    void LedControl::Setup()
    {
        ledPinState_ = LOW;
        function2State_ = 0;                  /* initialize function 3 output state off */ 
        function3PinState_ = LOW;             /* initialize function 3 output state off */
        function4PinState_ = LOW;             /* initialize function 4 output state off */
        function5PinState_ = LOW;             /* initialize function 5 output state off */

        pinMode(ledPin_, OUTPUT);        
        pinMode(function3Pin, OUTPUT);        
        pinMode(function4Pin, OUTPUT);        
        pinMode(function5Pin, OUTPUT);        
        digitalWrite(ledPin_,ledPinState_);
        digitalWrite(function3Pin,function3PinState_);
        digitalWrite(function4Pin,function4PinState_);
        digitalWrite(function5Pin,function5PinState_);
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
 * Method: void SwitchFunction(uint8_t functionID)
 *
 * Overview: switch digital function output on/off
 * @param: functionID the function ID to switch
 ********************************************************************/
  void LedControl::SwitchFunction(uint8_t functionID)
  {
    switch(functionID)
    {
      case 2:
        function2State_ = !function2State_; /* switch retractable landing gear */
      break;
      case 3:
#ifndef MEASURE_CYCLE_TIME      
        function3PinState_ = !function3PinState_;
        digitalWrite(function3Pin,function3PinState_);
#endif        
      break;
      case 4:
        function4PinState_ = !function4PinState_;
        digitalWrite(function4Pin,function4PinState_);
      break;
      case 5:
        function5PinState_ = !function5PinState_;
        digitalWrite(function5Pin,function5PinState_);
      break;
      default:
      break;
    }
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

 /*********************************************************************
 * Method: int GetFunction2State(void)
 *
 * Overview: returns the state of function 2 output
 * @return: the state of function 2 (0 or 1)
 ********************************************************************/
  uint8_t LedControl::GetFunction2State(void)
  {
    return function2State_;
  }
  
 /*********************************************************************
 * Method: void SetFunction3PinState(int state)
 *
 * Overview: write the state of function 3 output pin
 * @param: state the state to write to the function 3 output pin
 ********************************************************************/
  void LedControl::SetFunction3PinState(int state)
  {
    digitalWrite(function3Pin,state);
  }
