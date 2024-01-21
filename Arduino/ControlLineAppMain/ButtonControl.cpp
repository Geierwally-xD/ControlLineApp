/*
 	 Name:           ButtonControl.cpp
 	 Description:    definition of controlling button object
 	 Copyright:      Geierwally, 2023(c)
 */
#include "ButtonControl.h"

/*********************************************************************
 * Method: ButtonControl(int buttonPin, LedControl & statusLed)
 *
 * Overview: constructor with parameters:
 * buttonPin:  the digital input pin, where button is connected
 * statusLed: reference to the status led object
 ********************************************************************/
    ButtonControl::ButtonControl(int buttonPin, LedControl * statusLed)
    {
      buttonPin_ = buttonPin;               /* initialize digital input pin from constructor parameter */
      buttonPinState_ = LOW;                /* initialize last read in state of the button with LOW */
      buttonPressState_ = press_idle;       /* initialize last button press state with press_idle */
      buttonControlState_ = control_idle;   /* initialize last button control state with control_idle */
      statusLed_ = statusLed;               /* initialize reference to status LED */
    }

     
/*********************************************************************
 * Method: void Setup(void)
 *
 * Overview: initialize the button control attributes
 ********************************************************************/
    void ButtonControl::Setup()
    {
        pinMode(buttonPin_, INPUT_PULLUP);        
        buttonPinState_ = digitalRead(buttonPin_);
    }

 /*********************************************************************
 * Method: void Control(void)
 *
 * Overview: the button control main function task
 * set button press state
 ********************************************************************/
    void ButtonControl::Control() 
    {
      uint32_t buttonPressTime = 0;
      buttonPinState_ = digitalRead(buttonPin_);
      if(buttonPinState_ != buttonControlState_)
      {    
        buttonPressTimer_.Start();
      }
      buttonControlState_ = buttonPinState_;  /* set button control state to last button pin state */
      buttonPressTime = buttonPressTimer_.GetRelativeTickCount(); /* get time since start */
      switch (buttonControlState_) /* check previous button state */
      {
        case control_idle: /* wait for next button input */
        case buttonOff:
        default:
        break;
        case buttonOn:     /* check press state */
          if(buttonPressTimer_.IsExpired(buttonPressTimeout_[buttonPressState_]))
          {
            if(buttonPressState_ < _9sPressed)
            {
              buttonPressState_ ++;
              statusLed_->FlashLed(buttonPressState_,100,200);
              buttonPressTimer_.Start();
            }
          }
        break;
      }
    }

/*********************************************************************
 * Method uint8_t GetPressState(void)
 *
 * Overview: returns the button press state, if button was not pressed
 ********************************************************************/
    uint8_t ButtonControl::GetPressState()
    {
      uint8_t retVal = press_idle;
      if(buttonControlState_ != buttonOn)
      {
        retVal = buttonPressState_;
        buttonPressState_ = press_idle;
      }
      return(retVal);
    }