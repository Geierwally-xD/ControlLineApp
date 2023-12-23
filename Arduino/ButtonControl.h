/*
 	 Name:           ButtonControl.h
 	 Description:    declaration of button control object
 	 Copyright:      Geierwally, 2023(c)
 */
#ifndef BUTTON_CONTROL_h
#define BUTTON_CONTROL_h

#include "Arduino.h"
#include "TimerMs.h"
#include "LedControl.h"

class ButtonControl
{
  public:
  enum ButtonPressState
  {
    press_idle,
    _1sPressed,
    _2sPressed,  /* servo teach throttle */
    _3sPressed,  /* servo teach end position */
    _4sPressed,  /* teach servo revers */
    _5sPressed,  /* reaction on control line short */
    _6sPressed,  /* calibrate potentiometer */
    _7sPressed,  /* calibrate voltage protection */
    _8sPressed   /* teach flight timer */
  } buttonPressState; /* the last press-state of the button */ 

  enum ButtonControlState
  {
    buttonOn,
    buttonOff,
    control_idle
  } buttonControlState; /* the current control-state of the button task*/ 

/*********************************************************************
 * Method: ButtonControl(int buttonPin, LedControl & statusLed)
 *
 * Overview: constructor with parameters:
 * buttonPin: the digital input pin where button is connected
 * statusLed: reference to the status led object
 ********************************************************************/
	  ButtonControl(int buttonPin, LedControl * statusLed);
 
/*********************************************************************
 * Method: void Setup(void)
 *
 * Overview: initialize the button control attributes
 ********************************************************************/
    void Setup();

 /*********************************************************************
 * Method: void Control(void)
 *
 * Overview: the button control main function task
 * set button press state
 ********************************************************************/
    void Control();

/*********************************************************************
 * Method uint8_t GetPressState(void)
 *
 * Overview: returns the button press state
 ********************************************************************/
    uint8_t GetPressState();

  private:
    const uint32_t buttonPressTimeout_[9] = {50,1300,1600,1900,2200,2500,2800,3100,0}; /* 1000ms + led flash time */
    TimerMs buttonPressTimer_;  /* timer for button pressed */
    int buttonPin_;             /* digital input pin for the button */
    int buttonPinState_;        /* last read in state of the button */
    uint8_t buttonPressState_;  /* last button press state depending enum ButtonPressState */
    uint8_t buttonControlState_;/* last button control state */
    LedControl * statusLed_;    /* reference to status LED object */
};
#endif