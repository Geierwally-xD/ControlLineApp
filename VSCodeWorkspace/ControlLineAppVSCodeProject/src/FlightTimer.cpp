/*
 	 Name:           FlightTimer.cpp
 	 Description:    definition of flight timer object
 	 Copyright:      Geierwally, 2023(c)
 */

#include "FlightTimer.h"

/*********************************************************************
 * Method: FlightTimer()
 *
 * @parameter: statusLed reference to status LED instance
 * Overview: constructor
 ********************************************************************/
  FlightTimer::FlightTimer(LedControl * statusLed)
  {
    flightTimeCtrlState_ = FlightTime_Idle;
    statusLed_ = statusLed; /* initialize reference to status LED */
  }
  
/*********************************************************************
 * Method: void Setup(void)
 *
 * Overview: initialize flight timer attributes
 ********************************************************************/	
  void FlightTimer::Setup() 
  {
      flightTime_ = eePromStorage_.readFlightTime();
  }

 /*********************************************************************
 * Method: void Control(void)
 *
 * Overview: the flight timer control main function task
 * check elapsed flight time
 * @parameter: buttonPressState check button pressed for teach menu or timer start
 * @parameter: teachActive servo position teach active > 0 lock flight timer
 
 
 ********************************************************************/
  bool FlightTimer::Control(uint8_t buttonPressState, uint8_t teachActive)
  {
    bool flightTimerElapsed = false;
    /* other teach active stop flight timer */
    switch(buttonPressState)
    { 
      case FlightTime_Idle:
      case FlightTime_Main:
        /* nothing to do */
      break;
      case FlightTime_Teach:
        /* flight time teach menu pressed */
        flightTimeCtrlState_ = FlightTime_Teach;
        teachTimer_.Start();
        flightTime_ = 0;
      break;
      default:
        /* for all other teach menus switch to idle */
        flightTimeCtrlState_ = FlightTime_Idle;
      break;
    }
       
    switch(flightTimeCtrlState_)
    {
        default:
        case FlightTime_Idle:         /* wait for start or teach*/
          if(   (buttonPressState == FlightTime_Main)
              &&(teachActive == 0)
            )
          {
            remainingMinutes_ = flightTime_ +1; /* set remaining minutes for flashing flight time */
            flightTimeCtrlState_ = FlightTime_Main;
            flightTimer_.Start();
            flightEndLedTimer_.Start();
          }
          else if(buttonPressState == FlightTime_Teach)
          {
            flightTimeCtrlState_ = FlightTime_Teach;
          }
        break;
        case FlightTime_Main:         /* main task flight timer control */
          if(buttonPressState == FlightTime_Main)
          {
            flightTimeCtrlState_ = FlightTime_Idle; /* reset flight time */
          }
          else
          {
            unsigned long elapsedflightTime = flightTimer_.GetRelativeTickCount()/60000;
            if(elapsedflightTime < flightTime_)
            {
              elapsedflightTime = flightTime_ - elapsedflightTime;
              if(remainingMinutes_ != elapsedflightTime)
              { /* flash remaining flight time */
                remainingMinutes_ = elapsedflightTime;
                statusLed_->FlashLed(remainingMinutes_,300,300);                  
              }
            }
            else
            {
              flightTimerElapsed = true;
              if(flightEndLedTimer_.IsExpired(600))
              { /* flash LED end of flight time reached */
                flightEndLedTimer_.Start();
                statusLed_->FlashLed(1,200,200);
              }
            }
          }
        break;
        case FlightTime_Teach:        /* teach flight timer */
          if(buttonPressState == FlightTime_Main)
          {
            flightTimeCtrlState_ = FlightTime_Store; /* store flight time value in EE PROM*/
          }
          else
          {teach();}
        break;
        case FlightTime_Store:        /* store timer teach on EE PROM */
            eePromStorage_.writeFlightTime(flightTime_);
            flightTimeCtrlState_ = FlightTime_Idle;
        break;
    }
    return(flightTimerElapsed);
  }

  
 /*********************************************************************
 * Method: void teach(void)
 *
 * Overview: the flight timer teach function
 *
 ********************************************************************/
  void FlightTimer::teach(void)
  {
    if(teachTimer_.IsExpired(1000 + flightTime_ * 400))
    {
      teachTimer_.Start();
      if(flightTime_ < 10)
      {flightTime_ ++;}
      else
      {flightTime_ = 1;}
      statusLed_->FlashLed(flightTime_,200,200);
    }
  }