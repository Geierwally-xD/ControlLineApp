/*
 	 Name:           FlightTimer.h
 	 Description:    declaration of flight timer object
 	 Copyright:      Geierwally, 2023(c)
 */
#ifndef FLIGHT_TIMER_h
#define FLIGHT_TIMER_h

#include "Arduino.h"
#include "TimerMs.h"
#include "LedControl.h"
#include "EEPromStorage.h"

class FlightTimer
{

  public:
    enum FlightTimerTaskState
    {
        FlightTime_Idle = 0,     /* wait for start */
        FlightTime_Main = 1,     /* main task flight timer control */
        FlightTime_Teach = 8,    /* teach flight timer */
        FlightTime_Store,        /* store timer teach on EE PROM */
    } flightTimerTaskState;      /* states of flight timer task state machine */ 
/*********************************************************************
 * Method: FlightTimer(LedControl * statusLed)
 *
 * @parameter: statusLed reference to status LED instance
 * Overview: constructor
 ********************************************************************/
	FlightTimer(LedControl * statusLed);
 
 /*********************************************************************
 * Method: void Setup(void)
 *
 * Overview: initialize flight timer attributes
 ********************************************************************/	
  void Setup(void); 

/*********************************************************************
 * Method: void Start(void)
 *
 * Overview: sets the flight timer active
 ********************************************************************/
	void Start(); 
  
 /*********************************************************************
 * Method: void Control(void)
 *
 * Overview: the flight timer control main function task
 * check elapsed flight time
 * @parameter: buttonPressState check button pressed for teach menu or timer start
 * @parameter: teachActive servo position teach active > 0 lock flight timer
 ********************************************************************/
   void Control(uint8_t buttonPressState, uint8_t teachActive);


  private:

    TimerMs flightTimer_;          /* countdown flight time */
    TimerMs flightEndLedTimer_;    /* timer for flash LED flight end */
    TimerMs teachTimer_;           /* timer for teach flight time*/
    uint32_t flightTime_;          /* maximum flight time */
    uint8_t flightTimeCtrlState_; /* state of flight time control task */
    LedControl * statusLed_;      /* reference to status LED object */
    uint32_t remainingMinutes_;    /* remaining minutes flight time */
    EEPromStorage  eePromStorage_; /* instance of eeProm storage object */

     
 /*********************************************************************
 * Method: void teach(void)
 *
 * Overview: the flight timer teach function
 *
 ********************************************************************/
  void teach(void);
};


#endif


