/*
 	 Name:           TimerMs.cpp
 	 Description:    definition of millisecond timer object
 	 Copyright:      Geierwally, 2025(c)
 */

#include "TimerMs.h"

/*********************************************************************
 * Method: TimerMs()
 *
 * Overview: constructor
 ********************************************************************/
  TimerMs::TimerMs()
  {
    startTime = millis();
    isActive = false;
  }

/*********************************************************************
 * Method: void Start(void)
 *
 * Overview: sets the software timer active
 ********************************************************************/
  void TimerMs::Start()
  {
    startTime = millis();
    isActive = true;
  }

/*********************************************************************
 * Method: bool IsExpired(unsigned long timeout)
 *
 * Overview: checks timeout is expired and returns true if expired
 * otherwise false
 ********************************************************************/ 
  // check timer is expired, returns true if expired otherwise false
  bool TimerMs::IsExpired(unsigned long timeout)
  {
    bool retVal = false;
    if(true == isActive)
    {
      if(timeout < (millis() - startTime))
      {
        retVal = true;
        isActive = false;
      }
    }
    return(retVal);
  }

/*********************************************************************
 * Method: unsigned long GetRelativeTickCount(void)
 *
 * Overview: returns tickcount since timer start
 ********************************************************************/ 
  unsigned long TimerMs::GetRelativeTickCount()
  {
    return(millis() - startTime);
  }
