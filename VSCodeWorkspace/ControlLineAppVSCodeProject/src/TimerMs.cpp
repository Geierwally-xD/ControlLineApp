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
 * Method: bool IsExpired(uint32_t timeout)
 *
 * Overview: checks timeout is expired and returns true if expired
 * otherwise false
 ********************************************************************/ 
  // check timer is expired, returns true if expired otherwise false
  bool TimerMs::IsExpired(uint32_t timeout)
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
 * Method: uint32_t GetRelativeTickCount(void)
 *
 * Overview: returns tickcount since timer start
 ********************************************************************/ 
  uint32_t TimerMs::GetRelativeTickCount()
  {
    return(millis() - startTime);
  }
