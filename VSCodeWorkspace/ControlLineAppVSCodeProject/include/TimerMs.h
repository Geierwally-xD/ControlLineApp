/*
 	 Name:           TimerMs.h
 	 Description:    declaration of millisecond timer object
 	 Copyright:      Geierwally, 2023(c)
 */
#ifndef TIMER_MS_h
#define TIMER_MS_h

#include "Arduino.h"

class TimerMs
{

  public:
/*********************************************************************
 * Method: TimerMs()
 *
 * Overview: constructor
 ********************************************************************/
	TimerMs();

/*********************************************************************
 * Method: void Start(void)
 *
 * Overview: sets the software timer active
 ********************************************************************/
	void Start();

/*********************************************************************
 * Method: bool IsExpired(uint32_t timeout)
 *
 * Overview: checks timeout is expired and returns true if expired
 * otherwise false
 ********************************************************************/ 
	bool IsExpired(uint32_t timeout);
  
/*********************************************************************
 * Method: uint32_t GetRelativeTickCount(void)
 *
 * Overview: returns tickcount since timer start
 ********************************************************************/ 
  uint32_t GetRelativeTickCount();

  private:
    uint32_t startTime; /* attribute contains start time of the software timer */
	  bool isActive;      /* attribute for timer state, true if active, otherwise false */
};
#endif