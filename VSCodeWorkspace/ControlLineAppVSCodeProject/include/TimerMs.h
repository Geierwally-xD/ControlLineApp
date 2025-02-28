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
 * Method: bool IsExpired(unsigned long timeout)
 *
 * Overview: checks timeout is expired and returns true if expired
 * otherwise false
 ********************************************************************/ 
	bool IsExpired(unsigned long timeout);
  
/*********************************************************************
 * Method: unsigned long GetRelativeTickCount(void)
 *
 * Overview: returns tickcount since timer start
 ********************************************************************/ 
  unsigned long GetRelativeTickCount();

  private:
    unsigned long startTime; /* attribute contains start time of the software timer */
	  bool isActive;      /* attribute for timer state, true if active, otherwise false */
};
#endif