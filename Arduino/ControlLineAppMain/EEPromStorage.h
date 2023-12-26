/*
 	 Name:           EEPromStorage.h
 	 Description:    declaration of EEProm storage object
 	 Copyright:      Geierwally, 2023(c)
 */

#ifndef EEPROM_STORAGE_h
#define EEPROM_STORAGE_h

#include <EEPROM.h> 
#include "Arduino.h"
#include "TimerMs.h"

//#define TRACE_EEPROM_STORGE   /* console print of all into EEProm written values and from EEProm read values */
class EEPromStorage
{
  public:
/*********************************************************************
 * Method: EEPromStorage()
 *
 * Overview: constructor
 ********************************************************************/
	EEPromStorage(void);

/*********************************************************************
 * Method: uint32_t readFlightTime(void)
 *
 * Overview: read flight time from EEProm and does plausibility check
 * @return: flight time value
 ********************************************************************/
	uint32_t readFlightTime(void);

/*********************************************************************
 * Method: void writeFlightTime(uint32_t flightTime)
 *
 * Overview: write flight time to EEProm
 * @parameter: flightTime value
 ********************************************************************/
	void writeFlightTime(uint32_t flightTime);

/*********************************************************************
 * Method: int readCalMinVal(void)
 *
 * Overview: read calibrated minimum value from EEProm and does plausibility check
 * @return: calibrated minimum value
 ********************************************************************/
	int readCalMinVal(void);

/*********************************************************************
 * Method: void writeCalMinVal(int calMinVal)
 *
 * Overview: write calibrated minimum value to EEProm
 * @parameter: calMinVal calibrated minimum value
 ********************************************************************/
	void writeCalMinVal(int calMinVal);

/*********************************************************************
 * Method: int readCalMaxVal(void)
 *
 * Overview: read calibrated maximum value from EEProm and does plausibility check
 * @return: calibrated maximum value
 ********************************************************************/
	int readCalMaxVal(void);

/*********************************************************************
 * Method: void writeCalMaxVal(int calMaxVal)
 *
 * Overview: write calibrated maximum value to EEProm
 * @parameter: calMaxVal calibrated maximum value
 ********************************************************************/
	void writeCalMaxVal(int calMaxVal);

/*********************************************************************
 * Method: int readServoLimit(void)
 *
 * Overview: read Servo limit value from EEProm and does plausibility check
 * @return: servo limit value
 ********************************************************************/
	int readServoLimit(void);

/*********************************************************************
 * Method: void writeServoLimit(int servoLimit)
 *
 * Overview: write servo limit value to EEProm
 * @parameter: servoLimit teached servo limit value
 ********************************************************************/
	void writeServoLimit(int servoLimit);

/*********************************************************************
 * Method: int readServoThrottle(void)
 *
 * Overview: read Servo throttle value from EEProm and does plausibility check
 * @return: servo throttle value
 ********************************************************************/
	int readServoThrottle(void);

/*********************************************************************
 * Method: void writeServoThrottle(int servoThrottle)
 *
 * Overview: write servo throttle value to EEProm
 * @parameter: servoThrottle teached servo throttle value
 ********************************************************************/
	void writeServoThrottle(int servoLimit);

/*********************************************************************
 * Method: bool readServoRevers(void)
 *
 * Overview: read Servo revers value from EEProm and does plausibility check
 * @return: servo revers value
 ********************************************************************/
	bool readServoRevers(void);

/*********************************************************************
 * Method: void writeServoRevers(bool servoRevers)
 *
 * Overview: write servo revers value to EEProm
 * @parameter: servoRevers teached servo revers value
 ********************************************************************/
	void writeServoRevers(bool servoRevers);

/*********************************************************************
 * Method: bool readServoLineShortThrottle(void)
 *
 * Overview: read Servo line short throttle value from EEProm and does plausibility check
 * @return: servo line short throttle value
 ********************************************************************/
	bool readServoLineShortThrottle(void);

/*********************************************************************
 * Method: void writeServoLineShortThrottle(bool lineShortThrottle)
 *
 * Overview: write servo line short throttle value to EEProm
 * @parameter: lineShortThrottle teached servo line short throttle value
 ********************************************************************/
	void writeServoLineShortThrottle(bool lineShortThrottle);

/*********************************************************************
 * Method: int readVoltageProtCell_1(void)
 *
 * Overview: read voltage protection cell 1 value from EEProm
 * does plausibility check
 * @return: voltage protection of cell 1 value
 ********************************************************************/
	int readVoltageProtCell_1(void);

/*********************************************************************
 * Method: void writeVoltageProtCell_1(int voltProtCell_1)
 *
 * Overview: write voltage protection cell 1 value to EEProm
 * @parameter: voltProtCell_1 calibrated voltage protection cell 1 value
 ********************************************************************/
	void writeVoltageProtCell_1(int voltProtCell_1);

/*********************************************************************
 * Method: int readVoltageProtCell_2(void)
 *
 * Overview: read voltage protection cell 2 value from EEProm
 * does plausibility check
 * @return: voltage protection of cell 2 value
 ********************************************************************/
	int readVoltageProtCell_2(void);

/*********************************************************************
 * Method: void writeVoltageProtCell_2(int voltProtCell_2)
 *
 * Overview: write voltage protection cell 2 value to EEProm
 * @parameter: voltProtCell_2 calibrated voltage protection cell 2 value
 ********************************************************************/
	void writeVoltageProtCell_2(int voltProtCell_2);

  private:
  enum StorageAddress
  {
    FlightTimerAddress            = 0,                                                                                                                          /* uint32_t */
    CalibrateMinValAddress        = sizeof(uint32_t),                                                                                                           /* int */
    CalibrateMaxValAddress        = sizeof(uint32_t) + sizeof(int),                                                                                             /* int */
    ServoLimitAddress             = sizeof(uint32_t) + sizeof(int) + sizeof(int),                                                                               /* int */
    ServoThrottleAddress          = sizeof(uint32_t) + sizeof(int) + sizeof(int) + sizeof(int),                                                                 /* int */
    ServoReversAddress            = sizeof(uint32_t) + sizeof(int) + sizeof(int) + sizeof(int) + sizeof(int),                                                   /* bool */
    ServoLineShortThrottleAddress = sizeof(uint32_t) + sizeof(int) + sizeof(int) + sizeof(int) + sizeof(int) + sizeof(uint8_t),                                 /* bool */
    VoltageProtCell_1_Address     = sizeof(uint32_t) + sizeof(int) + sizeof(int) + sizeof(int) + sizeof(int) + sizeof(uint8_t) + sizeof(uint8_t),               /* int */
    VoltageProtCell_2_Address     = sizeof(uint32_t) + sizeof(int) + sizeof(int) + sizeof(int) + sizeof(int) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(int)  /* int */
  }storageAddress;
};

#endif