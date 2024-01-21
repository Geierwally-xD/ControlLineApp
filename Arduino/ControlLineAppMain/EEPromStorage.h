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

/*********************************************************************
 * Method: void readGyroCalib(float* X, float* Y, float* Z)
 *
 * Overview: read gyroscope calibration values from from EEProm
 * @parameter: references to X,Y,Z calibration values
 ********************************************************************/
  void readGyroCalib(float* X, float* Y, float* Z);

/*********************************************************************
 * Method: void writeGyroCalib(float X, float Y, float Z)
 *
 * Overview: write gyroscope calibration values to EEProm
 * @parameter: X,Y,Z calibration values
 ********************************************************************/
  void writeGyroCalib(float X, float Y, float Z);

/*********************************************************************
 * Method: uint8_t readGyroNormalInvers(void)
 *
 * Overview: read gyro assembly from EEProm
 * @return: 1 if normal assembly, 2 if invers assembly
 ********************************************************************/
	uint8_t readGyroNormalInvers(void);

/*********************************************************************
 * Method: void writeGyroNormalInvers(uint8_t gyroassembly)
 *
 * Overview: write gyroassembly to EEProm
 * @parameter: gyroassembly 1 = normal, 2 = invers
 ********************************************************************/
	void writeGyroNormalInvers(uint8_t gyroassembly);

  
/*********************************************************************
 * Method: int readGyroHorizontal(void)
 *
 * Overview: read Servo GyroHorizontal value from EEProm and does plausibility check
 * @return: servo GyroHorizontal value
 ********************************************************************/
	int readGyroHorizontal(void);

/*********************************************************************
 * Method: void writeServoGyroHorizontal(int gyroFlightHorizontal)
 *
 * Overview: write servo gyroFlightHorizontal value to EEProm
 * @parameter: gyroFlightHorizontal teached servo gyro horizontal flight value
 ********************************************************************/
  void writeServoGyroHorizontal(int gyroFlightHorizontal);

 /*********************************************************************
 * Method: int readGyroThrottle(void)
 *
 * Overview: read Servo GyroThrottle value from EEProm and does plausibility check
 * @return: servo GyroThrottle value
 ********************************************************************/
	int readGyroThrottle(void);

 /*********************************************************************
 * Method: void writeServoGyroThrottle(int gyroFlightThrottle)
 *
 * Overview: write servo gyroFlightThrottle value to EEProm
 * @parameter: gyroFlightThrottle teached servo gyro throttle flight value
 ********************************************************************/
  void writeServoGyroThrottle(int gyroFlightThrottle);   

 /*********************************************************************
 * Method: uint8_t readGyroExpo(void)
 *
 * Overview: read gyro expo value from EEProm and does plausibility check
 * @return: gyro expo value
 ********************************************************************/
  uint8_t readGyroExpo(void);
  
 /*********************************************************************
 * Method: void writeGyroExpo(uint8_t gyroExpo)
 *
 * Overview: write gyro expo value to EEProm
 * @parameter: gyroExpo teached gyro expo value
 ********************************************************************/
  void writeGyroExpo(uint8_t gyroExpo);

 /*********************************************************************
 * Method: uint8_t readGyroFlightActive(void)
 *
 * Overview: read gyro flight active value from EEProm and does plausibility check
 * @return: gyro fight active value
 ********************************************************************/
  uint8_t readGyroFlightActive(void);
  
 /*********************************************************************
 * Method: void writeGyroFlightActive(uint8_t gyroFlightActive)
 *
 * Overview: write gyro flight active value to EEProm
 * @parameter: gyroFlightActive gyro flight active value
 ********************************************************************/
  void writeGyroFlightActive(uint8_t gyroFlightActive);


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
    VoltageProtCell_2_Address     = sizeof(uint32_t) + sizeof(int) + sizeof(int) + sizeof(int) + sizeof(int) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(int), /* int */
    GyroCalibAddressX             = sizeof(uint32_t) + sizeof(int) + sizeof(int) + sizeof(int) + sizeof(int) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(int) + sizeof(int),                              /*float*/
    GyroCalibAddressY             = sizeof(uint32_t) + sizeof(int) + sizeof(int) + sizeof(int) + sizeof(int) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(int) + sizeof(int)+ sizeof(float),               /*float*/
    GyroCalibAddressZ             = sizeof(uint32_t) + sizeof(int) + sizeof(int) + sizeof(int) + sizeof(int) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(int) + sizeof(int)+ sizeof(float)+ sizeof(float), /*float*/
    GyroNormalInversAddress       = sizeof(uint32_t) + sizeof(int) + sizeof(int) + sizeof(int) + sizeof(int) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(int) + sizeof(int)+ sizeof(float)+ sizeof(float)+ sizeof(float), /*uint8_t*/
    GyroHorizontalAddress         = sizeof(uint32_t) + sizeof(int) + sizeof(int) + sizeof(int) + sizeof(int) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(int) + sizeof(int)+ sizeof(float)+ sizeof(float)+ sizeof(float) + sizeof(uint8_t), /*int*/
    GyroThrottleAddress           = sizeof(uint32_t) + sizeof(int) + sizeof(int) + sizeof(int) + sizeof(int) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(int) + sizeof(int)+ sizeof(float)+ sizeof(float)+ sizeof(float) + sizeof(uint8_t) + sizeof(int), /*int*/
    GyroExpoAddress               = sizeof(uint32_t) + sizeof(int) + sizeof(int) + sizeof(int) + sizeof(int) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(int) + sizeof(int)+ sizeof(float)+ sizeof(float)+ sizeof(float) + sizeof(uint8_t) + sizeof(int) + sizeof(int), /*uint8_t*/
    GyroFlightActiveAddress       = sizeof(uint32_t) + sizeof(int) + sizeof(int) + sizeof(int) + sizeof(int) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(int) + sizeof(int)+ sizeof(float)+ sizeof(float)+ sizeof(float) + sizeof(uint8_t) + sizeof(int) + sizeof(int) + sizeof(uint8_t) /*uint8_t*/
  }storageAddress;
};

#endif