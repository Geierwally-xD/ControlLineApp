/*
 	 Name:           EEPromStorage.cpp
 	 Description:    implementation of EEProm storage object
 	 Copyright:      Geierwally, 2023(c)
 */

 #include "EEPromStorage.h"
 
/*********************************************************************
 * Method: EEPromStorage()
 *
 * Overview: constructor
 ********************************************************************/ 	
  EEPromStorage::EEPromStorage(void)
  {}
 
/*********************************************************************
 * Method: uint32_t readFlightTime(void)
 *
 * Overview: read flight time from EEProm and does plausibility check
 * @return: flight time value
 ********************************************************************/
	uint32_t EEPromStorage::readFlightTime(void)
  {
    uint32_t retVal = 0;
    EEPROM.get(FlightTimerAddress,retVal);
    if(retVal > 10)
      retVal = 0;
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("read flight time = " + String(retVal));    
  #endif
    return(retVal);  
  }

/*********************************************************************
 * Method: void writeFlightTime(uint32_t flightTime)
 *
 * Overview: write flight time to EEProm
 * @parameter: flightTime value
 ********************************************************************/
	void EEPromStorage::writeFlightTime(uint32_t flightTime)
  {
    EEPROM.put(FlightTimerAddress,flightTime);
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("write flight time = " + String(flightTime));    
  #endif
  }

/*********************************************************************
 * Method: int readCalMinVal(void)
 *
 * Overview: read calibrated minimum value from EEProm and does plausibility check
 * @return: calibrated minimum value
 ********************************************************************/
	int EEPromStorage::readCalMinVal(void)
  {
    int retVal = 0;
    EEPROM.get(CalibrateMinValAddress,retVal);
    if((retVal < 300)||(retVal >700))
      retVal = 0;
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("read calib min val = " + String(retVal));    
  #endif
    return(retVal);  
  }

/*********************************************************************
 * Method: void writeCalMinVal(int calMinVal)
 *
 * Overview: write calibrated minimum value to EEProm
 * @parameter: calMinVal calibrated minimum value
 ********************************************************************/
	void EEPromStorage::writeCalMinVal(int calMinVal)
  {
    EEPROM.put(CalibrateMinValAddress,calMinVal);
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("write calib min val = " + String(calMinVal));    
  #endif
  }

  /*********************************************************************
 * Method: int readCalMaxVal(void)
 *
 * Overview: read calibrated maximum value from EEProm and does plausibility check
 * @return: calibrated maximum value
 ********************************************************************/
	int EEPromStorage::readCalMaxVal(void)
  {
    int retVal = 0;
    EEPROM.get(CalibrateMaxValAddress,retVal);
    if((retVal < 800)||(retVal >1300))
      retVal = 0;
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("read calib max val = " + String(retVal));    
  #endif
    return(retVal);  
  }

/*********************************************************************
 * Method: void writeCalMaxVal(int calMaxVal)
 *
 * Overview: write calibrated maximum value to EEProm
 * @parameter: calMaxVal calibrated maximum value
 ********************************************************************/
	void EEPromStorage::writeCalMaxVal(int calMaxVal)
  {
    EEPROM.put(CalibrateMaxValAddress,calMaxVal);
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("write calib max val = " + String(calMaxVal));    
  #endif
  }

/*********************************************************************
 * Method: int readServoLimit(void)
 *
 * Overview: read Servo limit value from EEProm and does plausibility check
 * @return: servo limit value
 ********************************************************************/
	int EEPromStorage::readServoLimit(void)
  {
    int retVal = 0;
    EEPROM.get(ServoLimitAddress,retVal);
    if((retVal < 0)||(retVal >180))
      retVal = 0;
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("read servo limit val = " + String(retVal));    
  #endif
    return(retVal);  
  }

/*********************************************************************
 * Method: void writeServoLimit(int servoLimit)
 *
 * Overview: write servo limit value to EEProm
 * @parameter: servoLimit teached servo limit value
 ********************************************************************/
	void EEPromStorage::writeServoLimit(int servoLimit)
 {
    EEPROM.put(ServoLimitAddress,servoLimit);
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("write servo limit val = " + String(servoLimit));    
  #endif
  }

/*********************************************************************
 * Method: int readServoThrottle(void)
 *
 * Overview: read Servo throttle value from EEProm and does plausibility check
 * @return: servo throttle value
 ********************************************************************/
	int EEPromStorage::readServoThrottle(void)
  {
    int retVal = 0;
    EEPROM.get(ServoThrottleAddress,retVal);
    if((retVal < 0)||(retVal >180))
      retVal = 0;
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("read servo throttle val = " + String(retVal));    
  #endif
    return(retVal);  
  }

/*********************************************************************
 * Method: void writeServoThrottle(int servoThrottle)
 *
 * Overview: write servo throttle value to EEProm
 * @parameter: servoThrottle teached servo throttle value
 ********************************************************************/
	void EEPromStorage::writeServoThrottle(int servoLimit)
 {
    EEPROM.put(ServoThrottleAddress,servoLimit);
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("write servo throttle val = " + String(servoLimit));    
  #endif
  }

  /*********************************************************************
 * Method: bool readServoRevers(void)
 *
 * Overview: read Servo revers value from EEProm and does plausibility check
 * @return: servo revers value
 ********************************************************************/
	bool EEPromStorage::readServoRevers(void)
  {
    uint8_t tempVal = 0;
    EEPROM.get(ServoReversAddress,tempVal);
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("read servo reverse val = " + String(tempVal));    
  #endif
    return((tempVal == 1)? true : false);  
  }
/*********************************************************************
 * Method: void writeServoRevers(bool servoRevers)
 *
 * Overview: write servo revers value to EEProm
 * @parameter: servoRevers teached servo revers value
 ********************************************************************/
	void EEPromStorage::writeServoRevers(bool servoRevers)
 {
    uint8_t tempVal = (servoRevers == true)? 1:0;
    EEPROM.put(ServoReversAddress,tempVal);
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("write servo revers val = " + String(tempVal));    
  #endif
  }

/*********************************************************************
 * Method: bool readServoLineShortThrottle(void)
 *
 * Overview: read Servo line short throttle value from EEProm and does plausibility check
 * @return: servo line short throttle value
 ********************************************************************/
	bool EEPromStorage::readServoLineShortThrottle(void)
  {
    uint8_t tempVal = 0;
    EEPROM.get(ServoLineShortThrottleAddress,tempVal);
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("read servo line short throttle val = " + String(tempVal));    
  #endif
    return((tempVal == 1)? true : false);  
  }

/*********************************************************************
 * Method: void writeServoLineShortThrottle(bool lineShortThrottle)
 *
 * Overview: write servo line short throttle value to EEProm
 * @parameter: lineShortThrottle teached servo line short throttle value
 ********************************************************************/
	void EEPromStorage::writeServoLineShortThrottle(bool lineShortThrottle)
  {
    uint8_t tempVal = (lineShortThrottle == true)? 1:0;
    EEPROM.put(ServoLineShortThrottleAddress,tempVal);
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("write servo line short throttle val = " + String(tempVal));    
  #endif
  }

/*********************************************************************
 * Method: int readVoltageProtCell_1(void)
 *
 * Overview: read voltage protection cell 1 value from EEProm
 * does plausibility check
 * @return: voltage protection of cell 1 value
 ********************************************************************/
	int EEPromStorage::readVoltageProtCell_1(void)
  {
    int retVal = 0;
    EEPROM.get(VoltageProtCell_1_Address,retVal);
    if((retVal < 700)||(retVal >1023))
      retVal = 0;
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("read voltage protection cell 1 val = " + String(retVal));    
  #endif
    return(retVal); 
  }  

/*********************************************************************
 * Method: void writeVoltageProtCell_1(int voltProtCell_1)
 *
 * Overview: write voltage protection cell 1 value to EEProm
 * @parameter: voltProtCell_1 calibrated voltage protection cell 1 value
 ********************************************************************/
	void EEPromStorage::writeVoltageProtCell_1(int voltProtCell_1)
  {
    EEPROM.put(VoltageProtCell_1_Address,voltProtCell_1);
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("write voltage protection cell 1 val = " + String(voltProtCell_1));    
  #endif
  }
/*********************************************************************
 * Method: int readVoltageProtCell_2(void)
 *
 * Overview: read voltage protection cell 2 value from EEProm
 * does plausibility check
 * @return: voltage protection of cell 2 value
 ********************************************************************/
	int EEPromStorage::readVoltageProtCell_2(void)
  {
    int retVal = 0;
    EEPROM.get(VoltageProtCell_2_Address,retVal);
    if((retVal < 800)||(retVal >1023))
      retVal = 0;
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("read voltage protection cell 2 val = " + String(retVal));    
  #endif
    return(retVal); 
  } 
/*********************************************************************
 * Method: void writeVoltageProtCell_2(int voltProtCell_2)
 *
 * Overview: write voltage protection cell 2 value to EEProm
 * @parameter: voltProtCell_2 calibrated voltage protection cell 2 value
 ********************************************************************/
	void EEPromStorage::writeVoltageProtCell_2(int voltProtCell_2)
  {
    EEPROM.put(VoltageProtCell_2_Address,voltProtCell_2);
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("write voltage protection cell 2 val = " + String(voltProtCell_2));    
  #endif
  }
 