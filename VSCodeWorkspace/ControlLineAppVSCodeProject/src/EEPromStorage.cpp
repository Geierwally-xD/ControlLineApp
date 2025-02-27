/*
 	 Name:           EEPromStorage.cpp
 	 Description:    implementation of EEProm storage object
 	 Copyright:      Geierwally, 2025(c)
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
 * Method: bool readServoThrottleSens(void)
 *
 * Overview: read Servo throttle sensitivity value from EEProm
 * @return: servo throttle sensitivity value
 ********************************************************************/
	uint8_t EEPromStorage::readServoThrottleSens(void)
  {
    uint8_t tempVal = 0;
    EEPROM.get(ServoThrottleSensAddress,tempVal);
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("read servo throttle sensitivity val = " + String(tempVal));    
  #endif
    return(tempVal);  
  }

/*********************************************************************
 * Method: void writeServoThrottleSens(uint8_t ThrottleSens)
 *
 * Overview: write servo throttle sensitivity value to EEProm
 * @parameter: ThrottleSens teached servo throttle sensitivity  value
 ********************************************************************/
	void EEPromStorage::writeServoThrottleSens(uint8_t ThrottleSens)
  {
    EEPROM.put(ServoThrottleSensAddress,ThrottleSens);
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("write servo throttle sensitivity val = " + String(ThrottleSens));    
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


/*********************************************************************
 * Method: void readGyroCalib(float* X, float* Y, float* Z)
 *
 * Overview: read gyroscope calibration values from from EEProm
 * @parameter: references to X,Y,Z calibration values
 ********************************************************************/
  void EEPromStorage::readGyroCalib(float* X, float* Y, float* Z)
  {
    float tempVal;
    EEPROM.get(GyroCalibAddressX,tempVal);
    *X = tempVal;
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("read gyro calibration X = " + String(tempVal));    
  #endif
    EEPROM.get(GyroCalibAddressY,tempVal);
    *Y = tempVal;
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("read gyro calibration Y = " + String(tempVal));    
  #endif
    EEPROM.get(GyroCalibAddressZ,tempVal);
    *Z = tempVal;
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("read gyro calibration Z = " + String(tempVal));    
  #endif
  } 
/*********************************************************************
 * Method: void writeGyroCalib(float X, float Y, float Z)
 *
 * Overview: write gyroscope calibration values to EEProm
 * @parameter: X,Y,Z calibration values
 ********************************************************************/
  void EEPromStorage::writeGyroCalib(float X, float Y, float Z)
  {
    EEPROM.put(GyroCalibAddressX,X);
    EEPROM.put(GyroCalibAddressY,Y);
    EEPROM.put(GyroCalibAddressZ,Z);
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("write gyroscope calib X val = " + String(X));    
    Serial.println("write gyroscope calib Y val = " + String(Y));    
    Serial.println("write gyroscope calib Z val = " + String(Z));    
  #endif
  }

  /*********************************************************************
 * Method: uint8_t readGyroNormalInvers(void)
 *
 * Overview: read gyro assembly from EEProm
 * @return: 1 if normal assembly, 2 if invers assembly
 ********************************************************************/
	uint8_t EEPromStorage::readGyroNormalInvers(void)
  {
    uint8_t retVal = 0;
    EEPROM.get(GyroNormalInversAddress,retVal);
    if((retVal <1)||(retVal >2))
    {
      retVal = 1; /* set to normal assembly if value is out of range */
    }
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("read gyro assembly = " + String(retVal));    
  #endif
    return(retVal); 
  }

/*********************************************************************
 * Method: void writeGyroNormalInvers(uint8_t gyroassembly)
 *
 * Overview: write gyroassembly to EEProm
 * @parameter: gyroassembly 1 = normal, 2 = invers
 ********************************************************************/
	void EEPromStorage::writeGyroNormalInvers(uint8_t gyroassembly)
  {
    EEPROM.put(GyroNormalInversAddress,gyroassembly);
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("write gyro assembly = " + String(gyroassembly));    
  #endif
  }

/*********************************************************************
 * Method: int readGyroHorizontal(void)
 *
 * Overview: read Servo GyroHorizontal value from EEProm and does plausibility check
 * @return: servo GyroHorizontal value
 ********************************************************************/
	int EEPromStorage::readGyroHorizontal(void)
  {
    int retVal = 0;
    EEPROM.get(GyroHorizontalAddress,retVal);
    if((retVal < 0)||(retVal >180))
      retVal = 0;
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("read servo gyro horizontal val = " + String(retVal));    
  #endif
    return(retVal);  
  }

/*********************************************************************
 * Method: void writeServoGyroHorizontal(int gyroFlightHorizontal)
 *
 * Overview: write servo gyroFlightHorizontal value to EEProm
 * @parameter: gyroFlightHorizontal teached servo gyro horizontal flight value
 ********************************************************************/
  void EEPromStorage::writeServoGyroHorizontal(int gyroFlightHorizontal)
  {
    EEPROM.put(GyroHorizontalAddress,gyroFlightHorizontal);
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("write servo gyro horizontal val = " + String(gyroFlightHorizontal));    
  #endif
  }

/*********************************************************************
 * Method: int readGyroThrottle(void)
 *
 * Overview: read Servo GyroThrottle value from EEProm and does plausibility check
 * @return: servo GyroThrottle value
 ********************************************************************/
	int EEPromStorage::readGyroThrottle(void)
  {
    int retVal = 0;
    EEPROM.get(GyroThrottleAddress,retVal);
    if((retVal < 0)||(retVal >180))
      retVal = 0;
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("read servo gyro throttle val = " + String(retVal));    
  #endif
    return(retVal);  
  }

/*********************************************************************
 * Method: void writeServoGyroThrottle(int gyroFlightThrottle)
 *
 * Overview: write servo gyroFlightThrottle value to EEProm
 * @parameter: gyroFlightThrottle teached servo gyro throttle flight value
 ********************************************************************/
  void EEPromStorage::writeServoGyroThrottle(int gyroFlightThrottle)
  {
    EEPROM.put(GyroThrottleAddress,gyroFlightThrottle);
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("write servo gyro throttle val = " + String(gyroFlightThrottle));    
  #endif
  }
 
 /*********************************************************************
 * Method: uint8_t readGyroExpo(void)
 *
 * Overview: read gyro expo value from EEProm and does plausibility check
 * @return: gyro expo value
 ********************************************************************/
  uint8_t EEPromStorage::readGyroExpo(void)
  {
    uint8_t retVal = 0;
    EEPROM.get(GyroExpoAddress,retVal);
    if((retVal < 0)||(retVal >100))
      retVal = 0;
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("read gyro expo val = " + String(retVal));    
  #endif
    return(retVal);  
  }
  
 /*********************************************************************
 * Method: void writeGyroExpo(uint8_t gyroExpo)
 *
 * Overview: write gyro expo value to EEProm
 * @parameter: gyroExpo teached gyro expo value
 ********************************************************************/
  void EEPromStorage::writeGyroExpo(uint8_t gyroExpo)
  {
    EEPROM.put(GyroExpoAddress,gyroExpo);
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("write gyro expo val = " + String(gyroExpo));    
  #endif
  }

 /*********************************************************************
 * Method: uint8_t readGyroFlightActive(void)
 *
 * Overview: read gyro flight active value from EEProm and does plausibility check
 * @return: gyro fight active value
 ********************************************************************/
  uint8_t EEPromStorage::readGyroFlightActive(void)
  {
    uint8_t retVal = 0;
    EEPROM.get(GyroFlightActiveAddress,retVal);
    if((retVal < 0)||(retVal >1))
      retVal = 0;
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("read gyro flight active val = " + String(retVal));    
  #endif
    return(retVal);  
  }
  
 /*********************************************************************
 * Method: void writeGyroFlightActive(uint8_t gyroFlightActive)
 *
 * Overview: write gyro flight active value to EEProm
 * @parameter: gyroFlightActive gyro flight active value
 ********************************************************************/
  void EEPromStorage::writeGyroFlightActive(uint8_t gyroFlightActive)
  {
    EEPROM.put(GyroFlightActiveAddress,gyroFlightActive);
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("write gyro flight active val = " + String(gyroFlightActive));    
  #endif
  }

 /*********************************************************************
 * Method: int readServoFunction(void)
 *
 * Overview: read function servo endposition value from EEProm
 * @return: endposition value from EEProm
 ********************************************************************/
  int EEPromStorage::readServoFunction(void)
  {
     int servoEndpos = 0;
     EEPROM.get(ServoFunctionAddress,servoEndpos);
  #ifdef TRACE_EEPROM_STORGE 
     Serial.println("read servo function endpos val = " + String(servoEndpos));    
  #endif
    return(servoEndpos);
  }

 /*********************************************************************
 * Method: void writeServoFunction(int servoEndpos)
 *
 * Overview: write function servo endposition value to EEProm
 * @parameter: function servo teached endposition value
 ********************************************************************/
	void EEPromStorage::writeServoFunction(int servoEndpos)
  {
     EEPROM.put(ServoFunctionAddress,servoEndpos);
   #ifdef TRACE_EEPROM_STORGE 
     Serial.println("write servo function endpos val = " + String(servoEndpos));    
   #endif
  }

  
 /*********************************************************************
 * Method: void writeServoFunctionActive(uint8_t active)
 *
 * Overview: write function servo function active value to EEProm
 * @parameter: function servo function active value
 * 0 = not active, 1 = active, 2 = active and reverse
 ********************************************************************/
void EEPromStorage::writeServoFunctionActive(uint8_t active)
  {
    EEPROM.put(ServoFunctionActiveAddress,active);
  #ifdef TRACE_EEPROM_STORGE
    Serial.println("write servo function active val = " + String(active)); 
  #endif
  }

/*********************************************************************
* Method: uint8_t readServoFunctionActive(void)
*
* Overview: read servo function active from EEProm
* @return: servo function active value from EEProm
* 0 = not active, 1 = active, 2 = active and reverse
********************************************************************/
 uint8_t EEPromStorage::readServoFunctionActive(void)
  {
      uint8_t tempVal = 0;
      EEPROM.get(ServoFunctionActiveAddress,tempVal);
    #ifdef TRACE_EEPROM_STORGE 
      Serial.println("read servo function active val = " + String(tempVal));    
    #endif
      return(tempVal);  
  }
 
/*********************************************************************
* Method: void writeServoFunctionReverse(uint8_t reverse)
*
* Overview: write function servo function reverse value to EEProm
* @parameter: function servo function reverse value
* 0 = normal, 1 = reverse
********************************************************************/
 void EEPromStorage::writeServoFunctionReverse(uint8_t reverse)
  {
    EEPROM.put(ServoFunctionReverseAddress,reverse);
  #ifdef TRACE_EEPROM_STORGE 
    Serial.println("write servo function reverse val = " + String(reverse));    
  #endif
  }

/*********************************************************************
* Method: uint8_t readServoFunctionReverse(void)
*
* Overview: read servo function reverse from EEProm
* @return: servo function reverse value from EEProm
* 0 = normal, 1 = reverse
********************************************************************/
 uint8_t EEPromStorage::readServoFunctionReverse(void)
  {
      uint8_t tempVal = 0;
      EEPROM.get(ServoFunctionReverseAddress,tempVal);
    #ifdef TRACE_EEPROM_STORGE 
      Serial.println("read servo function reverse val = " + String(tempVal));    
    #endif
      return(tempVal);  
  }


/*********************************************************************
 * Method: void writeServoFunctionSpeed(uint8_t speed)
 *
 * Overview: write function servo function speed value to EEProm
 * @parameter: function servo function speed value
 * 0 = off, 1 - 4 = speed
 ********************************************************************/
  void EEPromStorage::writeServoFunctionSpeed(uint8_t speed)
  {
    EEPROM.put(ServoFunctionSpeedAddress,speed);
  #ifdef TRACE_EEPROM_STORGE
    Serial.println("write servo function speed val = " + String(speed));
  #endif
}

/*********************************************************************
* Method: uint8_t readServoFunctionSpeed(void)
*
* Overview: read servo function speed from EEProm
* @return: servo function speed value from EEProm
* 0 = off, 1 - 4 = speed
********************************************************************/
  uint8_t EEPromStorage::readServoFunctionSpeed(void)
  {
    uint8_t tempVal = 0;
    EEPROM.get(ServoFunctionSpeedAddress,tempVal);
    #ifdef TRACE_EEPROM_STORGE 
    Serial.println("read servo function speed val = " + String(tempVal));    
    #endif
    return(tempVal);  
  }    

