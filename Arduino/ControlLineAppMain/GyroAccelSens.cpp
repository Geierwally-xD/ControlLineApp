/*
 	 Name:           GyroAccelSens.cpp
 	 Description:    definition of gyroscope acceleration sensor MPU6050 object
   according to https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
 	 Copyright:      Geierwally, 2024(c)
 */
 #include <Wire.h>
 #include "GyroAccelSens.h"

/*********************************************************************
 * Method: GyroAccelSens(uint8_t deviceAddress, LedControl * statusLed)
 *
 * Overview: constructor with parameters:
 * @param deviceAddress: I2C device address of MPU 6050, can be 0x68 or 0x69
 * @param statusLed:  reference to the status led object
 ********************************************************************/
    GyroAccelSens::GyroAccelSens(uint8_t deviceAddress, LedControl * statusLed)
    {
      device_Address_ = deviceAddress;
      statusLed_ = statusLed;
      LOOP_TIME_= 0.1; // 0.1 = 100ms
      for(uint8_t i = 0; i<3; i++)
      {
        accValue_[i] = 0.0; 
        accAngle_[i] = 0.0;
        accError_[i] = 0.0;
        gyroValue_[i] = 0.0;
        gyroAngle_[i] = 0.0;
        gyroError_[i] = 0.0;
      }
      temperature_ = 0; 
      accCorr_ = 0;

      gyroCorr  = 0.0;
      roll_ = 0.0;
      nick_ = 0.0;
      gier_ = 0.0;
      gyroAccelCtrlState_ = GyroAccel_Main;
      gyroAccelTeachState_ = gt_idle;
      expectedgyroAccelTeachState_ = gt_idle;
      gyroAccelAssembly_ = gyroAccelAssemblyNormal;
      teachLEDTimer_.Start();
      gyroAccelExpo_ = 0;
      gyroAccelExpoTeachIndex_ = 0;
      gyroFlightActive_ = 0;
    }


/*********************************************************************
 * Method: void Setup(uint8_t deviceAddress, LedControl * statusLed)
 *
 * Overview: initialize the gyroscope accelration sensor attributes
 *
 ********************************************************************/	
    void GyroAccelSens::Setup(void) 
    {
      Wire.begin();                             /* Initialize comunication */
      /* write sample rate register */
      Wire.beginTransmission(device_Address_);  /* Start communication with MPU6050 */
      Wire.write(SMPLRT_DIV);                   /* Talk to the register SMPLRT_DIV */
      Wire.write(0x07);                         /* place 0x07 into the SMPLRT_DIV register */
      Wire.endTransmission(true);               /* end the transmission */
      /* write power management register */
      Wire.beginTransmission(device_Address_);  /* Start communication with MPU6050 */
      Wire.write(PWR_MGMT_1);                   /* Talk to the register PWR_MGMT_1 */
      Wire.write(0x00);                         /* Make reset place a 0 into the PWR_MGMT_1 register */
      Wire.endTransmission(true);               /* end the transmission */
     /* write configuration register */
      Wire.beginTransmission(device_Address_);  /* Start communication with MPU6050 */
      Wire.write(CONFIG);                       /* Talk to the register CONFIG */
      Wire.write(0x00);                         /* place a 0 into the CONFIG register */
      Wire.endTransmission(true);               /* end the transmission */      
      /* Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g) */
      //Wire.beginTransmission(device_Address_);
      //Wire.write(ACCEL_CONFIG);                 /* Talk to the ACCEL_CONFIG register */
      //Wire.write(0x10);                         /* Set the register bits as 00010000 (+/- 8g full scale range) */
      //Wire.endTransmission(true);
      /* Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s) */
      //Wire.beginTransmission(device_Address_);
      //Wire.write(GYRO_CONFIG);                  /* Talk to the GYRO_CONFIG register */
      //Wire.write(0x10);                         /* Set the register bits as 00010000 (1000deg/s full scale) */
      //Wire.endTransmission(true);
      /* Configure interrupt enable register */
      //Wire.beginTransmission(device_Address_);
      //Wire.write(INT_ENABLE);                   /* Talk to the INT_ENABLE register */
      //Wire.write(0x01);                         /* place a 1 into the CONFIG register */
      //Wire.endTransmission(true);
      //delay(20);
      /* read gyroscope sensor calibration values from EEProm */
      eePromStorage_.readGyroCalib(& gyroError_[GyroX],& gyroError_[GyroY], & gyroError_[GyroZ]);
      /* Call this function if you need to get the IMU error values for your module */
      calculate_IMU_error();

      /* initialize start values and Kalman filter for X axis (roll) */
      accAngle_[AccX] -= accError_[AccX];
      gyroAngle_[GyroX] = accAngle_[AccX];
      roll_ = accAngle_[AccX] * RAD_TO_DEG;
      kalmanRoll_.setup(roll_);

      /* initialize start values and Kalman filter for Y axis (nick) */
      accAngle_[AccY] -= accError_[AccY];
      gyroAngle_[GyroY] = accAngle_[AccY];
      nick_ = accAngle_[AccY] * RAD_TO_DEG;
      kalmanNick_.setup(nick_);

      /* read gyro assembly value from eeProm */
      gyroAccelAssembly_ = eePromStorage_.readGyroNormalInvers();
      /* read gyro expo value from eeProm */
      gyroAccelExpo_ = eePromStorage_.readGyroExpo();
      gyroFlightActive_ = eePromStorage_.readGyroFlightActive();

      gyroAcceelTimer_.Start();
      while(gyroAcceelTimer_.IsExpired(20)==0)
      {} /* wait 20 ms */
      #ifdef DEBUG_EXPO_CURVE
      debugExpoCurve(50);
      #endif
    }

 /*********************************************************************
 * Method: void Control(uint8_t buttonPressState)
 *
 * Overview: the gyroscope / accelerometer  control task
 * - read gyroscope and accelerometer raw data
 * - calculate resulting angle 
 * - define work settins      
 * @param buttonPressState state of button control
 * @return !=0 if teach is active, otherwise 0
 ********************************************************************/
    uint8_t GyroAccelSens::Control(uint8_t buttonPressState)
    {
      calculateRollNick();        /* calculate the roll and nick angles from sensor raw data */
      if(buttonPressState = GyroAccel_Teach)
      {
        gyroAccelCtrlState_ = buttonPressState;
        gyroAccelTeachState_ = gt_idle;
        expectedgyroAccelTeachState_ = gt_idle;
      }
      switch(gyroAccelCtrlState_)
      {
        case GyroAccel_Main:
        break;
        case GyroAccel_Teach:
          gyroTeach(buttonPressState);
          if(gyroAccelTeachState_ == gt_leave)
          {/* teach was finished, return to main task state */
            gyroAccelCtrlState_ = GyroAccel_Main;
            gyroAccelTeachState_ = gt_idle;
          }
        break;
        case GyroAccel_Store:
        break;
        default:
        break;
      }
      
      return(gyroAccelCtrlState_ + gyroAccelTeachState_);      
    }

/*********************************************************************
 * Method: void calculate_IMU_error(void)
 *
 * Overview: calculate the accelerometer and gyro data error
 * call this function on setup
 *      
 ********************************************************************/
    void GyroAccelSens::calculate_IMU_error(void) 
    {
      uint8_t byteCount = 0;
      /* Read accelerometer values 200 times */
      for (uint8_t i = 0;i < 200; i++) 
      {
        Wire.beginTransmission(device_Address_);
        Wire.write(ACCEL_XOUT_H); /* Start with register 0x3B (ACCEL_XOUT_H) */
        Wire.endTransmission(false);
        byteCount = Wire.requestFrom((int)device_Address_, (int)6, (int)true); /* Read 6 registers total, each axis value is stored in 2 registers */
        /* For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet */
        accValue_[AccX] = (Wire.read() << 8 | Wire.read()) / 16384.0; /* X-axis value */
        accValue_[AccY] = (Wire.read() << 8 | Wire.read()) / 16384.0; /* Y-axis value */
        accValue_[AccZ] = (Wire.read() << 8 | Wire.read()) / 16384.0; /* Z-axis value */
        /* Calculating Roll and Pitch from the accelerometer data */
        accAngle_[AccX] = atan2(accValue_[AccY],accValue_[AccZ]) * 57.3;
        accAngle_[AccY] = atan2((-accValue_[AccX]),sqrt(accValue_[AccY] * accValue_[AccY] + accValue_[AccZ] * accValue_[AccZ])) * 57.3;
        //accAngle_[AccX] = (atan(accValue_[AccY] / sqrt(pow(accValue_[AccX], 2) + pow(accValue_[AccZ], 2))) * 180 / PI); 
        //accAngle_[AccY] = (atan(-1 * accValue_[AccX] / sqrt(pow(accValue_[AccY], 2) + pow(accValue_[AccZ], 2))) * 180 / PI); 
        /* Sum all readings */
        accError_[AccX] += accAngle_[AccX];
        accError_[AccY] += accAngle_[AccY];
      }
      /* Divide the sum by 200 to get the error value */
      accError_[AccX] /= 200;
      accError_[AccY] /= 200;
      /* Print the error values on the Serial Monitor */
  #ifdef TRACE_GYRO_ACCEL
      Serial.println("AccErrorX = " + String(accError_[AccX]));
      Serial.println("AccErrorY = " + String(accError_[AccY]));
  #endif  
    }

/*********************************************************************
 * Method: int getFlightAngleNick(uint8_t* gyroAssembly)
 *
 * Overview: call this nethod to get flight nick angle
 * @param: gyroAssembly reference to gyro assembly from servo control
 * @return calculated expo nick angle in degrees * 1000    
 ********************************************************************/
    int GyroAccelSens::getFlightAngleNick(uint8_t* gyroAssembly) 
    {
      static int nickOffset = 0;  
      int tempOffset = 0;
      int nickAngle = nick_ * 100.0;
      nickAngle = expo_Value(nickAngle, gyroAccelExpo_);
      *gyroAssembly = gyroAccelAssembly_;    /* write teached gyro assembly to servo control */
      return(nickAngle);
    }

/*********************************************************************
 * Method: void calculateRollNick(void)
 *
 * Overview: methode calculates roll and nick angles from sensor raw data
 *      
 ********************************************************************/
    void GyroAccelSens::calculateRollNick(void) 
    {  
      float elapsedTime = 0.0;
      uint8_t byteCount = 0;
      /* === Read acceleromter data === */
      Wire.beginTransmission(device_Address_);
      Wire.write(ACCEL_XOUT_H); /* Start with register 0x3B (ACCEL_XOUT_H) */
      Wire.endTransmission(false);
      byteCount = Wire.requestFrom((int)device_Address_, (int)6 , (int)true); /* Read 6 registers total, each axis value is stored in 2 registers */
      /* For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet */
      accValue_[AccX] = (Wire.read() << 8 | Wire.read()) / 16384.0; /* X-axis value */
      accValue_[AccY] = (Wire.read() << 8 | Wire.read()) / 16384.0; /* Y-axis value */
      accValue_[AccZ] = (Wire.read() << 8 | Wire.read()) / 16384.0; /* Z-axis value */
      /* Calculating Roll and Pitch from the accelerometer data according to https://wiki.dfrobot.com/How_to_Use_a_Three-Axis_Accelerometer_for_Tilt_Sensing*/
      accAngle_[AccX] = atan2(accValue_[AccY],accValue_[AccZ]) * RAD_TO_DEG;
      accAngle_[AccY] = atan2((-accValue_[AccX]),sqrt(accValue_[AccY] * accValue_[AccY] + accValue_[AccZ] * accValue_[AccZ])) * RAD_TO_DEG;

      /* instead of */
      //accAngle_[AccX] = (atan(accValue_[AccY] / sqrt(pow(accValue_[AccX], 2) + pow(accValue_[AccZ], 2))) * RAD_TO_DEG); 
      //accAngle_[AccY] = (atan(-1 * accValue_[AccX] / sqrt(pow(accValue_[AccY], 2) + pow(accValue_[AccZ], 2))) * RAD_TO_DEG); 
      /* Correct the outputs with the calculated error values */
      accAngle_[AccX] -= accError_[AccX]; /* AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details */
      accAngle_[AccY] -= accError_[AccY]; /* AccErrorY ~(-1.58) */

      /* === Read gyroscope data === */
      elapsedTime = (float)gyroAcceelTimer_.GetRelativeTickCount() / 1000.0;
      gyroAcceelTimer_.Start();
      Wire.beginTransmission(device_Address_);
      Wire.write(GYRO_XOUT_H); /* Gyro data first register address 0x43 */
      Wire.endTransmission(false);
      byteCount = Wire.requestFrom((int)device_Address_, (int)6, (int)true); /* Read 4 registers total, each axis value is stored in 2 registers */
      gyroValue_[GyroX] = (Wire.read() << 8 | Wire.read()) / 131.0; /* For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet */
      gyroValue_[GyroY] = (Wire.read() << 8 | Wire.read()) / 131.0;
      gyroValue_[GyroZ] = (Wire.read() << 8 | Wire.read()) / 131.0;
      /* Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees */
      gyroAngle_[GyroX] += gyroValue_[GyroX] * elapsedTime; // deg/s * s = deg
      gyroAngle_[GyroY] += gyroValue_[GyroY] * elapsedTime;
      gyroAngle_[GyroZ] += gyroValue_[GyroZ] * elapsedTime;
      /* Correct the outputs with the calibration error values */
      gyroAngle_[GyroX] -= gyroError_[GyroX];
      gyroAngle_[GyroY] -= gyroError_[GyroY];
      gyroAngle_[GyroZ] -= gyroError_[GyroZ];

      /* Complementary filter - combine acceleromter and gyro angle values */
      // roll_ = 0.96 * gyroAngle_[GyroX] + 0.04 * accAngle_[AccX];
      // nick_ = 0.96 * gyroAngle_[GyroY] + 0.04 * accAngle_[AccY];

      /* or use instead of complementary filter the kalman filter */
      accAngle_[AccX]  = accAngle_[AccX] * RAD_TO_DEG;
      roll_ = kalmanRoll_.KalmanFilter(accAngle_[AccX] , gyroValue_[GyroX], elapsedTime);
      accAngle_[AccY] = accAngle_[AccY] * RAD_TO_DEG;
      nick_ = kalmanNick_.KalmanFilter(accAngle_[AccY], gyroValue_[GyroY], elapsedTime);

     /* angle correction for 360 degrees */       
     /* if ((nick_< 0) && (accValue_[AccZ] >= 0)) 
      { nick_ = fabs(nick_);}
      else if ((nick_ < 0) && (accValue_[AccZ] < 0)) 
      { nick_ = 180 - fabs(nick_);}
      else if ((nick_ > 0) && (accValue_[AccZ] < 0)) 
      { nick_ = 180 + fabs(nick_);}
      else if ((nick_ > 0) && (accValue_[AccZ] >= 0)) 
      { nick_ = 360 - fabs(nick_);}

      if ((roll_ < 0) && (accValue_[AccZ] >= 0)) 
      { roll_ = fabs(roll_);}
      else if ((roll_ < 0) && (accValue_[AccZ] < 0)) 
      { roll_ = fabs(roll_);}
      else if ((roll_ > 0) && (accValue_[AccZ] < 0)) 
      { roll_ = 360 - fabs(roll_);}
      else if ((roll_ > 0) && (accValue_[AccZ] >= 0)) 
      { roll_ = 360 - fabs(roll_);}*/

     /* for calculating resulting angle float resAngle =  asin(((sin((roll_) / RAD_TO_DEG) + sin((nick_) / RAD_TO_DEG))/sqrt(2))) * RAD_TO_DEG; */
  #ifdef TRACE_GYRO_ACCEL 
      /* Print the values on the serial monitor */
      //Serial.println(String(roll_) + "/" + String(nick_) + "/" + String(resAngle));
      Serial.println(String(nick_) + "/" + String(accValue_[AccZ]));
  #endif    
    }    

/*********************************************************************
 * Method: int expo_Value(int x, short expo)
 * implemented according to formula from 
 * https://forum.arduino.cc/t/expo-einstellung-rc-mathematik/1150995/97
 * Overview: methode calculates expo value from parameter x
 *           in range -9000 to 9000
 * @param x: input value for expo calculation
 * @param expo: the expo value in range -100 to 100 %
 * @returns: from x calculated expo value
 *      
 ********************************************************************/
    int GyroAccelSens::expo_Value(int x, short expo) 
    { 
      int Y;
      int valueRange = 900; /* in that case from -9000 to + 9000*/
      double expoQuotient = expo / 100.0;
      int absEq = abs(expoQuotient);
      int absX = abs(x);
      int subAbsX = valueRange - absX;

      if (expo < 0) 
      {
        if(x > 0)
        {Y = (absEq / pow(valueRange, 2)) * pow(subAbsX, 3) + (1 - absEq) * (subAbsX) - valueRange;} 
        else
        {Y =  valueRange - ((absEq / pow(valueRange, 2)) * pow(subAbsX, 3) + (1 - absEq) * (subAbsX));}          
      } 
      else 
      {
        Y = (expoQuotient / pow(valueRange, 2)) * pow(x, 3) + (1 - expoQuotient) * x;
      }
      return Y;
    }

/*********************************************************************
 * Method: void debugExpoCurve(short absExpo)
 *
 * Overview: methode display possible throttle curves depending Expo
 * @param absExpo: the absolute expo value display curve - and + absExpo
 *   
 ********************************************************************/
  void GyroAccelSens::debugExpoCurve(short absExpo) 
    { 
      int throttlecurve[11];

      //int steeringcurve;
      for (int x = -900; x <= 900; x++) 
      {
        Serial.print(x);
        Serial.print(";");
        for(uint8_t i = 0; i<11;i++)
        {
          throttlecurve[i] = expo_Value(x,i*10);
          if(throttlecurve[i] <0)
          {
            throttlecurve[i] = map(throttlecurve[i],-900, 0, 50, 700);
          }
          else
          {
            throttlecurve[i] = map(throttlecurve[i], 0,900,700, 1000);
          }
          if(i==10)
          {
            Serial.println(throttlecurve[i]);
          }
          else
          {
            Serial.print(throttlecurve[i]);
            Serial.print(";");
          }
        }
      }
    }

/*********************************************************************
 * Method: uint8_t gyroTeach(uint8_t buttonPressState)
 *
 * Overview: gyroTeach task
 * teach gyro with following task substates
 * -gyro teach idle (wait for teach button press)
 * -gyro calibration 1
 * -normal / invert flight position 2
 * -gyro throttle position  3
 * -gyro flight active 4
 * -gyro flight off    5
 * -leave gyro teach 6
 * @param: buttonPressState  the button press state of button control
 * 
 ********************************************************************/	
    void GyroAccelSens:: gyroTeach(uint8_t buttonPressState)
    {
      switch(gyroAccelTeachState_)
      {
        case gt_idle:                /* wait for teach menu call */
          /* flash LED for each teach state wait until button pressed */
          if(teachLEDTimer_.IsExpired(teachLEDTimeout_[expectedgyroAccelTeachState_]))
          {
            if(expectedgyroAccelTeachState_ < gt_leave)
            {
              expectedgyroAccelTeachState_ ++;
              statusLed_->FlashLed(expectedgyroAccelTeachState_,100,200);
              teachLEDTimer_.Start();
            }
            else
            {
              expectedgyroAccelTeachState_ = gt_idle;
              teachLEDTimer_.Start();
            }
          }
          if(buttonPressState == teachMenuButton)
          { /* switch to teach state if button was pressed */
            gyroAccelTeachState_ = expectedgyroAccelTeachState_;
            gyroAccelExpoTeachIndex_ = 0;
          }
        break;
        case gt_sensorNormal:        /* sensor position normal in model assembly on top */
          gyroAccelAssembly_ = gyroAccelAssemblyNormal;
          eePromStorage_.writeGyroNormalInvers(gyroAccelAssembly_);
          gyroAccelTeachState_ = gt_idle;
          expectedgyroAccelTeachState_ = gt_idle;
        break;
        case gt_sensorInvers:        /* sensor position invers in model assembly on bottom */
          gyroAccelAssembly_ = gyroAccelAssemblyInvers;
          eePromStorage_.writeGyroNormalInvers(gyroAccelAssembly_);
          gyroAccelTeachState_ = gt_idle;
          expectedgyroAccelTeachState_ = gt_idle;        
        break;
        case gt_calib:               /* calibrate gyroscope sensors */
          calibrateGyro();
          /* write gyro calib to EEProm */
          eePromStorage_.writeGyroCalib(gyroError_[GyroX],gyroError_[GyroY], gyroError_[GyroZ]);
          gyroAccelTeachState_ = gt_idle;
          expectedgyroAccelTeachState_ = gt_idle;
        break;
        case gt_flightPosHorizontal: /* teach flight value normal and invert horizontal flight */
          gyroFlightActive_ = 0; /* reset gyro flight active on teach */
          if(buttonPressState == teachMenuButton)
          { /* switch to teach state if button was pressed */
            gyroAccelTeachState_ = gt_storeFlightPosHorizontal;
          }
        break;
        case gt_flightPosThrottle:   /* teach the gyro flight throttle position */
          gyroFlightActive_ = 0; /* reset gyro flight active on teach */
          if(buttonPressState == teachMenuButton)
          { /* switch to teach state if button was pressed */
            gyroAccelTeachState_ = gt_storeFlightPosThrottle;
          }
        break;
        case gt_Expo:                /* 0 - 100% expo curve to servo over gyro  */
          if(teachLEDTimer_.IsExpired(teachLEDTimeout_[gyroAccelExpoTeachIndex_]))
          {
            if(gyroAccelExpoTeachIndex_ < 11)
            {
              gyroAccelExpoTeachIndex_ ++;
              if(gyroAccelExpoTeachIndex_ < 11)
              {
                statusLed_->FlashLed(gyroAccelExpoTeachIndex_,100,200);
              }
              else
              { /* no expo, flash 200ms */
                statusLed_->FlashLed(gyroAccelExpoTeachIndex_,200,200);
              }
              teachLEDTimer_.Start();
            }
            else
            {
              gyroAccelExpoTeachIndex_ = 0;
              teachLEDTimer_.Start();
            }
          }
          if(buttonPressState == teachMenuButton)
          { /* switch to teach state if button was pressed */
            switch(gyroAccelExpoTeachIndex_)
            {
              case 0:
              case 11:
                gyroAccelExpo_ = 0;
              break;
              default:
                gyroAccelExpo_ = gyroAccelExpoTeachIndex_ * 10;
              break;
            }
            eePromStorage_.writeGyroExpo(gyroAccelExpo_);
            gyroAccelTeachState_ = gt_idle;
            expectedgyroAccelTeachState_ = gt_idle;    
          }
        break;
        case gt_flightActive:        /* set gyro flight active */
          gyroFlightActive_ = 1;
          eePromStorage_.writeGyroFlightActive(gyroFlightActive_);
        break;        
        case gt_flightOff:           /* deactivate gyro flight */
          gyroFlightActive_ = 0;
          eePromStorage_.writeGyroFlightActive(gyroFlightActive_);
        break;
        case gt_leave:               /* leave teach task */
        break;    
        case gt_storeFlightPosHorizontal: /* store horizontal teach position in ServoControl object */
          gyroFlightActive_ = eePromStorage_.readGyroFlightActive(); /* read previous gyro flight active */
          gyroAccelTeachState_ = gt_idle;
          expectedgyroAccelTeachState_ = gt_idle;
        break;   
        case gt_storeFlightPosThrottle: /* store throttle teach position in ServoControl object */
          gyroFlightActive_ = eePromStorage_.readGyroFlightActive(); /* read previous gyro flight active */
          gyroAccelTeachState_ = gt_idle;
          expectedgyroAccelTeachState_ = gt_idle;
        break;   
        default:
        break;
      }
    }

/*********************************************************************
 * Method: void calibrateGyro(void)
 *
 * Overview: calibrates the gyroscope sensor and write data into gyroError_
 *   
 ********************************************************************/
  void GyroAccelSens:: calibrateGyro(void)
  {
	  TimerMs calibrationTimer;
	  TimerMs gyroCycleTimer;
    uint32_t gyroCycleTime = 0;
    float elapsedTime = 0;
    Wire.beginTransmission(device_Address_);
    Wire.write(GYRO_XOUT_H); /* Start with register 0x43 (GYRO_XOUT_H) */
    Wire.endTransmission(false);
    Wire.requestFrom((int)device_Address_, (int)6, (int)true); /* Read 6 registers total, each axis value is stored in 2 registers */
    gyroValue_[GyroX] = (Wire.read() << 8 | Wire.read()) / 131.0; /* For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet */
    gyroValue_[GyroY] = (Wire.read() << 8 | Wire.read()) / 131.0;
    gyroValue_[GyroZ] = (Wire.read() << 8 | Wire.read()) / 131.0;
    calibrationTimer.Start();
    gyroCycleTimer.Start();
    /* 60 sec calibration time */
	  while(calibrationTimer.IsExpired(60000)==false)
	  {
	    gyroCycleTime = gyroCycleTimer.GetRelativeTickCount();
      gyroCycleTimer.Start();
      Wire.beginTransmission(device_Address_);
      Wire.write(GYRO_XOUT_H); /* Start with register 0x43 (GYRO_XOUT_H) */
      Wire.endTransmission(false);
      Wire.requestFrom((int)device_Address_, (int)6, (int)true); /* Read 6 registers total, each axis value is stored in 2 registers */
      gyroValue_[GyroX] = (Wire.read() << 8 | Wire.read()) / 131.0; /* For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet */
      gyroValue_[GyroY] = (Wire.read() << 8 | Wire.read()) / 131.0;
      gyroValue_[GyroZ] = (Wire.read() << 8 | Wire.read()) / 131.0;      
		  elapsedTime = (float)gyroCycleTime / 1000.0;
      /* Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees */
      gyroAngle_[GyroX] += gyroValue_[GyroX] * elapsedTime; // deg/s * s = deg
      gyroAngle_[GyroY] += gyroValue_[GyroY] * elapsedTime;
      gyroAngle_[GyroZ] += gyroValue_[GyroZ] * elapsedTime;
  		while(gyroCycleTimer.IsExpired(10)==false) /* 10ms gyro cycle*/
		  {}
	  }
	  gyroError_[GyroX] = gyroAngle_[GyroX] / 60;
	  gyroError_[GyroY] = gyroAngle_[GyroY] / 60;
	  gyroError_[GyroZ] = gyroAngle_[GyroZ] / 60;
      /* Print the error values on the Serial Monitor */
  #ifdef TRACE_GYRO_ACCEL
      Serial.println("GyroErrorX = " + String(gyroError_[GyroX]));
      Serial.println("GyroErrorY = " + String(gyroError_[GyroY]));
      Serial.println("GyroErrorZ = " + String(gyroError_[GyroZ]));
  #endif 
}

/*********************************************************************
 * Method: bool getGyroFlightActive(void)
 *
 * Overview: call this nethod to get gyro flight is active
 * @return true if gyroFlightActive_ is 1 , otherwise false    
 ********************************************************************/
bool GyroAccelSens::getGyroFlightActive(void) 
{
  return((gyroFlightActive_ == 1) ? true:false);
}
