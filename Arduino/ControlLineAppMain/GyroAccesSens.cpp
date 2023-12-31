/*
 	 Name:           GyroAccesSens.cpp
 	 Description:    definition of gyroscope acceleration sensor MPU6050 object
   according to https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
 	 Copyright:      Geierwally, 2023(c)
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
    }


/*********************************************************************
 * Method: void Setup(void)
 *
 * Overview: initialize the gyroscope accelration sensor attributes
 ********************************************************************/	
    void GyroAccelSens::Setup() 
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

      gyroAcceelTimer_.Start();
      while(gyroAcceelTimer_.IsExpired(20)==0)
      {} /* wait 20 ms */
    }

/*********************************************************************
 * Method: void Control(uint8_t buttonPressState)
 *
 * Overview: the gyroscope / accelerometer  control task
 * - read gyroscope and accelerometer raw data
 * - calculate resulting angle 
 * - define work settins      
 * @param buttonPressState state of button control
 ********************************************************************/
    void GyroAccelSens::Control(uint8_t buttonPressState)
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
      /* Calculating Roll and Pitch from the accelerometer data */
      accAngle_[AccX] = (atan(accValue_[AccY] / sqrt(pow(accValue_[AccX], 2) + pow(accValue_[AccZ], 2))) * 180 / PI); 
      accAngle_[AccY] = (atan(-1 * accValue_[AccX] / sqrt(pow(accValue_[AccY], 2) + pow(accValue_[AccZ], 2))) * 180 / PI); 
      /* Correct the outputs with the calculated error values */
      accAngle_[AccX] -= accError_[AccX]; /* AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details */
      accAngle_[AccY] -= accError_[AccY]; /* AccErrorY ~(-1.58) */
      /* === Read gyroscope data === */
      elapsedTime = gyroAcceelTimer_.GetRelativeTickCount() / 1000;
      gyroAcceelTimer_.Start();
      Wire.beginTransmission(device_Address_);
      Wire.write(GYRO_XOUT_H); /* Gyro data first register address 0x43 */
      Wire.endTransmission(false);
      byteCount = Wire.requestFrom((int)device_Address_, (int)6, (int)true); /* Read 4 registers total, each axis value is stored in 2 registers */
      gyroValue_[GyroX] = (Wire.read() << 8 | Wire.read()) / 131.0; /* For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet */
      gyroValue_[GyroY] = (Wire.read() << 8 | Wire.read()) / 131.0;
      gyroValue_[GyroZ] = (Wire.read() << 8 | Wire.read()) / 131.0;
      /* Correct the outputs with the calculated error values */
      gyroValue_[GyroX] -= gyroError_[GyroX];  /* GyroErrorX ~(-0.56) */
      gyroValue_[GyroY] -= gyroError_[GyroY];  /* GyroErrorY ~(2) */
      gyroValue_[GyroZ] -= gyroError_[GyroZ];  /* GyroErrorZ ~ (-0.8) */
      /* Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees */
      gyroAngle_[GyroX] += gyroValue_[GyroX] * elapsedTime; // deg/s * s = deg
      gyroAngle_[GyroY] += gyroValue_[GyroY] * elapsedTime;
      gier_ += gyroValue_[GyroZ] * elapsedTime;
      /* Complementary filter - combine acceleromter and gyro angle values */
      // roll_ = 0.96 * gyroAngle_[GyroX] + 0.04 * accAngle_[AccX];
      // nick_ = 0.96 * gyroAngle_[GyroY] + 0.04 * accAngle_[AccY];
      /* or use instead of complementary filter the kalman filter */
      roll_ = accAngle_[AccX] * RAD_TO_DEG;
      roll_ = kalmanRoll_.KalmanFilter(roll_, gyroValue_[GyroX], elapsedTime);
      nick_ = accAngle_[AccY] * RAD_TO_DEG;
      nick_ = kalmanNick_.KalmanFilter(nick_, gyroValue_[GyroY], elapsedTime);
  #ifdef TRACE_GYRO_ACCEL 
      /* Print the values on the serial monitor */
      Serial.println(String(roll_) + "/" + String(nick_) + "/" + String(gier_));
  #endif      
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
        accAngle_[AccX] = (atan(accValue_[AccY] / sqrt(pow(accValue_[AccX], 2) + pow(accValue_[AccZ], 2))) * 180 / PI); 
        accAngle_[AccY] = (atan(-1 * accValue_[AccX] / sqrt(pow(accValue_[AccY], 2) + pow(accValue_[AccZ], 2))) * 180 / PI); 
        /* Sum all readings */
        accError_[AccX] += accAngle_[AccX];
        accError_[AccY] += accAngle_[AccY];
      }
      /* Divide the sum by 200 to get the error value */
      accError_[AccX] /= 200;
      accError_[AccY] /= 200;

      /* Read gyro values 200 times */
      for (uint8_t j = 0;j < 200; j++) 
      {
        Wire.beginTransmission(device_Address_);
        Wire.write(GYRO_XOUT_H); /* Start with register 0x43 (GYRO_XOUT_H) */
        Wire.endTransmission(false);
        Wire.requestFrom((int)device_Address_, (int)6, (int)true); /* Read 6 registers total, each axis value is stored in 2 registers */
        gyroValue_[GyroX] = (Wire.read() << 8 | Wire.read()) / 131.0; /* For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet */
        gyroValue_[GyroY] = (Wire.read() << 8 | Wire.read()) / 131.0;
        gyroValue_[GyroZ] = (Wire.read() << 8 | Wire.read()) / 131.0;

        /* Sum all readings */
        gyroError_[GyroX] += gyroValue_[GyroX];
        gyroError_[GyroY] += gyroValue_[GyroY];
        gyroError_[GyroZ] += gyroValue_[GyroZ];
      }
      /* Divide the sum by 200 to get the error value */
      gyroError_[GyroX] /= 200;
      gyroError_[GyroY] /= 200;
      gyroError_[GyroZ] /= 200;
      /* Print the error values on the Serial Monitor */
      Serial.println("AccErrorX = " + String(accError_[AccX]));
      Serial.println("AccErrorY = " + String(accError_[AccY]));
      Serial.println("GyroErrorX = " + String(gyroError_[GyroX]));
      Serial.println("GyroErrorY = " + String(gyroError_[GyroY]));
      Serial.println("GyroErrorZ = " + String(gyroError_[GyroZ]));
    }
  