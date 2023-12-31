/*
 	 Name:           GyroAccesSens.h
 	 Description:    declaration of gyroscope acceleration sensor MPU6050 object
 	 Copyright:      Geierwally, 2023(c)
 */
#ifndef GYRO_ACCEL_SENS_h
#define GYRO_ACCEL_SENS_h

#include "Arduino.h"
#include "TimerMs.h"
#include "EEPromStorage.h"
#include "LedControl.h"
#include "Kalman.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define TRACE_GYRO_ACCEL   /* console print gyroscope and acceleration sensor values */
class GyroAccelSens
{
	public:

    enum GyroAccelTaskState
    {
        GyroAccel_Main,    /* main task gyro accel sensor */
        GyroAccerl_Store,   /* store gyro accel sensor mode */
    } gyroAccelTaskState;    /* states of gyro accel task state machine */ 
/*********************************************************************
 * Method: GyroAccelSens(uint8_t deviceAddress, LedControl * statusLed)
 *
 * Overview: constructor with parameters:
 * @param deviceAddress: I2C device address of MPU 6050, can be 0x68 or 0x69
 * @param statusLed:  reference to the status led object
 ********************************************************************/
    GyroAccelSens(uint8_t deviceAddress, LedControl * statusLed);

 /*********************************************************************
 * Method: void Setup(void)
 *
 * Overview: initialize the gyroscope accelerometer attributes
 ********************************************************************/
    void Setup(); 

 /*********************************************************************
 * Method: void Control(uint8_t buttonPressState)
 *
 * Overview: the gyroscope / accelerometer  control task
 * - read gyroscope and accelerometer raw data
 * - calculate resulting angle 
 * - define work settins      
 * @param buttonPressState state of button control
 ********************************************************************/
    void Control(uint8_t buttonPressState);   

    private:
    enum mpu6050Param
    {
      SDA            = 18,    /* I2C SDA pin (A4) */
      SCL            = 19,    /* I2C SCL pin (A5) */
      PWR_MGMT_1     = 0x6B,
      PWR_MGMT_2     = 0x6C,
      SMPLRT_DIV     = 0x19,
      CONFIG         = 0x1A,
      GYRO_CONFIG    = 0x1B,
      ACCEL_CONFIG   = 0x1C,
      INT_ENABLE     = 0x38,
      ACCEL_XOUT_H   = 0x3B,
      ACCEL_YOUT_H   = 0x3D,
      ACCEL_ZOUT_H   = 0x3F,
      GYRO_XOUT_H    = 0x43,
      GYRO_YOUT_H    = 0x45,
      GYRO_ZOUT_H    = 0x47
    };

    enum AccelIndex
    {
      AccX,
      AccY,
      AccZ
    };

    enum GyroIndex
    {
      GyroX,
      GyroY,
      GyroZ
    };

/*********************************************************************
 * Method: void calculate_IMU_error(void)
 *
 * Overview: calculate the accelerometer and gyro data error
 * call this function on setup
 *      
 ********************************************************************/
  void calculate_IMU_error(void); 

  const int ACCEL_OFFSET     = 200;
  const int GYRO_OFFSET      = -14;   // 151
  const float GYRO_SENSITITY = 16.4;  // 131 is sensivity of gyro from data sheet
  const float GYRO_SCALE     = 1;     //  0.02 by default - tweak as required
  const int accelAddress[3]  = {ACCEL_XOUT_H,ACCEL_YOUT_H,ACCEL_ZOUT_H};
  const int gyroAddress[3]   = {GYRO_XOUT_H,GYRO_YOUT_H,GYRO_ZOUT_H};  
  float LOOP_TIME_;                   // 0.1 = 100ms
  float accValue_[3]; 
  float accAngle_[3];
  float accError_[3];
  int temperature_ ; 
  int accCorr_;
  float gyroValue_[3];
  float gyroAngle_[3];
  float gyroError_[3];
  float gyroCorr;
  uint8_t device_Address_;    /* I2C device address of MPU6050, can be 0x68 or 0x69 */      
  LedControl * statusLed_;     /* reference to status LED object */ 
  TimerMs gyroAcceelTimer_;    /* delay timer for gyro accel calculation */
  float roll_;
  float nick_;
  float gier_;
  Kalman kalmanRoll_;
  Kalman kalmanNick_;
};
#endif

