/*
 	 Name:           GyroAccesSens.h
 	 Description:    declaration of gyroscope acceleration sensor MPU6050 object
 	 Copyright:      Geierwally, 2024(c)
 */
#ifndef GYRO_ACCEL_SENS_h
#define GYRO_ACCEL_SENS_h

#include "Arduino.h"
#include "TimerMs.h"
#include "EEPromStorage.h"
#include "LedControl.h"
#include "Kalman.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define teachMenuButton 1 /* switch teach menu state */


//#define TRACE_GYRO_ACCEL   /* console print gyroscope and acceleration sensor values */
//#define DEBUG_EXPO_CURVE
class GyroAccelSens
{
	public:

    enum GyroAccelTaskState
    {
        GyroAccel_Main,       /* main task gyro accel sensor */
        GyroAccel_Store,      /* store gyro accel sensor teach on eeprom */
        GyroAccel_Teach = 9   /* teach gyro accel sensor */
    } gyroAccelTaskState;     /* states of gyro accel task state machine */ 

    enum GyroTeachState
    {
        gt_idle,                /* wait for teach menu call */
        gt_sensorNormal,        /* sensor position normal in model assembly on top */
        gt_sensorInvers,        /* sensor position invers in model assembly on bottom */
        gt_calib,               /* calibrate gyroscope sensors */
        gt_flightPosHorizontal, /* teach flight value normal and invert horizontal flight */
        gt_flightPosThrottle,   /* teach the gyro flight throttle position */
        gt_Expo,                /* 0 - 100% expo curve to servo over gyro  */
        gt_flightActive,        /* set gyro flight active */
        gt_flightOff,           /* deactivate gyro flight */
        gt_leave,               /* leave teach task */
        gt_storeFlightPosHorizontal, /* store horizontal teach position in ServoControl object */
        gt_storeFlightPosThrottle    /* store throttle teach position in ServoControl object */
    }gyroTeachState;

    enum GyroAccelAssembly
    {
      gyroAccelAssemblyNormal = 1,
      gyroAccelAssemblyInvers
    } gyroAccelAssembly;
/*********************************************************************
 * Method: GyroAccelSens(uint8_t deviceAddress, LedControl * statusLed)
 *
 * Overview: constructor with parameters:
 * @param deviceAddress: I2C device address of MPU 6050, can be 0x68 or 0x69
 * @param statusLed:  reference to the status led object
 ********************************************************************/
    GyroAccelSens(uint8_t deviceAddress, LedControl * statusLed);

/*********************************************************************
 * Method: void Setup(uint8_t deviceAddress, LedControl * statusLed)
 *
 * Overview: initialize the gyroscope accelration sensor attributes
 ********************************************************************/	
    void Setup(void); 

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
    uint8_t Control(uint8_t buttonPressState); 
    
/*********************************************************************
 * Method: int getFlightAngleNick(uint8_t* gyroAssembly)
 *
 * Overview: call this nethod to get flight nick angle
 * @param: gyroAssembly reference to gyro assembly from servo control
 * @return calculated expo nick angle in degrees * 1000
 *      
 ********************************************************************/
    int getFlightAngleNick(uint8_t* gyroAssembly); 

/*********************************************************************
 * Method: bool getGyroFlightActive(void)
 *
 * Overview: call this nethod to get gyro flight is active
 * @return true if gyroFlightActive_ is 1 , otherwise false    
 ********************************************************************/
    bool getGyroFlightActive(void);

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
  void gyroTeach(uint8_t buttonPressState);
   
/*********************************************************************
 * Method: void calibrateGyro(void)
 *
 * Overview: calibrates the gyroscope sensor and write data into gyroError_
 *   
 ********************************************************************/
  void calibrateGyro(void);
 
/*********************************************************************
 * Method: void calculate_IMU_error(void)
 *
 * Overview: calculate the accelerometer and gyro data error
 * call this function on setup
 *      
 ********************************************************************/
  void calculate_IMU_error(void); 

/*********************************************************************
 * Method: void calculateRollNick(void)
 *
 * Overview: methode calculates roll and nick angles from sensor raw data
 *      
 ********************************************************************/
  void calculateRollNick(void); 

 /*********************************************************************
 * Method: int expo_Value(int x, short expo)
 *
 * Overview: methode calculates expo value from parameter x
 *           in range -90000 to 90000
 * @param x: input value for expo calculation
 * @param expo: the expo value in range -100 to 100 %
 * @returns: from x calculated expo value
 *      
 ********************************************************************/
  int expo_Value(int x, short expo);  
 
/*********************************************************************
 * Method: void debugExpoCurve(short absExpo)
 *
 * Overview: methode display possible throttle curves depending Expo
 * @param absExpo: the absolute expo value display curve - and + absExpo
 *   
 ********************************************************************/
  void debugExpoCurve(short absExpo);


  const uint32_t teachLEDTimeout_[12] = {50,1300,1600,1900,2200,2500,2800,3100,3400,3700,4000,4400}; /* 1000ms + led flash time */
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
  uint8_t device_Address_;     /* I2C device address of MPU6050, can be 0x68 or 0x69 */      
  LedControl * statusLed_;     /* reference to status LED object */ 
  TimerMs gyroAcceelTimer_;    /* delay timer for gyro accel calculation */
  float roll_;
  float nick_;
  float gier_;
  Kalman kalmanRoll_;
  Kalman kalmanNick_;
  uint8_t gyroAccelCtrlState_;
  uint8_t gyroAccelTeachState_;
  uint8_t expectedgyroAccelTeachState_;
  uint8_t gyroAccelExpo_;
  uint8_t gyroAccelExpoTeachIndex_;
  uint8_t gyroFlightActive_;
  TimerMs teachLEDTimer_;        /* timer for teach menu LED */
  EEPromStorage  eePromStorage_; /* instance of eeProm storage object */
  uint8_t gyroAccelAssembly_;    /* assembly state of gyroscope */
};

#endif

