/*
 	 Name:           ServoControl.h
 	 Description:    declaration of controlling servo motors object
 	 Copyright:      Geierwally, 2023(c)
 */
#ifndef SERVO_CONTROL_h
#define SERVO_CONTROL_h

#include <Servo.h>
#include "Arduino.h"
#include "TimerMs.h"
#include "LedControl.h"
#include "EEPromStorage.h"
#include "GyroAccelSens.h"

//#define TRACE_SERVO_CONTROL   /* console print ADC value servo input */
class ServoControl
{
	public:

    enum ServoTaskState
    {
          Servo_Main,  /* main task servo control */
         Servo_Store,  /* store servo teach */
      Servo_Throttle,  /* servo teach throttle */
         Servo_Limit,  /* servo teach end position */
        Servo_Revers,  /* teach servo revers */
     Servo_LineShort,  /* reaction on control line short */
     Servo_Calibrate,  /* calibrate the servo potentiometer */
     Servo_EndFlight   /* signal with throttle end of flight reached */ 
    } servoTaskState;  /* states of servo task state machine */ 
/*********************************************************************
 * Method: ServoControl(int controlPin, int servoPin, uint32_t moveDelay, LedControl * statusLed , GyroAccelSens* gyroAccelSens)
 *
 * @param Overview: constructor with parameters:
 * @param controlPin: the analog input pin for input encoder
 * @param servoPin:   the digital output pwm pin for servo control
 * @param moveDelay:  the move delay in ms for servo to reach position
 * @param statusLed:  reference to the status led object
 * @param gyroAccelSens: reference to gyroscope accelerometer sensor object 
 ********************************************************************/
    ServoControl(int controlPin, int servoPin, uint32_t moveDelay, LedControl * statusLed, GyroAccelSens* gyroAccelSens);

/*********************************************************************
 * Method: void Setup(void)
 *
 * Overview: initialize the servo control attributes
 ********************************************************************/
    void Setup(); 

 /*********************************************************************
 * Method: void Control(uint8_t buttonPressState)
 *
 * Overview: the servo control task
 * - move servo to position of input encoder
 * - teach servo positions
 * - calibrate potentiometer
 * @param buttonPressState state of button control
 * @return: the control state of task
 ********************************************************************/
    uint8_t Control(uint8_t buttonPressState);

 /*********************************************************************
 * Method: void StoreGyroFlightPosHorizontal(void)
 *
 * Overview: store horizontal gyro flight pos in eeProm
 *
 ********************************************************************/
    void StoreGyroFlightPosHorizontal (void);

 /*********************************************************************
 * Method: void StoreGyroFlightPosThrottle(void)
 *
 * Overview: store throttle gyro flight pos in eeProm
 *
 ********************************************************************/
    void StoreGyroFlightThrottle (void);    

 /*********************************************************************
 * Method: void SignalThrottleEndFlight(void)
 *
 * Overview: signal with throttle end of flight reached
 *
 ********************************************************************/
    void SignalThrottleEndFlight (void);

	private:
    enum calibrateState
    {
      calibrateMinVal,  /* initialize and start calibration for minimum value */
      calibrateMaxVal,  /* start calibration for maximum value */
      calibrateVerify,  /* verify minimum maximum value, exchange, if necessary */
      calibrateStore    /* write calibration values to EEProm */
    };
	
	Servo servo_;            /* attribute for servo object to control a servo */
	int controlPin_ = A0;    /* attribute for analog pin used to connect the encoder potentiometer */
	int servoPin_;           /* attribute for servo pwm output pin */
	int val_;                /* attribute for read value from the analog pin */
  int moveVal_;            /* servo move value */
	uint32_t servoDelay_;    /* wait delay for moving the servo */
	TimerMs servoTimer_;     /* delay timer for moving the servo */	
  TimerMs servoTeachTimer_; /* timer for teach reaction on short circuit */
  TimerMs servoEndFlightTimer_; /* timer for end flight reached */
  LedControl * statusLed_; /* reference to status LED object */ 
  uint8_t servoCtrlState_; /* state of servo control task */
  bool servoReverse_;      /* if true servo reverse, otherwise normal */
  bool servoLineShortThrottle_; /* if true servo throttle at control line short, otherwise no throttle */
  int servoLimit_;         /* maximum throttle position */
  int servoThrottle_;      /* servo throttle position */
  int servoGyroHorizontal_;/* servo horizontal position for gyro flight */
  int servoGyroThrottle_;  /* servo throttle position for gyro flight */
  int servoCalibMinVal_;   /* calibrate minimum value of potentiometer */
  int servoCalibMaxVal_;   /* calibrate maximum value of potentiometer */
  uint8_t calibrationState_; /* state of calibration subtask */
  EEPromStorage  eePromStorage_; /* instance of eeProm storage object */
  GyroAccelSens * gyroAccelSens_; /* reference to gyroscope accelerometer sensor object */
  bool gyroFlightActive_;         /* true if gyro flight is active, otherwise, false */
 
/*********************************************************************
 * Method: void servoLineshortTeach(void)
 *
 * Overview: toggle between full throttle or no throttle at control -
 * line short
 ********************************************************************/	
    void servoLineshortTeach(void);
 
/*********************************************************************
 * Method: uint8_t servoCalibrate(uint8_t buttonPressState)
 *
 * Overview: calibration task
 * calibrates min and max positions of potentiometer -
 * @param: buttonPressState  the button press state of button control
 * @return: the servo control state  
 ********************************************************************/	
    uint8_t servoCalibrate(uint8_t buttonPressState);
};
#endif