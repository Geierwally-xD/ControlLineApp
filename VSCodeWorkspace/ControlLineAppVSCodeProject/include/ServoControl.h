/*
 	 Name:           ServoControl.h
 	 Description:    declaration of controlling servo motors object
 	 Copyright:      Geierwally, 2025(c)
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
     Servo_Calibrate,  /* calibrate the servo potentiometer */
     Servo_EndFlight   /* signal with throttle end of flight reached */ 
    } servoTaskState;  /* states of servo task state machine */ 
/*********************************************************************
 * Method: ServoControl(int controlPin, int servoPin, LedControl * statusLed , GyroAccelSens* gyroAccelSens)
 *
 * Overview: constructor with parameters:
 * @param controlPin: the analog input pin for input encoder
 * @param servoPin:   the digital output pwm pin for servo control
 * @param statusLed:  reference to the status led object
 * @param gyroAccelSens: reference to gyroscope accelerometer sensor object 
 ********************************************************************/
    ServoControl(int controlPin, int servoPin, LedControl * statusLed, GyroAccelSens* gyroAccelSens);

/*********************************************************************
 * Method: void Setup(void)
 *
 * Overview: initialize the servo control attributes
 ********************************************************************/
    void Setup(); 

 /*********************************************************************
 * Method: void Control(uint8_t buttonPressState, uint8_t servoFunctionTeachActive)
 *
 * Overview: the servo control task
 * - move servo to position of input encoder
 * - teach servo positions
 * - calibrate potentiometer
 * @param buttonPressState state of button control
 * @param servoFunctionTeachActive for locking throttle servo move during function servo teach
 * @return: the control state of task
 ********************************************************************/
    uint8_t Control(uint8_t buttonPressState, uint8_t servoFunctionTeachActive);

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


 /*********************************************************************
 * Method: void getPoti(*int PotiValues)
 *
 * Overview: returns calibrated poti min max values and current poti value
 * @param: PotiValues the poti values array 0 = min, 1 = max, 2 = current
 ********************************************************************/
   void getPoti(int * PotiValues);    

	private:
    enum calibrateState
    {
      calibrateMinVal,  /* initialize and start calibration for minimum value */
      calibrateMaxVal,  /* start calibration for maximum value */
      calibrateVerify,  /* verify minimum maximum value, exchange, if necessary */
      calibrateStore    /* write calibration values to EEProm */
    };

    enum functionControlState
    {
      functionIdle,   /* no function active */
      functionGyro,   /* gyro flight control active */
      functionRetract,/* retract landing gear active */
      function3,      /* third function active */
      function4,      /* fourth function active */
      function5,      /* fifth function active */
      functionSwitch, /* throttleposition on poti wait for reset or next function */
      functionExecute, /* execute selected function */
      functionWait4Idle /* wait for triggering idle state (functionControlThrottlePos_ == false) */
    };

 /*********************************************************************
 * Method: bool checkThrottlePos(void)
 *
 * Overview: check for throttle position reached and stay in position
 * @param: none
 * @return: true if throttle position reached, otherwise false
 ********************************************************************/
    bool checkThrottlePos(void);

 /*********************************************************************
 * Method: void gyroFlightNormal(void)
 *
 * Overview: check servo position for gyro flight control in servo 
 * normal mode and switch gyro flight on if gyro flight is active 
 * @param: none
 *******************************************************************/
    void gyroFlightNormal(void);

 /*********************************************************************
 * Method: void gyroFlightReverse(void)
 *
 * Overview: check servo position for gyro flight control in servo 
 * reverse mode and switch gyro flight on if gyro flight is active 
 * @param: none
 ********************************************************************/
    void gyroFlightReverse(void); 

  /*********************************************************************
 * Method: void servoMove(void)
 *
 * Overview: moves servo to calculated position if no calibration or eeProm storage
 * @param: none
 ********************************************************************/
    void servoMove(void);
 
 /*********************************************************************
 * Method: void functionControl(void)
 *
 * Overview: the function control task controls gyro flight on off
 * and retract landing gear over the throttle potentiometer
 * 1 short throttle pulses for gyro flight on/off
 * 2 short throttle pulses for retract landing gear in/out
 * 3 short throttle pulses for switching third digital function
 * 4 short throttle pulses for switching fourth digital function
 * 5 short throttle pulses for switching fifth digital function
 * @param: none
 ********************************************************************/
    void functionControl(void);
   
 /*********************************************************************
 * Method: bool checkLineCircuit(void)
 *
 * Overview: check for short circuit on control lines and if detected..
 * - move servo to throttle minimum (limit position) if gyro flight 
 *   disabled and line short throttle active
 * - stay in horizontal position if gyro flight active
 * - move servo to full throttle position if no line short throttle active
 * @param: none
 * @return: true if short circuit detected, otherwise false
 ********************************************************************/
   bool checkLineCircuit(void);

/*********************************************************************
 * Method: void function_Execute(uint8_t functionID)
 *
 * Overview: the function executes current statement depending functionID
 * @param: functionID the function ID to execute
 * 1 switch gyro flight on/off
 * 2 move retract landing gear in/out
 * 3 switch third digital function on/off
 * 4 switch fourth digital function on/off
 * 5 switch fifth digital function on/off
 ********************************************************************/
    void function_Execute(uint8_t functionID);             
	
	Servo servo_Throttle_;    /* attribute for throttle servo */
	int controlPin_ = A0;    /* attribute for analog pin used to connect the encoder potentiometer */
	int servoThrottlePin_;   /* attribute for throttle servo pwm output pin */
   int servoRetractPin_;    /* attribute for retract landing gear servo pwm output pin */
	int val_;                /* attribute for read value from the analog pin */
   int moveVal_;            /* servo move value */
   uint32_t servoDelay_;    /* wait delay for moving the servo */
   TimerMs servoTimer_;     /* delay timer for moving the servo */	
   TimerMs servoTeachTimer_; /* timer for teach reaction on short circuit */
   TimerMs servoEndFlightTimer_; /* timer for end flight reached */
   TimerMs functionControlTimer_; /* timer for function control switching */
   LedControl * statusLed_; /* reference to status LED object */ 
   uint8_t servoCtrlState_; /* state of servo control task */
   uint8_t functionControlState_; /* state of function control task */
   bool functionControlThrottlePos_; /* function control throttle position reached*/
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