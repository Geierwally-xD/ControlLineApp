/*
 	 Name:           ServoFunction.h
 	 Description:    declaration of function servo motors object
 	 Copyright:      Geierwally, 2025(c)
 */
#ifndef SERVO_FUNCTION_h
#define SERVO_FUNCTION_h

#include <Servo.h>
#include "ServoControl.h"
#include "Arduino.h"
#include "TimerMs.h"
#include "LedControl.h"
#include "EEPromStorage.h"
class ServoFunction
{
    public:
    enum ServoFunctionTaskState
    {
        ServoFunction_Main,       /* main task servo function */
        ServoFunction_EndposMove, /* move servo to treached end position */
        ServoFunction_LimitMove,  /* move servo to limit position */
        ServoFunction_TeachFunction = 6,   /* servo teach function position */
    } servoFunctionTaskState;     /* states of servo task state machine */ 

    /*********************************************************************
 * Method: ServoFunction(int servoPin, LedControl * statusLed)
 *
 * Overview:  constructor with parameters:
 * @param servoPin:  the digital output pwm pin for servo function
 * @param statusLed:  reference to the status led object
 * @param throttleServo_: reference to the throttle servo object
 ********************************************************************/
    ServoFunction(int servoPin, LedControl * statusLed, ServoControl *  throttleServo);

/*********************************************************************
 * Method: void Setup(void)
 *
 * Overview: initialize the servo function attributes
 ********************************************************************/
    void Setup(); 

 /*********************************************************************
 * Method: void Control(uint8_t buttonPressState)
 *
 * Overview: the servo control task
 * - move servo to position of input encoder
 * - teach servo endpoint position
 * @param buttonPressState state of button control
 * @return: the control state of task
 ********************************************************************/
    uint8_t Control(uint8_t buttonPressState);    

    private:
    enum PotiValues
    {
      PotiMinVal = 0, /* minimum value of potentiometer */
      PotiMaxVal = 1, /* maximum value of potentiometer */
      PotiCurVal = 2  /* current value of potentiometer */
    } potivalues; /* poti values array index */

    enum ServoFunctionTeachState
    {
        sft_Idle,       /* no teach function active */ 
        sft_Endpos,     /* teach servo end position */
        sft_Revers,     /* teach servo revers */
        sft_Speed,      /* teach servo speed */
        sft_DigitalSpecial,  /* special function */
        sft_Leave,      /* leave teach function */
    } servoFunctionTeachState; /* states of servo teach state machine */

     
/*********************************************************************
 * Method: uint8_t servoFuctionTeach(uint8_t buttonPressState)
 *
 * Overview: servo function teach task
 * teach function servo with following task substates
 * -function servo teach idle (wait for teach button press)
 * -function servo teach end position 1
 * -function servo teach revers       2
 * -function servo teach servo speed  3
 * -function servo digital function   4
 * -function servo teach leave        5
 * @param: buttonPressState  the button press state of button control
 * @return: true if teach done otherwise false
 * 
 ********************************************************************/	
    bool servoFuctionTeach(uint8_t buttonPressState);   

    const uint32_t teachLEDTimeout_[12] = {50,1300,1600,1900,2200,2500,2800,3100,3400,3700,4000,4400}; /* 1000ms + led flash time */
    Servo servo_;               /* servo object */
    ServoControl *  throttleServo_; /* reference to the throttle servo object */
    TimerMs servoTimer_;        /* timer for servo control */
    LedControl * statusLed_;    /* reference to the status led object */
    EEPromStorage eepromStore_; /* eeprom storage object */
    int servoPin_;              /* digital output pin for servo function */
    int servoPosition_;         /* current servo position */
    int servoEndpos_;           /* servo endposition value */
    bool servoRevers_;          /* servo revers flag */
    uint8_t servoFunctionActive_;  /* servo function active flag */
    uint8_t servoDelay_;        /* wait delay for moving the servo */
    uint8_t servoFunctionCtrlState_; /* state of function servo control task */
    TimerMs teachLEDTimer_;        /* timer for teach menu LED */
    uint8_t expectedFunctionServoTeachState_; /* expected teach menu state */
    uint8_t functionServoTeachState_; /* teach menu state */
    uint8_t prevServoFunctionState_;  /* previous function servo state */
    uint8_t servoFunctionTeachIndex_; /* index of function servo teach state */

    
 /*********************************************************************
 * Method: bool servoMove(int val)
 *
 * Overview: moves function servo to position
 * @param: move position
 * @return: true if move done
 ********************************************************************/
    bool servoMove(int val);

};


#endif