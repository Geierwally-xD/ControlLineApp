/*
 	 Name:           ServoControl.cpp
 	 Description:    definition of controlling servo motors object
 	 Copyright:      Geierwally, 2025(c)
 */
#include "ServoControl.h"
#include "ServoFunction.h"

/*********************************************************************
 * Method: ServoControl(int controlPin, int servoPin, uint32_t moveDelay, LedControl * statusLed)
 *
 * Overview: constructor with parameters:
 * controlPin: the analog input pin for input encoder
 * servoPin:   the digital output pwm pin for servo control
 * statusLed: reference to the status led object
 * gyroAccelSens: reference to gyro accelerator sensor object
 ********************************************************************/
    ServoControl::ServoControl(int controlPin, int servoPin, LedControl * statusLed, GyroAccelSens* gyroAccelSens)
    {
      controlPin_ = controlPin;
      servoThrottlePin_ = servoPin;
      servoDelay_ = 0;
      statusLed_ = statusLed;
      gyroAccelSens_ = gyroAccelSens;
      servoCtrlState_ = Servo_Main;
      servoReverse_ = false;         /* if true servo reverse, otherwise normal */
      servoLineShortThrottle_ = false; /* no throttle on control line short active as default */
      servoLimit_ = 10;              /* maximum throttle position */
      servoThrottle_ = 20;          /* servo throttle position */
      servoGyroHorizontal_ = 10;    /* servo horizontal position for gyro flight */
      servoGyroThrottle_ = 20;      /* servo throttle position for gyro flight */
      moveVal_ = 90;                /* servo move position */
      servoCalibMinVal_ = 538;      /* servo potentiometer minimum ADC calib value */
      servoCalibMaxVal_ = 1023;      /* servo potentiometer maximum ADC calib value */
      calibrationState_ = calibrateMinVal; /* set start value for calibration task */
      functionControlState_ = functionIdle;
      functionControlThrottlePos_ = false;
    }

/*********************************************************************
 * Method: void Setup(void)
 *
 * Overview: initialize the servo control attributes
 ********************************************************************/	
    void ServoControl::Setup() 
    {
      servo_Throttle_.attach(servoThrottlePin_);    /* attaches the servo on pin 9 to the servo object */
      servoTimer_.Start();
      servoTeachTimer_.Start();
      servoEndFlightTimer_.Start();
      functionControlTimer_.Start();
      servoCtrlState_ = Servo_Main;
      servoDelay_ = gyroAccelSens_->getGyroServoSensitivity();
      moveVal_ = 90;                /* servo move position */
      servoCalibMinVal_ = eePromStorage_.readCalMinVal();
      servoCalibMaxVal_ = eePromStorage_.readCalMaxVal();
      servoThrottle_    = eePromStorage_.readServoThrottle();
      servoGyroHorizontal_ = eePromStorage_.readGyroHorizontal();
      servoGyroThrottle_ = eePromStorage_.readGyroThrottle();
      servoLimit_       = eePromStorage_.readServoLimit();
      servoLineShortThrottle_ = false; /* set true, if full throttle at line short requested */
      servoReverse_     = eePromStorage_.readServoRevers();
      functionControlThrottlePos_ = false;
      functionControlState_ = functionIdle;
    }

/*********************************************************************
 * Method: uint8_t servoCalibrate(uint8_t buttonPressState)
 *
 * Overview: calibration task
 * calibrates min and max positions of potentiometer -
 * @param: buttonPressState  the button press state of button control
 * @return: the servo control state  
 ********************************************************************/	
    uint8_t ServoControl::servoCalibrate(uint8_t buttonPressState)
    {
      uint8_t controlState = Servo_Calibrate;
      switch (calibrationState_)
      {
        case calibrateMinVal:  /* initialize and start calibration for minimum value */
          if(buttonPressState == Servo_Store)
          {
            calibrationState_ = calibrateMaxVal;
          }
          else
          {
            /* reads the minimum value of the potentiometer (value between 0 and 1023) */
            servoCalibMinVal_ = analogRead(controlPin_);      
            if(servoTeachTimer_.IsExpired(1300))
            {
              servoTeachTimer_.Start();
              statusLed_->FlashLed(1,200,100);
            }
          }
        break;
        case calibrateMaxVal:  /* start calibration for maximum value */
          if(buttonPressState == Servo_Store)
          {
            calibrationState_ = calibrateVerify;
          }
          else
          {
            /* reads the maximum value of the potentiometer (value between 0 and 1023) */
            servoCalibMaxVal_ = analogRead(controlPin_);      
            if(servoTeachTimer_.IsExpired(1600))
            {
              servoTeachTimer_.Start();
              statusLed_->FlashLed(2,200,100);
            }
          }
        break;
        case calibrateVerify:  /* verify minimum maximum value, exchange, if necessary */
          if(servoCalibMinVal_ > servoCalibMaxVal_)
          {
            int swabVal = servoCalibMinVal_;
            servoCalibMinVal_ = servoCalibMaxVal_;
            servoCalibMaxVal_ = swabVal;
          }
          calibrationState_ = calibrateStore;
        break;
        case calibrateStore:    /* write calibration values to EEProm */
          Serial.println("calibrate minimum = " + String(servoCalibMinVal_) + "\ncalibrate maximum = " + String(servoCalibMaxVal_));
          calibrationState_ = calibrateMinVal; /* reset sub task calibrate state */
          /* store values on EEProm */
          controlState = Servo_Store; 
          eePromStorage_.writeCalMaxVal(servoCalibMaxVal_);
          eePromStorage_.writeCalMinVal(servoCalibMinVal_);
        break;
        default:
        break;
      }
      return(controlState);
    }

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
    bool ServoControl::checkLineCircuit(void)
    {
      bool lineCircuit = false;
      if(false == servoReverse_)
      { /* no servo reverse */
            if((val_ > 178)&&(servoLineShortThrottle_)&&(!gyroFlightActive_))
            { /* move servo to minimum throttle if gyro flight is inactive 
                 and line short throttle is active */    
              val_ = servoLimit_;
              lineCircuit = true;
            } 
            else if(val_ < servoLimit_)
            { /* safe position, stay in minimum throttle if position < servo limit */
              val_ = servoLimit_;
              lineCircuit = true;
            }
            /* otherwise, stay in calculated position */
      }
      else
      { /* servo reverse */
            if((val_ < 26)&&(servoLineShortThrottle_)&&(!gyroFlightActive_))
            { /* move servo to minimum throttle if gyro flight is inactive 
                 and line short throttle is active */    
              val_ = servoLimit_;
              lineCircuit = true;
            } 
            else if(val_ > servoLimit_)
            { /* safe position, stay in minimum throttle if position > servo limit */
              val_ = servoLimit_;
              lineCircuit = true;
            }
            /* otherwise, stay in calculated position */
      }

      return(lineCircuit);
    } 

 /*********************************************************************
 * Method: bool checkThrottlePos(void)
 *
 * Overview: check for throttle position reached and stay in position
 * @param: none
 * @return: true if throttle position reached, otherwise false
 ********************************************************************/
    bool ServoControl::checkThrottlePos(void)
    {
      bool servoThrottlePosition = false;
      if(  ((false == servoReverse_)&&(val_ < servoThrottle_))
         ||((true == servoReverse_)&&(val_ > servoThrottle_))
        ) 
      {
        val_ = servoThrottle_;  /* set servo safe position (teached throttle position) */
        servoThrottlePosition = true;
        functionControlThrottlePos_ = true; /* inform functioncontrol that throttleposition reached */
      }
      return(servoThrottlePosition);
    }

 /*********************************************************************
 * Method: void gyroFlightNormal(void)
 *
 * Overview: check servo position for gyro flight control in servo 
 * normal mode and switch gyro flight on if gyro flight is active 
 * @param: none
 ********************************************************************/
    void ServoControl::gyroFlightNormal(void)
    {
      uint8_t gyroAssembly = 0;
      if(val_ > servoGyroHorizontal_) /* flight angle control active and position > servo gyro horizontal */
      { 
        functionControlThrottlePos_ = false; /* inform functioncontrol that throttleposition leafed */
        if(gyroFlightActive_) 
        {            
          int nickAngle = gyroAccelSens_->getFlightAngleNick(& gyroAssembly);
          if(gyroAssembly == GyroAccelSens::gyroAccelAssemblyNormal)
          {
            if(nickAngle <0)
            {
              val_ = map(nickAngle,-900, 0, servoGyroHorizontal_, 180);
            } 
            else
            {
              val_ = map(nickAngle,0,900, servoGyroThrottle_, servoGyroHorizontal_);
            } 
          }
          else
          { /* assembly gyro inverse */
            if(nickAngle <0)
            {
              val_ = map(nickAngle,0,-900, servoGyroHorizontal_, 180);
            } 
            else
            {
              val_ = map(nickAngle,900,0, servoGyroThrottle_, servoGyroHorizontal_);
            }
          }
          if(val_ < servoGyroThrottle_)
          {
              val_ = servoGyroThrottle_;
          }
        } /* gyro flight active */
      } /* val_ > servoGyroHorizontal_ */
    }

 /*********************************************************************
 * Method: void gyroFlightReverse(void)
 *
 * Overview: check servo position for gyro flight control in servo 
 * reverse mode and switch gyro flight on if gyro flight is active 
 * @param: none
 ********************************************************************/
    void ServoControl::gyroFlightReverse(void)
    {
      uint8_t gyroAssembly = 0;
      if(val_ < servoGyroHorizontal_) /* flight angle control active and position < servo gyro horizontal */
      {
        functionControlThrottlePos_ = false; /* inform functioncontrol that throttleposition leafed */
        if(gyroFlightActive_)
        {
          int nickAngle = gyroAccelSens_->getFlightAngleNick(& gyroAssembly);
          if(gyroAssembly == GyroAccelSens::gyroAccelAssemblyNormal)
          {
            if(nickAngle <0)
            {
              val_ = map(nickAngle,-900,0, servoGyroHorizontal_, 25);
            }
            else
            {
              val_ = map(nickAngle,0,900, servoGyroThrottle_, servoGyroHorizontal_);
            }               
          }
          else
          {
            if(nickAngle <0)
            {       
              val_ = map(nickAngle,0, -900, servoGyroHorizontal_, 25);
            }
            else
            {
              val_ = map(nickAngle, 900,0, servoGyroThrottle_, servoGyroHorizontal_);
            }               
          }
          if(val_ > servoGyroThrottle_)
          {
            val_ = servoGyroThrottle_;
          } 
        }   /* gyroFlightActive_ */           
      }  /* val_ < servoGyroHorizontal_ */    
  }

 /*********************************************************************
 * Method: void servoMove(void)
 *
 * Overview: moves servo to calculated position if no calibration or eeProm storage
 * @param: none
 ********************************************************************/
    void ServoControl::servoMove(void)
    {
      if((servoCtrlState_ != Servo_Calibrate)&&(servoCtrlState_ != Servo_Store))
      { /* lock servo move on poti endpoint calibration and eeprom storage */
        if(servoTimer_.IsExpired(servoDelay_)) /* waits for the servo to get there */
        {
          if(moveVal_ < val_)
            {moveVal_ ++;}
          else if(moveVal_ > val_)
            {moveVal_ --;}
 
  #ifdef TRACE_SERVO_CONTROL
          Serial.println(moveVal_);            /* servo move position */
  #endif      
          servo_Throttle_.write(moveVal_);                  /* sets the servo position according to the scaled value */
          servoTimer_.Start();                           
        }
      }
    }

 /*********************************************************************
 * Method: void getPoti(*int PotiValues)
 *
 * Overview: returns calibrated poti min max values and current poti value
 * @param: PotiValues the poti values array 0 = min, 1 = max, 2 = current
 ********************************************************************/
  void ServoControl::getPoti(int * PotiValues)
  {
    PotiValues[0] = servoCalibMinVal_;
    PotiValues[1] = servoCalibMaxVal_;
    PotiValues[2] = analogRead(controlPin_);
  }

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
    void ServoControl::functionControl(void)
    {
      static uint8_t prevFunctionControlState = functionIdle;  
      switch (functionControlState_)
      {
        case functionIdle:   /* no function active */
        case functionGyro:   /* gyro flight control active */
        case functionRetract:/* retract landing gear active */
        case function3:      /* third function active */
        case function4:      /* fourth function active */
        case function5:      /* fifth function active */
          if (functionControlThrottlePos_ == true)
          { /* throttleposition reached, prepare next switch */
            prevFunctionControlState = functionControlState_;
            functionControlState_ = functionSwitch;
            functionControlTimer_.Start();
          }
          else
          {
            if(true == functionControlTimer_.IsExpired(2000))
            { /* 2 seconds expired, execute selected function */
              functionControlState_ = (functionControlState_ != functionIdle)? functionExecute : functionIdle;
            } 
          } 
        break;
        case functionSwitch:  /* throttleposition on poti wait for reset or next function */
          if (functionControlThrottlePos_ == false)
          { /* switch to next function or idle if last function reached */
            functionControlState_ = (++prevFunctionControlState < functionSwitch)?prevFunctionControlState:functionIdle;  
            functionControlTimer_.Start();
          }
          else
          {
            if(true == functionControlTimer_.IsExpired(2000))
            {
              functionControlState_ = functionWait4Idle;
            } 
          } 
        break;
        case functionExecute: /* execute selected function */
          function_Execute(prevFunctionControlState); /* execute selected function */
          functionControlState_ = functionIdle;
          functionControlTimer_.Start();
        break;
        case functionWait4Idle: /* wait for triggering idle state (functionControlThrottlePos_ == false) */
          if (functionControlThrottlePos_ == false)
          { /* switch to idle if throttle position reached */
            functionControlState_ = functionIdle;
          }
        break;
        default:
        break;
      }
    } 

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
    void ServoControl::function_Execute(uint8_t functionID)
    {
      switch(functionID)
      {
        case functionGyro: /* switch gyro flight on/off */
        if(gyroAccelSens_->getGyroFlightActive())
        { 
          gyroFlightActive_ = !gyroFlightActive_;
          servoTeachTimer_.Start();
          if(gyroFlightActive_)
          { statusLed_->FlashLed(2,200,100);} /* gyro flight is enabled flash led two times */
          else
          { statusLed_->FlashLed(1,200,100);} /* gyro flight is disabled flash led one time */ 
        }
        else
        {
          gyroFlightActive_ = false;
        }
        break;
        case functionRetract: /* move retract landing gear in/out */
          statusLed_->SwitchFunction(functionRetract);
        break;
        case function3: /* switch third digital function on/off */
          statusLed_->FlashLed(3,200,100);
          statusLed_->SwitchFunction(function3);
        break;
        case function4: /* switch fourth digital function on/off */
          statusLed_->FlashLed(4,200,100);
          statusLed_->SwitchFunction(function4);
        break;
        case function5: /* switch fifth digital function on/off */
          statusLed_->FlashLed(5,200,100);
          statusLed_->SwitchFunction(function5);
        break;
        default:
        break;
      }
    }

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
    uint8_t ServoControl::Control(uint8_t buttonPressState, uint8_t servoFunctionTeachActive) 
    {
      val_ = analogRead(controlPin_);      /* reads the value of the potentiometer (value between 0 and 1023) */

      if(false == servoReverse_)
        {val_ = map(val_, servoCalibMinVal_, servoCalibMaxVal_, 25, 180);}   /* scale it for use with the servo (value between 0 and 180) */ 
      else
        {val_ = map(val_, servoCalibMinVal_, servoCalibMaxVal_, 180, 25);}
      switch(servoCtrlState_)
      {
        case Servo_Main:      /* main task servo control */
          if(  (buttonPressState > Servo_Store)
             &&(buttonPressState <= Servo_Calibrate)
            )
          {
            servoCtrlState_ = buttonPressState;
          }

          if(! checkLineCircuit())
          { /* not line circuit move detected */
            if(! checkThrottlePos())
            { /* no throttle position detected */
              if(false == servoReverse_)
              { /* servo normal  */
                gyroFlightNormal();
              }
              else
              { /* servo reverse */
                gyroFlightReverse();
              }
            } /* no throttle position detected */ 
          } /* not line circuit move detected */  
        break;
        case Servo_Store:     /* store servo teach and go back to Servo_Main */
          servoCtrlState_ = Servo_Main;
        break;
        case Servo_Throttle:  /* servo teach throttle */
          servoThrottle_ = val_;
          if(buttonPressState == Servo_Store)
          {
            eePromStorage_.writeServoThrottle(servoThrottle_);
            servoCtrlState_ = Servo_Store;
          }
        break;
        case Servo_Limit:     /* servo teach end position */
          servoLimit_ = val_;
          if(buttonPressState == Servo_Store)
          {
            eePromStorage_.writeServoLimit(servoLimit_);
            servoCtrlState_ = Servo_Store;
          }
        break;
        case Servo_Revers:    /* teach servo revers */
          servoReverse_ = !servoReverse_;
          eePromStorage_.writeServoRevers(servoReverse_);
          servoCtrlState_ = Servo_Store;
        break;
        case Servo_Calibrate: /* calibrate endpositions of potentiometer */
          servoCtrlState_ = servoCalibrate(buttonPressState);
        break;
        case Servo_EndFlight:
          val_ = servoGyroThrottle_;
          if(servoEndFlightTimer_.IsExpired(1000)) /* 1000 ms throttle time */
          {servoCtrlState_ = Servo_Main;}
        break;
        default:
        break;
      }
      if(servoFunctionTeachActive != ServoFunction::ServoFunction_TeachFunction)
      { /* lock throttle servo move on teach retract servo */
        servoMove(); /* move servo to calculated position */
      }
      functionControl(); /* function control task */
      return(servoCtrlState_);
    }


 /*********************************************************************
 * Method: void StoreGyroFlightPosHorizontal(void)
 *
 * Overview: store horizontal gyro flight pos in eeProm
 *
 ********************************************************************/
    void ServoControl::StoreGyroFlightPosHorizontal (void)
    {
      servoGyroHorizontal_ = val_;    /* servo horizontal position for gyro flight */
      eePromStorage_.writeServoGyroHorizontal(servoGyroHorizontal_);
    }

 /*********************************************************************
 * Method: void StoreGyroFlightPosThrottle(void)
 *
 * Overview: store throttle gyro flight pos in eeProm
 *
 ********************************************************************/
    void ServoControl::StoreGyroFlightThrottle (void)
    {
      servoGyroThrottle_ = val_;    /* servo horizontal position for gyro flight */
      eePromStorage_.writeServoGyroThrottle(servoGyroThrottle_);
    }  

 /*********************************************************************
 * Method: void SignalThrottleEndFlight(void)
 *
 * Overview: signal with throttle end of flight reached
 *
 ********************************************************************/
    void ServoControl::SignalThrottleEndFlight (void)
    {
      if(servoCtrlState_ == Servo_Main)
      {
        servoEndFlightTimer_.Start();
        servoCtrlState_ = Servo_EndFlight;
        val_ = servoGyroThrottle_; /* end of flight reached, move to servoGyroThrottle position */
      }
    }