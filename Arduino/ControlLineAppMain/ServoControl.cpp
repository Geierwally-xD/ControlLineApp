/*
 	 Name:           ServoControl.cpp
 	 Description:    definition of controlling servo motors object
 	 Copyright:      Geierwally, 2023(c)
 */
#include "ServoControl.h"

/*********************************************************************
 * Method: ServoControl(int controlPin, int servoPin, uint32_t moveDelay, LedControl * statusLed)
 *
 * Overview: constructor with parameters:
 * controlPin: the analog input pin for input encoder
 * servoPin:   the digital output pwm pin for servo control
 * moveDelay:  the move delay in ms for servo to reach position
 * statusLed: reference to the status led object
 * gyroAccelSens: reference to gyro accelerator sensor object
 ********************************************************************/
    ServoControl::ServoControl(int controlPin, int servoPin, uint32_t moveDelay, LedControl * statusLed, GyroAccelSens* gyroAccelSens)
    {
      controlPin_ = controlPin;
      servoPin_   = servoPin;
      servoDelay_ = moveDelay;
      statusLed_ = statusLed;
      gyroAccelSens_ = gyroAccelSens;
      servoCtrlState_ = Servo_Main;
      servoReverse_ = false;         /* if true servo reverse, otherwise normal */
      servoLineShortThrottle_ = true; /* throttle on control line short active as default */
      servoLimit_ = 10;              /* maximum throttle position */
      servoThrottle_ = 20;          /* servo throttle position */
      servoGyroHorizontal_ = 10;    /* servo horizontal position for gyro flight */
      servoGyroThrottle_ = 20;      /* servo throttle position for gyro flight */
      moveVal_ = 90;                /* servo move position */
      servoCalibMinVal_ = 538;      /* servo potentiometer minimum ADC calib value */
      servoCalibMaxVal_ = 1023;      /* servo potentiometer maximum ADC calib value */
      calibrationState_ = calibrateMinVal; /* set start value for calibration task */
    }

/*********************************************************************
 * Method: void Setup(void)
 *
 * Overview: initialize the servo control attributes
 ********************************************************************/	
    void ServoControl::Setup() 
    {
      servo_.attach(servoPin_);    /* attaches the servo on pin 9 to the servo object */
      servoTimer_.Start();
      servoTeachTimer_.Start();
      servoEndFlightTimer_.Start();
      servoCtrlState_ = Servo_Main;
      moveVal_ = 90;                /* servo move position */
      servoCalibMinVal_ = eePromStorage_.readCalMinVal();
      servoCalibMaxVal_ = eePromStorage_.readCalMaxVal();
      servoThrottle_    = eePromStorage_.readServoThrottle();
      servoGyroHorizontal_ = eePromStorage_.readGyroHorizontal();
      servoGyroThrottle_ = eePromStorage_.readGyroThrottle();
      servoLimit_       = eePromStorage_.readServoLimit();
      servoLineShortThrottle_ = eePromStorage_.readServoLineShortThrottle();
      servoReverse_     = eePromStorage_.readServoRevers();
    }

/*********************************************************************
 * Method: void servoLineshortTeach(void)
 *
 * Overview: toggle between full throttle or no throttle at control -
 * line short
 ********************************************************************/	
    void ServoControl::servoLineshortTeach(void)
    {
      if(servoTeachTimer_.IsExpired(1300))
      {
        servoTeachTimer_.Start();
        servoLineShortThrottle_ = !servoLineShortThrottle_;
        if(servoLineShortThrottle_)
        { statusLed_->FlashLed(1,200,100);}
        else
        { statusLed_->FlashLed(2,200,100);}
      }
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
 * Method: void Control(void)
 *
 * Overview: the servo control task
 * - move servo to position of input encoder
 * - teach servo positions
 * - calibrate potentiometer
 * @return: the control state of task
 ********************************************************************/
    uint8_t ServoControl::Control(uint8_t buttonPressState) 
    { 
      static bool gyroFlightToggled = false; /* avoid multible toggle of flight control active */
      uint8_t gyroAssembly = 0;
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
          if(false == servoReverse_)
          { /* no servo reverse */
            if((val_ > 178)&&(servoLineShortThrottle_)&&(!gyroFlightActive_))
            {
              val_ = servoLimit_;
            } /* short circuit on control lines and no line short throttle set*/
            else if(val_ < servoLimit_)
            {
              val_ = servoLimit_;
            }
            else if(val_ < servoThrottle_)
            {
              val_ = servoThrottle_;
              if(gyroAccelSens_->getGyroFlightActive())
              { 
                if(!gyroFlightToggled)
                {
                  gyroFlightActive_ = !gyroFlightActive_;
                  gyroFlightToggled = true;
                  servoTeachTimer_.Start();
                  if(gyroFlightActive_)
                    { statusLed_->FlashLed(2,200,100);} /* gyro flight is enabled flash led two times */
                  else
                    { statusLed_->FlashLed(1,200,100);} /* gyro flight is dissabled flash led one time */ 
                }
              }
              else
              {
                gyroFlightActive_ = false;
              }
            }
            else if(val_ > servoGyroHorizontal_) /* flight angle control active and position > servo gyro horizontal */
            { 
              gyroFlightToggled = false;
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
                { /* assembly gyro invers */
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
          else
          { /* servo reverse */
            if((val_ < 26)&&(servoLineShortThrottle_)&&(!gyroFlightActive_))
            {
              val_ = servoLimit_;
            } /* short circuit on control lines and no line short throttle set*/           
            else if(val_ > servoLimit_)
            {
              val_ = servoLimit_;
            }
            else if(val_ > servoThrottle_)
            {
              val_ = servoThrottle_;
              if(gyroAccelSens_->getGyroFlightActive())
              { 
                if(!gyroFlightToggled)
                {
                  gyroFlightActive_ = !gyroFlightActive_;
                  gyroFlightToggled = true;
                  servoTeachTimer_.Start();
                  if(gyroFlightActive_)
                    { statusLed_->FlashLed(2,200,100);} /* gyro flight is enabled flash led two times */
                  else
                    { statusLed_->FlashLed(1,200,100);} /* gyro flight is dissabled flash led one time */             
                }
              }
              else
              {
                gyroFlightActive_ = false;
              }
            }
            else if(val_ < servoGyroHorizontal_) /* flight angle control active and position < servo gyro horizontal */
            {
              gyroFlightToggled = false;
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
          } /* servo reverse */
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
        case Servo_LineShort: /*  */
          if(buttonPressState == Servo_Store)
          {
            servoCtrlState_ = buttonPressState;
            eePromStorage_.writeServoLineShortThrottle(servoLineShortThrottle_);
          }
          else
          {
            servoLineshortTeach();
          }
        break;
        case Servo_Calibrate:
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
      
      if(servoTimer_.IsExpired(servoDelay_)) /* waits for the servo to get there */
      {
        if(moveVal_ < val_)
          {moveVal_ ++;}
        else if(moveVal_ > val_)
          {moveVal_ --;}  
  #ifdef TRACE_SERVO_CONTROL
        Serial.println(moveVal_);            /* servo move position */
  #endif      
        servo_.write(moveVal_);                  /* sets the servo position according to the scaled value */
        servoTimer_.Start();                           
      }
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