/*
 	 Name:           ServoFunction.h
 	 Description:    definition of function servo motors object
 	 Copyright:      Geierwally, 2025(c)
 */

#include "ServoFunction.h"
#include "ButtonControl.h"


/*********************************************************************
 * Method: ServoFunction(int servoPin, LedControl * statusLed)
 *
 * Overview:  constructor with parameters:
 * @param servoPin:  the digital output pwm pin for servo function
 * @param statusLed:  reference to the status led object
 * @param throttleServo_: reference to the throttle servo object
 ********************************************************************/
    ServoFunction::ServoFunction(int servoPin, LedControl * statusLed, ServoControl *  throttleServo)
    {
        servoPin_ = servoPin;
        statusLed_ = statusLed;
        servoPosition_ = 0;
        servoEndpos_ = 0;
        servoDelay_ = 0;
        servoRevers_ = false;
        servoFunctionActive_ = 0; /* servo function off 0; servo function on 1; servo function inverse 3 */
        throttleServo_ = throttleServo;
        servoFunctionCtrlState_ = ServoFunction_Main;
        expectedFunctionServoTeachState_ = sft_Idle;
        functionServoTeachState_ = sft_Idle;
        prevServoFunctionState_ = 0;
    }
    
/*********************************************************************
 * Method: void Setup(void)
 *
 * Overview: initialize the servo function attributes
 ********************************************************************/	
    void ServoFunction::Setup() 
    {
        servo_.attach(servoPin_);      /* attaches the servo on pin to the servo object */
        servoTimer_.Start();
        teachLEDTimer_.Start();
        servoEndpos_ = eepromStore_.readServoFunction();
        servoRevers_ = (bool) eepromStore_.readServoFunctionReverse();
        servoFunctionActive_ = eepromStore_.readServoFunctionActive();
        servoDelay_ = eepromStore_.readServoFunctionSpeed();
        if(servoRevers_ == true)
            {servoPosition_ = 180;}
        else
            {servoPosition_ = 25;}
        servo_.write(servoPosition_);
        prevServoFunctionState_ = statusLed_->GetFunction2State();
        switch (servoFunctionActive_)
        {
            default:
            case 0:
            /* nothing to do */
            break;
            case 1: /* normal: switch servo function 3 off */
            statusLed_->SetFunction3PinState(0);
            break;
            case 3: /* inverse switch servo function 3 on */
            statusLed_->SetFunction3PinState(1);
            break;
        }
    }
   
 /*********************************************************************
 * Method: void Control(void)
 *
 * Overview: the function servo control task
 * - move servo to end- or limit - position depending
 * - teach function servo end position
 * - calibrate potentiometer
 * @return: 1 if servo teach function is active otherwise 0
 ********************************************************************/
    uint8_t ServoFunction::Control(uint8_t buttonPressState) 
    {
        uint8_t servoFunctionState2 = 0;
        bool moveDone = false;
        switch(servoFunctionCtrlState_)
        {
            case ServoFunction_Main:      /* main task servo control */
                if(buttonPressState == ServoFunction_TeachFunction)
                {
                    servoFunctionCtrlState_ = buttonPressState;
                }
                servoFunctionState2 = statusLed_->GetFunction2State();
                if(servoFunctionState2 != prevServoFunctionState_)
                {
                    if(servoFunctionState2 == 1)
                    {
                        servoTimer_.Start(); 
                        servoFunctionCtrlState_ = ServoFunction_EndposMove;
                    }
                    else
                    {
                        servoTimer_.Start(); 
                        servoFunctionCtrlState_ = ServoFunction_LimitMove;
                    }
                    prevServoFunctionState_ = servoFunctionState2;
                }
            break;
            case ServoFunction_EndposMove: /* move servo to treached end position */  
                moveDone = servoMove(servoEndpos_);
                if(moveDone == true)
                {
                    servoFunctionCtrlState_ = ServoFunction_Main;
                    switch (servoFunctionActive_)
                    {
                        default:
                        case 0:
                        /* nothing to do */
                        break;
                        case 1: /* normal: switch servo function 3 on */
                        statusLed_->SetFunction3PinState(1);
                        break;
                        case 2: /* inverse switch servo function 3 off */
                        statusLed_->SetFunction3PinState(0);
                        break;
                    }
                }
            break; 

            case ServoFunction_LimitMove:  /* move servo to limit position */
                if(servoRevers_ == true)
                {moveDone = servoMove(180);}
                else
                {moveDone = servoMove(25);}
                if(moveDone == true)
                {
                    servoFunctionCtrlState_ = ServoFunction_Main;
                    switch (servoFunctionActive_)
                    {
                        default:
                        case 0:
                        /* nothing to do */
                        break;
                        case 1: /* normal: switch servo function 3 off */
                        statusLed_->SetFunction3PinState(0);
                        break;
                        case 2: /* inverse switch servo function 3 on */
                        statusLed_->SetFunction3PinState(1);
                        break;
                    }
                }
            break;

            case ServoFunction_TeachFunction:   /* servo teach function position */
                if(true == servoFuctionTeach(buttonPressState))
                {
                    servoFunctionCtrlState_ = ServoFunction_Main;
                }
            break;
            default:
            break;        
        }
        return(servoFunctionCtrlState_);
    }
    
/*********************************************************************
 * Method: uint8_t servoFuctionTeach(uint8_t buttonPressState)
 *
 * Overview: servo function teach task
 * teach function servo with following task substates
 * -function servo teach idle (wait for teach button press)
 * -function servo teach end position 1
 * -function servo teach revers       2
 * -function servo teach servo speed  3
 * -function servo teach leave        4
 * @param: buttonPressState  the button press state of button control
 * 
 ********************************************************************/	
    bool ServoFunction:: servoFuctionTeach(uint8_t buttonPressState)
    {
        bool teachDone = false;
        int PotiValues[3] = {0,0,0}; /* array for poti values */
        switch(functionServoTeachState_)
        {   
            case sft_Idle:                /* wait for teach menu call */
                /* flash LED for each teach state wait until button pressed */
                if(teachLEDTimer_.IsExpired(teachLEDTimeout_[expectedFunctionServoTeachState_]))
                {
                    if(expectedFunctionServoTeachState_ < sft_Leave)
                    {
                        expectedFunctionServoTeachState_ ++;
                        statusLed_->FlashLed(expectedFunctionServoTeachState_,100,200);
                        teachLEDTimer_.Start();
                    }
                    else
                    {
                        expectedFunctionServoTeachState_ = sft_Idle;
                        teachLEDTimer_.Start();
                    } 
                }
                if(buttonPressState == ButtonControl::_1sPressed)
                { /* switch to teach state if button was pressed */
                    functionServoTeachState_ = expectedFunctionServoTeachState_;
                    servoFunctionTeachIndex_ = 0;
                }
            break;
            case sft_Endpos:
                throttleServo_->getPoti(PotiValues); /* get poti values */
                if(false == servoRevers_)
                {servoPosition_ = map(PotiValues[PotiCurVal], PotiValues[PotiMinVal], PotiValues[PotiMaxVal], 25, 180);}   /* scale it for use with the servo (value between 0 and 180) */ 
                else
                {servoPosition_ = map(PotiValues[PotiCurVal], PotiValues[PotiMinVal], PotiValues[PotiMaxVal], 180, 25);}   /* scale it for use with the servo (value between 0 and 180) */ 
                servo_.write(servoPosition_);
                if(buttonPressState == ButtonControl::_1sPressed)
                {
                    servoEndpos_ = servoPosition_;
                    eepromStore_.writeServoFunction(servoEndpos_);
                    prevServoFunctionState_ = 1; /* set previous function state to 1 (end pos move)*/
                    functionServoTeachState_ = sft_Idle;
                }
            break;
            case sft_Revers:
                if(teachLEDTimer_.IsExpired(teachLEDTimeout_[servoFunctionTeachIndex_]))
                {
                    if(servoFunctionTeachIndex_ < 2)
                    {
                        servoFunctionTeachIndex_ ++;
                    }
                    else
                    {
                        servoFunctionTeachIndex_ = 1;
                    }
                    statusLed_->FlashLed(servoFunctionTeachIndex_,100,200);
                    teachLEDTimer_.Start();
                }
                if(buttonPressState == ButtonControl::_1sPressed)
                { /* write servo function reverse to eeProm if button was pressed 0 = normal 1 = reverse*/
                    servoRevers_ = (bool)(servoFunctionTeachIndex_ - 1);
                    eepromStore_.writeServoFunctionReverse((uint8_t)servoRevers_);
                    functionServoTeachState_ = sft_Idle;   
                }  
            break;
            case sft_Speed:
            if(teachLEDTimer_.IsExpired(teachLEDTimeout_[servoFunctionTeachIndex_]))
            {
                if(servoFunctionTeachIndex_ < 6)
                {
                    servoFunctionTeachIndex_ ++;                   
                }
                else
                {
                    servoFunctionTeachIndex_ = 1;                    
                }

                if(servoFunctionTeachIndex_ == 6)
                {statusLed_->FlashLed(servoFunctionTeachIndex_,200,200);}
                else
                {statusLed_->FlashLed(servoFunctionTeachIndex_,100,200);}
                
                teachLEDTimer_.Start();
            }
            if(buttonPressState == ButtonControl::_1sPressed)
            { /* write servo special function state to eeProm if button was pressed 0 = off 1 = active 2 = inverse active */
                if(servoFunctionTeachIndex_ == 6)
                {servoFunctionTeachIndex_ = 0;}
                servoDelay_ = servoFunctionTeachIndex_ * 10;
                eepromStore_.writeServoFunctionSpeed(servoDelay_);
                functionServoTeachState_ = sft_Idle;   
            }  
            break;
            case sft_DigitalSpecial:
                if(teachLEDTimer_.IsExpired(teachLEDTimeout_[servoFunctionTeachIndex_]))
                {
                    if(servoFunctionTeachIndex_ < 3)
                    {
                        servoFunctionTeachIndex_ ++;
                    }
                    else
                    {
                        servoFunctionTeachIndex_ = 1;
                    }
                    if(servoFunctionTeachIndex_ == 3)
                    {statusLed_->FlashLed(servoFunctionTeachIndex_,200,200);}
                    else
                    {statusLed_->FlashLed(servoFunctionTeachIndex_,100,200);}
                    teachLEDTimer_.Start();
                }
                if(buttonPressState == ButtonControl::_1sPressed)
                { /* write servo special function state to eeProm if button was pressed 0 = off 1 = active 2 = inverse active */
                    if(servoFunctionTeachIndex_ == 3)
                    {servoFunctionTeachIndex_ = 0;}
                    servoFunctionActive_ = servoFunctionTeachIndex_;
                    eepromStore_.writeServoFunctionActive(servoFunctionActive_);
                    functionServoTeachState_ = sft_Idle;   
                }  
            break;
            case sft_Leave:
                functionServoTeachState_ = sft_Idle;
                teachDone = true;
            break;
            default:
            break;        
        }
        return(teachDone);
    }

 /*********************************************************************
 * Method: bool servoMove(int val)
 *
 * Overview: moves function servo to position
 * @param: move position
 * @return: true if move done
 ********************************************************************/
    bool ServoFunction::servoMove(int val)
    {
        bool moveDone = false;
        if(servoTimer_.IsExpired(servoDelay_)) /* waits for the servo to get there */
        {
            if(servoPosition_ < val)
            {servoPosition_ ++;}
            else if(servoPosition_ > val)
            {servoPosition_ --;}
            else
            {moveDone = true;}

        #ifdef TRACE_SERVO_CONTROL
            Serial.println(servoPosition_);   /* servo move position */
        #endif      
            servo_.write(servoPosition_);     /* sets the servo position according to the scaled value */
            servoTimer_.Start();                           
        }
        return(moveDone);
    }   