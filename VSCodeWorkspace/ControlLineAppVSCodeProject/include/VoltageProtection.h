/*
 	 Name:           VoltageProtection.h
 	 Description:    declaration of voltage protection object
 	 Copyright:      Geierwally, 2023(c)
 */

#ifndef VOLTAGE_PROTECTION_H
#define VOLTAGE_PROTECTION_H

#include "Arduino.h"
#include "TimerMs.h"
#include "EEPromStorage.h" 
#include "LedControl.h"

//#define TRACE_VOLTAGE_PROTECTION   /* console print of voltage protection traces */
#define LOCK_UNDER_VOLTAGE_OUTPUT    /* use this , if power supply comes over USB */

#define UnderVoltageDelay 3000       /* 3000ms delay for under voltage failure */
#define UnderVoltageLimit 3500       /* 3500mV cell voltage is under voltage limit */
class VoltageProtection
{
  public:
    enum VoltProtTaskState
    {
             VoltProt_Main,      /* main task voltage protection */
     VoltProt_UnderVoltage,
            VoltProt_Store,      /* store voltage protection on EEProm */
        VoltProt_Calibrate = 7   /* calibrate voltage protection level  */
    } voltProtTaskState;         /* states of voltage protection task state machine */

/*********************************************************************
 * Method: VoltageProtection(int voltCell_1_Pin,int voltCell_2_Pin, LedControl * statusLed)
 *
 * Overview: constructor with parameters
 * @param voltCell_1_Pin analog input of voltage cell 1
 * @param voltCell_2_Pin analog input of voltage cell 2
 * @param statusLed:  reference to the status led object 
 ********************************************************************/
	VoltageProtection(int voltCell_1_Pin,int voltCell_2_Pin, LedControl * statusLed);

/*********************************************************************
 * Method: void Setup(void)
 *
 * Overview: initialize the voltage protection attributes
 ********************************************************************/
  void Setup(); 

/*********************************************************************
 * Method: void Control(uint8_t buttonPressState)
 *
 * Overview: the voltage protection control task
 * - read voltage of battery cells over analog input pins 
 * - compare voltage level with minimum limit
 * - calibrate voltage inputs
 * @param buttonPressState state of button control
 ********************************************************************/
  void Control(uint8_t buttonPressState);

	private:

    int voltCell_1_Pin_;         /* attribute for analog pin for voltage input battery cell 1 */
    int voltCell_2_Pin_;         /* attribute for analog pin for voltage input battery cell 2 */
    LedControl * statusLed_;     /* reference to status LED object */ 
    int calibValCell_1_;         /* calibration value for full cell 1 4200 mV */
    int calibValCell_2_;         /* calibration value for full cell 2 4200 mV */
    TimerMs voltLimitTimer_;     /* delay timer for under voltage failure */
    uint8_t voltProtCtrlState_;  /* state of voltage protection  control task */
    EEPromStorage  eePromStorage_; /* instance of eeProm storage object */
};

#endif