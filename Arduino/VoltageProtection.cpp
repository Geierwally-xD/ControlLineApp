/*
 	 Name:           VoltageProtection.h
 	 Description:    definition of voltage protection object
 	 Copyright:      Geierwally, 2023(c)
 */


#include "VoltageProtection.h"

/*********************************************************************
 * Method: VoltageProtection(int voltCell_1_Pin,int voltCell_2_Pin, LedControl * statusLed)
 *
 * Overview: constructor with parameters
 * @param voltCell_1_Pin analog input of voltage cell 1
 * @param voltCell_2_Pin analog input of voltage cell 2
 * @param statusLed:  reference to the status led object 
 ********************************************************************/
	VoltageProtection::VoltageProtection(int voltCell_1_Pin,int voltCell_2_Pin, LedControl * statusLed)
  {
    voltCell_1_Pin_ = voltCell_1_Pin;
    voltCell_2_Pin_ = voltCell_2_Pin;
    statusLed_ = statusLed;
    voltProtCtrlState_ = VoltProt_Main;
  }

/*********************************************************************
 * Method: void Setup(void)
 *
 * Overview: initialize the voltate protection attributes
 ********************************************************************/
  void VoltageProtection::VoltageProtection::Setup()
  {
    /* start under voltage deley timer */
      voltLimitTimer_.Start();
    /* read calibrated protection level from EEProm */  
      calibValCell_1_ = eePromStorage_.readVoltageProtCell_1();
      calibValCell_2_ = eePromStorage_.readVoltageProtCell_2();
  } 

/*********************************************************************
 * Method: void Control(uint8_t buttonPressState)
 *
 * Overview: the voltage protection control task
 * - read voltage of battery cells over analog input pins 
 * - compare voltage level with minimum limit
 * - calibrate voltage inputs
 * @param buttonPressState state of button control
 ********************************************************************/
  void VoltageProtection::Control(uint8_t buttonPressState)
  {
    int volt_cell_1 = analogRead(voltCell_1_Pin_);      /* reads voltage cell 1 (value between 0 and 1023) */
    int volt_cell_2 = analogRead(voltCell_2_Pin_);      /* reads voltage cell 2 (value between 0 and 1023) */
    if(buttonPressState == VoltProt_Calibrate)
    {
      voltProtCtrlState_ = VoltProt_Calibrate;
    }
    switch(voltProtCtrlState_)
    {
      case VoltProt_Main:       /* main task voltage protection */
        volt_cell_1 = map(volt_cell_1, 0, calibValCell_1_, 0, 4200);
        volt_cell_2 = map(volt_cell_2, 0, calibValCell_2_, 0, 8400);
        volt_cell_2 -= volt_cell_1;
  #ifdef TRACE_VOLTAGE_PROTECTION
        Serial.println("voltage cell 1 = " + String(volt_cell_1));
        Serial.println("voltage cell 2 = " + String(volt_cell_2));
  #endif
        if(  (volt_cell_1 < UnderVoltageLimit)
           ||(volt_cell_2 < UnderVoltageLimit)
          )
        {
          if(voltLimitTimer_.IsExpired(UnderVoltageDelay))
          {
            voltProtCtrlState_ = VoltProt_UnderVoltage; /* over 3000 ms under voltage => failure */
            voltLimitTimer_.Start();
          }
        }
        else
        {
          voltLimitTimer_.Start(); /* no under voltage, restart limit timer */
        }
      break;
      case VoltProt_Store:       /* store voltage protection on EEProm */
        eePromStorage_.writeVoltageProtCell_1(calibValCell_1_);
        eePromStorage_.writeVoltageProtCell_2(calibValCell_2_);
        voltProtCtrlState_ = VoltProt_Main;
      break;
      case VoltProt_Calibrate:   /* calibrate voltage protection level  */
        calibValCell_1_ = volt_cell_1;
        calibValCell_2_ = volt_cell_2;
        voltProtCtrlState_ = VoltProt_Store;
      break;
      case VoltProt_UnderVoltage:/* under voltage failure */
        if(voltLimitTimer_.IsExpired(1600)) /* flash LED under voltage failure */
        {
          voltLimitTimer_.Start();
          statusLed_->FlashLed(3,100,100);
        }
      break;
      default:
      break;
    }
  }
  
