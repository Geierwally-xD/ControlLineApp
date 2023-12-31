/*
 	 Name:           Kalman.h
 	 Description:    declaration of Kalman filter object
   based on http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it
 	 Copyright:      Geierwally, 2023(c)
 */
#ifndef KALMAN_H
#define KALMAN_H

#include "Arduino.h"
#include "TimerMs.h"

//#define TRACE_GYRO_ACCEL   /* console print kalman filter process */
class Kalman
{
  public:
/*********************************************************************
 * Method: Kalman(void)
 *
 * Overview: constructor
 * 
 ********************************************************************/
	Kalman(void);

/*********************************************************************
 * Method: setup(float angleDeg)
 *
 * Overview: method initialize software Kalman- Filter
 * @param angleDeg: initializing angle for Kalman Filter
 * 
 ********************************************************************/
  void setup(float angleDeg);

/*********************************************************************
 * Method: KalmanFilter(float angleDeg, float gyroDat, float elapsedTime)
 *
 * Overview: method realizes a software Kalman- Filter
 * @param angleDeg: calculated angle of accelerometer
 * @param gyroDat:  rawdata of gyroscope
 * @param elapsedTime: cycle time of last gyroscope measurement
 * @return calculated angle result of KalmanFilter
 * 
 ********************************************************************/
	float KalmanFilter(float angleDeg, float gyroDat, float elapsedTime);

  private:
  	  /* Kalman filter variables */
	    float q_angle_;  /* Process noise variance for the accelerometer */
	    float q_bias_;   /* Process noise variance for the gyro bias */
	    float r_measure_;/* Measurement noise variance - this is actually the variance of the measurement noise */
	    float bias_;     /* The gyro bias calculated by the Kalman filter - part of the 2x1 state vector */
	    float rate_;     /* Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate */
	    float p_[2][2];  /* Error covariance matrix - This is a 2x2 matrix */
		  float kFangle_;  /* The angle calculated by the Kalman filter - part of the 2x1 state vector */
};

#endif