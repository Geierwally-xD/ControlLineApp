/*
 	 Name:           Kalman.cpp
 	 Description:    definition of Kalman filter object
   based on http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it
 	 Copyright:      Geierwally, 2023(c)
 */
#include "Kalman.h"
/*********************************************************************
 * Method: Kalman(void)
 *
 * Overview: constructor
 * 
 ********************************************************************/
	Kalman::Kalman(void)
  {
	  kFangle_    = 0.0f;
	  q_angle_    = 0.001f;
	  q_bias_     = 0.003f;
	  r_measure_  = 0.03f;
	  bias_       = 0.0f;    /* Reset bias */
	  p_[0][0]    = 0.0f;    /* Since we assume that the bias is 0 and we know the starting angle, the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical */
	  p_[0][1]    = 0.0f;
	  p_[1][0]    = 0.0f;
	  p_[1][1]    = 0.0f;
  }


/*********************************************************************
 * Method: setup(float angleDeg)
 *
 * Overview: method initialize software Kalman- Filter
 * @param angleDeg: initializing angle for Kalman Filter
 * 
 ********************************************************************/
  void Kalman::setup(float angleDeg)
  {
    kFangle_ = angleDeg;
	  q_angle_    = 0.001f;
	  q_bias_     = 0.003f;
	  r_measure_  = 0.03f;
	  bias_       = 0.0f;    /* Reset bias */
	  p_[0][0]    = 0.0f;    /* Since we assume that the bias is 0 and we know the starting angle, the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical */
	  p_[0][1]    = 0.0f;
	  p_[1][0]    = 0.0f;
	  p_[1][1]    = 0.0f;
  }

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
	float Kalman::KalmanFilter(float angleDeg, float gyroDat, float elapsedTime)
  {
     /* Step 1 */
    rate_ = gyroDat - bias_;
    kFangle_ += elapsedTime * rate_;
    /* Update estimation error covariance - Project the error covariance ahead */
    /* Step 2 */
    p_[0][0] += elapsedTime * (elapsedTime*p_[1][1] - p_[0][1] - p_[1][0] + q_angle_);
    p_[0][1] -= elapsedTime * p_[1][1];
    p_[1][0] -= elapsedTime * p_[1][1];
    p_[1][1] += q_bias_ * elapsedTime;

    /* Discrete Kalman filter measurement update equations - Measurement Update ("Correct") */
    /* Calculate Kalman gain - Compute the Kalman gain */
    /* Step 4 */
    float S = p_[0][0] + r_measure_; /* Estimate error */
    /* Step 5 */
    float K[2]; /* Kalman gain - This is a 2x1 vector */
    K[0] = p_[0][0] / S;
    K[1] = p_[1][0] / S;
    /* Calculate angle and bias - Update estimate with measurement zk (newAngle) */
    /* Step 3 */
    float y = angleDeg - kFangle_; /* Angle difference */
    /* Step 6 */
    kFangle_ += K[0] * y;
    bias_ += K[1] * y;
    /* Calculate estimation error covariance - Update the error covariance */
    /* Step 7 */
    float P00_temp = p_[0][0];
    float P01_temp = p_[0][1];
    p_[0][0] -= K[0] * P00_temp;
    p_[0][1] -= K[0] * P01_temp;
    p_[1][0] -= K[1] * P00_temp;
    p_[1][1] -= K[1] * P01_temp;   
    return(kFangle_ / RAD_TO_DEG);
  }