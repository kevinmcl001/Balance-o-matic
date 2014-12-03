//
// Created for the *Balance-O-Matic* robotics project
// Written by Kevin McLeod and Myles Shepherd, Summer 2014
// I haven't got a clue how all that copyright shit works
//
// KalmanFilter.h
// This is just a header file. See KalmanFilter.cpp for code that actually does something.
//

#include "Arduino.h"

class KalmanFilter
{
  public:
    KalmanFilter();
    float calculate(float thetaMeasured, float gyroMeasured, float dt);
	
  private:
    float theta;			//State angle (x[0])
	float bias;				//State gyroscope bias (x[1])
	
	float thetaDot;			//Angular velocity
	
	float P[2][2];			//Error covariance matrix (uncertainty in state estimate)
	
	float y;				//Innovation (difference in predicted and measured angle)
	
	float S;				//Innovation covariance (uncertainty in state measurement)
	
	float K[2];				//Kalman gain
	
	float qTheta;			//Accelerometer variance (Q[0][0])
	float qBias;			//Gyroscope bias variance (Q[1][1])
	float rTheta;			//Measurement variance
};

//1150x3597