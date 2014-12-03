//
// Created for the *Balance-O-Matic* robotics project
// Written by Kevin McLeod and Myles Shepherd, Summer 2014
// I haven't got a clue how all that copyright shit works
//
// KalmanFilter.cpp
// Kalman filter for smoothing and blending accelerometer/gyroscope data until deliciously creamy.
//
// Written using tutorial and examples from Kristian Lauszus and TKJ Electronics:
// http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
//

#include "Arduino.h"
#include "KalmanFilter.h"

//Kalman filter default constructor
KalmanFilter::KalmanFilter()
{
	//Obtained from sources as listed above
  	qTheta = 0.001;
	qBias = 0.003;
	rTheta = 0.03;
	
	//Approximate value (Just setting it to 0 worked too, so an inaccurate guess can't be worse, right?)
	bias = 3.0;
	
	//Starting angle is zero degrees
	theta = 0;
	
	//Pretend we're perfectly accurate (why not?)
	P[0][0] = 0;
	P[0][1] = 0;
	P[1][0] = 0;
	P[1][1] = 0;
}

//Kalman filter calculation
//Units: thetaMeasured (degrees), gyroMeasured (degrees/second), dt (seconds)
float KalmanFilter::calculate(float thetaMeasured, float gyroMeasured, float dt)
{
	//Calculate true angular velocity
    thetaDot = gyroMeasured - bias;
	
	//Predict current state (angle)
	theta = theta + thetaDot*dt;
	
	//Calculate error covariance matrix (uncertainty in state estimate)
	//P[0][0] += - dt * (P[1][0] + P[0][1]) + qTheta * dt;
	P[0][0] = P[0][0] + (P[1][1]*dt-P[0][1]-P[1][0]+qTheta)*dt;
	P[0][1] = P[0][1] - P[1][1]*dt;
	P[1][0] = P[1][0] - P[1][1]*dt;
	P[1][1] = P[1][1] + qBias*dt;
    
	//Calculate innovation (difference in predicted and measured angle)
	y = thetaMeasured - theta;

	//Calculate innovation covariance (uncertainty in observation)
	S = P[0][0] + rTheta;
	
	//Calculate Kalman gain
	K[0] = P[0][0]/S;
	K[1] = P[1][0]/S;
    
	//Update estimates of current state
	theta = theta + K[0]*y;
	bias = bias + K[1]*y;
	
	//Update error covariance matrix
	P[0][0] = P[0][0] - P[0][0]*K[0];
	P[0][1] = P[0][1] - P[0][1]*K[0];
	P[1][0] = P[1][0] - P[0][0]*K[1];
	P[1][1] = P[1][1] - P[0][1]*K[1];
    
    return theta;
}

//1150x3597