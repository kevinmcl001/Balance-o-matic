//
// Created for the *Balance-O-Matic* robotics project
// Written by Kevin McLeod and Myles Shepherd, Summer 2014
// I haven't got a clue how all that copyright shit works
//
// PIDController.h
// This is just a header file. See PIDController.cpp for code that actually does something.
//

#include "Arduino.h"

class PIDController
{
  public:
    PIDController();
	float calculate(float newAngle);
	void resetErrors();
	void setPGain(float gain);
	void setIGain(float gain);
	void setDGain(float gain);
	void setSumErrorMax(float max);
	void setSetSetPoint(float newSetPoint);
	void getGains(float* gains);
	void getErrors(float* errors);
	
  private:
    float setPoint;		//Controller setpoint
	
	float error;		//Current error
	float prevError;	//Previous loop error
	float sumError;		//Sum of error
	float deltaError;	//Change in error
	
	float kP;			//Proportional gain
	float kI;			//Integral gain
	float kD;			//Derivative gain
	
	float sumErrorMax;	//Put a lid on the integral contribution
};

//1150x3597