//
// Created for the *Balance-O-Matic* robotics project
// Written by Kevin McLeod and Myles Shepherd, Summer 2014
// I haven't got a clue how all that copyright shit works
//
// PIDController.cpp
// For all your control system needs!
//
// *Integral output term constrained to |255|
//

#include "Arduino.h"
#include "PIDController.h"

//PID controller default constructor
//Just set everything to zero. It'll be fine.
PIDController::PIDController()
{
    setPoint = 0.0;
	
	error = 0.0;
	prevError = 0.0;
	sumError = 0.0;
	deltaError = 0.0;
	
	kP = 0.0;
	kI = 0.0;
	kD = 0.0;

	sumErrorMax = 0.0;
}

//Do literally everything
float PIDController::calculate(float newAngle)
{
	//Calculate errors
	error = newAngle - setPoint;
	sumError += error;
	sumError = constrain(sumError,-sumErrorMax,sumErrorMax);	//Put a cap on the integral term
	deltaError = error - prevError;
	
	//Calculate PID output
	//Currently unbounded
	float output = error*kP + sumError*kI + deltaError*kD;
	
	//Save current error
	prevError = error;
	
	return output;
}

//Set all error terms to zero
void PIDController::resetErrors()
{
	error = 0;
	prevError = 0;
	sumError = 0;
	deltaError = 0;
}

//Set proportional gain
void PIDController::setPGain(float gain)
{
	kP = gain;
}

//Set integral gain
void PIDController::setIGain(float gain)
{
	kI = gain;
}

//Set derivative gain
void PIDController::setDGain(float gain)
{
	kD = gain;

}

//Set the maximum allowable sumError
void PIDController::setSumErrorMax(float max)
{
	sumErrorMax = max;
}

//Change setpoint
void PIDController::setSetSetPoint(float newSetPoint)
{
	setPoint = newSetPoint;
}

//Output all gain values [kP,kI,kD]
void PIDController::getGains(float* gains)
{
	gains[0] = kP;
	gains[1] = kI;
	gains[2] = kD;
}

//Output all PID errors [error,sumError,deltaError]
void PIDController::getErrors(float* errors)
{
	errors[0] = error;
	errors[1] = sumError;
	errors[2] = deltaError;
}

//1150x3597