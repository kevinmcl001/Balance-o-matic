//
// Created for the *Balance-O-Matic* robotics project
// Written by Kevin McLeod and Myles Shepherd, Summer 2014
// I haven't got a clue how all that copyright shit works
//
// Motor.h
// This is just a header file. See Motor.cpp for code that actually does something.
//

#include "Arduino.h"



class Motor
{
  public:
    Motor(int n);
	static void initializeMotorController();
	void setVelocity(int vel);
	int	getEncoderCount();
	float getSpeed(float dt);
	
	volatile int encoderCount;	//Motor encoder count (public so the interrupts can see it)
	
  private:	
	void setDirection(int value);
	void setSpeed(int speed);
	
    int pwmPin;					//Pin number for PWM signal
	int pinA;					//Pin number for inputA
	int pinB;					//Pin number for inputB
	float kM;					//Motor gain (to adjust for imbalance)
	int prevEncoderCount;		//For determining motor speed
};

//1150x3597