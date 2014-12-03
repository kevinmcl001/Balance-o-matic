//
// Created for the *Balance-O-Matic* robotics project
// Written by Kevin McLeod and Myles Shepherd, Summer 2014
// I haven't got a clue how all that copyright shit works
//
// Motor.cpp
// For communication with DC motors
//
// Written for Pololu 30:1 Metal Gearmotors (http://www.pololu.com/product/1443/)
// Using a Dual VNH2SP30 Motor Driver Carrier MD03A (http://www.pololu.com/product/708/)
// Pin numbers correspond to Arduino Uno (See MainProgram for connection diagram)
//

#include "Arduino.h"
#include "Motor.h"

// --- PIN DEFINITIONS ---
#define EN_A_R		2	//Encoder A, RIGHT		*Do not change: Interrupt attached (MainProgram::setup)
#define EN_A_L		3	//Encoder A, LEFT		*Do not change: Interrupt attached (MainProgram::setup)
#define	EN_B_R		4	//Encoder B, RIGHT
#define EN_B_L		5	//Encoder B, LEFT
#define EN_GND_R	A2	//Encoder Ground, RIGHT
#define EN_GND_L	A0	//Encoder Ground, LEFT
#define EN_VCC_R	A3	//Encoder Vcc (+5), RIGHT
#define EN_VCC_L	A1	//Encoder Vcc (+5), LEFT

#define IN_A_R		6	//Input A, Motor controller, RIGHT
#define IN_A_L		7	//Input A, Motor controller, LEFT
#define IN_B_R		8	//Input B, Motor controller, RIGHT
#define	IN_B_L		11	//Input B, Motor controller, LEFT
#define PWM_R		9	//PWM signal, RIGHT motor	*Do not change: PWM frequency modified (initializeMotorController)
#define PWM_L		10	//PWM signal, LEFT motor	*Do not change: PWM frequency changed (initializeMotorController)

//Pre-set Pin		GND	//Motor controller, Ground
#define MC_VCC		12	//Motor controller, Vcc (+5)



//Motor set-up and initialization
//LEFT motor is 0
//RIGHT motor is 1
Motor::Motor(int n)
{
	//Enable LEFT motor
	if(n == 0)
	{
		//Direction and PWM
		pinMode(IN_A_L, OUTPUT);
		pinMode(IN_B_L, OUTPUT);
		pinMode(PWM_L, OUTPUT);
		
		//Encoder input
		pinMode(EN_B_L, INPUT);
		pinMode(EN_A_L, INPUT);
		
		//Encoder power
		pinMode(EN_GND_L, OUTPUT);
		pinMode(EN_VCC_L, OUTPUT);
		
		//Enable power and ground
		digitalWrite(EN_GND_L,0);
		digitalWrite(EN_VCC_L,1);
		
		//Set pin definitions for later
		pwmPin = PWM_L;
		pinA = IN_A_L;
		pinB = IN_B_L;
		kM = 1.0;
		
		//Attach encoder interrupt
		//Int1 on pin 3
		//attachInterrupt(1,readEncoderLeft,RISING);
	}
	
	//Enable RIGHT motor
	else
	{
		//Direction and PWM
		pinMode(IN_A_R, OUTPUT);
		pinMode(IN_B_R, OUTPUT);
		pinMode(PWM_R, OUTPUT);
		
		//Encoder input
		pinMode(EN_B_R, INPUT);
		pinMode(EN_A_R, INPUT);
		
		//Encoder power
		pinMode(EN_GND_R, OUTPUT);
		pinMode(EN_VCC_R, OUTPUT);
		
		//Enable power and ground
		digitalWrite(EN_GND_R,0);
		digitalWrite(EN_VCC_R,1);
		
		//Set pin definitions for later
		pwmPin = PWM_R;
		pinA = IN_A_R;
		pinB = IN_B_R;
		kM = 0.95;
		
		//Attach encoder interrupt
		//Int0 on pin 2
		//attachInterrupt(0,readEncoderRight,RISING);
	}
	
	//Both motors start at 0
	encoderCount = 0;
	prevEncoderCount = 0;
}

//Set power pins and PWM frequency
void Motor::initializeMotorController()
{
	//Set motor controller power pin
	pinMode(MC_VCC, OUTPUT);
	
	//Enable power (Ground is dedicated pin)
	digitalWrite(MC_VCC,1);

	//Change pin 9, 10 PWM frequency to 3.9 kHz
	TCCR1B = TCCR1B & 0b11111000 | 0x2;
}

//Set motor direction
void Motor::setDirection(int dir)
{
	//Drive FORWARD
	if(dir >0)
	{
		digitalWrite(pinA,0);
		digitalWrite(pinB,1);
	}
	//Drive BACKWARDS
	else
	{
		digitalWrite(pinA,1);
		digitalWrite(pinB,0);
	}
}

//Set the speed of the motor
void Motor::setSpeed(int speed)
{
	//Constrain output
	speed = constrain(speed,-255,255);
	
	//Direction is already set (hopefully)
	speed = abs(speed);
	
	//Output PWM signal
	analogWrite(pwmPin,(int)(speed*kM));
}

//Set the speed and direction of the motor
void Motor::setVelocity(int vel)
{
	setDirection(vel);
	setSpeed(vel);
}

//Exactly what it says
int Motor::getEncoderCount()
{
	return encoderCount;
}

//Compute motor speed (rev/s)
float Motor::getSpeed(float dt)
{
	//Calculate number of counts motor has moved
	int deltaEncoderCount = encoderCount - prevEncoderCount;
	
	//Calculate motor speed
	//16 counts * 30:1 gear ratio = 480 counts/revolution
	float motorSpeed = (deltaEncoderCount/(480.0))/dt;
	
	//Save current motor speed
	prevEncoderCount = encoderCount;
	
	//Return speed
	return motorSpeed;
}

// --- ISRs moved to MainProgram because C++ is dumb ---

//ISR for reading RIGHT encoder
//Rising edge of EN_A_R (pin 2)
//Check EN_B_R (pin 0)
//Bit 0 of Port D (0x01)
//void Motor::readEncoderRight()
//{
//  if(PIND & 0x01)    encoderCount--;
//  else               encoderCount++;
//}

//ISR for reading LEFT encoder
//Rising edge of EN_A_L (pin 3)
//Check EN_B_L (pin 1)
//Bit 1 of Port D (0x02)
//void Motor::readEncoderLeft()
//{
//  if(PIND & 0x02)    encoderCount--;
//  else               encoderCount++;
//}

//1150x3597