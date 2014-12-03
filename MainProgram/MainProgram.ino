//
// Created for the *Balance-O-Matic* robotics project
// Written by Kevin McLeod and Myles Shepherd, Summer 2014
// I haven't got a clue how all that copyright shit works
//
// MainProgram.whateveranarduinofileis
// For balancing n' shit
//
// Does all the stuff
//
// Arduino Uno Rev 3 Conections (Please don't change this again!):
// *0:    NC                        *A5:    NC [Alternate connection: Sensor, Data (SCL)]
// *1:    NC                        *A4:    NC [Alternate connection: Sensor, Clock (SDA)]
// *2:    Encoder A, RIGHT          *A3:    Encoder Vcc(+5), RIGHT
// *3:    Encoder A, LEFT           *A2:    Encoder Ground, RIGHT
// *4:    Encoder B, RIGHT          *A1:    Encoder Vcc (+5), LEFT
// *5:    Encoder B, LEFT           *A0:    Encoder Ground, LEFT
// *6:    Controller A, RIGHT
// *7:    Controller A, LEFT        *Vin:   NC
//                                  *GND:   Debug
// *8:    Controller B, RIGHT       *GND:   Sensor Ground
// *9:    Controller PWM, RIGHT     *5V:    Sensor Vcc (+5)
// *10:   Controller PWM, LEFT      *3.3V:  NC
// *11:   Controller B, LEFT        *RESET: NC
// *12:   Controller Vcc (+5)       *IOREF: NC
// *13:   Debug
// *GND:  Controller Ground
// *AREF: NC
// *SDA:  Sensor, Data (SCL)
// *SLC:  Sensor, Clock (SDA)
//


// --- INCLUDE LIBRARIES ---
#include  <Motor.h>                 //Motors & Motor Controller
#include  <KalmanFilter.h>          //Kalman Filter
#include  <PIDController.h>         //PID controller

#include  <Wire.h>                  //I2C
#include  <LSM303.h>                //Accelerometer
#include  <L3G.h>                   //Gyrosope


// --- "GLOBAL" VARRIABLES & CONSTANTS ---

//Debugging
#define DEBUG_MODE 3                //0:No Output, 1:Data Logging, 2:Debugging, 3:PID Tuning
int printCount = 0;                 //Counter for data logging
float errors[3];                    //For printing PID errors
int command;                        //Command sent over Serial
float commandValue;                 //Value sent with command
float gains[3];                     //For printing PID gains
PIDController* controller;          //Controller being tuned


//Utilities
#define R2D  57.29578               //Convert Radiand to Degrees (fuck math)
#define MAX_TILT 45                 //Kill switch angle (deg)


//Motors
Motor rightMotor(1);                //Declare and initialize the RIGHT motor
Motor leftMotor(0);                 //Declare and initialize the LEFT motor


//Sensors
LSM303 accl;                        //Accelerometer object
L3G    gyro;                        //Gyroscope object


//Timing
#define  LOOP_TIME  10              //Time between each loop (ms)
float dt = (float)LOOP_TIME/1000.0; //Time betwwen loops (s)
unsigned long prevMillis = 0;       //Time loops was last executed (ms)
int loopCounter = 0;                //For dividing main loop frequency


//Kalman Filter   
KalmanFilter filter;                //Kalman filter


//PID
PIDController angleController;      //PID controller for vertical angle (I made this!)
float setAngle = 0.0;               //Balancing point (deg) *See angleTrim
#define  AkP  48.0                  //P gain
#define  AkI  8.0                   //I gain
#define  AkD  72.0                  //D gain
float motorValue = 0.0;             //PWM value to send to motors [-255,255], output of angleController

PIDController positionController;   //PID controller for horozontal position
float setPosition = 0;              //Home position (edcoder counts)
#define  XkP  -0.001                //P gain
#define  XkI  -0.0000               //I gain
#define  XkD  -0.03                 //D gain
float positionAngleOffset = 0.0;    //Angle position for angleController (deg), output of positionController

PIDController velocityController;   //PID controller for wheel velocity
float setVelocity = 0;              //Setpoint (rev/s)
#define VkP  0                 //P gain
#define VkI  0                //I gain
#define VkD  0                  //D gain
float velocityAngleOffset = 0.0;    //Angle position for angleController (deg), output of velocityController


//Control
float thetaMeasured = 0.0;          //Measured angle (deg)
float gyroMeasured = 0.0;           //Measured angular rate (deg/s)
float angleTrim = -0.85;            //Adjust angle set-point for asymmetry
float theta = 0.0;                  //Current angle, filtered (deg)
float encoderPosition = 0.0;        //Measured position (encoder counts)
float velocity = 0.0;               //Measured linear velocity (rev/s)


// --- MAIN FUNCTIONS ---

//Do some stuff before the other stuff
void setup()
{  
  if(DEBUG_MODE)
  {
    //Initialize serial communications
    delay(10);
    Serial.begin(115200);
    delay(10);
    Serial.setTimeout(0);
    
    //Buzzer for debugging
    pinMode(13,OUTPUT);
    
    //Only YOU can prevent segfaults!
    controller = &angleController;
  }
  
  //Configure motor controller
  Motor::initializeMotorController();
  
  //Attach encoder interrupt, RIGHT
  //Int0 on pin 2
  attachInterrupt(0,readEncoderRight,RISING);
  
  //Attach encoder interrupt, LEFT
  //Int1 on pin 3
  attachInterrupt(1,readEncoderLeft,RISING);
  
  //I2C communication
  Wire.begin();
  
  //Initialize sensors
  accl.init();  
  gyro.init();
  //No idea what "default" is (let's hope it's good!)
  accl.enableDefault();
  gyro.enableDefault();
  
  //Initialize PID controllers
  angleController.setPGain(AkP);
  angleController.setIGain(AkI);
  angleController.setDGain(AkD);
  angleController.setSetSetPoint(setAngle+angleTrim);
  if(AkI) angleController.setSumErrorMax(255.0/AkI);  //Don't let the I term take over
  
  positionController.setPGain(XkP);
  positionController.setIGain(XkI);
  positionController.setDGain(XkD);
  positionController.setSetSetPoint(setPosition);
  positionController.setSumErrorMax(1000);
  
  velocityController.setPGain(VkP);
  velocityController.setIGain(VkI);
  velocityController.setDGain(VkD);
  velocityController.setSetSetPoint(setVelocity);
  
  //Get out of the way!
  delay(1000);
  
  //Print to terminal
  if(DEBUG_MODE >= 2) Serial.println("Here we go!");
}

//Do all the other stuff
void loop()
{
  //Check if it's time to enter the loop
  if( (millis()-prevMillis) >= LOOP_TIME)
  {    
    //Update timming
    prevMillis = millis();
    
    /******************************************************/
    /*** Step 1: Read sensors & calculate current state ***/
    /******************************************************/
    
    //Read sensors    
    thetaMeasured = measureAngle();
    gyroMeasured = measureRate();

    //Use Kalman filter to calculate angle
    theta = filter.calculate(thetaMeasured,gyroMeasured,dt); 
    
    //Get the linear velocity
    velocity = measureVelocity();
    
    //Get encoder position
    encoderPosition = (leftMotor.getEncoderCount() + rightMotor.getEncoderCount())/2.0;
    
    /*****************************************/
    /*** Step 2: Calculate desired outputs ***/
    /*****************************************/
    
    //Update loop counter
    loopCounter++;
    
    //Position/Velocity controllers execute at 10Hz
    if(loopCounter>10)
    {
      //Compute tilt angle using encoder position (Don't run away!)
      positionAngleOffset = positionController.calculate(encoderPosition);
    
      //Compute tilt angle using velocity (Don't run away!)
      velocityAngleOffset = velocityController.calculate(velocity);
    
      //Send desired angle to second controller
      angleController.setSetSetPoint(positionAngleOffset+angleTrim);
      
      //Reset counter
      loopCounter =0;
    }
    
    //Compute PWM output using angle (Don't fall over!)
    motorValue = angleController.calculate(theta);
    
    /***************************/
    /*** Step 3: Send output ***/
    /***************************/
    
    //Check kill switch
    if(abs(theta) > MAX_TILT)
    {
       //Reset errors
       angleController.resetErrors();
       velocityController.resetErrors();
       
       //Stop motors if past safe angle
       motorValue = 0.0;
    }
    
    //Send output to motors
    setMotors((int)motorValue);  
    
    /*************/
    /*** Done! ***/
    /*************/    
    
    //Do some debugging/data logging/parameter tuning/whatever the hell else gets crammed in there...
    if(DEBUG_MODE) debug();          
    
  }
  //There is no else
}


// --- OTHER FUNCTIONS ---  

//Return current angle as measured by accelerometers (in degrees)
float measureAngle()
{
    //Read data from accelerometers
    accl.read();
    
    //Calculate current angle using math
    return -atan2(accl.a.x,accl.a.z)*R2D;
}

//Return current angular rate as measured by gyroscope (in degrees/second)
float measureRate()
{
   //Read data form gyroscope
   gyro.read();
   
   //Scale raw reading:
   //16 bit signed value, default sensitivity is +/-245 deg/s
   //65536/490 = 133.7469388...
   return gyro.g.y/133.747;
}

//Set the speed and direction of both motors
void setMotors(int value)
{
   leftMotor.setVelocity(value);
   rightMotor.setVelocity(value); 
}

//Return current average linear velocity using motor encoders
float measureVelocity()
{
   return (leftMotor.getSpeed(dt)+rightMotor.getSpeed(dt))/2.0; 
}

//Debugging function
void debug()
{
    //External Data Processing
    if(DEBUG_MODE == 1)
    {
      printCount++;
      //Send data at <50Hz
      if(printCount>2)
      {
        //Read current error values [error,sumError,deltaError]
        velocityController.getErrors(errors);
        
        //Procesing app. takes 6 channels
        Serial.print(errors[0]);
        Serial.print(" ");
        Serial.print(errors[1]);
        Serial.print(" ");
        Serial.print(errors[2]);
        Serial.print(" ");
        Serial.print(3);
        Serial.print(" ");
        Serial.print(4);
        Serial.print(" ");
        Serial.print(5);
        Serial.print('\r');
        printCount=0;
      }  
    }
    
    //Print debugin report
    if(DEBUG_MODE == 2)
    {
      Serial.print(encoderPosition);
      Serial.print("\t\t");
      Serial.println(positionAngleOffset);
    }
    
    //Recieve commands for PID tuning
    if(DEBUG_MODE == 3)
    {
       //Check for data transmition
       if(Serial.available() > 0)
       {
         //Command is second character
         command = Serial.read();
         
         //Value is next valid float in buffer
         commandValue = Serial.parseFloat();
         
         //A switch statment! How exciting! Look at those CS degrees paying off!
         switch(command)
         {
           //Change to angle controller
           case 97:   //ASCII code for 'a'
             controller = &angleController;
             Serial.println("ANGLE:");
             break;
           
           //Change to velocity controller
           case 118:  //ASCIIcode for 'v'
             controller = &velocityController;
             Serial.println("VELOCITY:");
             break;
           
           //Set P gain
           case 112:  //ASCII code for 'p'
             controller->setPGain(commandValue);
             break;
             
           //Set I gain
           case 105:  //ASCII code for 'i'
             controller->setIGain(commandValue);
             //if(commandValue) controller->setSumErrorMax(255.0/commandValue);
             //controller->resetErrors(); //That fucking I term!
             break;
             
           //Set D gain
           case 100:  //ASCII code for 'd'
             controller->setDGain(commandValue);
             break;
             
           //Set theta trim
           case 116:  //ASCII code for 't'
             angleController.setSetSetPoint(commandValue);
             Serial.print("Angle Trim: ");
             Serial.println(commandValue);
             break;
             
           //Print out some data
           case 113:  //ASCII code for 'q'
             Serial.println(encoderPosition);
             break;
             
           //Do nothing
           default:;
         }
         
         //Print out new gains
         controller->getGains(gains);
         Serial.print("kP = ");
         Serial.print(gains[0],4);
         Serial.print(",\tkI = ");
         Serial.print(gains[1],4);
         Serial.print(",\tkD = ");
         Serial.print(gains[2],4);
         Serial.println("");
         
         //Clear buffer
         while(Serial.available()) Serial.read();
       }
    } 
}

// --- Interrupt Service Routines ---

//ISR for reading RIGHT encoder
//Rising edge of EN_A_R (pin 2)
//Check EN_B_R (pin 4)
//Bit 4 of Port D (0x10)
void readEncoderRight()
{
  if(PIND & 0x10)    rightMotor.encoderCount--;
  else               rightMotor.encoderCount++;
}

//ISR for reading LEFT encoder
//Rising edge of EN_A_L (pin 3)
//Check EN_B_L (pin 5)
//Bit 5 of Port D (0x20)
void readEncoderLeft()
{
  if(PIND & 0x20)    leftMotor.encoderCount++;
  else               leftMotor.encoderCount--;
}

//1150x3597
