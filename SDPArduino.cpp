#include "SDPArduino.h"
#include <Arduino.h>
#include <Wire.h>
#include "State.h"


#define MotorBoardI2CAddress 4

void SDPsetup() {
  //Initial set up for arduino connected to the power board.
  pinMode(2,INPUT);
  pinMode(3,OUTPUT);
  pinMode(4,INPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,INPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,INPUT);
  pinMode(11,INPUT);
  pinMode(12,INPUT);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  digitalWrite(8,HIGH); //Pin 8 must be high to turn the radio on!
  Wire.begin(); //Makes arduino master of the I2C line.

  // Make sure we're all zero'd out
  motorAllStop();
}

void write_powers()
{
    for(int i=0; i < motor_count; i++){
        #ifdef FW_DEBUG
        Serial.print(i);
        Serial.print(F(": "));
        Serial.print(state.motors[i]->power);
        Serial.print(F(" "));
        #endif
        if(state.motors[i]->power < 0){
            motorBackward(state.motors[i]->port, abs(state.motors[i]->power));
        } else if (state.motors[i]->power == 0) {
            motorBrake(state.motors[i]->port, 255);
        } else {
            motorForward(state.motors[i]->port,  abs(state.motors[i]->power));
        }
        state.motors[i]->last_write = millis();
    }
    #ifdef FW_DEBUG
    Serial.println();
    #endif
}

void write_powers(int overwrite)
{
    for(int i=0; i < motor_count; i++){
        state.motors[i]->power = overwrite;
    }
    write_powers();
}

void motorForward(int motorNum, int motorPower) { //Makes Motor motorNum go forwards at a power of motorPower
  if (motorNum >= 0 and motorNum <= 5){
    if (motorPower < 0){ //Lowest power possible = 0.
      motorPower = 0;
    }
    if (motorPower > 255) {//Highest power possible = 255.
      motorPower = 255;
    }
    int motorMode = 2; //Mode 2 is Forward
    byte motor1 = motorNum<<5 | 24 | motorMode<<1 ;//Build Command Byte
    byte motor2 = motorPower;
    uint8_t sender[2] = {motor1, motor2};
    Wire.beginTransmission(MotorBoardI2CAddress); //open I2C communation to Motor Board.
    Wire.write(sender,2);                    //send data.
    byte fred = Wire.endTransmission();		//end I2C communcation.
  }
}

void motorBackward(int motorNum, int motorPower) { //Makes Motor motorNum go backwards at a power of motorPower
  if (motorNum >= 0 and motorNum <= 5){
    if (motorPower < 0){
      motorPower = 0;//Lowest power possible = 0.
    }
    if (motorPower > 255) {
      motorPower = 255;//Highest power possible = 255.
    }
    int motorMode = 3; //Mode 3 is Backwards.
    byte motor1 = motorNum<<5 | 24 | motorMode<<1 ;//Build Command Byte
    byte motor2 = motorPower; //Power Byte.
    uint8_t sender[2] = {motor1, motor2};
    Wire.beginTransmission(MotorBoardI2CAddress); //open I2C communation to Motor Board.
    Wire.write(sender,2);        	 // sends two byte  
    byte fred = Wire.endTransmission();//end I2C communcation.

  }
}

// ABANDON ALL HOPE, YE WHO ENTER HERE
void motorBrake(int motorNum, int motorPower) { //Makes Motor motorNum brake at a power of motorPower
  if (motorNum >= 0 and motorNum <= 5){
    if (motorPower < 0){ //Lowest power possible = 0.
      motorPower = 0;
    }
    if (motorPower > 255) {//Highest power possible = 255.
      motorPower = 255;
    }
    int motorMode = 1; //Mode 1 is Brake ??
    byte motor1 = motorNum<<5 | 24 | motorMode<<1 ;//Build Command Byte
    byte motor2 = motorPower;
    uint8_t sender[2] = {motor1, motor2};
    Wire.beginTransmission(MotorBoardI2CAddress); //open I2C communation to Motor Board.
    Wire.write(sender,2);                    //send data. 
    byte fred = Wire.endTransmission();		//end I2C communcation.
  }
}

void motorStop(int motorNum) { // stop motor motorNum
  if (motorNum >= 0 and motorNum <= 5){
    int motorMode = 0;				   //Mode 0, floats the motor.
    byte motor1 = motorNum<<5 | 16 | motorMode<<1; //Set up I2C byte to be send.
    uint8_t sender[1] = {motor1};
    Wire.beginTransmission(MotorBoardI2CAddress); //open I2C communation to Motor Board.
    Wire.write(sender,1);        	          // sends a byte  
    byte fred = Wire.endTransmission();		  // close commucation.
  }
}

void motorAllStop() {
  //I2C command to stop all Motors. 
  byte allStop = 1;				//Motor Board stops all motors if bit 0 is high.
  uint8_t sender[1] = {allStop};
  Wire.beginTransmission(MotorBoardI2CAddress); //open I2C communation to Motor Board.
  Wire.write(sender,1);        			// sends a byte
  byte fred = Wire.endTransmission();		//end I2C commucation.
}

void setPWMpin(int portNum, int power){ //PortNum is the Sensor port used, pwm is the power of the output (between 0-100)
  if (portNum >= 0 and portNum <= 3){
    if (power < 0){
      power = 0;
    }
    if (power > 100) {
      power = 100;
    }
    int pwm_value = int(power * 2.55); // PWM output is a value between 0-255 not 0-100.
    switch (portNum) {
    case 0:
      analogWrite(3,pwm_value);
      break;
    case 1:
      analogWrite(5,pwm_value);
      break;
    case 2:
      analogWrite(9,pwm_value);
      break;
    case 3:
      analogWrite(6,pwm_value);
      break;
    }
  }  
}

int readAnalogSensorData(int portNum){	//PortNum is the Sensor port used
  if (portNum >= 0 and portNum <= 3){
    int sensorData = -1;
    switch (portNum) {
    case 0:
      sensorData = analogRead(A3);
      break;
    case 1:
      sensorData = analogRead(A2);
      break;
    case 2:
      sensorData = analogRead(A1);
      break;
    case 3:
      sensorData = analogRead(A0);
      break;
    }
    return sensorData;
  }
  else{
    return -2; 
  }
}

int readDigitalSensorData(int portNum){ //PortNum is the Sensor port used
  if (portNum >= 0 and portNum <= 3){
    int sensorData = -1;
    switch (portNum) {
    case 0:
      sensorData = digitalRead(A3);
      break;
    case 1:
      sensorData = digitalRead(A2);
      break;
    case 2:
      sensorData = digitalRead(A1);
      break;
    case 3:
      sensorData = digitalRead(A0);
      break;
    }
    return sensorData;
  }
  else{
    return -2; 
  }
}



