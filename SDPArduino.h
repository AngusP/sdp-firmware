//SDPArduino.h
 
#ifndef HEADER_ARDUINOSDP
#define HEADER_ARDUINOSDP

#define FW_DEBUG

//Power is a value between 0 and 255. 
void SDPsetup(void);	//Sets up the arduino, run inside of void setup().
void helloWorld(void); //prints Hello World to serial.
void write_powers(void);
void write_powers(int overwrite);
void motorForward(int motorNum, int motorPower); //Makes Motor motorNum go forwards at a power of motorPower
void motorBackward(int motorNum, int motorPower); //Makes Motor motorNum go backwards at a power of motorPower
void motorBrake(int motorNum, int motorPower); // Makes motors stop FAST (I hope)
void motorStop(int motorNum); //Makes Motor motorNum stop
void motorAllStop(void);	//Makes all motors stop
void setPWMpin(int portNum, int power); // sets the output pin of port portNum to power%.
int readAnalogSensorData(int portNum); //reads the analog value of the input pin on port portNum.
int readTouchSensorData(int portNum); //reads the analog value of the output pin on port portNum.
void brake_motors();
   
#endif

