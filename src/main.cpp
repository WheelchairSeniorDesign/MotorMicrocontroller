/* Eren Tekbas for ECE Senior Design 2025 
MicroController Handling Reference Speed
This code is for the microcontroller to retrieve the target speed from the onboard 
computer and send it to the motor controller after adjusting it. 
It will also read the speed of the motor and send it to the onboard computer.
*/

#include <Arduino.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dacA;
Adafruit_MCP4725 dacB;
// For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)

int dacClockPin = 5; //GPIO number for these variables
int brakePin = 10;
int directionLPin = 12;
int directionRPin = 13;
int enablePin = 11;
int speedPin = 4;
int motorSpeedPin = 22;


//variables to be used in the code
bool brake; // brake for motor controller
bool directionL; // direction for motor controller LEFT
bool directionR; // direction for motor controller RIGHT
bool enable; // enable for motor controller
int motorSpeed; // value read from the motor speed sensor
int8_t refSpeedR; // value sent to the motor controller for speed of right motor
int8_t refSpeedL; // value sent to the motor controller for speed of left motor

struc refSpeed{ // struct to hold the reference speed
    int speedR;
    int speedL;
    }

void setup(){
    refSpeed joystickSpeed{}; // initiate struc which will hold the reference speed
    Serial.begin(115200); // start I2C communication protocol
    dacA.begin(0x62); // initiate the DACs
    dacB.begin(0x63);
    pinMode(dacClockPin,OUTPUT); // set the pins to be used as output
    pinMode(speedPin,OUTPUT);
    pinMode(directionLPin,OUTPUT);
    pinMode(directionRPin,OUTPUT);
    pinMode(brakePin,OUTPUT);
    pinMode(refSpeedPin,INPUT); // set the pins to be used as input
    pinMode(motorSpeedPin,INPUT);
}

void loop(){
  enable = true; // enable the motor controller
    /*joystickSpeed{} = digitalRead(refSpeedPin); // read the reference speed from the onboard computer
    speedR = joystickSpeed.speedR;
    speedL = joystickSpeed.speedL;
    */ 
   // this part will be discussed with the sensors team
    directionR = true; // initially forward direction
    directionL = true; // initially forward direction
    brake = false; // initially no brake
    if(speedR == 0 && speedL == 0){ // if the reference speed is 0, stop the motor
      brake = true; // activate the brake if joystick outputs 0 in both directions
    }
  
    else{
      brake = false; // deactivate the brake if joystick outputs a value other than 0
      if(speedR<0){
        directionR = false; // set the direction to right if the joystick outputs a negative value
        refSpeedR = speedR*-1; // set the reference speed to the absolute value of the joystick output
      }
      else if(speedL<0){
        directionL = false; // set the direction to left if the joystick outputs a positive value
        refSpeedL = speedL*-1; // set the reference speed to the absolute value of the joystick output
      }
      else{
        refSpeedR = speedR; // set the referrence speed to the joystick output
        refSpeedL = speedL;
      }
      refSpeedR = refSpeedR*4095/100; // adjust the reference speed Right to the motor controller
      refSpeedL = refSpeedL*4095/100; // adjust the reference speed Left to the motor controller

    }

    dacA.setVoltage(refSpeedR, false);
    dacB.setVoltage(refSpeedL, false);

}
