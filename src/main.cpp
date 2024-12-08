#include <Arduino.h>
#include <Adafruit_MCP4725.h>

#if defined(ROS) || defined(ROS_DEBUG)
#include "microRosFunctions.h"
#endif

Adafruit_MCP4725 dacA;
Adafruit_MCP4725 dacB;
// For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)

int dacClockPin = 5; //GP number for these variables
int brakePin = 10;
int directionLPin = 12;
int directionRPin = 13;
int enablePin = 11;
int speedPin = 4;
int refSpeedPin = 2;
int motorSpeedPin = 22;

int dacClock; //variables to be used in the code
int brake;
int directionL;
int directionR;
int enable;
int speed;
int refSpeedInt;
int motorSpeed;
refSpeed omegaRef;


void setup(){
    Serial.begin(115200);
    dacA.begin(0x62);
    dacB.begin(0x63);
    pinMode(dacClockPin,OUTPUT);
    pinMode(speedPin,OUTPUT);
    pinMode(directionLPin,OUTPUT);
    pinMode(directionRPin,OUTPUT);
    pinMode(brakePin,OUTPUT);
    pinMode(refSpeedPin,INPUT);

#ifdef ROS
    const char* nodeName = "motors_node";
    const char* subTopicName = "refSpeed";
    const char* pubTopicName = "motorSpeed";
    microRosSetup(1, nodeName, subTopicName, pubTopicName);
#elif ROS_DEBUG
    const char* nodeName = "motors_node";
    const char* subTopicName = "sensors";
    const char* pubTopicName = "motorSpeed";
    microRosSetup(1, nodeName, subTopicName, pubTopicName);
#endif
}

void loop(){

#if defined(ROS) || defined(ROS_DEBUG)
    checkSubs();
    omegaRef = getRefSpeed();
#endif

    refSpeedInt = digitalRead(refSpeedPin);
    if(refSpeedInt == 0){
      brake = 0;
      enable = 0;
    }

    refSpeedInt=refSpeedInt/4095;
    dacA.setVoltage(refSpeedInt*4095, false);
}
