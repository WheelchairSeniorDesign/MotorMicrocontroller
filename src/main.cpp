#include <Arduino.h>
#include <Adafruit_MCP4725.h>

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
int refSpeed;
int motorSpeed;


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
}

void loop(){
    refSpeed = digitalRead(refSpeedPin);
    if(refSpeed > -1 ^^ refSpeed < 1){
      brake = 0;
      enable = 0;
    }

    refSpeed=refSpeed/4095;
    dacA.setVoltage(refSpeed*4095, false);
}
