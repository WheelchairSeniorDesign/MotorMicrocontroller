#include <Arduino.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dacA;
Adafruit_MCP4725 dacB;
// For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
dacA.begin(0x62);
dacB.begin(0x63);

int dacClock = 5; //GP number for these variables
int brake = 10;
int directionL = 12;
int directionR = 13;
int enable = 11;
int speed = 4;
int refSpeed = 2;
int motorSpeed = 22;
void setup() { //read analogue input from the pin
pinMode(dacClock,OUTPUT);
pinMode(speed,OUTPUT);
pinMode(directionL,OUTPUT);
pinMode(directionR,OUTPUT);
pinMode(brake,OUTPUT);
pinMode(refSpeed,INPUT);
}

void loop() {
v1 = analogRead(feedbackPin);
  digitalWrite(triggerPin, HIGH);
  for (int i = 0; i < 10; i++) {
    delayMicroseconds(2);
    v2= bitRead(v1,9 - i);
    digitalWrite(outputPin,v2);
    delayMicroseconds(2);
    digitalWrite(outputPin,LOW);
}
  digitalWrite(triggerPin, LOW);}

