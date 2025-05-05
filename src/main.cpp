/* Eren Tekbas for ECE Senior Design 2025
MicroController Handling Reference Speed
This code is for the microcontroller to retrieve the target speed from the onboard
computer and send it to the motor controller after adjusting it.
It will also read the speed of the motor and send it to the onboard computer.
*/

#include <Arduino.h>
#include <Adafruit_MCP4725.h>
#include "RefSpeed.h"
#include "BatteryFunctions.h"

#if defined(ROS) || defined(ROS_DEBUG)
#include "microRosFunctions.h"
#endif

Adafruit_MCP4725 dacA;
Adafruit_MCP4725 dacB;

int dacClockPin = 5;
int brakePin = 10;
int directionLPin = 12;
int directionRPin = 13;
int enablePin = 11;
int speedPin = 4;
int motorSpeedPin = 22;
int batteryPin = 28;

int speedFreqRPin;
int speedFreqLPin;

bool brake;
bool directionL;
bool directionR;
bool enable;
int motorSpeed;
int16_t refSpeedR;
int16_t refSpeedL;
int16_t batteryValue;
refSpeed refSpeedSensors;

// Global speed variables used in ROS publisher
float speedR = 0;
float speedL = 0;

volatile uint32_t pulse_count_1 = 0;
volatile uint32_t pulse_count_2 = 0;

void pulse_handler_1() { pulse_count_1++; }
void pulse_handler_2() { pulse_count_2++; }

void setup() {
    Serial.begin(115200);
    initBatterySensor();

    while (!dacA.begin(0x62)) {
        Serial.println("DAC A not found");
        delay(500);
    }
    while (!dacB.begin(0x63)) {
        Serial.println("DAC B not found");
        delay(500);
    }

    pinMode(batteryPin, INPUT);
    pinMode(directionLPin, OUTPUT);
    pinMode(directionRPin, OUTPUT);
    pinMode(brakePin, OUTPUT);

    brake = false;
    enable = true;

    pinMode(speedFreqRPin, INPUT_PULLDOWN);
    pinMode(speedFreqLPin, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(speedFreqRPin), pulse_handler_1, RISING);
    attachInterrupt(digitalPinToInterrupt(speedFreqLPin), pulse_handler_2, RISING);

#if defined(ROS) || defined(ROS_DEBUG)
    microRosSetup(1, "motor_node", "ref_speed", "motor_speed");
#endif
}

void getFreq() {
    pulse_count_1 = 0;
    pulse_count_2 = 0;
    uint32_t start_time = millis();

    delay(1000);  // Measure for 1 second

    uint32_t elapsed_time = millis() - start_time;
    speedR = (pulse_count_1 * 1000.0) / elapsed_time;
    speedL = (pulse_count_2 * 1000.0) / elapsed_time;
}

void freqToSpeed() {
    // Converts frequency to speed in MPH
    speedR = (speedR * 10 / 21.33) * 3.14 * 12.5 * 60 / 63360;
    speedL = (speedL * 10 / 21.33) * 3.14 * 12.5 * 60 / 63360;
}

void loop() {
#if defined(ROS) || defined(ROS_DEBUG)
    checkSubs();
    refSpeedSensors = getRefSpeed();
#ifdef ROS_DEBUG
    transmitDac(refSpeedL, refSpeedR);
#endif
#endif

    enable = false;
    brake = (refSpeedSensors.rightSpeed != 0 || refSpeedSensors.leftSpeed != 0);

    directionR = !(refSpeedSensors.rightSpeed > 0);
    directionL = !(refSpeedSensors.leftSpeed > 0);

    float tempRefSpeedR = abs(refSpeedSensors.rightSpeed) * 1000 / 100;
    float tempRefSpeedL = abs(refSpeedSensors.leftSpeed) * 1000 / 100;

    refSpeedR = static_cast<int16_t>(tempRefSpeedR);
    refSpeedL = static_cast<int16_t>(tempRefSpeedL);

    digitalWrite(directionLPin, directionL);
    digitalWrite(directionRPin, directionR);
    digitalWrite(enablePin, enable);
    digitalWrite(brakePin, brake);

    dacA.setVoltage(refSpeedR, false);
    dacB.setVoltage(refSpeedL, false);

    getFreq();
    freqToSpeed();
}

