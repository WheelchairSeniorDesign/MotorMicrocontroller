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
int speedFreqRPin = 15;
int speedFreqLPin = 14;


//variables to be used in the code
bool brake; // brake for motor controller
bool directionL; // direction for motor controller LEFT
bool directionR; // direction for motor controller RIGHT
bool enable; // enable for motor controller
int motorSpeed; // value read from the motor speed sensor
int16_t refSpeedR; // value sent to the motor controller for speed of right motor
int16_t refSpeedL; // value sent to the motor controller for speed of left motor

/*struc refSpeed{ // struct to hold the reference speed
    int speedR;
    int speedL;
    }*/

// //variables to handle frequecy reading and tranfer to speed
// volatile uint32_t pulse_count_1 = 0;
// volatile uint32_t pulse_count_2 = 0;
//
// void pulse_handler_1() { pulse_count_1++; }
// void pulse_handler_2() { pulse_count_2++; }
//
// float freqR;
// float freqL;
// float speedR;
// float speedL;

void setup(){
    //refSpeed joystickSpeed{}; // initiate struc which will hold the reference speed
    Serial.begin(115200); // start I2C communication protocol
    dacA.begin(0x62); // initiate the DACs
    dacB.begin(0x63);

    pinMode(speedPin,OUTPUT);
    pinMode(directionLPin,OUTPUT);
    pinMode(directionRPin,OUTPUT);
    pinMode(brakePin,OUTPUT);
    //pinMode(refSpeedPin,INPUT); // set the pins to be used as input
    //pinMode(motorSpeedPin,INPUT);


    // pinMode(speedFreqRPin, INPUT_PULLDOWN);
    // pinMode(speedFreqLPin, INPUT_PULLDOWN);
    //
    // attachInterrupt(digitalPinToInterrupt(speedFreqRPin), pulse_handler_1, RISING);
    // attachInterrupt(digitalPinToInterrupt(speedFreqLPin), pulse_handler_2, RISING);

}

// void getFreq() {
//   pulse_count_1 = 0;
//   pulse_count_2 = 0;
//   uint32_t start_time = millis();
//
//   delay(1000);  // Measure for 1 second
//
//   uint32_t elapsed_time = millis() - start_time;
//   freqR = (pulse_count_1 * 1000.0) / elapsed_time;  // Hz
//   freqL = (pulse_count_2 * 1000.0) / elapsed_time;  // Hz
//
// }

//converssion of frequency to MPH
// void freqToSpeed(){
//   speedR = (freqR*10/21.33)*3.14*12.5*60/63360;
//   speedL = (freqL*10/21.33)*3.14*12.5*60/63360;
// }

// uncomment freq L

void loop(){

  //enable = true; // enable the motor controller
    /*joystickSpeed{} = digitalRead(refSpeedPin); // read the reference speed from the onboard computer
    speedR = joystickSpeed.speedR;
    speedL = joystickSpeed.speedL;
    */ 
   // this part will be discussed with the sensors team
    /*
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
    */


    // getFreq();
    // freqToSpeed();

    // Serial.print("Right motor freq: ");
    // //Serial.print(PULSE_PIN_1);
    // Serial.print(" Frequency: ");
    // Serial.print(freqR);
    // //Serial.print(freqL);
    // Serial.println(" Hz");
    // //Serial.flush();  // Ensures all data is sent before continuing
    //serial print does not work with dac.setVoltage


    refSpeedR= 4095;
    refSpeedL= 4095;

    digitalWrite(directionLPin,false);//true is backwards, false forward
    digitalWrite(directionRPin,false);
    digitalWrite(enablePin, false);
    digitalWrite(brakePin, true);
    dacA.setVoltage(refSpeedR, false);
    dacB.setVoltage(refSpeedL, false);


}
