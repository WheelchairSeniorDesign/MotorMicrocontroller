#include <Arduino.h>
#include "hardware/adc.h"

// Define ADC Pin for Raspberry Pi Pico
#define ADC_PIN 28  // GPIO 28 corresponds to ADC2

// Pico ADC Parameters
#define VREF 3.3                     // Reference voltage for Pico's ADC
#define ADC_MAX 4095.0                // 12-bit ADC (0-4095)

// Op-Amp & Voltage Divider Parameters
#define OPAMP_GAIN 10.1               // Non-inverting op-amp gain
#define VOLTAGE_DIVIDER_RATIO 0.01205 // (R2 / (R1 + R2)) = 10kΩ / (820kΩ + 10kΩ)

// Battery Voltage Limits
#define BATTERY_VOLTAGE_MAX 27.0      // Fully charged battery voltage (V)
#define BATTERY_VOLTAGE_MIN 21.0      // Discharged battery voltage (V)

void setup() {
    Serial.begin(115200);
    adc_init();                // Initialize ADC
    adc_gpio_init(ADC_PIN);     // Setup GPIO 28 for ADC
    adc_select_input(2);        // Select ADC2 (corresponds to GPIO 28)
}

float readBatteryVoltage() {
    uint16_t raw_adc = adc_read();  // Read 12-bit ADC value
    float adc_voltage = (raw_adc * VREF) / ADC_MAX;  // Convert ADC reading to voltage (0 - 3.3V)

    // Convert back to actual battery voltage
    float scaled_voltage = adc_voltage / VOLTAGE_DIVIDER_RATIO;  // Undo voltage divider
    float battery_voltage = scaled_voltage / OPAMP_GAIN;  // Undo op-amp gain

    return battery_voltage;
}

float calculateBatteryPercentage(float battery_voltage) {
    float percentage = ((battery_voltage - BATTERY_VOLTAGE_MIN) /
                        (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN)) * 100.0;
    if (percentage > 100.0) percentage = 100.0;
    if (percentage < 0.0) percentage = 0.0;
    return percentage;
}

void loop() {
    float battery_voltage = readBatteryVoltage();
    float battery_percentage = calculateBatteryPercentage(battery_voltage);

    // Corrected print statements
    Serial.print("Battery Voltage: ");
    Serial.print(battery_voltage, 2);
    Serial.print(" V | Charge: ");
    Serial.print(battery_percentage, 1);
    Serial.println("%");

    delay(3000);  // Update every 3 seconds
}