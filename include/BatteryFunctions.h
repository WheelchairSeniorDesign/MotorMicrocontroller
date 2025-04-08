//
// Created by Laxman on 03/02/25.
//
#include <Arduino.h>
#include "hardware/adc.h"

// Define ADC Pin for Raspberry Pi Pico
#define ADC_PIN 28 // GPIO 28 corresponds to ADC2

// Pico ADC Parameters
#define VREF 3.3
#define ADC_MAX 4095.0 // 12-bit ADC

// Op-Amp & Voltage Divider Parameters
#define OPAMP_GAIN 10.1
#define VOLTAGE_DIVIDER_RATIO 0.01205 // 10kΩ / (820kΩ + 10kΩ)

// Battery Voltage Range
#define BATTERY_VOLTAGE_MAX 27.0
#define BATTERY_VOLTAGE_MIN 21.0

// Initialize ADC to read battery voltage
void initBatterySensor()
{
    adc_init();             // Initialize ADC peripheral
    adc_gpio_init(ADC_PIN); // Enable ADC pin
    adc_select_input(2);    // ADC input 2 = GPIO 28
}

// Read raw ADC and convert to actual battery voltage
float readBatteryVoltage()
{
    uint16_t raw_adc = adc_read();                  // Raw ADC value (0–4095)
    float adc_voltage = (raw_adc * VREF) / ADC_MAX; // Convert to 0–3.3V
    float scaled_voltage = adc_voltage / VOLTAGE_DIVIDER_RATIO;
    float battery_voltage = scaled_voltage / OPAMP_GAIN;
    return battery_voltage;
}

// Convert battery voltage to 0–100% percentage
float calculateBatteryPercentage(float battery_voltage)
{
    float percentage = ((battery_voltage - BATTERY_VOLTAGE_MIN) /
                        (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN)) *
                       100.0;
    if (percentage > 100.0)
        percentage = 100.0;
    if (percentage < 0.0)
        percentage = 0.0;
    return percentage;
}
