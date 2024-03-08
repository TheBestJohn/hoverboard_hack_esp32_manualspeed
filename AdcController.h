#ifndef ADC_CONTROLLER_H
#define ADC_CONTROLLER_H

#include <Arduino.h>

// Constants for ADC handling
const int ADC_MAX_VALUE = 4095;
const int ADC_MID_VALUE = ADC_MAX_VALUE / 2;
const int ADC_DEADBAND = 100; // Deadband range in the middle that will output zero
const int MIN_SPEED = -1000;
const int MAX_SPEED = 1000;

// ADC input pins
const int adc_input_pin_right = 36;
const int adc_input_pin_left = 37;

void readAndProcessAdc();
void setMotorSpeedForGroup(const int* motorGroup, size_t motorGroupSize, int speed);

#endif // ADC_CONTROLLER_H