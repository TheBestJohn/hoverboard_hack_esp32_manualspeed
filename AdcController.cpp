#include "AdcController.h"
#include "Globals.h" // Include a file where global variables like motor_speed are declared

void readAndProcessAdc() {
    int analogValueRight = analogRead(adc_input_pin_right);
    int analogValueLeft = analogRead(adc_input_pin_left);
    int adcspeedinright = 0;
    int adcspeedinleft = 0;

    // Process right ADC value
    if (abs(analogValueRight - ADC_MID_VALUE) > ADC_DEADBAND) {
        adcspeedinright = map(analogValueRight, 0, ADC_MAX_VALUE, MIN_SPEED, MAX_SPEED);
    }

    // Process left ADC value
    if (abs(analogValueLeft - ADC_MID_VALUE) > ADC_DEADBAND) {
        adcspeedinleft = map(analogValueLeft, 0, ADC_MAX_VALUE, MIN_SPEED, MAX_SPEED);
    }

    // Debug output if enabled
#ifdef _DEBUG
    Serial.print("Analog Right: ");
    Serial.print(analogValueRight);
    Serial.print(", Speed: ");
    Serial.println(adcspeedinright);
    Serial.print("Analog Left: ");
    Serial.print(analogValueLeft);
    Serial.print(", Speed: ");
    Serial.println(adcspeedinleft);
#endif

    // Set motor speeds based on ADC values
    setMotorSpeedForGroup(motors_right, motor_count_right, adcspeedinright);
    setMotorSpeedForGroup(motors_left, motor_count_left, adcspeedinleft);
}

void setMotorSpeedForGroup(const int* motorGroup, size_t motorGroupSize, int speed) {
    for (size_t i = 0; i < motorGroupSize; ++i) {
        int motorIndex = motorGroup[i] - motoroffset;
        motor_speed[motorIndex] = speed;
    }
}