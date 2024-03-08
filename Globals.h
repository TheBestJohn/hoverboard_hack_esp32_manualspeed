#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>

// Constants for motor control
extern const size_t motor_count_total;
extern const size_t motor_count_right;
extern const size_t motor_count_left;

// Motor arrays
extern int motors_all[];
extern int motors_right[];
extern int motors_left[];

// Motor control variables
extern int motor_speed[];
extern int slave_state[];

// Motor offset
extern int motoroffset;

// Function prototypes for functions that will be used across multiple files
void setMotorSpeedForGroup(const int* motorGroup, size_t motorGroupSize, int speed);

#endif // GLOBALS_H