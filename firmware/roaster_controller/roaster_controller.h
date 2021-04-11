/// Andrue Peters
/// 4/10/2021

#ifndef COFFEE_ROASTER_H
#define COFFEE_ROASTER_H

/// constants for max31865 board
constexpr float MAX31865_RREF = 430;
constexpr float MAX31865_RNOMINAL = 100.0;

/// pinouts
constexpr int dcMotor = 16;           // the PWM pin the LED is attached to
constexpr int potPin = 17; /// A0 on esp8266
constexpr int rotaryAPin = 4;
constexpr int rotaryBPin = 0;
constexpr int ssrPin = 5;

/// PID constants
constexpr double KP = 35;
constexpr double KI = 1;
constexpr double KD = 0;

/// Return the temperature in Fahrenheit
float getTemperature();

/// Gets the encoder position, which is a direct mapping to the set temperature
/// Clamps the position to [0, 450] incluseivly 
int getEncoderPosition();

/// Returns the desired speed for the motor
/// Name should probably change in the future
int getMotorSpeed();

#endif