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

float getTemperature();
int getEncoderPosition();
int getMotorSpeed();


#endif