/// Andrue Peters
/// 4/10/2021
///
/// This sketch is a controller for a coffee roaster
/// Inputs are a Rotary Encoder for the temperature set point and a pot for the motor controller
/// This current design is for an ESP8266, but is fairly compatible with any Arduino compatible 
/// 
/// Things to note when porting the design:
///     The output of the DAC is 10 bits, so you might need to map to a different output range. (UNO is 8 bit)
///     The ADC is also 10 bits which means the pot mappings may be different

#include "algorithms.h"
#include "roaster_controller.h"

#include <Adafruit_MAX31865.h>
#include <PID_v2.h>
#include <RotaryEncoder.h>
#include <Ticker.h>


/// Adafruit driver object using software SPI
/// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(9, 10, 12, 13);

/// Rotary encoder driver object; used to set temperature
RotaryEncoder encoder(rotaryAPin, rotaryBPin, RotaryEncoder::LatchMode::FOUR3);

/// Initialize pid for heater with constants in roaster_controller.h
PID_v2 pid(KP, KI, KD, PID::Direct);

/// These tickers behave very similarly to hardware timer interrupts
Ticker dumpData;              ///< interrupt to dump data every second
Ticker sampleTemperature;     ///< interrupt to sample temperature every 100 ms

// flags used to toggle behavior
volatile bool shouldSampleTemperature = false;
volatile bool shouldDumpData = false;


void setSampleTemperature()
{
  shouldSampleTemperature = true;
}

void setDumpData()
{
  shouldDumpData = true;
}

/// Trigger encoder tick on every change of pin state
/// Used for hardware interrupt
ICACHE_RAM_ATTR void checkPosition()
{
  encoder.tick(); // just call tick() to check the state.
}

void setup() {
  Serial.begin(115200);
  thermo.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary

  // declare pin 9 to be an output:
  pinMode(dcMotor, OUTPUT);
  pinMode(ssrPin, OUTPUT);
  pinMode(potPin, INPUT);
  pinMode(rotaryAPin, INPUT);
  pinMode(rotaryBPin, INPUT);
  digitalWrite(ssrPin, LOW);    //< make the temperature OFF by default!
  analogWrite(dcMotor, analogRead(potPin) ); ///< we can make the motor on

  /// 10 bit dac, so that's values 0 to 1023
  pid.SetOutputLimits(0, 1023);
  pid.Start(getTemperature(), 0, 0);
  
  /// now attach hardware and timer interrupts
  attachInterrupt(rotaryAPin, checkPosition, CHANGE);
  attachInterrupt(rotaryBPin, checkPosition, CHANGE);

  sampleTemperature.attach(0.1, setSampleTemperature);
  dumpData.attach(1, setDumpData);
}

void loop() {

  /// keep track of interrupts
  static int prevSetPoint = 0;
  static int prevFanSpeed = 0;

  const float temperature = getTemperature();
  
  /// update the fan speed
  const int fanSpeed = getMotorSpeed();
  if (prevFanSpeed != fanSpeed) {
    prevFanSpeed = fanSpeed;
    analogWrite(dcMotor, fanSpeed);
  }
  
  /// update setpoint if needed
  const int setPoint = getEncoderPosition();
  if (setPoint != prevSetPoint) {
    pid.Setpoint(setPoint);
    prevSetPoint = setPoint;
  }

  /// sample the tempreature every 100 ms
  if (shouldSampleTemperature) {
      const float output = pid.Run(temperature);
      analogWrite(ssrPin, output);
      shouldSampleTemperature = false;
  }

  if (shouldDumpData) {
    Serial.print("Temperature"); Serial.print("\t"); Serial.print(temperature); Serial.print("\t");
    Serial.print("Setpoint"); Serial.print("\t"); Serial.println(setPoint);
    shouldDumpData = false;
  }
}

float getTemperature() 
{
  uint16_t rtd = thermo.readRTD();
  float ratio = rtd;

  // Check and print any faults
  uint8_t fault = thermo.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    thermo.clearFault();
  }
  
  /// convert to F and retrun
  return thermo.temperature(MAX31865_RNOMINAL, MAX31865_RREF) * 1.8 + 32;
}



int getEncoderPosition()
{
  constexpr float maxAcceleration = 5;
  constexpr float longCutoff = 10;

  // at 5 ms, we want to have maximum acceleration (factor maxAcceleration)
  constexpr float shortCutoff = 5;

  // To derive the calc. constants, compute as follows:
  // On an x(ms) - y(factor) plane resolve a linear formular factor(ms) = a * ms + b;
  // where  f(4)=10 and f(200)=1

  constexpr float a = (maxAcceleration - 1) / (shortCutoff - longCutoff);
  constexpr float b = 1 - longCutoff * a;

  static int lastPos = 0;
  int newPos = encoder.getPosition();
  if (lastPos != newPos) {

    // accelerate when there was a previous rotation in the same direction.
    unsigned long ms = encoder.getMillisBetweenRotations();

    if (ms < longCutoff) {
      // do some acceleration using factors a and b

      // limit to maximum acceleration
      if (ms < shortCutoff) {
        ms = shortCutoff;
      }

      float ticksActual_float = a * ms + b;
      long deltaTicks = (long)ticksActual_float * (newPos - lastPos);

      newPos = newPos + deltaTicks;
    }
    newPos = algorithms::clamp(newPos, 0, 450);
    encoder.setPosition(newPos);
    lastPos = newPos;
  } 

  return newPos;
}

int getMotorSpeed()
{
  const auto rawADC = analogRead(A0);
  return rawADC < 30 ? 0 : rawADC;
}