#include <Adafruit_MAX31865.h>
#include <ModbusRTU.h>
#include <PID_v2.h>
#include <RotaryEncoder.h>
#include <Ticker.h>

#include "algorithms.h"
#include "coffee_roaster.h"

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(9, 10, 12, 13);
ModbusRTU mb;

constexpr int SLAVE_ID = 1; // slave id for modbus protocol for Artisan
constexpr int TEMPERATRE_REGISTER = 2;
constexpr int SETPOINT_REGISTER = 4;
constexpr int FAN_PWM_REGISTER = 3;

double kp = 25, ki = 1, kd = 0;
PID_v2 pid(kp, ki, kd, PID::Direct);
RotaryEncoder encoder(rotaryAPin, rotaryBPin, RotaryEncoder::LatchMode::FOUR3);
Ticker dumpData;
Ticker sampleTemperature;

// flags
volatile bool shouldSampleTemperature = false;
volatile bool shouldDumpData = false;

void setSampleTemperature() { shouldSampleTemperature = true; }

void setDumpData() { shouldDumpData = true; }

/**
 * @brief The interrupt service routine will be called on any change of one of
 * the input signals.
 */
ICACHE_RAM_ATTR void checkPosition() {
  encoder.tick(); // just call tick() to check the state.
}

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200, SERIAL_8N1);
  thermo.begin(MAX31865_3WIRE); // set to 2WIRE or 4WIRE as necessary

  // declare pin 9 to be an output:
  pinMode(dcMotor, OUTPUT);
  pinMode(ssrPin, OUTPUT);
  digitalWrite(ssrPin, LOW);  
  pinMode(potPin, INPUT);
  pinMode(rotaryAPin, INPUT);
  pinMode(rotaryBPin, INPUT);
  digitalWrite(dcMotor, LOW);

  attachInterrupt(rotaryAPin, checkPosition, CHANGE);
  attachInterrupt(rotaryBPin, checkPosition, CHANGE);

  const float temperature = getTemperature();
  pid.SetOutputLimits(0, 1023);
  pid.Start(temperature, 0, 0);

  sampleTemperature.attach(0.1, setSampleTemperature);
  dumpData.attach(1, setDumpData);

  mb.begin(&Serial);
  mb.slave(SLAVE_ID);
  mb.addHreg(TEMPERATRE_REGISTER);
  mb.addHreg(SETPOINT_REGISTER);
  mb.addHreg(FAN_PWM_REGISTER);

  mb.Hreg(TEMPERATRE_REGISTER, static_cast<uint16_t>(temperature));
  mb.Hreg(SETPOINT_REGISTER, 0);
  mb.Hreg(FAN_PWM_REGISTER, 0);
}

void loop() {
  mb.task();

  /// keep track of interrupts
  static int prevSetPoint = 0;
  static int prevFanSpeed = 0;


  /// update the fan speed
  const int fanSpeed = getMotorSpeed();
  if (prevFanSpeed != fanSpeed) {
    prevFanSpeed = fanSpeed;
    analogWrite(dcMotor, fanSpeed);
  }

  /// update setpoint if needed
  const int setPoint = getEncoderPosition();
  if (setPoint != prevSetPoint) {
    mb.Hreg(SETPOINT_REGISTER, setPoint);
    pid.Setpoint(setPoint);
    prevSetPoint = setPoint;
  }

  /// sample the tempreature every 100 ms
  if (shouldSampleTemperature) {
    float temperature = getTemperature();
    mb.Hreg(TEMPERATRE_REGISTER, static_cast<uint16_t>(temperature));
    const float output = pid.Run(temperature);
    analogWrite(ssrPin, output);
    shouldSampleTemperature = false;
  }

  if (shouldDumpData) {
    /*Serial.print("Temperature");
    Serial.print("\t");
    Serial.print(temperature);
    Serial.print("\t");
    Serial.print("Setpoint");
    Serial.print("\t");
    Serial.println(setPoint);*/
    shouldDumpData = false;
  }
  yield();
  delay(10); // delay needed to avoid WDT
}

float getTemperature() {
  uint16_t rtd = thermo.readRTD();
  float ratio = rtd;

  // Check and print any faults
  uint8_t fault = thermo.readFault();
  if (fault) {
    Serial.print("Fault 0x");
    Serial.println(fault, HEX);
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
  return thermo.temperature(MAX31865_RNOMINAL, MAX31865_RREF) * 1.8 + 32;
}

int getEncoderPosition() {
  // the maximum acceleration is 10 times.
  constexpr float m = 5;

  // at 200ms or slower, there should be no acceleration. (factor 1)
  constexpr float longCutoff = 10;

  // at 5 ms, we want to have maximum acceleration (factor m)
  constexpr float shortCutoff = 5;

  // To derive the calc. constants, compute as follows:
  // On an x(ms) - y(factor) plane resolve a linear formular factor(ms) = a * ms
  // + b; where  f(4)=10 and f(200)=1

  constexpr float a = (m - 1) / (shortCutoff - longCutoff);
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
  } // if
  return newPos;
}

int getMotorSpeed() {
  const auto rawADC = analogRead(A0);
  return rawADC < 30 ? 0 : rawADC;
}
