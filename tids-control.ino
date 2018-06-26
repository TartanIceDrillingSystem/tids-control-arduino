/*
  Tartan Ice Drilling System (TIDS) on Arduino for backup fly-by-wire martian ice extraction.
  Copyright (C) 2018 Devin Gund (https://dgund.com)

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Requires Servo library for Arduino
#include <Servo.h>

// Requires HX711 library for Arduino (https://github.com/bogde/HX711)
#include "HX711.h"

// Relay pins
#define PIN_RELAY_1 39
#define PIN_RELAY_2 41
#define PIN_RELAY_3 43
#define PIN_RELAY_4 45
#define PIN_RELAY_5 47
#define PIN_RELAY_6 49
#define PIN_RELAY_7 51
#define PIN_RELAY_8 53

// Relay components
#define PIN_RELAY_CHILLER PIN_RELAY_6
#define PIN_RELAY_DRILLMOTOR PIN_RELAY_4
#define PIN_RELAY_HEATER1 PIN_RELAY_7
#define PIN_RELAY_HEATER2 PIN_RELAY_8
#define PIN_RELAY_XMOTOR PIN_RELAY_3
#define PIN_RELAY_ZMOTOR PIN_RELAY_2
#define PIN_RELAY_24V PIN_RELAY_5

// Drill motor pins
#define PIN_DRILLMOTOR_PWM 2

// X-Axis motor pins
#define PIN_XMOTOR_PLS 5
#define PIN_XMOTOR_DIR 4

// Z-axis motor pins
#define PIN_ZMOTOR_ENB_PWM 3
#define PIN_ZMOTOR_IN3 33
#define PIN_ZMOTOR_IN4 35

// Melting chamber cap servo motor pins
#define PIN_CAPSERVO1_PWM 6
#define PIN_CAPSERVO2_PWM 7

// Load cell pins
#define PIN_LOADCELL_DOUT 12
#define PIN_LOADCELL_CLK 11

// Current sensor pins
#define PIN_CURRENTSENSOR_ADC A0

// Relay defines
#define RELAY_ON LOW
#define RELAY_OFF HIGH

// PWM for melting chamber cap servo motor minimum angle
#define CAPSERVO_PULSEWIDTH_MIN 500
// PWM for melting chamber cap servo motor maximum angle
#define CAPSERVO_PULSEWIDTH_MAX 2500

// Open angle for melting chamber cap servo motor 1
#define CAPSERVO1_PWM_OPEN 20
// Closed angle for melting chamber cap servo motor 1
#define CAPSERVO1_PWM_CLOSED 90
// Open angle for melting chamber cap servo motor 2
#define CAPSERVO2_PWM_OPEN 160
// Closed angle for melting chamber cap servo motor 2
#define CAPSERVO2_PWM_CLOSED 90

// Steps per revolution for x-axis stepper motor
#define XMOTOR_STEPS_PER_REVOLUTION 1000.0
// Step factor for x-axis stepper motor
#define XMOTOR_STEP_FACTOR 1.0
// Speed for x-axis stepper motor, in RPM
#define XMOTOR_SPEED_RPM 100.0

// Pitch for x-axis leadscrew
#define XAXIS_PITCH 3.0

// Calibration factor for HX711 load cell
#define LOADCELL_CALIBRATION_FACTOR -56500.0

// Direction enum for x-axis and z-axis positioning
enum DIRECTION {
  FORWARD,
  BACKWARD
};

// HX711 load cell
HX711 loadCell(PIN_LOADCELL_DOUT, PIN_LOADCELL_CLK);

// Melting chamber cap servo motor 1
Servo capServo1;
// Melting chamber cap servo motor 2
Servo capServo2;

// Turn off all relay components
void turnOffAllRelays() {
  digitalWrite(PIN_RELAY_1, RELAY_OFF);
  digitalWrite(PIN_RELAY_2, RELAY_OFF);
  digitalWrite(PIN_RELAY_3, RELAY_OFF);
  digitalWrite(PIN_RELAY_4, RELAY_OFF);
  digitalWrite(PIN_RELAY_5, RELAY_OFF);
  digitalWrite(PIN_RELAY_6, RELAY_OFF);
  digitalWrite(PIN_RELAY_7, RELAY_OFF);
  digitalWrite(PIN_RELAY_8, RELAY_OFF);
}

// Turn on relay components for positioning
void turnOnPositioningMotors() {
  digitalWrite(PIN_RELAY_24V, RELAY_ON);
  digitalWrite(PIN_RELAY_XMOTOR, RELAY_ON);
  digitalWrite(PIN_RELAY_ZMOTOR, RELAY_ON);
}

// Turn off relay components for positioning
void turnOffPositioningMotors() {
  digitalWrite(PIN_RELAY_24V, RELAY_OFF);
  digitalWrite(PIN_RELAY_XMOTOR, RELAY_OFF);
  digitalWrite(PIN_RELAY_ZMOTOR, RELAY_OFF);
}

// Turn on drill relay and start drill at pwmSpeed
void startDrill(int pwmSpeed) {
  digitalWrite(PIN_RELAY_DRILLMOTOR, RELAY_ON);
  analogWrite(PIN_DRILLMOTOR_PWM, pwmSpeed);
}

// Stop drill and turn off drill relay
void stopDrill() {
  analogWrite(PIN_DRILLMOTOR_PWM, 0);
  digitalWrite(PIN_RELAY_DRILLMOTOR, RELAY_OFF);
}

// Open melting chamber cap
void openCap() {
  capServo1.write(CAPSERVO1_PWM_OPEN);
  capServo2.write(CAPSERVO2_PWM_OPEN);
}

// Close melting chamber cap
void closeCap() {
  capServo1.write(CAPSERVO1_PWM_CLOSED);
  capServo2.write(CAPSERVO2_PWM_CLOSED);
}

// Turn on chiller relay to start chiller
void startChiller() {
  digitalWrite(PIN_RELAY_CHILLER, RELAY_ON);
}

// Turn off chiller relay to stop chiller
void stopChiller() {
  digitalWrite(PIN_RELAY_CHILLER, RELAY_OFF);
}

// Turn on heater relays to start heater
void startHeater() {
  digitalWrite(PIN_RELAY_HEATER1, RELAY_ON);
  digitalWrite(PIN_RELAY_HEATER2, RELAY_ON);
}

// Turn off heater relays to stop heater
void stopHeater() {
  digitalWrite(PIN_RELAY_HEATER1, RELAY_OFF);
  digitalWrite(PIN_RELAY_HEATER2, RELAY_OFF);
}

// Rotate x-axis stepper motor by steps
void stepXMotor(long steps) {
  // Determine step delay based on speed and steps per revolution
  double stepDelayUS = (1000000.0 * 60.0 / XMOTOR_SPEED_RPM) / XMOTOR_STEPS_PER_REVOLUTION;

  Serial.println(stepDelayUS);

  for (long step = 0; step < steps; step++) {
    digitalWrite(PIN_XMOTOR_PLS, HIGH);
    digitalWrite(PIN_XMOTOR_PLS, LOW);
    delayMicroseconds(stepDelayUS);
  }
}

// Rotate x-axis stepper motor by angleDEG
void rotateXMotor(double angleDEG) {
  double degreesPerStep = 360.0f / (double)(XMOTOR_STEPS_PER_REVOLUTION);
  Serial.println(angleDEG);
  Serial.println(degreesPerStep);
  long numberOfSteps = floor((double)XMOTOR_STEP_FACTOR * angleDEG / degreesPerStep + 0.5);
  Serial.println(numberOfSteps);
  stepXMotor(numberOfSteps);
}

// Move x-axis by distanceMM in direciton, using stepper motor
void moveXAxisByDistance(int distanceMM, DIRECTION direction) {
  // Set direction
  if (direction == FORWARD) {
    digitalWrite(PIN_XMOTOR_DIR, HIGH);
  } else {
    digitalWrite(PIN_XMOTOR_DIR, LOW);
  }

  // Rotate motor
  double revolutions = ((double)distanceMM) / XAXIS_PITCH;
  double angleDEG = revolutions * 360.0;
  rotateXMotor(angleDEG);
}

// Start moving z-axis in direction, using motor
void startMovingZAxis(DIRECTION direction) {
  if (direction == FORWARD) {
    digitalWrite(PIN_ZMOTOR_IN3, HIGH);
    digitalWrite(PIN_ZMOTOR_IN4, LOW);
  } else {
    digitalWrite(PIN_ZMOTOR_IN3, LOW);
    digitalWrite(PIN_ZMOTOR_IN4, HIGH);
  }

  analogWrite(PIN_ZMOTOR_ENB_PWM, 255);
}

// Stop moving z-axis
void stopMovingZAxis() {
  analogWrite(PIN_ZMOTOR_ENB_PWM, 0);
}

// Move z-axis for timeMS in direction
void moveZAxisForTime(int timeMS, DIRECTION direction) {
  startMovingZAxis(direction);
  delay(timeMS);
  stopMovingZAxis();
}

// Clear weight on bit from the load cell
void clearWeightOnBit() {
  loadCell.tare();
}

// Get weight on bit from the load cell, in killigrams
float getWeightOnBitKg() {
  return loadCell.get_units();
}

// Get current for system, in amperes
float getCurrentA() {
  return ((float)analogRead(PIN_CURRENTSENSOR_ADC)) / 1023.0 * 10.0;
}

// Log weight on bit and current data to serial monitor
void logData() {
  Serial.print(getWeightOnBitKg());
  Serial.print(" kg, ");
  Serial.print(getCurrentA());
  Serial.println(" A,");
}

// Setup
void setup() {
  // Configure serial baud rate
  Serial.begin(9600);

  // Configure relay
  pinMode(PIN_RELAY_1, OUTPUT);
  pinMode(PIN_RELAY_2, OUTPUT);
  pinMode(PIN_RELAY_3, OUTPUT);
  pinMode(PIN_RELAY_4, OUTPUT);
  pinMode(PIN_RELAY_5, OUTPUT);
  pinMode(PIN_RELAY_6, OUTPUT);
  pinMode(PIN_RELAY_7, OUTPUT);
  pinMode(PIN_RELAY_8, OUTPUT);

  // Turn all relay pins off
  turnOffAllRelays();
  
  // Configure drill motor
  pinMode(PIN_DRILLMOTOR_PWM, OUTPUT);
  analogWrite(PIN_DRILLMOTOR_PWM, 0);

  // Configure x-axis stepper motor
  pinMode(PIN_XMOTOR_PLS, OUTPUT);
  digitalWrite(PIN_XMOTOR_PLS, LOW);
  pinMode(PIN_XMOTOR_DIR, OUTPUT);
  digitalWrite(PIN_XMOTOR_DIR, HIGH);

  // Configure z-axis motor
  pinMode(PIN_ZMOTOR_ENB_PWM, OUTPUT);
  analogWrite(PIN_ZMOTOR_ENB_PWM, 0);
  pinMode(PIN_ZMOTOR_IN3, OUTPUT);
  digitalWrite(PIN_ZMOTOR_IN3, HIGH);
  pinMode(PIN_ZMOTOR_IN4, OUTPUT);
  digitalWrite(PIN_ZMOTOR_IN4, LOW);

  // Configure melting chamber cap servo motors
  capServo1.attach(PIN_CAPSERVO1_PWM, CAPSERVO_PULSEWIDTH_MIN, CAPSERVO_PULSEWIDTH_MAX);
  capServo2.attach(PIN_CAPSERVO2_PWM, CAPSERVO_PULSEWIDTH_MIN, CAPSERVO_PULSEWIDTH_MAX);

  // Configure load cell
  pinMode(PIN_LOADCELL_DOUT, INPUT);
  pinMode(PIN_LOADCELL_CLK, OUTPUT);
  loadCell.set_scale(LOADCELL_CALIBRATION_FACTOR);
}

/*
  Loop and read commands to execute:

  relaysoff

  positioningon
  positioningoff

  xmovehome,<distanceMM>
  xmoveend,<distanceMM>

  zmovehome,<timeMS>
  zmoveend,<timeMS>
  zstarthome
  zstartend
  zstop

  drillstart,<pwmSpeed>
  drillstop

  capopen
  capclose

  chilleron
  chilleroff

  heateron
  heateroff

  wobtare
  logdata
*/
void loop() {
  // Wait for serial data
  while (Serial.available() == 0) {}

  // Read instruction
  String instruction = Serial.readString();
  Serial.println(instruction);

  // Split instruction into command and optional parameter
  String command = instruction;
  int parameter = 0;
  for (unsigned int i = 0; i < instruction.length(); i++) {
    if (instruction.substring(i, i+1) == ",") {
      command = instruction.substring(0, i);
      parameter = instruction.substring(i+1).toInt();
      break;
    }
  }

  // Execute command

  // relaysoff
  if (command.equalsIgnoreCase("relaysoff")) {
    turnOffAllRelays();
  }

  // positioningon
  else if (command.equalsIgnoreCase("positioningon")) {
    turnOnPositioningMotors();
  }

  // positioningoff
  else if (command.equalsIgnoreCase("positioningoff")) {
    turnOffPositioningMotors();
  }

  // drillstart
  else if (command.equalsIgnoreCase("drillstart")) {
    startDrill(parameter);
  }

  // drillstop
  else if (command.equalsIgnoreCase("drillstop")) {
    stopDrill();
  }

  // capopen
  else if (command.equalsIgnoreCase("capopen")) {
    openCap();
  }

  // capclose
  else if (command.equalsIgnoreCase("capclose")) {
    closeCap();
  }

  // chilleron
  else if (command.equalsIgnoreCase("chilleron")) {
    startChiller();
  }

  // chilleroff
  else if (command.equalsIgnoreCase("chilleroff")) {
    stopChiller();
  }

  // heateron
  else if (command.equalsIgnoreCase("heateron")) {
    startHeater();
  }

  // heateroff
  else if (command.equalsIgnoreCase("heateroff")) {
    stopHeater();
  }

  // xmovehome
  else if (command.equalsIgnoreCase("xmovehome")) {
    DIRECTION direction = BACKWARD;
    moveXAxisByDistance(parameter, direction);
  }

  // xmoveend
  else if (command.equalsIgnoreCase("xmoveend")) {
    DIRECTION direction = FORWARD;
    moveXAxisByDistance(parameter, direction);
  }

  // zmovehome
  else if (command.equalsIgnoreCase("zmovehome")) {
    DIRECTION direction = FORWARD;
    moveZAxisForTime(parameter, direction);
  }

  // zmoveend
  else if (command.equalsIgnoreCase("zmoveend")) {
    DIRECTION direction = BACKWARD;
    moveZAxisForTime(parameter, direction);
  }

  // zstarthome
  else if (command.equalsIgnoreCase("zstarthome")) {
    DIRECTION direction = FORWARD;
    startMovingZAxis(direction);
  }

  // zstartend
  else if (command.equalsIgnoreCase("zstartend")) {
    DIRECTION direction = BACKWARD;
    startMovingZAxis(direction);
  }

  // zstop
  else if (command.equalsIgnoreCase("zstop")) {
    stopMovingZAxis();
  }

  // wobtare
  else if (command.equalsIgnoreCase("wobtare")) {
    loadCell.tare();
  }

  // logdata
  else if (command.equalsIgnoreCase("logdata")) {
    logData();
  }

  // Invalid command
  else {
    Serial.println("Invalid commmand.");
  }

  // Log data after each instruction
  logData();
}

