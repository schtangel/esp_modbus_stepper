#include <ModbusRTU.h>
#include <ESP_FlexyStepper.h>

#define STEP_PIN 41
#define DIR_PIN 40
#define ENABLE_PIN 39
#define ENDSTOP_PIN 42

// Modbus Coil settings
#define COIL_ENABLE          0  // Motor enable
#define COIL_MOVE_ENABLE     1  // Movement enable
#define COIL_HOMING          2  // Start homing
#define COIL_ARRIVED         3  // Movement completion status
#define COIL_IS_HOMING       4  // Homing in progress
#define COIL_ENDSTOP_STATUS  5  // Endstop status

// Modbus Holding Registers settings
#define HREG_POSITION_SETPOINT 0  // Position setpoint
#define HREG_CURRENT_POSITION  1  // Current position
#define HREG_MAX_SPEED         2  // Maximum speed
#define HREG_ACCELERATION      3  // Acceleration

// Driver settings
#define STEPS_PER_REVOLUTION 800.0 // Microstep 4
#define REVOLUTIONS_PER_MM 1 // Thread pitch

// Modbus variables
ModbusRTU mb;
ESP_FlexyStepper stepper;

// State variables
bool coilEnable = true;
bool coilMoveEnable = true;
bool coilHoming = false;
bool coilArrived = true;
bool coilEndstopStatus = false;

const int16_t minPosition = 0; // zero
const int16_t maxPosition = 3400; // 34mm
int16_t positionSetpoint = 0;
int16_t currentPosition = 0;
uint16_t maxSpeed = 4000;
uint16_t acceleration = 1000;

// IO pin assignments
const int stepPin = STEP_PIN;
const int directionPin = DIR_PIN;
const int limitSwitchPin = ENDSTOP_PIN;

void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Enable motor by default
  pinMode(ENDSTOP_PIN, INPUT_PULLDOWN); // Endstop switch

  // Modbus initialization
  Serial.begin(9600);
  mb.begin(&Serial);
  mb.slave(1);

  // Modbus Coil settings
  mb.addCoil(COIL_ENABLE);
  mb.addCoil(COIL_MOVE_ENABLE);
  mb.addCoil(COIL_HOMING);
  mb.addCoil(COIL_ARRIVED);
  mb.addCoil(COIL_IS_HOMING);
  mb.addCoil(COIL_ENDSTOP_STATUS);

  mb.Coil(COIL_ENABLE, coilEnable);
  mb.Coil(COIL_MOVE_ENABLE, coilMoveEnable);
  mb.Coil(COIL_HOMING, coilHoming);
  mb.Coil(COIL_ARRIVED, coilArrived);
  mb.Coil(COIL_IS_HOMING, stepper.isMovingTowardsHome());
  mb.Coil(COIL_ENDSTOP_STATUS, coilEndstopStatus);

  // Modbus register settings
  mb.addHreg(HREG_POSITION_SETPOINT, positionSetpoint);
  mb.addHreg(HREG_CURRENT_POSITION, currentPosition);
  mb.addHreg(HREG_MAX_SPEED, maxSpeed);
  mb.addHreg(HREG_ACCELERATION, acceleration);

  // Initialize stepper
  stepper.connectToPins(stepPin, directionPin);
  stepper.setStepsPerRevolution(STEPS_PER_REVOLUTION);
  stepper.setStepsPerMillimeter(STEPS_PER_REVOLUTION * REVOLUTIONS_PER_MM);

  // Set speeds and acceleration
  stepper.setSpeedInStepsPerSecond(maxSpeed);
  stepper.setAccelerationInStepsPerSecondPerSecond(acceleration);
  stepper.moveToHomeInSteps(1, maxSpeed, 50000, limitSwitchPin);
}

void loop() {
  // Process Modbus requests
  mb.task();

  // Update variables from Modbus
  coilEnable = mb.Coil(COIL_ENABLE);
  coilMoveEnable = mb.Coil(COIL_MOVE_ENABLE);
  coilHoming = mb.Coil(COIL_HOMING);
  maxSpeed = mb.Hreg(HREG_MAX_SPEED);
  acceleration = mb.Hreg(HREG_ACCELERATION);
  int16_t positionSetpointTemp = mb.Hreg(HREG_POSITION_SETPOINT);
  if ((positionSetpointTemp <= maxPosition) && (positionSetpointTemp >= minPosition)){
    positionSetpoint = positionSetpointTemp;
  }
  else {
    mb.Hreg(HREG_POSITION_SETPOINT, positionSetpoint);
  }

  // Update motor parameters
  stepper.setSpeedInStepsPerSecond(maxSpeed);
  stepper.setAccelerationInStepsPerSecondPerSecond(acceleration);

  // Motor control
  if (coilEnable && coilMoveEnable) {
    // Move to setpoint
    stepper.setTargetPositionInMillimeters(positionSetpoint / 100.0);
    stepper.processMovement();
  }

  // Perform homing
  if (coilHoming && !stepper.isMovingTowardsHome()) {
    stepper.moveToHomeInSteps(1, maxSpeed, 50000, limitSwitchPin);
    coilHoming = false;
    mb.Coil(COIL_HOMING, false); // Reset homing flag
  }

  // Update current position in register
  currentPosition = (uint16_t)(stepper.getCurrentPositionInMillimeters() * 100);
  mb.Hreg(HREG_CURRENT_POSITION, currentPosition);

  // Update movement completion flags
  coilArrived = stepper.motionComplete();
  mb.Coil(COIL_ARRIVED, coilArrived);

  // Update endstop status
  coilEndstopStatus = digitalRead(limitSwitchPin) == LOW;
  mb.Coil(COIL_ENDSTOP_STATUS, coilEndstopStatus);

  delayMicroseconds(30);
}
