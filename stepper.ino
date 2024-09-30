#include <ModbusRTU.h>
#include <ESP_FlexyStepper.h>

#define STEP_PIN 41
#define DIR_PIN 40
#define ENABLE_PIN 39
#define ENDSTOP_PIN 42

// Настройка флагов (Coils) Modbus
#define COIL_ENABLE          0  // Включение двигателя
#define COIL_MOVE_ENABLE     1  // Разрешение движения
#define COIL_HOMING          2  // Запуск хоуминга
#define COIL_ARRIVED         3  // Статус завершения движения
#define COIL_IS_HOMING       4  // Выполняется ли хоуминг
#define COIL_ENDSTOP_STATUS  5  // Статус концевого выключателя

// Настройка регистров (Holding Registers) Modbus
#define HREG_POSITION_SETPOINT 0  // Уставка позиции
#define HREG_CURRENT_POSITION  1  // Текущая позиция
#define HREG_MAX_SPEED         2  // Максимальная скорость
#define HREG_ACCELERATION      3  // Ускорение

// Настройка драйвера
#define STEPS_PER_REVOLUTION 800.0 // Microstep 4
#define REVOLUTIONS_PER_MM 1 // Шаг резьбы

// Переменные Modbus
ModbusRTU mb;
ESP_FlexyStepper stepper;

// Переменные состояния
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
  digitalWrite(ENABLE_PIN, LOW); // Активируем двигатель по умолчанию
  pinMode(ENDSTOP_PIN, INPUT_PULLDOWN); // Концевой выключатель

  // Инициализация Modbus
  Serial.begin(9600);
  mb.begin(&Serial);
  mb.slave(1);

  // Настройка флагов (Coils) Modbus
  mb.addCoil(COIL_ENABLE);
  mb.addCoil(COIL_MOVE_ENABLE);
  mb.addCoil(COIL_HOMING);
  mb.addCoil(COIL_ARRIVED);
  mb.addCoil(COIL_IS_HOMING);
  mb.addCoil(COIL_ENDSTOP_STATUS);

  mb.Coil(COIL_ENABLE, coilEnable);            
  mb.Coil(COIL_MOVE_ENABLE, coilMoveEnable);       
  mb.Coil(COIL_HOMING, false);            
  mb.Coil(COIL_ARRIVED, false);            
  mb.Coil(COIL_IS_HOMING, false);         
  mb.Coil(COIL_ENDSTOP_STATUS, false);    

  // Настройка регистров (Holding Registers) Modbus
  mb.addHreg(HREG_POSITION_SETPOINT);
  mb.addHreg(HREG_CURRENT_POSITION);
  mb.addHreg(HREG_MAX_SPEED);
  mb.addHreg(HREG_ACCELERATION);

  mb.Hreg(HREG_POSITION_SETPOINT, 0);     
  mb.Hreg(HREG_CURRENT_POSITION, 0);      
  mb.Hreg(HREG_MAX_SPEED, maxSpeed);         
  mb.Hreg(HREG_ACCELERATION, acceleration);       

  // Инициализация шагового двигателя
  stepper.connectToPins(stepPin, directionPin);
  stepper.setStepsPerRevolution(STEPS_PER_REVOLUTION);
  stepper.setStepsPerMillimeter(STEPS_PER_REVOLUTION * REVOLUTIONS_PER_MM); 

  // Установка скоростей и ускорения
  stepper.setSpeedInStepsPerSecond(maxSpeed);
  stepper.setAccelerationInStepsPerSecondPerSecond(acceleration);
  stepper.moveToHomeInSteps(1, maxSpeed, 50000, limitSwitchPin);
}

void loop() {
  // Обработка Modbus запросов
  mb.task();

  // Обновление переменных из Modbus
  coilEnable = mb.Coil(COIL_ENABLE);
  coilMoveEnable = mb.Coil(COIL_MOVE_ENABLE);
  coilHoming = mb.Coil(COIL_HOMING);
  maxSpeed = mb.Hreg(HREG_MAX_SPEED);
  acceleration = mb.Hreg(HREG_ACCELERATION);
  int16_t positionSetpointTemp = mb.Hreg(HREG_POSITION_SETPOINT);
  if ((positionSetpointTemp <= maxPosition) || (positionSetpointTemp >= minPosition)){
    positionSetpoint = positionSetpointTemp;
  }
  else {
    mb.Hreg(HREG_POSITION_SETPOINT, positionSetpoint);
  }

  // Обновление параметров двигателя
  stepper.setSpeedInStepsPerSecond(maxSpeed);
  stepper.setAccelerationInStepsPerSecondPerSecond(acceleration);

  // Управление двигателем
  if (coilEnable && coilMoveEnable) {
    // Движение к уставке
    stepper.setTargetPositionInMillimeters(positionSetpoint / 100.0);
    stepper.processMovement();
  }

  // Выполнение хоуминга
  if (coilHoming && !stepper.isMovingTowardsHome()) {
    stepper.moveToHomeInSteps(1, maxSpeed, 50000, limitSwitchPin);
    coilHoming = false;
    mb.Coil(COIL_HOMING, false); // Сбрасываем флаг хоуминга
  }

  // Обновление текущей позиции в регистре
  currentPosition = (uint16_t)(stepper.getCurrentPositionInMillimeters() * 100);
  mb.Hreg(HREG_CURRENT_POSITION, currentPosition);

  // Обновление флагов завершения движения
  coilArrived = stepper.motionComplete();
  mb.Coil(COIL_ARRIVED, coilArrived);

  // Обновление статуса концевого выключателя
  coilEndstopStatus = digitalRead(limitSwitchPin) == LOW;
  mb.Coil(COIL_ENDSTOP_STATUS, coilEndstopStatus);

  delayMicroseconds(30);
}
