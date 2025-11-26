#include "Stability.h"

#include <Arduino.h>
#include <MPU9250_asukiaaa.h>
#include <SimpleFOC.h>
#include <Wire.h>

namespace Stability {

// ====================== HARDWARE ======================

// Motor BLDC (7 pares de polos)
BLDCMotor motor = BLDCMotor(7);

// Driver L6234 (3PWM + EN)
BLDCDriver3PWM driver = BLDCDriver3PWM(25, 26, 27, 14);

// Sensor magnético AS5600 (I2C)
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// IMU MPU9250
MPU9250_asukiaaa imu;

// I2C ESP32
const int I2C_SDA = 21;
const int I2C_SCL = 22;

// ====================== PARÂMETROS ======================

const float VOLTAGE_SUPPLY = 8.0f; // tensão da fonte [V]
const float VOLTAGE_LIMIT = 7.0f;  // limite de tensão p/ motor [V]

// limites de velocidade da roda
const float MAX_WHEEL_VEL = 80.0f; // [rad/s] máx
const float MIN_WHEEL_VEL = 3.0f;  // [rad/s] mín p/ vencer inércia

// ganho: quanto da velocidade do satélite vira velocidade da roda
const float GYRO_TO_VEL_GAIN = 5.0f; // ajuste fino depois

// deadband: abaixo disso consideramos "parado"
const float GYRO_DEADBAND_RADPS = 0.02f; // ~1.1 deg/s

// histerese: só mudamos a direção se a velocidade passar desse valor
const float DIR_THRESHOLD_RADPS = 0.3f; // ~17 deg/s

// filtro exponencial para o gyro (0.1 = mais suave, 0.5 = responde mais rápido)
const float GYRO_FILTER_ALPHA = 0.2f;

// correção de sinal global (se tudo inverter, troca pra -1)
const int SIGN_CORRECTION = 1;

// passo máximo de mudança de velocidade por iteração
const float MAX_WHEEL_VEL_STEP =
    0.05f; // [rad/s] por passo (~5 rad/s por segundo se CONTROL_PERIOD_MS=10)

// período do loop de controle (em ms)
const unsigned long CONTROL_PERIOD_MS = 10; // 100 Hz

// ====================== VARIÁVEIS DE ESTADO ======================

// offset do gyro Z (rad/s), calculado na calibração
float gyroOffsetZ = 0.0f;

// filtro do gyro
float satRateFiltered = 0.0f;

// última direção "confiável" (baseada no gyro)
int lastDirection = 0; // -1, 0, +1

// velocidade atual que estamos mandando pro motor
float wheel_current_velocity = 0.0f;

// tempo do último passo de controle
unsigned long lastControlTime = 0;

// flags internas
const int GYRO_CALIB_SAMPLES = 500;
static bool hardwareInitialized = false;
static bool gyroCalibrated = false;

// ====================== FUNÇÕES INTERNAS ======================

// Calibração do gyro Z
static void calibrateGyroZ() {
  Serial.println(
      F("[Stability] Calibrando gyro Z, mantenha o satelite PARADO..."));
  delay(500);

  float sum = 0.0f;
  for (int i = 0; i < GYRO_CALIB_SAMPLES; ++i) {
    imu.gyroUpdate();
    sum += imu.gyroZ(); // rad/s
    delay(5);
  }
  gyroOffsetZ = sum / (float)GYRO_CALIB_SAMPLES;

  Serial.print(F("[Stability] Offset gyro Z = "));
  Serial.print(gyroOffsetZ, 6);
  Serial.println(F(" rad/s"));
}

// Leitura do gyro Z com filtro exponencial
// Retorna velocidade angular do satélite em rad/s (eixo Z), filtrada
static float readSatelliteRateRadPerSec() {
  imu.gyroUpdate();

  // leitura bruta compensada do offset
  float gz = imu.gyroZ() - gyroOffsetZ; // [rad/s]

  // filtro exponencial: satRateFiltered = satRateFiltered + α*(gz -
  // satRateFiltered)
  satRateFiltered += GYRO_FILTER_ALPHA * (gz - satRateFiltered);

  return satRateFiltered;
}

// Calcula a velocidade alvo da roda a partir da velocidade do satélite
// Aplica deadband, histerese de direção, ganho e velocidade mínima
static float computeWheelTargetVelocity(float satRateRadPerSec) {
  float absRate = fabsf(satRateRadPerSec);

  // Zona morta: satélite praticamente parado, roda parada
  if (absRate < GYRO_DEADBAND_RADPS) {
    return 0.0f;
  }

  // Direção instantânea atual
  int signNow = (satRateRadPerSec > 0.0f) ? 1 : -1;

  // Histerese de direção: só atualiza direção se passar do limiar
  if (absRate > DIR_THRESHOLD_RADPS) {
    lastDirection = signNow;
  }

  // Se ainda não temos direção confiável, não mexe na roda
  if (lastDirection == 0) {
    return 0.0f;
  }

  // Usa SEMPRE o lastDirection para definir o sinal,
  // e o módulo da velocidade (absRate) para a amplitude.
  float target = GYRO_TO_VEL_GAIN * absRate * (float)lastDirection *
                 (float)SIGN_CORRECTION;

  // Garante velocidade mínima pra vencer a inércia
  if (fabsf(target) < MIN_WHEEL_VEL) {
    target = (lastDirection >= 0 ? MIN_WHEEL_VEL : -MIN_WHEEL_VEL);
  }

  // Saturação
  if (target > MAX_WHEEL_VEL)
    target = MAX_WHEEL_VEL;
  if (target < -MAX_WHEEL_VEL)
    target = -MAX_WHEEL_VEL;

  return target;
}

// Aplica rampa (slew rate limit) na velocidade da roda
static float applyVelocityRamp(float current, float target) {
  float diff = target - current;

  if (diff > MAX_WHEEL_VEL_STEP)
    diff = MAX_WHEEL_VEL_STEP;
  if (diff < -MAX_WHEEL_VEL_STEP)
    diff = -MAX_WHEEL_VEL_STEP;

  return current + diff;
}

// ====================== API PÚBLICA ======================

void begin() {
  // Inicializa hardware somente uma vez
  if (!hardwareInitialized) {
    // I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    // Wire.setClock(400000); // se quiser I2C rápido

    // MPU
    imu.setWire(&Wire);
    imu.beginGyro();

    // AS5600
    sensor.init();
    motor.linkSensor(&sensor);

    // Driver
    driver.voltage_power_supply = VOLTAGE_SUPPLY;
    driver.voltage_limit = VOLTAGE_LIMIT;
    if (!driver.init()) {
      Serial.println(F("[Stability] ERRO: driver.init() falhou!"));
      while (1) {
        // trava aqui para debug (você pode trocar por outro tratamento)
      }
    }
    motor.linkDriver(&driver);

    // Motor em open loop de velocidade
    motor.voltage_limit = VOLTAGE_LIMIT;
    motor.controller = MotionControlType::velocity_openloop;

    if (!motor.init()) {
      Serial.println(F("[Stability] ERRO: motor.init() falhou!"));
      while (1) {
        // trava aqui para debug
      }
    }

    hardwareInitialized = true;
  }

  // Calibração do gyro Z apenas uma vez (idealmente com satélite parado)
  if (!gyroCalibrated) {
    calibrateGyroZ();
    gyroCalibrated = true;
  }

  // Estado inicial do controle
  wheel_current_velocity = 0.0f;
  lastControlTime = millis();
  lastDirection = 0;
  satRateFiltered = 0.0f;

  Serial.println(F("[Stability] begin() concluido"));
}

void update() {
  unsigned long now = millis();

  // Loop de controle em ~100 Hz
  if (now - lastControlTime >= CONTROL_PERIOD_MS) {
    lastControlTime = now;

    // Lê gyro (já com offset + filtro)
    float satRateRadPerSec = readSatelliteRateRadPerSec();

    // Calcula velocidade alvo da roda
    float targetWheelVelocity = computeWheelTargetVelocity(satRateRadPerSec);

    // Aplica rampa pra não dar tranco
    wheel_current_velocity =
        applyVelocityRamp(wheel_current_velocity, targetWheelVelocity);

    // Debug opcional
    /*
    Serial.print(F("[Stability] satRate(rad/s)="));
    Serial.print(satRateRadPerSec, 4);
    Serial.print(F("  target="));
    Serial.print(targetWheelVelocity, 4);
    Serial.print(F("  ramped="));
    Serial.println(wheel_current_velocity, 4);
    */
  }

  // Comanda o motor em toda iteração
  motor.move(wheel_current_velocity);
}

// ========== GETTERS (para debug/telemetria) ==========

float getSatRateRadPerSec() { return satRateFiltered; }

float getWheelVelocity() { return wheel_current_velocity; }

float getGyroOffsetZ() { return gyroOffsetZ; }

} // namespace Stability
