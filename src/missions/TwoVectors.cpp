#include "TwoVectors.h"

#include <Arduino.h>
#include <MPU9250_asukiaaa.h>
#include <SimpleFOC.h>
#include <Wire.h>

namespace TwoVectors {

// ====================== HARDWARE ======================

// Roda de reação (BLDC + L6234 + AS5600)
BLDCMotor motor(7);                    // 7 pares de polos
BLDCDriver3PWM driver(25, 26, 27, 14); // IN1, IN2, IN3, EN
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// MPU9250
MPU9250_asukiaaa imu;

// I2C ESP32
const int I2C_SDA = 21;
const int I2C_SCL = 22;

// ====================== PARÂMETROS DO MOTOR ======================

const float SUPPLY_VOLTAGE = 8.0f;
const float MOTOR_VOLTAGE_LIMIT = 7.0f;

// ====================== CONTROLE DE ATITUDE (YAW) ======================

// estado do satélite
float yawAngleRad = 0.0f;  // ângulo [rad], 0 = orientação no boot
float yawRateRad_s = 0.0f; // taxa [rad/s]

// setpoint
float targetAngleDeg = 0.0f;
float targetAngleRad = 0.0f;

bool hasSetpoint = false;

// bias (offset) do giroscópio Z [deg/s] calculado na calibração
float gyroZBiasDeg_s = 0.0f;

// ajustes de sinal:
const float GYRO_SIGN = +1.0f;
const float DIR_SIGN = +1.0f;

// filtro exponencial no gyro
const float GYRO_FILTER_ALPHA = 0.2f; // aumenta pra ficar mais rápido

// zona morta de posição
const float ANGLE_DEADZONE_DEG = 3.0f;
const float ANGLE_DEADZONE_RAD = ANGLE_DEADZONE_DEG * PI / 180.0f;

// limite máximo de erro para o controlador
const float ANGLE_MAX_ERROR_DEG = 120.0f;
const float ANGLE_MAX_ERROR_RAD = ANGLE_MAX_ERROR_DEG * PI / 180.0f;

// ganhos do controlador de posição
float KP_ANGLE = 5.0f;
float KD_RATE = 0.8f;

// limites da roda
const float MAX_WHEEL_SPEED = 80.0f; // [rad/s]
const float MIN_WHEEL_SPEED = 3.0f;  // [rad/s]

// ====================== RAMPA DE VELOCIDADE ======================

float targetVelocityCmd = 0.0f; // comando “ideal” de velocidade
float rampedVelocity = 0.0f;    // comando suavizado que vai pro motor

const float VEL_STEP = 2.0f;               // [rad/s] por passo
const unsigned long RAMP_INTERVAL_MS = 50; // intervalo entre passos

unsigned long lastRampTime = 0;
unsigned long lastAttUpdateMicros = 0;

// controle de inicialização
static bool hardwareInitialized = false;

// ====================== FUNÇÕES INTERNAS ======================

static float wrapPi(float a) {
  while (a > PI)
    a -= 2.0f * PI;
  while (a < -PI)
    a += 2.0f * PI;
  return a;
}

static void atualizaRampa() {
  unsigned long now = millis();
  if (now - lastRampTime < RAMP_INTERVAL_MS)
    return;
  lastRampTime = now;

  float diff = targetVelocityCmd - rampedVelocity;

  if (fabsf(diff) <= VEL_STEP) {
    rampedVelocity = targetVelocityCmd;
  } else {
    rampedVelocity += (diff > 0.0f) ? VEL_STEP : -VEL_STEP;
  }
}

// Atualiza yawAngleRad e yawRateRad_s com filtro + integração
static void atualizarAtitude() {
  unsigned long nowMicros = micros();
  float dt = 0.0f;
  if (lastAttUpdateMicros != 0) {
    dt = (nowMicros - lastAttUpdateMicros) * 1e-6f;
  }
  lastAttUpdateMicros = nowMicros;

  if (imu.gyroUpdate() == 0) {
    // leitura do gyro em deg/s
    float gz_deg_s = imu.gyroZ() - gyroZBiasDeg_s;
    gz_deg_s *= GYRO_SIGN;

    float gz_rad_s = gz_deg_s * PI / 180.0f;

    // filtro exponencial
    yawRateRad_s += GYRO_FILTER_ALPHA * (gz_rad_s - yawRateRad_s);

    // integra a taxa para obter o ângulo
    if (dt > 0.0f && dt < 0.05f) { // evita spikes absurdos
      yawAngleRad += yawRateRad_s * dt;
      yawAngleRad = wrapPi(yawAngleRad);
    }
  }
}

// Controle PD de atitude -> targetVelocityCmd
static void controleDeAtitude() {
  if (!hasSetpoint) {
    targetVelocityCmd = 0.0f;
    return;
  }

  // erro de ângulo = alvo - atual
  float errorAngle = wrapPi(targetAngleRad - yawAngleRad);
  float limitedError =
      constrain(errorAngle, -ANGLE_MAX_ERROR_RAD, ANGLE_MAX_ERROR_RAD);

  float absError = fabsf(limitedError);

  // ganho proporcional reduzido perto do alvo
  float Kp_eff = KP_ANGLE;
  if (absError < ANGLE_DEADZONE_RAD) {
    Kp_eff *= 0.3f; // só 30% do ganho dentro da deadzone
  }

  // PD na planta: u = Kp*erro - Kd*yawRate
  float u = Kp_eff * limitedError - KD_RATE * yawRateRad_s;

  // converte para velocidade da roda
  float wheelVel = DIR_SIGN * u;

  // saturação
  if (wheelVel > MAX_WHEEL_SPEED)
    wheelVel = MAX_WHEEL_SPEED;
  if (wheelVel < -MAX_WHEEL_SPEED)
    wheelVel = -MAX_WHEEL_SPEED;

  // longe do alvo, garante velocidade mínima pra vencer atrito
  if (absError > ANGLE_DEADZONE_RAD * 1.5f &&
      fabsf(wheelVel) < MIN_WHEEL_SPEED) {
    wheelVel = (wheelVel >= 0.0f) ? MIN_WHEEL_SPEED : -MIN_WHEEL_SPEED;
  }

  targetVelocityCmd = wheelVel;
}

// Calibração do gyroZ (offset em deg/s)
static void calibrarGyroZ() {
  const int N = 500;
  float soma = 0.0f;

  // Serial.println(
  //     F("[TwoVectors] Calibrando gyro Z, mantenha o satelite parado..."));

  for (int i = 0; i < N; i++) {
    if (imu.gyroUpdate() == 0) {
      soma += imu.gyroZ();
    }
    delay(5);
  }

  gyroZBiasDeg_s = soma / (float)N;

  // Serial.print(F("[TwoVectors] Bias gyroZ (deg/s): "));
  // Serial.println(gyroZBiasDeg_s, 6);
}

// ====================== API PÚBLICA ======================

void begin() {
  if (!hardwareInitialized) {
    // Debug SimpleFOC (opcional)
    SimpleFOCDebug::enable(&Serial);

    // I2C
    Wire.begin(I2C_SDA, I2C_SCL);

    // IMU
    imu.setWire(&Wire);
    imu.beginAccel();
    imu.beginGyro();

    // calibração do gyro e zero de ângulo
    calibrarGyroZ();
    yawAngleRad = 0.0f;
    yawRateRad_s = 0.0f;

    // Encoder da roda
    sensor.init();
    motor.linkSensor(&sensor);

    // Driver BLDC
    driver.voltage_power_supply = SUPPLY_VOLTAGE;
    driver.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    // if (!driver.init()) {
    //   Serial.println(F("[TwoVectors] ERRO: driver.init() falhou!"));
    //   while (1) {
    //     delay(1000);
    //   }
    // }
    motor.linkDriver(&driver);

    // Motor em controle de velocidade FOC
    motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    motor.controller = MotionControlType::velocity;

    motor.PID_velocity.P = 0.5f;
    motor.PID_velocity.I = 10.0f;
    motor.PID_velocity.D = 0.0f;
    motor.PID_velocity.output_ramp = 1000.0f;
    motor.PID_velocity.limit = MOTOR_VOLTAGE_LIMIT;

    motor.LPF_velocity.Tf = 0.03f;

    // if (!motor.init()) {
    //   Serial.println(F("[TwoVectors] ERRO: motor.init() falhou!"));
    //   while (1) {
    //     delay(1000);
    //   }
    // }

    // FOC
    motor.initFOC();

    // Estado inicial (somente NA PRIMEIRA VEZ)
    lastRampTime = millis();
    lastAttUpdateMicros = micros();
    hasSetpoint = false;
    targetAngleDeg = 0.0f;
    targetAngleRad = 0.0f;

    hardwareInitialized = true;
    // Serial.println(F("[TwoVectors] Hardware inicializado"));
  } else {
    // Chamadas futuras de begin(): só ressincronizamos o tempo de rampa
    lastRampTime = millis();
    lastAttUpdateMicros = micros();

    // Serial.println(F("[TwoVectors] begin() chamado (hardware já inicializado,
    // "
    //                  "setpoint preservado)."));
  }
}

void update() {
  // FOC interno
  motor.loopFOC();

  // atualiza atitude (gyro -> yaw e yawRate)
  atualizarAtitude();

  // controlador de atitude -> targetVelocityCmd
  controleDeAtitude();

  // rampa de velocidade
  atualizaRampa();

  // comanda a roda
  motor.move(rampedVelocity);

  // Debug opcional
  /*
  static unsigned long lastPrint = 0;
  unsigned long now = millis();
  if (now - lastPrint > 200) {
    lastPrint = now;

    Serial.print(F("[TwoVectors] Yaw_deg="));
    Serial.print(yawAngleRad * 180.0f / PI);
    Serial.print(F("  YawRate_deg_s="));
    Serial.print(yawRateRad_s * 180.0f / PI);
    Serial.print(F("  Target_deg="));
    Serial.print(targetAngleDeg);
    Serial.print(F("  targetVel="));
    Serial.print(targetVelocityCmd);
    Serial.print(F("  rampedVel="));
    Serial.println(rampedVelocity);
  }
  */
}

// ----------- API de comando -----------

void setTargetAngleDeg(float angleDeg) {
  targetAngleDeg = angleDeg;

  // normaliza para [-180, 180)
  while (targetAngleDeg > 180.0f)
    targetAngleDeg -= 360.0f;
  while (targetAngleDeg <= -180.0f)
    targetAngleDeg += 360.0f;

  targetAngleRad = targetAngleDeg * PI / 180.0f;
  hasSetpoint = true;

  // Serial.print(F("[TwoVectors] Novo setpoint (deg): "));
  // Serial.println(targetAngleDeg);
}

void clearSetpoint() {
  hasSetpoint = false;
  targetAngleDeg = 0.0f;
  targetAngleRad = 0.0f;
  targetVelocityCmd = 0.0f;
  // não zerei rampedVelocity pra deixar a rampa trazer até zero suavemente
}

// ----------- Getters -----------

float getYawAngleRad() { return yawAngleRad; }

float getYawRateRadPerSec() { return yawRateRad_s; }

float getTargetAngleDeg() { return targetAngleDeg; }

float getTargetAngleRad() { return targetAngleRad; }

float getWheelTargetVelocity() { return targetVelocityCmd; }

float getWheelRampedVelocity() { return rampedVelocity; }

float getGyroBiasDegPerSec() { return gyroZBiasDeg_s; }

} // namespace TwoVectors
