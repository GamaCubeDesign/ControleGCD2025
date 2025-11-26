#include "SunPointing.h"

#include <Arduino.h>
#include <MPU9250_asukiaaa.h>
#include <SimpleFOC.h>
#include <Wire.h>

namespace SunPointing {

// ====================== HARDWARE ======================

// RODA + BLDC + AS5600
BLDCMotor motor(7);                    // 7 pares de polos
BLDCDriver3PWM driver(25, 26, 27, 14); // IN1, IN2, IN3, EN
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// MPU9250
MPU9250_asukiaaa imu;

// I2C
const int I2C_SDA = 21;
const int I2C_SCL = 22;

// Painéis solares (mesma convenção do seu código)
// +X = face "principal" (alvo do Sun Pointing) -> painelPX
// +Y = 90° CCW em torno de +Z                  -> painelPY
// -X = oposto de +X                            -> painelMX
// -Y = oposto de +Y                            -> painelMY
const int painelPX = 32; // +X (GPIO32 - ADC1_CH4)
const int painelPY = 33; // +Y (GPIO33 - ADC1_CH5)
const int painelMX = 34; // -X (GPIO34 - ADC1_CH6)
const int painelMY = 35; // -Y (GPIO35 - ADC1_CH7)

// ====================== CALIBRAÇÃO PAINÉIS ======================

// ESP32 ADC: 12 bits -> [0..4095]
// Considerando ~3.5 V de full scale efetivo
const float ADC_REF_VOLT = 3.5f;
const int ADC_MAX = 4095;
const float STEP = ADC_REF_VOLT / (float)ADC_MAX;

// Tensões máximas medidas em calibração (luz a 0° para cada face)
float v0max = 2.64f; // +X
float v1max = 2.70f; // +Y
float v2max = 2.56f; // -X
float v3max = 2.60f; // -Y

// Limite mínimo para considerar "há Sol" (soma das quatro faces)
const float MIN_LIGHT_LEVEL = 0.3f; // ajuste em teste

// ====================== PARÂMETROS DO MOTOR ======================

const float SUPPLY_VOLTAGE = 8.0f;      // sua fonte
const float MOTOR_VOLTAGE_LIMIT = 7.0f; // limite de tensão pra roda

// ====================== PARÂMETROS SUN POINTING ======================

// Referência: queremos Sol na face +X -> ângulo alvo = 0 rad
const float TARGET_ANGLE_RAD = 0.0f;

// Zona morta perto do alvo: dentro deste erro angular não giramos mais
const float ANGLE_DEADZONE_DEG = 5.0f; // ±5°
const float ANGLE_DEADZONE_RAD = ANGLE_DEADZONE_DEG * PI / 180.0f;

// Erro máximo que faz sentido para controle proporcional (saturação)
const float ANGLE_MAX_ERROR_DEG = 90.0f;
const float ANGLE_MAX_ERROR_RAD = ANGLE_MAX_ERROR_DEG * PI / 180.0f;

// Ganho proporcional: transforma erro de ângulo [rad] em velocidade da roda
// [rad/s]
float KP_ANGLE = 20.0f; // ajuste em bancada

// Velocidades da roda
const float MAX_WHEEL_SPEED = 80.0f; // limite de segurança [rad/s]
const float MIN_WHEEL_SPEED = 3.0f;  // mínimo pra vencer atrito [rad/s]

// Rampa de velocidade
const float VEL_STEP = 2.0f;               // passo da rampa [rad/s]
const unsigned long RAMP_INTERVAL_MS = 50; // tempo entre passos da rampa [ms]

// ====================== VARIÁVEIS DE ESTADO ======================

float targetVelocityCmd = 0.0f; // saída do controlador (antes da rampa)
float rampedVelocity = 0.0f;    // valor usado em motor.move()
unsigned long lastRampTime = 0;

// Para debug / telemetria
float lastSunAngleRad = 0.0f; // [0, 2π)
float lastGyroZRad_s = 0.0f;

// Controle de inicialização de hardware
static bool hardwareInitialized = false;

// ====================== FUNÇÕES INTERNAS ======================

// Normaliza ângulo para o intervalo [-π, π]
static float wrapPi(float a) {
  while (a > PI)
    a -= 2.0f * PI;
  while (a < -PI)
    a += 2.0f * PI;
  return a;
}

// Leitura dos painéis solares e cálculo do ângulo da luz no referencial do
// satélite Retorno: ângulo em rad [0, 2π]
static float calcularVetorSolarRad(bool *sunOk) {
  // Leitura dos ADCs -> tensão em cada face
  float vPX = analogRead(painelPX) * STEP; // +X
  float vPY = analogRead(painelPY) * STEP; // +Y
  float vMX = analogRead(painelMX) * STEP; // -X
  float vMY = analogRead(painelMY) * STEP; // -Y

  float total = vPX + vPY + vMX + vMY;
  if (sunOk) {
    *sunOk = (total > MIN_LIGHT_LEVEL);
  }

  // delta_x > 0 => mais luz no +X do que no -X
  // delta_y > 0 => mais luz no +Y do que no -Y
  float delta_x = (vPX / v0max) - (vMX / v2max);
  float delta_y = (vPY / v1max) - (vMY / v3max);

  // Limita a [-1, 1] para evitar problemas numéricos
  delta_x = constrain(delta_x, -1.0f, 1.0f);
  delta_y = constrain(delta_y, -1.0f, 1.0f);

  // Ângulo da luz no referencial do satélite:
  // 0 rad  -> Sol na face +X
  // pi/2   -> Sol na face +Y
  // pi     -> Sol na face -X
  // -pi/2  -> Sol na face -Y
  float angulo = atan2f(delta_y, delta_x); // [-π, π]

  if (angulo < 0) {
    angulo += 2.0f * PI; // [0, 2π)
  }

  return angulo;
}

static void atualizaRampa() {
  unsigned long now = millis();
  if (now - lastRampTime < RAMP_INTERVAL_MS) {
    return;
  }
  lastRampTime = now;

  float diff = targetVelocityCmd - rampedVelocity;

  if (fabsf(diff) <= VEL_STEP) {
    // já estamos próximos do alvo
    rampedVelocity = targetVelocityCmd;
  } else {
    // sobe ou desce em degraus de VEL_STEP
    rampedVelocity += (diff > 0.0f) ? VEL_STEP : -VEL_STEP;
  }
}

static float lerGiroZ_rad_s() {
  float gZ_deg_s = 0.0f;

  if (imu.gyroUpdate() == 0) {
    gZ_deg_s = imu.gyroZ(); // graus/s
  }

  // Converte para rad/s
  return gZ_deg_s * (PI / 180.0f);
}

// ====================== API PÚBLICA ======================

void begin() {
  if (!hardwareInitialized) {
    // Debug da SimpleFOC (opcional)
    SimpleFOCDebug::enable(&Serial);

    // I2C
    Wire.begin(I2C_SDA, I2C_SCL);

    // IMU
    imu.setWire(&Wire);
    imu.beginAccel();
    imu.beginGyro();

    // AS5600
    sensor.init(); // usa o Wire
    motor.linkSensor(&sensor);

    // Driver e BLDC
    driver.voltage_power_supply = SUPPLY_VOLTAGE;
    driver.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    if (!driver.init()) {
      Serial.println(F("[SunPointing] ERRO: driver L6234 nao inicializou!"));
      while (1) {
        delay(1000);
      }
    }
    motor.linkDriver(&driver);

    // Configura motor para controle de VELOCIDADE
    motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    motor.controller = MotionControlType::velocity;

    // PID
    motor.PID_velocity.P = 0.2f;
    motor.PID_velocity.I = 5.0f;
    motor.PID_velocity.D = 0.0f;
    motor.PID_velocity.output_ramp = 1000.0f; // [rad/s^2] máx variação
    motor.PID_velocity.limit = MOTOR_VOLTAGE_LIMIT;

    motor.LPF_velocity.Tf = 0.01f; // filtro passa-baixa para medição de vel.

    if (!motor.init()) {
      Serial.println(F("[SunPointing] ERRO: motor nao inicializou!"));
      while (1) {
        delay(1000);
      }
    }

    // Inicializa FOC (alinhamento do sensor + calibração)
    motor.initFOC();

    hardwareInitialized = true;
    Serial.println(F("[SunPointing] Hardware inicializado"));
  }

  // Estado inicial de controle
  targetVelocityCmd = 0.0f;
  rampedVelocity = 0.0f;
  lastRampTime = millis();
  lastSunAngleRad = 0.0f;
  lastGyroZRad_s = 0.0f;

  Serial.println(F("[SunPointing] begin() concluido. Sistema pronto."));
}

void update() {
  // Atualiza FOC (obrigatório para SimpleFOC)
  motor.loopFOC();

  // Lê sensores solares e calcula ângulo da luz
  bool sunOk = false;
  float sunAngleRad = calcularVetorSolarRad(&sunOk); // [0, 2π)
  lastSunAngleRad = sunAngleRad;

  // Lê giro Z (para debug / futura derivativa)
  float gyroZ_rad_s = lerGiroZ_rad_s();
  lastGyroZRad_s = gyroZ_rad_s;

  // Lógica básica de controle de atitude
  if (!sunOk) {
    // Sem Sol suficiente: não faz sentido apontar.
    // -> Para roda suavemente (targetVelocityCmd = 0).
    targetVelocityCmd = 0.0f;
  } else {
    // Converte ângulo da luz para erro em relação à face +X (0 rad)
    float errorAngle = wrapPi(sunAngleRad - TARGET_ANGLE_RAD); // [-π, π]

    // Se já estamos alinhados (erro pequeno), para a roda para manter
    // apontamento
    if (fabsf(errorAngle) < ANGLE_DEADZONE_RAD) {
      // Aqui você poderia usar gyroZ_rad_s para um controle mais fino,
      // mas, para simplificar, só pedimos velocidade zero.
      targetVelocityCmd = 0.0f;
    } else {
      // Controle PROPORCIONAL simples:
      //
      // - errorAngle > 0  -> Sol está "para +Y" em relação à face +X
      //   => queremos girar o satélite no sentido CCW (positivo em +Z).
      //   Pela conservação do momento angular: a roda deve acelerar no sentido
      //   oposto (CW). Assumindo que velocidade positiva do motor é CCW,
      //   isso significa que a roda deve girar com velocidade NEGATIVA.

      // Limita o módulo do erro para não explodir a resposta
      float limitedError =
          constrain(errorAngle, -ANGLE_MAX_ERROR_RAD, ANGLE_MAX_ERROR_RAD);

      float wheelVel = -KP_ANGLE * limitedError; // [rad/s]

      // Saturação de velocidade máxima
      if (wheelVel > MAX_WHEEL_SPEED)
        wheelVel = MAX_WHEEL_SPEED;
      if (wheelVel < -MAX_WHEEL_SPEED)
        wheelVel = -MAX_WHEEL_SPEED;

      // Garante módulo mínimo (para superar atrito) quando houver erro
      // relevante
      if (fabsf(wheelVel) < MIN_WHEEL_SPEED) {
        wheelVel = (wheelVel >= 0.0f) ? MIN_WHEEL_SPEED : -MIN_WHEEL_SPEED;
      }

      targetVelocityCmd = wheelVel;
    }
  }

  // Aplica rampa de velocidade para evitar trancos
  atualizaRampa();

  // Comanda a roda de reação
  motor.move(rampedVelocity);

  // Debug opcional
  /*
  Serial.print(F("[SunPointing] sunAngle="));
  Serial.print(lastSunAngleRad, 3);
  Serial.print(F("  gyroZ="));
  Serial.print(lastGyroZRad_s, 3);
  Serial.print(F("  targetVel="));
  Serial.print(targetVelocityCmd, 3);
  Serial.print(F("  rampedVel="));
  Serial.println(rampedVelocity, 3);
  */
}

// ====================== GETTERS (debug / telemetria) ======================

float getSunAngleRad() { return lastSunAngleRad; }

float getGyroZRadPerSec() { return lastGyroZRad_s; }

float getTargetVelocity() { return targetVelocityCmd; }

float getRampedVelocity() { return rampedVelocity; }

} // namespace SunPointing
