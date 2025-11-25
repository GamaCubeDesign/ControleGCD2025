#pragma once

namespace TwoVectors {

// Inicializa a missão de controle de atitude por ângulo alvo (yaw).
// Configura IMU, motor, driver, encoder, PID, etc.
void begin();

// Atualiza o controle de atitude.
// Deve ser chamada periodicamente pelo MissionController.
void update();

// ----------- API de comando (para TT&C / WiFi / Serial) -----------

// Define o ângulo alvo em graus (referencial relativo ao boot ou a um zero
// interno). Exemplo: setTargetAngleDeg(90.0f) -> girar o satélite para +90° de
// yaw.
void setTargetAngleDeg(float angleDeg);

// Limpa o setpoint: missão TwoVectors entra em "idle" (velocidade alvo = 0).
void clearSetpoint();

// ----------- Getters opcionais (debug / telemetria) -----------

// Ângulo atual estimado do satélite (yaw) em radianos, no intervalo [-π, π]
float getYawAngleRad();

// Taxa de rotação em yaw [rad/s]
float getYawRateRadPerSec();

// Setpoint atual em graus / radianos
float getTargetAngleDeg();
float getTargetAngleRad();

// Comando de velocidade alvo da roda [rad/s] (antes da rampa)
float getWheelTargetVelocity();

// Velocidade efetivamente aplicada na roda [rad/s] (depois da rampa)
float getWheelRampedVelocity();

// Bias do giroscópio Z [deg/s] estimado na calibração
float getGyroBiasDegPerSec();

} // namespace TwoVectors
