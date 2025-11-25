#pragma once

namespace SunPointing {

// Inicializa a missão de Sun Pointing
// - I2C (IMU + AS5600)
// - Motor BLDC + driver L6234
// - PID de velocidade / FOC
// - Estados internos de controle
void begin();

// Atualiza o controle de Sun Pointing.
// Deve ser chamada periodicamente pelo MissionController.
void update();

// --------- Getters opcionais (úteis para debug/telemetria) ---------

// Último ângulo do Sol no referencial do satélite [rad], em [0, 2π)
float getSunAngleRad();

// Última leitura do giroscópio Z [rad/s]
float getGyroZRadPerSec();

// Comando de velocidade alvo da roda [rad/s] (saída do controlador antes da
// rampa)
float getTargetVelocity();

// Velocidade da roda efetivamente aplicada em motor.move() [rad/s]
float getRampedVelocity();

} // namespace SunPointing
