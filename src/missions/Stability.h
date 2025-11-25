#pragma once

namespace Stability {

// Inicializa a missão de estabilização (hardware + estados)
void begin();

// Atualiza o controle de estabilização.
// Deve ser chamada periodicamente pelo MissionController.
void update();

// --- OPCIONAIS (úteis para debug/telemetria) ---

// Velocidade angular do satélite (eixo Z) em rad/s (valor filtrado)
float getSatRateRadPerSec();

// Velocidade atual da roda de reação em rad/s (comando enviado ao motor)
float getWheelVelocity();

// Offset do giroscópio Z em rad/s (calculado na calibração)
float getGyroOffsetZ();

} // namespace Stability
