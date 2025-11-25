#pragma once

namespace SolarVector {

// Inicializa o módulo de vetor solar (configura pinos etc.)
void begin();

// Atualiza a leitura do vetor solar.
// Deve ser chamada periodicamente pelo MissionController.
void update();

// Retorna o último ângulo estimado em graus [0, 360).
float getEstimatedAngle();

} // namespace SolarVector
