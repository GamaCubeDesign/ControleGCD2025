#include "MissionController.h"

// Headers das missoes
#include "missions/SolarVector.h"
#include "missions/Stability.h"
#include "missions/SunPointing.h"
#include "missions/TwoVectors.h"

// ====================== ESTADO INTERNO ======================

// Missao atual
static MissionType currentMission = MissionType::None;

// Estrutura com ponteiros para as funcoes de cada missao
struct MissionVTable {
  void (*begin)();
  void (*update)();
};

// Indices:
//   0 -> None
//   1 -> Stability
//   2 -> SunPointing
//   3 -> SolarVector
//   4 -> TwoVectors
static const MissionVTable missionTable[] = {
    // None
    {nullptr, nullptr},

    // Stability
    {&Stability::begin, &Stability::update},

    // SunPointing
    {&SunPointing::begin, &SunPointing::update},

    // SolarVector
    {&SolarVector::begin, &SolarVector::update},

    // TwoVectors
    {&TwoVectors::begin, &TwoVectors::update},
};

static constexpr uint8_t MISSION_TABLE_SIZE =
    sizeof(missionTable) / sizeof(missionTable[0]);

// ====================== FUNÇÕES PÚBLICAS ======================

void missionControllerBegin() {
  // Se você tiver algo de hardware global (Wire.begin(), motor, imu, etc.)
  // pode inicializar aqui.
  //
  // Por enquanto, só garante que estamos em None.
  currentMission = MissionType::None;
}

void missionControllerSetMission(MissionType mission) {
  currentMission = mission;

  const uint8_t idx = static_cast<uint8_t>(currentMission);

  if (idx >= MISSION_TABLE_SIZE) {
    // Valor invalido -> volta para None
    currentMission = MissionType::None;
    return;
  }

  // Chama o begin da missao (se existir)
  if (missionTable[idx].begin) {
    missionTable[idx].begin();
  }
}

void missionControllerUpdate() {
  const uint8_t idx = static_cast<uint8_t>(currentMission);

  if (idx >= MISSION_TABLE_SIZE) {
    return;
  }

  // Chama o update da missao (se existir)
  if (missionTable[idx].update) {
    missionTable[idx].update();
  }
}
