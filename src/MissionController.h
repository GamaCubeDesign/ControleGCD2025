#pragma once

#include <stdint.h>

// Enum com os tipos de missao suportados.
//
// A ORDEM importa se você usar tabelas internas no MissionController.cpp.
// None = 0, e os demais vêm na sequência.
enum class MissionType : uint8_t {
  None = 0,
  Stability,
  SunPointing,
  SolarVector,
  TwoVectors
};

// Inicializa o controlador de missoes.
// Use para inicializacoes globais, se precisar (Wire, IMU, motor, etc.).
void missionControllerBegin();

// Define qual missao deve rodar.
// Chama o "begin" da missao correspondente (Stability::begin(), etc.).
void missionControllerSetMission(MissionType mission);

// Atualiza a missao atual.
// Chama o "update" da missao correspondente.
void missionControllerUpdate();
