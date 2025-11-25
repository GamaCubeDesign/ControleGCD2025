#include <Arduino.h>
#include "MissionController.h"

// Guarda última missão ativa para detectar mudança
static MissionType lastMission = MissionType::None;

// Função de reset de software da ESP32
static void cubesatSoftReset() {
  // Serial.println(F("[SYSTEM] Reset de software solicitado via Serial..."));
  // Serial.flush();   // garante que a mensagem seja enviada
  delay(200);       // pequeno atraso para transmissão
  ESP.restart();    // reinicia a ESP32
}

// Lê um comando simples da Serial e converte em MissionType.
// Se não houver comando novo, retorna currentMission para manter o estado.
static MissionType readMissionFromSignal(MissionType currentMission) {
  if (!Serial.available()) {
    return currentMission;
  }

  const char c = Serial.read();

  switch (c) {
    case '1':
      return MissionType::Stability;

    case '2':
      return MissionType::SunPointing;

    case '3':
      return MissionType::SolarVector;

    case '4':
      return MissionType::TwoVectors;

    case 'r':
    case 'R':
      // Faz reset de software da ESP32
      cubesatSoftReset();
      // Em teoria não chega aqui, mas retornamos o estado atual por segurança
      return currentMission;

    default:
      // caractere desconhecido: ignora e mantém a missão atual
      return currentMission;
  }
}

// Só para centralizar os prints de nomes de missão
static void printMissionName(MissionType mission) {
  switch (mission) {
    case MissionType::Stability:
      // Serial.print(F("Stability (1)"));
      break;

    case MissionType::SunPointing:
      // Serial.print(F("SunPointing (2)"));
      break;

    case MissionType::SolarVector:
      // Serial.print(F("SolarVector (3)"));
      break;

    case MissionType::TwoVectors:
      // Serial.print(F("TwoVectors (4)"));
      break;

    case MissionType::None:
    default:
      // Serial.print(F("None"));
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // tempo para a Serial "subir" após o reset

  // Serial.println();
  // Serial.println(F("=== CubeSat main iniciado ==="));
  // Serial.println(F("Comandos de missao via Serial:"));
  // Serial.println(F("  '1' -> Stability"));
  // Serial.println(F("  '2' -> SunPointing"));
  // Serial.println(F("  '3' -> SolarVector"));
  // Serial.println(F("  '4' -> TwoVectors"));
  // Serial.println(F("  'r' -> Reset da ESP32"));
  // Serial.println();

  // Inicializa recursos globais do controlador (se houver)
  missionControllerBegin();

  // Define missão inicial
  lastMission = MissionType::Stability;
  missionControllerSetMission(lastMission);

  // Serial.print(F("Missao inicial: "));
  printMissionName(lastMission);
  // Serial.println();
}

void loop() {
  // Lê se chegou algum comando novo para trocar de missão
  MissionType newMission = readMissionFromSignal(lastMission);

  // Se mudou, notifica o MissionController
  if (newMission != lastMission) {
    missionControllerSetMission(newMission);

    // Serial.print(F("Mudando para missao: "));
    printMissionName(newMission);
    // Serial.println();

    lastMission = newMission;
  }

  // Atualiza a missão atual:
  //   Stability::update()
  //   SunPointing::update()
  //   SolarVector::update()
  //   TwoVectors::update()
  missionControllerUpdate();

  // Importante: nao usar delay() aqui para nao travar o laço principal.
  // Cada missao deve controlar sua propria taxa de update (usando millis()).
}
