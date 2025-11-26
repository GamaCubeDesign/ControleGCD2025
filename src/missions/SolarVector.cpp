#include "SolarVector.h"
#include <Arduino.h> // para analogRead, pinMode, constrain, PI, millis()

namespace SolarVector {

// ====================== HARDWARE ======================

// Pinos dos sensores solares em cada face
const int painel1 = 32; // +X (GPIO32 - ADC1_CH4)
const int painel2 = 33; // +Y (GPIO33 - ADC1_CH5)
const int painel3 = 34; // -X (GPIO34 - ADC1_CH6)
const int painel4 = 35; // -Y (GPIO35 - ADC1_CH7)

// ADC para ESP32 (12 bits)
const float STEP = (3.5f / 4095.0f);

// Tensões máximas de calibração (a 0 grau, em Volts)
float v0max = 2.64f; // +X
float v1max = 2.70f; // +Y
float v2max = 2.56f; // -X
float v3max = 2.60f; // -Y

// Ângulo estimado (em graus)
static float angulo_est = 0.0f;

// Controle de taxa de amostragem (não bloqueante)
static const unsigned long SAMPLE_PERIOD_MS = 200; // ajusta aqui se quiser
static unsigned long lastSampleMs = 0;

// ====================== FUNÇÕES INTERNAS ======================

static float calcularVetorSolar() {
  // Leitura das placas (convertendo para tensão)
  float adc0 = analogRead(painel1) * STEP; // +X
  float adc1 = analogRead(painel2) * STEP; // +Y
  float adc2 = analogRead(painel3) * STEP; // -X
  float adc3 = analogRead(painel4) * STEP; // -Y

  // MÉTODO CORRIGIDO: Iluminação complementar
  float delta_x = (adc0 / v0max) - (adc2 / v2max);
  float delta_y = (adc1 / v1max) - (adc3 / v3max);

  // Limita valores
  delta_x = constrain(delta_x, -1.0f, 1.0f);
  delta_y = constrain(delta_y, -1.0f, 1.0f);

  // Calcula ângulo usando atan2
  float angulo = atan2f(delta_y, delta_x);

  // Garante ângulo positivo [0, 2π]
  if (angulo < 0) {
    angulo += 2.0f * PI;
  }

  // Converte para graus
  float angulo_deg = angulo * 180.0f / PI;
  return angulo_deg;
}

// ====================== API PÚBLICA ======================

void begin() {
  // Configura pinos explicitamente
  pinMode(painel1, INPUT);
  pinMode(painel2, INPUT);
  pinMode(painel3, INPUT);
  pinMode(painel4, INPUT);

  lastSampleMs = 0;
  angulo_est = 0.0f;
}

void update() {
  unsigned long now = millis();

  // Respeita o período de amostragem
  if (now - lastSampleMs < SAMPLE_PERIOD_MS) {
    return;
  }
  lastSampleMs = now;

  angulo_est = calcularVetorSolar();

  // Debug opcional
  Serial.print(F("[SolarVector] Angulo estimado: "));
  Serial.println(angulo_est);
}

float getEstimatedAngle() { return angulo_est; }

} // namespace SolarVector
