#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#include "MissionController.h"
#include "missions/SunPointing.h"   // para getEstimatedAngle()
#include "missions/SolarVector.h"
#include "missions/Stability.h"
#include "missions/TwoVectors.h"
// Opcional: se você quiser que o Código 2 realmente configure
// um ângulo alvo em alguma missão (TwoVectors, por exemplo),
// pode incluir o header correspondente e criar um setTargetAngleDeg() lá.
// #include "missions/TwoVectors.h"

WebServer server(80);

// ====================== CONFIG WI-FI (ALTERAR AQUI) ======================
// Deixa tudo concentrado num lugar só para ser fácil de preencher.
// Depois, se quiser algo 100% dinâmico, dá para ler SSID/senha via Serial
// e salvar em NVS/SPIFFS.

const char* WIFI_SSID     = "SEU_SSID_AQUI";
const char* WIFI_PASSWORD = "SUA_SENHA_AQUI";

// ====================== ESTADO GLOBAL ======================

static MissionType currentMission = MissionType::None;

// Esse bool liga/desliga o "Código 1" (vetor solar)
bool code1Running = false;

// Último ângulo solar enviado para a interface
float ultimoAngulo = 0.0f;

// Se quiser usar o Código 2 como apontamento de ângulo genérico
float targetAngleDeg = 0.0f;

// ====================== FUNÇÕES AUXILIARES ======================

static void printMissionName(MissionType mission) {
  switch (mission) {
    case MissionType::Stability:
      Serial.print(F("Stability (1)"));
      break;
    case MissionType::SunPointing:
      Serial.print(F("SunPointing (2/3)"));
      break;
    case MissionType::SolarVector:
      Serial.print(F("SolarVector (1)"));
      break;
    case MissionType::TwoVectors:
      Serial.print(F("TwoVectors (2)"));
      break;
    case MissionType::None:
    default:
      Serial.print(F("None (0)"));
      break;
  }
}

static void connectWiFi() {
  Serial.println();
  Serial.print(F("[WiFi] Conectando em SSID: "));
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println(F("[WiFi] Conectado!"));
  Serial.print(F("[WiFi] IP: "));
  Serial.println(WiFi.localIP());
}

// ====================== HANDLERS HTTP ======================

void handleRoot() {
  // Página baseada na que você mandou, só limpei o texto pra ficar mais genérico.
  String html = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <head>
    <title>Gama Cube Design - AOCS</title>
    <meta charset="UTF-8">
    <style>
      body { font-family: Arial; margin: 40px; }
      button { padding: 12px 24px; margin: 10px; font-size: 16px; }
      #resposta {
        background: #f0f0f0;
        padding: 15px;
        margin: 20px 0;
        border-radius: 5px;
        min-height: 20px;
      }
      #anguloAtual {
        background: #e0f7fa;
        padding: 15px;
        margin: 20px 0;
        border-radius: 5px;
        font-size: 18px;
        font-weight: bold;
        text-align: center;
      }
      .input-group { margin: 15px 0; }
      input[type="number"] { padding: 8px; font-size: 16px; width: 100px; }
      .status {
        padding: 5px 10px;
        border-radius: 3px;
        font-size: 14px;
        margin-left: 10px;
      }
      .ativo { background: #4CAF50; color: white; }
      .inativo { background: #f44336; color: white; }
    </style>
  </head>
  <body>
    <h1>Gama Cube Design - AOCS</h1>
    <p>ESP32 conectado via Wi-Fi</p>

    <div class="input-group">
      <h3>Vetor/Sun Vector <span id="status1" class="status inativo">PARADO</span></h3>
      <button onclick="toggleCode1()" id="btnCode1">Iniciar Código 1</button>
    </div>

    <div id="anguloAtual">
      Ângulo solar: <span id="valorAngulo">--</span>°
    </div>

    <div class="input-group">
      <h3>Ângulos quaisquer</h3>
      <input type="number" id="inteiroInput" value="0" min="0" max="1000">
      <button onclick="runCode(2)">Executar Código 2</button>
    </div>

    <div class="input-group">
      <h3>Sun Pointing</h3>
      <button onclick="runCode(3)">Executar Código 3</button>
    </div>

    <div class="input-group">
      <h3>Estabilização a 60RPM</h3>
      <button onclick="runCode(4)">Executar Código 4</button>
    </div>

    <div id="resposta">
      Clique em um botão para ver a resposta aqui...
    </div>

    <script>
      let code1Ativo = false;
      let intervaloAtualizacao = null;

      function toggleCode1() {
        code1Ativo = !code1Ativo;
        const btn = document.getElementById('btnCode1');
        const status = document.getElementById('status1');
        const anguloDiv = document.getElementById('anguloAtual');

        fetch('/run1')
          .then(response => response.text())
          .then(data => {
            document.getElementById('resposta').innerHTML = data;

            if (code1Ativo) {
              btn.textContent = 'Parar Código 1';
              status.textContent = 'ATIVO';
              status.className = 'status ativo';
              anguloDiv.style.display = 'block';

              // Inicia atualização automática a cada 2 segundos
              iniciarAtualizacaoAngulo();
            } else {
              btn.textContent = 'Iniciar Código 1';
              status.textContent = 'PARADO';
              status.className = 'status inativo';
              document.getElementById('valorAngulo').textContent = '--';

              // Para a atualização automática
              pararAtualizacaoAngulo();
            }
          })
          .catch(error => {
            document.getElementById('resposta').innerHTML = 'Erro: ' + error;
          });
      }

      function iniciarAtualizacaoAngulo() {
        atualizarAngulo();
        intervaloAtualizacao = setInterval(atualizarAngulo, 2000);
      }

      function pararAtualizacaoAngulo() {
        if (intervaloAtualizacao) {
          clearInterval(intervaloAtualizacao);
          intervaloAtualizacao = null;
        }
      }

      function atualizarAngulo() {
        fetch('/getAngle')
          .then(response => response.text())
          .then(angulo => {
            const num = parseFloat(angulo);
            if (!isNaN(num)) {
              document.getElementById('valorAngulo').textContent = num.toFixed(2);
            }
          })
          .catch(error => {
            console.error('Erro ao atualizar ângulo:', error);
          });
      }

      function runCode(code) {
        let url = '/run' + code;
        const respostaDiv = document.getElementById('resposta');

        // Para o Código 2: pega o valor do input
        if (code === 2) {
          const valor = document.getElementById('inteiroInput').value;
          url += '?valor=' + valor;
        }

        respostaDiv.innerHTML = '🔄 Executando Código ' + code + '...';

        fetch(url)
          .then(response => response.text())
          .then(data => {
            respostaDiv.innerHTML = '<strong>Resposta do ESP32:</strong><br>' + data;
          })
          .catch(error => {
            respostaDiv.innerHTML = 'Erro: ' + error;
          });
      }

      // Inicializa a interface
      document.getElementById('anguloAtual').style.display = 'none';
    </script>
  </body>
  </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

// Retorna o ângulo atual do SunPointing
void handleGetAngle() {
  // Lê o último ângulo estimado pela missão SunPointing
  float ang = SolarVector::getEstimatedAngle();
  ultimoAngulo = ang;
  server.send(200, "text/plain", String(ang, 2));
}

// Código 1 - alterna ligar/desligar SunPointing/SolarVector
void handleRunCode1() {
  code1Running = !code1Running;

  if (code1Running) {
    // Aqui você pode escolher qual missão está ligada ao "vetor solar":
    // - SunPointing, se ela já calcula o vetor + controle
    // - SolarVector, se for uma missão separada
    currentMission = MissionType::SolarVector;
    missionControllerSetMission(currentMission);

    Serial.println(F("[WiFi] Código 1 INICIADO - Missão SunPointing ativa"));
    server.send(200, "text/plain", "Código 1 INICIADO - Missão SunPointing ativa");
  } else {
    // Quando parar, você pode voltar para None ou para alguma missão default
    currentMission = MissionType::None;
    missionControllerSetMission(currentMission);

    Serial.println(F("[WiFi] Código 1 PARADO - Missão None"));
    server.send(200, "text/plain", "Código 1 PARADO - Missão None");
  }
}

// Código 2 - recebe um inteiro via interface (ex: ângulo alvo)
void handleRunCode2() {
  if (server.hasArg("valor")) {
    float valorRecebido = server.arg("valor").toFloat();

    // Define o alvo em graus na missão TwoVectors
    TwoVectors::setTargetAngleDeg(valorRecebido);

    // Garante que a missão TwoVectors está ativa
    currentMission = MissionType::TwoVectors;
    missionControllerSetMission(currentMission);

    Serial.print(F("[WiFi] Código 2 executado - Novo setpoint (deg): "));
    Serial.println(valorRecebido);

    server.send(200, "text/plain",
                "Código 2 executado - Apontamento alvo (deg): " +
                String(valorRecebido));
  } else {
    server.send(400, "text/plain", "Erro: Nenhum valor fornecido");
  }
}

// Código 3 - Sun Pointing
void handleRunCode3() {
  currentMission = MissionType::SunPointing;
  missionControllerSetMission(currentMission);

  Serial.println(F("[WiFi] Código 3 executado - Missão SunPointing"));
  server.send(200, "text/plain", "Código 3 executado - Missão SunPointing ativada");
}

// Código 4 - Estabilização (Stability)
void handleRunCode4() {
  currentMission = MissionType::Stability;
  missionControllerSetMission(currentMission);

  Serial.println(F("[WiFi] Código 4 executado - Missão Stability"));
  server.send(200, "text/plain", "Código 4 executado - Missão Stability ativada");
}

// ====================== SETUP & LOOP ======================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println(F("=== CubeSat WiFi WebServer Mission Control ==="));

  // Inicializa controlador de missões
  missionControllerBegin();

  // Missão inicial (se quiser, pode ser None)
  // currentMission = MissionType::Stability;
  missionControllerSetMission(currentMission);

  Serial.print(F("[MAIN] Missão inicial: "));
  printMissionName(currentMission);
  Serial.println();

  // Conecta ao Wi-Fi
  connectWiFi();

  // Configura rotas HTTP
  server.on("/", handleRoot);
  server.on("/run1", handleRunCode1);
  server.on("/run2", handleRunCode2);
  server.on("/run3", handleRunCode3);
  server.on("/run4", handleRunCode4);
  server.on("/getAngle", handleGetAngle);

  server.begin();
  Serial.println(F("[WiFi] Servidor HTTP iniciado na porta 80"));
}

void loop() {
  // Trata requisições HTTP
  server.handleClient();

  // Atualiza a missão atual (Stability::update(), SunPointing::update(), etc.)
  missionControllerUpdate();

  // Nenhum delay aqui -> mantém o servidor responsivo.
}
