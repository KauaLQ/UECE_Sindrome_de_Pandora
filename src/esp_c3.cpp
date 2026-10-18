#include <Arduino.h>
#include "HX711.h"

// ----- HX711 -----
HX711 scale;
const int LOADCELL_DOUT_PIN = 4;
const int LOADCELL_SCK_PIN = 5;
const float ZERO_THRESHOLD = 0.5; // Limiar para ignorar pequenas flutuações em torno de zero (em gramas)
float calibration_factor = 420.0; // Fator de calibração inicial
// -----------------

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n--- Inicializando Balança com HX711 e ESP32-C3 ---");
  Serial.println("Comandos disponíveis no Monitor Serial:");
  Serial.println("  [t] ou [T] - Tarar (zerar) a balança");
  Serial.println("  [c] ou [C] - Exibir fator de calibração atual");

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);         // Inicializa a biblioteca HX711
  scale.set_scale(calibration_factor);                      // Configura o fator de calibração inicial
  Serial.println("Tarando a balança... Mantenha-a vazia."); // Zera a balança (Tara).
  scale.tare(); 
  Serial.println("Balança pronta para uso!");
}

void loop() {
  if (Serial.available() > 0) { // Verifica se há comandos enviados pela Serial
    char cmd = Serial.read();
    if (cmd == 't' || cmd == 'T') {
      Serial.println("\n[CMD] Executando tara...");
      scale.tare();
      Serial.println("[CMD] Balança zerada com sucesso!");
    } else if (cmd == 'c' || cmd == 'C') {
      Serial.print("\n[CMD] Fator de calibração atual: ");
      Serial.println(calibration_factor);
    }
  }

  if (scale.is_ready()) { // Verifica se o HX711 está pronto
    float weight = scale.get_units(10); // Lê a média de 10 leituras para maior estabilidade
    if (abs(weight) < ZERO_THRESHOLD) { // Aplica o filtro de zona morta para zerar pequenas oscilações
      weight = 0.0;
    }
    Serial.print("Peso aferido: ");
    Serial.print(weight, 2); // Exibe com 2 casas decimais
    Serial.println(" g");
  } else {
    Serial.println("HX711 não encontrado ou desconectado.");
  }
  delay(400);
}