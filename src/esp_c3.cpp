#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "HX711.h"
#include <Wire.h>
#include <Adafruit_AHTX0.h>

// ---------- Configurações de rede ----------
const char* WIFI_SSID     = "KAUA_LQ";
const char* WIFI_SENHA    = "12345678";
const char* WS_HOST       = "192.168.1.101"; // IP do PC rodando o server python
const uint16_t WS_PORT    = 8080;
const char* WS_PATH       = "/ws/caixa_areia";

// ----- HX711 -----
HX711 scale;
const int LOADCELL_DOUT_PIN = 4;
const int LOADCELL_SCK_PIN = 5;
const float ZERO_THRESHOLD = 0.5; // Limiar para ignorar pequenas flutuações em torno de zero (em gramas)
float calibration_factor = 420.0; // Fator de calibração inicial
// -----------------

// ----- AHT10 (temperatura e umidade, I2C) -----
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1
#define INTERVALO_LEITURA_AHT_MS 2000
Adafruit_AHTX0 aht;
// ----------------------------------------------

// ---------- Presença do gato (histerese) ----------
#define PESO_INICIO_GRAVACAO_G  200.0f  // acima disso: gato entrou -> começa a gravar
#define PESO_FIM_GRAVACAO_G     100.0f  // abaixo disso: gato saiu  -> para de gravar
#define PESO_AMOSTRAS           3       // leituras por média (HX711 a 10 SPS => ~0.3 s por leitura)
#define LEITURAS_CONFIRMA_ENTRADA 2     // leituras consecutivas > 200 g para iniciar
#define LEITURAS_CONFIRMA_SAIDA   3     // leituras consecutivas < 100 g para finalizar

// ---------- Pinos do INMP441 (I2S) ----------
#define I2S_WS_PIN   6
#define I2S_SCK_PIN  7
#define I2S_SD_PIN   3
#define I2S_PORT     I2S_NUM_0

// ---------- Áudio ----------
#define SAMPLE_RATE       16000
#define AUDIO_SHIFT_BITS  14   // 16 = ganho unitário; 14 = +12 dB (com saturação, sem wrap)
#define CANAL_OFFSET      0    // 0 = canal esquerdo (pino L/R do INMP441 no GND)

#define CHUNK_AMOSTRAS 512
#define FILA_TAMANHO 40
#define MAX_ENVIOS_POR_ITERACAO 4

typedef struct {
  int16_t dados[CHUNK_AMOSTRAS];
  size_t quantidade;
} ChunkAudio;

QueueHandle_t filaAudio;
TaskHandle_t tarefaCapturaHandle;
TaskHandle_t tarefaPesoHandle;
TaskHandle_t tarefaTemperaturaHandle;

WebSocketsClient webSocket;

// ---------- Estado ----------
enum EstadoCaixa {
  LIVRE,      // sem gato (ou entre 100 g e 200 g sem gravação em andamento)
  GRAVANDO    // gato presente, áudio sendo capturado e enviado
};

EstadoCaixa estado = LIVRE;
volatile bool gravando = false;

// Compartilhado entre a tarefa de peso e o loop()
volatile float pesoAtual = 0.0f;
volatile uint32_t pesoSeq = 0;      // incrementa a cada leitura nova
volatile bool hxOk = false;
volatile bool tareSolicitada = false;

int contConfirmaEntrada = 0;
int contConfirmaSaida = 0;

// ---------- I2S ----------
void i2sInstalar() {
  i2s_config_t config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 10,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pins = {
    // Sem esta linha o campo mck_io_num fica 0 e o driver passa a emitir o MCLK
    // no GPIO0 (o INMP441 não usa MCLK). Aqui o GPIO0 é o SDA do I2C do AHT10.
    .mck_io_num = I2S_PIN_NO_CHANGE,
    .bck_io_num = I2S_SCK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD_PIN
  };

  i2s_driver_install(I2S_PORT, &config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pins);
}

// Converte 32 bits (INMP441 = 24 bits alinhados à esquerda) para 16 bits,
// com saturação: um miado alto vira "topo achatado" em vez de estourar (wrap).
inline int16_t converter32para16(int32_t amostra32) {
  int32_t v = amostra32 >> AUDIO_SHIFT_BITS;
  if (v > 32767) v = 32767;
  else if (v < -32768) v = -32768;
  return (int16_t)v;
}

// ---------- Tarefa dedicada de captura de áudio ----------
// O ESP32-C3 tem um único núcleo, então usa xTaskCreate (prioridade maior que o loop).
void tarefaCapturaAudio(void *parametro) {
  static int32_t bufferI2S[CHUNK_AMOSTRAS * 2];

  for (;;) {
    if (gravando) {
      size_t bytesLidos = 0;
      i2s_read(I2S_PORT, bufferI2S, sizeof(bufferI2S), &bytesLidos, portMAX_DELAY);
      size_t amostrasLidas = bytesLidos / sizeof(int32_t);

      ChunkAudio chunk;
      size_t out = 0;
      for (size_t i = CANAL_OFFSET; i < amostrasLidas && out < CHUNK_AMOSTRAS; i += 2) {
        chunk.dados[out++] = converter32para16(bufferI2S[i]);
      }
      chunk.quantidade = out;

      if (out > 0) {
        if (xQueueSend(filaAudio, &chunk, 0) != pdTRUE) {
          // fila cheia: descarta o chunk mais antigo e guarda o mais novo
          ChunkAudio descartado;
          xQueueReceive(filaAudio, &descartado, 0);
          xQueueSend(filaAudio, &chunk, 0);
        }
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(5));
    }
  }
}

// ---------- Tarefa de leitura da célula de carga ----------
// get_units() bloqueia (HX711 a 10 SPS é lento). Rodando em tarefa própria,
// o loop() nunca trava e o envio de áudio não sofre atrasos.
void tarefaPeso(void *parametro) {
  for (;;) {
    if (tareSolicitada) {
      Serial.println("\n[CMD] Executando tara...");
      scale.tare();
      tareSolicitada = false;
      Serial.println("[CMD] Balança zerada com sucesso!");
    }

    // Espera o HX711 ficar pronto (até 1 s). O "5" cede a CPU entre as checagens.
    if (scale.wait_ready_timeout(1000, 5)) {
      float weight = scale.get_units(PESO_AMOSTRAS);
      if (fabsf(weight) < ZERO_THRESHOLD) { // filtro de zona morta
        weight = 0.0f;
      }
      pesoAtual = weight;
      hxOk = true;
      pesoSeq++;
    } else {
      hxOk = false; // sem resposta por 1 s: desconexão de verdade
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ---------- Tarefa de leitura de temperatura/umidade (AHT10) ----------
// Roda separada das demais: a conversão do AHT10 leva ~80 ms e não deve
// atrasar o WebSocket nem a leitura da célula de carga.
void tarefaTemperatura(void *parametro) {
  bool sensorOk = false;

  for (;;) {
    if (!sensorOk) {
      sensorOk = aht.begin(&Wire);
      if (sensorOk) {
        Serial.println("[AHT10] Sensor inicializado.");
      } else {
        Serial.println("[AHT10] Não encontrado. Verifique SDA/SCL, alimentação e endereço 0x38. Tentando novamente...");
        vTaskDelay(pdMS_TO_TICKS(5000));
        continue;
      }
    }

    sensors_event_t umidade, temperatura;
    if (aht.getEvent(&umidade, &temperatura)) {
      Serial.printf("[AHT10] Temperatura: %.1f °C | Umidade: %.1f %%\n",
                    temperatura.temperature, umidade.relative_humidity);
    } else {
      Serial.println("[AHT10] Falha na leitura.");
      sensorOk = false; // força reinicialização na próxima volta
    }

    vTaskDelay(pdMS_TO_TICKS(INTERVALO_LEITURA_AHT_MS));
  }
}

// ---------- WebSocket ----------
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.println("[WS] Conectado ao servidor");
      // Se a conexão voltou no meio de uma gravação, abre um novo arquivo no servidor
      if (estado == GRAVANDO) {
        webSocket.sendTXT("start_audio");
      }
      break;
    case WStype_DISCONNECTED:
      Serial.println("[WS] Desconectado do servidor");
      break;
    case WStype_TEXT:
      Serial.printf("[WS] Servidor: %s\n", String((char*)payload).substring(0, length).c_str());
      break;
    default:
      break;
  }
}

// ---------- Controle da gravação ----------
void iniciarGravacao() {
  Serial.printf("[GATO] Entrou na caixa (%.1f g). Iniciando gravação.\n", (float)pesoAtual);
  i2s_zero_dma_buffer(I2S_PORT);  // descarta áudio velho que ficou no DMA
  xQueueReset(filaAudio);
  gravando = true;
  estado = GRAVANDO;
  webSocket.sendTXT("start_audio");
}

void finalizarGravacao() {
  Serial.printf("[GATO] Saiu da caixa (%.1f g). Finalizando gravação.\n", (float)pesoAtual);
  gravando = false;
  vTaskDelay(pdMS_TO_TICKS(50)); // deixa a tarefa de captura terminar o chunk em andamento

  ChunkAudio chunk;
  while (xQueueReceive(filaAudio, &chunk, 0) == pdTRUE) {
    if (chunk.quantidade > 0 && webSocket.isConnected()) {
      webSocket.sendBIN((uint8_t*)chunk.dados, chunk.quantidade * sizeof(int16_t));
      webSocket.loop();
    }
  }
  if (webSocket.isConnected()) {
    webSocket.sendTXT("stop_audio");
  }
  estado = LIVRE;
}

// Máquina de estados baseada na histerese 200 g / 100 g.
// Só avalia quando chega uma leitura NOVA do HX711.
void tratarPresenca() {
  static uint32_t seqVisto = 0;
  if (pesoSeq == seqVisto) return;
  seqVisto = pesoSeq;
  float w = pesoAtual;

  if (estado == LIVRE) {
    if (w > PESO_INICIO_GRAVACAO_G) {
      contConfirmaEntrada++;
      // Só começa se o servidor estiver conectado; senão tenta de novo na próxima leitura
      if (contConfirmaEntrada >= LEITURAS_CONFIRMA_ENTRADA && webSocket.isConnected()) {
        contConfirmaEntrada = 0;
        contConfirmaSaida = 0;
        iniciarGravacao();
      }
    } else {
      contConfirmaEntrada = 0;
    }
  } else { // GRAVANDO
    if (w < PESO_FIM_GRAVACAO_G) {
      contConfirmaSaida++;
      if (contConfirmaSaida >= LEITURAS_CONFIRMA_SAIDA) {
        contConfirmaSaida = 0;
        finalizarGravacao();
      }
    } else {
      contConfirmaSaida = 0; // entre 100 g e 200 g (ou acima): continua gravando
    }
  }
}

// Envia os chunks de áudio pendentes (só durante a gravação)
void tratarEnvioAudio() {
  if (estado != GRAVANDO) return;

  ChunkAudio chunk;
  int envios = 0;
  while (envios < MAX_ENVIOS_POR_ITERACAO && xQueueReceive(filaAudio, &chunk, 0) == pdTRUE) {
    if (chunk.quantidade > 0 && webSocket.isConnected()) {
      webSocket.sendBIN((uint8_t*)chunk.dados, chunk.quantidade * sizeof(int16_t));
    }
    envios++;
  }
}

void tratarSerial() {
  if (Serial.available() > 0) { // Verifica se há comandos enviados pela Serial
    char cmd = Serial.read();
    if (cmd == 't' || cmd == 'T') {
      if (estado == GRAVANDO) {
        Serial.println("\n[CMD] Tara ignorada: gravação em andamento.");
      } else {
        tareSolicitada = true; // executada pela tarefaPeso (dona do HX711)
      }
    } else if (cmd == 'c' || cmd == 'C') {
      Serial.print("\n[CMD] Fator de calibração atual: ");
      Serial.println(calibration_factor);
    }
  }
}

void tratarImpressaoPeso() {
  static unsigned long ultimo = 0;
  if (millis() - ultimo < 400) return;
  ultimo = millis();

  if (hxOk) {
    Serial.printf("Peso aferido: %.2f g%s\n", (float)pesoAtual, estado == GRAVANDO ? "  [GRAVANDO]" : "");
  } else {
    Serial.println("HX711 não encontrado ou desconectado.");
  }
}

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

  // ----- I2C / AHT10 -----
  Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.begin();

  // ----- WiFi -----
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_SENHA);
  Serial.print("Conectando ao WiFi");
  unsigned long inicioWifi = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - inicioWifi < 20000) {
    delay(300);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi conectado. IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nSem WiFi ainda (o driver continuará tentando em segundo plano).");
  }

  // ----- Áudio -----
  i2sInstalar();
  filaAudio = xQueueCreate(FILA_TAMANHO, sizeof(ChunkAudio));
  xTaskCreate(tarefaCapturaAudio, "CapturaAudio", 6144, NULL, 3, &tarefaCapturaHandle);
  xTaskCreate(tarefaPeso, "LeituraPeso", 4096, NULL, 1, &tarefaPesoHandle);
  xTaskCreate(tarefaTemperatura, "LeituraTemp", 4096, NULL, 1, &tarefaTemperaturaHandle);

  // ----- WebSocket -----
  webSocket.begin(WS_HOST, WS_PORT, WS_PATH);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
}

void loop() {
  webSocket.loop();
  tratarSerial();
  tratarPresenca();
  tratarEnvioAudio();
  tratarImpressaoPeso();
}