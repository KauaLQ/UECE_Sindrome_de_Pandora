#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include "HX711.h"

// ---------- Rede ----------
const char* WIFI_SSID  = "KAUA_LQ";
const char* WIFI_SENHA = "12345678";
const char* WS_HOST    = "192.168.1.101"; // IP do PC rodando o server python
const uint16_t WS_PORT = 8080;
const char* WS_PATH    = "/ws/caixa_areia";

// ---------- HX711 ----------
const int   LOADCELL_DOUT_PIN = 4;
const int   LOADCELL_SCK_PIN  = 5;
const float ZERO_THRESHOLD    = 0.5;   // zona morta em torno de zero (g)
const float CALIBRATION_FACTOR = 420.0;
#define PESO_AMOSTRAS 3                // leituras por média (~0.3 s a 10 SPS)

// ---------- Presença do gato (histerese) ----------
#define PESO_INICIO_GRAVACAO_G    200.0f // acima: gato entrou -> grava
#define PESO_FIM_GRAVACAO_G       100.0f // abaixo: gato saiu  -> para
#define LEITURAS_CONFIRMA_ENTRADA 2      // leituras consecutivas para iniciar
#define LEITURAS_CONFIRMA_SAIDA   3      // leituras consecutivas para finalizar

// ---------- AHT10 (I2C) ----------
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1
#define INTERVALO_LEITURA_AHT_MS 2000

// ---------- INMP441 (I2S) ----------
#define I2S_WS_PIN   6
#define I2S_SCK_PIN  7
#define I2S_SD_PIN   3
#define I2S_PORT     I2S_NUM_0

// ---------- Áudio ----------
#define SAMPLE_RATE       16000
#define AUDIO_SHIFT_BITS  14   // 16 = ganho unitário; 14 = +12 dB (com saturação)
#define CANAL_OFFSET      0    // 0 = canal esquerdo (pino L/R do INMP441 no GND)
#define CHUNK_AMOSTRAS    512
#define FILA_AUDIO_TAMANHO 40
#define FILA_PESO_TAMANHO  10

typedef struct {
  int16_t dados[CHUNK_AMOSTRAS];
  size_t quantidade;
} ChunkAudio;

// ---------- Objetos compartilhados ----------
HX711 scale;
Adafruit_AHTX0 aht;
WebSocketsClient webSocket;

QueueHandle_t filaAudio;          // capturaAudio  -> gerenciamento (chunks PCM)
QueueHandle_t filaPeso;           // peso          -> gerenciamento (float, em g)
SemaphoreHandle_t wsMutex;        // WebSocketsClient não é thread-safe: todo acesso passa por aqui

volatile bool gravando = false;       // lido pela tarefa de captura
volatile bool wsConectado = false;    // atualizado pelo callback do WebSocket
volatile bool wsNovaConexao = false;  // callback avisa que (re)conectou

enum EstadoCaixa { LIVRE, GRAVANDO };

// ---------------------------------------------------------------------
//  WebSocket
// ---------------------------------------------------------------------
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.println("[WS] Conectado ao servidor");
      wsConectado = true;
      wsNovaConexao = true;
      break;
    case WStype_DISCONNECTED:
      Serial.println("[WS] Desconectado do servidor");
      wsConectado = false;
      break;
    case WStype_TEXT:
      Serial.printf("[WS] Servidor: %.*s\n", (int)length, (char*)payload);
      break;
    default:
      break;
  }
}

// Envios feitos fora do loop() passam pelo mutex, para não colidirem com webSocket.loop()
void wsEnviarTXT(const char* texto) {
  if (!wsConectado) return;
  xSemaphoreTake(wsMutex, portMAX_DELAY);
  webSocket.sendTXT(texto);
  xSemaphoreGive(wsMutex);
}

void wsEnviarChunk(const ChunkAudio& chunk) {
  if (!wsConectado || chunk.quantidade == 0) return;
  xSemaphoreTake(wsMutex, portMAX_DELAY);
  webSocket.sendBIN((const uint8_t*)chunk.dados, chunk.quantidade * sizeof(int16_t));
  xSemaphoreGive(wsMutex);
}

// ---------------------------------------------------------------------
//  I2S / captura de áudio
// ---------------------------------------------------------------------
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

// 32 bits (INMP441 = 24 bits alinhados à esquerda) -> 16 bits, com saturação
inline int16_t converter32para16(int32_t amostra32) {
  int32_t v = amostra32 >> AUDIO_SHIFT_BITS;
  if (v > 32767) v = 32767;
  else if (v < -32768) v = -32768;
  return (int16_t)v;
}

void tarefaCapturaAudio(void *parametro) {
  static int32_t bufferI2S[CHUNK_AMOSTRAS * 2];

  for (;;) {
    if (!gravando) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    size_t bytesLidos = 0;
    i2s_read(I2S_PORT, bufferI2S, sizeof(bufferI2S), &bytesLidos, portMAX_DELAY);
    size_t amostrasLidas = bytesLidos / sizeof(int32_t);

    ChunkAudio chunk;
    size_t out = 0;
    for (size_t i = CANAL_OFFSET; i < amostrasLidas && out < CHUNK_AMOSTRAS; i += 2) {
      chunk.dados[out++] = converter32para16(bufferI2S[i]);
    }
    chunk.quantidade = out;

    if (out > 0 && xQueueSend(filaAudio, &chunk, 0) != pdTRUE) {
      // fila cheia: descarta o chunk mais antigo e guarda o mais novo
      ChunkAudio descartado;
      xQueueReceive(filaAudio, &descartado, 0);
      xQueueSend(filaAudio, &chunk, 0);
    }
  }
}

// ---------------------------------------------------------------------
//  Gerenciamento da caixa: máquina de estados + envio do áudio
// ---------------------------------------------------------------------
static EstadoCaixa estado = LIVRE;   // só esta tarefa lê/escreve
static int contConfirmaEntrada = 0;
static int contConfirmaSaida = 0;

static void iniciarGravacao(float peso) {
  Serial.printf("[GATO] Entrou na caixa (%.1f g). Iniciando gravação.\n", peso);
  i2s_zero_dma_buffer(I2S_PORT);  // descarta áudio velho que ficou no DMA
  xQueueReset(filaAudio);
  gravando = true;
  estado = GRAVANDO;
  wsEnviarTXT("start_audio");
}

static void finalizarGravacao(float peso) {
  Serial.printf("[GATO] Saiu da caixa (%.1f g). Finalizando gravação.\n", peso);
  gravando = false;
  vTaskDelay(pdMS_TO_TICKS(50)); // deixa a captura terminar o chunk em andamento

  ChunkAudio chunk;
  while (xQueueReceive(filaAudio, &chunk, 0) == pdTRUE) {
    wsEnviarChunk(chunk);
  }
  wsEnviarTXT("stop_audio");
  estado = LIVRE;
}

// Avalia uma leitura de peso (histerese 200 g / 100 g)
static void tratarPresenca(float peso) {
  Serial.printf("Peso aferido: %.2f g%s\n", peso, estado == GRAVANDO ? "  [GRAVANDO]" : "");

  if (estado == LIVRE) {
    if (peso > PESO_INICIO_GRAVACAO_G) {
      contConfirmaEntrada++;
      // Só começa com o servidor conectado; senão tenta de novo na próxima leitura
      if (contConfirmaEntrada >= LEITURAS_CONFIRMA_ENTRADA && wsConectado) {
        contConfirmaEntrada = 0;
        contConfirmaSaida = 0;
        iniciarGravacao(peso);
      }
    } else {
      contConfirmaEntrada = 0;
    }
  } else { // GRAVANDO
    if (peso < PESO_FIM_GRAVACAO_G) {
      contConfirmaSaida++;
      if (contConfirmaSaida >= LEITURAS_CONFIRMA_SAIDA) {
        contConfirmaSaida = 0;
        finalizarGravacao(peso);
      }
    } else {
      contConfirmaSaida = 0; // entre 100 g e 200 g (ou acima): continua gravando
    }
  }
}

// Envia o áudio acumulado. Bloqueia até 20 ms esperando o primeiro chunk
// (acorda assim que ele chega) e depois esvazia o que mais houver na fila.
static void enviarAudioPendente() {
  ChunkAudio chunk;
  TickType_t espera = pdMS_TO_TICKS(20);
  while (xQueueReceive(filaAudio, &chunk, espera) == pdTRUE) {
    wsEnviarChunk(chunk);
    espera = 0;
  }
}

void tarefaGerenciamentoCaixa(void *parametro) {
  float peso;

  for (;;) {
    if (estado == LIVRE) {
      // Nada a fazer até chegar uma leitura nova de peso
      xQueueReceive(filaPeso, &peso, portMAX_DELAY);
      tratarPresenca(peso);
    } else {
      // Se o WebSocket reconectou no meio da gravação, abre um novo arquivo no servidor
      if (wsNovaConexao) {
        wsNovaConexao = false;
        wsEnviarTXT("start_audio");
      }
      enviarAudioPendente();
      while (xQueueReceive(filaPeso, &peso, 0) == pdTRUE) {
        tratarPresenca(peso);
      }
    }
  }
}

// ---------------------------------------------------------------------
//  Leitura da célula de carga
// ---------------------------------------------------------------------
void tarefaPeso(void *parametro) {
  for (;;) {
    // Espera o HX711 ficar pronto (até 1 s). O "5" cede a CPU entre as checagens.
    if (scale.wait_ready_timeout(1000, 5)) {
      float peso = scale.get_units(PESO_AMOSTRAS);
      if (fabsf(peso) < ZERO_THRESHOLD) {
        peso = 0.0f;
      }
      xQueueSend(filaPeso, &peso, 0); // se a fila encher, descarta a leitura
    } else {
      Serial.println("HX711 não encontrado ou desconectado.");
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ---------------------------------------------------------------------
//  Leitura de temperatura/umidade
// ---------------------------------------------------------------------
void tarefaTemperatura(void *parametro) {
  bool sensorOk = false;

  for (;;) {
    if (!sensorOk) {
      sensorOk = aht.begin(&Wire);
      if (sensorOk) {
        Serial.println("[AHT10] Sensor inicializado.");
      } else {
        Serial.println("[AHT10] Não encontrado. Tentando novamente...");
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

// ---------------------------------------------------------------------
//  setup / super loop
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n--- Inicializando monitor da caixa de areia (ESP32-C3) ---");

  // ----- HX711 -----
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(CALIBRATION_FACTOR);
  Serial.println("Tarando a balança... Mantenha-a vazia.");
  scale.tare();
  Serial.println("Balança pronta para uso!");

  // ----- I2C (AHT10) -----
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

  // ----- Filas, mutex e I2S -----
  wsMutex   = xSemaphoreCreateMutex();
  filaAudio = xQueueCreate(FILA_AUDIO_TAMANHO, sizeof(ChunkAudio));
  filaPeso  = xQueueCreate(FILA_PESO_TAMANHO, sizeof(float));
  i2sInstalar();

  // ----- WebSocket -----
  webSocket.begin(WS_HOST, WS_PORT, WS_PATH);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);

  // ----- Tarefas (o ESP32-C3 tem um único núcleo) -----
  xTaskCreate(tarefaCapturaAudio,       "CapturaAudio", 6144, NULL, 3, NULL);
  xTaskCreate(tarefaGerenciamentoCaixa, "GerCaixa",     6144, NULL, 2, NULL);
  xTaskCreate(tarefaPeso,               "LeituraPeso",  4096, NULL, 1, NULL);
  xTaskCreate(tarefaTemperatura,        "LeituraTemp",  4096, NULL, 1, NULL);
}

void loop() {
  xSemaphoreTake(wsMutex, portMAX_DELAY);
  webSocket.loop();
  xSemaphoreGive(wsMutex);
  vTaskDelay(pdMS_TO_TICKS(1));
}