#include <Arduino.h>
#include <WiFi.h>
#include "esp_websocket_client.h"
#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include "HX711.h"

// ---------- Rede ----------
const char* WIFI_SSID  = "KAUA_LQ";
const char* WIFI_SENHA = "12345678";
const char* WS_HOST    = "192.168.1.104"; // IP do PC rodando o server python
const uint16_t WS_PORT = 8080;
const char* WS_PATH    = "/ws/caixa_areia";
#define WS_PING_INTERVALO_S   5      // heartbeat: detecta conexão "morta" em poucos segundos
#define WS_PING_TIMEOUT_S     10
#define WS_TIMEOUT_ENVIO_MS   500    // quanto um envio pode esperar antes de ser considerado falha
#define INTERVALO_TELEMETRIA_MS 2000 // período de envio do pacote JSON de telemetria

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
#define CHUNK_AMOSTRAS    512  // 32 ms por chunk
#define FILA_AUDIO_TAMANHO 64  // ~2 s de folga caso a rede engasgue
#define FILA_PESO_TAMANHO  10
#define ATRASO_STOP_MS     50  // espera o último chunk da captura chegar na fila antes do stop_audio

typedef struct {
  int16_t dados[CHUNK_AMOSTRAS];
  size_t quantidade;
} ChunkAudio;

// ---------- Objetos compartilhados ----------
HX711 scale;
Adafruit_AHTX0 aht;
esp_websocket_client_handle_t wsClient = NULL;

QueueHandle_t filaAudio;   // captura -> tarefaEnvioAudio
QueueHandle_t filaPeso;    // peso    -> gerenciamento (float, em g)

volatile bool gravando = false;             // gerenciamento escreve; captura e envio leem
volatile bool wsConectado = false;          // atualizado pelo callback do WebSocket
volatile uint32_t wsConexoes = 0;           // incrementa a cada (re)conexão
volatile uint32_t fimGravacaoMs = 0;        // instante em que gravando virou false
volatile uint32_t chunksDescartados = 0;    // diagnóstico: chunks perdidos por fila cheia
volatile uint32_t falhasEnvio = 0;          // diagnóstico: chunks que a rede recusou

// Últimos valores medidos (escritos pelas tarefas de sensor, lidos pela telemetria).
// Cada "Valido" vira false quando o sensor falha, e o JSON envia null no lugar do valor.
volatile float pesoAtual = 0.0f;
volatile bool  pesoValido = false;
volatile float temperaturaAtual = 0.0f;
volatile float umidadeAtual = 0.0f;
volatile bool  ahtValido = false;

enum EstadoCaixa { LIVRE, GRAVANDO };

// ---------------------------------------------------------------------
//  WebSocket (esp_websocket_client)
// ---------------------------------------------------------------------
// O callback roda na tarefa interna do cliente: só atualiza flags e imprime.
static void wsEventHandler(void *args, esp_event_base_t base, int32_t id, void *dados) {
  esp_websocket_event_data_t *ev = (esp_websocket_event_data_t *)dados;
  switch (id) {
    case WEBSOCKET_EVENT_CONNECTED:
      Serial.println("[WS] Conectado ao servidor");
      wsConectado = true;
      wsConexoes++;
      break;
    case WEBSOCKET_EVENT_DISCONNECTED:
      Serial.println("[WS] Desconectado do servidor");
      wsConectado = false;
      break;
    case WEBSOCKET_EVENT_ERROR:
      Serial.println("[WS] Erro de conexão");
      break;
    case WEBSOCKET_EVENT_DATA:
      if (ev->op_code == 0x01) { // texto
        Serial.printf("[WS] Servidor: %.*s\n", ev->data_len, (const char *)ev->data_ptr);
      }
      break;
    default:
      break;
  }
}

void wsIniciar() {
  static char uri[96];
  snprintf(uri, sizeof(uri), "ws://%s:%u%s", WS_HOST, WS_PORT, WS_PATH);

  esp_websocket_client_config_t cfg = {};
  cfg.uri = uri;
  cfg.ping_interval_sec = WS_PING_INTERVALO_S;
  cfg.pingpong_timeout_sec = WS_PING_TIMEOUT_S;

  wsClient = esp_websocket_client_init(&cfg);
  esp_websocket_register_events(wsClient, WEBSOCKET_EVENT_ANY, wsEventHandler, NULL);
  esp_websocket_client_start(wsClient); // reconecta sozinho se o servidor/WiFi cair
}

// Envios: o cliente tem lock interno, então podem ser feitos de qualquer tarefa.
// Retornam true se a rede aceitou os dados.
static bool wsEnviarTXT(const char *texto) {
  return esp_websocket_client_send_text(wsClient, texto, strlen(texto),
                                        pdMS_TO_TICKS(WS_TIMEOUT_ENVIO_MS)) >= 0;
}

static bool wsEnviarChunk(const ChunkAudio &chunk) {
  return esp_websocket_client_send_bin(wsClient, (const char *)chunk.dados,
                                       chunk.quantidade * sizeof(int16_t),
                                       pdMS_TO_TICKS(WS_TIMEOUT_ENVIO_MS)) >= 0;
}

// ---------------------------------------------------------------------
//  Envio do áudio: start_audio -> chunks -> stop_audio (ordem garantida,
//  pois tudo sai desta única tarefa)
// ---------------------------------------------------------------------
void tarefaEnvioAudio(void *parametro) {
  bool sessaoAberta = false;
  uint32_t conexoesVistas = 0;
  ChunkAudio chunk;

  for (;;) {
    // Houve (re)conexão: a sessão antiga não existe mais no servidor
    if (conexoesVistas != wsConexoes) {
      conexoesVistas = wsConexoes;
      sessaoAberta = false;
    }

    if (!wsConectado) {
      // Sem servidor: não acumula áudio velho para mandar depois da reconexão
      if (gravando) xQueueReset(filaAudio);
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    if (gravando && !sessaoAberta) {
      if (wsEnviarTXT("start_audio")) {
        sessaoAberta = true;
      } else {
        vTaskDelay(pdMS_TO_TICKS(100)); // rede recusou: tenta de novo
        continue;
      }
    }

    if (!sessaoAberta) {
      vTaskDelay(pdMS_TO_TICKS(20)); // nada para enviar
      continue;
    }

    // Espera até 20 ms por um chunk (acorda assim que ele chega)
    if (xQueueReceive(filaAudio, &chunk, pdMS_TO_TICKS(20)) == pdTRUE) {
      if (chunk.quantidade > 0 && !wsEnviarChunk(chunk)) {
        falhasEnvio++;
      }
    }

    // Gravação terminou e a fila esvaziou: fecha a sessão
    if (!gravando && uxQueueMessagesWaiting(filaAudio) == 0 && (millis() - fimGravacaoMs) > ATRASO_STOP_MS) {
      if (wsEnviarTXT("stop_audio")) {
        sessaoAberta = false;
      }
    }
  }
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
      chunksDescartados++;
    }
  }
}

// ---------------------------------------------------------------------
//  Gerenciamento da caixa: máquina de estados (sem I/O de rede)
// ---------------------------------------------------------------------
static EstadoCaixa estado = LIVRE;   // só esta tarefa lê/escreve
static int contConfirmaEntrada = 0;
static int contConfirmaSaida = 0;

static void iniciarGravacao(float peso) {
  Serial.printf("[GATO] Entrou na caixa (%.1f g). Iniciando gravação.\n", peso);
  i2s_zero_dma_buffer(I2S_PORT);  // descarta áudio velho que ficou no DMA
  xQueueReset(filaAudio);
  chunksDescartados = 0;
  falhasEnvio = 0;
  gravando = true;                // a tarefaEnvioAudio percebe e envia start_audio
  estado = GRAVANDO;
}

static void finalizarGravacao(float peso) {
  Serial.printf("[GATO] Saiu da caixa (%.1f g). Finalizando gravação.\n", peso);
  fimGravacaoMs = millis();
  gravando = false;               // a tarefaEnvioAudio esvazia a fila e envia stop_audio
  estado = LIVRE;
  Serial.printf("[AUDIO] Chunks descartados: %lu | falhas de envio: %lu\n",
                (unsigned long)chunksDescartados, (unsigned long)falhasEnvio);
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

void tarefaGerenciamentoCaixa(void *parametro) {
  float peso;
  for (;;) {
    // Dorme até chegar uma leitura nova de peso
    xQueueReceive(filaPeso, &peso, portMAX_DELAY);
    tratarPresenca(peso);
  }
}

// ---------------------------------------------------------------------
//  Leitura da célula de carga
// ---------------------------------------------------------------------
// Média de PESO_AMOSTRAS leituras. Espera o HX711 ficar pronto DORMINDO (delay de 5 ms),
// em vez do busy-wait interno do get_units(N), que ocuparia a CPU e atrasaria o WebSocket.
static bool lerPeso(float &peso) {
  float soma = 0;
  for (int i = 0; i < PESO_AMOSTRAS; i++) {
    if (!scale.wait_ready_timeout(1000, 5)) return false;
    soma += scale.get_units(1);
  }
  peso = soma / PESO_AMOSTRAS;
  return true;
}

void tarefaPeso(void *parametro) {
  for (;;) {
    float peso;
    if (lerPeso(peso)) {
      if (fabsf(peso) < ZERO_THRESHOLD) {
        peso = 0.0f;
      }
      pesoAtual = peso;
      pesoValido = true;
      if (xQueueSend(filaPeso, &peso, 0) != pdTRUE) {
        // fila cheia: descarta a leitura mais antiga, para o gerenciamento nunca ver dado velho
        float velho;
        xQueueReceive(filaPeso, &velho, 0);
        xQueueSend(filaPeso, &peso, 0);
      }
    } else {
      pesoValido = false;
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
        ahtValido = false;
        vTaskDelay(pdMS_TO_TICKS(5000));
        continue;
      }
    }

    sensors_event_t umidade, temperatura;
    if (aht.getEvent(&umidade, &temperatura)) {
      temperaturaAtual = temperatura.temperature;
      umidadeAtual = umidade.relative_humidity;
      ahtValido = true;
      Serial.printf("[AHT10] Temperatura: %.1f °C | Umidade: %.1f %%\n",
                    temperatura.temperature, umidade.relative_humidity);
    } else {
      Serial.println("[AHT10] Falha na leitura.");
      ahtValido = false;
      sensorOk = false; // força reinicialização na próxima volta
    }

    vTaskDelay(pdMS_TO_TICKS(INTERVALO_LEITURA_AHT_MS));
  }
}

static void formatarNumero(char *dst, size_t n, bool valido, float v, int casas) {
  if (valido && !isnan(v)) snprintf(dst, n, "%.*f", casas, v);
  else                     snprintf(dst, n, "null");
}

static int montarTelemetria(char *buf, size_t n) {
  // Copia local: os valores podem mudar durante a montagem
  bool pv = pesoValido, av = ahtValido, gato = gravando;
  float p = pesoAtual, t = temperaturaAtual, u = umidadeAtual;

  char sp[16], st[16], su[16];
  formatarNumero(sp, sizeof(sp), pv, p, 2);
  formatarNumero(st, sizeof(st), av, t, 1);
  formatarNumero(su, sizeof(su), av, u, 1);

  return snprintf(buf, n,
    "{\"tipo\":\"telemetria\",\"peso_g\":%s,\"temperatura_c\":%s,\"umidade_pct\":%s,"
    "\"gato_presente\":%s,\"uptime_s\":%lu}",
    sp, st, su, gato ? "true" : "false", (unsigned long)(millis() / 1000));
}

void tarefaTelemetria(void *parametro) {
  char json[192];
  TickType_t ultimoEnvio = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&ultimoEnvio, pdMS_TO_TICKS(INTERVALO_TELEMETRIA_MS)); // cadência fixa, sem acumular atraso
    if (!wsConectado) continue;

    int tam = montarTelemetria(json, sizeof(json));
    if (tam > 0 && tam < (int)sizeof(json)) {
      wsEnviarTXT(json); // se falhar, o próximo pacote sai no próximo ciclo
    }
  }
}

// ---------------------------------------------------------------------
//  setup / loop
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
  WiFi.setSleep(false); // sem economia de energia do rádio: evita picos de latência que travam o envio do áudio
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

  // ----- Filas e I2S -----
  filaAudio = xQueueCreate(FILA_AUDIO_TAMANHO, sizeof(ChunkAudio));
  filaPeso  = xQueueCreate(FILA_PESO_TAMANHO, sizeof(float));
  i2sInstalar();

  // ----- WebSocket -----
  wsIniciar();

  // ----- Tarefas (o ESP32-C3 tem um único núcleo) -----
  xTaskCreate(tarefaCapturaAudio,       "CapturaAudio", 6144, NULL, 3, NULL);
  xTaskCreate(tarefaEnvioAudio,         "EnvioAudio",   6144, NULL, 2, NULL);
  xTaskCreate(tarefaGerenciamentoCaixa, "GerCaixa",     4096, NULL, 2, NULL);
  xTaskCreate(tarefaPeso,               "LeituraPeso",  4096, NULL, 1, NULL);
  xTaskCreate(tarefaTemperatura,        "LeituraTemp",  4096, NULL, 1, NULL);
  xTaskCreate(tarefaTelemetria,         "Telemetria",   4096, NULL, 1, NULL);
}

void loop() {
  // Tudo roda em tarefas; o loop() não tem mais o que fazer.
  vTaskDelay(pdMS_TO_TICKS(1000));
}