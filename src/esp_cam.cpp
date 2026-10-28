/*
 * ESP32-S3-CAM + OV3660  ->  stream JPEG por TCP bruto
 *
 * Protocolo (por frame):
 *     [uint32 little-endian: tamanho N][N bytes de JPEG]
 *
 * Por que TCP bruto com prefixo de tamanho?
 *  - Sem HTTP/multipart: zero parsing de cabeçalhos/boundary, overhead de 4 bytes.
 *  - Sem biblioteca de servidor: só um socket + loop em uma única task.
 *  - TCP_NODELAY desliga o algoritmo de Nagle (sem atraso de agregação).
 *  - Diferente de UDP, não perde fragmentos de um JPEG (um fragmento perdido
 *    corrompe o frame inteiro e exigiria reassembly/retransmissão manual).
 *  - A "frescura" do vídeo vem de CAMERA_GRAB_LATEST: se a rede atrasar, os
 *    frames antigos são descartados no driver e sempre enviamos o mais recente.
 */

#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"

// ====================== CONFIGURAÇÃO DO USUÁRIO ======================
#define WIFI_SSID      "ssid"
#define WIFI_PASS      "password"
#define TCP_PORT       5000

#define FRAME_SIZE     FRAMESIZE_QVGA   // FRAMESIZE_QVGA (320x240) ou FRAMESIZE_VGA (640x480)
#define JPEG_QUALITY   20              // 0-63, MENOR = melhor qualidade/maior arquivo
#define XCLK_HZ        22000000        // 20 MHz (OV3660 aceita até ~24 MHz)
#define FB_COUNT       2               // 2 buffers em PSRAM: captura em paralelo ao envio
#define FLIP_VERTICAL  1               // OV3660 costuma sair de cabeça p/ baixo: ponha 1 se necessário
// =====================================================================

// ---------- Pinout ESP32-S3-CAM (OV2640/OV3660, padrão Freenove / ESP32-S3-EYE) ----------
// Confira com o esquemático da SUA placa; clones podem variar.
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   15
#define SIOD_GPIO_NUM    4   // SCCB SDA
#define SIOC_GPIO_NUM    5   // SCCB SCL
#define Y9_GPIO_NUM     16   // D7
#define Y8_GPIO_NUM     17   // D6
#define Y7_GPIO_NUM     18   // D5
#define Y6_GPIO_NUM     12   // D4
#define Y5_GPIO_NUM     10   // D3
#define Y4_GPIO_NUM      8   // D2
#define Y3_GPIO_NUM      9   // D1
#define Y2_GPIO_NUM     11   // D0
#define VSYNC_GPIO_NUM   6
#define HREF_GPIO_NUM    7
#define PCLK_GPIO_NUM   13

WiFiServer server(TCP_PORT);

static bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;

  c.xclk_freq_hz = XCLK_HZ;
  c.pixel_format = PIXFORMAT_JPEG;          // JPEG por hardware do sensor: CPU livre
  c.frame_size   = FRAME_SIZE;
  c.jpeg_quality = JPEG_QUALITY;
  c.fb_count     = FB_COUNT;
  c.fb_location  = CAMERA_FB_IN_PSRAM;
  c.grab_mode    = CAMERA_GRAB_LATEST;      // descarta frames velhos -> menor latência

  if (esp_camera_init(&c) != ESP_OK) return false;

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    if (FLIP_VERTICAL) s->set_vflip(s, 1);
    // OV3660: pequeno ajuste de brilho/saturação inicial (opcional)
    if (s->id.PID == OV3660_PID) {
      s->set_brightness(s, 1);
      s->set_saturation(s, -2);
    }
  }
  return true;
}

void setup() {
  Serial.begin(115200);

  if (!initCamera()) {
    Serial.println("Falha ao iniciar a camera");
    delay(2000);
    ESP.restart();
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                     // CRÍTICO: desliga power-save do Wi-Fi (corta dezenas de ms)
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  while (WiFi.status() != WL_CONNECTED) delay(100);

  server.begin();
  server.setNoDelay(true);
  Serial.printf("Pronto: %s:%d\n", WiFi.localIP().toString().c_str(), TCP_PORT);
}

void loop() {
  WiFiClient client = server.accept();      // aguarda o receiver conectar
  if (!client) { delay(5); return; }

  client.setNoDelay(true);                  // desativa Nagle
  Serial.println("Cliente conectado");

  uint32_t frames = 0, t0 = millis();

  while (client.connected()) {
    camera_fb_t *fb = esp_camera_fb_get();  // frame mais recente (GRAB_LATEST)
    if (!fb) continue;

    uint32_t len = fb->len;
    bool ok = client.write((const uint8_t *)&len, 4) == 4 &&
              client.write(fb->buf, len) == len;
    esp_camera_fb_return(fb);               // devolve o buffer ao driver imediatamente
    if (!ok) break;

    if (++frames % 100 == 0) {
      Serial.printf("TX %.1f fps\n", 100000.0f / (millis() - t0));
      t0 = millis();
    }
  }

  client.stop();
  Serial.println("Cliente desconectado");
}
