#include "camera.h"
#include "esp32-hal-ledc.h"
#include "esp_err.h"
#include "esp_http_server.h"
#include "http_parser.h"
#include "motor.h"
#include "ws.h"
#include <WiFi.h>
#include <cstddef>

const char *ssid = "RatmapAP";
const char *password = "Ratmap19";

esp_err_t ok_handler(httpd_req_t *req) {
  const char *resp_str = "ok";
  httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

const frame_handler_t ws_frame_handlers[] = {
#ifdef MOTOR_CONTROLS
    motorHandler,
#endif
    flashHandler,
    ping_frame_handler,
};

extern const size_t ws_frame_handler_count;

httpd_handle_t startServer() {
  httpd_handle_t server = NULL;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = ok_handler,
        .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &index_uri);

    static const httpd_uri_t ws = {.uri = "/ws",
                                   .method = HTTP_GET,
                                   .handler = ws_handler,
                                   .user_ctx = NULL,
                                   .is_websocket = true};
  }
  return server;
}

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  setupLedFlash(LED_GPIO_NUM);
#ifdef MOTOR_CONTROLS
  setupMotor(DEFAULT_MOTOR);
#endif

  esp_err_t result = setupCamera();

  if (result == ESP_OK) {
    ledcWrite(LEDC_CHANNEL_7, 8);
    delay(200);
    ledcWrite(LEDC_CHANNEL_7, 0);
    delay(200);
  }

  Serial.println();
  Serial.print("WiFi connected, IP: ");
  Serial.println(WiFi.localIP());

  startServer();
  startStreamServer();

  ledcWrite(LEDC_CHANNEL_7, 8);
  delay(200);
  ledcWrite(LEDC_CHANNEL_7, 0);
}

void loop() {
  updateMotor();
  vTaskDelay(pdMS_TO_TICKS(50));
};
