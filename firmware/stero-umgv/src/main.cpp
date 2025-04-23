#include "camera.h"
#include "esp32-hal-ledc.h"
#include "esp_http_server.h"
#include "http_parser.h"
#include "motor.h"
#include "soc/soc.h"
#include <WiFi.h>

const char *ssid = "hotspot?";
const char *password = "12345678";

httpd_handle_t startServer() {
  httpd_handle_t server = NULL;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  if (httpd_start(&server, &config) == ESP_OK) {
#ifdef MOTOR_CONTROLS
    httpd_uri_t motor_uri = {.uri = "/motor",
                             .method = HTTP_GET,
                             .handler = motorHandler,
                             .user_ctx = NULL};
    httpd_register_uri_handler(server, &motor_uri);
#endif
  }
  return server;
}

void setup() {
  Serial.begin(115200);

  setupCamera();
  setupLedFlash(LED_GPIO_NUM);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  ledcWrite(LEDC_CHANNEL_7, 128);
  delay(200);
  ledcWrite(LEDC_CHANNEL_7, 0);

  Serial.println();
  Serial.print("WiFi connected, IP: ");
  Serial.println(WiFi.localIP());

#ifdef MOTOR_CONTROLS
  setupMotors(DEFAULT_MOTORS);
#endif

  startServer();
  startStreamServer();
}

void loop() {
  // Nothing needed here; HTTP server runs in background.
#ifdef MOTOR_CONTROLS
  updateMotor();
#endif
  delay(100);
};
