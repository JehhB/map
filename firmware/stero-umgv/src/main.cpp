#include "camera.h"
#include "esp_http_server.h"
#include <Arduino.h>
#include <WiFi.h>

const char *ssid = "hotspot?";
const char *password = "12345678";

httpd_uri_t stream_uri = {.uri = "/stream",
                          .method = HTTP_GET,
                          .handler = streamHandler,
                          .user_ctx = NULL};

httpd_handle_t startServer() {
  httpd_handle_t server = NULL;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_register_uri_handler(server, &stream_uri);
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
  Serial.println();
  Serial.print("WiFi connected, IP: ");
  Serial.println(WiFi.localIP());

  startServer();
}

void loop() {
  // Nothing needed here; HTTP server runs in background.
};
