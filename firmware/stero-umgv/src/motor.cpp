#include "motor.h"
#include "driver/gpio.h"
#include "esp32-hal-ledc.h"
#include "esp_err.h"
#include "esp_http_server.h"
#include "hal/gpio_types.h"
#include <Arduino.h>
#include <cstdint>
#include <cstdlib>

typedef struct {
  int8_t speed;
  int64_t updated_at;
} motor_state_t;

motor_state_t globalMotorState = {
    .speed = 0,
    .updated_at = 0,
};

void updateMotor() {
  int64_t time = millis();

  if (time - globalMotorState.updated_at < MOTOR_FALLOF_MS) {
    setMotor(globalMotorState.speed, DEFAULT_MOTOR);
  } else {
    setMotor(0, DEFAULT_MOTOR);
  }
}

void motorTask(void *param) {
  while (1) {
    updateMotor();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

esp_err_t motorHandler(httpd_req_t *req, httpd_ws_frame_t *ws_pkt) {
  if (ws_pkt->type != HTTPD_WS_TYPE_BINARY || ws_pkt->len < 2) {
    return ESP_FAIL;
  }

  uint8_t opcode = ((uint8_t *)ws_pkt->payload)[0];

  if (opcode != FRAME_MOTOR_UPDATE) {
    return ESP_FAIL;
  }

  int8_t speed = ((int8_t *)ws_pkt->payload)[1];

  globalMotorState.speed = speed;
  globalMotorState.updated_at = millis();

  log_i("Updated motor speed: %d", speed);

  const uint8_t resp[] = {0};
  httpd_ws_frame_t resp_frame;
  memset(&resp_frame, 0, sizeof(httpd_ws_frame_t));
  resp_frame.type = HTTPD_WS_TYPE_BINARY;
  resp_frame.payload = (uint8_t *)resp;
  resp_frame.len = 1;

  esp_err_t ret = httpd_ws_send_frame(req, &resp_frame);

  if (ret != ESP_OK) {
    log_e("Failed to send response: %d", ret);
  }

  return ESP_OK;
}

void setupMotor(const motor_config_t &config) {
  ledcSetup(config.ledcChannel, 3000, 8);
  ledcAttachPin(config.pwm, config.ledcChannel);
  ledcWrite(config.ledcChannel, 0);

  gpio_config_t gpio_conf = {
      .pin_bit_mask = (1ULL << config.forward) | (1ULL << config.backward),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  gpio_config(&gpio_conf);
  gpio_set_level(config.forward, 0);
  gpio_set_level(config.backward, 0);
}

void setMotor(int8_t speed, const motor_config_t &config) {
  if (speed < 0) {
    ledcWrite(config.ledcChannel, -speed * 2 - 1);
    gpio_set_level(config.forward, 0);
    gpio_set_level(config.backward, 1);
  } else {
    ledcWrite(config.ledcChannel, speed * 2 + 1);
    gpio_set_level(config.backward, 0);
    gpio_set_level(config.forward, 1);
  }
}
