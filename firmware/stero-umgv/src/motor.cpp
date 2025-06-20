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

esp_err_t motorHandler(httpd_req_t *req) {
  char query[127];
  char value[15];

  bool isUpdated = false;

  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  if (httpd_query_key_value(query, "s", value, sizeof(value)) == ESP_OK) {
    isUpdated = true;
    globalMotorState.speed = strtol(value, NULL, 10);
  }

  if (isUpdated) {
    globalMotorState.updated_at = millis();
    httpd_resp_sendstr(req, "Updated motor state");
  } else {
    httpd_resp_sendstr(req, "No updates");
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

    log_i("Updated speed (Backward: %d)", -speed * 2 - 1, 0, 1);
  } else {
    ledcWrite(config.ledcChannel, speed * 2 + 1);
    gpio_set_level(config.backward, 0);
    gpio_set_level(config.forward, 1);

    log_i("Updated speed (Forward: %d)", speed * 2 + 1, );
  }
}
