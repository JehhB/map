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
  int8_t speed_left;
  int8_t speed_right;
  int64_t updated_at;
} motor_state_t;

motor_state_t globalMotorState = {
    .speed_left = 0,
    .speed_right = 0,
    .updated_at = 0,
};

void updateMotor() {
  int64_t time = millis();

  if (time - globalMotorState.updated_at < MOTOR_FALLOF_MS) {
    setMotorL(globalMotorState.speed_left, DEFAULT_MOTORS);
    setMotorR(globalMotorState.speed_right, DEFAULT_MOTORS);
  } else {
    setMotorL(0, DEFAULT_MOTORS);
    setMotorR(0, DEFAULT_MOTORS);
  }

  /*
  Serial.printf("Elapsed: %lld\tLeft: %d\tRight: %d\n",
                time - globalMotorState.updated_at, globalMotorState.speed_left,
                globalMotorState.speed_right);
                */
}

esp_err_t motorHandler(httpd_req_t *req) {
  char query[127];
  char value[15];

  bool isUpdated = false;

  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  if (httpd_query_key_value(query, "l", value, sizeof(value)) == ESP_OK) {
    isUpdated = true;
    globalMotorState.speed_left = strtol(value, NULL, 10);
  }

  if (httpd_query_key_value(query, "r", value, sizeof(value)) == ESP_OK) {
    isUpdated = true;
    globalMotorState.speed_right = strtol(value, NULL, 10);
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

  gpio_config_t io_conf = {};
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.intr_type = GPIO_INTR_DISABLE;

  io_conf.pin_bit_mask = (1ULL << config.forward);
  gpio_config(&io_conf);

  io_conf.pin_bit_mask = (1ULL << config.backward);
  gpio_config(&io_conf);
}

void setupMotors(const motors_t &config) {
  setupMotor(config.left);
  setupMotor(config.right);
}

void setMotorL(int8_t speed, const motors_t &config) {
  if (speed < 0) {
    ledcWrite(config.left.ledcChannel, -speed * 2 - 1);
    gpio_set_level(config.left.forward, 0);
    gpio_set_level(config.left.backward, 1);
  } else {
    ledcWrite(config.left.ledcChannel, speed * 2);
    gpio_set_level(config.left.backward, 0);
    gpio_set_level(config.left.forward, 1);
  }
}

void setMotorR(int8_t speed, const motors_t &config) {
  if (speed < 0) {
    ledcWrite(config.right.ledcChannel, -speed * 2 - 1);
    gpio_set_level(config.right.forward, 0);
    gpio_set_level(config.right.backward, 1);
  } else {
    ledcWrite(config.right.ledcChannel, speed * 2);
    gpio_set_level(config.right.backward, 0);
    gpio_set_level(config.right.forward, 1);
  }
}
