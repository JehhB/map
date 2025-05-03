#include "motor.h"
#include "esp32-hal-ledc.h"
#include "esp_err.h"
#include "esp_http_server.h"
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
  ledcAttachPin(config.pin, config.ledcChannel);
}

void setupMotors(const motors_t &config) {
  setupMotor(config.left_forward);
  setupMotor(config.right_forward);
  setupMotor(config.left_backward);
  setupMotor(config.right_backward);
}

void setMotorL(int8_t speed, const motors_t &config) {
  if (speed < 0) {
    ledcWrite(config.left_backward.ledcChannel, -speed * 2 - 1);
    ledcWrite(config.left_forward.ledcChannel, 0);
  } else {
    ledcWrite(config.left_forward.ledcChannel, speed * 2 + 1);
    ledcWrite(config.left_backward.ledcChannel, 0);
  }
}

void setMotorR(int8_t speed, const motors_t &config) {
  if (speed < 0) {
    ledcWrite(config.right_backward.ledcChannel, -speed * 2 - 1);
    ledcWrite(config.right_forward.ledcChannel, 0);
  } else {
    ledcWrite(config.right_forward.ledcChannel, speed * 2 + 1);
    ledcWrite(config.right_backward.ledcChannel, 0);
  }
}
