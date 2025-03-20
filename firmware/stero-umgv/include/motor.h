#pragma once

#include "esp_err.h"
#include "esp_http_server.h"
#include "hal/gpio_types.h"
#include "hal/ledc_types.h"

#define MOTOR_FALLOF_MS 300

#define MOTOR_IN1 12
#define MOTOR_IN2 13
#define MOTOR_IN3 15
#define MOTOR_IN4 14

typedef struct {
  gpio_num_t pin;
  ledc_channel_t ledcChannel;
} motor_config_t;

typedef struct {
  motor_config_t leftForward;
  motor_config_t leftBackward;
  motor_config_t rightForward;
  motor_config_t rightBackward;
} motors_t;

const motors_t DEFAULT_MOTORS = {
    .leftForward = {GPIO_NUM_12, LEDC_CHANNEL_1},
    .leftBackward = {GPIO_NUM_13, LEDC_CHANNEL_2},
    .rightForward = {GPIO_NUM_15, LEDC_CHANNEL_3},
    .rightBackward = {GPIO_NUM_14, LEDC_CHANNEL_4},
};

esp_err_t motorHandler(httpd_req_t *req);
void setupMotors(const motors_t &config);

void motorTask(void *params);
void setMotorL(int8_t speed, const motors_t &config);
void setMotorR(int8_t speed, const motors_t &config);
