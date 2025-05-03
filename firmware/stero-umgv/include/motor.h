#pragma once

#include "esp_err.h"
#include "esp_http_server.h"
#include "hal/gpio_types.h"
#include "hal/ledc_types.h"

#define MOTOR_FALLOF_MS 500

typedef struct {
  gpio_num_t pin;
  ledc_channel_t ledcChannel;
} motor_config_t;

typedef struct {
  motor_config_t left_forward;
  motor_config_t left_backward;
  motor_config_t right_forward;
  motor_config_t right_backward;
} motors_t;

const motors_t DEFAULT_MOTORS = {
    .left_forward = {GPIO_NUM_12, LEDC_CHANNEL_1},
    .left_backward = {GPIO_NUM_13, LEDC_CHANNEL_2},
    .right_forward = {GPIO_NUM_14, LEDC_CHANNEL_3},
    .right_backward = {GPIO_NUM_15, LEDC_CHANNEL_4},
};

esp_err_t motorHandler(httpd_req_t *req);
void setupMotors(const motors_t &config);

void updateMotor();
void motorTask(void *param);
void setMotorL(int8_t speed, const motors_t &config);
void setMotorR(int8_t speed, const motors_t &config);
