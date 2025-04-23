#pragma once

#include "esp_err.h"
#include "esp_http_server.h"
#include "hal/gpio_types.h"
#include "hal/ledc_types.h"

#define MOTOR_FALLOF_MS 300

typedef struct {
  gpio_num_t pwm;
  gpio_num_t forward;
  gpio_num_t backward;
  ledc_channel_t ledcChannel;
} motor_config_t;

typedef struct {
  motor_config_t left;
  motor_config_t right;
} motors_t;

const motors_t DEFAULT_MOTORS = {
    .left = {GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_15, LEDC_CHANNEL_1},
    .right = {GPIO_NUM_14, GPIO_NUM_2, GPIO_NUM_0, LEDC_CHANNEL_2},
};

esp_err_t motorHandler(httpd_req_t *req);
void setupMotors(const motors_t &config);

void motorTask(void *params);
void updateMotor();
void setMotorL(int8_t speed, const motors_t &config);
void setMotorR(int8_t speed, const motors_t &config);
