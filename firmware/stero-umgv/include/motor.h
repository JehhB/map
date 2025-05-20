#pragma once

#include "esp_err.h"
#include "esp_http_server.h"
#include "hal/gpio_types.h"
#include "hal/ledc_types.h"

#define MOTOR_FALLOF_MS 500
#define FRAME_MOTOR_FORWARD 0x10
#define FRAME_MOTOR_BACKWARD 0x11

typedef struct {
  gpio_num_t pwm;
  gpio_num_t forward;
  gpio_num_t backward;
  ledc_channel_t ledcChannel;
} motor_config_t;

const motor_config_t DEFAULT_MOTOR = {
    .pwm = GPIO_NUM_15,
    .forward = GPIO_NUM_12,
    .backward = GPIO_NUM_13,
    .ledcChannel = LEDC_CHANNEL_1,
};

esp_err_t motorHandler(httpd_req_t *req, httpd_ws_frame_t *ws_pkt);
void setupMotor(const motor_config_t &config);

void updateMotor();
void motorTask(void *param);
void setMotor(int8_t speed, const motor_config_t &config);
