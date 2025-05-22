#pragma once

#include "esp_err.h"
#include "esp_http_server.h"
#include "hal/gpio_types.h"
#include "hal/ledc_types.h"

#define MOTOR_FALLOF_MS 500
#define FRAME_MOTOR_UPDATE 0x10

typedef struct {
  gpio_num_t pwm;
  ledc_channel_t ledcChannel;
  gpio_num_t forward;
  gpio_num_t backward;
} motor_config_t;

const motor_config_t DEFAULT_MOTOR = {
    .pwm = GPIO_NUM_13,
    .ledcChannel = LEDC_CHANNEL_1,
    .forward = GPIO_NUM_15,
    .backward = GPIO_NUM_14,
};

esp_err_t motorHandler(httpd_req_t *req, httpd_ws_frame_t *ws_pkt);
void setupMotor(const motor_config_t &config);

void updateMotor();
void motorTask(void *param);
void setMotor(int8_t speed, const motor_config_t &config);
