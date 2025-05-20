#include "camera.h"
#include "esp32-hal-ledc.h"
#include "esp32-hal-log.h"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "hal/ledc_types.h"
#include "sensor.h"

#define FPS_CAP 25
#define FPS_CAP_STRING "25"

esp_err_t streamHandler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  struct timeval _timestamp;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[128];
  static int64_t last_frame = 0;

  const int64_t min_frame_time_us = 1000000 / FPS_CAP;

  if (!last_frame) {
    last_frame = esp_timer_get_time();
  }

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", FPS_CAP_STRING);

  while (true) {
    int64_t current_time = esp_timer_get_time();
    int64_t elapsed_since_last = current_time - last_frame;

    if (elapsed_since_last < min_frame_time_us) {
      int64_t delay_time_ms = (min_frame_time_us - elapsed_since_last) / 1000;
      if (delay_time_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(delay_time_ms));
      }
      current_time = esp_timer_get_time();
    }

    fb = esp_camera_fb_get();
    if (!fb) {
      log_e("Camera capture failed");
      res = ESP_FAIL;
    } else {
      _timestamp.tv_sec = fb->timestamp.tv_sec;
      _timestamp.tv_usec = fb->timestamp.tv_usec;

      if (fb->format != PIXFORMAT_JPEG) {
        bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        esp_camera_fb_return(fb);
        fb = NULL;
        if (!jpeg_converted) {
          log_e("JPEG compression failed");
          res = ESP_FAIL;
        }
      } else {
        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;
      }
    }

    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY,
                                  strlen(_STREAM_BOUNDARY));
    }

    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 128, _STREAM_PART, _jpg_buf_len,
                             _timestamp.tv_sec, _timestamp.tv_usec);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }

    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }

    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }

    if (res != ESP_OK) {
      log_e("Send frame failed");
      break;
    }

    int64_t frame_end = esp_timer_get_time();
    int64_t frame_time = frame_end - last_frame;
    last_frame = frame_end;

    frame_time /= 1000;

    log_i("MJPG: %uB %ums (%.1ffps)", (uint32_t)(_jpg_buf_len),
          (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time);
  }

  return res;
}

esp_err_t setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_SVGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.jpeg_quality = 11;
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_DRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    log_e("Camera init failed with error 0x%x", err);
    return err;
  }

  return ESP_OK;
}

void setupLedFlash(int pin) {
  ledcSetup(LEDC_CHANNEL_7, 5000, 8);
  ledcAttachPin(pin, LEDC_CHANNEL_7);
}

esp_err_t flashHandler(httpd_req_t *req, httpd_ws_frame_t *ws_pkt) {
  if (ws_pkt->type != HTTPD_WS_TYPE_BINARY || ws_pkt->len < 2) {
    return ESP_FAIL;
  }

  uint8_t opcode = ((uint8_t *)ws_pkt->payload)[0];

  if (opcode != FRAME_FLASHLIGHT) {
    return ESP_FAIL;
  }

  uint8_t duty_cycle = ((uint8_t *)ws_pkt->payload)[1];

  // Set flashlight duty cycle
  ledcWrite(LEDC_CHANNEL_7, duty_cycle);

  log_i("Updated flashlight duty cycle: %d", duty_cycle);

  // Send acknowledgment
  const char *resp = "Flashlight updated";
  httpd_ws_frame_t resp_frame;
  memset(&resp_frame, 0, sizeof(httpd_ws_frame_t));
  resp_frame.type = HTTPD_WS_TYPE_TEXT;
  resp_frame.payload = (uint8_t *)resp;
  resp_frame.len = strlen(resp);

  esp_err_t ret = httpd_ws_send_frame(req, &resp_frame);
  if (ret != ESP_OK) {
    log_e("Failed to send response: %d", ret);
  }

  return ESP_OK;
}

httpd_handle_t startStreamServer() {
  httpd_handle_t server = NULL;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port += 1;
  config.ctrl_port += 1;

  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_uri_t stream_uri = {.uri = "/",
                              .method = HTTP_GET,
                              .handler = streamHandler,
                              .user_ctx = NULL};
    httpd_register_uri_handler(server, &stream_uri);
  }

  return server;
}
