#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "sensor.h"
#include <WiFi.h>

// Replace with your WiFi credentials
const char *ssid = "hotspot?";
const char *password = "12345678";

// Camera pin configuration
camera_config_t camera_config = {
    .pin_pwdn = GPIO_NUM_13,
    .pin_reset = GPIO_NUM_12,
    .pin_xclk = -1,
    .pin_sccb_sda = GPIO_NUM_8,
    .pin_sccb_scl = GPIO_NUM_9,
    .pin_d7 = GPIO_NUM_39,
    .pin_d6 = GPIO_NUM_38,
    .pin_d5 = GPIO_NUM_6,
    .pin_d4 = GPIO_NUM_5,
    .pin_d3 = GPIO_NUM_4,
    .pin_d2 = GPIO_NUM_3,
    .pin_d1 = GPIO_NUM_2,
    .pin_d0 = GPIO_NUM_1,
    .pin_vsync = GPIO_NUM_11,
    .pin_href = GPIO_NUM_10,
    .pin_pclk = GPIO_NUM_7,
    .xclk_freq_hz = 12000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_VGA,
    .jpeg_quality = 12,
    .fb_count = 2,
    .grab_mode = CAMERA_GRAB_LATEST,
};

// Handler for the "/capture" endpoint
esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                        "Camera capture failed");
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition",
                     "inline; filename=capture.jpg");
  esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

httpd_uri_t capture_uri = {.uri = "/capture",
                           .method = HTTP_GET,
                           .handler = capture_handler,
                           .user_ctx = NULL};

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

esp_err_t jpg_stream_httpd_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len;
  uint8_t *_jpg_buf;
  char *part_buf[64];
  static int64_t last_frame = 0;
  if (!last_frame) {
    last_frame = esp_timer_get_time();
  }

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      res = ESP_FAIL;
      break;
    }
    if (fb->format != PIXFORMAT_JPEG) {
      bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
      if (!jpeg_converted) {
        esp_camera_fb_return(fb);
        res = ESP_FAIL;
      }
    } else {
      _jpg_buf_len = fb->len;
      _jpg_buf = fb->buf;
    }

    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY,
                                  strlen(_STREAM_BOUNDARY));
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);

      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (fb->format != PIXFORMAT_JPEG) {
      free(_jpg_buf);
    }
    esp_camera_fb_return(fb);
    if (res != ESP_OK) {
      break;
    }
    int64_t fr_end = esp_timer_get_time();
    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;
    frame_time /= 1000;
    ESP_LOGI(TAG, "MJPG: %uKB %ums (%.1ffps)", (uint32_t)(_jpg_buf_len / 1024),
             (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time);
  }

  last_frame = 0;
  return res;
}

httpd_uri_t stream_uri = {.uri = "/stream",
                          .method = HTTP_GET,
                          .handler = jpg_stream_httpd_handler,
                          .user_ctx = NULL};

esp_err_t root_handler(httpd_req_t *req) {
  const char resp[] = "ESP32S3 Camera HTTP Server using esp_http_server";
  httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

httpd_uri_t root_uri = {
    .uri = "/", .method = HTTP_GET, .handler = root_handler, .user_ctx = NULL};

httpd_handle_t startCameraServer() {
  httpd_handle_t server = NULL;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_register_uri_handler(server, &root_uri);
    httpd_register_uri_handler(server, &capture_uri);
    httpd_register_uri_handler(server, &stream_uri);
  }
  return server;
}

void setup() {
  Serial.begin(115200);

  // Initialize the camera
  if (esp_camera_init(&camera_config) != ESP_OK) {
    Serial.println("Camera init failed");
    return;
  }

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi connected, IP: ");
  Serial.println(WiFi.localIP());

  // Start HTTP server
  startCameraServer();
}

void loop() {
  // Nothing needed here; HTTP server runs in background.
};
