#pragma once

#include "camera_pins.h"
#include "esp_err.h"
#include "esp_http_server.h"

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: "
                                  "%u\r\nX-Timestamp: %d.%06d\r\n\r\n";
#define FRAME_FLASHLIGHT 0x20

esp_err_t setupCamera();
httpd_handle_t startStreamServer();

void setupLedFlash(int pin);
esp_err_t flashHandler(httpd_req_t *req, httpd_ws_frame_t *ws_pkt);
