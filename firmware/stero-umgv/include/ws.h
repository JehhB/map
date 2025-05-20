#pragma once

#include "esp_err.h"
#include "esp_http_server.h"
#include <cstddef>

#define MAX_WS_FRAME_SIZE 8

typedef esp_err_t (*frame_handler_t)(httpd_req_t *req,
                                     httpd_ws_frame_t *ws_pkt);

extern const frame_handler_t ws_frame_handlers[];
extern const size_t ws_frame_handler_count;

esp_err_t ping_frame_handler(httpd_req_t *req, httpd_ws_frame_t *ws_pkt);
esp_err_t ws_handler(httpd_req_t *req);
