#include "ws.h"
#include "esp32-hal-log.h"
#include "esp_http_server.h"
#include <cstring>

esp_err_t ws_handler(httpd_req_t *req) {
  httpd_ws_frame_t ws_pkt;
  uint8_t static_buf[MAX_WS_FRAME_SIZE]; // Static buffer for frame payload
  esp_err_t ret;
  memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));

  if (req->method == HTTP_GET) {
    log_i("Handshake done, the new connection was opened");
    return ESP_OK;
  }

  ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
  if (ret != ESP_OK) {
    log_e("httpd_ws_recv_frame failed with %d", ret);
    return ret;
  }

  if (ws_pkt.len > MAX_WS_FRAME_SIZE) {
    log_e("Frame too large for static buffer: %d > %d", ws_pkt.len,
          MAX_WS_FRAME_SIZE);
    return ESP_ERR_INVALID_SIZE;
  }

  if (ws_pkt.len) {
    ws_pkt.payload = static_buf;

    ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
    if (ret != ESP_OK) {
      log_e("httpd_ws_recv_frame failed with %d", ret);
      return ret;
    }
  }

  for (size_t i = 0; i < ws_frame_handler_count; ++i) {
    frame_handler_t handler = ws_frame_handlers[i];

    if ((*handler)(req, &ws_pkt) == ESP_OK) {
      log_i("Frame handled by handler at index %d", i);
      return ESP_OK;
    }
  }

  log_w("No handler processed the frame (type: %d, len: %d)", ws_pkt.type,
        ws_pkt.len);

  if (ws_pkt.type == HTTPD_WS_TYPE_BINARY && ws_pkt.len >= 1) {
    log_w("No handler processed the frame (opcode: 0x%x)", ws_pkt.payload[0]);
  }

  return ESP_OK;
}
