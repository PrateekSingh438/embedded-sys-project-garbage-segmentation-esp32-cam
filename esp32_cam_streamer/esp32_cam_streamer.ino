#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"

// Wi-Fi network created by the ESP32-CAM. Connect the computer running
// main.py to this access point.
const char *ssid = "GarbageRobot";
// Set a private password (at least 8 characters) before flashing.
const char *password = "CHANGE_ME";

httpd_handle_t stream_server = nullptr;

// Stream camera frames as MJPEG at /stream.
static esp_err_t stream_handler(httpd_req_t *req) {
  esp_err_t result = httpd_resp_set_type(
      req, "multipart/x-mixed-replace;boundary=frame");
  if (result != ESP_OK) return result;

  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  const char boundary[] = "\r\n--frame\r\n";

  while (true) {
    camera_fb_t *frame = esp_camera_fb_get();
    if (frame == nullptr) {
      Serial.println("Camera capture failed");
      return ESP_FAIL;
    }

    char header[100];
    int header_len = snprintf(
        header, sizeof(header),
        "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
        static_cast<unsigned int>(frame->len));

    if (header_len < 0 || header_len >= static_cast<int>(sizeof(header))) {
      esp_camera_fb_return(frame);
      return ESP_FAIL;
    }

    result = httpd_resp_send_chunk(req, boundary, sizeof(boundary) - 1);
    if (result == ESP_OK)
      result = httpd_resp_send_chunk(req, header, header_len);
    if (result == ESP_OK)
      result = httpd_resp_send_chunk(
          req, reinterpret_cast<const char *>(frame->buf), frame->len);

    esp_camera_fb_return(frame);
    if (result != ESP_OK) {
      Serial.println("Video viewer disconnected");
      return result;
    }
    delay(1);
  }
}

bool start_stream_server() {
  httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
  server_config.server_port = 81;
  server_config.stack_size = 8192;

  esp_err_t result = httpd_start(&stream_server, &server_config);
  if (result != ESP_OK) {
    Serial.printf("Server start failed: 0x%x\n", result);
    return false;
  }

  httpd_uri_t stream_uri = {};
  stream_uri.uri = "/stream";
  stream_uri.method = HTTP_GET;
  stream_uri.handler = stream_handler;

  result = httpd_register_uri_handler(stream_server, &stream_uri);
  if (result != ESP_OK) {
    Serial.printf("Stream registration failed: 0x%x\n", result);
    httpd_stop(stream_server);
    stream_server = nullptr;
    return false;
  }
  return true;
}

bool setup_camera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  // AI-Thinker ESP32-CAM pin configuration.
  config.pin_d0 = 5;
  config.pin_d1 = 18;
  config.pin_d2 = 19;
  config.pin_d3 = 21;
  config.pin_d4 = 36;
  config.pin_d5 = 39;
  config.pin_d6 = 34;
  config.pin_d7 = 35;
  config.pin_xclk = 0;
  config.pin_pclk = 22;
  config.pin_vsync = 25;
  config.pin_href = 23;
  config.pin_sccb_sda = 26;
  config.pin_sccb_scl = 27;
  config.pin_pwdn = 32;
  config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    Serial.println("PSRAM detected: using 640 x 480");
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    Serial.println("No PSRAM: using 320 x 240");
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 15;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t result = esp_camera_init(&config);
  if (result != ESP_OK) {
    Serial.printf("Camera initialization failed: 0x%x\n", result);
    return false;
  }
  Serial.println("Camera initialized");
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Keep the white flash LED off.
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);

  if (!setup_camera()) return;

  WiFi.mode(WIFI_AP);
  IPAddress local_ip(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  if (!WiFi.softAPConfig(local_ip, gateway, subnet)) {
    Serial.println("Failed to configure access point IP");
    return;
  }
  if (!WiFi.softAP(ssid, password)) {
    Serial.println("Failed to create Wi-Fi network");
    return;
  }

  Serial.println("Wi-Fi network created");
  Serial.print("Network name: ");
  Serial.println(ssid);
  if (!start_stream_server()) return;

  Serial.println("Connect your computer to the Wi-Fi network, then open:");
  Serial.print("http://");
  Serial.print(WiFi.softAPIP());
  Serial.println(":81/stream");
}

void loop() { delay(1000); }
