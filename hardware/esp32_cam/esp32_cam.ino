#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "secrets.h"

#define DISABLE_BROWNOUT 1

// ---------------- Stream tuning ----------------
#define STREAM_DELAY_MS 50 // ~15-18 fps smooth streaming

// Default to Preset 1 (320x240) for lag-free video and fast YOLO inference
#define DEFAULT_PRESET 1
int currentPreset = DEFAULT_PRESET;

// AI Thinker ESP32-CAM camera pins
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// ESP32 UART2: RX=GPIO13 (from Uno D1), TX=GPIO14 (to Uno D0)
HardwareSerial RobotSerial(2);
const int ROBOT_RX = 13;
const int ROBOT_TX = 14;

WebServer server(80);
WiFiServer streamServer(81);

String gpsData = "";
String gpsLine = "";
unsigned long lastGpsMillis = 0;

void readRobotSerial()
{
  while (RobotSerial.available())
  {
    char c = (char)RobotSerial.read();

    if (c == '\n')
    {
      gpsLine.trim();

      if (gpsLine.startsWith("GPS,"))
      {
        gpsData = gpsLine;
        lastGpsMillis = millis();
      }

      gpsLine = "";
    }
    else if (c != '\r')
    {
      if (gpsLine.length() < 120)
      {
        gpsLine += c;
      }
      else
      {
        gpsLine = "";
      }
    }
  }
}

String currentGps()
{
  if (lastGpsMillis == 0 || millis() - lastGpsMillis > 6000)
  {
    return "GPS: no data from Arduino";
  }
  return gpsData;
}

void handleRoot()
{
  String page = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>GarbageBot</title>
<style>
body{font-family:Arial;text-align:center;background:#f2f2f2}
img{width:95%;max-width:640px}
</style>
</head>
<body>
<h2>GarbageBot Camera</h2>
<p>Close this tab before running camera.py (the stream supports one viewer).</p>
<img src="http://192.168.4.1:81/stream">
<p id="gps">GPS: waiting</p>
<script>
async function updateGPS(){
 try{
  const r=await fetch('/gps');
  document.getElementById('gps').innerText=await r.text();
 }catch(e){}
}
setInterval(updateGPS,2000);
</script>
</body>
</html>
)rawliteral";

  server.sendHeader("Connection", "close");
  server.send(200, "text/html", page);
}

void handleCommand()
{
  if (!server.hasArg("c") || server.arg("c").length() == 0)
  {
    server.sendHeader("Connection", "close");
    server.send(400, "text/plain", "Missing command");
    return;
  }

  char c = server.arg("c")[0];

  if (c != 'F' && c != 'B' && c != 'L' &&
      c != 'R' && c != 'S' &&
      !(c >= '1' && c <= '9'))
  {
    server.sendHeader("Connection", "close");
    server.send(400, "text/plain", "Invalid command");
    return;
  }

  RobotSerial.write(c);

  String reply = "OK\n";
  reply += currentGps();
  reply += "\nUP=";
  reply += String(millis() / 1000);
  reply += "\nRES=";
  reply += String(currentPreset);
  reply += "\n";

  server.sendHeader("Connection", "close");
  server.send(200, "text/plain", reply);
}

void handleGPS()
{
  server.sendHeader("Connection", "close");
  server.send(200, "text/plain", currentGps());
}

void applyPreset(int p)
{
  sensor_t *s = esp_camera_sensor_get();
  if (!s)
    return;

  if (!psramFound())
    p = 1;
  if (p < 1)
    p = 1;
  if (p > 4)
    p = 4;

  // Higher JPEG quality number = smaller file size = much faster Wi-Fi streaming
  switch (p)
  {
  case 1:
    s->set_framesize(s, FRAMESIZE_QVGA);
    s->set_quality(s, 14);
    break;
  case 2:
    s->set_framesize(s, FRAMESIZE_CIF);
    s->set_quality(s, 16);
    break;
  case 3:
    s->set_framesize(s, FRAMESIZE_HVGA);
    s->set_quality(s, 18);
    break;
  default:
    s->set_framesize(s, FRAMESIZE_VGA);
    s->set_quality(s, 20);
    break;
  }
  currentPreset = p;
}

void handleRes()
{
  if (server.hasArg("p"))
  {
    applyPreset(server.arg("p").toInt());
  }
  server.sendHeader("Connection", "close");
  server.send(200, "text/plain", "RES=" + String(currentPreset));
}

void streamTask(void *parameter)
{
  WiFiClient pending;

  while (true)
  {
    WiFiClient client;

    if (pending)
    {
      client = pending;
      pending = WiFiClient();
    }
    else
    {
      client = streamServer.available();
    }

    if (!client)
    {
      vTaskDelay(pdMS_TO_TICKS(15));
      continue;
    }

    client.setNoDelay(true);
    client.setTimeout(1);

    String request = "";
    unsigned long start = millis();

    while (client.connected() && millis() - start < 1200)
    {
      while (client.available())
      {
        char c = client.read();
        request += c;
        if (request.endsWith("\r\n\r\n") || request.length() > 1000)
        {
          break;
        }
      }
      if (request.endsWith("\r\n\r\n") || request.length() > 1000)
      {
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(2));
    }

    if (request.indexOf("GET /stream") < 0)
    {
      client.print("HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n");
      client.stop();
      continue;
    }

    client.print(
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n\r\n");

    while (client.connected())
    {
      WiFiClient next = streamServer.available();
      if (next)
      {
        pending = next;
        break;
      }

      camera_fb_t *fb = esp_camera_fb_get();

      if (!fb)
      {
        vTaskDelay(pdMS_TO_TICKS(20));
        continue;
      }

      // Discard tiny/corrupted frames before sending
      if (fb->len < 400)
      {
        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }

      char hdr[128];
      int hlen = snprintf(hdr, sizeof(hdr),
                          "\r\n--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                          (unsigned int)fb->len);

      if (client.write((const uint8_t *)hdr, hlen) != (size_t)hlen)
      {
        esp_camera_fb_return(fb);
        break;
      }

      size_t written = client.write(fb->buf, fb->len);
      esp_camera_fb_return(fb);

      if (written != fb->len)
      {
        break;
      }

      vTaskDelay(pdMS_TO_TICKS(STREAM_DELAY_MS));
    }

    client.stop();
    vTaskDelay(pdMS_TO_TICKS(15));
  }
}

void tuneSensor()
{
  sensor_t *s = esp_camera_sensor_get();
  if (!s)
    return;

  s->set_brightness(s, 0);
  s->set_contrast(s, 0);
  s->set_saturation(s, 0);
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 1);
  s->set_ae_level(s, 0);
  s->set_gain_ctrl(s, 1);
  s->set_gainceiling(s, GAINCEILING_4X);
  s->set_bpc(s, 1);
  s->set_wpc(s, 1);
  s->set_raw_gma(s, 1);
  s->set_lenc(s, 1);
  s->set_dcw(s, 1);
}

void setup()
{
#if DISABLE_BROWNOUT
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif

  Serial.begin(115200);

  RobotSerial.setRxBufferSize(512);
  RobotSerial.begin(9600, SERIAL_8N1, ROBOT_RX, ROBOT_TX);

  camera_config_t config = {};
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

  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  // CRITICAL FIX: 10 MHz XCLK prevents OV2640 DMA overflow (stops green half-frames!)
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound())
  {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 15;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
  }
  else
  {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 16;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed: 0x%x - restarting\n", err);
    delay(3000);
    ESP.restart();
  }

  tuneSensor();
  applyPreset(DEFAULT_PRESET);

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAP(AP_SSID, AP_PASSWORD, 6, 0, 2);
  WiFi.setTxPower(WIFI_POWER_15dBm);

  streamServer.begin();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/cmd", HTTP_GET, handleCommand);
  server.on("/gps", HTTP_GET, handleGPS);
  server.on("/res", HTTP_GET, handleRes);
  server.begin();

  xTaskCreatePinnedToCore(
      streamTask,
      "CameraStream",
      8192,
      nullptr,
      1,
      nullptr,
      1);

  Serial.println("GarbageBot ready");
  Serial.println(WiFi.softAPIP());
}

void loop()
{
  readRobotSerial();
  server.handleClient();
  readRobotSerial();
  delay(4);
}
