#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include "secrets.h"


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

// ESP32 UART2: RX=GPIO13, TX=GPIO14
HardwareSerial RobotSerial(2);
const int ROBOT_RX = 13;
const int ROBOT_TX = 14;

WebServer server(80);
WiFiServer streamServer(81);

String gpsData = "";
String gpsLine = "";
unsigned long lastGpsMillis = 0;

// loop() and all web handlers run in the same task,
// so no mutex is needed for gpsData.
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
setInterval(updateGPS,1000);
</script>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", page);
}

void handleCommand()
{
  if (!server.hasArg("c") || server.arg("c").length() == 0)
  {
    server.send(400, "text/plain", "Missing command");
    return;
  }

  char c = server.arg("c")[0];

  if (c != 'F' && c != 'B' && c != 'L' &&
      c != 'R' && c != 'S' &&
      !(c >= '0' && c <= '9'))
  {
    server.send(400, "text/plain", "Invalid command");
    return;
  }

  RobotSerial.write(c);
  server.send(200, "text/plain", "OK");
}

void handleGPS()
{
  String out;

  if (lastGpsMillis == 0 || millis() - lastGpsMillis > 3000)
  {
    out = "GPS: no data from Arduino";
  }
  else
  {
    out = gpsData;
  }

  server.send(200, "text/plain", out);
}

void streamTask(void *parameter)
{
  const char *boundary = "\r\n--frame\r\n";

  while (true)
  {
    WiFiClient client = streamServer.available();

    if (!client)
    {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    client.setNoDelay(true);

    // Read the HTTP request
    String request = "";
    unsigned long start = millis();

    while (client.connected() && millis() - start < 1500)
    {
      while (client.available())
      {
        char c = client.read();
        request += c;

        if (request.endsWith("\r\n\r\n") || request.length() > 1500)
        {
          break;
        }
      }

      if (request.endsWith("\r\n\r\n") || request.length() > 1500)
      {
        break;
      }

      vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (request.indexOf("GET /stream") < 0)
    {
      client.print("HTTP/1.1 404 Not Found\r\n");
      client.print("Connection: close\r\n\r\n");
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
      camera_fb_t *fb = esp_camera_fb_get();

      if (!fb)
      {
        vTaskDelay(pdMS_TO_TICKS(30));
        continue;
      }

      size_t len = fb->len;

      client.print(boundary);
      client.print("Content-Type: image/jpeg\r\n");
      client.print("Content-Length: ");
      client.print(len);
      client.print("\r\n\r\n");

      size_t written = client.write(fb->buf, len);
      esp_camera_fb_return(fb);

      if (written != len)
      {
        break; // client disconnected or write failed
      }

      client.print("\r\n");
      vTaskDelay(pdMS_TO_TICKS(20));
    }

    client.stop();
  }
}

void setup()
{
  Serial.begin(115200);

  RobotSerial.begin(9600, SERIAL_8N1, ROBOT_RX, ROBOT_TX);

  camera_config_t config = {}; // zero-initialize every field
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

  // If this line fails to compile on your board package,
  // rename pin_sscb_* to pin_sccb_* (or the reverse).
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound())
  {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
  }
  else
  {
    config.frame_size = FRAMESIZE_QQVGA;
    config.jpeg_quality = 16;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed: 0x%x\n", err);
    while (true)
    {
      delay(1000);
    }
  }

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  streamServer.begin();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/cmd", HTTP_GET, handleCommand);
  server.on("/gps", HTTP_GET, handleGPS);
  server.begin();

  xTaskCreatePinnedToCore(
      streamTask,
      "CameraStream",
      8192,
      nullptr,
      1,
      nullptr,
      0);

  Serial.println("GarbageBot ready");
  Serial.println(WiFi.softAPIP());
}

void loop()
{
  server.handleClient();
  readRobotSerial();
  delay(2);
}