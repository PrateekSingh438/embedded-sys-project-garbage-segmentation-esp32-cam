# Installation and configuration

## Firmware

Install Arduino IDE and Espressif ESP32 board support. Select AI Thinker ESP32-CAM for hardware/esp32-cam/esp.ino. ESP32 core provides camera and WebServer; install TinyGPSPlus for Uno; SoftwareSerial is included with AVR core. Copy hardware/esp32-cam/secrets.example.h to secrets.h and set a unique access point password. secrets.h is ignored by git. Flash hardware/arduino/arduino.ino to the Uno.

## Python on Windows

From the repository root:

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r ai-detection/requirements.txt
python ai-detection/camera.py
```

Join the ESP Wi-Fi network first. Python expects the ESP at 192.168.4.1 and loads ai-detection/models/yolo11n-seg.pt. Keep the video window focused to drive. The ESP page reports one viewer at a time; close that page before launching Python.

Endpoints: http://192.168.4.1/ ; stream http://192.168.4.1:81/stream ; GPS http://192.168.4.1/gps ; stop http://192.168.4.1/cmd?c=S. Dependency versions are not pinned. Exact hardware and package versions are unknown.