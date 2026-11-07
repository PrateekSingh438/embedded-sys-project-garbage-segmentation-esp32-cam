# Hardware and wiring

## Pin map from source

| Uno pin | Connection |
|---|---|
| D5 PWM, D8, D9 | L298N ENA, IN1, IN2 |
| D6 PWM, D10, D11 | L298N ENB, IN3, IN4 |
| D4 | GPS TX (SoftwareSerial receive) |
| D3 | SoftwareSerial TX, unused by current GPS code |
| D0/RX, D1/TX | ESP serial link |

ESP UART2 uses GPIO13 RX and GPIO14 TX at 9600 baud, 8N1. Connect ESP TX GPIO14 to Uno RX (D0), and Uno TX (D1) to ESP RX GPIO13 through a resistor divider because the Uno output is 5 V and ESP GPIO is 3.3 V. Share ground. Disconnect D0/D1 while uploading Uno firmware.

## Component roles and power

ESP32-CAM captures and serves video. Uno handles motor commands and GPS parsing. NEO-6M supplies NMEA data. L298N drives two 3–6 V DC motors. Battery and chassis enable mobility, but exact battery voltage/current, regulator, motor model, and wiring harness are not supplied. Select power components from the actual module datasheets. Give the ESP32-CAM a stable 5 V supply that handles current peaks. Never power motors from Uno GPIO or its 5 V output. Connect grounds together. Test with wheels lifted at low speed.

LoRa/SX1262 is not present in source. Hardware has not been physically verified.
