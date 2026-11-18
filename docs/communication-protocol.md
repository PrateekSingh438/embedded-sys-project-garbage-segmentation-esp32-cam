# Communication protocol

The laptop connects to the ESP32-CAM access point at `192.168.4.1`. HTTP runs on port 80 and the MJPEG stream runs on port 81.

| Endpoint | Behavior |
|---|---|
| `GET /` | Camera preview page; use one stream viewer at a time. |
| `GET /cmd?c=F` | Send one command byte: `F`, `B`, `L`, `R`, `S`, or speed `1` through `9`. The response includes the latest GPS line, ESP uptime, and camera preset. |
| `GET /gps` | Latest GPS telemetry, or `GPS: no data from Arduino` if no line arrived recently. |
| `GET /res?p=1` | Select camera preset 1 through 4. No-PSRAM boards are limited to preset 1. |
| `GET :81/stream` | MJPEG video. |

ESP UART2 and Uno hardware Serial use 9600 baud, 8N1. Commands are individual bytes without line endings. GPS status is newline-terminated: `GPS,latitude,longitude,satellites,hdop`, `GPS,NO_FIX,SAT=n`, or `GPS,NO_DATA,CHECK_WIRING`. HDOP is `-1` when the receiver has no valid HDOP reading. The Python UI may show a held last fix for up to 15 seconds or use software-assisted coordinates indoors; those states are identified in the UI and are not hardware satellite fixes.

GPS to Uno is NMEA over SoftwareSerial at 9600, GPS TX to D4. Python refreshes movement commands about every 250 ms and stop commands about every 800 ms. The Uno stops motors after 1.5 seconds without a valid command. LoRa/SX1262 and cloud links are not implemented.
