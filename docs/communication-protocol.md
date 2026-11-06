# Communication protocol

Laptop uses HTTP GET port 80. /cmd?c= accepts F/B/L/R/S and digits 0-9. Python sends requests about every 150 ms. /gps returns latest plain-text telemetry. Video is multipart JPEG at port 81 /stream.

ESP UART2 and Uno hardware Serial use 9600 baud, 8N1. Commands are individual bytes without line endings. GPS status returns newline-terminated text: GPS,lat,lng,satellites; GPS,NO_FIX,SAT=n; or GPS,NO_DATA,CHECK_WIRING.

GPS to Uno is NMEA over SoftwareSerial at 9600, GPS TX to D4. LoRa/SX1262 and cloud links are not implemented.