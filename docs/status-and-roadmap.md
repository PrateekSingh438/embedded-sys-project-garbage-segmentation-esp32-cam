# Implementation status and roadmap

Implemented in source, physically unverified: ESP camera AP/MJPEG/web endpoints, UART relay, Uno command parser and motor outputs, one-second stop watchdog, TinyGPSPlus parsing, laptop video segmentation overlay, keyboard drive, GPS text display.

Partial or unknown: model training data and garbage category suitability; physical wiring, electrical margins and measured performance; dependency and board package versions.

Absent/planned: garbage-trained model and evaluation; event schema and persistence; detection-to-fresh-GPS association; saved image references; history/map dashboard; autonomous navigation and collection; LoRa; robot identity and health data; automated and hardware-in-loop checks.

Recommended order: validate electrical design and stop behavior; establish detector classes and evaluation; define event format and GPS freshness; add persistent logging; pin software versions; then consider dashboard and long-range communication.