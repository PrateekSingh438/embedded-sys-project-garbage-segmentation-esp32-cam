# Implementation status and roadmap

Implemented in source, physically unverified: ESP camera AP/MJPEG/web endpoints and four resolution presets, UART relay, Uno command parser and motor outputs, 1.5-second stop watchdog, startup motor self-test, TinyGPSPlus parsing, laptop video segmentation overlay and keyboard drive, GPS smoothing/hold/assisted modes, multi-frame detection confirmation, CSV and snapshot logging, live map panel and Folium export.

Partial or unknown: model training data and garbage category suitability; whether live CSV `FIX` records came from satellite fixes or assisted/held positions (the current schema does not encode that distinction); physical wiring, electrical margins and measured performance; dependency and board package versions.

Absent/planned: garbage-trained model and evaluation; explicit GPS subtype/freshness fields in each detection record; autonomous navigation and collection; LoRa; automated and hardware-in-loop checks.

Recommended order: validate electrical design and stop behavior; establish detector classes and evaluation; add GPS subtype and freshness fields to the detection schema; pin software versions; then consider autonomy and long-range communication.
