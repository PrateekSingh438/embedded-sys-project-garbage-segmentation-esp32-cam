# Troubleshooting

| Symptom | Checks |
|---|---|
| Wi-Fi AP missing | ESP power, upload, secrets header and serial startup output |
| Blank stream | AI Thinker board selection, camera init output, supply, single viewer |
| Python cannot connect | Join AP, check 192.168.4.1:81/stream, close browser viewer |
| Model missing | Keep file in ai-detection/models |
| GPS NO_DATA | GPS power, TX to D4, ground and baud |
| GPS NO_FIX | Test outdoors with open sky and allow acquisition time |
| ESP shows no Arduino data | UART crossing, common ground, baud and input voltage protection |
| Motors do not move | Motor supply, L298N enable configuration, ground, pins and command path |
| Stops after one second | Expected watchdog behavior when commands stop arriving |
| Direction reversed | Swap motor leads or direction logic |
| ESP resets when motors start | Review supply current, grounding and motor noise |

Physical operation has not been verified.