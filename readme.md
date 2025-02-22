# esp32_enclosure_heater

ESP32 based controller for an active enclosure heating system for 3D printers.

It should have buttons to select the heater ON and OFF, and to adjust the desired temperature. Then is should use a PWM out and some mosfets to control the power provided to some heater elements. The PWM should be controlled by a PID controller to provide a stable temperature.