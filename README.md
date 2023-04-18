# PowDivert
DIY 3 Phase Power Diverter for PV system

First test Video
https://youtube.com/shorts/sCgPW7VnUGQ

Main Components:

1) v1 = 3x PWM 8A Dimmers + external isolated triacs (BTA41-600B) = https://www.tindie.com/products/bugrovs2012/pwm-8a-ac-light-dimmer-module-50hz-60hz-tasmota/#specs

v2 = 3x PWM 16A Dimmers = https://www.tindie.com/products/bugrovs2012/pwm-16a-3500w-triac-leading-edge-dimmer-50hz-60hz/

2) ESP32 TTGO TFT board = https://cleste.ro/placa-dezvoltare-cu-esp32-bluetooth-si-display.html

3) 3Phase Heater (6kW = 3x 2kW) = https://www.okazii.ro/rezistente-boiler-5-kw-6kw-7-5-kw-10kw-a233442016 

4) 3P Contactor = https://www.dedeman.ro/oradea/contactor-tvs-25a-3p-1nd-2-2kw-220v-lc1e2510m5/p/1039446

5) 4P MCB 16A = https://www.dedeman.ro/oradea/intrerupator-automat-modular-schneider-electric-ik60-a9k24416-4p-16a-curba-c/p/1033292

6) Small 5V High Level trigger relay board ( to Contactor coil) = https://cleste.ro/modul-releu-1-canal-5v-high-level.html

7) Temperature sensors = 2x DS18B20 (Boiler/Puffer + Heatsink for v1 only) = https://www.optimusdigital.ro/ro/senzori/1465-senzor-de-temperatura-ds18b20-to-92.html

8) Safety thermostat max. 75C ( to Contactor coil)

9) Cables Min. 5x2.5 + wires + Wago connectors + pin terminals

10) Heatsink 60x190mm (for v1 only) = https://componenteonline.ro/Radiatorextrudataluminiu60mmx1905mmCuloareSTONECOLD--T137016 

11) 5V FAN 60x60mm (for v1 only) = 3x https://dalap.ro/ventilator-racire-dalap-saf-5v-dc-60x60x25-mm-3000-rmin-3679

12) 5V Power supply, min. 2A

13) External Push Button for ESP32

14) Small Test PCB board for ESP32 wiring/connections

15) Distribution box, 12 modules = https://www.celon.ro/materiale-si-echipamente-electrice-1684/tablouri-cofrete-dulapuri-1399/cofrete-modulare-1456/tablou-siguranta-12-modul-1x12-aplicat-vl-cs12a-viko-413700.html

16) Small box 80x80x40 for ESP32 = https://www.dedeman.ro/oradea/doza-derivatie-aparenta-gewiss-gw44003-6-intrari-ip44-80-x-80-x-40-mm/p/1059379

17) >>> Filters to suppress Triacs EMI and RFI noise 

// For 1 Phase only you can use PWM 16A Dimmer + Upshield Board for ESP32 WiFi Kit module (https://cleste.ro/modul-nodemcu-wifi-esp8266-cp2102-cu-display-oled.html) with FreeDS software (https://freeds.es/)

Work in progress...
