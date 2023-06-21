# PowDivert
DIY 3 Phase Power Diverter for PV system

Can divert all excess power from each phase to a boiler/puffer.
Power related messages, are obtained from Huawei SmartMeter and Inverter, over WiFi ModbusTCP connection. 
In order to establish the ModbusTCP connection with the Dongle/Inverter, first you need to update the Dongle and Inverter Firmware, to the latest version (at least V100R001C00SPC133 for Dongle and V100R001C00SPC153 for Inverter).
You must activate ModbusTCP from Huawei Dongle, with Installer account (Local O&M > ModbusTCP > Unrestricted access > Time interval 1-3 seconds) and set a fixed IP on your router for both Huawei Dongle and your Diverter.

Two different working modes can be enabled: Overvoltage protection (power will be diverted only when the grid voltage rise above set value, ex.252V) and Normal mode (diverter always enabled and active when boiler temp. is below set value).

Boost mode can be enabled for low temperature in the boiler, to heatup the water to a preset value, when there is no excess power from PV system.

When there is no communication with the Inverter/Smartmeter, the diverter will be disabled in a few minutes. 

1st Test Video  
https://youtube.com/shorts/sCgPW7VnUGQ

2nd Test Video = Thermostat function  
https://youtu.be/1lXEFfLnF-U

3rd Test Video = Live  
https://youtu.be/6jQ1DJV2dLw

Video Description:

>Div.A/B/C   kW = power diverted on each phase to the 3 phase heater (3x 2300W)  
>Boiler C = boiler temperature in Celsius  
>Heats C = Heatsink temperature in Celsius  

>Info. received from Huawei SmartMeter and Inverter, using WiFi Modbus TCP:  
>Grid  kW = Total power injection (Green) / consumption (Red)  
>Solar kW = Total Solar power from PV system  

>Grid A/B/C  kW = power injection (Green) / consumption (Red) on each phase  
>Grid A/B/C   V = Grid Voltage on each phase  

----- Software ----- 

No WebUI, so you need to compile the project in Arduino, with your settings. Search for "Config" part in the sourcecode.  

A preconfigured, portable Arduino package, can be downloaded from here = https://drive.google.com/file/d/1l2-xdqns-cGS3Vl_VYF_oUsG2-rnJhcs/view?usp=sharing

  
----- Hardware ----- 

Main Components:

1) v1 = 3x PWM 8A Dimmers + external isolated triacs (BTA41-600B) = https://www.tindie.com/products/bugrovs2012/pwm-8a-ac-light-dimmer-module-50hz-60hz-tasmota/#specs  
v2 = NEW - MOSFET driver 3x PWM 10A Dimmers = https://www.tindie.com/products/bugrovs2012/pwm10a-2200w-mosfet-trailing-edge-dimmer-freeds/

3) ESP32 TTGO TFT board = https://cleste.ro/placa-dezvoltare-cu-esp32-bluetooth-si-display.html

4) 3Phase Heater (6kW = 3x 2kW) = https://www.okazii.ro/rezistente-boiler-5-kw-6kw-7-5-kw-10kw-a233442016 

5) 3P Contactor = https://www.dedeman.ro/oradea/contactor-tvs-25a-3p-1nd-2-2kw-220v-lc1e2510m5/p/1039446

6) 4P MCB 16A = https://www.dedeman.ro/oradea/intrerupator-automat-modular-schneider-electric-ik60-a9k24416-4p-16a-curba-c/p/1033292

7) Small 5V High Level trigger relay board ( to Contactor coil) = https://cleste.ro/modul-releu-1-canal-5v-high-level.html

8) Temperature sensors = 2x DS18B20 (Boiler/Puffer + Heatsink for v1 only) = https://www.optimusdigital.ro/ro/senzori/1465-senzor-de-temperatura-ds18b20-to-92.html

9) Safety thermostat max. 75C ( to Contactor coil)

10) Cables Min. 5x2.5 + wires + Wago connectors + pin terminals

11) Heatsink 60x190mm (for v1 only) = https://componenteonline.ro/Radiatorextrudataluminiu60mmx1905mmCuloareSTONECOLD--T137016 

12) 5V FAN 60x60mm (Optional, for v1 only) = 1-3x https://dalap.ro/ventilator-racire-dalap-saf-5v-dc-60x60x25-mm-3000-rmin-3679

13) 5V Power supply, min. 2A

14) External Push Button for ESP32

15) Small Test PCB board for ESP32 wiring/connections

16) Distribution box, 12 modules = https://www.celon.ro/materiale-si-echipamente-electrice-1684/tablouri-cofrete-dulapuri-1399/cofrete-modulare-1456/tablou-siguranta-12-modul-1x12-aplicat-vl-cs12a-viko-413700.html

17) Small box 80x80x40 for ESP32 = https://www.dedeman.ro/oradea/doza-derivatie-aparenta-gewiss-gw44003-6-intrari-ip44-80-x-80-x-40-mm/p/1059379

18) > ToDo = Filters to suppress Triacs EMI and RFI noise 

> For 1 Phase only you can use PWM 16A Dimmer + Upshield Board for ESP32 WiFi Kit module (https://cleste.ro/modul-nodemcu-wifi-esp8266-cp2102-cu-display-oled.html) with FreeDS software (https://freeds.es/) and similar commercial box: https://www.hornbach.ro/p/doza-aplicata-pentru-legaturi-pawbol-150x110-mm-ip65-gri-cu-capac-transparent-fara-presetupe/10567244/  
> 1 Phase Test Video  
> https://youtu.be/vAFfa62qKx4

>Work in progress...
