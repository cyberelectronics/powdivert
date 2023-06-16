//PowDivert v1.3 =>  3Phase Solar Excess Power Diverter with ESP32 TTGO Display module ------ by CyberElectronics
// PWM Dimmers from ex. Krida Electronics on Tindie https://www.tindie.com/stores/bugrovs2012/  
// or MPDMv4.1 with some modifications, from NextEVO on Tindie https://www.tindie.com/products/next_evo1/universal-ac-mains-dimmer-mpdmv41/  
//=================================================================================================
// eModbus library: Copyright 2020 by Michael Harwerth, Bert Melis and the contributors to ModbusClient
//                  MIT license - see license.md for details
// =================================================================================================

#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <SPIFFS.h>
#include <EEPROM.h>
#include "Tickerstaub.h"

#include <OneWire.h>
#include <DallasTemperature.h>

// Include the header for the ModbusClient TCP style
#include <ModbusClientTCPasync.h>

// -------- STA Mode
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <ArduinoJson.h> 

#include <TFT_eSPI.h>      // Hardware-specific library
TFT_eSPI tft = TFT_eSPI(); // Invoke custom library

#include <Arduino.h>
#include <WiFi.h>

#include "Button2.h";

#define TFT_DISPOFF 0x28
#define TFT_BL              4   // Display backlight control pin
#define ADC_EN              14  //ADC_EN is the ADC detection enable port
#define ADC_PIN             34
#define BUTTON_A_PIN  35
#define BUTTON_B_PIN  0
#define ONE_WIRE_BUS 15

#define FAN_A_control 2
#define RELAY_control 22
#define PWM_A_pin 17
#define PWM_B_pin 12
#define PWM_C_pin 13

//TFT y positions for texts and numbers
#define toptext 20            // y coordinates for text 
#define bottomtext 145        //
#define topdraw 75            // and numbers
#define bottomdraw 200        // TTGO 135x250 TFT display

Button2 buttonA = Button2(BUTTON_A_PIN);
Button2 buttonB = Button2(BUTTON_B_PIN);


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// Config ===============================================================================================

// Modify here the Temperature sensor Address, displayed on Serial Monitor after boot
DeviceAddress sensor_boiler = { 0x28, 0xAF, 0x83, 0x81, 0xE3, 0x81, 0x3C, 0x60 }; // Boiler temperature sensor 
DeviceAddress sensor_A =      { 0x28, 0x79, 0x4, 0x81, 0xE3, 0xF3, 0x3C, 0xAB };  // Triac/Mosfet heatsink temperature sensor    


// Define Smartmeter / Huawei Inverter Register Address
#define SM_ACTIV_P 37113         // SmartMeter Total Active Power register address
#define SM_GRIDA_V 37101         // SmartMeter Grid Phase A Voltage
#define SM_GRIDB_V 37103         // SmartMeter Grid Phase B Voltage
#define SM_GRIDC_V 37105         // SmartMeter Grid Phase C Voltage
#define SM_GRIDA_P 37132         // SmartMeter Grid Phase A Power
#define SM_GRIDB_P 37134         // SmartMeter Grid Phase B Power
#define SM_GRIDC_P 37136         // SmartMeter Grid Phase C Power
#define IN_ACTIV_P 32080         // Inverter Total Active Power

#define MaxPower 2000            // Set Heater Max Power (W), for resistive loads only, measure U and I then MaxPower P = U*I 
bool GridV_control = false;      // true/false = enable/disable if you want to reduce the Grid Voltage, by turning on the heater, when Grid V is above 252V          
bool BOOST_mode = false;         // Turn ON the heaters at max power, when boiler temp drops below 38C (boiler_LOW_Warning)
bool boiler_onoff = true;        // Boiler state at start, ON = true

#define max_voltage_warning 252.0   // Turn ON the heater above this value, using Excess Power, when GridV control is enabled and Display Red Color on TFT
#define min_voltage_heaterOFF 250.0 // Turn OFF the heater below this value, when GridV control is enabled 

#define boiler_LOW_Warning 38.0     // HotWater Low temperature warning (RED color below this value) and turn ON (forced) the heater, even there is no excess power from PV
#define boiler_keepMIN_temp 45.0    // turn ON the heater with full power to keep this minimum temperature in the boiler 
#define boiler_set_temp 65.0        // set the boiler max temperature 
#define boiler_hyst_temp 63.0       // Hysteresis - temperature when we turn on again the heating process
#define boiler_LOW_Error -30.0      // Boiler Temp sensor Error (will return the value of -127 )
#define boiler_HIGH_Warning 70.0    // HotWater High temperature warning (RED color above this value); ERROR - PWM driver module or temp sensor ; Mechanical Safety thermostat should be set above this value
#define divertA_LOW_Warning -30.0     // Heatsink = PWM Driver Module Temp sensor Error (will return the value of -127 )
#define divertA_HIGH_Warning 70.0     // Heatsink = PWM Driver Module Temp sensor Error or Temp too high

#define request_interval 1500       // request 1 Modbus message, every 1.5 seconds
#define temp_interval 10000         // measure temperature every 10 seconds
#define pagenumbers 2               // number of pages -1, to display on TFT
#define max_message_number 7        // max number of messages request from inverter/smartmeter
#define SM_GRIDA_P_rec_interval 10  // turn off the heater A, after no response for 10 message requests (error / no response from inverter/SmartMeter) 
#define SM_GRIDB_P_rec_interval 10  // turn off the heater B, after no response for 10 message requests (error / no response from inverter/SmartMeter) 
#define SM_GRIDC_P_rec_interval 10  // turn off the heater C, after no response for 10 message requests (error / no response from inverter/SmartMeter) 
#define IN_ACTIV_P_rec_interval 10  // turn off the Contactor, after no response for 10 message requests (error / no response from Inverter) 

#define HOMEminhumi_Warning 30.0    // RED color below this humidity value (%)
#define HOMEmaxhumi_Warning 60.0    // RED color above this humidity value (%)

int ledBacklight = 80; // Initial TFT backlight intensity on a scale of 0 to 100 percent. Initial value is 80.

bool debugmode = true;


// Huawei inverter Dongle, must be set to Local O&M - ModbusTCP, Unrestricted access, access time interval 1-3seconds and must have Fixed IP, set on your WiFi router.
#ifndef MY_SSID
#define MY_SSID "YourHomeWiFiSSID"  // your Home Network WiFi SSID
#endif
#ifndef MY_PASS
#define MY_PASS "YourHomeWiFiPASS"  // your Home Network WiFi Password
#endif

IPAddress ip = {192, 168, 0, 101};   // Your Huawei inverter/Dongle IP address
uint16_t port = 502;   //6607;       // port of modbus server

// END Config ============================================================================================
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


// Setting PWM properties
const int pwmFreq = 1000;     // or 3000 Hz
const int pwmResolution = 8;
const int pwmLedChannelTFT = 0;
const int PWM_A_channel = 1;
const int PWM_B_channel = 2;
const int PWM_C_channel = 3;

static byte lookup_PWM[41] = { 0,18,20,26,31,36,39,43,47,52,56,59,64,67,71,75,78,82,85,88,94,98,103,106,111,114,118,123,126,132,137,142,147,155,162,167,178,189,206,242,255};  // Calib. for Krida module, PWM values to increase power lin. for every 2000W / 40 = 50W

const int calib_points = 40;
int onecalib_point;     // power value of 1 calibration point , ex. MaxPower / calib_points = 2000W / 40 = 50W
int lookup_divertA_PWM, lookup_divertB_PWM, lookup_divertC_PWM; // stores the lookuptable PWM value [0-255]
int calib_index;        // lookuptable index
int calcA_power, calcB_power, calcC_power;        // only to display the calculated value of Heater Power
byte PWM_A_DutyCycle = 0;
byte PWM_B_DutyCycle = 0;
byte PWM_C_DutyCycle = 0;
byte PWM_A_percent; 
byte disp_counter;
int divertA_power, divertA_cache, divertA_last;
int divertB_power, divertB_cache, divertB_last;
int divertC_power, divertC_cache, divertC_last;
int SM_GRIDA_P_rec_counter;
int SM_GRIDB_P_rec_counter;
int SM_GRIDC_P_rec_counter;
int IN_ACTIV_P_rec_counter;

int sec_interval = 10;  // sec_interval/10 = sec.
int sensor_read_period;
int sensor_calc_period;

float temp_average; 
float boiler_temp, divertA_temp; //, ambient_temp;   // boiler temperature from Boiler Device

int32_t value;
int32_t sm_activ_p;
float sm_grida_v;
float sm_gridb_v;
float sm_gridc_v;
int32_t sm_grida_p;
int32_t sm_gridb_p;
int32_t sm_gridc_p;
int32_t in_activ_p;

uint8_t request_counter, measure_temp_interval;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

uint16_t x,y;
byte button;
byte ledBacklight_percent, ledBacklight_percent_buffer;
boolean clicked;
boolean mbus_error;          // error flag - Huawei ModBus Not Responded
boolean boiler_temp_error;   // temperature issues

boolean divertA_temp_error;  // Diverter modules high temp or sensor error
boolean boiler_quickheat_flag; 
boolean boiler_hyst_temp_flag; // true = hysteresis active (temp between 63.0C <> 65.0C)
boolean keepon_flag_a, keepon_flag_b, keepon_flag_c;  
boolean SM_GRIDA_P_rec_flag, SM_GRIDB_P_rec_flag, SM_GRIDC_P_rec_flag, IN_ACTIV_P_rec_flag; // Phase A/B/C Power message received flag      
int ms_counter; // incremented every 100ms
int request_interval_counter; // incremented every 100ms

//define your default values here, if there are different values in config.json, they are overwritten.
char mqtt_server[40];
char mqtt_port[6] = "8080";

//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
                              Serial.println("Should save config");
                              shouldSaveConfig = true;
                            }

// constants won't change:
const long interval = 100;           // interval to check datas (milliseconds)
unsigned long currentMillis;
unsigned long lastMillis = 0;        // will store last time update

char ssid[] = MY_SSID;                     // SSID and ...
char pass[] = MY_PASS;                     // password for the WiFi network used

// Create a ModbusTCP client instance
ModbusClientTCPasync MB(ip, port);

// Temperature sensors
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress sensor_address; 
int total_devices;

void requestmessage();
void measure_temp();
void display_time();
void OTA_check();

Ticker timer_modbus(requestmessage, request_interval); // 
Ticker timer_temperature(measure_temp, temp_interval); // 
Ticker timer_display(display_time, 1000); // display pages 
Ticker timer_OTA(OTA_check, 5000); // display pages 

//SYSTEM_THREAD(ENABLED)

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// Define an onData handler function to receive the regular responses
// Arguments are Modbus server ID, the function code requested, the message data and length of it, 
// plus a user-supplied token to identify the causing request

void handleData(ModbusMessage response, uint32_t token) 
{

                        //Serial.printf("Response: serverID=%d, FC=%d, Token=%08X, length=%d:\n", response.getServerID(), response.getFunctionCode(), token, response.size());
                        // uncomment for print response in hex, from server (Huawei Dongle)  
                       /* for (auto& byte : response) {
                          Serial.printf("%02X ", byte);
                        }
                        Serial.println("");
                        */

                        response.get(3, value);
                        
                        Serial.print(token);  // Print Register number
                        
                        switch(token){
                                       case SM_ACTIV_P:  // Smart Meter Total Active Power
                                                          sm_activ_p = value;
                                                          Serial.print(" = SM Active Power [W] = ");  
                                                          Serial.println(sm_activ_p);
                                                          
                                                          break;
                                       case SM_GRIDA_V:  
                                                          sm_grida_v = value/10.0;
                                                          Serial.print(" = SM Grid A Voltage [V] = ");  
                                                          Serial.println(sm_grida_v);
                                                          break;                   
                                       case SM_GRIDB_V:  
                                                          sm_gridb_v = value/10.0;
                                                          Serial.print(" = SM Grid B Voltage [V] = ");  
                                                          Serial.println(sm_gridb_v);
                                                          break;   
                                       case SM_GRIDC_V:  
                                                          sm_gridc_v = value/10.0;
                                                          Serial.print(" = SM Grid C Voltage [V] = ");  
                                                          Serial.println(sm_gridc_v);
                                                          break;                    
                                       case SM_GRIDA_P:  
                                                          sm_grida_p = value;
                                                          divertA_cache = sm_grida_p;
                                                          Serial.print(" = SM Grid A Power [W] = ");  
                                                          Serial.println(sm_grida_p);
                                                          SM_GRIDA_P_rec_flag = true;
                                                          SM_GRIDA_P_rec_counter = 0;
                                                          break;    
                                       case SM_GRIDB_P:  
                                                          sm_gridb_p = value;
                                                          divertB_cache = sm_gridb_p;
                                                          Serial.print(" = SM Grid B Power [W] = ");  
                                                          Serial.println(sm_gridb_p);
                                                          SM_GRIDB_P_rec_flag = true;
                                                          SM_GRIDB_P_rec_counter = 0;
                                                          break;    
                                       case SM_GRIDC_P:  
                                                          sm_gridc_p = value;
                                                          divertC_cache = sm_gridc_p;
                                                          Serial.print(" = SM Grid C Power [W] = ");  
                                                          Serial.println(sm_gridc_p);
                                                          SM_GRIDC_P_rec_flag = true;
                                                          SM_GRIDC_P_rec_counter = 0;
                                                          break;   
                                       case IN_ACTIV_P: // Inverter Total Active Power 
                                                          in_activ_p = value;
                                                          Serial.print(" = IN Active Power [W] = ");   
                                                          Serial.println(in_activ_p);
                                                          IN_ACTIV_P_rec_flag = true;
                                                          IN_ACTIV_P_rec_counter = 0;
                                                          break;                                        
                                                          
                          }
}

// Define an onError handler function to receive error responses
// Arguments are the error code returned and a user-supplied token to identify the causing request
void handleError(Error error, uint32_t token) 
                  {
                    // ModbusError wraps the error code and provides a readable error message for it
                    ModbusError me(error);
                    Serial.printf("Error response: %02X - %s token: %d\n", (int)me, (const char *)me, token);
}



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Setup() - initialization happens here
void setup() {

                pinMode(BUTTON_A_PIN, INPUT_PULLUP);
                pinMode(ADC_EN, OUTPUT);
                digitalWrite(ADC_EN, HIGH);
                pinMode(TFT_BL, OUTPUT); 
                pinMode(PWM_A_pin, OUTPUT);
                pinMode(PWM_B_pin, OUTPUT);
                pinMode(PWM_C_pin, OUTPUT);
                pinMode(FAN_A_control, OUTPUT);
                pinMode(RELAY_control, OUTPUT);
                digitalWrite(RELAY_control, LOW);

                onecalib_point = MaxPower / calib_points;   // => ex. 1850W / 40 = 42.5W

                ledcSetup(pwmLedChannelTFT, pwmFreq, pwmResolution);
                ledcSetup(PWM_A_channel, pwmFreq, pwmResolution);
                ledcSetup(PWM_B_channel, pwmFreq, pwmResolution);
                ledcSetup(PWM_C_channel, pwmFreq, pwmResolution);
                
                ledcAttachPin(TFT_BL, pwmLedChannelTFT);
                ledcAttachPin(PWM_A_pin, PWM_A_channel);
                ledcAttachPin(PWM_B_pin, PWM_B_channel);
                ledcAttachPin(PWM_C_pin, PWM_C_channel);
                
                ledcWrite(pwmLedChannelTFT, ledBacklight);
                ledcWrite(PWM_A_channel, PWM_A_DutyCycle);
                ledcWrite(PWM_B_channel, PWM_B_DutyCycle);
                ledcWrite(PWM_C_channel, PWM_C_DutyCycle);
                               
 // Init Serial monitor
                Serial.begin(115200);
                while (!Serial) {}
                Serial.println("__ OK __");
                
                sensors.begin();
                total_devices = sensors.getDeviceCount();  // check how many temp sensors we have

                Serial.print("Locating Temp Sensors");
                Serial.print("Found ");
                Serial.print(total_devices, DEC);
                Serial.println(" devices.");

                for(int i=0;i<total_devices; i++){
                                                      if(sensors.getAddress(sensor_address, i)){
                                                        Serial.print("Found device ");
                                                        Serial.print(i, DEC);
                                                        Serial.print(" with address: ");
                                                        printAddress(sensor_address);
                                                        Serial.println();
                                                      } else {
                                                        Serial.print("Found device at ");
                                                        Serial.print(i, DEC);
                                                        Serial.print(" but could not detect address. Check circuit connection!");
                                                      }
                                                    }
                
                
                measure_temp();
              
                tft.begin();
                tft.setRotation(0);  // 0 & 2 Portrait. 1 & 3 landscape
                tft.fillScreen(TFT_BLACK);
                
             
              
              // Connect to WiFi
                WiFi.begin(ssid, pass);
                delay(200);

                while (WiFi.waitForConnectResult() != WL_CONNECTED) {
                                                                      Serial.println("Connection Failed! Rebooting...");
                                                                      delay(5000);
                                                                      ESP.restart();
                                                                    }
                
                //while (WiFi.status() != WL_CONNECTED) {
                //  Serial.print(". ");
                //  delay(1000);
               // }
               
                IPAddress wIP = WiFi.localIP();
                Serial.printf("WIFi IP address: %u.%u.%u.%u\n", wIP[0], wIP[1], wIP[2], wIP[3]);

ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

ArduinoOTA.begin();

                
              // Set up ModbusTCP client.
              // - provide onData handler function
                MB.onDataHandler(&handleData);
              // - provide onError handler function
                MB.onErrorHandler(&handleError);
              // Set message timeout to 2000ms and interval between requests to the same host to 200ms
                MB.setTimeout(10000);
              // Start ModbusTCP background task
                MB.setIdleTimeout(60000);
              // Sets the maximum number of messages that are sent to the server at once. 
                MB.setMaxInflightRequests(1);
                                                 
                          buttonA.setClickHandler(click);
                          buttonB.setClickHandler(click);
                          buttonA.setLongClickHandler(longpress);
              
                connectedscreen();
              
                tft.setTextDatum(MC_DATUM);
                x = tft.width()/2;

                 timer_modbus.start();
                 timer_temperature.start();
                 timer_display.start();
                 timer_OTA.start();
  
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> loop()
void loop() {
                
                 timer_OTA.update();
                 
                 buttonA.loop();
                 buttonB.loop();

                 
                 timer_modbus.update();
                 timer_temperature.update();
                 timer_display.update();
                 
    
}
//>>>>>>>>  END Loop >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

void boiler_ON(){
                   boiler_onoff = true;
                   digitalWrite(RELAY_control, HIGH);
}

void boiler_OFF(){
                    ledcWrite(PWM_A_channel, 0);  // turn OFF the Heater A
                    ledcWrite(PWM_B_channel, 0);  // turn OFF the heater B
                    ledcWrite(PWM_C_channel, 0);  // turn OFF the heater C
                    digitalWrite(RELAY_control, LOW); // turn OFF the contactor
                    calcA_power = calcB_power = calcC_power = 0;          // Display 0 for "DivA/B/C Power", on TFT
                    divertA_power = divertB_power = divertC_power = 0;    // Reinit the variable
             
}

//>>>>>>>> Check temperatures and all other conditions to start the heater on all Phases >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void check_conditions(){
                           
                           
                           if(boiler_temp_error){  // boiler temperature sensor ERROR or Boiler should be turned OFF
                                                                    boiler_OFF();   
                                                                       
                           }
                           else {
                                 if(BOOST_mode){
                                                   if((boiler_temp < boiler_LOW_Warning)and !boiler_quickheat_flag){  // very low temperature in boiler, => Start Boost Mode
                                                                                                                      
                                                                                                                      if (!boiler_onoff) {
                                                                                                                                            boiler_ON();
                                                                                                                      }
                                                                                                                      
                                                                                                                      else if(!divertA_temp_error){  // no temp sensor error or high temp, so start Boost Mode
                                                                                                                                                      boiler_quickheat_flag = true;   // Boost Mode ON flag
                                                                                                                                                      ledcWrite(PWM_A_channel, 255);  // turn ON the Heater A at full power, now
                                                                                                                                                      calcA_power = MaxPower;         // Display Max Power for "DivA Power", on TFT
                                                                                                                                                      ledcWrite(PWM_B_channel, 255);  // turn ON the Heater B at full power, now
                                                                                                                                                      calcB_power = MaxPower;         // Display Max Power for "DivB Power", on TFT
                                                                                                                                                      ledcWrite(PWM_C_channel, 255);  // turn ON the Heater C at full power, now
                                                                                                                                                      calcC_power = MaxPower;         // Display Max Power for "DivC Power", on TFT
                                                                                                                                                    }
                                                                                                                                                                                                                           
                                                                                                                   }
                                                   else if((boiler_temp > boiler_keepMIN_temp) and boiler_quickheat_flag){  // keep Heater ON until temperature rise above boiler_keepMIN_temp, then turn OFF
                                                                                                                             boiler_quickheat_flag = false; // clear the Boost Mode flag
                                                                                                                             boiler_OFF();
                                                                                                                          }
                                                }
                                   else if((boiler_temp > boiler_set_temp) or boiler_hyst_temp_flag) {  // boiler temperature above set (65.0C) or hyst active (temp between 63.0C <> 65.0C)
                                                                                                                   boiler_hyst_temp_flag = true;
                                                                                                                   if(boiler_temp < boiler_hyst_temp) boiler_hyst_temp_flag = false;  // temp falls below 63.0C, next time go to Heating process
                                                                                                                   
                                                                                                                   boiler_OFF();
                                                                                                                }
        
                                   else{   // boiler temp is below set and hyst value, now check if there are messages from Inverter/SM and the temp of the PWM driver modules (Heatsink) are below max value

                                         if(IN_ACTIV_P_rec_counter >= IN_ACTIV_P_rec_interval or !in_activ_p) {  // Inverter Activ Power message, not received in predefined periode OR inverter in Standby Mode 
                                                                                                                                                IN_ACTIV_P_rec_counter = 0;
                                                                                                                                                in_activ_p = 0; // reinit the Inverter Activ Power variable
                                                                                                                                                if(!boiler_quickheat_flag) boiler_OFF();
                                                                                                                                                
                                                                                                                                            }
                                         else if(in_activ_p) {  // Inverter Power is positive => Turn ON the Contactor and the heaters
                                            
                                                                   boiler_ON();                                                                                
                                                              
                                          
                                                                // >>>>>>>>>>>> Phase A - Heater Control >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>                                            
                                                                   if(SM_GRIDA_P_rec_counter >= SM_GRIDA_P_rec_interval) {  // excess Power message for Phase A, not received in predefined periode 
                                                                                                                                                                        SM_GRIDA_P_rec_counter = 0;
                                                                                                                                                                        if(!boiler_quickheat_flag) {  // Boost Mode not enabled, so we can turn OFF Heater A
                                                                                                                                                                                                      ledcWrite(PWM_A_channel, 0);  // turn OFF the heater A
                                                                                                                                                                                                      calcA_power = 0;              // Display 0 for "DivA Power", on TFT
                                                                                                                                                                                                      divertA_power = 0;            // Reinit the variable
                                                                                                                                                                                                     }
                                                                                                                                                                        
                                                                                                                           }
                                                                   else {   
                                                                             // message received OK, Divert A driver module temperature OK  =>  heat with excess power                                             
                                                                                                                                  
                                                                             if(SM_GRIDA_P_rec_flag and !divertA_temp_error){  // Phase A Power message received flag and Divert A driver module temperature OK
                                                                                                                               SM_GRIDA_P_rec_flag = false; // clear the flag
                                                                                                                               
                                                                                                                               if (!GridV_control){ // "Reduce Grid Voltage" Mode is OFF
                                                                                                                                                      set_pwm_DIVERTA();  // set Power for Heater A
                                                                                                                                                  }
                                                                                                                               else {     // "Reduce Grid Voltage" Mode ON with small ON/OFF hysteresis
                                                                                                                                          
                                                                                                                                          if ((sm_grida_v < min_voltage_heaterOFF) and keepon_flag_a)  {  // Grid voltage below MIN value (251.0V) and was turned ON above Max set value (252.0V)
                                                                                                                                                                                                        keepon_flag_a = false;
                                                                                                                                                                                                        ledcWrite(PWM_A_channel, 0);  // turn OFF the heater A
                                                                                                                                                                                                        calcA_power = 0;  // Display 0 for "DivA Power", on TFT
                                                                                                                                                                                                        divertA_power = 0;            // Reinit the variable
                                                                                                                                                                                                      }
                                                                                                                                          else if((sm_grida_v > max_voltage_warning) or keepon_flag_a)  {  // if voltage on Phase A, is above set value (252.0V) or it was already turned ON at Max value (252.0V)  
                                                                                                                                                                                                         keepon_flag_a = true; // keep on until Grid Voltage falls below set MIN value (251.0V)
                                                                                                                                                                                                         set_pwm_DIVERTA();                  
                                                                                                                                                                                                      }
                                                                                                                                     }
                                                                                                       
                                                                                                       
                                                                             }
                                                                            else if(divertA_temp_error)  {  // Heatsink temperature too HIGH or sensor Error
                                                                                                            ledcWrite(PWM_A_channel, 0);  // turn OFF the heater A
                                                                                                            calcA_power = 0;              // Display 0 for "DivA/B/C Power", on TFT
                                                                                                            divertA_power = 0;            // Reinit the variable
                                                                                                            
                                                                                                          }
                                                                   }
                                                                // >>>>>>>>>>>> END Phase A - Heater Control  
                        
                                                                // >>>>>>>>>>>> Phase B - Heater Control >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>                                            
                                                                   if(SM_GRIDB_P_rec_counter >= SM_GRIDB_P_rec_interval) {  // excess Power message for Phase B, not received in predefined periode 
                                                                                                                                                                        SM_GRIDB_P_rec_counter = 0;
                                                                                                                                                                        if(!boiler_quickheat_flag) {  // Boost Mode not enabled, so we can turn OFF Heater B
                                                                                                                                                                                                      ledcWrite(PWM_B_channel, 0);  // turn OFF the heater B
                                                                                                                                                                                                      calcB_power = 0;              // Display 0 for "DivB Power", on TFT
                                                                                                                                                                                                      divertB_power = 0;            // Reinit the variable
                                                                                                                                                                                                     }
                                                                                                                                                                        
                                                                                                                                                                    }
                                                                   else {   
                                                                             // message received OK, Divert A driver module temperature OK  =>  heat with excess power                                             
                                                                                                                                  
                                                                             if(SM_GRIDB_P_rec_flag and !divertA_temp_error){  // Phase B Power message received flag and Heatsink temperature OK
                                                                                                                               SM_GRIDB_P_rec_flag = false; // clear the flag
                                                                                                                               
                                                                                                                               if (!GridV_control){ // "Reduce Grid Voltage" Mode is OFF
                                                                                                                                                      set_pwm_DIVERTB();  // set Power for Heater B
                                                                                                                                                  }
                                                                                                                               else {     // "Reduce Grid Voltage" Mode ON with small ON/OFF hysteresis
                                                                                                                                          
                                                                                                                                          if ((sm_gridb_v < min_voltage_heaterOFF) and keepon_flag_b)  {  // Grid voltage below MIN value (251.0V) and was turned ON above Max set value (252.0V)
                                                                                                                                                                                                        keepon_flag_b = false;
                                                                                                                                                                                                        ledcWrite(PWM_B_channel, 0);  // turn OFF the heater B
                                                                                                                                                                                                        calcB_power = 0;  // Display 0 for "DivB Power", on TFT
                                                                                                                                                                                                        divertB_power = 0;            // Reinit the variable
                                                                                                                                                                                                      }
                                                                                                                                          else if((sm_gridb_v > max_voltage_warning) or keepon_flag_b)  {  // if voltage on Phase B, is above set value (252.0V) or it was already turned ON at Max value (252.0V)  
                                                                                                                                                                                                         keepon_flag_b = true; // keep on until Grid Voltage falls below set MIN value (251.0V)
                                                                                                                                                                                                         set_pwm_DIVERTB();                  
                                                                                                                                                                                                      }
                                                                                                                                     }
                                                                                                       
                                                                                                       
                                                                             }
                                                                            else if(divertA_temp_error)  {  // Heatsink temperature too HIGH or sensor Error
                                                                                                            ledcWrite(PWM_B_channel, 0);  // turn OFF the heater B
                                                                                                            calcB_power = 0;              // Display 0 for "DivA/B/C Power", on TFT
                                                                                                            divertB_power = 0;            // Reinit the variable
                                                                                                            
                                                                                                          }
                                                                   }
                                                                // >>>>>>>>>>>> END Phase B - Heater Control  
                        
                                                                // >>>>>>>>>>>> Phase C - Heater Control >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>                                            
                                                                   if(SM_GRIDC_P_rec_counter >= SM_GRIDC_P_rec_interval) {  // excess Power message for Phase C, not received in predefined periode 
                                                                                                                                                                        SM_GRIDC_P_rec_counter = 0;
                                                                                                                                                                        if(!boiler_quickheat_flag) {  // Boost Mode not enabled, so we can turn OFF Heater C
                                                                                                                                                                                                      ledcWrite(PWM_C_channel, 0);  // turn OFF the heater C
                                                                                                                                                                                                      calcC_power = 0;              // Display 0 for "DivB Power", on TFT
                                                                                                                                                                                                      divertC_power = 0;            // Reinit the variable
                                                                                                                                                                                                     }
                                                                                                                                                                        
                                                                                                                                                                    }
                                                                   else {   
                                                                             // message received OK, Heatsink temperature OK  =>  heat with excess power                                             
                                                                                                                                  
                                                                             if(SM_GRIDC_P_rec_flag and !divertA_temp_error){  // Phase C Power message received flag and Divert A driver module temperature OK
                                                                                                                               SM_GRIDC_P_rec_flag = false; // clear the flag
                                                                                                                               
                                                                                                                               if (!GridV_control){ // "Reduce Grid Voltage" Mode is OFF
                                                                                                                                                      set_pwm_DIVERTC();  // set Power for Heater C
                                                                                                                                                  }
                                                                                                                               else {     // "Reduce Grid Voltage" Mode ON with small ON/OFF hysteresis
                                                                                                                                          
                                                                                                                                          if ((sm_gridc_v < min_voltage_heaterOFF) and keepon_flag_c)  {  // Grid voltage below MIN value (251.0V) and was turned ON above Max set value (252.0V)
                                                                                                                                                                                                        keepon_flag_c = false;
                                                                                                                                                                                                        ledcWrite(PWM_C_channel, 0);  // turn OFF the heater C
                                                                                                                                                                                                        calcC_power = 0;  // Display 0 for "DivC Power", on TFT
                                                                                                                                                                                                        divertC_power = 0;            // Reinit the variable
                                                                                                                                                                                                      }
                                                                                                                                          else if((sm_gridc_v > max_voltage_warning) or keepon_flag_c)  {  // if voltage on Phase C, is above set value (252.0V) or it was already turned ON at Max value (252.0V)  
                                                                                                                                                                                                         keepon_flag_c = true; // keep on until Grid Voltage falls below set MIN value (251.0V)
                                                                                                                                                                                                         set_pwm_DIVERTC();                  
                                                                                                                                                                                                      }
                                                                                                                                     }
                                                                                                       
                                                                                                       
                                                                             }
                                                                            else if(divertA_temp_error)  {  // Divert A driver module temperature too HIGH or sensor Error
                                                                                                            ledcWrite(PWM_C_channel, 0);  // turn OFF the heater C
                                                                                                            calcC_power = 0;              // Display 0 for "DivA/B/C Power", on TFT
                                                                                                            divertC_power = 0;            // Reinit the variable
                                                                                                            
                                                                                                          }
                                                                   }
                                                                // >>>>>>>>>>>> END Phase C - Heater Control 
                                                            }
                                 }
                           }
                       
}

// Request message from Modbus Slave devices (Smartmeter / Inverter)
void requestmessage(){
                      // Create request for
                      // - token
                      // - server ID = 1
                      // - function code = 0x03 (read holding register)
                      // - start address to read = word 10
                      // - number of words to read = 2
                      // - token to match the response with the request. We take the current millis() value for it.
                      //
                      // If something is missing or wrong with the call parameters, we will immediately get an error code 
                      // and the request will not be issued
                      //Serial.printf("sending request with token %lu \n", lastMillis);
                     
                      // err = MB.addRequest((uint32_t)lastMillis, 1, READ_HOLD_REGISTER, 10, 4);

                      //Error err;
                                     
       
                      switch (request_counter){

                                                 case 0: 
                                                        MB.addRequest(SM_ACTIV_P, 1, READ_HOLD_REGISTER, SM_ACTIV_P, 2); // SmartMeter Total Active Power: 37113
                                                        break;
                                                 case 1: 
                                                        MB.addRequest(IN_ACTIV_P, 1, READ_HOLD_REGISTER, IN_ACTIV_P, 2); // Inverter Total Active Power 32080
                                                        IN_ACTIV_P_rec_counter++;
                                                        break;
                                                 case 2: 
                                                        MB.addRequest(SM_GRIDA_V, 1, READ_HOLD_REGISTER, SM_GRIDA_V, 2); // Smart Meter Grid A Voltage 37101
                                                        break;
                                                 case 3:
                                                        MB.addRequest(SM_GRIDB_V, 1, READ_HOLD_REGISTER, SM_GRIDB_V, 2); // Smart Meter Grid B Voltage 37103
                                                        break;
                                                 case 4:   
                                                        MB.addRequest(SM_GRIDC_V, 1, READ_HOLD_REGISTER, SM_GRIDC_V, 2); // Smart Meter Grid C Voltage 37105
                                                        break;
                                                 case 5:
                                                        MB.addRequest(SM_GRIDA_P, 1, READ_HOLD_REGISTER, SM_GRIDA_P, 2); // Smart Meter Grid A Power 37132
                                                        SM_GRIDA_P_rec_counter++;
                                                        break;
                                                 case 6:
                                                        MB.addRequest(SM_GRIDB_P, 1, READ_HOLD_REGISTER, SM_GRIDB_P, 2); // Smart Meter Grid B Power 37134
                                                        SM_GRIDB_P_rec_counter++;
                                                        break;
                                                 case 7:
                                                        MB.addRequest(SM_GRIDC_P, 1, READ_HOLD_REGISTER, SM_GRIDC_P, 2); // Smart Meter Grid C Power 37136
                                                        SM_GRIDC_P_rec_counter++;
                                                        break;   
                                                 //default:    
                      }

                      request_counter++;
                      if(request_counter > max_message_number) request_counter = 0;
                      
                      check_conditions();  // check conditions to set all heaters Power / PWM channels
                      
                                               
                      
                      //  if (err != SUCCESS) {
                      //   ModbusError e(err);
                      //   Serial.printf("Error creating request: %02X - %s\n", (int)e, (const char *)e);
                      // }
                      // Else the request is processed in the background task and the onData/onError handler functions will get the result.
                      //
                      // The output on the Serial Monitor will be (depending on your WiFi and Modbus the data will be different):
                      //     __ OK __
                      //     . WIFi IP address: 192.168.178.74
                      //     Response: serverID=20, FC=3, Token=0000056C, length=11:
                      //     14 03 04 01 F6 FF FF FF 00 C0 A8
  
  }

void measure_temp(){
                       
                       //SINGLE_THREADED_BLOCK() {
                       sensors.requestTemperatures();
                       
                       boiler_temp = sensors.getTempC(sensor_boiler);  // boiler temperature
                       divertA_temp = sensors.getTempC(sensor_A);      // triac/mosfet heatsink temperature
                       //ambient_temp = sensors.getTempC(sensor_amb);
                      // }
                       
                       if((boiler_temp < boiler_LOW_Error) or (boiler_temp > boiler_HIGH_Warning)){
                                                                                                     boiler_temp_error = true;
                                                                                                     Serial.println("Boiler temp. sensor ERROR");
                       }
                       else {
                               
                               boiler_temp_error = false;                                                                                                                                                                                
                       }

                       if(BOOST_mode and (boiler_temp < boiler_LOW_Warning)){  Serial.println("Boiler temp. too LOW! BOOST Mode ON!");
                                                               
                                                                           }
                       
                       if ((divertA_temp < divertA_LOW_Warning) or (divertA_temp > divertA_HIGH_Warning))  { 
                                                                                                              divertA_temp_error = true;
                                                                                                              Serial.println("Heatsink temp. ERROR! Heaters => OFF!");       
                                                                                                             }
                       else  divertA_temp_error = false;
                                                                    
                                              
                       Serial.print("Boiler (C): ");
                       Serial.println(boiler_temp);
                       Serial.print("Heatsink (C): ");
                       Serial.println(divertA_temp);
                       //Serial.print("Ambient (C): ");
                       //Serial.println(ambient_temp);
                       
                       
}

void set_pwm_backlight(){
                           // set backlight percent 
                           if(ledBacklight_percent!=ledBacklight_percent_buffer){  // value was changed?
                                                                                   ledBacklight_percent_buffer = ledBacklight_percent;
                                                                                   ledBacklight = map(ledBacklight_percent,0,100,0,255); // input value from Blynk 0>100, output value to PWM 0>255
                                                                                   ledcWrite(pwmLedChannelTFT, ledBacklight);
                           }
                           Serial.print("TFT Backlight (%): ");
                           Serial.println(ledBacklight_percent);
  
}
// Set PWM for Diverter Phase A Module 
void set_pwm_DIVERTA(){
                        
                            if((sm_grida_p > -100) and (sm_grida_p < 100)) divertA_cache = 0;
                            divertA_power += divertA_cache; 
                           
                            if(divertA_power < 50) divertA_power = 0;  // minimum power to increase divert power (50W)
                            else if(divertA_power > MaxPower) divertA_power = MaxPower; // excess power more than MaxPower (heater power = 2000W)
                            if (divertA_power != divertA_last){ //value was changed?
                                                                divertA_last = divertA_power;
                                                                calib_index = divertA_power / onecalib_point;  // select index from lookup table => measured power on Phase A / 50W for 2000W heater (2000W / 40[calib points]) 
                                                                lookup_divertA_PWM = lookup_PWM[calib_index];  // lookup PWM value
                                                                ledcWrite(PWM_A_channel, lookup_divertA_PWM);  // set PWM value
                                                                calcA_power = calib_index * onecalib_point;    // calculated heater power on Phase A, to display on TFT
                                                                //PWM_A_percent = map(lookup_divertA_PWM, 0, 255, 0, 100);
                            }
                            
                            Serial.print("Divert A Power: ");
                            Serial.println(calcA_power);
                            
                           
}

// Set PWM for Diverter Phase B Module 
void set_pwm_DIVERTB(){
                        
                            if((sm_gridb_p > -100) and (sm_gridb_p < 100)) divertB_cache = 0;
                            divertB_power += divertB_cache; 
                           
                            if(divertB_power < 50) divertB_power = 0;  // minimum power to increase divert power (50W)
                            else if(divertB_power > MaxPower) divertB_power = MaxPower; // excess power more than MaxPower (heater power = 2000W)
                            if (divertB_power != divertB_last){ //value was changed?
                                                                divertB_last = divertB_power;
                                                                calib_index = divertB_power / onecalib_point;  // select index from lookup table => measured power on Phase A / 50W for 2000W heater (2000W / 40[calib points]) 
                                                                lookup_divertB_PWM = lookup_PWM[calib_index];  // lookup PWM value
                                                                ledcWrite(PWM_B_channel, lookup_divertB_PWM);  // set PWM value
                                                                calcB_power = calib_index * onecalib_point;    // calculated heater power on Phase A, to display on TFT
                                                                //PWM_B_percent = map(lookup_divertB_PWM, 0, 255, 0, 100);
                            }
                            
                            Serial.print("Divert B Power: ");
                            Serial.println(calcB_power);
                            
                           
}

// Set PWM for Diverter Phase C Module 
void set_pwm_DIVERTC(){
                        
                            if((sm_gridc_p > -100) and (sm_gridc_p < 100)) divertC_cache = 0;
                            divertC_power += divertC_cache; 
                           
                            if(divertC_power < 50) divertC_power = 0;  // minimum power to increase divert power (50W)
                            else if(divertC_power > MaxPower) divertC_power = MaxPower; // excess power more than MaxPower (heater power = 2000W)
                            if (divertC_power != divertC_last){ //value was changed?
                                                                divertC_last = divertC_power;
                                                                calib_index = divertC_power / onecalib_point;  // select index from lookup table => measured power on Phase A / 50W for 2000W heater (2000W / 40[calib points]) 
                                                                lookup_divertC_PWM = lookup_PWM[calib_index];  // lookup PWM value
                                                                ledcWrite(PWM_C_channel, lookup_divertC_PWM);  // set PWM value
                                                                calcC_power = calib_index * onecalib_point;    // calculated heater power on Phase A, to display on TFT
                                                                //PWM_C_percent = map(lookup_divertC_PWM, 0, 255, 0, 100);
                            }
                            
                            Serial.print("Divert C Power: ");
                            Serial.println(calcC_power);
                            
                           
}


void display_time(){ 
                               if(clicked){  // clear the display, because of different textPadding on pages
                                             clicked = 0;
                                             tft.fillScreen(TFT_BLACK); 
                                           }
                                                                                                    
                            switch (button){  // select page to display
                                                                                      
                                                case 0: page1(); break; // Divert A/B/C + Boiler/Heatsink Temp
                                                case 1: page2(); break; // Grid Total + Solar Total
                                                case 2: page3(); break; // Grid A/B/C Power + Grid A/B/C Voltage 
                                                
                                           }   

}

// Arduino OTA handle
void OTA_check(){
                    ArduinoOTA.handle(); 
}

//Print Temp sensor address
void printAddress(DeviceAddress deviceAddress) {
                                                    for (uint8_t i = 0; i < 8; i++){
                                                                                      Serial.print(", 0x");
                                                                                      Serial.print(deviceAddress[i], HEX);
                                                                                   }
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//--------------------------------- Buttons clicked
void click(Button2& btn) {
                            clicked = 1;
                            
                            if (btn == buttonB) {
                                                  if(button) button--;
                                                  else button = pagenumbers;  // number of pages -1
                                                  
                                                  // for calibration only (lookup table)
                                                  //PWM_A_DutyCycle--;
                                                  //set_pwm_DIVERTA();
                                                  
                                                  //DEBUG_PORT.println("A clicked");
                                                } 
                            else if (btn == buttonA) {
                                                        if(button < pagenumbers) button++;
                                                        else button = 0;
                                                  
                                                        // for calibration only (lookup table)
                                                        //PWM_A_DutyCycle++;
                                                        //set_pwm_DIVERTA();
                                                        
                                                        //DEBUG_PORT.println("B clicked");
                                                      }
                         }

//--------------------------------- END Buttons Clicked

//Not used --------------------------------- Check Button Longpress
void longpress(Button2& btn) {  // Turn the Boiler ON/OFF
                                unsigned int time = btn.wasPressedFor();
                                if (time > 2000) {
                                                   Serial.println("Button A longpressed");
                                                   //boiler_onoff = !boiler_onoff;
                                                   // do something
                                                 }
}
//--------------------------------- END Check Button Longpress

// Display text/values >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

//Not Used =====>  Home Temperature
void HOMEtemperature(boolean topbottom){ 
                                                           
                              
                                          tft.setTextColor(TFT_WHITE,TFT_BLUE);
                                          if (topbottom) { // top = 1
                                                           tft.drawString("    Home   C   ", 0, toptext, 4); 
                                                           //check warning levels            
                                                           //if((temp_average < HOMEmintemp_Warning) || (temp_average > HOMEmaxtemp_Warning)) draw_warningbox_top();
                                                           //else  draw_normalbox_top();
                                                           draw_normalbox_top();
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawFloat( temp_average,1, x, topdraw, 6);
                                                         }
                                          else           { // bottom = 0
                                                           tft.drawString("    Home   C   ", 0, bottomtext, 4); 
                                                           //check warning levels            
                                                           //if((temp_average < HOMEmintemp_Warning) || (temp_average > HOMEmaxtemp_Warning)) draw_warningbox_bottom();
                                                           //else  draw_normalbox_bottom();
                                                           draw_normalbox_bottom();
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawFloat( temp_average,1, x, bottomdraw, 6);
                                                         }   
  
}

//=====>  Smart Meter Active Power
void SM_ActivePower(boolean topbottom){ 
                                                           
                              
                                          tft.setTextColor(TFT_WHITE,TFT_BLUE);
                                          if (topbottom) { // top = 1
                                                           tft.drawString("    Grid   kW   ", 0, toptext, 4); 
                                                           if(sm_activ_p < 0) draw_warningbox_top();  // Power from Grid
                                                           else  draw_greenbox_top();  // Power to Grid
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawFloat(abs((float)sm_activ_p)/1000,2, x, topdraw, 6);
                                                         }
                                          else           { // bottom = 0
                                                           tft.drawString("    Grid   kW   ", 0, bottomtext, 4); 
                                                           if(sm_activ_p < 0) draw_warningbox_bottom();  // Power from Grid
                                                           else  draw_greenbox_bottom();  // Power to Grid
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawFloat(abs((float)sm_activ_p)/1000,2, x, bottomdraw, 6);
                                                         }   
            
}

//=====>  Inverter Active Power
void IN_ActivePower(boolean topbottom){ 
                                                           
                              
                                          tft.setTextColor(TFT_WHITE,TFT_BLUE);
                                          if (topbottom) { // top = 1
                                                           tft.drawString("   Solar   kW   ", 0, toptext, 4); 
                                                           draw_greenbox_top();
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawFloat((float)in_activ_p/1000,2, x, topdraw, 6);
                                                         }
                                          else           { // bottom = 0
                                                           tft.drawString("   Solar   kW   ", 0, bottomtext, 4); 
                                                           draw_greenbox_bottom();
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawFloat((float)in_activ_p/1000,2, x, bottomdraw, 6);
                                                         }   
                
}

//=====> Smart Meter Grid Phase A Voltage
void SM_GridaVoltage(boolean topbottom){ 
                                                           
                              
                                          tft.setTextColor(TFT_WHITE,TFT_BLUE);
                                          if (topbottom) { // top = 1
                                                           tft.drawString("   Grid A    V   ", 0, toptext, 4); 
                                                           if(sm_grida_v >= max_voltage_warning) draw_warningbox_top();  // Grid Voltage critical
                                                           else  draw_normalbox_top();   
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawNumber(sm_grida_v, x, topdraw, 6);
                                                           
                                                         }
                                          else           { // bottom = 0
                                                           tft.drawString("   Grid A    V   ", 0, bottomtext, 4); 
                                                           if(sm_grida_v >= max_voltage_warning) draw_warningbox_bottom();  // Grid Voltage critical
                                                           else  draw_normalbox_bottom();
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawNumber(sm_grida_v, x, bottomdraw, 6);
                                                         }   
           
}

//=====> Smart Meter Grid Phase B Voltage
void SM_GridbVoltage(boolean topbottom){ 
                                                           
                              
                                          tft.setTextColor(TFT_WHITE,TFT_BLUE);
                                          if (topbottom) { // top = 1
                                                           tft.drawString("   Grid B    V   ", 0, toptext, 4); 
                                                           if(sm_gridb_v >= max_voltage_warning) draw_warningbox_top();  // Grid Voltage critical
                                                           else  draw_normalbox_top();   
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawNumber(sm_gridb_v, x, topdraw, 6);
                                                           
                                                         }
                                          else           { // bottom = 0
                                                           tft.drawString("   Grid B    V   ", 0, bottomtext, 4); 
                                                           if(sm_gridb_v >= max_voltage_warning) draw_warningbox_bottom();  // Grid Voltage critical
                                                           else  draw_normalbox_bottom();
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawNumber(sm_gridb_v, x, bottomdraw, 6);
                                                         }   
                       
}

//=====> Smart Meter Grid Phase C Voltage
void SM_GridcVoltage(boolean topbottom){ 
                                                           
                              
                                          tft.setTextColor(TFT_WHITE,TFT_BLUE);
                                          if (topbottom) { // top = 1
                                                           tft.drawString("   Grid C    V   ", 0, toptext, 4); 
                                                           if(sm_gridc_v >= max_voltage_warning) draw_warningbox_top();  // Grid Voltage critical
                                                           else  draw_normalbox_top();   
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawNumber(sm_gridc_v, x, topdraw, 6);
                                                           
                                                         }
                                          else           { // bottom = 0
                                                           tft.drawString("   Grid C    V   ", 0, bottomtext, 4); 
                                                           if(sm_gridc_v >= max_voltage_warning) draw_warningbox_bottom();  // Grid Voltage critical
                                                           else  draw_normalbox_bottom();
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawNumber(sm_gridc_v, x, bottomdraw, 6);
                                                         }   
           
}

//=====>  Smart Meter Grid Phase A Power
void SM_GridaPower(boolean topbottom){ 
                                                           
                              
                                        tft.setTextColor(TFT_WHITE,TFT_BLUE);
                                        if (topbottom) { // top = 1
                                                         tft.drawString("     Grid A   kW  ", 0, toptext, 4); 
                                                         if(sm_grida_p < 0) draw_warningbox_top();  // Power from Grid
                                                         else  draw_greenbox_top();  // Power to Grid
                                                         tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                         tft.drawFloat(abs((float)sm_grida_p)/1000,2, x, topdraw, 6);
                                                       }
                                        else           { // bottom = 0
                                                         tft.drawString("     Grid A   kW  ", 0, bottomtext, 4); 
                                                         if(sm_grida_p < 0) draw_warningbox_bottom();  // Power from Grid
                                                         else  draw_greenbox_bottom();  // Power to Grid
                                                         tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                         tft.drawFloat(abs((float)sm_grida_p)/1000,2, x, bottomdraw, 6);
                                                       }   
                      
}

//=====>  Smart Meter Grid Phase B Power
void SM_GridbPower(boolean topbottom){ 
                                                           
                              
                                        tft.setTextColor(TFT_WHITE,TFT_BLUE);
                                        if (topbottom) { // top = 1
                                                         tft.drawString("     Grid B   kW  ", 0, toptext, 4); 
                                                         if(sm_gridb_p < 0) draw_warningbox_top();  // Power from Grid
                                                         else  draw_greenbox_top();  // Power to Grid
                                                         tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                         tft.drawFloat(abs((float)sm_gridb_p)/1000,2, x, topdraw, 6);
                                                       }
                                        else           { // bottom = 0
                                                         tft.drawString("     Grid A   kW  ", 0, bottomtext, 4); 
                                                         if(sm_gridb_p < 0) draw_warningbox_bottom();  // Power from Grid
                                                         else  draw_greenbox_bottom();  // Power to Grid
                                                         tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                         tft.drawFloat(abs((float)sm_gridb_p)/1000,2, x, bottomdraw, 6);
                                                       }   
                      
}

//=====>  Smart Meter Grid Phase C Power
void SM_GridcPower(boolean topbottom){ 
                                                           
                              
                                        tft.setTextColor(TFT_WHITE,TFT_BLUE);
                                        if (topbottom) { // top = 1
                                                         tft.drawString("     Grid C   kW  ", 0, toptext, 4); 
                                                         if(sm_gridc_p < 0) draw_warningbox_top();  // Power from Grid
                                                         else  draw_greenbox_top();  // Power to Grid
                                                         tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                         tft.drawFloat(abs((float)sm_gridc_p)/1000,2, x, topdraw, 6);
                                                       }
                                        else           { // bottom = 0
                                                         tft.drawString("     Grid C   kW  ", 0, bottomtext, 4); 
                                                         if(sm_gridc_p < 0) draw_warningbox_bottom();  // Power from Grid
                                                         else  draw_greenbox_bottom();  // Power to Grid
                                                         tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                         tft.drawFloat(abs((float)sm_gridc_p)/1000,2, x, bottomdraw, 6);
                                                       }   
                      
}

//=====>  Boiler Temperature 
void boiler_temperature(boolean topbottom){
                                            
                                             tft.setTextColor(TFT_WHITE,TFT_BLUE);
                                             if (topbottom) { // top = 1
                                                                 tft.drawString("    Boiler   C   ", 0, toptext, 4);
                                                                 if(boiler_temp < boiler_LOW_Warning || boiler_temp > boiler_HIGH_Warning) draw_warningbox_top();
                                                                 else  draw_normalbox_top();
                                                                 tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                                 tft.drawFloat( boiler_temp, 1, x, topdraw, 6); //1 decimal places , x = center
                                                            }
                                             else           { // bottom = 0
                                                                 tft.drawString("    Boiler   C   ", 0, bottomtext, 4);
                                                                 if(boiler_temp < boiler_LOW_Warning || boiler_temp > boiler_HIGH_Warning) draw_warningbox_bottom();
                                                                 else  draw_normalbox_bottom();
                                                                 tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                                 tft.drawFloat( boiler_temp, 1, x, bottomdraw, 6); //1 decimal places , x = center
                                                            }    
}

//=====> Heatsink = Diverter Driver A/B/C temperatures
void divertA_temperature(boolean topbottom){
                                            
                                             tft.setTextColor(TFT_WHITE,TFT_BLUE);
                                             if (topbottom) { // top = 1
                                                                 tft.drawString("    Heats    C   ", 0, toptext, 4);
                                                                 if(divertA_temp < divertA_LOW_Warning || divertA_temp > divertA_HIGH_Warning) draw_warningbox_top();
                                                                 else  draw_normalbox_top();
                                                                 tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                                 tft.drawFloat( divertA_temp, 1, x, topdraw, 6); //1 decimal places , x = center
                                                            }
                                             else           { // bottom = 0
                                                                 tft.drawString("    Heats    C   ", 0, bottomtext, 4);
                                                                 if(divertA_temp < divertA_LOW_Warning || divertA_temp > divertA_HIGH_Warning) draw_warningbox_bottom();
                                                                 else  draw_normalbox_bottom();
                                                                 tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                                 tft.drawFloat( divertA_temp, 1, x, bottomdraw, 6); //1 decimal places , x = center
                                                            }    
}

/* Not used
//=====> Ambient temperature
void ambient_temperature(boolean topbottom){
                                            
                                             tft.setTextColor(TFT_WHITE,TFT_BLUE);
                                             if (topbottom) { // top = 1
                                                                 tft.drawString("  Ambient   C  ", 0, toptext, 4);
                                                                 //if(divertC_temp < divertC_LOW_Warning || divertC_temp > divertC_HIGH_Warning) draw_warningbox_top();
                                                                 //else  draw_normalbox_top();
                                                                 draw_normalbox_top();
                                                                 tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                                 tft.drawFloat( ambient_temp, 1, x, topdraw, 6); //1 decimal places , x = center
                                                            }
                                             else           { // bottom = 0
                                                                 tft.drawString(" Ambient  C ", 0, bottomtext, 4);
                                                                 draw_normalbox_bottom();
                                                                 tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                                 tft.drawFloat( ambient_temp, 1, x, bottomdraw, 6); //1 decimal places , x = center
                                                            }    
}
*/


//=====> Calculated Divert A Power
void divA_power(boolean topbottom){ 
                                                           
                              
                                          tft.setTextColor(TFT_WHITE,TFT_BLUE);
                                          if (topbottom) { // top = 1
                                                           tft.drawString("    Div.A    kW  ", 0, toptext, 4); 
                                                           draw_normalbox_top(); 
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawFloat((float)calcA_power/1000,2, x, topdraw, 6);
                                                           
                                                         }
                                          else           { // bottom = 0
                                                           tft.drawString("    Div.A    kW  ", 0, bottomtext, 4); 
                                                           draw_normalbox_bottom();
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawFloat((float)calcA_power/1000,2, x, bottomdraw, 6);
                                                         }   
           
}

//=====> Calculated Divert B Power
void divB_power(boolean topbottom){ 
                                                           
                              
                                          tft.setTextColor(TFT_WHITE,TFT_BLUE);
                                          if (topbottom) { // top = 1
                                                           tft.drawString("    Div.B    kW  ", 0, toptext, 4); 
                                                           draw_normalbox_top(); 
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawFloat((float)calcB_power/1000,2, x, topdraw, 6);
                                                           
                                                         }
                                          else           { // bottom = 0
                                                           tft.drawString("    Div.B    kW  ", 0, bottomtext, 4); 
                                                           draw_normalbox_bottom();
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawFloat((float)calcB_power/1000,2, x, bottomdraw, 6);
                                                         }   
           
}

//=====> Calculated Divert C Power
void divC_power(boolean topbottom){ 
                                                           
                              
                                          tft.setTextColor(TFT_WHITE,TFT_BLUE);
                                          if (topbottom) { // top = 1
                                                           tft.drawString("    Div.C    kW  ", 0, toptext, 4); 
                                                           draw_normalbox_top(); 
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawFloat((float)calcC_power/1000,2, x, topdraw, 6);
                                                           
                                                         }
                                          else           { // bottom = 0
                                                           tft.drawString("    Div.C    kW  ", 0, bottomtext, 4); 
                                                           draw_normalbox_bottom();
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawFloat((float)calcC_power/1000,2, x, bottomdraw, 6);
                                                         }   
           
}
  
//--------------------------------- Screen Connected
void connectedscreen() {
                          
                          tft.setTextColor(TFT_BLUE);
                          tft.setCursor(4, 35);
                          tft.setFreeFont(&Orbitron_Light_24);
                          
                          if(WiFi.status() != WL_CONNECTED) tft.println("  No WiFi");
                          else {
                                 tft.println("    WiFi ");
                                 tft.setCursor(0, 70);
                                 tft.println("      OK!"); 
                                }
                                        
                          delay(2000);
                          
                          tft.fillScreen(TFT_BLACK);
}
//-------------------------------- END Screen Connected

//-------------------------------- Draw objects Begin
void init_background(){
                         tft.setTextPadding(0);
                         tft.drawRect(0, 6, 135, 104, TFT_BLUE); // Blue rectangle
                         tft.drawRect(0, 131, 135, 104, TFT_BLUE); // Blue rectangle
                         //tft.drawLine(0, 120, 135, 120, TFT_BLUE);
                         
}

void draw_warningbox_top(){
                         tft.setTextColor(TFT_YELLOW,TFT_RED); 
                         tft.fillRect(7, 39, 122, 65, TFT_RED);
}

void draw_normalbox_top(){
                         tft.setTextColor(TFT_YELLOW,TFT_BLACK);
                         tft.fillRect(7, 39, 122, 65, TFT_BLACK); 
}

void draw_greenbox_top(){
                         tft.setTextColor(TFT_BLACK,TFT_GREEN); 
                         tft.fillRect(7, 39, 122, 65, TFT_GREEN);
}

void draw_warningbox_bottom(){
                         tft.setTextColor(TFT_YELLOW,TFT_RED); 
                         tft.fillRect(7, 163, 122, 65, TFT_RED);
}

void draw_normalbox_bottom(){
                         tft.setTextColor(TFT_YELLOW,TFT_BLACK);
                         tft.fillRect(7, 163, 122, 65, TFT_BLACK); 
}

void draw_greenbox_bottom(){
                         tft.setTextColor(TFT_BLACK,TFT_GREEN); 
                         tft.fillRect(7, 163, 122, 65, TFT_GREEN);
}
//--------------------------------- Draw objects END

// here you can config your Display as you wish
//--------------------------------- Display Page 1
void page1(){   
                init_background(); // init background image
               
                //SM_ActivePower(1);    // display Smart Meter Total Active Power
                disp_counter++; 
                switch (disp_counter) {   // change display values every 2 seconds 
                                        case 2:   divA_power(1); boiler_temperature(0); break; // display divA_power on TOP and HotWater temperature on BOTTOM
                                        case 4:   divB_power(1); divertA_temperature(0); break; // display divB_power on TOP and Heatsink temperature on BOTTOM 
                                        case 6:   divC_power(1); boiler_temperature(0); disp_counter = 0;  break;
                                       }
                
                                          
}
//--------------------------------- END Display Page 1

//--------------------------------- Display Page 2
void page2(){   
                init_background(); // init background image

                SM_ActivePower(1);    // display Smart Meter Total Active Power
                IN_ActivePower(0);    // display Inverter Total Active Power
               
}
//--------------------------------- END Display Page 2

//--------------------------------- Display Page 3
void page3(){   
                init_background(); // init background image

                disp_counter++; 
                switch (disp_counter) {   // change display values every 2 seconds 
                                        case 2:   SM_GridaPower(1);  SM_GridaVoltage(0); break; // display divA_power on TOP and HotWater temperature on BOTTOM
                                        case 4:   SM_GridbPower(1);  SM_GridbVoltage(0); break; // display divB_power on TOP and Heatsink temperature on BOTTOM 
                                        case 6:   SM_GridcPower(1);  SM_GridcVoltage(0); disp_counter = 0; break;
                                       }

                                
}
//--------------------------------- END Display Page 3
