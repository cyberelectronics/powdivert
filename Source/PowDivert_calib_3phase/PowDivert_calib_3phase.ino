//PowDivert v1.0 =>  3Phase Solar Excess Power Diverter for MPDMv4.1 PWM Dimmer and ESP32 TTGO Display module ------ by CyberElectronics 
//=================================================================================================
// eModbus: Copyright 2020 by Michael Harwerth, Bert Melis and the contributors to ModbusClient
//               MIT license - see license.md for details
// =================================================================================================

// Example code to show the usage of the eModbus library. 
// Please refer to root/Readme.md for a full description.

// Includes: <Arduino.h> for Serial etc., WiFi.h for WiFi support


#include <SPIFFS.h>
#include <EEPROM.h>
#include "Tickerstaub.h"

#include <OneWire.h>
#include <DallasTemperature.h>

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
// Config ==================================================
// Define Smartmeter / Huawei Inverter Register Address

// Modify here the Temperature sensor Address, displayed on Serial Monitor after boot
DeviceAddress sensor_boiler = { 0x28, 0x54, 0x2B, 0x79, 0x97, 0x15, 0x3, 0xFC };
DeviceAddress sensor_A =      { 0x28, 0xFA, 0x32, 0x72, 0x4D, 0x20, 0x1, 0xB5 };
DeviceAddress sensor_B =      { 0x28, 0x56, 0x2E, 0x7, 0xD6, 0x1, 0x3C, 0x4F };
DeviceAddress sensor_C =      { 0x28, 0xE3, 0x68, 0x7, 0xD6, 0x1, 0x3C, 0xD6 };
DeviceAddress sensor_amb =    { 0x28, 0xA7, 0x65, 0x7, 0xD6, 0x1, 0x3C, 0x93 };

#define SM_ACTIV_P 37113         // SmartMeter Total Active Power register address
#define SM_GRIDA_V 37101         // SmartMeter Grid Phase A Voltage
#define SM_GRIDB_V 37103         // SmartMeter Grid Phase B Voltage
#define SM_GRIDC_V 37105         // SmartMeter Grid Phase C Voltage
#define SM_GRIDA_P 37132         // SmartMeter Grid Phase A Power
#define SM_GRIDB_P 37134         // SmartMeter Grid Phase B Power
#define SM_GRIDC_P 37136         // SmartMeter Grid Phase C Power
#define IN_ACTIV_P 32080         // Inverter Total Active Power

#define MaxPower 2000            // Heater Max Power (W)

#define request_interval 1000       // request 1 Modbus message, every 1 seconds
#define temp_interval 10000         // measure temperature every 10 seconds
#define pagenumbers 6               // number of pages to display -1 
#define max_message_number 7        // max number of messages request from inverter/smartmeter
#define max_voltage_warning 250     // Display Red Color above this value

#define car_off_delay 5             // Delay in minutes to send a notification (TTS) after the car was stoppped
#define boiler_LOW_Warning 35.0     // HotWater Low temperature warning (RED color below this value)
#define boiler_HIGH_Warning 70.0    // HotWater High temperature warning (RED color above this value)
#define divertA_LOW_Warning -30.0     // Temp sensor Error
#define divertA_HIGH_Warning 80.0     // Temp sensor Error or Temp too high
#define divertB_LOW_Warning -30.0     // Temp sensor Error
#define divertB_HIGH_Warning 80.0     // Temp sensor Error or Temp too high
#define divertC_LOW_Warning -30.0     // Temp sensor Error
#define divertC_HIGH_Warning 80.0     // Temp sensor Error or Temp too high


#define ning 30.0    // RED color above this temperature value (Celsius)
#define HOMEminhumi_Warning 30.0    // RED color below this humidity value (%)
#define HOMEmaxhumi_Warning 60.0    // RED color above this humidity value (%)
#define BATTtemp_Warning 40.0       // RED color above this value (Celsius)
#define SOCpercent_Warning 10       // RED color below this value (Percent)
#define AUXSOCpercent_Warning 30    // RED color below this value (Percent)
#define BATTv_LOW_Warning 320       // Main Battery Low Voltage warning (Volts)
#define BATTv_HIGH_Warning 420      // Main Battery High Voltage warning (Volts)
#define AUXBATTv_LOW_Warning 12.2   // Main Battery Low Voltage warning (Volts)
#define AUXBATTv_HIGH_Warning 15.0  // Main Battery High Voltage warning (Volts)


int ledBacklight = 80; // Initial TFT backlight intensity on a scale of 0 to 100 percent. Initial value is 80.

bool debugmode = true;


// END Config ================================================
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

// Include the header for the ModbusClient TCP style
#include <ModbusClientTCPasync.h>

#ifndef MY_SSID
#define MY_SSID "Telelink_2.4G" 
//#define MY_SSID "Home1"
#endif
#ifndef MY_PASS
#define MY_PASS "al13nc0nc3pt" 
//#define MY_PASS "22222222"
#endif

// Setting PWM properties, do not change this!
const int pwmFreq = 1000;
const int pwmResolution = 8;
const int pwmLedChannelTFT = 0;
const int PWM_A_channel = 1;
const int PWM_B_channel = 2;
const int PWM_C_channel = 3;


byte PWM_A_DutyCycle = 0;
byte PWM_B_DutyCycle = 0;
byte PWM_C_DutyCycle = 0;

byte PWM_A_percent; 
int divertA_power, divertA_cache, divertA_last;

int sec_interval = 10;  // sec_interval/10 = sec.
int sensor_read_period;
int sensor_calc_period;

float temp_average; 
float boiler_temp, divertA_temp, divertB_temp, divertC_temp, ambient_temp;   // boiler temperature from Boiler Device

uint16_t value;
int16_t sm_activ_p;
float sm_grida_v;
float sm_gridb_v;
float sm_gridc_v;
int16_t sm_grida_p;
int16_t sm_gridb_p;
int16_t sm_gridc_p;
int16_t in_activ_p;

//uint32_t active_power;
uint8_t negative, request_counter, measure_temp_interval;
uint8_t sm_activ_p_neg, sm_grida_p_neg, sm_gridb_p_neg, sm_gridc_p_neg;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

uint16_t x,y;
byte button;
byte ledBacklight_percent, ledBacklight_percent_buffer;
boolean clicked;
boolean mbus_error;    // error flag - Huawei ModBus Not Responded

int ms_counter; // incremented every 100ms
int request_interval_counter; // incremented every 100ms

//define your default values here, if there are different values in config.json, they are overwritten.
char mqtt_server[40];
char mqtt_port[6] = "8080";
char blynk_token[40] = "";
char blynk_car_token[40] = "";  // Config from WebUI; Here leave it empty - Car device blynk token

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
IPAddress ip = {192, 168, 0, 101};          // IP address of modbus server
//IPAddress ip = {192, 168, 50, 206};          // IP address of modbus server
uint16_t port = 502;                      // port of modbus server

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

Ticker timer_modbus(requestmessage, request_interval); // 
Ticker timer_temperature(measure_temp, temp_interval); // 
Ticker timer_display(display_time, 1000); // display pages 


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
                          
                        negative = response[3];  // FF = check if the value is Negative
                        response.get(5, value);
                        
                        Serial.print(token);  // Print Register number
                        
                        switch(token){
                                       case SM_ACTIV_P:  // Smart Meter Total Active Power
                                                          sm_activ_p = value;
                                                          divertA_cache = sm_activ_p;
                                                          sm_activ_p_neg = negative;
                                                          Serial.print(" = SM Active Power [W] = ");  
                                                          if(sm_activ_p_neg) {
                                                                               sm_activ_p = 65535 - sm_activ_p;  // received negative value
                                                                               divertA_cache = -sm_activ_p;
                                                                               Serial.println(-sm_activ_p);
                                                                             }
                                                          else Serial.println(sm_activ_p);
                                                          //set_pwm_DIVERTA();
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
                                                          sm_grida_p_neg = negative;
                                                          Serial.print(" = SM Grid A Power [W] = ");  
                                                          if(sm_grida_p_neg) {
                                                                               sm_grida_p = 65535 - sm_grida_p;  // check byte 3 if FF => negative values
                                                                               Serial.println(-sm_grida_p);
                                                                             }
                                                          else Serial.println(sm_grida_p);
                                                          break;    
                                       case SM_GRIDB_P:  
                                                          sm_gridb_p = value;
                                                          sm_gridb_p_neg = negative;
                                                          Serial.print(" = SM Grid B Power [W] = ");  
                                                          if(sm_gridb_p_neg) {
                                                                               sm_gridb_p = 65535 - sm_gridb_p;  // check byte 3 if FF => negative values
                                                                               Serial.println(-sm_gridb_p);
                                                                             }
                                                          else Serial.println(sm_gridb_p);
                                                          break;    
                                       case SM_GRIDC_P:  
                                                          sm_gridc_p = value;
                                                          sm_gridc_p_neg = negative;
                                                          Serial.print(" = SM Grid C Power [W] = ");  
                                                          if(sm_gridc_p_neg) {
                                                                               sm_gridc_p = 65535 - sm_gridc_p;  // check byte 3 if FF => negative values
                                                                               Serial.println(-sm_gridc_p);
                                                                             }
                                                          else Serial.println(sm_gridc_p);
                                                          break;   
                                       case IN_ACTIV_P: // Inverter Total Active Power 
                                                          in_activ_p = value;
                                                          Serial.print(" = IN Active Power [W] = ");   
                                                          Serial.println(in_activ_p);
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
                
                
              
                tft.begin();
                tft.setRotation(0);  // 0 & 2 Portrait. 1 & 3 landscape
                tft.fillScreen(TFT_BLACK);
                
             
              
              // Connect to WiFi
                WiFi.begin(ssid, pass);
                delay(200);
                while (WiFi.status() != WL_CONNECTED) {
                  Serial.print(". ");
                  delay(1000);
                }
                IPAddress wIP = WiFi.localIP();
                Serial.printf("WIFi IP address: %u.%u.%u.%u\n", wIP[0], wIP[1], wIP[2], wIP[3]);

                
                
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
              
                connectedscreen();
              
                tft.setTextDatum(MC_DATUM);
                x = tft.width()/2;

                 timer_modbus.start();
                 timer_temperature.start();
                 timer_display.start();
  
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> loop()
void loop() {
                 buttonA.loop();
                 buttonB.loop();

                 //timer_modbus.update();
                 //timer_temperature.update();
                 timer_display.update();
    
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
                                                        break;
                                                 case 6:
                                                        MB.addRequest(SM_GRIDB_P, 1, READ_HOLD_REGISTER, SM_GRIDB_P, 2); // Smart Meter Grid B Power 37134
                                                        break;
                                                 case 7:
                                                        MB.addRequest(SM_GRIDC_P, 1, READ_HOLD_REGISTER, SM_GRIDC_P, 2); // Smart Meter Grid C Power 37136
                                                        break;   
                                                 //default:    
                      }

                      request_counter++;
                      if(request_counter > max_message_number) request_counter = 0;
                      
                      //set_pwm();                                    
                      
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
                       sensors.requestTemperatures();
                       
                       boiler_temp = sensors.getTempC(sensor_boiler);
                       divertA_temp = sensors.getTempC(sensor_A);
                       divertB_temp = sensors.getTempC(sensor_B);
                       divertC_temp = sensors.getTempC(sensor_C);
                       ambient_temp = sensors.getTempC(sensor_amb);
                       
                       
                       Serial.print("Boiler (C): ");
                       Serial.println(boiler_temp);
                       Serial.print("Div. A (C): ");
                       Serial.println(divertA_temp);
                       Serial.print("Div. B (C): ");
                       Serial.println(divertB_temp);
                       Serial.print("Div. C (C): ");
                       Serial.println(divertC_temp);
                       Serial.print("Ambient (C): ");
                       Serial.println(ambient_temp);
                       
                       
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

void set_pwm_DIVERTA(){
                        
                            if((sm_activ_p > -100) and (sm_activ_p < 100)) divertA_cache = 0;
                   
                            divertA_power += divertA_cache; 
                           
                            if(divertA_power < 100) divertA_power = 0;
                            else if(divertA_power > MaxPower) divertA_power = MaxPower;
                            /*if (divertA_power != divertA_last){ //value was changed?
                                                                divertA_last = divertA_power;
                                                                PWM_A_DutyCycle = map(divertA_power, 0, MaxPower, 0, 255);
                                                                ledcWrite(PWM_A_channel, PWM_A_DutyCycle);
                                                                PWM_A_percent = map(PWM_A_DutyCycle, 0, 255, 0, 100);
                            }
                            */
                            ledcWrite(PWM_A_channel, PWM_A_DutyCycle);
                            
                            //Serial.print("PWM A (%): ");
                            //Serial.print(PWM_A_percent);
                            Serial.print("PWM A value: ");
                            Serial.println(PWM_A_DutyCycle);
                            
                           // cycle = 0;
}

void set_pwm_DIVERTB(){
                            ledcWrite(PWM_B_channel, PWM_B_DutyCycle);
                            
                            Serial.print("PWM B value: ");
                            Serial.println(PWM_B_DutyCycle);
   
  }

void set_pwm_DIVERTC(){
                            ledcWrite(PWM_C_channel, PWM_C_DutyCycle);
                            
                            Serial.print("PWM C value: ");
                            Serial.println(PWM_C_DutyCycle);
   
  }


void set_pwm(){
                   
                   set_pwm_backlight();
                   set_pwm_DIVERTA();
  
  }


void display_time(){ 
                               if(clicked){  // clear the display, because of different textPadding on pages
                                             clicked = 0;
                                             tft.fillScreen(TFT_BLACK); 
                                           }
                                                                                                    
                            switch (button){  // select page to display
                                                                                      
                                                case 0: page1(); break; // Divert + boiler temp
                                                case 1: page2(); break; // Grid Total + Solar Total
                                                case 2: page3(); break; // Grid A Power + Grid A Voltage 
                                                case 3: page4(); break; // Grid B Power + Grid B Voltage 
                                                case 4: page5(); break; // Grid C Power + Grid C Voltage 
                                                case 5: page6(); break; // Divert A temp + Divert B temp
                                                case 6: page7(); break; // Divert C temp + boiler temp 
                                             // case 7: page8(); break;
                                           }   

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
                                                  //if(button) button--;
                                                  //else button = pagenumbers;  // number of pages -1
                                                  PWM_A_DutyCycle--;
                                                  PWM_B_DutyCycle--;
                                                  PWM_C_DutyCycle--;
                                                  set_pwm_DIVERTA();
                                                  set_pwm_DIVERTB();
                                                  set_pwm_DIVERTC();
                                                  
                                                  //DEBUG_PORT.println("A clicked");
                                                } 
                            else if (btn == buttonA) {
                                                        //if(button < pagenumbers) button++;
                                                        //else button = 0;
                                                        PWM_A_DutyCycle++;
                                                        PWM_B_DutyCycle++;
                                                        PWM_C_DutyCycle++;
                                                        set_pwm_DIVERTA();
                                                        set_pwm_DIVERTB();
                                                        set_pwm_DIVERTC();
                                                        
                                                        //DEBUG_PORT.println("B clicked");
                                                      }
                         }

//--------------------------------- END Buttons Clicked
/*
//--------------------------------- Check Button Longpress
void longpress(Button2& btn) {  // Reset Trip Counter
                                unsigned int time = btn.wasPressedFor();
                                if (time > 2000) {
                                                   // do something
                                                 }
}
//--------------------------------- END Check Button Longpress
*/
// Display text/values >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

//=====>  Home Temperature
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
                                                           if(sm_activ_p_neg) draw_warningbox_top();  // Power from Grid
                                                           else  draw_greenbox_top();  // Power to Grid
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawFloat((float)sm_activ_p/1000,2, x, topdraw, 6);
                                                         }
                                          else           { // bottom = 0
                                                           tft.drawString("    Grid   kW   ", 0, bottomtext, 4); 
                                                           if(sm_activ_p_neg) draw_warningbox_bottom();  // Power from Grid
                                                           else  draw_greenbox_bottom();  // Power to Grid
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawFloat((float)sm_activ_p/1000,2, x, bottomdraw, 6);
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
                                                           if(sm_grida_v > max_voltage_warning) draw_warningbox_top();  // Grid Voltage critical
                                                           else  draw_normalbox_top();   
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawNumber(sm_grida_v, x, topdraw, 6);
                                                           
                                                         }
                                          else           { // bottom = 0
                                                           tft.drawString("   Grid A    V   ", 0, bottomtext, 4); 
                                                           if(sm_grida_v > max_voltage_warning) draw_warningbox_bottom();  // Grid Voltage critical
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
                                                           if(sm_gridb_v > max_voltage_warning) draw_warningbox_top();  // Grid Voltage critical
                                                           else  draw_normalbox_top();   
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawNumber(sm_gridb_v, x, topdraw, 6);
                                                           
                                                         }
                                          else           { // bottom = 0
                                                           tft.drawString("   Grid B    V   ", 0, bottomtext, 4); 
                                                           if(sm_gridb_v > max_voltage_warning) draw_warningbox_bottom();  // Grid Voltage critical
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
                                                           if(sm_gridc_v > max_voltage_warning) draw_warningbox_top();  // Grid Voltage critical
                                                           else  draw_normalbox_top();   
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawNumber(sm_gridc_v, x, topdraw, 6);
                                                           
                                                         }
                                          else           { // bottom = 0
                                                           tft.drawString("   Grid C    V   ", 0, bottomtext, 4); 
                                                           if(sm_gridc_v > max_voltage_warning) draw_warningbox_bottom();  // Grid Voltage critical
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
                                                         if(sm_grida_p_neg) draw_warningbox_top();  // Power from Grid
                                                         else  draw_greenbox_top();  // Power to Grid
                                                         tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                         tft.drawFloat((float)sm_grida_p/1000,2, x, topdraw, 6);
                                                       }
                                        else           { // bottom = 0
                                                         tft.drawString("     Grid A   kW  ", 0, bottomtext, 4); 
                                                         if(sm_grida_p_neg) draw_warningbox_bottom();  // Power from Grid
                                                         else  draw_greenbox_bottom();  // Power to Grid
                                                         tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                         tft.drawFloat((float)sm_grida_p/1000,2, x, bottomdraw, 6);
                                                       }   
                      
}

//=====>  Smart Meter Grid Phase B Power
void SM_GridbPower(boolean topbottom){ 
                                                           
                              
                                        tft.setTextColor(TFT_WHITE,TFT_BLUE);
                                        if (topbottom) { // top = 1
                                                         tft.drawString("     Grid B   kW  ", 0, toptext, 4); 
                                                         if(sm_gridb_p_neg) draw_warningbox_top();  // Power from Grid
                                                         else  draw_greenbox_top();  // Power to Grid
                                                         tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                         tft.drawFloat((float)sm_gridb_p/1000,2, x, topdraw, 6);
                                                       }
                                        else           { // bottom = 0
                                                         tft.drawString("     Grid A   kW  ", 0, bottomtext, 4); 
                                                         if(sm_gridb_p_neg) draw_warningbox_bottom();  // Power from Grid
                                                         else  draw_greenbox_bottom();  // Power to Grid
                                                         tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                         tft.drawFloat((float)sm_gridb_p/1000,2, x, bottomdraw, 6);
                                                       }   
                      
}

//=====>  Smart Meter Grid Phase C Power
void SM_GridcPower(boolean topbottom){ 
                                                           
                              
                                        tft.setTextColor(TFT_WHITE,TFT_BLUE);
                                        if (topbottom) { // top = 1
                                                         tft.drawString("     Grid C   kW  ", 0, toptext, 4); 
                                                         if(sm_gridc_p_neg) draw_warningbox_top();  // Power from Grid
                                                         else  draw_greenbox_top();  // Power to Grid
                                                         tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                         tft.drawFloat((float)sm_gridc_p/1000,2, x, topdraw, 6);
                                                       }
                                        else           { // bottom = 0
                                                         tft.drawString("     Grid C   kW  ", 0, bottomtext, 4); 
                                                         if(sm_gridc_p_neg) draw_warningbox_bottom();  // Power from Grid
                                                         else  draw_greenbox_bottom();  // Power to Grid
                                                         tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                         tft.drawFloat((float)sm_gridc_p/1000,2, x, bottomdraw, 6);
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

//=====> Diverter Driver A temperature
void divertA_temperature(boolean topbottom){
                                            
                                             tft.setTextColor(TFT_WHITE,TFT_BLUE);
                                             if (topbottom) { // top = 1
                                                                 tft.drawString("    Div.A    C   ", 0, toptext, 4);
                                                                 if(divertA_temp < divertA_LOW_Warning || divertA_temp > divertA_HIGH_Warning) draw_warningbox_top();
                                                                 else  draw_normalbox_top();
                                                                 tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                                 tft.drawFloat( divertA_temp, 1, x, topdraw, 6); //1 decimal places , x = center
                                                            }
                                             else           { // bottom = 0
                                                                 tft.drawString("    Div.A    C   ", 0, bottomtext, 4);
                                                                 if(divertA_temp < divertA_LOW_Warning || divertA_temp > divertA_HIGH_Warning) draw_warningbox_bottom();
                                                                 else  draw_normalbox_bottom();
                                                                 tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                                 tft.drawFloat( divertA_temp, 1, x, bottomdraw, 6); //1 decimal places , x = center
                                                            }    
}

//=====> Diverter Driver B temperature
void divertB_temperature(boolean topbottom){
                                            
                                             tft.setTextColor(TFT_WHITE,TFT_BLUE);
                                             if (topbottom) { // top = 1
                                                                 tft.drawString("    Div.B    C   ", 0, toptext, 4);
                                                                 if(divertB_temp < divertB_LOW_Warning || divertB_temp > divertB_HIGH_Warning) draw_warningbox_top();
                                                                 else  draw_normalbox_top();
                                                                 tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                                 tft.drawFloat( divertB_temp, 1, x, topdraw, 6); //1 decimal places , x = center
                                                            }
                                             else           { // bottom = 0
                                                                 tft.drawString("    Div.B    C   ", 0, bottomtext, 4);
                                                                 if(divertB_temp < divertB_LOW_Warning || divertB_temp > divertB_HIGH_Warning) draw_warningbox_bottom();
                                                                 else  draw_normalbox_bottom();
                                                                 tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                                 tft.drawFloat( divertB_temp, 1, x, bottomdraw, 6); //1 decimal places , x = center
                                                            }    
}

//=====> Diverter Driver C temperature
void divertC_temperature(boolean topbottom){
                                            
                                             tft.setTextColor(TFT_WHITE,TFT_BLUE);
                                             if (topbottom) { // top = 1
                                                                 tft.drawString("    Div.C    C   ", 0, toptext, 4);
                                                                 if(divertC_temp < divertC_LOW_Warning || divertC_temp > divertC_HIGH_Warning) draw_warningbox_top();
                                                                 else  draw_normalbox_top();
                                                                 tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                                 tft.drawFloat( divertC_temp, 1, x, topdraw, 6); //1 decimal places , x = center
                                                            }
                                             else           { // bottom = 0
                                                                 tft.drawString("    Div.C    C   ", 0, bottomtext, 4);
                                                                 if(divertC_temp < divertC_LOW_Warning || divertC_temp > divertC_HIGH_Warning) draw_warningbox_bottom();
                                                                 else  draw_normalbox_bottom();
                                                                 tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                                 tft.drawFloat( divertC_temp, 1, x, bottomdraw, 6); //1 decimal places , x = center
                                                            }    
}


//=====> Diverter Driver C temperature
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

//=====> Display Diverter A PWM register
void divertA_PWM(boolean topbottom){ 
                                                           
                              
                                          tft.setTextColor(TFT_WHITE,TFT_BLUE);
                                          if (topbottom) { // top = 1
                                                           tft.drawString("   PWM    A   ", 0, toptext, 4); 
                                                           
                                                           draw_normalbox_top();   
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawNumber(PWM_A_DutyCycle, x, topdraw, 6);
                                                           
                                                         }
                                          else           { // bottom = 0
                                                           tft.drawString("   PWM    A   ", 0, bottomtext, 4); 
                                                          
                                                           draw_normalbox_bottom();
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawNumber(PWM_A_DutyCycle, x, bottomdraw, 6);
                                                         }   
           
}

//=====> Display Diverter B PWM register
void divertB_PWM(boolean topbottom){ 
                                                           
                              
                                          tft.setTextColor(TFT_WHITE,TFT_BLUE);
                                          if (topbottom) { // top = 1
                                                           tft.drawString("   PWM    B   ", 0, toptext, 4); 
                                                           draw_normalbox_top();   
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawNumber(PWM_B_DutyCycle, x, topdraw, 6);
                                                           
                                                         }
                                          else           { // bottom = 0
                                                           tft.drawString("   PWM    B   ", 0, bottomtext, 4); 
                                                           draw_normalbox_bottom();
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawNumber(PWM_B_DutyCycle, x, bottomdraw, 6);
                                                         }   
           
}

//=====> Display Diverter B PWM register
void divertC_PWM(boolean topbottom){ 
                                                           
                              
                                          tft.setTextColor(TFT_WHITE,TFT_BLUE);
                                          if (topbottom) { // top = 1
                                                           tft.drawString("   PWM    B   ", 0, toptext, 4); 
                                                           draw_normalbox_top();   
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawNumber(PWM_C_DutyCycle, x, topdraw, 6);
                                                           
                                                         }
                                          else           { // bottom = 0
                                                           tft.drawString("   PWM    B   ", 0, bottomtext, 4); 
                                                           draw_normalbox_bottom();
                                                           tft.setTextPadding( tft.textWidth("-88.8", 6) );
                                                           tft.drawNumber(PWM_C_DutyCycle, x, bottomdraw, 6);
                                                         }   
           
}
  
//--------------------------------- Screen Connected
void connectedscreen() {
                          
                          tft.setTextColor(TFT_BLUE);
                          tft.setCursor(4, 35);
                          tft.setFreeFont(&Orbitron_Light_24);
                          
                          if(WiFi.status() != WL_CONNECTED) tft.println("  No WiFi");
                          /*else if (mbus_error){
                                                 tft.println("  No MB  ");
                                                 tft.setCursor(0, 70);
                                                 tft.println(" WiFi OK!"); 
                                              }
                          */
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
                divertA_PWM(1);
                divertB_PWM(0); // display HotWater temperature on BOTTOM
                          
}
//--------------------------------- END Display Page 1

//--------------------------------- Display Page 2
void page2(){   
                init_background(); // init background image

                divertC_PWM(1);
                //SM_ActivePower(1);    // display Smart Meter Total Active Power
                IN_ActivePower(0);    // display Inverter Total Active Power
               
}
//--------------------------------- END Display Page 2

//--------------------------------- Display Page 3
void page3(){   
                init_background(); // init background image

                SM_GridaPower(1);      // display Smart Meter Grid Phase A Power
                SM_GridaVoltage(0);    // display Smart Meter Grid Phase A Voltage
                
}
//--------------------------------- END Display Page 3

//--------------------------------- Display Page 4
void page4(){   
                init_background(); // init background image

                SM_GridbPower(1);      // display Smart Meter Grid Phase B Power
                SM_GridbVoltage(0);    // display Smart Meter Grid Phase B Voltage
                
}
//--------------------------------- END Display Page 4

//--------------------------------- Display Page 5
void page5(){   
                init_background(); // init background image

                SM_GridcPower(1);      // display Smart Meter Grid Phase C Power
                SM_GridcVoltage(0);    // display Smart Meter Grid Phase C Voltage
                
}
//--------------------------------- END Display Page 5

//--------------------------------- Display Page 6
void page6(){   
                init_background(); // init background image

                divertA_temperature(1);    // display Divert Driver A temperature
                divertB_temperature(0);    // display Divert Driver B temperature
                
}
//--------------------------------- END Display Page 6

//--------------------------------- Display Page 7
void page7(){   
                init_background(); // init background image

                divertC_temperature(1);      // display Divert Driver C temperature
                ambient_temperature(0);      // display Boiler Temperature
                
}
//--------------------------------- END Display Page 7
