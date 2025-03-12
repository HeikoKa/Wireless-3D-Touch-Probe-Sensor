/**  3D Sensor Client
  *   
  *  @author Heiko Kalte  
  *  @date 11.03.2025 
  *  Copyright by Heiko Kalte (h.kalte@gmx.de)
  */
  
  // TODO/Suggestions
  // battery loading could be send to the server to be displayed in the webserver
  
  // Using Arduino ESP32 PICO-D4 (but it is a ESP32 PICO V3-02 device)

char *clientSwVersion = "V02.00";
#include <WiFiUdp.h>
#include <Ticker.h>
#include "Z:\Projekte\Mill\HeikosMill\3D Taster\Arduino\GIT\3D-Touch-Sensor\src\3D-Header.h"  // Arduino IDE does not support relative paths

#ifdef CLIENT_ESP32
    #include <WiFi.h>
#else //ESP8266
    #include <ESP8266WiFi.h>
#endif

#include <Adafruit_NeoPixel.h>    // for RGB LED

using namespace CncSensor;

#ifndef CLIENT_ESP32
  ADC_MODE(ADC_VCC);                                          // measure supply Voltage of the board with analog in, only ESP8266
#endif

WiFiUDP    Udp;
long       rssi;                                              // WLAN signal strength
char       packetBuffer[UDP_PACKET_MAX_SIZE];                 // general buffer to hold UDP messages
int        server_alive_cnt         = 0;                      // current "server is alive counter" value
bool       serviceRequest           = false;                  // interrupt service can request a service intervall by this flag
int        clientHwPcbRevision      = 0;                      // From client PCB version 2 onwards the version can be read out from the PCB coded by 3 hardware inputs
uint32_t   transmitCounter          = 0;                      // counter for identify each sended Wifi message, is incremented with every UDP message

struct statesType{
    bool    touchState               = LOW;                   // current touch sensor state
    bool    chargingBat              = false;                 // global variable to indicate if battery is charging
    bool    wlanComplete             = false;                 // global variable to indicate if wlan connection is established completely
    bool    wlanCompleteBlink        = true;                  // blink wlan LED if enlighted
    bool    batteryError             = false;                 // error with the battery
    bool    rssiError                = false;                 // error with the WLAN rssi
    bool    transmissionError        = false;                 // error with the touch state transmission
    bool    aliveCounterError        = false;                 // error with server alive counter
}; 

statesType states; 

#ifdef CLIENT_ESP32
  hw_timer_t *serviceTimer = NULL;
  portMUX_TYPE timerMux    = portMUX_INITIALIZER_UNLOCKED;
#endif

#ifdef CYCLETIME
  int32_t bTimeLow,  eTimeLow;
  int32_t bTimeHigh, eTimeHigh;

  static inline int32_t asm_ccount(void){              // asm-helpers taken from https://sub.nanona.fi/esp8266/timing-and-ticks.html, reading the CCOUT register with clock ticks
    int32_t r;
    asm volatile ("rsr %0, ccount" : "=r"(r));
    return r;
  }
#endif

Adafruit_NeoPixel pixels(1, CLIENT_RGB_LED_OUT, NEO_GRB + NEO_KHZ800);

static inline bool sendWifiMessage(String msg, bool blocking){  
  transmitCounter++;                                            // increment identifier each time sendWifiMessage() is called
  #ifdef CYCLETIME
    bTimeHigh = asm_ccount();                                   // take begin time for client to server to client cycle time measurement
  #endif
  msg = msg + "_" + String(transmitCounter);                    // add transmitCounter as identifier to each message
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());            // send UDP packet to server to indicate the 3D touch change 
  #ifdef CLIENT_ESP32
    Udp.printf(msg.c_str());
  #else //ESP8266
    Udp.write(msg.c_str());
  #endif
  Udp.endPacket();
  #if defined(DEBUG) && defined(DEBUG_SHOW_ALL_TRANSMISSIONS)   // show all transmissions content only if DEBUG and DEBUG_SHOW_ALL_TRANSMISSIONS
    Serial.printf("sendWifiMessage() Sending Wifi message: ");
    Serial.println(msg.c_str());
  #endif
  // finished first sending, waiting for reply if blocking is requested
  if(blocking){
    int counter = 0;   // counter for timeout 
    for (int retry = 0; retry < TRANSMISSION_RETRY_MAX; retry++){
      counter = 0;   // set counter for timeout to zero everytime a new retry is started
      while(counter < SERVER_AQUN_CNT_MAX){ // repeate until server sends acknowledge message or the maximum counter loops are reached (timeout)
        counter++;
        if (Udp.parsePacket()){  // received UDP package
          int len = Udp.read(packetBuffer, UDP_PACKET_MAX_SIZE);
          if (len > 0)
          packetBuffer[len] = '\0';
          if(!strcmp(packetBuffer, msg.c_str())){    // check if exactly the same message comes back from server
            #ifdef CYCLETIME     
              eTimeHigh = asm_ccount();      //take end time for client to server to client cycle time measurement
              #ifdef DEBUG
                Serial.printf("sendWifiMessage() Measured Wifi cycle time for sensor HIGH activation: %u ticks\n", ((uint32_t)(eTimeHigh - bTimeHigh)));
              #endif
              String cycle_msg = String(CLIENT_CYCLE_MSG) + "_" + String((uint32_t)(eTimeHigh - bTimeHigh)); // pack end-time minus begin-time into a message
              sendWifiMessage(cycle_msg.c_str(), false);
            #endif
            #ifdef DEBUG
              Serial.printf("sendWifiMessage() Detected acknowledge message from server (retries: %d, counter value: %d, max counter: %d) message is ", retry, counter, SERVER_AQUN_CNT_MAX);
              Serial.println(packetBuffer);
            #endif
            return true;
          } //if(!strcmp(packetBuffer, msg.c_str()))
        } //if(Udp.parsePacket())
      } //end while
      #ifdef DEBUG
        Serial.printf("sendWifiMessage() No acknowledge message from server %d retry of maximum %d retrys, ", retry, TRANSMISSION_RETRY_MAX);
      #endif
    }  //end for 
    return false;  // number of retry have not been successfull
  } //if(blocking)
  return true;  // there is no false, if blocking is not enabled
} //sendWifiMessage()


 static inline void doSensorHigh(void){
  #if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("doSensorHigh() running on core ");
    Serial.println(xPortGetCoreID());
  #endif
  states.touchState = HIGH;                            // remember the current state
  controlLed(NONE);
  String high_msg;
  #ifdef DEBUG
    Serial.println("doSensorHigh() Sending HIGH message");
  #endif
  high_msg = String(CLIENT_TOUCH_HIGH_MSG) + "_" + String(states.transmissionError); 	// put the current transmission error state into the UDP message
  if (!sendWifiMessage(high_msg, true))							// call blocking upd message
    states.transmissionError = true;
} // end void doSensorHigh(void)


static inline void doSensorLow(void){
  #if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("doSensorLow() running on core ");
    Serial.println(xPortGetCoreID());
  #endif
  states.touchState = LOW;                      // remember the current state
  controlLed(NONE);   
  String low_msg;
  #ifdef DEBUG
    Serial.println("doSensorLow() Sending LOW message");
  #endif
  low_msg = String(CLIENT_TOUCH_LOW_MSG) + "_" + String(states.transmissionError); // put the current touch error state into the UDP message
  if (!sendWifiMessage(low_msg, true))							// call blocking upd message
		states.transmissionError = true;
} //end void doSensorLow(void)


static inline void checkAliveCounter(void){
  #if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("checkAliveCounter() running on core ");
    Serial.println(xPortGetCoreID());
  #endif
  if (server_alive_cnt <= SERVER_ALIVE_CNT_MAX){
    #ifdef DEBUG
      Serial.printf("checkAliveCounter() Server alive counter current: %d, max reconnect: %d, max server dead: %d\n", server_alive_cnt, SERVER_ALIVE_CNT_MAX, SERVER_ALIVE_CNT_DEAD);
    #endif
    server_alive_cnt++;                                          // increase "the server has not answered" alive counter
    states.aliveCounterError = false;                            // no error, no red LED
  }else{
    if (server_alive_cnt <= SERVER_ALIVE_CNT_DEAD){
      // server alive messages are missing, but try to reconnect
      #ifdef DEBUG
        Serial.printf("checkAliveCounter() No server alive Wifi message during %d service intervals.\n", server_alive_cnt);
      #endif
      server_alive_cnt++;                                        // keep incrementing server alive counter during reconnect tries
      states.wlanComplete      = false;                          // server is (temporary?) dead the connection must be re-establisched
      states.aliveCounterError = true;                           // set internal state to indicate error
    }else{
      // server seems to be dead, client goes to sleep
      #ifdef DEBUG
        Serial.printf("checkAliveCounter() No server alive Wifi message during %d service intervals. Going to sleep\n", server_alive_cnt);
      #endif
      server_alive_cnt         = 0;
      states.wlanComplete      = false;
      states.aliveCounterError = true;                           // set internal state to indicate error
      goSleep();                                                 // switch off external power
    } // end if SERVER_ALIVE_CNT_DEAD 
  } // end if SERVER_ALIVE_CNT_MAX
} // end checkAliveCounter()


static inline void checkBatteryVoltage(void){
  #if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("checkBatteryVoltage() running on core ");
    Serial.println(xPortGetCoreID());
  #endif
  String cycle_msg;
  //check if battery is charging
  if(digitalRead(CLIENT_CHARGE_IN) == LOW){
    states.chargingBat = true;
    #ifdef DEBUG
        Serial.printf("checkBatteryVoltage() Battery is charging\n");
    #endif
  }else{
    states.chargingBat = false;                                //battery is not charging
  }
  // measure battery voltage
  #ifdef CLIENT_ESP32
    int analogVolts =analogRead(CLIENT_ANALOG_CHANNEL);
    float batVoltage = 6.6*analogVolts/4096 + BAT_CORRECTION;                          // Battery Voltage is divided by 2 by resistors on PCB
  #else
    int analogVolts = ESP.getVcc();
    float batVoltage = analogVolts / 1000.0 + BAT_CORRECTION;
  #endif
  
  if (batVoltage < BAT_LOW_VOLT){
    if (batVoltage < BAT_CRIT_VOLT){                                              // check if battery voltage is critical
      cycle_msg = String(CLIENT_BAT_CRITICAL_MSG) + "_" + String(batVoltage);     // pack message and bat voltage in the UDP frame
      #ifdef DEBUG
        Serial.printf("checkBatteryVoltage() Voltage is critical, current value is ");
        Serial.println(batVoltage);
      #endif
    }else{ //if(batVoltage < BAT_CRIT_VOLT)                                       // battery is only low
      cycle_msg = String(CLIENT_BAT_LOW_MSG) + "_" + String(batVoltage);          // pack message and bat voltage in the UDP frame
      #ifdef DEBUG
        Serial.printf("checkBatteryVoltage() Voltage is low, current value is ");
        Serial.println(batVoltage);
      #endif
    }  // if(batVoltage < BAT_CRIT_VOLT)
    states.batteryError = true;                           // indicate low or critical battery by LED
  }else{ // if(batVoltage < BAT_LOW_VOLT)                                         // battery is ok
    cycle_msg = String(CLIENT_BAT_OK_MSG) + "_" + String(batVoltage);   // pack message and bat voltage in the UDP frame
    #ifdef DEBUG
      Serial.printf("checkBatteryVoltage() Bat Voltage is ok, measured value is %.2fV (low bat is %.2fV and critical bat is %.2fV)\n", batVoltage, BAT_LOW_VOLT, BAT_CRIT_VOLT);
    #endif
    states.batteryError = false;
  } // if(batVoltage < BAT_LOW_VOLT)
  sendWifiMessage(cycle_msg.c_str(), false);
} // end checkBatteryVoltage(void)


String collectClientData(void){
  String msg = CLIENT_INFO_MSG + String("_");
  #ifdef CLIENT_ESP32
   msg += String("ESP32_");
  #else
   msg += String("ESP8266_");
  #endif
  #ifdef DEBUG
   msg += String("DEBUG On_");
  #else
   msg += String("DEBUG Off_");
  #endif
  #ifdef CYCLETIME
   msg += String("CYCLETIME On_");
  #else
   msg += String("CYCLETIME Off_");
  #endif
  msg += clientSwVersion;
  msg += String("_");
  msg += clientHwPcbRevision;
  return msg;
} // collectClientData(void)


static inline void checkWlanStatus(void) {
  #if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("checkWlanStatus() running on core ");
    Serial.println(xPortGetCoreID());
  #endif
  if (WiFi.status() != WL_CONNECTED){
    #ifdef DEBUG
      Serial.printf("checkWlanStatus() WLAN is not connected anymore\n");
    #endif
    states.wlanComplete = false;
  } //if (states.wlanComplete == true)
  rssi = WiFi.RSSI();
  if (rssi < WIFI_RSSI_REPORT_LEVEL){
    #ifdef DEBUG
      Serial.print("checkWlanStatus() ERROR: WLAN signal strength is critical, RSSI:");
      Serial.println(rssi);
    #endif
    states.rssiError = true;
  }else{
    #ifdef DEBUG
      Serial.print("checkWlanStatus() WLAN signal strength is ok, RSSI:");
      Serial.println(rssi);
    #endif
    states.rssiError = false;
  } // end if(rssi < WIFI_RSSI_REPORT_LEVEL)
  String cycle_msg = String(CLIENT_RSSI_MSG) + "_" + String(rssi);   // pack message and rssi in the UDP frame
  sendWifiMessage(cycle_msg, false);
} //end void checkWlanStatus()


static inline void sendAliveMsg(void) {
  #if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("sendAliveMsg() running on core ");
    Serial.println(xPortGetCoreID());
  #endif
  #ifdef DEBUG
    Serial.printf("sendAliveMsg() Sending Wifi alive message to server\n");
  #endif
	sendWifiMessage(CLIENT_ALIVE_MSG, false);
} // end void sendAliveMsg(void)


static inline void doService(void){
  // Does all service activities that are called regularly by timer interrupt
  #if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("doService() running on core ");
    Serial.println(xPortGetCoreID());
  #endif
  #ifdef DEBUG
    Serial.printf("\ndoService() ***** start service *****\n");
  #endif
  checkBatteryVoltage();                                  // check battery in service
  checkAliveCounter();                                    // check server alive counter in service
  sendAliveMsg();                                         // send client alive in service
  checkWlanStatus();                                      // check WLAN status and signal strength
  states.transmissionError = false;                       // delete transmission error during service
  controlLed(NONE);                                       // set LED immediately
  #ifdef CLIENT_ESP32
    portENTER_CRITICAL_ISR(&timerMux);                    // protect access to serviceRequest 
  #endif
  serviceRequest = false;                                 // reset service flag after service
  #ifdef CLIENT_ESP32
    portEXIT_CRITICAL_ISR(&timerMux);                     // release protection of data
  #endif
  #ifdef DEBUG
    Serial.printf("doService() ***** end service *****\n\n");
  #endif
} // end void doService(void)


// ISR attribute for ESP32 "IRAM_ATTR"
void ICACHE_RAM_ATTR serviceIsr(void){                    // timer interrupt service routine
  #if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("serviceIsr() running on core ");
    Serial.println(xPortGetCoreID());
  #endif
  #ifdef CLIENT_ESP32
    portENTER_CRITICAL_ISR(&timerMux);                    // protect access to serviceRequest 
  #endif
    serviceRequest = true;
  #ifdef CLIENT_ESP32
    portEXIT_CRITICAL_ISR(&timerMux);                     // release protection of data
  #endif
} // end serviceIsr(void)


void wlanInit(void){
  #if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("wlanInit() running on core ");
    Serial.println(xPortGetCoreID());
  #endif
  WiFi.config(clientIpAddr, gateway, subnet);             // set manual IP address if in Soft Access Point (SAP) mode
  #ifdef DEBUG
    Serial.println("\n");
    Serial.printf("wlanInit() Connecting to %s ", SSID);
  #endif
  states.wlanCompleteBlink = true;                        // switch on blinking
  WiFi.begin(SSID, password);                             // connect to WLAN 
  while (WiFi.status() != WL_CONNECTED){                  // stay in this loop until a WLAN connection could be established
    controlLed(NONE);                                     // set LED immediately
    if (!states.wlanCompleteBlink)                        // if blinking is enabled, do not add an additional delay, because delay is added inside controlLed()
      delay(WLAN_INIT_PAUSE);                             // wait for a moment before checking connection again
    yield();
    #ifdef DEBUG
      Serial.print(".");
    #endif
    if (serviceRequest == true){                          // do service routine if isr has set the service flag
      checkAliveCounter();                                // check Alive Counter to go to sleep after a while if no server is available
      serviceRequest = false;                             // reset service flag after small alive counter service
    }
  }//end while WiFi.status

  Udp.begin(clientUdpPort);                               // now listening for a server to send UDP messages
  #ifdef DEBUG
    Serial.println(" connected to WLAN network");
  #endif
  
  while(!states.wlanComplete){
    #ifdef DEBUG
      Serial.printf("wlanInit() Sending hello message to IP %s, UDP port %d\n", serverIpAddr.toString().c_str(), serverUdpPort);
    #endif
    sendWifiMessage(CLIENT_HELLO_MSG, false);
    int packetSize = Udp.parsePacket();
    if (packetSize){                                      // listening for reply udp message
      if (!strcmp(Udp.remoteIP().toString().c_str(), serverIpAddr.toString().c_str())){ //check server address
        #ifdef DEBUG
          Serial.printf("wlanInit() UDP packet received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
        #endif
        int len = Udp.read(packetBuffer, UDP_PACKET_MAX_SIZE);
        if (len > 0)
          packetBuffer[len] = '\0';
        if(!strcmp(packetBuffer, SERVER_HELLO_MSG)){
          #ifdef DEBUG
            Serial.println("WlanInit() Detected server hello message, send a reply");
          #endif
		      sendWifiMessage(CLIENT_REPLY_MSG, false);
        } // end if(!strcmp(packetBuffer, SERVER_HELLO_MSG))
        states.wlanComplete = true;                             // WLAN connection is complete
        controlLed(NONE); // set LED immediately
        #ifdef DEBUG
          Serial.print("wlanInit() WLAN is complete\n");
        #endif 
      }else{   // unexpected UDP sender                                          // server address is unexpected
        #ifdef DEBUG
          Serial.printf("wlanInit() Received UDP packet from unexpected IP: %s, port %d\n",  Udp.remoteIP().toString().c_str(), Udp.remotePort());
        #endif
        states.wlanComplete = false;                            // wrong WLAN server address
      } // end if(!strcmp(Udp.remoteIP().toString().c_str()
    }else{ // else of if (packetSize)                           // no UDP packet received yet
      #ifdef DEBUG
        Serial.printf("wlanInit() No respond from server yet\n");
      #endif
      
      //digitalWrite(CLIENT_WLAN_LED, HIGH);                       // toggle WLAN LED quickly to indicate connection try
      //delay(FAST_BLINK);
      //digitalWrite(CLIENT_WLAN_LED, LOW);                        // toggle WLAN LED quickly to indicate connection try
      //delay(FAST_BLINK);
      //yield();
      controlLed(NONE); // set LED immediately
    } //end if(packetSize)
    if (serviceRequest == true)                                   //do service routine if isr has set the service flag
      doService();
  } //end while(!states.wlanComplete)
  
  //Send collected infos about client to server, e.g. to be displayed in Webserver
  String cycle_msg = collectClientData();
  #ifdef DEBUG
    Serial.printf("wlanInit() Sending Wifi message with client infos: %s\n", cycle_msg.c_str());
  #endif
	sendWifiMessage(cycle_msg.c_str(), false);
}//end void WlanInit()


void controlLed(fadingType fading){
  #if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("controlLed() running on core ");
    Serial.println(xPortGetCoreID());
  #endif
  if (fading == NONE){ // special case LED fading at power up and power down /sleep
    if(states.touchState){  // current touch state has LED Priority over all other LEDs
      pixels.setPixelColor(0, pixels.Color(TOUCH_COLOR)); 
    }else{
      if (!states.wlanComplete){
        pixels.setPixelColor(0, pixels.Color(WLAN_COLOR)); 
        if (states.wlanCompleteBlink){
          pixels.show();
          delay(SLOW_BLINK);
          pixels.setPixelColor(0, pixels.Color(0, 0, 0)); // switch off
          pixels.show();   // Send the updated pixel colors to the hardware.
          delay(SLOW_BLINK);
        } //if(states.wlanCompleteBlink)
      }else{  //if(!states.wlanComplete)
        if (states.rssiError){
          pixels.setPixelColor(0, pixels.Color(RSSI_ERROR_COLOR));
        }else{
          if (states.transmissionError){
            pixels.setPixelColor(0, pixels.Color(WIFI_ERROR_COLOR));
          }else{
            if (states.aliveCounterError){
              pixels.setPixelColor(0, pixels.Color(ALIVE_ERRROR_COLOR));
            }else{
              if (states.chargingBat){
                pixels.setPixelColor(0, pixels.Color(CHARGE_COLOR));
              }else{
                if (states.batteryError){
                  pixels.setPixelColor(0, pixels.Color(BATTERY_ERROR_COLOR));
                }else{ //if none of the above applys, make LED white to indicate, that the sensor is on power
                  pixels.setPixelColor(0, pixels.Color(POWER_COLOR));
                } //if(states.batteryError)
              } //if(states.chargingBat)
            } //if(states.aliveCounterError)
          } //if(states.transmissionError)
        } //if(states.rssiError)
      } //if(states.wlanComplete)
    } //if states.touchState
  }else{ //if(!fading)
    if (fading == UP){
      if(CLIENT_ALLOW_LED_FADING){
        pixels.setPixelColor(0, pixels.Color(POWER_COLOR));                               // Power color during fading
        for(int i = 0; i <= CLIENT_RGB_BRIGHTNESS; i = i + 1){
          pixels.setPixelColor(0, pixels.Color(i,i,i));
          pixels.show();
          delay(CLIENT_RGB_FADE_SPEED);
        } //end for
      } //if(CLIENT_ALLOW_LED_FADING){
    }else{  //if (fading=UP)
      if(CLIENT_ALLOW_LED_FADING){
        pixels.setPixelColor(0, pixels.Color(POWER_COLOR));                               // Power color during fading
        for(int i = CLIENT_RGB_BRIGHTNESS; i >= 0; i = i - 1){
          pixels.setPixelColor(0, pixels.Color(i,i,i));
          pixels.show();
          delay(CLIENT_RGB_FADE_SPEED);
        } //end for
      } //if(CLIENT_ALLOW_LED_FADING)
    } // if (fading==UP)
  } //if(!fading)
  pixels.show();                                                                      // Send the updated pixel colors to the hardware.
} //end controlLed


static inline void initIo(void) {
  #ifdef CLIENT_ESP32
    analogReadResolution(12);   //set the resolution to 12 bits (0-4095) for ESP32 only
  #endif
  pixels.begin();                                                 // initialize NeoPixel RBG LED
  pinMode(CLIENT_SLEEP_OUT,    OUTPUT);                           // control external sleep/power hardware
  pinMode(CLIENT_TOUCH_IN,     INPUT_PULLUP);                     // set DIO to input with pullup
  pinMode(CLIENT_CHARGE_IN,    INPUT_PULLUP);                     // set DIO to input with pullup
  #ifdef CLIENT_HW_REVISION_2_0
    pinMode(CLIENT_HW_REVISION_0, INPUT);                         // client hardware PCB revision bit 0 (only for PCB revision 2.0 or later)
    pinMode(CLIENT_HW_REVISION_1, INPUT);                         // client hardware PCB revision bit 1 (only for PCB revision 2.0 or later)
    pinMode(CLIENT_HW_REVISION_2, INPUT);                         // client hardware PCB revision bit 2 (only for PCB revision 2.0 or later)
    clientHwPcbRevision = (digitalRead(CLIENT_HW_REVISION_2) << 2) + (digitalRead(CLIENT_HW_REVISION_1)<< 1) + digitalRead(CLIENT_HW_REVISION_0); //construct version number from 3 input bit
  #endif
  digitalWrite(CLIENT_SLEEP_OUT,  HIGH);                          // switch on output to prevent external sleep hardware to send cpu and board to sleep
}

void showStateColors(void){
  //show all RGB colors
  pixels.setPixelColor(0, pixels.Color(CHARGE_COLOR));
  pixels.show();
  delay(SLOW_BLINK);
  pixels.setPixelColor(0, pixels.Color(TOUCH_COLOR));
  pixels.show();
  delay(SLOW_BLINK);
  pixels.setPixelColor(0, pixels.Color(WLAN_COLOR));
  pixels.show();
  delay(SLOW_BLINK);
  pixels.setPixelColor(0, pixels.Color(RSSI_ERROR_COLOR));
  pixels.show();
  delay(SLOW_BLINK);
  pixels.setPixelColor(0, pixels.Color(BATTERY_ERROR_COLOR));
  pixels.show();
  delay(SLOW_BLINK);
  pixels.setPixelColor(0, pixels.Color(WIFI_ERROR_COLOR));
  pixels.show();
  delay(SLOW_BLINK);
  pixels.setPixelColor(0, pixels.Color(ALIVE_ERRROR_COLOR));
  pixels.show();
  delay(SLOW_BLINK);
}

void goAlive(void){
  #ifdef DEBUG
    Serial.println("goAlive() Going alive");
  #endif
  states.wlanCompleteBlink = false;                             // stop blinking when going to sleep
  controlLed(UP); 
  #ifdef SHOW_STATE_COLORS
    showStateColors();
  #endif
}  //goAlive()


void goSleep(){  
  if (!((NO_SLEEP_WHILE_CHARGING) && (digitalRead(CLIENT_CHARGE_IN) == LOW))){   // prevent goint to sleep if battery is charging and the NO_SLEEP_WHILE_CHARGING flag is set
    #ifdef DEBUG
      Serial.println("goSleep() Going to sleep");
    #endif
    // LED shutdown
    states.wlanCompleteBlink = false;                                         // stop blinking when going to sleep
    controlLed(DOWN);                                                    // decrease LED brightness during shutdown and indicate fading is true
    delay(1000);
    digitalWrite(CLIENT_SLEEP_OUT, LOW);                                        // switch off external power
  }else{  // if(!((NO_SLEEP_WHILE_CHARGING)
  #ifdef DEBUG
    Serial.println("goSleep() Not going to sleep while charging");
  #endif
  } // if(!((NO_SLEEP_WHILE_CHARGING)
} //end void goSleep()


void setup(void) {
  #ifdef DEBUG
    Serial.begin(BAUD_RATE);                                                    // Setup Serial Interface with baud rate
    Serial.println("setup() I am the 3D Touch Probe Sensor Client");
    Serial.println("setup() Copyright by Heiko Kalte 2025 (h.kalte@gmx.de)");
  #endif
  initIo();
  goAlive();
  //Setup timer interrup for service routines
  #ifdef CLIENT_ESP32
    serviceTimer = timerBegin(1000000);                                         // Set timer frequency to 1MHz
    timerAttachInterrupt(serviceTimer, &serviceIsr);                            // Attach onTimer function to our timer. 
    timerAlarm(serviceTimer, SERVICE_INTERVALL_ESP32, true, 0);                 // Set alarm to call onTimer function every second (value in microseconds). Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
    timerStart(serviceTimer);
  #else //ESP8266
    timer1_attachInterrupt(serviceIsr);
    timer1_enable(TIM_DIV256, TIM_EDGE, TIM_LOOP);                              // DIV256 means one tick is 3.2us, total intervall time is 3.2us x SERVICE_INTERVALL, max is ~27 sec
    timer1_write(SERVICE_INTERVALL);                                            // has to start early, to allow the client to go to sleep, if the server is dead
  #endif
  wlanInit();                                                                   // Init wlan communication with server
} //end setup()


void loop(void){  
  //reconnect to server if connection got lost
  if (states.wlanComplete == false){
    #ifdef DEBUG
      Serial.printf("loop() Lost connection to server, need to re-init wlan connection\n");
    #endif
    wlanInit();
  }

  // for more responsiveness check for battery charging status changes within loop()
  if (states.chargingBat != (digitalRead(CLIENT_CHARGE_IN) == LOW)){
    #ifdef DEBUG
      Serial.printf("loop() charging state changed, calling checkBatteryVoltage()\n");
      Serial.println(states.chargingBat);
      Serial.println(digitalRead(CLIENT_CHARGE_IN)==LOW);
    #endif
    checkBatteryVoltage();
    controlLed(NONE); // set LED immediately
  }
   
  // do pin polling instead of interrupt, check for state changes high->low or low->high
  if ((digitalRead(CLIENT_TOUCH_IN) == (HIGH != CLIENT_TOUCH_POLARITY)) && (states.touchState == LOW)){  // if current internal state LOW and Pin high
    doSensorHigh();
    //delayMicroseconds(TOUCH_PIN_DEBOUNCE);                  // pauses for debouncing the touch input pin
  }
  
  if ((digitalRead(CLIENT_TOUCH_IN) == (LOW != CLIENT_TOUCH_POLARITY)) && (states.touchState == HIGH)){
    doSensorLow();
    //delayMicroseconds(TOUCH_PIN_DEBOUNCE);                  // pauses for debouncing the touch input pin
  }

  // check for UDP commands from server
  int packetSize = Udp.parsePacket();
  if (packetSize){  // received UDP package
    int len = Udp.read(packetBuffer, UDP_PACKET_MAX_SIZE);
    if (len > 0)
      packetBuffer[len] = '\0';

    // receiving sleep message from server
    if (!strcmp(packetBuffer, SERVER_SLEEP_MSG)){
      #ifdef DEBUG
        Serial.println("loop() Detected sleep command\n");
      #endif
      goSleep();                                            // go to sleep
      return;
    }

    // receiving alive message from server and resetting the alive counter
    if(!strcmp(packetBuffer, SERVER_ALIVE_MSG)){
      #ifdef DEBUG
        Serial.println("loop() Detected server alive command");
      #endif
      server_alive_cnt = 0;                               //reset alive counter for server
      return;
    }

    // receiving hello message from server and sending reply
    if(!strcmp(packetBuffer, SERVER_HELLO_MSG)){
      #ifdef DEBUG
        Serial.println("loop() Detected server hello command, send a reply");
      #endif
	    sendWifiMessage(CLIENT_REPLY_MSG, false);
      return;
    }
    
    // receiving reply message from server and do nothing
    if(!strcmp(packetBuffer, SERVER_REPLY_MSG)){
      #ifdef DEBUG
        Serial.println("loop() Detected server reply message.");
      #endif
      return;
    }

    //receiving unknown UDP message
    #ifdef DEBUG
      Serial.printf("loop() Received unknown command by UDP: ");
      Serial.println(packetBuffer);
    #endif
  } // end if(packetSize)

  if (serviceRequest == true)        //do service routine if isr has set the service flag
      doService();

} //end loop()