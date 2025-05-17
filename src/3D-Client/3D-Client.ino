/**  3D Sensor Client
  *   
  *  @author Heiko Kalte  
  *  @date 16.05.2025 
  *  Copyright by Heiko Kalte (h.kalte@gmx.de)
  */
  
  // TODO/Suggestions
  
  // *****************************************************************************************************
  // Use Arduino ESP32 PICO-D4 (but it is a ESP32 PICO V3-02 device) do not accidently use server settings
  // Latest Tests under esp32 V3.1.3 (3.2 seems not to work!!!)
  // *****************************************************************************************************

char clientSwVersion[] = "3.00.04";
#include <WiFiUdp.h>
#include <Ticker.h>
//#include "Z:\Projekte\Mill\HeikosMill\3D Taster\Arduino\GIT\3D-Touch-Sensor\src\3D-Header.h"
#include "C:\Users\stell\Desktop\src\3D-Header.h"  // Arduino IDE does not support relative paths
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>    // for RGB LED
#include "esp_chip_info.h"        // for detail print outs about ESP chip, see also https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/Esp.cpp

using namespace CncSensor;

WiFiUDP    Udp;
char       packetBuffer[UDP_PACKET_MAX_SIZE];                 // general buffer to hold UDP messages
int        serverAliveCnt           = 0;                      // current "server is alive counter" value
bool       serviceRequest           = false;                  // interrupt service can request a service intervall by this flag
uint32_t   transmitCounter          = 0;                      // counter for identify each sended Wifi message, is incremented with every UDP message
uint32_t   autoSleepTimer           = AUTO_SLEEP_TIMER_CYCLES;// init automatic sleep timer
bool       touchAction              = false;                  // flag that indicates if any touch action (high/low) has taken place, relevant for the auto sleep timer/watchdog
uint8_t    arrayIndexBat            = 0;                      // global array index for battery voltage
uint8_t    arrayIndexRssi           = 0;                      // global array index for rssi
uint8_t    arrayIndexCharging       = 0;                      // global array index for battery charging state

struct statesType{
    bool    touchState               = LOW;                   // current touch sensor state
    bool    chargingVoltageApplied   = false;                 // variable to indicate battery charging voltage is applied
    bool    chargingBat              = false;                 // variable to indicate if battery is charging (charging voltage can be applied, but bat is not charging, because it is full)
    bool    wlanComplete             = false;                 // variable to indicate if wlan connection is established completely
    bool    wlanCompleteBlink        = true;                  // blink wlan LED if enlighted
    bool    batteryError             = false;                 // error with the battery
    bool    rssiError                = false;                 // error with the WLAN rssi
    bool    transmissionError        = false;                 // error with the touch state transmission
    bool    aliveCounterError        = false;                 // error with server alive counter
    uint8_t mayorHwVersion           = 0;                     // mayor hardware PCB version
    uint8_t mayorSwVersion           = 0;                     // mayor software version
    uint8_t minorSwVersion           = 0;                     // minor software version
    uint8_t maintSwVersion           = 0;                     // maintenance software version
}; 

float batVoltArray[] = {   3.7,   3.7,   3.7};                // array of battery voltages to calculate a mean voltage of the last 3 measurements
long  rssiArray[]    = {   -60,   -60,   -60};                // array of wlan rssi signal strength values to calculate a mean value of the last 3 measurements
bool  chargeArray[]  = { false, false, false};                // array of battery charging states to prevent blinking at the end of the charge process

statesType states; 
hw_timer_t *serviceTimer = NULL;
portMUX_TYPE timerMux    = portMUX_INITIALIZER_UNLOCKED;

#ifdef CYCLETIME
  int32_t beginTime, endTime;
#endif

Adafruit_NeoPixel pixels(1, CLIENT_RGB_LED_OUT, NEO_GRB + NEO_KHZ800);       // RBG LED

static inline void checkRssi(void){
  // check WLAN signal stength (average of the last 3 values)
  rssiArray[arrayIndexRssi] = WiFi.RSSI();                      // write current rssi into array
  arrayIndexRssi = arrayIndexRssi < 2 ? ++arrayIndexRssi:0;     // increment index and reset to 0 if bigger than 2
  #if defined(DEBUG) && defined(VERBOSE)
    Serial.printf("checkRssi() Last three RSSI values were: %d, %d and %d\n", rssiArray[0], rssiArray[1], rssiArray[2]);
  #endif
  int sum = 0;
  for(int i=0; i<3; i++){
    sum = sum + rssiArray[i];
  } //end for
  long rssi = sum/3; // average of the last three
  if (rssi < WIFI_RSSI_REPORT_LEVEL){
    #ifdef DEBUG
      Serial.print("checkRssi() Warning: WLAN signal strength is low, average RSSI:");
      Serial.println(rssi);
    #endif
    states.rssiError = true;
  }else{
    #ifdef DEBUG
      Serial.print("checkRssi() WLAN signal strength is ok, average RSSI:");
      Serial.println(rssi);
    #endif
    states.rssiError = false;
  } // end if
  String cycle_msg = String(CLIENT_RSSI_MSG) + "_" + String(rssi);   // pack message and rssi in the UDP frame
  sendWifiMessage(cycle_msg, false);
} //checkRssi()

void printChipInfo(void){
  // serial print out details about the ESP chip
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  
  #ifdef DEBUG
    Serial.printf("printChipInfo() Chip Model: %d (see esp_chip_info.h for details)\n", chip_info.model);   //#### macht core dump
    Serial.printf("printChipInfo() Cores: %d\n", chip_info.cores);
    Serial.printf("printChipInfo() Revision number: %d\n", chip_info.revision);
  #endif
}

static inline bool sendWifiMessage(String msg, bool blocking){ 
  #if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("sendWifiMessage() running on core ");
    Serial.println(xPortGetCoreID());
  #endif 
  transmitCounter++;                                            // increment udp message identifier each time sendWifiMessage() is called
  #ifdef CYCLETIME
    beginTime = micros();                                       // take begin time for client to server to client cycle time measurement
  #endif
  msg = msg + "_" + String(transmitCounter);                    // add transmitCounter as identifier to each message
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());            // send UDP packet to server to indicate the 3D touch change 
  Udp.printf(msg.c_str());
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
          if(!strcmp(packetBuffer, msg.c_str())){     // check if exactly the same message comes back from server
            #ifdef CYCLETIME     
              endTime = micros();                     // take end time for client to server to client cycle time measurement
              #ifdef DEBUG
                Serial.printf("sendWifiMessage() Measured Wifi cycle time for sensor HIGH activation: %u ticks\n", ((uint32_t)(endTime - beginTime)));
              #endif
              String cycle_msg = String(CLIENT_CYCLE_MSG) + "_" + String((uint32_t)(endTime - beginTime)); // pack end-time minus begin-time into a message
              sendWifiMessage(cycle_msg.c_str(), false);
            #endif
            #ifdef DEBUG
              Serial.printf("sendWifiMessage() Detected acknowledge message from server (retries: %d, delay counter: %d, max counter: %d) message is ", retry, counter, SERVER_AQUN_CNT_MAX);
              Serial.println(packetBuffer);
            #endif
            return true;
          } //if(!strcmp(packetBuffer, msg.c_str()))
        } //if(Udp.parsePacket())
      } //end while
      #ifdef DEBUG
        Serial.printf("sendWifiMessage() No acknowledge message from server %d retry of maximum %d retrys\n", retry, TRANSMISSION_RETRY_MAX);
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
  touchAction = true;                                   // indicates that any kind of action has taken place (for auto sleep timer)
  states.touchState = HIGH;                             // remember the current state
  controlLed(NONE);
  String high_msg;
  #ifdef DEBUG
    Serial.println("doSensorHigh() Sending HIGH message");
  #endif
  high_msg = String(CLIENT_TOUCH_HIGH_MSG) + "_" + String(states.transmissionError); 	// put the current transmission error state into the UDP message
  if (!sendWifiMessage(high_msg, true)){						// call blocking upd message
    states.transmissionError = true;
    #ifdef DEBUG
      Serial.println("doSensorHigh() Transmission Error while sending High message");
    #endif
  }
} // end void doSensorHigh(void)


static inline void doSensorLow(void){
  // handle a low signal from the sensor input
  #if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("doSensorLow() running on core ");
    Serial.println(xPortGetCoreID());
  #endif
  touchAction = true;                                                               // indicates that any kind of action has taken place (for auto sleep timer)
  states.touchState = LOW;                                                          // remember the current state
  controlLed(NONE);   
  String low_msg;
  #ifdef DEBUG
    Serial.println("doSensorLow() Sending LOW message");
  #endif
  low_msg = String(CLIENT_TOUCH_LOW_MSG) + "_" + String(states.transmissionError);  // put the current touch error state into the UDP message
  if (!sendWifiMessage(low_msg, true)){							                                // call blocking upd message
		states.transmissionError = true;
    #ifdef DEBUG
      Serial.println("doSensorLow() Transmission Error while sending LOW message");
    #endif
  }
} //end void doSensorLow(void)


static inline void checkAliveCounter(void){
  // check server alive counter
  #if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("checkAliveCounter() running on core ");
    Serial.println(xPortGetCoreID());
  #endif
  if (serverAliveCnt <= SERVER_ALIVE_CNT_MAX){
    #ifdef DEBUG
      Serial.printf("checkAliveCounter() Server alive counter current: %d, max reconnect: %d, max server dead: %d\n", serverAliveCnt, SERVER_ALIVE_CNT_MAX, SERVER_ALIVE_CNT_DEAD);
    #endif
    serverAliveCnt++;                                          // increase "the server has not answered" alive counter
    states.aliveCounterError = false;                            // no error, no red LED
  }else{
    if (serverAliveCnt <= SERVER_ALIVE_CNT_DEAD){
      // server alive messages are missing, but try to reconnect
      #ifdef DEBUG
        Serial.printf("checkAliveCounter() No server alive Wifi message during %d service intervals.\n", serverAliveCnt);
      #endif
      serverAliveCnt++;                                         // keep incrementing server alive counter during reconnect tries
      states.wlanComplete      = false;                           // server is (temporary?) dead the connection must be re-establisched
      states.aliveCounterError = true;                            // set internal state to indicate error
    }else{
      // server seems to be dead, client goes to sleep
      #ifdef DEBUG
        Serial.printf("checkAliveCounter() No server alive Wifi message during %d service intervals. Going to sleep\n", serverAliveCnt);
      #endif
      serverAliveCnt           = 0;
      states.wlanComplete      = false;
      states.aliveCounterError = true;                            // set internal state to indicate error
      goSleep();
    } // end if SERVER_ALIVE_CNT_DEAD 
  } // end if SERVER_ALIVE_CNT_MAX

  if (AUTO_SLEEP_TIMER_ENABLE){                                   // check if automatic sleep timer is enabled by config parameter
    if (touchAction) {                                            // check any action in the meantime
      autoSleepTimer = AUTO_SLEEP_TIMER_CYCLES;                   // if there were some action in the meantime, reload the auto sleep timer again
      touchAction = false;
    }else{
      autoSleepTimer--;
      #ifdef DEBUG
        Serial.printf("checkAliveCounter() Auto sleep timer is enabled, current counter value is %d, max %d\n", autoSleepTimer, AUTO_SLEEP_TIMER_CYCLES);
      #endif
    } // end if(touchAction)
    if (autoSleepTimer <= 0){                                     // autosleep timer timeout, go to sleep
      autoSleepTimer = AUTO_SLEEP_TIMER_CYCLES;                   // reload the autosleep timer again, in case sleeping is prevented
      goSleep();                                                  // go to sleep
    } //end if(autoSleepTimer <= 0)
  } //end if(AUTO_SLEEP_TIMER_ENABLE)
} // end void checkAliveCounter()


void checkChargingVoltage(void){
  #if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("checkChargingVoltage() running on core ");
    Serial.println(xPortGetCoreID());
  #endif
  if(digitalRead(CLIENT_CHARGE_VOLTAGE_IN) == LOW){  //CLIENT_CHARGE_VOLTAGE_IN is applied by an inverting transistor 
    states.chargingVoltageApplied = true;
    #ifdef DEBUG
      Serial.printf("checkChargingVoltage() Battery Charging Voltage is applied\n");
    #endif
  }else{
    states.chargingVoltageApplied = false;
    #ifdef DEBUG
      Serial.printf("checkChargingVoltage() No Battery Charging Voltage\n");
    #endif
  }
    if (SLEEP_DURING_CHARGING && states.chargingVoltageApplied){
      #ifdef DEBUG
        Serial.printf("loop() Charging Supply was applied and SLEEP_DURING_CHARGING is enabled. Going to sleep\n");
      #endif
      goSleep();
    } // end if (SLEEP_DURING_CHARGING && states.chargingVoltageApplied)
} //checkChargingVoltage(void)


void checkChargingState(void){
  #if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("checkChargingState() running on core ");
    Serial.println(xPortGetCoreID());
  #endif
  arrayIndexCharging = arrayIndexCharging < 2 ? ++arrayIndexCharging:0;    // increment index and reset to 0 if bigger than 2
  if(digitalRead(CLIENT_CHARGE_IN) == LOW){
    states.chargingBat = true;
    chargeArray[arrayIndexCharging] = true;   // save state charging in the array
    #ifdef DEBUG
        Serial.printf("checkChargingState() Battery is charging\n");
    #endif
  }else{ //current charging state input is "not charging"
    chargeArray[arrayIndexCharging] = false;                            // save state not-charging in the array
    if (states.chargingVoltageApplied){                                 // check if charging stopped because charging voltage is disconnected
      if(!chargeArray[0] && !chargeArray[1] && !chargeArray[2])         // if charging voltage is still applied wait for 3 consecutive charging false values 
        states.chargingBat = false;
      else
        states.chargingBat = true;                                      // if not all 3 charging array inputs are "not charging" set chrging status to "charging"
    }else{ // charging Voltage is not applied 
      states.chargingBat = false;                                       // if charging has stopped because charging voltage is disconnected
    } //end if(states.chargingVoltageApplied)
  } // end if(digitalRead(CLIENT_CHARGE_IN) == LOW){
  controlLed(NONE);                                                     // set LED immediately
} // void checkChargingState(void)


inline void checkBatteryVoltage(void){
  // read analog battery voltage and compare to voltage limits
  #if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("checkBatteryVoltage() running on core ");
    Serial.println(xPortGetCoreID());
  #endif
  String cycle_msg;
  int analogVolts =analogRead(CLIENT_ANALOG_CHANNEL);                           // read analog battery voltage
  float batVoltage = 6.6*analogVolts/4096 + BAT_CORRECTION;                     // Battery Voltage is divided by 2 by resistors on PCB
  batVoltArray[arrayIndexBat] = batVoltage;                                     // store the last 3 voltage values in an array to build an average
  arrayIndexBat = arrayIndexBat < 2 ? ++arrayIndexBat:0;                        // increment index and reset to 0 if bigger than 2
  float sum = 0;
  for(int i=0; i<3; i++)                                                        // sum up last three battery voltages
    sum = sum + batVoltArray[i];
  float average = sum/3;                                                        // calculate average of the last 2 measurements
  #if defined(DEBUG) && defined(VERBOSE)
    Serial.printf("checkBatteryVoltage() Last three battery voltage values were: %.2f, %.2f and %.2f.\n", batVoltArray[0], batVoltArray[1], batVoltArray[2]);
  #endif
  if (average < BAT_LOW_VOLT){ 
    if (average < BAT_CRIT_VOLT){
      // CRITICAL battery voltage
      cycle_msg = String(CLIENT_BAT_CRITICAL_MSG) + "_" + String(average);      // pack message and bat voltage in the UDP frame
      #ifdef DEBUG
        Serial.printf("checkBatteryVoltage() Voltage is critical, current value is ");
        Serial.println(average);
      #endif
      states.batteryError = true;                                               // indicate low or critical battery by LED
    }else{                                                                      // check if battery voltage is critical
      // LOW battery voltage
      cycle_msg = String(CLIENT_BAT_LOW_MSG) + "_" + String(average);           // pack message and bat voltage in the UDP frame
      #ifdef DEBUG
        Serial.printf("checkBatteryVoltage() Voltage is low, current value is ");
        Serial.println(average);
      #endif
      states.batteryError = true;                                               // indicate low or critical battery by LED
    } //end if(average < BAT_CRIT_VOLT)
  }else{ // else if(average < BAT_LOW_VOLT)
    // NORMAL battery voltage
    cycle_msg = String(CLIENT_BAT_OK_MSG) + "_" + String(average);              // pack message and bat voltage in the UDP frame
    #ifdef DEBUG
      Serial.printf("checkBatteryVoltage() Bat Voltage is ok, average measured value is %.2fV (low bat is %.2fV and critical bat is %.2fV)\n", average, BAT_LOW_VOLT, BAT_CRIT_VOLT);
    #endif
    states.batteryError = false;
  } // end if(average < BAT_LOW_VOLT)
  sendWifiMessage(cycle_msg.c_str(), false);
} // end void checkBatteryVoltage(void)


String collectClientData(void){
  // collect client data to transmit to server for display in webserver
  String msg = CLIENT_INFO_MSG + String("_");
  msg += String("ESP32_");
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
  msg += states.mayorHwVersion;
  return msg;
} // collectClientData(void)


static inline void checkWlanStatus(void) {
  // check WLAN status
  #if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("checkWlanStatus() running on core ");
    Serial.println(xPortGetCoreID());
  #endif
  if (WiFi.status() != WL_CONNECTED){
    #ifdef DEBUG
      Serial.printf("checkWlanStatus() WLAN is not connected anymore\n");
    #endif
    states.wlanComplete = false;
  } //end if
  checkRssi();                       // check WLAN signal strength
} //end void checkWlanStatus()


static inline void sendAliveMsg(void){
  // send regular alive messages to server
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
  // Do all service activities that are called regularly by timer interrupt
  #if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("doService() running on core ");
    Serial.println(xPortGetCoreID());
  #endif
  #ifdef DEBUG
    Serial.printf("\ndoService() ***** start service *****\n");
  #endif
  checkChargingVoltage();                                 // check if charging voltage is applied
  checkChargingState();                                   // check for battery charging or not charging
  checkBatteryVoltage();                                  // check battery voltage
  checkAliveCounter();                                    // check server alive counter
  sendAliveMsg();                                         // send client alive
  checkWlanStatus();                                      // check WLAN status and signal strength
  if (states.transmissionError == true){
    states.transmissionError = false;                     // delete transmission error during each service
    #ifdef DEBUG
      Serial.printf("\ndoService() deleting transmission error flag\n");
    #endif
  }
  controlLed(NONE);                                       // set LED immediately
  portENTER_CRITICAL_ISR(&timerMux);                      // protect access to serviceRequest 
  serviceRequest = false;                                 // reset service flag after service
  portEXIT_CRITICAL_ISR(&timerMux);                       // release protection of data
  #ifdef DEBUG
    Serial.printf("doService() ***** end service *****\n\n");
  #endif
} // end void doService(void)


void IRAM_ATTR serviceIsr(void){                        // timer interrupt service routine
  portENTER_CRITICAL_ISR(&timerMux);                    // protect access to serviceRequest 
  serviceRequest = true;
  portEXIT_CRITICAL_ISR(&timerMux);                     // release protection of data
} // end serviceIsr(void)


void wlanInit(void){
  // init wireless connection to server 
  #if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("wlanInit() running on core ");
    Serial.println(xPortGetCoreID());
  #endif
  WiFi.config(clientIpAddr, gateway, subnet);             // set manual IP address
  #ifdef DEBUG
    Serial.printf("\nwlanInit() Connecting to %s ", SSID);
  #endif
  states.wlanCompleteBlink = true;                        // switch on blinking
  WiFi.begin(SSID, password);                             // connect to WLAN 
  while (WiFi.status() != WL_CONNECTED){                  // stay in this loop until a WLAN connection could be established
    controlLed(NONE);                                     // set LED immediately
    delay(WLAN_INIT_PAUSE);                             // wait for a moment before checking connection again
    yield();
    #ifdef DEBUG
      Serial.print(".");
    #endif
    if (serviceRequest == true){                          // do service routine if isr has set the service flag
      checkAliveCounter();                                // check Alive Counter to go to sleep after a while if no server is available
      serviceRequest = false;                             // reset service flag after small alive counter service
    } // end if
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
        if (!strncmp (packetBuffer, SERVER_HELLO_MSG, strlen(SERVER_HELLO_MSG))){
          #ifdef DEBUG
            Serial.println("wlanInit() Detected server hello message, send a reply");
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
      controlLed(NONE); // set LED immediately
    } //end if(packetSize)
    if (serviceRequest == true)                                   //do service routine if isr has set the service flag
      doService();
  } //end while(!states.wlanComplete)
  
  String cycle_msg = collectClientData();                         //Send collected infos about client to server, e.g. to be displayed in Webserver
  #ifdef DEBUG
    Serial.printf("wlanInit() Sending Wifi message with client infos: %s\n", cycle_msg.c_str());
  #endif
	sendWifiMessage(cycle_msg.c_str(), false);
  // after successfull connection, send the current sensor state to the server, even if it has not changed
  if (digitalRead(CLIENT_TOUCH_IN) == (HIGH != CLIENT_TOUCH_POLARITY))  // if current internal state LOW and Pin high
    doSensorHigh();
  else
    doSensorLow();
}//end void WlanInit()


void controlLed(fadingType fading){
  // set the RGB LED color according to the current state
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


void initIo(void){
  // initialize the ESP32 IO
  analogReadResolution(12);                                       // set the resolution to 12 bits (0-4095)
  pixels.begin();                                                 // initialize NeoPixel RBG LED
  pinMode(CLIENT_SLEEP_OUT,         OUTPUT);                      // controls external sleep/power hardware
  pinMode(CLIENT_TOUCH_IN,          INPUT_PULLUP);                // set Touch DIO to input with pullup
  pinMode(CLIENT_CHARGE_VOLTAGE_IN, INPUT_PULLUP);                // set charge voltage DIO to input with pullup
  pinMode(CLIENT_CHARGE_IN,         INPUT_PULLUP);                // set charging state DIO to input with pullup
  #ifdef CLIENT_HW_REVISION_2_0
    pinMode(CLIENT_HW_REVISION_0, INPUT);                         // client hardware PCB revision bit 0 (only for PCB revision 2.0 or later)
    pinMode(CLIENT_HW_REVISION_1, INPUT);                         // client hardware PCB revision bit 1 (only for PCB revision 2.0 or later)
    pinMode(CLIENT_HW_REVISION_2, INPUT);                         // client hardware PCB revision bit 2 (only for PCB revision 2.0 or later)
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

void getVersionNumbers(){
  //takes SW version string and converts it into 3 integers (mayor, minor and maintenance)
  char mayorSwVersionChar[3];                         // temp variable for mayor software version
  char minorSwVersionChar[3];                         // temp variable for minor software version
  char maintSwVersionChar[3];                         // temp variable for maintenance software version
  char tempVersionString[16];                         // temp version string
  char *token;
  
  strcpy(tempVersionString, clientSwVersion);
  token = strtok(tempVersionString, ".");             // cut out token from software version e.g. 03.02.01 by delimiter "."
  strcpy(mayorSwVersionChar, token);                  // copy token to mayor version
  states.mayorSwVersion = atoi(mayorSwVersionChar);   // convert mayor version to integer
  token = strtok(NULL, ".");                          // proceed with minor version
  strcpy(minorSwVersionChar, token);
  states.minorSwVersion = atoi(minorSwVersionChar);
  token = strtok(NULL, ".");                          // proceed with maintenance version
  strcpy(maintSwVersionChar, token);
  states.maintSwVersion = atoi(maintSwVersionChar);
  
  // convert the three hardware input bit to a mayor hardware version number
  #ifdef CLIENT_HW_REVISION_2_0
    states.mayorHwVersion = (digitalRead(CLIENT_HW_REVISION_2) << 2) + (digitalRead(CLIENT_HW_REVISION_1)<< 1) + digitalRead(CLIENT_HW_REVISION_0); //construct version number from 3 input bit
  #endif
 }


void goAlive(void){
  // do everything that needs to be done when going alive
  #ifdef DEBUG
    Serial.println("goAlive() Going alive");
  #endif
  states.wlanCompleteBlink = false;                             // stop blinking when going alive
  controlLed(UP); 
  #ifdef SHOW_STATE_COLORS
    showStateColors();
  #endif
}  //goAlive()


void goSleep(){ 
  // go to sleep to save battery 
  if (!(NO_SLEEP_WHILE_CHARGING && states.chargingBat)){    // prevent goint to sleep if battery is charging and the NO_SLEEP_WHILE_CHARGING flag is set
    #ifdef DEBUG
      Serial.println("goSleep() Going to sleep");
    #endif
    // LED shutdown
    bool temp = states.wlanCompleteBlink;                                         // save current blinking state
    states.wlanCompleteBlink = false;                                             // stop blinking when going to sleep
    controlLed(DOWN);                                                             // decrease LED brightness during shutdown and indicate fading is true
    delay(1000);
    states.wlanCompleteBlink = temp;                                              // recover blinking state, does only make sanse if the client does not go to sleep, e.g. by open sensor (instant re-wakeup)
    digitalWrite(CLIENT_SLEEP_OUT, LOW);                                          // switch off external power
  }else{  // if(!((NO_SLEEP_WHILE_CHARGING)
  #ifdef DEBUG
    Serial.println("goSleep() Not going to sleep, because NO_SLEEP_WHILE_CHARGING is set");
  #endif
  } // if(!((NO_SLEEP_WHILE_CHARGING)
} //end void goSleep()


void setup(void){
  // general setup function
  #ifdef DEBUG
    Serial.begin(BAUD_RATE);                                                    // Setup Serial Interface with baud rate
  #endif

  initIo(); // init all IO channels
  getVersionNumbers();   // fills data structure with software and hardware versio numbers
  
  #ifdef DEBUG
    Serial.println("\n\n\nsetup() I am the 3D Touch Probe Sensor Client");
    Serial.println("setup() Copyright by Heiko Kalte 2025 (h.kalte@gmx.de)");
    Serial.printf("setup() My software version is: %s\n", clientSwVersion);
    Serial.printf("setup() My hardware/PCB version is: %d\n", states.mayorHwVersion);
    if (states.mayorSwVersion > states.mayorHwVersion)
      Serial.printf("setup() WARNING: Software mayor version (%d) is higher than hardware mayor version (%d)\n", states.mayorSwVersion, states.mayorHwVersion);

    printChipInfo();
  #endif
  goAlive();
  //Setup timer interrup for service routines
  serviceTimer = timerBegin(1000000);                                         // Set timer frequency to 1MHz
  timerAttachInterrupt(serviceTimer, &serviceIsr);                            // Attach onTimer function to our timer. 
  timerAlarm(serviceTimer, SERVICE_INTERVALL_ESP32, true, 0);                 // Set alarm to call onTimer function every second (value in microseconds). Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
  wlanInit();                                                                   // Init wlan communication with server
} //end setup()


void loop(void){  
  //re-reconnect to server if connection got lost
  if (states.wlanComplete == false){
    #ifdef DEBUG
      Serial.printf("loop() Lost connection to server, need to re-init wlan connection\n");
    #endif
    wlanInit();
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

  // for more responsiveness check for battery CHARGING STATUS changes within loop() not only during service function
  if ((states.chargingBat == false) && (digitalRead(CLIENT_CHARGE_IN) == LOW)){ // if charging has just begun (state is non-charging, but current input says charging)
    checkChargingState();
  }

  // for more responsiveness check for battery CHARGING VOLTAGE status changes within loop() not only during service function
  if (states.chargingVoltageApplied != (digitalRead(CLIENT_CHARGE_VOLTAGE_IN) == LOW)){    // Charging voltage is inverted by transistor. If current charging voltage state is different from the charging voltage state
    checkChargingVoltage();
    checkChargingState();
  }

  // check for UDP commands from server
  int packetSize = Udp.parsePacket();
  if (packetSize){  // received UDP package
    int len = Udp.read(packetBuffer, UDP_PACKET_MAX_SIZE);
    if (len > 0)
      packetBuffer[len] = '\0';

    // receiving sleep message from server
    if (!strncmp (packetBuffer, SERVER_SLEEP_MSG, strlen(SERVER_SLEEP_MSG))){    // wäre besser, wenn der server auch mal was an die Nachrichten anhängen sollte
      #ifdef DEBUG
        Serial.println("loop() Detected sleep command\n");
      #endif
      goSleep();                                            // go to sleep
      return;
    }

    // receiving alive message from server and resetting the alive counter
    if (!strncmp (packetBuffer, SERVER_ALIVE_MSG, strlen(SERVER_ALIVE_MSG))){
      #ifdef DEBUG
        Serial.println("loop() Detected server alive command");
      #endif
      serverAliveCnt = 0;                               //reset alive counter for server
      return;
    }

    // receiving hello message from server and sending reply
    if (!strncmp (packetBuffer, SERVER_HELLO_MSG, strlen(SERVER_HELLO_MSG))){
      #ifdef DEBUG
        Serial.println("loop() Detected server hello command, send a reply");
      #endif
	    sendWifiMessage(CLIENT_REPLY_MSG, false);
      return;
    }
    
    // receiving reply message from server and do nothing
    if (!strncmp (packetBuffer, SERVER_REPLY_MSG, strlen(SERVER_REPLY_MSG))){
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