
/**  3D Sensor Client
  *   
  *  @author Heiko Kalte  
  *  @date 22.02.2025 
  *  Copyright by Heiko Kalte (h.kalte@gmx.de)
  */
  
  // TODO/Suggestions
  // battery loading could be send to the server to be displayed in the webserver
  // identifier in udp messages?

  // Using Arduino ESP32 PICO-D4 (but ist is a ESP32 PICO V3-02 device)

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
  ADC_MODE(ADC_VCC);                                         // measure supply Voltage of the board with analog in, only ESP8266
#endif

int        server_alive_cnt         = 0;                     // current "server is alive counter" value
bool       high_command_acknowledge = true;                  // indicates if the sended HIGH command was acknowledge by the server, sometimes UDP packages seem to get lost
bool       low_command_acknowledge  = true;                  // indicates if the sended LOW command was acknowledge by the server, sometimes UDP packages seem to get lost
int        ackCounterLow            = 0;                     // counter for timeout waiting for the server to replay to the low UDP command
int        ackCounterHigh           = 0;                     // counter for timeout waiting for the server to replay to the high UDP command
char       packetBuffer[UDP_PACKET_MAX_SIZE];                // general buffer to hold UDP messages
WiFiUDP    Udp;
long       rssi;                                             // WLAN signal strength
bool       serviceRequest           = false;                 // interrupt service can request a service intervall by this flag
int        clientHwPcbRevision      = 0;                     // From client PCB version 2 onwards the version can be read out from the PCB coded by 3 input bit
int        analogVolts              = 0;
struct statesType
{
    bool    touchState               = LOW;                   // current touch sensor state
    bool    charging                 = false;                 // global variable to indicate if battery is charging
    bool    wlanComplete             = false;                 // global variable to indicate if wlan connection is established completely
    bool    wlanCompleteBlink        = true;                  // blink wlan LED if enlighted
    bool    batteryError             = false;                 // error with the battery
    bool    rssiError                = false;                 // error with the WLAN rssi
    bool    transmissionError        = false;                 // error with the touch state
    bool    aliveCounterError        = false;                 // error with server alive counter
}; 
statesType states; 

#ifdef CLIENT_ESP32
  hw_timer_t *serviceTimer = NULL;
  portMUX_TYPE timerMux               = portMUX_INITIALIZER_UNLOCKED;
#endif

#ifdef CYCLETIME
  int32_t bTimeLow,  eTimeLow;
  int32_t bTimeHigh, eTimeHigh;

  static inline int32_t asm_ccount(void) {              // asm-helpers taken from https://sub.nanona.fi/esp8266/timing-and-ticks.html, reading the CCOUT register with clock ticks
    int32_t r;
    asm volatile ("rsr %0, ccount" : "=r"(r));
    return r;
  }
#endif

Adafruit_NeoPixel pixels(1, CLIENT_RGB_LED_OUT, NEO_GRB + NEO_KHZ800);

static inline bool sendWifiMessage(String msg, bool blocking, bool udp)
{
	if (udp){
		Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());          // send UDP packet to server to indicate the 3D touch change 
		#ifdef CLIENT_ESP32
      Udp.printf(msg.c_str());
		#else //ESP8266
			Udp.write(msg.c_str());
		#endif
		Udp.endPacket();
		if(blocking){
			//########## blocking begin
      //##### while no answer
        if(!strcmp(packetBuffer, msg.c_str())){    // check if exactly the same message comes back from server
        #ifdef DEBUG
          Serial.println("loop() Detected acknowledge message from server.");
        #endif
        //high_command_acknowledge = true;
        //ackCounterHigh           = 0;                                             // reset counter, when receiving the server acknowledge message
        /*#ifdef CYCLETIME     
          eTimeHigh = asm_ccount();                                               //take end time for client to server to client cycle time measurement
          #ifdef DEBUG
            Serial.printf("loop() Measured Wifi cycle time for sensor HIGH activation: %u ticks\n", ((uint32_t)(eTimeHigh - bTimeHigh)));
          #endif
          //send cycles to server e.g. to be displayed by the webserver
          String cycle_msg = CLIENT_CYCLE_MSG + String((uint32_t)(eTimeHigh - bTimeHigh)); // pack end-time minus begin-time into a message
          sendWifiMessage(cycle_msg.c_str(), false, true);
        #endif
        */
        
        return true;
        } //if(!strcmp(packetBuffer, msg.c_str()))
      // end while
      //##### blocking ende
			return false;
		} //if(blocking)
		return true;
	}else{ // not upd message
		// tcp or others
    return false; // not implemented yet
	}
	return false;
}


 static inline void doSensorHigh(void){
  #if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("doSensorHigh() running on core ");
    Serial.println(xPortGetCoreID());
  #endif
  String high_msg;
  #ifdef DEBUG
    Serial.println(CLIENT_TOUCH_HIGH_MSG);
  #endif
  if (states.wlanComplete){
      high_msg = CLIENT_TOUCH_HIGH_MSG + String(states.transmissionError); // put the current touch error state into the UDP message
	  sendWifiMessage(high_msg, false, true);
      // rückgabewert überprüfen und blocking einbauen
	  high_command_acknowledge = false;                           // set acknowledge to false until the server responses with the same command
      states.touchState        = HIGH;                            // remember the current state
      controlLed(NONE);                                           // set LED immediately
      #ifdef CYCLETIME
        bTimeHigh = asm_ccount();                                 // take begin time for client to server to client cycle time measurement
      #endif
  } // end if (states.wlanComplete)
} // end void doSensorHigh(void)


static inline void doSensorLow(void){
  #if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("doSensorLow() running on core ");
    Serial.println(xPortGetCoreID());
  #endif
  String low_msg;
  #ifdef DEBUG
    Serial.println(CLIENT_TOUCH_LOW_MSG);
  #endif
  if (states.wlanComplete){
    low_msg = CLIENT_TOUCH_LOW_MSG + String(states.transmissionError); // put the current touch error state into the UDP message
	sendWifiMessage(low_msg, false, true);
      // rückgabewert überprüfen und blocking einbauen
    low_command_acknowledge = false;                    // set acknowledge to false until the server responses with the same command
    states.touchState       = LOW;                      // remember the current state
    controlLed(NONE); // set LED immediately
    #ifdef CYCLETIME
      bTimeLow = asm_ccount();                          // take begin time for client to server to client cycle time measurement
    #endif
  } //end if (states.wlanComplete)
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
    states.charging = true;
    #ifdef DEBUG
        Serial.printf("checkBatteryVoltage() Battery is charging\n");
    #endif
  }else{
    states.charging = false;
  }

  // measure battery voltage
  #ifdef CLIENT_ESP32
    analogVolts =analogRead(CLIENT_ANALOG_CHANNEL);
    float batVoltage = 6.6*analogVolts/4096 + BAT_CORRECTION;                          // Battery Voltage is divided by 2 by resistors on PCB
  #else
    int analogVolts = ESP.getVcc();
    float batVoltage = analogVolts / 1000.0 + BAT_CORRECTION;
  #endif
  
  if (batVoltage < BAT_LOW_VOLT){
    if (batVoltage < BAT_CRIT_VOLT){                                // check if battery voltage is critical
      cycle_msg = CLIENT_BAT_CRITICAL_MSG + String(batVoltage);     // pack message and bat voltage in the UDP frame
      #ifdef DEBUG
        Serial.printf("checkBatteryVoltage() Voltage is critical, current value is ");
        Serial.println(batVoltage);
      #endif
    }else{ //if(batVoltage < BAT_CRIT_VOLT)                         // battery is only low
      cycle_msg = CLIENT_BAT_LOW_MSG + String(batVoltage);          // pack message and bat voltage in the UDP frame
      #ifdef DEBUG
        Serial.printf("checkBatteryVoltage() Voltage is low, current value is ");
        Serial.println(batVoltage);
      #endif
    }  // if(batVoltage < BAT_CRIT_VOLT)
    states.batteryError = true;                           // indicate low or critical battery by LED
  }else{ // if(batVoltage < BAT_LOW_VOLT)                                             // battery is ok
    cycle_msg = CLIENT_BAT_OK_MSG + String(batVoltage);   // pack message and bat voltage in the UDP frame
    #ifdef DEBUG
      Serial.printf("checkBatteryVoltage() Bat Voltage is ok, measured value is %.2fV (low bat is %.2fV and critical bat is %.2fV)\n", batVoltage, BAT_LOW_VOLT, BAT_CRIT_VOLT);
    #endif
    states.batteryError = false;
  } // if(batVoltage < BAT_LOW_VOLT)

  if (states.wlanComplete == true){               // check for WLAN connection to avoid exception
    sendWifiMessage(cycle_msg.c_str(), false, true);
  }else{ // if(states.wlanComplete == true)
    #ifdef DEBUG
      Serial.println("checkBatteryVoltage() Not sending Wifi message, due to missing server connection");
    #endif
  }  // if(states.wlanComplete == true)
} // end checkBatteryVoltage(void)

String collectClientData(void){
  String msg = CLIENT_INFO_MSG;
  #ifdef CLIENT_ESP32
   msg += String("ESP32; ");
  #else
   msg += String("ESP8266; ");
  #endif
  #ifdef DEBUG
   msg += String("DEBUG On; ");
  #else
   msg += String("DEBUG Off; ");
  #endif
  #ifdef CYCLETIME
   msg += String("CYCLETIME On; ");
  #else
   msg += String("CYCLETIME Off; ");
  #endif
  msg += clientSwVersion;
  msg += String("; ");
  msg += clientHwPcbRevision;
  return msg;
} // collectClientData(void)

static inline void checkWlanStatus(void) {
  #if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("checkWlanStatus() running on core ");
    Serial.println(xPortGetCoreID());
  #endif

  if (states.wlanComplete == true){
    if (WiFi.status() != WL_CONNECTED){
      #ifdef DEBUG
        Serial.printf("checkWlanStatus() WLAN is not connected anymore\n");
      #endif
      states.wlanComplete = false;
    }
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
  } //end if(states.wlanComplete)

  // ##### twice check of states.wlanComplete == true, this could be imrpoved
  if (states.wlanComplete == true){
    String cycle_msg = CLIENT_RSSI_MSG + String(rssi);   // pack message and rssi in the UDP frame
	  sendWifiMessage(cycle_msg, false, true);
  }else{
    #ifdef DEBUG
      Serial.printf("checkWlanStatus() Not sending Wifi message du to missing server connection\n");
    #endif
  } // if(states.wlanComplete == true)
} //end void checkWlanStatus()


static inline void sendAliveMsg(void) {
  #if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("sendAliveMsg() running on core ");
    Serial.println(xPortGetCoreID());
  #endif
  // send alive message
  if (states.wlanComplete == true){               // check for WLAN connection to avoid exception
    #ifdef DEBUG
      Serial.printf("sendAliveMsg() Sending Wifi alive message to server\n");
    #endif
	  sendWifiMessage(CLIENT_ALIVE_MSG, false, true);
  }else{ //if(states.wlanComplete == true)
    #ifdef DEBUG
      Serial.println("sendAliveMsg() Not sending Wifi alive message because of missing server connection");
    #endif
  } // if(states.wlanComplete == true)
} // end void sendAliveMsg(void)


static inline void doService(void){
  #if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("doService() running on core ");
    Serial.println(xPortGetCoreID());
  #endif
  // Does all service activities that are called regularly by timer interrupt
  #ifdef DEBUG
    Serial.printf("\ndoService() ***** start service *****\n");
  #endif
  checkBatteryVoltage();                                  // check battery in service
  checkAliveCounter();                                    // check server alive counter in service
  sendAliveMsg();                                         // send client alive in service
  checkWlanStatus();                                      // check WLAN status and signal strength
  states.transmissionError =false;                        // delete transmission error 
  controlLed(NONE);                                       // set LED immediately
  #ifdef CLIENT_ESP32
    portENTER_CRITICAL_ISR(&timerMux);                    // protect access to serviceRequest 
  #endif
    serviceRequest = false;                               // reset service flag after service
  #ifdef CLIENT_ESP32
    portEXIT_CRITICAL_ISR(&timerMux);                  // release protection of data
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
    controlLed(NONE); // set LED immediately
    if (!states.wlanCompleteBlink)                        // if blinking is enabled, do not add an additional delay
      delay(WLAN_INIT_PAUSE);                             // wait for a moment before checking connection again
    yield();
    #ifdef DEBUG
      Serial.print(".");
    #endif
    if (serviceRequest == true)                           // do service routine if isr has set the service flag
      doService();
  }//end while WiFi.status

  Udp.begin(clientUdpPort);                               // now listening for a server to send UDP messages
  #ifdef DEBUG
    Serial.println(" connected to WLAN network");
  #endif
  
  while(!states.wlanComplete){
    #ifdef DEBUG
      Serial.printf("wlanInit() Sending hello message to IP %s, UDP port %d\n", serverIpAddr.toString().c_str(), serverUdpPort);
    #endif
    sendWifiMessage(CLIENT_HELLO_MSG, false, true);
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
            Serial.println("initWlan() Detected server hello message during initWlan(), send a reply");
          #endif
		  sendWifiMessage(CLIENT_REPLY_MSG, false, true);
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
      } // end if (!strcmp(Udp.remoteIP().toString().c_str()
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
  
  //Send infos about client
  String cycle_msg = collectClientData();
  #ifdef DEBUG
    Serial.printf("wlanInit() Sending Wifi message with client infos: %s\n", cycle_msg.c_str());
  #endif
	sendWifiMessage(cycle_msg.c_str(), false, true);
}//end void WlanInit()


void controlLed(fadingType fading){
  #if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("controlLed() running on core ");
    Serial.println(xPortGetCoreID());
  #endif

  // brightness is currently only used in RGB mode 
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
              if (states.charging){
                pixels.setPixelColor(0, pixels.Color(CHARGE_COLOR));
              }else{
                if (states.batteryError){
                  pixels.setPixelColor(0, pixels.Color(BATTERY_ERROR_COLOR));
                }else{ //if none of the above applys, make LED white to indicate, that the sensor is on power
                  pixels.setPixelColor(0, pixels.Color(POWER_COLOR));
                } //end if battery charging
              }
            }
          }
        }
      } //end if(states.wlanComplete)
    } //end if states.touchState
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
    }else{  //if (fading == UP)
      if(CLIENT_ALLOW_LED_FADING){
        pixels.setPixelColor(0, pixels.Color(POWER_COLOR));                               // Power color during fading
        for(int i = CLIENT_RGB_BRIGHTNESS; i >= 0; i = i - 1){
          pixels.setPixelColor(0, pixels.Color(i,i,i));
          pixels.show();
          delay(CLIENT_RGB_FADE_SPEED);
        } //end for
      } //if(CLIENT_ALLOW_LED_FADING)
    } // if (fading ==UP)
  } //if(!fading)
  pixels.show();                                                                      // Send the updated pixel colors to the hardware.
} //end controlRgbLed


static inline void initIo(void) {
  #ifdef CLIENT_ESP32
    analogReadResolution(12);   //set the resolution to 12 bits (0-4095) for ESP32 only
  #endif

  pixels.begin();                                               // initialize NeoPixel RBG LED

  pinMode(CLIENT_SLEEP_OUT,    OUTPUT);                           // control external sleep hardware
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


static inline void goAlive(){
  #ifdef DEBUG
    Serial.println("goAlive() Going alive");
  #endif
  states.wlanCompleteBlink = false;                             // stop blinking when going to sleep
  controlLed(UP); 
}  // goAlive()


static inline void goSleep(){  

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
  /*#if defined(DEBUG) && defined(DEBUG_SHOW_CORE)
    Serial.print("loop() running on core ");
    Serial.println(xPortGetCoreID());
  #endif*/
  
  //reconnect to server if connection got lost
  if (states.wlanComplete == false){
    #ifdef DEBUG
      Serial.printf("loop() Lost connection to server, need to re-init wlan connection\n");
    #endif
    wlanInit();
  }

  // for more responsiveness check for battery charging status changes
  if (states.charging != (digitalRead(CLIENT_CHARGE_IN) == LOW)){
    #ifdef DEBUG
      Serial.printf("loop() charging state changed, calling checkBatteryVoltage()\n");
      //Serial.printf(states.charging);
      //Serial.printf(digitalRead(CLIENT_CHARGE_IN)==LOW);
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

  //increase the counter, if an LOW/HIGH acknowledge from the server is pending
   if (low_command_acknowledge == false)
      ackCounterLow++;                                    // increase server acknowledge counter if client waits for a LOW/HIGH reply from the server

   if (high_command_acknowledge == false)
      ackCounterHigh++;                                   // increase server acknowledge counter if client waits for a LOW/HIGH reply from the server

   //Re-send LOW messages to server if server does not reply within time out (avoid cnc crash during sensing)
   if (ackCounterLow > SERVER_AQUN_CNT_MAX){
     if(low_command_acknowledge == false){
        #ifdef DEBUG
          Serial.printf("loop() Acknowledge timeout error for LOW message, trying to re-transmit\n");
        #endif
        doSensorLow();                                    // server did not answer to low message from client, re-transmit
        ackCounterLow   = 0;                              // reset counter for re-transmitting low message
        states.transmissionError = true;                           // indicate error by red LED
        controlLed(NONE); // set LED immediately
     }
   }

   //Re-send HIGH messages to server if server does not reply within time out (avoid CNC crash during sensing)
   if (ackCounterHigh > SERVER_AQUN_CNT_MAX){
     if(high_command_acknowledge == false){
        #ifdef DEBUG
          Serial.printf("loop() Acknowledge timeout error for HIGH message, trying to re-transmit\n");
        #endif
        doSensorHigh();                                   // server did not answer to low message from client, re-transmit
        ackCounterHigh  = 0;                              // reset counter for re-transmitting low message
        states.transmissionError = true;                           // indicate error by red LED
        controlLed(NONE); // set LED immediately
     }
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
      goSleep();
      return;
    }

    // receiving alive message from server and resetting the alive counter
    if(!strcmp(packetBuffer, SERVER_ALIVE_MSG)){
      #ifdef DEBUG
        Serial.println("loop() Detected server alive command");
      #endif
      server_alive_cnt = 0; //reset alive counter for server
      return;
    }

    // receiving hello message from server
    if(!strcmp(packetBuffer, SERVER_HELLO_MSG)){
      #ifdef DEBUG
        Serial.println("loop() Detected server hello command, send a reply");
      #endif
	    sendWifiMessage(CLIENT_REPLY_MSG, false, true);
      return;
    }
    
    // receiving reply message from server and do nothing
    if(!strcmp(packetBuffer, SERVER_REPLY_MSG)){
      #ifdef DEBUG
        Serial.println("loop() Detected server reply message.");
      #endif
      return;
    }

    // receiving HIGH acknowledge message from server
    if(!strcmp(packetBuffer, CLIENT_TOUCH_HIGH_MSG)){
      #ifdef DEBUG
        Serial.println("loop() Detected HIGH acknowledge message from server.");
      #endif
      high_command_acknowledge = true;
      ackCounterHigh           = 0;                                             // reset counter, when receiving the server acknowledge message
      #ifdef CYCLETIME
        eTimeHigh = asm_ccount();                                               //take end time for client to server to client cycle time measurement
        #ifdef DEBUG
          Serial.printf("loop() Measured Wifi cycle time for sensor HIGH activation: %u ticks\n", ((uint32_t)(eTimeHigh - bTimeHigh)));
        #endif
        //send cycles to server e.g. to be displayed by the webserver
        String cycle_msg = CLIENT_CYCLE_MSG + String((uint32_t)(eTimeHigh - bTimeHigh)); // pack end-time minus begin-time into a message
        sendWifiMessage(cycle_msg.c_str(), false, true);
      #endif
      return;
    }

    //receiving LOW acknowledge message from server
    if(!strcmp(packetBuffer, CLIENT_TOUCH_LOW_MSG)){
      #ifdef DEBUG
        Serial.println("loop() Detected LOW acknowledge message from server.");
      #endif
      low_command_acknowledge = true;
      ackCounterLow           = 0;
      #ifdef CYCLETIME
          eTimeLow = asm_ccount();             //take end time for client to server to client cycle time measurement
        #ifdef DEBUG
          Serial.printf("loop() Measured Wifi cycle time for sensor LOW activation: %u ticks\n", ((uint32_t)(eTimeLow - bTimeLow)));
        #endif
        //send cycles to server e.g. to be displayed by the webserver
        String cycle_msg = CLIENT_CYCLE_MSG + String((uint32_t)(eTimeLow - bTimeLow)); // pack end-time minus begin-time into a message
		    sendWifiMessage(cycle_msg.c_str(), false, true);
      #endif
      return;
    } // end if(!strcmp(packetBuffer, CLIENT_TOUCH_LOW_MSG))
    
    //receiving unknown UDP message
    #ifdef DEBUG
      Serial.println("loop() Received unknown command by UDP");
      Serial.println(packetBuffer);
    #endif
  } // end if(packetSize)

  if (serviceRequest == true)        //do service routine if isr has set the service flag
      doService();

} //end loop()