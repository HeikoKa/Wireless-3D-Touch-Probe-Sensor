/**  Client
  *   
  *  @author Heiko Kalte  
  *  @date 29.11.2024 
  * 
  *  @version 0.2
  */
  // 0.1:   initial version
  // 0.2:   Changed function due to new sleep hardware (former deep sleep consumes to much power)

  //TODO
  
  
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Ticker.h>
#include "Z:\Projekte\Mill\HeikosMill\3D Taster\Arduino\GIT\3D-Touch-Sensor\src\3D-Header.h"  // Arduino IDE does not support relative paths

using namespace CncSensor;

ADC_MODE(ADC_VCC);                                        // measure supply Voltage of the board with analog in 

bool    wlan_complete            = false;                 // global variable to indicate if wlan connection is established completely 
int     server_alive_cnt         = 0;                     // current "server is alive counter" value
bool    touch_state              = LOW;                   // current touch sensor state

bool    high_command_acknowledge = true;                  // indicates if the sended HIGH command was acknowledge by the server, sometimes UDP packages seem to get lost
bool    low_command_acknowledge  = true;                  // indicates if the sended LOW command was acknowledge by the server, sometimes UDP packages seem to get lost
int     ackCounterLow            = 0;                     // counter for timeout waiting for the server to replay to the low UDP command
int     ackCounterHigh           = 0;                     // counter for timeout waiting for the server to replay to the high UDP command
char    packetBuffer[UDP_PACKET_MAX_SIZE];                // general buffer to hold UDP messages
bool    batteryError             = false;                 // error with the battery
bool    rssiError                = false;                 // error with the WLAN rssi
bool    touchStateError          = false;                 // error with the touch state
bool    aliveCounterError        = false;                 // error with server alive counter
WiFiUDP Udp;
long    rssi;                                             // WLAN signal strength


#ifdef CYCLETIME
  int32_t bTimeLow, eTimeLow;
  int32_t bTimeHigh, eTimeHigh;

  static inline int32_t asm_ccount(void) {              // asm-helpers taken from https://sub.nanona.fi/esp8266/timing-and-ticks.html, reading the CCOUT register with clock ticks
    int32_t r;
    asm volatile ("rsr %0, ccount" : "=r"(r));
    return r;
  }
#endif


 static inline void doSensorHigh(void){
  String high_msg;
  digitalWrite(TOUCH_LED, LOW);                           // indicate the current touch state by LED output
  #ifdef DEBUG
    Serial.println(CLIENT_TOUCH_HIGH_MSG);
  #endif
  if (wlan_complete){
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());          // send UDP packet to server to indicate the 3D touch change 
      high_msg = CLIENT_TOUCH_HIGH_MSG + String(touchStateError); // put the current touch error state into the UDP message
      Udp.write(high_msg.c_str());
      Udp.endPacket();
      high_command_acknowledge = false;                   // set acknowledge to false until the server responses with the same command
      touch_state              = HIGH;                    // remember the current state
      #ifdef CYCLETIME
        bTimeHigh = asm_ccount();                         // take begin time for client to server to client cycle time measurement
      #endif
  }
}


static inline void doSensorLow(void){
  String low_msg;
  digitalWrite(TOUCH_LED, HIGH);                          // indicate the current touch state by LED output
  #ifdef DEBUG
    Serial.println(CLIENT_TOUCH_LOW_MSG);
  #endif
  if (wlan_complete){
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());        // send UDP packet to server to indicate the 3D touch change
    low_msg = CLIENT_TOUCH_LOW_MSG + String(touchStateError); // put the current touch error state into the UDP message
    Udp.write(low_msg.c_str());
    Udp.endPacket();
    low_command_acknowledge = false;                    // set acknowledge to false until the server responses with the same command
    touch_state             = LOW;                      // remember the current state
    #ifdef CYCLETIME
      bTimeLow = asm_ccount();                          // take begin time for client to server to client cycle time measurement
    #endif
  } //wlan_complete
}


static inline void checkAliveCounter(void){
  if (server_alive_cnt < SERVER_ALIVE_CNT_MAX){
    #ifdef DEBUG
      Serial.printf("checkAliveCounter(): Server alive counter current: %d, max reconnect: %d, max server dead: %d\n", server_alive_cnt, SERVER_ALIVE_CNT_MAX, SERVER_ALIVE_CNT_DEAD);
    #endif
    server_alive_cnt++;                                   // increase "the server has not answered" alive counter
    aliveCounterError = false;                            // no error, no red LED
  }else{
    if (server_alive_cnt < SERVER_ALIVE_CNT_DEAD){
      // server alive messages are missing, but try to reconnect
      #ifdef DEBUG
        Serial.printf("checkAliveCounter(): No server alive udp message during %d service intervals. Going to sleep\n", server_alive_cnt);
      #endif
      server_alive_cnt++;                                 // keep incrementing server alive counter during reconnect tries
      wlan_complete     = false;                          // server is (temporary?) dead the connection must be re-establisched
      aliveCounterError = true;                           // set red LED to indicate error
    }else{
      // server seems to be dead, client goes to sleep
      #ifdef DEBUG
        Serial.printf("checkAliveCounter(): No server alive udp message during %d service intervals. Going to sleep\n", server_alive_cnt);
      #endif
      server_alive_cnt  = 0;
      wlan_complete     = false;
      aliveCounterError = true;                           // set red LED to indicate error
      digitalWrite(SLEEP_OUT, HIGH);                      // switch off external power

    } // end if SERVER_ALIVE_CNT_DEAD 
  } // end if SERVER_ALIVE_CNT_MAX
} //end checkAliveCounter()



static inline void checkBatteryVoltage(void){
  String cycle_msg;
  float batVoltage = (ESP.getVcc() / 1000.0);            // measure battery voltage and recalculate the sensor value to the battery voltage

  if (batVoltage < BAT_LOW_VOLT){
    if (batVoltage < BAT_CRIT_VOLT){                      // check if battery voltage is critical
      cycle_msg = CLIENT_BAT_CRITICAL_MSG + String(batVoltage); // pack message and bat voltage in the UDP frame
      #ifdef DEBUG
        Serial.printf("checkBatteryVoltage(): Voltage is critical, current value is ");
        Serial.println(batVoltage);
      #endif
    }else{                                                // battery is only low
      cycle_msg = CLIENT_BAT_LOW_MSG + String(batVoltage); // pack message and bat voltage in the UDP frame
      #ifdef DEBUG
        Serial.printf("checkBatteryVoltage(): Voltage is low, current value is ");
        Serial.println(batVoltage);
      #endif
    }
    batteryError = true;                                  // indicate low or critical battery by LED
  }else{                                                  // battery is ok
    cycle_msg = CLIENT_BAT_OK_MSG + String(batVoltage);   // pack message and bat voltage in the UDP frame
    #ifdef DEBUG
      Serial.printf("checkBatteryVoltage(): Bat Voltage is ok, measured value is %.2fV (low bat is %.2fV and critical bat is %.2fV)\n", batVoltage, BAT_LOW_VOLT, BAT_CRIT_VOLT);
    #endif
    batteryError = false;
  }
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(cycle_msg.c_str());
  Udp.endPacket();
}


static inline void checkWlanStatus(void) {
  if (wlan_complete){
    if (WiFi.status() != WL_CONNECTED){
      #ifdef DEBUG
        Serial.printf("checkWlanStatus(): WLAN is not connected anymore\n");
      #endif
      wlan_complete = false;
    }

    rssi = WiFi.RSSI();
    if (rssi < WIFI_RSSI_REPORT_LEVEL){
      #ifdef DEBUG
        Serial.print("checkWlanStatus(): ERROR: WLAN signal strength is critical, RSSI:");
        Serial.println(rssi);
      #endif
      rssiError = true;
    }else{
      #ifdef DEBUG
        Serial.print("checkWlanStatus(): WLAN signal strength is ok, RSSI:");
        Serial.println(rssi);
      #endif
      rssiError = false;
    }
  } //end if (wlan_complete)
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  String cycle_msg = CLIENT_RSSI_MSG + String(rssi);   // pack message and rssi in the UDP frame
  Udp.write(cycle_msg.c_str());
  Udp.endPacket();
} //end void checkWlanStatus()


static inline void sendAliveMsg(void) {
  // send alive message regardless of the WLAN state and if connection is established
  #ifdef DEBUG
    Serial.printf("sendAliveMsg(): Sending UDP alive message to server\n");
  #endif
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write(CLIENT_ALIVE_MSG);
  Udp.endPacket();
}

static inline void setErrorLED(bool batteryError, bool rssiError, bool touchStateError, bool aliveCounterError){
  if (batteryError || rssiError || touchStateError || aliveCounterError)
    digitalWrite(CLIENT_ERROR_OUT, LOW);                // enlight red LED if any error has occurred
  else
    digitalWrite(CLIENT_ERROR_OUT, HIGH);               // switch off light if no error is reported
}

static inline void doService(void) {
  // Does all service activities that are called regularly by timer interrupt
  #ifdef DEBUG
    Serial.printf("\ndoService(): start service\n");
  #endif
  checkBatteryVoltage();                                  // check battery in service
  checkAliveCounter();                                    // check server alive counter in service
  sendAliveMsg();                                         // send client alive in service
  checkWlanStatus();                                      // check WLAN status and signal strength
  setErrorLED(batteryError, rssiError, touchStateError, aliveCounterError);  // set the red error LED
  #ifdef DEBUG
    Serial.printf("doService(): end\n\n");
  #endif
}

void ICACHE_RAM_ATTR serviceIsr(void){                    // timer interrupt service routine
  doService();
}


void wlanInit(void){
  WiFi.config(clientIpAddr, gateway, subnet);             // set manual IP address if in Soft Access Point (SAP) mode
  #ifdef DEBUG
    Serial.println("\n\n");
    Serial.printf("wlanInit(): Connecting to %s ", SSID);
  #endif
  
  WiFi.begin(SSID, password);                             // connect to WLAN 
  while (WiFi.status() != WL_CONNECTED){                  // stay in this loop until a WLAN connection could be established
    digitalWrite(WLAN_LED, HIGH);                         // blink WLAN LED slowly to indicate connection try
    delay(SLOW_BLINK);
    digitalWrite(WLAN_LED, LOW);                          // blink WLAN LED slowly to indicate connection try
    delay(SLOW_BLINK);
    yield();
    #ifdef DEBUG
      Serial.print(".");
    #endif
  }//end while

  Udp.begin(clientUdpPort);                               // now listening for a server to send UDP messages

  #ifdef DEBUG
    Serial.println(" connected to WLAN network");
  #endif
  
  while(!wlan_complete){
    #ifdef DEBUG
      Serial.printf("wlanInit(): Sending hello message to IP %s, UDP port %d\n", serverIpAddr.toString().c_str(), serverUdpPort);
    #endif
    Udp.beginPacket(serverIpAddr, serverUdpPort);         // Udp.remoteIP() is not defind at this stage
    Udp.write(CLIENT_HELLO_MSG);                          // sending hello message to server
    Udp.endPacket();

    int packetSize = Udp.parsePacket();
    if (packetSize){                                      // listening for reply
      if (!strcmp(Udp.remoteIP().toString().c_str(), serverIpAddr.toString().c_str())){ //check server address
        #ifdef DEBUG
          Serial.printf("wlanInit(): UDP packet received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
        #endif
        int len = Udp.read(packetBuffer, UDP_PACKET_MAX_SIZE);
        if (len > 0)
          packetBuffer[len] = '\0';
        if(!strcmp(packetBuffer, SERVER_HELLO_MSG)){
          #ifdef DEBUG
            Serial.println("initWlan(): Detected server hello message during initWlan(), send a reply");
          #endif
          Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
          Udp.write(CLIENT_REPLY_MSG);
          Udp.endPacket();
        }
        wlan_complete = true;                             // WLAN connection is complete
        digitalWrite(WLAN_LED, HIGH);                     // switch off WLAN LED to indicate that WLAN connection is complete (no more blinking)
        #ifdef DEBUG
          Serial.print("wlanInit(): WLAN is complete\n");
        #endif 
      }else{                                              // server address is unexpected
        #ifdef DEBUG
          Serial.printf("wlanInit(): Received UDP packet from unexpected IP: %s, port %d\n",  Udp.remoteIP().toString().c_str(), Udp.remotePort());
        #endif
        wlan_complete = false;                            // wrong WLAN server address
      }
    }else{                                                // no UDP packet received yet
      #ifdef DEBUG
        Serial.printf("wlanInit(): No respond from server yet\n");
      #endif
      digitalWrite(WLAN_LED, HIGH);                       // toggle WLAN LED quickly to indicate connection try
      delay(FAST_BLINK);
      digitalWrite(WLAN_LED, LOW);                        // toggle WLAN LED quickly to indicate connection try
      delay(FAST_BLINK);
      yield();
    } //end if (packetSize)
  }//end while(!wlan_complete)
}//end void WlanInit()


void setup(void) {
  #ifdef DEBUG
    Serial.begin(115200);                                 // Setup Serial Interface with baud rate
  #endif

  //initialize digital output pins
  pinMode(POWER_LED,         OUTPUT);                     // LED to show power is on
  pinMode(WLAN_LED,          OUTPUT);                     // LED to show WLAN connection state
  pinMode(CLIENT_ERROR_OUT,  OUTPUT);                     // Error LED
  pinMode(TOUCH_LED,         OUTPUT);                     // Touch LED visual output
  pinMode(SLEEP_OUT,         OUTPUT);                     // control external sleep hardware
  
  //initialize digital input pins and isr
  pinMode(TOUCH_IN, INPUT_PULLUP);                        // set DIO to input with pullup
  
  //Setup timer interrup for service routines
  timer1_attachInterrupt(serviceIsr);
  timer1_enable(TIM_DIV256, TIM_EDGE, TIM_LOOP);          // DIV256 means one tick is 3.2us, total intervall time is 3.2us x SERVICE_INTERVALL, max is ~27 sec
  timer1_write(SERVICE_INTERVALL);                        // has to start early, to allow the client to go to sleep, if the server is dead

  //write default values of digital outputs
  digitalWrite(POWER_LED,         LOW);                   // switch on power LED
  digitalWrite(WLAN_LED,         HIGH);                   // switch off WLAN connection LED
  digitalWrite(CLIENT_ERROR_OUT, HIGH);                   // switch off error LED
  digitalWrite(TOUCH_LED,        HIGH);                   // switch off touch LED
  digitalWrite(SLEEP_OUT,         LOW);                   // switch on output to prevent external sleep hardware to send cpu to sleep

  // Init wlan communication with server
  wlanInit();
} //end setup()



void loop(void){
  //reconnect to server if connection got lost
   if (wlan_complete == false){
    #ifdef DEBUG
      Serial.printf("loop(): Lost connection to server, need to re-init wlan connection\n");
    #endif
    wlanInit();
   }
  
  // do pin polling instead of interrupt, check for state changes high->low or low->high
  if ((digitalRead(TOUCH_IN) == HIGH) && (touch_state == LOW)){
    doSensorHigh();
    delayMicroseconds(TOUCH_PIN_DEBOUNCE);                  // pauses for debouncing the touch input pin
  }
  
  if ((digitalRead(TOUCH_IN) == LOW) && (touch_state == HIGH)){
    doSensorLow();
    delayMicroseconds(TOUCH_PIN_DEBOUNCE);                  // pauses for debouncing the touch input pin
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
          Serial.printf("loop(): Acknowledge timeout error for LOW message, trying to re-transmit\n");
        #endif
        doSensorLow();                                    // server did not answer to low message from client, re-transmit
        ackCounterLow   = 0;                              // reset counter for re-transmitting low message
        touchStateError = true;                           // indicate error by red LED
     }
   }

   //Re-send HIGH messages to server if server does not reply within time out (avoid CNC crash during sensing)
   if (ackCounterHigh > SERVER_AQUN_CNT_MAX){
     if(high_command_acknowledge == false){
        #ifdef DEBUG
          Serial.printf("loop(): Acknowledge timeout error for HIGH message, trying to re-transmit\n");
        #endif
        doSensorHigh();                                   // server did not answer to low message from client, re-transmit
        ackCounterHigh  = 0;                              // reset counter for re-transmitting low message
        touchStateError = true;                           // indicate error by red LED
     }
   }
  
  // check for UDP commands from server
  int  packetSize = Udp.parsePacket();

  if (packetSize){  // received UDP package
    int len = Udp.read(packetBuffer, UDP_PACKET_MAX_SIZE);
    if (len > 0)
      packetBuffer[len] = '\0';

    // receiving sleep message from server
    if (!strcmp(packetBuffer, SERVER_SLEEP_MSG)){
      #ifdef DEBUG
        Serial.println("loop(): Detected sleep command\nGoing to deep sleep...");
      #endif
      digitalWrite(SLEEP_OUT, HIGH);                 // switch off external power
    }

    // receiving alive message from server and resetting the alive counter
    if(!strcmp(packetBuffer, SERVER_ALIVE_MSG)){
      #ifdef DEBUG
        Serial.println("loop(): Detected server alive command\n");
      #endif
      server_alive_cnt = 0; //reset alive counter for server
    }

    // receiving hello message from server
    if(!strcmp(packetBuffer, SERVER_HELLO_MSG)){
      #ifdef DEBUG
        Serial.println("loop(): Detected server hello command, send a reply");
      #endif
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(CLIENT_REPLY_MSG);
      Udp.endPacket();
    }
    
    // receiving reply message from server and do nothing
    if(!strcmp(packetBuffer, SERVER_REPLY_MSG)){
      #ifdef DEBUG
        Serial.println("loop(): Detected server reply message.");
      #endif
    }

    // receiving HIGH acknowledge message from server
    if(!strcmp(packetBuffer, CLIENT_TOUCH_HIGH_MSG)){
      #ifdef DEBUG
        Serial.println("loop(): Detected HIGH acknowledge message from server.");
      #endif
      high_command_acknowledge = true;
      ackCounterHigh           = 0;                       // reset counter, when receiving the server acknowledge message
      #ifdef CYCLETIME
        eTimeHigh = asm_ccount();             //take end time for client to server to client cycle time measurement
        #ifdef DEBUG
          Serial.printf("loop(): Measured UDP cycle time for sensor HIGH activation: %u ticks\n", ((uint32_t)(eTimeHigh - bTimeHigh)));
        #endif
        //send cycles to server e.g. to be displayed by the webserver
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        String cycle_msg = CLIENT_CYCLE_MSG + String((uint32_t)(eTimeHigh - bTimeHigh)); // pack end-time minus begin-time into a message
        Udp.write(cycle_msg.c_str());
        Udp.endPacket();
      #endif
    }

       //receiving LOW acknowledge message from server
       if(!strcmp(packetBuffer, CLIENT_TOUCH_LOW_MSG)){
       #ifdef DEBUG
         Serial.println("loop(): Detected LOW acknowledge message from server.");
       #endif
       low_command_acknowledge = true;
       ackCounterLow           = 0;
       #ifdef CYCLETIME
          eTimeLow = asm_ccount();             //take end time for client to server to client cycle time measurement
        #ifdef DEBUG
          Serial.printf("loop(): Measured UDP cycle time for sensor LOW activation: %u ticks\n", ((uint32_t)(eTimeLow - bTimeLow)));
        #endif
        //send cycles to server e.g. to be displayed by the webserver
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        String cycle_msg = CLIENT_CYCLE_MSG + String((uint32_t)(eTimeLow - bTimeLow)); // pack end-time minus begin-time into a message
        Udp.write(cycle_msg.c_str());
        Udp.endPacket();
      #endif
    }
    /*
    //receiving unknown message from server
    #ifdef DEBUG
      Serial.println("loop(): Detected unknown command");
    #endif
    */
   
  }//if (packetSize)
} //end loop()

