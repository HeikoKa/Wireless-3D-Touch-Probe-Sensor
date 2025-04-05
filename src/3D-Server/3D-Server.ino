 /*  3D Sensor Server/Basestation
  *   
  *  @author Heiko Kalte  
  *  @date 05.04.2025 
  *  Copyright by Heiko Kalte (h.kalte@gmx.de)
  */

//TODO
// * are round trip tick measurement for ESP32 the same as for ESP8266?
// * are there any mem leaks?
// * reset client info function bei sleep
// * transmitCounter is not used 

// ***************************************************
// Use Arduino IDE with board package ESP32-WROOM-DA
// ***************************************************
  
char *serverSwVersion = "3.00.01";
#include "Z:\Projekte\Mill\HeikosMill\3D Taster\Arduino\GIT\3D-Touch-Sensor\src\3D-Header.h"

#include <stdio.h>
#include <stdlib.h>
#ifdef SERVER_ESP32
    #include <WiFi.h>
  #else //ESP8266
    #include <ESP8266WiFi.h>
#endif
#include <WiFiUdp.h>
#include <Ticker.h>

#ifdef WEBSERVER
  #ifdef SERVER_ESP32
    #include <WebServer.h>
  #else //ESP8266
    #include <ESP8266WebServer.h>
  #endif
  #include <WiFiClient.h>
#endif

using namespace CncSensor;

char*         errCheck;
WiFiUDP       Udp;                                              // UDP object
int           rssi;                                             // Wifi signal strength
char          packetBuffer[UDP_PACKET_MAX_SIZE];                // buffer for in/outcoming packets
int           clientBatteryStatus        = CLIENT_BAT_OK;       // init with battery is ok
float         clientBateryVoltage        = 0.0;                 // client voltage init value
bool          wlan_complete              = false;               // global variable to indicate if wlan connection is established completely 
bool          msg_lost                   = false;               // did a UDP message got lost
int           client_alive_counter       = 0;                   // current client alive counter
int           serverHwPcbRevision        = 0;                   // From baestation/server PCB revision 3 onwards the revision can be read out from the PCB coded by 3 input bit
uint32_t      transmitCounter            = 0;                   // counter for identify each sended Wifi message, is incremented with every UDP message
volatile bool serviceRequest             = false;               // interrupt service can request a service intervall by this flag

struct clientStateType{
    String    micro       = "";                                 // micro controller type of the client
    String    debug       = "";                                 // was debug enabled during client compilation
    String    cycle       = "";                                 // was cycletime measurement enabled durcing client compilation
    String    swVersion   = "";                                 // software version of the client
    String    hwVersion   = "";                                 // PCB or hardware version of the client
    String    charging    = "";                                 // global variable to indicate if battery is charging (not used at the moment)
    }; 
clientStateType clientStates;

#ifdef SERVER_ESP32
  hw_timer_t *serviceTimer            = NULL;
  portMUX_TYPE timerMux               = portMUX_INITIALIZER_UNLOCKED;
#endif

#ifdef CYCLETIME
  // following code is for measuring the cycle time for UDP messages from server to client to server
  uint32_t  ticks;                                          // ticks from client for delay measurement
  int       ticksArray[SERVER_TICKS_ARRAY_SIZE]={0};        // array of ticks from client delay measurement
  char      ticksString[50*SERVER_TICKS_ARRAY_SIZE];        // string of array tick 
  int       ticks_pointer    = 0;
  uint32_t  totalNumberTicks = 0;


  static inline void setNewTicks(uint32_t ticks){
    if (ticks_pointer > 9)
      ticks_pointer = 0;

    ticksArray[ticks_pointer] = ticks;
    ticks_pointer++;
    totalNumberTicks++;
  } // end setNewTicks();


  static inline uint32_t getLatestTicks(){
    return ticksArray[ticks_pointer];
  } // end getLatestTicks()


  static inline char* getAllTicks(){
    int i;
    float average = 0;
    char dstring[50];

    strcpy(ticksString, "<p>Client Ticks measurement:</p>");
    strcat(ticksString, "\n<p>Total number of measured ticks: ");
    sprintf(dstring, "%d", totalNumberTicks);
    strcat(ticksString, dstring);
    strcat(ticksString, "</p>");                      

    for (i= 0; i <= SERVER_TICKS_ARRAY_SIZE-1; i++){
      average = average + (float) ticksArray[i];          // sum up ticks for average calculationte average
      sprintf(dstring, "%d: %d, %.2fms", i, ticksArray[i], ((float)ticksArray[i] * NANOSEC_PER_TICK / 1000000)); 
      strcat(ticksString, "<p>");                         // concatenate message string for webserver
      strcat(ticksString, dstring);                       // concatenate message string for webserver
      strcat(ticksString, "</p>");                        // concatenate message string for webserver
    } // end for loop
    
    strcat(ticksString, "\n<p>Average ticks/ms: ");
    if (totalNumberTicks < SERVER_TICKS_ARRAY_SIZE)       // check if tick array is already full 
      average = average / totalNumberTicks;               // calculate final average of the ticks measurements
    else
      average = average / SERVER_TICKS_ARRAY_SIZE;        // calculate final average of the ticks measurements
    sprintf(dstring, "%.2f, %.2fms", average, (average * NANOSEC_PER_TICK / 1000000)); 
    strcat(ticksString, dstring);
    strcat(ticksString, "</p>");
    
    #ifdef DEBUG
      Serial.printf("getAllTicks(): %s\n", ticksString);
    #endif
    return ticksString;
  } // end getAllTicks()
#endif //CYCLETIME

#ifdef WEBSERVER
    #ifdef SERVER_ESP32
      WebServer server(80);
    #else //ESP8266
      ESP8266WebServer server(80);
    #endif
    // root web page of the server
    void handleRoot() {
        String message = "<h1>Webserver 3D Touch Probe</h1>";
        message += "<h3>by Heiko Kalte (h.kalte@gmx.de)</h3>";
        message += "<p> Server SW version: ";
        message += serverSwVersion;
        message += "</p>";
        message += "<p> Server hardware PCB version: ";
        message += serverHwPcbRevision;
        message += "</p>";
        message += "<p>Server IP: ";
        message += serverIpAddr.toString().c_str();
        message += ", Port: ";
        message += serverUdpPort;
        message += "</p>";
        message += "<p>Client IP: ";
        message += clientIpAddr.toString().c_str();
        message += ", Port: ";
        message += clientUdpPort;
        message += "</p>";
        message += "<p>Number of connected clients: ";
        message += WiFi.softAPgetStationNum();
        message += "</p>";
        #ifdef SERVER_ESP32
          message += "<p>Server ESP32: yes</p>";
        #else
          message += "<p>Server ESP32: no</p>";
        #endif
        #ifdef DEBUG
          message += "<p>Server DEBUG: yes</p>";
        #else
          message += "<p>Server DEBUG: no</p>";
        #endif
        #ifdef CYCLETIME
          message += "<p>Server CYCLETIME: yes</p>";
        #else
          message += "<p>Server CYCLETIME: no</p>";
        #endif
        message += "<p>Client Infos: </p>";
        message += "<p> ";
        message += clientStates.micro;
        message += "</p>";
        message += "<p> ";
        message += clientStates.debug;
        message += "</p>";
        message += "<p> ";
        message += clientStates.cycle;
        message += "</p>";
        message += "<p> ";
        message += clientStates.swVersion;
        message += "</p>";
        message += "<p> ";
        message += clientStates.hwVersion;
        message += "</p>";
        message += "<p> ";
        message += clientStates.charging;
        message += "</p>";
     
        if (client_alive_counter < CLIENT_ALIVE_CNT_MAX)
          message += "<p>Client alive: yes</p>";
        else
          message += "<p>Client alive: no</p>";

        if (msg_lost == true)
          message += "<p>Lost UDP message: true</p>";
        else
          message += "<p>Lost UDP message: false</p>";

        switch (clientBatteryStatus){
            case CLIENT_BAT_OK:
                message += "<p>Client battery status: ok</p>";
                break;
            case CLIENT_BAT_LOW:
                message += "<p>Client battery status: low</p>";
                break;
            case CLIENT_BAT_CRITICAL:
                message += "<p>Client battery status: critical</p>";
                break;
            default:
                message += "<p>Client battery status: undefined</p>";
                break;
        }
        
        message += "<p>Client battery voltage: ";
        message += clientBateryVoltage;
        message += "</p>";

        message += "<p>Client WIFI strength: ";
        message += rssi;
        message += "</p>";
        
        #ifdef CYCLETIME
          message += getAllTicks();
        #endif

        server.send(200, "text/html", message);
    }

    // web side requests that are different from root
    void handleNotFound(){
        server.send(404, "text/plain", "404: Not found");       // Send HTTP status 404 (Not Found) when there's no handler for the URI in the request
    }
#endif  //#ifdef WEBSERVER


static inline void sendWifiMessage(String msg){  
  transmitCounter++;                                            // increment message identifier each time sendWifiMessage() is called
  //msg = msg + "_" + String(transmitCounter); 
  Udp.beginPacket(clientIpAddr, clientUdpPort);                 // send UDP packet to server to indicate the 3D touch change 
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
} //sendWifiMessage()


static inline void sendAliveMsg(){
  sendWifiMessage(SERVER_ALIVE_MSG);
  #ifdef DEBUG
    Serial.printf("sendAliveMsg(): Sending server alive message to client IP: %s and Port: %d\n", clientIpAddr.toString().c_str(), clientUdpPort);
  #endif
} // end sendAliveMsg()


static inline void checkClientStatus(){
  if(wlan_complete){  //if WLAN connection is established, check status of client
    // check if client is alive
    if (client_alive_counter >= CLIENT_ALIVE_CNT_MAX){
      #ifdef DEBUG
         Serial.printf("checkClientStatus(): Client alive messages are missing\n");
      #endif 
      wlan_complete = false;  // set wlan status to false, in loop function a new wlan init will be triggert
    }else{
      client_alive_counter++;  // increment client alive counter every service call, will be set to 0 when receiving alive msg
      #ifdef DEBUG
         Serial.printf("checkClientStatus(): Incrementing client_alive_counter=%d max=%d\n", client_alive_counter, CLIENT_ALIVE_CNT_MAX);
      #endif 
    }

    // check if client battery low
    if (clientBatteryStatus == CLIENT_BAT_LOW){
      #ifdef DEBUG
         Serial.printf("checkClientStatus(): Client battery is low\n");
      #endif
      digitalWrite(SERVER_BAT_ALM_LED, LOW);
      #ifdef SERVER_HW_REVISION_3_0
        digitalWrite(SERVER_BAT_ALM_OUT, HIGH != SERVER_BAT_ALM_OUT_POLARITY);  // indicated battery low to CNC controller 
      #endif
    }

    // check if client battery critical
    if (clientBatteryStatus == CLIENT_BAT_CRITICAL){
      #ifdef DEBUG
        Serial.printf("checkClientStatus(): Client battery is critical\n");
      #endif
      digitalWrite(SERVER_BAT_ALM_LED, LOW);
      #ifdef SERVER_HW_REVISION_3_0
        digitalWrite(SERVER_BAT_ALM_OUT, HIGH != SERVER_BAT_ALM_OUT_POLARITY);  // indicated critical bat to CNC controller
      #endif
    }

    // check if client battery ok
    if (clientBatteryStatus == CLIENT_BAT_OK){
      #ifdef DEBUG
        Serial.printf("checkClientStatus(): Client battery is ok\n");
      #endif
      digitalWrite(SERVER_BAT_ALM_LED, HIGH);
      #ifdef SERVER_HW_REVISION_3_0
        digitalWrite(SERVER_BAT_ALM_OUT, LOW != SERVER_BAT_ALM_OUT_POLARITY);  // indicated good battery to CNC controller 
      #endif
    }  
  }else{   //if(wlan_complete)
    #ifdef DEBUG
      Serial.printf("checkClientStatus(): No battery check done, because WLAN is not established\n");
    #endif
  } //end if(wlan_complete)
} //end checkClientStatus()


static inline void doService(){
  #ifdef DEBUG
    Serial.printf("\ndoService(): ***** start service *****\n");
  #endif
  sendAliveMsg();                                       // send regular alive messages to client
  checkClientStatus();                                  // check regularly client status
  digitalWrite(SERVER_SLEEP_LED, HIGH);                 // if server requested client to go to sleep, switch off Sleep LED with this service 
  #ifdef DEBUG
    Serial.printf("doService(): ***** end service *****\n\n");
  #endif
  #ifdef SERVER_ESP32
    portENTER_CRITICAL_ISR(&timerMux);                      // protect acces to "serviceRequest"
  #endif
  serviceRequest = false;                                   // reset service flag
  #ifdef SERVER_ESP32
    portEXIT_CRITICAL_ISR(&timerMux);                       // release protection of "serviceRequest"
  #endif
}


//void ICACHE_RAM_ATTR serviceIsr(){                        // timer interrupt for regular service
void IRAM_ATTR  serviceIsr(){                               // timer interrupt for regular service
  #ifdef SERVER_ESP32
    portENTER_CRITICAL_ISR(&timerMux);                      // protect access to serviceRequest 
  #endif
  serviceRequest = true;                                    // exit isr as quickly as possible and start service routine in main loop
  #ifdef SERVER_ESP32
    portEXIT_CRITICAL_ISR(&timerMux);                       // release protection of data
  #endif
}


void wlanInit(){
  digitalWrite(SERVER_BAT_ALM_LED, HIGH);                   // reset battery alarm for client if client is no connected yet or not connected anymore
  #ifdef SERVER_HW_REVISION_3_0
    digitalWrite(SERVER_BAT_ALM_OUT, LOW != SERVER_BAT_ALM_OUT_POLARITY);
  #endif

  #ifdef DEBUG
    Serial.printf("\nwlanInit(): Soft-AP server IP: ");
    serverIpAddr = WiFi.softAPIP();                       // the server IP is set earlier, so WIFI.softAPIP() should not result in a different IP
    Serial.println(serverIpAddr);
  #endif
  
  //Waiting for client to connect to the soft AP
  #ifdef DEBUG
    Serial.print("wlanInit(): Waiting for clients to connect");
  #endif
  
  while(WiFi.softAPgetStationNum() < 1){                              // loop until the first client connects, can be a false client. Better check for the right IP address of the connected client
    #ifdef DEBUG
      Serial.print(".");
    #endif

    #ifdef SERVER_HW_REVISION_3_0
      digitalWrite(SERVER_WLAN_OUT, LOW != SERVER_WLAN_OUT_POLARITY);  // WLAN is not establised is indicated to CNC controller
      digitalWrite(SERVER_WLAN_LED, HIGH);                             // Switch off WLAN connection LED
      delay(SLOW_BLINK);                                               // Blink WLAN LED, but keep WLAN ouput to cnc controller false
      digitalWrite(SERVER_WLAN_LED, LOW);                              // Switch on WLAN connection LED
      delay(SLOW_BLINK);                                               // Blink WLAN LED, but keep WLAN ouput to cnc controller false
    #else
      //do not blink LED if LED ouput is also used as ouput for the cnc controller (PCB revision <3.0) to avoid false detection by CNC Controller
      digitalWrite(SERVER_WLAN_LED, HIGH);                             // switch off WLAN connection LED
      delay(SLOW_BLINK);                                               // even if output is not blinking, wait a short time to prevent 
    #endif
    yield();
    if (serviceRequest == true)        //do service routine if isr has set the service flag
      doService();
  } //while(WiFi.softAPgetStationNum() < 1)

  #ifdef DEBUG
    Serial.println("Ready");
    Serial.printf("wlanInit(): Clients connected: %d\n", WiFi.softAPgetStationNum());
  #endif

  // A client is connected at this point, now check by UDP messages, if it is the right client
  Udp.begin(serverUdpPort);                               //setup UDP receiver
  while(!wlan_complete){
    //send hello message to the connected client
    #ifdef DEBUG
      Serial.printf("wlanInit(): Sending Hello message to client IP: ");
      Serial.printf(clientIpAddr.toString().c_str());
      Serial.printf(" and Port: ");
      Serial.printf("%d\n", clientUdpPort);
    #endif
    sendWifiMessage(SERVER_HELLO_MSG);
    // listen for UDP respond from client
    int packetSize = Udp.parsePacket();
    if (packetSize){
      // udp packet received
      int len = Udp.read(packetBuffer, UDP_PACKET_MAX_SIZE);
      if (len > 0)
          packetBuffer[len] = '\0';
      #ifdef DEBUG
        Serial.printf("wlanInit(): UDP packet received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
      #endif
      // if receive hello from then reply otherwise do nothing
      if (!strcmp (packetBuffer, CLIENT_HELLO_MSG)){
        #ifdef DEBUG
          Serial.printf("wlanInit(): Detected client hello command during wlanInit, send a reply\n\n");
        #endif
        sendWifiMessage(SERVER_REPLY_MSG);      
      }  // if(!strcmp (packetBuffer, CLIENT_HELLO_MSG))
      // at this point a client is connected and UDP messages have been exchanged
      wlan_complete = true;                                     // WLAN connection is complete, stop listening for further incoming UPD packages
      digitalWrite(SERVER_WLAN_LED, LOW);                       // switch on WLAN connection LED to indicate sucessfull connection
      #ifdef SERVER_HW_REVISION_3_0
        digitalWrite(SERVER_WLAN_OUT, HIGH != SERVER_WLAN_OUT_POLARITY);  // successful WLAN establishing is indicated to CNC controller 
      #endif
      
      #ifdef DEBUG
        Serial.print("wlanInit(): WLAN is complete\n");
      #endif
    }else{ //if(packetSize)
      #ifdef DEBUG
        Serial.printf("wlanInit(): No respond from client yet\n");
      #endif

      #ifdef SERVER_HW_REVISION_3_0
        digitalWrite(SERVER_WLAN_OUT, LOW != SERVER_WLAN_OUT_POLARITY);  // indicate no successfull WLAN/UDP connection to CNC controller 
        digitalWrite(SERVER_WLAN_LED, HIGH);                             // switch off WLAN LED 
        delay(FAST_BLINK);                                               // make a small pause
        digitalWrite(SERVER_WLAN_LED, LOW);                              // switch on WLAN LED 
        delay(FAST_BLINK);                                               // make a small pause
      #else // PCB revision <3.0
        digitalWrite(SERVER_WLAN_LED, HIGH);                             // still switch off WLAN connection LED for PCB <3.0 to avoid false detection by CNC controller
        delay(FAST_BLINK);                                               // make a small pause
      #endif
      yield(); 
    } //end if(packetSize)

    #ifdef WEBSERVER                                                     //handle client access if webserver is enabled
      server.handleClient();                                             // need to be called even before loop(), to allow webbrowser to connect even if the 3D touch client is not connected
    #endif
    
    if (serviceRequest == true)        //do service routine if isr has set the service flag
      doService();
  } //while(!wlan_complete)
}//void wlanInit()


void setup(){
  #ifdef DEBUG
    Serial.begin(BAUD_RATE);                                     // Setup Serial Interface with baud rate
    Serial.println("\n\nsetup(): I am the 3D Touch Probe Sensor Server/Basestation");
    Serial.println("setup(): Copyright by Heiko Kalte 2025 (h.kalte@gmx.de)");
    Serial.printf("setup(): My software version is: %s", serverSwVersion);
    Serial.println();
  #endif

  //initialize digital input/output pins
  pinMode(SERVER_POWER_LED,      OUTPUT);                        // LED to show power is on
  pinMode(SERVER_WLAN_LED,       OUTPUT);                        // LED to show WLAN connection state
  pinMode(SERVER_TOUCH_LED,      OUTPUT);                        // Touch Output indicated the client has detected something
  pinMode(SERVER_ERROR_LED,      OUTPUT);                        // Error Output, something is wrong, CNC should stop
  pinMode(SERVER_BAT_ALM_LED,    OUTPUT);                        // Battery Alarm output to indicate to CNC input 
  pinMode(SERVER_SLEEP_LED,      OUTPUT);                        // Send Client to sleep LED
  pinMode(SERVER_SLEEP_IN, INPUT_PULLUP);                        // Send Client to sleep external input
  
  #ifdef SERVER_HW_REVISION_3_0
    pinMode(SERVER_HW_REVISION_0, INPUT);                        // server/basestation hardware PCB revisio bit 0 (only for PCB revision 3.0 or later)
    pinMode(SERVER_HW_REVISION_1, INPUT);                        // server/basestation hardware PCB revisio bit 1 (only for PCB revision 3.0 or later)
    pinMode(SERVER_HW_REVISION_2, INPUT);                        // server/basestation hardware PCB revisio bit 2 (only for PCB revision 3.0 or later)
    serverHwPcbRevision = (digitalRead(SERVER_HW_REVISION_2) << 2) + (digitalRead(SERVER_HW_REVISION_1)<< 1) + digitalRead(SERVER_HW_REVISION_0); //construct version number from 3 input bit
    #ifdef DEBUG
      Serial.printf("setup(): My hardware/PCB version is: %d", serverHwPcbRevision);
      Serial.println();
    #endif
  #endif

  #ifdef SERVER_HW_REVISION_3_0
  //hardware version 3.0 utilizes two outputs one to control the LED and one to control the output to the CNC controller
    pinMode(SERVER_WLAN_OUT,      OUTPUT);                        // Separt WLAN established Output for CNC controller 
    pinMode(SERVER_TOUCH_OUT,     OUTPUT);                        // Separt Touch output for CNC controller
    pinMode(SERVER_ERROR_OUT,     OUTPUT);                        // Separt Error output for CNC controller
    pinMode(SERVER_BAT_ALM_OUT,   OUTPUT);                        // Separt Battery alarm output for CNC controller
  #endif

  //Initialize timer interrup for battery control
  #ifdef SERVER_ESP32
    serviceTimer = timerBegin(1000000);                          // Set timer frequency to 1MHz
    timerAttachInterrupt(serviceTimer, &serviceIsr);             // Attach onTimer function to our timer. 
    timerAlarm(serviceTimer, SERVICE_INTERVALL_ESP32, true, 0);  // Set alarm to call onTimer function every second (value in microseconds). Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
    timerStart(serviceTimer);
  #else //ESP8266
    timer1_attachInterrupt(serviceIsr);
    timer1_enable(TIM_DIV256, TIM_EDGE, TIM_LOOP);               // DIV256 means one tick is 3.2us, total intervall time is 3.2us x SERVICE_INTERVALL, max is ~27 sec
    timer1_write(SERVICE_INTERVALL);                             // Setup serice intervall
  #endif
  
  //write default values of digital outputs for LEDs
  digitalWrite(SERVER_POWER_LED,   LOW);                         // switch on power LED
  digitalWrite(SERVER_WLAN_LED,    HIGH);                        // switch off WLAN connection LED
  digitalWrite(SERVER_TOUCH_LED,   HIGH);                        // switch off touch LED
  digitalWrite(SERVER_ERROR_LED,   HIGH);                        // switch off general error LED
  digitalWrite(SERVER_BAT_ALM_LED, HIGH);                        // switch off battery error/low LED
  digitalWrite(SERVER_SLEEP_LED,   HIGH);                        // switch off sleep request LED
  
  #ifdef SERVER_HW_REVISION_3_0
    //write default values of digital outputs to CNC controller. From hardware version 3.0 two outputs are utilized, one to control the LED and one to control the output to the CNC controller
    digitalWrite(SERVER_WLAN_OUT,    LOW != SERVER_WLAN_OUT_POLARITY);    // Default value for WLAN output, considering the polarity
    digitalWrite(SERVER_TOUCH_OUT,   LOW != SERVER_TOUCH_OUT_POLARITY);   // Default value for Touch output, considering the polarity
    digitalWrite(SERVER_ERROR_OUT,   LOW != SERVER_ERROR_OUT_POLARITY);   // Default value for Error output, considering the polarity
    digitalWrite(SERVER_BAT_ALM_OUT, LOW != SERVER_BAT_ALM_OUT_POLARITY); // Default value for Battery alarm output, considering the polarity
  #endif

  //Setup Soft-AP
  #ifdef DEBUG
      Serial.print("setup(): Setting Soft-AP...");
  #endif
  WiFi.softAPConfig(serverIpAddr, gateway, subnet);
  boolean result = WiFi.softAP(SSID, password);
  if(result == true){
    #ifdef DEBUG
      Serial.println("Ready");
    #endif

    #ifdef WEBSERVER
      server.on("/", handleRoot);
      server.onNotFound(handleNotFound);                  // When a client requests an unknown URI (i.e. something other than "/"), call function "handleNotFound"
      server.begin();
      #ifdef DEBUG
        Serial.println("setup(): HTTP Webserver started");
      #endif
    #endif

    wlanInit();
  }else{ // Soft-AP setup went wrong
    #ifdef DEBUG
      Serial.println("Failed!");
    #endif
    digitalWrite(SERVER_ERROR_LED, LOW);                                  //switch on error LED
    #ifdef SERVER_HW_REVISION_3_0
      digitalWrite(SERVER_ERROR_OUT, HIGH != SERVER_ERROR_OUT_POLARITY);  // Set Error output to indicated to the CNC controller 
    #endif
  } // end if(result == true)
} //end setup() 



void loop(){
    
  #ifdef WEBSERVER
    server.handleClient();  ////handle client access if webserver is enabled
  #endif
    
  // check if WLAN client got lost. Better check would be compare the client IP address to the expected client IP address, other webbrowsers connections are clients as well
  if ((WiFi.softAPgetStationNum() < 1)||(wlan_complete == false)){
    wlan_complete = false;
    #ifdef DEBUG
      Serial.printf("loop(): Currently %d stations are connected and wlan_complete is %d\n", WiFi.softAPgetStationNum(), wlan_complete);
    #endif
    wlanInit();
  } //if (WiFi)

  // send SLEEP message to client if the server input pin is low, because CNC does not need the sensor in the next minutes, save battery. It is asumed to use an inverting transistor at the sleep input.
  if (((digitalRead(SERVER_SLEEP_IN) == HIGH)&&(SERVER_SLEEP_IN_POLARITY)) || ((digitalRead(SERVER_SLEEP_IN) == LOW)&&(!SERVER_SLEEP_IN_POLARITY))){
    #ifdef DEBUG
      Serial.printf("loop(): CNC controller wants to send the client asleep\n");
    #endif
    sendWifiMessage(SERVER_SLEEP_MSG);
    digitalWrite(SERVER_SLEEP_LED, LOW);                                              // indicate sleep request by LED
    //reset some client status infos after sending the client asleep
    wlan_complete = false;                                                            // set Wifi incomplete
    digitalWrite(SERVER_TOUCH_LED, HIGH);                                             // indicate current touch state by LED
    #ifdef SERVER_HW_REVISION_3_0
      digitalWrite(SERVER_TOUCH_OUT, HIGH != SERVER_TOUCH_OUT_POLARITY);              // indicate touch high to CNC controller 
    #endif
    clientBatteryStatus = CLIENT_BAT_OK;                                              // set back battery to ok, to switch off alarm led
    digitalWrite(SERVER_BAT_ALM_LED, HIGH);
    #ifdef SERVER_HW_REVISION_3_0
      digitalWrite(SERVER_BAT_ALM_OUT, LOW != SERVER_BAT_ALM_OUT_POLARITY);           // indicated good battery to CNC controller 
    #endif
  } // end if (digitalRead)

  // receive incoming UDP packets
  int packetSize = Udp.parsePacket();
  if (packetSize){
    int len = Udp.read(packetBuffer, UDP_PACKET_MAX_SIZE);
    if (len > 0)
      packetBuffer[len] = '\0';

    //received client sensor HIGH command
    if (!strncmp (packetBuffer, CLIENT_TOUCH_HIGH_MSG, strlen(CLIENT_TOUCH_HIGH_MSG))){
      digitalWrite(SERVER_TOUCH_LED, LOW);                                        // indicate current touch state by LED
      #ifdef SERVER_HW_REVISION_3_0
        digitalWrite(SERVER_TOUCH_OUT, HIGH != SERVER_TOUCH_OUT_POLARITY);        // indicate touch high to CNC controller 
      #endif
      char *highMsgAttach = packetBuffer + strlen(CLIENT_TOUCH_HIGH_MSG);         // cut/decode additional information out of the client message
      if (!strncmp (highMsgAttach, "1", strlen("1")))                             // decode the next 0 or 1 after the message text, this is an info about lost messages
        msg_lost = true;
      #ifdef DEBUG
        Serial.printf("loop(): Client touch is high. Sending back the status. Attachment is %s\n", highMsgAttach);
      #endif
      sendWifiMessage(packetBuffer);
      return;
    }

    //received client sensor LOW command
    if (!strncmp (packetBuffer, CLIENT_TOUCH_LOW_MSG, strlen(CLIENT_TOUCH_LOW_MSG))){
      digitalWrite(SERVER_TOUCH_LED, HIGH);
      #ifdef SERVER_HW_REVISION_3_0
        digitalWrite(SERVER_TOUCH_OUT, LOW != SERVER_TOUCH_OUT_POLARITY);  // indicate touch low to CNC controller 
      #endif
      char *lowMsgAttach = packetBuffer + strlen(CLIENT_TOUCH_LOW_MSG);    // cut/decode additional information out of the client message
      if (!strncmp (lowMsgAttach, "1", strlen("1")))                       // decode the next 0 or 1 after the message text, this is an info about lost messages
        msg_lost = true;
      #ifdef DEBUG
          Serial.printf("loop(): Client touch is low. Sending back the status. Attachment is %s\n", lowMsgAttach);
      #endif
      sendWifiMessage(packetBuffer);
      return;
    }

    //received client BATTERY low
    if (!strncmp (packetBuffer, CLIENT_BAT_LOW_MSG, strlen(CLIENT_BAT_LOW_MSG))){
      clientBatteryStatus= CLIENT_BAT_LOW;
      char *temp = strtok (packetBuffer,"_");                       // identifier CLIENT_BAT_LOW_MSG
      temp = strtok (NULL, "_");                                    // battery voltage
      clientBateryVoltage = strtof(temp, &errCheck);                // convert string to float
      #ifdef DEBUG
        Serial.printf("loop(): Received Client battery is low (%.2fV)\n\n", clientBateryVoltage);
      #endif
      return;
    }

    //received client BATTERY critical
    if (!strncmp (packetBuffer, CLIENT_BAT_CRITICAL_MSG, strlen(CLIENT_BAT_CRITICAL_MSG))){
      clientBatteryStatus = CLIENT_BAT_CRITICAL;
      char *temp = strtok (packetBuffer,"_");                       // identifier CLIENT_BAT_CRITICAL_MSG
      temp = strtok (NULL, "_");                                    // battery voltage
      clientBateryVoltage = strtof(temp, &errCheck);                // convert string to float
      #ifdef DEBUG
        Serial.printf("loop(): Received Client battery is critical (%.2fV)\n", clientBateryVoltage);
      #endif
      return;
    }

  
    //received client BATTERY is ok
    if (!strncmp (packetBuffer, CLIENT_BAT_OK_MSG, strlen(CLIENT_BAT_OK_MSG))){
      clientBatteryStatus= CLIENT_BAT_OK;
      char *temp = strtok (packetBuffer,"_");                       // identifier CLIENT_BAT_OK_MSG
      temp = strtok (NULL, "_");                                    // battery voltage
      clientBateryVoltage = strtof(temp, &errCheck);                // convert string to float
      #ifdef DEBUG
        Serial.printf("loop(): Received Client battery is ok message (%.2fV)\n", clientBateryVoltage);
      #endif
      return;
    }

    //received client is ALIVE
    if (!strncmp (packetBuffer, CLIENT_ALIVE_MSG, strlen(CLIENT_ALIVE_MSG))){
      client_alive_counter = 0;    //reset client alive counter when receiving alive message
      #ifdef DEBUG
        Serial.printf("loop(): Received -Client is alive- message\n");
      #endif
      return;
    }

    //received client HELLO message
    if (!strncmp (packetBuffer, CLIENT_HELLO_MSG, strlen(CLIENT_HELLO_MSG))){
      #ifdef DEBUG
        Serial.printf("loop(): Detected client hello command, send a hello reply\n\n");
      #endif
      sendWifiMessage(SERVER_REPLY_MSG);
      return;
    }

    //received client REPLY message
    if (!strncmp (packetBuffer, CLIENT_REPLY_MSG, strlen(CLIENT_REPLY_MSG))){
      #ifdef DEBUG
        Serial.printf("loop(): Detected client reply message\n");
      #endif
      return;
    }
          
    //received client measures LOOP CYCLE message
    if (!strncmp (packetBuffer, CLIENT_CYCLE_MSG, strlen(CLIENT_CYCLE_MSG))){     // trying to decod the CLIENT_CYCLE_MSG prefix from the msg
      char *temp = strtok (packetBuffer,"_");                                     // identifier CLIENT_CYCLE_MSG
      temp = strtok (NULL, "_");                                                  // decode cycle time 
      //char *temp = packetBuffer + strlen(CLIENT_CYCLE_MSG);                     // cut/decode the raw number of ticks out of the client message
      #ifdef CYCLETIME
          ticks = atoi(temp);                                                      // convert string of ticks to integer of ticks
          setNewTicks(ticks);                                                      // safe the current ticks value in an array
        #ifdef DEBUG
            Serial.printf("loop(): Received Cycle measurement message from client: %d ticks\n", ticks);
        #endif
      #endif
      return;
    }

    //received RSSI 
    if (!strncmp (packetBuffer, CLIENT_RSSI_MSG, strlen(CLIENT_RSSI_MSG))){     // trying to decod the CLIENT_RSSI_MSG prefix from the msg
      //char *temp = packetBuffer + strlen(CLIENT_RSSI_MSG);                    // cut/decode the raw number of ticks out of the client message
      char *temp = strtok (packetBuffer,"_");                                   // cut  CLIENT_RSSI_MSG
      temp = strtok (NULL, "_");                                                // decode RSSI value 
      rssi = atoi(temp);
      #ifdef DEBUG
          Serial.printf("loop(): Received RSSI message from client: %d\n", rssi);
      #endif
      return;
    }

    //received CLIENT INFO message, several information about the client paramters and version
    if (!strncmp (packetBuffer, CLIENT_INFO_MSG, strlen(CLIENT_INFO_MSG))){    // decode the CLIENT_INFO_MSG prefix from the msg
      char *temp             = strtok (packetBuffer,"_");                      // identifier CLIENT_INFO_MSG
      clientStates.micro     = strtok (NULL, "_");                             // decode Microprocessor Type
      clientStates.debug     = strtok (NULL, "_");                             // decode DEBUG 
      clientStates.cycle     = strtok (NULL, "_");                             // decode CYCLETIME 
      clientStates.swVersion = strtok (NULL, "_");                             // decode Software Version 
      clientStates.hwVersion = strtok (NULL, "_");                             // decode Hardware Version 
     #ifdef DEBUG
          Serial.printf("loop(): Received client info message: CPU: %s, Debug: %s, Cycle: %s, SW rev: %s and HW rev: %s\n", clientStates.micro, clientStates.debug, clientStates.cycle, clientStates.swVersion, clientStates.hwVersion);
      #endif
      return;
    }

    //receiving unknown UDP message
    #ifdef DEBUG
      Serial.println("loop(): Received unknown command by UDP");
      Serial.println(packetBuffer);
    #endif
  } //if (packetSize)

  if (serviceRequest == true)        //do service routine if isr has set the service flag
    doService(); 
}//end loop()