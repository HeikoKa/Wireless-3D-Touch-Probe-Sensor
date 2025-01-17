 /*  Server
  *   
  *  @author Heiko Kalte  
  *  @date 12.01.2025 
  */

char *serverVersion = "V02.01";
#include "Z:\Projekte\Mill\HeikosMill\3D Taster\Arduino\GIT\3D-Touch-Sensor\src\3D-Header.h"
#include <stdio.h>
#include <stdlib.h>
#include <Adafruit_NeoPixel.h>
//#include <WS2812FX.h>
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

//TODOs
// check ESP32 interrupt handling
// are round trip tick measurement for ESP32 the same time?
// beide rote LEDs


int           clientBatteryStatus        = CLIENT_BAT_OK;       // init with battery is ok
float         clientBateryVoltage        = 0.0;                 // client voltage
bool          wlan_complete              = false;               // global variable to indicate if wlan connection is established completely 
bool          msg_lost                   = false;               // did a UDP message got lost
int           client_alive_counter       = 0;                   // client alive counter
WiFiUDP       Udp;                                              // UDP object
int           rssi;
volatile bool serviceRequest             = false;               // interrupt service can request a service intervall by this flag
char          packetBuffer[UDP_PACKET_MAX_SIZE];                // buffer for in/outcoming packets
char          clientInfo[UDP_PACKET_MAX_SIZE];                  // client info
#ifdef SERVER_ESP32
  hw_timer_t *serviceTimer            = NULL;
  portMUX_TYPE timerMux               = portMUX_INITIALIZER_UNLOCKED;
#endif

#ifdef CYCLETIME
  uint32_t  ticks;                                          // ticks from client performance measurement
  int       ticksArray[SERVER_TICKS_ARRAY_SIZE]={0};        // array of ticks from client performance measurement
  char      ticksString[50*SERVER_TICKS_ARRAY_SIZE];        // string of array tick 
  int       ticks_pointer    = 0;
  uint32_t  totalNumberTicks = 0;

  static inline void setNewTicks(uint32_t ticks){
    if (ticks_pointer > 9)
      ticks_pointer = 0;

    ticksArray[ticks_pointer] = ticks;
    ticks_pointer++;
    totalNumberTicks++;
  }

  static inline uint32_t getLatestTicks(){
    return ticksArray[ticks_pointer];
  }

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
    }
    
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
  }
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
        message += serverVersion;
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
        message += "<p>Client Infos: ";
        message += clientInfo;
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

    // requests that are different from root
    void handleNotFound(){
        server.send(404, "text/plain", "404: Not found");     // Send HTTP status 404 (Not Found) when there's no handler for the URI in the request
    }
#endif


static inline void sendAliveMsg(){
  if(wlan_complete){                                      // check if WLAN connection is established
    Udp.beginPacket(clientIpAddr, clientUdpPort);
    // udp.write(message,11); //Nachrichten text und länge
    #ifdef SERVER_ESP32
      Udp.printf(SERVER_ALIVE_MSG);                          // send alive messages to client
    #else //ESP8266
      Udp.write(SERVER_ALIVE_MSG);                          // send alive messages to client
    #endif
    Udp.endPacket();
    #ifdef DEBUG
      Serial.printf("sendAliveMsg(): Sending server alive message to client IP: %s and Port: %d\n", clientIpAddr.toString().c_str(), clientUdpPort);
    #endif
  }else{
    #ifdef DEBUG
      Serial.printf("sendAliveMsg(): No alive message send, because WLAN is not established\n");
    #endif
  }
}


static inline void checkClientStatus(){
  if(wlan_complete){  //if WLAN connection is established, check status of client
  
    // check if client is alive
    if (client_alive_counter > CLIENT_ALIVE_CNT_MAX){
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
      digitalWrite(SERVER_BAT_ALM_OUT, LOW);
      //digitalWrite(SERVER_ERROR_OUT, HIGH);
    }

    // check if client battery critical
    if (clientBatteryStatus == CLIENT_BAT_CRITICAL){
      #ifdef DEBUG
        Serial.printf("checkClientStatus(): Client battery is critical\n");
      #endif
      digitalWrite(SERVER_BAT_ALM_OUT, LOW);
      //digitalWrite(SERVER_ERROR_OUT, LOW);
    }

    // check if client battery ok
    if (clientBatteryStatus == CLIENT_BAT_OK){
      #ifdef DEBUG
        Serial.printf("checkClientStatus(): Client battery is ok\n");
      #endif
      digitalWrite(SERVER_BAT_ALM_OUT, HIGH);
      //digitalWrite(SERVER_ERROR_OUT, HIGH);
    }  
  }else{   //if(wlan_complete)
    #ifdef DEBUG
      Serial.printf("checkClientStatus(): No battery check done, because WLAN is not established\n");
    #endif
  } //end if(wlan_complete)
} //end void checkClientStatus()


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
  serviceRequest = false;                                 //reset service flag
  #ifdef SERVER_ESP32
    portEXIT_CRITICAL_ISR(&timerMux);                       // release protection of "serviceRequest"
  #endif
}


//void ICACHE_RAM_ATTR serviceIsr(){                        // timer interrupt for regular service
void IRAM_ATTR  serviceIsr(){                        // timer interrupt for regular service
  #ifdef SERVER_ESP32
    portENTER_CRITICAL_ISR(&timerMux);                 // protect access to serviceRequest 
  #endif
  serviceRequest = true;                             // exit isr as quickly as possible and start service routine in main loop
  #ifdef SERVER_ESP32
    portEXIT_CRITICAL_ISR(&timerMux);                  // release protection of data
  #endif
}


void wlanInit(){
  #ifdef DEBUG
    Serial.printf("\n\nwlanInit(): Soft-AP server IP: ");
    serverIpAddr = WiFi.softAPIP();                       // the server IP is set earlier, so WIFI.softAPIP() should not result in a different IP
    Serial.println(serverIpAddr);
  #endif
  
  //Waiting for client to connect to the soft AP
  #ifdef DEBUG
    Serial.print("wlanInit(): Waiting for clients to connect");
  #endif

  while(WiFi.softAPgetStationNum() < 1){                  // loop until the first client connects
    #ifdef DEBUG
      Serial.print(".");
    #endif

    #ifdef SERVER_RGB_LED
      neopixelWrite(SERVER_WLAN_LED,0,0,SERVER_RGB_BRIGHTNESS); // RGB LED Blue ######################### muss gelb sein
      delay(SLOW_BLINK);
      neopixelWrite(SERVER_WLAN_LED,0,0,0); // LED Off 
      delay(SLOW_BLINK);
    #else //normal LED
      digitalWrite(SERVER_WLAN_LED, HIGH);                         // switch off WLAN connection LED
      //do not blink LED to avoid false detection by CNC Controller
      //delay(SLOW_BLINK); 
      //digitalWrite(SERVER_WLAN_LED, LOW);                        // switch on WLAN connection LED
      delay(SLOW_BLINK);      //################################# kann das raus
    #endif
    yield();
    if (serviceRequest == true)        //do service routine if isr has set the service flag
      doService();
  } //waiting for clients to connect either 3D touch probe client or a webbrowser if webserver is enabled

  #ifdef DEBUG
    Serial.println("Ready");
    Serial.printf("wlanInit(): Clients connected: %d\n", WiFi.softAPgetStationNum());
  #endif

  Udp.begin(serverUdpPort);                               //setup UDP receiver

  while(!wlan_complete){
    //send hello message
    #ifdef DEBUG
      Serial.printf("wlanInit(): Sending Hello message to client IP: ");
      Serial.printf(clientIpAddr.toString().c_str());
      Serial.printf(" and Port: ");
      Serial.printf("%d\n", clientUdpPort);
    #endif
    Udp.beginPacket(clientIpAddr, clientUdpPort);
    #ifdef SERVER_ESP32
     Udp.printf(SERVER_HELLO_MSG);
    #else //ESP8266
      Udp.write(SERVER_HELLO_MSG);
    #endif    
    Udp.endPacket(); 
    
    // listen for respond from client
    int packetSize = Udp.parsePacket();
    if (packetSize){
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
        Udp.beginPacket(clientIpAddr, clientUdpPort);
        #ifdef SERVER_ESP32
          Udp.printf(SERVER_REPLY_MSG);
        #else //ESP8266
          Udp.write(SERVER_REPLY_MSG);
        #endif
        Udp.endPacket();        
      }

      wlan_complete = true;                               // WLAN connection is complete, stop listening for further incoming UPD packages
      #ifdef SERVER_RGB_LED
          neopixelWrite(SERVER_WLAN_LED,0,0,SERVER_RGB_BRIGHTNESS);
        #else
          digitalWrite(SERVER_WLAN_LED, LOW);                        // switch on WLAN connection LED to indicate sucessfull connection
      #endif
      #ifdef DEBUG
        Serial.print("wlanInit(): WLAN is complete\n");
      #endif
    }else{   //if (packetSize)
      #ifdef DEBUG
        Serial.printf("wlanInit(): No respond from client yet\n");
      #endif
      #ifdef SERVER_RGB_LED
          neopixelWrite(SERVER_WLAN_LED,0,0,0);      // ############### sind die  Farben richtig#####
        #else
          digitalWrite(SERVER_WLAN_LED, HIGH);                       // switch off WLAN connection LED
          //Do not blink WLAN LED to avoid false detection by CNC controller
          delay(FAST_BLINK);          // ############################## kann das raus, verzögert das nur unnötig die WLAN Verbindung
      #endif
      yield(); 
    } //if (packetSize)

    #ifdef WEBSERVER
    //handle client access if webserver is enabled
      server.handleClient();                              // need to be called even before loop(), to allow webbrowser to connect even if the 3D touch client is not connected
    #endif
  } //while(!wlan_complete)
}//void wlanInit()


void setup(){
  #ifdef DEBUG
    Serial.begin(BAUD_RATE);                                     // Setup Serial Interface with baud rate
    Serial.println("setup(): I am the 3D Touch Probe Sensor Server");
  #endif

  //initialize digital input/output pins
  pinMode(SERVER_POWER_LED,      OUTPUT);                        // LED to show power is on
  pinMode(SERVER_WLAN_LED,       OUTPUT);                        // LED to show WLAN connection state
  pinMode(SERVER_TOUCH_OUT,      OUTPUT);                        // Touch Output indicated the client has detected something
  pinMode(SERVER_ERROR_OUT,      OUTPUT);                        // Error Output, something is wrong, CNC should stop
  pinMode(SERVER_BAT_ALM_OUT,    OUTPUT);                        // Battery Alarm output to indicate to CNC input 
  pinMode(SERVER_SLEEP_LED,      OUTPUT);                        // Send Client to sleep LED
  pinMode(SERVER_SLEEP_IN, INPUT_PULLUP);                        // Send Client to sleep external input ############# pullup kann eigentlich raus, da ein TRansistor am eingang ist
  
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
  

  //write default values of digital outputs
  digitalWrite(SERVER_POWER_LED,   LOW);                         // switch on power LED
  //############ ESP32 RGB LED?
  digitalWrite(SERVER_WLAN_LED,    HIGH);                        // switch off WLAN connection LED
  digitalWrite(SERVER_TOUCH_OUT,   HIGH);                        // switch off tough output
  digitalWrite(SERVER_ERROR_OUT,   HIGH);                        // switch off error output
  digitalWrite(SERVER_BAT_ALM_OUT, HIGH);                        // switch off error output
  digitalWrite(SERVER_SLEEP_LED,   HIGH);                        // switch off sleep output
  
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
    digitalWrite(SERVER_ERROR_OUT, LOW);                         //switch on error output
  } // end if(result == true)
} //end setup() 




void loop(){

  char* errCheck;
    
  #ifdef WEBSERVER
    server.handleClient();  ////handle client access if webserver is enabled
  #endif
    
  // WLAN client got lost
  // better check would be compare the client IP address to the expected client IP address, other webbrowsers are clients as well
  if ((WiFi.softAPgetStationNum() < 1)||(wlan_complete == false)){
    wlan_complete = false;
    #ifdef DEBUG
      Serial.printf("loop(): Currently %d stations are connected and wlan_complete is %d\n", WiFi.softAPgetStationNum(), wlan_complete);
    #endif
    wlanInit();
  } //if (WiFi)

  // send sleep message to client if the server input pin is low (CNC does not need the sensor in the next minutes)
  if (((digitalRead(SERVER_SLEEP_IN) == HIGH)&&(!SERVER_SLEEP_IN_POLARITY)) || ((digitalRead(SERVER_SLEEP_IN) == LOW)&&(SERVER_SLEEP_IN_POLARITY))){
    digitalWrite(SERVER_SLEEP_LED,   LOW);                        // indicate sleep request by LED
    #ifdef DEBUG
      Serial.printf("loop(): CNC controller wants to send the client asleep\n");
    #endif
    Udp.beginPacket(clientIpAddr, clientUdpPort);
    #ifdef SERVER_ESP32
      Udp.printf(SERVER_SLEEP_MSG);
    #else //ESP8266
      Udp.write(SERVER_SLEEP_MSG);
    #endif

    Udp.endPacket();
  } // end if (digitalRead)

  // receive incoming UDP packets
  int packetSize = Udp.parsePacket();
  if (packetSize){
    int len = Udp.read(packetBuffer, UDP_PACKET_MAX_SIZE);
    if (len > 0)
      packetBuffer[len] = '\0';

    //received client sensor high command
    if (!strncmp (packetBuffer, CLIENT_TOUCH_HIGH_MSG, strlen(CLIENT_TOUCH_HIGH_MSG))){
      digitalWrite(SERVER_TOUCH_OUT, LOW);                                              // indicate current touch state by LED
      char *highMsgAttach = packetBuffer + strlen(CLIENT_TOUCH_HIGH_MSG);        // ##################was macht das für einen Sinn, die string länge an den String zu hängen ###############cut/decode additional information out of the client message
      if (!strncmp (highMsgAttach, "1", strlen("1")))
        msg_lost = true;
      #ifdef DEBUG
        Serial.printf("loop(): Client touch is high. Sending back the status. Attachment is %s\n", highMsgAttach);
      #endif
      Udp.beginPacket(clientIpAddr, clientUdpPort);
      #ifdef SERVER_ESP32
        Udp.printf(CLIENT_TOUCH_HIGH_MSG);
      #else //ESP8266
        Udp.write(CLIENT_TOUCH_HIGH_MSG);
      #endif
      Udp.endPacket();
      return;
    }

    //received client sensor low command
    if (!strncmp (packetBuffer, CLIENT_TOUCH_LOW_MSG, strlen(CLIENT_TOUCH_LOW_MSG))){
      digitalWrite(SERVER_TOUCH_OUT, HIGH);
      char *lowMsgAttach = packetBuffer + strlen(CLIENT_TOUCH_LOW_MSG);        // cut/decode additional information out of the client message
      if (!strncmp (lowMsgAttach, "1", strlen("1")))
        msg_lost = true;
      #ifdef DEBUG
          Serial.printf("loop(): Client touch is low. Sending back the status. Attachment is %s\n", lowMsgAttach);
      #endif
      Udp.beginPacket(clientIpAddr, clientUdpPort);
      #ifdef SERVER_ESP32
        Udp.printf(CLIENT_TOUCH_LOW_MSG);
      #else //ESP8266
        Udp.write(CLIENT_TOUCH_LOW_MSG);
      #endif
      Udp.endPacket();
      return;
    }

    //received client battery low
    if (!strncmp (packetBuffer, CLIENT_BAT_LOW_MSG, strlen(CLIENT_BAT_LOW_MSG))){
      clientBatteryStatus= CLIENT_BAT_LOW;
      #ifdef DEBUG
          Serial.printf("loop(): Client battery is low\n\n");
      #endif
      return;
    }

    //received client battery critical
    if (!strncmp (packetBuffer, CLIENT_BAT_CRITICAL_MSG, strlen(CLIENT_BAT_CRITICAL_MSG))){
      clientBatteryStatus = CLIENT_BAT_CRITICAL;
      #ifdef DEBUG
          Serial.printf("loop(): Client battery is critical\n\n");
      #endif
      return;
    }

    //received client battery is ok
    if (!strncmp (packetBuffer, CLIENT_BAT_OK_MSG, strlen(CLIENT_BAT_OK_MSG))){
      clientBatteryStatus= CLIENT_BAT_OK;
      char *temp = packetBuffer + strlen(CLIENT_BAT_OK_MSG);        // cut/decode the raw voltage number out of the client message
      clientBateryVoltage = strtof(temp, &errCheck);                // convert string to float
      #ifdef DEBUG
          Serial.printf("loop(): Received Client battery is ok message, current bat voltage is %.2fV\n", clientBateryVoltage);
      #endif
      return;
    }

    //received client is alive
    if (!strncmp (packetBuffer, CLIENT_ALIVE_MSG, strlen(CLIENT_ALIVE_MSG))){
      client_alive_counter = 0;    //reset client alive counter when receiving alive message
      #ifdef DEBUG
          Serial.printf("loop(): Received -Client is alive- message\n");
      #endif
      return;
    }

    //received client hello message
    if (!strncmp (packetBuffer, CLIENT_HELLO_MSG, strlen(CLIENT_HELLO_MSG))){
      #ifdef DEBUG
          Serial.printf("loop(): Detected client hello command, send a hello reply\n\n");
      #endif
      Udp.beginPacket(clientIpAddr, clientUdpPort);
      #ifdef SERVER_ESP32
        Udp.printf(SERVER_REPLY_MSG);
      #else //ESP8266
        Udp.write(SERVER_REPLY_MSG);
      #endif
      Udp.endPacket();
      return;
    }

    //received client reply message
    if (!strncmp (packetBuffer, CLIENT_REPLY_MSG, strlen(CLIENT_REPLY_MSG))){
      #ifdef DEBUG
        Serial.printf("loop(): Detected client reply message\n");
      #endif
      return;
    }
          
    //received client measures loop cycle message
    if (!strncmp (packetBuffer, CLIENT_CYCLE_MSG, strlen(CLIENT_CYCLE_MSG))){    // trying to decod the CLIENT_CYCLE_MSG prefix from the msg
      char *temp = packetBuffer + strlen(CLIENT_CYCLE_MSG);                    // cut/decode the raw number of ticks out of the client message
      #ifdef CYCLETIME
        ticks = atoi(temp);                                                      // convert string of ticks to integer of ticks
        setNewTicks(ticks);                                                      // safe the current ticks value in an array
      #endif
      #ifdef DEBUG
          Serial.printf("loop(): Received Cycle measurement message from client: %d ticks\n", ticks);
      #endif
      return;
    }

    //received rssi 
    if (!strncmp (packetBuffer, CLIENT_RSSI_MSG, strlen(CLIENT_RSSI_MSG))){      // trying to decod the CLIENT_RSSI_MSG prefix from the msg
      char *temp = packetBuffer + strlen(CLIENT_RSSI_MSG);                    // cut/decode the raw number of ticks out of the client message
      rssi = atoi(temp);
      #ifdef DEBUG
          Serial.printf("loop(): Received RSSI message from client: %d\n", rssi);
      #endif
      return;
    }

    //received client info message
    if (!strncmp (packetBuffer, CLIENT_INFO_MSG, strlen(CLIENT_INFO_MSG))){    // decode the CLIENT_INFO_MSG prefix from the msg
      char *temp = packetBuffer + strlen(CLIENT_INFO_MSG);                    // cut/decode the raw infos out of the client message
      //clientInfo = packetBuffer + strlen(CLIENT_INFO_MSG);                   // cut/decode the raw infos out of the client message
      strcpy(clientInfo, temp);
      #ifdef DEBUG
          Serial.printf("loop(): Received client info message: %s \n", clientInfo);
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
