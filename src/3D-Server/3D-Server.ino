/**  server
  *   
  *  @author Heiko Kalte  
  *  @date 20.04.2024 
  * 
  *  @version 0.1
  */
  //  0.1   initial version


#include <stdio.h>
#include <stdlib.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Ticker.h>
#include "Z:\Projekte\Mill\HeikosMill\3D Taster\Arduino\GIT\3D-Touch-Sensor\src\3D-Header.h"

#ifdef WEBSERVER
    #include <ESP8266WebServer.h>
    #include <WiFiClient.h>
#endif

using namespace CncSensor;

//TODOs


int       clientBatteryStatus        = CLIENT_BAT_OK;       // init with battery is ok
float     clientBateryVoltage        = 0.0;                 // client voltage
bool      wlan_complete              = false;               // global variable to indicate if wlan connection is established completely 
bool      msg_lost                   = false;               // did a UDP message got lost
int       client_alive_counter       = 0;                   // client alive counter
WiFiUDP   Udp;                                              // UDP object
int       rssi;
char      packetBuffer[UDP_PACKET_MAX_SIZE];                // buffer for in/outcoming packets

#ifdef CYCLETIME
  uint32  ticks;                                              // ticks from client performance measurement
  int    ticksArray[SERVER_TICKS_ARRAY_SIZE]={0};             // array of ticks from client performance measurement
  char   ticksString[50*SERVER_TICKS_ARRAY_SIZE];             // string of array tick 
  int    ticks_pointer    = 0;
  uint32 totalNumberTicks = 0; 

  static inline void setNewTicks(uint32 ticks){
    if (ticks_pointer > 9)
      ticks_pointer = 0;

    ticksArray[ticks_pointer] = ticks;
    ticks_pointer++;
    totalNumberTicks++;
  }

  static inline uint32 getLatestTicks(){
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
      //to display:
        // Version of server and version of client
        // my IP expected client IP
        // sending alive messages every x Sek

    ESP8266WebServer server(80);
    
    // root web page of the server
    void handleRoot() {
        String message = "<h1>Webserver 3D Touch Probe</h1>";
        message += "<h3>by Heiko Kalte (h.kalte@gmx.de)</h3>";
        message += "<p>Number of connected clients: ";
        message += WiFi.softAPgetStationNum();
        message += "</p>";
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
    Udp.write(SERVER_ALIVE_MSG);                          // send alive messages to client
    Udp.endPacket();
    #ifdef DEBUG
      Serial.printf("sendAliveMsg(): Sending server alive message to client IP: %s and Port: %d\n", clientIpAddr.toString().c_str(), clientUdpPort);
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
      digitalWrite(BAT_ALM_OUT, LOW);
      digitalWrite(ERROR_OUT, HIGH);
    }

    // check if client battery critical
    if (clientBatteryStatus == CLIENT_BAT_CRITICAL){
      #ifdef DEBUG
        Serial.printf("checkClientStatus(): Client battery is critical\n");
      #endif
      digitalWrite(BAT_ALM_OUT, LOW);
      digitalWrite(ERROR_OUT, LOW);
    }

    // check if client battery ok
    if (clientBatteryStatus == CLIENT_BAT_OK){
      #ifdef DEBUG
        Serial.printf("checkClientStatus(): Client battery is ok\n");
      #endif
      digitalWrite(BAT_ALM_OUT, HIGH);
      digitalWrite(ERROR_OUT, HIGH);
    }  
  } //end if(wlan_complete)
} //end void checkClientStatus()


static inline void doService(){
  #ifdef DEBUG
    Serial.printf("\ndoService(): start service\n");
  #endif
    sendAliveMsg();                                       // send regular alive messages to client
    checkClientStatus();                                  // check regularly client status 
  #ifdef DEBUG
    Serial.printf("doService(): end service\n\n");
  #endif
}


void ICACHE_RAM_ATTR serviceIsr(){                        // timer interrupt for regular service
  doService();
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
    digitalWrite(WLAN_LED, HIGH);                         // switch off WLAN connection LED
    //do not blink LED to avoid false detection by CNC Controller
    //delay(SLOW_BLINK); 
    //digitalWrite(WLAN_LED, LOW);                        // switch on WLAN connection LED
    delay(SLOW_BLINK); 
    yield();
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
    Udp.write(SERVER_HELLO_MSG);
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
          Serial.printf("wlanInit(): Detected client hello command durcing wlanInit, send a reply\n\n");
        #endif
        Udp.beginPacket(clientIpAddr, clientUdpPort);
        Udp.write(SERVER_REPLY_MSG);
        Udp.endPacket();        
      }

      //clientIpAddr  = Udp.remoteIP();                     //######check ob dies die richtige ist?
      //clientUdpPort = Udp.remotePort();
      wlan_complete = true;                               // WLAN connection is complete, stop listening for further incoming UPD packages
      digitalWrite(WLAN_LED, LOW);                        // switch on WLAN connection LED to indicate sucessfull connection
      #ifdef DEBUG
        Serial.print("wlanInit(): WLAN is complete\n");
      #endif
    }else{
      #ifdef DEBUG
        Serial.printf("wlanInit(): No respond from client yet\n");
      #endif
      digitalWrite(WLAN_LED, HIGH);                       // switch off WLAN connection LED
      //Do not blink WLAN LED to avoid false detection by CNC controller
      //delay(FAST_BLINK); 
      //digitalWrite(WLAN_LED, LOW);                      // switch on WLAN connection LED
      delay(FAST_BLINK);
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
    Serial.begin(115200);                                 // Setup Serial Interface with baud rate
  #endif

  //initialize digital input/output pins
  pinMode(POWER_LED,      OUTPUT);                        // LED to show power is on
  pinMode(WLAN_LED,       OUTPUT);                        // LED to show WLAN connection state
  pinMode(TOUCH_OUT,      OUTPUT);                        // Touch Output indicated the client has detected something
  pinMode(ERROR_OUT,      OUTPUT);                        // Error Output, something is wrong, CNC should stop
  pinMode(BAT_ALM_OUT,    OUTPUT);                        // Battery Alarm output to indicate to CNC input 
  pinMode(SLEEP_IN, INPUT_PULLUP);                        // Send Client to sleep
 
 //Initialize timer interrup for battery control
  timer1_attachInterrupt(serviceIsr);
  timer1_enable(TIM_DIV256, TIM_EDGE, TIM_LOOP);          // DIV256 means one tick is 3.2us, total intervall time is 3.2us x SERVICE_INTERVALL, max is ~27 sec
  timer1_write(SERVICE_INTERVALL);                        // Setup serice intervall
  
  //write default values of digital outputs
  digitalWrite(POWER_LED,   LOW);                         // switch on power LED
  digitalWrite(WLAN_LED,    HIGH);                        // switch off WLAN connection LED
  digitalWrite(TOUCH_OUT,   HIGH);                        // switch off tough output
  digitalWrite(ERROR_OUT,   HIGH);                        // switch off error output
  digitalWrite(BAT_ALM_OUT, HIGH);                        // switch off error output
  
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
        Serial.println("setup(): HTTP server started");
      #endif
    #endif

    wlanInit();
  }else{
    #ifdef DEBUG
      Serial.println("Failed!");
    #endif
    digitalWrite(ERROR_OUT, LOW);                         //switch on error output
  } // end if(result == true)
} //end setup() 


void loop(){

  char* errCheck;
    
  #ifdef WEBSERVER
    //handle client access if webserver is enabled
    server.handleClient();
  #endif
    
  // WLAN client got lost
  if ((WiFi.softAPgetStationNum() < 1)||(wlan_complete == false)){ //############################### das passt nicht mehr richtig
    wlan_complete = false;
    #ifdef DEBUG
      Serial.printf("loop(): Currently %d stations are connected and wlan_complete is %d\n", WiFi.softAPgetStationNum(), wlan_complete);
    #endif
    wlanInit();
  }

  // send sleep message to client if the server input pin is low (CNC does not need the sensor in the next minutes)
  if (digitalRead(SLEEP_IN) == LOW){
    #ifdef DEBUG
      Serial.printf("loop(): CNC controller wants to send the client asleep\n");
    #endif
    Udp.beginPacket(clientIpAddr, clientUdpPort);
    Udp.write(SERVER_SLEEP_MSG);
    Udp.endPacket();
  }

    // receive incoming UDP packets
    int  packetSize = Udp.parsePacket();
    if (packetSize){
        int len = Udp.read(packetBuffer, UDP_PACKET_MAX_SIZE);
        if (len > 0)
          packetBuffer[len] = '\0';

        //received client sensor high command
        if (!strncmp (packetBuffer, CLIENT_TOUCH_HIGH_MSG, strlen(CLIENT_TOUCH_HIGH_MSG))){
          digitalWrite(TOUCH_OUT, LOW);                                              // indicate current touch state by LED
          char *highMsgAttach = packetBuffer + strlen(CLIENT_TOUCH_HIGH_MSG);        // cut/decode additional information out of the client message
          if (!strncmp (highMsgAttach, "1", strlen("1")))
            msg_lost = true;
          #ifdef DEBUG
            Serial.printf("loop(): Client touch is high. Sending back the status. Attachment is %s\n", highMsgAttach);
          #endif
            Udp.beginPacket(clientIpAddr, clientUdpPort);
            Udp.write(CLIENT_TOUCH_HIGH_MSG);             // send back client high messages to client for response/feedback
            Udp.endPacket();
        }

        //received client sensor low command
        if (!strncmp (packetBuffer, CLIENT_TOUCH_LOW_MSG, strlen(CLIENT_TOUCH_LOW_MSG))){
            digitalWrite(TOUCH_OUT, HIGH);
            char *lowMsgAttach = packetBuffer + strlen(CLIENT_TOUCH_LOW_MSG);        // cut/decode additional information out of the client message
            if (!strncmp (lowMsgAttach, "1", strlen("1")))
              msg_lost = true;
            #ifdef DEBUG
                Serial.printf("loop(): Client touch is low. Sending back the status. Attachment is %s\n", lowMsgAttach);
            #endif
            Udp.beginPacket(clientIpAddr, clientUdpPort);
            Udp.write(CLIENT_TOUCH_LOW_MSG);                                        // send back client low messages to client for response/feedback
            Udp.endPacket();
        }

        //received client battery low
        if (!strncmp (packetBuffer, CLIENT_BAT_LOW_MSG, strlen(CLIENT_BAT_LOW_MSG))){
            clientBatteryStatus= CLIENT_BAT_LOW;
            #ifdef DEBUG
                Serial.printf("loop(): Client battery is low\n\n");
            #endif
        }

        //received client battery critical
        if (!strncmp (packetBuffer, CLIENT_BAT_CRITICAL_MSG, strlen(CLIENT_BAT_CRITICAL_MSG))){
          clientBatteryStatus = CLIENT_BAT_CRITICAL;
          #ifdef DEBUG
              Serial.printf("loop(): Client battery is critical\n\n");
          #endif
        }

        //received client battery is ok
        if (!strncmp (packetBuffer, CLIENT_BAT_OK_MSG, strlen(CLIENT_BAT_OK_MSG))){
            clientBatteryStatus= CLIENT_BAT_OK;
            char *temp = packetBuffer + strlen(CLIENT_BAT_OK_MSG);        // cut/decode the raw voltage number out of the client message
            clientBateryVoltage = strtof(temp, &errCheck);                // convert string to float
            #ifdef DEBUG
                Serial.printf("loop(): Received Client battery is ok message, current bat voltage is %.2fV\n", clientBateryVoltage);
            #endif
        }

        //received client is alive
        if (!strncmp (packetBuffer, CLIENT_ALIVE_MSG, strlen(CLIENT_ALIVE_MSG))){
            client_alive_counter = 0;    //reset client alive counter when receiving alive message
            #ifdef DEBUG
                Serial.printf("loop(): Received -Client is alive- message\n");
            #endif
        }

        //received client hello message
        if (!strncmp (packetBuffer, CLIENT_HELLO_MSG, strlen(CLIENT_HELLO_MSG))){
            #ifdef DEBUG
                Serial.printf("loop(): Detected client hello command, send a hello reply\n\n");
            #endif
            Udp.beginPacket(clientIpAddr, clientUdpPort);
            Udp.write(SERVER_REPLY_MSG);
            Udp.endPacket();
        }

        //received client reply message
        if (!strncmp (packetBuffer, CLIENT_REPLY_MSG, strlen(CLIENT_REPLY_MSG))){
            #ifdef DEBUG
              Serial.printf("loop(): Detected client reply message\n");
            #endif
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
        }

        //received rssi 
        if (!strncmp (packetBuffer, CLIENT_RSSI_MSG, strlen(CLIENT_RSSI_MSG))){      // trying to decod the CLIENT_RSSI_MSG prefix from the msg
            char *temp = packetBuffer + strlen(CLIENT_RSSI_MSG);                    // cut/decode the raw number of ticks out of the client message
            rssi = atoi(temp);
            #ifdef DEBUG
                Serial.printf("loop(): Received RSSI message from client: %d\n", rssi);
            #endif
        }

        /*
        // unknown command
        #ifdef DEBUG
          Serial.printf("loop(): UDP content is unknown. Content is %s\n\n", packetBuffer);
        #endif
        */
    }
}//end loop()
