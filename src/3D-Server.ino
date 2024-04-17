/**  server
  *   
  *  @author Heiko Kalte  
  *  @date 04.04.2024 
  * 
  *  @version 2.1
  */
  //  2.1   Send back HIGH and LOW messages from client to ensure the data was send correctly
  //        set "static inline" attribute to some functions 
  //        added basic webserver functions (prototype)
  //        supporting cycle time measurement messages from the client

#include <stdio.h>
#include <stdlib.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Ticker.h>
#include "Z:\Projekte\Mill\HeikosMill\3D Taster\Arduino\Sourcen\3D Taster\Header\CncSensor.h"

#ifdef WEBSERVER
    #include <ESP8266WebServer.h>
    #include <WiFiClient.h>
#endif

using namespace CncSensor;

//TODOs
//#################### es verbinden sich jetzt zwei clients mit dem server, einmal der client und einmal ein Rechner am webserver, dads muss berücksichtigt werden
// WLAN access mode https://arduino-esp8266.readthedocs.io/en/latest/esp8266wifi/soft-access-point-examples.html
// ESP8266WebServer server(80);            //setup webserver: https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/examples/WiFiAccessPoint/WiFiAccessPoint.ino
// Allow asking for version to prevent client, server, header version mix
// It is more usefull to provide a webserver by the client, otherwise all status Data must be transfered to the server
// Signal strength empfangen


int       clientBatteryStatus        = CLIENT_BAT_OK;       // init with battery is ok
float     clientBateryVoltage        = 0.0;                  // client voltage
bool      wlan_complete              = false;               // global variable to indicate if wlan connection is established completely 
int       client_alive_counter       = 0;                   // client alive counter
WiFiUDP   Udp;                                              // UDP object
char      packetBuffer[UDP_PACKET_MAX_SIZE];                // buffer for in/outcoming packets

#ifdef CYCLETIME
  int  ticks;                                             // ticks from client performance measurement
  int  ticksArray[SERVER_TICKS_ARRAY_SIZE]={0};         // array of ticks from client performance measurement
  char ticksString[10*SERVER_TICKS_ARRAY_SIZE];           // string of array tick 
  int ticks_pointer = 0; 

  static inline void setNewTicks(uint32 ticks){
    if (ticks_pointer >= 9)
      ticks_pointer = 0;
    else
      ticks_pointer++;
    ticksArray[ticks_pointer] = ticks;
  }

  static inline uint32 getLatestTicks(){
    return ticksArray[ticks_pointer];
  }

  static inline char* getAllTicks(){
    int i;
    float average = 0;
    char dstring[10];

    //####################################### html ausgabe stat

    strcpy(ticksString, "Client Ticks measurement:\n\t");
    for (i= 0; i <= SERVER_TICKS_ARRAY_SIZE-1; i++){
      average = average + (float) ticksArray[i];                  // calculate average
      itoa(ticksArray[i], dstring, 10);                   // convert ticks integer to char/string
      strcat(ticksString, dstring);                       // concatenate message string with converted integer string
      strcat(ticksString, "\n\t");                        // add new line and tab
    }
    strcat(ticksString, "\nAverage ticks:");

    average = average / SERVER_TICKS_ARRAY_SIZE;          // calculate final average of the ticks measurements
    sprintf(dstring, "%.2f", average); 
    strcat(ticksString, "\n\t");                        // add new line and tab
    strcat(ticksString, dstring);

    #ifdef DEBUG
      Serial.printf("getAllTicks(): %s\n", ticksString);
    #endif
    return ticksString;
  }

#endif //CYCLETIME

#ifdef WEBSERVER
    ESP8266WebServer server(80);
    // root web page of the server
    void handleRoot() {
        //server.send(200, "text/html", "<h1>You are connected</h1>");
        String message = getAllTicks();
        message += "Ein neuer Taxt";
        message += "noch ein neuer Text";
/*

          message += "DEBUG: not defined";                    #else        #ifdef DEBUG
        #endif
        #ifdef CYCLETIME
          message += "CYCLETIME: defined";
        #else
          message += "CYCLETIME: not defined";
        #endif
        #ifdef WEBSERVER
          message += "WEBSERVER: defined";
        #else
          message += "WEBSERVER: not defined";
        #endif
        #ifndef DEBUG
        */
        server.send(200, "text/html", message);
        
        //to display:
        // Header
        // präprozessor settings
        // Version of server and version of client
        // client status
        // durchschnitt von ticks 
        // sending alive messages every x Sek
        // Client Status (battery, WIFI signal, WLAN status)
        
            
            /* https://forum.arduino.cc/t/webserver-ota-client-print-server-send/693298/30
            // ################zusammenbau der Daten
            void Datenzeigen() {
              String message = "Daten angekommen\n";
              message += "URI: ";
              message += server.uri();
              message += "\nMethod: ";
              message += (server.method() == HTTP_GET) ? "GET" : "POST";
              message += "\nArguments: ";
              message += server.args();
              message += "\n";
              for (uint8_t i = 0; i < server.args(); i++) {
                message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
              }
              Serial.println(message);
            }
            */
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
  }
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
  boolean result = WiFi.softAP(ssid, password);
  if(result == true){
    #ifdef DEBUG
      Serial.println("Ready");
    #endif
    wlanInit();
  }else{
    #ifdef DEBUG
      Serial.println("Failed!");
    #endif
    digitalWrite(ERROR_OUT, LOW);                         //switch on error output
  } // end if(result == true)
  #ifdef WEBSERVER
      server.on("/", handleRoot);
      server.onNotFound(handleNotFound);                  // When a client requests an unknown URI (i.e. something other than "/"), call function "handleNotFound"
      server.begin();
      Serial.println("setup(): HTTP server started");
  #endif
} //end setup() 


void loop(){

  char* errCheck;
    
  #ifdef WEBSERVER
    //handle client access if webserver is enabled
    server.handleClient();
  #endif
    
  // WLAN client got lost
  if ((WiFi.softAPgetStationNum() < 1)||(wlan_complete == false)){
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
      //wlan_complete = false;                            //client will disconnect
      Udp.beginPacket(clientIpAddr, clientUdpPort);
      Udp.write(SERVER_SLEEP_MSG);
      Udp.endPacket();
    }

    // receive incoming UDP packets
    int  packetSize = Udp.parsePacket();
    //if (wlan_complete){ // skip everything except for the interrupts (timer and digital in) if wlan connection is not established
      if (packetSize){
        int len = Udp.read(packetBuffer, UDP_PACKET_MAX_SIZE);
        if (len > 0)
          packetBuffer[len] = '\0';

        //received client sensor high command
        if (!strcmp (packetBuffer, CLIENT_TOUCH_HIGH_MSG)){
          digitalWrite(TOUCH_OUT, LOW);
          #ifdef DEBUG
            Serial.printf("loop(): Client touch is high. Sending back the status\n");
          #endif
            Udp.beginPacket(clientIpAddr, clientUdpPort);
            Udp.write(CLIENT_TOUCH_HIGH_MSG);             // send back client high messages to client for response/feedback
            Udp.endPacket();
        }else{

            //received client sensor low command
            if (!strcmp (packetBuffer, CLIENT_TOUCH_LOW_MSG)){
                digitalWrite(TOUCH_OUT, HIGH);
                #ifdef DEBUG
                    Serial.printf("loop(): Client touch is low.  Sending back the status\n");
                #endif
                Udp.beginPacket(clientIpAddr, clientUdpPort);
                Udp.write(CLIENT_TOUCH_LOW_MSG);          // send back client low messages to client for response/feedback
                Udp.endPacket();
            }else{

                //received client battery low
                if (!strncmp (packetBuffer, CLIENT_BAT_LOW_MSG, strlen(CLIENT_BAT_LOW_MSG))){
                    clientBatteryStatus= CLIENT_BAT_LOW;
                    #ifdef DEBUG
                        Serial.printf("loop(): Client battery is low\n\n");
                    #endif
                }else{

                  //received client battery critical
                  if (!strncmp (packetBuffer, CLIENT_BAT_CRITICAL_MSG, strlen(CLIENT_BAT_CRITICAL_MSG))){
                    clientBatteryStatus = CLIENT_BAT_CRITICAL;
                    #ifdef DEBUG
                        Serial.printf("loop(): Client battery is critical\n\n");
                    #endif
                  }else{

                        //received client battery is ok
                        if (!strncmp (packetBuffer, CLIENT_BAT_OK_MSG, strlen(CLIENT_BAT_OK_MSG))){
                            clientBatteryStatus= CLIENT_BAT_OK;
                            char *temp = packetBuffer + strlen(CLIENT_BAT_OK_MSG);        // cut/decode the raw voltage number out of the client message
                            clientBateryVoltage = strtof(temp, &errCheck);                // convert string to float
                            #ifdef DEBUG
                                Serial.printf("loop(): Received Client battery is ok message, current bat voltage is %.2fV\n", clientBateryVoltage);
                            #endif
                        }else{

                            //received client is alive
                            if (!strncmp (packetBuffer, CLIENT_ALIVE_MSG, strlen(CLIENT_ALIVE_MSG))){
                                client_alive_counter = 0;    //reset client alive counter when receiving alive message
                                #ifdef DEBUG
                                    Serial.printf("loop(): Received -Client is alive- message\n");
                                #endif
                            }else{

                                //received client hello message
                                if (!strncmp (packetBuffer, CLIENT_HELLO_MSG, strlen(CLIENT_HELLO_MSG))){
                                    #ifdef DEBUG
                                        Serial.printf("loop(): Detected client hello command, send a hello reply\n\n");
                                    #endif
                                    Udp.beginPacket(clientIpAddr, clientUdpPort);
                                    Udp.write(SERVER_REPLY_MSG);
                                    Udp.endPacket();
                                }else{

                                    //received client reply message
                                    if (!strncmp (packetBuffer, CLIENT_REPLY_MSG, strlen(CLIENT_REPLY_MSG))){
                                        #ifdef DEBUG
                                          Serial.printf("loop(): Detected client reply message\n");
                                        #endif
                                     }else{
                                         
                                        //received client measures loop cycle message
                                        if (!strncmp (packetBuffer, CLIENT_CYCLE_MSG, strlen(CLIENT_CYCLE_MSG))){    // trying to decod the CLIENT_CYCLE_MSG prefix from the msg
                                            char *temp = packetBuffer + strlen(CLIENT_CYCLE_MSG);                    // cut/decode the raw number of ticks out of the client message
                                            ticks = atoi(temp);                                                      // convert string of ticks to integer of ticks
                                            setNewTicks(ticks);                                                      // safe the current ticks value in an array
                                            #ifdef DEBUG
                                                Serial.printf("loop(): Received Cycle measurement message from client: %d ticks\n", ticks);
                                            #endif
                                        }else{

                                            // unknown command
                                            #ifdef DEBUG
                                              Serial.printf("loop(): UDP content is unknown. Content is %s\n\n", packetBuffer);
                                            #endif
                                        }
                                     }
                                }
                            }
                        }    
                    }    
                }
            }
         }
    }
  //}
}//end loop()