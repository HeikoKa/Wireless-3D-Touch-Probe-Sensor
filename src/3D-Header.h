/**  Header
  *   
  *  @author Heiko Kalte  
  *  @date 20.04.2024 
  * 
  *  @version 0.1
  */
  // 0.1:   Time measurement for directive CYCLETIME
  //        Renamed Client BAT_LED to CLIENT_ERROR_OUT
  //        Reduced waiting times for "alive"
  //        New message CLIENT_CYCLE_MSG added 

#ifndef CNCSENSOR_H_
#define CNCSENSOR_H_

//#define DEBUG       // enable debug output via serial interface on client and server
#define CYCLETIME   // enable measuring the round trip delay from client to server back to client
#define WEBSERVER   // enable webserver

// Pinbelegung: https://fearby.com/wp-content/uploads/2022/03/WeMosMiniD1ProGuide.png

namespace CncSensor{
 
  const char*        SSID                   = "3D-Touch-WIFI";          // WLAN network name
  const char*        password               = "123456789";              // WLAN password

  //server consts 
  const char*        SERVER_SLEEP_MSG       = "sleep";                  // UDP message from server to client to send client asleep
  const char*        SERVER_HELLO_MSG       = "hello";                  // UDP message from server to client to say hello
  const char*        SERVER_REPLY_MSG       = "reply";                  // UDP message from server to client to reply to hello msg
  const char*        SERVER_ALIVE_MSG       = "alive";                  // UDP message from server to show server is alive
  const int          CLIENT_ALIVE_CNT_MAX   = 2;                        // maximum client alive counter value
  const long         WIFI_RSSI_REPORT_LEVEL = -65;                      // RSSI level, WLAN signal strength e.g. -40 is better -70 is worse (best was -44)
  const int          SERVER_TICKS_ARRAY_SIZE= 10;                       // Array Size of measured ticks
  //client consts
  const char*        CLIENT_HELLO_MSG       = "hello";                  // UDP message from client to server to say hello
  const char*        CLIENT_REPLY_MSG       = "reply";                  // UDP message from client to server to reply to hello msg
  const char*        CLIENT_TOUCH_HIGH_MSG  = "high";                   // UDP message from client if 3D touch probe is high
  const char*        CLIENT_TOUCH_LOW_MSG   = "low";                    // UDP message from client if 3D touch probe is low
  const char*        CLIENT_ALIVE_MSG       = "alive";                  // UDP message from client alive
  const char*        CLIENT_RSSI_MSG        = "rssi";                   // UDP message from client with rssi (WLAN signal strength)
  const char*        CLIENT_CYCLE_MSG       = "CYC:";                   // UDP message from client to server to transmit the last measured cycle time ticks
  const int          SERVER_ALIVE_CNT_MAX   = 1;                        // [24.03.24] reduced von 5 to 1, maximum server alive counter value, try to reconnect
  const int          SERVER_ALIVE_CNT_DEAD  = 2;                        // [24.03.24] reduced von 5 to 2, maximum server alive counter value, server seems to be dead
  const int          SERVER_AQUN_CNT_MAX    = 2000;                     // number of loop cycles before the server must acknowledge the high/low messages
  const int          TOUCH_PIN_DEBOUNCE     = 700;                      // debounce time in µs for touch input pin


  //client and server consts
  const uint32_t     SERVICE_INTERVALL      = 4000000;                  // [24.03.24]changed 8388607 to 4000000 from timer ticks for service interrupt (max 8388607)
  const int          UDP_PACKET_MAX_SIZE    = 16;                       // UDP buffer size
  const uint8_t      SLOW_BLINK             = 1000;                     // delay for slow blinking LED
  const uint8_t      FAST_BLINK             = 500;                      // delay for fast blinking LED

  //IP addresses
  IPAddress         serverIpAddr(192,168,  2,1);                        // local IP address of server
  IPAddress         clientIpAddr(192,168,  2,2);                        // local IP address of client
  IPAddress         gateway     (192,168,  2,0);                        // gateway of server, need to have a value, but does not exist
  IPAddress         subnet      (255,255,255,0);                        // subnet mask of WLAN network

  //global variables
  unsigned int      serverUdpPort           = 4211;                     // server port to listen on
  unsigned int      clientUdpPort           = 4210;                     // client port to listen on

  //Battery consts
  const float       BAT_LOW_VOLT            = 2.9;                      // battery voltage below this, is indicated as low battery state
  const float       BAT_CRIT_VOLT           = 2.7;                      // battery voltage below this, is indicated as critical battery state
  const char*       CLIENT_BAT_OK_MSG       = "bat ok";                 // UDP message for battery voltage is ok
  const int         CLIENT_BAT_OK           = 1;                        // internal coding for battery is ok
  const char*       CLIENT_BAT_LOW_MSG      = "bat low";                // UDP message for battery is low 
  const int         CLIENT_BAT_LOW          = 2;                        // internal coding for battery is low
  const char*       CLIENT_BAT_CRITICAL_MSG = "bat critical";           // UDP message for battery is critical
  const int         CLIENT_BAT_CRITICAL     = 3;                        // internal coding for battery is critical

  //LEDs Common server and client (given in GPIO x)
  const uint8_t     POWER_LED               = 2;                        // (D4) Power LED 
  const uint8_t     WLAN_LED                = 0;                        // (D3) LED to inducate the current WLAN state

  //Server specific LEDs (given in GPIO x)
  const uint8_t     TOUCH_OUT               = 5;                        // (D1) LED to indicate a touch of the 3D sense
  const uint8_t     ERROR_OUT               = 14;                       // (D5) ERROR output that can hold the cnc controller, e.g.critical battery or no more alive msg from client
  const uint8_t     SLEEP_IN                = 12;                       // (D6) Input pin for the CNC controller to indicate that the slave can go to sleep
  const uint8_t     BAT_ALM_OUT             = 4;                        // (D2) client battery is low

  //Client specific LEDs (given in GPIO x)
  const uint8_t     CLIENT_ERROR_OUT        = 5;                        // (D1) LED to indicate error
  const uint8_t     TOUCH_LED               = 4;                        // (D2) LED to indicate a touch of the 3D sensor
  const uint8_t     TOUCH_IN                = 12;                       // (D6) digital input pin to listen
  const uint8_t     PREVENT_RESET_OUT       = 14;                       // (D5) prevents reset by touch pin when client is alive
}
#endif