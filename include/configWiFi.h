// config: ////////////////////////////////////////////////////////////
// 

#ifndef MY_CONFIGWIFI_H
#define MY_CONFIGWIFI_H

#include <WiFi.h>

IPAddress netmask(255, 255, 255, 0);

#define bufferSize 1024


#ifdef INIDEF_WIFI_STA

#if(0)
const char *ssid = esp32AP_wifi_ssid;
const char *pw = esp32AP_wifi_pw;
IPAddress ip_gway   (192, 168, 66,  1);
IPAddress ip_static (192, 168, 66, 22); // client
#endif

#if(0)
const char *ssid = android_wifi_ssid;
const char *pw = android_wifi_pw;
IPAddress ip_gway   (192, 168, 43,  1);
IPAddress ip_static (192, 168, 43, 11);
#endif

#if(1)
const char *ssid = netcomm24_wifi_ssid;
const char *pw = netcomm24_wifi_pw;
IPAddress ip_gway   (192, 168, 20,  1);
IPAddress ip_static (192, 168, 20, 11);
#endif

#endif


#ifdef INIDEF_WIFI_AP
const char *ssid = esp32AP_wifi_ssid;
const char *pw = esp32AP_wifi_pw;
IPAddress ip_gway   (192, 168, 66,  1);
IPAddress ip_static (192, 168, 66,  1); // server
#endif


bool debug = true;


#endif // MY_CONFIGWIFI_H

// END_OF_FILE
