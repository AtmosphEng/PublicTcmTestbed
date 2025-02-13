// config: ////////////////////////////////////////////////////////////
// 

#ifndef CONFIG_NETWORK_H
#define CONFIG_NETWORK_H


#include <WiFi.h>
#include "..\..\Credentials\Credentials.h"


#if(1) // AAATEST

#ifdef NETWORK_NETCOMM
char ssid[] = netcomm24_wifi_ssid;
char pw[] = netcomm24_wifi_pw;
#endif

#ifdef NETWORK_AP_WAYSIDE_MONO // embedded target hosts its own access point.
char ssid[] = ap_wayside_wifi_ssid;
char pw[] = ap_wayside_wifi_pw;
#endif

#ifdef NETWORK_AP_ONBOARD_LINEAR_ENC // embedded target hosts its own access point.
char ssid[] = ap_onboard_wifi_ssid;
char pw[] = ap_onboard_wifi_pw;
#endif

#ifdef NETWORK_HOTSPOT_LINUX
char ssid[] = hotspot2_wifi_ssid;
char pw[] = hotspot2_wifi_pw;
#endif

#ifdef TCP_SERVER_BUILD
#define DEF_WIFI_IP 			NET_ADDR_BYTE_1,NET_ADDR_BYTE_2,NET_ADDR_SUBNET_BYTE,TARGET_NUM_SERVER
#define TARGET_NUM TARGET_NUM_SERVER
//WiFiServer tcpServer(DEF_SERVER_PORT);
#endif

#ifdef TCP_CLIENT_BUILD
#define DEF_WIFI_IP 			NET_ADDR_BYTE_1,NET_ADDR_BYTE_2,NET_ADDR_SUBNET_BYTE,TARGET_NUM_CLIENT
#define TARGET_NUM TARGET_NUM_CLIENT
#endif

#define DEF_WIFI_GWAY NET_ADDR_BYTE_1,NET_ADDR_BYTE_2,NET_ADDR_SUBNET_BYTE,1  // default gateway. macro concatenation.
IPAddress netmask(255, 255, 255, 0); 				// default


IPAddress ip_static(DEF_WIFI_IP);
IPAddress ip_gway(DEF_WIFI_GWAY);

WiFiServer tcpServer(DEF_SERVER_PORT); // also required for client build when linking to CommsHelper
WiFiClient tcpClient;


#if(1)
#include "../../libraries/VirtSerial/src/VirtSerial.h"
VirtSerial myCom0com; // Instatiating this and not referring to it in other code causes a link error. Why?
#endif

#if(0)
#include "../../libraries/VirtSerial/src/VirtSerial.h"
VirtSerial myCom0com; // my class object.
#endif


#endif // AAATEST


#endif // CONFIG_NETWORK_H

// END_OF_FILE
