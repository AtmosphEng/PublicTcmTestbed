//
// myConfig.h
//
#ifndef MY_CONFIG_H
#define MY_CONFIG_H

// MAJOR FUNCTION *******************************************************************************************
//#define MASTER

// tcmenu UI ************************************************************************************************
#ifdef INIDEF_LILYGO_T_DISPLAY_S3
#ifdef MASTER
#define TCMENU_INPUT_2_BUTTON // if defined, this re-allocates GPIO pins to enable Serial2
#else
#define TCMENU_INPUT_ENC1 // not currently used
#endif
#endif

//#define CONF_EXTRA_ENC2_TCW // TCWRotaryEncoder2 - NOT defined in TcmDesigner
//#define CONF_EXTRA_ENC3_CCW // CCWRotaryEncoder3 - NOT defined in TcmDesigner

// BluetoothSerial - only available in older ESP32 e.g. ESP32-WROOM *****************************************
//#define SERIALBT_CLASSIC
//#define SERIALBT_TRANSPARENT_BRIDGE_FOR_SERIAL2

//#define TARGET_NUM=1000 // MUTUALLY-EXCLUSIVE: BT_MASTER
//#define TARGET_NUM=5 // MUTUALLY-EXCLUSIVE: BT_SLAVE

#define INIDEF_WIFI_GWAY 192,168,20,1

#define TARGET_NUM 33

#if (TARGET_NUM == 5)
#define INIDEF_WIFI_IP2 192,168,20,5 // IP_ADDR_DOUBLE_DEFINITION WITH ARDUINOOTA UPLOAD
#elif ((TARGET_NUM == 12))
#define INIDEF_WIFI_IP2 192,168,20,12 // IP_ADDR_DOUBLE_DEFINITION WITH ARDUINOOTA UPLOAD
#elif (TARGET_NUM == 21)
#define INIDEF_WIFI_IP2 192,168,20,21 // IP_ADDR_DOUBLE_DEFINITION WITH ARDUINOOTA UPLOAD
#elif (TARGET_NUM == 22)
#define INIDEF_WIFI_IP2 192,168,20,22 // IP_ADDR_DOUBLE_DEFINITION WITH ARDUINOOTA UPLOAD
#elif (TARGET_NUM == 33)
#define INIDEF_WIFI_IP2 192,168,20,33 // IP_ADDR_DOUBLE_DEFINITION WITH ARDUINOOTA UPLOAD
#else
#define INIDEF_WIFI_IP2 192,168,20,99 // default
#endif

#ifdef INIDEF_MEGA2560_ETHERNET
#include <Ethernet.h>
#undef ETH_CLK_MODE // From Lilygo github example.
#define ETH_CLK_MODE        ETH_CLOCK_GPIO17_OUT
#define ETH_POWER_PIN       5
#endif


#endif // ifndef for include only once.

// END_OF_FILE
