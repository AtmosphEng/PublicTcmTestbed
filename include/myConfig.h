//
// myConfig.h
//
#ifndef MY_CONFIG_H
#define MY_CONFIG_H



#ifdef INIDEF_MEGA2560

#ifdef INIDEF_MEGA2560_ETHERNET
#include <Ethernet.h>
#undef ETH_CLK_MODE // From Lilygo github example.
#define ETH_CLK_MODE        ETH_CLOCK_GPIO17_OUT
#define ETH_POWER_PIN       5
#endif

#endif

// enc1 and IO pin numbers defined in tcmDesigner
#define CONF_EXTRA_ENC2_TCW // TCWRotaryEncoder2
#define CONF_EXTRA_ENC3_CCW // CCWRotaryEncoder3


#endif // ifndef for include only once.

// END_OF_FILE
