#ifndef AMBIENT_SERVICE_CONFIG_H__
#define AMBIENT_SERVICE_CONFIG_H__

#include "board_config.h"

#if AMBIENT_SERVICE_ENABLED == 1

/***************** TEMP *********************/
#define TEMP_INITIAL_CONFIG         0b11110000										  //Initial configuration of the TEMP sensor. Disabled, 0.5Hz.
#define TEMP_DEBUG                  1												  //Enables TEMP RTT debug


/***************** PR *********************/
#define PR_INITIAL_CONFIG           0b11110000										  //Initial configuration of the PR sensor. Disabled, 0.5Hz.
#define PR_DEBUG                    1												  //Enables PR RTT debug


/***************** HUM *********************/
#define HUM_INITIAL_CONFIG          0b11110000										  //Initial configuration of the PR sensor. Disabled, 0.5Hz.
#define HUM_DEBUG                   1												  //Enables HUM RTT debug

#endif /*AMBIENT_SERVICE_ENABLED*/

#endif /* AMBIENT_SERVICE_CONFIG_H__ */
