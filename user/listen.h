/*
 * packet.h
 *
 *  Created on: Feb 13, 2016
 *      Author: n0ll
 */

#ifndef LISTEN_H_
#define LISTEN_H_

#ifdef FREERTOS
#include "c_types.h"
#endif

#define MAX_DATA_SIZE       (128)

typedef enum {
    ID_NONE,
    ID_DISCOVERY,
    ID_BOARD_GET,
    ID_PWM_SET_PERIOD,
    ID_LED_GET,
    ID_LED_SET,
    ID_REL_GET,
    ID_REL_SET,
    ID_MAX_ID,
} id_t;

#endif /* LISTEN_H_ */
