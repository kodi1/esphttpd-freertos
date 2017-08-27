/*
 * debug.h
 *
 *  Created on: Feb 9, 2016
 *      Author: n0ll
 */

#ifndef USER_DEBUG_H_
#define USER_DEBUG_H_

#ifdef DEBUG_MSG
#include <stdio.h>
#define DBG_LOG(format,args...) do {printf("%s:%d " format "\n", __FUNCTION__, __LINE__, ##args);} while (0);
#else
#define DBG_LOG(format,args...) do { } while (0);
#endif

#endif /* USER_DEBUG_H_ */
