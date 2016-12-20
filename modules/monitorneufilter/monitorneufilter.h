/*
 *
 *  Created on: Dec , 2016
 *      Author: federico.corradi@inilabs.com
 */

#ifndef MONITORNEUFILTER_H_
#define MONITORNEUFILTER_H_

#include "main.h"
#include "modules/ini/dynapse_common.h"

#include <libcaer/events/spike.h>

void caerMonitorNeuFilter(uint16_t moduleID, int16_t eventSourceID);

#endif /* MONITORNEUFILTER_H_ */
