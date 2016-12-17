/*
 *
 *  Created on: Dec , 2016
 *      Author: federico.corradi@inilabs.com
 */

#ifndef MEANRATEFILTER_H_
#define MEANRATEFILTER_H_

#include "main.h"
#include "modules/ini/dynapse_common.h"

#include <libcaer/events/spike.h>

void caerMonitorNeuFilter(uint16_t moduleID, int16_t eventSourceID);

#endif /* MEANRATEFILTER_H_ */
