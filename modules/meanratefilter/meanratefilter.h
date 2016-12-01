/*
 *
 *  Created on: Dec , 2016
 *      Author: federico.corradi@inilabs.com
 */

#ifndef MEANRATEFILTER_H_
#define MEANRATEFILTER_H_

#include "main.h"

#include <libcaer/events/spike.h>

void caerMeanRateFilter(uint16_t moduleID, caerSpikeEventPacket spike);

#endif /* MEANRATEFILTER_H_ */
