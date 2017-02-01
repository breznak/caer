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
#include <libcaer/events/frame.h> //display

void caerMeanRateFilter(uint16_t moduleID, caerSpikeEventPacket spike, caerFrameEventPacket *freqplot);

#endif /* MEANRATEFILTER_H_ */
