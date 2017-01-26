/*
 *
 *  Created on: Dec , 2016
 *      Author: federico.corradi@inilabs.com
 */

#ifndef MEANRATEFILTERDVS_H_
#define MEANRATEFILTERDVS_H_

#include "main.h"
#include "modules/ini/dvs128.h"

#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h> //display

#ifdef DVS128
#else
	#error "MEANRATEFILTER_DVS ONLY works for DVS128. TODO: extend stateSource with deviceState for DAVIS and advanced prototype sensors."
#endif

void caerMeanRateFilterDVS(uint16_t moduleID, int16_t eventSourceID, caerPolarityEventPacket polarity, caerFrameEventPacket *freqplot);

#endif /* MEANRATEFILTERDVS_H_ */
