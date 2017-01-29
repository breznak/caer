/*
 *
 *  Created on: Jan , 2017
 *      Author: federico.corradi@inilabs.com
 */

#ifndef DVSTODYNAPSE_H_
#define DVSTODYNAPSE_H_

#include "main.h"
#include "modules/ini/dynapse_common.h"

#include <libcaer/devices/dynapse.h>
#include <libcaer/events/spike.h>
#include <libcaer/events/polarity.h>

void caerDvsToDynapse(uint16_t moduleID, int16_t eventSourceID, caerSpikeEventPacket spike, caerPolarityEventPacket polarity);

#endif /* DVSTODYNAPSE_H_ */
