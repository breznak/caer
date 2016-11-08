/*
 * dynapseinit.h
 *
 *  Created on: Nov, 2016
 *      Author: federico.corradi@inilabs.com
 */

#ifndef DYNAPSEINIT_H_
#define DYNAPSEINIT_H_

#include "main.h"
#include "settings.h"

#include <libcaer/events/spike.h>

void caerDynapseInit(uint16_t moduleID, caerSpikeEventPacket spike);

#endif /* DYNAPSEINIT_H_ */
