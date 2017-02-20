/*
 * helloworld.h
 *
 *  Created on: Feb 2017 for tutorial on dynap-se
 *      Author: federico
 */

#ifndef HELLOWORLD_H_
#define HELLOWORLD_H_

#include "main.h"
#include "modules/ini/dynapse_common.h"	// useful constants

#include <libcaer/events/spike.h>

void caerHelloWorldModule(uint16_t moduleID, caerSpikeEventPacket spike);

#endif /* HELLOWORLD_H_ */
