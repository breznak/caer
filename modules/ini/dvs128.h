/*
 * dvs128.h
 *
 *  Created on: Nov 26, 2013
 *      Author: chtekk
 */

#ifndef DVS128_H_
#define DVS128_H_

#include "main.h"

#include <libcaer/events/packetContainer.h>
#include <libcaer/events/special.h>
#include <libcaer/events/polarity.h>

#ifdef HAVE_PTHREADS
#include "ext/c11threads_posix.h"
#endif

#include <math.h>
#include <stdatomic.h>
#include <sys/types.h>

#include <libcaer/devices/dvs128.h>

struct caer_input_dvs128_state {
	caerDeviceHandle deviceState;
	sshsNode eventSourceConfigNode;
};

typedef struct caer_input_dvs128_state *caerInputDVSState;

caerEventPacketContainer caerInputDVS128(uint16_t moduleID);

#endif /* DVS128_H_ */
