#ifndef DYNAPSE_FX2_H_
#define DYNAPSE_FX2_H_

#include "main.h"

#include <libcaer/libcaer.h>
#include <libcaer/devices/dynapse.h>
#include <libcaer/events/packetContainer.h>
#include <libcaer/events/spike.h>
#include <libcaer/events/special.h> // reset events etc..

caerEventPacketContainer caerInputDYNAPSEFX2(uint16_t moduleID);

#endif /* DYNAPSE_FX2_H_ */
