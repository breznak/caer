#ifndef OPENCVOPTICFLOW_H_
#define OPENCVOPTICFLOW_H_

#include "main.h"

#include <libcaer/events/frame.h>

caerFrameEventPacket caerOpticFlow(uint16_t moduleID, caerFrameEventPacket frameInput);

#endif /* OPENCVOPTICFLOW_H_ */
