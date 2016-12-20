#ifndef STEREOMATCHING_H_
#define STEREOMATCHING_H_

#include "main.h"

#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>

void caerStereoMatching(uint16_t moduleID, caerFrameEventPacket frame_0, caerFrameEventPacket frame_1);

#endif /* STEREOMATCHING_H_ */
