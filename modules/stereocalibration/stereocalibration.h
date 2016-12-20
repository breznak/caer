#ifndef STEREOCALIBRATION_H_
#define STEREOCALIBRATION_H_

#include "main.h"

#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>

void caerStereoCalibration(uint16_t moduleID, caerFrameEventPacket frame_0, caerFrameEventPacket frame_1);

#endif /* STEREOCALIBRATION_H_ */
