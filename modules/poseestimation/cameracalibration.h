#ifndef POSEESTIMATION_H_
#define POSEESTIMATION_H_

#include "main.h"

#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>

void caerPoseCalibration(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame);

#endif /* POSEESTIMATION_H_ */
