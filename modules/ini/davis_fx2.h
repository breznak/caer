#ifndef DAVIS_FX2_H_
#define DAVIS_FX2_H_

#include "main.h"
#include "events/polarity.h"
#include "events/special.h"
#include "events/frame.h"
#include "events/imu6.h"

#define DAVIS_FX2_VID 0x152A
#define DAVIS_FX2_PID 0x841B
#define DAVIS_FX2_DID_TYPE 0x00

#define VR_CHIP_BIAS 0xC0
#define VR_CHIP_DIAG 0xC1

void caerInputDAVISFX2(uint16_t moduleID, caerPolarityEventPacket *polarity, caerFrameEventPacket *frame,
	caerIMU6EventPacket *imu6, caerSpecialEventPacket *special);

#endif /* DAVIS_FX2_H_ */
