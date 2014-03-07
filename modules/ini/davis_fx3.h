#ifndef DAVIS_FX3_H_
#define DAVIS_FX3_H_

#include "main.h"
#include "events/polarity.h"
#include "events/special.h"
#include "events/frame.h"
#include "events/imu6.h"

#define DAVIS_FX3_VID 0x152A
#define DAVIS_FX3_PID 0x8410
#define DAVIS_FX3_DID_TYPE 0x00

#define DEBUG_ENDPOINT 0x81
#define DEBUG_TRANSFER_NUM 4
#define DEBUG_TRANSFER_SIZE 64

#define DATA_ENDPOINT 0x82

#define VR_DATA_ENABLE 0xBF
#define VR_CHIP_BIAS 0xC0
#define VR_CHIP_DIAG 0xC1
#define VR_FPGA_SREG 0xC2

void caerInputDAViSFX3(uint16_t moduleID, caerPolarityEventPacket *polarity, caerSpecialEventPacket *special,
	caerFrameEventPacket *frame, caerIMU6EventPacket *imu6);

#endif /* DAVIS_FX3_H_ */
