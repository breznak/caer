#ifndef DAVIS_FX3_H_
#define DAVIS_FX3_H_

#include "main.h"
#include "events/polarity.h"
#include "events/special.h"
#include "events/frame.h"
#include "events/imu6.h"

#define DAVIS_FX3_VID 0x152A
#define DAVIS_FX3_PID 0x841A
#define DAVIS_FX3_DID_TYPE 0x01

#define FPGA_CHIPBIAS 5

#define DEBUG_ENDPOINT 0x81
#define DEBUG_TRANSFER_NUM 4
#define DEBUG_TRANSFER_SIZE 64

void caerInputDAVISFX3(uint16_t moduleID, caerPolarityEventPacket *polarity, caerFrameEventPacket *frame,
	caerIMU6EventPacket *imu6, caerSpecialEventPacket *special);

#endif /* DAVIS_FX3_H_ */
