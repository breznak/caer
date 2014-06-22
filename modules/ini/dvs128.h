/*
 * dvs128.h
 *
 *  Created on: Nov 26, 2013
 *      Author: chtekk
 */

#ifndef DVS128_H_
#define DVS128_H_

#include "main.h"
#include "events/polarity.h"
#include "events/special.h"

#define DVS128_VID 0x152A
#define DVS128_PID 0x8400
#define DVS128_DID_TYPE 0x00

#define DVS128_ARRAY_SIZE_X 128
#define DVS128_ARRAY_SIZE_Y 128

#define DATA_ENDPOINT 0x86

#define VENDOR_REQUEST_START_TRANSFER 0xB3
#define VENDOR_REQUEST_STOP_TRANSFER 0xB4
#define VENDOR_REQUEST_SEND_BIASES 0xB8

void caerInputDVS128(uint16_t moduleID, caerPolarityEventPacket *polarity, caerSpecialEventPacket *special);

#endif /* DVS128_H_ */
