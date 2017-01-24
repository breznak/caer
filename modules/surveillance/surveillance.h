/*
 * backgroundactivityfilter.h
 *
 *  Created on: Jan  2017
 *      Author: Tianyu
 */

#ifndef SURVEILLANCE_H_
#define SURVEILLANCE_H_

#include "main.h"

#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>

void caerSurveillanceFilter(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame);

#endif /* SURVEILLANCE_H_ */
