/*
 * mediantrckerfilter.h
 *
 *  Created on: Jan  2017
 *      Author: Tianyu
 */

#ifndef MEDIANTRACKER_H_
#define MEDIANTRACKER_H_

#include "main.h"

#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>

void caerMediantrackerFilter(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame);

#endif /* MEDIANTRACKER_H_ */
