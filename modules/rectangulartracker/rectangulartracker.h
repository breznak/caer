/*
 * rectangulartracker.h
 *
 *  Created on: Jan  2017
 *      Author: Tianyu
 */

#ifndef RECTANGULARTRACKER_H_
#define RECTANGULARTRACKER_H_

#include "main.h"

#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>

void caerRectangulartrackerFilter(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame);

#endif /* RECTANGULARTRACKER_H_ */
