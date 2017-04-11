/*
 * rectangulartracker_dynamic.h
 *
 *  Created on: Jan  2017
 *      Author: Tianyu
 */

#ifndef RECTANGULARTRACKER_H_
#define RECTANGULARTRACKER_H_

#define _USE_MATH_DEFINES
#include <math.h> // defines M_PI

#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>

void caerRectangulartrackerDynamicFilter(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame);

#endif /* RECTANGULARTRACKER_H_ */
