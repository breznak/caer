/*
 * mediantrckerfilter.h
 *
 *  Created on: Jan  2017
 *      Author: Tianyu
 */

#ifndef MEANFILTER_H_
#define MEANFILTER_H_

#include "main.h"

#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>

void caerMeanfilterFilter(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame);

#endif /* MEANFILTER_H_ */
