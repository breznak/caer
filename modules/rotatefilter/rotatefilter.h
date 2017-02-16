/*
 * rotatefilter.h
 *
 *  Created on: Feb  2017
 *      Author: Tianyu
 */

#ifndef ROTATEFILTER_H_
#define ROTATEFILTER_H_

#include "main.h"

#include <libcaer/events/polarity.h>

void caerRotateFilter(uint16_t moduleID, caerPolarityEventPacket polarity);

#endif /* ROTATEFILTER_H_ */
