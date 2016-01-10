/*
 * imagestreamerfilter.h
 *
 *  Created on: Jan 12, 2016
 *      Author: federico.corradi@inilabs.com, chtekk
 */

#ifndef IMAGESTREAMERFILTER_H_
#define IMAGESTREAMERFILTER_H_

#include "main.h"

#include <libcaer/events/polarity.h>

void caerImageStreamerFilter(uint16_t moduleID, caerPolarityEventPacket polarity);

#endif /* IMAGESTREAMERFILTER_H_ */
