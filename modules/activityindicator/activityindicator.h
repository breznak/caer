/*
 * activityindicator.h
 *
 *  Created on: Feb  2017
 *      Author: Tianyu
 */

#ifndef ACTIVITYINDICATOR_H_
#define ACTIVITYINDICATOR_H_

#include "main.h"

#include <libcaer/events/polarity.h>

void caerActivityIndicator(uint16_t moduleID, caerPolarityEventPacket polarity);

#endif /* ACTIVITYINDICATOR_H_ */
