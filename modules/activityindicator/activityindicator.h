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
#include <libcaer/events/frame.h>
#define STATUSCHAR	1024

typedef enum {Verylow, low, median, high} activityLevel;
struct activity_results {
	char stringValue[STATUSCHAR];
	long activityValue;
};

typedef struct activity_results *AResults;

AResults caerActivityIndicator(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame);

#endif /* ACTIVITYINDICATOR_H_ */
