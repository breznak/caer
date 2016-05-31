#ifndef ACCUMULATEFILTER_H_
#define ACCUMULATEFILTER_H_

#include "main.h"
#include <libcaer/events/polarity.h>

typedef enum {POLARITY_ON, POLARITY_OFF, POLARITY_REPLACE, POLARITY_BOTH} polarity_t; // parsed from respective string values in config: "on", "off", "replace", "both"

void caerAccumulateFilter(uint16_t moduleID, caerPolarityEventPacket polarity);

#endif
