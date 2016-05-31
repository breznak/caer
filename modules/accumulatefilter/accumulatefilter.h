#ifndef ACCUMULATEFILTER_H_
#define ACCUMULATEFILTER_H_

#include "main.h"
#include <libcaer/events/polarity.h>
#include "ext/buffers.h"

typedef enum {POLARITY_ON, POLARITY_OFF, POLARITY_REPLACE, POLARITY_BOTH} polarity_t; // parsed from respective string values in config: "on", "off", "replace", "both"

void caerAccumulateFilter(uint16_t moduleID, caerPolarityEventPacket polarity); // this method processes the data, returns nothing
//these return required type of result, but do not alter the internal state anyhow
simple2DBufferByte caerAccumulateFilterGet2D(uint16_t moduleID);
caerPolarityEventPacket caerAccumulateFilterGetPacket(uint16_t moduleID);
int64_t* caerAccumulateFilterGet1D(uint16_t moduleID);

void caerAccumulateFilterAll(uint16_t moduleID, caerPolarityEventPacket polarity, caerPolarityEventPacket *filtered, simple2DBufferByte *result2d, int64_t* result1d);
#endif
