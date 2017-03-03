/*
 *  spatialBandpassFilter.h
 *
 *  Copyright May 13, 2006 Tobi Delbruck, Inst. of Neuroinformatics, UNI-ETH Zurich
 *
 *  Created on: 2016
 *  @author Tobi Delbruck
 *  @author lnguyen
 */

#ifndef SPATIALBANDPASSFILTER_H_
#define SPATIALBANDPASSFILTER_H_

#include "main.h"

#include <libcaer/events/polarity.h>

/**
 * Does an event-based spatial high pass filter, so that only small objects pass
 * through.
 */
void caerSpatialBandPassFilter(uint16_t moduleID, caerPolarityEventPacket polarity);

#endif /* SPATIALBANDPASSFILTER_H_ */
