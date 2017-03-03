/*
 *  refractoryFilter.h
 *
 *  Copyright May 13, 2006 Tobi Delbruck, Inst. of Neuroinformatics, UNI-ETH Zurich
 *
 *  Created on: 2016
 *  @author Tobi Delbruck
 *  @author lnguyen
 */

#ifndef REFRACTORYFILTER_H_
#define REFRACTORYFILTER_H_

#include "main.h"

#include <libcaer/events/polarity.h>

/**
 * Adds a refractory period to pixels so that the events pass only if there is
 * sufficient time delay since the last event from that pixel; ie. it knocks out high
 * firing rates from cells. 
 * The option "passShortISIsEnabled" inverts the logic.
 * redundant events.
 *
 */
void caerRefractoryFilter(uint16_t moduleID, caerPolarityEventPacket polarity);

#endif /* REFRACTORYFILTER_H_ */
