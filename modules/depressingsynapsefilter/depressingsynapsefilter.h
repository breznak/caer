/*
 * depressingsynapsefilter.h
 *
 *  Created on: April 2017
 *      Author: Tianyu
 */

#ifndef DEPRESSINGSYNAPSEFILTER_H_
#define DEPRESSINGSYNAPSEFILTER_H_

#include "main.h"

#include <libcaer/events/polarity.h>

void caerDepressingSynapseFilter(uint16_t moduleID, caerPolarityEventPacket polarity);

#endif /* DEPRESSINGSYNAPSEFILTER_H_ */
