/*
 * trainingfromcaffe.h
 *
 *  Created on: March, 2017
 *      Author: federico.corradi@inilabs.com
 */

#ifndef TRAININGFROMCAFFE_H_
#define TRAININGFROMCAFFE_H_

#include "main.h"
#include "modules/ini/dynapse_common.h"


void caerTrainingFromCaffeFilter(uint16_t moduleID, caerSpikeEventPacket spike,  int groupId);

#endif /* TRAININGFROMCAFFE_H_ */
