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
#include <libcaer/events/frame.h>
#include <libcaer/events/spike.h>
#include "ext/buffers.h"
#include "ext/portable_time.h"

struct TFCFilter_state {
	bool doTraining;
	int32_t freqStim;
	bool init;
	// maps
	simple2DBufferInt group_a;
	simple2DBufferInt group_b;
	simple2DBufferInt group_c;
	simple2DBufferInt group_d;
	simple2DBufferInt tmp;
	float threshold;
	float measureMinTime;
	double measureStartedAt;
	bool startedMeas;
	struct timespec internalTime;
	// usb utils
	caerInputDynapseState eventSourceModuleState;
	sshsNode eventSourceConfigNode;
};
typedef struct TFCFilter_state *TFCFilterState;

void caerTrainingFromCaffeFilter(uint16_t moduleID, caerSpikeEventPacket spike,  int groupId);
void caerTrainFromMakeFrame(uint16_t moduleID, caerFrameEventPacket *grpup_a, caerFrameEventPacket *grpup_b,
	caerFrameEventPacket *grpup_c, caerFrameEventPacket *grpup_d, int size);
static bool allocateActivityMap(TFCFilterState state);
void resetMap_a(caerModuleData moduleData, int size);
void resetMap_b(caerModuleData moduleData, int size);
void resetMap_c(caerModuleData moduleData, int size);
void resetMap_d(caerModuleData moduleData, int size);
bool normalize_buffef_map_sigma(caerModuleData moduleData, int size);

#endif /* TRAININGFROMCAFFE_H_ */
