/*
 *
 *  Created on: Jan, 2017
 *      Author: federico.corradi@inilabs.com
 */

#include <time.h>
#include "meanratefilter_dvs.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/buffers.h"
#include "ext/colorjet/colorjet.h"

struct MRFilter_state {
	caerDeviceHandle eventSourceDeviceHandle;
	sshsNode eventSourceConfigNode;
	simple2DBufferFloat frequencyMap;
	simple2DBufferLong spikeCountMap;
	int8_t subSampleBy;
	int32_t colorscaleMax;
	int32_t colorscaleMin;
	float targetFreq;
	double measureMinTime;
	double measureStartedAt;
	bool startedMeas;
	bool doSetFreq;
	struct timespec tstart;
	struct timespec tend;
};

typedef struct MRFilter_state *MRFilterState;

static bool caerMeanRateFilterInit(caerModuleData moduleData);
static void caerMeanRateFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerMeanRateFilterConfig(caerModuleData moduleData);
static void caerMeanRateFilterExit(caerModuleData moduleData);
static void caerMeanRateFilterReset(caerModuleData moduleData, uint16_t resetCallSourceID);
static bool allocateFrequencyMap(MRFilterState state, int16_t sourceID);
static bool allocateSpikeCountMap(MRFilterState state, int16_t sourceID);

static struct caer_module_functions caerMeanRateFilterFunctions = { .moduleInit = &caerMeanRateFilterInit, .moduleRun =
	&caerMeanRateFilterRun, .moduleConfig = &caerMeanRateFilterConfig, .moduleExit = &caerMeanRateFilterExit,
	.moduleReset = &caerMeanRateFilterReset };

void caerMeanRateFilterDVS(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket *freqplot) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "MeanRateFilterDVS", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerMeanRateFilterFunctions, moduleData, sizeof(struct MRFilter_state), 2, polarity, freqplot);
}

static bool caerMeanRateFilterInit(caerModuleData moduleData) {
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "colorscaleMax", 150);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "colorscaleMin", 0);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "targetFreq", 100);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "measureMinTime", 0.3f);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doSetFreq", false);

	MRFilterState state = moduleData->moduleState;

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	// internals
	state->startedMeas = false;
	state->measureStartedAt = 0.0f;
	state->measureMinTime = sshsNodeGetFloat(moduleData->moduleNode, "measureMinTime");

	// Nothing that can fail here.
	return (true);
}

static void caerMeanRateFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket);
	caerFrameEventPacket *freqplot = va_arg(args, caerFrameEventPacket*);

	// Only process packets with content.
	if (polarity == NULL) {
		return;
	}

	MRFilterState state = moduleData->moduleState;

	int sourceID = caerEventPacketHeaderGetEventSource(&polarity->packetHeader);
	sshsNode sourceInfoNodeCA = caerMainloopGetSourceInfo(sourceID);
	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	if (!sshsNodeAttributeExists(sourceInfoNode, "dataSizeX", SSHS_SHORT)) { //to do for visualizer change name of field to a more generic one
		sshsNodePutShort(sourceInfoNode, "dataSizeX", sshsNodeGetShort(sourceInfoNodeCA, "dvsSizeX"));
		sshsNodePutShort(sourceInfoNode, "dataSizeY", sshsNodeGetShort(sourceInfoNodeCA, "dvsSizeY"));
	}

	// If the map is not allocated yet, do it.
	if (state->frequencyMap == NULL) {
		if (!allocateFrequencyMap(state, caerEventPacketHeaderGetEventSource(&polarity->packetHeader))) {
			// Failed to allocate memory, nothing to do.
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for frequencyMap.");
			return;
		}
	}
	// If the map is not allocated yet, do it.
	if (state->spikeCountMap == NULL) {
		if (!allocateSpikeCountMap(state, caerEventPacketHeaderGetEventSource(&polarity->packetHeader))) {
			// Failed to allocate memory, nothing to do.
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for spikeCountMap.");
			return;
		}
	}

	// if not measuring, let's start
	if (state->startedMeas == false) {
		clock_gettime(CLOCK_MONOTONIC, &state->tstart);
		state->measureStartedAt = (double) state->tstart.tv_sec + 1.0e-9 * state->tstart.tv_nsec;
		state->startedMeas = true;
	}

	//sshsNode sourceInfoNode = caerMainloopGetSourceInfo(caerEventPacketHeaderGetEventSource(&polarity->packetHeader));

	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "dataSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "dataSizeY");

	// get current time
	clock_gettime(CLOCK_MONOTONIC, &state->tend);
	double now = ((double) state->tend.tv_sec + 1.0e-9 * state->tend.tv_nsec);
	// if we measured for enough time..
	if (state->measureMinTime <= (now - state->measureStartedAt)) {

		//caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "\nfreq measurement completed.\n");
		state->startedMeas = false;

		//update frequencyMap
		for (size_t x = 0; x < sizeX; x++) {
			for (size_t y = 0; y < sizeY; y++) {
				if (state->measureMinTime > 0) {
					state->frequencyMap->buffer2d[x][y] = (float) state->spikeCountMap->buffer2d[x][y]
						/ (float) state->measureMinTime;
				}
				//reset
				state->spikeCountMap->buffer2d[x][y] = 0;
			}
		}

		// set the biases if asked
		if (state->doSetFreq) {
			//TODO: NOT IMPLEMENTED
		}
	}

	// update filter parameters
	caerMeanRateFilterConfig(moduleData);

	// Iterate over events and update frequency
	CAER_POLARITY_ITERATOR_VALID_START(polarity)
	// Get values on which to operate.
		int64_t ts = caerPolarityEventGetTimestamp64(caerPolarityIteratorElement, polarity);
		uint16_t x = caerPolarityEventGetX(caerPolarityIteratorElement);
		uint16_t y = caerPolarityEventGetY(caerPolarityIteratorElement);

		// Update value into maps 
		state->spikeCountMap->buffer2d[x][y] += 1;
	CAER_POLARITY_ITERATOR_VALID_END

	// put info into frame
	*freqplot = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, sizeX, sizeY, 3);
	if (*freqplot != NULL) {
		caerFrameEvent singleplot = caerFrameEventPacketGetEvent(*freqplot, 0);

#ifdef DVS128
		uint32_t counter = 0;
		for (size_t x = 0; x < sizeX; x++) {
			for (size_t y = 0; y < sizeY; y++) {
				COLOUR col = GetColour((double) state->frequencyMap->buffer2d[y][x], state->colorscaleMin, state->colorscaleMax);
				singleplot->pixels[counter] = (uint16_t) ( (int)(col.r*65535));			// red
				singleplot->pixels[counter + 1] = (uint16_t) ( (int)(col.g*65535));// green
				singleplot->pixels[counter + 2] = (uint16_t) ( (int)(col.b*65535) );// blue
				counter += 3;
			}
		}
#else
		uint32_t counter = 0;
		for (size_t x = 0; x < sizeY; x++) {
			for (size_t y = 0; y < sizeX; y++) {
				COLOUR col = GetColour((double) state->frequencyMap->buffer2d[y][x], state->colorscaleMin,
					state->colorscaleMax);
				singleplot->pixels[counter] = (uint16_t) ((int) (col.r * 65535));			// red
				singleplot->pixels[counter + 1] = (uint16_t) ((int) (col.g * 65535));		// green
				singleplot->pixels[counter + 2] = (uint16_t) ((int) (col.b * 65535));		// blue
				counter += 3;
			}
		}
#endif

		//add info to the frame
		caerFrameEventSetLengthXLengthYChannelNumber(singleplot, sizeX, sizeY, 3, *freqplot);
		//validate frame
		caerFrameEventValidate(singleplot, *freqplot);
	}

}

static void caerMeanRateFilterConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	MRFilterState state = moduleData->moduleState;

	state->colorscaleMax = sshsNodeGetInt(moduleData->moduleNode, "colorscaleMax");
	state->colorscaleMin = sshsNodeGetInt(moduleData->moduleNode, "colorscaleMin");
	state->targetFreq = sshsNodeGetFloat(moduleData->moduleNode, "targetFreq");
	state->measureMinTime = sshsNodeGetFloat(moduleData->moduleNode, "measureMinTime");
	state->doSetFreq = sshsNodeGetBool(moduleData->moduleNode, "doSetFreq");

}

static void caerMeanRateFilterExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	MRFilterState state = moduleData->moduleState;

	// Ensure maps are freed.
	simple2DBufferFreeFloat(state->frequencyMap);
	simple2DBufferFreeLong(state->spikeCountMap);
}

static void caerMeanRateFilterReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);

	MRFilterState state = moduleData->moduleState;

	// Reset maps to all zeros (startup state).
	simple2DBufferResetLong(state->spikeCountMap);
	simple2DBufferResetFloat(state->frequencyMap);
}

static bool allocateSpikeCountMap(MRFilterState state, int16_t sourceID) {
	// Get size information from source.
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo(U16T(sourceID));
	if (sourceInfoNode == NULL) {
		// This should never happen, but we handle it gracefully.
		caerLog(CAER_LOG_ERROR, __func__, "Failed to get source info to allocate map.");
		return (false);
	}

	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "dataSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "dataSizeY");

	state->spikeCountMap = simple2DBufferInitLong((size_t) sizeX, (size_t) sizeY);
	if (state->spikeCountMap == NULL) {
		return (false);
	}

	for (size_t x = 0; x < sizeX; x++) {
		for (size_t y = 0; y < sizeY; y++) {
			state->spikeCountMap->buffer2d[x][y] = 0; // init to zero
		}
	}

	return (true);
}

static bool allocateFrequencyMap(MRFilterState state, int16_t sourceID) {
	// Get size information from source.
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo(U16T(sourceID));
	if (sourceInfoNode == NULL) {
		// This should never happen, but we handle it gracefully.
		caerLog(CAER_LOG_ERROR, __func__, "Failed to get source info to allocate map.");
		return (false);
	}

	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "dataSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "dataSizeY");

	state->frequencyMap = simple2DBufferInitFloat((size_t) sizeX, (size_t) sizeY);
	if (state->frequencyMap == NULL) {
		return (false);
	}

	for (size_t x = 0; x < sizeX; x++) {
		for (size_t y = 0; y < sizeY; y++) {
			state->frequencyMap->buffer2d[x][y] = 0.0f; // init to zero
		}
	}

	return (true);
}

