/*
 *
 *  Created on: Dec, 2016
 *      Author: federico.corradi@inilabs.com
 */

#include "meanratefilter.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/buffers.h"

struct MRFilter_state {
	simple2DBufferLong timestampMap;
	simple2DBufferFloat frequencyMap;
	int32_t deltaT;
	int8_t subSampleBy;
};

typedef struct MRFilter_state *MRFilterState;

static bool caerMeanRateFilterInit(caerModuleData moduleData);
static void caerMeanRateFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerMeanRateFilterConfig(caerModuleData moduleData);
static void caerMeanRateFilterExit(caerModuleData moduleData);
static void caerMeanRateFilterReset(caerModuleData moduleData, uint16_t resetCallSourceID);
static bool allocateFrequencyMap(MRFilterState state, int16_t sourceID);
static bool allocateTimestampMap(MRFilterState state, int16_t sourceID);

static struct caer_module_functions caerMeanRateFilterFunctions = { .moduleInit =
	&caerMeanRateFilterInit, .moduleRun = &caerMeanRateFilterRun, .moduleConfig =
	&caerMeanRateFilterConfig, .moduleExit = &caerMeanRateFilterExit, .moduleReset =
	&caerMeanRateFilterReset };

void caerMeanRateFilter(uint16_t moduleID, caerSpikeEventPacket spike) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "MRFilter", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerMeanRateFilterFunctions, moduleData, sizeof(struct MRFilter_state), 1, spike);
}

static bool caerMeanRateFilterInit(caerModuleData moduleData) {
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "deltaT", 30000);
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "subSampleBy", 0);

	MRFilterState state = moduleData->moduleState;

	state->deltaT = sshsNodeGetInt(moduleData->moduleNode, "deltaT");
	state->subSampleBy = sshsNodeGetByte(moduleData->moduleNode, "subSampleBy");

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	// Nothing that can fail here.
	return (true);
}

static void caerMeanRateFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerSpikeEventPacket spike = va_arg(args, caerSpikeEventPacket);

	// Only process packets with content.
	if (spike == NULL) {
		return;
	}

	MRFilterState state = moduleData->moduleState;

	// If the map is not allocated yet, do it.
	if (state->frequencyMap == NULL) {
		if (!allocateFrequencyMap(state, caerEventPacketHeaderGetEventSource(&spike->packetHeader))) {
			// Failed to allocate memory, nothing to do.
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for frequencyMap.");
			return;
		}
	}
	// If the map is not allocated yet, do it.
	if (state->timestampMap == NULL) {
		if (!allocateTimestampMap(state, caerEventPacketHeaderGetEventSource(&spike->packetHeader))) {
			// Failed to allocate memory, nothing to do.
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for frequencyMap.");
			return;
		}
	}

	// Iterate over events and update frequency
	CAER_SPIKE_ITERATOR_VALID_START(spike)
		// Get values on which to operate.
		int64_t ts = caerSpikeEventGetTimestamp64(caerSpikeIteratorElement, spike);
		uint16_t x = caerSpikeEventGetX(caerSpikeIteratorElement);
		uint16_t y = caerSpikeEventGetY(caerSpikeIteratorElement);

		// Apply sub-sampling.
		x = U16T(x >> state->subSampleBy);
		y = U16T(y >> state->subSampleBy);

		// Update value into maps 
		state->frequencyMap->buffer2d[x][y] = ((float)ts - state->timestampMap->buffer2d[x][y])/2.0;
		state->timestampMap->buffer2d[x][y] = ts;

	CAER_SPIKE_ITERATOR_VALID_END
}

static void caerMeanRateFilterConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	MRFilterState state = moduleData->moduleState;

	state->deltaT = sshsNodeGetInt(moduleData->moduleNode, "deltaT");
	state->subSampleBy = sshsNodeGetByte(moduleData->moduleNode, "subSampleBy");
}

static void caerMeanRateFilterExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	MRFilterState state = moduleData->moduleState;

	// Ensure map is freed.
	simple2DBufferFreeFloat(state->frequencyMap);
}

static void caerMeanRateFilterReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);

	MRFilterState state = moduleData->moduleState;

	// Reset timestamp map to all zeros (startup state).
	simple2DBufferResetLong(state->timestampMap);
	simple2DBufferResetFloat(state->frequencyMap);
}

static bool allocateTimestampMap(MRFilterState state, int16_t sourceID) {
	// Get size information from source.
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo(U16T(sourceID));
	if (sourceInfoNode == NULL) {
		// This should never happen, but we handle it gracefully.
		caerLog(CAER_LOG_ERROR, __func__, "Failed to get source info to timestamp frequency map.");
		return (false);
	}

	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "dataSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "dataSizeY");

	state->timestampMap = simple2DBufferInitLong((size_t) sizeX, (size_t) sizeY);
	if (state->timestampMap == NULL) {
		return (false);
	}

	// TODO: size the map differently if subSampleBy is set!
	return (true);
}

static bool allocateFrequencyMap(MRFilterState state, int16_t sourceID) {
	// Get size information from source.
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo(U16T(sourceID));
	if (sourceInfoNode == NULL) {
		// This should never happen, but we handle it gracefully.
		caerLog(CAER_LOG_ERROR, __func__, "Failed to get source info to allocate frequency map.");
		return (false);
	}

	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "dataSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "dataSizeY");

	state->frequencyMap = simple2DBufferInitFloat((size_t) sizeX, (size_t) sizeY);
	if (state->frequencyMap == NULL) {
		return (false);
	}

	// TODO: size the map differently if subSampleBy is set!
	return (true);
}
