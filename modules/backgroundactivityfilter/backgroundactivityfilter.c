/*
 * backgroundactivityfilter.c
 *
 *  Created on: Jan 20, 2014
 *      Author: chtekk
 */

#include "backgroundactivityfilter.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/buffers.h"

struct BAFilter_state {
	simple2DBufferLong timestampMap;
	int32_t deltaT;
	int8_t subSampleBy;
};

typedef struct BAFilter_state *BAFilterState;

static bool caerBackgroundActivityFilterInit(caerModuleData moduleData);
static void caerBackgroundActivityFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerBackgroundActivityFilterConfig(caerModuleData moduleData);
static void caerBackgroundActivityFilterExit(caerModuleData moduleData);
static void caerBackgroundActivityFilterReset(caerModuleData moduleData, uint16_t resetCallSourceID);
static bool allocateTimestampMap(BAFilterState state, int16_t sourceID);

static struct caer_module_functions caerBackgroundActivityFilterFunctions = { .moduleInit =
	&caerBackgroundActivityFilterInit, .moduleRun = &caerBackgroundActivityFilterRun, .moduleConfig =
	&caerBackgroundActivityFilterConfig, .moduleExit = &caerBackgroundActivityFilterExit, .moduleReset =
	&caerBackgroundActivityFilterReset };

void caerBackgroundActivityFilter(uint16_t moduleID, caerPolarityEventPacket polarity) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "BAFilter", PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerBackgroundActivityFilterFunctions, moduleData, sizeof(struct BAFilter_state), 1, polarity);
}

static bool caerBackgroundActivityFilterInit(caerModuleData moduleData) {
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "deltaT", 30000);
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "subSampleBy", 0);

	BAFilterState state = moduleData->moduleState;

	state->deltaT = sshsNodeGetInt(moduleData->moduleNode, "deltaT");
	state->subSampleBy = sshsNodeGetByte(moduleData->moduleNode, "subSampleBy");

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	// Nothing that can fail here.
	return (true);
}

static void caerBackgroundActivityFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket);

	// Only process packets with content.
	if (polarity == NULL) {
		return;
	}

	BAFilterState state = moduleData->moduleState;

	// If the map is not allocated yet, do it.
	if (state->timestampMap == NULL) {
		if (!allocateTimestampMap(state, caerEventPacketHeaderGetEventSource(&polarity->packetHeader))) {
			// Failed to allocate memory, nothing to do.
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for timestampMap.");
			return;
		}
	}

	// Iterate over events and filter out ones that are not supported by other
	// events within a certain region in the specified timeframe.
	CAER_POLARITY_ITERATOR_VALID_START(polarity)
		// Get values on which to operate.
		int64_t ts = caerPolarityEventGetTimestamp64(caerPolarityIteratorElement, polarity);
		uint16_t x = caerPolarityEventGetX(caerPolarityIteratorElement);
		uint16_t y = caerPolarityEventGetY(caerPolarityIteratorElement);

		// Apply sub-sampling.
		x = U16T(x >> state->subSampleBy);
		y = U16T(y >> state->subSampleBy);

		// Get value from map.
		int64_t lastTS = state->timestampMap->buffer2d[x][y];

		if ((I64T(ts - lastTS) >= I64T(state->deltaT)) || (lastTS == 0)) {
			// Filter out invalid.
			caerPolarityEventInvalidate(caerPolarityIteratorElement, polarity);
		}

		// Update neighboring region.
		size_t sizeMaxX = (state->timestampMap->sizeX - 1);
		size_t sizeMaxY = (state->timestampMap->sizeY - 1);

		if (x > 0) {
			state->timestampMap->buffer2d[x - 1][y] = ts;
		}
		if (x < sizeMaxX) {
			state->timestampMap->buffer2d[x + 1][y] = ts;
		}

		if (y > 0) {
			state->timestampMap->buffer2d[x][y - 1] = ts;
		}
		if (y < sizeMaxY) {
			state->timestampMap->buffer2d[x][y + 1] = ts;
		}

		if (x > 0 && y > 0) {
			state->timestampMap->buffer2d[x - 1][y - 1] = ts;
		}
		if (x < sizeMaxX && y < sizeMaxY) {
			state->timestampMap->buffer2d[x + 1][y + 1] = ts;
		}

		if (x > 0 && y < sizeMaxY) {
			state->timestampMap->buffer2d[x - 1][y + 1] = ts;
		}
		if (x < sizeMaxX && y > 0) {
			state->timestampMap->buffer2d[x + 1][y - 1] = ts;
		}
	CAER_POLARITY_ITERATOR_VALID_END
}

static void caerBackgroundActivityFilterConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	BAFilterState state = moduleData->moduleState;

	state->deltaT = sshsNodeGetInt(moduleData->moduleNode, "deltaT");
	state->subSampleBy = sshsNodeGetByte(moduleData->moduleNode, "subSampleBy");
}

static void caerBackgroundActivityFilterExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	BAFilterState state = moduleData->moduleState;

	// Ensure map is freed.
	simple2DBufferFreeLong(state->timestampMap);
}

static void caerBackgroundActivityFilterReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);

	BAFilterState state = moduleData->moduleState;

	// Reset timestamp map to all zeros (startup state).
	simple2DBufferResetLong(state->timestampMap);
}

static bool allocateTimestampMap(BAFilterState state, int16_t sourceID) {
	// Get size information from source.
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo(U16T(sourceID));
	if (sourceInfoNode == NULL) {
		// This should never happen, but we handle it gracefully.
		caerLog(CAER_LOG_ERROR, __func__, "Failed to get source info to allocate timestamp map.");
		return (false);
	}

	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "dvsSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "dvsSizeY");

	state->timestampMap = simple2DBufferInitLong((size_t) sizeX, (size_t) sizeY);
	if (state->timestampMap == NULL) {
		return (false);
	}

	// TODO: size the map differently if subSampleBy is set!
	return (true);
}
