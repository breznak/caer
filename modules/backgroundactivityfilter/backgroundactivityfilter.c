/*
 * backgroundactivityfilter.c
 *
 *  Created on: Jan 20, 2014
 *      Author: chtekk
 */

#include "backgroundactivityfilter.h"
#include "base/mainloop.h"
#include "base/module.h"

struct BAFilter_state {
	uint32_t **timestampMap;
	uint32_t deltaT;
	uint16_t sizeMaxX;
	uint16_t sizeMaxY;
	uint8_t subSampleBy;
};

typedef struct BAFilter_state *BAFilterState;

static bool caerBackgroundActivityFilterInit(caerModuleData moduleData);
static void caerBackgroundActivityFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerBackgroundActivityFilterConfig(caerModuleData moduleData);
static void caerBackgroundActivityFilterExit(caerModuleData moduleData);
static bool allocateTimestampMap(BAFilterState state, uint16_t sourceID);

static struct caer_module_functions caerBackgroundActivityFilterFunctions = { .moduleInit =
	&caerBackgroundActivityFilterInit, .moduleRun = &caerBackgroundActivityFilterRun, .moduleConfig =
	&caerBackgroundActivityFilterConfig, .moduleExit = &caerBackgroundActivityFilterExit };

void caerBackgroundActivityFilter(uint16_t moduleID, caerPolarityEventPacket polarity) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "BAFilter");

	caerModuleSM(&caerBackgroundActivityFilterFunctions, moduleData, sizeof(struct BAFilter_state), 1, polarity);
}

static bool caerBackgroundActivityFilterInit(caerModuleData moduleData) {
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "deltaT", 30000);
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "subSampleBy", 0);

	sshsNodeAddAttrListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	BAFilterState state = moduleData->moduleState;

	state->deltaT = sshsNodeGetInt(moduleData->moduleNode, "deltaT");
	state->subSampleBy = sshsNodeGetByte(moduleData->moduleNode, "subSampleBy");

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
			caerLog(LOG_ERROR, "Failed to allocate memory for BAFilter timestampMap.");
			return;
		}
	}

	// Iterate over events and filter out ones that are not supported by other
	// events within a certain region in the specified timeframe.
	caerPolarityEvent currEvent = NULL;

	for (uint32_t i = 0; i < caerEventPacketHeaderGetEventNumber(&polarity->packetHeader); i++) {
		currEvent = caerPolarityEventPacketGetEvent(polarity, i);

		// Only operate on valid events!
		if (caerPolarityEventIsValid(currEvent)) {
			// Get values on which to operate.
			uint32_t ts = caerPolarityEventGetTimestamp(currEvent);
			uint16_t x = caerPolarityEventGetX(currEvent);
			uint16_t y = caerPolarityEventGetY(currEvent);

			// Apply sub-sampling.
			x >>= state->subSampleBy;
			y >>= state->subSampleBy;

			// Get value from map.
			uint32_t lastTS = state->timestampMap[x][y];

			if ((ts - lastTS) >= state->deltaT || lastTS == 0) {
				// Filter out invalid.
				caerPolarityEventInvalidate(currEvent, polarity);
			}

			// Update neighboring region.
			if (x > 0) {
				state->timestampMap[x - 1][y] = ts;
			}
			if (x < state->sizeMaxX) {
				state->timestampMap[x + 1][y] = ts;
			}

			if (y > 0) {
				state->timestampMap[x][y - 1] = ts;
			}
			if (y < state->sizeMaxY) {
				state->timestampMap[x][y + 1] = ts;
			}

			if (x > 0 && y > 0) {
				state->timestampMap[x - 1][y - 1] = ts;
			}
			if (x < state->sizeMaxX && y < state->sizeMaxY) {
				state->timestampMap[x + 1][y + 1] = ts;
			}

			if (x > 0 && y < state->sizeMaxY) {
				state->timestampMap[x - 1][y + 1] = ts;
			}
			if (x < state->sizeMaxX && y > 0) {
				state->timestampMap[x + 1][y - 1] = ts;
			}
		}
	}
}

static void caerBackgroundActivityFilterConfig(caerModuleData moduleData) {
	caerModuleResetConfigUpdate(moduleData);

	BAFilterState state = moduleData->moduleState;

	state->deltaT = sshsNodeGetInt(moduleData->moduleNode, "deltaT");
	state->subSampleBy = sshsNodeGetByte(moduleData->moduleNode, "subSampleBy");
}

static void caerBackgroundActivityFilterExit(caerModuleData moduleData) {
	BAFilterState state = moduleData->moduleState;

	// Ensure map is freed.
	free(state->timestampMap[0]);
	free(state->timestampMap);
	state->timestampMap = NULL;
}

static bool allocateTimestampMap(BAFilterState state, uint16_t sourceID) {
	// Get size information from source.
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo(sourceID);
	uint16_t sizeX = sshsNodeGetShort(sourceInfoNode, "sizeX");
	uint16_t sizeY = sshsNodeGetShort(sourceInfoNode, "sizeY");

	// Initialize double-indirection contiguous 2D array, so that array[x][y]
	// is possible, see http://c-faq.com/aryptr/dynmuldimary.html for info.
	state->timestampMap = calloc(sizeX, sizeof(uint32_t *));
	if (state->timestampMap == NULL) {
		return (false); // Failure.
	}

	state->timestampMap[0] = calloc((size_t) (sizeX * sizeY), sizeof(uint32_t));
	if (state->timestampMap[0] == NULL) {
		free(state->timestampMap);
		state->timestampMap = NULL;

		return (false); // Failure.
	}

	for (size_t i = 1; i < sizeX; i++) {
		state->timestampMap[i] = state->timestampMap[0] + (i * sizeY);
	}

	// Assign max ranges for arrays (0 to MAX-1).
	state->sizeMaxX = (uint16_t) (sizeX - 1);
	state->sizeMaxY = (uint16_t) (sizeY - 1);

	// TODO: size the map differently if subSampleBy is set!
	return (true);
}
