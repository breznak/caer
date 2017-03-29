/*
 *  spatialBandpassFilter.c
 *
 *  Copyright May 13, 2006 Tobi Delbruck, Inst. of Neuroinformatics, UNI-ETH Zurich
 *
 *  Created on: 2016
 *  @author Tobi Delbruck
 *  @author lnguyen
 */

#include <modules/spatialbandpassfilter/spatialbandpassfilter.h>
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/buffers.h"

#include <math.h>
#include <stdlib.h>

struct SBPFilter_state {
	/**
	 * the time in timestamp ticks (1us at present) that a spike in surround
	 * will inhibit a spike from center passing through.
	 */
	int64_t dtSurround;

	// radius of center
	int16_t centerRadius;

	// radius of surrounding region
	int16_t surroundRadius;

	simple2DBufferLong surroundTimestamps;
	simple2DBufferLong centerTimestamps;

	uint16_t sizeX_SBPF;
	uint16_t sizeY_SBPF;

};
typedef struct SBPFilter_state *SBPFilterState;

// Offset is a relative position
typedef struct {
	int16_t x;		// position x
	int16_t y;		// position y
}Offset;

#define MAX_SURROUND_OFFSETS 8
#define MAX_CENTER_OFFSETS 1


// these arrays hold relative offsets to write to for center and surround timestamp splatts
Offset centerOffsets[MAX_CENTER_OFFSETS], surroundOffsets[MAX_SURROUND_OFFSETS];

// computes an array of offsets that we write to when we getString an event
static void computeOffsets(caerModuleData moduleData);

static bool caerSpatialBandPassFilterInit(caerModuleData moduleData);
static void caerSpatialBandPassFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerSpatialBandPassFilterConfig(caerModuleData moduleData);
static void caerSpatialBandPassFilterExit(caerModuleData moduleData);
static bool allocateSBPFilterTimestampMap(SBPFilterState state, int16_t sourceID);

static struct caer_module_functions caerSpatialBandPassFilterFunctions =
		{ .moduleInit = &caerSpatialBandPassFilterInit, .moduleRun =
				&caerSpatialBandPassFilterRun, .moduleConfig =
				&caerSpatialBandPassFilterConfig, .moduleExit = &caerSpatialBandPassFilterExit };

void caerSpatialBandPassFilter(uint16_t moduleID, caerPolarityEventPacket polarity) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "SBPFilter", CAER_MODULE_PROCESSOR);

	caerModuleSM(&caerSpatialBandPassFilterFunctions, moduleData,
			sizeof(struct SBPFilter_state), 1, polarity);
}

static bool caerSpatialBandPassFilterInit(caerModuleData moduleData) {
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "centerRadius", 0);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "surroundRadius", 1);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "dtSurround", 8000);

	SBPFilterState state = moduleData->moduleState;

	state->centerRadius = sshsNodeGetInt(moduleData->moduleNode, "centerRadius");
	state->surroundRadius = sshsNodeGetInt(moduleData->moduleNode, "surroundRadius");
	state->dtSurround = sshsNodeGetInt(moduleData->moduleNode, "dtSurround");

	//caerLog(CAER_LOG_WARNING, "spatial", "dtSurround: %zd", state->dtSurround);

	computeOffsets(moduleData);

	return (true);
}

// main filter's logic
static void caerSpatialBandPassFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	SBPFilterState state = moduleData->moduleState;

	// Interpret variable arguments (same as above in main function).
	caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket);

	// Only process packets with content.
	if (polarity == NULL) {
		return;
	}

	// If the map is not allocated yet, do it.
	if ( state->surroundTimestamps == NULL) {
		if (!allocateSBPFilterTimestampMap(state, caerEventPacketHeaderGetEventSource(&polarity->packetHeader))) {
			// Failed to allocate memory, nothing to do.
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for timestampMap.");
			return;
		}
	}

	// polarity event number
	int32_t n = caerEventPacketHeaderGetEventNumber(&polarity->packetHeader);

	if (n == 0) {
		return;
	}

	// Iterate over events and filter out ones that are not supported by other
	// events within a certain region in the specified timeframe.
	CAER_POLARITY_ITERATOR_VALID_START(polarity)
			// Get values on which to operate.
		int64_t ts = caerPolarityEventGetTimestamp64(caerPolarityIteratorElement, polarity);
		uint16_t x = caerPolarityEventGetX(caerPolarityIteratorElement);
		uint16_t y = caerPolarityEventGetY(caerPolarityIteratorElement);

		// Get value from map.
		int64_t lastTS = state->surroundTimestamps->buffer2d[x][y];
		int64_t deltaT = ts - lastTS;

		// if the event occurred too close after a surround spike don't pass it.
		if (deltaT < I64T(state->dtSurround)) {
			// Filter out invalid.
			caerPolarityEventInvalidate(caerPolarityIteratorElement, polarity);
		}

		// write surround
		for( int i = 0; i< MAX_SURROUND_OFFSETS ; i++){
			uint16_t kx = x + surroundOffsets[i].x ;
			if (kx < 0 || kx > state->sizeX_SBPF) {
			    continue;
			}
			uint16_t ky = y + surroundOffsets[i].y;
			if (ky < 0 || ky > state->sizeY_SBPF) {
			    continue;
			}
			state->surroundTimestamps->buffer2d[kx][ky] = ts;
		}
	CAER_POLARITY_ITERATOR_VALID_END
}

//private method
void computeOffsets(caerModuleData moduleData) {

	SBPFilterState state = moduleData->moduleState;

	uint16_t i = 0, j = 0 ;
	for (int x = -state->surroundRadius; x <= state->surroundRadius; x++) {  //TODO optimize?
		for (int y = -state->surroundRadius; y <= state->surroundRadius; y++) {

			if ((x <= state->centerRadius && x >= -state->centerRadius
					&& y <= state->centerRadius && y >= -state->centerRadius)) {
				// if we are in center we are not surround
				centerOffsets[j].x = x;
				centerOffsets[j].y = y;
				j++;
			} else {
				surroundOffsets[i].x = x;
				surroundOffsets[i].y = y;
				i++;
			}
		}
	}
}

static void caerSpatialBandPassFilterConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	SBPFilterState state = moduleData->moduleState;

	state->centerRadius = sshsNodeGetInt(moduleData->moduleNode, "centerRadius");
	state->surroundRadius = sshsNodeGetInt(moduleData->moduleNode, "surroundRadius");
	state->dtSurround = sshsNodeGetInt(moduleData->moduleNode, "dtSurround");
}

static void caerSpatialBandPassFilterExit(caerModuleData moduleData) {
	SBPFilterState state = moduleData->moduleState;

	// Ensure map is freed.
	simple2DBufferFreeLong(       state->surroundTimestamps);
	simple2DBufferFreeLong(state->centerTimestamps);
}

static bool allocateSBPFilterTimestampMap(SBPFilterState state, int16_t sourceID) {
	// Get size information from source.
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo((uint16_t) sourceID); //sourcID == chip
	size_t sizeX = sshsNodeGetShort(sourceInfoNode, "dvsSizeX");
	size_t sizeY = sshsNodeGetShort(sourceInfoNode, "dvsSizeY");

	// Assign max ranges for arrays (0 to MAX-1).
	state->sizeX_SBPF = (uint16_t) (sizeX - 1);
	state->sizeY_SBPF = (uint16_t) (sizeY - 1);   // minus 1 to avoid -1 in loop

	// Initialize double-indirection contiguous 2D array, so that array[x][y]
	// is possible, see http://c-faq.com/aryptr/dynmuldimary.html for info.
	state->surroundTimestamps = simple2DBufferInitLong(sizeX, sizeY);
	if (state->surroundTimestamps == NULL) {
		return (false); // Failure.
	}

	state->centerTimestamps = simple2DBufferInitLong(sizeX, sizeY);
	if (state->centerTimestamps == NULL) {
		return (false); // Failure.
	}

	return (true);
}
