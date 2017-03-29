/*
 *  refractoryFilter.c
 *
 *  Copyright May 13, 2006 Tobi Delbruck, Inst. of Neuroinformatics, UNI-ETH Zurich
 *
 *  Created on: 2016
 *  @author Tobi Delbruck
 *  @author lnguyen
 */

#include <modules/refractoryfilter/refractoryfilter.h>
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/buffers.h"

#include <math.h>
#include <stdlib.h>
#include <assert.h>

struct RFilter_state {
    /**
	 * the time in timestamp ticks (1us at present) that a spike needs to be
	 * supported by a prior event in the neighborhood by to pass through
	 *
	 * Events with less than this delta time in us are blocked
	 */
	int32_t refractoryPeriodUs;

	/** size X,Y of the camera's resolution */
	uint16_t maxX, maxY;

    /**
	 * the amount to subsample x and y event location by in bit shifts when
	 * writing to past event times map. This effectively increases the range of
	 * support. E.g. setting subSamplingShift to 1 quadruples range because both
	 * x and y are shifted right by one bit
	 *
	 * Past event addresses are subsampled by this many bits in x and y
	 */
	int8_t subsampleBy;

     /** Inverts filtering so that only events with short ISIs are passed through.
         * If refractoryPeriodUs==0, then you can block all events with idential timestamp from the same pixel.
         */
	bool passShortISIsEnabled;

	/** 2D matrix of longs, holds timestamp activations */
	simple2DBufferLong lastTimestamps;

};
typedef struct RFilter_state *RFilterState;

// required methods implementation
static bool caerRefractoryFilterInit(caerModuleData moduleData);
static void caerRefractoryFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerRefractoryFilterConfig(caerModuleData moduleData);
static void caerRefractoryFilterExit(caerModuleData moduleData);
static bool allocateRFilterTimestampMap(RFilterState state, int16_t sourceID);

static struct caer_module_functions caerRefractoryFilterFunctions =
		{ .moduleInit = &caerRefractoryFilterInit, 
	.moduleRun =&caerRefractoryFilterRun, 
	.moduleConfig =&caerRefractoryFilterConfig, 
	.moduleExit = &caerRefractoryFilterExit };

void caerRefractoryFilter(uint16_t moduleID, caerPolarityEventPacket polarity) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "RFilter", CAER_MODULE_PROCESSOR);

	caerModuleSM(&caerRefractoryFilterFunctions, moduleData,
			sizeof(struct RFilter_state), 1, polarity);
}

static bool caerRefractoryFilterInit(caerModuleData moduleData) {
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "refractoryPeriodUs", 586);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "subsampleBy", 0);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "passShortISIsEnabled", true); 


	RFilterState state = moduleData->moduleState;

	state->refractoryPeriodUs = sshsNodeGetInt(moduleData->moduleNode, "refractoryPeriodUs");
	state->subsampleBy = sshsNodeGetInt(moduleData->moduleNode, "subsampleBy");
	state->passShortISIsEnabled = sshsNodeGetBool(moduleData->moduleNode, "passShortISIsEnabled");

	return (true);
}

// logic of the refractory filter
static void caerRefractoryFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	RFilterState state = moduleData->moduleState;

	// Interpret variable arguments (same as above in main function).
	caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket);

	// Only process packets with content.
	if (polarity == NULL) {
		return;
	}

	// If the map is not allocated yet, do it.
	if ( state->lastTimestamps == NULL) {
		if (!allocateRFilterTimestampMap(state, caerEventPacketHeaderGetEventSource(&polarity->packetHeader))) {
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

		assert(x<= state->maxX && x>=0 && y<= state->maxY && y>=0);

		// Apply sub-sampling.
		x = U16T(x >> state->subsampleBy);
		y = U16T(y >> state->subsampleBy);

		// Get value from map.
		int64_t lastTS = state->lastTimestamps->buffer2d[x][y];
		int64_t deltat = (ts - lastTS);
		// if refractoryPeriodUs==0, then all events with ISI==0 pass if passShortISIsEnabled
		bool longISI = (deltat > I64T(state->refractoryPeriodUs)) || (lastTS == 0);

		if( longISI && state->passShortISIsEnabled){
			// Filter out invalid.
			caerPolarityEventInvalidate(caerPolarityIteratorElement, polarity);
		}

		state->lastTimestamps->buffer2d[x][y] = ts;

	CAER_POLARITY_ITERATOR_VALID_END
}


static void caerRefractoryFilterConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	RFilterState state = moduleData->moduleState;

	state->refractoryPeriodUs = sshsNodeGetInt(moduleData->moduleNode, "refractoryPeriodUs");
	state->subsampleBy = sshsNodeGetInt(moduleData->moduleNode, "subsampleBy");
	state->passShortISIsEnabled = sshsNodeGetBool(moduleData->moduleNode, "passShortISIsEnabled"); 
}

static void caerRefractoryFilterExit(caerModuleData moduleData) {
	RFilterState state = moduleData->moduleState;
	simple2DBufferFreeLong(state->lastTimestamps); 
}

static bool allocateRFilterTimestampMap(RFilterState state, int16_t sourceID) {
	// Get size information from source.
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo((uint16_t) sourceID);
	uint16_t sizeX = (uint16_t) sshsNodeGetShort(sourceInfoNode, "dvsSizeX");
	uint16_t sizeY = (uint16_t) sshsNodeGetShort(sourceInfoNode, "dvsSizeY");

	// Initialize double-indirection contiguous 2D array, so that array[x][y]
	// is possible, see http://c-faq.com/aryptr/dynmuldimary.html for info.
        state->lastTimestamps = simple2DBufferInitLong((size_t) sizeX, (size_t) sizeY);
	if (state->lastTimestamps == NULL) {
		return (false); // Failure.
	}

	//also store the sizeX,Y information into state, as a performance optimization, as the operation is quite expensive
	state->maxX = sizeX;
	state->maxY = sizeY;

	return (true);
}
