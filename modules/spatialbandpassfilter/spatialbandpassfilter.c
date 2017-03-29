/*
 *  spatialBandpassFilter.c
 *
 *  Copyright May 13, 2006 Tobi Delbruck, Inst. of Neuroinformatics, UNI-ETH Zurich
 *
 *  Created on: 2016
 *  @author Tobi Delbruck
 *  @author lnguyen
 */

/**
 * nice description of spatial band-pass filters: http://ict.udlap.mx/people/oleg/docencia/IMAGENES/chapter3/image_321_IS548.html
 * this implementation of SBPF will: 
 *	- given time window dtSurround (us) and Neighborhood (as surrounding of shape "O" with range= (centerRadius, surroundRadius> )
 *	- if an event happens within (time now - dtSurround, time now) in the Neighborhood, then
 * 	- invalidate all events in the center (radius=(0, centerRadius>) 
 */

#include <modules/spatialbandpassfilter/spatialbandpassfilter.h>
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/buffers.h"

#include <math.h>
#include <stdlib.h>
#include <assert.h>

struct SBPFilter_state {
	/**
	 * the time in timestamp ticks (1us at present) that a spike in surround
	 * will inhibit a spike from center passing through.
	 */
	uint64_t dtSurround;

	// radius of surrounding region, computed as: Center~~~(centerRadius-----------------surroundRadius>......
	uint16_t centerRadius;//aka max distance for center, aka min distance of surround radius
	uint16_t surroundRadius;// aka max distance of surround radius

	simple2DBufferLong surroundTimestamps;

	uint16_t sizeX_SBPF;
	uint16_t sizeY_SBPF;

};
typedef struct SBPFilter_state *SBPFilterState;

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

	caerSpatialBandPassFilterConfig(moduleData);
	//caerLog(CAER_LOG_WARNING, "spatial", "dtSurround: %zd", state->dtSurround);

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

		// if the event occurred too close after a surround spike don't pass it. -> invalidate all evts in center
		if (deltaT < I64T(state->dtSurround)) {
			// Filter out invalid in center
			for (int offsetCenterX = -state->centerRadius; offsetCenterX <= state->centerRadius; offsetCenterX++) {
				for (int offsetCenterY = -state->centerRadius; offsetCenterY <= state->centerRadius; offsetCenterY++) {
					// skip those out-of-bounds
					int cx = x + offsetCenterX;
					int cy = y + offsetCenterY;
					if (cx < 0 || cx > state->sizeX_SBPF || cy < 0 || cy > state->sizeY_SBPF) {
						continue;
					} else { //invalidate
					
						caerPolarityEventInvalidate(caerPolarityIteratorElement, polarity);
		}
				}
			}
		} //end if-deltaT

		// update surround & let evt pass
		for (int offsetSurroundX = -state->surroundRadius; offsetSurroundX <= state-> surroundRadius; offsetSurroundX++) {
			for (int offsetSurroundY = -state->surroundRadius; offsetSurroundY <= state->surroundRadius; offsetSurroundY ++) {
				int sx = x + offsetSurroundX;
				int sy = y + offsetSurroundY;
				if (sx <0 || sx > state->sizeX_SBPF || sy < 0 || sy > state->sizeY_SBPF) { //out of bounds
					continue;
				} else if (abs(offsetSurroundX) <= state->centerRadius && abs(offsetSurroundY) <= state->centerRadius) { //within center -> skip
					continue;
				} else { // inside neighborhood -> update timestamps
					state->surroundTimestamps->buffer2d[sx][sy] = ts;
				}
			}
		} //end outer for
	CAER_POLARITY_ITERATOR_VALID_END
}

static void caerSpatialBandPassFilterConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	SBPFilterState state = moduleData->moduleState;

	state->centerRadius = sshsNodeGetInt(moduleData->moduleNode, "centerRadius");
	state->surroundRadius = sshsNodeGetInt(moduleData->moduleNode, "surroundRadius");
	assert(state->centerRadius < state->surroundRadius);

	state->dtSurround = sshsNodeGetInt(moduleData->moduleNode, "dtSurround");
	assert(state->dtSurround > 0); 
}

static void caerSpatialBandPassFilterExit(caerModuleData moduleData) {
	SBPFilterState state = moduleData->moduleState;

	// Ensure map is freed.
	simple2DBufferFreeLong(       state->surroundTimestamps);
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

	return (true);
}
