/*
 *
 *  Created on: Dec, 2016
 *      Author: federico.corradi@inilabs.com
 */

#include "meanratefilter.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/buffers.h"
#include "libcaer/devices/dynapse.h"

struct MRFilter_state {
	sshsNode eventSourceModuleState;
	sshsNode eventSourceConfigNode;
	simple2DBufferLong timestampMap;
	simple2DBufferFloat frequencyMap;
	int8_t subSampleBy;
	int32_t colorscaleMax;
	int32_t colorscaleMin;
};

typedef struct {
    double r,g,b;
} COLOUR;

typedef struct MRFilter_state *MRFilterState;

static bool caerMeanRateFilterInit(caerModuleData moduleData);
static void caerMeanRateFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerMeanRateFilterConfig(caerModuleData moduleData);
static void caerMeanRateFilterExit(caerModuleData moduleData);
static void caerMeanRateFilterReset(caerModuleData moduleData, uint16_t resetCallSourceID);
static bool allocateFrequencyMap(MRFilterState state, int16_t sourceID);
static bool allocateTimestampMap(MRFilterState state, int16_t sourceID);
COLOUR GetColour(double v, double vmin, double vmax);

static struct caer_module_functions caerMeanRateFilterFunctions = { .moduleInit =
	&caerMeanRateFilterInit, .moduleRun = &caerMeanRateFilterRun, .moduleConfig =
	&caerMeanRateFilterConfig, .moduleExit = &caerMeanRateFilterExit, .moduleReset =
	&caerMeanRateFilterReset };

void caerMeanRateFilter(uint16_t moduleID,  int16_t eventSourceID, caerSpikeEventPacket spike, caerFrameEventPacket *freqplot) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "MRFilter", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerMeanRateFilterFunctions, moduleData, sizeof(struct MRFilter_state), 3, eventSourceID, spike, freqplot);
}

static bool caerMeanRateFilterInit(caerModuleData moduleData) {
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "subSampleBy", 0);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "colorscaleMax", 500);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "colorscaleMin", 0);

	MRFilterState state = moduleData->moduleState;

	state->subSampleBy = sshsNodeGetByte(moduleData->moduleNode, "subSampleBy");

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	if (!sshsNodeAttributeExists(sourceInfoNode, "apsSizeX", SSHS_SHORT)) { //to do for visualizer change name of field to a more generic one
		sshsNodePutShort(sourceInfoNode, "apsSizeX", DYNAPSE_X4BOARD_NEUY); 
		sshsNodePutShort(sourceInfoNode, "apsSizeY", DYNAPSE_X4BOARD_NEUY);
	}

	// Nothing that can fail here.
	return (true);
}

static void caerMeanRateFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	int16_t eventSourceID = va_arg(args, int16_t);
	caerSpikeEventPacket spike = va_arg(args, caerSpikeEventPacket);
	caerFrameEventPacket *freqplot = va_arg(args, caerFrameEventPacket*);

	// Only process packets with content.
	if (spike == NULL) {
		return;
	}

	MRFilterState state = moduleData->moduleState;

	// example to get the USB handle
	// please consider also that we are passing source module id
	// as argument to the filter
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(eventSourceID));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(eventSourceID));
	caerInputDynapseState stateSource = state->eventSourceModuleState;
	// one could now use the state for changing biases
	caerDeviceConfigSet(stateSource, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U2);
	//generate random value for injection current
	uint32_t random_number = rand() % 255 + 1;
	struct caer_dynapse_info dynapse_info = caerDynapseInfoGet(stateSource->deviceState);
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "%s --- ID: %d, Master: %d,  Logic: %d,  ChipID: %d.\n",
		dynapse_info.deviceString, dynapse_info.deviceID, dynapse_info.deviceIsMaster, dynapse_info.logicVersion,
		dynapse_info.chipID);


	//update parameters
	caerMeanRateFilterConfig(moduleData);

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
		state->frequencyMap->buffer2d[y][x] = ((float)ts - state->timestampMap->buffer2d[y][x])/2.0;
		state->timestampMap->buffer2d[y][x] = ts;

	CAER_SPIKE_ITERATOR_VALID_END

	sshsNode sourceInfoNode = caerMainloopGetSourceInfo(caerEventPacketHeaderGetEventSource(&spike->packetHeader));
	uint16_t sizeX =sshsNodeGetShort(sourceInfoNode, "dataSizeX");
	uint16_t sizeY =sshsNodeGetShort(sourceInfoNode, "dataSizeY");

	// put info into frame
	*freqplot = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, sizeX, sizeY, 3);
	if (*freqplot != NULL) {
		caerFrameEvent singleplot = caerFrameEventPacketGetEvent(*freqplot, 0);

		uint32_t counter = 0;
		for (size_t i = 0; i < sizeX; i++) {
			for (size_t ys = 0; ys < sizeY; ys++) {

				COLOUR col  = GetColour((double) state->frequencyMap->buffer2d[i][ys]/1000.0, state->colorscaleMin, state->colorscaleMax);

				singleplot->pixels[counter] = (uint16_t) ( (int)(col.r*65535));			// red
				singleplot->pixels[counter + 1] = (uint16_t) ( (int)(col.g*65535));		// green
				singleplot->pixels[counter + 2] = (uint16_t) ( (int)(col.b*65535) );		// blue
				counter += 3;

			}
		}

		//add info to the frame
		caerFrameEventSetLengthXLengthYChannelNumber(singleplot, sizeX, sizeY, 3, *freqplot);
		//validate frame
		caerFrameEventValidate(singleplot, *freqplot);
	}

}

static void caerMeanRateFilterConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	MRFilterState state = moduleData->moduleState;

	state->subSampleBy = sshsNodeGetByte(moduleData->moduleNode, "subSampleBy");
	state->colorscaleMax = sshsNodeGetInt(moduleData->moduleNode, "colorscaleMax");
	state->colorscaleMin = sshsNodeGetInt(moduleData->moduleNode, "colorscaleMin");

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

COLOUR GetColour(double v,double vmin,double vmax)
{
   COLOUR c = {1.0,1.0,1.0}; // white
   double dv;

   if (v < vmin)
      v = vmin;
   if (v > vmax)
      v = vmax;
   dv = vmax - vmin;

   if (v < (vmin + 0.25 * dv)) {
      c.r = 0;
      c.g = 4 * (v - vmin) / dv;
   } else if (v < (vmin + 0.5 * dv)) {
      c.r = 0;
      c.b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
   } else if (v < (vmin + 0.75 * dv)) {
      c.r = 4 * (v - vmin - 0.5 * dv) / dv;
      c.b = 0;
   } else {
      c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
      c.b = 0;
   }

   return(c);
}
