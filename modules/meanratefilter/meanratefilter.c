/*
 *
 *  Created on: Dec, 2016
 *      Author: federico.corradi@inilabs.com
 */

#include <time.h>
#include "meanratefilter.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/buffers.h"
#include "libcaer/devices/dynapse.h"

struct MRFilter_state {
	sshsNode eventSourceModuleState;
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
	struct timespec tstart;			//struct is defined in gen_spike.c
	struct timespec tend;
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
static bool allocateSpikeCountMap(MRFilterState state, int16_t sourceID);
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
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "colorscaleMax", 500);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "colorscaleMin", 0);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "targetFreq", 100);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "measureMinTime", 3.0);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doSetFreq", false);

	MRFilterState state = moduleData->moduleState;

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	if (!sshsNodeAttributeExists(sourceInfoNode, "dataSizeX", SSHS_SHORT)) { //to do for visualizer change name of field to a more generic one
		sshsNodePutShort(sourceInfoNode, "dataSizeX", DYNAPSE_X4BOARD_NEUX);
		sshsNodePutShort(sourceInfoNode, "dataSizeY", DYNAPSE_X4BOARD_NEUY);
	}

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
	int16_t eventSourceID = va_arg(args, int16_t);
	caerSpikeEventPacket spike = va_arg(args, caerSpikeEventPacket);
	caerFrameEventPacket *freqplot = va_arg(args, caerFrameEventPacket*);

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
	if (state->spikeCountMap == NULL) {
		if (!allocateSpikeCountMap(state, caerEventPacketHeaderGetEventSource(&spike->packetHeader))) {
			// Failed to allocate memory, nothing to do.
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for spikeCountMap.");
			return;
		}
	}

	// --- start  usb handle / from spike event source id
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(eventSourceID));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(eventSourceID));
	caerInputDynapseState stateSource = state->eventSourceModuleState;
	struct caer_dynapse_info dynapse_info = caerDynapseInfoGet(stateSource->deviceState);
	// --- end usb handle

	// if not measuring, let's start
	if( state->startedMeas == false ){
		clock_gettime(CLOCK_MONOTONIC, &state->tstart);
		state->measureStartedAt = (double) state->tstart.tv_sec + 1.0e-9 * state->tstart.tv_nsec;
		state->startedMeas = true;
	}

	sshsNode sourceInfoNode = caerMainloopGetSourceInfo(caerEventPacketHeaderGetEventSource(&spike->packetHeader));

	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "dataSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "dataSizeY");

	// get current time
	clock_gettime(CLOCK_MONOTONIC, &state->tend);
	double now = ((double) state->tend.tv_sec + 1.0e-9 * state->tend.tv_nsec);
	// if we measured for enough time..
	if( state->measureMinTime <= (now - state->measureStartedAt) ){

		//caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "\nfreq measurement completed.\n");
		state->startedMeas = false;

		//update frequencyMap
		for (size_t x = 0; x < sizeX; x++) {
			for (size_t y = 0; y < sizeY; y++) {
				if(state->measureMinTime > 0){
					state->frequencyMap->buffer2d[x][y] = (float)state->spikeCountMap->buffer2d[x][y]/(float)state->measureMinTime;
				}
				//reset
				state->spikeCountMap->buffer2d[x][y] = 0;
			}
		}

		// set the biases if asked
		if(state->doSetFreq){

			//collect data for all cores
			float sum[DYNAPSE_X4BOARD_COREX][DYNAPSE_X4BOARD_COREY];
			float mean[DYNAPSE_X4BOARD_COREX][DYNAPSE_X4BOARD_COREY];
			float var[DYNAPSE_X4BOARD_COREX][DYNAPSE_X4BOARD_COREY];
			// init
			for(size_t x=0; x<DYNAPSE_X4BOARD_COREX; x++){
				for(size_t y=0; y<DYNAPSE_X4BOARD_COREY; y++){
					sum[x][y] = 0.0f;
					mean[x][y] = 0.0f;
					var[x][y] = 0.0f;
				}
			}
			float max_freq = 0.0f;
			//loop over all cores
			for(size_t corex=0; corex<DYNAPSE_X4BOARD_COREX; corex++){
				for(size_t corey=0; corey<DYNAPSE_X4BOARD_COREY; corey++){
					max_freq = 0.0f;
					//get sum from core
					for(size_t x=0+(corex*DYNAPSE_CONFIG_NEUROW);x<DYNAPSE_CONFIG_NEUROW+(corex*DYNAPSE_CONFIG_NEUROW);x++){
						for(size_t y=0+(corey*DYNAPSE_CONFIG_NEUCOL);y<DYNAPSE_CONFIG_NEUCOL+(corey*DYNAPSE_CONFIG_NEUCOL);y++){
							sum[corex][corey] += state->frequencyMap->buffer2d[x][y]; //Hz
							if(max_freq < state->frequencyMap->buffer2d[x][y]){
								max_freq = state->frequencyMap->buffer2d[x][y];
							}
						}
					}
					//calculate mean
					mean[corex][corey] = sum[corex][corey]/(float)DYNAPSE_CONFIG_NUMNEURONS_CORE;
					//calculate variance
					for(size_t x=0+(corex*DYNAPSE_CONFIG_NEUROW);x<DYNAPSE_CONFIG_NEUROW+(corex*DYNAPSE_CONFIG_NEUROW);x++){
						for(size_t y=0+(corey*DYNAPSE_CONFIG_NEUCOL);y<DYNAPSE_CONFIG_NEUCOL+(corey*DYNAPSE_CONFIG_NEUCOL);y++){
							float f = (state->frequencyMap->buffer2d[x][y]) - mean[corex][corey];
							var[corex][corey] += f*f;
						}
					}
					caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString,
							"\nmean[%d][%d] = %f Hz var[%d][%d] = %f  max_freq %f\n",
							corex, corey, mean[corex][corey], corex, corey, var[corex][corey], max_freq);
				}
			}

			// now decide how to change the bias setting
			for(size_t corex=0; corex<DYNAPSE_X4BOARD_COREX; corex++){
				for(size_t corey=0; corey<DYNAPSE_X4BOARD_COREY; corey++){

					int chipid;
					int coreid;

					// which chip/core should we address ?
					if(corex < 2 && corey < 2 ){
						chipid = DYNAPSE_CONFIG_DYNAPSE_U0;
						coreid = corex << 1 | corey;
					}else if(corex < 2 && corey >= 2 ){
						chipid = DYNAPSE_CONFIG_DYNAPSE_U2;
						coreid = corex << 1 | (corey-2);
					}else if(corex >= 2 && corey < 2){
						chipid = DYNAPSE_CONFIG_DYNAPSE_U1;
						coreid = (corex-2) << 1 | corey;
					}else if(corex >= 2 && corey >= 2){
						chipid = DYNAPSE_CONFIG_DYNAPSE_U3;
						coreid = (corex-2) << 1 | (corey-2) ;
					}

					caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString,
						"\nmean[%d][%d] = %f Hz var[%d][%d] = %f chipid = %d coreid %d\n",
						corex, corey, mean[corex][corey], corex, corey, var[corex][corey], chipid, coreid);

					// current dc settings
					sshsNode chipNode = sshsGetRelativeNode(state->eventSourceConfigNode,chipIDToName(chipid, true));
				    sshsNode biasNode = sshsGetRelativeNode(chipNode, "bias/");

				    // select right bias name
				    char biasName[] = "C0_IF_DC_P"; // "CX_IF_DC_P" max bias name length is 10
					if(coreid == 0){
						memcpy(biasName, "C0_IF_DC_P", 10);
					}else if(coreid == 1){
						memcpy(biasName, "C1_IF_DC_P", 10);
					}else if(coreid == 2){
						memcpy(biasName, "C2_IF_DC_P", 10);
					}else if(coreid == 3){
						memcpy(biasName, "C3_IF_DC_P", 10);
					}
					// Add trailing slash to node name (required!).
					size_t biasNameLength = strlen(biasName);
					char biasNameFull[biasNameLength + 2];
					memcpy(biasNameFull, biasName, biasNameLength);
					biasNameFull[biasNameLength] = '/';
					biasNameFull[biasNameLength + 1] = '\0';

					// Get biasConfNode for this particular bias.
					sshsNode biasConfigNode = sshsGetRelativeNode(biasNode, biasNameFull);

					// Read current coarse and fine settings.
					uint8_t coarseValue = sshsNodeGetByte(biasConfigNode, "coarseValue");
					uint16_t fineValue = sshsNodeGetShort(biasConfigNode, "fineValue");

					caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString,
											"\n BIAS %s coarse %d fine %d\n",
											biasNameFull, coarseValue, fineValue );

					bool changed = false;
					int step = 15; // fine step value
					// compare current frequency with target
					if( (state->targetFreq - mean[corex][corey]) > 0 ){
						// we need to increase freq.. increase fine
						if(fineValue < (255-step)){
							fineValue = fineValue + step;
							changed = true;
						}else{
							// if we did not reach the max value
							if(coarseValue != 0){
								fineValue = step;
								coarseValue += -1; // coarse 0 is max 7 is min
								changed = true;
							}else{
								caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString,
										"\n Reached Limit for Bias\n");
							}
						}
					}else if( (state->targetFreq - mean[corex][corey]) < 0){
						// we need to reduce freq
						if( (fineValue - step) > 0){
							fineValue = fineValue - step;
							changed = true;
						}else{
							// if we did not reach the max value
							if(coarseValue != 7){
								fineValue = step;
								coarseValue += +1; // coarse 0 is max 7 is min
								changed = true;
							}else{
								caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString,
										"\n Reached Limit for Bias\n");
							}
						}
					}
					if(changed){
						//generate bits to send
						uint32_t bits = generatesBitsCoarseFineBiasSetting(state->eventSourceConfigNode, &dynapse_info,
								biasName, coarseValue, fineValue, "HighBias", "Normal", "PBias", true, chipid);
						//send bits to the usb
						caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chipid);
						caerDeviceConfigSet(((caerInputDynapseState) moduleData->moduleState)->deviceState,
							DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, bits);

					}

				}
			}
		}
	}

	// update filter parameters
	caerMeanRateFilterConfig(moduleData);

	// Iterate over events and update frequency
	CAER_SPIKE_ITERATOR_VALID_START(spike)
		// Get values on which to operate.
		int64_t ts = caerSpikeEventGetTimestamp64(caerSpikeIteratorElement, spike);
		uint16_t x = caerSpikeEventGetX(caerSpikeIteratorElement);
		uint16_t y = caerSpikeEventGetY(caerSpikeIteratorElement);

		// Update value into maps 
		state->spikeCountMap->buffer2d[x][y] += 1;

	CAER_SPIKE_ITERATOR_VALID_END


	// put info into frame
	*freqplot = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, sizeX, sizeY, 3);
	if (*freqplot != NULL) {
		caerFrameEvent singleplot = caerFrameEventPacketGetEvent(*freqplot, 0);

		uint32_t counter = 0;
		for (size_t x = 0; x < sizeX; x++) {
			for (size_t y = 0; y < sizeY; y++) {
				COLOUR col  = GetColour((double) state->frequencyMap->buffer2d[y][x], state->colorscaleMin, state->colorscaleMax);
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

	for(size_t x=0; x<sizeX; x++){
		for(size_t y=0; y<sizeY; y++){
			state->spikeCountMap->buffer2d[x][y] = 0; // init to zero
		}
	}

	// TODO: size the map differently if subSampleBy is set!
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

	for(size_t x=0; x<sizeX; x++){
		for(size_t y=0; y<sizeY; y++){
			state->frequencyMap->buffer2d[x][y] = 0.0f; // init to zero
		}
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
