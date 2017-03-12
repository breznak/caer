/*
 * effectivetransferfunction.c
 *
 *  Created on: Feb 2017 - http://www.nature.com/articles/srep14730
 *      Author: federico
 */

#include "effectivetransferfunction.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/buffers.h"

struct ETFFilter_state {
	// user settings
	bool doMeasurement;
	int chipId;
	bool init;
	int numSteps;
	int stepnum;
	//collect data from all cores
	float sum[50][DYNAPSE_X4BOARD_COREX][DYNAPSE_X4BOARD_COREY];
	float mean[50][DYNAPSE_X4BOARD_COREX][DYNAPSE_X4BOARD_COREY];
	float var[50][DYNAPSE_X4BOARD_COREX][DYNAPSE_X4BOARD_COREY];
	// usb utils
	caerInputDynapseState eventSourceModuleState;
	simple2DBufferFloat ETFMapFreq;
	simple2DBufferLong ETFMapSpike;
};

typedef struct ETFFilter_state *ETFFilterState;

static bool caerEffectiveTransferFunctionInit(caerModuleData moduleData);
static void caerEffectiveTransferFunctionRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerEffectiveTransferFunctionConfig(caerModuleData moduleData);
static void caerEffectiveTransferFunctionExit(caerModuleData moduleData);
static void caerEffectiveTransferFunctionReset(caerModuleData moduleData, uint16_t resetCallSourceID);
void caerDynapseSetBias(caerInputDynapseState state, uint32_t chipId, uint32_t coreId, const char *biasName_t,
	uint8_t coarseValue, uint16_t fineValue, const char *lowHigh, const char *npBias);

static bool allocateETFMapSpikes(ETFFilterState state, int16_t xsize, int16_t ysize);
static bool allocateETFMapFreq(ETFFilterState state, int16_t xsize, int16_t ysize);

static struct caer_module_functions caerEffectiveTransferFunctionFunctions = { .moduleInit =
	&caerEffectiveTransferFunctionInit, .moduleRun = &caerEffectiveTransferFunctionRun, .moduleConfig =
	&caerEffectiveTransferFunctionConfig, .moduleExit = &caerEffectiveTransferFunctionExit, .moduleReset =
	&caerEffectiveTransferFunctionReset };

caerPoint4DEventPacket caerEffectiveTransferFunction(uint16_t moduleID, caerSpikeEventPacket spike) {

	caerPoint4DEventPacket ETFData = NULL;

	caerModuleData moduleData = caerMainloopFindModule(moduleID, "EffectiveTransferFunction", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return (NULL);
	}

	caerModuleSM(&caerEffectiveTransferFunctionFunctions, moduleData, sizeof(struct ETFFilter_state), 2, spike,
		&ETFData);

	return (ETFData);
}

static bool caerEffectiveTransferFunctionInit(caerModuleData moduleData) {
	// create parameters
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doMeasurement", false);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "chipId", 0);

	ETFFilterState state = moduleData->moduleState;

	// update node state
	state->doMeasurement = sshsNodeGetBool(moduleData->moduleNode, "doMeasurement");
	state->chipId = sshsNodeGetInt(moduleData->moduleNode, "chipId");
	state->init = false;
	state->stepnum = 0;

	// init
	for (size_t x = 0; x < DYNAPSE_X4BOARD_COREX; x++) {
		for (size_t y = 0; y < DYNAPSE_X4BOARD_COREY; y++) {
			state->sum[state->stepnum][x][y] = 0.0f;
			state->mean[state->stepnum][x][y] = 0.0f;
			state->var[state->stepnum][x][y] = 0.0f;
		}
	}

	// Add config listeners last - let's the user interact with the parameter -
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	// Nothing that can fail here.
	return (true);
}

static void caerEffectiveTransferFunctionRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerSpikeEventPacket spike = va_arg(args, caerSpikeEventPacket);
	caerPoint4DEventPacket *ETFData = va_arg(args, caerPoint4DEventPacket *);

	// Only process packets with content.
	if (spike == NULL) {
		return;
	}

	ETFFilterState state = moduleData->moduleState;
	// first find out which one is the module producing the spikes. and get usb handle
	// --- start  usb handle / from spike event source id
	int sourceID = caerEventPacketHeaderGetEventSource(&spike->packetHeader);
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(sourceID));
	/*state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(sourceID));*/
	if (state->eventSourceModuleState == NULL) {
		return;
	}
	caerInputDynapseState stateSource = state->eventSourceModuleState;
	if (stateSource->deviceState == NULL) {
		return;
	}
	// --- end usb handle

	// If the map is not allocated yet, do it.
	if (state->ETFMapFreq == NULL) {
		if (!allocateETFMapFreq(state, DYNAPSE_CONFIG_XCHIPSIZE, DYNAPSE_CONFIG_YCHIPSIZE)) {
			// Failed to allocate memory, nothing to do.
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for frequencyMap.");
			return;
		}
	}
	// If the map is not allocated yet, do it.
	if (state->ETFMapSpike == NULL) {
		if (!allocateETFMapSpikes(state, DYNAPSE_CONFIG_XCHIPSIZE, DYNAPSE_CONFIG_YCHIPSIZE)) {
			// Failed to allocate memory, nothing to do.
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for spikeCountMap.");
			return;
		}
	}

	// if false do init
	if (!state->init) {
		/*
		 * do init... */
		//int bits_chipU0[DYNAPSE_MAX_USER_USB_PACKET_SIZE];
		//int counter = 0;
		if (state->chipId == DYNAPSE_CONFIG_DYNAPSE_U0 || state->chipId == DYNAPSE_CONFIG_DYNAPSE_U1
			|| state->chipId == DYNAPSE_CONFIG_DYNAPSE_U2 || state->chipId == DYNAPSE_CONFIG_DYNAPSE_U3) {
			// is chip id is valid
			caerDeviceConfigSet(state->eventSourceModuleState->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID,
				(uint32_t) state->chipId);
			// set biases for input spikes
			for (uint32_t coreId = 0; coreId < 4; coreId++) {
				caerDynapseSetBias(stateSource, (uint32_t) state->chipId, coreId, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
				caerDynapseSetBias(stateSource, (uint32_t) state->chipId, coreId, "IF_AHTHR_N", 7, 1, "HighBias", "NBias");
				caerDynapseSetBias(stateSource, (uint32_t) state->chipId, coreId, "IF_AHW_P", 7, 1, "HighBias", "PBias");
				caerDynapseSetBias(stateSource, (uint32_t) state->chipId, coreId, "IF_BUF_P", 3, 80, "HighBias", "PBias");
				caerDynapseSetBias(stateSource, (uint32_t) state->chipId, coreId, "IF_CASC_N", 7, 1, "HighBias", "NBias");
				caerDynapseSetBias(stateSource, (uint32_t) state->chipId, coreId, "IF_DC_P", 7, 2, "HighBias", "PBias");
				caerDynapseSetBias(stateSource, (uint32_t) state->chipId, coreId, "IF_NMDA_N", 7, 1, "HighBias", "PBias");
				caerDynapseSetBias(stateSource, (uint32_t) state->chipId, coreId, "IF_RFR_N", 0, 108, "HighBias", "NBias");
				caerDynapseSetBias(stateSource, (uint32_t) state->chipId, coreId, "IF_TAU1_N", 6, 24, "LowBias", "NBias");
				caerDynapseSetBias(stateSource, (uint32_t) state->chipId, coreId, "IF_TAU2_N", 5, 15, "HighBias", "NBias");
				caerDynapseSetBias(stateSource, (uint32_t) state->chipId, coreId, "IF_THR_N", 3, 20, "HighBias", "NBias");
				caerDynapseSetBias(stateSource, (uint32_t) state->chipId, coreId, "NPDPIE_TAU_F_P", 5, 41, "HighBias",
					"PBias");
				caerDynapseSetBias(stateSource, (uint32_t) state->chipId, coreId, "NPDPIE_TAU_S_P", 7, 40, "HighBias",
					"NBias");
				caerDynapseSetBias(stateSource, (uint32_t) state->chipId, coreId, "NPDPIE_THR_F_P", 2, 200, "HighBias",
					"PBias");
				caerDynapseSetBias(stateSource, (uint32_t) state->chipId, coreId, "NPDPIE_THR_S_P", 7, 0, "HighBias",
					"PBias");
				caerDynapseSetBias(stateSource, (uint32_t) state->chipId, coreId, "NPDPII_TAU_F_P", 7, 40, "HighBias",
					"NBias");
				caerDynapseSetBias(stateSource, (uint32_t) state->chipId, coreId, "NPDPII_TAU_S_P", 7, 40, "HighBias",
					"NBias");
				caerDynapseSetBias(stateSource, (uint32_t) state->chipId, coreId, "NPDPII_THR_F_P", 7, 40, "HighBias",
					"PBias");
				caerDynapseSetBias(stateSource, (uint32_t) state->chipId, coreId, "NPDPII_THR_S_P", 7, 40, "HighBias",
					"PBias");
				caerDynapseSetBias(stateSource, (uint32_t) state->chipId, coreId, "PS_WEIGHT_EXC_F_N", 0, 216, "HighBias",
					"NBias");
				caerDynapseSetBias(stateSource, (uint32_t) state->chipId, coreId, "PS_WEIGHT_EXC_S_N", 7, 1, "HighBias",
					"NBias");
				caerDynapseSetBias(stateSource, (uint32_t) state->chipId, coreId, "PS_WEIGHT_INH_F_N", 7, 1, "HighBias",
					"NBias");
				caerDynapseSetBias(stateSource, (uint32_t) state->chipId, coreId, "PS_WEIGHT_INH_S_N", 7, 1, "HighBias",
					"NBias");
				caerDynapseSetBias(stateSource, (uint32_t) state->chipId, coreId, "PULSE_PWLK_P", 0, 43, "HighBias",
					"PBias");
				caerDynapseSetBias(stateSource, (uint32_t) state->chipId, coreId, "R2R_P", 4, 85, "HighBias", "PBias");
			}
			caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Started clearing cam..");
			uint32_t bits[DYNAPSE_CONFIG_NUMNEURONS * DYNAPSE_X4BOARD_NEUX];
			int numConfig = -1;
			for (uint32_t neuronId = 0; neuronId < DYNAPSE_CONFIG_NUMNEURONS; neuronId++) {
				numConfig = -1;
				for (uint32_t camId = 0; camId < DYNAPSE_X4BOARD_NEUX; camId++) {
					if (camId == 0) {
						numConfig++;
						bits[numConfig] = caerDynapseGenerateCamBits(5, neuronId, camId, 3);
					}
					else {
						numConfig++;
						bits[numConfig] = caerDynapseGenerateCamBits(0, neuronId, camId, 0);
					}
				}
				// send data with libusb host transfer in packet
				if (!caerDynapseSendDataToUSB(stateSource->deviceState, bits, numConfig)) {
					caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "USB transfer failed");
				}
			}
			caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "CAM cleared successfully.");
		}
		else {
			//
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
				"Invalid chip Id, please choose one among 0,4,8,12");
		}
		// init done
		state->init = true;
	}
	long int storeMeasure = false;

	if (atomic_load(&state->eventSourceModuleState->genSpikeState.ETFphase_num) != state->stepnum) {
		storeMeasure = true;
		state->stepnum = atomic_load(&state->eventSourceModuleState->genSpikeState.ETFphase_num);	// from thread
	}

	// store measure and reset counts
	// not the first time as we need to measure for at least a while
	if (storeMeasure && (state->stepnum != 0)) {

		caerLog(CAER_LOG_NOTICE, __func__, "ETF storeMeasure");

		//update frequencyMap
		float phaseDur = atomic_load(
			&stateSource->genSpikeState.ETFduration) / stateSource->genSpikeState.ETFstepnum;
		//caerLog(CAER_LOG_NOTICE, __func__, "Phase Duration %f\n", phaseDur);
		for (int16_t x = 0; x < DYNAPSE_CONFIG_XCHIPSIZE; x++) {
			for (int16_t y = 0; y < DYNAPSE_CONFIG_YCHIPSIZE; y++) {
				// update freq map
				state->ETFMapFreq->buffer2d[x][y] = (float) state->ETFMapSpike->buffer2d[x][y] / phaseDur;

				//reset
				state->ETFMapSpike->buffer2d[x][y] = 0;
			}
		}

		//float max_freq = 0.0f;
		//loop over all cores
		for (int16_t x = 0; x < DYNAPSE_CONFIG_XCHIPSIZE; x++) {
			for (int16_t y = 0; y < DYNAPSE_CONFIG_YCHIPSIZE; y++) {

				int corex = 0;
				int corey = 0;
				if (x < 16 && y < 16) {
					corex = 0;
					corey = 0;
				}
				else if (x < 16 && y >= 16) {
					corex = 0;
					corey = 1;
				}
				else if (x >= 16 && y < 16) {
					corex = 1;
					corey = 0;
				}
				else if (x >= 16 && y >= 16) {
					corex = 1;
					corey = 1;
				}

				state->sum[state->stepnum][corex][corey] += state->ETFMapFreq->buffer2d[x][y]; //Hz

			}
		}

		for (int16_t x = 0; x < DYNAPSE_CONFIG_XCHIPSIZE; x++) {
			for (int16_t y = 0; y < DYNAPSE_CONFIG_YCHIPSIZE; y++) {

				int corex = 0;
				int corey = 0;
				if (x < 16 && y < 16) {
					corex = 0;
					corey = 0;
				}
				else if (x < 16 && y >= 16) {
					corex = 0;
					corey = 1;
				}
				else if (x >= 16 && y < 16) {
					corex = 1;
					corey = 0;
				}
				else if (x >= 16 && y >= 16) {
					corex = 1;
					corey = 1;
				}

				//calculate mean
				state->mean[state->stepnum][corex][corey] = state->sum[state->stepnum][corex][corey]
					/ (float) DYNAPSE_CONFIG_NUMNEURONS_CORE;

				float f = (state->ETFMapFreq->buffer2d[x][y]) - state->mean[state->stepnum][corex][corey];
				state->var[state->stepnum][corex][corey] += f * f;

			}
		}

		// now the measurement has finished, save results
		if (state->stepnum == stateSource->genSpikeState.ETFstepnum) {

			// allocate 4d event if null
			if (*ETFData == NULL) {
				*ETFData = caerPoint4DEventPacketAllocate(
					(DYNAPSE_X4BOARD_COREY / 2) * (DYNAPSE_X4BOARD_COREX / 2) * stateSource->genSpikeState.ETFstepnum,
					(int16_t) sourceID, NULL);
				caerMainloopFreeAfterLoop(&free, *ETFData);
			}
			// get last event
			int numspikes = caerEventPacketHeaderGetEventNumber(&spike->packetHeader);
			caerSpikeEvent last_ev = caerSpikeEventPacketGetEvent(spike, numspikes);
			int ts = 0;
			if (last_ev == NULL) {
				ts = 1;
			}
			else {
				ts = caerSpikeEventGetTimestamp(last_ev);
			}
			// fill 4d events
			int counterEvs = 0;
			for (size_t corex = 0; corex < DYNAPSE_X4BOARD_COREX / 2; corex++) {
				for (size_t corey = 0; corey < DYNAPSE_X4BOARD_COREY / 2; corey++) {
					for (int numS = 0; numS < stateSource->genSpikeState.ETFstepnum; numS++) {
						// print mean and variance
						caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString,
							"\nStep Num %d - mean[%d][%d] = %f Hz -  var[%d][%d] = %f \n", numS, (int) corex,
							(int) corey, (double) state->mean[numS][corex][corey], (int) corex, (int) corey,
							(double) state->var[numS][corex][corey]);

						// set timestamp for 4d event
						caerPoint4DEvent evt = caerPoint4DEventPacketGetEvent(*ETFData, counterEvs);
						// ts as last ts
						caerPoint4DEventSetTimestamp(evt, ts);
						caerPoint4DEventSetX(evt, (float) corex);
						caerPoint4DEventSetY(evt, (float) corey);
						caerPoint4DEventSetZ(evt, (float) state->mean[numS][corex][corey]);
						caerPoint4DEventSetW(evt, (float) state->var[numS][corex][corey]);

						// validate event
						caerPoint4DEventValidate(evt, *ETFData);
						counterEvs++;
					}
				}
			}

			// reset
			for (size_t x = 0; x < DYNAPSE_X4BOARD_COREX; x++) {
				for (size_t y = 0; y < DYNAPSE_X4BOARD_COREY; y++) {
					for (int step = 0; step < 10; step++) {
						state->sum[step][x][y] = 0.0f;
						state->mean[step][x][y] = 0.0f;
						state->var[step][x][y] = 0.0f;
					}
				}
			}

		}

	}

	// Iterate over spikes in the packet
	CAER_SPIKE_ITERATOR_VALID_START(spike)
	//int32_t timestamp = caerSpikeEventGetTimestamp(caerSpikeIteratorElement);
		uint8_t chipid = caerSpikeEventGetChipID(caerSpikeIteratorElement);
		uint32_t neuronid = caerSpikeEventGetNeuronID(caerSpikeIteratorElement);
		uint8_t coreid = caerSpikeEventGetSourceCoreID(caerSpikeIteratorElement);

		int chipToMonitor = 0;
		if (stateSource->genSpikeState.chip_id == 0) {
			chipToMonitor = 1;	// DYNAPSE_U0 comes back with id 1 (zero not allowed in sram)
		}
		else {
			chipToMonitor = stateSource->genSpikeState.chip_id;
		}
		if (chipToMonitor == chipid) {
			// neuron id
			neuronid = ((uint32_t) coreid * 256) + neuronid;
			//convert linear index to 2d
			int x = neuronid % 32;
			int y = neuronid / 32;

			state->ETFMapSpike->buffer2d[x][y] += 1;
		}CAER_SPIKE_ITERATOR_VALID_END

	// update parameters
	// this will update parameters, from user input
	state->doMeasurement = sshsNodeGetBool(moduleData->moduleNode, "doMeasurement");
	if (state->doMeasurement != atomic_load(&stateSource->genSpikeState.doStim)) {
		if (state->doMeasurement) {
			atomic_store(&stateSource->genSpikeState.ETFdone, false); // we just started
			atomic_store(&stateSource->genSpikeState.ETFstarted, true);
			atomic_store(&stateSource->genSpikeState.stim_type, 11); //STIM_ETF
			atomic_store(&stateSource->genSpikeState.doStim, state->doMeasurement);	// pass it to the thread
		}
		else {
			atomic_store(&stateSource->genSpikeState.doStim, state->doMeasurement);
			atomic_store(&stateSource->genSpikeState.ETFdone, true); // we stop
			atomic_store(&stateSource->genSpikeState.ETFstarted, false);
		}
	}

}

static void caerEffectiveTransferFunctionConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	//ETFFilterState state = moduleData->moduleState;

}

static void caerEffectiveTransferFunctionExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	//ETFFilterState state = moduleData->moduleState;

	sshsNodePutBool(moduleData->moduleNode, "doMeasurement", false);

}

static void caerEffectiveTransferFunctionReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);

	ETFFilterState state = moduleData->moduleState;

}

static bool allocateETFMapFreq(ETFFilterState state, int16_t xsize, int16_t ysize) {
	// Get size information from source.

	int16_t sizeX = xsize;
	int16_t sizeY = ysize;

	state->ETFMapFreq = simple2DBufferInitFloat((size_t) sizeX, (size_t) sizeY);
	if (state->ETFMapFreq == NULL) {
		return (false);
	}

	for (int16_t x = 0; x < sizeX; x++) {
		for (int16_t y = 0; y < sizeY; y++) {
			state->ETFMapFreq->buffer2d[x][y] = 0.0f; // init to zero
		}
	}

	return (true);
}

static bool allocateETFMapSpikes(ETFFilterState state, int16_t xsize, int16_t ysize) {
	// Get size information from source.

	int16_t sizeX = xsize;
	int16_t sizeY = ysize;

	state->ETFMapSpike = simple2DBufferInitLong((size_t) sizeX, (size_t) sizeY);
	if (state->ETFMapSpike == NULL) {
		return (false);
	}

	for (int16_t x = 0; x < sizeX; x++) {
		for (int16_t y = 0; y < sizeY; y++) {
			state->ETFMapSpike->buffer2d[x][y] = 0; // init to zero
		}
	}

	return (true);
}
