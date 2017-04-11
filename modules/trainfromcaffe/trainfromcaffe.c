/*
 * trainfromcaffe.c
 *
 *  Created on: March, 2017
 *      Author: federico.corradi@inilabs.com
 */

#include "trainfromcaffe.h"
#include "base/mainloop.h"
#include "base/module.h"

static bool caerTrainingFromCaffeFilterInit(caerModuleData moduleData);
static void caerTrainingFromCaffeFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerTrainingFromCaffeFilterConfig(caerModuleData moduleData);
static void caerTrainingFromCaffeFilterExit(caerModuleData moduleData);
static void caerTrainingFromCaffeFilterReset(caerModuleData moduleData, uint16_t resetCallSourceID);

static struct caer_module_functions caerTrainingFromCaffeFilterFunctions = { .moduleInit =
	&caerTrainingFromCaffeFilterInit, .moduleRun = &caerTrainingFromCaffeFilterRun, .moduleConfig =
	&caerTrainingFromCaffeFilterConfig, .moduleExit = &caerTrainingFromCaffeFilterExit, .moduleReset =
	&caerTrainingFromCaffeFilterReset };

void caerTrainingFromCaffeFilter(uint16_t moduleID, caerSpikeEventPacket spike, int groupId) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "TrainFromCaffe", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerTrainingFromCaffeFilterFunctions, moduleData, sizeof(struct TFCFilter_state), 2, spike, groupId);
}

static bool caerTrainingFromCaffeFilterInit(caerModuleData moduleData) {
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doTraining", false);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "freqStim", 30);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "threshold", 0.5);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "measureMinTime", 30.0);

	TFCFilterState state = moduleData->moduleState;

	state->doTraining = sshsNodeGetBool(moduleData->moduleNode, "doTraining");
	state->freqStim = sshsNodeGetInt(moduleData->moduleNode, "freqStim");
	state->threshold = sshsNodeGetFloat(moduleData->moduleNode, "threshold");
	state->measureMinTime = sshsNodeGetFloat(moduleData->moduleNode, "measureMinTime");

	state->init = false;

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	state->startedMeas = false;
	state->measureStartedAt = 0.0;
	srand(time(NULL));   // should only be called once

	// Nothing that can fail here.
	return (true);
}

static void caerTrainingFromCaffeFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerSpikeEventPacket spike = va_arg(args, caerSpikeEventPacket);
	int groupId = va_arg(args, int);

	// Only process packets with content.
	if (groupId < 0 || spike == NULL) {
		return;
	}

	TFCFilterState state = moduleData->moduleState;

	// now we can do crazy processing etc..
	// first find out which one is the module producing the spikes. and get usb handle
	// --- start  usb handle / from spike event source id
	int sourceID = caerEventPacketHeaderGetEventSource(&spike->packetHeader);
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(sourceID));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(sourceID));
	if (state->eventSourceModuleState == NULL || state->eventSourceConfigNode == NULL) {
		return;
	}
	caerInputDynapseState stateSource = state->eventSourceModuleState;
	if (stateSource->deviceState == NULL) {
		return;
	}
	// --- end usb handle

	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	if (!sshsNodeAttributeExists(sourceInfoNode, "dataSizeX", SSHS_SHORT)) { //to do for visualizer change name of field to a more generic one
		sshsNodePutShort(sourceInfoNode, "dataSizeX", DYNAPSE_X4BOARD_NEUX);
		sshsNodePutShort(sourceInfoNode, "dataSizeY", DYNAPSE_X4BOARD_NEUY);
	}

	// If the map is not allocated yet, do it.
	if (state->group_a == NULL || state->group_b == NULL || state->group_c == NULL || state->group_d == NULL
		|| state->tmp == NULL) {
		if (!allocateActivityMap(state)) {
			// Failed to allocate memory, nothing to do.
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for activityMaps.");
			return;
		}
	}

	//if we have a results give commands to the spike generator to send different spike trains
	if (state->init == false) {
		// do init , program cams
		atomic_store(&stateSource->genSpikeState.chip_id, 0);
		caerDeviceConfigSet((caerDeviceHandle) stateSource->deviceState,
		DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, (uint32_t) atomic_load(&stateSource->genSpikeState.chip_id)); //select chip
		caerLog(CAER_LOG_NOTICE, __func__, "Programming cam...");
		for (int coreId = 0; coreId < DYNAPSE_CONFIG_NUMCORES; coreId++) {
			for (int neuronId = 0; neuronId < DYNAPSE_CONFIG_NUMNEURONS_CORE; neuronId++) {
				for (int camId = 0; camId < DYNAPSE_CONFIG_NUMCAM; camId++) {

					if (camId < 10) {
						if (caerDynapseWriteCam(stateSource->deviceState, coreId + 1, (coreId << 8) | neuronId, camId,
						DYNAPSE_CONFIG_CAMTYPE_F_EXC)) {
							;
						}
						else {
							caerLog(CAER_LOG_NOTICE, __func__, "cannot program CAM");
						}
					}
					else {
						if (caerDynapseWriteCam(stateSource->deviceState, 0, (coreId << 8) | neuronId, camId,
							DYNAPSE_CONFIG_CAMTYPE_F_INH)) {
							;
						}
						else {
							caerLog(CAER_LOG_NOTICE, __func__, "cannot program CAM");
						}
					}
				}
			}
		}
		caerLog(CAER_LOG_NOTICE, __func__, "Done.");
		caerLog(CAER_LOG_NOTICE, __func__, "Set biases..");
		for (uint32_t coreId = 0; coreId < 4; coreId++) {
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U0, coreId, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U0, coreId, "IF_AHTHR_N", 7, 1, "HighBias", "NBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U0, coreId, "IF_AHW_P", 7, 1, "HighBias", "PBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U0, coreId, "IF_BUF_P", 3, 80, "HighBias", "PBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U0, coreId, "IF_CASC_N", 7, 1, "HighBias", "NBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U0, coreId, "IF_DC_P", 7, 2, "HighBias", "PBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U0, coreId, "IF_NMDA_N", 7, 1, "HighBias", "PBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U0, coreId, "IF_RFR_N", 0, 108, "HighBias", "NBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U0, coreId, "IF_TAU1_N", 6, 24, "LowBias", "NBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U0, coreId, "IF_TAU2_N", 5, 15, "HighBias", "NBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U0, coreId, "IF_THR_N", 3, 20, "HighBias", "NBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U0, coreId, "NPDPIE_TAU_F_P", 5, 41, "HighBias",
				"PBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U0, coreId, "NPDPIE_TAU_S_P", 7, 40, "HighBias",
				"NBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U0, coreId, "NPDPIE_THR_F_P", 2, 200, "HighBias",
				"PBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U0, coreId, "NPDPIE_THR_S_P", 7, 0, "HighBias",
				"PBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U0, coreId, "NPDPII_TAU_F_P", 7, 40, "HighBias",
				"NBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U0, coreId, "NPDPII_TAU_S_P", 7, 40, "HighBias",
				"NBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U0, coreId, "NPDPII_THR_F_P", 7, 40, "HighBias",
				"PBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U0, coreId, "NPDPII_THR_S_P", 7, 40, "HighBias",
				"PBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U0, coreId, "PS_WEIGHT_EXC_F_N", 0, 216, "HighBias",
				"NBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U0, coreId, "PS_WEIGHT_EXC_S_N", 7, 1, "HighBias",
				"NBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U0, coreId, "PS_WEIGHT_INH_F_N", 7, 1, "HighBias",
				"NBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U0, coreId, "PS_WEIGHT_INH_S_N", 7, 1, "HighBias",
				"NBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U0, coreId, "PULSE_PWLK_P", 0, 43, "HighBias",
				"PBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U0, coreId, "R2R_P", 4, 85, "HighBias", "PBias");
		}
		caerLog(CAER_LOG_NOTICE, __func__, "Done");
		state->init = true;
	} // end init

	if (state->doTraining) {

		// teaching signal
		sshsNode spikeGenNode = sshsGetRelativeNode(stateSource->eventSourceConfigNode, "DYNAPSEFX2/spikeGen/");
		sshsNodePutBool(spikeGenNode, "doStim", false);
		atomic_store(&stateSource->genSpikeState.doStim, false);
		atomic_store(&stateSource->genSpikeState.stim_type, 2);
		sshsNodePutInt(spikeGenNode, "stim_type", 2);
		atomic_store(&stateSource->genSpikeState.core_d, 15);
		sshsNodePutInt(spikeGenNode, "core_d", 15);
		atomic_store(&stateSource->genSpikeState.address, groupId + 1);
		sshsNodePutInt(spikeGenNode, "address", groupId + 1);
		atomic_store(&stateSource->genSpikeState.dx, 0);
		sshsNodePutInt(spikeGenNode, "dx", 0);
		atomic_store(&stateSource->genSpikeState.dy, 0);
		sshsNodePutInt(spikeGenNode, "dy", 0);
		atomic_store(&stateSource->genSpikeState.sx, 0);
		sshsNodePutInt(spikeGenNode, "sx", 0);
		atomic_store(&stateSource->genSpikeState.sy, 0);
		sshsNodePutInt(spikeGenNode, "sy", 0);
		atomic_store(&stateSource->genSpikeState.stim_avr, 30);
		sshsNodePutInt(spikeGenNode, "stim_avr", 102);
		atomic_store(&stateSource->genSpikeState.repeat, false);
		sshsNodePutBool(spikeGenNode, "repeat", false);
		atomic_store(&stateSource->genSpikeState.stim_duration, 1);
		sshsNodePutInt(spikeGenNode, "stim_duration", 1);
		atomic_store(&stateSource->genSpikeState.chip_id, 0);
		sshsNodePutInt(spikeGenNode, "chip_id", 0);
		atomic_store(&stateSource->genSpikeState.doStim, true);				// pass it to the thread
		sshsNodePutBool(spikeGenNode, "doStim", true);
	}

	//loop over all events and update activations Matrixes
	CAER_SPIKE_ITERATOR_VALID_START(spike)

		uint16_t x = caerSpikeEventGetX(caerSpikeIteratorElement);
		uint16_t y = caerSpikeEventGetY(caerSpikeIteratorElement);

		// depending on current groupId put activity in different maps
		if (groupId == 0) {
			state->group_a->buffer2d[x][y] += 1;
		}
		else if (groupId == 1) {
			state->group_b->buffer2d[x][y] += 1;
		}
		else if (groupId == 2) {
			state->group_c->buffer2d[x][y] += 1;
		}
		else if (groupId == 3) {
			state->group_d->buffer2d[x][y] += 1;
		}

	CAER_SPIKE_ITERATOR_VALID_END

	// time for update
	// get current time
	portable_clock_gettime_monotonic(&state->internalTime);
	double now = ((double) state->internalTime.tv_sec + 1.0e-9 * state->internalTime.tv_nsec);

	// if not measuring, let's start
	if (state->startedMeas == false) {
		portable_clock_gettime_monotonic(&state->internalTime);
		state->measureStartedAt = (double) state->internalTime.tv_sec + 1.0e-9 * state->internalTime.tv_nsec;
		state->startedMeas = true;
	}

	// if we measured for enough time..
	if ((double) state->measureMinTime <= (double) (now - state->measureStartedAt)) {
		// restart
		state->startedMeas = false;
		int changed_num = 0;
		normalize_buffef_map_sigma(moduleData, DYNAPSE_X4BOARD_NEUX);

		//caerLog(CAER_LOG_NOTICE, __func__, "change feature maps");

		// now check activity/features and if there is overlap
		// change features maps, only for bad features
		// state->threshold
		for (size_t x = 0; x < DYNAPSE_X4BOARD_NEUX; x++) {
			for (size_t y = 0; y < DYNAPSE_X4BOARD_NEUY; y++) {
				bool changing = false;
				int chipid = 0;

				// if activity is over threshold
				// in at least in two maps or not active at all
				// does this neuron have an useful feature?
				if ((state->group_a->buffer2d[x][y] > state->threshold && state->group_b->buffer2d[x][y] > state->threshold)
					|| (state->group_a->buffer2d[x][y] > state->threshold && state->group_c->buffer2d[x][y] > state->threshold)
					|| (state->group_b->buffer2d[x][y] > state->threshold && state->group_c->buffer2d[x][y] > state->threshold)) {

					//caerLog(CAER_LOG_NOTICE, __func__, "Done GroupA:%d GroupB:%d GroupC:%d GroupD:%d\n",
					//	state->group_a->buffer2d[x][y], state->group_b->buffer2d[x][y], state->group_c->buffer2d[x][y],
					//	state->group_d->buffer2d[x][y]);


					if (x >= DYNAPSE_CONFIG_XCHIPSIZE && y < DYNAPSE_CONFIG_YCHIPSIZE) {
						changing = true;
						chipid = DYNAPSE_CONFIG_DYNAPSE_U2;
						changed_num++;
						//caerLog(CAER_LOG_NOTICE, __func__, "double active: feature sin %d,%d need a new feature", x, y);
					}
					else if (x < DYNAPSE_CONFIG_XCHIPSIZE && y >= DYNAPSE_CONFIG_YCHIPSIZE) {
						changing = true;
						chipid = DYNAPSE_CONFIG_DYNAPSE_U1;
						changed_num++;
						//caerLog(CAER_LOG_NOTICE, __func__, "double active: feature cos %d,%d need a new feature", x, y);
					}
					//caerLog(CAER_LOG_NOTICE, __func__, "pixel %d,%d need a new feature", x, y);
				}

				if ((state->group_a->buffer2d[x][y] < 10 && state->group_b->buffer2d[x][y] < 10)
					|| (state->group_a->buffer2d[x][y] < 10 && state->group_c->buffer2d[x][y] < 10)
					|| (state->group_b->buffer2d[x][y] < 10 && state->group_c->buffer2d[x][y] < 10)) {

					if (x >= DYNAPSE_CONFIG_XCHIPSIZE && y < DYNAPSE_CONFIG_YCHIPSIZE) {
						changing = true;
						changed_num++;
						chipid = DYNAPSE_CONFIG_DYNAPSE_U2;
					}
					else if (x < DYNAPSE_CONFIG_XCHIPSIZE && y >= DYNAPSE_CONFIG_YCHIPSIZE) {
						changing = true;
						changed_num++;
						chipid = DYNAPSE_CONFIG_DYNAPSE_U1;
					}
				}

				// changing feature maps / receptive field
				if (changing && ( (rand() % 20) > 10) ) {
					// find chipid and coreid
					int posx = x;
					int posy = y;

					// select chip
					uint16_t chipId = 0;
					if (posx >= (int) DYNAPSE_CONFIG_XCHIPSIZE && posy >= (int) DYNAPSE_CONFIG_YCHIPSIZE) {
						chipId = DYNAPSE_CONFIG_DYNAPSE_U3;
					}
					else if (posx < (int) DYNAPSE_CONFIG_XCHIPSIZE && posy >= (int) DYNAPSE_CONFIG_YCHIPSIZE) {
						chipId = DYNAPSE_CONFIG_DYNAPSE_U2;
					}
					else if (posx >= (int) DYNAPSE_CONFIG_XCHIPSIZE && posy < (int) DYNAPSE_CONFIG_YCHIPSIZE) {
						chipId = DYNAPSE_CONFIG_DYNAPSE_U1;
					}
					else if (posx < (int) DYNAPSE_CONFIG_XCHIPSIZE && posy < (int) DYNAPSE_CONFIG_YCHIPSIZE) {
						chipId = DYNAPSE_CONFIG_DYNAPSE_U0;
					}
					// adjust coordinate for chip
					if (chipId == DYNAPSE_CONFIG_DYNAPSE_U3) {
						posx = posx - DYNAPSE_CONFIG_XCHIPSIZE;
						posy = posy - DYNAPSE_CONFIG_YCHIPSIZE;
					}
					else if (chipId == DYNAPSE_CONFIG_DYNAPSE_U2) {
						posy = posy - DYNAPSE_CONFIG_YCHIPSIZE;
					}
					else if (chipId == DYNAPSE_CONFIG_DYNAPSE_U1) {
						posx = posx - DYNAPSE_CONFIG_XCHIPSIZE;
					}
					// select core
					uint8_t coreid = 0;
					if (posx >= 16 && posy < 16) {
						coreid = 2;
					}
					else if (posx >= 16 && posy >= 16) {
						coreid = 3;
					}
					else if (posx < 16 && posy < 16) {
						coreid = 0;
					}
					else if (posx < 16 && posy >= 16) {
						coreid = 1;
					}
					// adjust coordinates for cores
					if (coreid == 1) {
						posy = posy - DYNAPSE_CONFIG_NEUCOL;
					}
					else if (coreid == 0) {
						;
					}
					else if (coreid == 2) {
						posx = posx - DYNAPSE_CONFIG_NEUCOL;
					}
					else if (coreid == 3) {
						posx = posx - DYNAPSE_CONFIG_NEUCOL;
						posy = posy - DYNAPSE_CONFIG_NEUCOL;
					}

					// linear index
					uint32_t indexLin = (uint32_t) posx * DYNAPSE_CONFIG_NEUCOL + (uint32_t) posy;
					if (indexLin > 255) {
						indexLin = 255;
					}

					// now we have
					// - coreid
					// - chipId
					// - indexLin  of neuron

					int bits[DYNAPSE_MAX_USER_USB_PACKET_SIZE];
					int numConfig = -1;

					// select chip
					caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, (uint32_t) chipId);
					for(size_t camId = 0; camId < DYNAPSE_CONFIG_NUMCAM; camId ++){
						uint32_t synapse_row = camId; 								// 0 - 63 cam ID
						uint32_t neuron_row = (indexLin & 0xf0) >> 4;				// 0 - 255
						uint32_t row = neuron_row << 6 | synapse_row; 				// neuron that will receive from sourceneuron_inmap
						uint32_t column = indexLin & 0xf;

						// generate random CAM content for this neuron
						uint8_t ei =  (uint32_t) (rand() % 2);
						uint8_t fs =  (uint32_t) (rand() % 2);
						uint32_t sourceneuron_inmap = (uint32_t) (rand() % 256);
						uint8_t source_core =  (uint32_t) (rand() % 4);

						numConfig++;
						bits[numConfig] = ei << 29 | fs << 28 | sourceneuron_inmap << 20 | source_core << 18 | 1 << 17
							| coreid << 15 | row << 5 | column | 0 << 4;
					}
					if(numConfig > 0){
						if (!caerDynapseSendDataToUSB(stateSource->deviceState, bits, numConfig)) {
							caerLog(CAER_LOG_ERROR, __func__, "USB transfer failed");
						}
					}

				}

			} // loop y
		} // loop x

		 caerLog(CAER_LOG_NOTICE, __func__, "Neuron selected num %d (before applying prob)", changed_num);

		 resetMap_a(moduleData, DYNAPSE_X4BOARD_NEUX);
		 resetMap_b(moduleData, DYNAPSE_X4BOARD_NEUX);
		 resetMap_c(moduleData, DYNAPSE_X4BOARD_NEUX);
		 resetMap_d(moduleData, DYNAPSE_X4BOARD_NEUX);

	} // time measurements

	caerTrainingFromCaffeFilterConfig(moduleData); // update user parameters

}

static void caerTrainingFromCaffeFilterConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	TFCFilterState state = moduleData->moduleState;

	state->doTraining = sshsNodeGetBool(moduleData->moduleNode, "doTraining");
	state->freqStim = sshsNodeGetInt(moduleData->moduleNode, "freqStim");
	state->threshold = sshsNodeGetFloat(moduleData->moduleNode, "threshold");
	state->measureMinTime = sshsNodeGetFloat(moduleData->moduleNode, "measureMinTime");

}

static void caerTrainingFromCaffeFilterExit(caerModuleData moduleData) {
// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	TFCFilterState state = moduleData->moduleState;

}

static void caerTrainingFromCaffeFilterReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);

	TFCFilterState state = moduleData->moduleState;

}

void caerTrainFromMakeFrame(uint16_t moduleID, caerFrameEventPacket *grpup_a, caerFrameEventPacket *grpup_b,
	caerFrameEventPacket *grpup_c, caerFrameEventPacket *grpup_d, int size) {

	caerModuleData moduleData = caerMainloopFindModule(moduleID, "TrainFromCaffe", CAER_MODULE_PROCESSOR);

	TFCFilterState state = moduleData->moduleState;
	if (state->group_a == NULL || state->group_b == NULL || state->group_c == NULL || state->group_d == NULL) {
		return;
	}
	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	if (!sshsNodeAttributeExists(sourceInfoNode, "dataSizeX", SSHS_SHORT)) { //to do for visualizer change name of field to a more generic one
		sshsNodePutShort(sourceInfoNode, "dataSizeX", size);
		sshsNodePutShort(sourceInfoNode, "dataSizeY", size);
	}

	normalize_buffef_map_sigma(moduleData, size);

	// put info into frame
	*grpup_a = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, size, size, 3);
	caerMainloopFreeAfterLoop(&free, *grpup_a);

	*grpup_b = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, size, size, 3);
	caerMainloopFreeAfterLoop(&free, *grpup_b);

	*grpup_c = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, size, size, 3);
	caerMainloopFreeAfterLoop(&free, *grpup_c);

	*grpup_d = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, size, size, 3);
	caerMainloopFreeAfterLoop(&free, *grpup_d);

	if (*grpup_a != NULL) {
		caerFrameEvent singleplota = caerFrameEventPacketGetEvent(*grpup_a, 0);
		uint32_t counter = 0;
		for (size_t x = 0; x < size; x++) {
			for (size_t y = 0; y < size; y++) {
				//COLOUR col  = GetColour((double) state->frequencyMap->buffer2d[y][x], state->colorscaleMin, state->colorscaleMax);
				//caerLog(CAER_LOG_NOTICE, __func__, "after alloc %d", state->group_a->buffer2d[x][y]);
				singleplota->pixels[counter] = (uint16_t) (0); // red
				singleplota->pixels[counter + 1] = (uint16_t) (state->group_a->buffer2d[y][x] * 255); // green
				singleplota->pixels[counter + 2] = (uint16_t) (0); // blue
				counter += 3;
			}
		}
		//add info to the frame
		caerFrameEventSetLengthXLengthYChannelNumber(singleplota, size, size, 3, *grpup_a);
		//validate frame
		caerFrameEventValidate(singleplota, *grpup_a);
	}

	if (*grpup_b != NULL) {
		caerFrameEvent singleplotb = caerFrameEventPacketGetEvent(*grpup_b, 0);
		uint32_t counter = 0;
		for (size_t x = 0; x < size; x++) {
			for (size_t y = 0; y < size; y++) {
				//COLOUR col  = GetColour((double) state->frequencyMap->buffer2d[y][x], state->colorscaleMin, state->colorscaleMax);
				singleplotb->pixels[counter] = (uint16_t) (0); // red
				singleplotb->pixels[counter + 1] = (uint16_t) (0); // green
				singleplotb->pixels[counter + 2] = (uint16_t) (state->group_b->buffer2d[y][x] * 255); // blue
				counter += 3;
			}
		}
		//add info to the frame
		caerFrameEventSetLengthXLengthYChannelNumber(singleplotb, size, size, 3, *grpup_b);
		//validate frame
		caerFrameEventValidate(singleplotb, *grpup_b);
	}

	if (*grpup_c != NULL) {
		caerFrameEvent singleplotc = caerFrameEventPacketGetEvent(*grpup_c, 0);
		uint32_t counter = 0;
		for (size_t x = 0; x < size; x++) {
			for (size_t y = 0; y < size; y++) {
				//COLOUR col  = GetColour((double) state->frequencyMap->buffer2d[y][x], state->colorscaleMin, state->colorscaleMax);
				singleplotc->pixels[counter] = (uint16_t) (state->group_c->buffer2d[y][x] * 255); // red
				singleplotc->pixels[counter + 1] = (uint16_t) (0); // green
				singleplotc->pixels[counter + 2] = (uint16_t) (0); // blue
				counter += 3;
			}
		}
		//add info to the frame
		caerFrameEventSetLengthXLengthYChannelNumber(singleplotc, size, size, 3, *grpup_c);
		//validate frame
		caerFrameEventValidate(singleplotc, *grpup_c);
	}

	if (*grpup_d != NULL) {
		caerFrameEvent singleplotd = caerFrameEventPacketGetEvent(*grpup_d, 0);
		uint32_t counter = 0;
		for (size_t x = 0; x < size; x++) {
			for (size_t y = 0; y < size; y++) {
				//COLOUR col  = GetColour((double) state->frequencyMap->buffer2d[y][x], state->colorscaleMin, state->colorscaleMax);
				singleplotd->pixels[counter] = (uint16_t) (state->group_d->buffer2d[y][x] * 255); // red
				singleplotd->pixels[counter + 1] = (uint16_t) (state->group_d->buffer2d[y][x] * 255); // green
				singleplotd->pixels[counter + 2] = (uint16_t) (0); // blue
				counter += 3;
			}
		}
		//add info to the frame
		caerFrameEventSetLengthXLengthYChannelNumber(singleplotd, size, size, 3, *grpup_d);
		//validate frame
		caerFrameEventValidate(singleplotd, *grpup_d);
	}


}

// resets
void resetMap_a(caerModuleData moduleData, int size) {
	TFCFilterState state = moduleData->moduleState;

	for (int col_idx = 0; col_idx < size; col_idx++) {
		for (int row_idx = 0; row_idx < size; row_idx++) {
			state->group_a->buffer2d[col_idx][row_idx] = 0;
		}
	}

}
void resetMap_b(caerModuleData moduleData, int size) {
	TFCFilterState state = moduleData->moduleState;
	for (int col_idx = 0; col_idx < size; col_idx++) {
		for (int row_idx = 0; row_idx < size; row_idx++) {
			state->group_b->buffer2d[col_idx][row_idx] = 0;
		}
	}

}
void resetMap_c(caerModuleData moduleData, int size) {
	TFCFilterState state = moduleData->moduleState;
	for (int col_idx = 0; col_idx < size; col_idx++) {
		for (int row_idx = 0; row_idx < size; row_idx++) {
			state->group_c->buffer2d[col_idx][row_idx] = 0;
		}
	}

}
void resetMap_d(caerModuleData moduleData, int size) {
	TFCFilterState state = moduleData->moduleState;
	for (int col_idx = 0; col_idx < size; col_idx++) {
		for (int row_idx = 0; row_idx < size; row_idx++) {
			state->group_d->buffer2d[col_idx][row_idx] = 0;
		}
	}
}

//This function implement 3 sigma normalization
bool normalize_buffef_map_sigma(caerModuleData moduleData, int size) {
	TFCFilterState state = moduleData->moduleState;

	int sum_a = 0, count_a = 0;
	int sum_b = 0, count_b = 0;
	int sum_c = 0, count_c = 0;
	int sum_d = 0, count_d = 0;
	for (int i = 0; i < size; i++) {
		for (int j = 0; j < size; j++) {
			if (state->group_a->buffer2d[i][j] != 0) {
				sum_a += state->group_a->buffer2d[i][j];
				count_a++;
			}
			if (state->group_b->buffer2d[i][j] != 0) {
				sum_b += state->group_b->buffer2d[i][j];
				count_b++;
			}
			if (state->group_c->buffer2d[i][j] != 0) {
				sum_c += state->group_c->buffer2d[i][j];
				count_c++;
			}
			if (state->group_d->buffer2d[i][j] != 0) {
				sum_d += state->group_d->buffer2d[i][j];
				count_d++;
			}
		}
	}

	float mean_a = 0;
	if (count_a > 0)
		mean_a = sum_a / count_a;
	float mean_b = 0;
	if (count_b > 0)
		mean_b = sum_b / count_b;
	float mean_c = 0;
	if (count_c > 0)
		mean_c = sum_c / count_c;
	float mean_d = 0;
	if (count_d > 0)
		mean_d = sum_d / count_d;

	float var_a = 0;
	float var_b = 0;
	float var_c = 0;
	float var_d = 0;
	for (int i = 0; i < size; i++) {
		for (int j = 0; j < size; j++) {
			if (state->group_a->buffer2d[i][j] != 0) {
				float f = state->group_a->buffer2d[i][j] - mean_a;
				var_a += f * f;
			}
			if (state->group_b->buffer2d[i][j] != 0) {
				float f = state->group_b->buffer2d[i][j] - mean_b;
				var_b += f * f;
			}
			if (state->group_c->buffer2d[i][j] != 0) {
				float f = state->group_c->buffer2d[i][j] - mean_c;
				var_c += f * f;
			}
			if (state->group_d->buffer2d[i][j] != 0) {
				float f = state->group_d->buffer2d[i][j] - mean_d;
				var_d += f * f;
			}
		}
	}

	float sig_a = 0;
	if (count_a > 0)
		sig_a = sqrt(var_a / count_a);
	if (sig_a < (0.1f / 255.0f)) {
		sig_a = 0.1f / 255.0f;
	}
	float sig_b = 0;
	if (count_b > 0)
		sig_b = sqrt(var_b / count_b);
	if (sig_b < (0.1f / 255.0f)) {
		sig_b = 0.1f / 255.0f;
	}
	float sig_c = 0;
	if (count_c > 0)
		sig_c = sqrt(var_c / count_c);
	if (sig_c < (0.1f / 255.0f)) {
		sig_c = 0.1f / 255.0f;
	}
	float sig_d = 0;
	sig_d = sqrt(var_d / count_d);
	if (sig_d < (0.1f / 255.0f)) {
		sig_d = 0.1f / 255.0f;
	}

	float numSDevs = 3;
	float mean_png_gray;
	float range_a, halfrange_a;
	float range_b, halfrange_b;
	float range_c, halfrange_c;
	float range_d, halfrange_d;

	mean_png_gray = 0; // rectified
	range_a = numSDevs * sig_a * (1.0f / 256.0f);
	halfrange_a = 0;
	range_b = numSDevs * sig_b * (1.0f / 256.0f);
	halfrange_b = 0;
	range_c = numSDevs * sig_c * (1.0f / 256.0f);
	halfrange_c = 0;
	range_d = numSDevs * sig_d * (1.0f / 256.0f);
	halfrange_d = 0;

	for (int col_idx = 0; col_idx < size; col_idx++) {
		for (int row_idx = 0; row_idx < size; row_idx++) {
			if (state->group_a->buffer2d[col_idx][row_idx] == 0) {
				state->group_a->buffer2d[col_idx][row_idx] = mean_png_gray;
			}
			else {
				float f = 0;
				if (range_a > 0)
					f = (state->group_a->buffer2d[col_idx][row_idx] + halfrange_a) / range_a;
				if (f > 256) {
					f = 256;
				}
				else if (f < 0) {
					f = 0;
				}
				state->group_a->buffer2d[col_idx][row_idx] = (int) floor(f); //shift by 256 included in previous computations
			}
			if (state->group_b->buffer2d[col_idx][row_idx] == 0) {
				state->group_b->buffer2d[col_idx][row_idx] = mean_png_gray;
			}
			else {
				float f = 0;
				if (range_b > 0)
					f = (state->group_b->buffer2d[col_idx][row_idx] + halfrange_b) / range_b;
				if (f > 256) {
					f = 256;
				}
				else if (f < 0) {
					f = 0;
				}
				state->group_b->buffer2d[col_idx][row_idx] = (int) floor(f); //shift by 256 included in previous computations
			}
			if (state->group_c->buffer2d[col_idx][row_idx] == 0) {
				state->group_c->buffer2d[col_idx][row_idx] = mean_png_gray;
			}
			else {
				float f = 0;
				if (range_c > 0)
					f = (state->group_c->buffer2d[col_idx][row_idx] + halfrange_c) / range_c;
				if (f > 256) {
					f = 256;
				}
				else if (f < 0) {
					f = 0;
				}
				state->group_c->buffer2d[col_idx][row_idx] = (int) floor(f); //shift by 256 included in previous computations
			}
			if (state->group_d->buffer2d[col_idx][row_idx] == 0) {
				state->group_d->buffer2d[col_idx][row_idx] = mean_png_gray;
			}
			else {
				float f = 0;
				if (range_d > 0)
					f = (state->group_d->buffer2d[col_idx][row_idx] + halfrange_d) / range_d;
				if (f > 256) {
					f = 256;
				}
				else if (f < 0) {
					f = 0;
				}
				state->group_d->buffer2d[col_idx][row_idx] = (int) floor(f); //shift by 256 included in previous computations
			}

		}
	}

	return (true);

}

bool allocateActivityMap(TFCFilterState state) {

	int16_t sizeX = DYNAPSE_X4BOARD_NEUX;
	int16_t sizeY = DYNAPSE_X4BOARD_NEUY;

	state->group_a = simple2DBufferInitInt((size_t) sizeX, (size_t) sizeY);
	if (state->group_a == NULL) {
		return (false);
	}
	state->group_b = simple2DBufferInitInt((size_t) sizeX, (size_t) sizeY);
	if (state->group_b == NULL) {
		return (false);
	}
	state->group_c = simple2DBufferInitInt((size_t) sizeX, (size_t) sizeY);
	if (state->group_c == NULL) {
		return (false);
	}
	state->group_d = simple2DBufferInitInt((size_t) sizeX, (size_t) sizeY);
	if (state->group_d == NULL) {
		return (false);
	}
	state->tmp = simple2DBufferInitInt((size_t) sizeX, (size_t) sizeY);
	if (state->tmp == NULL) {
		return (false);
	}
	return (true);
}
