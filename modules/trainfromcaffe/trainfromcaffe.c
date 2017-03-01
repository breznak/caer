/*
 * trainfromcaffe.c
 *
 *  Created on: March, 2017
 *      Author: federico.corradi@inilabs.com
 */

#include "trainfromcaffe.h"
#include "base/mainloop.h"
#include "base/module.h"

struct TFCFilter_state {
	bool doTraining;
	int32_t freqStim;
	bool init;
	// usb utils
	caerInputDynapseState eventSourceModuleState;
	sshsNode eventSourceConfigNode;
};

typedef struct TFCFilter_state *TFCFilterState;

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

	caerModuleSM(&caerTrainingFromCaffeFilterFunctions, moduleData, sizeof(struct TFCFilter_state), 1, spike, groupId);
}

static bool caerTrainingFromCaffeFilterInit(caerModuleData moduleData) {
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doTraining", false);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "freqStim", 100);

	TFCFilterState state = moduleData->moduleState;

	state->doTraining = sshsNodeGetBool(moduleData->moduleNode, "doTraining");
	state->freqStim = sshsNodeGetInt(moduleData->moduleNode, "freqStim");
	state->init = false;

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

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
						if (caerDynapseWriteCam(stateSource->deviceState, coreId + 1, (coreId << 8)  | neuronId, camId,
							DYNAPSE_CONFIG_CAMTYPE_F_EXC)) {
							;
						}
						else {
							caerLog(CAER_LOG_NOTICE, __func__, "cannot program CAM");
						}
					}
					else {
						if (caerDynapseWriteCam(stateSource->deviceState, 0, (coreId << 8)  | neuronId,  camId,
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
	}
	if (state->doTraining) {

		sshsNode spikeGenNode = sshsGetRelativeNode(stateSource->eventSourceConfigNode, "DYNAPSEFX2/spikeGen/");
		sshsNodePutBool(spikeGenNode, "doStim", false);
		atomic_store(&stateSource->genSpikeState.doStim, false);

		atomic_store(&stateSource->genSpikeState.stim_type, 2);
		sshsNodePutInt(spikeGenNode, "stim_type", 2);

		atomic_store(&stateSource->genSpikeState.core_d, 15);
		sshsNodePutInt(spikeGenNode, "core_d", 15);

		atomic_store(&stateSource->genSpikeState.address, groupId+1);
		sshsNodePutInt(spikeGenNode, "address", groupId+1);

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
}

static void caerTrainingFromCaffeFilterConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	TFCFilterState state = moduleData->moduleState;

	state->doTraining = sshsNodeGetBool(moduleData->moduleNode, "doTraining");
	state->freqStim = sshsNodeGetInt(moduleData->moduleNode, "freqStim");

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

