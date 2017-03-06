/*
 * helloworld.c
 *
 *  Created on: Feb 2017 for tutorial on dynap-se
 *      Author: federico
 */

#include "helloworld.h"
#include "base/mainloop.h"
#include "base/module.h"

struct HWFilter_state {
	// user settings
	//bool setCam;
	//bool loadBiases;
	//bool setSram;
	bool init;
	// usb utils
	caerInputDynapseState eventSourceModuleState;
	sshsNode eventSourceConfigNode;
};

typedef struct HWFilter_state *HWFilterState;

static bool caerHelloWorldModuleInit(caerModuleData moduleData);
static void caerHelloWorldModuleRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerHelloWorldModuleConfig(caerModuleData moduleData);
static void caerHelloWorldModuleExit(caerModuleData moduleData);
static void caerHelloWorldModuleReset(caerModuleData moduleData, uint16_t resetCallSourceID);

static struct caer_module_functions caerHelloWorldModuleFunctions = { .moduleInit =
	&caerHelloWorldModuleInit, .moduleRun = &caerHelloWorldModuleRun, .moduleConfig =
	&caerHelloWorldModuleConfig, .moduleExit = &caerHelloWorldModuleExit, .moduleReset =
	&caerHelloWorldModuleReset };

void caerHelloWorldModule(uint16_t moduleID, caerSpikeEventPacket spike) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "Hello-world", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerHelloWorldModuleFunctions, moduleData, sizeof(struct HWFilter_state), 1, spike);
}

static bool caerHelloWorldModuleInit(caerModuleData moduleData) {
	// create parameters
	//sshsNodePutBoolIfAbsent(moduleData->moduleNode, "loadBiases", false);
	//sshsNodePutBoolIfAbsent(moduleData->moduleNode, "setSram", false);
	//sshsNodePutBoolIfAbsent(moduleData->moduleNode, "setCam", false);

	HWFilterState state = moduleData->moduleState;

	// update node state
	//state->loadBiases = sshsNodeGetBool(moduleData->moduleNode, "loadBiases");
	//state->setSram = sshsNodeGetBool(moduleData->moduleNode, "setSram");
	//state->setCam = sshsNodeGetBool(moduleData->moduleNode, "setCam");

	state->init = false;

	// Add config listeners last - let's the user interact with the parameter -
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	// Nothing that can fail here.
	return (true);
}

static void caerHelloWorldModuleRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerSpikeEventPacket spike = va_arg(args, caerSpikeEventPacket);

	// Only process packets with content.
	if (spike == NULL) {
		return;
	}

	HWFilterState state = moduleData->moduleState;

  	// now we can do crazy processing etc..
	// first find out which one is the module producing the spikes. and get usb handle
	// --- start  usb handle / from spike event source id
	int sourceID = caerEventPacketHeaderGetEventSource(&spike->packetHeader);
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(sourceID));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(sourceID));
	if(state->eventSourceModuleState == NULL || state->eventSourceConfigNode == NULL){
		return;
	}
	caerInputDynapseState stateSource = state->eventSourceModuleState;
	if(stateSource->deviceState == NULL){
		return;
	}
	// --- end usb handle


	if(state->init == false){
		// do the initialization

		caerLog(CAER_LOG_NOTICE, __func__, "start init of hello world");
		// load biases
		for(size_t coreid=0; coreid<4 ; coreid++){
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U3, coreid, "IF_DC_P", 5, 125, "HighBias", "PBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U3, coreid, "IF_THR_N", 5, 125, "HighBias", "NBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U3, coreid, "IF_TAU1_N", 6, 125, "HighBias", "NBias");
			caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U3, coreid, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");

			/*caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U3, coreid, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
			caerDynapseSetBias(stateSource, (uint) state->chipId, coreId, "IF_AHTHR_N", 7, 1, "HighBias", "NBias");
			caerDynapseSetBias(stateSource, (uint) state->chipId, coreId, "IF_AHW_P", 7, 1, "HighBias", "PBias");
			caerDynapseSetBias(stateSource, (uint) state->chipId, coreId, "IF_BUF_P", 3, 80, "HighBias", "PBias");
			caerDynapseSetBias(stateSource, (uint) state->chipId, coreId, "IF_CASC_N", 7, 1, "HighBias", "NBias");
			caerDynapseSetBias(stateSource, (uint) state->chipId, coreId, "IF_DC_P", 7, 2, "HighBias", "PBias");
			caerDynapseSetBias(stateSource, (uint) state->chipId, coreId, "IF_NMDA_N", 7, 1, "HighBias", "PBias");
			caerDynapseSetBias(stateSource, (uint) state->chipId, coreId, "IF_RFR_N", 0, 108, "HighBias", "NBias");
			caerDynapseSetBias(stateSource, (uint) state->chipId, coreId, "IF_TAU1_N", 6, 24, "LowBias", "NBias");
			caerDynapseSetBias(stateSource, (uint) state->chipId, coreId, "IF_TAU2_N", 5, 15, "HighBias", "NBias");
			caerDynapseSetBias(stateSource, (uint) state->chipId, coreId, "IF_THR_N", 3, 20, "HighBias", "NBias");
			caerDynapseSetBias(stateSource, (uint) state->chipId, coreId, "NPDPIE_TAU_F_P", 5, 41, "HighBias",
				"PBias");
			caerDynapseSetBias(stateSource, (uint) state->chipId, coreId, "NPDPIE_TAU_S_P", 7, 40, "HighBias",
				"NBias");
			caerDynapseSetBias(stateSource, (uint) state->chipId, coreId, "NPDPIE_THR_F_P", 2, 200, "HighBias",
				"PBias");
			caerDynapseSetBias(stateSource, (uint) state->chipId, coreId, "NPDPIE_THR_S_P", 7, 0, "HighBias",
				"PBias");
			caerDynapseSetBias(stateSource, (uint) state->chipId, coreId, "NPDPII_TAU_F_P", 7, 40, "HighBias",
				"NBias");
			caerDynapseSetBias(stateSource, (uint) state->chipId, coreId, "NPDPII_TAU_S_P", 7, 40, "HighBias",
				"NBias");
			caerDynapseSetBias(stateSource, (uint) state->chipId, coreId, "NPDPII_THR_F_P", 7, 40, "HighBias",
				"PBias");
			caerDynapseSetBias(stateSource, (uint) state->chipId, coreId, "NPDPII_THR_S_P", 7, 40, "HighBias",
				"PBias");
			caerDynapseSetBias(stateSource, (uint) state->chipId, coreId, "PS_WEIGHT_EXC_F_N", 0, 216, "HighBias",
				"NBias");
			caerDynapseSetBias(stateSource, (uint) state->chipId, coreId, "PS_WEIGHT_EXC_S_N", 7, 1, "HighBias",
				"NBias");
			caerDynapseSetBias(stateSource, (uint) state->chipId, coreId, "PS_WEIGHT_INH_F_N", 7, 1, "HighBias",
				"NBias");
			caerDynapseSetBias(stateSource, (uint) state->chipId, coreId, "PS_WEIGHT_INH_S_N", 7, 1, "HighBias",
				"NBias");
			caerDynapseSetBias(stateSource, (uint) state->chipId, coreId, "PULSE_PWLK_P", 0, 43, "HighBias",
				"PBias");
			caerDynapseSetBias(stateSource, (uint) state->chipId, coreId, "R2R_P", 4, 85, "HighBias", "PBias");*/

		}

		// --- set sram
		//  0 - select which chip to configure
		caerDeviceConfigSet(state->eventSourceModuleState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U3);
		//  1 -configure
		caerDynapseWriteSram(stateSource->deviceState, 3, 255, 3, DYNAPSE_CONFIG_SRAM_DIRECTION_X_WEST, 1, DYNAPSE_CONFIG_SRAM_DIRECTION_Y_NORTH,
			1, 1, 15);
		caerDynapseWriteSram(stateSource->deviceState, 3, 255, 3, 0, 0, 0, 0, 2, 2);
		caerDynapseWriteSram(stateSource->deviceState, 3, 255, 3, 0, 0, DYNAPSE_CONFIG_SRAM_DIRECTION_Y_NORTH, 1, 3, 2);
		//set cam
		caerDynapseWriteCam(stateSource->deviceState, 1023, 256, 0, DYNAPSE_CONFIG_CAMTYPE_F_EXC);

		// chip u0
		caerDeviceConfigSet(state->eventSourceModuleState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U0);
		for(size_t neuronid = 0; neuronid < 1024; neuronid++){
			for(size_t camid=32; camid<64; camid++){
				caerDynapseWriteCam(stateSource->deviceState, neuronid, 0, camid, DYNAPSE_CONFIG_CAMTYPE_F_EXC);
			}
		}
		for(size_t camid=0; camid<32; camid++){
			caerDynapseWriteCam(stateSource->deviceState, 1023, 255, camid, DYNAPSE_CONFIG_CAMTYPE_F_EXC);
			caerDynapseWriteCam(stateSource->deviceState, 1023, 511, camid, DYNAPSE_CONFIG_CAMTYPE_F_EXC);
			caerDynapseWriteCam(stateSource->deviceState, 1023, 767, camid, DYNAPSE_CONFIG_CAMTYPE_F_EXC);
			caerDynapseWriteCam(stateSource->deviceState, 1023, 1023, camid, DYNAPSE_CONFIG_CAMTYPE_F_EXC);

		}
		// chip u2
		caerDeviceConfigSet(state->eventSourceModuleState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U2);
		caerDynapseWriteCam(stateSource->deviceState, 1023, 256, 0, DYNAPSE_CONFIG_CAMTYPE_F_EXC);


		// set the spike generator to excite the source neuron
		caerDeviceConfigSet(state->eventSourceModuleState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U3);
		for(size_t camid=0; camid<64; camid++){
			caerDynapseWriteCam(stateSource->deviceState, 12, 1023, camid, DYNAPSE_CONFIG_CAMTYPE_F_EXC);
		}
		// set the spike generator to excite neuron 255 of chipid u3 by using address 8
		atomic_store(&stateSource->genSpikeState.chip_id, DYNAPSE_CONFIG_DYNAPSE_U3);
		atomic_store(&stateSource->genSpikeState.stim_type, 2); // regular
		atomic_store(&stateSource->genSpikeState.stim_avr, 100); // Hz
		atomic_store(&stateSource->genSpikeState.stim_duration, 10); // seconds
		atomic_store(&stateSource->genSpikeState.dx, 0); // no hop
		atomic_store(&stateSource->genSpikeState.dy, 0); // no hop
		atomic_store(&stateSource->genSpikeState.core_d, 8); // 1 0 0 0
		atomic_store(&stateSource->genSpikeState.address, 12); // cam content
		atomic_store(&stateSource->genSpikeState.doStim, true); // cam content

		caerLog(CAER_LOG_NOTICE, __func__, "init completed");

		state->init = true;
	}

	// Iterate over spikes in the packet
	/*CAER_SPIKE_ITERATOR_VALID_START(spike)
		int32_t timestamp =  caerSpikeEventGetTimestamp(caerSpikeIteratorElement);
	  	uint8_t chipid 	  =  caerSpikeEventGetChipID(caerSpikeIteratorElement);
	  	uint8_t neuronid  =  caerSpikeEventGetNeuronID(caerSpikeIteratorElement);
	  	uint8_t coreid    =  caerSpikeEventGetSourceCoreID(caerSpikeIteratorElement);


	CAER_SPIKE_ITERATOR_VALID_END*/


  	// sending bits to the USB, for
	// programing sram content
  	// sending spikes
	//caerDeviceConfigSet(state->eventSourceModuleState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U2);
	//caerDeviceConfigSet(state->eventSourceModuleState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, bits);

  	// send to usb in packets
  	// bool caerDynapseSendDataToUSB(caerDeviceHandle handle, int * data, int numConfig);

}

static void caerHelloWorldModuleConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	HWFilterState state = moduleData->moduleState;

	// this will update parameters, from user input

}

static void caerHelloWorldModuleExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	HWFilterState state = moduleData->moduleState;

	// here we should free memory and other shutdown procedures if needed

}

static void caerHelloWorldModuleReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);

	HWFilterState state = moduleData->moduleState;

}
