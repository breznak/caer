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
	bool setCam;
	bool displayNeuNumber;
	bool controlBiases;
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
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "displayNeuNumber", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "controlBiases", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "setCam", false);

	HWFilterState state = moduleData->moduleState;

	// update node state
	state->displayNeuNumber = sshsNodeGetBool(moduleData->moduleNode, "displayNeuNumber");
	state->controlBiases = sshsNodeGetBool(moduleData->moduleNode, "controlBiases");
	state->setCam = sshsNodeGetBool(moduleData->moduleNode, "setCam");


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

	// Iterate over spikes in the packet
	CAER_SPIKE_ITERATOR_VALID_START(spike)
		int32_t timestamp =  caerSpikeEventGetTimestamp(caerSpikeIteratorElement);
	  	uint8_t chipid 	  =  caerSpikeEventGetChipID(caerSpikeIteratorElement);
	  	uint8_t neuronid  =  caerSpikeEventGetNeuronID(caerSpikeIteratorElement);
	  	uint8_t coreid    =  caerSpikeEventGetSourceCoreID(caerSpikeIteratorElement);

	  	// only if user wants to display
	  	if(state->displayNeuNumber){
			// identify spikes from chip0 core 0
			if(chipid == DYNAPSE_CONFIG_DYNAPSE_U0 && coreid == 0 ){
				// tell which neuron
				caerLog(CAER_LOG_NOTICE, __func__, "Neuron id %d \n", neuronid);
			}
	  	}

	  	// ////////////////////////////////////
	  	// example of controlling the biases ..
		///////////////////////////////////////
	  	if(state->controlBiases){
			uint32_t bits = 0 ;
			if(neuronid == 150){
				caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U2, 0, "IF_DC_P", 1, 125, "HighBias", "PBias");
			}else{
				caerDynapseSetBias(stateSource, DYNAPSE_CONFIG_DYNAPSE_U2, 0, "IF_DC_P", 4, 125, "HighBias", "PBias");
			}
	  	}
	CAER_SPIKE_ITERATOR_VALID_END


  	// ///////////////////////////
  	// example set connections ..
  	///////////////////////////////
  	// program cam content
	if(state->setCam){
		bool ret = setCamContent(stateSource, DYNAPSE_CONFIG_DYNAPSE_U2, false, true, 12, 0, 4, 6, 2);
		if(!ret){
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "failed to set CAM content");
		}
	}

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
	state->displayNeuNumber = sshsNodeGetBool(moduleData->moduleNode, "displayNeuNumber");
	state->controlBiases = sshsNodeGetBool(moduleData->moduleNode, "controlBiases");
	state->setCam = sshsNodeGetBool(moduleData->moduleNode, "setCam");

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
