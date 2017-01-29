/*
 *
 *  Created on: Jan, 2017
 *      Author: federico.corradi@inilabs.com
 */

#include "dvstodynapse.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "libcaer/devices/dynapse.h"

struct DvsToDynapse_state {
	sshsNode eventSourceModuleState;
	sshsNode eventSourceConfigNode;
	bool doMapping;
	int chipId;
};

typedef struct DvsToDynapse_state *DvsToDynapseState;

static bool caerDvsToDynapseInit(caerModuleData moduleData);
static void caerDvsToDynapseRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerDvsToDynapseConfig(caerModuleData moduleData);
static void caerDvsToDynapseExit(caerModuleData moduleData);
static void caerDvsToDynapseReset(caerModuleData moduleData, uint16_t resetCallSourceID);

static struct caer_module_functions caerDvsToDynapseFunctions = { .moduleInit =
	&caerDvsToDynapseInit, .moduleRun = &caerDvsToDynapseRun, .moduleConfig =
	&caerDvsToDynapseConfig, .moduleExit = &caerDvsToDynapseExit, .moduleReset =
	&caerDvsToDynapseReset };

void caerDvsToDynapse(uint16_t moduleID,  int16_t eventSourceID, caerSpikeEventPacket spike, caerPolarityEventPacket polarity) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "DvsToDynapse", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerDvsToDynapseFunctions, moduleData, sizeof(struct DvsToDynapse_state), 2, eventSourceID, spike, polarity);
}

static bool caerDvsToDynapseInit(caerModuleData moduleData) {

	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doMapping", false);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "chipId", DYNAPSE_CONFIG_DYNAPSE_U0); // 0,4,8,12

	DvsToDynapseState state = moduleData->moduleState;

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	if (!sshsNodeAttributeExists(sourceInfoNode, "dataSizeX", SSHS_SHORT)) { //to do for visualizer change name of field to a more generic one
		sshsNodePutShort(sourceInfoNode, "dataSizeX", DYNAPSE_X4BOARD_NEUX);
		sshsNodePutShort(sourceInfoNode, "dataSizeY", DYNAPSE_X4BOARD_NEUY);
	}

	state->doMapping = sshsNodeGetBool(moduleData->moduleNode, "doMapping");
	state->chipId = sshsNodeGetInt(moduleData->moduleNode, "chipId");


	// Nothing that can fail here.
	return (true);
}

static void caerDvsToDynapseRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	int16_t eventSourceID = va_arg(args, int);
	caerSpikeEventPacket spike = va_arg(args, caerSpikeEventPacket);	// from dynapse
	caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket); // from dvs

	// Only process packets with content.
	if (polarity == NULL) {
		return;
	}

	DvsToDynapseState state = moduleData->moduleState;

	// --- start  usb handle / from spike event source id
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(eventSourceID));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(eventSourceID));
	if(state->eventSourceModuleState == NULL || state->eventSourceConfigNode == NULL){
		return;
	}
	caerInputDynapseState stateSource = state->eventSourceModuleState;
	if(stateSource->deviceState == NULL){
		return;
	}
	//struct caer_dynapse_info dynapse_info = caerDynapseInfoGet(stateSource->deviceState);
	// --- end usb handle

	sshsNode sourceInfoNode = caerMainloopGetSourceInfo(caerEventPacketHeaderGetEventSource(&spike->packetHeader));
	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "dataSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "dataSizeY");

	// update filter parameters
	caerDvsToDynapseConfig(moduleData);

	// if mapping is on
	if(state->doMapping){
		caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, state->chipId);

		// prepare packet that will be sended via USB
		// Iterate over all DVS events
		int32_t numSpikes = caerEventPacketHeaderGetEventNumber(&polarity->packetHeader);
		uint32_t bits[numSpikes]; // = calloc(numSpikes, sizeof (uint8_t));
		uint32_t numConfig = 0;
		uint32_t idxConfig = 0;

		CAER_POLARITY_ITERATOR_VALID_START(polarity)
			int pol_x = caerPolarityEventGetX(caerPolarityIteratorElement);
			int pol_y = caerPolarityEventGetY(caerPolarityIteratorElement);
			bool pol_pol = caerPolarityEventGetPolarity(caerPolarityIteratorElement);

			// generate coordinates in  chip_size x chip_size
			int new_x = round (((double)pol_x / sizeX)*sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE));
			int new_y = round (((double)pol_y / sizeY)*sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE));

			int core_dest = 0 ;
			if(new_x < sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE) &&
					new_y < sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE) ){
				core_dest = 0;
			}else if(new_x >= sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE) &&
					new_y < sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE) ){
				core_dest = 2;
			}else if(new_x >= sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE) &&
					new_y >= sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE) ){
				core_dest = 3;
			}else if(new_x < sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE) &&
					new_y >= sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE) ){
				core_dest = 1;
			}

			if(core_dest == 1 || core_dest == 3){
				new_y = new_y - 16 ;
			}
			if(core_dest == 2 || core_dest == 3){
				new_x = new_x - 16 ;
			}

			// generate bits to send
			// depending on chipId we route them outside differently..
			// debug only - no stimulating -
			int dx;
			int dy;
			int sx;
			int sy;
			if(state->chipId == DYNAPSE_CONFIG_DYNAPSE_U0){
				dx = 0;
				dy = 2;
				sx = 0;
				sy = 1;
			}else if(state->chipId == DYNAPSE_CONFIG_DYNAPSE_U1){
				dx = 1;
				dy = 2;
				sx = 1;
				sy = 1;
			}else if(state->chipId == DYNAPSE_CONFIG_DYNAPSE_U2){
				dx = 0;
				dy = 1;
				sx = 0;
				sy = 1;
			}else if(state->chipId == DYNAPSE_CONFIG_DYNAPSE_U3){
				dx = 1;
				dy = 1;
				sx = 1;
				sy = 1;
			}else{
				dx = 0;
				dy = 2;
				sx = 0;
				sy = 1;
			}
			int core_d = 0;
			int core_s = core_dest;

			int address = new_x*sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE)+new_y;
			uint32_t value = core_d | 0 << 16
					| 0 << 17 | 1 << 13 |
					core_s << 18 |
					address << 20 |
					dx << 4 |
					sx << 6 |
					dy << 7 |
					sy << 9;

			bits[numConfig] = value;
			numConfig++;
		CAER_POLARITY_ITERATOR_VALID_END

		// send data with libusb host transfer in packet
		if(!caerDynapseSendDataToUSB(stateSource->deviceState, bits, numConfig)){
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
					"USB transfer failed, maybe you have tried to send too many data: numConfig is %d, "
					"however maxnumConfig %d", numConfig, DYNAPSE_MAX_USER_USB_PACKET_SIZE);
		}

	}

}

static void caerDvsToDynapseConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	DvsToDynapseState state = moduleData->moduleState;
	state->doMapping = sshsNodeGetBool(moduleData->moduleNode, "doMapping");
	state->chipId = sshsNodeGetInt(moduleData->moduleNode, "chipId");


}

static void caerDvsToDynapseExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	DvsToDynapseState state = moduleData->moduleState;


}

static void caerDvsToDynapseReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);

	DvsToDynapseState state = moduleData->moduleState;

}


