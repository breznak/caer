/*
 *
 *  Created on: Jan, 2017
 *      Author: federico.corradi@inilabs.com
 */

#include "dvstodynapse.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "libcaer/devices/dynapse.h"
#include "ext/buffers.h"

struct DvsToDynapse_state {
	sshsNode eventSourceModuleState;
	sshsNode eventSourceConfigNode;
	bool doMapping;
	uint16_t chipId;						//U0->0,U1->8,U2->4,U3->12
	simple2DBufferLong DownsampledMap;
	int threshold;							//after which we send spikes out
	bool programCam;
	bool programBiases;
};

typedef struct DvsToDynapse_state *DvsToDynapseState;

static bool caerDvsToDynapseInit(caerModuleData moduleData);
static void caerDvsToDynapseRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerDvsToDynapseConfig(caerModuleData moduleData);
static void caerDvsToDynapseExit(caerModuleData moduleData);
static void caerDvsToDynapseReset(caerModuleData moduleData, uint16_t resetCallSourceID);
static bool allocateDownsampledMap(DvsToDynapseState state, int16_t sourceID);

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
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "threshold", 10); // 0,4,8,12

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
	state->threshold = sshsNodeGetInt(moduleData->moduleNode, "threshold");
	state->DownsampledMap = NULL;
	state->programCam = true;
	state->programBiases = true;
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
	// --- end usb handle

	// program destination cam if necessary
	if(state->programCam){
		programMapInCam(stateSource, state);
	}
	// program destination biases if necessary
	if(state->programBiases){
		programBiases(stateSource, state);
	}
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo(caerEventPacketHeaderGetEventSource(&polarity->packetHeader));
	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "dvsSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "dvsSizeY");

	// update filter parameters
	caerDvsToDynapseConfig(moduleData);

	// if mapping is on
	if(state->doMapping){
		// If the map is not allocated yet, do it.
		if (state->DownsampledMap == NULL) {
			if (!allocateDownsampledMap(state, caerEventPacketHeaderGetEventSource(&polarity->packetHeader))) {
				// Failed to allocate memory, nothing to do.
				caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for DownsampledMap.");
				return;
			}
		}
		// select destination chip
		if(     state->chipId == DYNAPSE_CONFIG_DYNAPSE_U0 ||
				state->chipId == DYNAPSE_CONFIG_DYNAPSE_U1 ||
				state->chipId == DYNAPSE_CONFIG_DYNAPSE_U2 ||
				state->chipId == DYNAPSE_CONFIG_DYNAPSE_U3){
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, state->chipId);
		}else{
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Chip Id selected is non valid, please select one of 0,4,8,12\n");
		}
		// prepare packet that will be transferred via USB
		// Iterate over all DVS events
		int32_t numSpikes = caerEventPacketHeaderGetEventNumber(&polarity->packetHeader);

		// reset previous map downsampled
		for(size_t i=0; i<DYNAPSE_CONFIG_XCHIPSIZE; i++){
			for(size_t j=0; j<DYNAPSE_CONFIG_YCHIPSIZE; j++){
				state->DownsampledMap->buffer2d[i][j] = 0;
			}
		}

		CAER_POLARITY_ITERATOR_VALID_START(polarity)
			int pol_x = caerPolarityEventGetX(caerPolarityIteratorElement);
			int pol_y = caerPolarityEventGetY(caerPolarityIteratorElement);
			bool pol_pol = caerPolarityEventGetPolarity(caerPolarityIteratorElement);

			// generate coordinates in  chip_size x chip_size
			int new_x = round (((double)pol_x / sizeX)*DYNAPSE_CONFIG_XCHIPSIZE);
			int new_y = round (((double)pol_y / sizeY)*DYNAPSE_CONFIG_YCHIPSIZE);

			//update DownsampledMap
			if(new_x >= DYNAPSE_CONFIG_XCHIPSIZE){
				new_x = 31;
			}
			if(new_y >= DYNAPSE_CONFIG_YCHIPSIZE){
				new_y = 31;
			}
			// only positives spikes, bandwidth reduction
			if(pol_pol == 1){
				state->DownsampledMap->buffer2d[new_x][new_y] += 1;
			}else{
				state->DownsampledMap->buffer2d[new_x][new_y] -= 1;
			}
		CAER_POLARITY_ITERATOR_VALID_END

		// prepare data for usb transfer
		uint32_t bits[numSpikes];
		uint32_t numConfig = 0;
		uint32_t idxConfig = 0;
		bool dataReady = false;

		//only let pass above the state->threshold
		for(size_t i=0; i<DYNAPSE_CONFIG_XCHIPSIZE; i++){
			for(size_t j=0; j<DYNAPSE_CONFIG_YCHIPSIZE; j++){
				int nx = i;
				int ny = j;
				if(state->DownsampledMap->buffer2d[i][j] > state->threshold){
					// generate bits to send
					// core selection
					int core_dest = 0 ;
					if(i < sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE) &&
							j < sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE) ){
						core_dest = 0;
					}else if(i >= sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE) &&
							j < sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE) ){
						core_dest = 2;
					}else if(i >= sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE) &&
							j >= sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE) ){
						core_dest = 3;
					}else if(i < sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE) &&
							j >= sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE) ){
						core_dest = 1;
					}
					// adjusts coordinates
					if(core_dest == 1 || core_dest == 3){
						ny = ny - 16 ;
					}
					if(core_dest == 2 || core_dest == 3){
						nx = nx - 16 ;
					}
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
					int core_d;
					if(     state->chipId == DYNAPSE_CONFIG_DYNAPSE_U0 ||
									state->chipId == DYNAPSE_CONFIG_DYNAPSE_U1 ||
									state->chipId == DYNAPSE_CONFIG_DYNAPSE_U2 ||
									state->chipId == DYNAPSE_CONFIG_DYNAPSE_U3){
						core_d = state->chipId;
					}else{
						core_d = 0;
					}
					int core_s = core_dest;
					int address = nx*sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE)+ny;
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
					dataReady = true;
				}
			}
		}

		// we got data
		if(dataReady){
			// send data with libusb host transfer in packet
			if(!caerDynapseSendDataToUSB(stateSource->deviceState, bits, numConfig)){
				caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
						"USB transfer failed, maybe you have tried to send too many data: numConfig is %d, "
						"however maxnumConfig %d", numConfig, DYNAPSE_MAX_USER_USB_PACKET_SIZE);
			}
		}
	}

}

static void caerDvsToDynapseConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	DvsToDynapseState state = moduleData->moduleState;
	state->doMapping = sshsNodeGetBool(moduleData->moduleNode, "doMapping");
	uint16_t new_chip_id = sshsNodeGetInt(moduleData->moduleNode, "chipId");
	// if destionation chip id changed, we need to program the cam
	if(new_chip_id != state->chipId){
		state->chipId = new_chip_id;
		state->programCam = true;
		state->programBiases = true;
	}
	state->threshold = sshsNodeGetInt(moduleData->moduleNode, "threshold");

}

static void caerDvsToDynapseExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	DvsToDynapseState state = moduleData->moduleState;
	// Ensure maps are freed.
	simple2DBufferFreeLong(state->DownsampledMap);

}

static void caerDvsToDynapseReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);

}

static bool allocateDownsampledMap(DvsToDynapseState state, int16_t sourceID) {
	// Get size information from source.
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo(U16T(sourceID));
	if (sourceInfoNode == NULL) {
		// This should never happen, but we handle it gracefully.
		caerLog(CAER_LOG_ERROR, __func__, "Failed to get source info to allocate map.");
		return (false);
	}

	int16_t sizeX = DYNAPSE_CONFIG_XCHIPSIZE;
	int16_t sizeY = DYNAPSE_CONFIG_YCHIPSIZE;

	state->DownsampledMap = simple2DBufferInitLong((size_t) sizeX, (size_t) sizeY);
	if (state->DownsampledMap == NULL) {
		return (false);
	}

	for(size_t x=0; x<sizeX; x++){
		for(size_t y=0; y<sizeY; y++){
			state->DownsampledMap->buffer2d[x][y] = 0; // init to zero
		}
	}

	return (true);
}

/* program destinatio cam addresses 64 - synapses per input -*/
void programMapInCam(caerInputDynapseState state, DvsToDynapseState stateMod) {
	if (state == NULL) {
		return;
	}
	caerDeviceHandle usb_handle = (caerDeviceHandle) state->deviceState;
	caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, stateMod->chipId); //U0->0,U1->8,U2->4,U3->12
	uint32_t neuronId, camId;
	uint32_t bits[DYNAPSE_CONFIG_NUMNEURONS_CORE*DYNAPSE_CONFIG_NUMCAM];
	uint32_t numConfig = -1;
	caerLog(CAER_LOG_NOTICE, "DvsToDynapse", "Started programming cam..");
	for (neuronId = 0; neuronId < DYNAPSE_CONFIG_NUMNEURONS_CORE; neuronId++) {
		numConfig = -1;
		for (camId = 0; camId < DYNAPSE_CONFIG_NUMCAM; camId++) {
			numConfig++;
			bits[numConfig]=caerDynapseGenerateCamBits(neuronId, neuronId, camId, DYNAPSE_CONFIG_CAMTYPE_F_EXC);
		}
		// send data with libusb host transfer in packet
		if(!caerDynapseSendDataToUSB(usb_handle, bits, numConfig)){
			caerLog(CAER_LOG_ERROR, "DvsToDynapse", "USB transfer failed, ");
		}
	}
	caerLog(CAER_LOG_NOTICE, "DvsToDynapse", "CAM programmed successfully.");
	stateMod->programCam = false;
}

void programBiases(caerInputDynapseState state, DvsToDynapseState stateMod) {
	if (state == NULL) {
		return;
	}
	caerDeviceHandle usb_handle = (caerDeviceHandle) state->deviceState;
	caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, stateMod->chipId); //U0->0,U1->8,U2->4,U3->12
	// set biases for all cores
	for (size_t coreId = 0; coreId < 4; coreId++) {
		// soma
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "IF_AHTHR_N", 7, 1, "HighBias", "NBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "IF_AHW_P", 7, 1, "HighBias", "PBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "IF_BUF_P", 3, 80, "HighBias", "PBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "IF_DC_P", 5, 2, "HighBias", "PBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "IF_RFR_N", 2, 180, "HighBias", "NBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "IF_TAU1_N", 4, 255, "LowBias", "NBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "IF_THR_N", 2, 180, "HighBias", "NBias");
		//synapses
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "NPDPIE_TAU_F_P", 6, 150, "HighBias", "PBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "NPDPIE_THR_F_P", 0, 220, "HighBias", "PBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "PS_WEIGHT_EXC_F_N", 0, 250, "HighBias", "NBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "R2R_P", 4, 85, "HighBias", "PBias");
	}
	stateMod->programBiases = false;
}
