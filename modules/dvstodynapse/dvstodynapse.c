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
	bool programCamAllChips;
	bool singleChipMode;
};

typedef struct DvsToDynapse_state *DvsToDynapseState;

static bool caerDvsToDynapseInit(caerModuleData moduleData);
static void caerDvsToDynapseRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerDvsToDynapseConfig(caerModuleData moduleData);
static void caerDvsToDynapseExit(caerModuleData moduleData);
static void caerDvsToDynapseReset(caerModuleData moduleData, uint16_t resetCallSourceID);
static bool allocateDownsampledMap(DvsToDynapseState state, int16_t sourceID);
void programBiases(caerInputDynapseState state, DvsToDynapseState stateMod);
void programMapInCam(caerInputDynapseState state, DvsToDynapseState stateMod);
void programMapInAllCam(caerInputDynapseState state, DvsToDynapseState stateMod);

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
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "chipId", DYNAPSE_CONFIG_DYNAPSE_U3); // 0,4,8,12
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "threshold", 10);
	sshsNodePutBool(moduleData->moduleNode, "programCamAllChips", false);
	sshsNodePutBool(moduleData->moduleNode, "singleChipMode", true);

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
	state->programCamAllChips = sshsNodeGetBool(moduleData->moduleNode, "programCamAllChips"); // false at startup
	state->singleChipMode = sshsNodeGetBool(moduleData->moduleNode, "singleChipMode"); // true at startup
	state->DownsampledMap = NULL;
	state->programBiases = true;
	state->programCam = true;
	//state->programFourChips = false;
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
	// program as destination all chips in board
	if(state->programCamAllChips){
		programMapInAllCam(stateSource, state);
		sshsNodePutBool(moduleData->moduleNode, "programCamAllChips", false);

		uint16_t allChips[4] = {DYNAPSE_CONFIG_DYNAPSE_U0,DYNAPSE_CONFIG_DYNAPSE_U1,
											DYNAPSE_CONFIG_DYNAPSE_U2,DYNAPSE_CONFIG_DYNAPSE_U3};

		for(size_t this_chip=0; this_chip < 4; this_chip++){
			state->chipId = allChips[this_chip];
			programBiases(stateSource, state);
		}
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

		// prepare packet that will be transferred via USB
		// Iterate over all DVS events
		int32_t numSpikes = caerEventPacketHeaderGetEventNumber(&polarity->packetHeader);

		// reset previous map downsampled
		for(size_t i=0; i<DYNAPSE_X4BOARD_NEUX; i++){
			for(size_t j=0; j<DYNAPSE_X4BOARD_NEUY; j++){
				state->DownsampledMap->buffer2d[i][j] = 0;
			}
		}

		CAER_POLARITY_ITERATOR_VALID_START(polarity)
			int pol_x = caerPolarityEventGetX(caerPolarityIteratorElement);
			int pol_y = caerPolarityEventGetY(caerPolarityIteratorElement);
			bool pol_pol = caerPolarityEventGetPolarity(caerPolarityIteratorElement);


			// generate coordinates in  chip_size x chip_size
			int new_x, new_y;
			if(state->singleChipMode){
				new_x = round (((double)pol_x / sizeX)*DYNAPSE_CONFIG_XCHIPSIZE);
				new_y = round (((double)pol_y / sizeY)*DYNAPSE_CONFIG_YCHIPSIZE);
				//update DownsampledMap
				if(new_x >= DYNAPSE_CONFIG_XCHIPSIZE){
					new_x = DYNAPSE_CONFIG_XCHIPSIZE-1;
				}
				if(new_y >= DYNAPSE_CONFIG_YCHIPSIZE){
					new_y = DYNAPSE_CONFIG_YCHIPSIZE-1;
				}
			}else{
				//not single chip, but use all 4 chips
				new_x = round (((double)pol_x / sizeX)*DYNAPSE_X4BOARD_NEUX);
				new_y = round (((double)pol_y / sizeY)*DYNAPSE_X4BOARD_NEUY);
				//update DownsampledMap
				if(new_x >= DYNAPSE_X4BOARD_NEUX){
					new_x = DYNAPSE_X4BOARD_NEUX-1;
				}
				if(new_y >= DYNAPSE_X4BOARD_NEUY){
					new_y = DYNAPSE_X4BOARD_NEUX-1;
				}
			}

			// only positives spikes, usb bandwidth reduction
			if(pol_pol == 1){
				state->DownsampledMap->buffer2d[new_x][new_y] += 1;
			}else{
				state->DownsampledMap->buffer2d[new_x][new_y] -= 1;
			}
		CAER_POLARITY_ITERATOR_VALID_END

		// prepare data for usb transfer

		// case single chip mapping
		uint32_t bits[numSpikes];
		uint32_t numConfig = 0;

		// case multi chip mapping
		uint32_t bits_chipU0[numSpikes];
		uint32_t bits_chipU1[numSpikes];
		uint32_t bits_chipU2[numSpikes];
		uint32_t bits_chipU3[numSpikes];
		uint32_t numConfig_chipU0 = 0;
		uint32_t numConfig_chipU1 = 0;
		uint32_t numConfig_chipU2 = 0;
		uint32_t numConfig_chipU3 = 0;

		bool dataReady = false;
		bool dataReady_multi = false;

		//only let pass above the state->threshold
		if(state->singleChipMode){
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
							int core_s = core_dest;
							int address = nx*sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE)+ny;

							uint32_t value = 15 | 0 << 16
															| 0 << 17 | 1 << 13 |
															core_s << 18 |
															address << 20 |
															0 << 4 |
															0 << 6 |
															0 << 7 |
															0 << 9;

							bits[numConfig] = value;
							numConfig++;
							dataReady = true;

					}
				}
			}
		}else{
			for(size_t i=0; i<DYNAPSE_X4BOARD_NEUX; i++){
				for(size_t j=0; j<DYNAPSE_X4BOARD_NEUY; j++){
					int nx = i;
					int ny = j;
					if(state->DownsampledMap->buffer2d[i][j] > state->threshold){
						// multichip mode, chip selection
						int chip_dest;
						if(i < DYNAPSE_CONFIG_XCHIPSIZE &&
								j < DYNAPSE_CONFIG_YCHIPSIZE ){
							chip_dest = DYNAPSE_CONFIG_DYNAPSE_U0;
						}else if(i >= DYNAPSE_CONFIG_XCHIPSIZE &&
								j < DYNAPSE_CONFIG_YCHIPSIZE ){
							chip_dest = DYNAPSE_CONFIG_DYNAPSE_U1;
						}else if(i >= DYNAPSE_CONFIG_XCHIPSIZE &&
								j >= DYNAPSE_CONFIG_YCHIPSIZE ){
							chip_dest = DYNAPSE_CONFIG_DYNAPSE_U3;
						}else if(i < DYNAPSE_CONFIG_XCHIPSIZE &&
								j >= DYNAPSE_CONFIG_YCHIPSIZE ){
							chip_dest = DYNAPSE_CONFIG_DYNAPSE_U2;
						}
						// core selection
						int core_dest = 0;
						if(chip_dest == DYNAPSE_CONFIG_DYNAPSE_U0){
							if(i < 16 &&
									j < 16 ){
								core_dest = 0;
							}else if(i >= 16 &&
									j < 16 ){
								core_dest = 2;
							}else if(i >= 16 &&
									j >= 16 ){
								core_dest = 3;
							}else if(i < 16 &&
									j >= 16 ){
								core_dest = 1;
							}
						}else if(chip_dest == DYNAPSE_CONFIG_DYNAPSE_U2){
							if(i < 16 &&
									j < 16*3 ){
								core_dest = 0;
							}else if(i >= 16 &&
									j < 16*3 ){
								core_dest = 2;
							}else if(i >= 16*3 &&
									j >= 16*3 ){
								core_dest = 3;
							}else if(i < 16 &&
									j >= 16*3 ){
								core_dest = 1;
							}
						}else if(chip_dest == DYNAPSE_CONFIG_DYNAPSE_U3){
							if(i < 16*3 &&
									j < 16 ){
								core_dest = 0;
							}else if(i >= 16*3 &&
									j < 16 ){
								core_dest = 2;
							}else if(i >= 16*3 &&
									j >= 16 ){
								core_dest = 3;
							}else if(i < 16*3 &&
									j >= 16 ){
								core_dest = 1;
							}
						}else if(chip_dest == DYNAPSE_CONFIG_DYNAPSE_U3){
							if(i < 16*3 &&
									j < 16*3 ){
								core_dest = 0;
							}else if(i >= 16*3 &&
									j < 16*3 ){
								core_dest = 2;
							}else if(i >= 16*3 &&
									j >= 16*3 ){
								core_dest = 3;
							}else if(i < 16*3 &&
									j >= 16*3 ){
								core_dest = 1;
							}
						}
						// adjusts coordinates for chips
						if(chip_dest == DYNAPSE_CONFIG_DYNAPSE_U2 ||
								chip_dest == DYNAPSE_CONFIG_DYNAPSE_U3){
							ny = ny - 32 ;
						}
						if(chip_dest == DYNAPSE_CONFIG_DYNAPSE_U1 ||
								chip_dest == DYNAPSE_CONFIG_DYNAPSE_U3){
							nx = nx - 32 ;
						}
						// adjusts coordinates for cores
						if(core_dest == 1 || core_dest == 3){
							ny = ny - 16 ;
						}
						if(core_dest == 2 || core_dest == 3){
							nx = nx - 16 ;
						}

						//caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "nx %d - ny %d, ", nx, ny);

						int core_s = core_dest;
						int address = nx*sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE)+ny;

						uint32_t value = 15 | 0 << 16
														| 0 << 17 | 1 << 13 |
														core_s << 18 |
														address << 20 |
														0 << 4 |
														0 << 6 |
														0 << 7 |
														0 << 9;

						if(chip_dest == DYNAPSE_CONFIG_DYNAPSE_U0){
							bits_chipU0[numConfig_chipU0] = value;
							numConfig_chipU0++;
						}else if(chip_dest == DYNAPSE_CONFIG_DYNAPSE_U1){
							bits_chipU1[numConfig_chipU1] = value;
							numConfig_chipU1++;
						}else if(chip_dest == DYNAPSE_CONFIG_DYNAPSE_U2){
							bits_chipU2[numConfig_chipU2] = value;
							numConfig_chipU2++;
						}else if(chip_dest == DYNAPSE_CONFIG_DYNAPSE_U3){
							bits_chipU3[numConfig_chipU3] = value;
							numConfig_chipU3++;
						}

						dataReady_multi = true;
					}
				}
			}
		}

		// map on multiple devices
		if(dataReady_multi){
			// send data with libusb host transfer in packet
			if(numConfig_chipU0>0){
				// select destination chip
				caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U0);
				//send data
				if(!caerDynapseSendDataToUSB(stateSource->deviceState, bits_chipU0, numConfig_chipU0)){
					caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
							"USB transfer failed");
				}
			}
			if(numConfig_chipU1>0){
				// select destination chip
				caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U1);
				//send data
				if(!caerDynapseSendDataToUSB(stateSource->deviceState, bits_chipU1, numConfig_chipU1)){
					caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
							"USB transfer failed");
				}
			}
			if(numConfig_chipU2>0){
				// select destination chip
				caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U2);
				//send data
				if(!caerDynapseSendDataToUSB(stateSource->deviceState, bits_chipU2, numConfig_chipU2)){
					caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
							"USB transfer failed");
				}
			}
			if(numConfig_chipU3>0){
				// select destination chip
				caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U3);
				//send data
				if(!caerDynapseSendDataToUSB(stateSource->deviceState, bits_chipU3, numConfig_chipU3)){
					caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
							"USB transfer failed");
				}
			}
		}
		// we got data
		if(dataReady){
			// select destination chip
			if(     state->chipId == DYNAPSE_CONFIG_DYNAPSE_U0 ||
					state->chipId == DYNAPSE_CONFIG_DYNAPSE_U1 ||
					state->chipId == DYNAPSE_CONFIG_DYNAPSE_U2 ||
					state->chipId == DYNAPSE_CONFIG_DYNAPSE_U3){
				caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, state->chipId);
			}else{
				caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Chip Id selected is non valid, please select one of 0,4,8,12\n");
			}
			// send data with libusb host transfer in packet
			if(!caerDynapseSendDataToUSB(stateSource->deviceState, bits, numConfig)){
				caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
						"USB transfer failed");
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
	state->programCamAllChips = sshsNodeGetBool(moduleData->moduleNode, "programCamAllChips");
	state->programCamAllChips = sshsNodeGetBool(moduleData->moduleNode, "programCamAllChips");
	// here the user needs to make sure to program all cams
	state->singleChipMode = sshsNodeGetBool(moduleData->moduleNode, "singleChipMode");
	//chipid 0,4,8,12 -> programMapInCam(), loadBiases, calculateNeurons
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

	int16_t sizeX = DYNAPSE_X4BOARD_NEUX;
	int16_t sizeY = DYNAPSE_X4BOARD_NEUY;

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

/* program destination cam addresses 64 - synapses per input -*/
void programMapInAllCam(caerInputDynapseState state, DvsToDynapseState stateMod) {
	if (state == NULL) {
		return;
	}
	uint32_t allChipsInBoards[4] = {DYNAPSE_CONFIG_DYNAPSE_U0,DYNAPSE_CONFIG_DYNAPSE_U1,
									DYNAPSE_CONFIG_DYNAPSE_U2,DYNAPSE_CONFIG_DYNAPSE_U3};
	caerDeviceHandle usb_handle = (caerDeviceHandle) state->deviceState;

	for(size_t this_chip=0; this_chip < 4; this_chip++){
		//stateMod->chipId = allChipsInBoards[this_chip];
		caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, allChipsInBoards[this_chip]); //U0->0,U1->8,U2->4,U3->12
		uint32_t neuronId, camId;
		uint32_t bits[DYNAPSE_CONFIG_NUMNEURONS*DYNAPSE_CONFIG_NUMCAM];
		uint32_t numConfig = -1;
		caerLog(CAER_LOG_NOTICE, "DvsToDynapse", "Started cleaning cam chipId %d.", allChipsInBoards[this_chip]);
		for (size_t neuronId = 0; neuronId < DYNAPSE_CONFIG_NUMNEURONS; neuronId++) {
			numConfig = -1;
			for (size_t camId = 0; camId < DYNAPSE_CONFIG_NUMCAM; camId++) {
				numConfig++;
				bits[numConfig]=caerDynapseGenerateCamBits(0, neuronId, camId, DYNAPSE_CONFIG_CAMTYPE_F_EXC);
			}
			// send data with libusb host transfer in packet
			if(!caerDynapseSendDataToUSB(usb_handle, bits, numConfig)){
				caerLog(CAER_LOG_ERROR, "DvsToDynapse", "USB transfer failed.");
			}
		}
		caerLog(CAER_LOG_NOTICE, "DvsToDynapse", "CAM cleaned successfully, chipId %d.", allChipsInBoards[this_chip]);

		caerLog(CAER_LOG_NOTICE, "DvsToDynapse", "Programming CAM content, chipId %d..", allChipsInBoards[this_chip]);
		caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, allChipsInBoards[this_chip]);
		for (neuronId = 0;
				neuronId < DYNAPSE_CONFIG_XCHIPSIZE * DYNAPSE_CONFIG_YCHIPSIZE;
				neuronId++) {
			caerDynapseWriteCam(state->deviceState, neuronId, neuronId, 0, DYNAPSE_CONFIG_CAMTYPE_F_EXC);
		}
		caerLog(CAER_LOG_NOTICE, "DvsToDynapse", "CAM programmed successfully, chipId %d..", allChipsInBoards[this_chip]);
	}

	stateMod->programCam = false;
}

/* program destination cam addresses 64 - synapses per input -*/
void programMapInCam(caerInputDynapseState state, DvsToDynapseState stateMod) {
	if (state == NULL) {
		return;
	}
	caerDeviceHandle usb_handle = (caerDeviceHandle) state->deviceState;
	caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, stateMod->chipId); //U0->0,U1->8,U2->4,U3->12
	uint32_t neuronId, camId;
	uint32_t bits[DYNAPSE_CONFIG_NUMNEURONS*DYNAPSE_CONFIG_NUMCAM];
	uint32_t numConfig = -1;
	caerLog(CAER_LOG_NOTICE, "DvsToDynapse", "Started cleaning cam..");
	for (size_t neuronId = 0; neuronId < DYNAPSE_CONFIG_NUMNEURONS; neuronId++) {
		numConfig = -1;
		for (size_t camId = 0; camId < DYNAPSE_CONFIG_NUMCAM; camId++) {
			numConfig++;
			bits[numConfig]=caerDynapseGenerateCamBits(0, neuronId, camId, DYNAPSE_CONFIG_CAMTYPE_F_EXC);
		}
		// send data with libusb host transfer in packet
		if(!caerDynapseSendDataToUSB(usb_handle, bits, numConfig)){
			caerLog(CAER_LOG_ERROR, "DvsToDynapse", "USB transfer failed, ");
		}
	}
	caerLog(CAER_LOG_NOTICE, "DvsToDynapse", "CAM cleaned successfully.");

	caerLog(CAER_LOG_NOTICE, "DvsToDynapse", "Programming CAM content...");
	caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, stateMod->chipId);
	for (neuronId = 0;
			neuronId < DYNAPSE_CONFIG_XCHIPSIZE * DYNAPSE_CONFIG_YCHIPSIZE;
			neuronId++) {
		caerDynapseWriteCam(state->deviceState, neuronId, neuronId, 0, DYNAPSE_CONFIG_CAMTYPE_F_EXC);
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
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "IF_AHTAU_N", 7, 35, "LowBias","NBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "IF_AHTHR_N", 7, 1, "HighBias","NBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "IF_AHW_P", 7, 1, "HighBias","PBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "IF_BUF_P", 3, 80, "HighBias","PBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "IF_CASC_N", 7, 1, "HighBias","NBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "IF_DC_P", 5, 2, "HighBias","PBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "IF_NMDA_N", 7, 1, "HighBias","NBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "IF_RFR_N", 2, 180, "HighBias","NBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "IF_TAU1_N", 4, 255, "LowBias","NBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "IF_TAU2_N", 6, 15, "LowBias","NBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "IF_THR_N", 2, 180, "HighBias","NBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "NPDPIE_TAU_F_P", 6, 150,"HighBias", "PBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "NPDPIE_TAU_S_P", 7, 40,"HighBias", "NBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "NPDPIE_THR_F_P", 0, 220,"HighBias", "PBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "NPDPIE_THR_S_P", 7, 0,"HighBias", "PBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "NPDPII_TAU_F_P", 6, 150,"HighBias", "NBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "NPDPII_TAU_S_P", 7, 40,"HighBias", "NBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "NPDPII_THR_F_P", 0, 220,"HighBias", "PBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "NPDPII_THR_S_P", 7, 0,"HighBias", "PBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "PS_WEIGHT_EXC_F_N", 0, 250,"HighBias", "NBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "PS_WEIGHT_EXC_S_N", 1, 250,"HighBias", "NBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "PS_WEIGHT_INH_F_N", 0, 250,"HighBias", "NBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "PS_WEIGHT_INH_S_N", 1, 250,"HighBias", "NBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "PULSE_PWLK_P", 3, 50,"HighBias", "PBias");
		caerDynapseSetBiasBits(state, stateMod->chipId, coreId, "R2R_P", 4, 85, "HighBias","PBias");
	}
	stateMod->programBiases = false;
}

