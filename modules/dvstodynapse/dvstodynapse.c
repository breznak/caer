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
	caerInputDynapseState eventSourceModuleState;
	sshsNode eventSourceConfigNode;
	bool doMapping;
	int chipId;						//U0->0,U1->8,U2->4,U3->12
	simple2DBufferLong DownsampledMap;
	int threshold;							//after which we send spikes out
	bool programCam;
	bool programBiases;
	bool programCamAllChips;
	bool singleChipMode;
	bool programConvolutionMapping;
	bool enableMapping;
};

typedef struct DvsToDynapse_state *DvsToDynapseState;

static bool caerDvsToDynapseInit(caerModuleData moduleData);
static void caerDvsToDynapseRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerDvsToDynapseConfig(caerModuleData moduleData);
static void caerDvsToDynapseExit(caerModuleData moduleData);
static void caerDvsToDynapseReset(caerModuleData moduleData, uint16_t resetCallSourceID);
static bool allocateDownsampledMap(DvsToDynapseState state, int16_t sourceID);
void programBiasesDvsToDynapse(caerInputDynapseState state, DvsToDynapseState stateMod);
void programMapInCam(caerInputDynapseState state, DvsToDynapseState stateMod);
//void programMapInAllCam(caerInputDynapseState state, DvsToDynapseState stateMod);
void programConvolutionMappingSram(caerInputDynapseState state, DvsToDynapseState stateMod, caerModuleData module);

static struct caer_module_functions caerDvsToDynapseFunctions = { .moduleInit =
	&caerDvsToDynapseInit, .moduleRun = &caerDvsToDynapseRun, .moduleConfig =
	&caerDvsToDynapseConfig, .moduleExit = &caerDvsToDynapseExit, .moduleReset =
	&caerDvsToDynapseReset };

void caerDvsToDynapse(uint16_t moduleID,  caerSpikeEventPacket spike, caerPolarityEventPacket polarity) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "DvsToDynapse", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerDvsToDynapseFunctions, moduleData, sizeof(struct DvsToDynapse_state), 2, spike, polarity);
}

static bool caerDvsToDynapseInit(caerModuleData moduleData) {

	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doMapping", false);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "chipId", DYNAPSE_CONFIG_DYNAPSE_U3); // 0,4,8,12
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "threshold", 10);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "programCamAllChips", true);
	//sshsNodePutBoolIfAbsent(moduleData->moduleNode, "singleChipMode", true);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "programConvolutionMapping", true);
	// set default values
	sshsNodePutBool(moduleData->moduleNode, "programCamAllChips", true);
	sshsNodePutBool(moduleData->moduleNode, "singleChipMode", true);
	sshsNodePutBool(moduleData->moduleNode, "programConvolutionMapping", false);

	DvsToDynapseState state = moduleData->moduleState;

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	if (!sshsNodeAttributeExists(sourceInfoNode, "dataSizeX", SSHS_SHORT)) { //to do for visualizer change name of field to a more generic one
		sshsNodePutShort(sourceInfoNode, "dataSizeX", DYNAPSE_X4BOARD_NEUX);
		sshsNodePutShort(sourceInfoNode, "dataSizeY", DYNAPSE_X4BOARD_NEUY);
	}

	state->doMapping = sshsNodeGetBool(moduleData->moduleNode, "doMapping");
	state->chipId = (uint16_t) sshsNodeGetInt(moduleData->moduleNode, "chipId");
	state->threshold = sshsNodeGetInt(moduleData->moduleNode, "threshold");
	state->programCamAllChips = sshsNodeGetBool(moduleData->moduleNode, "programCamAllChips"); // false at startup
	state->singleChipMode = true;//sshsNodeGetBool(moduleData->moduleNode, "singleChipMode"); // true at startup
	state->DownsampledMap = NULL;
	state->programBiases = true;
	state->programCam = true;
	state->programConvolutionMapping = sshsNodeGetBool(moduleData->moduleNode, "programConvolutionMapping"); // true at startup
	state->enableMapping = false;

	// Nothing that can fail here.
	return (true);
}

static void caerDvsToDynapseRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerSpikeEventPacket spike = va_arg(args, caerSpikeEventPacket);	// from dynapse
	caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket); // from dvs

	// Only process packets with content.
	if (polarity == NULL || spike == NULL) {
		return;
	}

	DvsToDynapseState state = moduleData->moduleState;

	// --- start  usb handle / from spike event source id
	int eventSourceID = caerEventPacketHeaderGetEventSource(&spike->packetHeader);
	state->eventSourceModuleState = (caerInputDynapseState) caerMainloopGetSourceState(U16T(eventSourceID));
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
		programBiasesDvsToDynapse(stateSource, state);
	}
	// program as destination all chips in board
	if(state->programCamAllChips){

		int allChips[3] = {DYNAPSE_CONFIG_DYNAPSE_U1,DYNAPSE_CONFIG_DYNAPSE_U2,DYNAPSE_CONFIG_DYNAPSE_U3};
		for(size_t this_chip=0; this_chip < 3; this_chip++){
			state->chipId =  allChips[this_chip];
			sshsNodePutInt(moduleData->moduleNode, "chipId", allChips[this_chip]);
			caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Programming chip id %d,", state->chipId);
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, (uint32_t) state->chipId);
			programMapInCam(stateSource, state);
			programBiasesDvsToDynapse(stateSource, state);
		}
		sshsNodePutBool(moduleData->moduleNode, "programCamAllChips", false);
		state->programCamAllChips = false;
		programConvolutionMappingSram(stateSource, state, moduleData);
		// now switch on mapping
		state->enableMapping = true;
	}
	// program convolution mapping
	// we map 8x8 kernels to chip U1, Chip U2
	if(state->programConvolutionMapping){
		programConvolutionMappingSram(stateSource, state, moduleData);
	}

	sshsNode sourceInfoNode = caerMainloopGetSourceInfo( (uint16_t) caerEventPacketHeaderGetEventSource(&polarity->packetHeader));
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
				new_x = (int) round (((double)pol_x / sizeX)*DYNAPSE_CONFIG_XCHIPSIZE);
				new_y = (int) round (((double)pol_y / sizeY)*DYNAPSE_CONFIG_YCHIPSIZE);
				//update DownsampledMap
				if(new_x >= DYNAPSE_CONFIG_XCHIPSIZE){
					new_x = DYNAPSE_CONFIG_XCHIPSIZE-1;
				}
				if(new_y >= DYNAPSE_CONFIG_YCHIPSIZE){
					new_y = DYNAPSE_CONFIG_YCHIPSIZE-1;
				}
			}else{
				//not single chip, but use all 4 chips
				new_x = (int) round (((double)pol_x / sizeX)*DYNAPSE_X4BOARD_NEUX);
				new_y = (int) round (((double)pol_y / sizeY)*DYNAPSE_X4BOARD_NEUY);
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
		int bits[DYNAPSE_MAX_USER_USB_PACKET_SIZE];
		int numConfig = 0;

		// case multi chip mapping
		int bits_chipU0[DYNAPSE_MAX_USER_USB_PACKET_SIZE];
		int bits_chipU1[DYNAPSE_MAX_USER_USB_PACKET_SIZE];
		int bits_chipU2[DYNAPSE_MAX_USER_USB_PACKET_SIZE];
		int bits_chipU3[DYNAPSE_MAX_USER_USB_PACKET_SIZE];
		int numConfig_chipU0 = 0;
		int numConfig_chipU1 = 0;
		int numConfig_chipU2 = 0;
		int numConfig_chipU3 = 0;

		bool dataReady = false;
		bool dataReady_multi = false;

		//only let pass above the state->threshold
		if(state->singleChipMode){
			for(int i=0; i<DYNAPSE_CONFIG_XCHIPSIZE; i++){
				for(int j=0; j<DYNAPSE_CONFIG_YCHIPSIZE; j++){
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
							int address = (int) nx*16+(int)ny;

							uint32_t value = (uint32_t) 15 | 0 << 16
															| 0 << 17 | 1 << 13 |
															(uint) core_s << 18 |
															(uint) address << 20 |
															0 << 4 |
															0 << 6 |
															0 << 7 |
															0 << 9;

							if(numConfig+1 >= DYNAPSE_MAX_USER_USB_PACKET_SIZE){
								caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Breaking transaction\n");
								// we got data
								// select destination chip
								if(     state->chipId == DYNAPSE_CONFIG_DYNAPSE_U0 ||
										state->chipId == DYNAPSE_CONFIG_DYNAPSE_U1 ||
										state->chipId == DYNAPSE_CONFIG_DYNAPSE_U2 ||
										state->chipId == DYNAPSE_CONFIG_DYNAPSE_U3){
									caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, (uint32_t) state->chipId);
								}else{
									caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Chip Id selected is non valid, please select one of 0,4,8,12\n");
								}
								// send data with libusb host transfer in packet
								if(!caerDynapseSendDataToUSB(stateSource->deviceState, bits, (int) numConfig)){
									caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
											"USB transfer failed");
								}
								numConfig = 0;
								dataReady = false;
							}else{
								bits[numConfig] = (int) value;
								numConfig++;
								dataReady = true;
							}


					}
				}
			}
		}else{
			for(int i=0; i<DYNAPSE_X4BOARD_NEUX; i++){
				for(int j=0; j<DYNAPSE_X4BOARD_NEUY; j++){
					int nx = i;
					int ny = j;
					if(state->DownsampledMap->buffer2d[i][j] > state->threshold){
						// multichip mode, chip selection
						int chip_dest = 0 ;
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
						int core_s = core_dest;
						int address = (int) nx*16+(int)ny;

						uint32_t value = (uint32_t) 15 | 0 << 16
														| 0 << 17 | 1 << 13 |
														core_s << 18 |
														address << 20 |
														0 << 4 |
														0 << 6 |
														0 << 7 |
														0 << 9;

						if(numConfig_chipU0+1 >= DYNAPSE_MAX_USER_USB_PACKET_SIZE ||
							numConfig_chipU1+1 >= DYNAPSE_MAX_USER_USB_PACKET_SIZE ||
							numConfig_chipU2+1 >= DYNAPSE_MAX_USER_USB_PACKET_SIZE ||
							numConfig_chipU3+1 >= DYNAPSE_MAX_USER_USB_PACKET_SIZE){
							caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Breaking transaction\n");
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
							numConfig_chipU0 = 0;
							numConfig_chipU1 = 0;
							numConfig_chipU2 = 0;
							numConfig_chipU3 = 0;
							dataReady_multi = false;
						}else{
							if(chip_dest == DYNAPSE_CONFIG_DYNAPSE_U0){
								bits_chipU0[numConfig_chipU0] = (int) value;
								numConfig_chipU0++;
							}else if(chip_dest == DYNAPSE_CONFIG_DYNAPSE_U1){
								bits_chipU1[numConfig_chipU1] =  (int) value;
								numConfig_chipU1++;
							}else if(chip_dest == DYNAPSE_CONFIG_DYNAPSE_U2){
								bits_chipU2[numConfig_chipU2] =  (int) value;
								numConfig_chipU2++;
							}else if(chip_dest == DYNAPSE_CONFIG_DYNAPSE_U3){
								bits_chipU3[numConfig_chipU3] =  (int) value;
								numConfig_chipU3++;
							}
							dataReady_multi = true;
						}

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
				caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, (uint32_t) state->chipId);
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

	if(state->enableMapping){
		sshsNodePutBool(moduleData->moduleNode, "doMapping", true);
		state->doMapping = true;
	}
}

static void caerDvsToDynapseConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	DvsToDynapseState state = moduleData->moduleState;
	state->doMapping = sshsNodeGetBool(moduleData->moduleNode, "doMapping");
	uint16_t new_chip_id = (uint16_t) sshsNodeGetInt(moduleData->moduleNode, "chipId");
	// if destionation chip id changed, we need to program the cam
	if(new_chip_id != state->chipId){
		state->chipId = new_chip_id;
		state->programCam = true;
		state->programBiases = true;
	}
	state->threshold = sshsNodeGetInt(moduleData->moduleNode, "threshold");
	state->programCamAllChips = sshsNodeGetBool(moduleData->moduleNode, "programCamAllChips");
	state->programConvolutionMapping =  sshsNodeGetBool(moduleData->moduleNode, "programConvolutionMapping");

	// here the user needs to make sure to program all cams
	state->singleChipMode = sshsNodeGetBool(moduleData->moduleNode, "singleChipMode");
	//chipid 0,4,8,12 -> programMapInCam(), loadBiases, calculateNeurons
}

static void caerDvsToDynapseExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	DvsToDynapseState state = moduleData->moduleState;
	//disable mapping
	sshsNodePutBool(moduleData->moduleNode, "doMapping", false);
	state->doMapping = false;
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

/* program SRAM cam - to map to convolutions -*/
void programConvolutionMappingSram(caerInputDynapseState state, DvsToDynapseState stateMod, caerModuleData moduleD) {

	if(stateMod->chipId != DYNAPSE_CONFIG_DYNAPSE_U3){
		caerLog(CAER_LOG_ERROR, "DvsToDynapse", "Mapping convolution is not possible with chipId different than 12!!");
	}else{
		caerDeviceHandle usb_handle = (caerDeviceHandle) state->deviceState;
		caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U3);
		// proceed with the mapping
		// init sram_id = 1;
		int bits[DYNAPSE_MAX_USER_USB_PACKET_SIZE];
		int numConfig = -1;
		caerLog(CAER_LOG_NOTICE, "DvsToDynapse", "Mapping convolutions started..");
		for(int sramId=1; sramId < 3; sramId++){
			numConfig = -1;
			for(int neuronId = 0; neuronId < DYNAPSE_CONFIG_NUMNEURONS_CORE; neuronId++){
				for(int coreId = 0; coreId < DYNAPSE_CONFIG_NUMCORES; ++coreId){
					numConfig++;
					uint8_t dx = 0;
					uint8_t sx = 0;
					uint8_t dy = 0;
					uint8_t sy = 0;
					uint16_t destinationcoreId;
					if(sramId == 2){
						dx = 0;
						sx = 0;
						dy = 1;//1;
						sy = 0;
					}else if(sramId == 1){
						dx = 1;//1;
						sx = 1;//1;
						dy = 0;
						sy = 0;
					}
					if(coreId == 0){
						destinationcoreId = 1;
					}else if(coreId == 1){
						destinationcoreId = 2;
					}else if(coreId == 2){
						destinationcoreId = 4;
					}else if(coreId == 3){
						destinationcoreId = 8;
					}
					int cc = coreId;
					bits[numConfig] = neuronId << 7 | sramId << 5 | cc << 15 | 1 << 17 | 1 << 4
							| destinationcoreId << 18 | sy << 27 | dy << 25 |  dx << 22 | sx << 24 |
							cc << 28;
				}
			}
			// send data with libusb host transfer in packet
			if(!caerDynapseSendDataToUSB(usb_handle, bits, numConfig)){
				caerLog(CAER_LOG_ERROR, "DvsToDynapse", "USB transfer failed while programming SRAM for chip DYNAPSE_U3");
			}
		}
		caerLog(CAER_LOG_NOTICE, "DvsToDynapse", "Mapping convolutions done");
		sshsNodePutBool(moduleD->moduleNode, "programConvolutionMapping", false);
		stateMod->programConvolutionMapping = sshsNodeGetBool(moduleD->moduleNode, "programConvolutionMapping");
	}
}

/* program destination cam addresses 64 - synapses per input -*/
void programMapInCam(caerInputDynapseState state, DvsToDynapseState stateMod) {
	if (state == NULL) {
		return;
	}
	caerDeviceHandle usb_handle = (caerDeviceHandle) state->deviceState;
	caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, (uint32_t) stateMod->chipId); //U0->0,U1->8,U2->4,U3->12
	uint32_t neuronId;
	int bits[DYNAPSE_MAX_USER_USB_PACKET_SIZE];
	int numConfig = -1;
	// now fill it with content
	caerLog(CAER_LOG_NOTICE, "DvsToDynapse", "Programming CAM content...");
	caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID,  (uint32_t) stateMod->chipId);
	/*for (neuronId = 0;
			neuronId < DYNAPSE_CONFIG_XCHIPSIZE * DYNAPSE_CONFIG_YCHIPSIZE;
			neuronId++) {*/
	for(size_t coreId=0; coreId < 4; coreId++){
		for(size_t neuronId=0; neuronId < 256; neuronId++){
			numConfig = -1;
			for(size_t camId=0; camId < 64; camId++){
				numConfig++;
				uint32_t ei = 1;
				uint32_t fs = 1;
				uint32_t address = neuronId;
				uint32_t source_core = coreId;
				uint32_t neuron_row = (neuronId & 0xf0) >> 4;
				uint32_t synapse_row = camId;
				uint32_t row = neuron_row << 6 | synapse_row;
				uint32_t column = neuronId & 0xf;
				bits[numConfig]= ei << 29 | fs << 28 | address << 20 | source_core << 18 | 1 << 17
					| coreId << 15 | row << 5 | column;
				if(coreId == 3){
					//caerLog(CAER_LOG_ERROR, "DvsToDynapse", "bits[numConfig] %d\n", bits[numConfig]);
				}
			}
			// send data with libusb host transfer in packet
			if(!caerDynapseSendDataToUSB(usb_handle, bits, numConfig)){
				caerLog(CAER_LOG_ERROR, "DvsToDynapse", "USB transfer failed");
			}
		}
	}
	//}
	caerLog(CAER_LOG_NOTICE, "DvsToDynapse", "CAM programmed successfully.");

	stateMod->programCam = false;
}

void programBiasesDvsToDynapse(caerInputDynapseState state, DvsToDynapseState stateMod) {
	if (state == NULL) {
		return;
	}
	caerDeviceHandle usb_handle = (caerDeviceHandle) state->deviceState;
	caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, (uint32_t) stateMod->chipId); //U0->0,U1->8,U2->4,U3->12
	// set biases for all cores
	for (uint32_t coreId = 0; coreId < 4; coreId++) {
		caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_AHTAU_N", 7, 35, "LowBias","NBias");
		caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_AHTHR_N", 7, 1, "HighBias","NBias");
		caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_AHW_P", 7, 1, "HighBias","PBias");
		caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_BUF_P", 3, 80, "HighBias","PBias");
		caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_CASC_N", 7, 1, "HighBias","NBias");
		caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_DC_P", 5, 2, "HighBias","PBias");
		caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_NMDA_N", 7, 1, "HighBias","NBias");
		caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_RFR_N", 2, 180, "HighBias","NBias"); // 4
		caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_TAU1_N", 3, 40, "LowBias","NBias");
		caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_TAU2_N", 6, 15, "LowBias","NBias");
		caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_THR_N", 2, 180, "HighBias","NBias");
		caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPIE_TAU_F_P", 6, 150,"HighBias", "PBias");
		caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPIE_TAU_S_P", 7, 40,"HighBias", "NBias");
		caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPIE_THR_F_P", 4, 220,"HighBias", "PBias");
		caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPIE_THR_S_P", 7, 0,"HighBias", "PBias");
		caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPII_TAU_F_P", 6, 150,"HighBias", "NBias");
		caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPII_TAU_S_P", 7, 40,"HighBias", "NBias");
		caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPII_THR_F_P", 0, 220,"HighBias", "PBias");
		caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPII_THR_S_P", 7, 0,"HighBias", "PBias");
		caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "PS_WEIGHT_EXC_F_N", 0, 70,"HighBias", "NBias");
		caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "PS_WEIGHT_EXC_S_N", 1, 250,"HighBias", "NBias");
		caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "PS_WEIGHT_INH_F_N", 0, 250,"HighBias", "NBias");
		caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "PS_WEIGHT_INH_S_N", 1, 250,"HighBias", "NBias");
		caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "PULSE_PWLK_P", 3, 50,"HighBias", "PBias");
		caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "R2R_P", 4, 85, "HighBias","PBias");
	}
	stateMod->programBiases = false;
}

