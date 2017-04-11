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
	int chipId;						//U0->0,U1->8,U2->4,U3->12
	simple2DBufferLong DownsampledMap;
	int threshold;							//after which we send spikes out
	bool init;
	bool CNNMode;
	float numStdDevsForBoundingBox;
	int F;	//filter size	default 8x8
	int W;	//input size	default 32x32
	int P;	//zero-padding, only zero is supported
	int S;	//stride		default 8
	int R;	//repeat same filter - due to mismatch one filter can be applied several times -
	int numfilters; // number of filters - only thing that results from the others -
	int outputSize1; // number of output neurons at the output of the first layer
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

static struct caer_module_functions caerDvsToDynapseFunctions = { .moduleInit = &caerDvsToDynapseInit, .moduleRun =
	&caerDvsToDynapseRun, .moduleConfig = &caerDvsToDynapseConfig, .moduleExit = &caerDvsToDynapseExit, .moduleReset =
	&caerDvsToDynapseReset };

void caerDvsToDynapse(uint16_t moduleID, caerSpikeEventPacket spike, caerPolarityEventPacket polarity, caerPoint4DEventPacket medianData) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "DvsToDynapse", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerDvsToDynapseFunctions, moduleData, sizeof(struct DvsToDynapse_state), 3, spike, polarity, medianData);
}

static bool caerDvsToDynapseInit(caerModuleData moduleData) {

	sshsNodePutIntIfAbsent(moduleData->moduleNode, "threshold", 5);
	DvsToDynapseState state = moduleData->moduleState;

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	if (!sshsNodeAttributeExists(sourceInfoNode, "dataSizeX", SSHS_SHORT)) { //to do for visualizer change name of field to a more generic one
		sshsNodePutShort(sourceInfoNode, "dataSizeX", DYNAPSE_X4BOARD_NEUX);
		sshsNodePutShort(sourceInfoNode, "dataSizeY", DYNAPSE_X4BOARD_NEUY);
	}

	state->chipId = DYNAPSE_CONFIG_DYNAPSE_U3; //(uint16_t) sshsNodeGetInt(moduleData->moduleNode, "chipId");
	state->threshold = sshsNodeGetInt(moduleData->moduleNode, "threshold");
	state->init = true; //sshsNodeGetBool(moduleData->moduleNode, "programCamAllChips"); // false at startup
	state->DownsampledMap = NULL;

	state->CNNMode = true;
	state->F = 8;	//filter size	default 8x8
	state->W = 32;	//input size	default 32x32
	state->P = 0;	//zero-padding, only zero is supported
	state->S = 8;	//stride		default 8
	state->R = 8;	//repeat same filter - due to mismatch one filter can be applied several times -
	state->numStdDevsForBoundingBox = 1.4;

	// size of first output Layer  - for cnn mode -
	state->outputSize1 = (int) ((state->W - state->F + (2 * state->P)) / state->S) + 1;
	// calculate number of kernels/filters
	state->numfilters = (int) DYNAPSE_CONFIG_NUMNEURONS / (state->outputSize1 * state->outputSize1);
	state->numfilters = state->numfilters / state->R;

	// Nothing that can fail here.
	return (true);
}

static void caerDvsToDynapseRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerSpikeEventPacket spike = va_arg(args, caerSpikeEventPacket);	// from dynapse
	caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket); // from dvs
	caerPoint4DEventPacket medianData = va_arg(args, caerPoint4DEventPacket); // from dvs

	DvsToDynapseState state = moduleData->moduleState;
	// --- start  usb handle / from spike event source id
	int eventSourceID = 1;//caerEventPacketHeaderGetEventSource(&spike->packetHeader);
	state->eventSourceModuleState = (caerInputDynapseState) caerMainloopGetSourceState(U16T(eventSourceID));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(eventSourceID));
	if (state->eventSourceModuleState == NULL || state->eventSourceConfigNode == NULL) {
		return;
	}
	caerInputDynapseState stateSource = state->eventSourceModuleState;
	if (stateSource->deviceState == NULL) {
		return;
	}
	// --- end usb handle

	// program as destination all chips in board
	if (state->init) {

		int allChips[1] = { DYNAPSE_CONFIG_DYNAPSE_U3 };
		for (size_t this_chip = 0; this_chip < 1; this_chip++) {
			state->chipId = allChips[this_chip];
			sshsNodePutInt(moduleData->moduleNode, "chipId", allChips[this_chip]);
			caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Programming chip id %d,", state->chipId);
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID,
				(uint32_t) state->chipId);
			programMapInCam(stateSource, state);
//			programBiasesDvsToDynapse(stateSource, state);
		}
		// init done
		state->init = false;
//		programConvolutionMappingSram(stateSource, state, moduleData);

	}

	// Only process packets with content.
	// will probably need to add an always active neuron , by setting TAU2.
	if (polarity == NULL) {
		return;
	}

	sshsNode sourceInfoNode = caerMainloopGetSourceInfo(
		(uint16_t) caerEventPacketHeaderGetEventSource(&polarity->packetHeader));
	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "dvsSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "dvsSizeY");

	// update filter parameters
	caerDvsToDynapseConfig(moduleData);

	// If the map is not allocated yet, do it.
	if (state->DownsampledMap == NULL) {
		if (!allocateDownsampledMap(state, caerEventPacketHeaderGetEventSource(&polarity->packetHeader))) {
			// Failed to allocate memory, nothing to do.
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for DownsampledMap.");
			return;
		}
	}

	// reset previous map downsampled
	for (size_t i = 0; i < DYNAPSE_X4BOARD_NEUX; i++) {
		for (size_t j = 0; j < DYNAPSE_X4BOARD_NEUY; j++) {
			state->DownsampledMap->buffer2d[i][j] = 0;
		}
	}

	// use median tracker information
	float xmedian=0;
	float ymedian=0;
	float xmedianstd=0;
	float ymedianstd=0;

	if(medianData != NULL){

		CAER_POINT4D_ITERATOR_ALL_START(medianData)
			xmedian = caerPoint4DEventGetX(caerPoint4DIteratorElement);
			ymedian = caerPoint4DEventGetY(caerPoint4DIteratorElement);
			xmedianstd = caerPoint4DEventGetZ(caerPoint4DIteratorElement);
			ymedianstd = caerPoint4DEventGetW(caerPoint4DIteratorElement);
		CAER_POINT4D_ITERATOR_ALL_END

		if(xmedianstd > 0 && ymedianstd > 0){
			sizeX = (int) floor((xmedian + xmedianstd * state->numStdDevsForBoundingBox) - (xmedian - xmedianstd * state->numStdDevsForBoundingBox));
			sizeY = (int) floor((ymedian + ymedianstd * state->numStdDevsForBoundingBox) - (ymedian - ymedianstd * state->numStdDevsForBoundingBox));
		}

		//caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "median x,y %f %f - std x,y, %f, %f - sizeX %d, sizeY %d",
		//	xmedian, ymedian, xmedianstd, ymedianstd, sizeX, sizeY);

		CAER_POLARITY_ITERATOR_VALID_START(polarity)
				int xx = caerPolarityEventGetX(caerPolarityIteratorElement);
				int yy = caerPolarityEventGetY(caerPolarityIteratorElement);

				if (		 (xx <= (int) (xmedian + xmedianstd * state->numStdDevsForBoundingBox))
								&& (xx > (int) (xmedian - xmedianstd * state->numStdDevsForBoundingBox))
								&& (yy < (int) (ymedian + ymedianstd * state->numStdDevsForBoundingBox))
								&& (yy > (int) (ymedian - ymedianstd * state->numStdDevsForBoundingBox))){
					;
				}else{
					caerPolarityEventInvalidate(caerPolarityIteratorElement, polarity);
				}
		CAER_POLARITY_ITERATOR_VALID_END

	}


	CAER_POLARITY_ITERATOR_VALID_START(polarity)
		int pol_x = caerPolarityEventGetX(caerPolarityIteratorElement);
		int pol_y = caerPolarityEventGetY(caerPolarityIteratorElement);
		bool pol_pol = caerPolarityEventGetPolarity(caerPolarityIteratorElement);

		if(medianData != NULL){
			pol_x = pol_x - (int) floor(xmedian - xmedianstd * state->numStdDevsForBoundingBox);
			pol_y = pol_y - (int) floor(ymedian - ymedianstd * state->numStdDevsForBoundingBox);
			if(pol_x < 0){
				pol_x = 0;
			}
			if(pol_y < 0){
				pol_y = 0;
			}
			//caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString,
			//	"pol_x,y %d, %d - xmedian %f ymedian %f - stdx %f, stdy %f - sizeX %d, sizeY %d",
			//	pol_x, pol_y, xmedian , ymedian, xmedianstd, ymedianstd, sizeX, sizeY);

			caerPolarityEventSetX(caerPolarityIteratorElement, pol_x);
			caerPolarityEventSetY(caerPolarityIteratorElement, pol_y);
		}

		// generate coordinates in  chip_size x chip_size
		int new_x, new_y;
		new_x = (int) round(((double) pol_x / sizeX) * DYNAPSE_CONFIG_XCHIPSIZE);
		new_y = (int) round(((double) pol_y / sizeY) * DYNAPSE_CONFIG_YCHIPSIZE);
		//update DownsampledMap
		if (new_x >= DYNAPSE_CONFIG_XCHIPSIZE) {
			new_x = DYNAPSE_CONFIG_XCHIPSIZE - 1;
		}
		if (new_y >= DYNAPSE_CONFIG_YCHIPSIZE) {
			new_y = DYNAPSE_CONFIG_YCHIPSIZE - 1;
		}

		// only positives spikes, usb bandwidth reduction
		state->DownsampledMap->buffer2d[new_x][new_y] += 1;
//		if (pol_pol == 1) {
//			state->DownsampledMap->buffer2d[new_x][new_y] += 1;
//		}
//		else {
//			state->DownsampledMap->buffer2d[new_x][new_y] -= 1;
//		}
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

	//loop
	for (int i = 0; i < DYNAPSE_CONFIG_XCHIPSIZE; i++) {
		for (int j = 0; j < DYNAPSE_CONFIG_YCHIPSIZE; j++) {
			int nx = i;
			int ny = j;
			//only let pass above the state->threshold
			if (state->DownsampledMap->buffer2d[i][j] > state->threshold) {
				// generate bits to send
				// core selection
				int core_dest = 0;
				if (i < sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE) && j < sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE)) {
					core_dest = 0;
				}
				else if (i >= sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE) && j < sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE)) {
					core_dest = 2;
				}
				else if (i >= sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE) && j >= sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE)) {
					core_dest = 3;
				}
				else if (i < sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE) && j >= sqrt(DYNAPSE_CONFIG_NUMNEURONS_CORE)) {
					core_dest = 1;
				}
				// adjusts coordinates
				if (core_dest == 1 || core_dest == 3) {
					ny = ny - 16;
				}
				if (core_dest == 2 || core_dest == 3) {
					nx = nx - 16;
				}
				int core_s = core_dest;
				int address = (int) nx * 16 + (int) ny;

				uint32_t value = (uint32_t) 15 | 0 << 16 | 0 << 17 | 1 << 13 | (uint) core_s << 18
					| (uint) address << 20 | 0 << 4 | 0 << 6 | 0 << 7 | 0 << 9;

				if (numConfig + 1 >= DYNAPSE_MAX_USER_USB_PACKET_SIZE) {
					caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Breaking transaction\n");
					// we got data
					// select destination chip
					if (state->chipId == DYNAPSE_CONFIG_DYNAPSE_U0 || state->chipId == DYNAPSE_CONFIG_DYNAPSE_U1
						|| state->chipId == DYNAPSE_CONFIG_DYNAPSE_U2 || state->chipId == DYNAPSE_CONFIG_DYNAPSE_U3) {
						caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID,
							(uint32_t) state->chipId);
					}
					else {
						caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
							"Chip Id selected is non valid, please select one of 0,4,8,12\n");
					}
					// send data with libusb host transfer in packet
					if (!caerDynapseSendDataToUSB(stateSource->deviceState, bits, (int) numConfig)) {
						caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "USB transfer failed");
					}
					numConfig = 0;
					dataReady = false;
				}
				else {
					bits[numConfig] = (int) value;
					numConfig++;
					dataReady = true;
				}

			}
		}
	}
	// we got data
	if (dataReady) {
		// select destination chip
		if (state->chipId == DYNAPSE_CONFIG_DYNAPSE_U0 || state->chipId == DYNAPSE_CONFIG_DYNAPSE_U1
			|| state->chipId == DYNAPSE_CONFIG_DYNAPSE_U2 || state->chipId == DYNAPSE_CONFIG_DYNAPSE_U3) {
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID,
					DYNAPSE_CONFIG_DYNAPSE_U3); //(uint32_t) state->chipId);
		}
		else {
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
				"Chip Id selected is non valid, please select one of 0,4,8,12\n");
		}
		// send data with libusb host transfer in packet
		if (!caerDynapseSendDataToUSB(stateSource->deviceState, bits, numConfig)) {
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "USB transfer failed");
		}
	}

}

static void caerDvsToDynapseConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	DvsToDynapseState state = moduleData->moduleState;
	state->threshold = sshsNodeGetInt(moduleData->moduleNode, "threshold");
	//state->programConvolutionMapping = sshsNodeGetBool(moduleData->moduleNode, "programConvolutionMapping");
	//state->CNNMode = sshsNodeGetBool(moduleData->moduleNode, "CNNMode");

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

	for (size_t x = 0; x < sizeX; x++) {
		for (size_t y = 0; y < sizeY; y++) {
			state->DownsampledMap->buffer2d[x][y] = 0; // init to zero
		}
	}

	return (true);
}

/* program SRAM cam - to map to convolutions -*/
void programConvolutionMappingSram(caerInputDynapseState state, DvsToDynapseState stateMod, caerModuleData moduleD) {

	if (stateMod->chipId != DYNAPSE_CONFIG_DYNAPSE_U3) {
		caerLog(CAER_LOG_ERROR, "DvsToDynapse", "Mapping convolution is not possible with chipId different than 12!!");
	}
	else {
		caerDeviceHandle usb_handle = (caerDeviceHandle) state->deviceState;
		caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U3);
		// proceed with the mapping
		// init sram_id = 1;
		int bits[DYNAPSE_MAX_USER_USB_PACKET_SIZE];
		int numConfig = -1;
		caerLog(CAER_LOG_NOTICE, "DvsToDynapse", "Mapping convolutions started..");
		if (stateMod->CNNMode) {
			for (int sramId = 1; sramId < 3; sramId++) {
				numConfig = -1;
				for (size_t neuronId = 0; neuronId < DYNAPSE_CONFIG_NUMNEURONS_CORE; neuronId++) {
					//for (size_t filterId = 0; filterId < stateMod->numfilters ; filterId++) {
					for (int coreId = 0; coreId < DYNAPSE_CONFIG_NUMCORES; ++coreId) {
						uint8_t dx = 0;
						uint8_t sx = 0;
						uint8_t dy = 0;
						uint8_t sy = 0;
						numConfig++;
						int destinationcoreId = 15; // all cores
						// use SRAM for projections to chips U1 and U2
						if (sramId == 1) {
							dx = 0;
							sx = 0;
							dy = 1;
							sy = 0;				// NORTH ONE
						}
						else if (sramId == 2) {
							dx = 1;		//1;
							sx = 1;		//1;
							dy = 0;
							sy = 0;				// WEST ONE
						}
						bits[numConfig] = neuronId << 7 | sramId << 5 | coreId << 15 | 1 << 17 | 1 << 4
							| destinationcoreId << 18 | sy << 27 | dy << 25 | dx << 22 | sx << 24 | coreId << 28;
					}
				}
				// send data with libusb host transfer in packet
				if (!caerDynapseSendDataToUSB(usb_handle, bits, numConfig)) {
					caerLog(CAER_LOG_ERROR, "DvsToDynapse",
						"USB transfer failed while programming SRAM for chip DYNAPSE_U3");
				}
			}
		}
		else {
			for (int sramId = 1; sramId < 3; sramId++) {
				numConfig = -1;
				for (int neuronId = 0; neuronId < DYNAPSE_CONFIG_NUMNEURONS_CORE; neuronId++) {
					for (int coreId = 0; coreId < DYNAPSE_CONFIG_NUMCORES; ++coreId) {
						numConfig++;
						uint8_t dx = 0;
						uint8_t sx = 0;
						uint8_t dy = 0;
						uint8_t sy = 0;
						uint16_t destinationcoreId;
						if (sramId == 2) {
							dx = 0;
							sx = 0;
							dy = 1;		//1;
							sy = 0;
						}
						else if (sramId == 1) {
							dx = 1;		//1;
							sx = 1;		//1;
							dy = 0;
							sy = 0;
						}
						if (coreId == 0) {
							destinationcoreId = 1;
						}
						else if (coreId == 1) {
							destinationcoreId = 2;
						}
						else if (coreId == 2) {
							destinationcoreId = 4;
						}
						else if (coreId == 3) {
							destinationcoreId = 8;
						}
						int cc = coreId;
						bits[numConfig] = neuronId << 7 | sramId << 5 | cc << 15 | 1 << 17 | 1 << 4
							| destinationcoreId << 18 | sy << 27 | dy << 25 | dx << 22 | sx << 24 | cc << 28;
					}
				}
				// send data with libusb host transfer in packet
				if (!caerDynapseSendDataToUSB(usb_handle, bits, numConfig)) {
					caerLog(CAER_LOG_ERROR, "DvsToDynapse",
						"USB transfer failed while programming SRAM for chip DYNAPSE_U3");
				}
			}
		}
		caerLog(CAER_LOG_NOTICE, "DvsToDynapse", "Mapping convolutions done");
	}

}

/* program destination cam addresses 64 - synapses per input -*/
void programMapInCam(caerInputDynapseState state, DvsToDynapseState stateMod) {
	if (state == NULL) {
		return;
	}
	caerDeviceHandle usb_handle = (caerDeviceHandle) state->deviceState;
	uint32_t neuronId;
	int bits[DYNAPSE_MAX_USER_USB_PACKET_SIZE];
	int numConfig;
	caerLog(CAER_LOG_NOTICE, "DvsToDynapse", "Programming CAM content...");
	caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, (uint32_t) stateMod->chipId);
	if (stateMod->CNNMode) {
		if (stateMod->chipId == DYNAPSE_CONFIG_DYNAPSE_U1 || stateMod->chipId == DYNAPSE_CONFIG_DYNAPSE_U2) {
			//compute Gabor filters
			double x, y;
			double filters_sin[8][8][16];
			double filters_cos[8][8][16];
			//int w = 3; 				// wave number
			int h = 8; 					// kernel size
			double q = M_PI / 2.0;		// filters orientation
			for (int w = 0; w < 16; w++) {
				for (int i = 0; i < h; i++) {
					x = i - 0.5 * (h - 1);
					for (int j = 0; j < h; j++) {
						y = j - 0.5 * (h - 1);
						filters_sin[i][j][w] = exp((-16 / (h * h)) * (x * x + y * y))
							* cos((2 * M_PI * w / h) * (x * cos(q) + y * sin(q))) / (h * h);
						filters_cos[i][j][w] = exp((-16 / (h * h)) * (x * x + y * y))
							* sin((2 * M_PI * w / h) * (x * cos(q) + y * sin(q))) / (h * h);
						//printf(" %f ", filters_sin[i][j][w]);
					}
					//printf("\n");
				}
				//printf("\n\n");
			}
			int mapid = 0;
			for (int targetneu = 0; targetneu < 256; targetneu++) {
				numConfig = -1;
				for (int filterpixx = 0; filterpixx < 8; filterpixx++) {
					for (int filterpixy = 0; filterpixy < 8; filterpixy++) {
						int mapxcoor = mapid % 4;
						int mapycoor = mapid / 4;

						uint32_t ei = 1;
						uint32_t fs = 1;
						// here we use Gabor's patches
						if (stateMod->chipId == DYNAPSE_CONFIG_DYNAPSE_U1) {
							if (filters_cos[filterpixx][filterpixy][mapid] > 0) {
								ei = 1;
								fs = 1;
							}
							else {
								ei = 0;
								fs = 0;
							}
						}
						else {
							if (filters_sin[filterpixx][filterpixy][mapid] > 0) {
								ei = 1;
								fs = 1;
							}
							else {
								ei = 0;
								fs = 0;
							}
						}

						int sourceneuron_inmap;
						sourceneuron_inmap = (16 * filterpixx) + (filterpixy) + ((mapycoor % 2) * 8)
							+ ((mapxcoor % 2) * 16 * 8);
						int source_core;
						if (mapxcoor < 2 && mapycoor < 2) {
							source_core = 0;
						}
						else if (mapxcoor < 2 && mapycoor >= 2) {
							source_core = 1;
						}
						else if (mapxcoor >= 2 && mapycoor >= 2) {
							source_core = 3;
						}
						else if (mapxcoor >= 2 && mapycoor < 2) {
							source_core = 2;
						}
						uint32_t synapse_row = filterpixx * 8 + filterpixy; 		// 0 - 63 cam ID
						uint32_t neuron_row = (targetneu & 0xf0) >> 4;				// 0 - 255
						uint32_t row = neuron_row << 6 | synapse_row; // neuron that will receive from sourceneuron_inmap
						uint32_t column = targetneu & 0xf;							//

						numConfig++;
						bits[numConfig] = ei << 29 | fs << 28 | sourceneuron_inmap << 20 | source_core << 18 | 1 << 17
							| source_core << 15 | row << 5 | column | 0 << 4;

					}
				}
				// send data with libusb host transfer in packet
				if(numConfig > 0){
					if (!caerDynapseSendDataToUSB(usb_handle, bits, numConfig)) {
						caerLog(CAER_LOG_ERROR, "DvsToDynapse", "USB transfer failed");
					}
				}
				mapid++;
				if (mapid == 16) {
					mapid = 0;
				}
			}
		}
		if (stateMod->chipId == DYNAPSE_CONFIG_DYNAPSE_U3) {
			// subsampling map
			for (size_t coreId = 0; coreId < 4; coreId++) {
				for (size_t neuronId = 0; neuronId < 256; neuronId++) {
					numConfig = -1;
					for (size_t camId = 0; camId < 64; camId++) {
						numConfig++;
						uint32_t ei = 1;
						uint32_t fs = 1;
						uint32_t address = neuronId;
						uint32_t source_core = coreId;
						uint32_t neuron_row = (neuronId & 0xf0) >> 4;
						uint32_t synapse_row = camId;
						uint32_t row = neuron_row << 6 | synapse_row;	// 0 - 1023
						uint32_t column = neuronId & 0xf;				// 0 - 15
						bits[numConfig] = ei << 29 | fs << 28 | address << 20 | source_core << 18 | 1 << 17
							| coreId << 15 | row << 5 | column;
					}
					// send data with libusb host transfer in packet
					if (!caerDynapseSendDataToUSB(usb_handle, bits, numConfig)) {
						caerLog(CAER_LOG_ERROR, "DvsToDynapse", "USB transfer failed");
					}
				}
			}
		}
	}
	else {
		// debug map
		for (size_t coreId = 0; coreId < 4; coreId++) {
			for (size_t neuronId = 0; neuronId < 256; neuronId++) {
				numConfig = -1;
				for (size_t camId = 0; camId < 64; camId++) {
					numConfig++;
					uint32_t ei = 1;
					uint32_t fs = 1;
					uint32_t address = neuronId;
					uint32_t source_core = coreId;
					uint32_t neuron_row = (neuronId & 0xf0) >> 4;
					uint32_t synapse_row = camId;
					uint32_t row = neuron_row << 6 | synapse_row;	// 0 - 1023
					uint32_t column = neuronId & 0xf;	// 0 - 15
					//caerLog(CAER_LOG_NOTICE, "DvsToDynapse", "row %d col %d", row, column);
					bits[numConfig] = ei << 29 | fs << 28 | address << 20 | source_core << 18 | 1 << 17 | coreId << 15
						| row << 5 | column;
				}
				// send data with libusb host transfer in packet
				if (!caerDynapseSendDataToUSB(usb_handle, bits, numConfig)) {
					caerLog(CAER_LOG_ERROR, "DvsToDynapse", "USB transfer failed");
				}
			}
		}
	}

	caerLog(CAER_LOG_NOTICE, "DvsToDynapse", "CAM programmed successfully.");

}

void programBiasesDvsToDynapse(caerInputDynapseState state, DvsToDynapseState stateMod) {
	if (state == NULL) {
		return;
	}
	caerDeviceHandle usb_handle = (caerDeviceHandle) state->deviceState;
	caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, (uint32_t) stateMod->chipId); //U0->0,U1->8,U2->4,U3->12
// set biases for all cores
	if (stateMod->CNNMode) {
		if (stateMod->chipId == DYNAPSE_CONFIG_DYNAPSE_U1) {
			for (uint32_t coreId = 0; coreId < 4; coreId++) {
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_AHTAU_N", 2, 35, "LowBias", "NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_AHTHR_N", 4, 2, "HighBias", "NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_AHW_P", 3, 4, "HighBias", "PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_BUF_P", 1, 80, "HighBias", "PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_CASC_N", 0, 25, "HighBias", "NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_DC_P", 7, 60, "HighBias", "PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_NMDA_N", 3, 1, "HighBias", "NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_RFR_N", 3, 3, "HighBias", "NBias"); // 4
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_TAU1_N", 3, 18, "LowBias", "NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_TAU2_N", 4, 100, "LowBias", "NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_THR_N", 2, 166, "HighBias", "NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPIE_TAU_F_P", 5, 40, "HighBias",
					"PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPIE_TAU_S_P", 7, 1, "HighBias",
					"NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPIE_THR_F_P", 1, 80, "HighBias",
					"PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPIE_THR_S_P", 7, 150, "HighBias",
					"PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPII_TAU_F_P", 4, 150, "HighBias",
					"NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPII_TAU_S_P", 7, 40, "HighBias",
					"NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPII_THR_F_P", 3, 1, "HighBias",
					"PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPII_THR_S_P", 6, 40, "HighBias",
					"PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "PS_WEIGHT_EXC_F_N", 1, 150, "HighBias",
					"NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "PS_WEIGHT_EXC_S_N", 7, 1, "HighBias",
					"NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "PS_WEIGHT_INH_F_N", 7, 1, "HighBias",
					"NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "PS_WEIGHT_INH_S_N", 7, 1, "HighBias",
					"NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "PULSE_PWLK_P", 3, 106, "HighBias",
					"PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "R2R_P", 4, 85, "HighBias", "PBias");
			}
		}
		else if (stateMod->chipId == DYNAPSE_CONFIG_DYNAPSE_U2) {
			for (uint32_t coreId = 0; coreId < 4; coreId++) {
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_AHTAU_N", 2, 35, "LowBias", "NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_AHTHR_N", 4, 2, "HighBias", "NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_AHW_P", 3, 4, "HighBias", "PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_BUF_P", 1, 80, "HighBias", "PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_CASC_N", 0, 25, "HighBias", "NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_DC_P", 7, 60, "HighBias", "PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_NMDA_N", 3, 1, "HighBias", "NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_RFR_N", 3, 3, "HighBias", "NBias"); // 4
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_TAU1_N", 3, 18, "LowBias", "NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_TAU2_N", 4, 100, "LowBias", "NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_THR_N", 2, 166, "HighBias", "NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPIE_TAU_F_P", 5, 40, "HighBias",
					"PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPIE_TAU_S_P", 7, 1, "HighBias",
					"NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPIE_THR_F_P", 1, 80, "HighBias",
					"PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPIE_THR_S_P", 7, 150, "HighBias",
					"PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPII_TAU_F_P", 4, 150, "HighBias",
					"NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPII_TAU_S_P", 7, 40, "HighBias",
					"NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPII_THR_F_P", 3, 1, "HighBias",
					"PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPII_THR_S_P", 6, 40, "HighBias",
					"PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "PS_WEIGHT_EXC_F_N", 1, 150, "HighBias",
					"NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "PS_WEIGHT_EXC_S_N", 7, 1, "HighBias",
					"NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "PS_WEIGHT_INH_F_N", 7, 1, "HighBias",
					"NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "PS_WEIGHT_INH_S_N", 7, 1, "HighBias",
					"NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "PULSE_PWLK_P", 3, 106, "HighBias",
					"PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "R2R_P", 4, 85, "HighBias", "PBias");
			}
		}
		else if (stateMod->chipId == DYNAPSE_CONFIG_DYNAPSE_U3) {
			for (uint32_t coreId = 0; coreId < 4; coreId++) {
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_AHTHR_N", 7, 1, "HighBias", "NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_AHW_P", 7, 1, "HighBias", "PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_BUF_P", 3, 80, "HighBias", "PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_CASC_N", 7, 1, "HighBias", "NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_DC_P", 7, 2, "HighBias", "PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_NMDA_N", 7, 1, "HighBias", "NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_RFR_N", 3, 180, "HighBias", "NBias"); // 4
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_TAU1_N", 3, 10, "LowBias", "NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_TAU2_N", 6, 15, "LowBias", "NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_THR_N", 2, 180, "HighBias", "NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPIE_TAU_F_P", 6, 150, "HighBias",
					"PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPIE_TAU_S_P", 7, 40, "HighBias",
					"NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPIE_THR_F_P", 4, 220, "HighBias",
					"PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPIE_THR_S_P", 7, 0, "HighBias",
					"PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPII_TAU_F_P", 6, 150, "HighBias",
					"NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPII_TAU_S_P", 7, 40, "HighBias",
					"NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPII_THR_F_P", 0, 220, "HighBias",
					"PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPII_THR_S_P", 7, 0, "HighBias",
					"PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "PS_WEIGHT_EXC_F_N", 0, 255, "HighBias",
					"NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "PS_WEIGHT_EXC_S_N", 1, 250, "HighBias",
					"NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "PS_WEIGHT_INH_F_N", 0, 250, "HighBias",
					"NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "PS_WEIGHT_INH_S_N", 1, 250, "HighBias",
					"NBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "PULSE_PWLK_P", 3, 50, "HighBias",
					"PBias");
				caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "R2R_P", 4, 85, "HighBias", "PBias");
			}
		}
	}
	else {
		for (uint32_t coreId = 0; coreId < 4; coreId++) {
			caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
			caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_AHTHR_N", 7, 1, "HighBias", "NBias");
			caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_AHW_P", 7, 1, "HighBias", "PBias");
			caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_BUF_P", 3, 80, "HighBias", "PBias");
			caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_CASC_N", 7, 1, "HighBias", "NBias");
			caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_DC_P", 5, 2, "HighBias", "PBias");
			caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_NMDA_N", 7, 1, "HighBias", "NBias");
			caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_RFR_N", 2, 180, "HighBias", "NBias"); // 4
			caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_TAU1_N", 3, 40, "LowBias", "NBias");
			caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_TAU2_N", 6, 15, "LowBias", "NBias");
			caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "IF_THR_N", 2, 180, "HighBias", "NBias");
			caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPIE_TAU_F_P", 6, 150, "HighBias",
				"PBias");
			caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPIE_TAU_S_P", 7, 40, "HighBias",
				"NBias");
			caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPIE_THR_F_P", 4, 220, "HighBias",
				"PBias");
			caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPIE_THR_S_P", 7, 0, "HighBias", "PBias");
			caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPII_TAU_F_P", 6, 150, "HighBias",
				"NBias");
			caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPII_TAU_S_P", 7, 40, "HighBias",
				"NBias");
			caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPII_THR_F_P", 0, 220, "HighBias",
				"PBias");
			caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "NPDPII_THR_S_P", 7, 0, "HighBias", "PBias");
			caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "PS_WEIGHT_EXC_F_N", 0, 70, "HighBias",
				"NBias");
			caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "PS_WEIGHT_EXC_S_N", 1, 250, "HighBias",
				"NBias");
			caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "PS_WEIGHT_INH_F_N", 0, 250, "HighBias",
				"NBias");
			caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "PS_WEIGHT_INH_S_N", 1, 250, "HighBias",
				"NBias");
			caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
			caerDynapseSetBias(state, (uint32_t) stateMod->chipId, coreId, "R2R_P", 4, 85, "HighBias", "PBias");
		}
	}
//stateMod->programBiases = false;
}

