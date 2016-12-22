/*
 * Created on: Dec, 2016
 * Author: dongchen@ini.uzh.ch
 */

#include "learningfilter.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/buffers.h"
#include "libcaer/devices/dynapse.h"
#include <math.h>

#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include <signal.h>

struct LFilter_state {
	sshsNode eventSourceModuleState;
	sshsNode eventSourceConfigNode;
	int32_t colorscaleMax;
	int32_t colorscaleMin;
	int8_t reset;
	float resetExProbability;
	int8_t resetExType;
	int8_t resetInType;
	double learningRate;
	int16_t dataSizeX;
	int16_t dataSizeY;
	int16_t visualizerSizeX;
	int16_t visualizerSizeY;
	int16_t apsSizeX;
	int16_t apsSizeY;
	bool stimulate;
};

struct LFilter_memory {
	simple2DBufferInt connectionMap; //size: TOTAL_NEURON_NUM_ON_CHIP by TOTAL_NEURON_NUM_ON_CHIP
	simple2DBufferInt camMap;
	simple2DBufferInt camSize;
	simple2DBufferInt sramMap;
	simple2DBufferInt sramMapContent;
	simple2DBufferDouble weightMap;
	simple2DBufferInt synapseMap;
	simple2DBufferLong spikeQueue;
	simple2DBufferInt filterMap;
	simple2DBufferInt filterCamMap;
	simple2DBufferInt filterMapSize;
	uint64_t spikeCounter;
	uint64_t preRdPointer;
	uint64_t wrPointer;
};

double deltaWeights[DELTA_WEIGHT_LUT_LENGTH];

typedef struct {
	uint16_t r,g,b;
} COLOUR;

typedef struct LFilter_state *LFilterState;
typedef struct LFilter_memory LFilterMemory; // *LFilterMemory

static LFilterMemory memory;
static int8_t reseted = 0;
static int64_t time_count = 0;
static int64_t time_count_last = 0;
static int32_t pattern = 0;
static struct itimerval oldtv;

static bool caerLearningFilterInit(caerModuleData moduleData); //It may not run at the beginning of the experiment ????????????
static void caerLearningFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerLearningFilterConfig(caerModuleData moduleData);
static void caerLearningFilterExit(caerModuleData moduleData);
static void caerLearningFilterReset(caerModuleData moduleData, uint16_t resetCallSourceID);
static bool ModifySynapse(caerModuleData moduleData, int16_t eventSourceID, int64_t preNeuronAddr, int64_t postNeuronAddr, double deltaWeight);
static bool ResetNetwork(caerModuleData moduleData, int16_t eventSourceID);
static bool BuildSynapse(caerModuleData moduleData, int16_t eventSourceID, uint32_t neuron_addr1, uint32_t neuron_addr2, int16_t type, int8_t real_virtual_tag);
static bool WriteCam(caerModuleData moduleData, int16_t eventSourceID, uint32_t preNeuronAddr, uint32_t postNeuronAddr, uint32_t camId, int16_t synapseType);
static bool WriteSram(caerModuleData moduleData, int16_t eventSourceID, uint32_t preNeuronAddr, uint32_t postNeuronAddr, uint32_t sramId);
static void Shuffle1DArray(int64_t *array, int64_t Range);
static void GetRand1DArray(int64_t *array, int64_t Range, int64_t CamNumAvailable);
static void GetRand1DBinaryArray(int64_t *binaryArray, int64_t Range, int64_t CamNumAvailable);
static bool ResetBiases(caerModuleData moduleData, int16_t eventSourceID);
static bool EnableStimuliGen(caerModuleData moduleData, int16_t eventSourceID, int32_t pattern);
static bool DisableStimuliGen(caerModuleData moduleData, int16_t eventSourceID);
static bool ClearAllCam(caerModuleData moduleData, int16_t eventSourceID);
static void setBiasBits(caerModuleData moduleData, int16_t eventSourceID, uint32_t chipId, uint32_t coreId, const char *biasName_t,
		uint8_t coarseValue, uint16_t fineValue, const char *lowHigh, const char *npBias);
static void SetTimer(void);
static void SignalHandler(int m);

COLOUR GetColourW(double v, double vmin, double vmax);
COLOUR GetColourS(double v, double vmin, double vmax);

static struct caer_module_functions caerLearningFilterFunctions = { .moduleInit =
	&caerLearningFilterInit, .moduleRun = &caerLearningFilterRun, .moduleConfig =
	&caerLearningFilterConfig, .moduleExit = &caerLearningFilterExit, .moduleReset =
	&caerLearningFilterReset };

void caerLearningFilter(uint16_t moduleID, int16_t eventSourceID, caerSpikeEventPacket spike, caerFrameEventPacket *weightplot, caerFrameEventPacket *synapseplot) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "LFilter", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}
	caerModuleSM(&caerLearningFilterFunctions, moduleData, sizeof(struct LFilter_state), 4, eventSourceID, spike, weightplot, synapseplot);
}

static bool caerLearningFilterInit(caerModuleData moduleData) {
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "colorscaleMax", VMAX); //500
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "colorscaleMin", VMIN);
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "reset", 0);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "resetExProbability", 1);
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "resetExType", 2); //2 for test, should be 1
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "resetInType", 1);
	sshsNodePutDoubleIfAbsent(moduleData->moduleNode, "learningRate", 1);
	sshsNodePutShortIfAbsent(moduleData->moduleNode, "dataSizeX", VISUALIZER_HEIGHT); //640
	sshsNodePutShortIfAbsent(moduleData->moduleNode, "dataSizeY", VISUALIZER_WIDTH); //480
	sshsNodePutShortIfAbsent(moduleData->moduleNode, "visualizerSizeX", VISUALIZER_HEIGHT); //640
	sshsNodePutShortIfAbsent(moduleData->moduleNode, "visualizerSizeY", VISUALIZER_WIDTH); //480
	sshsNodePutShortIfAbsent(moduleData->moduleNode, "apsSizeX", VISUALIZER_HEIGHT); //640
	sshsNodePutShortIfAbsent(moduleData->moduleNode, "apsSizeY", VISUALIZER_WIDTH); //480
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "stimulate", true); //false

	LFilterState state = moduleData->moduleState;
	state->reset = sshsNodeGetByte(moduleData->moduleNode, "reset");
	state->resetExType = sshsNodeGetByte(moduleData->moduleNode, "resetExType");
	state->resetInType = sshsNodeGetByte(moduleData->moduleNode, "resetInType");
	state->dataSizeX = sshsNodeGetShort(moduleData->moduleNode, "dataSizeX");
	state->dataSizeY = sshsNodeGetShort(moduleData->moduleNode, "dataSizeY");
	state->visualizerSizeX = sshsNodeGetShort(moduleData->moduleNode, "visualizerSizeX");
	state->visualizerSizeY = sshsNodeGetShort(moduleData->moduleNode, "visualizerSizeY");
	state->apsSizeX = sshsNodeGetShort(moduleData->moduleNode, "apsSizeX");
	state->apsSizeY = sshsNodeGetShort(moduleData->moduleNode, "apsSizeY");
	state->stimulate = sshsNodeGetBool(moduleData->moduleNode, "stimulate");

	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	if (!sshsNodeAttributeExists(sourceInfoNode, "apsSizeX", SSHS_SHORT)) {
		sshsNodePutShort(sourceInfoNode, "apsSizeX", VISUALIZER_HEIGHT); //DYNAPSE_X4BOARD_NEUY
		sshsNodePutShort(sourceInfoNode, "apsSizeY", VISUALIZER_WIDTH); //DYNAPSE_X4BOARD_NEUY
	}

	return (true); // Nothing that can fail here.
}

static void caerLearningFilterConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);
	LFilterState state = moduleData->moduleState;
	state->colorscaleMax = sshsNodeGetInt(moduleData->moduleNode, "colorscaleMax");
	state->colorscaleMin = sshsNodeGetInt(moduleData->moduleNode, "colorscaleMin");
	state->reset = sshsNodeGetByte(moduleData->moduleNode, "reset");
	state->resetExProbability = sshsNodeGetFloat(moduleData->moduleNode, "resetExProbability");
	state->resetExType = sshsNodeGetByte(moduleData->moduleNode, "resetExType");
	state->resetInType = sshsNodeGetByte(moduleData->moduleNode, "resetInType");
	state->learningRate = sshsNodeGetDouble(moduleData->moduleNode, "learningRate");
	state->dataSizeX = sshsNodeGetShort(moduleData->moduleNode, "dataSizeX");
	state->dataSizeY = sshsNodeGetShort(moduleData->moduleNode, "dataSizeY");
	state->visualizerSizeX = sshsNodeGetShort(moduleData->moduleNode, "visualizerSizeX");
	state->visualizerSizeY = sshsNodeGetShort(moduleData->moduleNode, "visualizerSizeY");
	state->visualizerSizeX = sshsNodeGetShort(moduleData->moduleNode, "apsSizeX");
	state->visualizerSizeY = sshsNodeGetShort(moduleData->moduleNode, "apsSizeY");
	state->stimulate = sshsNodeGetBool(moduleData->moduleNode, "stimulate");
}

static void caerLearningFilterExit(caerModuleData moduleData) { // Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);
}

static void caerLearningFilterReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(moduleData);
	UNUSED_ARGUMENT(resetCallSourceID);
//	ResetNetwork(moduleData);
}

static void caerLearningFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	int16_t eventSourceID = va_arg(args, int); 	// Interpret variable arguments (same as above in main function).
	caerSpikeEventPacket spike = va_arg(args, caerSpikeEventPacket);
	caerFrameEventPacket *weightplot = va_arg(args, caerFrameEventPacket*);
	caerFrameEventPacket *synapseplot = va_arg(args, caerFrameEventPacket*);

	LFilterState state = moduleData->moduleState;
	uint32_t counterW, counterS;
	COLOUR colW, colS;
	uint16_t sizeX = VISUALIZER_HEIGHT;
	uint16_t sizeY = VISUALIZER_WIDTH;
	if (memory.synapseMap == NULL) {
		for (int i = 0; i < DELTA_WEIGHT_LUT_LENGTH; i++) {
			deltaWeights[i] = exp(i/1000);
		}
		if (!ResetNetwork(moduleData, eventSourceID)) { // Failed to allocate memory, nothing to do.
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for synapseMap.");
			return;
		}
		int64_t i, j, ys, row_id, col_id, feature_id;
		double warrayW[sizeX][sizeY];
		for (i = 0; i < sizeX; i++)
			for (j = 0; j < sizeY; j++)
				warrayW[i][j] = 0;
		*weightplot = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, sizeX, sizeY, 3); //put info into frame
		if (*weightplot != NULL) {
			caerFrameEvent singleplotW = caerFrameEventPacketGetEvent(*weightplot, 0);
			for (i = 0; i < INPUT_N; i++) {
				for (j = 0; j < FEATURE1_N * FEATURE1_LAYERS_N; j++) {
					if ((int)(i/INPUT_L) >= (int)((j%FEATURE1_N)/FEATURE1_L)
							&& (int)(i/INPUT_L) < (int)((j%FEATURE1_N)/FEATURE1_L) + FILTER1_L
							&& i%INPUT_W >= (j%FEATURE1_N)%FEATURE1_W
							&& i%INPUT_W < (j%FEATURE1_N)%FEATURE1_W + FILTER1_W) {
						row_id = FILTER1_L*(int)((j%FEATURE1_N)/FEATURE1_W) + (int)(i/INPUT_W) - (int)((j%FEATURE1_N)/FEATURE1_W);
						col_id = FILTER1_W*((j%FEATURE1_N)%FEATURE1_W)+i%INPUT_W-(j%FEATURE1_N)%FEATURE1_W;
						feature_id = (int)(j/FEATURE1_N);
						warrayW[(feature_id >> 1)*FILTER1_L*FEATURE1_L + row_id][(feature_id & 0x1)*FILTER1_W*FEATURE1_W + col_id]
							= memory.weightMap->buffer2d[(i & 0xf | ((i & 0x10) >> 4) << 8 | ((i & 0x1e0) >> 5) << 4 | ((i & 0x200) >> 9) << 9)][j+TOTAL_NEURON_NUM_ON_CHIP*2]; //i
					}
				}
			}
			counterW = 0;
			for (i = 0; i < sizeX; i++) {
				for (ys = 0; ys < sizeY; ys++) {
					colW  = GetColourW((double) warrayW[i][ys], (double) VMIN, (double) VMAX); //-500, 500); // warray[i][ys]/1000
					singleplotW->pixels[counterW] = colW.r; //(uint16_t) ( (int)(colW.r) ); //*65535		// red
					singleplotW->pixels[counterW + 1] = colW.g; //(uint16_t) ( (int)(colW.g) ); //*65535	// green
					singleplotW->pixels[counterW + 2] = colW.b; //(uint16_t) ( (int)(colW.b) ); //*65535	// blue
					counterW += 3;
				}
			}
			caerFrameEventSetLengthXLengthYChannelNumber(singleplotW, sizeX, sizeY, 3, *weightplot); //add info to the frame
			caerFrameEventValidate(singleplotW, *weightplot); //validate frame
		}
/*		double warrayS[sizeX][sizeY];
		for (i = 0; i < sizeX; i++)
			for (j = 0; j < sizeY; j++)
				warrayS[i][j] = 0;
		*synapseplot = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, sizeX, sizeY, 3); //put info into frame
		if (*synapseplot != NULL) {
			caerFrameEvent singleplotS = caerFrameEventPacketGetEvent(*synapseplot, 0);
			for (i = 0; i < INPUT_N; i++) {
				for (j = 0; j < FEATURE1_N * FEATURE1_LAYERS_N; j++) {
					if ((int)(i/INPUT_L) >= (int)((j%FEATURE1_N)/FEATURE1_L)
							&& (int)(i/INPUT_L) < (int)((j%FEATURE1_N)/FEATURE1_L) + FILTER1_L
							&& i%INPUT_W >= (j%FEATURE1_N)%FEATURE1_W
							&& i%INPUT_W < (j%FEATURE1_N)%FEATURE1_W + FILTER1_W) {
						row_id = FILTER1_L*(int)((j%FEATURE1_N)/FEATURE1_W) + (int)(i/INPUT_W) - (int)((j%FEATURE1_N)/FEATURE1_W);
						col_id = FILTER1_W*((j%FEATURE1_N)%FEATURE1_W)+i%INPUT_W-(j%FEATURE1_N)%FEATURE1_W;
						feature_id = (int)(j/FEATURE1_N);
						warrayS[(feature_id >> 1)*FILTER1_L*FEATURE1_L + row_id][(feature_id & 0x1)*FILTER1_W*FEATURE1_W + col_id]
							= memory.synapseMap->buffer2d[(i & 0xf | ((i & 0x10) >> 4) << 8 | ((i & 0x1e0) >> 5) << 4 | ((i & 0x200) >> 9) << 9)][j+TOTAL_NEURON_NUM_ON_CHIP*2]; //i
					}
				}
			}
			counterS = 0;
			for (i = 0; i < sizeX; i++) {
				for (ys = 0; ys < sizeY; ys++) {
					colS  = GetColourS((double) warrayS[i][ys], (double) VMIN, (double) VMAX); //-500, 500); //
					singleplotS->pixels[counterS] = colS.r; //(uint16_t) ( (int)(colW.r) ); //*65535		// red
					singleplotS->pixels[counterS + 1] = colS.g; //(uint16_t) ( (int)(colW.g) ); //*65535	// green
					singleplotS->pixels[counterS + 2] = colS.b; //(uint16_t) ( (int)(colW.b) ); //*65535	// blue
					counterS += 3;
				}
			}
			caerFrameEventSetLengthXLengthYChannelNumber(singleplotS, sizeX, sizeY, 3, *synapseplot); //add info to the frame
			caerFrameEventValidate(singleplotS, *synapseplot); //validate frame
		} */
	}

	if (spike == NULL) { // Only process packets with content.
		return;
	}

	caerLearningFilterConfig(moduleData); // Update parameters
	int64_t neuronAddr = 0;

	if (state->reset == 1) {
		if (reseted == 0) {
			ResetNetwork(moduleData, eventSourceID);
			printf("\nNetwork reseted \n");
			int64_t i, j, ys, row_id, col_id, feature_id;
			double warrayW[sizeX][sizeY];
			for (i = 0; i < sizeX; i++)
				for (j = 0; j < sizeY; j++)
					warrayW[i][j] = 0;
			*weightplot = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, sizeX, sizeY, 3); //put info into frame
			if (*weightplot != NULL) {
				caerFrameEvent singleplotW = caerFrameEventPacketGetEvent(*weightplot, 0);
				for (i = 0; i < INPUT_N; i++) {
					for (j = 0; j < FEATURE1_N * FEATURE1_LAYERS_N; j++) {
						if ((int)(i/INPUT_L) >= (int)((j%FEATURE1_N)/FEATURE1_L)
								&& (int)(i/INPUT_L) < (int)((j%FEATURE1_N)/FEATURE1_L) + FILTER1_L
								&& i%INPUT_W >= (j%FEATURE1_N)%FEATURE1_W
								&& i%INPUT_W < (j%FEATURE1_N)%FEATURE1_W + FILTER1_W) {
							row_id = FILTER1_L*(int)((j%FEATURE1_N)/FEATURE1_W) + (int)(i/INPUT_W) - (int)((j%FEATURE1_N)/FEATURE1_W);
							col_id = FILTER1_W*((j%FEATURE1_N)%FEATURE1_W)+i%INPUT_W-(j%FEATURE1_N)%FEATURE1_W;
							feature_id = (int)(j/FEATURE1_N);
							warrayW[(feature_id >> 1)*FILTER1_L*FEATURE1_L + row_id][(feature_id & 0x1)*FILTER1_W*FEATURE1_W + col_id]
								= memory.weightMap->buffer2d[(i & 0xf | ((i & 0x10) >> 4) << 8 | ((i & 0x1e0) >> 5) << 4 | ((i & 0x200) >> 9) << 9)][j+TOTAL_NEURON_NUM_ON_CHIP*2]; //i
						}
					}
				}
				counterW = 0;
				for (i = 0; i < sizeX; i++) {
					for (ys = 0; ys < sizeY; ys++) {
						colW  = GetColourW((double) warrayW[i][ys], (double) VMIN, (double) VMAX); //-500, 500); // warray[i][ys]/1000
						singleplotW->pixels[counterW] = colW.r; //(uint16_t) ( (int)(colW.r) ); //*65535		// red
						singleplotW->pixels[counterW + 1] = colW.g; //(uint16_t) ( (int)(colW.g) ); //*65535	// green
						singleplotW->pixels[counterW + 2] = colW.b; //(uint16_t) ( (int)(colW.b) ); //*65535	// blue
						counterW += 3;
					}
				}
				caerFrameEventSetLengthXLengthYChannelNumber(singleplotW, sizeX, sizeY, 3, *weightplot); //add info to the frame
				caerFrameEventValidate(singleplotW, *weightplot); //validate frame
			}
			double warrayS[sizeX][sizeY];
			for (i = 0; i < sizeX; i++)
				for (j = 0; j < sizeY; j++)
					warrayS[i][j] = 0;
			*synapseplot = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, sizeX, sizeY, 3); //put info into frame
			if (*synapseplot != NULL) {
				caerFrameEvent singleplotS = caerFrameEventPacketGetEvent(*synapseplot, 0);
				for (i = 0; i < INPUT_N; i++) {
					for (j = 0; j < FEATURE1_N * FEATURE1_LAYERS_N; j++) {
						if ((int)(i/INPUT_L) >= (int)((j%FEATURE1_N)/FEATURE1_L)
								&& (int)(i/INPUT_L) < (int)((j%FEATURE1_N)/FEATURE1_L) + FILTER1_L
								&& i%INPUT_W >= (j%FEATURE1_N)%FEATURE1_W
								&& i%INPUT_W < (j%FEATURE1_N)%FEATURE1_W + FILTER1_W) {
							row_id = FILTER1_L*(int)((j%FEATURE1_N)/FEATURE1_W) + (int)(i/INPUT_W) - (int)((j%FEATURE1_N)/FEATURE1_W);
							col_id = FILTER1_W*((j%FEATURE1_N)%FEATURE1_W)+i%INPUT_W-(j%FEATURE1_N)%FEATURE1_W;
							feature_id = (int)(j/FEATURE1_N);
							warrayS[(feature_id >> 1)*FILTER1_L*FEATURE1_L + row_id][(feature_id & 0x1)*FILTER1_W*FEATURE1_W + col_id]
								= memory.synapseMap->buffer2d[(i & 0xf | ((i & 0x10) >> 4) << 8 | ((i & 0x1e0) >> 5) << 4 | ((i & 0x200) >> 9) << 9)][j+TOTAL_NEURON_NUM_ON_CHIP*2]; //i
						}
					}
				}
				counterS = 0;
				for (i = 0; i < sizeX; i++) {
					for (ys = 0; ys < sizeY; ys++) {
						colS  = GetColourS((double) warrayS[i][ys], (double) VMIN, (double) VMAX); //-500, 500); //
						singleplotS->pixels[counterS] = colS.r; //(uint16_t) ( (int)(colW.r) ); //*65535		// red
						singleplotS->pixels[counterS + 1] = colS.g; //(uint16_t) ( (int)(colW.g) ); //*65535	// green
						singleplotS->pixels[counterS + 2] = colS.b; //(uint16_t) ( (int)(colW.b) ); //*65535	// blue
						counterS += 3;
					}
				}
				caerFrameEventSetLengthXLengthYChannelNumber(singleplotS, sizeX, sizeY, 3, *synapseplot); //add info to the frame
				caerFrameEventValidate(singleplotS, *synapseplot); //validate frame
			}
			printf("Visualizer reseted \n");
			reseted = 1;
		}
	} else {
		reseted = 0;
	}

	CAER_SPIKE_ITERATOR_VALID_START(spike) // Iterate over events and update weight

		int64_t ts = caerSpikeEventGetTimestamp64(caerSpikeIteratorElement, spike); // Get values on which to operate.

		uint32_t neuronId = caerSpikeEventGetNeuronID(caerSpikeIteratorElement);
		uint32_t coreId = caerSpikeEventGetSourceCoreID(caerSpikeIteratorElement);
		uint32_t chipId_t = caerSpikeEventGetChipID(caerSpikeIteratorElement);
		uint32_t chipId;

		if (chipId_t == 1) //DYNAPSE_CONFIG_DYNAPSE_U0 Why can I receive chip Id 1???
			chipId = 1;
		else if (chipId_t == DYNAPSE_CONFIG_DYNAPSE_U1)
			chipId = 2;
		else if (chipId_t == DYNAPSE_CONFIG_DYNAPSE_U2)
			chipId = 3;
		else if (chipId_t == DYNAPSE_CONFIG_DYNAPSE_U3)
			chipId = 4;

		if (chipId > 0 && chipId <= 4) {
			neuronAddr = chipId << 10 | coreId << 8 | neuronId;
			memory.spikeQueue->buffer2d[memory.wrPointer][0] = neuronAddr; // Put spike address into the queue
			memory.spikeQueue->buffer2d[memory.wrPointer][1] = ts; // Put spike address into the queue
			memory.spikeCounter += 1;
			memory.wrPointer = (memory.wrPointer + 1) % SPIKE_QUEUE_LENGTH;
		}

		uint8_t endSearching = 0;
		int64_t deltaTimeAccumulated = 0;

		int64_t i, j, row_id_t, col_id_t, row_id, col_id, feature_id;
		if (memory.wrPointer - memory.preRdPointer >= MINIMUM_CONSIDERED_SPIKE_NUM) {

			int64_t preSpikeAddr = memory.spikeQueue->buffer2d[memory.preRdPointer][0];
			int64_t preSpikeTime = memory.spikeQueue->buffer2d[memory.preRdPointer][1];
			memory.spikeCounter -= 1;
			memory.preRdPointer = (memory.preRdPointer + 1) % SPIKE_QUEUE_LENGTH;
			double new_weight, new_synapse;
			for (uint64_t postRdPointer = (memory.preRdPointer + 1) % SPIKE_QUEUE_LENGTH;
					endSearching != 1;
					postRdPointer = (postRdPointer + 1) % SPIKE_QUEUE_LENGTH) {
				int64_t postSpikeAddr = memory.spikeQueue->buffer2d[postRdPointer][0];
				int64_t postSpikeTime = memory.spikeQueue->buffer2d[postRdPointer][1];
				int64_t deltaTime = (int64_t) (postSpikeTime - preSpikeTime); //should be positive
				//if the time delay is out of considered range

				if (deltaTime <= 0)
					break;

				if (deltaTime < MAXIMUM_CONSIDERED_SPIKE_DELAY) { // DELTA_WEIGHT_LUT_LENGTH
					double deltaWeight = deltaWeights[DELTA_WEIGHT_LUT_LENGTH-deltaTime];
					ModifySynapse(moduleData, eventSourceID, preSpikeAddr, postSpikeAddr, deltaWeight);

					if (*weightplot != NULL) {
						caerFrameEvent singleplotW = caerFrameEventPacketGetEvent(*weightplot, 0);
						if (memory.connectionMap->buffer2d[preSpikeAddr-MEMORY_NEURON_ADDR_OFFSET][postSpikeAddr-MEMORY_NEURON_ADDR_OFFSET] == 1) {
							new_weight = memory.weightMap->buffer2d[preSpikeAddr-MEMORY_NEURON_ADDR_OFFSET][postSpikeAddr-MEMORY_NEURON_ADDR_OFFSET] + deltaWeight * state->learningRate;
							colW  = GetColourW(new_weight, (double) VMIN, (double) VMAX); // / 1000 ?
	//						counterW = (uint32_t) (((preSpikeAddr-MEMORY_NEURON_ADDR_OFFSET) * VISUALIZER_WIDTH)+(postSpikeAddr-MEMORY_NEURON_ADDR_OFFSET-TOTAL_NEURON_NUM_ON_CHIP))*3; //-TOTAL_NEURON_NUM_ON_CHIP
							i = preSpikeAddr-MEMORY_NEURON_ADDR_OFFSET;
							j = (postSpikeAddr-MEMORY_NEURON_ADDR_OFFSET)%TOTAL_NEURON_NUM_ON_CHIP;
							feature_id = (int)(j/FEATURE1_N);
							row_id_t = FILTER1_L * (int)((j%FEATURE1_N)/FEATURE1_W) + (int)(i/INPUT_W) - (int)((j%FEATURE1_N)/FEATURE1_W);
							row_id = (feature_id >> 1)*FILTER1_L*FEATURE1_L + row_id_t;
							col_id_t = FILTER1_W * ((j%FEATURE1_N)%FEATURE1_W) + i % INPUT_W - (j%FEATURE1_N)%FEATURE1_W;
							col_id = (feature_id & 0x1)*FILTER1_W*FEATURE1_W + col_id_t;
							counterW = (uint32_t) ((row_id * VISUALIZER_WIDTH) + col_id) * 3;
							singleplotW->pixels[counterW] = colW.r; //(uint16_t) ( (int)(colW.r) ); //*65535		// red 65000; //
							singleplotW->pixels[counterW + 1] = colW.g; //(uint16_t) ( (int)(colW.g) ); //*65535	// green
							singleplotW->pixels[counterW + 2] = colW.b; //(uint16_t) ( (int)(colW.b) ); //*65535	// blue
						}
						int64_t preSpikeAddr_t = preSpikeAddr;
						int64_t postSpikeAddr_t = postSpikeAddr;
						preSpikeAddr = postSpikeAddr_t;
						postSpikeAddr = preSpikeAddr_t;
						if (memory.connectionMap->buffer2d[preSpikeAddr-MEMORY_NEURON_ADDR_OFFSET][postSpikeAddr-MEMORY_NEURON_ADDR_OFFSET] == 1) {
							new_weight = memory.weightMap->buffer2d[preSpikeAddr-MEMORY_NEURON_ADDR_OFFSET][postSpikeAddr-MEMORY_NEURON_ADDR_OFFSET] - deltaWeight * state->learningRate;
							colW  = GetColourW(new_weight, (double) VMIN, (double) VMAX); // / 1000 ?
	//						counterW = (uint32_t) (((postSpikeAddr-MEMORY_NEURON_ADDR_OFFSET) * VISUALIZER_WIDTH)+(preSpikeAddr-MEMORY_NEURON_ADDR_OFFSET-TOTAL_NEURON_NUM_ON_CHIP))*3; //-TOTAL_NEURON_NUM_ON_CHIP
							i = preSpikeAddr-MEMORY_NEURON_ADDR_OFFSET;
							j = (postSpikeAddr-MEMORY_NEURON_ADDR_OFFSET)%TOTAL_NEURON_NUM_ON_CHIP;
							feature_id = (int)(j/FEATURE1_N);
							row_id_t = FILTER1_L * (int)((j%FEATURE1_N)/FEATURE1_W) + (int)(i/INPUT_W) - (int)((j%FEATURE1_N)/FEATURE1_W);
							row_id = (feature_id >> 1)*FILTER1_L*FEATURE1_L + row_id_t;
							col_id_t = FILTER1_W * ((j%FEATURE1_N)%FEATURE1_W) + i % INPUT_W - (j%FEATURE1_N)%FEATURE1_W;
							col_id = (feature_id & 0x1)*FILTER1_W*FEATURE1_W + col_id_t;
							counterW = (uint32_t) ((row_id * VISUALIZER_WIDTH) + col_id) * 3;
							singleplotW->pixels[counterW] = colW.r; //(uint16_t) ( (int)(colW.r) ); //*65535		// red
							singleplotW->pixels[counterW + 1] = colW.g; //(uint16_t) ( (int)(colW.g) ); //*65535	// green
							singleplotW->pixels[counterW + 2] = colW.b; //(uint16_t) ( (int)(colW.b) ); //*65535	// blue
						}
					}

/*					if (*synapseplot != NULL) {
						caerFrameEvent singleplotS = caerFrameEventPacketGetEvent(*weightplot, 0);
						if (memory.connectionMap->buffer2d[preSpikeAddr-MEMORY_NEURON_ADDR_OFFSET][postSpikeAddr-MEMORY_NEURON_ADDR_OFFSET] == 1) {
							new_synapse = 0; //memory.weightMap->buffer2d[preSpikeAddr-MEMORY_NEURON_ADDR_OFFSET][postSpikeAddr-MEMORY_NEURON_ADDR_OFFSET] + deltaWeight;
							colW  = GetColourW(new_synapse, (double) VMIN, (double) VMAX); // / 1000 ?
	//						counterW = (uint32_t) (((preSpikeAddr-MEMORY_NEURON_ADDR_OFFSET) * VISUALIZER_WIDTH)+(postSpikeAddr-MEMORY_NEURON_ADDR_OFFSET-TOTAL_NEURON_NUM_ON_CHIP))*3; //-TOTAL_NEURON_NUM_ON_CHIP
							i = preSpikeAddr-MEMORY_NEURON_ADDR_OFFSET;
							j = (postSpikeAddr-MEMORY_NEURON_ADDR_OFFSET)%TOTAL_NEURON_NUM_ON_CHIP;
							feature_id = (int)(j/FEATURE1_N);
							row_id_t = FILTER1_L * (int)((j%FEATURE1_N)/FEATURE1_W) + (int)(i/INPUT_W) - (int)((j%FEATURE1_N)/FEATURE1_W);
							row_id = (feature_id >> 1)*FILTER1_L*FEATURE1_L + row_id_t;
							col_id_t = FILTER1_W * ((j%FEATURE1_N)%FEATURE1_W) + i % INPUT_W - (j%FEATURE1_N)%FEATURE1_W;
							col_id = (feature_id & 0x1)*FILTER1_W*FEATURE1_W + col_id_t;
							counterS = (uint32_t) ((row_id * VISUALIZER_WIDTH) + col_id) * 3;
							singleplotS->pixels[counterS] = colW.r; //(uint16_t) ( (int)(colW.r) ); //*65535		// red 65000; //
							singleplotS->pixels[counterS + 1] = colW.g; //(uint16_t) ( (int)(colW.g) ); //*65535	// green
							singleplotS->pixels[counterS + 2] = colW.b; //(uint16_t) ( (int)(colW.b) ); //*65535	// blue
						}
						int64_t preSpikeAddr_t = preSpikeAddr;
						int64_t postSpikeAddr_t = postSpikeAddr;
						preSpikeAddr = postSpikeAddr_t;
						postSpikeAddr = preSpikeAddr_t;
						if (memory.connectionMap->buffer2d[preSpikeAddr-MEMORY_NEURON_ADDR_OFFSET][postSpikeAddr-MEMORY_NEURON_ADDR_OFFSET] == 1) {
							new_synapse = 0; //memory.weightMap->buffer2d[preSpikeAddr-MEMORY_NEURON_ADDR_OFFSET][postSpikeAddr-MEMORY_NEURON_ADDR_OFFSET] - deltaWeight;
							colW  = GetColourW(new_synapse, (double) VMIN, (double) VMAX); // / 1000 ?
	//						counterW = (uint32_t) (((postSpikeAddr-MEMORY_NEURON_ADDR_OFFSET) * VISUALIZER_WIDTH)+(preSpikeAddr-MEMORY_NEURON_ADDR_OFFSET-TOTAL_NEURON_NUM_ON_CHIP))*3; //-TOTAL_NEURON_NUM_ON_CHIP
							i = preSpikeAddr-MEMORY_NEURON_ADDR_OFFSET;
							j = (postSpikeAddr-MEMORY_NEURON_ADDR_OFFSET)%TOTAL_NEURON_NUM_ON_CHIP;
							feature_id = (int)(j/FEATURE1_N);
							row_id_t = FILTER1_L * (int)((j%FEATURE1_N)/FEATURE1_W) + (int)(i/INPUT_W) - (int)((j%FEATURE1_N)/FEATURE1_W);
							row_id = (feature_id >> 1)*FILTER1_L*FEATURE1_L + row_id_t;
							col_id_t = FILTER1_W * ((j%FEATURE1_N)%FEATURE1_W) + i % INPUT_W - (j%FEATURE1_N)%FEATURE1_W;
							col_id = (feature_id & 0x1)*FILTER1_W*FEATURE1_W + col_id_t;
							counterS = (uint32_t) ((row_id * VISUALIZER_WIDTH) + col_id) * 3;
							singleplotS->pixels[counterS] = colW.r; //(uint16_t) ( (int)(colW.r) ); //*65535		// red
							singleplotS->pixels[counterS + 1] = colW.g; //(uint16_t) ( (int)(colW.g) ); //*65535	// green
							singleplotS->pixels[counterS + 2] = colW.b; //(uint16_t) ( (int)(colW.b) ); //*65535	// blue
						}
					} */

				}

				if (postRdPointer == memory.wrPointer - 1)
					endSearching = 1; //break;
				else if (deltaTimeAccumulated > MAXIMUM_CONSIDERED_SPIKE_DELAY) //MAXIMUM_CONSIDERED_SPIKE_DELAY
					endSearching = 1; //break;
//				else if (deltaTimeAccumulated > 10)
//					break;
				deltaTimeAccumulated += deltaTime;
//				printf("deltaTimeAccumulated: %d\n", (int) deltaTimeAccumulated);
			}
//			printf("spikeCounter: %d\n", (int) memory.spikeCounter);
		}

	CAER_SPIKE_ITERATOR_VALID_END

	if (state->stimulate == true) {
		if (time_count > time_count_last + 5) {
			DisableStimuliGen(moduleData, eventSourceID);
			time_count_last = time_count;
		}

		if (time_count > time_count_last + 1) {
			pattern = (pattern + 1) % 3 + 4;
			EnableStimuliGen(moduleData, eventSourceID, pattern);
			time_count_last = time_count;
		}
	} else {
		DisableStimuliGen(moduleData, eventSourceID);
	}
}

bool ModifySynapse(caerModuleData moduleData, int16_t eventSourceID, int64_t preSpikeAddr, int64_t postSpikeAddr, double deltaWeight) {

//	caerDeviceHandle usb_handle = ((caerInputDynapseState) moduleData->moduleState)->deviceState;
	LFilterState state = moduleData->moduleState;
	double new_weight;
	int64_t min = MEMORY_NEURON_ADDR_OFFSET;
	int64_t i;
	int32_t filterSize;
	int64_t	preNeuronId;
	uint32_t camId;
	int8_t synapseType;
	int8_t min_initialized;
	int64_t preNeuronAddr, postNeuronAddr;
	preNeuronAddr = preSpikeAddr; postNeuronAddr = postSpikeAddr;
	if (memory.connectionMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] == 1) {
		new_weight = memory.weightMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] + deltaWeight;
		filterSize = memory.filterMapSize->buffer2d[postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][0];
		int8_t camFound = 0;
		min_initialized = 0;
		for (i = 0; i < filterSize; i++) {
			preNeuronId = memory.filterMap->buffer2d[postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][i];
			if (memory.synapseMap->buffer2d[preNeuronId-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] > 0) { //Real synapse exists
				if (min_initialized == 0) {
					min = preNeuronId;
					min_initialized = 1;
				}
				if (preNeuronId == preNeuronAddr) { //synapse already exists
					camFound = 1;
					break;
				}
				if (memory.weightMap->buffer2d[preNeuronId-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] < memory.weightMap->buffer2d[min-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET]) {
					min = preNeuronId;
				}
			}
		}
		if (camFound == 0) {
			//inhibit weight is not counted in this algorithm right now
			//adaptive mechanism is implemented on neuron
			//synapse will be increased step by step, but synapse weight will not be rounded to 1 or 2
			if (memory.camSize->buffer2d[postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][0] < TOTAL_CAM_NUM - 1) { //CAM not full
				for (camId = 0; camId < TOTAL_CAM_NUM; camId++) {
					if (memory.camMap->buffer2d[postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][camId] == 0)
						break;
				}
				synapseType = EXCITATORY_SLOW_SYNAPSE_ID;
				WriteCam(moduleData, eventSourceID, (uint32_t) preNeuronAddr, (uint32_t) postNeuronAddr, camId, synapseType);
				memory.synapseMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] = synapseType;
				memory.camMap->buffer2d[postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][camId] = 1;
				memory.camSize->buffer2d[postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][0] += 1;
				memory.filterMapSize->buffer2d[postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][0] += 1;
			} else if (memory.weightMap->buffer2d[min-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] < new_weight) { //should be always true memory.synapseMap->buffer2d[preNeuronAddr][postNeuronAddr] == NO_SYNAPSE_ID
				synapseType = EXCITATORY_SLOW_SYNAPSE_ID;
				camId = (uint32_t) memory.filterCamMap->buffer2d[min-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET]; //replace the CAM of MIN by the strengthened one
				WriteCam(moduleData, eventSourceID, (uint32_t) preNeuronAddr, (uint32_t) postNeuronAddr, camId, synapseType);
				memory.synapseMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] = synapseType;
				memory.synapseMap->buffer2d[min-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] = NO_SYNAPSE_ID;
				memory.filterCamMap->buffer2d[postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][min-MEMORY_NEURON_ADDR_OFFSET] = 0;
				memory.filterCamMap->buffer2d[postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] = (int32_t) camId;
			}
		} else if (camFound == 1 && new_weight > memory.weightMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] * state->learningRate) {
			if (memory.synapseMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] == EXCITATORY_SLOW_SYNAPSE_ID) {
				synapseType = EXCITATORY_FAST_SYNAPSE_ID;
				camId = (uint32_t) memory.filterCamMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET]; //get its CAM id
				WriteCam(moduleData, eventSourceID, (uint32_t) preNeuronAddr, (uint32_t) postNeuronAddr, camId, synapseType);
				memory.synapseMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] = synapseType;
			}
		}
		memory.weightMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] = new_weight;
//		printf("+%f\n", new_weight);
	}

	preNeuronAddr = postSpikeAddr; postNeuronAddr = preSpikeAddr;
	if (memory.connectionMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] == 1) {
		new_weight = memory.weightMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] - deltaWeight;
		filterSize = memory.filterMapSize->buffer2d[postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][0];
		min_initialized = 0;
		for (i = 0; i < filterSize; i++) {
			preNeuronId = memory.filterMap->buffer2d[postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][i];
			if (memory.synapseMap->buffer2d[preNeuronId-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] > 0) { //Real synapse exists
				if (min_initialized == 0) {
					min = preNeuronId;
					min_initialized = 1;
				}
				if (memory.weightMap->buffer2d[preNeuronId-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] < memory.weightMap->buffer2d[min-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET])
					min = preNeuronId;
			}
		}
		if (memory.weightMap->buffer2d[min-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] > new_weight) {
			//inhibit weight is not counted in this algorithm right now
			//adaptive mechanism is implemented on neuron
			//synapse will be increased step by step, but synapse weight will not be rounded to 1 or 2
			if (memory.synapseMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] == EXCITATORY_FAST_SYNAPSE_ID) {
				synapseType = EXCITATORY_SLOW_SYNAPSE_ID;
				camId = (uint32_t) memory.filterCamMap->buffer2d[min-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET]; //replace the CAM of MIN by the strengthened one
				WriteCam(moduleData, eventSourceID, (uint32_t) preNeuronAddr, (uint32_t) postNeuronAddr, camId, synapseType);
				memory.synapseMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] = synapseType;
			}
			else if (memory.synapseMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] == EXCITATORY_SLOW_SYNAPSE_ID) {
				synapseType = NO_SYNAPSE_ID;
				camId = (uint32_t) memory.filterCamMap->buffer2d[min-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET]; //replace the CAM of MIN by the strengthened one
				WriteCam(moduleData, eventSourceID, (uint32_t) preNeuronAddr, (uint32_t) postNeuronAddr, camId, synapseType);
				memory.filterCamMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] = 0;
				memory.synapseMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] = synapseType;
				memory.filterMapSize->buffer2d[postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][0] -= 1;
				memory.camMap->buffer2d[postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][camId] = 0;
				memory.camSize->buffer2d[postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][camId] -= 1;
			}
		}
		memory.weightMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] = new_weight;
	}

	return (true);
}

//reset the network to the initial state
bool ResetNetwork(caerModuleData moduleData, int16_t eventSourceID)
{
	time_count = 0;
	signal(SIGALRM, SignalHandler); //register the hand-made timer function
	SetTimer();

	ClearAllCam(moduleData, eventSourceID); //only for 1st chip

	LFilterState state = moduleData->moduleState;
	int8_t exType = state->resetExType; //initial synapse type fast or slow
	int8_t inType = state->resetInType; //initial synapse type fast or slow

	memory.connectionMap = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) TOTAL_NEURON_NUM_ON_BOARD);
	memory.filterMap = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) MAXIMUM_FILTER_SIZE);
	memory.filterCamMap = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) MAXIMUM_FILTER_SIZE);
	memory.filterMapSize = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) FILTER_MAP_SIZE_WIDTH);

	memory.weightMap = simple2DBufferInitDouble((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) TOTAL_NEURON_NUM_ON_BOARD);
	memory.synapseMap = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) TOTAL_NEURON_NUM_ON_BOARD);
	memory.camMap = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) TOTAL_CAM_NUM);
	memory.camSize = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) CAM_SIZE_WIDTH);
	memory.sramMap = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) TOTAL_SRAM_NUM);
	memory.sramMapContent = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) TOTAL_SRAM_NUM);
	memory.spikeQueue = simple2DBufferInitLong((size_t) SPIKE_QUEUE_LENGTH, (size_t) SPIKE_QUEUE_WIDTH);

	uint32_t chipId, coreId;
	//create stimuli layer
	uint32_t neuronId;
	uint32_t stimuli_layer[INPUT_N];
	for (neuronId = 0; neuronId < INPUT_N; neuronId++) {
		chipId = VIRTUAL_CHIP_ID;
		stimuli_layer[neuronId] = chipId << NEURON_CHIPID_SHIFT |
				(neuronId & 0xf | ((neuronId & 0x10) >> 4) << 8 | ((neuronId & 0x1e0) >> 5) << 4 | ((neuronId & 0x200) >> 9) << 9);
	}
	//create input layer
	uint32_t input_layer[INPUT_N];
	for (neuronId = 0; neuronId < INPUT_N; neuronId++) {
		chipId = CHIP_UP_LEFT_ID;
		input_layer[neuronId] = chipId << NEURON_CHIPID_SHIFT |
				(neuronId & 0xf | ((neuronId & 0x10) >> 4) << 8 | ((neuronId & 0x1e0) >> 5) << 4 | ((neuronId & 0x200) >> 9) << 9);
	}
	//create feature layer 1
	uint32_t feature_layer1[FEATURE1_N * FEATURE1_LAYERS_N];
	for (neuronId = 0; neuronId < FEATURE1_N * FEATURE1_LAYERS_N; neuronId++) {
		chipId = CHIP_DOWN_LEFT_ID;
		feature_layer1[neuronId] = chipId << NEURON_CHIPID_SHIFT | neuronId;
	}
	//create pooling layer 1
	uint32_t pooling_layer1[POOLING1_N * POOLING1_LAYERS_N];
	for (neuronId = 0; neuronId < POOLING1_N * POOLING1_LAYERS_N; neuronId++) {
		chipId = CHIP_DOWN_LEFT_ID;
		coreId = CORE_DOWN_RIGHT_ID;
		pooling_layer1[neuronId] = chipId << NEURON_CHIPID_SHIFT | coreId << NEURON_COREID_SHIFT |
				neuronId & 0x7 | ((neuronId & 0xf0) >> 4) << 3 | ((neuronId & 0x8) >> 3) << 7;
	}
	//create feature layer 2
	uint32_t feature_layer2[FEATURE2_N * FEATURE2_LAYERS_N];
	for (neuronId = 0; neuronId < FEATURE2_N * FEATURE2_LAYERS_N; neuronId++) {
		chipId = CHIP_UP_RIGHT_ID;
		feature_layer2[neuronId] = chipId << NEURON_CHIPID_SHIFT |
				neuronId & 0x3 | ((neuronId & 0xf0) >> 4) << 2 | ((neuronId & 0xc) >> 2) << 6 | ((neuronId & 0x300) >> 8) << 8;
	}
	//create pooling layer 2
	uint32_t pooling_layer2[POOLING2_N * POOLING2_LAYERS_N];
	for (neuronId = 0; neuronId < POOLING2_N * POOLING2_LAYERS_N; neuronId++) {
		chipId = CHIP_UP_RIGHT_ID;
		coreId = CORE_DOWN_LEFT_ID;
		pooling_layer2[neuronId] = chipId << NEURON_CHIPID_SHIFT | coreId << NEURON_COREID_SHIFT |
				neuronId & 0x1 | ((neuronId & 0xf0) >> 4) << 2 | ((neuronId & 0xe) >> 1) << 5;
	}
	//create output layer 1
	uint32_t output_layer1[OUTPUT1_N];
	for (neuronId = 0; neuronId < OUTPUT1_N; neuronId++) {
		chipId = CHIP_DOWN_RIGHT_ID;
		output_layer1[neuronId] = chipId << NEURON_CHIPID_SHIFT | neuronId;
	}
	//create output layer 2
	uint32_t output_layer2[OUTPUT2_N];
	for (neuronId = 0; neuronId < OUTPUT2_N; neuronId++) {
		chipId = CHIP_DOWN_RIGHT_ID;
		coreId = CORE_DOWN_LEFT_ID;
		output_layer2[neuronId] = chipId << NEURON_CHIPID_SHIFT | coreId << NEURON_COREID_SHIFT | neuronId;
	}

	//stimuli to input
	int preNeuronId;
	int postNeuronId;
	int randNumCount;
	for (preNeuronId = 0; preNeuronId < INPUT_N; preNeuronId++)
		for (postNeuronId = 0; postNeuronId < INPUT_N; postNeuronId++) {
			if (preNeuronId == postNeuronId) {
				BuildSynapse(moduleData, eventSourceID, stimuli_layer[preNeuronId], input_layer[postNeuronId], 2, EXTERNAL_REAL_SYNAPSE); //exType
			}
		}
	//input to feature1
	for (postNeuronId = 0; postNeuronId < FEATURE1_N*FEATURE1_LAYERS_N; postNeuronId++) { //first sweep POST, then PRE
		//generate random binary number 1D array
		int64_t rand1DBinaryArray[FILTER1_N]; //FILTER1_N-FEATURE1_CAM_INHIBITORY_N
		GetRand1DBinaryArray(rand1DBinaryArray, FILTER1_N, TOTAL_CAM_NUM); //FILTER1_N-FEATURE1_CAM_INHIBITORY_N
		randNumCount = 0;
		for (preNeuronId = 0; preNeuronId < INPUT_N; preNeuronId++) {
			int pre_id = preNeuronId;
			int post_id = postNeuronId;
			if ((int)(pre_id/INPUT_W) >= (int)((post_id%FEATURE1_N)/FEATURE1_W)
					&& (int)(pre_id/INPUT_W) < (int)((post_id%FEATURE1_N)/FEATURE1_W) + FILTER1_L
					&& pre_id%INPUT_W >= (post_id%FEATURE1_N)%FEATURE1_W
					&& pre_id%INPUT_W < (post_id%FEATURE1_N)%FEATURE1_W + FILTER1_W) {
				//randomly reset, depends on the ratio of total CAM number and FILTER1_N-FEATURE1_CAM_INHIBITORY_N
				if (rand1DBinaryArray[randNumCount] == 1) //build a real synapse
					BuildSynapse(moduleData, eventSourceID, input_layer[preNeuronId], feature_layer1[postNeuronId], exType, REAL_SYNAPSE);
				else
					BuildSynapse(moduleData, eventSourceID, input_layer[preNeuronId], feature_layer1[postNeuronId], exType, VIRTUAL_SYNAPSE);
				randNumCount += 1;
			}
		}
	}
	//feature1 to feature1
	for (preNeuronId = 0; preNeuronId < FEATURE1_N*FEATURE1_LAYERS_N; preNeuronId++)
		for (postNeuronId = 0; postNeuronId < FEATURE1_N*FEATURE1_LAYERS_N; postNeuronId++) {
			if ((int)(preNeuronId/FEATURE1_N)!=(int)(postNeuronId/FEATURE1_N) && (preNeuronId%FEATURE1_N)==(postNeuronId%FEATURE1_N)) {
				BuildSynapse(moduleData, eventSourceID, feature_layer1[preNeuronId], feature_layer1[postNeuronId], (int16_t) (-1 * inType), REAL_SYNAPSE);
			}
		}
	//feature1 to pooling1
	for (preNeuronId = 0; preNeuronId < FEATURE1_N*FEATURE1_LAYERS_N; preNeuronId++)
		for (postNeuronId = 0; postNeuronId < POOLING1_N*POOLING1_LAYERS_N; postNeuronId++) {
			if ((int)((int)((preNeuronId%FEATURE1_N)/FEATURE1_L)/(int)(FEATURE1_L/POOLING1_L)) == (int)((postNeuronId%POOLING1_N)/POOLING1_L)
					&& (int)(((preNeuronId%FEATURE1_N)%FEATURE1_W)/(int)(FEATURE1_W/POOLING1_W)) == (postNeuronId%POOLING1_N)%POOLING1_W) {
				BuildSynapse(moduleData, eventSourceID, feature_layer1[preNeuronId], pooling_layer1[postNeuronId], exType, REAL_SYNAPSE);
			}
		}
	//pooling1 to pooling1
	for (preNeuronId = 0; preNeuronId < POOLING1_N*POOLING1_LAYERS_N; preNeuronId++)
		for (postNeuronId = 0; postNeuronId < POOLING1_N*POOLING1_LAYERS_N; postNeuronId++) {
			if ((int)(preNeuronId/POOLING1_N)!=(int)(postNeuronId/POOLING1_N) && (preNeuronId%POOLING1_N)==(postNeuronId%POOLING1_N)) {
				BuildSynapse(moduleData, eventSourceID, pooling_layer1[preNeuronId], pooling_layer1[postNeuronId], inType, REAL_SYNAPSE);
			}
		}
	//pooling1 to feature2
	for (postNeuronId = 0; postNeuronId < FEATURE2_N*FEATURE2_LAYERS_N; postNeuronId++) {
		//generate random binary number 1D array
		int64_t rand1DBinaryArray[FILTER2_N * POOLING1_LAYERS_N];
		GetRand1DBinaryArray(rand1DBinaryArray, FILTER2_N * POOLING1_LAYERS_N, TOTAL_CAM_NUM);
		randNumCount = 0;
		for (preNeuronId = 0; preNeuronId < POOLING1_N*POOLING1_LAYERS_N; preNeuronId++) {
			if ((int)((preNeuronId%POOLING1_N)/POOLING1_W) >= (int)((postNeuronId%FEATURE2_N)/FEATURE2_W) &&
					(int)((preNeuronId%POOLING1_N)/POOLING1_W) < (int)((postNeuronId%FEATURE2_N)/FEATURE2_W) + FILTER2_L &&
					(preNeuronId%POOLING1_N)%POOLING1_W >= (postNeuronId%FEATURE2_N)%FEATURE2_W &&
					(preNeuronId%POOLING1_N)%POOLING1_W < (postNeuronId%FEATURE2_N)%FEATURE2_W + FILTER2_W) {
				//randomly reset, depends on the ratio of total CAM number and FILTER2_N-FEATURE2_CAM_INHIBITORY_N
				if (rand1DBinaryArray[randNumCount] == 1)
					BuildSynapse(moduleData, eventSourceID, pooling_layer1[preNeuronId], feature_layer2[postNeuronId], exType, REAL_SYNAPSE);
				else
					BuildSynapse(moduleData, eventSourceID, pooling_layer1[preNeuronId], feature_layer2[postNeuronId], exType, VIRTUAL_SYNAPSE);
				randNumCount += 1;
			}
		}
	}
	//feature2 to feature2
	for (preNeuronId = 0; preNeuronId < FEATURE2_N*FEATURE2_LAYERS_N; preNeuronId++)
		for (postNeuronId = 0; postNeuronId < FEATURE2_N*FEATURE2_LAYERS_N; postNeuronId++) {
			if ((int)(preNeuronId/FEATURE2_N)!=(int)(postNeuronId/FEATURE2_N) && (preNeuronId%FEATURE2_N)==(postNeuronId%FEATURE2_N)) {
				BuildSynapse(moduleData, eventSourceID, feature_layer2[preNeuronId], feature_layer2[postNeuronId], inType, REAL_SYNAPSE);
			}
		}
	//feature2 to pooling2
	for (postNeuronId = 0; postNeuronId < POOLING2_N*POOLING2_LAYERS_N; postNeuronId++) {
		//generate random binary number 1D array
		int64_t rand1DBinaryArray[(FEATURE2_N/POOLING2_N)*FEATURE2_LAYERS_N];
		GetRand1DBinaryArray(rand1DBinaryArray, (FEATURE2_N/POOLING2_N)*FEATURE2_LAYERS_N, TOTAL_CAM_NUM);
		randNumCount = 0;
		for (preNeuronId = 0; preNeuronId < FEATURE2_N*FEATURE2_LAYERS_N; preNeuronId++) {
			if ((int)((int)((preNeuronId%FEATURE2_N)/FEATURE2_L)/(int)(FEATURE2_L/POOLING2_L)) == (int)((postNeuronId%POOLING2_N)/POOLING2_L)
					&& (int)(((preNeuronId%FEATURE2_N)%FEATURE2_W)/(int)(FEATURE2_W/POOLING2_W)) == (postNeuronId%POOLING2_N)%POOLING2_W) {
				//randomly reset, depends on the ratio of total CAM number and (FEATURE2_N/POOLING2_N)*FEATURE2_LAYERS_N-POOLING2_CAM_INHIBITORY_N
				if (rand1DBinaryArray[randNumCount] == 1)
					BuildSynapse(moduleData, eventSourceID, feature_layer2[preNeuronId], pooling_layer2[postNeuronId], exType, REAL_SYNAPSE);
				else
					BuildSynapse(moduleData, eventSourceID, feature_layer2[preNeuronId], pooling_layer2[postNeuronId], exType, VIRTUAL_SYNAPSE);
				randNumCount += 1;
			}
		}
	}
	//pooling2 to pooling2
	for (preNeuronId = 0; preNeuronId < POOLING2_N*POOLING2_LAYERS_N; preNeuronId++)
		for (postNeuronId = 0; postNeuronId < POOLING2_N*POOLING2_LAYERS_N; postNeuronId++) {
			if ((int)(preNeuronId/POOLING2_N)!=(int)(postNeuronId/POOLING2_N) && (preNeuronId%POOLING2_N)==(postNeuronId%POOLING2_N)) {
				BuildSynapse(moduleData, eventSourceID, pooling_layer2[preNeuronId], pooling_layer2[postNeuronId], inType, REAL_SYNAPSE);
			}
		}
	//pooling2 to output1
	for (postNeuronId = 0; postNeuronId < OUTPUT1_N; postNeuronId++) {
		//generate random binary number 1D array
		int64_t rand1DBinaryArray[POOLING2_N*POOLING2_LAYERS_N];
		GetRand1DBinaryArray(rand1DBinaryArray, POOLING2_N*POOLING2_LAYERS_N, TOTAL_CAM_NUM);
		randNumCount = 0;
		for (preNeuronId = 0; preNeuronId < POOLING2_N*POOLING2_LAYERS_N; preNeuronId++) {
			//randomly reset, depends on the ratio of total CAM number and POOLING2_N*POOLING2_LAYERS_N
			if (rand1DBinaryArray[randNumCount] == 1)
				BuildSynapse(moduleData, eventSourceID, pooling_layer2[preNeuronId], output_layer1[postNeuronId], exType, REAL_SYNAPSE);
			else
				BuildSynapse(moduleData, eventSourceID, pooling_layer2[preNeuronId], output_layer1[postNeuronId], exType, VIRTUAL_SYNAPSE);
			randNumCount += 1;
		}
	}
	//output1 to output2
/*	for (postNeuronId = 0; postNeuronId < OUTPUT2_N; postNeuronId++) {
		//generate random binary number 1D array
		int64_t rand1DBinaryArray[OUTPUT1_N];
		GetRand1DBinaryArray(rand1DBinaryArray, OUTPUT1_N, TOTAL_CAM_NUM);
		randNumCount = 0;
		for (preNeuronId = 0; preNeuronId < OUTPUT1_N; preNeuronId++) {
			//randomly reset, depends on the ratio of total CAM number and OUTPUT1_N
			if (rand1DBinaryArray[randNumCount] == 1)
				BuildSynapse(moduleData, eventSourceID, output_layer1[preNeuronId], output_layer2[postNeuronId], exType, REAL_SYNAPSE);
			else
				BuildSynapse(moduleData, eventSourceID, output_layer1[preNeuronId], output_layer2[postNeuronId], exType, VIRTUAL_SYNAPSE);
			randNumCount += 1;
		}
	} */

	ResetBiases(moduleData, eventSourceID);
//	EnableStimuliGen(moduleData, eventSourceID, 4);

	return (true);
}

bool ClearAllCam(caerModuleData moduleData, int16_t eventSourceID) {
	uint32_t neuronId, camId;
	for (neuronId = 0; neuronId < 32 * 32; neuronId++) {
		for (camId = 0; camId < 64; camId++) {
			WriteCam(moduleData, eventSourceID, 0, 1 << 10 | neuronId, camId, 0);
			WriteCam(moduleData, eventSourceID, 0, 2 << 10 | neuronId, camId, 0);
			WriteCam(moduleData, eventSourceID, 0, 3 << 10 | neuronId, camId, 0);
			WriteCam(moduleData, eventSourceID, 0, 4 << 10 | neuronId, camId, 0);
		}
	}
	return (true);
}

bool EnableStimuliGen(caerModuleData moduleData, int16_t eventSourceID, int32_t pattern) {
	LFilterState state = moduleData->moduleState;

	// --- start  usb handle / from spike event source id
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(eventSourceID));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(eventSourceID));
//	caerInputDynapseState stateSource = state->eventSourceModuleState;
	// --- end usb handle

	sshsNode deviceConfigNodeMain = sshsGetRelativeNode(state->eventSourceConfigNode, chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
	sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");
//	sshsNodePutShort(spikeNode, "dataSizeX", DYNAPSE_X4BOARD_NEUX);
	sshsNodePutBool(spikeNode, "running", true);
	sshsNodePutInt(spikeNode, "stim_type", pattern);
	sshsNodePutInt(spikeNode, "stim_duration", 100);
	sshsNodePutInt(spikeNode, "stim_avr", 1); //2000
	sshsNodePutBool(spikeNode, "repeat", true);
	sshsNodePutBool(spikeNode, "doStim", true);
	return (true);
}

bool DisableStimuliGen(caerModuleData moduleData, int16_t eventSourceID) {
	LFilterState state = moduleData->moduleState;

	// --- start  usb handle / from spike event source id
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(eventSourceID));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(eventSourceID));
//	caerInputDynapseState stateSource = state->eventSourceModuleState;
	// --- end usb handle

	sshsNode deviceConfigNodeMain = sshsGetRelativeNode(state->eventSourceConfigNode, chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
	sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");
//	sshsNodePutShort(spikeNode, "dataSizeX", DYNAPSE_X4BOARD_NEUX);
	sshsNodePutBool(spikeNode, "running", true);
	sshsNodePutBool(spikeNode, "doStim", false);
	return (true);
}

//build synapses when reseting
bool BuildSynapse(caerModuleData moduleData, int16_t eventSourceID, uint32_t preNeuronAddr, uint32_t postNeuronAddr, int16_t synapseType, int8_t real_virtual_tag)
{
	uint32_t sramId, camId, sram_id, cam_id;
	int chipCoreId;
	int sramFound;
	int sramAvailable;
	//for SRAM
	if (real_virtual_tag != EXTERNAL_REAL_SYNAPSE) {
		sramFound = 0;
		for(sram_id = 0; sram_id < TOTAL_SRAM_NUM; sram_id++) { //search for available SRAM
			chipCoreId = (int) (postNeuronAddr >> 8);
			if(memory.sramMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][sram_id] == 1 &&
							memory.sramMapContent->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][sram_id] == chipCoreId) { //start the searching from second SRAM, for visualization
				sramFound = 1;
			}
		}
		if (sramFound == 0) {
			sramAvailable = 0;
			for(sram_id = 0; sram_id < TOTAL_SRAM_NUM; sram_id++) { //search for available SRAM
				if (sramAvailable == 0 && memory.sramMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][(sram_id + 1) % TOTAL_SRAM_NUM] == 0) {
					sramAvailable = 1;
					sramId = (sram_id + 1) % TOTAL_SRAM_NUM; //(sram_id + 1) % TOTAL_SRAM_NUM; keep the SRAM for viewer
				}
			}
			if (sramAvailable == 1 && sramId != 0) { //sramId != 0 && sramId != 1 && sramId != 2 && sramId != 3
				WriteSram(moduleData, eventSourceID, preNeuronAddr, postNeuronAddr, sramId);
				memory.sramMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][sramId] = 1; //taken
				chipCoreId = (int) (postNeuronAddr >> 8);
				memory.sramMapContent->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][sramId] = chipCoreId; //taken
			}
		}
	}
/*	if (real_virtual_tag != EXTERNAL_REAL_SYNAPSE) {
		for(sram_id = 0; sram_id < TOTAL_SRAM_NUM; sram_id++) { //search for available SRAM
			chipCoreId = (int) preNeuronAddr >> 8;
			sramId = (sram_id + 1) % TOTAL_SRAM_NUM; //(sram_id + 1) % TOTAL_SRAM_NUM; keep the SRAM for viewer
			if(memory.sramMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][sramId] == 0 ||
					(memory.sramMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][sramId] == 1 &&
							memory.sramMapContent->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][sramId] != chipCoreId)) { //start the searching from second SRAM, for visualization
				if (sramId != 0) { //sramId != 0 && sramId != 1 && sramId != 2 && sramId != 3
					WriteSram(moduleData, eventSourceID, preNeuronAddr, postNeuronAddr, sramId);
				}
				memory.sramMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][sramId] = 1; //taken
				memory.sramMapContent->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][sramId] = chipCoreId; //taken
			}
		}
	} */
	//for CAM
	for(cam_id = 0; cam_id < TOTAL_CAM_NUM; cam_id++) //search for available CAM
		if(memory.camMap->buffer2d[postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][cam_id] == 0){
			camId = cam_id;
			break;
		}

	if (real_virtual_tag == REAL_SYNAPSE || real_virtual_tag == EXTERNAL_REAL_SYNAPSE) {
		WriteCam(moduleData, eventSourceID, preNeuronAddr, postNeuronAddr, camId, synapseType);
		memory.camMap->buffer2d[postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][camId] = (int32_t) preNeuronAddr;
		memory.camSize->buffer2d[postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][0] += 1;
	}
	//memories for the chip
	if (synapseType > 0) { //if it is EX synapse
		int32_t memoryId = memory.filterMapSize->buffer2d[postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][0];
		memory.filterMap->buffer2d[postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][memoryId] = (int32_t) preNeuronAddr;
		memory.filterCamMap->buffer2d[postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][memoryId] = (int32_t) camId;
		memory.filterMapSize->buffer2d[postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][0] += 1;
		if (real_virtual_tag != EXTERNAL_REAL_SYNAPSE) {
			memory.connectionMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] = 1; //there is an EX connection
			memory.weightMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] = 8; //initial weight
			if (real_virtual_tag == REAL_SYNAPSE)
				memory.synapseMap->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr-MEMORY_NEURON_ADDR_OFFSET] = synapseType;
		}
	}
	return (true);
}

//write neuron CAM when a synapse is built or modified
bool WriteCam(caerModuleData moduleData, int16_t eventSourceID, uint32_t preNeuronAddr, uint32_t postNeuronAddr, uint32_t camId, int16_t synapseType)
{
//	caerDeviceHandle usb_handle = ((caerInputDynapseState) moduleData->moduleState)->deviceState;
	LFilterState state = moduleData->moduleState;

	// --- start  usb handle / from spike event source id
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(eventSourceID));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(eventSourceID));
	caerInputDynapseState stateSource = state->eventSourceModuleState;
	// --- end usb handle

//	sshsNode deviceConfigNodeMain = sshsGetRelativeNode(moduleData->moduleNode, chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
//	sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");
//	sshsNodePutShort(spikeNode, "dataSizeX", DYNAPSE_X4BOARD_NEUX);

	uint32_t chipId_t, chipId, bits;
	chipId_t = postNeuronAddr >> NEURON_CHIPID_SHIFT;
	if (chipId_t == 1)
		chipId = DYNAPSE_CONFIG_DYNAPSE_U0;
	else if (chipId_t == 2)
		chipId = DYNAPSE_CONFIG_DYNAPSE_U1;
	else if (chipId_t == 3)
		chipId = DYNAPSE_CONFIG_DYNAPSE_U2;
	else if (chipId_t == 4)
		chipId = DYNAPSE_CONFIG_DYNAPSE_U3;
	uint32_t ei, fs;
    uint32_t address = preNeuronAddr & NEURON_ADDRESS_BITS;
    uint32_t source_core = (preNeuronAddr & NEURON_COREID_BITS) >> NEURON_COREID_SHIFT;
    if (synapseType > 0) //if it is EX synapse
    	ei = EXCITATORY_SYNAPSE;
    else
    	ei = INHIBITORY_SYNAPSE;
    if (abs(synapseType) == FAST_SYNAPSE_ID)
    	fs = FAST_SYNAPSE;
    else if (abs(synapseType) == SLOW_SYNAPSE_ID)
    	fs = SLOW_SYNAPSE;
    else if (abs(synapseType) == NO_SYNAPSE_ID) {
        address = NO_SYNAPSE_ADDRESS;
        source_core = NO_SYNAPSE_CORE;
    }
    uint32_t coreId = (postNeuronAddr & NEURON_COREID_BITS) >> NEURON_COREID_SHIFT;
    uint32_t neuron_row = (postNeuronAddr & NEURON_ROW_BITS) >> NEURON_ROW_SHIFT;
    uint32_t synapse_row = camId;
    uint32_t row = neuron_row << CAM_NEURON_ROW_SHIFT | synapse_row;
	uint32_t column = postNeuronAddr & NEURON_COL_BITS;
    bits = ei << CXQ_CAM_EI_SHIFT |
    		fs << CXQ_CAM_FS_SHIFT |
    		address << CXQ_ADDR_SHIFT |
    		source_core << CXQ_SOURCE_CORE_SHIFT |
    		CXQ_PROGRAM |
    		coreId << CXQ_PROGRAM_COREID_SHIFT |
    		row << CXQ_PROGRAM_ROW_SHIFT |
    		column << CXQ_PROGRAM_COLUMN_SHIFT;
	caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chipId);
	caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, bits); //this is the 30 bits
	return (true);
}

//write neuron SRAM when a synapse is built or modified
bool WriteSram(caerModuleData moduleData, int16_t eventSourceID, uint32_t preNeuronAddr, uint32_t postNeuronAddr, uint32_t sramId)
{
//	caerDeviceHandle usb_handle = ((caerInputDynapseState) moduleData->moduleState)->deviceState;
	LFilterState state = moduleData->moduleState;

	// --- start  usb handle / from spike event source id
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(eventSourceID));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(eventSourceID));
	caerInputDynapseState stateSource = state->eventSourceModuleState;
	// --- end usb handle

	uint32_t chipId, bits;
	chipId = preNeuronAddr >> NEURON_CHIPID_SHIFT;
	if (chipId == 1)
		chipId = DYNAPSE_CONFIG_DYNAPSE_U0;
	else if (chipId == 2)
		chipId = DYNAPSE_CONFIG_DYNAPSE_U1;
	else if (chipId == 3)
		chipId = DYNAPSE_CONFIG_DYNAPSE_U2;
	else if (chipId == 4)
		chipId = DYNAPSE_CONFIG_DYNAPSE_U3;
	uint32_t virtual_coreId = (preNeuronAddr & NEURON_COREID_BITS) >> NEURON_COREID_SHIFT;
	uint32_t source_chipId = (preNeuronAddr >> NEURON_CHIPID_SHIFT) - 1; //for calculation
	uint32_t destination_chipId = (postNeuronAddr >> NEURON_CHIPID_SHIFT) - 1; //for calculation
	uint32_t sy, dy, sx, dx;
    if ((source_chipId / BOARD_HEIGHT) >= (destination_chipId / BOARD_HEIGHT))
        sy = EVENT_DIRECTION_Y_DOWN; //EVENT_DIRECTION_Y_UP;
    else
        sy = EVENT_DIRECTION_Y_DOWN;
    if ((source_chipId % BOARD_WIDTH) <= (destination_chipId % BOARD_WIDTH))
        sx = EVENT_DIRECTION_X_RIGHT; //EVENT_DIRECTION_X_RIGHT;
    else
        sx = EVENT_DIRECTION_X_LEFT; //EVENT_DIRECTION_X_LEFT;
    if (source_chipId == destination_chipId)
    	dy = 0;
    else
    	dy = 1; //(uint32_t) abs((int32_t)(source_chipId / BOARD_HEIGHT) - (int32_t)(destination_chipId / BOARD_HEIGHT));
    dx = 0; //(uint32_t) abs((int32_t)(source_chipId % BOARD_WIDTH) - (int32_t)(destination_chipId % BOARD_WIDTH));
    uint32_t dest_coreId = (uint32_t) (1 << ((postNeuronAddr & NEURON_COREID_BITS) >> NEURON_COREID_SHIFT));
    uint32_t coreId = (preNeuronAddr & NEURON_COREID_BITS) >> NEURON_COREID_SHIFT;
    uint32_t neuron_row = (preNeuronAddr & NEURON_ROW_BITS) >> NEURON_ROW_SHIFT;
    uint32_t neuron_column = preNeuronAddr & NEURON_COL_BITS;
    uint32_t synapse_row = sramId;
    uint32_t row = neuron_row << SRAM_NEURON_ROW_SHIFT | neuron_column << SRAM_NEURON_COL_SHIFT | synapse_row; //synapse_row 0 1 cleared why?
    uint32_t column = SRAM_COL_VALUE;
    bits = virtual_coreId << CXQ_SRAM_VIRTUAL_SOURCE_CORE_SHIFT |
    		sy << CXQ_SRAM_SY_SHIFT |
    		dy << CXQ_SRAM_DY_SHIFT |
    		sx << CXQ_SRAM_SX_SHIFT |
    		dx << CXQ_SRAM_DX_SHIFT |
    		dest_coreId << CXQ_SRAM_DEST_CORE_SHIFT |
    		CXQ_PROGRAM |
    		coreId << CXQ_PROGRAM_COREID_SHIFT |
    		row << CXQ_PROGRAM_ROW_SHIFT |
    		column << CXQ_PROGRAM_COLUMN_SHIFT;
	caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chipId);
	caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, bits); //this is the 30 bits
	return(true);
}

void Shuffle1DArray(int64_t *array, int64_t Range) {
	if (Range > 1) {
		int64_t i;
		for (i = 0; i < Range; i++) {
			int64_t j = i + rand() / (RAND_MAX / (Range - i) + 1);
			int64_t t = array[j];
			array[j] = array[i];
			array[i] = t;
		}
	}
}
void GetRand1DArray(int64_t *array, int64_t Range, int64_t CamNumAvailable) {
	int64_t temp[Range]; //sizeof(array) doesn't work
	int64_t i;
	for (i = 0; i < Range; i++) {
		temp[i] = i;
	}
	Shuffle1DArray(temp, Range);
	for (i = 0; i < CamNumAvailable; i++) {
		array[i] = temp[i];
	}
}
void GetRand1DBinaryArray(int64_t *binaryArray, int64_t Range, int64_t CamNumAvailable) {
	int64_t array[CamNumAvailable];
	GetRand1DArray(array, Range, CamNumAvailable);
	int64_t i;
	int64_t num;
	for (i = 0; i < Range; i++) {
		binaryArray[i] = 0;
	}
	for (i = 0; i < CamNumAvailable; i++) {
		num = array[i];
		binaryArray[num] = 1;
	}
}

COLOUR GetColourW(double v, double vmin, double vmax)
{
   COLOUR c = {0,0,0}; //{65535, 65535, 65535}; // white
   double dv;
   double value;

   if (v < vmin)
      v = vmin;
   if (v > vmax)
      v = vmax;
   dv = vmax - vmin;

   if (v < (vmin + dv / 4)) {
      c.r = 0;
      value = ( 4 * (v - vmin) / dv ) * 65535;
      if (value > 30000)
    	  c.g = 30000;
      else if (value < 0)
    	  c.g = 0;
      else
    	  c.g = (uint16_t) value;
   } else if (v < (vmin + dv / 2)) {
      c.r = 0;
      value = (1 + 4 * (vmin + dv / 4 - v) / dv) * 65535;
      if (value > 30000)
    	  c.b = 30000;
      else if (value < 0)
    	  c.b = 0;
      else
    	  c.b = (uint16_t) value;
   } else if (v < (vmin + dv * 3 / 4)) {
      c.b = 0;
      value = (4 * (v - vmin - dv / 2) / dv) * 65535;
      if (value > 30000)
    	  c.r = 30000;
      else if (value < 0)
    	  c.r = 0;
      else
    	  c.r = (uint16_t) value;
   } else {
      c.b = 0;
      value = (1 + 4 * (vmin + dv * 3 / 4 - v) / dv) * 65535;
      if (value > 30000)
    	  c.g = 30000;
      else if (value < 0)
    	  c.g = 0;
      else
    	  c.g = (uint16_t) value;
   }

   return(c);
}

COLOUR GetColourS(double v, double vmin, double vmax)
{
   COLOUR c = {0,0,0}; //{65535, 65535, 65535}; // white
   double dv;
   double value;

   if (v < vmin)
      v = vmin;
   if (v > vmax)
      v = vmax;
   dv = vmax - vmin;

   if (v < (vmin + dv / 4)) {
      c.r = 0;
      value = ( 4 * (v - vmin) / dv ) * 65535;
      if (value > 30000)
    	  c.g = 30000;
      else if (value < 0)
    	  c.g = 0;
      else
    	  c.g = (uint16_t) value;
   } else if (v < (vmin + dv / 2)) {
      c.r = 0;
      value = (1 + 4 * (vmin + dv / 4 - v) / dv) * 65535;
      if (value > 30000)
    	  c.b = 30000;
      else if (value < 0)
    	  c.b = 0;
      else
    	  c.b = (uint16_t) value;
   } else if (v < (vmin + dv * 3 / 4)) {
      c.b = 0;
      value = (4 * (v - vmin - dv / 2) / dv) * 65535;
      if (value > 30000)
    	  c.r = 30000;
      else if (value < 0)
    	  c.r = 0;
      else
    	  c.r = (uint16_t) value;
   } else {
      c.b = 0;
      value = (1 + 4 * (vmin + dv * 3 / 4 - v) / dv) * 65535;
      if (value > 30000)
    	  c.g = 30000;
      else if (value < 0)
    	  c.g = 0;
      else
    	  c.g = (uint16_t) value;
   }

   return(c);
}

bool ResetBiases(caerModuleData moduleData, int16_t eventSourceID) {
	LFilterState state = moduleData->moduleState;

	// --- start  usb handle / from spike event source id
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(eventSourceID));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(eventSourceID));
	caerInputDynapseState stateSource = state->eventSourceModuleState;
	// --- end usb handle

	uint32_t chipId_t, chipId, coreId;
	uint32_t bits;

	for (chipId_t = 0; chipId_t < 4; chipId_t++) { //1 4

		if (chipId_t == 0)
			chipId = DYNAPSE_CONFIG_DYNAPSE_U0;
		else if (chipId_t == 1)
			chipId = DYNAPSE_CONFIG_DYNAPSE_U1;
		else if (chipId_t == 2)
			chipId = DYNAPSE_CONFIG_DYNAPSE_U2;
		else if (chipId_t == 3)
			chipId = DYNAPSE_CONFIG_DYNAPSE_U3;

		caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chipId);

		for (coreId = 0; coreId < 4; coreId++) {
			//sweep all the biases
		    // select right bias name
			//caer-ctrl
			//put /1/1-DYNAPSEFX2/DYNAPSE_CONFIG_DYNAPSE_U1/bias/C0_IF_DC_P/ coarseValue byte 6
			if (chipId == DYNAPSE_CONFIG_DYNAPSE_U0) {
				if (coreId == 0) {
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_AHTHR_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_AHW_P", 7, 0, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_BUF_P", 3, 80, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_CASC_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_DC_P", 7, 0, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_NMDA_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_RFR_N", 5, 255, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_TAU1_N", 4, 200, "LowBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_TAU2_N", 6, 15, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_THR_N", 2, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_TAU_F_P", 6, 200, "HighBias", "PBias"); //105
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_TAU_S_P", 7, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_THR_F_P", 0, 220, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_THR_S_P", 7, 0, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_TAU_F_P", 7, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_TAU_S_P", 7, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_THR_F_P", 7, 40, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_THR_S_P", 7, 40, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_EXC_F_N", 0, 76, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_EXC_S_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_INH_F_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "R2R_P", 4, 85, "HighBias", "PBias");
				} else {
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_AHTHR_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_AHW_P", 7, 0, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_BUF_P", 3, 80, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_CASC_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_DC_P", 7, 0, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_NMDA_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_RFR_N", 5, 255, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_TAU1_N", 4, 200, "LowBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_TAU2_N", 6, 15, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_THR_N", 2, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_TAU_F_P", 6, 105, "HighBias", "PBias"); //105
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_TAU_S_P", 7, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_THR_F_P", 0, 220, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_THR_S_P", 7, 0, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_TAU_F_P", 7, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_TAU_S_P", 7, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_THR_F_P", 7, 40, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_THR_S_P", 7, 40, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_EXC_F_N", 0, 76, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_EXC_S_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_INH_F_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "R2R_P", 4, 85, "HighBias", "PBias");
				}
			}
			else if (chipId == DYNAPSE_CONFIG_DYNAPSE_U2) { // DYNAPSE_CONFIG_DYNAPSE_U2 = 4
				if (coreId == 0) {
/*					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_AHTHR_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_AHW_P", 7, 0, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_BUF_P", 3, 80, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_CASC_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_DC_P", 7, 0, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_NMDA_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_RFR_N", 6, 100, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_TAU1_N", 4, 200, "LowBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_TAU2_N", 6, 15, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_THR_N", 2, 213, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_TAU_F_P", 6, 50, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_TAU_S_P", 7, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_THR_F_P", 0, 250, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_THR_S_P", 7, 0, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_TAU_F_P", 7, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_TAU_S_P", 7, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_THR_F_P", 7, 40, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_THR_S_P", 7, 40, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_EXC_F_N", 0, 250, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_EXC_S_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_INH_F_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "R2R_P", 4, 85, "HighBias", "PBias"); */
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_AHTHR_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_AHW_P", 7, 0, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_BUF_P", 3, 80, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_CASC_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_DC_P", 7, 0, "HighBias", "PBias"); //4, 170
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_NMDA_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_RFR_N", 5, 255, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_TAU1_N", 4, 200, "LowBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_TAU2_N", 6, 15, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_THR_N", 2, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_TAU_F_P", 6, 105, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_TAU_S_P", 7, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_THR_F_P", 0, 76, "HighBias", "PBias"); //0, 220
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_THR_S_P", 7, 0, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_TAU_F_P", 6, 105, "HighBias", "PBias"); //7, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_TAU_S_P", 7, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_THR_F_P", 0, 220, "HighBias", "PBias"); //7, 40, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_THR_S_P", 7, 40, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_EXC_F_N", 0, 250, "HighBias", "NBias"); //0, 76
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_EXC_S_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_INH_F_N", 3, 250, "HighBias", "NBias"); //7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "R2R_P", 4, 85, "HighBias", "PBias");
				} else if (coreId == 1) {
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_AHTHR_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_AHW_P", 7, 0, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_BUF_P", 3, 80, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_CASC_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_DC_P", 7, 0, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_NMDA_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_RFR_N", 5, 255, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_TAU1_N", 4, 200, "LowBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_TAU2_N", 6, 15, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_THR_N", 4, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_TAU_F_P", 6, 200, "HighBias", "PBias"); //105
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_TAU_S_P", 7, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_THR_F_P", 0, 220, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_THR_S_P", 7, 0, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_TAU_F_P", 6, 105, "HighBias", "PBias"); //7, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_TAU_S_P", 7, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_THR_F_P", 5, 220, "HighBias", "PBias"); //7, 40, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_THR_S_P", 7, 40, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_EXC_F_N", 0, 76, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_EXC_S_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_INH_F_N", 7, 76, "HighBias", "NBias"); //7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "R2R_P", 4, 85, "HighBias", "PBias");
				} else if (coreId == 2) {
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_AHTHR_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_AHW_P", 7, 0, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_BUF_P", 3, 80, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_CASC_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_DC_P", 7, 0, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_NMDA_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_RFR_N", 5, 255, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_TAU1_N", 4, 200, "LowBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_TAU2_N", 6, 15, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_THR_N", 4, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_TAU_F_P", 6, 200, "HighBias", "PBias"); //105
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_TAU_S_P", 7, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_THR_F_P", 0, 220, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_THR_S_P", 7, 0, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_TAU_F_P", 4, 105, "HighBias", "PBias"); //7, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_TAU_S_P", 7, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_THR_F_P", 4, 220, "HighBias", "PBias"); //7, 40, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_THR_S_P", 7, 40, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_EXC_F_N", 0, 76, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_EXC_S_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_INH_F_N", 7, 76, "HighBias", "NBias"); //7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "R2R_P", 4, 85, "HighBias", "PBias");
				} else if (coreId == 3) {
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_AHTHR_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_AHW_P", 7, 0, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_BUF_P", 3, 80, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_CASC_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_DC_P", 7, 0, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_NMDA_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_RFR_N", 5, 255, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_TAU1_N", 4, 200, "LowBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_TAU2_N", 6, 15, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_THR_N", 4, 20, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_TAU_F_P", 6, 105, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_TAU_S_P", 7, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_THR_F_P", 0, 220, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_THR_S_P", 7, 0, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_TAU_F_P", 6, 105, "HighBias", "PBias"); //7, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_TAU_S_P", 7, 40, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_THR_F_P", 5, 220, "HighBias", "PBias"); //7, 40, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_THR_S_P", 7, 40, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_EXC_F_N", 0, 76, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_EXC_S_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_INH_F_N", 0, 76, "HighBias", "NBias"); //7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "NBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
					setBiasBits(moduleData, eventSourceID, chipId, coreId, "R2R_P", 4, 85, "HighBias", "PBias");
				}
			}
			else {
				setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
				setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_AHTHR_N", 7, 0, "HighBias", "NBias");
				setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_AHW_P", 7, 0, "HighBias", "PBias");
				setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_BUF_P", 3, 80, "HighBias", "PBias");
				setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_CASC_N", 7, 0, "HighBias", "NBias");
				setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_DC_P", 7, 0, "HighBias", "PBias");
				setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_NMDA_N", 7, 0, "HighBias", "NBias");
				setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_RFR_N", 5, 255, "HighBias", "NBias");
				setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_TAU1_N", 4, 200, "LowBias", "NBias");
				setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_TAU2_N", 6, 15, "HighBias", "NBias");
				setBiasBits(moduleData, eventSourceID, chipId, coreId, "IF_THR_N", 4, 40, "HighBias", "NBias");
				setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_TAU_F_P", 6, 105, "HighBias", "PBias");
				setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_TAU_S_P", 7, 40, "HighBias", "NBias");
				setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_THR_F_P", 0, 220, "HighBias", "PBias");
				setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPIE_THR_S_P", 7, 0, "HighBias", "PBias");
				setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_TAU_F_P", 6, 105, "HighBias", "PBias"); //7, 40, "HighBias", "NBias");
				setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_TAU_S_P", 7, 40, "HighBias", "NBias");
				setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_THR_F_P", 0, 220, "HighBias", "PBias"); //7, 40, "HighBias", "PBias");
				setBiasBits(moduleData, eventSourceID, chipId, coreId, "NPDPII_THR_S_P", 7, 40, "HighBias", "PBias");
				setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_EXC_F_N", 0, 76, "HighBias", "NBias");
				setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_EXC_S_N", 7, 0, "HighBias", "NBias");
				setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_INH_F_N", 0, 76, "HighBias", "NBias"); //7, 0, "HighBias", "NBias");
				setBiasBits(moduleData, eventSourceID, chipId, coreId, "PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "NBias");
				setBiasBits(moduleData, eventSourceID, chipId, coreId, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
				setBiasBits(moduleData, eventSourceID, chipId, coreId, "R2R_P", 4, 85, "HighBias", "PBias");
			}

		}
	}
	return (true);
}

void setBiasBits(caerModuleData moduleData, int16_t eventSourceID, uint32_t chipId, uint32_t coreId, const char *biasName_t,
		uint8_t coarseValue, uint16_t fineValue, const char *lowHigh, const char *npBias) {
	LFilterState state = moduleData->moduleState;

	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(eventSourceID));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(eventSourceID));
	caerInputDynapseState stateSource = state->eventSourceModuleState;

	struct caer_dynapse_info dynapse_info = caerDynapseInfoGet(stateSource->deviceState);

    size_t biasNameLength = strlen(biasName_t);
    char biasName[biasNameLength+3];

	biasName[0] = 'C';
	if (coreId == 0)
		biasName[1] = '0';
	else if (coreId == 1)
		biasName[1] = '1';
	else if (coreId == 2)
		biasName[1] = '2';
	else if (coreId == 3)
		biasName[1] = '3';
	biasName[2] = '_';

	uint32_t i;
	for(i = 0; i < biasNameLength + 3; i++) {
		biasName[3+i] = biasName_t[i];
	}
//	uint32_t bits = 0;
	uint32_t bits = generatesBitsCoarseFineBiasSetting(state->eventSourceConfigNode, &dynapse_info,
			biasName, coarseValue, fineValue, lowHigh, "Normal", npBias, true, chipId);

	caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, bits);
//	return (bits);
}

void SetTimer() {
  struct itimerval itv;
  itv.it_interval.tv_sec = 1;
  itv.it_interval.tv_usec = 0;
  itv.it_value.tv_sec = 1;
  itv.it_value.tv_usec = 0;
  setitimer(ITIMER_REAL, &itv, &oldtv);
}

void SignalHandler(int m) {
	time_count = (time_count + 1) % 4294967295;
//	printf("%d\n", time_count);
//	printf("%d\n", m);
}
