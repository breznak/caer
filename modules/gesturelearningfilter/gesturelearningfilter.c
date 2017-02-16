/*
 * Created on: Dec, 2016
 * Author: dongchen@ini.uzh.ch
 */

#include "gesturelearningfilter.h"
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

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "settings.h"

struct GFilter_state {
	caerInputDynapseState eventSourceModuleState;
	sshsNode eventSourceConfigNode;
	int eventSourceID;
	int fileInputSourceID;
	int32_t colorscaleMax;
	int32_t colorscaleMin;
	int8_t reset;
	float resetExProbability;
	int8_t resetExType;
	int8_t resetInType;
	double learningRateForward;
	double learningRateBackward;
	int16_t dataSizeX;
	int16_t dataSizeY;
	int16_t visualizerSizeX;
	int16_t visualizerSizeY;
	int16_t apsSizeX;
	int16_t apsSizeY;
	bool stimulate;
	bool learning;
	int32_t maxSynapseFeature;
	int32_t maxSynapseOutput;
};

struct GFilter_memory {
	simple2DBufferInt connectionMap; //store all the connections, size: TOTAL_NEURON_NUM_ON_CHIP by TOTAL_NEURON_NUM_ON_CHIP
	simple2DBufferInt connectionCamMap;	//store the CAM id for each pre-post neurons pair
	simple2DBufferInt camMap;			//the CAMs are available or not
	simple2DBufferInt camSize;			//available CAM
	simple2DBufferInt sramMap;			//the SRAMs are available or not
	simple2DBufferInt sramMapContent;	//the chipId + coreId information for each SRAM
	simple2DBufferDouble weightMap;		//store all the weights
	simple2DBufferInt synapseMap;		//store all the synapses
	simple2DBufferLong spikeFifo;		//FIFO for storing all the events
	simple2DBufferInt filterMap;		//store the pre-neuron address for each filter
	simple2DBufferInt camMapContentSource;	//store all the CAM content for each filter
	simple2DBufferInt camMapContentType;	//store all the synapse type for each filter
	simple2DBufferInt filterMapSize;	//size for every filter
	simple2DBufferInt outputMap;
	simple2DBufferInt outputMapDisabled;
	uint64_t spikeCounter;				//number of spikes in the FIFO
	uint64_t preRdPointer;				//pre-read pointer for the FIFO
	uint64_t wrPointer;					//write pointer for the FIFO
};

double deltaWeights[DELTA_WEIGHT_LUT_LENGTH];
double synapseUpgradeThreshold[SYNAPSE_UPGRADE_THRESHOLD_LUT_LENGTH];

typedef struct {
	uint16_t r, g, b;
} COLOUR;

typedef struct GFilter_state *GFilterState;
typedef struct GFilter_memory GFilterMemory;

static GFilterMemory memory;
static int8_t reseted = 0;
static int8_t teachingSignalDisabled = 0;
static int time_count = 0;
static int time_count_last = 0;
static int32_t stimuliPattern = 0;
static struct itimerval oldtv;
static int stimdisabled = 0;
static int pattern_number = 3; //3 or 4

static int usb_packet_maximum_size = 1024; //1366 1365 1360 1350 1300 1200 1100 1000 100 85 OK; 8000 5000 3000 2000 1500 1400 1375 1370 1367
// case multi chip mapping
static int bits_chipU0[DYNAPSE_MAX_USER_USB_PACKET_SIZE];
static int bits_chipU1[DYNAPSE_MAX_USER_USB_PACKET_SIZE];
static int bits_chipU2[DYNAPSE_MAX_USER_USB_PACKET_SIZE];
static int bits_chipU3[DYNAPSE_MAX_USER_USB_PACKET_SIZE];
static int numConfig_chipU0 = 0;
static int numConfig_chipU1 = 0;
static int numConfig_chipU2 = 0;
static int numConfig_chipU3 = 0;

static bool caerGestureLearningFilterInit(caerModuleData moduleData); //It may not run at the beginning of the experiment ????????????
static void caerGestureLearningFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerGestureLearningFilterConfig(caerModuleData moduleData);
static void caerGestureLearningFilterExit(caerModuleData moduleData);
static void caerGestureLearningFilterReset(caerModuleData moduleData, int resetCallSourceID);

static void ModifyForwardSynapseG(caerModuleData moduleData, int64_t preNeuronAddr, int64_t postNeuronAddr,
	double deltaWeight, caerFrameEventPacket *synapseplotfeature, caerFrameEventPacket *weightplotfeature);
static void ModifyBackwardSynapseG(caerModuleData moduleData, int64_t preNeuronAddr, int64_t postNeuronAddr,
	double deltaWeight, caerFrameEventPacket *weightplotfeature);
static bool ResetNetworkG(caerModuleData moduleData);
static bool BuildSynapseG(caerModuleData moduleData, uint32_t preNeuronAddr, uint32_t postNeuronAddr,
	uint32_t virtualNeuronAddr, int16_t synapseType, int8_t realOrVirtualSynapse, int8_t virtualNeuronAddrEnable);
static bool WriteCamG(caerModuleData moduleData, uint32_t preNeuronAddr, uint32_t postNeuronAddr,
	uint32_t virtualNeuronAddr, uint32_t camId, int16_t synapseType, int8_t virtualNeuronAddrEnable, int8_t stdp);
static int GetWriteCamBitsG(caerModuleData moduleData, uint32_t preNeuronAddr, uint32_t postNeuronAddr,
	uint32_t virtualNeuronAddr, uint32_t camId, int16_t synapseType, int8_t virtualNeuronAddrEnable);
static int GetWriteCamChipIdG(uint32_t postNeuronAddr);
static bool WriteSramG(caerModuleData moduleData, uint32_t preNeuronAddr, uint32_t postNeuronAddr,
	uint32_t virtualNeuronAddr, uint32_t sramId, int8_t virtualNeuronAddrEnable, int8_t stdp);
static void Shuffle1DArrayG(int64_t *array, int64_t Range);
static void GetRand1DArrayG(int64_t *array, int64_t Range, int64_t CamNumAvailable);
static void GetRand1DBinaryArrayG(int64_t *binaryArray, int64_t Range, int64_t CamNumAvailable);
static bool ResetBiasesG(caerModuleData moduleData);
static bool EnableStimuliGenG(caerModuleData moduleData, int32_t pattern);
static bool DisableStimuliGenG(caerModuleData moduleData);
static bool EnableStimuliGenPrimitiveCamG(caerModuleData moduleData);
static bool DisableStimuliGenPrimitiveCamG(caerModuleData moduleData);
static bool EnableTeachingSignalG(caerModuleData moduleData);
static bool DisableTeachingSignalG(caerModuleData moduleData);
static bool EnableTeachingG(caerModuleData moduleData);
static bool DisableTeachingG(caerModuleData moduleData);
static bool SetInputLayerCamG(caerModuleData moduleData);
static bool ClearAllCamG(caerModuleData moduleData);
static void setBiasBitsG(caerModuleData moduleData, uint32_t chipId, uint32_t coreId, const char *biasName_t,
	uint8_t coarseValue, uint16_t fineValue, const char *lowHigh, const char *npBias);
static void SetTimerG(void);
static void SignalHandlerG(int m);
static bool ConfigureChipG(caerModuleData moduleData, int8_t chipId);

COLOUR GetColourWG(double v, double vmin, double vmax);
COLOUR GetColourSG(int v);

static struct caer_module_functions caerGestureLearningFilterFunctions = { .moduleInit = &caerGestureLearningFilterInit,
	.moduleRun = &caerGestureLearningFilterRun, .moduleConfig = &caerGestureLearningFilterConfig, .moduleExit =
		&caerGestureLearningFilterExit, .moduleReset = &caerGestureLearningFilterReset };

void caerGestureLearningFilter(uint16_t moduleID, int fileInputID, caerSpikeEventPacket spike,
	caerFrameEventPacket *weightplotfeature, caerFrameEventPacket *synapseplotfeature) { //used now
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "LFilter", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}
	caerModuleSM(&caerGestureLearningFilterFunctions, moduleData, sizeof(struct GFilter_state), 4, fileInputID, spike,
		weightplotfeature, synapseplotfeature);
}

static bool caerGestureLearningFilterInit(caerModuleData moduleData) {
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "colorscaleMax", VMAX); //500
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "colorscaleMin", VMIN);
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "reset", 0);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "resetExProbability", 1);
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "resetExType", 1); //1 //2 for test, should be 1
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "resetInType", 2); //1
	sshsNodePutDoubleIfAbsent(moduleData->moduleNode, "learningRateForward", 5); //1
	sshsNodePutDoubleIfAbsent(moduleData->moduleNode, "learningRateBackward", 2); //5 //2); //1
	sshsNodePutShortIfAbsent(moduleData->moduleNode, "dataSizeX", VISUALIZER_HEIGHT_FEATURE); //640
	sshsNodePutShortIfAbsent(moduleData->moduleNode, "dataSizeY", VISUALIZER_WIDTH_FEATURE); //480
	sshsNodePutShortIfAbsent(moduleData->moduleNode, "visualizerSizeX", VISUALIZER_HEIGHT_FEATURE); //640
	sshsNodePutShortIfAbsent(moduleData->moduleNode, "visualizerSizeY", VISUALIZER_WIDTH_FEATURE); //480
	sshsNodePutShortIfAbsent(moduleData->moduleNode, "apsSizeX", VISUALIZER_HEIGHT_FEATURE); //640
	sshsNodePutShortIfAbsent(moduleData->moduleNode, "apsSizeY", VISUALIZER_WIDTH_FEATURE); //480
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "stimulate", true); //false
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "learning", true); //true); //false
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "maxSynapseFeature", 5); //3); //128); //500
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "maxSynapseOutput", 5); //5 //128); //500

	GFilterState state = moduleData->moduleState;
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
	state->learning = sshsNodeGetBool(moduleData->moduleNode, "learning");
	state->maxSynapseFeature = sshsNodeGetInt(moduleData->moduleNode, "maxSynapseFeature");
	state->maxSynapseOutput = sshsNodeGetInt(moduleData->moduleNode, "maxSynapseOutput");

	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	state->eventSourceID = NULL;

	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	if (!sshsNodeAttributeExists(sourceInfoNode, "apsSizeX", SSHS_SHORT)) {
		sshsNodePutShort(sourceInfoNode, "apsSizeX", VISUALIZER_HEIGHT_FEATURE); //DYNAPSE_X4BOARD_NEUY
		sshsNodePutShort(sourceInfoNode, "apsSizeY", VISUALIZER_WIDTH_FEATURE); //DYNAPSE_X4BOARD_NEUY
	}

	state->eventSourceID = NULL;
	state->fileInputSourceID = NULL;

	return (true); // Nothing that can fail here.
}

static void caerGestureLearningFilterConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);
	GFilterState state = moduleData->moduleState;
	state->colorscaleMax = sshsNodeGetInt(moduleData->moduleNode, "colorscaleMax");
	state->colorscaleMin = sshsNodeGetInt(moduleData->moduleNode, "colorscaleMin");
	state->reset = sshsNodeGetByte(moduleData->moduleNode, "reset");
	state->resetExProbability = sshsNodeGetFloat(moduleData->moduleNode, "resetExProbability");
	state->resetExType = sshsNodeGetByte(moduleData->moduleNode, "resetExType");
	state->resetInType = sshsNodeGetByte(moduleData->moduleNode, "resetInType");
	state->learningRateForward = sshsNodeGetDouble(moduleData->moduleNode, "learningRateForward");
	state->learningRateBackward = sshsNodeGetDouble(moduleData->moduleNode, "learningRateBackward");
	state->dataSizeX = sshsNodeGetShort(moduleData->moduleNode, "dataSizeX");
	state->dataSizeY = sshsNodeGetShort(moduleData->moduleNode, "dataSizeY");
	state->visualizerSizeX = sshsNodeGetShort(moduleData->moduleNode, "visualizerSizeX");
	state->visualizerSizeY = sshsNodeGetShort(moduleData->moduleNode, "visualizerSizeY");
	state->visualizerSizeX = sshsNodeGetShort(moduleData->moduleNode, "apsSizeX");
	state->visualizerSizeY = sshsNodeGetShort(moduleData->moduleNode, "apsSizeY");
	state->stimulate = sshsNodeGetBool(moduleData->moduleNode, "stimulate");
	state->learning = sshsNodeGetBool(moduleData->moduleNode, "learning");
	state->maxSynapseFeature = sshsNodeGetInt(moduleData->moduleNode, "maxSynapseFeature");
	state->maxSynapseOutput = sshsNodeGetInt(moduleData->moduleNode, "maxSynapseOutput");
}

static void caerGestureLearningFilterExit(caerModuleData moduleData) { // Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	GFilterState state = moduleData->moduleState;

	// --- start USB handle / from spike event source id
	state->eventSourceModuleState = caerMainloopGetSourceState(state->eventSourceID);
	state->eventSourceConfigNode = caerMainloopGetSourceNode(state->eventSourceID);
	// --- end USB handle

	sshsNode deviceConfigNodeMain = sshsGetRelativeNode(state->eventSourceConfigNode,
		chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
	sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");
	sshsNodePutBool(spikeNode, "running", true);
	sshsNodePutBool(spikeNode, "repeat", false);
	sshsNodePutBool(spikeNode, "doStim", false);

}

static void caerGestureLearningFilterReset(caerModuleData moduleData, int resetCallSourceID) {
	UNUSED_ARGUMENT(moduleData);
	UNUSED_ARGUMENT(resetCallSourceID);
//	ResetNetworkG(moduleData);
}

static void caerGestureLearningFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	int fileInputId = va_arg(args, int);
	caerSpikeEventPacket spike = va_arg(args, caerSpikeEventPacket);
	caerFrameEventPacket *weightplotfeature = va_arg(args, caerFrameEventPacket*);
	caerFrameEventPacket *synapseplotfeature = va_arg(args, caerFrameEventPacket*);

	GFilterState state = moduleData->moduleState;

	if(spike == NULL){
		return;
	}
	// init
	if (state->eventSourceID == NULL) {
		state->eventSourceID = caerEventPacketHeaderGetEventSource(&spike->packetHeader); // into state so that all functions can use it after init.
	}
	//if(state->fileInputSourceID == NULL){
	state->fileInputSourceID = fileInputId;
	//}

	uint32_t counterS;
	COLOUR colS;
	uint16_t sizeX = VISUALIZER_HEIGHT_FEATURE;
	uint16_t sizeY = VISUALIZER_WIDTH_FEATURE;
	if (memory.synapseMap == NULL) {
		int64_t i, j, ys, row_id, col_id, feature_id;
		//initialize lookup tables
		for (i = 0; i < DELTA_WEIGHT_LUT_LENGTH; i++) {
			deltaWeights[i] = exp((double) i / 1000);
		}
		for (i = 0; i < SYNAPSE_UPGRADE_THRESHOLD_LUT_LENGTH; i++) {
			synapseUpgradeThreshold[i] = exp((double) i / 1000); //i //i/2 i/100 i/10 too small //i 0 //1; //exp( (double) i/1000);
		}
		if (!ResetNetworkG(moduleData)) { // Failed to allocate memory, nothing to do.
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for synapseMap.");
			return;
		}
		int warrayS[sizeX][sizeY];
		for (i = 0; i < sizeX; i++)
			for (j = 0; j < sizeY; j++)
				warrayS[i][j] = 0;
		*synapseplotfeature = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, sizeX, sizeY, 3); //put info into frame
		if (*synapseplotfeature != NULL) {
			caerFrameEvent singleplotS = caerFrameEventPacketGetEvent(*synapseplotfeature, 0);
			//for feature maps
			for (i = 0; i < INPUT_N; i++) {
				for (j = 0; j < FEATURE1_N * FEATURE1_LAYERS_N; j++) {
					if ((int) (i / INPUT_L)
						>= (int) ((j % FEATURE1_N) / FEATURE1_L)&& (int)(i/INPUT_L) < (int)((j%FEATURE1_N)/FEATURE1_L) + FILTER1_L
						&& i%INPUT_W >= (j%FEATURE1_N)%FEATURE1_W
						&& i%INPUT_W < (j%FEATURE1_N)%FEATURE1_W + FILTER1_W) {
						row_id = (FILTER1_L + 1 * 2) * (int) ((j % FEATURE1_N) / FEATURE1_W) + (int) (i / INPUT_W)
							- (int) ((j % FEATURE1_N) / FEATURE1_W);
						col_id = (FILTER1_W + 1 * 2) * ((j % FEATURE1_N) % FEATURE1_W) + i % INPUT_W
							- (j % FEATURE1_N) % FEATURE1_W;
						feature_id = (int) (j / FEATURE1_N);
						warrayS[(feature_id >> 1) * (FILTER1_L * FEATURE1_L + 16 * 2) + row_id][(feature_id & 0x1)
							* (FILTER1_W * FEATURE1_W + 16 * 2) + col_id] = memory.synapseMap->buffer2d[((i & 0xf)
							| ((i & 0x10) >> 4) << 8 | ((i & 0x1e0) >> 5) << 4 | ((i & 0x200) >> 9) << 9)][j
							+ TOTAL_NEURON_NUM_ON_CHIP * 2]; //i
					}
				}
			}
			for (i = 0; i < sizeX; i++) {
				for (ys = 0; ys < sizeY; ys++) {
					if (i < FILTER1_L * FEATURE1_L + 16 * 2 - 2 || ys < FILTER1_W * FEATURE1_W + 16 * 2 - 2) {
						if (i % (FILTER1_L + 1 * 2) == (FILTER1_L + 1 * 2 - 2)
							|| ys % (FILTER1_W + 1 * 2) == (FILTER1_W + 1 * 2 - 2)
							|| i % (FILTER1_L + 1 * 2) == (FILTER1_L + 1 * 2 - 1)
							|| ys % (FILTER1_W + 1 * 2) == (FILTER1_W + 1 * 2 - 1)) {
							warrayS[i][ys] = -1;
						}
					}
				}
			}
			//for output layer
			for (i = 0; i < FEATURE1_N * FEATURE1_LAYERS_N; i++) {
				for (j = 0; j < OUTPUT2_N; j++) {
					row_id = (int) (i / 16);
					col_id = i % 16 + j * (16 + 1 * 2);
					warrayS[(FILTER1_L + 1 * 2) * FEATURE1_L + row_id][(FILTER1_W + 1 * 2) * FEATURE1_W + col_id] =
						memory.synapseMap->buffer2d[i + TOTAL_NEURON_NUM_ON_CHIP * 2][j + TOTAL_NEURON_NUM_ON_CHIP * 2
							+ TOTAL_NEURON_NUM_IN_CORE * 3];
				}
			}
			/*			for (i = 0; i < FEATURE1_N * FEATURE1_LAYERS_N; i++) {
			 for (j = 0; j < OUTPUT2_N; j++) {
			 row_id = (int) (i/16);
			 col_id = i%16 + j * 16;
			 warrayS[(FILTER1_L+1*2)*FEATURE1_L + row_id][(FILTER1_W+1*2)*FEATURE1_W + col_id] = -1;
			 row_id = (int) (i/16);
			 col_id = i%16 + j * (16+1);
			 warrayS[(FILTER1_L+1*2)*FEATURE1_L + row_id][(FILTER1_W+1*2)*FEATURE1_W + col_id] = -1;
			 }
			 } */
			counterS = 0;
			for (i = 0; i < sizeX; i++) {
				for (ys = 0; ys < sizeY; ys++) {
					colS = GetColourSG((int) warrayS[ys][i]);
					singleplotS->pixels[counterS] = colS.r;
					singleplotS->pixels[counterS + 1] = colS.g;
					singleplotS->pixels[counterS + 2] = colS.b;
					counterS += 3;
				}
			}
			caerFrameEventSetLengthXLengthYChannelNumber(singleplotS, sizeX, sizeY, 3, *synapseplotfeature); //add info to the frame
			caerFrameEventValidate(singleplotS, *synapseplotfeature); //validate frame
		}
	}

	if (spike == NULL) { // Only process packets with content.
		return;
	}

	caerGestureLearningFilterConfig(moduleData); // Update parameters
	int64_t neuronAddr = 0;

	// keep changing the input pattern
	if (state->stimulate == true) { //it runs when there is a spike
		if (stimdisabled == 0 && abs(time_count - time_count_last) >= 5) {
			DisableStimuliGenG(moduleData);
			time_count_last = time_count;
			stimdisabled = 1;
		}
		else if (stimdisabled == 1 && abs(time_count - time_count_last) >= 1) {
			if (pattern_number == 3) {
				stimuliPattern = (stimuliPattern + 1) % 3;
			}
			else if (pattern_number == 4) {
				stimuliPattern = (stimuliPattern + 1) % 4;
			}
			EnableStimuliGenG(moduleData, stimuliPattern);
			time_count_last = time_count;
			stimdisabled = 0;
		}
	}
	else {
		DisableStimuliGenG(moduleData);
	}

	CAER_SPIKE_ITERATOR_VALID_START(spike) // Iterate over events and update weight

		if (state->learning == true) {
			EnableTeachingG(moduleData);

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
				memory.spikeFifo->buffer2d[memory.wrPointer][0] = neuronAddr; // Put spike address into the queue
				memory.spikeFifo->buffer2d[memory.wrPointer][1] = ts; // Put spike address into the queue
				memory.spikeCounter += 1;
				memory.wrPointer = (memory.wrPointer + 1) % SPIKE_QUEUE_LENGTH;
			}

			uint8_t endSearching = 0;
			int64_t deltaTimeAccumulated = 0;

			if (memory.wrPointer - memory.preRdPointer >= MINIMUM_CONSIDERED_SPIKE_NUM) {

				int64_t preSpikeAddr = memory.spikeFifo->buffer2d[memory.preRdPointer][0];
				int64_t preSpikeTime = memory.spikeFifo->buffer2d[memory.preRdPointer][1];
				memory.spikeCounter -= 1;
				memory.preRdPointer = (memory.preRdPointer + 1) % SPIKE_QUEUE_LENGTH;
				for (uint64_t postRdPointer = (memory.preRdPointer + 1) % SPIKE_QUEUE_LENGTH; endSearching != 1;
					postRdPointer = (postRdPointer + 1) % SPIKE_QUEUE_LENGTH) {
					int64_t postSpikeAddr = memory.spikeFifo->buffer2d[postRdPointer][0];
					int64_t postSpikeTime = memory.spikeFifo->buffer2d[postRdPointer][1];
					int64_t deltaTime = (int64_t) (postSpikeTime - preSpikeTime); //should be positive

					if (deltaTime <= 0)
						break;

					if (deltaTime < MAXIMUM_CONSIDERED_SPIKE_DELAY) {
						double deltaWeight = deltaWeights[DELTA_WEIGHT_LUT_LENGTH - deltaTime];
						if (memory.connectionMap->buffer2d[preSpikeAddr - MEMORY_NEURON_ADDR_OFFSET][postSpikeAddr
							- MEMORY_NEURON_ADDR_OFFSET] == 1) {
							ModifyForwardSynapseG(moduleData, preSpikeAddr, postSpikeAddr, deltaWeight,
								synapseplotfeature, weightplotfeature);
						}
						if (memory.connectionMap->buffer2d[postSpikeAddr - MEMORY_NEURON_ADDR_OFFSET][preSpikeAddr
							- MEMORY_NEURON_ADDR_OFFSET] == 1) {
							ModifyBackwardSynapseG(moduleData, preSpikeAddr, postSpikeAddr, deltaWeight,
								weightplotfeature);
						}
					}

					if (postRdPointer == memory.wrPointer - 1)
						endSearching = 1;
					else if (deltaTimeAccumulated > MAXIMUM_CONSIDERED_SPIKE_DELAY)
						endSearching = 1;
					deltaTimeAccumulated += deltaTime;
				}
			}
		}
		else {
			DisableTeachingG(moduleData);
		}

	CAER_SPIKE_ITERATOR_VALID_END

}

void ModifyForwardSynapseG(caerModuleData moduleData, int64_t preSpikeAddr, int64_t postSpikeAddr, double deltaWeight,
	caerFrameEventPacket *synapseplotfeature, caerFrameEventPacket *weightplotfeature) {

	GFilterState state = moduleData->moduleState;

	double new_weight;
	int64_t min = MEMORY_NEURON_ADDR_OFFSET;
	int64_t i, j, preAddr, postAddr;
	int64_t preNeuronId;
	uint32_t camId;
	int8_t synapseType = 0;
	int8_t synapseUpgrade = 0;
	int64_t preNeuronAddr, postNeuronAddr;
	int32_t new_synapse_add = 0;
	int32_t new_synapse_sub = 0;
	int32_t replaced_synapse = 0;
	preNeuronAddr = preSpikeAddr;
	postNeuronAddr = postSpikeAddr;

	uint32_t counterS;
	COLOUR colS;
	int64_t row_id_t, col_id_t, row_id, col_id, feature_id;
	caerFrameEvent singleplotS;

	int32_t output_disabled = 0;
	int output_counter = 0;
	int64_t neuron_address;
	uint32_t post_neuron_address;

	uint32_t k;
	int n;

	if (memory.connectionMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr
		- MEMORY_NEURON_ADDR_OFFSET] == 1) {

		new_weight = memory.weightMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr
			- MEMORY_NEURON_ADDR_OFFSET] + deltaWeight * state->learningRateForward;

		double increased_weight = 0;
		if (new_weight
			> memory.weightMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr
				- MEMORY_NEURON_ADDR_OFFSET]) {

			int current_synapse = memory.synapseMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr
				- MEMORY_NEURON_ADDR_OFFSET];
			synapseUpgrade = 0;

			output_disabled = 0; //memory.outputMapDisabled->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][(postNeuronAddr & 0xff)]
			if ((postNeuronAddr & 0x3c00) >> 10 == 3 && (postNeuronAddr & 0x300) >> 8 == 3) {
				output_disabled =
					memory.outputMapDisabled->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][(postNeuronAddr & 0xff)
						/ 4];
				if (output_disabled == 0) {
					output_counter = 0;
					for (i = 0; i < OUTPUT2_N; i++) {
						if (memory.outputMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][i] == 1) {
							output_counter += 1;
						}
					}
					if (output_counter >= 2
						|| (output_counter == 1
							&& memory.outputMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][(postNeuronAddr
								& 0xff) / 4] != 1)) {
						output_disabled = 1;
						//					memory.outputMapDisabled->buffer2d[preNeuronAddr-MEMORY_NEURON_ADDR_OFFSET][(postNeuronAddr & 0xff)] = 1;
						for (k = 0; k < OUTPUT2_N; k++) {
							post_neuron_address = 3 << 10 | 3 << 8 | (k * 4);
							for (i = 0; i < OUTPUT2_N; i++) {
								if (memory.outputMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][i] == 1) {
									if (pattern_number == 3) {
										n = 60;
									}
									else if (pattern_number == 4) {
										n = 59;
									}
									for (j = 0; j < n; j++) {
										camId = (uint32_t) j;
										if (memory.camMap->buffer2d[post_neuron_address - MEMORY_NEURON_ADDR_OFFSET][camId]
											!= 0) {
											neuron_address = memory.camMapContentSource->buffer2d[post_neuron_address
												- MEMORY_NEURON_ADDR_OFFSET][camId];
											if (neuron_address == preNeuronAddr) {
												WriteCamG(moduleData, 0, post_neuron_address, 0, (uint32_t) camId, 0, 0,
													1);
												memory.camMap->buffer2d[post_neuron_address - MEMORY_NEURON_ADDR_OFFSET][camId] =
													0;
												memory.camSize->buffer2d[post_neuron_address - MEMORY_NEURON_ADDR_OFFSET][0] -=
													1;
												memory.connectionCamMap->buffer2d[post_neuron_address
													- MEMORY_NEURON_ADDR_OFFSET][preNeuronAddr
													- MEMORY_NEURON_ADDR_OFFSET] = 0;
												memory.camMapContentType->buffer2d[post_neuron_address
													- MEMORY_NEURON_ADDR_OFFSET][camId] = 0;
												memory.camMapContentSource->buffer2d[post_neuron_address
													- MEMORY_NEURON_ADDR_OFFSET][camId] = 0;
											}
										}
									}
								}
							}
							memory.outputMapDisabled->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][k] = 1;
							memory.synapseMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_address
								- MEMORY_NEURON_ADDR_OFFSET] = 0;
							if (*synapseplotfeature != NULL) {
								singleplotS = caerFrameEventPacketGetEvent(*synapseplotfeature, 0);
							}
							if (*synapseplotfeature != NULL) {
								int64_t preNeuronAddr_t = preNeuronAddr;
								int64_t postNeuronAddr_t = post_neuron_address;
								colS = GetColourSG(new_synapse_sub);
								preAddr = (preNeuronAddr_t - MEMORY_NEURON_ADDR_OFFSET) % TOTAL_NEURON_NUM_ON_CHIP;
								postAddr = (postNeuronAddr_t - MEMORY_NEURON_ADDR_OFFSET) % TOTAL_NEURON_NUM_ON_CHIP;
								i = preAddr;
								j = postAddr;
								row_id_t = (int) (i / 16);
								col_id_t = i % 16 + ((j % 256) / 4) * (16 + 1 * 2);
								row_id = (FILTER1_L + 1 * 2) * FEATURE1_L + row_id_t;
								col_id = (FILTER1_W + 1 * 2) * FEATURE1_W + col_id_t;
								counterS = (uint32_t) ((row_id * 606) + col_id) * 3;
								singleplotS->pixels[counterS] = colS.r;
								singleplotS->pixels[counterS + 1] = colS.g;
								singleplotS->pixels[counterS + 2] = colS.b;
							}
						}

					}
				}
			}

			if (deltaWeight * state->learningRateForward > synapseUpgradeThreshold[current_synapse]
				&& (((postSpikeAddr & 0x3c00) >> 10 == 3 && (postSpikeAddr & 0x300) >> 8 != 3
					&& current_synapse <= state->maxSynapseFeature)
					|| ((postSpikeAddr & 0x3c00) >> 10 == 3 && (postSpikeAddr & 0x300) >> 8 == 3
						&& current_synapse <= state->maxSynapseOutput && output_disabled == 0))) {
				increased_weight = increased_weight + deltaWeight * state->learningRateForward;

				int slowFound = 0;
				int minFound = 0;
				double current_weight = 0;
				double current_weight_t = 0;
				double min_weight = 0;
				int synapseType_t = 0;

				int availableCamFound = 0;
				uint32_t cam_id;

				int camsize = 0;
				uint32_t camsize_limit = 0;

				if ((postSpikeAddr & 0x3c00) >> 10 == 3 && (postSpikeAddr & 0x300) >> 8 != 3) {
					camsize_limit = TOTAL_CAM_NUM_LEARNING;
				}
				else if ((postSpikeAddr & 0x3c00) >> 10 == 3 && (postSpikeAddr & 0x300) >> 8 == 3) {
					if (pattern_number == 3) {
						n = 60;
					}
					else if (pattern_number == 4) {
						n = 59;
					}
					camsize_limit = n;
				}

				camsize = memory.camSize->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][0];

				if (camsize < (int) camsize_limit) {
					for (cam_id = 0; cam_id < camsize_limit; cam_id++) { //search for available CAM
						if (memory.camMap->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][cam_id] == 0) {
							camId = cam_id;
							availableCamFound = 1;
							break;
						}
					}
				}

				if (availableCamFound == 0) {
					for (i = 0; i < camsize_limit; i++) {
						if (memory.camMap->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][i] != 0) {
							preNeuronId = memory.camMapContentSource->buffer2d[postNeuronAddr
								- MEMORY_NEURON_ADDR_OFFSET][i];
							synapseType_t = memory.camMapContentType->buffer2d[postNeuronAddr
								- MEMORY_NEURON_ADDR_OFFSET][i];
							if (synapseType_t > 0) { //Real synapse exists
								if (preNeuronId != preNeuronAddr && minFound == 0) {
									minFound = 1;
									min = preNeuronId;
									min_weight =
										memory.weightMap->buffer2d[min - MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr
											- MEMORY_NEURON_ADDR_OFFSET];
									camId = (uint32_t) i;
									replaced_synapse = memory.camMapContentType->buffer2d[postNeuronAddr
										- MEMORY_NEURON_ADDR_OFFSET][i];
								}
								if (preNeuronId == preNeuronAddr && synapseType_t == EXCITATORY_SLOW_SYNAPSE_ID) { //synapse already exists
									slowFound = 1;
									camId = (uint32_t) i;
									break;
								}
								current_weight_t =
									memory.weightMap->buffer2d[preNeuronId - MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr
										- MEMORY_NEURON_ADDR_OFFSET];
								if (minFound == 1 && preNeuronId != preNeuronAddr && current_weight_t < min_weight) {
									min = preNeuronId;
									min_weight =
										memory.weightMap->buffer2d[min - MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr
											- MEMORY_NEURON_ADDR_OFFSET];
									camId = (uint32_t) i;
									replaced_synapse = memory.camMapContentType->buffer2d[postNeuronAddr
										- MEMORY_NEURON_ADDR_OFFSET][i];
								}
							}
						}
					}
				}

				if (availableCamFound == 1) {
					current_weight =
						memory.weightMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr
							- MEMORY_NEURON_ADDR_OFFSET];
					new_weight = current_weight + increased_weight;
					synapseUpgrade = 1;
					synapseType = EXCITATORY_SLOW_SYNAPSE_ID;
					DisableStimuliGenPrimitiveCamG(moduleData);
					WriteCamG(moduleData, (uint32_t) preNeuronAddr, (uint32_t) postNeuronAddr, 0, camId, synapseType, 0,
						1);
					EnableStimuliGenPrimitiveCamG(moduleData);
					if ((postNeuronAddr & 0x3c00) >> 10 == 3 && (postNeuronAddr & 0x300) >> 8 == 3) {
						if (new_weight > 10)
							memory.outputMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][(postNeuronAddr & 0xff)
								/ 4] = 1;
					}
					new_synapse_add = current_synapse + EXCITATORY_SLOW_SYNAPSE_ID;
					memory.synapseMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr
						- MEMORY_NEURON_ADDR_OFFSET] = new_synapse_add;
					memory.connectionCamMap->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][preNeuronAddr
						- MEMORY_NEURON_ADDR_OFFSET] = (int32_t) camId;
					memory.camMapContentType->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][camId] =
					EXCITATORY_SLOW_SYNAPSE_ID;
					memory.camMapContentSource->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][camId] =
						(int32_t) preNeuronAddr;

					memory.camMap->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][camId] =
						(int32_t) preNeuronAddr;
					memory.camSize->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][0] += 1;
				}

				if (slowFound == 1) {
					synapseUpgrade = 1;
					synapseType = EXCITATORY_FAST_SYNAPSE_ID;
					DisableStimuliGenPrimitiveCamG(moduleData);
					WriteCamG(moduleData, (uint32_t) preNeuronAddr, (uint32_t) postNeuronAddr, 0, camId, synapseType, 0,
						1);
					EnableStimuliGenPrimitiveCamG(moduleData);
					if ((postNeuronAddr & 0x3c00) >> 10 == 3 && (postNeuronAddr & 0x300) >> 8 == 3) {
						if (new_weight > 10)
							memory.outputMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][(postNeuronAddr & 0xff)
								/ 4] = 1;
					}
					new_synapse_add = current_synapse + (EXCITATORY_FAST_SYNAPSE_ID - EXCITATORY_SLOW_SYNAPSE_ID);
					memory.synapseMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr
						- MEMORY_NEURON_ADDR_OFFSET] = new_synapse_add;
					memory.camMapContentType->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][camId] =
					EXCITATORY_FAST_SYNAPSE_ID;
					memory.weightMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr
						- MEMORY_NEURON_ADDR_OFFSET] = new_weight;
				}
				else if (minFound == 1 && increased_weight > min_weight) {
					current_weight =
						memory.weightMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr
							- MEMORY_NEURON_ADDR_OFFSET];
					new_weight = current_weight + increased_weight;
					synapseUpgrade = 1;
					synapseType = EXCITATORY_SLOW_SYNAPSE_ID;
					DisableStimuliGenPrimitiveCamG(moduleData);
					WriteCamG(moduleData, (uint32_t) preNeuronAddr, (uint32_t) postNeuronAddr, 0, camId, synapseType, 0,
						1);
					EnableStimuliGenPrimitiveCamG(moduleData);
					if ((postNeuronAddr & 0x3c00) >> 10 == 3 && (postNeuronAddr & 0x300) >> 8 == 3) {
						if (new_weight > 10)
							memory.outputMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][(postNeuronAddr & 0xff)
								/ 4] = 1;
					}
					new_synapse_add = current_synapse + EXCITATORY_SLOW_SYNAPSE_ID;
					new_synapse_sub = memory.synapseMap->buffer2d[min - MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr
						- MEMORY_NEURON_ADDR_OFFSET] - replaced_synapse;
					if ((postNeuronAddr & 0x3c00) >> 10 == 3 && (postNeuronAddr & 0x300) >> 8 == 3) {
						if (new_synapse_sub == 0) {
							memory.outputMap->buffer2d[min - MEMORY_NEURON_ADDR_OFFSET][(postNeuronAddr & 0xff) / 4] =
								0;
						}
					}
					memory.synapseMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr
						- MEMORY_NEURON_ADDR_OFFSET] = new_synapse_add;
					memory.synapseMap->buffer2d[min - MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr
						- MEMORY_NEURON_ADDR_OFFSET] = new_synapse_sub; //NO_SYNAPSE_ID
					memory.connectionCamMap->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][min
						- MEMORY_NEURON_ADDR_OFFSET] = 0;
					memory.connectionCamMap->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][preNeuronAddr
						- MEMORY_NEURON_ADDR_OFFSET] = (int32_t) camId;
					memory.camMapContentType->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][camId] =
					EXCITATORY_SLOW_SYNAPSE_ID;
					memory.camMapContentSource->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][camId] =
						(int32_t) preNeuronAddr;
					if ((postSpikeAddr & 0x3c00) >> 10 == 3 && (postSpikeAddr & 0x300) >> 8 != 3) {
						if (*synapseplotfeature != NULL) {
							singleplotS = caerFrameEventPacketGetEvent(*synapseplotfeature, 0);
						}
						if (*synapseplotfeature != NULL) {
							int64_t preNeuronAddr_t = min;
							int64_t postNeuronAddr_t = postSpikeAddr;
							colS = GetColourSG(new_synapse_sub);
							preAddr = preNeuronAddr_t - MEMORY_NEURON_ADDR_OFFSET;
							postAddr = (postNeuronAddr_t - MEMORY_NEURON_ADDR_OFFSET) % TOTAL_NEURON_NUM_ON_CHIP;
							i = (preAddr & 0xf) | ((preAddr & 0x100) >> 8) << 4 | ((preAddr & 0xf0) >> 4) << 5
								| ((preAddr & 0x200) >> 9) << 9;
							j = postAddr;
							feature_id = (int) (j / FEATURE1_N);
							row_id_t = (FILTER1_L + 1 * 2) * (int) ((j % FEATURE1_N) / FEATURE1_W) + (int) (i / INPUT_W)
								- (int) ((j % FEATURE1_N) / FEATURE1_W);
							row_id = (feature_id >> 1) * (FILTER1_L * FEATURE1_L + 16 * 2) + row_id_t;
							col_id_t = (FILTER1_W + 1 * 2) * ((j % FEATURE1_N) % FEATURE1_W) + i % INPUT_W
								- (j % FEATURE1_N) % FEATURE1_W;
							col_id = (feature_id & 0x1) * (FILTER1_W * FEATURE1_W + 16 * 2) + col_id_t;
							counterS = (uint32_t) ((col_id * 606) + row_id) * 3; //((row_id * VISUALIZER_WIDTH_FEATURE) + col_id) * 3; doesn't work
							singleplotS->pixels[counterS] = colS.r;
							singleplotS->pixels[counterS + 1] = colS.g;
							singleplotS->pixels[counterS + 2] = colS.b;
						}
					}
					else if ((postSpikeAddr & 0x3c00) >> 10 == 3 && (postSpikeAddr & 0x300) >> 8 == 3) {
						if (*synapseplotfeature != NULL) {
							singleplotS = caerFrameEventPacketGetEvent(*synapseplotfeature, 0);
						}
						if (*synapseplotfeature != NULL) {
							int64_t preNeuronAddr_t = min;
							int64_t postNeuronAddr_t = postSpikeAddr;
							colS = GetColourSG(new_synapse_sub);
							preAddr = (preNeuronAddr_t - MEMORY_NEURON_ADDR_OFFSET) % TOTAL_NEURON_NUM_ON_CHIP;
							postAddr = (postNeuronAddr_t - MEMORY_NEURON_ADDR_OFFSET) % TOTAL_NEURON_NUM_ON_CHIP;
							i = preAddr;
							j = postAddr;
							row_id_t = (int) (i / 16);
							col_id_t = i % 16 + ((j % 256) / 4) * (16 + 1 * 2);
							row_id = (FILTER1_L + 1 * 2) * FEATURE1_L + row_id_t;
							col_id = (FILTER1_W + 1 * 2) * FEATURE1_W + col_id_t;
							counterS = (uint32_t) ((row_id * 606) + col_id) * 3;
							singleplotS->pixels[counterS] = colS.r;
							singleplotS->pixels[counterS + 1] = colS.g;
							singleplotS->pixels[counterS + 2] = colS.b;
						}
					}

					memory.weightMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr
						- MEMORY_NEURON_ADDR_OFFSET] = new_weight;
				}

				if (synapseUpgrade == 1) {
					if ((postSpikeAddr & 0x3c00) >> 10 == 3 && (postSpikeAddr & 0x300) >> 8 != 3) {
						if (*synapseplotfeature != NULL) {
							singleplotS = caerFrameEventPacketGetEvent(*synapseplotfeature, 0);
						}
						if (*synapseplotfeature != NULL) {
							int64_t preNeuronAddr_t = preSpikeAddr;
							int64_t postNeuronAddr_t = postSpikeAddr;
							colS = GetColourSG(new_synapse_add);
							preAddr = preNeuronAddr_t - MEMORY_NEURON_ADDR_OFFSET;
							postAddr = (postNeuronAddr_t - MEMORY_NEURON_ADDR_OFFSET) % TOTAL_NEURON_NUM_ON_CHIP;
							i = (preAddr & 0xf) | ((preAddr & 0x100) >> 8) << 4 | ((preAddr & 0xf0) >> 4) << 5
								| ((preAddr & 0x200) >> 9) << 9;
							j = postAddr;
							feature_id = (int) (j / FEATURE1_N);
							row_id_t = (FILTER1_L + 1 * 2) * (int) ((j % FEATURE1_N) / FEATURE1_W) + (int) (i / INPUT_W)
								- (int) ((j % FEATURE1_N) / FEATURE1_W);
							row_id = (feature_id >> 1) * (FILTER1_L * FEATURE1_L + 16 * 2) + row_id_t;
							col_id_t = (FILTER1_W + 1 * 2) * ((j % FEATURE1_N) % FEATURE1_W) + i % INPUT_W
								- (j % FEATURE1_N) % FEATURE1_W;
							col_id = (feature_id & 0x1) * (FILTER1_W * FEATURE1_W + 16 * 2) + col_id_t;
							counterS = ((col_id * 606) + row_id) * 3;
							singleplotS->pixels[counterS] = colS.r;
							singleplotS->pixels[counterS + 1] = colS.g;
							singleplotS->pixels[counterS + 2] = colS.b;
						}
					}
					else if ((postSpikeAddr & 0x3c00) >> 10 == 3 && (postSpikeAddr & 0x300) >> 8 == 3) {
						if (*synapseplotfeature != NULL) {
							singleplotS = caerFrameEventPacketGetEvent(*synapseplotfeature, 0);
						}
						if (*synapseplotfeature != NULL) {
							int64_t preNeuronAddr_t = preSpikeAddr;
							int64_t postNeuronAddr_t = postSpikeAddr;
							colS = GetColourSG(new_synapse_add);
							preAddr = (preNeuronAddr_t - MEMORY_NEURON_ADDR_OFFSET) % TOTAL_NEURON_NUM_ON_CHIP;
							postAddr = (postNeuronAddr_t - MEMORY_NEURON_ADDR_OFFSET) % TOTAL_NEURON_NUM_ON_CHIP;
							i = preAddr;
							j = postAddr;
							row_id_t = (int) (i / 16);
							col_id_t = i % 16 + ((j % 256) / 4) * (16 + 1 * 2);
							row_id = (FILTER1_L + 1 * 2) * FEATURE1_L + row_id_t;
							col_id = (FILTER1_W + 1 * 2) * FEATURE1_W + col_id_t;
							counterS = (uint32_t) ((row_id * 606) + col_id) * 3; //VISUALIZER_WIDTH_FEATURE
							singleplotS->pixels[counterS] = colS.r;
							singleplotS->pixels[counterS + 1] = colS.g;
							singleplotS->pixels[counterS + 2] = colS.b;
						}
					}
				}
			}
		}
	}
}

void ModifyBackwardSynapseG(caerModuleData moduleData, int64_t preSpikeAddr, int64_t postSpikeAddr, double deltaWeight,
	caerFrameEventPacket *weightplotfeature) {

	GFilterState state = moduleData->moduleState;

	double new_weight;
	int64_t preNeuronAddr, postNeuronAddr;
	preNeuronAddr = postSpikeAddr;
	postNeuronAddr = preSpikeAddr;
	if (memory.connectionMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr
		- MEMORY_NEURON_ADDR_OFFSET] == 1) {
		new_weight = memory.weightMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr
			- MEMORY_NEURON_ADDR_OFFSET] - deltaWeight * state->learningRateBackward;
		memory.weightMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET] =
			new_weight;
	}
}

//reset the network to the initial state
bool ResetNetworkG(caerModuleData moduleData) {
	DisableStimuliGenG(moduleData);
	ResetBiasesG(moduleData);
	time_count = 0;
	signal(SIGALRM, SignalHandlerG); //register the hand-made timer function
	SetTimerG();

	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Clearing all CAMs..");
	ClearAllCamG(moduleData); //only for 1st chip
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Done");
//	SetInputLayerCamG(moduleData, eventSourceID);

	GFilterState state = moduleData->moduleState;
	int8_t exType = state->resetExType; //initial synapse type fast or slow
	int8_t inType = state->resetInType; //initial synapse type fast or slow

	memory.connectionMap = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD,
		(size_t) TOTAL_NEURON_NUM_ON_BOARD);
	memory.filterMap = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) MAXIMUM_FILTER_SIZE);
	memory.outputMap = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) OUTPUT2_N);
	memory.outputMapDisabled = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) OUTPUT2_N);
	memory.camMapContentSource = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) TOTAL_CAM_NUM);
	memory.camMapContentType = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) TOTAL_CAM_NUM);
	memory.connectionCamMap = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) MAXIMUM_FILTER_SIZE);
	memory.filterMapSize = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) FILTER_MAP_SIZE_WIDTH);

	memory.weightMap = simple2DBufferInitDouble((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) TOTAL_NEURON_NUM_ON_BOARD);
	memory.synapseMap = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) TOTAL_NEURON_NUM_ON_BOARD);
	memory.camMap = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) TOTAL_CAM_NUM);
	memory.camSize = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) CAM_SIZE_WIDTH);
	memory.sramMap = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) TOTAL_SRAM_NUM);
	memory.sramMapContent = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) TOTAL_SRAM_NUM);
	memory.spikeFifo = simple2DBufferInitLong((size_t) SPIKE_QUEUE_LENGTH, (size_t) SPIKE_QUEUE_WIDTH);

	uint32_t chipId;
	uint32_t coreId;
	uint32_t neuronId;

	//create input layer
	uint32_t input_layer[INPUT_N];
	for (neuronId = 0; neuronId < INPUT_N; neuronId++) {
		chipId = CHIP_UP_LEFT_ID;
		input_layer[neuronId] = chipId << NEURON_CHIPID_SHIFT
			| ((neuronId & 0xf) | ((neuronId & 0x10) >> 4) << 8 | ((neuronId & 0x1e0) >> 5) << 4
				| ((neuronId & 0x200) >> 9) << 9);
	}
	//create feature layer 1
	uint32_t feature_layer1[FEATURE1_N * FEATURE1_LAYERS_N];
	for (neuronId = 0; neuronId < FEATURE1_N * FEATURE1_LAYERS_N; neuronId++) {
		chipId = CHIP_DOWN_LEFT_ID;
		feature_layer1[neuronId] = chipId << NEURON_CHIPID_SHIFT | neuronId;
	}
	//create output layer 2
	uint32_t output_layer2[OUTPUT2_N];
	for (neuronId = 0; neuronId < OUTPUT2_N; neuronId++) {
		chipId = CHIP_DOWN_LEFT_ID;
		coreId = CORE_DOWN_RIGHT_ID;
		if (neuronId == 0)
			output_layer2[neuronId] = chipId << NEURON_CHIPID_SHIFT | coreId << NEURON_COREID_SHIFT | 0;
		if (neuronId == 1)
			output_layer2[neuronId] = chipId << NEURON_CHIPID_SHIFT | coreId << NEURON_COREID_SHIFT | 4;
		if (neuronId == 2)
			output_layer2[neuronId] = chipId << NEURON_CHIPID_SHIFT | coreId << NEURON_COREID_SHIFT | 8;
		if (neuronId == 3)
			output_layer2[neuronId] = chipId << NEURON_CHIPID_SHIFT | coreId << NEURON_COREID_SHIFT | 12;
	}

	int preNeuronId, postNeuronId;
	int core_id;
	uint32_t preNeuronAddr, postNeuronAddr;
	int randNumCount;
	uint32_t virtualNeuronAddr = 0;
	int8_t virtualNeuronAddrEnable = 0;

	int8_t inhibitoryValid[FEATURE1_LAYERS_N][TOTAL_NEURON_NUM_ON_CHIP];
	uint32_t inhibitoryVirtualNeuronCoreId[FEATURE1_LAYERS_N][TOTAL_NEURON_NUM_IN_CORE];

	for (core_id = 0; core_id < FEATURE1_LAYERS_N; core_id++) {
		for (neuronId = 0; neuronId < TOTAL_NEURON_NUM_ON_CHIP; neuronId++) {
			inhibitoryValid[core_id][neuronId] = 0;
		}
	}

	//randomly select 1 neuron from 4 neurons
	for (core_id = 0; core_id < FEATURE1_LAYERS_N; core_id++) {
		for (neuronId = 0; neuronId < TOTAL_NEURON_NUM_IN_CORE; neuronId++) {
			coreId = (uint32_t) (rand() % 4); //0 //randomly choose one value in 0, 1, 2, 3
			preNeuronAddr = coreId << NEURON_COREID_SHIFT | neuronId;
			inhibitoryValid[core_id][preNeuronAddr] = 1;
			inhibitoryVirtualNeuronCoreId[core_id][neuronId] = coreId;
		}
	}

	//input to feature1
	/*for (postNeuronId = 0; postNeuronId < FEATURE1_N * FEATURE1_LAYERS_N; postNeuronId++) { //FEATURE1_N*FEATURE1_LAYERS_N //first sweep POST, then PRE
		//generate random binary number 1D array
		int64_t rand1DBinaryArray[FILTER1_N]; //FILTER1_N-FEATURE1_CAM_INHIBITORY_N
		GetRand1DBinaryArrayG(rand1DBinaryArray, FILTER1_N, TOTAL_CAM_NUM_LEARNING); //FILTER1_N-FEATURE1_CAM_INHIBITORY_N
		randNumCount = 0;
		for (preNeuronId = 0; preNeuronId < INPUT_N; preNeuronId++) {
			int pre_id = preNeuronId;
			int post_id = postNeuronId;
			if ((int) (pre_id / INPUT_W)
				>= (int) ((post_id % FEATURE1_N) / FEATURE1_W)&& (int)(pre_id/INPUT_W) < (int)((post_id%FEATURE1_N)/FEATURE1_W) + FILTER1_L
				&& pre_id%INPUT_W >= (post_id%FEATURE1_N)%FEATURE1_W
				&& pre_id%INPUT_W < (post_id%FEATURE1_N)%FEATURE1_W + FILTER1_W) {
				//randomly reset, depends on the ratio of total CAM number and FILTER1_N-FEATURE1_CAM_INHIBITORY_N
				preNeuronAddr = input_layer[preNeuronId];
				postNeuronAddr = feature_layer1[postNeuronId];
				if (rand1DBinaryArray[randNumCount] == 1
					&& inhibitoryValid[(postNeuronAddr & 0x300) >> 8][preNeuronAddr & 0x3ff] == 0) //build a real synapse
					BuildSynapseG(moduleData, preNeuronAddr, postNeuronAddr, virtualNeuronAddr, exType, REAL_SYNAPSE,
						virtualNeuronAddrEnable);
				else if (inhibitoryValid[(postNeuronAddr & 0x300) >> 8][preNeuronAddr & 0x3ff] == 0)
					BuildSynapseG(moduleData, preNeuronAddr, postNeuronAddr, virtualNeuronAddr, exType, VIRTUAL_SYNAPSE,
						virtualNeuronAddrEnable);
				randNumCount += 1;
			}
		}
	}
	//feature1 to feature1
	virtualNeuronAddrEnable = 1;
	for (preNeuronId = 0; preNeuronId < FEATURE1_N * FEATURE1_LAYERS_N; preNeuronId++)
		for (postNeuronId = 0; postNeuronId < FEATURE1_N * FEATURE1_LAYERS_N; postNeuronId++) {
			if ((int) (preNeuronId / FEATURE1_N) != (int) (postNeuronId / FEATURE1_N)
				&& (preNeuronId % FEATURE1_N) == (postNeuronId % FEATURE1_N)) {
				preNeuronAddr = feature_layer1[preNeuronId];
				postNeuronAddr = feature_layer1[postNeuronId];
				coreId = inhibitoryVirtualNeuronCoreId[(postNeuronAddr & 0x300) >> 8][preNeuronAddr & 0xff];
				virtualNeuronAddr = ((preNeuronAddr & 0x3c00) >> 10) << 10 | coreId << 8 | (preNeuronAddr & 0xff);
				BuildSynapseG(moduleData, preNeuronAddr, postNeuronAddr, virtualNeuronAddr, (int16_t) (-1 * inType),
				REAL_SYNAPSE, virtualNeuronAddrEnable);
			}
		}
	virtualNeuronAddrEnable = 0;
	virtualNeuronAddr = 0;
	int n;
	//feature1 to output2
	for (postNeuronId = 0; postNeuronId < OUTPUT2_N; postNeuronId++) {
		//generate random binary number 1D array
		int64_t rand1DBinaryArray[FEATURE1_N * FEATURE1_LAYERS_N];
		if (pattern_number == 3) {
			n = 60; //59
		}
		else if (pattern_number == 4) {
			n = 59; //58
		}
		GetRand1DBinaryArrayG(rand1DBinaryArray, FEATURE1_N * FEATURE1_LAYERS_N, n); //TOTAL_CAM_NUM_LEARNING 60
		randNumCount = 0;
		for (preNeuronId = 0; preNeuronId < FEATURE1_N * FEATURE1_LAYERS_N; preNeuronId++) {
			//randomly reset, depends on the ratio of total CAM number and OUTPUT1_N
			preNeuronAddr = feature_layer1[preNeuronId];
			postNeuronAddr = output_layer2[postNeuronId];
			if (rand1DBinaryArray[randNumCount] == 1 && (preNeuronAddr & 0x3ff) != 0) {
				BuildSynapseG(moduleData, preNeuronAddr, postNeuronAddr, virtualNeuronAddr, exType, REAL_SYNAPSE,
					virtualNeuronAddrEnable);
			}
			else
				BuildSynapseG(moduleData, preNeuronAddr, postNeuronAddr, virtualNeuronAddr, exType, VIRTUAL_SYNAPSE,
					virtualNeuronAddrEnable);
			randNumCount += 1;
		}
	}
	for (postNeuronId = 0; postNeuronId < OUTPUT2_N; postNeuronId++) {
		for (preNeuronId = 0; preNeuronId < OUTPUT2_N; preNeuronId++) {
			if (preNeuronId != postNeuronId) {
				preNeuronAddr = output_layer2[preNeuronId];
				postNeuronAddr = output_layer2[postNeuronId];
				BuildSynapseG(moduleData, preNeuronAddr, postNeuronAddr, virtualNeuronAddr, (int16_t) (-1 * inType),
				REAL_SYNAPSE, virtualNeuronAddrEnable);
			}
		}
	}*/

	SetInputLayerCamG(moduleData); //It's different thread, should be put in the end.

	ConfigureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U0);
	ConfigureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U1);
	ConfigureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U2);
	ConfigureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U3);

	return (true);
}

bool ConfigureChipG(caerModuleData moduleData, int8_t chipId) {

	GFilterState state = moduleData->moduleState;

	// --- start  usb handle / from spike event source id
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(state->eventSourceID));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->eventSourceID));
	caerInputDynapseState stateSource = state->eventSourceModuleState;

	if (chipId == DYNAPSE_CONFIG_DYNAPSE_U0) {
		caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID,
		DYNAPSE_CONFIG_DYNAPSE_U0);
		if (!caerDynapseSendDataToUSB(stateSource->deviceState, bits_chipU0, (int) numConfig_chipU0)) { // - 1, - 10;  1 works
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "USB transfer failed");
		}
		numConfig_chipU0 = 0;
	}
	else if (chipId == DYNAPSE_CONFIG_DYNAPSE_U1) {
		caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID,
		DYNAPSE_CONFIG_DYNAPSE_U1);
		if (!caerDynapseSendDataToUSB(stateSource->deviceState, bits_chipU1, (int) numConfig_chipU1)) {
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "USB transfer failed");
		}
		numConfig_chipU1 = 0;
	}
	else if (chipId == DYNAPSE_CONFIG_DYNAPSE_U2) {
		caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID,
		DYNAPSE_CONFIG_DYNAPSE_U2);
		if (!caerDynapseSendDataToUSB(stateSource->deviceState, bits_chipU2, (int) numConfig_chipU2)) {
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "USB transfer failed");
		}
		numConfig_chipU2 = 0;
	}
	else if (chipId == DYNAPSE_CONFIG_DYNAPSE_U3) {
		caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID,
		DYNAPSE_CONFIG_DYNAPSE_U3);
		if (!caerDynapseSendDataToUSB(stateSource->deviceState, bits_chipU3, (int) numConfig_chipU3)) {
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "USB transfer failed");
		}
		numConfig_chipU3 = 0;
	}

}

//build synapses when reseting
bool BuildSynapseG(caerModuleData moduleData, uint32_t preNeuronAddr, uint32_t postNeuronAddr,
	uint32_t virtualNeuronAddr, int16_t synapseType, int8_t realOrVirtualSynapse, int8_t virtualNeuronAddrEnable) {
	uint32_t sramId, camId, sram_id, cam_id;
	int chipCoreId;
	int sramFound, camFound;
	int sramAvailable;
	int output_disabled;
	uint32_t i;
	//for SRAM
	if (realOrVirtualSynapse != EXTERNAL_REAL_SYNAPSE) {
		sramFound = 0;
		for (sram_id = 0; sram_id < TOTAL_SRAM_NUM; sram_id++) { //search for available SRAM
			chipCoreId = (int) (postNeuronAddr >> 8);
			if (memory.sramMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][sram_id] == 1
				&& memory.sramMapContent->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][sram_id] == chipCoreId) { //start the searching from second SRAM, for visualization
				sramFound = 1;
			}
		}
		if (sramFound == 0) {
			sramAvailable = 0;
			for (sram_id = 0; sram_id < TOTAL_SRAM_NUM; sram_id++) { //search for available SRAM
				if (sramAvailable == 0
					&& memory.sramMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][(sram_id + 1)
						% TOTAL_SRAM_NUM] == 0) {
					sramAvailable = 1;
					sramId = (sram_id + 1) % TOTAL_SRAM_NUM; //(sram_id + 1) % TOTAL_SRAM_NUM; keep the SRAM for viewer
				}
			}
			if (sramAvailable == 1 && sramId != 0) { //sramId != 0 && sramId != 1 && sramId != 2 && sramId != 3
				WriteSramG(moduleData, preNeuronAddr, postNeuronAddr, virtualNeuronAddr, sramId,
					virtualNeuronAddrEnable, 0);

				memory.sramMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][sramId] = 1; //taken
				chipCoreId = (int) (postNeuronAddr >> 8);
				memory.sramMapContent->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][sramId] = chipCoreId; //taken
			}
		}
	}
	//for CAM
	camFound = 0;
	for (cam_id = 0; cam_id < TOTAL_CAM_NUM; cam_id++) { //search for available CAM
		if (synapseType > 0) {
			if (memory.camMapContentSource->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][cam_id]
				== (int32_t) preNeuronAddr) {
				camFound = 1;
				break;
			}
		}
		else {
			if (memory.camMapContentSource->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][cam_id]
				== (int32_t) preNeuronAddr
				|| (memory.camMapContentSource->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][cam_id]
					== (int32_t) virtualNeuronAddr && virtualNeuronAddrEnable == 1)) {
				camFound = 1;
				break;
			}
		}
		if (memory.camMap->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][cam_id] == 0) {
			camId = cam_id;
			break;
		}
	}

	if (realOrVirtualSynapse == REAL_SYNAPSE || realOrVirtualSynapse == EXTERNAL_REAL_SYNAPSE) {
		if (camFound == 0) {
			output_disabled = 0;
			if ((postNeuronAddr & 0x3c00) >> 10 == 3 && (postNeuronAddr & 0x300) >> 8 == 3) {
				for (i = 0; i < 3; i++) {
					if (memory.outputMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][i] == 1
						&& (postNeuronAddr & 0xff) != i) {
						output_disabled = 1;
						break;
					}
				}
				if (output_disabled == 0) {
					WriteCamG(moduleData, preNeuronAddr, postNeuronAddr, virtualNeuronAddr, camId, synapseType,
						virtualNeuronAddrEnable, 0);
					memory.outputMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr & 0xff] = 1;
				}
			}
			else {
				WriteCamG(moduleData, preNeuronAddr, postNeuronAddr, virtualNeuronAddr, camId, synapseType,
					virtualNeuronAddrEnable, 0);
			}
			WriteCamG(moduleData, preNeuronAddr, postNeuronAddr, virtualNeuronAddr, camId, synapseType,
				virtualNeuronAddrEnable, 0);
			memory.camMap->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][camId] = (int32_t) preNeuronAddr;
			if (synapseType > 0) {
				memory.camSize->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][0] += 1;
			}
		}
	}
	//memories for the chip
	if (synapseType > 0) { //if it is EX synapse
		int32_t memoryId = memory.filterMapSize->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][0];
		memory.filterMap->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][memoryId] = (int32_t) preNeuronAddr;
		memory.connectionCamMap->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][memoryId] = (int32_t) camId;
		memory.filterMapSize->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][0] += 1;
		if (realOrVirtualSynapse != EXTERNAL_REAL_SYNAPSE) {
			memory.connectionMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr
				- MEMORY_NEURON_ADDR_OFFSET] = 1; //there is an EX connection
			memory.weightMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr
				- MEMORY_NEURON_ADDR_OFFSET] = 1; //8 initial weight
			if (realOrVirtualSynapse == REAL_SYNAPSE) {
				memory.synapseMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][postNeuronAddr
					- MEMORY_NEURON_ADDR_OFFSET] = synapseType;
				if (camFound == 0) {
					memory.camMapContentType->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][camId] = synapseType;
					memory.camMapContentSource->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][camId] =
						(int32_t) preNeuronAddr;
					if ((postNeuronAddr & 0x3c00) >> 10 == 3 && (postNeuronAddr & 0x300) >> 8 == 3) {
						memory.outputMap->buffer2d[preNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][(postNeuronAddr & 0xff)
							/ 4] = 1;
					}
				}
			}
		}
	}
	else {
		memory.camMapContentType->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][camId] = synapseType;
		memory.camMapContentSource->buffer2d[postNeuronAddr - MEMORY_NEURON_ADDR_OFFSET][camId] =
			(int32_t) preNeuronAddr;
	}
	return (true);
}

//get chip id based on post neuron
static int GetWriteCamChipIdG(uint32_t postNeuronAddr) {

	uint32_t chipId_t, chipId;
	chipId_t = postNeuronAddr >> NEURON_CHIPID_SHIFT;
	if (chipId_t == 1)
		chipId = DYNAPSE_CONFIG_DYNAPSE_U0;
	else if (chipId_t == 2)
		chipId = DYNAPSE_CONFIG_DYNAPSE_U1;
	else if (chipId_t == 3)
		chipId = DYNAPSE_CONFIG_DYNAPSE_U2;
	else if (chipId_t == 4)
		chipId = DYNAPSE_CONFIG_DYNAPSE_U3;

	return (chipId);

}

//write neuron CAM when a synapse is built or modified
static int GetWriteCamBitsG(caerModuleData moduleData, uint32_t preNeuronAddr, uint32_t postNeuronAddr,
	uint32_t virtualNeuronAddr, uint32_t camId, int16_t synapseType, int8_t virtualNeuronAddrEnable) {

	GFilterState state = moduleData->moduleState;

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
	uint32_t ei = 0;
	uint32_t fs = 0;
	uint32_t address = preNeuronAddr & NEURON_ADDRESS_BITS;
	uint32_t source_core = 0;
	if (virtualNeuronAddrEnable == 0)
		source_core = (preNeuronAddr & NEURON_COREID_BITS) >> NEURON_COREID_SHIFT;
	else
		source_core = (virtualNeuronAddr & NEURON_COREID_BITS) >> NEURON_COREID_SHIFT; //to change
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
	bits = ei << CXQ_CAM_EI_SHIFT | fs << CXQ_CAM_FS_SHIFT | address << CXQ_ADDR_SHIFT
		| source_core << CXQ_SOURCE_CORE_SHIFT |
		CXQ_PROGRAM | coreId << CXQ_PROGRAM_COREID_SHIFT | row << CXQ_PROGRAM_ROW_SHIFT
		| column << CXQ_PROGRAM_COLUMN_SHIFT;

	return (bits);

}

//write neuron CAM when a synapse is built or modified
bool WriteCamG(caerModuleData moduleData, uint32_t preNeuronAddr, uint32_t postNeuronAddr, uint32_t virtualNeuronAddr,
	uint32_t camId, int16_t synapseType, int8_t virtualNeuronAddrEnable, int8_t stdp) {

	GFilterState state = moduleData->moduleState;

	// --- start  usb handle / from spike event source id
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(state->eventSourceID));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->eventSourceID));

	caerInputDynapseState stateSource = state->eventSourceModuleState;
	// --- end usb handle

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
	uint32_t ei = 0;
	uint32_t fs = 0;
	uint32_t address = preNeuronAddr & NEURON_ADDRESS_BITS;
	uint32_t source_core = 0;
	if (virtualNeuronAddrEnable == 0)
		source_core = (preNeuronAddr & NEURON_COREID_BITS) >> NEURON_COREID_SHIFT;
	else
		source_core = (virtualNeuronAddr & NEURON_COREID_BITS) >> NEURON_COREID_SHIFT; //to change
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
	bits = ei << CXQ_CAM_EI_SHIFT | fs << CXQ_CAM_FS_SHIFT | address << CXQ_ADDR_SHIFT
		| source_core << CXQ_SOURCE_CORE_SHIFT |
		CXQ_PROGRAM | coreId << CXQ_PROGRAM_COREID_SHIFT | row << CXQ_PROGRAM_ROW_SHIFT
		| column << CXQ_PROGRAM_COLUMN_SHIFT;
	if (stdp == 0) {
		if (chipId == DYNAPSE_CONFIG_DYNAPSE_U0) {
			if (numConfig_chipU0 == usb_packet_maximum_size) { //DYNAPSE_MAX_USER_USB_PACKET_SIZE
				ConfigureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U0);
			}
			bits_chipU0[numConfig_chipU0] = (int) bits;
			numConfig_chipU0++;
		}
		else if (chipId == DYNAPSE_CONFIG_DYNAPSE_U1) {
			if (numConfig_chipU1 == usb_packet_maximum_size) {
				ConfigureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U1);
			}
			bits_chipU1[numConfig_chipU1] = (int) bits;
			numConfig_chipU1++;
		}
		else if (chipId == DYNAPSE_CONFIG_DYNAPSE_U2) {
			if (numConfig_chipU2 == usb_packet_maximum_size) {
				ConfigureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U2);
			}
			bits_chipU2[numConfig_chipU2] = (int) bits;
			numConfig_chipU2++;
		}
		else if (chipId == DYNAPSE_CONFIG_DYNAPSE_U3) {
			if (numConfig_chipU3 == usb_packet_maximum_size) {
				ConfigureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U3);
			}
			bits_chipU3[numConfig_chipU3] = (int) bits;
			numConfig_chipU3++;
		}
	}
	else {
		caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chipId);
		caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, bits); //this is the 30 bits
	}

	return (true);
}

//write neuron SRAM when a synapse is built
bool WriteSramG(caerModuleData moduleData, uint32_t preNeuronAddr, uint32_t postNeuronAddr, uint32_t virtualNeuronAddr,
	uint32_t sramId, int8_t virtualNeuronAddrEnable, int8_t stdp) {
//	caerDeviceHandle usb_handle = ((caerInputDynapseState) moduleData->moduleState)->deviceState; doesn't work
	GFilterState state = moduleData->moduleState;

	// --- start  usb handle / from spike event source id
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(state->eventSourceID));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->eventSourceID));

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
	uint32_t virtual_coreId = 0;
	if (virtualNeuronAddrEnable == 0)
		virtual_coreId = (preNeuronAddr & NEURON_COREID_BITS) >> NEURON_COREID_SHIFT;
	else
		virtual_coreId = (virtualNeuronAddr & NEURON_COREID_BITS) >> NEURON_COREID_SHIFT;
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
	uint32_t row = neuron_row << SRAM_NEURON_ROW_SHIFT | neuron_column << SRAM_NEURON_COL_SHIFT | synapse_row;
	uint32_t column = SRAM_COL_VALUE;
	bits = virtual_coreId << CXQ_SRAM_VIRTUAL_SOURCE_CORE_SHIFT | sy << CXQ_SRAM_SY_SHIFT | dy << CXQ_SRAM_DY_SHIFT
		| sx << CXQ_SRAM_SX_SHIFT | dx << CXQ_SRAM_DX_SHIFT | dest_coreId << CXQ_SRAM_DEST_CORE_SHIFT |
		CXQ_PROGRAM | coreId << CXQ_PROGRAM_COREID_SHIFT | row << CXQ_PROGRAM_ROW_SHIFT
		| column << CXQ_PROGRAM_COLUMN_SHIFT;
	if (stdp == 0) {
		if (chipId == DYNAPSE_CONFIG_DYNAPSE_U0) {
			if (numConfig_chipU0 == usb_packet_maximum_size) { //DYNAPSE_MAX_USER_USB_PACKET_SIZE
				ConfigureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U0);
			}
			bits_chipU0[numConfig_chipU0] = (int) bits;
			numConfig_chipU0++;
		}
		else if (chipId == DYNAPSE_CONFIG_DYNAPSE_U1) {
			if (numConfig_chipU1 == usb_packet_maximum_size) {
				ConfigureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U1);
			}
			bits_chipU1[numConfig_chipU1] = (int) bits;
			numConfig_chipU1++;
		}
		else if (chipId == DYNAPSE_CONFIG_DYNAPSE_U2) {
			if (numConfig_chipU2 == usb_packet_maximum_size) {
				ConfigureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U2);
			}
			bits_chipU2[numConfig_chipU2] = (int) bits;
			numConfig_chipU2++;
		}
		else if (chipId == DYNAPSE_CONFIG_DYNAPSE_U3) {
			if (numConfig_chipU3 == usb_packet_maximum_size) {
				ConfigureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U3);
			}
			bits_chipU3[numConfig_chipU3] = (int) bits;
			numConfig_chipU3++;
		}
	}
	else {
		caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chipId);
		caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, bits); //this is the 30 bits
	}
	return (true);
}

bool ClearAllCamG(caerModuleData moduleData) {

	GFilterState state = moduleData->moduleState;

	uint32_t neuronId, camId;
	for (neuronId = 0; neuronId < 32 * 32; neuronId++) {
		for (camId = 0; camId < 64; camId++) {
			WriteCamG(moduleData, 0, 1 << 10 | neuronId, 0, camId, 0, 0, 0);
		}
	}

	/*
	 // --- start USB handle / from spike event source id
	 state->eventSourceModuleState = caerMainloopGetSourceState(state->eventSourceID);
	 state->eventSourceConfigNode = caerMainloopGetSourceNode(state->eventSourceID);
	 // --- end USB handle
	 caerInputDynapseState stateSource = state->eventSourceModuleState;

	 caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U0);
	 caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CLEAR_CAM, 0, 0);

	 caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U1);
	 caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CLEAR_CAM, 0, 0);

	 caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U2);
	 caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CLEAR_CAM, 0, 0);

	 caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U3);
	 caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CLEAR_CAM, 0, 0);*/

}

void Shuffle1DArrayG(int64_t *array, int64_t Range) {
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

bool EnableStimuliGenG(caerModuleData moduleData, int32_t pattern) {
	GFilterState state = moduleData->moduleState;

	// --- start USB handle / from spike event source id
	sshsNode inputNode = caerMainloopGetSourceNode(U16T(state->fileInputSourceID));

	if (sshsNodeAttributeExists(inputNode, "filePath", SSHS_STRING)) {
		if (pattern == 0) {
			sshsNodePutString(inputNode, "filePath", PAPER);
		}
		else if (pattern == 1) {
			sshsNodePutString(inputNode, "filePath", ROCK);
		}
		else if (pattern == 2) {
			sshsNodePutString(inputNode, "filePath", SCISSORS);
		}
	}
	else {
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Error input is not from file\n");
	}

	return (true);
}

bool DisableStimuliGenG(caerModuleData moduleData) {
	GFilterState state = moduleData->moduleState;

	// --- start USB handle / from spike event source id
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(state->eventSourceID));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->eventSourceID));
	// --- end USB handle

	sshsNode deviceConfigNodeMain = sshsGetRelativeNode(state->eventSourceConfigNode,
		chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
	sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");
	sshsNodePutBool(spikeNode, "running", true);
	sshsNodePutBool(spikeNode, "doStim", false);
	return (true);
}

bool EnableStimuliGenPrimitiveCamG(caerModuleData moduleData) {
	GFilterState state = moduleData->moduleState;

	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->eventSourceID));

	sshsNode deviceConfigNodeMain = sshsGetRelativeNode(state->eventSourceConfigNode,
		chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
	sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");

	sshsNodePutBool(spikeNode, "doStimPrimitiveCam", true);
	return (true);
}

bool DisableStimuliGenPrimitiveCamG(caerModuleData moduleData) {
	GFilterState state = moduleData->moduleState;

	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->eventSourceID));

	sshsNode deviceConfigNodeMain = sshsGetRelativeNode(state->eventSourceConfigNode,
		chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
	sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");

	sshsNodePutBool(spikeNode, "doStimPrimitiveCam", false);
	return (true);
}

bool EnableTeachingSignalG(caerModuleData moduleData) {
	GFilterState state = moduleData->moduleState;

	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->eventSourceID));

	sshsNode deviceConfigNodeMain = sshsGetRelativeNode(state->eventSourceConfigNode,
		chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
	sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");

	sshsNodePutBool(spikeNode, "sendInhibitoryStimuli", false);
	sshsNodePutBool(spikeNode, "sendTeachingStimuli", true);
	return (true);
}

bool DisableTeachingSignal(caerModuleData moduleData) {
	GFilterState state = moduleData->moduleState;

	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->eventSourceID));

	sshsNode deviceConfigNodeMain = sshsGetRelativeNode(state->eventSourceConfigNode,
		chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
	sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");

	sshsNodePutBool(spikeNode, "sendInhibitoryStimuli", true);
	sshsNodePutBool(spikeNode, "sendTeachingStimuli", false);
	return (true);
}

bool EnableTeachingG(caerModuleData moduleData) {
	GFilterState state = moduleData->moduleState;

	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->eventSourceID));

	sshsNode deviceConfigNodeMain = sshsGetRelativeNode(state->eventSourceConfigNode,
		chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
	sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");

	sshsNodePutBool(spikeNode, "teaching", true);
	return (true);
}

bool DisableTeachingG(caerModuleData moduleData) {
	GFilterState state = moduleData->moduleState;

	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->eventSourceID));

	sshsNode deviceConfigNodeMain = sshsGetRelativeNode(state->eventSourceConfigNode,
		chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
	sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");

	sshsNodePutBool(spikeNode, "teaching", false);
	return (true);
}

bool SetInputLayerCamG(caerModuleData moduleData) {

	int64_t rowId, colId;
	int64_t num = 32 / 2; //DYNAPSE_CONFIG_NUMCAM;
	// generate pattern A
	uint32_t spikePatternA[DYNAPSE_CONFIG_XCHIPSIZE][DYNAPSE_CONFIG_YCHIPSIZE];
	int cx, cy, r;
	cx = 16;
	cy = 16;
	r = 14;
	for (rowId = 0; rowId < DYNAPSE_CONFIG_XCHIPSIZE; rowId++)
		for (colId = 0; colId < DYNAPSE_CONFIG_YCHIPSIZE; colId++)
			spikePatternA[rowId][colId] = 0;
	for (rowId = cx - r; rowId <= cx + r; rowId++)
		for (colId = cy - r; colId <= cy + r; colId++)
			if (((cx - rowId) * (cx - rowId) + (cy - colId) * (cy - colId) <= r * r + sqrt(r))
				&& ((cx - rowId) * (cx - rowId) + (cy - colId) * (cy - colId) >= r * r - r))
				spikePatternA[rowId][colId] = 1;

	uint32_t spikePatternB[DYNAPSE_CONFIG_XCHIPSIZE][DYNAPSE_CONFIG_YCHIPSIZE];
	for (rowId = -num; rowId < num; rowId++) {
		for (colId = -num; colId < num; colId++) {
			if (abs((int) rowId) + abs((int) colId) == num) // Change this condition >= <=
				spikePatternB[rowId + DYNAPSE_CONFIG_XCHIPSIZE / 2][colId + DYNAPSE_CONFIG_YCHIPSIZE / 2] = 1;
			else
				spikePatternB[rowId + DYNAPSE_CONFIG_XCHIPSIZE / 2][colId + DYNAPSE_CONFIG_YCHIPSIZE / 2] = 0;
		}
	}

	uint32_t spikePatternC[DYNAPSE_CONFIG_XCHIPSIZE][DYNAPSE_CONFIG_YCHIPSIZE];
	for (rowId = -num; rowId < num; rowId++) {
		for (colId = -num; colId < num; colId++) {
			if (abs((int) rowId) == abs((int) colId)) // Change this condition
				spikePatternC[rowId + DYNAPSE_CONFIG_XCHIPSIZE / 2][colId + DYNAPSE_CONFIG_YCHIPSIZE / 2] = 1;
			else
				spikePatternC[rowId + DYNAPSE_CONFIG_XCHIPSIZE / 2][colId + DYNAPSE_CONFIG_YCHIPSIZE / 2] = 0;
		}
	}

	uint32_t neuronId;

	for (rowId = 0; rowId < DYNAPSE_CONFIG_XCHIPSIZE; rowId++) {
		for (colId = 0; colId < DYNAPSE_CONFIG_YCHIPSIZE; colId++) {
			neuronId = (uint32_t) (1 << 10 | ((rowId & 0X10) >> 4) << 9 | ((colId & 0X10) >> 4) << 8
				| (rowId & 0xf) << 4 | (colId & 0xf));

			if (pattern_number == 3) {
				if (spikePatternA[rowId][colId] == 1)
					WriteCamG(moduleData, 1, neuronId, 0, 0, 2, 0, 0);
				if (spikePatternB[rowId][colId] == 1)
					WriteCamG(moduleData, 2, neuronId, 0, 1, 2, 0, 0);
				if (spikePatternC[rowId][colId] == 1)
					WriteCamG(moduleData, 3, neuronId, 0, 2, 2, 0, 0);
			}
			else if (pattern_number == 4) {
				if (spikePatternB[rowId][colId] == 1 && rowId <= 16)
					WriteCamG(moduleData, 1, neuronId, 0, 0, 2, 0, 0);
				if (spikePatternB[rowId][colId] == 1 && colId <= 16)
					WriteCamG(moduleData, 2, neuronId, 0, 1, 2, 0, 0);
				if (spikePatternB[rowId][colId] == 1 && rowId >= 16)
					WriteCamG(moduleData, 3, neuronId, 0, 2, 2, 0, 0);
				if (spikePatternB[rowId][colId] == 1 && colId >= 16)
					WriteCamG(moduleData, 4, neuronId, 0, 3, 2, 0, 0);
			}
		}
	}

	if (pattern_number == 3) {
		neuronId = 3 << 10 | 3 << 8 | 0;
		WriteCamG(moduleData, neuronId, neuronId, 0, 62, 2, 0, 0);
		WriteCamG(moduleData, 3 << 8 | 3, neuronId, 0, 63, -2, 0, 0);

		neuronId = 3 << 10 | 3 << 8 | 4;
		WriteCamG(moduleData, neuronId, neuronId, 0, 62, 2, 0, 0);
		WriteCamG(moduleData, 3 << 8 | 3, neuronId, 0, 63, -2, 0, 0);

		neuronId = 3 << 10 | 3 << 8 | 8;
		WriteCamG(moduleData, neuronId, neuronId, 0, 62, 2, 0, 0);
		WriteCamG(moduleData, 3 << 8 | 3, neuronId, 0, 63, -2, 0, 0);
	}
	else if (pattern_number == 4) {
		neuronId = 3 << 10 | 3 << 8 | 0;
		WriteCamG(moduleData, neuronId, neuronId, 0, 62, 2, 0, 0);
		WriteCamG(moduleData, 3 << 8 | 3, neuronId, 0, 63, -2, 0, 0);

		neuronId = 3 << 10 | 3 << 8 | 4;
		WriteCamG(moduleData, neuronId, neuronId, 0, 62, 2, 0, 0);
		WriteCamG(moduleData, 3 << 8 | 3, neuronId, 0, 63, -2, 0, 0);

		neuronId = 3 << 10 | 3 << 8 | 8;
		WriteCamG(moduleData, neuronId, neuronId, 0, 62, 2, 0, 0);
		WriteCamG(moduleData, 3 << 8 | 3, neuronId, 0, 63, -2, 0, 0);

		neuronId = 3 << 10 | 3 << 8 | 12;
		WriteCamG(moduleData, neuronId, neuronId, 0, 62, 2, 0, 0);
		WriteCamG(moduleData, 3 << 8 | 3, neuronId, 0, 63, -2, 0, 0);
	}

	return (true);
}

COLOUR GetColourWG(double v, double vmin, double vmax) {
	COLOUR c = { 0, 0, 0 }; //{65535, 65535, 65535}; // white
	double dv;
	double value;

	if (v < vmin)
		v = vmin;
	if (v > vmax)
		v = vmax;
	dv = vmax - vmin;

	if (v < (vmin + dv / 4)) {
		c.r = 0;
		value = (4 * (v - vmin) / dv) * 65535;
		if (value > 30000)
			c.g = 30000;
		else if (value < 0)
			c.g = 0;
		else
			c.g = (uint16_t) value;
	}
	else if (v < (vmin + dv / 2)) {
		c.r = 0;
		value = (1 + 4 * (vmin + dv / 4 - v) / dv) * 65535;
		if (value > 30000)
			c.b = 30000;
		else if (value < 0)
			c.b = 0;
		else
			c.b = (uint16_t) value;
	}
	else if (v < (vmin + dv * 3 / 4)) {
		c.b = 0;
		value = (4 * (v - vmin - dv / 2) / dv) * 65535;
		if (value > 30000)
			c.r = 30000;
		else if (value < 0)
			c.r = 0;
		else
			c.r = (uint16_t) value;
	}
	else {
		c.b = 0;
		value = (4 * (v - vmin - dv / 2) / dv) * 65535;
		if (value > 30000)
			c.r = 30000;
		else if (value < 0)
			c.r = 0;
		else
			c.r = (uint16_t) value;
	}
	return (c);
}

COLOUR GetColourSG(int v) //, double vmin, double vmax
{
	COLOUR c = { 0, 0, 0 };
	if (v == 0) { //black
		c.r = 0;
		c.g = 0;
		c.b = 0;
	}
	else if (0 < v && v <= 128) { //v <= 128
//		c.r = (uint16_t) ((v & 0x7) * 30);
//		c.g = (uint16_t) (((v & 0x38) >> 3) * 30);
//		c.b = (uint16_t) (((v & 0x1c0) >> 6) * 30);
		// gray-scale plot
		c.r = (uint16_t) (v * 30);
		c.g = (uint16_t) (v * 30);
		c.b = (uint16_t) (v * 30);
	}
	else { //white
		c.r = 255;
		c.g = 255;
		c.b = 255;
	}
	c.r = (uint16_t) (c.r * 257);
	c.g = (uint16_t) (c.g * 257);
	c.b = (uint16_t) (c.b * 257);
	return (c);
}

//set default bias
bool ResetBiasesG(caerModuleData moduleData) {
	GFilterState state = moduleData->moduleState;

	// --- start  usb handle / from spike event source id
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(state->eventSourceID));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->eventSourceID));

	caerInputDynapseState stateSource = state->eventSourceModuleState;
	// --- end usb handle

	uint32_t chipId_t, chipId, coreId;

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
			if (chipId == DYNAPSE_CONFIG_DYNAPSE_U3) {
				setBiasBitsG(moduleData, chipId, coreId, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "IF_AHTHR_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "IF_AHW_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chipId, coreId, "IF_BUF_P", 3, 80, "HighBias", "PBias");
				setBiasBitsG(moduleData, chipId, coreId, "IF_CASC_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "IF_DC_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chipId, coreId, "IF_NMDA_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "IF_RFR_N", 6, 255, "HighBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "IF_TAU1_N", 4, 200, "LowBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "IF_TAU2_N", 6, 15, "HighBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "IF_THR_N", 4, 255, "HighBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "NPDPIE_TAU_F_P", 6, 105, "HighBias", "PBias"); //105
				setBiasBitsG(moduleData, chipId, coreId, "NPDPIE_TAU_S_P", 7, 40, "HighBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "NPDPIE_THR_F_P", 0, 220, "HighBias", "PBias");
				setBiasBitsG(moduleData, chipId, coreId, "NPDPIE_THR_S_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chipId, coreId, "NPDPII_TAU_F_P", 7, 40, "HighBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "NPDPII_TAU_S_P", 7, 40, "HighBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "NPDPII_THR_F_P", 7, 40, "HighBias", "PBias");
				setBiasBitsG(moduleData, chipId, coreId, "NPDPII_THR_S_P", 7, 40, "HighBias", "PBias");
				setBiasBitsG(moduleData, chipId, coreId, "PS_WEIGHT_EXC_F_N", 0, 76, "HighBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "PS_WEIGHT_EXC_S_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "PS_WEIGHT_INH_F_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
				setBiasBitsG(moduleData, chipId, coreId, "R2R_P", 4, 85, "HighBias", "PBias");
			}
			else if (chipId == DYNAPSE_CONFIG_DYNAPSE_U2 || chipId == DYNAPSE_CONFIG_DYNAPSE_U1
				|| chipId == DYNAPSE_CONFIG_DYNAPSE_U0) { // DYNAPSE_CONFIG_DYNAPSE_U2 = 4
				setBiasBitsG(moduleData, chipId, coreId, "IF_AHTAU_N", 4, 2, "LowBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "IF_AHTHR_N", 3, 4, "HighBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "IF_AHW_P", 1, 80, "HighBias", "PBias");
				setBiasBitsG(moduleData, chipId, coreId, "IF_BUF_P", 3, 80, "HighBias", "PBias");
				setBiasBitsG(moduleData, chipId, coreId, "IF_CASC_N", 0, 25, "HighBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "IF_DC_P", 7, 60, "HighBias", "PBias");
				setBiasBitsG(moduleData, chipId, coreId, "IF_NMDA_N", 3, 1, "HighBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "IF_RFR_N", 3, 3, "HighBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "IF_TAU1_N", 4, 18, "LowBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "IF_TAU2_N", 4, 100, "HighBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "IF_THR_N", 2, 166, "HighBias", "NBias"); //0, 200 //3, 150 //4, 40
				setBiasBitsG(moduleData, chipId, coreId, "NPDPIE_TAU_F_P", 5, 40, "HighBias", "PBias"); //70!!!!! //6, 200 //105
				setBiasBitsG(moduleData, chipId, coreId, "NPDPIE_TAU_S_P", 6, 200, "HighBias", "PBias");
				setBiasBitsG(moduleData, chipId, coreId, "NPDPIE_THR_F_P", 1, 80, "HighBias", "PBias");
				setBiasBitsG(moduleData, chipId, coreId, "NPDPIE_THR_S_P", 7, 40, "HighBias", "PBias");
				setBiasBitsG(moduleData, chipId, coreId, "NPDPII_TAU_F_P", 5, 40, "HighBias", "PBias"); //7, 40, "HighBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "NPDPII_TAU_S_P", 7, 1, "HighBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "NPDPII_THR_F_P", 1, 80, "HighBias", "PBias"); //7, 40, "HighBias", "PBias");
				setBiasBitsG(moduleData, chipId, coreId, "NPDPII_THR_S_P", 7, 150, "HighBias", "PBias");
				setBiasBitsG(moduleData, chipId, coreId, "PS_WEIGHT_EXC_F_N", 1, 100, "HighBias", "NBias"); //29!!!
				setBiasBitsG(moduleData, chipId, coreId, "PS_WEIGHT_EXC_S_N", 1, 50, "HighBias", "NBias"); //19!!! //0, 38
				setBiasBitsG(moduleData, chipId, coreId, "PS_WEIGHT_INH_F_N", 0, 250, "HighBias", "NBias"); //7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chipId, coreId, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
				setBiasBitsG(moduleData, chipId, coreId, "R2R_P", 3, 160, "HighBias", "PBias");
			}
		}
	}
	return (true);
}

void setBiasBitsG(caerModuleData moduleData, uint32_t chipId, uint32_t coreId, const char *biasName_t,
	uint8_t coarseValue, uint16_t fineValue, const char *lowHigh, const char *npBias) {
	GFilterState state = moduleData->moduleState;

	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(state->eventSourceID));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->eventSourceID));

	caerInputDynapseState stateSource = state->eventSourceModuleState;

	size_t biasNameLength = strlen(biasName_t);
	char biasName[biasNameLength + 3];

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
	for (i = 0; i < biasNameLength + 3; i++) {
		biasName[3 + i] = biasName_t[i];
	}
	uint32_t bits = generatesBitsCoarseFineBiasSetting(state->eventSourceConfigNode, biasName, coarseValue, fineValue,
		lowHigh, "Normal", npBias, true, chipId);

	caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, bits);
}

void SetTimerG() {
	struct itimerval itv;
	itv.it_interval.tv_sec = 1;
	itv.it_interval.tv_usec = 0;
	itv.it_value.tv_sec = 1;
	itv.it_value.tv_usec = 0;
	setitimer(ITIMER_REAL, &itv, &oldtv);
}

void SignalHandlerG(int m) {
	time_count = (time_count + 1) % 4294967295;
//	printf("%d\n", time_count);
//	printf("%d\n", m);
}

void GetRand1DArrayG(int64_t *array, int64_t Range, int64_t CamNumAvailable) {
	int64_t temp[Range]; //sizeof(array) doesn't work
	int64_t i;
	for (i = 0; i < Range; i++) {
		temp[i] = i;
	}
	Shuffle1DArrayG(temp, Range);
	for (i = 0; i < CamNumAvailable; i++) {
		array[i] = temp[i];
	}
}
void GetRand1DBinaryArrayG(int64_t *binaryArray, int64_t Range, int64_t CamNumAvailable) {
	int64_t array[CamNumAvailable];
	GetRand1DArrayG(array, Range, CamNumAvailable);
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
