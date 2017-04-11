/*
 * Created on: Feb. 2017
 * Author: dongchen@ini.uzh.ch
 */

//learning algorithm: spike queue, normalization, homeostatic
//kernel size 8by8

//disable and enable input cores

//to add to disable reused feature layer 2 neurons

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
	uint16_t event_source_id;
	int file_input_source_id;

	double forward_learning_rate;
	double backward_learning_rate;

	int32_t max_synapse_feature_layer1;
	int32_t max_synapse_output_layer;

	int8_t reset;

	int8_t resetExType;
	int8_t resetInType;

	int32_t colorscaleMax;
	int32_t colorscaleMin;

	bool stimulate;
	bool learning;

	bool learning_feature1;
	bool learning_feature2;
	bool learning_feature3;
	bool learning_output;
	bool learning_output_phase2;

	bool teaching;
	bool inject;
};

struct GFilter_memory {
	//connection between neurons
	simple2DBufferInt connection_map;
	simple2DBufferDouble weight_map;
	simple2DBufferDouble weight_map_temp;
	simple2DBufferInt rounded_weight_map;
	simple2DBufferInt synapse_map;
	simple2DBufferInt synapse_cam_map;

	//CAM state
	simple2DBufferInt cam_map;
	simple2DBufferInt cam_map_size;
	simple2DBufferInt cam_map_content_source;
	simple2DBufferInt cam_map_content_type;

	//SRAM state
	simple2DBufferInt sram_map;
	simple2DBufferInt sram_map_content;

	//filter state
	simple2DBufferInt filter_map_addr;
	simple2DBufferInt filter_map_cam_id;
	simple2DBufferInt filter_map_size;
	simple2DBufferDouble filter_map_highest;
	simple2DBufferDouble filter_map_lowest;

	//spike queue
	simple2DBufferLong spike_fifo;
	uint64_t spike_cnt;
	uint64_t post_rd_pointer;
	uint64_t wr_pointer;

	//learning algorithm
	uint64_t package_spike_counter;
	uint64_t pre_rd_pointer_old;

	//for output layer 1
	simple2DBufferInt outputMap;
	simple2DBufferInt outputMapDisabled;
};

double deltaWeights[DELTA_WEIGHT_LUT_LENGTH];
double synapseUpgradeThreshold[SYNAPSE_UPGRADE_THRESHOLD_LUT_LENGTH];

typedef struct {
	uint16_t r, g, b;
} COLOUR;

typedef struct GFilter_state *GFilterState;
typedef struct GFilter_memory GFilterMemory;

static GFilterMemory memory;
static int64_t time_cnt = 0;
static int64_t time_cnt_last = 0;
static int64_t pattern_period;
static int32_t stimuli_pattern;
static struct itimerval oldtv;
static int8_t stimdisabled = 0;
static int pattern_style = 5; //3;//5; //3 or 4 only thing to change

static int n;

static int teaching;

static int output_bias_reseted;

static int feature_bias_reseted = 1;

static int usb_packet_maximum_size; //10 //1366; //1024

static int file_input_enable = 1; //1;

static int iteration = 0;

//for sending USB packages
static uint32_t bits_chip_U0[DYNAPSE_MAX_USER_USB_PACKET_SIZE];
static uint32_t bits_chip_U1[DYNAPSE_MAX_USER_USB_PACKET_SIZE];
static uint32_t bits_chip_U2[DYNAPSE_MAX_USER_USB_PACKET_SIZE];
static uint32_t bits_chip_U3[DYNAPSE_MAX_USER_USB_PACKET_SIZE];
static int num_config_chip_U0 = 0;
static int num_config_chip_U1 = 0;
static int num_config_chip_U2 = 0;
static int num_config_chip_U3 = 0;


//create input layer
uint32_t input_layer[INPUT_LAYER_N];
uint32_t anti_input_layer[INPUT_LAYER_N];

//create feature layer 1
uint32_t feature_layer1[FEATURE_LAYER1_N*FEATURE_LAYER1_NUM];
uint32_t anti_feature_layer1[TOTAL_NEURON_NUM_ON_CHIP];

//create feature layer 2
uint32_t feature_layer2[FEATURE_LAYER2_N*FEATURE_LAYER2_NUM];
uint32_t anti_feature_layer2[TOTAL_NEURON_NUM_ON_CHIP];

//create feature layer 3
uint32_t feature_layer3[FEATURE_LAYER3_N * FEATURE_LAYER3_NUM];
uint32_t anti_feature_layer3[TOTAL_NEURON_NUM_ON_CHIP];

//create output layer
uint32_t output_layer[OUTPUT_LAYER_N * OUTPUT_LAYER_NUM];
uint32_t anti_output_layer[TOTAL_NEURON_NUM_ON_CHIP];

static bool caerGestureLearningFilterInit(caerModuleData moduleData); //It may not run at the beginning of the experiment ????????????
static void caerGestureLearningFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerGestureLearningFilterConfig(caerModuleData moduleData);
static void caerGestureLearningFilterExit(caerModuleData moduleData);
static void caerGestureLearningFilterReset(caerModuleData moduleData, int resetCallSourceID);


static int64_t getNeuronId(int64_t chip_id, int64_t row_id, int64_t col_id);
static int64_t encodeInputLayerNeuronAddress(int64_t neuron_address);
static int64_t decodeInputLayerNeuronAddress(int64_t neuron_address);

static double forwardUpdateWeightG(caerModuleData moduleData, int64_t pre_neuron_addr, int64_t post_neuron_addr, double forwardDeltaWeight);
static void normalizeIncreasedWeightG(caerModuleData moduleData, int64_t pre_neuron_addr, double increased_delta_weight);
static void roundWeightG(caerModuleData moduleData, int64_t post_neuron_addr);
static void updateSynapseG(caerModuleData moduleData, int64_t post_neuron_addr,
		caerFrameEventPacket *synapsePlotInputFeature1, caerFrameEventPacket *synapsePlotFeature1Feature2,
		caerFrameEventPacket *synapsePlotFeature2Feature3, caerFrameEventPacket *synapsePlotFeature3Output);
static void modifySynapse(caerModuleData moduleData, int64_t pre_neuron_addr, int64_t post_neuron_addr, int32_t synapse_type,
		caerFrameEventPacket *synapsePlotInputFeature1, caerFrameEventPacket *synapsePlotFeature1Feature2,
		caerFrameEventPacket *synapsePlotFeature2Feature3, caerFrameEventPacket *synapsePlotFeature3Output);
static void modifyPotentialSynapse(caerModuleData moduleData, int64_t pre_neuron_addr, int64_t post_neuron_addr,
		int32_t synapse_type, int32_t cam_id,
		caerFrameEventPacket *synapsePlotInputFeature1, caerFrameEventPacket *synapsePlotFeature1Feature2,
		caerFrameEventPacket *synapsePlotFeature2Feature3, caerFrameEventPacket *synapsePlotFeature3Output);
static int32_t decodeSynapseType(int32_t synapse_type);

static void updateSynapsePlotG(int64_t pre_neuron_addr, int64_t post_neuron_addr, int32_t new_synapse,
		caerFrameEventPacket *synapsePlotInputFeature1, caerFrameEventPacket *synapsePlotFeature1Feature2,
		caerFrameEventPacket *synapsePlotFeature2Feature3, caerFrameEventPacket *synapsePlotFeature3Output);

COLOUR getColourWG(double v, double vmin, double vmax);
COLOUR getColourSG(float v);

static bool resetNetworkG(caerModuleData moduleData);
static void resetBiasesG(caerModuleData moduleData);
static void resetBiasesGDC(caerModuleData moduleData);
static void setBiasBitsG(caerModuleData moduleData, uint32_t chip_id, uint32_t core_id, const char *biasName_t,
		uint8_t coarseValue, uint16_t fineValue, const char *lowHigh, const char *npBias);

static void setInputLayerCamG(caerModuleData moduleData);
static void setFileInputLayerCamG(caerModuleData moduleData);
static void setOutputLayerCamG(caerModuleData moduleData);
static void clearAllCam(caerModuleData moduleData);

static void buildSynapseG(caerModuleData moduleData, uint32_t pre_neuron_addr, uint32_t post_neuron_addr,
		uint32_t virtual_neuron_addr, int32_t synapse_type, int8_t realOrVirtualSynapse, int8_t virtual_neuron_addr_enable,
		uint32_t cam_id_search_start);

static bool saveNeuronForTeaching(uint32_t pre_neuron_addr, uint32_t post_neuron_addr);

static uint32_t getWriteCamChipIdG(uint32_t post_neuron_addr);
static uint32_t getWriteCamBitsG(uint32_t pre_neuron_addr, uint32_t post_neuron_addr,
		uint32_t virtual_neuron_addr, uint32_t cam_id, int32_t synapse_type, int8_t virtual_neuron_addr_enable);
static void writeCamG(caerModuleData moduleData, uint32_t pre_neuron_addr, uint32_t post_neuron_addr,
		uint32_t virtual_neuron_addr, uint32_t cam_id, int32_t synapse_type, int8_t virtual_neuron_addr_enable, int8_t stdp);
static void writeSramG(caerModuleData moduleData, uint32_t pre_neuron_addr, uint32_t chip_core_id,
		uint32_t virtual_neuron_addr, uint32_t sram_id, int8_t virtual_neuron_addr_enable, int8_t stdp);

static bool configureChipG(caerModuleData moduleData, int8_t chip_id);

static void shuffle1DArrayG(int64_t *array, int64_t Range);
static void getRand1DArrayG(int64_t *array, int64_t Range, int64_t available_cam_num);
static void getRand1DBinaryArrayG(int64_t *binaryArray, int64_t Range, int64_t available_cam_num);
static void setTimerG(void);
static void signalHandlerG(int m);

static bool enableStimuliGenG(caerModuleData moduleData, int32_t pattern);
static bool disableStimuliGenG(caerModuleData moduleData);
static void enableStimuliGenPrimitiveCamG(caerModuleData moduleData);
static void disableStimuliGenPrimitiveCamG(caerModuleData moduleData);

static void enableTeachingSignalG(caerModuleData moduleData);
static void enableTeachingG(caerModuleData moduleData);
static void disableTeachingG(caerModuleData moduleData);

static void enableFileStimuliRunG(caerModuleData moduleData);
static void enableFileStimuliG(caerModuleData moduleData, int32_t pattern);
static void disableFileStimuliG(caerModuleData moduleData);

static void enableTeachingStimuliGenG(caerModuleData moduleData, int32_t pattern);
static void disableTeachingStimuliGenG(caerModuleData moduleData);

static void disableOutputCores(caerModuleData moduleData, int32_t pattern);
static void disableAllOutputCores(caerModuleData moduleData);
static void enableAllOutputCores(caerModuleData moduleData);
static void disableOutputCore(caerModuleData moduleData);
static void enableOutputCore(caerModuleData moduleData);

static void disableInputChip(caerModuleData moduleData);
static void enableInputChip(caerModuleData moduleData);

static void setTestingBias(caerModuleData moduleData);

static void injectCurrent(caerModuleData moduleData, int32_t inject, int32_t layer);

static struct caer_module_functions caerGestureLearningFilterFunctions = { .moduleInit = &caerGestureLearningFilterInit,
	.moduleRun = &caerGestureLearningFilterRun, .moduleConfig = &caerGestureLearningFilterConfig, .moduleExit =
		&caerGestureLearningFilterExit, .moduleReset = &caerGestureLearningFilterReset };

void caerGestureLearningFilter(uint16_t moduleID, int fileInputID, caerSpikeEventPacket spike,
	caerFrameEventPacket *synapsePlotInputFeature1,
	caerFrameEventPacket *synapsePlotFeature1Feature2,
	caerFrameEventPacket *synapsePlotFeature2Feature3,
	caerFrameEventPacket *synapsePlotFeature3Output) { //used now
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "GFilter", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}
	caerModuleSM(&caerGestureLearningFilterFunctions, moduleData, sizeof(struct GFilter_state), 7, fileInputID, spike,
			synapsePlotInputFeature1,
			synapsePlotFeature1Feature2,
			synapsePlotFeature2Feature3,
			synapsePlotFeature3Output);
}

static bool caerGestureLearningFilterInit(caerModuleData moduleData) {
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "colorscaleMax", VMAX); //500
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "colorscaleMin", VMIN);
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "reset", 0);
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "resetExType", 1); //1 //2 for test, should be 1
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "resetInType", 1); //1
	sshsNodePutDoubleIfAbsent(moduleData->moduleNode, "learningRateForward", FORWARD_LEARNING_RATE); //40 good //20 //5
	sshsNodePutDoubleIfAbsent(moduleData->moduleNode, "learningRateBackward", BACKWARD_LEARNING_RATE); //2 //0 //2
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "stimulate", true);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "learning", true); //true
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "teaching", true); //true
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "inject", true); //true
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "learning_feature1", true); //true //false
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "learning_feature2", true); //true //false
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "learning_feature3", true); //false
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "learning_output", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "learning_output_phase2", true); //false
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "maxSynapseFeature", 55); //3); //128); //500
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "maxSynapseOutput", 55);

	GFilterState state = moduleData->moduleState;
	state->reset = sshsNodeGetByte(moduleData->moduleNode, "reset");
	state->resetExType = sshsNodeGetByte(moduleData->moduleNode, "resetExType");
	state->resetInType = sshsNodeGetByte(moduleData->moduleNode, "resetInType");
	state->stimulate = sshsNodeGetBool(moduleData->moduleNode, "stimulate");
	state->learning = sshsNodeGetBool(moduleData->moduleNode, "learning");
	state->teaching = sshsNodeGetBool(moduleData->moduleNode, "teaching");
	state->inject = sshsNodeGetBool(moduleData->moduleNode, "inject");
	state->learning_feature1 = sshsNodeGetBool(moduleData->moduleNode, "learning_feature1");
	state->learning_feature2 = sshsNodeGetBool(moduleData->moduleNode, "learning_feature2");
	state->learning_feature3 = sshsNodeGetBool(moduleData->moduleNode, "learning_feature3");
	state->learning_output = sshsNodeGetBool(moduleData->moduleNode, "learning_output");
	state->learning_output_phase2 = sshsNodeGetBool(moduleData->moduleNode, "learning_output_phase2");
	state->max_synapse_feature_layer1 = sshsNodeGetInt(moduleData->moduleNode, "maxSynapseFeature");
	state->max_synapse_output_layer = sshsNodeGetInt(moduleData->moduleNode, "maxSynapseOutput");

	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	if (!sshsNodeAttributeExists(sourceInfoNode, "apsSizeX", SSHS_SHORT)) {
		sshsNodePutShort(sourceInfoNode, "apsSizeX", VISUALIZER_HEIGHT_FEATURE); //DYNAPSE_X4BOARD_NEUY
		sshsNodePutShort(sourceInfoNode, "apsSizeY", VISUALIZER_WIDTH_FEATURE); //DYNAPSE_X4BOARD_NEUY
	}

	state->event_source_id = 0;
	state->file_input_source_id = 0;

	return (true); // Nothing that can fail here.
}

static void caerGestureLearningFilterConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);
	GFilterState state = moduleData->moduleState;
	state->colorscaleMax = sshsNodeGetInt(moduleData->moduleNode, "colorscaleMax");
	state->colorscaleMin = sshsNodeGetInt(moduleData->moduleNode, "colorscaleMin");
	state->reset = sshsNodeGetByte(moduleData->moduleNode, "reset");
	state->resetExType = sshsNodeGetByte(moduleData->moduleNode, "resetExType");
	state->resetInType = sshsNodeGetByte(moduleData->moduleNode, "resetInType");
	state->forward_learning_rate = sshsNodeGetDouble(moduleData->moduleNode, "learningRateForward");
	state->backward_learning_rate = sshsNodeGetDouble(moduleData->moduleNode, "learningRateBackward");
	state->stimulate = sshsNodeGetBool(moduleData->moduleNode, "stimulate");
	state->learning = sshsNodeGetBool(moduleData->moduleNode, "learning");
	state->teaching = sshsNodeGetBool(moduleData->moduleNode, "teaching");
	state->inject = sshsNodeGetBool(moduleData->moduleNode, "inject");
	state->learning_feature1 = sshsNodeGetBool(moduleData->moduleNode, "learning_feature1");
	state->learning_feature2 = sshsNodeGetBool(moduleData->moduleNode, "learning_feature2");
	state->learning_feature3 = sshsNodeGetBool(moduleData->moduleNode, "learning_feature3");
	state->learning_output = sshsNodeGetBool(moduleData->moduleNode, "learning_output");
	state->learning_output_phase2 = sshsNodeGetBool(moduleData->moduleNode, "learning_output_phase2");
	state->max_synapse_feature_layer1 = sshsNodeGetInt(moduleData->moduleNode, "maxSynapseFeature");
	state->max_synapse_output_layer = sshsNodeGetInt(moduleData->moduleNode, "maxSynapseOutput");
}

static void caerGestureLearningFilterExit(caerModuleData moduleData) {
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	GFilterState state = moduleData->moduleState;

	//get USB handle from spike event source id
	state->eventSourceModuleState = caerMainloopGetSourceState(state->event_source_id);
	state->eventSourceConfigNode = caerMainloopGetSourceNode(state->event_source_id);

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
}

static void caerGestureLearningFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	int fileInputId = va_arg(args, int);
	caerSpikeEventPacket spike = va_arg(args, caerSpikeEventPacket);
	caerFrameEventPacket *synapsePlotInputFeature1 = va_arg(args, caerFrameEventPacket*);
	caerFrameEventPacket *synapsePlotFeature1Feature2 = va_arg(args, caerFrameEventPacket*);
	caerFrameEventPacket *synapsePlotFeature2Feature3 = va_arg(args, caerFrameEventPacket*);
	caerFrameEventPacket *synapsePlotFeature3Output = va_arg(args, caerFrameEventPacket*);

	GFilterState state = moduleData->moduleState;

	if (state->event_source_id == 0) {
		if (spike != NULL)
			state->event_source_id = (uint16_t) caerEventPacketHeaderGetEventSource(&spike->packetHeader);
		else
			state->event_source_id = EVENTSOURCEID;
	}
	//if(state->file_input_source_id == NULL){
	state->file_input_source_id = fileInputId;
	//}

	uint32_t counterS;
	COLOUR col_s;
	uint16_t sizeX = VISUALIZER_X_FEATURE;
	uint16_t sizeY = VISUALIZER_Y_FEATURE;
	uint16_t visualizer_height = VISUALIZER_HEIGHT_FEATURE;
	uint16_t visualizer_width = VISUALIZER_WIDTH_FEATURE;

	if (memory.synapse_map == NULL) {

		caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Initialize gesture learning filter..");
		int64_t i, j, ys, filter_row_id, filter_col_id, feature_map_row_id, feature_map_col_id, feature_row_id, feature_col_id;
		//initialize lookup tables
		for (i = 0; i < DELTA_WEIGHT_LUT_LENGTH; i++) {
			deltaWeights[i] = exp((double) i / 35) * 10;
		}
		for (i = 0; i < SYNAPSE_UPGRADE_THRESHOLD_LUT_LENGTH; i++) {
			synapseUpgradeThreshold[i] = exp((double) i / 1000); //i //i/2 i/100 i/10 too small //i 0 //1; //exp( (double) i/1000);
		}

		output_bias_reseted = 0;

		usb_packet_maximum_size = USB_PACKET_MAXIMUM_SIZE_INITIALIZATION;

		caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Programming..");
		if (!resetNetworkG(moduleData)) { // Failed to allocate memory, nothing to do.
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for synapse_map.");
			return;
		}

		usb_packet_maximum_size = USB_PACKET_MAXIMUM_SIZE_LEARNING;

		caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Done");
		float warrayS[visualizer_height][visualizer_width];

		//for feature layer 1
		sizeX = VISUALIZER_X_FEATURE1;
		sizeY = VISUALIZER_Y_FEATURE1;
		for (i = 0; i < visualizer_width; i++)
			for (j = 0; j < visualizer_width; j++)
				warrayS[i][j] = 0;
		*synapsePlotInputFeature1 = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, visualizer_height, visualizer_width, 3); //put info into frame
		if (*synapsePlotInputFeature1 != NULL) {
			caerFrameEvent singleplotS = caerFrameEventPacketGetEvent(*synapsePlotInputFeature1, 0);
			//for feature maps
			for (i = 0; i < INPUT_LAYER_N; i++) {
				for (j = 0; j < FEATURE_LAYER1_N * FEATURE_LAYER1_NUM; j++) {
					int pre_id, post_id;
					int64_t pre_addr, post_addr;
					pre_addr = input_layer[i];
					post_addr = feature_layer1[j];
					pre_id = i & 0x3ff;
					post_id = j & 0x3ff;
					if ((int)(pre_id/INPUT_LAYER_W) >= ((int)(post_id/28)%FEATURE_LAYER1_L)*FILTER1_STEP_L
							&& (int)(pre_id/INPUT_LAYER_W) < ((int)(post_id/28)%FEATURE_LAYER1_L)*FILTER1_STEP_L + FILTER1_L
							&& pre_id%INPUT_LAYER_W >= ((post_id%28)%FEATURE_LAYER1_W)*FILTER1_STEP_W
							&& pre_id%INPUT_LAYER_W < ((post_id%28)%FEATURE_LAYER1_W)*FILTER1_STEP_W + FILTER1_W)
					{

						filter_row_id = 1 + ((int)(post_id/28)%FEATURE_LAYER1_W) * (FILTER1_L+1) +
								(int)(pre_id/INPUT_LAYER_W)-((int)(post_id/28)%FEATURE_LAYER1_L)*FILTER1_STEP_L;
						filter_col_id = 1 + ((post_id%28)%FEATURE_LAYER1_W) * (FILTER1_W+1) +
								(pre_id%INPUT_LAYER_W) - ((post_id%28)%FEATURE_LAYER1_W)*FILTER1_STEP_W;

						feature_row_id = (int)((int)(post_id/28) / FEATURE_LAYER1_L) * VISUALIZER_FEATURE1_MAP_X;
						feature_col_id = (int)((post_id%28) / FEATURE_LAYER1_W) * VISUALIZER_FEATURE1_MAP_Y;

						warrayS[filter_row_id + feature_row_id][filter_col_id + feature_col_id]
							= (float) memory.synapse_map->buffer2d[pre_addr-MEMORY_NEURON_ADDR_OFFSET][post_addr-MEMORY_NEURON_ADDR_OFFSET];

					}
				}
			}
			for (i = 0; i < sizeX; i++) {
				for (j = 0; j < sizeY; j++) {
					if (i%VISUALIZER_FEATURE1_MAP_X == VISUALIZER_FEATURE1_MAP_X-1 || j%VISUALIZER_FEATURE1_MAP_Y == VISUALIZER_FEATURE1_MAP_Y-1)
						warrayS[i][j] = 0;
					else if (i%VISUALIZER_FEATURE1_MAP_X == VISUALIZER_FEATURE1_MAP_X-2 || j%VISUALIZER_FEATURE1_MAP_Y == VISUALIZER_FEATURE1_MAP_Y-2)
						warrayS[i][j] = WHITE_EDGE_COLOR_VALUE;
					else if ((i % VISUALIZER_FEATURE1_MAP_X) % (FILTER1_W+1) == 0 ||
							(j % VISUALIZER_FEATURE1_MAP_Y) % (FILTER1_L+1) == 0)
						warrayS[i][j] = WHITE_EDGE_COLOR_VALUE;
				}
			}
			counterS = 0;
			for (i = 0; i < visualizer_height; i++) {
				for (ys = 0; ys < visualizer_width; ys++) {
					col_s  = getColourSG(warrayS[ys][i]);
					singleplotS->pixels[counterS] = col_s.r;
					singleplotS->pixels[counterS + 1] = col_s.g;
					singleplotS->pixels[counterS + 2] = col_s.b;
					counterS += 3;
				}
			}
			caerFrameEventSetLengthXLengthYChannelNumber(singleplotS, visualizer_height, visualizer_width, 3, *synapsePlotInputFeature1); //add info to the frame
			caerFrameEventValidate(singleplotS, *synapsePlotInputFeature1); //validate frame
		}

		//for feature layer 2
		sizeX = VISUALIZER_X_FEATURE2;
		sizeY = VISUALIZER_Y_FEATURE2;
		visualizer_height = VISUALIZER_HEIGHT_FEATURE2;
		visualizer_width = VISUALIZER_WIDTH_FEATURE2;
		for (i = 0; i < visualizer_height; i++)
			for (j = 0; j < visualizer_width; j++)
				warrayS[i][j] = 0;
		*synapsePlotFeature1Feature2 = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, visualizer_height, visualizer_width, 3); //put info into frame
		if (*synapsePlotFeature1Feature2 != NULL) {
			caerFrameEvent singleplotS = caerFrameEventPacketGetEvent(*synapsePlotFeature1Feature2, 0);
			//for feature maps
			for (i = 0; i < FEATURE_LAYER1_N * FEATURE_LAYER1_NUM; i++) {
				for (j = 0; j < FEATURE_LAYER2_N * FEATURE_LAYER2_NUM; j++) {
					int pre_id, post_id;
					int64_t pre_addr, post_addr;
					pre_addr = feature_layer1[i];
					post_addr = feature_layer2[j];
					pre_id = i & 0x3ff;
					post_id = j & 0x3ff;
					if ((int)(pre_id/28) % 7 >= ((int)(post_id/30)%FEATURE_LAYER2_L)*FILTER2_STEP_L
							&& (int)(pre_id/28) % 7 < ((int)(post_id/30)%FEATURE_LAYER2_L)*FILTER2_STEP_L + FILTER2_L
							&& (pre_id%28) % 7 >= ((post_id%30)%FEATURE_LAYER2_W)*FILTER2_STEP_W
							&& (pre_id%28) % 7 < ((post_id%30)%FEATURE_LAYER2_W)*FILTER2_STEP_W + FILTER2_W)
					{

						filter_row_id = 1 + (int)((int)(pre_id/28) / FEATURE_LAYER1_L) * (FILTER2_L+1) +
								(int)(pre_id/28)%7-((int)(post_id/30)%FEATURE_LAYER2_L)*FILTER2_STEP_L;
						filter_col_id = 1 + (int)((pre_id%28) / FEATURE_LAYER1_W) * (FILTER2_W+1) +
								(pre_id%28)%7 - ((post_id%30)%FEATURE_LAYER2_W)*FILTER2_STEP_W;

						feature_map_row_id = ((int)(post_id/30) % FEATURE_LAYER2_L) * VISUALIZER_FEATURE2_MAP_X;
						feature_map_col_id = (int)((post_id%30) % FEATURE_LAYER2_W) * VISUALIZER_FEATURE2_MAP_Y;

						int feature_maps_row_id = (int)((int)(post_id/30) / FEATURE_LAYER2_L) * VISUALIZER_FEATURE2_MAPS_X;
						int feature_maps_col_id = (int)((post_id%30) / FEATURE_LAYER2_W) * VISUALIZER_FEATURE2_MAPS_Y;

						warrayS[filter_row_id + feature_map_row_id + feature_maps_row_id][filter_col_id + feature_map_col_id + feature_maps_col_id]
							= (float) memory.synapse_map->buffer2d[pre_addr-MEMORY_NEURON_ADDR_OFFSET][post_addr-MEMORY_NEURON_ADDR_OFFSET];

					}
				}
			}
			for (i = 0; i < sizeX; i++) {
				for (j = 0; j < sizeY; j++) {

					if (i % VISUALIZER_FEATURE2_MAPS_X == VISUALIZER_FEATURE2_MAPS_X-1 ||
							j % VISUALIZER_FEATURE2_MAPS_Y == VISUALIZER_FEATURE2_MAPS_Y-1)
						warrayS[i][j] = 0;

					else if ((i%VISUALIZER_FEATURE2_MAPS_X) % VISUALIZER_FEATURE2_MAP_X == VISUALIZER_FEATURE2_MAP_X-1 ||
							(j%VISUALIZER_FEATURE2_MAPS_Y) % VISUALIZER_FEATURE2_MAP_Y == VISUALIZER_FEATURE2_MAP_X-1)
						warrayS[i][j] = 0;

					else if ((i%VISUALIZER_FEATURE2_MAPS_X) % VISUALIZER_FEATURE2_MAP_X == VISUALIZER_FEATURE2_MAP_X-2 ||
							(j%VISUALIZER_FEATURE2_MAPS_Y) % VISUALIZER_FEATURE2_MAP_Y == VISUALIZER_FEATURE2_MAP_X-2)
						warrayS[i][j] = WHITE_EDGE_COLOR_VALUE;

					else if (((i%VISUALIZER_FEATURE2_MAPS_X) % VISUALIZER_FEATURE2_MAP_X) % (FILTER2_W+1) == 0 ||
							((j%VISUALIZER_FEATURE2_MAPS_Y) % VISUALIZER_FEATURE2_MAP_Y) % (FILTER2_L+1) == 0)
						warrayS[i][j] = WHITE_EDGE_COLOR_VALUE;

				}
			}
			counterS = 0;
			for (i = 0; i < visualizer_height; i++) {
				for (ys = 0; ys < visualizer_width; ys++) {
					col_s  = getColourSG(warrayS[ys][i]);
					singleplotS->pixels[counterS] = col_s.r;
					singleplotS->pixels[counterS + 1] = col_s.g;
					singleplotS->pixels[counterS + 2] = col_s.b;
					counterS += 3;
				}
			}
			caerFrameEventSetLengthXLengthYChannelNumber(singleplotS, visualizer_height, visualizer_width, 3, *synapsePlotFeature1Feature2); //add info to the frame
			caerFrameEventValidate(singleplotS, *synapsePlotFeature1Feature2); //validate frame
		}

		//for feature layer 3
		sizeX = VISUALIZER_X_FEATURE3;
		sizeY = VISUALIZER_Y_FEATURE3;
		visualizer_height = VISUALIZER_HEIGHT_FEATURE3;
		visualizer_width = VISUALIZER_WIDTH_FEATURE3;
		for (i = 0; i < visualizer_height; i++)
			for (j = 0; j < visualizer_width; j++)
				warrayS[i][j] = 0;
		*synapsePlotFeature2Feature3 = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, visualizer_height, visualizer_width, 3);
		if (*synapsePlotFeature2Feature3 != NULL) {
			caerFrameEvent singleplotS = caerFrameEventPacketGetEvent(*synapsePlotFeature2Feature3, 0);
			int64_t pre_addr, post_addr;

			for (j = 0; j < FEATURE_LAYER3_N * FEATURE_LAYER3_NUM; j++) {
				post_addr = feature_layer3[j];
				for (i = 0; i < TOTAL_CAM_NUM_LEARNING_F3; i++) {
					int64_t pre_id, post_id;

					pre_addr = memory.filter_map_addr->buffer2d[post_addr-MEMORY_NEURON_ADDR_OFFSET][i];

					pre_id = i;
					post_id = j;

					filter_row_id = 1 + (int)(pre_id / 8) + ((int)((post_id%256)/16)%8)*(8+1); //
					filter_col_id = 1 + pre_id % 8 + (((post_id%256)%16)%8)*(8+1); //

					feature_map_row_id = ((int)(post_id/256) >> 1) * VISUALIZER_FEATURE3_MAP_X;
					feature_map_col_id = ((int)(post_id/256) & 0x1) * VISUALIZER_FEATURE3_MAP_Y;

					int feature_maps_row_id = (int)((int)((post_id%256)/16)/8) * VISUALIZER_FEATURE3_MAPS_X;
					int feature_maps_col_id = (int)(((post_id%256)%16)/8) * VISUALIZER_FEATURE3_MAPS_Y;

					warrayS[filter_row_id + feature_map_row_id + feature_maps_row_id][filter_col_id + feature_map_col_id + feature_maps_col_id]
						= (float) memory.synapse_map->buffer2d[pre_addr-MEMORY_NEURON_ADDR_OFFSET][post_addr-MEMORY_NEURON_ADDR_OFFSET];
				}
			}

			for (i = 0; i < sizeX; i++) {
				for (j = 0; j < sizeY; j++) {

					if (i % VISUALIZER_FEATURE3_MAPS_X == VISUALIZER_FEATURE3_MAPS_X-1 ||
							j % VISUALIZER_FEATURE3_MAPS_Y == VISUALIZER_FEATURE3_MAPS_Y-1)
						warrayS[i][j] = 0;

					else if ((i%VISUALIZER_FEATURE3_MAPS_X) % VISUALIZER_FEATURE3_MAP_X == VISUALIZER_FEATURE3_MAP_X-1 ||
							(j%VISUALIZER_FEATURE3_MAPS_Y) % VISUALIZER_FEATURE3_MAP_Y == VISUALIZER_FEATURE3_MAP_X-1)
						warrayS[i][j] = 0;

					else if ((i%VISUALIZER_FEATURE3_MAPS_X) % VISUALIZER_FEATURE3_MAP_X == VISUALIZER_FEATURE3_MAP_X-2 ||
							(j%VISUALIZER_FEATURE3_MAPS_Y) % VISUALIZER_FEATURE3_MAP_Y == VISUALIZER_FEATURE3_MAP_X-2)
						warrayS[i][j] = WHITE_EDGE_COLOR_VALUE;

					else if (((i%VISUALIZER_FEATURE3_MAPS_X) % VISUALIZER_FEATURE3_MAP_X) % (8+1) == 0 ||
							((j%VISUALIZER_FEATURE3_MAPS_Y) % VISUALIZER_FEATURE3_MAP_Y) % (8+1) == 0)
						warrayS[i][j] = WHITE_EDGE_COLOR_VALUE;

				}
			}
			counterS = 0;
			for (i = 0; i < visualizer_height; i++) {
				for (ys = 0; ys < visualizer_width; ys++) {
					col_s  = getColourSG(warrayS[ys][i]);
					singleplotS->pixels[counterS] = col_s.r;
					singleplotS->pixels[counterS + 1] = col_s.g;
					singleplotS->pixels[counterS + 2] = col_s.b;
					counterS += 3;
				}
			}
			caerFrameEventSetLengthXLengthYChannelNumber(singleplotS, visualizer_height, visualizer_width, 3, *synapsePlotFeature2Feature3); //add info to the frame
			caerFrameEventValidate(singleplotS, *synapsePlotFeature2Feature3); //validate frame

		}

		//for output layer
		sizeX = VISUALIZER_X_OUTPUT;
		sizeY = VISUALIZER_Y_OUTPUT;
		visualizer_height = VISUALIZER_HEIGHT_OUTPUT;
		visualizer_width = VISUALIZER_WIDTH_OUTPUT;
		for (i = 0; i < visualizer_height; i++)
			for (j = 0; j < visualizer_width; j++)
				warrayS[i][j] = 0;
		*synapsePlotFeature3Output = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, visualizer_height, visualizer_width, 3);
		if (*synapsePlotFeature3Output != NULL) {
			caerFrameEvent singleplotS = caerFrameEventPacketGetEvent(*synapsePlotFeature3Output, 0);
			int64_t pre_addr, post_addr;

			for (j = 0; j < OUTPUT_LAYER_N * OUTPUT_LAYER_NUM; j++) {
				post_addr = output_layer[j];
				for (i = 0; i < FEATURE_LAYER3_N * FEATURE_LAYER3_NUM; i++) {
					int64_t pre_id, post_id;

					pre_addr = feature_layer3[i];

					pre_id = i;
					post_id = j;

					filter_row_id = 1 + (int)((pre_id%256)/16) + ((int)(pre_id/256) >> 1) * (16+1);
					filter_col_id = 1 + (pre_id%256)%16 + ((int)(pre_id/256) & 0x1) * (16+1);

					feature_map_row_id = ((int)(post_id/4)%2) * VISUALIZER_OUTPUT_MAP_X;
					feature_map_col_id = ((post_id%4)%2) * VISUALIZER_OUTPUT_MAP_Y;

					int feature_maps_row_id = (int)((int)(post_id/4)/2) * VISUALIZER_OUTPUT_MAPS_X;
					int feature_maps_col_id = (int)((post_id%4)/2) * VISUALIZER_OUTPUT_MAPS_Y;

					warrayS[filter_row_id + feature_map_row_id + feature_maps_row_id][filter_col_id + feature_map_col_id + feature_maps_col_id]
						= (float) memory.synapse_map->buffer2d[pre_addr-MEMORY_NEURON_ADDR_OFFSET][post_addr-MEMORY_NEURON_ADDR_OFFSET];
				}
			}

			for (i = 0; i < sizeX; i++) {
				for (j = 0; j < sizeY; j++) {

					if (i % VISUALIZER_OUTPUT_MAPS_X == VISUALIZER_OUTPUT_MAPS_X-1 ||
							j % VISUALIZER_OUTPUT_MAPS_Y == VISUALIZER_OUTPUT_MAPS_Y-1)
						warrayS[i][j] = 0;

					else if ((i%VISUALIZER_OUTPUT_MAPS_X) % VISUALIZER_OUTPUT_MAP_X == VISUALIZER_OUTPUT_MAP_X-1 ||
							(j%VISUALIZER_OUTPUT_MAPS_Y) % VISUALIZER_OUTPUT_MAP_Y == VISUALIZER_OUTPUT_MAP_Y-1)
						warrayS[i][j] = 0;

					else if ((i % VISUALIZER_OUTPUT_MAPS_X) % VISUALIZER_OUTPUT_MAP_X == VISUALIZER_OUTPUT_MAP_X - 2 ||
							(j % VISUALIZER_OUTPUT_MAPS_Y) % VISUALIZER_OUTPUT_MAP_Y == VISUALIZER_OUTPUT_MAP_Y - 2)
						warrayS[i][j] = WHITE_EDGE_COLOR_VALUE;

					else if (((i%VISUALIZER_OUTPUT_MAPS_X) % VISUALIZER_OUTPUT_MAP_X) % (16+1) == 0 ||
							((j%VISUALIZER_OUTPUT_MAPS_Y) % VISUALIZER_OUTPUT_MAP_Y) % (16+1) == 0)
						warrayS[i][j] = WHITE_EDGE_COLOR_VALUE;
				}
			}
			counterS = 0;
			for (i = 0; i < visualizer_height; i++) {
				for (ys = 0; ys < visualizer_width; ys++) {
					col_s  = getColourSG(warrayS[ys][i]);
					singleplotS->pixels[counterS] = col_s.r;
					singleplotS->pixels[counterS + 1] = col_s.g;
					singleplotS->pixels[counterS + 2] = col_s.b;
					counterS += 3;
				}
			}
			caerFrameEventSetLengthXLengthYChannelNumber(singleplotS, visualizer_height, visualizer_width, 3, *synapsePlotFeature3Output); //add info to the frame
			caerFrameEventValidate(singleplotS, *synapsePlotFeature3Output); //validate frame

		}

	}

	caerGestureLearningFilterConfig(moduleData); // Update parameters

//---------------------------------------------------------------MAIN ITERATION LOOP---------------------------------------------------------------
	// keep changing the input pattern
	if (state->learning == true) {
		pattern_period = PATTERN_PERIOD_LEARNING;
	} else {
		pattern_period = PATTERN_PERIOD_TESTING;
	}

	if (file_input_enable == 0) {
		if (state->stimulate == true) { //it runs when there is a spike
			if (stimdisabled == 0 && abs((int)(time_cnt - time_cnt_last)) >= pattern_period) {
//				disableStimuliGenG(moduleData);
				time_cnt_last = time_cnt;
				stimdisabled = 1;
			} else if (stimdisabled == 1 && abs((int)(time_cnt - time_cnt_last)) >= PATTERN_PERIOD_INTERVAL) { //1
				if (pattern_style == 3) {
					stimuli_pattern = 1; //(stimuli_pattern + 1) % 3;
					enableStimuliGenG(moduleData, stimuli_pattern + 7);
				} else if (pattern_style == 4) {
					stimuli_pattern = (stimuli_pattern + 1) % 4;
					enableStimuliGenG(moduleData, stimuli_pattern + 7);
				} else if (pattern_style == 5) {
					stimuli_pattern = (stimuli_pattern + 1) % 4; //0
					enableStimuliGenG(moduleData, ((stimuli_pattern + 1) * 16));
				}
				time_cnt_last = time_cnt;
				stimdisabled = 0;
			}
		}
		else {
			disableStimuliGenG(moduleData);
		}
	} else {
		if (state->stimulate == true) { //it runs when there is a spike
			if (stimdisabled == 0 && abs((int)(time_cnt - time_cnt_last)) >= pattern_period) {
				stimuli_pattern = (stimuli_pattern + 1) % 4;
				if (stimuli_pattern == 0) {
					iteration = (iteration + 1) % 3;
					feature_bias_reseted = 1;
				}
				enableFileStimuliG(moduleData, stimuli_pattern);
				if (state->learning == true && state->teaching == true) { //state->learning_output == true &&
					disableTeachingStimuliGenG(moduleData);
					teaching = 0;
//					disableAllOutputCores(moduleData);
				}
//				disableFileStimuliG(moduleData);
				disableOutputCore(moduleData);
				disableInputChip(moduleData);
				time_cnt_last = time_cnt;
				stimdisabled = 1;
			} else if (stimdisabled == 1 && abs((int)(time_cnt - time_cnt_last)) >= PATTERN_PERIOD_INTERVAL) {
				if (pattern_style == 5) {
//					stimuli_pattern = (stimuli_pattern + 1) % 4;
//					enableFileStimuliG(moduleData, stimuli_pattern);
					if (state->learning == true && state->teaching == true) { //state->learning_output == true &&
						enableTeachingStimuliGenG(moduleData, stimuli_pattern);
						teaching = 1;
//						disableOutputCores(moduleData, stimuli_pattern);
					}
					enableOutputCore(moduleData);
					enableInputChip(moduleData);
//					stimuli_pattern = (stimuli_pattern + 1) % 4; //
				}
				time_cnt_last = time_cnt;
				stimdisabled = 0;
			}
		}
		else {
			if (state->learning_feature3 == true)
				disableTeachingStimuliGenG(moduleData);
			disableFileStimuliG(moduleData);
		}
	}

	if (state->teaching == false && teaching == 1) {
		disableTeachingStimuliGenG(moduleData);
		teaching = 0;
	}

	if (memory.spike_cnt >= MAXIMUM_CONSIDERED_SPIKE_NUM && state->learning == true && state->inject == true && feature_bias_reseted == 1
			&& state->learning_output == false) {
		if (iteration == 0) {
			injectCurrent(moduleData, 1, 0);
			injectCurrent(moduleData, 2, 1);
			injectCurrent(moduleData, 2, 2);
		} else if (iteration == 1) {
			injectCurrent(moduleData, 0, 0);
			injectCurrent(moduleData, 1, 1);
			injectCurrent(moduleData, 2, 2);
		} else {
			injectCurrent(moduleData, 0, 0);
			injectCurrent(moduleData, 0, 1);
			injectCurrent(moduleData, 1, 2);
		}
		feature_bias_reseted = 0;
	}
	else if (feature_bias_reseted == 0 && (state->inject == false || state->learning_output == true)) {
		injectCurrent(moduleData, 0, 0);
		injectCurrent(moduleData, 0, 1);
		injectCurrent(moduleData, 0, 2);
		feature_bias_reseted = 1;
	}

	if(spike == NULL){
		return;
	}

	//triggered by one spike
	CAER_SPIKE_ITERATOR_VALID_START(spike) //iterate over received spikes

		if (state->learning == true) {

			if (memory.spike_cnt < MAXIMUM_CONSIDERED_SPIKE_NUM)
				memory.spike_cnt += 1;

			enableTeachingG(moduleData);

			int64_t ts = caerSpikeEventGetTimestamp64(caerSpikeIteratorElement, spike); //get values on which to operate.

			uint32_t neuron_id = caerSpikeEventGetNeuronID(caerSpikeIteratorElement);
			uint32_t core_id = caerSpikeEventGetSourceCoreID(caerSpikeIteratorElement);
			uint32_t chip_id_t = caerSpikeEventGetChipID(caerSpikeIteratorElement);
			uint32_t chip_id;

			if (chip_id_t == DYNAPSE_CONFIG_DYNAPSE_U0_SPECIAL) //note: not DYNAPSE_CONFIG_DYNAPSE_U0
				chip_id = 1;
			else if (chip_id_t == DYNAPSE_CONFIG_DYNAPSE_U1)
				chip_id = 2;
			else if (chip_id_t == DYNAPSE_CONFIG_DYNAPSE_U2)
				chip_id = 3;
			else if (chip_id_t == DYNAPSE_CONFIG_DYNAPSE_U3)
				chip_id = 4;
			else
				chip_id = 4;

			int64_t spikeAddr = chip_id << NEURON_CHIPID_SHIFT | core_id << NEURON_COREID_SHIFT | neuron_id;
			memory.spike_fifo->buffer2d[memory.wr_pointer][0] = spikeAddr; //put spike address into the queue
			memory.spike_fifo->buffer2d[memory.wr_pointer][1] = ts; //put spike address into the queue

			memory.wr_pointer = (memory.wr_pointer + 1) % SPIKE_QUEUE_LENGTH;

			memory.post_rd_pointer = memory.wr_pointer;

			int64_t post_spike_addr = spikeAddr; //memory.spike_fifo->buffer2d[memory.post_rd_pointer][0];
			int64_t post_spike_time = ts; //memory.spike_fifo->buffer2d[memory.post_rd_pointer][1];

			int64_t post_neuron_chip_id = (post_spike_addr & NEURON_CHIPID_BITS) >> NEURON_CHIPID_SHIFT;
			int64_t post_neuron_core_id = (post_spike_addr & NEURON_COREID_BITS) >> NEURON_COREID_SHIFT;
			int post_neuron_id = (int) anti_output_layer[post_spike_addr & 0x3ff];
			int64_t post_neuron_output_id = ((int)((int)(post_neuron_id/4)/2) << 1 |
					(int)((post_neuron_id%4)/2));

			if (memory.spike_cnt >= MAXIMUM_CONSIDERED_SPIKE_NUM &&
					((post_neuron_chip_id == CHIP_UP_RIGHT_ID && state->learning_feature1 == true && state->learning_output == false) ||
					(post_neuron_chip_id == CHIP_DOWN_LEFT_ID && state->learning_feature2 == true && state->learning_output == false) ||
					(post_neuron_chip_id == CHIP_UP_LEFT_ID && post_neuron_core_id != 3 &&
							state->learning_feature3 == true && state->learning_output == false) ||
					(post_neuron_chip_id == CHIP_UP_LEFT_ID && post_neuron_core_id == 3 &&
							post_neuron_output_id == stimuli_pattern && state->learning_output == true))) //
			{

				uint8_t end_searching = 0;
				int64_t pre_spike_addr, pre_spike_time;
				double total_increased_weight = 0;

				for (uint64_t pre_rd_pointer = (memory.post_rd_pointer - 1) % SPIKE_QUEUE_LENGTH;
						end_searching != 1;
						pre_rd_pointer = (pre_rd_pointer - 1) % SPIKE_QUEUE_LENGTH) {

					pre_spike_addr = memory.spike_fifo->buffer2d[pre_rd_pointer][0];
					pre_spike_time = memory.spike_fifo->buffer2d[pre_rd_pointer][1];

					int64_t delta_time = (int64_t) (post_spike_time - pre_spike_time) / 1000;

					if (delta_time > MAXIMUM_CONSIDERED_SPIKE_INTERVAL) {
						end_searching = 1;
						break;
					}

					if (abs((int)pre_rd_pointer-(int)memory.post_rd_pointer) > MAXIMUM_CONSIDERED_SPIKE_NUM) {
						end_searching = 1;
						break;
					}

					if (delta_time >= MINIMUM_CONSIDERED_SPIKE_INTERVAL && delta_time <= MAXIMUM_CONSIDERED_SPIKE_INTERVAL) {

						if (memory.connection_map->buffer2d[pre_spike_addr - MEMORY_NEURON_ADDR_OFFSET][post_spike_addr - MEMORY_NEURON_ADDR_OFFSET] == 1) {

							double random_number = rand() % 100;
							double random_number_threshold = PROBABILITY * 100;

							if (feature_bias_reseted == 0)
								random_number_threshold = PROBABILITY_INJECTING * 100;

//							if (post_neuron_chip_id == CHIP_UP_LEFT_ID && post_neuron_core_id == 3)
//								random_number_threshold = OUTPUT_PROBABILITY * 100;

							if (random_number <= random_number_threshold || (post_neuron_chip_id == CHIP_UP_LEFT_ID && post_neuron_core_id == 3)) { //

								double forward_delta_weight;
								if (total_increased_weight > NEURON_LOW_HIGH_ACTIVITY_THRESHOLD)
									forward_delta_weight = deltaWeights[DELTA_WEIGHT_LUT_LENGTH - delta_time] * (-1);
								else
									forward_delta_weight = deltaWeights[DELTA_WEIGHT_LUT_LENGTH - delta_time];

								if (post_neuron_chip_id == CHIP_UP_LEFT_ID && post_neuron_core_id != 3)
									forward_delta_weight = deltaWeights[DELTA_WEIGHT_LUT_LENGTH - delta_time] * 2;

								double increased_weight = forwardUpdateWeightG(moduleData, pre_spike_addr, post_spike_addr, forward_delta_weight);

								total_increased_weight += increased_weight; //forward_delta_weight * state->forward_learning_rate;

							}

						}

					}

				}

				if (total_increased_weight != 0) {

					normalizeIncreasedWeightG(moduleData, post_spike_addr, total_increased_weight);

					roundWeightG(moduleData, post_spike_addr);

					updateSynapseG(moduleData, post_spike_addr,
							synapsePlotInputFeature1, synapsePlotFeature1Feature2,
							synapsePlotFeature2Feature3, synapsePlotFeature3Output);

				}
			}
		}
		else if (feature_bias_reseted == 0) {
			disableTeachingStimuliGenG(moduleData);

			injectCurrent(moduleData, 0, 0);
			injectCurrent(moduleData, 0, 1);
			injectCurrent(moduleData, 0, 2);

			feature_bias_reseted = 1;

//			if (output_bias_reseted == 0) {
//				enableAllOutputCores(moduleData);
//				output_bias_reseted = 1;
//			}
//			setTestingBias(moduleData);
		}

	CAER_SPIKE_ITERATOR_VALID_END
}

double forwardUpdateWeightG(caerModuleData moduleData, int64_t pre_neuron_addr, int64_t post_neuron_addr, double forward_delta_weight) {
	/* Modify connection weight with probability according to forward pair of spikes */
	GFilterState state = moduleData->moduleState;

	int64_t post_neuron_chip_id = (post_neuron_addr & NEURON_CHIPID_BITS) >> NEURON_CHIPID_SHIFT;
	int64_t post_neuron_core_id = (post_neuron_addr & NEURON_COREID_BITS) >> NEURON_COREID_SHIFT;
	int post_neuron_id = (int) anti_output_layer[post_neuron_addr & 0x3ff];
	int64_t post_neuron_output_id = ((int)((int)(post_neuron_id/4)/2) << 1) | (int)((post_neuron_id%4)/2);

	double weight_change = 0;

	if ((post_neuron_chip_id == CHIP_UP_RIGHT_ID && state->learning_feature1 == true) ||
			(post_neuron_chip_id == CHIP_DOWN_LEFT_ID && state->learning_feature2 == true) ||
			(post_neuron_chip_id == CHIP_UP_LEFT_ID && post_neuron_core_id != 3 && state->learning_feature3 == true) ||
			(post_neuron_chip_id == CHIP_UP_LEFT_ID && post_neuron_core_id == 3 && state->learning_output == true
					&& memory.outputMapDisabled->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_output_id] != 1))
	{

		double current_weight = memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];

		double new_weight_increased;
		new_weight_increased = current_weight + forward_delta_weight * state->forward_learning_rate;

		memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] = new_weight_increased;

		if (new_weight_increased > memory.filter_map_highest->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0]) {
			memory.filter_map_highest->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] = new_weight_increased;
		}

		weight_change = forward_delta_weight * state->forward_learning_rate;

	}

	if (post_neuron_chip_id == CHIP_UP_LEFT_ID && post_neuron_core_id == 3 && state->learning_output == true
			&& memory.outputMapDisabled->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_output_id] != 1) {

		if (state->learning_output_phase2 == true) {
			memory.outputMap->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_output_id] = 1;

			int output_count = 0;
			int output_id;
			for (output_id = 0; output_id < OUTPUT_LAYER_NUM; output_id++) {
				if (memory.outputMap->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][output_id] == 1)
					output_count += 1;
			}
			if (output_count >= 2) { //2
				for (output_id = 0; output_id < OUTPUT_LAYER_NUM; output_id++) {
					memory.outputMapDisabled->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][output_id] = 1;
				}
			}

			for (int neuron_id = 0; neuron_id < OUTPUT_LAYER_N*OUTPUT_LAYER_NUM; neuron_id++) {

				int64_t new_post_neuron_addr = output_layer[neuron_id];

				int64_t new_post_neuron_output_id = ((int)((int)(neuron_id/4)/2) << 1) | (int)((neuron_id%4)/2);

				if (memory.outputMapDisabled->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_output_id] == 1) {

					double new_weight_decreased = -10; //0;

					memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] =
							new_weight_decreased;

					if (new_weight_decreased < memory.filter_map_lowest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0]) {
						memory.filter_map_lowest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] = new_weight_decreased;
					}

					weight_change = new_weight_decreased;
				}
			}
		}

		/*
		for (int neuron_id = 0; neuron_id < OUTPUT_LAYER_N*OUTPUT_LAYER_NUM; neuron_id++) {

			int64_t new_post_neuron_addr = output_layer[neuron_id];

			int64_t new_post_neuron_output_id = ((int)((int)(((new_post_neuron_addr & 0x3ff) % 256)/4)/2) << 1 |
					(int)((((new_post_neuron_addr & 0x3ff) % 256)%4)/2));

			if (new_post_neuron_output_id != post_neuron_output_id) {
				double current_weight = memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];

				double new_weight_decreased = current_weight - forward_delta_weight * state->forward_learning_rate;

				memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] =
						new_weight_decreased;

				if (new_weight_decreased < memory.filter_map_lowest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0]) {
					memory.filter_map_lowest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] = new_weight_decreased;
				}
			}
		}
		weight_change = - forward_delta_weight * state->forward_learning_rate;
		*/
	}

	if (post_neuron_chip_id == CHIP_UP_RIGHT_ID && state->learning_feature1 == true) { //post_neuron_chip_id == CHIP_DOWN_LEFT_ID ||

		for (int feature_map_id = 0; feature_map_id < FEATURE_LAYER1_NUM; feature_map_id++) {

			int neuron_id = (int) anti_feature_layer1[post_neuron_addr & 0x3ff];

			int new_neuron_id = (((feature_map_id & 0xc) >> 2)*FEATURE_LAYER1_L + (int)(neuron_id / 28)%FEATURE_LAYER1_L) * 28 +
					(feature_map_id & 0x3)*FEATURE_LAYER1_W + (neuron_id % 28)%FEATURE_LAYER1_L;

			int64_t new_post_neuron_addr = feature_layer1[new_neuron_id];

			if (new_post_neuron_addr != post_neuron_addr) {

				if (memory.connection_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] == 1) {

					double current_weight = memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];
					double new_weight_decreased = current_weight - forward_delta_weight * state->forward_learning_rate * COMPENSATE_RATIO;

					memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] =
							new_weight_decreased;

					if (new_weight_decreased < memory.filter_map_highest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0]) {
						memory.filter_map_lowest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] = new_weight_decreased;
					}

				}
			}
		}

	}
	else if (post_neuron_chip_id == CHIP_DOWN_LEFT_ID && state->learning_feature2 == true) {

		for (int feature_map_id = 0; feature_map_id < FEATURE_LAYER2_NUM; feature_map_id++) {

			int neuron_id = (int) anti_feature_layer2[post_neuron_addr & 0x3ff];

			int new_neuron_id = ((int)(feature_map_id / 6)*FEATURE_LAYER2_L + (int)(neuron_id / 30)%FEATURE_LAYER2_L) * 30 +
					(feature_map_id % 6)*FEATURE_LAYER2_W + (neuron_id % 30)%FEATURE_LAYER2_L;

			int64_t new_post_neuron_addr = feature_layer2[new_neuron_id];

			if (new_post_neuron_addr != post_neuron_addr) {

				if (memory.connection_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] == 1) {

					double current_weight = memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];
					double new_weight_decreased = current_weight - forward_delta_weight * state->forward_learning_rate * COMPENSATE_RATIO;

					memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] =
							new_weight_decreased;

					if (new_weight_decreased < memory.filter_map_highest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0]) {
						memory.filter_map_lowest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] = new_weight_decreased;
					}

				}
			}
		}

	}
	else if (post_neuron_chip_id == CHIP_UP_LEFT_ID && post_neuron_core_id != 3 && state->learning_feature3 == true
			&& memory.outputMapDisabled->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_output_id] == 0) {

		for (int feature_map_id = 0; feature_map_id < 4; feature_map_id++) {

			int neuron_id = (int) anti_feature_layer3[post_neuron_addr & 0x3ff];

			int new_neuron_id = (int)(neuron_id/256) << 8 |
					( (((int)((neuron_id%256)/16)%8) + (int)(feature_map_id/2)*8)*16 +
							((((neuron_id%256)%16)%8) + (feature_map_id%2)*8) );

			int64_t new_post_neuron_addr = feature_layer3[new_neuron_id];

			if (new_post_neuron_addr != post_neuron_addr) {

				if (memory.connection_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] == 1) {

					double current_weight = memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];
					double new_weight_decreased = current_weight - forward_delta_weight * state->forward_learning_rate * COMPENSATE_RATIO; // * COMPENSATE_RATIO

					memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] =
							new_weight_decreased;

					if (new_weight_decreased < memory.filter_map_highest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0]) {
						memory.filter_map_lowest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] = new_weight_decreased;
					}

				}
			}
		}

	}

	return (weight_change);

}

void normalizeIncreasedWeightG(caerModuleData moduleData, int64_t post_neuron_addr, double increased_delta_weight) {
	/* normalize weight distribution */
	GFilterState state = moduleData->moduleState;
	int64_t post_neuron_chip_id = (post_neuron_addr & NEURON_CHIPID_BITS) >> NEURON_CHIPID_SHIFT;
	int64_t post_neuron_core_id = (post_neuron_addr & NEURON_COREID_BITS) >> NEURON_COREID_SHIFT;

	//normalize the weight
	double balance_value;
	if (post_neuron_chip_id == CHIP_UP_LEFT_ID && post_neuron_core_id == 3)
		balance_value = increased_delta_weight / (FEATURE_LAYER3_N * FEATURE_LAYER3_NUM); //TOTAL_CAM_NUM_LEARNING_O;
	else
		balance_value = increased_delta_weight / TOTAL_CAM_NUM_LEARNING_F1;

	if ((post_neuron_chip_id == CHIP_UP_RIGHT_ID && state->learning_feature1 == true) ||
			(post_neuron_chip_id == CHIP_DOWN_LEFT_ID && state->learning_feature2 == true) ||
			(post_neuron_chip_id == CHIP_UP_LEFT_ID && post_neuron_core_id != 3 && state->learning_feature3 == true))
	{ //(post_neuron_chip_id == CHIP_UP_LEFT_ID && post_neuron_core_id == 3 && state->learning_output == true)

		for (int filter_neuron_id = 0;
				filter_neuron_id < memory.filter_map_size->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
				filter_neuron_id++)
		{
			int64_t pre_neuron_addr = memory.filter_map_addr->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][filter_neuron_id];
			double current_synapse_weight = memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];
			double new_synapse_weight = current_synapse_weight - balance_value;
			memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] = new_synapse_weight;
		}

		if (memory.filter_map_highest->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] > WEIGHT_EX_UP_THRESHOLD) {
			double ratio = WEIGHT_EX_UP_THRESHOLD / memory.filter_map_highest->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
			for (int filter_neuron_id = 0;
					filter_neuron_id < memory.filter_map_size->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
					filter_neuron_id++)
			{
				int64_t pre_neuron_addr = memory.filter_map_addr->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][filter_neuron_id];
				double current_synapse_weight = memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];
				if (current_synapse_weight > 0) {
					double new_synapse_weight = current_synapse_weight * ratio;
					memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] = new_synapse_weight;
				}
			}
			memory.filter_map_highest->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] =
					memory.filter_map_highest->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] * ratio;
		}
		if (memory.filter_map_lowest->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] < WEIGHT_IN_UP_THRESHOLD) {
			double ratio = WEIGHT_IN_UP_THRESHOLD / memory.filter_map_lowest->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
			for (int filter_neuron_id = 0;
					filter_neuron_id < memory.filter_map_size->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
					filter_neuron_id++)
			{
				int64_t pre_neuron_addr = memory.filter_map_addr->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][filter_neuron_id];
				double current_synapse_weight = memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];
				if (current_synapse_weight < 0) {
					double new_synapse_weight = current_synapse_weight * ratio;
					memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] = new_synapse_weight;
				}
			}
			memory.filter_map_lowest->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] =
					memory.filter_map_lowest->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] * ratio;
		}

	}

	if (post_neuron_chip_id == CHIP_UP_RIGHT_ID && state->learning_feature1 == true) { //post_neuron_chip_id == CHIP_DOWN_LEFT_ID ||

		for (int feature_map_id = 0; feature_map_id < FEATURE_LAYER1_NUM; feature_map_id++) {

			int neuron_id = (int) anti_feature_layer1[post_neuron_addr & 0x3ff];

			int new_neuron_id = (((feature_map_id & 0xc) >> 2)*FEATURE_LAYER1_L + (int)(neuron_id / 28)%FEATURE_LAYER1_L) * 28 +
					(feature_map_id & 0x3)*FEATURE_LAYER1_W + (neuron_id % 28)%FEATURE_LAYER1_L;

			int64_t new_post_neuron_addr = feature_layer1[new_neuron_id];

			if (new_post_neuron_addr != post_neuron_addr) {

				balance_value = (increased_delta_weight * state->forward_learning_rate * COMPENSATE_RATIO) / TOTAL_CAM_NUM_LEARNING_F1;

				for (int filter_neuron_id = 0;
						filter_neuron_id < memory.filter_map_size->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
						filter_neuron_id++)
				{
					int64_t pre_neuron_addr = memory.filter_map_addr->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][filter_neuron_id];
					double current_synapse_weight = memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];
					double new_synapse_weight = current_synapse_weight + balance_value;
					memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] = new_synapse_weight;
				}

				if (memory.filter_map_highest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] > WEIGHT_EX_UP_THRESHOLD) {
					double ratio = WEIGHT_EX_UP_THRESHOLD / memory.filter_map_highest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
					for (int filter_neuron_id = 0;
							filter_neuron_id < memory.filter_map_size->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
							filter_neuron_id++)
					{
						int64_t pre_neuron_addr = memory.filter_map_addr->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][filter_neuron_id];
						double current_synapse_weight = memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];
						if (current_synapse_weight > 0) {
							double new_synapse_weight = current_synapse_weight * ratio;
							memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] = new_synapse_weight;
						}
					}
					memory.filter_map_highest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] =
							memory.filter_map_highest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] * ratio;
				}
				if (memory.filter_map_lowest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] < WEIGHT_IN_UP_THRESHOLD) {
					double ratio = WEIGHT_IN_UP_THRESHOLD / memory.filter_map_lowest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
					for (int filter_neuron_id = 0;
							filter_neuron_id < memory.filter_map_size->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
							filter_neuron_id++)
					{
						int64_t pre_neuron_addr = memory.filter_map_addr->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][filter_neuron_id];
						double current_synapse_weight = memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];
						if (current_synapse_weight < 0) {
							double new_synapse_weight = current_synapse_weight * ratio;
							memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] = new_synapse_weight;
						}
					}
					memory.filter_map_lowest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] =
							memory.filter_map_lowest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] * ratio;
				}

			}
		}

	}
	else if (post_neuron_chip_id == CHIP_DOWN_LEFT_ID && state->learning_feature2 == true) {
		for (int feature_map_id = 0; feature_map_id < FEATURE_LAYER2_NUM; feature_map_id++) {

			int neuron_id = (int) anti_feature_layer2[post_neuron_addr & 0x3ff];

			int new_neuron_id = ((int)(feature_map_id / 6)*FEATURE_LAYER2_L + (int)(neuron_id / 30)%FEATURE_LAYER2_L) * 30 +
					(feature_map_id % 6)*FEATURE_LAYER2_W + (neuron_id % 30)%FEATURE_LAYER2_L;

			int64_t new_post_neuron_addr = feature_layer2[new_neuron_id];

			if (new_post_neuron_addr != post_neuron_addr) {

				balance_value = (increased_delta_weight * state->forward_learning_rate * COMPENSATE_RATIO) / TOTAL_CAM_NUM_LEARNING_F2;

				for (int filter_neuron_id = 0;
						filter_neuron_id < memory.filter_map_size->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
						filter_neuron_id++)
				{
					int64_t pre_neuron_addr = memory.filter_map_addr->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][filter_neuron_id];
					double current_synapse_weight = memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];
					double new_synapse_weight = current_synapse_weight + balance_value;
					memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] = new_synapse_weight;
				}

				if (memory.filter_map_highest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] > WEIGHT_EX_UP_THRESHOLD) {
					double ratio = WEIGHT_EX_UP_THRESHOLD / memory.filter_map_highest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
					for (int filter_neuron_id = 0;
							filter_neuron_id < memory.filter_map_size->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
							filter_neuron_id++)
					{
						int64_t pre_neuron_addr = memory.filter_map_addr->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][filter_neuron_id];
						double current_synapse_weight = memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];
						if (current_synapse_weight > 0) {
							double new_synapse_weight = current_synapse_weight * ratio;
							memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] = new_synapse_weight;
						}
					}
					memory.filter_map_highest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] =
							memory.filter_map_highest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] * ratio;
				}
				if (memory.filter_map_lowest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] < WEIGHT_IN_UP_THRESHOLD) {
					double ratio = WEIGHT_IN_UP_THRESHOLD / memory.filter_map_lowest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
					for (int filter_neuron_id = 0;
							filter_neuron_id < memory.filter_map_size->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
							filter_neuron_id++)
					{
						int64_t pre_neuron_addr = memory.filter_map_addr->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][filter_neuron_id];
						double current_synapse_weight = memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];
						if (current_synapse_weight < 0) {
							double new_synapse_weight = current_synapse_weight * ratio;
							memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] = new_synapse_weight;
						}
					}
					memory.filter_map_lowest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] =
							memory.filter_map_lowest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] * ratio;
				}

			}
		}
	}
	else if (post_neuron_chip_id == CHIP_UP_LEFT_ID && post_neuron_core_id != 3 && state->learning_feature3 == true) { //
		for (int feature_map_id = 0; feature_map_id < 4; feature_map_id++) {

			int neuron_id = (int) anti_feature_layer3[post_neuron_addr & 0x3ff];

			int new_neuron_id = (int)(neuron_id/256) << 8 |
					( (((int)((neuron_id%256)/16)%8) + (int)(feature_map_id/2)*8)*16 +
							((((neuron_id%256)%16)%8) + (feature_map_id%2)*8) );

			int64_t new_post_neuron_addr = feature_layer3[new_neuron_id];

			if (new_post_neuron_addr != post_neuron_addr) {

				balance_value = (increased_delta_weight * state->forward_learning_rate * COMPENSATE_RATIO) / TOTAL_CAM_NUM_LEARNING_F1;

				for (int filter_neuron_id = 0;
						filter_neuron_id < memory.filter_map_size->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
						filter_neuron_id++)
				{
					int64_t pre_neuron_addr = memory.filter_map_addr->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][filter_neuron_id];
					double current_synapse_weight = memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];
					double new_synapse_weight = current_synapse_weight + balance_value;
					memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] = new_synapse_weight;
				}

				if (memory.filter_map_highest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] > WEIGHT_EX_UP_THRESHOLD) {
					double ratio = memory.filter_map_highest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] / WEIGHT_EX_UP_THRESHOLD;
					for (int filter_neuron_id = 0;
							filter_neuron_id < memory.filter_map_size->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
							filter_neuron_id++)
					{
						int64_t pre_neuron_addr = memory.filter_map_addr->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][filter_neuron_id];
						double current_synapse_weight = memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];
						if (current_synapse_weight > 0) {
							double new_synapse_weight = current_synapse_weight * ratio;
							memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] = new_synapse_weight;
						}
					}
					memory.filter_map_highest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] =
							memory.filter_map_highest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] * ratio;
				}
				if (memory.filter_map_lowest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] < WEIGHT_IN_UP_THRESHOLD) {
					double ratio = memory.filter_map_lowest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] / WEIGHT_IN_UP_THRESHOLD;
					for (int filter_neuron_id = 0;
							filter_neuron_id < memory.filter_map_size->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
							filter_neuron_id++)
					{
						int64_t pre_neuron_addr = memory.filter_map_addr->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][filter_neuron_id];
						double current_synapse_weight = memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];
						if (current_synapse_weight < 0) {
							double new_synapse_weight = current_synapse_weight * ratio;
							memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] = new_synapse_weight;
						}
					}
					memory.filter_map_lowest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] =
							memory.filter_map_lowest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] * ratio;
				}

			}
		}
	}
}

void roundWeightG(caerModuleData moduleData, int64_t post_neuron_addr) {
	/* regulate weight distribution */
	GFilterState state = moduleData->moduleState;
	int64_t post_neuron_chip_id = (post_neuron_addr & NEURON_CHIPID_BITS) >> NEURON_CHIPID_SHIFT;
	int64_t post_neuron_core_id = (post_neuron_addr & NEURON_COREID_BITS) >> NEURON_COREID_SHIFT;

	if ((post_neuron_chip_id == CHIP_UP_RIGHT_ID && state->learning_feature1 == true) ||
			(post_neuron_chip_id == CHIP_DOWN_LEFT_ID && state->learning_feature2 == true) ||
			(post_neuron_chip_id == CHIP_UP_LEFT_ID && post_neuron_core_id != 3 && state->learning_feature3 == true) ||
			(post_neuron_chip_id == CHIP_UP_LEFT_ID && post_neuron_core_id == 3 && state->learning_output == true))
	{
		for (int filter_neuron_id = 0;
				filter_neuron_id < memory.filter_map_size->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
				filter_neuron_id++)
		{
			int64_t pre_neuron_addr = memory.filter_map_addr->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][filter_neuron_id];
			double current_synapse_weight = memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];

			if (current_synapse_weight > EX_IN_BALANCE_THRESHOLD_L) {
				if (current_synapse_weight > EX_FS_THRESHOLD) {
					int32_t rounded_synapse_weight;
					if (post_neuron_chip_id == CHIP_UP_LEFT_ID && post_neuron_core_id == 3)
						rounded_synapse_weight = FAST_EX_SYNAPSE_VALUE;
					else
						rounded_synapse_weight = FAST_EX_SYNAPSE_VALUE;
					memory.rounded_weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] =
							rounded_synapse_weight;
				} else {
					int32_t rounded_synapse_weight;
					if (post_neuron_chip_id == CHIP_UP_LEFT_ID && post_neuron_core_id == 3)
						rounded_synapse_weight = NO_SYNAPSE_ID; //FAST_EX_SYNAPSE_VALUE;
					else
						rounded_synapse_weight = FAST_EX_SYNAPSE_VALUE; //SLOW_EX_SYNAPSE_VALUE
					memory.rounded_weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] =
							rounded_synapse_weight;
				}

			} else if (current_synapse_weight <= EX_IN_BALANCE_THRESHOLD_L) {
				if (current_synapse_weight < IN_FS_THRESHOLD) {
					int32_t rounded_synapse_weight;
					if (post_neuron_chip_id == CHIP_UP_LEFT_ID && post_neuron_core_id == 3)
						rounded_synapse_weight = NO_SYNAPSE_ID;
					else
						rounded_synapse_weight = FAST_IN_SYNAPSE_VALUE;
					memory.rounded_weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] =
							rounded_synapse_weight;
				} else {
					int32_t rounded_synapse_weight;
					if (post_neuron_chip_id == CHIP_UP_LEFT_ID && post_neuron_core_id == 3)
						rounded_synapse_weight = NO_SYNAPSE_ID;
					else
						rounded_synapse_weight = FAST_IN_SYNAPSE_VALUE; //SLOW_IN_SYNAPSE_VALUE
					memory.rounded_weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] =
							rounded_synapse_weight;
				}
			} else {
				memory.rounded_weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] = 0;
			}
		}
	}

	if (post_neuron_chip_id == CHIP_UP_RIGHT_ID && state->learning_feature1 == true) {

		for (int feature_map_id = 0; feature_map_id < FEATURE_LAYER1_NUM; feature_map_id++) {

			int neuron_id = (int) anti_feature_layer1[post_neuron_addr & 0x3ff];

			int new_neuron_id = (((feature_map_id & 0xc) >> 2)*FEATURE_LAYER1_L + (int)(neuron_id / 28)%FEATURE_LAYER1_L) * 28 +
					(feature_map_id & 0x3)*FEATURE_LAYER1_W + (neuron_id % 28)%FEATURE_LAYER1_L;

			int64_t new_post_neuron_addr = feature_layer1[new_neuron_id];

			if (new_post_neuron_addr != post_neuron_addr) {

				for (int filter_neuron_id = 0;
						filter_neuron_id < memory.filter_map_size->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
						filter_neuron_id++)
				{
					int64_t pre_neuron_addr = memory.filter_map_addr->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][filter_neuron_id];
					double current_synapse_weight = memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];

					if (current_synapse_weight > EX_IN_BALANCE_THRESHOLD_L) {
						if (current_synapse_weight > EX_FS_THRESHOLD) { //memory.filter_map_highest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] / 2) {
							int32_t rounded_synapse_weight = FAST_EX_SYNAPSE_VALUE;
							memory.rounded_weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] =
									rounded_synapse_weight;
						} else {
							int32_t rounded_synapse_weight = FAST_EX_SYNAPSE_VALUE; //SLOW_EX_SYNAPSE_VALUE
							memory.rounded_weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] =
									rounded_synapse_weight;
						}
					} else if (current_synapse_weight <= EX_IN_BALANCE_THRESHOLD_L) {
						if (current_synapse_weight < IN_FS_THRESHOLD) { //memory.filter_map_lowest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] / 2) {
							int32_t rounded_synapse_weight = FAST_IN_SYNAPSE_VALUE;
							memory.rounded_weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] =
									rounded_synapse_weight;
						} else {
							int32_t rounded_synapse_weight = FAST_IN_SYNAPSE_VALUE; //SLOW_IN_SYNAPSE_VALUE
							memory.rounded_weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] =
									rounded_synapse_weight;
						}
					} else {
						memory.rounded_weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] = 0;
					}
				}

			}
		}

	}
	else if (post_neuron_chip_id == CHIP_DOWN_LEFT_ID && state->learning_feature2 == true) {

		for (int feature_map_id = 0; feature_map_id < FEATURE_LAYER2_NUM; feature_map_id++) {

			int neuron_id = (int) anti_feature_layer2[post_neuron_addr & 0x3ff];

			int new_neuron_id = ((int)(feature_map_id / 6)*FEATURE_LAYER2_L + (int)(neuron_id / 30)%FEATURE_LAYER2_L) * 30 +
					(feature_map_id % 6)*FEATURE_LAYER2_W + (neuron_id % 30)%FEATURE_LAYER2_L;

			int64_t new_post_neuron_addr = feature_layer2[new_neuron_id];

			if (new_post_neuron_addr != post_neuron_addr) {

				for (int filter_neuron_id = 0;
						filter_neuron_id < memory.filter_map_size->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
						filter_neuron_id++)
				{
					int64_t pre_neuron_addr = memory.filter_map_addr->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][filter_neuron_id];
					double current_synapse_weight = memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];

					if (current_synapse_weight > EX_IN_BALANCE_THRESHOLD_L && current_synapse_weight < EX_IN_BALANCE_THRESHOLD_H) {
						if (current_synapse_weight > EX_FS_THRESHOLD) { //memory.filter_map_highest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] / 2) {
							int32_t rounded_synapse_weight = FAST_EX_SYNAPSE_VALUE;
							memory.rounded_weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] =
									rounded_synapse_weight;
						} else {
							int32_t rounded_synapse_weight = FAST_EX_SYNAPSE_VALUE; //SLOW_EX_SYNAPSE_VALUE
							memory.rounded_weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] =
									rounded_synapse_weight;
						}
					} else if (current_synapse_weight <= EX_IN_BALANCE_THRESHOLD_L || current_synapse_weight >= EX_IN_BALANCE_THRESHOLD_H) {
						if (current_synapse_weight < IN_FS_THRESHOLD ) { //memory.filter_map_lowest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] / 2) {
							int32_t rounded_synapse_weight = FAST_IN_SYNAPSE_VALUE;
							memory.rounded_weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] =
									rounded_synapse_weight;
						} else {
							int32_t rounded_synapse_weight = FAST_IN_SYNAPSE_VALUE; //SLOW_IN_SYNAPSE_VALUE
							memory.rounded_weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] =
									rounded_synapse_weight;
						}
					} else {
						memory.rounded_weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] = 0;
					}
				}

			}
		}

	}
	else if (post_neuron_chip_id == CHIP_UP_LEFT_ID && post_neuron_core_id != 3 && state->learning_feature3 == true) { // && post_neuron_output_id == stimuli_pattern

		for (int feature_map_id = 0; feature_map_id < 4; feature_map_id++) {

			int neuron_id = (int) anti_feature_layer3[post_neuron_addr & 0x3ff];

			int new_neuron_id = (int)(neuron_id/256) << 8 |
					( (((int)((neuron_id%256)/16)%8) + (int)(feature_map_id/2)*8)*16 +
							((((neuron_id%256)%16)%8) + (feature_map_id%2)*8) );

			int64_t new_post_neuron_addr = feature_layer3[new_neuron_id];

			if (new_post_neuron_addr != post_neuron_addr) {

				for (int filter_neuron_id = 0;
						filter_neuron_id < memory.filter_map_size->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
						filter_neuron_id++)
				{
					int64_t pre_neuron_addr = memory.filter_map_addr->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][filter_neuron_id];
					double current_synapse_weight = memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];

					if (current_synapse_weight > 0) {
						if (current_synapse_weight > memory.filter_map_highest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] / 2) {
							int32_t rounded_synapse_weight = FAST_EX_SYNAPSE_VALUE;
							memory.rounded_weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] =
									rounded_synapse_weight;
						} else {
							int32_t rounded_synapse_weight = FAST_EX_SYNAPSE_VALUE; //SLOW_EX_SYNAPSE_VALUE
							memory.rounded_weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] =
									rounded_synapse_weight;
						}
					} else if (current_synapse_weight < 0) {
						if (current_synapse_weight < memory.filter_map_lowest->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] / 2) {
							int32_t rounded_synapse_weight = FAST_IN_SYNAPSE_VALUE;
							memory.rounded_weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] =
									rounded_synapse_weight;
						} else {
							int32_t rounded_synapse_weight = FAST_IN_SYNAPSE_VALUE; //SLOW_IN_SYNAPSE_VALUE
							memory.rounded_weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] =
									rounded_synapse_weight;
						}
					} else {
						memory.rounded_weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] = 0;
					}
				}

			}
		}

	}

}

void updateSynapseG(caerModuleData moduleData, int64_t post_neuron_addr,
		caerFrameEventPacket *synapsePlotInputFeature1, caerFrameEventPacket *synapsePlotFeature1Feature2,
		caerFrameEventPacket *synapsePlotFeature2Feature3, caerFrameEventPacket *synapsePlotFeature3Output)
{
	/* update synapses on chip */
	GFilterState state = moduleData->moduleState;

	uint32_t post_neuron_chip_id = (uint32_t) (post_neuron_addr & NEURON_CHIPID_BITS) >> NEURON_CHIPID_SHIFT;
	int64_t post_neuron_core_id = (post_neuron_addr & NEURON_COREID_BITS) >> NEURON_COREID_SHIFT;

	//no potential synapse
	if ((post_neuron_chip_id == CHIP_UP_RIGHT_ID && state->learning_feature1 == true) ||
			(post_neuron_chip_id == CHIP_DOWN_LEFT_ID && state->learning_feature2 == true) ||
			(post_neuron_chip_id == CHIP_UP_LEFT_ID && post_neuron_core_id != 3 && state->learning_feature3 == true)) { // && post_neuron_output_id == stimuli_pattern && post_neuron_output_id == stimuli_pattern

		for (int filter_neuron_id = 0;
				filter_neuron_id < memory.filter_map_size->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
				filter_neuron_id++)
		{

			int64_t pre_neuron_addr = memory.filter_map_addr->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][filter_neuron_id];
			int32_t synapse_type = memory.rounded_weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];

			if (synapse_type != memory.synapse_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET]) {

				modifySynapse(moduleData, pre_neuron_addr, post_neuron_addr, synapse_type,
						synapsePlotInputFeature1, synapsePlotFeature1Feature2,
						synapsePlotFeature2Feature3, synapsePlotFeature3Output);

			}
		}
	}

	double random_number = rand() % 100;
	double random_number_threshold = PALASTICITY_PROBABILITY * 100;

	if (random_number <= random_number_threshold) {

		if (post_neuron_chip_id == CHIP_UP_RIGHT_ID && state->learning_feature1 == true) {

			for (int feature_map_id = 0; feature_map_id < FEATURE_LAYER1_NUM; feature_map_id++) {

				int neuron_id = (int) anti_feature_layer1[post_neuron_addr & 0x3ff];

				int new_neuron_id = (((feature_map_id & 0xc) >> 2)*FEATURE_LAYER1_L + (int)(neuron_id / 28)%FEATURE_LAYER1_L) * 28 +
						(feature_map_id & 0x3)*FEATURE_LAYER1_W + (neuron_id % 28)%FEATURE_LAYER1_L;

				int64_t new_post_neuron_addr = feature_layer1[new_neuron_id];

				if (new_post_neuron_addr != post_neuron_addr) {

					for (int filter_neuron_id = 0;
							filter_neuron_id < memory.filter_map_size->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
							filter_neuron_id++)
					{

						int64_t pre_neuron_addr = memory.filter_map_addr->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][filter_neuron_id];
						int32_t synapse_type = memory.rounded_weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];

						if (synapse_type != memory.synapse_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET]) {

							modifySynapse(moduleData, pre_neuron_addr, new_post_neuron_addr, synapse_type,
									synapsePlotInputFeature1, synapsePlotFeature1Feature2,
									synapsePlotFeature2Feature3, synapsePlotFeature3Output);

						}
					}

				}
			}

		}
		else if (post_neuron_chip_id == CHIP_DOWN_LEFT_ID && state->learning_feature2 == true) {

			for (int feature_map_id = 0; feature_map_id < FEATURE_LAYER2_NUM; feature_map_id++) {

				int neuron_id = (int) anti_feature_layer2[post_neuron_addr & 0x3ff];

				int new_neuron_id = ((int)(feature_map_id / 6)*FEATURE_LAYER2_L + (int)(neuron_id / 30)%FEATURE_LAYER2_L) * 30 +
						(feature_map_id % 6)*FEATURE_LAYER2_W + (neuron_id % 30)%FEATURE_LAYER2_L;

				int64_t new_post_neuron_addr = feature_layer2[new_neuron_id];

				if (new_post_neuron_addr != post_neuron_addr) {

					for (int filter_neuron_id = 0;
							filter_neuron_id < memory.filter_map_size->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
							filter_neuron_id++)
					{

						int64_t pre_neuron_addr = memory.filter_map_addr->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][filter_neuron_id];
						int32_t synapse_type = memory.rounded_weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];

						if (synapse_type != memory.synapse_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET]) {

							modifySynapse(moduleData, pre_neuron_addr, new_post_neuron_addr, synapse_type,
									synapsePlotInputFeature1, synapsePlotFeature1Feature2,
									synapsePlotFeature2Feature3, synapsePlotFeature3Output);

						}
					}

				}
			}

		}
		else if (post_neuron_chip_id == CHIP_UP_LEFT_ID && post_neuron_core_id != 3 && state->learning_feature3 == true) {

			for (int feature_map_id = 0; feature_map_id < 4; feature_map_id++) {

				int neuron_id = (int) anti_feature_layer3[post_neuron_addr & 0x3ff];

				int new_neuron_id = (int)(neuron_id/256) << 8 |
						( (((int)((neuron_id%256)/16)%8) + (int)(feature_map_id/2)*8)*16 +
								((((neuron_id%256)%16)%8) + (feature_map_id%2)*8) );

				int64_t new_post_neuron_addr = feature_layer3[new_neuron_id];

				if (new_post_neuron_addr != post_neuron_addr) {

					for (int filter_neuron_id = 0;
							filter_neuron_id < memory.filter_map_size->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
							filter_neuron_id++)
					{

						int64_t pre_neuron_addr = memory.filter_map_addr->buffer2d[new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][filter_neuron_id];
						int32_t synapse_type = memory.rounded_weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];

						if (synapse_type != memory.synapse_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][new_post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET]) {

							modifySynapse(moduleData, pre_neuron_addr, new_post_neuron_addr, synapse_type,
									synapsePlotInputFeature1, synapsePlotFeature1Feature2,
									synapsePlotFeature2Feature3, synapsePlotFeature3Output);

						}
					}

				}
			}

		}

	}

	//potential synapse
	if (post_neuron_chip_id == CHIP_UP_LEFT_ID && post_neuron_core_id == 3 && state->learning_output == true) {

		random_number = rand() % 100;
		random_number_threshold = OUTPUT_PROBABILITY * 100;
		if (random_number <= random_number_threshold) {
			int32_t cam_id, current_cam_id;
			current_cam_id = 0;

			for (int filter_neuron_id = 0;
					filter_neuron_id < memory.filter_map_size->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
					filter_neuron_id++)
			{

				int64_t pre_neuron_addr = memory.filter_map_addr->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][filter_neuron_id];
				int32_t synapse_type = memory.rounded_weight_map->buffer2d[pre_neuron_addr -
				                       MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];

				if (synapse_type != NO_SYNAPSE_ID && current_cam_id <= TOTAL_CAM_NUM_LEARNING_O) {

					int64_t replaced_pre_neuron_addr = memory.cam_map_content_source->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][current_cam_id];
					modifyPotentialSynapse(moduleData, replaced_pre_neuron_addr, post_neuron_addr, NO_SYNAPSE_ID, current_cam_id,
							synapsePlotInputFeature1, synapsePlotFeature1Feature2,
							synapsePlotFeature2Feature3, synapsePlotFeature3Output);

					modifyPotentialSynapse(moduleData, pre_neuron_addr, post_neuron_addr, synapse_type, current_cam_id,
							synapsePlotInputFeature1, synapsePlotFeature1Feature2,
							synapsePlotFeature2Feature3, synapsePlotFeature3Output);

					current_cam_id = current_cam_id + 1;

				}
			}

			if (current_cam_id <= TOTAL_CAM_NUM_LEARNING_O) {
				for (cam_id = current_cam_id; cam_id <= TOTAL_CAM_NUM_LEARNING_O; cam_id++) {
					if (memory.cam_map_content_type->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][cam_id] != NO_SYNAPSE_ID) {

						int64_t pre_neuron_addr = memory.cam_map_content_source->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][cam_id];
						modifyPotentialSynapse(moduleData, pre_neuron_addr, post_neuron_addr, NO_SYNAPSE_ID, cam_id,
								synapsePlotInputFeature1, synapsePlotFeature1Feature2,
								synapsePlotFeature2Feature3, synapsePlotFeature3Output);

					}

				}
			}
		}

	}

}

void modifySynapse(caerModuleData moduleData, int64_t pre_neuron_addr, int64_t post_neuron_addr, int32_t synapse_type,
		caerFrameEventPacket *synapsePlotInputFeature1, caerFrameEventPacket *synapsePlotFeature1Feature2,
		caerFrameEventPacket *synapsePlotFeature2Feature3, caerFrameEventPacket *synapsePlotFeature3Output) {
	/* create a new synapse */

	int32_t cam_id = memory.synapse_cam_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET];

	writeCamG(moduleData, (uint32_t) pre_neuron_addr, (uint32_t) post_neuron_addr, 0, (uint32_t) cam_id, synapse_type, 0, 0);
	memory.synapse_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] = synapse_type;
	memory.cam_map_content_type->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][cam_id] = synapse_type;

	if (synapse_type != 0) {
		memory.cam_map->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][cam_id] = 1;
	} else {
		memory.cam_map->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][cam_id] = 0;
	}

	updateSynapsePlotG(pre_neuron_addr, post_neuron_addr, synapse_type,
			synapsePlotInputFeature1, synapsePlotFeature1Feature2, synapsePlotFeature2Feature3, synapsePlotFeature3Output);

//	uint32_t post_neuron_chip_id = (uint32_t) (post_neuron_addr & NEURON_CHIPID_BITS) >> NEURON_CHIPID_SHIFT;
	int8_t chip_id;
	chip_id = (int8_t) getWriteCamChipIdG(post_neuron_addr); //post_neuron_chip_id);

	configureChipG(moduleData, chip_id); //use UBS package to send commands, 100 times faster
}

void modifyPotentialSynapse(caerModuleData moduleData, int64_t pre_neuron_addr, int64_t post_neuron_addr, int32_t synapse_type, int32_t cam_id,
		caerFrameEventPacket *synapsePlotInputFeature1, caerFrameEventPacket *synapsePlotFeature1Feature2,
		caerFrameEventPacket *synapsePlotFeature2Feature3, caerFrameEventPacket *synapsePlotFeature3Output) {

	writeCamG(moduleData, (uint32_t) pre_neuron_addr, (uint32_t) post_neuron_addr, 0, (uint32_t) cam_id, synapse_type, 0, 0);
	memory.synapse_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] = synapse_type;
	memory.cam_map_content_type->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][cam_id] = synapse_type;

	memory.cam_map_content_source->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][cam_id] = (int32_t) pre_neuron_addr;

	if (synapse_type != 0) {
		memory.cam_map->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][cam_id] = 1;
	} else {
		memory.cam_map->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][cam_id] = 0;
	}

	updateSynapsePlotG(pre_neuron_addr, post_neuron_addr, synapse_type,
			synapsePlotInputFeature1, synapsePlotFeature1Feature2, synapsePlotFeature2Feature3, synapsePlotFeature3Output);

	int8_t chip_id;
	chip_id = (int8_t) getWriteCamChipIdG(post_neuron_addr);

	configureChipG(moduleData, chip_id);
}

void updateSynapsePlotG(int64_t pre_neuron_addr, int64_t post_neuron_addr, int32_t new_synapse,
		caerFrameEventPacket *synapsePlotInputFeature1, caerFrameEventPacket *synapsePlotFeature1Feature2,
		caerFrameEventPacket *synapsePlotFeature2Feature3, caerFrameEventPacket *synapsePlotFeature3Output)
{
	/* update synapse plots */
	uint32_t counterS;
	COLOUR col_s;
	int64_t filter_row_id, filter_col_id, feature_row_id, feature_col_id, feature_map_row_id, feature_map_col_id;
	caerFrameEvent singleplotS;

	uint32_t post_neuron_chip_id = (uint32_t) (post_neuron_addr & NEURON_CHIPID_BITS) >> NEURON_CHIPID_SHIFT;
	int64_t post_neuron_core_id = (post_neuron_addr & NEURON_COREID_BITS) >> NEURON_COREID_SHIFT;

	//for feature layers
	if (post_neuron_chip_id == CHIP_UP_RIGHT_ID) { //post_neuron_chip_id == CHIP_DOWN_LEFT_ID ||
		caerFrameEventPacket *synapseplotfeature;
		synapseplotfeature = synapsePlotInputFeature1;

		if (*synapseplotfeature != NULL) {
			singleplotS = caerFrameEventPacketGetEvent(*synapseplotfeature, 0);

			col_s  = getColourSG((float)new_synapse);

			int pre_id;
			int post_id;
			pre_id = decodeInputLayerNeuronAddress(pre_neuron_addr) & 0x3ff;
			post_id = (int) anti_feature_layer1[post_neuron_addr & 0x3ff];

			filter_row_id = 1 + ((int)(post_id/28)%FEATURE_LAYER1_W) * (FILTER1_L+1) +
					(int)(pre_id/INPUT_LAYER_W)-((int)(post_id/28)%FEATURE_LAYER1_L)*FILTER1_STEP_L;
			filter_col_id = 1 + ((post_id%28)%FEATURE_LAYER1_W) * (FILTER1_W+1) +
					(pre_id%INPUT_LAYER_W) - ((post_id%28)%FEATURE_LAYER1_W)*FILTER1_STEP_W;

			feature_row_id = (int)((int)(post_id/28) / FEATURE_LAYER1_L) * VISUALIZER_FEATURE1_MAP_X;
			feature_col_id = (int)((post_id%28) / FEATURE_LAYER1_W) * VISUALIZER_FEATURE1_MAP_Y;

			counterS = (uint32_t) (((filter_col_id + feature_col_id) * 544) + (filter_row_id + feature_row_id)) * 3;
			singleplotS->pixels[counterS] = col_s.r;
			singleplotS->pixels[counterS + 1] = col_s.g;
			singleplotS->pixels[counterS + 2] = col_s.b;
		}
	}
	else if (post_neuron_chip_id == CHIP_DOWN_LEFT_ID) {
		caerFrameEventPacket *synapseplotfeature;
		synapseplotfeature = synapsePlotFeature1Feature2;

		if (*synapseplotfeature != NULL) {
			singleplotS = caerFrameEventPacketGetEvent(*synapseplotfeature, 0);

			col_s  = getColourSG((float)new_synapse);

			int pre_id;
			int post_id;
			pre_id = (int) anti_feature_layer1[pre_neuron_addr & 0x3ff];
			post_id = (int) anti_feature_layer2[post_neuron_addr & 0x3ff];

			filter_row_id = 1 + (int)((int)(pre_id/28) / FEATURE_LAYER1_L) * (FILTER2_L+1) +
					(int)(pre_id/28)%7-((int)(post_id/30)%FEATURE_LAYER2_L)*FILTER2_STEP_L;
			filter_col_id = 1 + (int)((pre_id%28) / FEATURE_LAYER1_W) * (FILTER2_W+1) +
					(pre_id%28)%7 - ((post_id%30)%FEATURE_LAYER2_W)*FILTER2_STEP_W;

			feature_map_row_id = ((int)(post_id/30) % FEATURE_LAYER2_L) * VISUALIZER_FEATURE2_MAP_X;
			feature_map_col_id = (int)((post_id%30) % FEATURE_LAYER2_W) * VISUALIZER_FEATURE2_MAP_Y;

			int feature_maps_row_id = (int)((int)(post_id/30) / FEATURE_LAYER2_L) * VISUALIZER_FEATURE2_MAPS_X;
			int feature_maps_col_id = (int)((post_id%30) / FEATURE_LAYER2_W) * VISUALIZER_FEATURE2_MAPS_Y;

			counterS = (uint32_t) (((filter_col_id + feature_map_col_id + feature_maps_col_id) * 544) +
					(filter_row_id + feature_map_row_id + feature_maps_row_id)) * 3;
			singleplotS->pixels[counterS] = col_s.r;
			singleplotS->pixels[counterS + 1] = col_s.g;
			singleplotS->pixels[counterS + 2] = col_s.b;
		}
	}
	else if (post_neuron_chip_id == CHIP_UP_LEFT_ID && post_neuron_core_id != CORE_DOWN_RIGHT_ID) {
		caerFrameEventPacket *synapseplotfeature;
		synapseplotfeature = synapsePlotFeature2Feature3;

		if (*synapseplotfeature != NULL) {
			singleplotS = caerFrameEventPacketGetEvent(*synapseplotfeature, 0);

			col_s  = getColourSG((float)new_synapse);

			int pre_id;
			int post_id;
			pre_id = memory.synapse_cam_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET]; //(int) anti_feature_layer2[pre_neuron_addr & 0x3ff];
			post_id = (int) anti_feature_layer3[post_neuron_addr & 0x3ff];

			filter_row_id = 1 + (int)(pre_id / 8) + ((int)((post_id%256)/16)%8)*(8+1); //
			filter_col_id = 1 + pre_id % 8 + (((post_id%256)%16)%8)*(8+1); //

			feature_map_row_id = ((int)(post_id/256) >> 1) * VISUALIZER_FEATURE3_MAP_X;
			feature_map_col_id = ((int)(post_id/256) & 0x1) * VISUALIZER_FEATURE3_MAP_Y;

			int feature_maps_row_id = (int)((int)((post_id%256)/16)/8) * VISUALIZER_FEATURE3_MAPS_X;
			int feature_maps_col_id = (int)(((post_id%256)%16)/8) * VISUALIZER_FEATURE3_MAPS_Y;

			counterS = (uint32_t) (((filter_col_id + feature_map_col_id + feature_maps_col_id) * VISUALIZER_Y_FEATURE3) +
					(filter_row_id + feature_map_row_id + feature_maps_row_id)) * 3;
			singleplotS->pixels[counterS] = col_s.r;
			singleplotS->pixels[counterS + 1] = col_s.g;
			singleplotS->pixels[counterS + 2] = col_s.b;
		}
	}
	else if (post_neuron_chip_id == CHIP_UP_LEFT_ID && post_neuron_core_id == CORE_DOWN_RIGHT_ID) {
		caerFrameEventPacket *synapseplotoutput;
		synapseplotoutput = synapsePlotFeature3Output;

		if (*synapseplotoutput != NULL) {
			singleplotS = caerFrameEventPacketGetEvent(*synapseplotoutput, 0);

			col_s  = getColourSG((float)new_synapse);

			int pre_id;
			int post_id;
			pre_id = (int) anti_feature_layer3[pre_neuron_addr & 0x3ff];
			post_id = (int) anti_output_layer[post_neuron_addr & 0x3ff];

			filter_row_id = 1 + (int)((pre_id%256)/16) + ((int)(pre_id/256) >> 1) * (16+1);
			filter_col_id = 1 + (pre_id%256)%16 + ((int)(pre_id/256) & 0x1) * (16+1);

			feature_map_row_id = ((int)(post_id/4)%2) * VISUALIZER_OUTPUT_MAP_X;
			feature_map_col_id = ((post_id%4)%2) * VISUALIZER_OUTPUT_MAP_Y;

			int feature_maps_row_id = (int)((int)(post_id/4)/2) * VISUALIZER_OUTPUT_MAPS_X;
			int feature_maps_col_id = (int)((post_id%4)/2) * VISUALIZER_OUTPUT_MAPS_Y;

			counterS = (uint32_t) (((filter_col_id + feature_map_col_id + feature_maps_col_id) * VISUALIZER_Y_OUTPUT) +
					(filter_row_id + feature_map_row_id + feature_maps_row_id)) * 3;
			singleplotS->pixels[counterS] = col_s.r;
			singleplotS->pixels[counterS + 1] = col_s.g;
			singleplotS->pixels[counterS + 2] = col_s.b;
		}
	}
}

//--------------------------------------------------------RESET NETWORK AND BASIC FUNCTIONS--------------------------------------------------------
//reset the network and the learning algorithm to the initial state
bool resetNetworkG(caerModuleData moduleData) {

	memory.spike_cnt = 0;

	memory.package_spike_counter = 0;

	stimuli_pattern = 0;

	disableStimuliGenG(moduleData);
//	resetBiasesG(moduleData);
	resetBiasesGDC(moduleData);

	time_cnt = 0; //initialize time counter for changing input stimuli patterns
	signal(SIGALRM, signalHandlerG); //register the hand-made timer function
	setTimerG();

	if (file_input_enable == 0)
		clearAllCam(moduleData); //only for 1st chip
	else
		clearAllCam(moduleData);

	GFilterState state = moduleData->moduleState;
//	state->learning = false;
	int8_t exType = state->resetExType; //initial synapse type fast or slow
	int8_t inType = state->resetInType; //initial synapse type fast or slow

	//connection between neurons
	memory.connection_map = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) TOTAL_NEURON_NUM_ON_BOARD);
	memory.weight_map = simple2DBufferInitDouble((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) TOTAL_NEURON_NUM_ON_BOARD);
	memory.weight_map_temp = simple2DBufferInitDouble((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) TOTAL_NEURON_NUM_ON_BOARD);
	memory.rounded_weight_map = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) TOTAL_NEURON_NUM_ON_BOARD);
	memory.synapse_map = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) TOTAL_NEURON_NUM_ON_BOARD);
	memory.synapse_cam_map = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) TOTAL_NEURON_NUM_ON_BOARD);

	//CAM state
	memory.cam_map = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) TOTAL_CAM_NUM); //CAM taken or not
	memory.cam_map_size = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) CAM_MAP_SIZE_WIDTH); //available CAM number
	memory.cam_map_content_source = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) TOTAL_CAM_NUM);
	memory.cam_map_content_type = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) TOTAL_CAM_NUM);

	//SRAM state
	memory.sram_map = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) TOTAL_SRAM_NUM); //SRAM taken or not
	memory.sram_map_content = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) TOTAL_SRAM_NUM); //destChip, sourceCore, DestCore Id

	//filter state
	memory.filter_map_addr = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) MAXIMUM_FILTER_SIZE); //store pre_neuron_addr
	memory.filter_map_cam_id = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) MAXIMUM_FILTER_SIZE); //store CAM address
	memory.filter_map_size = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) FILTER_MAP_SIZE_WIDTH); //filter's current size
	memory.filter_map_highest = simple2DBufferInitDouble((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) FILTER_MAP_HIGHEST_WIDTH); //filter's weight sum
	memory.filter_map_lowest = simple2DBufferInitDouble((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) FILTER_MAP_LOWEST_WIDTH);

	//spike queue
	memory.spike_fifo = simple2DBufferInitLong((size_t) SPIKE_QUEUE_LENGTH, (size_t) SPIKE_QUEUE_WIDTH);

	//output map
	memory.outputMap = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) OUTPUT_LAYER_NUM);
	memory.outputMapDisabled = simple2DBufferInitInt((size_t) TOTAL_NEURON_NUM_ON_BOARD, (size_t) OUTPUT_LAYER_NUM);

	//create the connectivity of the network
	uint32_t chip_id, core_id, random_core_id, neuron_id, address, count_id;

	//create input layer
	for (neuron_id = 0; neuron_id < INPUT_LAYER_N; neuron_id++) {
		chip_id = CHIP_DOWN_RIGHT_ID; //CHIP_UP_LEFT_ID; //CHIP_DOWN_RIGHT_ID;
		int64_t encoded_neuron_address = encodeInputLayerNeuronAddress(neuron_id);
		address = chip_id << NEURON_CHIPID_SHIFT | (uint32_t) encoded_neuron_address;
		input_layer[neuron_id] = address;
		anti_input_layer[address & 0xff] = neuron_id;
	}

	//create feature layer 1
	count_id = 0;
	for (neuron_id = 0; neuron_id < 1024; neuron_id++) {
		chip_id = CHIP_UP_RIGHT_ID;
		if ((neuron_id % 32) % 8 != 7 && (int)(neuron_id / 32) % 8 != 7) {
			int64_t encoded_neuron_address = encodeInputLayerNeuronAddress(neuron_id);
			address = chip_id << NEURON_CHIPID_SHIFT | encoded_neuron_address;
			feature_layer1[count_id] = address;
			anti_feature_layer1[address & 0x3ff] = count_id;
			count_id += 1;
		}
	}

	//create feature layer 2
	count_id = 0;
	for (neuron_id = 0; neuron_id < 1024; neuron_id++) {
		chip_id = CHIP_DOWN_LEFT_ID;
		if ((neuron_id % 32) != 30 && (neuron_id % 32) != 31 &&
				(int)(neuron_id / 32) != 30 && (int)(neuron_id / 32) != 31) {
			int64_t encoded_neuron_address = encodeInputLayerNeuronAddress(neuron_id);
			address = chip_id << NEURON_CHIPID_SHIFT | encoded_neuron_address;
			feature_layer2[count_id] = address;
			anti_feature_layer2[address & 0x3ff] = count_id;
			count_id += 1;
		}
	}

	//create output layer 1
	count_id = 0;
	for (neuron_id = 0; neuron_id < FEATURE_LAYER3_N * FEATURE_LAYER3_NUM; neuron_id++) {
		chip_id = CHIP_UP_LEFT_ID;
		address = chip_id << NEURON_CHIPID_SHIFT | neuron_id;
		feature_layer3[count_id] = address;
		anti_feature_layer3[address & 0x3ff] = count_id;
		count_id += 1;
	}

	//create output layer 2
/*	count_id = 0;
	for (neuron_id = 0; neuron_id < 256; neuron_id++) {
		chip_id = CHIP_UP_LEFT_ID;
		core_id = CORE_DOWN_RIGHT_ID;
		if (((int)(neuron_id / 16) % 8) < 2 && (neuron_id % 16) % 8 < 2) {
			address = chip_id << NEURON_CHIPID_SHIFT | core_id << NEURON_COREID_SHIFT | neuron_id;
			output_layer[count_id] = address;
			anti_output_layer[address & 0x3ff] = count_id;
			count_id += 1;
		}
	}
*/
	//create output layer 3
	count_id = 0;
	for (neuron_id = 0; neuron_id < 256; neuron_id++) {
		chip_id = CHIP_UP_LEFT_ID;
		core_id = CORE_DOWN_RIGHT_ID;
		if (((int)(neuron_id / 16) % 8) < 2 && (neuron_id % 16) % 8 < 2) {
			address = chip_id << NEURON_CHIPID_SHIFT | core_id << NEURON_COREID_SHIFT | neuron_id;
			output_layer[count_id] = address;
			anti_output_layer[address & 0x3ff] = count_id;
			count_id += 1;
		}
	}

	int pre_neuron_id, post_neuron_id, post_output_id, pre_output_id;
	uint32_t pre_neuron_addr, post_neuron_addr;
	int randNumCount, randNumCountEI;
	uint32_t virtual_neuron_addr = 0;
	int8_t virtual_neuron_addr_enable = 0;

	//from input layer to feature layer 1
	virtual_neuron_addr_enable = 0;
	for (post_neuron_id = 0; post_neuron_id < FEATURE_LAYER1_N*FEATURE_LAYER1_NUM; post_neuron_id++) { //first sweep POST, then PRE
		//generate random binary number 1D array
		int64_t rand1DBinaryArrayEI[TOTAL_CAM_NUM_LEARNING_F1]; //FILTER1_N-FEATURE1_CAM_INHIBITORY_N
		getRand1DBinaryArrayG(rand1DBinaryArrayEI, TOTAL_CAM_NUM_LEARNING_F1, TOTAL_CAM_NUM_LEARNING_F1/2);
		randNumCountEI = 0;
		for (pre_neuron_id = 0; pre_neuron_id < INPUT_LAYER_N; pre_neuron_id++) {
			int pre_id;
			int post_id;
			pre_id = pre_neuron_id & 0x3ff;
			post_id = post_neuron_id & 0x3ff;
			if ((int)(pre_id/INPUT_LAYER_W) >= ((int)(post_id/28)%FEATURE_LAYER1_L)*FILTER1_STEP_L
					&& (int)(pre_id/INPUT_LAYER_W) < ((int)(post_id/28)%FEATURE_LAYER1_L)*FILTER1_STEP_L + FILTER1_L
					&& pre_id%INPUT_LAYER_W >= ((post_id%28)%FEATURE_LAYER1_W)*FILTER1_STEP_W
					&& pre_id%INPUT_LAYER_W < ((post_id%28)%FEATURE_LAYER1_W)*FILTER1_STEP_W + FILTER1_W)
			{
				//randomly reset, depends on the ratio of total CAM number and FILTER1_N-FEATURE1_CAM_INHIBITORY_N
				pre_neuron_addr = input_layer[pre_neuron_id];
				post_neuron_addr = feature_layer1[post_neuron_id];
				if (rand1DBinaryArrayEI[randNumCountEI] == 1) {
					buildSynapseG(moduleData, pre_neuron_addr, post_neuron_addr, virtual_neuron_addr,
							FAST_EX_SYNAPSE_VALUE, REAL_SYNAPSE, virtual_neuron_addr_enable, FIRST_CAM_ID); //SLOW_EX_SYNAPSE_VALUE
				} else {
					buildSynapseG(moduleData, pre_neuron_addr, post_neuron_addr, virtual_neuron_addr,
							FAST_IN_SYNAPSE_VALUE, REAL_SYNAPSE, virtual_neuron_addr_enable, FIRST_CAM_ID); //SLOW_IN_SYNAPSE_VALUE
				}
				randNumCountEI += 1;
			}
		}
	}

	//from feature layer 1 to feature layer 2
	virtual_neuron_addr_enable = 0;
	for (post_neuron_id = 0; post_neuron_id < FEATURE_LAYER2_N*FEATURE_LAYER2_NUM; post_neuron_id++) { //first sweep POST, then PRE
		//generate random binary number 1D array
		int64_t rand1DBinaryArrayEI[TOTAL_CAM_NUM_LEARNING_F2]; //FILTER1_N-FEATURE1_CAM_INHIBITORY_N
		getRand1DBinaryArrayG(rand1DBinaryArrayEI, TOTAL_CAM_NUM_LEARNING_F2, (int)(TOTAL_CAM_NUM_LEARNING_F2/2));

		int64_t rand1DBinaryArray[FILTER2_N*FEATURE_LAYER1_NUM]; //FILTER1_N-FEATURE1_CAM_INHIBITORY_N
		getRand1DBinaryArrayG(rand1DBinaryArray, FILTER2_N*FEATURE_LAYER1_NUM, TOTAL_CAM_NUM_LEARNING_F2);

		randNumCount = 0;
		randNumCountEI = 0;
		for (pre_neuron_id = 0; pre_neuron_id < FEATURE_LAYER1_N*FEATURE_LAYER1_NUM; pre_neuron_id++) {
			int pre_id;
			int post_id;
			pre_id = pre_neuron_id & 0x3ff;
			post_id = post_neuron_id & 0x3ff;
			if ((int)(pre_id/28) % 7 >= ((int)(post_id/30)%FEATURE_LAYER2_L)*FILTER2_STEP_L
					&& (int)(pre_id/28) % 7 < ((int)(post_id/30)%FEATURE_LAYER2_L)*FILTER2_STEP_L + FILTER2_L
					&& (pre_id%28) % 7 >= ((post_id%30)%FEATURE_LAYER2_W)*FILTER2_STEP_W
					&& (pre_id%28) % 7 < ((post_id%30)%FEATURE_LAYER2_W)*FILTER2_STEP_W + FILTER2_W)
			{
				//randomly reset, depends on the ratio of total CAM number and FILTER1_N-FEATURE1_CAM_INHIBITORY_N
				pre_neuron_addr = feature_layer1[pre_neuron_id];
				post_neuron_addr = feature_layer2[post_neuron_id];
				if (rand1DBinaryArray[randNumCount] == 1) { //rand1DBinaryArray[randNumCount] == 1
					if (rand1DBinaryArrayEI[randNumCountEI] == 1) {
						buildSynapseG(moduleData, pre_neuron_addr, post_neuron_addr, virtual_neuron_addr,
								FAST_EX_SYNAPSE_VALUE, REAL_SYNAPSE, virtual_neuron_addr_enable, FIRST_CAM_ID); //SLOW_EX_SYNAPSE_VALUE
					} else {
						buildSynapseG(moduleData, pre_neuron_addr, post_neuron_addr, virtual_neuron_addr,
								FAST_IN_SYNAPSE_VALUE, REAL_SYNAPSE, virtual_neuron_addr_enable, FIRST_CAM_ID); //SLOW_IN_SYNAPSE_VALUE
					}
					randNumCountEI += 1;
				}
				randNumCount += 1;
			}
		}
	}

	//for feature layer 2 to feature layer 3
	virtual_neuron_addr_enable = 0;
	for (post_neuron_id = 0; post_neuron_id < FEATURE_LAYER3_N * FEATURE_LAYER3_NUM / 4; post_neuron_id++) {

		int64_t rand1DBinaryArrayEI[TOTAL_CAM_NUM_LEARNING_F3]; //FILTER1_N-FEATURE1_CAM_INHIBITORY_N
		getRand1DBinaryArrayG(rand1DBinaryArrayEI, TOTAL_CAM_NUM_LEARNING_F3, (int)(TOTAL_CAM_NUM_LEARNING_F3/2)); //TOTAL_CAM_NUM_LEARNING_O/2

		int64_t rand1DBinaryArray[FEATURE_LAYER2_N*FEATURE_LAYER2_NUM]; //FILTER1_N-FEATURE1_CAM_INHIBITORY_N
		getRand1DBinaryArrayG(rand1DBinaryArray, FEATURE_LAYER2_N*FEATURE_LAYER2_NUM, TOTAL_CAM_NUM_LEARNING_F3);

		for (post_output_id = 0; post_output_id < 4; post_output_id++) {

			randNumCount = 0;
			randNumCountEI = 0;
			for (pre_neuron_id = 0; pre_neuron_id < FEATURE_LAYER2_N*FEATURE_LAYER2_NUM; pre_neuron_id++) {

				pre_neuron_addr = feature_layer2[pre_neuron_id];
				post_neuron_addr = feature_layer3[(int)(post_neuron_id/64) << 8 |
				                                 (((int)((post_neuron_id%64)/8) + (int)(post_output_id/2)*8)*16 +
				                                		 ((int)((post_neuron_id%64)%8) + (int)(post_output_id%2)*8))];

				if (rand1DBinaryArray[randNumCount] == 1) { //&& (pre_neuron_addr & 0xff) != 255 && (pre_neuron_addr & 0xff) != 254
					if (rand1DBinaryArrayEI[randNumCountEI] == 1) {
						buildSynapseG(moduleData, pre_neuron_addr, post_neuron_addr, virtual_neuron_addr,
								FAST_EX_SYNAPSE_VALUE, REAL_SYNAPSE, virtual_neuron_addr_enable, FIRST_CAM_ID); //SLOW_EX_SYNAPSE_VALUE
					} else {
						buildSynapseG(moduleData, pre_neuron_addr, post_neuron_addr, virtual_neuron_addr,
								FAST_IN_SYNAPSE_VALUE, REAL_SYNAPSE, virtual_neuron_addr_enable, FIRST_CAM_ID); //SLOW_IN_SYNAPSE_VALUE NO_SYNAPSE_ID
					}
					randNumCountEI += 1;
				}
				randNumCount += 1;
			}

		}

	}

	//for feature layer 3 to output layer
	virtual_neuron_addr_enable = 0;
	for (post_neuron_id = 0; post_neuron_id < OUTPUT_LAYER_N; post_neuron_id++) {

		int64_t rand1DBinaryArray[FEATURE_LAYER3_N * FEATURE_LAYER3_NUM];
		getRand1DBinaryArrayG(rand1DBinaryArray, FEATURE_LAYER3_N * FEATURE_LAYER3_NUM, TOTAL_CAM_NUM_LEARNING_O);

		for (post_output_id = 0; post_output_id < OUTPUT_LAYER_NUM; post_output_id++) {

			randNumCount = 0;
			for (pre_neuron_id = 0; pre_neuron_id < FEATURE_LAYER3_N * FEATURE_LAYER3_NUM; pre_neuron_id++) {

				pre_neuron_addr = feature_layer3[pre_neuron_id];
				post_neuron_addr = output_layer[((int)(post_neuron_id/2) + (post_output_id/2)*2)*4 +
				                                ((post_neuron_id%2) + (post_output_id%2)*2)];

				if (rand1DBinaryArray[randNumCount] == 1) { //&& (pre_neuron_addr & 0xff) != 255 && (pre_neuron_addr & 0xff) != 254
					buildSynapseG(moduleData, pre_neuron_addr, post_neuron_addr, virtual_neuron_addr,
							NO_SYNAPSE_ID, REAL_SYNAPSE, virtual_neuron_addr_enable, FIRST_CAM_ID); //FAST_EX_SYNAPSE_VALUE //NO_SYNAPSE_ID
				}
				else {
					buildSynapseG(moduleData, pre_neuron_addr, post_neuron_addr, virtual_neuron_addr,
							NO_SYNAPSE_ID, VIRTUAL_SYNAPSE, virtual_neuron_addr_enable, FIRST_CAM_ID);
				}
				randNumCount += 1;

			}

		}

	}

	//for output layer to output layer, EX and IN
	virtual_neuron_addr_enable = 0;
	for (pre_output_id = 0; pre_output_id < OUTPUT_LAYER_NUM; pre_output_id++) {
		for (post_output_id = 0; post_output_id < OUTPUT_LAYER_NUM; post_output_id++) {

			if (pre_output_id == post_output_id) {

				for (post_neuron_id = 0; post_neuron_id < OUTPUT_LAYER_N; post_neuron_id++) {

					for (pre_neuron_id = 0; pre_neuron_id < OUTPUT_LAYER_N; pre_neuron_id++) {

						if (pre_neuron_id != post_neuron_id) {
							pre_neuron_addr = output_layer[((int)(pre_output_id/2)*2 + (int)(pre_neuron_id/2))*4 +
							                               (pre_output_id%2)*2 + pre_neuron_id%2];
							post_neuron_addr = output_layer[((int)(post_output_id/2)*2 + (int)(post_neuron_id/2))*4 +
							                                (post_output_id%2)*2 + post_neuron_id%2];

							buildSynapseG(moduleData, pre_neuron_addr, post_neuron_addr, virtual_neuron_addr,
									FAST_EX_SYNAPSE_VALUE, REAL_SYNAPSE_WITHOUT_LEARNING, virtual_neuron_addr_enable, FIRST_CAM_ID); //SLOW_EX_SYNAPSE_VALUE
						}

					}

				}

			} else {

				for (post_neuron_id = 0; post_neuron_id < OUTPUT_LAYER_N; post_neuron_id++) {

					for (pre_neuron_id = 0; pre_neuron_id < OUTPUT_LAYER_N; pre_neuron_id++) {

						pre_neuron_addr = output_layer[((int)(pre_output_id/2)*2 + (int)(pre_neuron_id/2))*4 +
						                               (pre_output_id%2)*2 + pre_neuron_id%2];
						post_neuron_addr = output_layer[((int)(post_output_id/2)*2 + (int)(post_neuron_id/2))*4 +
						                                (post_output_id%2)*2 + post_neuron_id%2];

						buildSynapseG(moduleData, pre_neuron_addr, post_neuron_addr, virtual_neuron_addr,
								FAST_IN_SYNAPSE_VALUE, REAL_SYNAPSE_WITHOUT_LEARNING, virtual_neuron_addr_enable, FIRST_CAM_ID);

					}

				}
			}
		}
	}

	setOutputLayerCamG(moduleData); //It's different thread, should be put in the end.

	if (file_input_enable == 0)
		setInputLayerCamG(moduleData);
//	else
//		setFileInputLayerCamG(moduleData);
//	setInputLayerCamG(moduleData);

	configureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U0);
	configureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U1);
	configureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U2);
	configureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U3);

//	resetBiasesG(moduleData);

	//enable the teaching signal
	enableTeachingSignalG(moduleData);
	enableTeachingG(moduleData);

	memory.wr_pointer = 0;
	memory.post_rd_pointer = 0;

//	state->learning = true;
//	state->learning_feature1 = true;

	for (neuron_id = 0; neuron_id < FEATURE_LAYER1_N * FEATURE_LAYER1_NUM; neuron_id++) {
		post_neuron_addr = feature_layer1[neuron_id];
		memory.filter_map_highest->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] = 1;
		memory.filter_map_lowest->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] = -1;
	}
	for (neuron_id = 0; neuron_id < FEATURE_LAYER1_N * FEATURE_LAYER1_NUM; neuron_id++) {
		post_neuron_addr = feature_layer2[neuron_id];
		memory.filter_map_highest->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] = 1;
		memory.filter_map_lowest->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] = -1;
	}

	resetBiasesG(moduleData);

	enableFileStimuliRunG(moduleData);

	return (true);
}

bool saveNeuronForTeaching(uint32_t pre_neuron_addr, uint32_t post_neuron_addr) {
	if ((post_neuron_addr & 0xff) == ((0 << 4) | 0) || (post_neuron_addr & 0xff) == ((0 << 4) | 8) ||
			(post_neuron_addr & 0xff) == ((8 << 4) | 0) || (post_neuron_addr & 0xff) == ((8 << 4) | 8)) {
		return (true);
	} else if ((pre_neuron_addr & 0xff) == ((0 << 4) | 0) || (pre_neuron_addr & 0xff) == ((0 << 4) | 8) ||
			(pre_neuron_addr & 0xff) == ((8 << 4) | 0) || (pre_neuron_addr & 0xff) == ((8 << 4) | 8)) {
		return (true);
	} else {
		return (false);
	}
}

void setOutputLayerCamG(caerModuleData moduleData) {

	int output_id, neuron_id;
	uint32_t chip_id, neuron_address;
	uint32_t sourceAddress, destAddress, cam_id;

	chip_id = CHIP_UP_LEFT_ID;

	for (neuron_id = 0; neuron_id < OUTPUT_LAYER_N * OUTPUT_LAYER_NUM; neuron_id++) {

		output_id = (int)((int)(neuron_id/4)/2) << 1 | (int)((neuron_id%4)/2);

		neuron_address = output_layer[neuron_id];
		destAddress = chip_id << NEURON_CHIPID_SHIFT | neuron_address;

		if (output_id == 0) {
			sourceAddress = chip_id << NEURON_CHIPID_SHIFT | 3 << NEURON_COREID_SHIFT | 251;
		} else if (output_id == 1) {
			sourceAddress = chip_id << NEURON_CHIPID_SHIFT | 3 << NEURON_COREID_SHIFT | 252;
		} else if (output_id == 2) {
			sourceAddress = chip_id << NEURON_CHIPID_SHIFT | 3 << NEURON_COREID_SHIFT | 253;
		} else {
			sourceAddress = chip_id << NEURON_CHIPID_SHIFT | 3 << NEURON_COREID_SHIFT | 254;
		}

		cam_id = 63;
		writeCamG(moduleData, sourceAddress, destAddress, 0, cam_id, 2, 0, 0);

		if (output_id == 0) {
			sourceAddress = chip_id << NEURON_CHIPID_SHIFT | 3 << NEURON_COREID_SHIFT | 241;
		} else if (output_id == 1) {
			sourceAddress = chip_id << NEURON_CHIPID_SHIFT | 3 << NEURON_COREID_SHIFT | 242;
		} else if (output_id == 2) {
			sourceAddress = chip_id << NEURON_CHIPID_SHIFT | 3 << NEURON_COREID_SHIFT | 243;
		} else {
			sourceAddress = chip_id << NEURON_CHIPID_SHIFT | 3 << NEURON_COREID_SHIFT | 244;
		}

		cam_id = 62;
		writeCamG(moduleData, sourceAddress, destAddress, 0, cam_id, -2, 0, 0);

	}
}

void setInputLayerCamG(caerModuleData moduleData) {

	int64_t rowId, colId;
	int64_t num = 32/2; //DYNAPSE_CONFIG_NUMCAM;
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
			if (((cx - rowId) * (cx - rowId)
					+ (cy - colId) * (cy - colId) <= r * r + sqrt(r))
					&& ((cx - rowId) * (cx - rowId)
							+ (cy - colId) * (cy - colId) >= r * r - r))
				spikePatternA[rowId][colId] = 1;

	uint32_t spikePatternB[DYNAPSE_CONFIG_XCHIPSIZE][DYNAPSE_CONFIG_YCHIPSIZE];
	for (rowId = -num; rowId < num; rowId++) {
		for (colId = -num; colId < num; colId++) {
			if (abs((int) rowId) + abs((int) colId) == num) // Change this condition >= <=
				spikePatternB[rowId + DYNAPSE_CONFIG_XCHIPSIZE/2][colId + DYNAPSE_CONFIG_YCHIPSIZE/2] = 1;
			else
				spikePatternB[rowId + DYNAPSE_CONFIG_XCHIPSIZE/2][colId + DYNAPSE_CONFIG_YCHIPSIZE/2] = 0;
		}
	}

	uint32_t spikePatternC[DYNAPSE_CONFIG_XCHIPSIZE][DYNAPSE_CONFIG_YCHIPSIZE];
	for (rowId = -num; rowId < num; rowId++) {
		for (colId = -num; colId < num; colId++) {
			if (abs((int) rowId) == abs((int) colId)) // Change this condition
				spikePatternC[rowId + DYNAPSE_CONFIG_XCHIPSIZE/2][colId + DYNAPSE_CONFIG_YCHIPSIZE/2] = 1;
			else
				spikePatternC[rowId + DYNAPSE_CONFIG_XCHIPSIZE/2][colId + DYNAPSE_CONFIG_YCHIPSIZE/2] = 0;
		}
	}

	uint32_t neuron_id;
	uint32_t i;

	for (rowId = 0; rowId < DYNAPSE_CONFIG_XCHIPSIZE; rowId++) {
		for (colId = 0; colId < DYNAPSE_CONFIG_YCHIPSIZE; colId++) {
			if (pattern_style == 3) {
				neuron_id = (uint32_t) getNeuronId(4, rowId, colId);
				if (spikePatternA[rowId][colId] == 1)
					writeCamG(moduleData, 1, neuron_id, 0, 0, 2, 0, 0);
				if (spikePatternB[rowId][colId] == 1)
					writeCamG(moduleData, 2, neuron_id, 0, 1, 2, 0, 0);
				if (spikePatternC[rowId][colId] == 1)
					writeCamG(moduleData, 3, neuron_id, 0, 2, 2, 0, 0);
			} else if (pattern_style == 4) {
				neuron_id = (uint32_t) getNeuronId(4, rowId, colId);
				if (spikePatternB[rowId][colId] == 1 && rowId <= 16)
					writeCamG(moduleData, 1, neuron_id, 0, 0, 2, 0, 0);
				if (spikePatternB[rowId][colId] == 1 && colId <= 16)
					writeCamG(moduleData, 2, neuron_id, 0, 1, 2, 0, 0);
				if (spikePatternB[rowId][colId] == 1 && rowId >= 16)
					writeCamG(moduleData, 3, neuron_id, 0, 2, 2, 0, 0);
				if (spikePatternB[rowId][colId] == 1 && colId >= 16)
					writeCamG(moduleData, 4, neuron_id, 0, 3, 2, 0, 0);
			} else if (pattern_style == 5) {
				if (spikePatternB[rowId][colId] == 1 && rowId <= 16) {
					for (i = 0; i < 16; i++) {
						neuron_id = (uint32_t) getNeuronId(4, rowId + i, colId);
						writeCamG(moduleData, i, neuron_id, 0, i, 2, 0, 0);
					}
				}
				if (spikePatternB[rowId][colId] == 1 && colId <= 16) {
					for (i = 0; i < 16; i++) {
						neuron_id = (uint32_t) getNeuronId(4, rowId, colId + i);
						writeCamG(moduleData, 16+i, neuron_id, 0, 16+i, 2, 0, 0);
					}
				}
				if (spikePatternB[rowId][colId] == 1 && rowId >= 16) {
					for (i = 0; i < 16; i++) {
						neuron_id = (uint32_t) getNeuronId(4, rowId - i, colId);
						writeCamG(moduleData, 16*2+i, neuron_id, 0, 16*2+i, 2, 0, 0);
					}
				}
				if (spikePatternB[rowId][colId] == 1 && colId >= 16) {
					for (i = 0; i < 16; i++) {
						neuron_id = (uint32_t) getNeuronId(4, rowId, colId - i);
						writeCamG(moduleData, 16*3+i, neuron_id, 0, 16*3+i, 2, 0, 0);
					}
				}
			}
		}
	}
}

int64_t getNeuronId(int64_t chip_id, int64_t rowId, int64_t colId) {
	int64_t neuron_id;
	neuron_id = chip_id << 10 | ((rowId & 0X10) >> 4) << 9 | ((colId & 0X10) >> 4) << 8 |(rowId & 0xf) << 4 | (colId & 0xf);
	return neuron_id;
}

int64_t encodeInputLayerNeuronAddress(int64_t neuron_address) {
	int64_t encoded_neuron_address;
	encoded_neuron_address = (neuron_address & 0xf) | ((neuron_address & 0x1e0) >> 5) << 4 |
			((neuron_address & 0x10) >> 4) << 8 | ((neuron_address & 0x200) >> 9) << 9;
	return(encoded_neuron_address);
}

int64_t decodeInputLayerNeuronAddress(int64_t neuron_address) {
	int64_t decoded_neuron_address;
	decoded_neuron_address = (neuron_address & 0xf) | ((neuron_address & 0x100) >> 8) << 4 |
			((neuron_address & 0xf0) >> 4) << 5 | ((neuron_address & 0x200) >> 9) << 9;
	return(decoded_neuron_address);
}

void clearAllCam(caerModuleData moduleData) {
	/*clear all the CAMs on board*/
	uint32_t neuron_id, cam_id;

	for (neuron_id = 0; neuron_id < TOTAL_NEURON_NUM_ON_CHIP; neuron_id++) {
		for (cam_id = 0; cam_id < TOTAL_CAM_NUM; cam_id++) {
//			writeCamG(moduleData, 0, (4 << 10) | neuron_id, 0, cam_id, 0, 0, 0);
			writeCamG(moduleData, 0, (3 << 10) | neuron_id, 0, cam_id, 0, 0, 0);
			writeCamG(moduleData, 0, (2 << 10) | neuron_id, 0, cam_id, 0, 0, 0);
			writeCamG(moduleData, (3 << 8) | 255, (1 << 10) | neuron_id, 0, cam_id, 0, 0, 0);
			//writeCamG(moduleData, (3 << 8) | 253, (1 << 10) | neuron_id, 0, cam_id, 0, 0, 0);
		}
	}

	configureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U0);
	configureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U1);
	configureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U2);
	configureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U3);

}

bool configureChipG(caerModuleData moduleData, int8_t chip_id) {

	GFilterState state = moduleData->moduleState;

	// --- start  usb handle / from spike event source id
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(state->event_source_id));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->event_source_id));
	caerInputDynapseState stateSource = state->eventSourceModuleState;

	if (chip_id == DYNAPSE_CONFIG_DYNAPSE_U0 && num_config_chip_U0 > 0) {
		disableStimuliGenPrimitiveCamG(moduleData);
		caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID,
		DYNAPSE_CONFIG_DYNAPSE_U0);
		if (!caerDynapseSendDataToUSB(stateSource->deviceState, bits_chip_U0, (int) num_config_chip_U0)) {
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "USB transfer failed");
		}
		enableStimuliGenPrimitiveCamG(moduleData);
		num_config_chip_U0 = 0;
	}
	else if (chip_id == DYNAPSE_CONFIG_DYNAPSE_U1 && num_config_chip_U1 > 0) {
		disableStimuliGenPrimitiveCamG(moduleData);
		caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID,
		DYNAPSE_CONFIG_DYNAPSE_U1);
		if (!caerDynapseSendDataToUSB(stateSource->deviceState, bits_chip_U1, (int) num_config_chip_U1)) {
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "USB transfer failed");
		}
		enableStimuliGenPrimitiveCamG(moduleData);
		num_config_chip_U1 = 0;
	}
	else if (chip_id == DYNAPSE_CONFIG_DYNAPSE_U2 && num_config_chip_U2 > 0) {
		disableStimuliGenPrimitiveCamG(moduleData);
		caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID,
		DYNAPSE_CONFIG_DYNAPSE_U2);
		if (!caerDynapseSendDataToUSB(stateSource->deviceState, bits_chip_U2, (int) num_config_chip_U2)) {
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "USB transfer failed");
		}
		enableStimuliGenPrimitiveCamG(moduleData);
		num_config_chip_U2 = 0;
	}
	else if (chip_id == DYNAPSE_CONFIG_DYNAPSE_U3 && num_config_chip_U3 > 0) {
		disableStimuliGenPrimitiveCamG(moduleData);
		caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID,
		DYNAPSE_CONFIG_DYNAPSE_U3);
		if (!caerDynapseSendDataToUSB(stateSource->deviceState, bits_chip_U3, (int) num_config_chip_U3)) {
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "USB transfer failed");
		}
		enableStimuliGenPrimitiveCamG(moduleData);
		num_config_chip_U3 = 0;
	}
	return (true);
}

//build synapses when reseting
void buildSynapseG(caerModuleData moduleData, uint32_t pre_neuron_addr, uint32_t post_neuron_addr, uint32_t virtual_neuron_addr,
		int32_t synapse_type, int8_t realOrVirtualSynapse, int8_t virtual_neuron_addr_enable, uint32_t cam_id_search_start)
{
	uint32_t sram_id, cam_id, sram_id_t, cam_id_t;
	uint32_t chip_core_id, chip_id, destCoreId, source_core_id;
	int sram_found, sram_slot_found, camFound, availableCamFound;
	int sramAvailable;

	//for SRAM
	if (realOrVirtualSynapse != EXTERNAL_REAL_SYNAPSE) {
		sram_found = 0;
		sram_slot_found = 0;
		for (sram_id_t = 0; sram_id_t < TOTAL_SRAM_NUM; sram_id_t++) { //search for available SRAM //start the searching from second SRAM, for visualization
			chip_id = (post_neuron_addr >> 10);
			destCoreId = (post_neuron_addr & 0x300) >> 8;
			if (virtual_neuron_addr_enable == 0)
				source_core_id = (pre_neuron_addr & 0x300) >> 8;
			else
				source_core_id = (virtual_neuron_addr & 0x300) >> 8;
			if (memory.sram_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][sram_id_t] == 1) {
				if (memory.sram_map_content->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][sram_id_t] >> 6 == (int) chip_id &&
						(memory.sram_map_content->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][sram_id_t] & 0x30) >> 4 == (int) source_core_id) {
					if (((memory.sram_map_content->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][sram_id_t] & 0xf) & (1 << destCoreId)) != 0) {
						sram_found = 1;
					} else {
						sram_slot_found = 1;
					}
					break;
				}
			}
		}
		if (sram_slot_found == 1) {
			chip_core_id = (uint32_t) (memory.sram_map_content->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][sram_id_t] | (1 << ((post_neuron_addr & 0x300) >> 8)));
			writeSramG(moduleData, pre_neuron_addr, chip_core_id, virtual_neuron_addr, sram_id_t, virtual_neuron_addr_enable, 0);
			memory.sram_map_content->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][sram_id_t] = (int) chip_core_id; //SRAM is taken
		}
		if (sram_found == 0 && sram_slot_found == 0) {
			sramAvailable = 0;
			for (sram_id_t = 0; sram_id_t < TOTAL_SRAM_NUM; sram_id_t++) { //search for available SRAM
				if (sramAvailable == 0
					&& memory.sram_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][(sram_id_t + 1) % TOTAL_SRAM_NUM] == 0) {
					sramAvailable = 1;
					sram_id = (sram_id_t + 1) % TOTAL_SRAM_NUM; //(sram_id + 1) % TOTAL_SRAM_NUM; keep the SRAM for viewer
				}
			}
			if (sramAvailable == 1 && sram_id != 0) { //sram_id != 0 && sram_id != 1 && sram_id != 2 && sram_id != 3
				if (virtual_neuron_addr_enable == 0)
					source_core_id = (uint32_t) ((pre_neuron_addr & 0x300) >> 8);
				else
					source_core_id = (uint32_t) ((virtual_neuron_addr & 0x300) >> 8);
				chip_core_id = (((post_neuron_addr >> 10) << 6) | (source_core_id << 4) | (uint32_t) (1 << ((post_neuron_addr & 0x300) >> 8)));
				writeSramG(moduleData, pre_neuron_addr, chip_core_id, virtual_neuron_addr, sram_id, virtual_neuron_addr_enable, 0);
				memory.sram_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][sram_id] = 1; //SRAM is taken
				memory.sram_map_content->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][sram_id] = (int) chip_core_id;
			}
		}
	}
	//for CAM
	camFound = 0;
	if (realOrVirtualSynapse != VIRTUAL_SYNAPSE) { // || realOrVirtualSynapse == EXTERNAL_REAL_SYNAPSE || realOrVirtualSynapse == REAL_SYNAPSE_WITHOUT_LEARNING
		for (cam_id_t = 0; cam_id_t < TOTAL_CAM_NUM; cam_id_t++) { //search for existing CAM
			if (memory.cam_map->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][cam_id_t] == 1) {
				if (memory.cam_map_content_source->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][cam_id_t]
					== (int32_t) pre_neuron_addr && virtual_neuron_addr_enable == 0) {
					if (memory.cam_map_content_type->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][cam_id_t]
                        == synapse_type) { //(int32_t)
						camFound = 1;
						break;
					}
				} else if (memory.cam_map_content_source->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][cam_id_t]
				    == (int32_t) virtual_neuron_addr && virtual_neuron_addr_enable == 1) {
					if (memory.cam_map_content_type->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][cam_id_t]
                        == synapse_type) { //(int32_t)
						camFound = 1;
						break;
					}
				}
			}
		}
		availableCamFound = 0;
		if (camFound == 0) {
			for (cam_id_t = cam_id_search_start; cam_id_t < TOTAL_CAM_NUM; cam_id_t++) { //search for available CAM
				if (memory.cam_map->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][cam_id_t] == 0) {
					cam_id = cam_id_t;
					availableCamFound = 1;
					break;
				}
			}
		}
		if (camFound == 0 && availableCamFound == 1) {
			writeCamG(moduleData, pre_neuron_addr, post_neuron_addr, virtual_neuron_addr, cam_id, synapse_type,
					virtual_neuron_addr_enable, 0);
			memory.cam_map->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][cam_id] = 1; //CAM taken (int32_t) pre_neuron_addr;
			memory.cam_map_content_type->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][cam_id] = synapse_type;
			memory.cam_map_content_source->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][cam_id] = (int32_t) pre_neuron_addr;
			if (synapse_type > 0) {
				memory.cam_map_size->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] += 1;
			}
		}
	}
	//memories for the chip
	if (realOrVirtualSynapse == REAL_SYNAPSE || realOrVirtualSynapse == VIRTUAL_SYNAPSE) {
		int32_t memoryId = memory.filter_map_size->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0];
		memory.filter_map_addr->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][memoryId] = (int32_t) pre_neuron_addr;
		memory.filter_map_cam_id->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][memoryId] = (int32_t) cam_id;
		memory.filter_map_size->buffer2d[post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][0] += 1;
		memory.connection_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] = 1;
	}
	if (realOrVirtualSynapse == REAL_SYNAPSE) {
		memory.weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] = synapse_type * 10; //initial weight
		memory.rounded_weight_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] =
				synapse_type;
		memory.synapse_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] = synapse_type;
		memory.synapse_cam_map->buffer2d[pre_neuron_addr - MEMORY_NEURON_ADDR_OFFSET][post_neuron_addr - MEMORY_NEURON_ADDR_OFFSET] =
				(int32_t) cam_id;
	}
}

//get chip id based on post neuron
uint32_t getWriteCamChipIdG(uint32_t post_neuron_addr)
{
	uint32_t chip_id_t, chip_id;
	chip_id_t = (post_neuron_addr & NEURON_CHIPID_BITS) >> NEURON_CHIPID_SHIFT; //post_neuron_addr >> NEURON_CHIPID_SHIFT;

	if (chip_id_t == 1)
		chip_id = DYNAPSE_CONFIG_DYNAPSE_U0;
	else if (chip_id_t == 2)
		chip_id = DYNAPSE_CONFIG_DYNAPSE_U1;
	else if (chip_id_t == 3)
		chip_id = DYNAPSE_CONFIG_DYNAPSE_U2;
	else if (chip_id_t == 4)
		chip_id = DYNAPSE_CONFIG_DYNAPSE_U3;

	return (chip_id);
}

//write neuron CAM when a synapse is built or modified
uint32_t getWriteCamBitsG(uint32_t pre_neuron_addr, uint32_t post_neuron_addr,
	uint32_t virtual_neuron_addr, uint32_t cam_id, int32_t synapse_type, int8_t virtual_neuron_addr_enable)
{

	uint32_t bits;
	uint32_t ei = 0;
	uint32_t fs = 0;
	uint32_t address = pre_neuron_addr & NEURON_ADDRESS_BITS;
	uint32_t source_core = 0;

	if (virtual_neuron_addr_enable == 0)
		source_core = (pre_neuron_addr & NEURON_COREID_BITS) >> NEURON_COREID_SHIFT;
	else
		source_core = (virtual_neuron_addr & NEURON_COREID_BITS) >> NEURON_COREID_SHIFT; //to change
	if (synapse_type > 0) //if it is EX synapse
		ei = EXCITATORY_SYNAPSE_ID; //EXCITATORY_SYNAPSE_ID;
	else
		ei = INHIBITORY_SYNAPSE_ID;
	if (abs(synapse_type) == FAST_SYNAPSE_VALUE)
		fs = FAST_SYNAPSE_ID;
	else if (abs(synapse_type) == SLOW_SYNAPSE_VALUE)
		fs = SLOW_SYNAPSE_ID;
	else if (abs(synapse_type) == NO_SYNAPSE_ID) {
		address = NO_SYNAPSE_ADDRESS;
		source_core = NO_SYNAPSE_CORE;
	}
	uint32_t core_id = (post_neuron_addr & NEURON_COREID_BITS) >> NEURON_COREID_SHIFT;
	uint32_t neuron_row = (post_neuron_addr & NEURON_ROW_BITS) >> NEURON_ROW_SHIFT;
	uint32_t synapse_row = cam_id;
	uint32_t row = neuron_row << CAM_NEURON_ROW_SHIFT | synapse_row;
	uint32_t column = post_neuron_addr & NEURON_COL_BITS;
	bits = ei << CXQ_CAM_EI_SHIFT | fs << CXQ_CAM_FS_SHIFT | address << CXQ_ADDR_SHIFT
		| source_core << CXQ_SOURCE_CORE_SHIFT |
		CXQ_PROGRAM | core_id << CXQ_PROGRAM_COREID_SHIFT | row << CXQ_PROGRAM_ROW_SHIFT
		| column << CXQ_PROGRAM_COLUMN_SHIFT;

	return (bits);
}

//write neuron CAM when a synapse is built or modified
void writeCamG(caerModuleData moduleData, uint32_t pre_neuron_addr, uint32_t post_neuron_addr, uint32_t virtual_neuron_addr,
	uint32_t cam_id, int32_t synapse_type, int8_t virtual_neuron_addr_enable, int8_t stdp) {

	GFilterState state = moduleData->moduleState;

	// --- start  usb handle / from spike event source id
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(state->event_source_id));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->event_source_id));

	caerInputDynapseState stateSource = state->eventSourceModuleState;
	// --- end usb handle

	uint32_t chip_id, bits;
	chip_id = getWriteCamChipIdG(post_neuron_addr);
	bits = getWriteCamBitsG(pre_neuron_addr, post_neuron_addr, virtual_neuron_addr, cam_id, synapse_type, virtual_neuron_addr_enable);

	if (stdp == 0) {
		if (chip_id == DYNAPSE_CONFIG_DYNAPSE_U0) {
			if (num_config_chip_U0 == usb_packet_maximum_size) { //DYNAPSE_MAX_USER_USB_PACKET_SIZE
				configureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U0);
			}
			bits_chip_U0[num_config_chip_U0] = (uint32_t) bits;
			num_config_chip_U0++;
		}
		else if (chip_id == DYNAPSE_CONFIG_DYNAPSE_U1) {
			if (num_config_chip_U1 == usb_packet_maximum_size) {
				configureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U1);
			}
			bits_chip_U1[num_config_chip_U1] = (uint32_t) bits;
			num_config_chip_U1++;
		}
		else if (chip_id == DYNAPSE_CONFIG_DYNAPSE_U2) {
			if (num_config_chip_U2 == usb_packet_maximum_size) {
				configureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U2);
			}
			bits_chip_U2[num_config_chip_U2] = (uint32_t) bits;
			num_config_chip_U2++;
		}
		else if (chip_id == DYNAPSE_CONFIG_DYNAPSE_U3) {
			if (num_config_chip_U3 == usb_packet_maximum_size) {
				configureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U3);
			}
			bits_chip_U3[num_config_chip_U3] = (uint32_t) bits;
			num_config_chip_U3++;
		}
	}
	else {
		caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chip_id);
		caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, bits); //this is the 30 bits
	}

}

//write neuron SRAM when a synapse is built
void writeSramG(caerModuleData moduleData, uint32_t pre_neuron_addr, uint32_t chip_core_id, uint32_t virtual_neuron_addr,
	uint32_t sram_id, int8_t virtual_neuron_addr_enable, int8_t stdp) {
//	caerDeviceHandle usb_handle = ((caerInputDynapseState) moduleData->moduleState)->deviceState; doesn't work
	GFilterState state = moduleData->moduleState;

	// --- start  usb handle / from spike event source id
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(state->event_source_id));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->event_source_id));

	caerInputDynapseState stateSource = state->eventSourceModuleState;
	// --- end usb handle

	uint32_t chip_id, bits;
	chip_id = pre_neuron_addr >> NEURON_CHIPID_SHIFT;
	if (chip_id == 1)
		chip_id = DYNAPSE_CONFIG_DYNAPSE_U0;
	else if (chip_id == 2)
		chip_id = DYNAPSE_CONFIG_DYNAPSE_U1;
	else if (chip_id == 3)
		chip_id = DYNAPSE_CONFIG_DYNAPSE_U2;
	else if (chip_id == 4)
		chip_id = DYNAPSE_CONFIG_DYNAPSE_U3;
	uint32_t virtual_core_id = 0;
	if (virtual_neuron_addr_enable == 0)
		virtual_core_id = (pre_neuron_addr & NEURON_COREID_BITS) >> NEURON_COREID_SHIFT;
	else
		virtual_core_id = (virtual_neuron_addr & NEURON_COREID_BITS) >> NEURON_COREID_SHIFT;
	uint32_t source_chipId = (pre_neuron_addr >> NEURON_CHIPID_SHIFT) - 1; //for calculation
	uint32_t destination_chipId = (chip_core_id >> CHIPCOREID_CHIPID_SHIFT) - 1; //for calculation
	uint32_t sy, dy, sx, dx;
	if ((source_chipId / BOARD_CHIPS_Y_NUM) >= (destination_chipId / BOARD_CHIPS_Y_NUM))
		sy = EVENT_DIRECTION_Y_UP; //EVENT_DIRECTION_Y_UP;
	else
		sy = EVENT_DIRECTION_Y_DOWN;
	if ((source_chipId % BOARD_CHIPS_X_NUM) <= (destination_chipId % BOARD_CHIPS_X_NUM))
		sx = EVENT_DIRECTION_X_RIGHT; //EVENT_DIRECTION_X_RIGHT;
	else
		sx = EVENT_DIRECTION_X_LEFT; //EVENT_DIRECTION_X_LEFT;
	if (source_chipId == destination_chipId) {
		dx = 0;
		dy = 0;
	} else {
		dx = (uint32_t) abs((int32_t)(source_chipId % BOARD_CHIPS_X_NUM) - (int32_t)(destination_chipId % BOARD_CHIPS_X_NUM));
		dy = (uint32_t) abs((int32_t)(source_chipId / BOARD_CHIPS_X_NUM) - (int32_t)(destination_chipId / BOARD_CHIPS_X_NUM));
	}
	uint32_t destination_coreId = chip_core_id & DESTINATION_COREID_BITS;
	uint32_t core_id = (pre_neuron_addr & NEURON_COREID_BITS) >> NEURON_COREID_SHIFT; //(chip_core_id & CHIPCOREID_SOURCECOREID_BITS) >> CHIPCOREID_SOURCECOREID_SHIFT;
	uint32_t neuron_row = (pre_neuron_addr & NEURON_ROW_BITS) >> NEURON_ROW_SHIFT;
	uint32_t neuron_column = pre_neuron_addr & NEURON_COL_BITS;
	uint32_t synapse_row = sram_id;
	uint32_t row = neuron_row << SRAM_NEURON_ROW_SHIFT | neuron_column << SRAM_NEURON_COL_SHIFT | synapse_row;
	uint32_t column = SRAM_COL_VALUE;
	bits = virtual_core_id << CXQ_SRAM_VIRTUAL_SOURCE_CORE_SHIFT | sy << CXQ_SRAM_SY_SHIFT | dy << CXQ_SRAM_DY_SHIFT
		| sx << CXQ_SRAM_SX_SHIFT | dx << CXQ_SRAM_DX_SHIFT | destination_coreId << CXQ_SRAM_DEST_CORE_SHIFT |
		CXQ_PROGRAM | core_id << CXQ_PROGRAM_COREID_SHIFT | row << CXQ_PROGRAM_ROW_SHIFT
		| column << CXQ_PROGRAM_COLUMN_SHIFT;
	if (stdp == 0) {
		if (chip_id == DYNAPSE_CONFIG_DYNAPSE_U0) {
			if (num_config_chip_U0 == usb_packet_maximum_size) {
				configureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U0);
			}
			bits_chip_U0[num_config_chip_U0] = (uint32_t) bits;
			num_config_chip_U0++;
		}
		else if (chip_id == DYNAPSE_CONFIG_DYNAPSE_U1) {
			if (num_config_chip_U1 == usb_packet_maximum_size) {
				configureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U1);
			}
			bits_chip_U1[num_config_chip_U1] = (uint32_t) bits;
			num_config_chip_U1++;
		}
		else if (chip_id == DYNAPSE_CONFIG_DYNAPSE_U2) {
			if (num_config_chip_U2 == usb_packet_maximum_size) {
				configureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U2);
			}
			bits_chip_U2[num_config_chip_U2] = (uint32_t) bits;
			num_config_chip_U2++;
		}
		else if (chip_id == DYNAPSE_CONFIG_DYNAPSE_U3) {
			if (num_config_chip_U3 == usb_packet_maximum_size) {
				configureChipG(moduleData, DYNAPSE_CONFIG_DYNAPSE_U3);
			}
			bits_chip_U3[num_config_chip_U3] = (uint32_t) bits;
			num_config_chip_U3++;
		}
	}
	else {
		caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chip_id);
		caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, bits); //this is the 30 bits
	}

}

void enableFileStimuliRunG(caerModuleData moduleData) {
	GFilterState state = moduleData->moduleState;

	// --- start USB handle / from spike event source id
	sshsNode inputNode = caerMainloopGetSourceNode(U16T(state->file_input_source_id));

	if (sshsNodeAttributeExists(inputNode, "filePath", SSHS_STRING)) {
		sshsNodePutString(inputNode, "filePath", HEART);
		sshsNodePutBool(inputNode, "running", true);
	}
	else {
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Error input is not from file\n");
	}
}

void enableFileStimuliG(caerModuleData moduleData, int32_t pattern) {
	GFilterState state = moduleData->moduleState;

	// --- start USB handle / from spike event source id
	sshsNode inputNode = caerMainloopGetSourceNode(U16T(state->file_input_source_id));

	if (sshsNodeAttributeExists(inputNode, "filePath", SSHS_STRING)) {
		if (pattern == 0) {
			sshsNodePutString(inputNode, "filePath", HEART);
//			sshsNodePutBool(inputNode, "running", true);
		}
		else if (pattern == 1) {
			sshsNodePutString(inputNode, "filePath", SPADE);
//			sshsNodePutBool(inputNode, "running", true);
		}
		else if (pattern == 2) {
			sshsNodePutString(inputNode, "filePath", CLUB);
//			sshsNodePutBool(inputNode, "running", true);
		}
		else if (pattern == 3) {
			sshsNodePutString(inputNode, "filePath", DIAMOND);
//			sshsNodePutBool(inputNode, "running", true);
		}
	}
	else {
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Error input is not from file\n");
	}
}

void disableFileStimuliG(caerModuleData moduleData) {
	GFilterState state = moduleData->moduleState;

	// --- start USB handle / from spike event source id
	sshsNode inputNode = caerMainloopGetSourceNode(U16T(state->file_input_source_id));
	sshsNodePutBool(inputNode, "running", false);
/*	if (sshsNodeAttributeExists(inputNode, "filePath", SSHS_STRING)) {
		sshsNodePutBool(inputNode, "running", false);
	}
	else {
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Error input is not from file\n");
	}*/
}

void enableTeachingStimuliGenG(caerModuleData moduleData, int32_t pattern) {
	GFilterState state = moduleData->moduleState;

	// --- start USB handle / from spike event source id
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(state->event_source_id));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->event_source_id));

	// --- end USB handle

	sshsNode deviceConfigNodeMain = sshsGetRelativeNode(state->eventSourceConfigNode, chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
	sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");
//	sshsNodePutBool(spikeNode, "running", true);
	sshsNodePutInt(spikeNode, "stim_type", pattern + 11);
	sshsNodePutInt(spikeNode, "stim_duration", 10); //100
	sshsNodePutInt(spikeNode, "stim_avr", 20); //20 //5
	sshsNodePutInt(spikeNode, "chip_id", DYNAPSE_CONFIG_DYNAPSE_U0);
	sshsNodePutBool(spikeNode, "repeat", true);
	sshsNodePutBool(spikeNode, "doStim", true);
//	sshsNodePutBool(spikeNode, "running", true);
}

void disableTeachingStimuliGenG(caerModuleData moduleData) {
	GFilterState state = moduleData->moduleState;

	// --- start USB handle / from spike event source id
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(state->event_source_id));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->event_source_id));
	// --- end USB handle

	sshsNode deviceConfigNodeMain = sshsGetRelativeNode(state->eventSourceConfigNode, chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
	sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");
//	sshsNodePutBool(spikeNode, "running", true); //true
	sshsNodePutBool(spikeNode, "doStim", false);
}

bool enableStimuliGenG(caerModuleData moduleData, int32_t pattern) {
	GFilterState state = moduleData->moduleState;

	// --- start USB handle / from spike event source id
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(state->event_source_id));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->event_source_id));

	// --- end USB handle

	sshsNode deviceConfigNodeMain = sshsGetRelativeNode(state->eventSourceConfigNode, chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
	sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");
	sshsNodePutBool(spikeNode, "running", true);
	sshsNodePutInt(spikeNode, "stim_type", pattern);
	sshsNodePutInt(spikeNode, "stim_duration", 100);
	sshsNodePutInt(spikeNode, "stim_avr", 20); //5
	sshsNodePutInt(spikeNode, "chip_id", DYNAPSE_CONFIG_DYNAPSE_U3);
	sshsNodePutBool(spikeNode, "repeat", true);
	sshsNodePutBool(spikeNode, "doStim", true);
	return (true);
}

bool disableStimuliGenG(caerModuleData moduleData) {
	GFilterState state = moduleData->moduleState;

	// --- start USB handle / from spike event source id
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(state->event_source_id));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->event_source_id));
	// --- end USB handle

	sshsNode deviceConfigNodeMain = sshsGetRelativeNode(state->eventSourceConfigNode, chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
	sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");
	sshsNodePutBool(spikeNode, "running", true);
	sshsNodePutBool(spikeNode, "doStim", false);
	return (true);
}

void enableStimuliGenPrimitiveCamG(caerModuleData moduleData) {
	if (file_input_enable == 0) {
		GFilterState state = moduleData->moduleState;
		state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->event_source_id));
		sshsNode deviceConfigNodeMain = sshsGetRelativeNode(state->eventSourceConfigNode, chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
		sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");

		sshsNodePutBool(spikeNode, "doStimPrimitiveCam", true);
	} else {
		GFilterState state = moduleData->moduleState;

		// --- start USB handle / from spike event source id
		sshsNode inputNode = caerMainloopGetSourceNode(U16T(state->file_input_source_id));

		sshsNodePutBool(inputNode, "pause", false);

		//enable teaching signal
		state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->event_source_id));
		sshsNode deviceConfigNodeMain = sshsGetRelativeNode(state->eventSourceConfigNode, chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
		sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");

		sshsNodePutBool(spikeNode, "doStimPrimitiveCam", true);

//		sshsNodePutBool(inputNode, "running", true);

/*		if (sshsNodeAttributeExists(inputNode, "filePath", SSHS_STRING)) {
			if (stimuli_pattern == 0) {
				sshsNodePutString(inputNode, "filePath", HEART);
				sshsNodePutBool(inputNode, "running", true);
			}
			else if (stimuli_pattern == 1) {
				sshsNodePutString(inputNode, "filePath", SPADE);
				sshsNodePutBool(inputNode, "running", true);
			}
			else if (stimuli_pattern == 2) {
				sshsNodePutString(inputNode, "filePath", CLUB);
				sshsNodePutBool(inputNode, "running", true);
			}
			else if (stimuli_pattern == 3) {
				sshsNodePutString(inputNode, "filePath", DIAMOND);
				sshsNodePutBool(inputNode, "running", true);
			}
		} */

/*		if (sshsNodeAttributeExists(inputNode, "filePath", SSHS_STRING)) {
			sshsNodePutBool(inputNode, "running", true);
		}
		else {
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Error input is not from file\n");
		}

		state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->event_source_id));
		sshsNode deviceConfigNodeMain = sshsGetRelativeNode(state->eventSourceConfigNode, chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
		sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");
		sshsNodePutBool(spikeNode, "doStimPrimitiveCam", true);*/
	}
}

void disableStimuliGenPrimitiveCamG(caerModuleData moduleData) {
	if (file_input_enable == 0) {
		GFilterState state = moduleData->moduleState;
		state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->event_source_id));

		sshsNode deviceConfigNodeMain = sshsGetRelativeNode(state->eventSourceConfigNode,
			chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
		sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");

		sshsNodePutBool(spikeNode, "doStimPrimitiveCam", false);
	} else {
		GFilterState state = moduleData->moduleState;

		// --- start USB handle / from spike event source id
		sshsNode inputNode = caerMainloopGetSourceNode(U16T(state->file_input_source_id));

		sshsNodePutBool(inputNode, "pause", true);

		//disable teaching signal
		state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->event_source_id));

		sshsNode deviceConfigNodeMain = sshsGetRelativeNode(state->eventSourceConfigNode,
			chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
		sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");

		sshsNodePutBool(spikeNode, "doStimPrimitiveCam", false);


//		sshsNodePutBool(inputNode, "running", false);

/*		if (sshsNodeAttributeExists(inputNode, "filePath", SSHS_STRING)) {
			sshsNodePutBool(inputNode, "running", false);
		}
		else {
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Error input is not from file\n");
		}

		state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->event_source_id));
		sshsNode deviceConfigNodeMain = sshsGetRelativeNode(state->eventSourceConfigNode, chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
		sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");
		sshsNodePutBool(spikeNode, "doStimPrimitiveCam", false); */
	}
}

void enableTeachingSignalG(caerModuleData moduleData) {
	GFilterState state = moduleData->moduleState;
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->event_source_id));

	sshsNode deviceConfigNodeMain = sshsGetRelativeNode(state->eventSourceConfigNode, chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
	sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");

	sshsNodePutBool(spikeNode, "sendInhibitoryStimuli", false);
	sshsNodePutBool(spikeNode, "sendTeachingStimuli", true);
	sshsNodePutBool(spikeNode, "setCam", false);
	sshsNodePutBool(spikeNode, "setCamSingle", false);
}

void enableTeachingG(caerModuleData moduleData) {

	GFilterState state = moduleData->moduleState;
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->event_source_id));

	sshsNode deviceConfigNodeMain = sshsGetRelativeNode(state->eventSourceConfigNode,
		chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
	sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");

	sshsNodePutBool(spikeNode, "teaching", true);
}

void disableTeachingG(caerModuleData moduleData) {

	GFilterState state = moduleData->moduleState;
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->event_source_id));

	sshsNode deviceConfigNodeMain = sshsGetRelativeNode(state->eventSourceConfigNode, chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
	sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");

	sshsNodePutBool(spikeNode, "teaching", false);
}

COLOUR getColourWG(double v, double vmin, double vmax) {
	/* get weight plot colors */

	COLOUR c = { 0, 0, 0 };
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

COLOUR getColourSG(float v)
{
	/* get synapse plot colors */
	COLOUR c = { 0, 0, 0 };

	if (v == WHITE_EDGE_COLOR_VALUE) {
		//white
		c.r = 65535;
		c.g = 65535;
		c.b = 65535;
		return (c);
	}

	if (v == 0) {
		//black
		c.r = 0;
		c.g = 0;
		c.b = 0;
	} else if (0 < v && v <= 128) {
		//gray-scale plot
		c.r = 0; //(uint16_t) (v * 30);
		c.g = (uint16_t) (v * 30);
		c.b = (uint16_t) (v * 30);
	} else {
		//gray-scale plot
		c.r = (uint16_t) (v * 30);
		c.g = (uint16_t) (v * 30);
		c.b = 0;
	}
	c.r = (uint16_t) (c.r * 257 * 100);
	c.g = (uint16_t) (c.g * 257 * 100);
	c.b = (uint16_t) (c.b * 257 * 100);

	return (c);
}

void resetBiasesG(caerModuleData moduleData)
{
	/* set default biases */
	GFilterState state = moduleData->moduleState;

	//get USB handle from spike event source id
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(state->event_source_id));
	caerInputDynapseState stateSource = state->eventSourceModuleState;

	uint32_t chip_id_t, chip_id, core_id;

	for (chip_id_t = 0; chip_id_t < 4; chip_id_t++) { //1 4

		if (chip_id_t == 0)
			chip_id = DYNAPSE_CONFIG_DYNAPSE_U0;
		else if (chip_id_t == 1)
			chip_id = DYNAPSE_CONFIG_DYNAPSE_U1;
		else if (chip_id_t == 2)
			chip_id = DYNAPSE_CONFIG_DYNAPSE_U2;
		else if (chip_id_t == 3)
			chip_id = DYNAPSE_CONFIG_DYNAPSE_U3;

		caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chip_id);

		for (core_id = 0; core_id < 4; core_id++) {
			//sweep all the biases
			if (chip_id == DYNAPSE_CONFIG_DYNAPSE_U3) {
				//for real DVS recordings
/*				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTHR_N", 7, 1, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHW_P", 7, 1, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_BUF_P", 3, 80, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_CASC_N", 7, 1, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 5, 2, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_NMDA_N", 7, 1, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_RFR_N", 2, 180, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 3, 40, "LowBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU2_N", 6, 15, "LowBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 3, 180, "HighBias", "NBias"); //3, 180 //2, 180
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_F_P", 6, 150, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_S_P", 7, 40, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_F_P", 4, 220, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_S_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_F_P", 6, 150, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_S_P", 7, 40, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_F_P", 0, 220, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_S_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_F_N", 0, 70, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_S_N", 1, 250, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_F_N", 0, 250, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_S_N", 1, 250, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "R2R_P", 4, 85, "HighBias", "PBias"); */
/*				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTHR_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHW_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_BUF_P", 3, 80, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_CASC_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_NMDA_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_RFR_N", 5, 103, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 4, 200, "LowBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU2_N", 6, 15, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 0, 30, "HighBias", "NBias"); //0, 30 //0, 200 //3, 150 //4, 40
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_F_P", 7, 30, "HighBias", "PBias"); //70!!!!! //6, 200 //105
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_S_P", 6, 200, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_F_P", 0, 220, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_S_P", 0, 220, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_F_P", 7, 30, "HighBias", "PBias"); //7, 40, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_S_P", 7, 40, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_F_P", 0, 200, "HighBias", "PBias"); //7, 40, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_S_P", 7, 40, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_F_N", 1, 100, "HighBias", "NBias"); //29!!!
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_S_N", 1, 50, "HighBias", "NBias"); //19!!! //0, 38
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_F_N", 0, 250, "HighBias", "NBias"); //7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "R2R_P", 4, 85, "HighBias", "PBias"); */
				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTHR_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHW_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_BUF_P", 3, 80, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_CASC_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_NMDA_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_RFR_N", 5, 255, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 4, 200, "LowBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU2_N", 6, 15, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 4, 100, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_F_P", 6, 105, "HighBias", "PBias"); //105
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_S_P", 7, 40, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_F_P", 0, 220, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_S_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_F_P", 7, 40, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_S_P", 7, 40, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_F_P", 7, 40, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_S_P", 7, 40, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_F_N", 0, 76, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_S_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_F_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "R2R_P", 4, 85, "HighBias", "PBias");
			}
			else if (chip_id == DYNAPSE_CONFIG_DYNAPSE_U2) {
				if (core_id == 0 || core_id == 1) {
					setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTHR_N", 7, 0, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_AHW_P", 7, 0, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_BUF_P", 3, 80, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_CASC_N", 7, 0, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 7, 0, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_NMDA_N", 7, 0, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_RFR_N", 5, 103, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 3, 100, "LowBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU2_N", 6, 15, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 0, 190-5, "HighBias", "NBias"); //0, 210 //0, 185 //0, 175 //0, 190 //0, 160
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_F_P", 6, 200, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_S_P", 6, 200, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_F_P", 3, 170, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_S_P", 3, 170, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_F_P", 5, 30, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_S_P", 5, 30, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_F_P", 2, 220, "HighBias", "PBias"); //1, 220 //2, 220
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_S_P", 2, 220, "HighBias", "PBias"); //1, 220 //2, 220
					setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_F_N", 1, 200, "HighBias", "NBias"); //190 //1, 230 //1, 200
					setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_S_N", 1, 200, "HighBias", "NBias"); //190 //1, 130 //1, 100
					setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_F_N", 0, 200, "HighBias", "NBias"); //2, 200 //0, 200 //2, 200
					setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_S_N", 0, 200, "HighBias", "NBias"); //2, 100 //0, 100 //2, 100
					setBiasBitsG(moduleData, chip_id, core_id, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "R2R_P", 4, 85, "HighBias", "PBias");
				} else {
					setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTHR_N", 7, 0, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_AHW_P", 7, 0, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_BUF_P", 3, 80, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_CASC_N", 7, 0, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 7, 0, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_NMDA_N", 7, 0, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_RFR_N", 5, 103, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 3, 100, "LowBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU2_N", 6, 15, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 0, 180-0, "HighBias", "NBias"); //0, 185 //0, 175 //0, 190 //0, 160
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_F_P", 6, 200, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_S_P", 6, 200, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_F_P", 3, 170, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_S_P", 3, 170, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_F_P", 5, 30, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_S_P", 5, 200, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_F_P", 2, 220, "HighBias", "PBias"); //1, 220 //2, 220
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_S_P", 2, 220, "HighBias", "PBias"); //1, 220 //2, 220
					setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_F_N", 1, 200, "HighBias", "NBias"); //190 //1, 230 //1, 200
					setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_S_N", 1, 200, "HighBias", "NBias"); //190 //1, 130 //1, 100
					setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_F_N", 0, 200, "HighBias", "NBias"); //2, 200 //0, 200 //2, 200
					setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_S_N", 0, 200, "HighBias", "NBias"); //2, 100 //0, 100 //2, 100
					setBiasBitsG(moduleData, chip_id, core_id, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "R2R_P", 4, 85, "HighBias", "PBias");
				}
			}
			else if (chip_id == DYNAPSE_CONFIG_DYNAPSE_U1) {
				if (core_id == 0 || core_id == 1) {
					setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTHR_N", 7, 0, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_AHW_P", 7, 0, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_BUF_P", 3, 80, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_CASC_N", 7, 0, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 7, 0, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_NMDA_N", 7, 0, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_RFR_N", 5, 103, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 3, 35, "LowBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU2_N", 6, 15, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 0, 160+10, "HighBias", "NBias"); //0, 160
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_F_P", 6, 200, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_S_P", 6, 200, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_F_P", 3, 170, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_S_P", 3, 170, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_F_P", 5, 30, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_S_P", 5, 30, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_F_P", 2, 220, "HighBias", "PBias"); //1, 220 //2, 220
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_S_P", 2, 220, "HighBias", "PBias"); //1, 220 //2, 220
					setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_F_N", 2, 200, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_S_N", 2, 200, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_F_N", 0, 200, "HighBias", "NBias"); //0, 200 //2, 200
					setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_S_N", 0, 200, "HighBias", "NBias"); //0, 100 //2, 100
					setBiasBitsG(moduleData, chip_id, core_id, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "R2R_P", 4, 85, "HighBias", "PBias");
				} else {
					setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTHR_N", 7, 0, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_AHW_P", 7, 0, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_BUF_P", 3, 80, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_CASC_N", 7, 0, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 7, 0, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_NMDA_N", 7, 0, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_RFR_N", 5, 103, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 3, 35, "LowBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU2_N", 6, 15, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 0, 190-10, "HighBias", "NBias"); //0, 190 //0, 160
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_F_P", 6, 200, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_S_P", 6, 200, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_F_P", 3, 170, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_S_P", 3, 170, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_F_P", 5, 30, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_S_P", 5, 30, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_F_P", 2, 220, "HighBias", "PBias"); //1, 220 //2, 220
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_S_P", 2, 220, "HighBias", "PBias"); //1, 220 //2, 220
					setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_F_N", 2, 200, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_S_N", 2, 200, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_F_N", 0, 200, "HighBias", "NBias"); //0, 200 //2, 200
					setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_S_N", 0, 200, "HighBias", "NBias"); //0, 100 //2, 100
					setBiasBitsG(moduleData, chip_id, core_id, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "R2R_P", 4, 85, "HighBias", "PBias");
				}
			}
			else if (chip_id == DYNAPSE_CONFIG_DYNAPSE_U0) {
/*				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTHR_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHW_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_BUF_P", 3, 80, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_CASC_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_NMDA_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_RFR_N", 5, 103, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 4, 200, "LowBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU2_N", 6, 15, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 1, 45, "HighBias", "NBias"); //3, 150 //4, 40
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_F_P", 6, 100, "HighBias", "PBias"); //6, 200 //105
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_S_P", 6, 200, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_F_P", 0, 220, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_S_P", 0, 220, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_F_P", 6, 50, "HighBias", "PBias"); //6, 105 //7, 40, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_S_P", 7, 40, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_F_P", 0, 250, "HighBias", "PBias"); //7, 40, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_S_P", 7, 40, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_F_N", 3, 40, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_S_N", 0, 19, "HighBias", "NBias"); //0, 38
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_F_N", 3, 140, "HighBias", "NBias"); //7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "R2R_P", 4, 85, "HighBias", "PBias"); */
/*				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTHR_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHW_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_BUF_P", 3, 80, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_CASC_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_NMDA_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_RFR_N", 5, 103, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 4, 160, "LowBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU2_N", 6, 15, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 2, 100, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_F_P", 7, 30, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_S_P", 6, 200, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_F_P", 0, 250, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_S_P", 0, 220, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_F_P", 7, 30, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_S_P", 5, 40, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_F_P", 0, 200, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_S_P", 0, 87, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_F_N", 0, 250, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_S_N", 3, 70, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_F_N", 0, 250, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_S_N", 0, 164, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "R2R_P", 4, 85, "HighBias", "PBias"); */
/*				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTHR_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHW_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_BUF_P", 3, 80, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_CASC_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 7, 0, "HighBias", "PBias"); //0, 20 //0, 255 //7, 0
				setBiasBitsG(moduleData, chip_id, core_id, "IF_NMDA_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_RFR_N", 5, 103, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 4, 200, "LowBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU2_N", 6, 15, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 1, 200, "HighBias", "NBias"); //0, 30 //0, 200 //3, 150 //4, 40
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_F_P", 7, 30, "HighBias", "PBias"); //70!!!!! //6, 200 //105
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_S_P", 6, 200, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_F_P", 0, 220, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_S_P", 0, 220, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_F_P", 7, 30, "HighBias", "PBias"); //7, 40, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_S_P", 6, 200, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_F_P", 0, 220, "HighBias", "PBias"); //7, 40, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_S_P", 0, 220, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_F_N", 0, 200, "HighBias", "NBias"); //29!!!
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_S_N", 0, 100, "HighBias", "NBias"); //19!!! //0, 38
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_F_N", 0, 100, "HighBias", "NBias"); //7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_S_N", 1, 50, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "R2R_P", 4, 85, "HighBias", "PBias"); */
				if (core_id != 3) {
					setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTHR_N", 7, 0, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_AHW_P", 7, 0, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_BUF_P", 3, 80, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_CASC_N", 7, 0, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 7, 0, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_NMDA_N", 7, 0, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_RFR_N", 5, 103, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 3, 100, "LowBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU2_N", 6, 15, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 0, 160+15, "HighBias", "NBias"); //160+10
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_F_P", 6, 200, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_S_P", 6, 200, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_F_P", 3, 170, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_S_P", 3, 170, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_F_P", 5, 30, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_S_P", 5, 30, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_F_P", 2, 220, "HighBias", "PBias"); //1, 220 //2, 220
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_S_P", 2, 220, "HighBias", "PBias"); //1, 220 //2, 220
					setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_F_N", 1, 200, "HighBias", "NBias"); //190 //1, 230 //1, 200
					setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_S_N", 1, 200, "HighBias", "NBias"); //190 //1, 130 //1, 100
					setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_F_N", 0, 200, "HighBias", "NBias"); //2, 200 //0, 200 //2, 200
					setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_S_N", 0, 200, "HighBias", "NBias"); //2, 100 //0, 100 //2, 100
					setBiasBitsG(moduleData, chip_id, core_id, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "R2R_P", 4, 85, "HighBias", "PBias");
				} else {
					setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTHR_N", 7, 0, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_AHW_P", 7, 0, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_BUF_P", 3, 80, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_CASC_N", 7, 0, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 7, 0, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_NMDA_N", 7, 0, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_RFR_N", 5, 20, "HighBias", "NBias"); //5, 50 //5, 103
					setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 3, 40, "LowBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU2_N", 6, 15, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 0, 160, "HighBias", "NBias"); //0, 30 //0, 200 //3, 150 //4, 40
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_F_P", 7, 200, "HighBias", "PBias"); //70!!!!! //6, 200 //105
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_S_P", 0, 255, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_F_P", 2, 170, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_S_P", 7, 0, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_F_P", 6, 30, "HighBias", "PBias"); //7, 40, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_S_P", 0, 255, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_F_P", 2, 220, "HighBias", "PBias"); //7, 40, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_S_P", 7, 0, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_F_N", 0, 250, "HighBias", "NBias"); //29!!!
					setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_S_N", 7, 0, "HighBias", "NBias"); //19!!! //0, 38
					setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_F_N", 0, 250, "HighBias", "NBias"); //2, 200 //7, 0, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "NBias");
					setBiasBitsG(moduleData, chip_id, core_id, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
					setBiasBitsG(moduleData, chip_id, core_id, "R2R_P", 4, 85, "HighBias", "PBias");
				}

			}
		}
	}
}

void resetBiasesGDC(caerModuleData moduleData)
{
	/* set default biases */
	GFilterState state = moduleData->moduleState;

	//get USB handle from spike event source id
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(state->event_source_id));
	caerInputDynapseState stateSource = state->eventSourceModuleState;

	uint32_t chip_id_t, chip_id, core_id;

	for (chip_id_t = 0; chip_id_t < 4; chip_id_t++) { //1 4

		if (chip_id_t == 0)
			chip_id = DYNAPSE_CONFIG_DYNAPSE_U0;
		else if (chip_id_t == 1)
			chip_id = DYNAPSE_CONFIG_DYNAPSE_U1;
		else if (chip_id_t == 2)
			chip_id = DYNAPSE_CONFIG_DYNAPSE_U2;
		else if (chip_id_t == 3)
			chip_id = DYNAPSE_CONFIG_DYNAPSE_U3;

		caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chip_id);

		for (core_id = 0; core_id < 4; core_id++) {
			//sweep all the biases
			if (chip_id == DYNAPSE_CONFIG_DYNAPSE_U3) {
				//for real DVS recordings
				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTHR_N", 7, 1, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHW_P", 7, 1, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_BUF_P", 3, 80, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_CASC_N", 7, 1, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 5, 2, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_NMDA_N", 7, 1, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_RFR_N", 2, 180, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 3, 40, "LowBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU2_N", 6, 15, "LowBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 3, 180, "HighBias", "NBias"); //3, 180 //2, 180
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_F_P", 6, 150, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_S_P", 7, 40, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_F_P", 4, 220, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_S_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_F_P", 6, 150, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_S_P", 7, 40, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_F_P", 0, 220, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_S_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_F_N", 0, 70, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_S_N", 1, 250, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_F_N", 0, 250, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_S_N", 1, 250, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "R2R_P", 4, 85, "HighBias", "PBias");
			}
			else if (chip_id == DYNAPSE_CONFIG_DYNAPSE_U2) {
				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTHR_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHW_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_BUF_P", 3, 80, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_CASC_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 0, 255, "HighBias", "PBias"); //0, 255 //7, 0
				setBiasBitsG(moduleData, chip_id, core_id, "IF_NMDA_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_RFR_N", 5, 103, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 7, 200, "LowBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU2_N", 6, 15, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 0, 200, "HighBias", "NBias"); //0, 200 //3, 150 //4, 40
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_F_P", 7, 30, "HighBias", "PBias"); //70!!!!! //6, 200 //105
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_S_P", 6, 200, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_F_P", 0, 220, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_S_P", 0, 220, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_F_P", 7, 30, "HighBias", "PBias"); //7, 40, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_S_P", 6, 200, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_F_P", 0, 220, "HighBias", "PBias"); //7, 40, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_S_P", 0, 220, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_F_N", 0, 200, "HighBias", "NBias"); //29!!!
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_S_N", 0, 100, "HighBias", "NBias"); //19!!! //0, 38
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_F_N", 1, 100, "HighBias", "NBias"); //7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_S_N", 1, 50, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "R2R_P", 4, 85, "HighBias", "PBias");
			}
			else if (chip_id == DYNAPSE_CONFIG_DYNAPSE_U1) {
				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTHR_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHW_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_BUF_P", 3, 80, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_CASC_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 0, 255, "HighBias", "PBias"); //0, 20 //0, 255 //7, 0
				setBiasBitsG(moduleData, chip_id, core_id, "IF_NMDA_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_RFR_N", 5, 103, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 7, 200, "LowBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU2_N", 6, 15, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 0, 200, "HighBias", "NBias"); //0, 30 //0, 200 //3, 150 //4, 40
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_F_P", 7, 30, "HighBias", "PBias"); //70!!!!! //6, 200 //105
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_S_P", 6, 200, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_F_P", 0, 220, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_S_P", 0, 220, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_F_P", 7, 30, "HighBias", "PBias"); //7, 40, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_S_P", 6, 200, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_F_P", 0, 220, "HighBias", "PBias"); //7, 40, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_S_P", 0, 220, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_F_N", 0, 200, "HighBias", "NBias"); //29!!!
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_S_N", 0, 100, "HighBias", "NBias"); //19!!! //0, 38
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_F_N", 1, 100, "HighBias", "NBias"); //7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_S_N", 1, 50, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "R2R_P", 4, 85, "HighBias", "PBias");
			}
			else if (chip_id == DYNAPSE_CONFIG_DYNAPSE_U0) {
				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTAU_N", 7, 35, "LowBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHTHR_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_AHW_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_BUF_P", 3, 80, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_CASC_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 0, 255, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_NMDA_N", 7, 0, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_RFR_N", 5, 103, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 4, 160, "LowBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU2_N", 6, 15, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 0, 250, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_F_P", 7, 30, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_TAU_S_P", 6, 200, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_F_P", 0, 250, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPIE_THR_S_P", 0, 220, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_F_P", 7, 30, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_TAU_S_P", 5, 40, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_F_P", 0, 200, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "NPDPII_THR_S_P", 0, 87, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_F_N", 0, 250, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_EXC_S_N", 3, 70, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_F_N", 0, 250, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PS_WEIGHT_INH_S_N", 0, 164, "HighBias", "NBias");
				setBiasBitsG(moduleData, chip_id, core_id, "PULSE_PWLK_P", 3, 50, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "R2R_P", 4, 85, "HighBias", "PBias");
			}
		}
	}
}

void disableOutputCores(caerModuleData moduleData, int32_t pattern) {
	GFilterState state = moduleData->moduleState;
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(state->event_source_id));
	caerInputDynapseState stateSource = state->eventSourceModuleState;

	uint32_t chip_id, core_id;

	chip_id = DYNAPSE_CONFIG_DYNAPSE_U0;
	caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chip_id);

	for (core_id = 0; core_id < 4; core_id++) {
		if ((int32_t) core_id == pattern) {
			setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 4, 200, "LowBias", "NBias");
			setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 1, 200, "HighBias", "NBias");
		} else {
			setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 0, 255, "LowBias", "NBias");
			setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 7, 0, "HighBias", "NBias");
		}
	}
}

void disableAllOutputCores(caerModuleData moduleData) {
	GFilterState state = moduleData->moduleState;
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(state->event_source_id));
	caerInputDynapseState stateSource = state->eventSourceModuleState;

	uint32_t chip_id, core_id;

	chip_id = DYNAPSE_CONFIG_DYNAPSE_U0;
	caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chip_id);

	for (core_id = 0; core_id < 4; core_id++) {
		setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 0, 255, "LowBias", "NBias");
		setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 7, 0, "HighBias", "NBias");
	}
}

void disableOutputCore(caerModuleData moduleData) {
	GFilterState state = moduleData->moduleState;
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(state->event_source_id));
	caerInputDynapseState stateSource = state->eventSourceModuleState;

	uint32_t chip_id, core_id;

	chip_id = DYNAPSE_CONFIG_DYNAPSE_U0;
	core_id = CORE_DOWN_RIGHT_ID;
//	disableFileStimuliG(moduleData);
	caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chip_id);

	setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 0, 255, "LowBias", "NBias");
	setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 7, 0, "HighBias", "NBias");
//	enableFileStimuliG(moduleData, stimuli_pattern);
}

void enableOutputCore(caerModuleData moduleData) {
	GFilterState state = moduleData->moduleState;
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(state->event_source_id));
	caerInputDynapseState stateSource = state->eventSourceModuleState;

	uint32_t chip_id, core_id;

	chip_id = DYNAPSE_CONFIG_DYNAPSE_U0;
	core_id = CORE_DOWN_RIGHT_ID;
	disableFileStimuliG(moduleData);
	caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chip_id);

	setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 3, 40, "LowBias", "NBias");
	setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 0, 160, "HighBias", "NBias"); //1, 40
	enableFileStimuliG(moduleData, stimuli_pattern);

}

void setTestingBias(caerModuleData moduleData) {
	GFilterState state = moduleData->moduleState;
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(state->event_source_id));
	caerInputDynapseState stateSource = state->eventSourceModuleState;

	uint32_t chip_id, core_id;

	chip_id = DYNAPSE_CONFIG_DYNAPSE_U0;
	caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chip_id);

	for (core_id = 0; core_id < 3; core_id++) {
		setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 7, 0, "HighBias", "PBias");
	}
}

void disableInputChip(caerModuleData moduleData) {
	GFilterState state = moduleData->moduleState;
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(state->event_source_id));
	caerInputDynapseState stateSource = state->eventSourceModuleState;

	uint32_t chip_id, core_id;

	chip_id = DYNAPSE_CONFIG_DYNAPSE_U3;

	caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chip_id);

	for (core_id = 0; core_id < 4; core_id++) {
		setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 0, 255, "LowBias", "NBias");
		setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 7, 0, "HighBias", "NBias");
	}
}

void enableInputChip(caerModuleData moduleData) {
	GFilterState state = moduleData->moduleState;
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(state->event_source_id));
	caerInputDynapseState stateSource = state->eventSourceModuleState;

	uint32_t chip_id, core_id;

	chip_id = DYNAPSE_CONFIG_DYNAPSE_U3;

	caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chip_id);

	for (core_id = 0; core_id < 4; core_id++) {
		setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 3, 40, "LowBias", "NBias");
		setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 3, 180, "HighBias", "NBias");
	}
}

void enableAllOutputCores(caerModuleData moduleData) {
	GFilterState state = moduleData->moduleState;
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(state->event_source_id));
	caerInputDynapseState stateSource = state->eventSourceModuleState;

	uint32_t chip_id, core_id;

	chip_id = DYNAPSE_CONFIG_DYNAPSE_U0;
	caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chip_id);

	for (core_id = 0; core_id < 4; core_id++) {
		if (core_id == 0) {
			setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 4, 200, "LowBias", "NBias");
			setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 1, 250, "HighBias", "NBias");
		} if (core_id == 2) {
			setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 4, 200, "LowBias", "NBias");
			setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 0, 100, "HighBias", "NBias");
		} else if (core_id == 3) {
			setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 4, 200, "LowBias", "NBias");
			setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 1, 150, "HighBias", "NBias");
		} else {
			setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 4, 200, "LowBias", "NBias");
			setBiasBitsG(moduleData, chip_id, core_id, "IF_THR_N", 1, 200, "HighBias", "NBias");
		}
	}
}

void injectCurrent(caerModuleData moduleData, int32_t inject, int32_t layer) {
	GFilterState state = moduleData->moduleState;
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(state->event_source_id));
	caerInputDynapseState stateSource = state->eventSourceModuleState;

	uint32_t chip_id, core_id;

	if (inject == 1) {
		if (layer == 0 && state->learning_feature1 == true) {
			chip_id = DYNAPSE_CONFIG_DYNAPSE_U1;
			disableStimuliGenPrimitiveCamG(moduleData);
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chip_id);
			for (core_id = 0; core_id < 4; core_id++) {
//				setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 0, 100, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 3, 10, "LowBias", "NBias");
			}
			enableStimuliGenPrimitiveCamG(moduleData);
		}
		if (layer == 1 && state->learning_feature2 == true) {
			chip_id = DYNAPSE_CONFIG_DYNAPSE_U2;
			disableStimuliGenPrimitiveCamG(moduleData);
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chip_id);
			for (core_id = 0; core_id < 4; core_id++) {
//				setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 0, 100, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 3, 50, "LowBias", "NBias");
			}
			enableStimuliGenPrimitiveCamG(moduleData);
		}
		if (layer == 2 && state->learning_feature3 == true) {
			chip_id = DYNAPSE_CONFIG_DYNAPSE_U0;
			disableStimuliGenPrimitiveCamG(moduleData);
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chip_id);
			for (core_id = 0; core_id < 3; core_id++) {
//				setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 0, 100, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 3, 70, "LowBias", "NBias");
			}
			enableStimuliGenPrimitiveCamG(moduleData);
		}
	} else if (inject == 0) {
		if (layer == 0 && state->learning_feature1 == true) {
			chip_id = DYNAPSE_CONFIG_DYNAPSE_U1;
			disableStimuliGenPrimitiveCamG(moduleData);
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chip_id);
			for (core_id = 0; core_id < 4; core_id++) {
//				setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 3, 35, "LowBias", "NBias"); //3, 40
			}
			enableStimuliGenPrimitiveCamG(moduleData);
		}
		if (layer == 1 && state->learning_feature2 == true) {
			chip_id = DYNAPSE_CONFIG_DYNAPSE_U2;
			disableStimuliGenPrimitiveCamG(moduleData);
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chip_id);
			for (core_id = 0; core_id < 4; core_id++) {
//				setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 3, 100, "LowBias", "NBias"); //3, 100
			}
			enableStimuliGenPrimitiveCamG(moduleData);
		}
		if (layer == 2 && state->learning_feature3 == true) {
			chip_id = DYNAPSE_CONFIG_DYNAPSE_U0;
			disableStimuliGenPrimitiveCamG(moduleData);
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chip_id);
			for (core_id = 0; core_id < 3; core_id++) {
//				setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 3, 100, "LowBias", "NBias"); //3, 200
			}
			enableStimuliGenPrimitiveCamG(moduleData);
		}
	} else if (inject == 2) {
		if (layer == 0 && state->learning_feature1 == true) {
			chip_id = DYNAPSE_CONFIG_DYNAPSE_U1;
			disableStimuliGenPrimitiveCamG(moduleData);
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chip_id);
			for (core_id = 0; core_id < 4; core_id++) {
//				setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 0, 255, "LowBias", "NBias"); //3, 40
			}
			enableStimuliGenPrimitiveCamG(moduleData);
		}
		if (layer == 1 && state->learning_feature2 == true) {
			chip_id = DYNAPSE_CONFIG_DYNAPSE_U2;
			disableStimuliGenPrimitiveCamG(moduleData);
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chip_id);
			for (core_id = 0; core_id < 4; core_id++) {
//				setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 0, 255, "LowBias", "NBias"); //3, 100
			}
			enableStimuliGenPrimitiveCamG(moduleData);
		}
		if (layer == 2 && state->learning_feature3 == true) {
			chip_id = DYNAPSE_CONFIG_DYNAPSE_U0;
			disableStimuliGenPrimitiveCamG(moduleData);
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, chip_id);
			for (core_id = 0; core_id < 3; core_id++) {
//				setBiasBitsG(moduleData, chip_id, core_id, "IF_DC_P", 7, 0, "HighBias", "PBias");
				setBiasBitsG(moduleData, chip_id, core_id, "IF_TAU1_N", 0, 255, "LowBias", "NBias"); //3, 200
			}
			enableStimuliGenPrimitiveCamG(moduleData);
		}
	}
}

void setBiasBitsG(caerModuleData moduleData, uint32_t chip_id, uint32_t core_id, const char *biasName_t,
	uint8_t coarseValue, uint16_t fineValue, const char *lowHigh, const char *npBias)
{

	GFilterState state = moduleData->moduleState;
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(state->event_source_id));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(state->event_source_id));
	caerInputDynapseState stateSource = state->eventSourceModuleState;

	size_t biasNameLength = strlen(biasName_t);
	char biasName[biasNameLength + 3];

	biasName[0] = 'C';
	if (core_id == 0)
		biasName[1] = '0';
	else if (core_id == 1)
		biasName[1] = '1';
	else if (core_id == 2)
		biasName[1] = '2';
	else if (core_id == 3)
		biasName[1] = '3';
	biasName[2] = '_';

	uint32_t i;
	for (i = 0; i < biasNameLength + 3; i++) {
		biasName[3 + i] = biasName_t[i];
	}

	uint32_t bits = (uint32_t) generatesBitsCoarseFineBiasSetting(state->eventSourceConfigNode, biasName, coarseValue, fineValue,
		lowHigh, "Normal", npBias, true, chip_id);

	caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, bits);
}

void setTimerG() {
	struct itimerval itv;
	itv.it_interval.tv_sec = 1;
	itv.it_interval.tv_usec = 0;
	itv.it_value.tv_sec = 1;
	itv.it_value.tv_usec = 0;
	setitimer(ITIMER_REAL, &itv, &oldtv);
}

void signalHandlerG(int m) {
	time_cnt = (time_cnt + 1) % 4294967295;
	n = m;
}

void shuffle1DArrayG(int64_t *array, int64_t Range) {
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

void getRand1DArrayG(int64_t *array, int64_t Range, int64_t available_cam_num) {
	int64_t temp[Range];
	int64_t i;
	for (i = 0; i < Range; i++) {
		temp[i] = i;
	}
	shuffle1DArrayG(temp, Range);
	for (i = 0; i < available_cam_num; i++) {
		array[i] = temp[i];
	}
}

void getRand1DBinaryArrayG(int64_t *binaryArray, int64_t Range, int64_t available_cam_num) {
	int64_t array[available_cam_num];
	getRand1DArrayG(array, Range, available_cam_num);
	int64_t i;
	int64_t num;
	for (i = 0; i < Range; i++) {
		binaryArray[i] = 0;
	}
	for (i = 0; i < available_cam_num; i++) {
		num = array[i];
		binaryArray[num] = 1;
	}
}

