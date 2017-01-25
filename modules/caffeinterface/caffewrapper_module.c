/* Caffe Interface cAER module
 *  Author: federico.corradi@inilabs.com
 */

#include "base/mainloop.h"
#include "base/module.h"
#include "wrapper.h"

struct caffewrapper_state {
	uint32_t *integertest;
	char * file_to_classify;
	double detThreshold;
	bool doPrintOutputs;
	bool doShowActivations;
	bool doNormInputImages;
	struct MyCaffe* cpp_class; //pointer to cpp_class_object
};

typedef struct caffewrapper_state *caffewrapperState;

static bool caerCaffeWrapperInit(caerModuleData moduleData);
static void caerCaffeWrapperRun(caerModuleData moduleData, size_t argsNumber,
		va_list args);
static void caerCaffeWrapperExit(caerModuleData moduleData);

static struct caer_module_functions caerCaffeWrapperFunctions = { .moduleInit =
		&caerCaffeWrapperInit, .moduleRun = &caerCaffeWrapperRun,
		.moduleConfig =
		NULL, .moduleExit = &caerCaffeWrapperExit };


const char * caerCaffeWrapper(uint16_t moduleID, int * classifyhist, int size,
		double *classificationResults,
		caerFrameEventPacket *networkActivity, int sizeDisplay) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "caerCaffeWrapper", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return (NULL);
	}

	caerModuleSM(&caerCaffeWrapperFunctions, moduleData,
			sizeof(struct caffewrapper_state), 5, classifyhist, size,
			classificationResults, networkActivity, sizeDisplay);

	return (NULL);
}

static bool caerCaffeWrapperInit(caerModuleData moduleData) {

	caffewrapperState state = moduleData->moduleState;
	sshsNodePutDoubleIfAbsent(moduleData->moduleNode, "detThreshold", 0.96);
	state->detThreshold = sshsNodeGetDouble(moduleData->moduleNode,
			"detThreshold");
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doPrintOutputs", false);
	state->doPrintOutputs = sshsNodeGetBool(moduleData->moduleNode,
			"doPrintOutputs");
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doShowActivations", false);
	state->doShowActivations = sshsNodeGetBool(moduleData->moduleNode,
			"doShowActivations");
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doNormInputImages", false);
	state->doNormInputImages = sshsNodeGetBool(moduleData->moduleNode,
				"doNormInputImages");

	//Initializing caffe network..
	state->cpp_class = newMyCaffe();
	MyCaffe_init_network(state->cpp_class);

	return (true);
}

static void caerCaffeWrapperExit(caerModuleData moduleData) {
	caffewrapperState state = moduleData->moduleState;
	deleteMyCaffe(state->cpp_class); //free memory block
}

static void caerCaffeWrapperRun(caerModuleData moduleData, size_t argsNumber,
		va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	int * hist = va_arg(args, int *);
	int size = va_arg(args, int);
	char *classificationResults = va_arg(args, char*);
	caerFrameEventPacket *networkActivity = va_arg(args, caerFrameEventPacket*);
	int sizeDisplay = va_arg(args, int);

	caffewrapperState state = moduleData->moduleState;

	//update module state
	state->detThreshold = sshsNodeGetDouble(moduleData->moduleNode,
			"detThreshold");
	state->doPrintOutputs = sshsNodeGetBool(moduleData->moduleNode,
			"doPrintOutputs");
	state->doShowActivations = sshsNodeGetBool(moduleData->moduleNode,
			"doShowActivations");
	state->doNormInputImages = sshsNodeGetBool(moduleData->moduleNode,
			"doNormInputImages");

	//allocate single frame
	int frame_x = sizeDisplay;
	int frame_y = sizeDisplay;

	// set source info for this module if not yet defined
	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode,
			"sourceInfo/");
	if (!sshsNodeAttributeExists(sourceInfoNode, "visualizerSizeX", SSHS_SHORT)) {
		sshsNodePutShort(sourceInfoNode, "visualizerSizeX", (uint16_t)frame_x);
		sshsNodePutShort(sourceInfoNode, "visualizerSizeY", (uint16_t)frame_y);
	}

	*networkActivity = caerFrameEventPacketAllocate(1,
			I16T(moduleData->moduleID), 0, frame_x, frame_y, 1);
	if (*networkActivity != NULL) {
		caerFrameEvent single_frame = caerFrameEventPacketGetEvent(
				*networkActivity, 0);
		//add info to the frame
		caerFrameEventSetLengthXLengthYChannelNumber(single_frame, frame_x,
				frame_y, 1, *networkActivity); // to do remove hard coded size
				MyCaffe_file_set(state->cpp_class, hist, size,
						classificationResults, state->detThreshold,
						state->doPrintOutputs, &single_frame,
						state->doShowActivations, state->doNormInputImages);
		// validate frame
		if (single_frame != NULL) {
			caerFrameEventValidate(single_frame, *networkActivity);
		} else {
			*networkActivity = NULL;
		}

	}
	return;
}
