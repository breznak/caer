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
	struct MyClass* cpp_class; //pointer to cpp_class_object
};

typedef struct caffewrapper_state *caffewrapperState;

static bool caerCaffeWrapperInit(caerModuleData moduleData);
static void caerCaffeWrapperRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerCaffeWrapperExit(caerModuleData moduleData);

static struct caer_module_functions caerCaffeWrapperFunctions = { .moduleInit = &caerCaffeWrapperInit, .moduleRun =
	&caerCaffeWrapperRun, .moduleConfig =
NULL, .moduleExit = &caerCaffeWrapperExit };

const char * caerCaffeWrapper(uint16_t moduleID, char ** file_string, double *classificationResults, int max_img_qty,
	caerFrameEventPacket *networkActivity) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "caerCaffeWrapper");
	if (moduleData == NULL) {
		return (NULL);
	}

	caerModuleSM(&caerCaffeWrapperFunctions, moduleData, sizeof(struct caffewrapper_state), 4, file_string,
		classificationResults, max_img_qty, networkActivity);

	return (NULL);
}

static bool caerCaffeWrapperInit(caerModuleData moduleData) {

	caffewrapperState state = moduleData->moduleState;
	sshsNodePutDoubleIfAbsent(moduleData->moduleNode, "detThreshold", 0.96);
	state->detThreshold = sshsNodeGetDouble(moduleData->moduleNode, "detThreshold");
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doPrintOutputs", false);
	state->doPrintOutputs = sshsNodeGetBool(moduleData->moduleNode, "doPrintOutputs");
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doShowActivations", false);
	state->doShowActivations = sshsNodeGetBool(moduleData->moduleNode, "doShowActivations");

	//Initializing caffe network..
	state->cpp_class = newMyClass();
	MyClass_init_network(state->cpp_class);

	return (true);
}

static void caerCaffeWrapperExit(caerModuleData moduleData) {
	caffewrapperState state = moduleData->moduleState;
	deleteMyClass(state->cpp_class); //free memory block
}

static void caerCaffeWrapperRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);
	caffewrapperState state = moduleData->moduleState;
	char ** file_string = va_arg(args, char **);
	double *classificationResults = va_arg(args, double*);
	int max_img_qty = va_arg(args, int);
	caerFrameEventPacket *networkActivity = va_arg(args, caerFrameEventPacket*);

	//update module state
	state->detThreshold = sshsNodeGetDouble(moduleData->moduleNode, "detThreshold");
	state->doPrintOutputs = sshsNodeGetBool(moduleData->moduleNode, "doPrintOutputs");
	state->doShowActivations = sshsNodeGetBool(moduleData->moduleNode, "doShowActivations");

	//allocate single frame
	*networkActivity = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, 640, 480, 3);
	caerFrameEvent single_frame = caerFrameEventPacketGetEvent(*networkActivity, 0);
	//add info to the frame
	caerFrameEventSetLengthXLengthYChannelNumber(single_frame, 640, 480, 3, *networkActivity); // to do remove hard coded size
	//single_frame->pixels[0] = (uint16_t) (20);

	for (int i = 0; i < max_img_qty; ++i) {
		if (file_string[i] != NULL) {
			MyClass_file_set(state->cpp_class, file_string[i], &classificationResults[i], state->detThreshold,
				state->doPrintOutputs, single_frame, state->doShowActivations);
		}
	}
	// validate frame
	if (single_frame != NULL) {
		caerFrameEventValidate(single_frame, *networkActivity);
	}
	else {
		*networkActivity = NULL;
	}

	return;
}
