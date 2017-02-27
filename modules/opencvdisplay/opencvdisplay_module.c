/* openCV Interface cAER module
 *  Author: federico.corradi@inilabs.com
 */

#include "base/mainloop.h"
#include "base/module.h"
#include "wrapper.h"
#include "opencvdisplay_module.h"

struct opencvwrapper_state {
	struct MyOpenCV* cpp_class; //pointer to cpp_class_object
};

typedef struct opencvwrapper_state *opencvwrapperState;

static bool caerOpenCVDisplayInit(caerModuleData moduleData);
static void caerOpenCVDisplayRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerOpenCVDisplayExit(caerModuleData moduleData);

static struct caer_module_functions caerOpenCVDisplayFunctions = { .moduleInit = &caerOpenCVDisplayInit, .moduleRun =
	&caerOpenCVDisplayRun, .moduleConfig =
NULL, .moduleExit = &caerOpenCVDisplayExit };

caerFrameEventPacket caerOpenCVDisplay(uint16_t moduleID, AResults displaystruct) {

	caerFrameEventPacket frame = NULL;

	caerModuleData moduleData = caerMainloopFindModule(moduleID, "caerOpenCVDisplay", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return(frame);
	}

	caerModuleSM(&caerOpenCVDisplayFunctions, moduleData, sizeof(struct opencvwrapper_state), 2, displaystruct, &frame);

	return(frame);
}

static bool caerOpenCVDisplayInit(caerModuleData moduleData) {

	opencvwrapperState state = moduleData->moduleState;

	//Initializing caffe network..
	state->cpp_class = newMyOpenCV();

	return (true);
}

static void caerOpenCVDisplayExit(caerModuleData moduleData) {
	opencvwrapperState state = moduleData->moduleState;
	deleteMyOpenCV(state->cpp_class); //free memory block
}

static void caerOpenCVDisplayRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	AResults displayInfo = va_arg(args, AResults);
	caerFrameEventPacket *frame = va_arg(args, caerFrameEventPacket*);

	opencvwrapperState state = moduleData->moduleState;

	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	if (!sshsNodeAttributeExists(sourceInfoNode, "dataSizeX", SSHS_SHORT)) { //to do for visualizer change name of field to a more generic one
		sshsNodePutShort(sourceInfoNode, "dataSizeX", FRAMESIZE);
		sshsNodePutShort(sourceInfoNode, "dataSizeY", FRAMESIZE);
	}

	*frame = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, FRAMESIZE, FRAMESIZE, 1);
	caerMainloopFreeAfterLoop(&free, *frame);
	if (*frame != NULL) {
		caerFrameEvent single_frame = caerFrameEventPacketGetEvent(*frame, 0);
		//add info to the frame
		caerFrameEventSetLengthXLengthYChannelNumber(single_frame, FRAMESIZE, FRAMESIZE, 1, *frame); // to do remove hard coded size
		MyOpenCV_generate(state->cpp_class, displayInfo, &single_frame);
		// validate frame
		if (single_frame != NULL) {
			caerFrameEventValidate(single_frame, *frame);
		}
		else {
			*frame = NULL;
		}

	}
	return;
}
