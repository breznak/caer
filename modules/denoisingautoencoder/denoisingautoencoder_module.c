/* openCV Interface cAER module
 *  Author: federico.corradi@inilabs.com
 */

#include "base/mainloop.h"
#include "base/module.h"
#include "wrapper.h"
#include "denoisingautoencoder_module.h"

struct denoisingautoencoder_state {
	struct MyDenAutoEncoder* cpp_class; //pointer to cpp_class_object
};

typedef struct denoisingautoencoder_state *denoisingautoencoderState;

static bool caerDenAutoEncoderInit(caerModuleData moduleData);
static void caerDenAutoEncoderRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerDenAutoEncoderExit(caerModuleData moduleData);

static struct caer_module_functions caerDenAutoEncoderFunctions = { .moduleInit = &caerDenAutoEncoderInit, .moduleRun =
	&caerDenAutoEncoderRun, .moduleConfig =
NULL, .moduleExit = &caerDenAutoEncoderExit };

caerFrameEventPacket caerDenAutoEncoder(uint16_t moduleID, caerFrameEventPacket frameInput) {

	caerFrameEventPacket frame = NULL;

	caerModuleData moduleData = caerMainloopFindModule(moduleID, "caerDenAutoEncoder", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return(frame);
	}

	caerModuleSM(&caerDenAutoEncoderFunctions, moduleData, sizeof(struct denoisingautoencoder_state), 1, frameInput, &frame);

	return(frame);
}

static bool caerDenAutoEncoderInit(caerModuleData moduleData) {

	denoisingautoencoderState state = moduleData->moduleState;

	//Initializing denoising auto-encoder..
	state->cpp_class = newMyDenAutoEncoder();

	return (true);
}

static void caerDenAutoEncoderExit(caerModuleData moduleData) {
	denoisingautoencoderState state = moduleData->moduleState;
	deleteMyDenAutoEncoder(state->cpp_class); //free memory block
}

static void caerDenAutoEncoderRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	caerFrameEventPacket frameInput = va_arg(args, caerFrameEventPacket);
	caerFrameEventPacket *frame = va_arg(args, caerFrameEventPacket*);

	if (frameInput == NULL) {
		return;
	}

	denoisingautoencoderState state = moduleData->moduleState;

	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	if (!sshsNodeAttributeExists(sourceInfoNode, "dataSizeX", SSHS_SHORT)) { //to do for visualizer change name of field to a more generic one
		sshsNodePutShort(sourceInfoNode, "dataSizeX", FRAMESIZE);
		sshsNodePutShort(sourceInfoNode, "dataSizeY", FRAMESIZE);
	}

	// loop over all input frames
	caerFrameEvent single_frame_in = caerFrameEventPacketGetEvent(frameInput, 0);

	*frame = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, FRAMESIZE, FRAMESIZE, 3);
	caerMainloopFreeAfterLoop(&free, *frame);
	if (*frame != NULL) {
		caerFrameEvent encoders = caerFrameEventPacketGetEvent(*frame, 0);
		//add info to the frame
		caerFrameEventSetLengthXLengthYChannelNumber(encoders, FRAMESIZE, FRAMESIZE, 3, *frame); // to do remove hard coded size
		MyDenAutoEncoder_generate(state->cpp_class, &single_frame_in, &encoders);
		// validate frame
		if (encoders != NULL) {
			caerFrameEventValidate(encoders, *frame);
		}
		else {
			*frame = NULL;
		}

	}
	return;
}
