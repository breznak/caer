#include "opticflow.h"
#include "opticflow_wrapper.h"

#include "base/mainloop.h"
#include "base/module.h"

struct OpticFlowState_struct {
	struct OpticFlowSettings_struct settings; // Struct containing all settings (shared)
	struct OpticFlow *cpp_class; // Pointer to cpp_class_object
};

typedef struct OpticFlowState_struct *OpticFlowState;

static bool caerOpticFlowInit(caerModuleData moduleData);
static void caerOpticFlowRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerOpticFlowConfig(caerModuleData moduleData);
static void caerOpticFlowExit(caerModuleData moduleData);
static void updateSettings(caerModuleData moduleData);

static struct caer_module_functions caerOpticFlowFunctions = { .moduleInit = &caerOpticFlowInit, .moduleRun =
	&caerOpticFlowRun, .moduleConfig = &caerOpticFlowConfig, .moduleExit = &caerOpticFlowExit };

caerFrameEventPacket caerOpticFlow(uint16_t moduleID, caerFrameEventPacket frameInput) {

	caerFrameEventPacket frame = NULL;

	caerModuleData moduleData = caerMainloopFindModule(moduleID, "OpticFlow", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return(frame);
	}

	caerModuleSM(&caerOpticFlowFunctions, moduleData, sizeof(struct OpticFlowState_struct), 2, frameInput, &frame);

	return (frame);
}

static bool caerOpticFlowInit(caerModuleData moduleData) {
	OpticFlowState state = moduleData->moduleState;

	// Create config settings.
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doOpticFlow", false); // Do OpticFlow using live images

	// Update all settings.
	updateSettings(moduleData);

	// Initialize C++ class for OpenCV integration.
	state->cpp_class = OpticFlow_init(&state->settings);
	if (state->cpp_class == NULL) {
		return (false);
	}

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	return (true);
}

static void updateSettings(caerModuleData moduleData) {
	OpticFlowState state = moduleData->moduleState;

	// Get current config settings.
	state->settings.doOpticFlow = sshsNodeGetBool(moduleData->moduleNode, "doOpticFlow");
}

static void caerOpticFlowExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	OpticFlowState state = moduleData->moduleState;

	OpticFlow_destroy(state->cpp_class);

}

static void caerOpticFlowConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	OpticFlowState state = moduleData->moduleState;

	// Reload all local settings.
	updateSettings(moduleData);

	// Update the C++ internal state, based on new settings.
	OpticFlow_updateSettings(state->cpp_class);

}

static void caerOpticFlowRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerFrameEventPacket frameInput = va_arg(args, caerFrameEventPacket);
	caerFrameEventPacket *frame = va_arg(args, caerFrameEventPacket*);

	if (frameInput == NULL) {
		return;
	}

	OpticFlowState state = moduleData->moduleState;

	caerFrameEvent eventS = caerFrameEventPacketGetEvent(frameInput, 0);
	int sizeX = caerFrameEventGetLengthX(eventS);
	int sizeY = caerFrameEventGetLengthY(eventS);

	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	if (!sshsNodeAttributeExists(sourceInfoNode, "dataSizeX", SSHS_SHORT)) { //to do for visualizer change name of field to a more generic one

		sshsNodePutShort(sourceInfoNode, "dataSizeX", sizeX);
		sshsNodePutShort(sourceInfoNode, "dataSizeY", sizeY);
	}

	*frame = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, sizeX, sizeY, 3);
	caerMainloopFreeAfterLoop(&free, *frame);
	if (*frame != NULL) {

		caerFrameEvent single_frame = caerFrameEventPacketGetEvent(*frame, 0);
		//add info to the frame
		caerFrameEventSetLengthXLengthYChannelNumber(single_frame, sizeX, sizeY, 3, *frame); // to do remove hard coded size

		caerFrameEvent single_frame_in = caerFrameEventPacketGetEvent(frameInput, 0);
		CAER_FRAME_ITERATOR_VALID_START(frameInput)

			int sizeX = caerFrameEventGetLengthX(caerFrameIteratorElement);
			int sizeY = caerFrameEventGetLengthY(caerFrameIteratorElement);
			OpticFlow_doOpticFlow(state->cpp_class, &single_frame, &single_frame_in, sizeX, sizeY);

			// validate frame
			if (single_frame != NULL) {
				caerFrameEventValidate(single_frame, *frame);
			}
			else {
				*frame = NULL;
			}

		CAER_FRAME_ITERATOR_VALID_END

	}

}
