#include "frameenhancer.h"
#include "base/mainloop.h"
#include "base/module.h"

struct FrameEnhancer_state {

};

typedef struct FrameEnhancer_state *FrameEnhancerState;

static bool caerFrameEnhancerInit(caerModuleData moduleData);
static void caerFrameEnhancerRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerFrameEnhancerConfig(caerModuleData moduleData);
static void caerFrameEnhancerExit(caerModuleData moduleData);

static struct caer_module_functions caerFrameEnhancerFunctions = { .moduleInit = &caerFrameEnhancerInit, .moduleRun =
	&caerFrameEnhancerRun, .moduleConfig = &caerFrameEnhancerConfig, .moduleExit = &caerFrameEnhancerExit };

caerFrameEventPacket caerFrameEnhancer(uint16_t moduleID, caerFrameEventPacket frame) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "FrameEnhancer");

	caerModuleSM(&caerFrameEnhancerFunctions, moduleData, sizeof(struct FrameEnhancer_state), 1, frame);

	return (frame);
}

static bool caerFrameEnhancerInit(caerModuleData moduleData) {
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	FrameEnhancerState state = moduleData->moduleState;

	// Nothing that can fail here.
	return (true);
}

static void caerFrameEnhancerRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerFrameEventPacket frame = va_arg(args, caerFrameEventPacket);

	// Only process packets with content.
	if (frame == NULL) {
		return;
	}

	FrameEnhancerState state = moduleData->moduleState;

}

static void caerFrameEnhancerConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	FrameEnhancerState state = moduleData->moduleState;

}

static void caerFrameEnhancerExit(caerModuleData moduleData) {
	FrameEnhancerState state = moduleData->moduleState;

}
