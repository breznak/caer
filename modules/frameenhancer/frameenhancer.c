#include "frameenhancer.h"
#include "base/mainloop.h"
#include "base/module.h"
#include <libcaer/frame_utils.h>

struct FrameEnhancer_state {
	bool doDemosaic;
	int demosaicType;
	bool doContrast;
	int contrastType;
};

typedef struct FrameEnhancer_state *FrameEnhancerState;

static bool caerFrameEnhancerInit(caerModuleData moduleData);
static void caerFrameEnhancerRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerFrameEnhancerConfig(caerModuleData moduleData);
static void caerFrameEnhancerExit(caerModuleData moduleData);

static struct caer_module_functions caerFrameEnhancerFunctions = { .moduleInit = &caerFrameEnhancerInit, .moduleRun =
	&caerFrameEnhancerRun, .moduleConfig = &caerFrameEnhancerConfig, .moduleExit = &caerFrameEnhancerExit };

caerFrameEventPacket caerFrameEnhancer(uint16_t moduleID, caerFrameEventPacket frame) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "FrameEnhancer", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return (NULL);
	}

	// By default, same as input frame packet.
	caerFrameEventPacket enhancedFrame = frame;

	caerModuleSM(&caerFrameEnhancerFunctions, moduleData, sizeof(struct FrameEnhancer_state), 2, frame, &enhancedFrame);

	return (enhancedFrame);
}

static bool caerFrameEnhancerInit(caerModuleData moduleData) {
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doDemosaic", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doContrast", false);

#if defined(LIBCAER_HAVE_OPENCV) && LIBCAER_HAVE_OPENCV == 1
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "demosaicType", "opencv_edge_aware");
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "contrastType", "opencv_normalization");
#else
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "demosaicType", "standard");
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "contrastType", "standard");
#endif

	// Initialize configuration.
	caerFrameEnhancerConfig(moduleData);

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	// Nothing that can fail here.
	return (true);
}

static void caerFrameEnhancerRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerFrameEventPacket frame = va_arg(args, caerFrameEventPacket);
	caerFrameEventPacket *enhancedFrame = va_arg(args, caerFrameEventPacket *);

	// Only process packets with content.
	if (frame == NULL) {
		return;
	}

	FrameEnhancerState state = moduleData->moduleState;

	if (state->doDemosaic) {
#if defined(LIBCAER_HAVE_OPENCV) && LIBCAER_HAVE_OPENCV == 1
		switch (state->demosaicType) {
			case 0:
				*enhancedFrame = caerFrameUtilsDemosaic(frame);
				break;

			case 1:
				*enhancedFrame = caerFrameUtilsOpenCVDemosaic(frame, DEMOSAIC_NORMAL);
				break;

			case 2:
				*enhancedFrame = caerFrameUtilsOpenCVDemosaic(frame, DEMOSAIC_EDGE_AWARE);
				break;
		}
#else
		*enhancedFrame = caerFrameUtilsDemosaic(frame);
#endif

		// This creates a new, independent frame, which also needs to be
		// correctly reclaimed at the end of the mainloop run.
		caerMainloopFreeAfterLoop(&free, *enhancedFrame);
	}

	if (state->doContrast) {
#if defined(LIBCAER_HAVE_OPENCV) && LIBCAER_HAVE_OPENCV == 1
		switch (state->contrastType) {
			case 0:
				caerFrameUtilsContrast(*enhancedFrame);
				break;

			case 1:
				caerFrameUtilsOpenCVContrast(*enhancedFrame, CONTRAST_NORMALIZATION);
				break;

			case 2:
				caerFrameUtilsOpenCVContrast(*enhancedFrame, CONTRAST_HISTOGRAM_EQUALIZATION);
				break;

			case 3:
				caerFrameUtilsOpenCVContrast(*enhancedFrame, CONTRAST_CLAHE);
				break;
		}
#else
		caerFrameUtilsContrast(*enhancedFrame);
#endif
	}
}

static void caerFrameEnhancerConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	FrameEnhancerState state = moduleData->moduleState;

	state->doDemosaic = sshsNodeGetBool(moduleData->moduleNode, "doDemosaic");

	char *demosaicType = sshsNodeGetString(moduleData->moduleNode, "demosaicType");

	if (caerStrEquals(demosaicType, "opencv_normal")) {
		state->demosaicType = 1;
	}
	else if (caerStrEquals(demosaicType, "opencv_edge_aware")) {
		state->demosaicType = 2;
	}
	else {
		// Standard, non-OpenCV method.
		state->demosaicType = 0;
	}

	free(demosaicType);

	state->doContrast = sshsNodeGetBool(moduleData->moduleNode, "doContrast");

	char *contrastType = sshsNodeGetString(moduleData->moduleNode, "contrastType");

	if (caerStrEquals(contrastType, "opencv_normalization")) {
		state->contrastType = 1;
	}
	else if (caerStrEquals(contrastType, "opencv_histogram_equalization")) {
		state->contrastType = 2;
	}
	else if (caerStrEquals(contrastType, "opencv_clahe")) {
		state->contrastType = 3;
	}
	else {
		// Standard, non-OpenCV method.
		state->contrastType = 0;
	}

	free(contrastType);
}

static void caerFrameEnhancerExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);
}
