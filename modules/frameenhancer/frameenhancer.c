#include "frameenhancer.h"
#include "base/mainloop.h"
#include "base/module.h"
#include <libcaer/frame_utils.h>
#ifdef ENABLE_FRAMEENHANCER_OPENCV
	#include <libcaer/frame_utils_opencv.h>
#endif

struct FrameEnhancer_state {
	bool doDemosaic;
	int demosaicType;
	bool doContrast;
	int contrastType;
	bool doWhiteBalance;
	int whiteBalanceType;
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

	caerFrameEventPacket enhancedFrame = NULL;

	caerModuleSM(&caerFrameEnhancerFunctions, moduleData, sizeof(struct FrameEnhancer_state), 2, frame, &enhancedFrame);

	return (enhancedFrame);
}

static bool caerFrameEnhancerInit(caerModuleData moduleData) {
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doDemosaic", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doContrast", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doWhiteBalance", false);

#ifdef ENABLE_FRAMEENHANCER_OPENCV
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "demosaicType", "opencv_edge_aware");
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "contrastType", "opencv_clahe");
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "whiteBalanceType", "opencv_grayworld");
#else
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "demosaicType", "standard");
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "contrastType", "standard");
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "whiteBalanceType", "standard");
#endif

	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	FrameEnhancerState state = moduleData->moduleState;

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
#ifdef ENABLE_FRAMEENHANCER_OPENCV
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
	}

	if (state->doWhiteBalance) {
#ifdef ENABLE_FRAMEENHANCER_OPENCV
		switch (state->whiteBalanceType) {
			case 0:
				caerFrameUtilsWhiteBalance(frame);
				break;

			case 1:
				caerFrameUtilsOpenCVWhiteBalance(frame, WHITEBALANCE_SIMPLE);
				break;

			case 2:
				caerFrameUtilsOpenCVWhiteBalance(frame, WHITEBALANCE_GRAYWORLD);
				break;
		}
#else
		caerFrameUtilsWhiteBalance(frame);
#endif
	}

	if (state->doContrast) {
#ifdef ENABLE_FRAMEENHANCER_OPENCV
		switch (state->contrastType) {
			case 0:
				caerFrameUtilsContrast(frame);
				break;

			case 1:
				caerFrameUtilsOpenCVContrast(frame, CONTRAST_NORMALIZATION);
				break;

			case 2:
				caerFrameUtilsOpenCVContrast(frame, CONTRAST_HISTOGRAM_EQUALIZATION);
				break;

			case 3:
				caerFrameUtilsOpenCVContrast(frame, CONTRAST_CLAHE);
				break;
		}
#else
		caerFrameUtilsContrast(frame);
#endif
	}
}

static void caerFrameEnhancerConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	FrameEnhancerState state = moduleData->moduleState;

}

static void caerFrameEnhancerExit(caerModuleData moduleData) {
	FrameEnhancerState state = moduleData->moduleState;

}
