#include "stereomatching.h"
#include "matching_settings.h"
#include "stereomatching_wrapper.h"
#include "base/mainloop.h"
#include "base/module.h"

struct StereoMatchingState_struct {
	struct StereoMatchingSettings_struct settings; // Struct containing all settings (shared)
	struct StereoMatching *cpp_class; 			  // Pointer to cpp_class_object
	uint64_t lastFrameTimestamp_cam0;
	uint64_t lastFrameTimestamp_cam1;
	uint32_t points_found;
	uint32_t last_points_found;
	size_t lastFoundPoints;
	bool calibrationLoaded;
};

typedef struct StereoMatchingState_struct *StereoMatchingState;

static bool caerStereoMatchingInit(caerModuleData moduleData);
static void caerStereoMatchingRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerStereoMatchingConfig(caerModuleData moduleData);
static void caerStereoMatchingExit(caerModuleData moduleData);
static void updateSettings(caerModuleData moduleData);

static struct caer_module_functions caerStereoMatchingFunctions = { .moduleInit = &caerStereoMatchingInit,
	.moduleRun = &caerStereoMatchingRun, .moduleConfig = &caerStereoMatchingConfig, .moduleExit =
		&caerStereoMatchingExit };

void caerStereoMatching(uint16_t moduleID, caerFrameEventPacket frame_0, caerFrameEventPacket frame_1) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "StereoMatching", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerStereoMatchingFunctions, moduleData, sizeof(struct StereoMatchingState_struct), 2, frame_0,
		frame_1);
}

static bool caerStereoMatchingInit(caerModuleData moduleData) {
	StereoMatchingState state = moduleData->moduleState;

	// Create config settings.
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doMatching", false); // Do calibration using live images
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "captureDelay", 2000);
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "loadFileName_extrinsic", "extrinsics.xml"); // The name of the file from which to load the calibration
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "loadFileName_intrinsic", "intrinsics.xml"); // The name of the file from which to load the calibration

	// Update all settings.
	updateSettings(moduleData);

	// Initialize C++ class for OpenCV integration.
	state->cpp_class = StereoMatching_init(&state->settings);
	if (state->cpp_class == NULL) {
		return (false);
	}

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	return (true);
}

static void updateSettings(caerModuleData moduleData) {
	StereoMatchingState state = moduleData->moduleState;

	state->settings.doMatching = sshsNodeGetBool(moduleData->moduleNode, "doMatching");
	state->settings.captureDelay = sshsNodeGetInt(moduleData->moduleNode, "captureDelay");
	state->settings.loadFileName_extrinsic = sshsNodeGetString(moduleData->moduleNode, "loadFileName_extrinsic");
	state->settings.loadFileName_intrinsic = sshsNodeGetString(moduleData->moduleNode, "loadFileName_intrinsic");


}

static void caerStereoMatchingConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	StereoMatchingState state = moduleData->moduleState;

}

static void caerStereoMatchingExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	StereoMatchingState state = moduleData->moduleState;


}

static void caerStereoMatchingRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerFrameEventPacket frame_0 = va_arg(args, caerFrameEventPacket);
	caerFrameEventPacket frame_1 = va_arg(args, caerFrameEventPacket);

	StereoMatchingState state = moduleData->moduleState;

	// At this point we always try to load the calibration settings for undistorsion.
	// Maybe they just got created or exist from a previous run.
	if (!state->calibrationLoaded) {
		state->calibrationLoaded = StereoMatching_loadCalibrationFile(state->cpp_class, &state->settings);
	}

	// Stereo Camera calibration is done only using frames.
	if (state->settings.doMatching && frame_0 != NULL && frame_1 != NULL) {

		//caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Looking for calibration patterns...");

		bool frame_0_pattern = false;
		uint64_t frame_0_ts = NULL;
		bool frame_1_pattern = false;
		uint64_t frame_1_ts = NULL;
		void * foundPoint_cam1 = NULL;
		void * foundPoint_cam0 = NULL;


		// get last valid frame in packet for both cameras
		caerFrameEventPacket currFramePacket_cam0 = (caerFrameEventPacket) frame_0;
		caerFrameEvent currFrameEvent_cam0;
		bool have_frame_0 = false;

		for (int32_t i = caerEventPacketHeaderGetEventNumber(&currFramePacket_cam0->packetHeader) - 1; i >= 0; i--) {
			currFrameEvent_cam0 = caerFrameEventPacketGetEvent(currFramePacket_cam0, i);
			if (caerFrameEventIsValid(currFrameEvent_cam0)) {
				//currFrameEvent_cam0
				have_frame_0 = true;
				break;
			}
		}

		caerFrameEventPacket currFramePacket_cam1 = (caerFrameEventPacket) frame_1;
		caerFrameEvent currFrameEvent_cam1;
		bool have_frame_1 = false;

		for (int32_t i = caerEventPacketHeaderGetEventNumber(&currFramePacket_cam1->packetHeader) - 1; i >= 0; i--) {
			currFrameEvent_cam1 = caerFrameEventPacketGetEvent(currFramePacket_cam1, i);
			if (caerFrameEventIsValid(currFrameEvent_cam1)) {
				//currFrameEvent_cam1
				have_frame_1 = true;
				break;
			}
		}

		if(have_frame_1 && have_frame_0){
			//we got frames proceed with stereo matching
			StereoMatching_stereoMatch(state->cpp_class,  &state->settings, currFrameEvent_cam0, currFrameEvent_cam1);
		}

	}

	// update settings
	updateSettings(moduleData);

}
