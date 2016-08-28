#include "cameracalibration.h"
#include "calibration_settings.h"
#include "calibration_wrapper.h"
#include "base/mainloop.h"
#include "base/module.h"

struct PoseCalibrationState_struct {
	struct PoseCalibrationSettings_struct settings; // Struct containing all settings (shared)
	struct PoseCalibrarion *cpp_class; // Pointer to cpp_class_object
	uint64_t lastFrameTimestamp;
	size_t lastFoundPoints;
	bool calibrationLoaded;
};

typedef struct PoseCalibrationState_struct *PoseCalibrationState;

static bool caerPoseCalibrationInit(caerModuleData moduleData);
static void caerPoseCalibrationRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerPoseCalibrationConfig(caerModuleData moduleData);
static void caerPoseCalibrationExit(caerModuleData moduleData);
static void updateSettings(caerModuleData moduleData);

static struct caer_module_functions caerPoseCalibrationFunctions = { .moduleInit = &caerPoseCalibrationInit,
	.moduleRun = &caerPoseCalibrationRun, .moduleConfig = &caerPoseCalibrationConfig, .moduleExit =
		&caerPoseCalibrationExit };

void caerPoseCalibration(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "PoseEstimation", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerPoseCalibrationFunctions, moduleData, sizeof(struct PoseCalibrationState_struct), 2, polarity,
		frame);
}

static bool caerPoseCalibrationInit(caerModuleData moduleData) {
	PoseCalibrationState state = moduleData->moduleState;

	// Create config settings.
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "detectMarkers", false); // Do calibration using live images
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "saveFileName", "camera_calib.xml"); // The name of the file where to write the calculated calibration settings
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "loadFileName", "camera_calib.xml"); // The name of the file from which to load the calibration
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "captureDelay", 500000);

	// Update all settings.
	updateSettings(moduleData);

	// Initialize C++ class for OpenCV integration.
	state->cpp_class = posecalibration_init(&state->settings);
	if (state->cpp_class == NULL) {
		return (false);
	}

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);
	//not loaded at the init
	state->calibrationLoaded = false;

	return (true);
}

static void updateSettings(caerModuleData moduleData) {
	PoseCalibrationState state = moduleData->moduleState;

	state->settings.detectMarkers = sshsNodeGetBool(moduleData->moduleNode, "detectMarkers");
	state->settings.saveFileName = sshsNodeGetString(moduleData->moduleNode, "saveFileName");
	state->settings.loadFileName = sshsNodeGetString(moduleData->moduleNode, "loadFileName");
}

static void caerPoseCalibrationConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	PoseCalibrationState state = moduleData->moduleState;

}

static void caerPoseCalibrationExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	PoseCalibrationState state = moduleData->moduleState;

	//posecalibration_destroy(state->cpp_class);

	free(state->settings.saveFileName);
	free(state->settings.loadFileName);
}

static void caerPoseCalibrationRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket);
	caerFrameEventPacket frame = va_arg(args, caerFrameEventPacket);

	PoseCalibrationState state = moduleData->moduleState;

	// At this point we always try to load the calibration settings for undistortion.
	// Maybe they just got created or exist from a previous run.
	if (!state->calibrationLoaded) {
		state->calibrationLoaded = posecalibration_loadCalibrationFile(state->cpp_class, &state->settings);
	}

	// Marker pose estimation is done only using frames.
	if (state->settings.detectMarkers && frame != NULL) {
		CAER_FRAME_ITERATOR_VALID_START(frame)
		// Only work on new frames if enough time has passed between this and the last used one.
			uint64_t currTimestamp = U64T(caerFrameEventGetTSStartOfFrame64(caerFrameIteratorElement, frame));

			// If enough time has passed, try to add a new point set.
			if ((currTimestamp - state->lastFrameTimestamp) >= state->settings.captureDelay) {
				state->lastFrameTimestamp = currTimestamp;

				bool foundPoint = posecalibration_findMarkers(state->cpp_class, caerFrameIteratorElement);
				caerLog(CAER_LOG_WARNING, moduleData->moduleSubSystemString,
					"Searching for markers in the aruco set, result = %d.", foundPoint);
			}CAER_FRAME_ITERATOR_VALID_END

	}

	// update settings
	updateSettings(moduleData);

}
