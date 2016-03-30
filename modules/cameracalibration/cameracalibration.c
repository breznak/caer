#include "cameracalibration.h"
#include "calibration_settings.h"
#include "calibration_wrapper.h"
#include "base/mainloop.h"
#include "base/module.h"

struct CameraCalibrationState_struct {
	struct CameraCalibrationSettings_struct settings; // Struct containing all settings (shared)
	struct Calibration *cpp_class; // Pointer to cpp_class_object
};

typedef struct CameraCalibrationState_struct *CameraCalibrationState;

static bool caerCameraCalibrationInit(caerModuleData moduleData);
static void caerCameraCalibrationRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerCameraCalibrationConfig(caerModuleData moduleData);
static void caerCameraCalibrationExit(caerModuleData moduleData);
static void updateSettings(caerModuleData moduleData);

static struct caer_module_functions caerCameraCalibrationFunctions = { .moduleInit = &caerCameraCalibrationInit,
	.moduleRun = &caerCameraCalibrationRun, .moduleConfig = &caerCameraCalibrationConfig, .moduleExit =
		&caerCameraCalibrationExit };

void caerCameraCalibration(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "CameraCalibration");

	caerModuleSM(&caerCameraCalibrationFunctions, moduleData, sizeof(struct CameraCalibrationState_struct), 2, polarity,
		frame);
}

static bool caerCameraCalibrationInit(caerModuleData moduleData) {
	CameraCalibrationState state = moduleData->moduleState;

	// Create config settings.
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doCalibration", true); // Do calibration using live images
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "saveFileName", "camera_calib.xml"); // The name of the file where to write the calculated calibration settings
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "captureDelay", 500000); // Only use a frame for calibration if at least this much time has passed
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "minNumberOfPoints", 10);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "maxTotalError", 0.05f);
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "calibrationPattern", "chessboard"); // One of the Chessboard, circles, or asymmetric circle pattern
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "boardWidth", 5); // The size of the board (width)
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "boardHeigth", 5); // The size of the board (heigth)
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "boardSquareSize", 1.0f); // The size of a square in your defined unit (point, millimeter, etc.)
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "aspectRatio", 0.75f); // The aspect ratio
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "assumeZeroTangentialDistortion", false); // Assume zero tangential distortion
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "fixPrincipalPointAtCenter", false); // Fix the principal point at the center
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "useFisheyeModel", false); // Use Fisheye camera model for calibration

	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doUndistortion", false); // Do undistortion of incoming images using calibration loaded from file
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "loadFileName", "camera_calib.xml"); // The name of the file from which to load the calibration settings for undistortion

	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	// Update all settings.
	updateSettings(moduleData);

	// Initialize C++ class for OpenCV integration.
	state->cpp_class = calibration_init(&state->settings);

	return (true);
}

static void updateSettings(caerModuleData moduleData) {
	CameraCalibrationState state = moduleData->moduleState;

	// Get current config settings.
	state->settings.doCalibration = sshsNodeGetBool(moduleData->moduleNode, "doCalibration");
	state->settings.captureDelay = sshsNodeGetInt(moduleData->moduleNode, "captureDelay");
	state->settings.minNumberOfPoints = sshsNodeGetInt(moduleData->moduleNode, "minNumberOfPoints");
	state->settings.maxTotalError = sshsNodeGetFloat(moduleData->moduleNode, "maxTotalError");
	state->settings.boardWidth = sshsNodeGetInt(moduleData->moduleNode, "boardWidth");
	state->settings.boardHeigth = sshsNodeGetInt(moduleData->moduleNode, "boardHeigth");
	state->settings.boardSquareSize = sshsNodeGetFloat(moduleData->moduleNode, "boardSquareSize");
	state->settings.aspectRatio = sshsNodeGetFloat(moduleData->moduleNode, "aspectRatio");
	state->settings.assumeZeroTangentialDistortion = sshsNodeGetBool(moduleData->moduleNode,
		"assumeZeroTangentialDistortion");
	state->settings.fixPrincipalPointAtCenter = sshsNodeGetBool(moduleData->moduleNode, "fixPrincipalPointAtCenter");
	state->settings.useFisheyeModel = sshsNodeGetBool(moduleData->moduleNode, "useFisheyeModel");
	state->settings.doUndistortion = sshsNodeGetBool(moduleData->moduleNode, "doUndistortion");

	// Parse calibration pattern string.
	char *calibPattern = sshsNodeGetString(moduleData->moduleNode, "calibrationPattern");

	if (strcmp(calibPattern, "chessboard") == 0) {
		state->settings.calibrationPattern = CAMCALIB_CHESSBOARD;
	}
	else if (strcmp(calibPattern, "circlesGrid") == 0) {
		state->settings.calibrationPattern = CAMCALIB_CIRCLES_GRID;
	}
	else if (strcmp(calibPattern, "asymmetricCirclesGrid") == 0) {
		state->settings.calibrationPattern = CAMCALIB_ASYMMETRIC_CIRCLES_GRID;
	}
	else {
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
			"Invalid calibration pattern defined. Select one of: chessboard, circlesGrid or asymmetricCirclesGrid. Defaulting to chessboard.");

		state->settings.calibrationPattern = CAMCALIB_CHESSBOARD;
	}

	free(calibPattern);

	// Get file strings.
	state->settings.saveFileName = sshsNodeGetString(moduleData->moduleNode, "saveFileName");
	state->settings.loadFileName = sshsNodeGetString(moduleData->moduleNode, "loadFileName");
}

static void caerCameraCalibrationConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	CameraCalibrationState state = moduleData->moduleState;

	free(state->settings.saveFileName);
	free(state->settings.loadFileName);

	updateSettings(moduleData);

	calibration_updateSettings(state->cpp_class);
}

static void caerCameraCalibrationExit(caerModuleData moduleData) {
	CameraCalibrationState state = moduleData->moduleState;

	calibration_destroy(state->cpp_class);

	free(state->settings.saveFileName);
	free(state->settings.loadFileName);
}

static void caerCameraCalibrationRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket);
	caerFrameEventPacket frame = va_arg(args, caerFrameEventPacket);

	// Calibration is done only using frames.

	// Undistortion can be applied to both frames and events.

}
