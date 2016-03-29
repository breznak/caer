#include "cameracalibration.h"
#include "base/mainloop.h"
#include "base/module.h"

static bool caerCameraCalibrationInit(caerModuleData moduleData);
static void caerCameraCalibrationRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerCameraCalibrationConfig(caerModuleData moduleData);
static void caerCameraCalibrationExit(caerModuleData moduleData);

static struct caer_module_functions caerCameraCalibrationFunctions = { .moduleInit = &caerCameraCalibrationInit,
	.moduleRun = &caerCameraCalibrationRun, .moduleConfig = &caerCameraCalibrationConfig, .moduleExit =
		&caerCameraCalibrationExit };

void caerCameraCalibration(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "CameraCalibration");

	caerModuleSM(&caerCameraCalibrationFunctions, moduleData, sizeof(struct CameraCalibration_state), 2, polarity,
		frame);
}

static bool caerCameraCalibrationInit(caerModuleData moduleData) {
	CameraCalibrationState state = moduleData->moduleState;

	// Create config settings.
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doCalibration", true); // Do calibration using live images
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "saveFileName", "camera_calib.xml"); // The name of the file where to write the calculated calibration settings
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "captureDelay", 100000); // Only use a frame for calibration if at least this much time has passed
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

	// Get current config settings.
	state->doCalibration = sshsNodeGetBool(moduleData->moduleNode, "doCalibration");
	state->boardWidth = sshsNodeGetInt(moduleData->moduleNode, "boardWidth");
	state->boardHeigth = sshsNodeGetInt(moduleData->moduleNode, "boardHeigth");
	state->boardSquareSize = sshsNodeGetFloat(moduleData->moduleNode, "boardSquareSize");
	state->aspectRatio = sshsNodeGetFloat(moduleData->moduleNode, "aspectRatio");
	state->assumeZeroTangentialDistortion = sshsNodeGetBool(moduleData->moduleNode, "assumeZeroTangentialDistortion");
	state->fixPrincipalPointAtCenter = sshsNodeGetBool(moduleData->moduleNode, "fixPrincipalPointAtCenter");
	state->useFisheyeModel = sshsNodeGetBool(moduleData->moduleNode, "useFisheyeModel");
	state->doUndistortion = sshsNodeGetBool(moduleData->moduleNode, "doUndistortion");

	// Check input validity.
	if (state->boardWidth <= 0 || state->boardHeigth <= 0) {
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Invalid board size.");
		return (false);
	}

	if (state->boardSquareSize <= 10e-6f) {
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Invalid board square size.");
		return (false);
	}

	if (state->aspectRatio <= 0) {
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Invalid aspect ratio.");
		return (false);
	}

	// Parse calibration pattern string.
	char *calibPattern = sshsNodeGetString(moduleData->moduleNode, "calibrationPattern");

	if (strcmp(calibPattern, "chessboard") == 0) {
		state->calibrationPattern = CAMCALIB_CHESSBOARD;
	}
	else if (strcmp(calibPattern, "circlesGrid") == 0) {
		state->calibrationPattern = CAMCALIB_CIRCLES_GRID;
	}
	else if (strcmp(calibPattern, "asymmetricCirclesGrid") == 0) {
		state->calibrationPattern = CAMCALIB_ASYMMETRIC_CIRCLES_GRID;
	}
	else {
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
			"Invalid calibration pattern defined. Select one of: chessboard, circlesGrid or asymmetricCirclesGrid.");

		free(calibPattern);
		return (false);
	}

	free(calibPattern);

	// Parse file strings.
	state->saveFileName = sshsNodeGetString(moduleData->moduleNode, "saveFileName");

	state->loadFileName = sshsNodeGetString(moduleData->moduleNode, "loadFileName");

	return (true);
}

static void caerCameraCalibrationRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket);
	caerFrameEventPacket frame = va_arg(args, caerFrameEventPacket);

	// Only process packets with content.
	if (polarity == NULL && frame == NULL) {
		return;
	}
}

static void caerCameraCalibrationConfig(caerModuleData moduleData) {
}

static void caerCameraCalibrationExit(caerModuleData moduleData) {
	CameraCalibrationState state = moduleData->moduleState;
}
