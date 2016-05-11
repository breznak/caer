#include "calibration.hpp"
#include "calibration_wrapper.h"

PoseCalibration *posecalibration_init(PoseCalibrationSettings settings) {
	try {
		return (new PoseCalibration(settings));
	}
	catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "PoseCalibration()", "Failed with C++ exception: %s", ex.what());
		return (NULL);
	}
}

void posecalibration_destroy(PoseCalibration *calibClass) {
	try {
		delete calibClass;
	}
	catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "PoseCalibration_destroy()", "Failed with C++ exception: %s", ex.what());
	}
}

void posecalibration_updateSettings(PoseCalibration *calibClass) {
	
}

bool calibration_findMarkers(PoseCalibration *calibClass, caerFrameEvent frame) {
	try {
		return (calibClass->findMarkers(frame));
	}
	catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "calibration_findMarkers()", "Failed with C++ exception: %s", ex.what());
		return (false);
	}
}

