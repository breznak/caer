#include "calibration.hpp"
#include "calibration_wrapper.h"

extern "C" {

Calibration *calibration_init(CameraCalibrationSettings settings) {
	return (new Calibration(settings));
}

void calibration_destroy(Calibration *calibClass) {
	delete calibClass;
}

void calibration_updateSettings(Calibration *calibClass) {
	calibClass->updateSettings();
}

bool calibration_findNewPoints(Calibration *calibClass, caerFrameEvent frame) {
	return (calibClass->findNewPoints(frame));
}

size_t calibration_foundPoints(Calibration *calibClass) {
	return (calibClass->foundPoints());
}

bool calibration_runCalibrationAndSave(Calibration *calibClass) {
	return (calibClass->runCalibrationAndSave());
}

bool calibration_loadUndistortMatrices(Calibration *calibClass) {
	return (calibClass->loadUndistortMatrices());
}

void calibration_undistortEvent(Calibration *calibClass, caerPolarityEvent polarity) {
	calibClass->undistortEvent(polarity);
}

void calibration_undistortFrame(Calibration *calibClass, caerFrameEvent frame) {
	calibClass->undistortFrame(frame);
}

}
