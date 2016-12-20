#include "calibration.hpp"
#include "calibration_wrapper.h"

StereoCalibration *StereoCalibration_init(StereoCalibrationSettings settings) {
	try {
		return (new StereoCalibration(settings));
	} catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "StereoCalibration()",
				"Failed with C++ exception: %s", ex.what());
		return (NULL);
	}
}

void StereoCalibration_destroy(StereoCalibration *calibClass) {
	try {
		delete calibClass;
	} catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "StereoCalibration_destroy()",
				"Failed with C++ exception: %s", ex.what());
	}
}

void StereoCalibration_updateSettings(StereoCalibration *calibClass) {

}

void StereoCalibration_freeStereoVec(void * vec1, void * vec2) {

	vector<Point2f> *tmp1, *tmp2;
	tmp1 = ((vector<Point2f>*) vec1);
	tmp2 = ((vector<Point2f>*) vec2);

	if (tmp1 != NULL)
		delete tmp1;
	if (tmp2 != NULL)
		delete tmp2;

	return;
}

void StereoCalibration_clearImagePoints(StereoCalibration *calibClass){

	try {
			calibClass->clearImagePoints();
		} catch (const std::exception& ex) {
			caerLog(CAER_LOG_ERROR, "StereoCalibration_clearImagePoints()",
					"Failed with C++ exception: %s", ex.what());
		}

}

void StereoCalibration_addStereoCalibVec(StereoCalibration *calibClass,
		void * vec1, void * vec2) {

	vector<Point2f> *tmp1, *tmp2;
	tmp1 = ((vector<Point2f>*) vec1);
	tmp2 = ((vector<Point2f>*) vec2);

	try {
		calibClass->addStereoCalib(tmp1, tmp2);
	} catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "StereoCalibration_addStereoCalibVec()",
				"Failed with C++ exception: %s", ex.what());
	}

}

void * StereoCalibration_findNewPoints(StereoCalibration *calibClass,
		caerFrameEvent frame, int camid) {
	try {
		return (calibClass->findNewPoints(frame, camid));
	} catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "StereoCalibration_multicalib()",
				"Failed with C++ exception: %s", ex.what());
		return (NULL);
	}
}

bool StereoCalibration_stereoCalibrate(StereoCalibration *calibClass,
		StereoCalibrationSettings settings) {
	try {
		return (calibClass->stereoCalibrate(settings));
	} catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "StereoCalibration_stereoCalibrate()",
				"Failed with C++ exception: %s", ex.what());
		return (false);
	}
}

bool StereoCalibration_loadCalibrationFile(StereoCalibration *calibClass,
		StereoCalibrationSettings settings) {
	try {
		return (calibClass->loadCalibrationFile(settings));
	} catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "StereoCalibration_loadCalibrationFile()",
				"Failed with C++ exception: %s", ex.what());
		return (false);
	}
}
