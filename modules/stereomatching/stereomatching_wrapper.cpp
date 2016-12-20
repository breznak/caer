#include "stereomatching.hpp"
#include "stereomatching_wrapper.h"

StereoMatching *StereoMatching_init(StereoMatchingSettings settings) {
	try {
		return (new StereoMatching(settings));
	} catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "StereoMatching()",
				"Failed with C++ exception: %s", ex.what());
		return (NULL);
	}
}

void StereoMatching_destroy(StereoMatching *calibClass) {
	try {
		delete calibClass;
	} catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "StereoMatching_destroy()",
				"Failed with C++ exception: %s", ex.what());
	}
}

void StereoMatching_updateSettings(StereoMatching *calibClass) {

}

void StereoMatching_stereoMatch(StereoMatching *calibClass,
	caerFrameEvent  vec1, caerFrameEvent  vec2) {

	// Initialize OpenCV Mat based on caerFrameEvent data directly (no image copy).
	Size frameSize_cam0(caerFrameEventGetLengthX(vec1), caerFrameEventGetLengthY(vec1));
	Mat Image_cam0(frameSize_cam0, CV_16UC(caerFrameEventGetChannelNumber(vec1)), caerFrameEventGetPixelArrayUnsafe(vec1));

	Size frameSize_cam1(caerFrameEventGetLengthX(vec2), caerFrameEventGetLengthY(vec2));
	Mat Image_cam1(frameSize_cam1, CV_16UC(caerFrameEventGetChannelNumber(vec2)), caerFrameEventGetPixelArrayUnsafe(vec2));


}

bool StereoMatching_loadCalibrationFile(StereoMatching *calibClass,
		StereoMatchingSettings settings) {
	try {
		return (calibClass->loadCalibrationFile(settings));
	} catch (const std::exception& ex) {
		caerLog(CAER_LOG_ERROR, "StereoMatching_loadCalibrationFile()",
				"Failed with C++ exception: %s", ex.what());
		return (false);
	}
}
