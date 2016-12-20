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

bool StereoMatching_stereoMatch(StereoMatching *calibClass, StereoMatchingSettings settings,
	caerFrameEvent  vec1, caerFrameEvent  vec2) {

	try {
			return (calibClass->stereoMatch(settings, vec1, vec2));
	} catch (const std::exception& ex) {
			caerLog(CAER_LOG_ERROR, "StereoMatching_stereMatch()",
					"Failed with C++ exception: %s", ex.what());
			return (false);
	}

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
