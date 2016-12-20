#ifndef CALIBRATION_WRAPPER_H_
#define CALIBRATION_WRAPPER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "calibration_settings.h"

typedef struct StereoCalibration StereoCalibration;

StereoCalibration *StereoCalibration_init(StereoCalibrationSettings settings);
void StereoCalibration_destroy(StereoCalibration *calibClass);
void StereoCalibration_updateSettings(StereoCalibration *calibClass);
void * StereoCalibration_findNewPoints(StereoCalibration *calibClass,
		caerFrameEvent frame, int camid);
void StereoCalibration_freeStereoVec(void * vec1, void * vec2);
bool StereoCalibration_stereoCalibrate(StereoCalibration *calibClass,
		StereoCalibrationSettings settings);
void StereoCalibration_addStereoCalibVec(StereoCalibration *calibClass,
		void * vec1, void * vec2);
bool StereoCalibration_loadCalibrationFile(StereoCalibration *calibClass,
		StereoCalibrationSettings settings);
bool StereoCalibration_loadStereoCalibration(StereoCalibration *calibClass,
		StereoCalibrationSettings settings);
void StereoCalibration_clearImagePoints(StereoCalibration *calibClass);

#ifdef __cplusplus
}
#endif

#endif /* CALIBRATION_WRAPPER_H_ */
