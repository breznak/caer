#ifndef CALIBRATION_WRAPPER_H_
#define CALIBRATION_WRAPPER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "cameracalibration.h"

typedef struct Calibration Calibration;

Calibration *calibration_init(CameraCalibrationState state);
void calibration_destroy(Calibration *calibClass);
void calibration_updateSettings(Calibration *calibClass);
bool calibration_findNewPoints(Calibration *calibClass, caerFrameEvent frame);
size_t calibration_foundPoints(Calibration *calibClass);
bool calibration_runCalibrationAndSave(Calibration *calibClass);
bool calibration_loadUndistortMatrices(Calibration *calibClass);
void calibration_undistortEvent(Calibration *calibClass, caerPolarityEvent polarity);
void calibration_undistortFrame(Calibration *calibClass, caerFrameEvent frame);

#ifdef __cplusplus
}
#endif

#endif /* CALIBRATION_WRAPPER_H_ */
