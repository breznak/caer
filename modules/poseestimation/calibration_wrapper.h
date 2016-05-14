#ifndef CALIBRATION_WRAPPER_H_
#define CALIBRATION_WRAPPER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "calibration_settings.h"

typedef struct PoseCalibration PoseCalibration;

PoseCalibration *posecalibration_init(PoseCalibrationSettings settings);
void posecalibration_destroy(PoseCalibration *calibClass);
void posecalibration_updateSettings(PoseCalibration *calibClass);
bool posecalibration_findMarkers(PoseCalibration *calibClass, caerFrameEvent frame);
bool posecalibration_loadCalibrationFile(PoseCalibration *calibClass, PoseCalibrationSettings settings);

#ifdef __cplusplus
}
#endif

#endif /* CALIBRATION_WRAPPER_H_ */
