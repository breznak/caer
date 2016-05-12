#ifndef POSEESTIMATION_SETTINGS_H_
#define POSEESTIMATION_SETTINGS_H_


struct PoseCalibrationSettings_struct {
	bool detectMarkers;
	char *saveFileName;
	uint32_t captureDelay;
	char *loadFileName;
};

typedef struct PoseCalibrationSettings_struct *PoseCalibrationSettings;


#endif /* POSEESTIMATION_SETTINGS_H_ */
