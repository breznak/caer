#ifndef CAMERACALIBRATION_SETTINGS_H_
#define CAMERACALIBRATION_SETTINGS_H_

enum CameraCalibrationPattern { CAMCALIB_CHESSBOARD, CAMCALIB_CIRCLES_GRID, CAMCALIB_ASYMMETRIC_CIRCLES_GRID };

struct StereoCalibrationSettings_struct {
	bool doCalibration;
	bool isCalibrated;
	char *loadFileName_cam0;
	bool useFisheyeModel_cam0;
	bool useFisheyeModel_cam1;
	char *loadFileName_cam1;
	char *saveFileName_extrinsics;
	char *saveFileName_intrinsics;
	uint32_t captureDelay;
	uint32_t numPairsImagesBeforCalib;
	uint32_t boardWidth;
	uint32_t boardHeigth;
	int verbose;
	enum CameraCalibrationPattern calibrationPattern;
	float aspectRatio;
	bool assumeZeroTangentialDistortion;
	bool fixPrincipalPointAtCenter;
	float boardSquareSize;
	bool doDisparity;
	float acceptableAvrEpipolarErr;
	float acceptableRMSErr;

};

typedef struct StereoCalibrationSettings_struct *StereoCalibrationSettings;


#endif /* CAMERACALIBRATION_SETTINGS_H_ */
