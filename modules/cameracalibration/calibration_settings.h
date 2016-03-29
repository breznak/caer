#ifndef CALIBRATION_SETTINGS_H_
#define CALIBRATION_SETTINGS_H_

enum CameraCalibrationPattern { CAMCALIB_CHESSBOARD, CAMCALIB_CIRCLES_GRID, CAMCALIB_ASYMMETRIC_CIRCLES_GRID };

struct CameraCalibrationSettings_struct {
	bool doCalibration;
	char *saveFileName;
	enum CameraCalibrationPattern calibrationPattern;
	int boardWidth;
	int boardHeigth;
	float boardSquareSize;
	float aspectRatio;
	bool assumeZeroTangentialDistortion;
	bool fixPrincipalPointAtCenter;
	bool useFisheyeModel;
	bool doUndistortion;
	char *loadFileName;
	int imageWidth;
	int imageHeigth;
	int minNumberOfPoints;
	float maxTotalError;
};

typedef struct CameraCalibrationSettings_struct *CameraCalibrationSettings;

#endif /* CALIBRATION_SETTINGS_H_ */
