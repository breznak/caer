#ifndef CAMERACALIBRATION_H_
#define CAMERACALIBRATION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>

enum CameraCalibrationPattern { CALIB_CHESSBOARD, CALIB_CIRCLES_GRID, CALIB_ASYMMETRIC_CIRCLES_GRID };

struct CameraCalibration_state {
	bool doCalibration;
	int saveFileFd;
	enum CameraCalibrationPattern calibrationPattern;
	int boardWidth;
	int boardHeigth;
	float boardSquareSize;
	float aspectRatio;
	bool assumeZeroTangentialDistortion;
	bool fixPrincipalPointAtCenter;
	bool useFisheyeModel;
	bool doUndistortion;
	int loadFileFd;
};

typedef struct CameraCalibration_state *CameraCalibrationState;

void caerCameraCalibration(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame);

#ifdef __cplusplus
}
#endif

#endif /* CAMERACALIBRATION_H_ */
