#ifndef CAMERACALIBRATION_H_
#define CAMERACALIBRATION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>

enum CameraCalibrationPattern { CAMCALIB_CHESSBOARD, CAMCALIB_CIRCLES_GRID, CAMCALIB_ASYMMETRIC_CIRCLES_GRID };

struct CameraCalibration_state {
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
	// struct Calibration *cpp_class; // Pointer to cpp_class_object
};

typedef struct CameraCalibration_state *CameraCalibrationState;

void caerCameraCalibration(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame);

#ifdef __cplusplus
}
#endif

#endif /* CAMERACALIBRATION_H_ */
