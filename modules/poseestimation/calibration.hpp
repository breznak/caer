#ifndef POSEESTIMATION_HPP_
#define POSEESTIMATION_HPP_

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include "calibration_settings.h"

#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

class PoseCalibration {

public:
	PoseCalibration(PoseCalibrationSettings settings);
	bool findMarkers(caerFrameEvent frame);
	bool loadCalibrationFile(PoseCalibrationSettings settings);
	void updateSettings(PoseCalibrationSettings settings);
	Point3f convert2dto3dworldunit(Point2f point_in_image);
	
private:
	PoseCalibrationSettings settings = NULL;
	Mat undistortCameraMatrix;
	bool useFisheyeModel;
	Mat undistortDistCoeffs;

};

#endif /* CALIBRATION_HPP_ */
