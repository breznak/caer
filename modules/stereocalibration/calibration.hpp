#ifndef StereoCalibration_HPP_
#define StereoCalibration_HPP_

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdlib.h>
#include <ctype.h>

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

class StereoCalibration {

public:
	StereoCalibration(StereoCalibrationSettings settings);
	void * findNewPoints(caerFrameEvent frame, int camid);
	size_t foundPoints(int camid);bool multicalib(caerFrameEvent frame0,
			caerFrameEvent frame1);bool loadCalibrationFile(
			StereoCalibrationSettings settings);bool stereoCalibrate(
			StereoCalibrationSettings settings);
	void addStereoCalib(vector<Point2f> *vec1, vector<Point2f> *vec2);
	//bool loadStereoCalibration();
	//bool loadCalibrationFile(StereoCalibrationSettings settings);
	void clearImagePoints();
	void stereoRectifyHartley();
	void updateSettings(StereoCalibrationSettings settings);
	Point3f getCamerasLocation(Mat Rvec, Mat Tvec);

private:
	StereoCalibrationSettings settings = NULL;
	Size boardSize;
	int flag = 0; //fish eye model
	vector<vector<Point2f> > imagePoints_cam0;
	vector<vector<Point2f> > imagePoints_cam1;
	Mat undistortCameraMatrix_cam0;
	Mat undistortCameraMatrix_cam1;bool useFisheyeModel_cam0;bool useFisheyeModel_cam1;
	Mat undistortDistCoeffs_cam0;
	Mat undistortDistCoeffs_cam1;bool calibrationLoaded = false;
	Size imageSize;

};

#endif /* StereoCalibration_HPP_ */
