#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include "cameracalibration.h"

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

class Calibration {

public:
	Calibration(CameraCalibrationState state);
	void updateSettings(void);
	bool findNewPoints(caerFrameEvent frame);
	bool runCalibrationAndSave(void);

	bool loadUndistortMatrices(void);
	void undistortEvent(caerPolarityEvent polarity);
	void undistortFrame(caerFrameEvent frame);

private:
	CameraCalibrationState settings;
	int flag;
	Size boardSize;

	vector<vector<Point2f> > imagePoints;
	Mat cameraMatrix;
	Mat distCoeffs;

	Mat undistortRemap1;
	Mat undistortRemap2;

	double computeReprojectionErrors(const vector<vector<Point3f> >& objectPoints,
		const vector<vector<Point2f> >& imagePoints, const vector<Mat>& rvecs, const vector<Mat>& tvecs,
		const Mat& cameraMatrix, const Mat& distCoeffs, vector<float>& perViewErrors, bool fisheye);
	void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
		enum CameraCalibrationPattern patternType);
	bool runCalibration(Size& imageSize,
		Mat& cameraMatrix, Mat& distCoeffs, vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs,
		vector<Mat>& tvecs, vector<float>& reprojErrs, double& totalAvgErr);
	void saveCameraParams(Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
		const vector<Mat>& rvecs, const vector<Mat>& tvecs, const vector<float>& reprojErrs, double totalAvgErr);

};
