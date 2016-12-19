#include "calibration.hpp"
#include <fstream>
#include <iostream>

StereoCalibration::StereoCalibration(StereoCalibrationSettings settings) {
	this->settings = settings;

	updateSettings(this->settings);
}

void StereoCalibration::updateSettings(StereoCalibrationSettings settings) {
	this->settings = settings;

	if (settings->useFisheyeModel_cam0) {
		// The fisheye model has its own enum, so overwrite the flags.
		flag = fisheye::CALIB_FIX_SKEW | fisheye::CALIB_RECOMPUTE_EXTRINSIC | fisheye::CALIB_FIX_K2
			| fisheye::CALIB_FIX_K3 | fisheye::CALIB_FIX_K4;
	}
	else {
		flag = CALIB_FIX_K4 | CALIB_FIX_K5;

		if (settings->aspectRatio) {
			flag |= CALIB_FIX_ASPECT_RATIO;
		}

		if (settings->assumeZeroTangentialDistortion) {
			flag |= CALIB_ZERO_TANGENT_DIST;
		}

		if (settings->fixPrincipalPointAtCenter) {
			flag |= CALIB_FIX_PRINCIPAL_POINT;
		}
	}
	if (settings->useFisheyeModel_cam1) {
		// The fisheye model has its own enum, so overwrite the flags.
		flag = fisheye::CALIB_FIX_SKEW | fisheye::CALIB_RECOMPUTE_EXTRINSIC | fisheye::CALIB_FIX_K2
			| fisheye::CALIB_FIX_K3 | fisheye::CALIB_FIX_K4;
	}
	else {
		flag = CALIB_FIX_K4 | CALIB_FIX_K5;

		if (settings->aspectRatio) {
			flag |= CALIB_FIX_ASPECT_RATIO;
		}

		if (settings->assumeZeroTangentialDistortion) {
			flag |= CALIB_ZERO_TANGENT_DIST;
		}

		if (settings->fixPrincipalPointAtCenter) {
			flag |= CALIB_FIX_PRINCIPAL_POINT;
		}
	}

	// Update board size.
	boardSize.width = settings->boardWidth;
	boardSize.height = settings->boardHeigth;

	// Load calibration files keep this in the constructor
	if (loadCalibrationFile(this->settings)) {
		this->calibrationLoaded = true;
	}
	else {
		this->calibrationLoaded = false;
	}

	// Clear current image points.
	imagePoints_cam0.clear();
	imagePoints_cam1.clear();

}

bool StereoCalibration::stereoCalibrate(StereoCalibrationSettings settings) {
	this->settings = settings;

	Mat R, T, E, F;
	vector<vector<Point3f> > objectPoints;
	objectPoints.resize(imagePoints_cam0.size());

	for (int i = 0; i < imagePoints_cam0.size(); i++) {
		for (int j = 0; j < this->settings->boardHeigth; j++)
			for (int k = 0; k < this->settings->boardWidth; k++)
				objectPoints[i].push_back(
					Point3f(k * this->settings->boardSquareSize, j * this->settings->boardSquareSize, 0));
	}

	double rms = cv::stereoCalibrate(objectPoints, imagePoints_cam0, imagePoints_cam1, undistortCameraMatrix_cam0,
		undistortDistCoeffs_cam0, undistortCameraMatrix_cam1, undistortDistCoeffs_cam1, imageSize, R, T, E, F,
		CALIB_FIX_ASPECT_RATIO + CALIB_ZERO_TANGENT_DIST + CALIB_USE_INTRINSIC_GUESS + CALIB_SAME_FOCAL_LENGTH
			+ CALIB_RATIONAL_MODEL + CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
	cout << "done with RMS error=" << rms << endl;

	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0
	double err = 0;
	int npoints = 0;
	vector<Vec3f> lines[2];
	for (int i = 0; i < this->settings->numPairsImagesBeforCalib; i++) {
		int npt = (int) imagePoints_cam0[i].size();
		Mat imgpt[2];

		imgpt[0] = Mat(imagePoints_cam0[i]);
		undistortPoints(imgpt[0], imgpt[0], undistortCameraMatrix_cam0, undistortDistCoeffs_cam0, Mat(),
			undistortCameraMatrix_cam0);
		computeCorrespondEpilines(imgpt[0], 0 + 1, F, lines[0]);

		imgpt[1] = Mat(imagePoints_cam1[i]);
		undistortPoints(imgpt[1], imgpt[1], undistortCameraMatrix_cam1, undistortDistCoeffs_cam1, Mat(),
			undistortCameraMatrix_cam1);
		computeCorrespondEpilines(imgpt[1], 1 + 1, F, lines[1]);

		for (int j = 0; j < npt; j++) {
			double errij = fabs(
				imagePoints_cam0[i][j].x * lines[1][j][0] + imagePoints_cam0[i][j].y * lines[1][j][1] + lines[1][j][2])
				+ fabs(
					imagePoints_cam1[i][j].x * lines[0][j][0] + imagePoints_cam1[i][j].y * lines[0][j][1]
						+ lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	float avreperr = err / npoints;
	cout << "average epipolar err = " << avreperr << endl;

	// save new intrinsic parameters
	FileStorage fs(this->settings->saveFileName_intrinsics, FileStorage::WRITE);
	if (fs.isOpened()) {
		fs << "M1" << undistortCameraMatrix_cam0 << "D1" << undistortDistCoeffs_cam0 << "M2"
			<< undistortCameraMatrix_cam1 << "D2" << undistortDistCoeffs_cam1;
		fs.release();
	}
	else
		cout << "Error: can not save the intrinsic parameters\n";

	Mat R1, R2, P1, P2, Q;
	Rect validRoi[2];

	stereoRectify(undistortCameraMatrix_cam0, undistortDistCoeffs_cam0, undistortCameraMatrix_cam1,
		undistortDistCoeffs_cam1, imageSize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0],
		&validRoi[1]);

	fs.open(this->settings->saveFileName_extrinsics, FileStorage::WRITE);
	if (fs.isOpened()) {
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else
		cout << "Error: can not save the extrinsic parameters\n";

	// calibration has been completed
	this->settings->doCalibration = false;

	if ((rms <= this->settings->acceptableAvrEpipolarErr) && (avreperr <= this->settings->acceptableRMSErr))
		return (true);
	else
		return (false);
}

void StereoCalibration::stereoRectifyHartley() {

}

void StereoCalibration::clearImagePoints() {

	imagePoints_cam0.clear();
	imagePoints_cam1.clear();

}

void StereoCalibration::addStereoCalib(vector<Point2f>*vec1, vector<Point2f>*vec2) {

	imagePoints_cam0.push_back(*vec1);
	imagePoints_cam1.push_back(*vec2);

	return;
}

void * StereoCalibration::findNewPoints(caerFrameEvent frame, int camid) {
	if (frame == NULL || !caerFrameEventIsValid(frame)) {
		return (false);
	}

	// Initialize OpenCV Mat based on caerFrameEvent data directly (no image copy).
	Size frameSize(caerFrameEventGetLengthX(frame), caerFrameEventGetLengthY(frame));
	Mat orig(frameSize, CV_16UC(caerFrameEventGetChannelNumber(frame)), caerFrameEventGetPixelArrayUnsafe(frame));

	imageSize.width = caerFrameEventGetLengthX(frame);
	imageSize.height = caerFrameEventGetLengthY(frame);

	// Create a new Mat that has only 8 bit depth from the original 16 bit one.
	// findCorner functions in OpenCV only support 8 bit depth.
	Mat view;
	orig.convertTo(view, CV_8UC(orig.channels()), 1.0 / 256.0);

	int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;

	if (camid == 0) {
		if (!settings->useFisheyeModel_cam0) {
			// Fast check erroneously fails with high distortions like fisheye lens.
			chessBoardFlags |= CALIB_CB_FAST_CHECK;
		}
	}
	else if (camid == 1) {
		if (!settings->useFisheyeModel_cam1) {
			// Fast check erroneously fails with high distortions like fisheye lens.
			chessBoardFlags |= CALIB_CB_FAST_CHECK;
		}
	}

	// Find feature points on the input image.
	vector<Point2f> *pointBuf = new vector<Point2f>;

	bool found;

	switch (settings->calibrationPattern) {
		case CAMCALIB_CHESSBOARD:
			found = findChessboardCorners(view, boardSize, *pointBuf, chessBoardFlags);
			break;

		case CAMCALIB_CIRCLES_GRID:
			found = findCirclesGrid(view, boardSize, *pointBuf);
			break;

		case CAMCALIB_ASYMMETRIC_CIRCLES_GRID:
			found = findCirclesGrid(view, boardSize, *pointBuf, CALIB_CB_ASYMMETRIC_GRID);
			break;

		default:
			found = false;
			break;
	}

	if (found) {
		// Improve the found corners' coordinate accuracy for chessboard pattern.
		if (settings->calibrationPattern == CAMCALIB_CHESSBOARD) {
			Mat viewGray;

			// Only convert color if not grayscale already.
			if (view.channels() == GRAYSCALE) {
				viewGray = view;
			}
			else {
				if (view.channels() == RGB) {
					cvtColor(view, viewGray, COLOR_RGB2GRAY);
				}
				else if (view.channels() == RGBA) {
					cvtColor(view, viewGray, COLOR_RGBA2GRAY);
				}
			}

			cornerSubPix(viewGray, *pointBuf, Size(5, 5), Size(-1, -1),
				TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
		}

		return (pointBuf);
	}

	return (NULL);
}

size_t StereoCalibration::foundPoints(int camid) {
	if (camid == 0)
		return (imagePoints_cam0.size());
	if (camid == 1)
		return (imagePoints_cam1.size());
}

/*bool StereoCalibration::loadStereoCalibration(StereoCalibrationSettings settings) {

 }*/

bool StereoCalibration::loadCalibrationFile(StereoCalibrationSettings settings) {

	// Open file with undistort matrices.
	FileStorage fs(settings->loadFileName_cam0, FileStorage::READ);
	// Check file.
	if (!fs.isOpened()) {
		return (false);
	}
	fs["camera_matrix"] >> undistortCameraMatrix_cam0;
	fs["distortion_coefficients"] >> undistortDistCoeffs_cam0;
	fs["use_fisheye_model"] >> useFisheyeModel_cam0;
	if (!fs["camera_matrix"].empty() && !fs["distortion_coefficients"].empty()) {
		caerLog(CAER_LOG_NOTICE, "StereoCalibration CXX loadCalibrationFile() CAM0:",
			"Camera matrix and distorsion coefficients succesfully loaded");
	}
	else {
		caerLog(CAER_LOG_ERROR, "StereoCalibration CXX loadCalibrationFile() CAM0",
			"Camera matrix and distorsion coefficients not loaded");
	}
	// Close file.
	fs.release();

	FileStorage fs1(settings->loadFileName_cam1, FileStorage::READ);
	// Check file.
	if (!fs1.isOpened()) {
		return (false);
	}
	fs1["camera_matrix"] >> undistortCameraMatrix_cam1;
	fs1["distortion_coefficients"] >> undistortDistCoeffs_cam1;
	fs1["use_fisheye_model"] >> useFisheyeModel_cam1;
	if (!fs1["camera_matrix"].empty() && !fs1["distortion_coefficients"].empty()) {
		caerLog(CAER_LOG_NOTICE, "StereoCalibration CXX loadCalibrationFile() CAM1:",
			"Camera matrix and distorsion coefficients succesfully loaded");
	}
	else {
		caerLog(CAER_LOG_ERROR, "StereoCalibration CXX loadCalibrationFile() CAM1",
			"Camera matrix and distorsion coefficients not loaded");
	}
	// Close file.
	fs1.release();

	return (true);
}
