#include "stereomatching.hpp"
#include <fstream>
#include <iostream>

StereoMatching::StereoMatching(StereoMatchingSettings settings){

	updateSettings(this->settings);

}

bool StereoMatching::stereoMatch(StereoMatchingSettings settings, caerFrameEvent vec1, caerFrameEvent vec2) {
	this->settings = settings;

	// Initialize OpenCV Mat based on caerFrameEvent data directly (no image copy).
	Size frameSize_cam0(caerFrameEventGetLengthX(vec1), caerFrameEventGetLengthY(vec1));
	Mat Image_cam0(frameSize_cam0, CV_16UC(caerFrameEventGetChannelNumber(vec1)), caerFrameEventGetPixelArrayUnsafe(vec1));

	Size frameSize_cam1(caerFrameEventGetLengthX(vec2), caerFrameEventGetLengthY(vec2));
	Mat Image_cam1(frameSize_cam1, CV_16UC(caerFrameEventGetChannelNumber(vec2)), caerFrameEventGetPixelArrayUnsafe(vec2));

    cv::stereoRectify( M1, D1, M2, D2, frameSize_cam0, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, frameSize_cam0);

    /*Mat map11, map12, map21, map22;
    cv::initUndistortRectifyMap(M1, D1, R1, P1, frameSize_cam0, CV_16SC2, map11, map12);
    cv::initUndistortRectifyMap(M2, D2, R2, P2, frameSize_cam1, CV_16SC2, map21, map22);

    Mat img1r, img2r;
	cv::remap(Image_cam0, img1r, map11, map12, INTER_LINEAR);
	cv::remap(Image_cam1, img2r, map21, map22, INTER_LINEAR);

	Image_cam0 = img1r;
	Image_cam1 = img2r;*/

}

void StereoMatching::updateSettings(StereoMatchingSettings settings) {
	this->settings = settings;

}


bool StereoMatching::loadCalibrationFile(StereoMatchingSettings settings) {

	// Open file with undistort matrices.
	FileStorage fs(settings->loadFileName_intrinsic, FileStorage::READ);
	// Check file.
	if (!fs.isOpened()) {
		return (false);
	}

	fs["M1"] >> M1;
	fs["D1"] >> D1;
	fs["M2"] >> M2;
	fs["D2"] >> D2;

	// Close file.
	fs.release();

	FileStorage fs1(settings->loadFileName_extrinsic, FileStorage::READ);
	// Check file.
	if (!fs1.isOpened()) {
		return (false);
	}
    fs1["R"] >> R;
    fs1["T"] >> T;

	fs1.release();

	return (true);
}
