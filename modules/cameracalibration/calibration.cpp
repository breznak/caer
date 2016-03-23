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

void init(CameraCalibrationState state) {
	int flag = CALIB_FIX_K4 | CALIB_FIX_K5;

	if (state->fixPrincipalPointAtCenter) {
		flag |= CALIB_FIX_PRINCIPAL_POINT;
	}

	if (state->assumeZeroTangentialDistortion) {
		flag |= CALIB_ZERO_TANGENT_DIST;
	}

	if (state->aspectRatio) {
		flag |= CALIB_FIX_ASPECT_RATIO;
	}

	if (state->useFisheyeModel) {
		// the fisheye model has its own enum, so overwrite the flags
		flag = fisheye::CALIB_FIX_SKEW | fisheye::CALIB_RECOMPUTE_EXTRINSIC | fisheye::CALIB_FIX_K2
			| fisheye::CALIB_FIX_K3 | fisheye::CALIB_FIX_K4;
	}
}

vector<vector<Point2f> > imagePoints;
Mat cameraMatrix, distCoeffs;

void find(CameraCalibrationState state, Mat view) {
	vector<Point2f> pointBuf;

	bool found;

	int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;

	if (!state->useFisheyeModel) {
		// fast check erroneously fails with high distortions like fisheye
		chessBoardFlags |= CALIB_CB_FAST_CHECK;
	}

	Size boardSize;
	boardSize.width = state->boardWidth;
	boardSize.height = state->boardHeigth;

	switch (state->calibrationPattern) // Find feature points on the input format
	{
		case CALIB_CHESSBOARD:
			found = findChessboardCorners(view, boardSize, pointBuf, chessBoardFlags);
			break;
		case CALIB_CIRCLES_GRID:
			found = findCirclesGrid(view, boardSize, pointBuf);
			break;
		case CALIB_ASYMMETRIC_CIRCLES_GRID:
			found = findCirclesGrid(view, boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID);
			break;
		default:
			found = false;
			break;
	}

	if (found) {
		// improve the found corners' coordinate accuracy for chessboard
		if (state->calibrationPattern == CALIB_CHESSBOARD) {
			Mat viewGray;
			cvtColor(view, viewGray, COLOR_BGR2GRAY);
			cornerSubPix(viewGray, pointBuf, Size(11, 11), Size(-1, -1),
				TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
		}

		imagePoints.push_back(pointBuf);

		// Draw the corners.
		drawChessboardCorners(view, boardSize, Mat(pointBuf), found);
	}
}

void undistort(CameraCalibrationState state, Mat view) {
	if (state->doUndistortion) {
		Mat temp = view.clone();
		undistort(temp, view, cameraMatrix, distCoeffs);
	}
}

static double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                         const vector<vector<Point2f> >& imagePoints,
                                         const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                         const Mat& cameraMatrix , const Mat& distCoeffs,
                                         vector<float>& perViewErrors, bool fisheye)
{
    vector<Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for(size_t i = 0; i < objectPoints.size(); ++i )
    {
        if (fisheye)
        {
            fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix,
                                   distCoeffs);
        }
        else
        {
            projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
        }
        err = norm(imagePoints[i], imagePoints2, NORM_L2);

        size_t n = objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return (std::sqrt(totalErr/totalPoints));
}
//! [compute_errors]
//! [board_corners]
static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
                                     enum CameraCalibrationPattern patternType)
{
    corners.clear();

    switch(patternType)
    {
    case CALIB_CHESSBOARD:
    case CALIB_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; ++i )
            for( int j = 0; j < boardSize.width; ++j )
                corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
        break;

    case CALIB_ASYMMETRIC_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f((2*j + i % 2)*squareSize, i*squareSize, 0));
        break;
    default:
        break;
    }
}
//! [board_corners]
static bool runCalibration( CameraCalibrationState state, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                            vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                            vector<float>& reprojErrs,  double& totalAvgErr)
{
    //! [fixed_aspect]
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if(state->aspectRatio)
        cameraMatrix.at<double>(0,0) = state->aspectRatio;
    //! [fixed_aspect]
    if (state->useFisheyeModel) {
        distCoeffs = Mat::zeros(4, 1, CV_64F);
    } else {
        distCoeffs = Mat::zeros(8, 1, CV_64F);
    }

    Size boardSize;
    	boardSize.width = state->boardWidth;
    	boardSize.height = state->boardHeigth;

    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(boardSize, state->boardSquareSize, objectPoints[0], state->calibrationPattern);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    double rms;

    if (state->useFisheyeModel) {
        Mat _rvecs, _tvecs;
        rms = fisheye::calibrate(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, _rvecs,
                                 _tvecs, s.flag);

        rvecs.reserve(_rvecs.rows);
        tvecs.reserve(_tvecs.rows);
        for(int i = 0; i < int(objectPoints.size()); i++){
            rvecs.push_back(_rvecs.row(i));
            tvecs.push_back(_tvecs.row(i));
        }
    } else {
        rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs,
                              s.flag);
    }

    cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix,
                                            distCoeffs, reprojErrs, state->useFisheyeModel);

    return ok;
}

// Print camera parameters to the output file
static void saveCameraParams( CameraCalibrationState state, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                              const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                              const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
                              double totalAvgErr )
{
    FileStorage fs( s.outputFileName, FileStorage::WRITE );

    time_t tm;
    time( &tm );
    struct tm *t2 = localtime( &tm );
    char buf[1024];
    strftime( buf, sizeof(buf), "%c", t2 );

    fs << "calibration_time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nr_of_frames" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << s.boardSize.width;
    fs << "board_height" << s.boardSize.height;
    fs << "square_size" << s.squareSize;

    if( s.flag & CALIB_FIX_ASPECT_RATIO )
        fs << "fix_aspect_ratio" << s.aspectRatio;

    if (s.flag)
    {
        if (s.useFisheye)
        {
            sprintf(buf, "flags:%s%s%s%s%s%s",
                     s.flag & fisheye::CALIB_FIX_SKEW ? " +fix_skew" : "",
                     s.flag & fisheye::CALIB_FIX_K1 ? " +fix_k1" : "",
                     s.flag & fisheye::CALIB_FIX_K2 ? " +fix_k2" : "",
                     s.flag & fisheye::CALIB_FIX_K3 ? " +fix_k3" : "",
                     s.flag & fisheye::CALIB_FIX_K4 ? " +fix_k4" : "",
                     s.flag & fisheye::CALIB_RECOMPUTE_EXTRINSIC ? " +recompute_extrinsic" : "");
        }
        else
        {
            sprintf(buf, "flags:%s%s%s%s",
                     s.flag & CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "",
                     s.flag & CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
                     s.flag & CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "",
                     s.flag & CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "");
        }
        cvWriteComment(*fs, buf, 0);
    }

    fs << "flags" << s.flag;

    fs << "fisheye_model" << s.useFisheye;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;
    if (s.writeExtrinsics && !reprojErrs.empty())
        fs << "per_view_reprojection_errors" << Mat(reprojErrs);

    if(s.writeExtrinsics && !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for( size_t i = 0; i < rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(int(i), int(i+1)), Range(0,3));
            Mat t = bigmat(Range(int(i), int(i+1)), Range(3,6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        //cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "extrinsic_parameters" << bigmat;
    }

    if(s.writePoints && !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for( size_t i = 0; i < imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }
}

//! [run_and_save]
bool runCalibrationAndSave(Settings& s, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints)
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs, reprojErrs,
                             totalAvgErr);
    cout << (ok ? "Calibration succeeded" : "Calibration failed")
         << ". avg re projection error = " << totalAvgErr << endl;

    if (ok)
        saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs, imagePoints,
                         totalAvgErr);
    return ok;
}
