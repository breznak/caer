#include "calibration.hpp"

PoseCalibration::PoseCalibration(PoseCalibrationSettings settings) {
    this->settings = settings;

    updateSettings(this->settings);
}

void PoseCalibration::updateSettings(PoseCalibrationSettings settings) {
    this->settings = settings;
    
    // Load calibration files keep this in the constructor
    if(loadCalibrationFile(this->settings)){
        this->calibrationLoaded = true;
    }else{
        this->calibrationLoaded = false;
    }

}

bool PoseCalibration::findMarkers(caerFrameEvent frame) {
    if (frame == NULL || !caerFrameEventIsValid(frame) || this->calibrationLoaded == false) {
        if(this->calibrationLoaded == false){
            caerLog(CAER_LOG_NOTICE, "Pose Calibration findMarkers", "Camera matrix and distorsion coefficients not loaded, exit from filter!");
        }
	    return (false);
    }

    // Initialize OpenCV Mat based on caerFrameEvent data directly (no image copy).
    Size frameSize(caerFrameEventGetLengthX(frame), caerFrameEventGetLengthY(frame));
    Mat orig(frameSize, CV_16UC(caerFrameEventGetChannelNumber(frame)), caerFrameEventGetPixelArrayUnsafe(frame));

    // Create a new Mat that has only 8 bit depth from the original 16 bit one.
    // findCorner functions in OpenCV only support 8 bit depth.
    Mat view;
    orig.convertTo(view, CV_8UC(orig.channels()), 1.0 / 256.0);
    
    // marker detection with aruco markers integration in opencv
    aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);
    
    vector<int> ids; 
    vector<std::vector<cv::Point2f> > corners, rejected; 
    
    aruco::DetectorParameters *detectorParams = new aruco::DetectorParameters();
    detectorParams->doCornerRefinement = true; // do corner refinement in markers

    // detect
    aruco::detectMarkers(view, dictionary, corners, ids, *detectorParams, rejected);
    
    // if at least one marker has been detected
    if (ids.size() > 0){ 
        aruco::drawDetectedMarkers(view, corners, ids);
    }else{
        return (false);
    } 
       
    // from camera calibration 
    double fx, fy, m, distance, avr_size, x, object_image_sensor_mm;
    fx = undistortCameraMatrix.at<double>(0,0);
    fy = undistortCameraMatrix.at<double>(1,1);
    // from zhang method we ervecs tvecs to 3d pointsstimate pixels per mm (focal lenght))
    m = ( (fx+fy)/2.0 ) / focal_lenght_mm ;
    // estimate markers pose
    if( corners.size() > 0){
        Mat rvecs, tvecs;
        aruco::estimatePoseSingleMarkers(corners, 0.05, undistortCameraMatrix, undistortDistCoeffs, rvecs, tvecs); 
        // rvecs tvecs tell us where the marker is in camera coordinates [R T; 0 1] (for homogenous coordinates)
        // the inverse is [R^t -R^t*T; 0 1]
        for(unsigned int k=0; k<corners.size(); k++){    
            // project 3d axis to 2d image plane using rvecs and tvecs
            float length = 0.07;
            vector< Point3f > axisPoints;
            axisPoints.push_back(Point3f(0, 0, 0));
            axisPoints.push_back(Point3f(length, 0, 0));
            axisPoints.push_back(Point3f(0, length, 0));
            axisPoints.push_back(Point3f(0, 0, length));
            vector< Point2f > imagePoints;
            projectPoints(axisPoints, rvecs.row(k), tvecs.row(k), undistortCameraMatrix, undistortDistCoeffs, imagePoints);
            // draw axis lines
            line(view, imagePoints[0], imagePoints[1], Scalar(0, 0, 255), 3);
            line(view, imagePoints[0], imagePoints[2], Scalar(0, 255, 0), 3);
            line(view, imagePoints[0], imagePoints[3], Scalar(255, 0, 0), 3);             
            //estimate distance in mm
            cv::RotatedRect box = cv::minAreaRect(cv::Mat(corners[k]));
            cv::Point2f p = box.size;
            avr_size = ( p.x + p.y ) / 2.0; //in pixels
            // convert px/mm in the lower resolution
            // camera_max_resolution/m = camera_y_resolution/x
            x = (camera_y_resolution*m)/camera_y_resolution;
            object_image_sensor_mm = avr_size / x ;
            // calculate distance from object
            // distance_mm = object_real_world_mm * focal-length_mm / object_image_sensor_mm
            distance = object_real_world_mm * focal_lenght_mm / object_image_sensor_mm;
            caerLog(CAER_LOG_NOTICE, "PoseCalibration CXX findMarkers()", "\n distance corner %d for maker %d is at a distance %.2f mm ", k, ids[k], distance);
        }       
    }
    //place back the markers in the frame
    view.convertTo(orig, CV_16UC(orig.channels()), 256.0);

    return(true);

}

bool PoseCalibration::loadCalibrationFile(PoseCalibrationSettings settings) {

	// Open file with undistort matrices.
	FileStorage fs(settings->loadFileName, FileStorage::READ);

	// Check file.
	if (!fs.isOpened()) {
		return (false);
	}

	fs["camera_matrix"] >> undistortCameraMatrix;
	fs["distortion_coefficients"] >> undistortDistCoeffs;
	fs["use_fisheye_model"] >> useFisheyeModel;

	if (!fs["camera_matrix"].empty() && !fs["distortion_coefficients"].empty()) 
	{
		caerLog(CAER_LOG_NOTICE, "PoseCalibration CXX loadCalibrationFile()", "Camera matrix and distorsion coefficients succesfully loaded");
	}else{
		caerLog(CAER_LOG_ERROR, "PoseCalibration CXX loadCalibrationFile()", "Camera matrix and distorsion coefficients not loaded");    
	}    
		
	// Close file.
	fs.release();

	return (true);
}

