#include "calibration.hpp"

PoseCalibration::PoseCalibration(PoseCalibrationSettings settings) {
	this->settings = settings;

    updateSettings(this->settings);
}

void PoseCalibration::updateSettings(PoseCalibrationSettings settings) {
    this->settings = settings;
    
    // Load calibration files keep this in the constructor
    loadCalibrationFile(this->settings);

}

bool PoseCalibration::findMarkers(caerFrameEvent frame) {
    if (frame == NULL || !caerFrameEventIsValid(frame)) {
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
    cv::Ptr<aruco::DetectorParameters> parameters;
    cv::Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);
    
    vector<int> ids; 
    vector<std::vector<cv::Point2f> > corners, rejected; 
    
    cv::Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    detectorParams->doCornerRefinement = true; // do corner refinement in markers

    // detect
    aruco::detectMarkers(view, dictionary, corners, ids, detectorParams, rejected);
    
    // if at least one marker has been detected
    if (ids.size() > 0){ 
        aruco::drawDetectedMarkers(view, corners, ids);
    } 
       
    // estimate markers' pose
    if( corners.size() > 0){
        Mat rvecs, tvecs;
        aruco::estimatePoseSingleMarkers(corners, 0.05, undistortCameraMatrix, undistortDistCoeffs, rvecs, tvecs); 
        
        for(int i=0; i<corners.size(); i++){
            aruco::drawAxis(view, undistortCameraMatrix, undistortDistCoeffs, rvecs.row(i), tvecs.row(i), 0.07); 
            //cout<< rvecs.row(i) <<endl;
            //cout<< tvecs.row(i) <<endl;
            
            // We need inverse of the world->camera transform (camera->world) to calculate
            // camera's location
            Mat R;
            Rodrigues(rvecs.row(i), R);
            Mat cameraRotationVector;
            Rodrigues(R.t(),cameraRotationVector);
            Mat cameraTranslationVector;
            multiply(-cameraRotationVector.t(), tvecs.row(i), cameraTranslationVector);
            cout << endl;
            cout << "##############$$$$$$$$$$$$################" << endl;
            cout << "Camera position: " <<  cameraTranslationVector << endl;
            cout << "Camera pose: " << cameraRotationVector << endl;
            cout << "##############$$$$$$$$$$$$################" << endl;   
            cout << endl;
                
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



