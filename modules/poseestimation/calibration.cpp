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
    }else{
        return false;
    } 
       
    // from camera calibration 
    double fx, fy, m, distance, avr_size, x, object_image_sensor_mm;
    //double cx,cy //planned to be used
    fx = undistortCameraMatrix.at<double>(0,0);
    fy = undistortCameraMatrix.at<double>(1,1);
    //cx = undistortCameraMatrix.at<double>(0,2);
    //cy = undistortCameraMatrix.at<double>(1,2);
    // from zhang method we ervecs tvecs to 3d pointsstimate pixels per mm (focal lenght))
    m = ( (fx+fy)/2.0 ) / focal_lenght_mm ;
    // estimate markers pose
    if( corners.size() > 0){
        Mat rvecs, tvecs;
        aruco::estimatePoseSingleMarkers(corners, 0.05, undistortCameraMatrix, undistortDistCoeffs, rvecs, tvecs); 
        // rvecs tvecs tell us where the marker is in camera coordinates [R T; 0 1] (for homogenous coordinates)
        // the inverse is [R^t -R^t*T; 0 1]
        for(int k=0; k<corners.size(); k++){ 
        
            // draw camera pose in respect to marker
            // aruco::drawAxis(view, undistortCameraMatrix, undistortDistCoeffs, rvecs.row(k), tvecs.row(k), 0.07);                              
            // project camera pose axis points ..
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
            // get maker size in pixels
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
            //cout << endl << "distance to maker id " << corners[k][0] << " is " << distance  << endl << endl;

            // We need inverse of the world->camera transform (camera->world) to calculate
            // camera's location
            Mat R;
            Rodrigues(rvecs.row(k), R); // rvecs to rotation matrix 
            Mat cameraRotationVector;
            Rodrigues(R.t(),cameraRotationVector); 
            Mat cameraTranslationVector;
            multiply(-cameraRotationVector.t(), tvecs.row(k), cameraTranslationVector);
            
            cout << endl;
            cout << "##############$$$$$$$$$$$$################" << endl;
            cout << "Translation: " <<  cameraTranslationVector << endl;
            cout << "Roll: " << cameraRotationVector.row(0) << " Pitch: " << cameraRotationVector.row(1) << " Yaw: " << cameraRotationVector.row(2) << endl;
            cout << "##############$$$$$$$$$$$$################" << endl;   
            cout << endl;
               
            if(k == 0){   
                float length = 0.07;
                vector< Point3f > axisPoints;
                axisPoints.push_back(Point3f(0, 0, 0));
                axisPoints.push_back(Point3f(length, 0, 0));
                axisPoints.push_back(Point3f(0, length, 0));
                axisPoints.push_back(Point3f(0, 0, length));
                vector< Point2f > imagePoints;
                projectPoints(axisPoints, rvecs.row(k), tvecs.row(k), undistortCameraMatrix, undistortDistCoeffs, imagePoints);
                // draw axis lines
                //line(view, imagePoints[0], imagePoints[1], Scalar(0, 255, 255), 3);
                //line(view, imagePoints[0], imagePoints[2], Scalar(0, 255, 255), 3);
                //line(view, imagePoints[0], imagePoints[3], Scalar(0, 255, 255), 3);    
                Mat m(imagePoints); 
                //cerr << m.rows << " " << m.cols << " "<< m.channels() << endl;
                //cerr << m << endl;
               
                //http://stackoverflow.com/questions/12299870/computing-x-y-coordinate-3d-from-image-point
                cv::Mat uvPoint = cv::Mat::ones(3,1,cv::DataType<double>::type); //u,v,1
                uvPoint.at<double>(0,0) = m.at<double>(0,0); 
                uvPoint.at<double>(1,0) = m.at<double>(0,1);
                cv::Mat tempMat, tempMat2;
                double s;
                cv::Mat rotationMatrix(3,3,cv::DataType<double>::type);
                cv::Rodrigues(rvecs.row(k),rotationMatrix);
                tempMat = rotationMatrix.inv() * undistortCameraMatrix.inv() * uvPoint;
                //multiply(rotationMatrix.inv(),tvecs.row(k),tempMat2);
                //tempMat2 = rotationMatrix.inv() * tvecs.row(k);
                //s = 285 + tempMat2.at<double>(2,0); //285 represents the height Zconst
                //s /= tempMat.at<double>(2,0);
                //std::cout << "P = " << rotationMatrix.inv() * (s * undistortCameraMatrix.inv() * uvPoint - tvecs.row(k)) << std::endl;
                
            }
                
        }   
        
    }    

    //place back the markers in the frame
    view.convertTo(orig, CV_16UC(orig.channels()), 256.0);

    return(true);

}


/* convert2dto3dworldunit converts 2d point into a homogenous point
* 
* it gives it a third coordinate equal to 1 
* and then multiply by the inverse of your camera intrinsics matrix.
* Origin and direction will be define the ray in world space corresponding to that image point. 
* Note that here the origin is centered on the camera, you can use your camera pose to transform to a different origin. 
* Distortion coefficients map from your actual camera to the pinhole camera model and should be used 
* at the very beginning to find your actual 2d coordinate. 
* The steps then are 1) Undistort 2d coordinate with distortion coefficients
* 2) Convert to ray (as shown above) 3) Move that ray to whatever coordinate system you like. 
*/
Point3f PoseCalibration::convert2dto3dworldunit(Point2f point_in_image){

    cv::Matx31f hom_pt(point_in_image.x, point_in_image.y, 1);
    cv::Matx31f hom_pt1(1,1,1);
    multiply(undistortCameraMatrix.inv(), hom_pt, hom_pt1); //put in world coordinates
    cv::Point3f origin(0,0,0);
    cv::Point3f direction(hom_pt1(0),hom_pt1(1),hom_pt1(2));
    //To get a unit vector, direction just needs to be normalized
    direction *= 1/cv::norm(direction);

    return direction;
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

