#include "calibration.hpp"

PoseCalibration::PoseCalibration(PoseCalibrationSettings settings) {
	//this->settings = settings;

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

    //init marker aruco
    MarkerDetector MDetector;
    vector<Marker> Markers;

    //Ok, let's detect
    MDetector.detect(view,Markers);
    //for each marker, draw info and its boundaries in the image
    for (unsigned int i=0;i<Markers.size();i++) {
        cout<<Markers[i]<<endl;
        Markers[i].draw(view,Scalar(0,0,255),2);
        cout<< "Marker found"<<endl;
    }
    
    view.convertTo(orig, CV_16UC(orig.channels()), 256.0);

    return(1);

}





