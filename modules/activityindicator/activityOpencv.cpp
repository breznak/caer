/* OpenCV Interface
 *  Author: federico.corradi@inilabs.com
 */
#include "activityOpencv.hpp"
#include <string>


void OpenCV::generate(activityLevel status, int activeNum, caerFrameEvent *single_frame, int sizeX, int sizeY) {


	//printf("string %s int %d\n", status->stringValue, status->activityValue);
	//printf("sizeX %d sizeY %d\n", sizeX, sizeY);

	// Loading img
	cv::Mat img = cv::Mat(sizeY, sizeX, CV_8UC1);
	for(size_t i=0; i<sizeY; i++)
	{
		for(size_t j=0; j<sizeX; j++)
		{
			img.data[i*sizeX+j] = caerFrameEventGetPixel(*single_frame,j,i);
		}
	}

//	/* Print text */
	std::string text;
	if(status == Verylow){
		text = "Verylow";
	}
	if(status == low){
		text = "Low";
	}
	if(status == median){
		text = "Median";
	}
	if(status == high){
		text = "High";
	}
	cv::putText(img, text, cv::Point(30,25), CV_FONT_NORMAL, 0.6, cv::Scalar(255));
	std::string s = std::to_string(activeNum);
	cv::putText(img, s, cv::Point(55,80), CV_FONT_NORMAL, 0.6, cv::Scalar(255));


	for (int j = 0; j < sizeX; j++) {
	    for (int i = 0; i < sizeY; i++) {
	        uchar& uxy = img.at<uchar>(i, j);
	        int color = (int) uxy;
	        caerFrameEventSetPixel(*single_frame, j, i, color*256);
	    }
	}


}







