/* OpenCV Interface
 *  Author: federico.corradi@inilabs.com
 */
#include "activityOpencv.hpp"
#include <string>


void OpenCV::generate(activityLevel status, int activeNum, caerFrameEvent *single_frame, int sizeX, int sizeY, bool showEvents) {

	if (showEvents){
		// Loading img
		cv::Mat img = cv::Mat(sizeY, sizeX, CV_8UC3);
		for(size_t i=0; i<sizeY; i++)
		{
			for(size_t j=0; j<sizeX; j++)
			{
				cv::Vec3b& data = img.at<cv::Vec3b>(i,j);

				data[0] = caerFrameEventGetPixelForChannel(*single_frame,j,i,0);
				data[1] = caerFrameEventGetPixelForChannel(*single_frame,j,i,1);
				data[2] = caerFrameEventGetPixelForChannel(*single_frame,j,i,2);

				//		img.data[i*sizeX] = caerFrameEventGetPixelForChannel(*single_frame,j,i,0);
				//		img.data[i*sizeX+j] = caerFrameEventGetPixelForChannel(*single_frame,j,i,1);
				//		img.data[i*sizeX+j] = caerFrameEventGetPixelForChannel(*single_frame,j,i,2);
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
		cv::putText(img, text, cv::Point(30,25), CV_FONT_NORMAL, 0.6, cv::Scalar(255,255,255));
		std::string s = std::to_string(activeNum);
		cv::putText(img, s, cv::Point(55,80), CV_FONT_NORMAL, 0.6, cv::Scalar(255,255,255));

		// Put img back to the frame
		for (int j = 0; j < sizeX; j++) {
			for (int i = 0; i < sizeY; i++) {
				cv::Vec3b& uxy = img.at<cv::Vec3b>(i, j);
				int red = (int) uxy.val[0];
				int green = (int) uxy.val[1];
				int blue = (int) uxy.val[2];
				caerFrameEventSetPixelForChannel(*single_frame, j, i, 0, red*256);
				caerFrameEventSetPixelForChannel(*single_frame, j, i, 1, green*256);
				caerFrameEventSetPixelForChannel(*single_frame, j, i, 2, blue*256);
			}
		}
	}

	else{
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

		// Put img back to the frame
		for (int j = 0; j < sizeX; j++) {
			for (int i = 0; i < sizeY; i++) {
				uchar& uxy = img.at<uchar>(i, j);
				int color = (int) uxy;
				caerFrameEventSetPixel(*single_frame, j, i, color*256);
			}
		}
	}
}







