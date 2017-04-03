/* OpenCV Interface
 *  Author: federico.corradi@inilabs.com
 */
#include "peopleCountingOpenCV.hpp"

#include <string>


void OpenCV::generate(int nIn, int nOut, caerFrameEvent *single_frame, int sizeX, int sizeY) {

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
		}
	}

	//	/* Print text */
	std::string peopleIn = "In:";
	std::string peopleOut = "Out:";
	std::string numIn = std::to_string(nIn);
	std::string numOut = std::to_string(nOut);

	cv::putText(img, peopleIn, cv::Point(30,25), CV_FONT_NORMAL, 0.5, cv::Scalar(255,255,255));
	cv::putText(img, numIn, cv::Point(60,25), CV_FONT_NORMAL, 0.5, cv::Scalar(255,255,255));
	cv::putText(img, peopleOut, cv::Point(30,50), CV_FONT_NORMAL, 0.5, cv::Scalar(255,255,255));
	cv::putText(img, numOut, cv::Point(70,50), CV_FONT_NORMAL, 0.5, cv::Scalar(255,255,255));

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







