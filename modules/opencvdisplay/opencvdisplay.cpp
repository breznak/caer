/* OpenCV Interface
 *  Author: federico.corradi@inilabs.com
 */
#include "opencvdisplay.hpp"
#include <string>


void MyOpenCV::generate(AResults res, caerFrameEvent *single_frame) {


	//printf("string %s int %d\n", res->stringValue, res->activityValue);

	// Loading img
	cv::Mat img = cv::Mat(FRAMESIZE, FRAMESIZE, CV_8UC1);
	for(size_t i=0; i<FRAMESIZE; i++)
	{
		for(size_t j=0; j<FRAMESIZE; j++)
		{
			img.data[i*FRAMESIZE+j] = caerFrameEventGetPixel(*single_frame,i,j);
		}
	}

	/* Print text */
	cv::putText(img, res->stringValue, cv::Point(35,30), CV_FONT_NORMAL, 0.8, cv::Scalar(255));
	std::string s = std::to_string(res->activityValue);
	cv::putText(img, s, cv::Point(75,90), CV_FONT_NORMAL, 0.8, cv::Scalar(255));


	for (int j = 0; j < FRAMESIZE; j++) {
	    for (int i = 0; i < FRAMESIZE; i++) {
	        uchar& uxy = img.at<uchar>(i, j);
	        int color = (int) uxy;
	        caerFrameEventSetPixel(*single_frame, j, i, color*256);
	    }
	}


}







