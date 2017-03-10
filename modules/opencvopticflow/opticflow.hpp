#ifndef OPTICFLOW_HPP_
#define OPTICFLOW_HPP_

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include "opticflow_settings.h"

#include <libcaer/events/frame.h>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class OpticFlow {

public:
	OpticFlow(OpticFlowSettings settings);
	void updateSettings(void);
	bool doOpticFlow(caerFrameEvent * frame, caerFrameEvent * frameInput, int sizeX, int sizeY);


private:
	OpticFlowSettings settings = NULL;
	Size flowSize;
	Mat smallImage, oldSmallImage, diff, dx, dy;

	//Downsize the input frames by this ratio.
	//Can be set to 1, 2, 4...
	int ratio = 1;

	bool init = false;
	Mat image;
	int iRows, iCols;

	void MyLine( Mat img, Point start, Point end );

};

#endif /* OPTICFLOW_HPP_ */
