#ifndef STEREOMATCHING_HPP_
#define STEREOMATCHING_HPP_

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdlib.h>
#include <ctype.h>

#include "matching_settings.h"

#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

class StereoMatching {

public:
	StereoMatching(StereoMatchingSettings settings);
	void updateSettings(StereoMatchingSettings settings);
	bool loadCalibrationFile(StereoMatchingSettings settings);
	bool stereoMatch(StereoMatchingSettings settings, caerFrameEvent vec1, caerFrameEvent vec2);

private:
	Mat M1, D1, M2, D2;
	Mat R, T, R1, P1, R2, P2;
	Mat Q;
	StereoMatchingSettings settings = NULL;

};

#endif /* STEREOMATCHING_HPP_ */
