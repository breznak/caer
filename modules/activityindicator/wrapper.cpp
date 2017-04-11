/* C wrapper to caffe interface
 *  Author: federico.corradi@inilabs.com
 */
#include "wrapper.h"
#include "activityOpencv.hpp"

extern "C" {

OpenCV* newOpenCV() {
	return new OpenCV();
}

void OpenCV_generate(OpenCV* v, activityLevel status, int activeNum, caerFrameEvent *singleframe, int sizeX, int sizeY, bool showEvents){
	v->generate(status, activeNum, singleframe, sizeX, sizeY, showEvents);
}

void deleteOpenCV(OpenCV* v) {
	delete v;
}

}
