/* C wrapper to caffe interface
 *  Author: federico.corradi@inilabs.com
 */
#include "wrapper.h"

#include "peopleCountingOpenCV.hpp"

extern "C" {

OpenCV* newOpenCV() {
	return new OpenCV();
}

void OpenCV_generate(OpenCV* v, int nIn, int nOut, caerFrameEvent *singleframe, int sizeX, int sizeY){
	v->generate(nIn, nOut, singleframe, sizeX, sizeY);
}

void deleteOpenCV(OpenCV* v) {
	delete v;
}

}
