/* C wrapper to caffe interface
 *  Author: federico.corradi@inilabs.com
 */
#include "wrapper.h"
#include "opencvdisplay.hpp"

extern "C" {

MyOpenCV* newMyOpenCV() {
	return new MyOpenCV();
}

void MyOpenCV_generate(MyOpenCV* v, AResults displayInfo, caerFrameEvent *singleframe){
	v->generate(displayInfo, singleframe);
}

void deleteMyOpenCV(MyOpenCV* v) {
	delete v;
}

}
