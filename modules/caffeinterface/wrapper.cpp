/* C wrapper to caffe interface
 *  Author: federico.corradi@inilabs.com
 */
#include "classify.hpp"
#include "wrapper.h"

extern "C" {

MyCaffe* newMyCaffe() {
	return new MyCaffe();
}

void MyCaffe_file_set(MyCaffe* v, int * i, int size, double *b, double thr,
		bool printoutputs, caerFrameEvent *single_frame,
		bool showactivations) {
	v->file_set(i, size, b, thr, printoutputs, single_frame, showactivations);
}

void MyCaffe_init_network(MyCaffe *v) {
	return v->init_network();
}

void deleteMyCaffe(MyCaffe* v) {
	delete v;
}

}
