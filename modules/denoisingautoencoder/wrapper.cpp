/* C wrapper to caffe interface
 *  Author: federico.corradi@inilabs.com
 */
#include "wrapper.h"
#include "denoisingautoencoder.hpp"

extern "C" {

MyDenAutoEncoder* newMyDenAutoEncoder() {
	return new MyDenAutoEncoder();
}

void MyDenAutoEncoder_generate(MyDenAutoEncoder* v, caerFrameEvent *singleframein, caerFrameEvent *encoders){
	v->generate(singleframein, encoders);
}

void deleteMyDenAutoEncoder(MyDenAutoEncoder* v) {
	delete v;
}

}
