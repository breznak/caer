/* C wrapper to caffe interface
 *  Author: federico.corradi@gmail.com
 */
#ifndef __WRAPPER_H
#define __WRAPPER_H
#include <stdint.h>
#include <libcaer/events/frame.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct MyClass MyClass;

MyClass* newMyClass();

void MyClass_file_set(MyClass* v, char * i, double *b, double thr, bool printoutputs, caerFrameEvent *single_frame,
	bool showactivations);

char * MyClass_file_get(MyClass* v);

void MyClass_init_network(MyClass *v);

//void MyClass_Classifier(MyClass *v);

void deleteMyClass(MyClass* v);

const char * caerCaffeWrapper(uint16_t moduleID, char ** file_string, double *classificationResults, int max_img_qty,
	caerFrameEventPacket *networkActivity);

#ifdef __cplusplus
}
#endif
#endif
