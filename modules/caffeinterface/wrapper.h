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

typedef struct MyCaffe MyCaffe;

MyCaffe* newMyCaffe();

void MyCaffe_file_set(MyCaffe* v, char * i, double *b, double thr, bool printoutputs, caerFrameEvent *single_frame,
	bool showactivations);

char * MyCaffe_file_get(MyCaffe* v);

void MyCaffe_init_network(MyCaffe *v);

//void MyCaffe_Classifier(MyCaffe *v);

void deleteMyCaffe(MyCaffe* v);

const char * caerCaffeWrapper(uint16_t moduleID, char ** file_string, double *classificationResults, int max_img_qty,
	caerFrameEventPacket *networkActivity);

#ifdef __cplusplus
}
#endif
#endif
