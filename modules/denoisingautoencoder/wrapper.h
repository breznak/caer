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

typedef struct MyDenAutoEncoder MyDenAutoEncoder;

MyDenAutoEncoder* newMyDenAutoEncoder();

void MyDenAutoEncoder_generate(MyDenAutoEncoder* v, caerFrameEvent *singleframe, caerFrameEvent *encoders);

void deleteMyDenAutoEncoder(MyDenAutoEncoder* v);

#ifdef __cplusplus
}
#endif
#endif
