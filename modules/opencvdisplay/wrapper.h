/* C wrapper to caffe interface
 *  Author: federico.corradi@gmail.com
 */
#ifndef __WRAPPER_H
#define __WRAPPER_H
#include <stdint.h>
#include <libcaer/events/frame.h>
#include "modules/activityindicator/activityindicator.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct MyOpenCV MyOpenCV;

MyOpenCV* newMyOpenCV();

void MyOpenCV_generate(MyOpenCV* v, AResults displayInfo, caerFrameEvent *singleframe);

void deleteMyOpenCV(MyOpenCV* v);

#ifdef __cplusplus
}
#endif
#endif
