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

typedef struct OpenCV OpenCV;

OpenCV* newOpenCV();

void OpenCV_generate(OpenCV* v, activityLevel status, int activeNum, caerFrameEvent *singleframe, int sizeX, int sizeY, bool showEvents);

void deleteOpenCV(OpenCV* v);

#ifdef __cplusplus
}
#endif
#endif
