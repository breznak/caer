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

typedef struct OpenCV OpenCV;

OpenCV* newOpenCV();

void OpenCV_generate(OpenCV* v, int nIn, int nOut, caerFrameEvent *singleframe, int sizeX, int sizeY);

void deleteOpenCV(OpenCV* v);

#ifdef __cplusplus
}
#endif
#endif
