#ifndef IMAGESTREAMERVISUALIZER_H_
#define IMAGESTREAMERVISUALIZER_H_

#include "main.h"

#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>

#define IMAGESTREAMERVISUALIZER_SCREEN_WIDTH 400 
#define IMAGESTREAMERVISUALIZER_SCREEN_HEIGHT 400 

void caerImagestreamerVisualizer(uint16_t moduleID, unsigned char * disp_img, const int disp_img_size, bool * classific_results, int * classific_sizes, int max_img_qty);


#endif /* IMAGESTREAMERVISUALIZER_H_ */
