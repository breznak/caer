#ifndef IMAGESTREAMERVISUALIZER_H_
#define IMAGESTREAMERVISUALIZER_H_

#include "main.h"

#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>

#define IMAGESTREAMERVISUALIZER_SCREEN_WIDTH 400 
#define IMAGESTREAMERVISUALIZER_SCREEN_HEIGHT 400 

#define PIXEL_ZOOM 1 
#define TEXT_SPACING 20 // in pixels
#define SYSTEM_TIMEOUT 10 // in seconds
#define DIRECTORY_IMG "/home/hyo/caffe/examples/images/retina/"
#define SIZE_IMG 128 

void caerImagestreamerVisualizer(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame);

#endif /* IMAGESTREAMERVISUALIZER_H_ */
