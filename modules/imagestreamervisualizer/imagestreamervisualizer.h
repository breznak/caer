#ifndef IMAGESTREAMERVISUALIZER_H_
#define IMAGESTREAMERVISUALIZER_H_

#include "main.h"

#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>

#define IMAGESTREAMERVISUALIZER_SCREEN_WIDTH 400 
#define IMAGESTREAMERVISUALIZER_SCREEN_HEIGHT 400 

#define PIXEL_ZOOM 1 
#define DIRECTORY_IMG "/tmp/"
#define SIZE_IMG_W 128 
#define SIZE_IMG_H 128

void caerImagestreamerVisualizer(uint16_t moduleID, caerPolarityEventPacket polarity, char ** file_string,
	caerFrameEventPacket frame, char ** file_string_frame);

#endif /* IMAGESTREAMERVISUALIZER_H_ */
