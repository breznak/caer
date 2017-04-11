#ifndef IMAGEGENERATOR_H_
#define IMAGEGENERATOR_H_

#include "main.h"

#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>

#define IMAGEGENERATOR_SCREEN_WIDTH 400
#define IMAGEGENERATOR_SCREEN_HEIGHT 400

#define CAMERA_X 128
#define CAMERA_Y 128

#define PIXEL_ZOOM 1

#define FRAME_IMG_DIRECTORY "/tmp/"
#define CLASSIFY_IMG_DIRECTORY "/tmp/"

//we cut out a quadratic part of the spike image from the rectangular input of the camera
#define SIZE_QUADRATIC_MAP 128

void caerImageGenerator(uint16_t moduleID, caerPolarityEventPacket polarity, int classify_img_size, int * hist_packet, bool *haveimg);
void caerImageGeneratorMakeFrame(uint16_t moduleID, int * hist_packet,  caerFrameEventPacket *imagegeneratorFrame, int size);
void caerImageGeneratorAddText(uint16_t moduleID, int * hist_packet,  caerFrameEventPacket *imagegeneratorFrame, int size);

#endif /* IMAGEGENERATOR_H_ */
