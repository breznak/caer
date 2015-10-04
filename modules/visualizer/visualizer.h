#ifndef VISUALIZER_H_
#define VISUALIZER_H_

#include "main.h"

#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>

#define VISUALIZER_SCREEN_WIDTH 1024
#define VISUALIZER_SCREEN_HEIGHT 768

#define PIXEL_ZOOM 2

#define MAX_CHANNELS 4

void caerVisualizer(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame);

#endif /* VISUALIZER_H_ */
