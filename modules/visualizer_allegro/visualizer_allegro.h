#ifndef VISUALIZER_ALLEGRO_H_
#define VISUALIZER_ALLEGRO_H_

#include "main.h"

#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>

#define VISUALIZER_ALLEGRO_SCREEN_WIDTH 240
#define VISUALIZER_ALLEGRO_SCREEN_HEIGHT 180

#define PIXEL_ZOOM 2

#define MAX_CHANNELS 4

void caerVisualizerAllegro(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame);

#endif /* VISUALIZER_ALLEGRO_H_ */
