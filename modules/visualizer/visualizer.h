#ifndef VISUALIZER_H_
#define VISUALIZER_H_

#include "main.h"
#include "events/polarity.h"
#include "events/frame.h"

#define VISUALIZER_SCREEN_WIDTH 1024
#define VISUALIZER_SCREEN_HEIGHT 768

void caerVisualizer(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame);

#endif /* VISUALIZER_H_ */
