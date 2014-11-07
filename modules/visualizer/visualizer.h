#ifndef VISUALIZER_H_
#define VISUALIZER_H_

#include "main.h"
#include "events/polarity.h"
#include "events/frame.h"

void caerVisualizer(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame);

#endif /* VISUALIZER_H_ */
