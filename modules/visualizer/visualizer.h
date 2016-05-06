#ifndef VISUALIZER_H_
#define VISUALIZER_H_

#include "main.h"

#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>
#include <libcaer/events/imu6.h>

#define VISUALIZER_DEFAULT_ZOOM 2

typedef struct caer_visualizer_state *caerVisualizerState;

// For reuse inside other modules.
caerVisualizerState caerVisualizerInit(int32_t bitmapSizeX, int32_t bitmapSizeY, int32_t zoomFactor, bool doStatistics);
void caerVisualizerUpdate(caerVisualizerState state, caerEventPacketHeader packetHeader);
void caerVisualizerExit(caerVisualizerState state);

//typedef int (*caerVisualizerRenderer)(void *);

void caerVisualizer(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame, caerIMU6EventPacket imu);

void caerVisualizerSystemInit(void);

#endif /* VISUALIZER_H_ */
