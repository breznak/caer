#ifndef VISUALIZER_H_
#define VISUALIZER_H_

#include "main.h"

#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>
#include <libcaer/events/imu6.h>

#define VISUALIZER_DEFAULT_ZOOM 2

typedef struct caer_visualizer_state *caerVisualizerState;

typedef void (*caerVisualizerRenderer)(caerVisualizerState state, caerEventPacketHeader packetHeader);

// For reuse inside other modules.
caerVisualizerState caerVisualizerInit(caerVisualizerRenderer renderer, int32_t bitmapSizeX, int32_t bitmapSizeY,
	int32_t zoomFactor, bool doStatistics);
void caerVisualizerUpdate(caerVisualizerState state, caerEventPacketHeader packetHeader);
void caerVisualizerExit(caerVisualizerState state);

void caerVisualizerRendererPolarityEvents(caerVisualizerState state, caerEventPacketHeader polarityEventPacketHeader);
void caerVisualizerRendererFrameEvents(caerVisualizerState state, caerEventPacketHeader frameEventPacketHeader);
void caerVisualizerRendererIMU6Events(caerVisualizerState state, caerEventPacketHeader imu6EventPacketHeader);

void caerVisualizer(uint16_t moduleID, caerVisualizerRenderer renderer, caerEventPacketHeader packetHeader);

void caerVisualizerSystemInit(void);

#endif /* VISUALIZER_H_ */
