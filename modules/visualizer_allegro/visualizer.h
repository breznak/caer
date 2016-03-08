#ifndef VISUALIZER_ALLEGRO_H_
#define VISUALIZER_ALLEGRO_H_

#include "main.h"

#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>
#include <allegro5/allegro.h>
#include <allegro5/allegro_primitives.h>
#include <allegro5/allegro_font.h>
#include <allegro5/allegro_ttf.h>

#include "modules/statistics/statistics.h"

#define MAX_CHANNELS 4

struct caer_visualizer_state {
	ALLEGRO_DISPLAY *displayWindow;
	int32_t displayWindowSizeX;
	int32_t displayWindowSizeY;
	ALLEGRO_BITMAP  *bitmapRenderer;
	int32_t bitmapRendererSizeX;
	int32_t bitmapRendererSizeY;
	struct caer_statistics_state packetStatistics;
	int32_t packetSubsampleRendering;
	int32_t packetSubsampleCount;
};

typedef struct caer_visualizer_state *caerVisualizerState;

// For reuse inside other modules.
bool caerVisualizerInit(caerVisualizerState state);
void caerVisualizerUpdate(caerEventPacketHeader packetHeader, caerVisualizerState state);
void caerVisualizerExit(caerVisualizerState state);

void caerVisualizer(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame);

void caerVisualizerSystemInit(void);

#endif /* VISUALIZER_ALLEGRO_H_ */
