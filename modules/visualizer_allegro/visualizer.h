#ifndef VISUALIZER_ALLEGRO_H_
#define VISUALIZER_ALLEGRO_H_

#include "main.h"

#include <stdatomic.h>
#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>
#include <allegro5/allegro.h>
#include <allegro5/allegro_primitives.h>
#include <allegro5/allegro_font.h>
#include <allegro5/allegro_ttf.h>

#include "modules/statistics/statistics.h"
#include "ext/c11threads_posix.h"

#define VISUALIZER_DEFAULT_ZOOM 2

struct caer_visualizer_state {
	atomic_bool running;
	ALLEGRO_FONT *displayFont;
	ALLEGRO_DISPLAY *displayWindow;
	int32_t displayWindowZoomFactor;
	ALLEGRO_EVENT_QUEUE *displayEventQueue;
	ALLEGRO_TIMER *displayTimer;
	ALLEGRO_BITMAP *bitmapRenderer;
	int32_t bitmapRendererSizeX;
	int32_t bitmapRendererSizeY;
	mtx_t bitmapMutex;
	struct caer_statistics_state packetStatistics;
	int32_t packetSubsampleRendering;
	int32_t packetSubsampleCount;
};

typedef struct caer_visualizer_state *caerVisualizerState;

// For reuse inside other modules.
bool caerVisualizerInit(caerVisualizerState state, int32_t bitmapSizeX, int32_t bitmapSizeY, int32_t zoomFactor,
	bool doStatistics);
void caerVisualizerUpdate(caerEventPacketHeader packetHeader, caerVisualizerState state);
void caerVisualizerUpdateScreen(caerVisualizerState state);
void caerVisualizerExit(caerVisualizerState state);

void caerVisualizer(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame);

void caerVisualizerSystemInit(void);

#endif /* VISUALIZER_ALLEGRO_H_ */
