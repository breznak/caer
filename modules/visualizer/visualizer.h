#ifndef VISUALIZER_H_
#define VISUALIZER_H_

#include "main.h"

#include <stdatomic.h>
#include <libcaer/events/polarity.h>
#include <libcaer/events/frame.h>
#include <libcaer/events/imu6.h>
#include <allegro5/allegro.h>
#include <allegro5/allegro_primitives.h>
#include <allegro5/allegro_font.h>
#include <allegro5/allegro_ttf.h>

#include "ext/ringbuffer/ringbuffer.h"
#include "modules/statistics/statistics.h"

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
	RingBuffer dataTransfer;
	struct caer_statistics_state packetStatistics;
	int32_t packetSubsampleRendering;
	int32_t packetSubsampleCount;
};

typedef struct caer_visualizer_state *caerVisualizerState;

// For reuse inside other modules.
bool caerVisualizerInit(caerVisualizerState state, int32_t bitmapSizeX, int32_t bitmapSizeY, int32_t zoomFactor,
	bool doStatistics);
bool caerVisualizerInitGraphics(caerVisualizerState state);
void caerVisualizerUpdate(caerEventPacketHeader packetHeader, caerVisualizerState state);
void caerVisualizerUpdateScreen(caerVisualizerState state);
void caerVisualizerExit(caerVisualizerState state);
int caerVisualizerRenderThread(void *visualizerState);

void caerVisualizer(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame, caerIMU6EventPacket imu);

void caerVisualizerSystemInit(void);

#endif /* VISUALIZER_H_ */
