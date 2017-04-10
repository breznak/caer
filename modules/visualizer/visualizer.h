#ifndef VISUALIZER_H_
#define VISUALIZER_H_

#include "main.h"
#include "base/module.h"

#include <libcaer/events/packetContainer.h>
#include <allegro5/allegro.h>
#include <allegro5/allegro_font.h>

#define VISUALIZER_DEFAULT_ZOOM 2.0f
#define VISUALIZER_REFRESH_RATE 60.0f

struct caer_visualizer_public_state {
	void *eventSourceModuleState;
	sshsNode eventSourceConfigNode;
	sshsNode visualizerConfigNode;
	int32_t bitmapRendererSizeX;
	int32_t bitmapRendererSizeY;
	ALLEGRO_FONT *displayFont;
};

typedef struct caer_visualizer_public_state *caerVisualizerPublicState;
typedef struct caer_visualizer_state *caerVisualizerState;

typedef bool (*caerVisualizerRenderer)(caerVisualizerPublicState state, caerEventPacketContainer container,
	bool doClear);
typedef void (*caerVisualizerEventHandler)(caerVisualizerPublicState state, ALLEGRO_EVENT event);

// For reuse inside other modules.
caerVisualizerState caerVisualizerInit(caerVisualizerRenderer renderer, caerVisualizerEventHandler eventHandler,
	int32_t bitmapSizeX, int32_t bitmapSizeY, float defaultZoomFactor, bool defaultShowStatistics,
	caerModuleData parentModule, int16_t eventSourceID, int32_t bitmapRasterSizeX, int32_t bitmapRasterSizeY);
void caerVisualizerUpdate(caerVisualizerState state, caerEventPacketContainer container);
void caerVisualizerExit(caerVisualizerState state);
void caerVisualizerReset(caerVisualizerState state);

// Event packet visualizer.
void caerVisualizer(uint16_t moduleID, const char *name, caerVisualizerRenderer renderer,
	caerVisualizerEventHandler eventHandler, caerEventPacketHeader packetHeader);

// Default renderers.
bool caerVisualizerRendererPolarityEvents(caerVisualizerPublicState state, caerEventPacketContainer container,
	bool doClear);
bool caerVisualizerRendererFrameEvents(caerVisualizerPublicState state, caerEventPacketContainer container,
	bool doClear);
bool caerVisualizerRendererIMU6Events(caerVisualizerPublicState state, caerEventPacketContainer container, bool doClear);
bool caerVisualizerRendererPoint2DEvents(caerVisualizerPublicState state, caerEventPacketContainer container,
	bool doClear);
bool caerVisualizerRendererSpikeEvents(caerVisualizerPublicState state, caerEventPacketContainer container,
	bool doClear);
bool caerVisualizerRendererSpikeEventsRaster(caerVisualizerPublicState state, caerEventPacketContainer container,
bool doClear);
bool caerVisualizerRendererETF4D(caerVisualizerPublicState state, caerEventPacketContainer container,
bool doClear);

// Default event handlers.
void caerVisualizerEventHandlerSpikeEvents(caerVisualizerPublicState state, ALLEGRO_EVENT event);
void caerVisualizerEventHandlerFrameEvents(caerVisualizerPublicState state, ALLEGRO_EVENT event);

// Event packet container visualizer (multiple packets).
void caerVisualizerMulti(uint16_t moduleID, const char *name, caerVisualizerRenderer renderer,
	caerVisualizerEventHandler eventHandler, caerEventPacketContainer container);

// Default multi renderers.
bool caerVisualizerMultiRendererPolarityAndFrameEvents(caerVisualizerPublicState state,
	caerEventPacketContainer container,
	bool doClear);

void caerVisualizerSystemInit(void);

#endif /* VISUALIZER_H_ */
