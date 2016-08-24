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
	sshsNode eventSourceConfigNode;
	int32_t bitmapRendererSizeX;
	int32_t bitmapRendererSizeY;
	ALLEGRO_FONT *displayFont;
};

typedef struct caer_visualizer_public_state *caerVisualizerPublicState;
typedef struct caer_visualizer_state *caerVisualizerState;

typedef bool (*caerVisualizerRenderer)(caerVisualizerPublicState state, caerEventPacketContainer container, bool doClear);
typedef void (*caerVisualizerEventHandler)(caerVisualizerPublicState state, ALLEGRO_EVENT event);

// For reuse inside other modules.
caerVisualizerState caerVisualizerInit(caerVisualizerRenderer renderer, caerVisualizerEventHandler eventHandler,
	int32_t bitmapSizeX, int32_t bitmapSizeY, float defaultZoomFactor, bool defaultShowStatistics,
	caerModuleData parentModule, int16_t eventSourceID);
void caerVisualizerUpdate(caerVisualizerState state, caerEventPacketContainer container);
void caerVisualizerExit(caerVisualizerState state);
void caerVisualizerReset(caerVisualizerState state);

void caerVisualizer(uint16_t moduleID, const char *name, caerVisualizerRenderer renderer,
	caerVisualizerEventHandler eventHandler, caerEventPacketHeader packetHeader);

bool caerVisualizerRendererPolarityEvents(caerVisualizerPublicState state, caerEventPacketContainer container, bool doClear);
bool caerVisualizerRendererFrameEvents(caerVisualizerPublicState state, caerEventPacketContainer container, bool doClear);
bool caerVisualizerRendererIMU6Events(caerVisualizerPublicState state, caerEventPacketContainer container, bool doClear);
bool caerVisualizerRendererPoint2DEvents(caerVisualizerPublicState state, caerEventPacketContainer container, bool doClear);

void caerVisualizerMulti(uint16_t moduleID, const char *name, caerVisualizerRenderer renderer,
	caerVisualizerEventHandler eventHandler, caerEventPacketContainer container);

bool caerVisualizerMultiRendererPolarityAndFrameEvents(caerVisualizerPublicState state, caerEventPacketContainer container,
	bool doClear);

void caerVisualizerSystemInit(void);

#endif /* VISUALIZER_H_ */
