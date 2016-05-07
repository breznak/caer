#include "visualizer.h"
#include "base/mainloop.h"
#include "ext/c11threads_posix.h"
#include "ext/ringbuffer/ringbuffer.h"
#include "modules/statistics/statistics.h"

#include <stdatomic.h>
#include <allegro5/allegro_primitives.h>
#include <allegro5/allegro_font.h>
#include <allegro5/allegro_ttf.h>

struct caer_visualizer_state {
	atomic_bool running;
	ALLEGRO_FONT *displayFont;
	ALLEGRO_DISPLAY *displayWindow;
	ALLEGRO_EVENT_QUEUE *displayEventQueue;
	ALLEGRO_TIMER *displayTimer;
	ALLEGRO_BITMAP *bitmapRenderer;
	int32_t bitmapRendererSizeX;
	int32_t bitmapRendererSizeY;
	bool bitmapDrawUpdate;
	float displayWindowZoomFactor;
	RingBuffer dataTransfer;
	thrd_t renderingThread;
	caerVisualizerRenderer renderer;
	caerVisualizerEventHandler eventHandler;
	caerModuleData parentModule;
	bool showStatistics;
	struct caer_statistics_state packetStatistics;
	int32_t packetSubsampleRendering;
	int32_t packetSubsampleCount;
};

static bool caerVisualizerInitGraphics(caerVisualizerState state);
static void caerVisualizerUpdateScreen(caerVisualizerState state);
static void caerVisualizerExitGraphics(caerVisualizerState state);
static int caerVisualizerRenderThread(void *visualizerState);

#define xstr(a) str(a)
#define str(a) #a

#ifdef CM_SHARE_DIR
#define CM_SHARE_DIRECTORY xstr(CM_SHARE_DIR)
#else
#define CM_SHARE_DIRECTORY "/usr/share/caer"
#endif

#ifdef CM_BUILD_DIR
#define CM_BUILD_DIRECTORY xstr(CM_BUILD_DIR)
#else
#define CM_BUILD_DIRECTORY ""
#endif

#define GLOBAL_RESOURCES_DIRECTORY "ext/resources"
#define GLOBAL_FONT_NAME "LiberationSans-Bold.ttf"
#define GLOBAL_FONT_SIZE 20 // in pixels
#define GLOBAL_FONT_SPACING 5 // in pixels

// Calculated at system init.
static int STATISTICS_WIDTH = 0;
static int STATISTICS_HEIGHT = 0;

static const char *systemFont = CM_SHARE_DIRECTORY "/" GLOBAL_FONT_NAME;
static const char *buildFont = CM_BUILD_DIRECTORY "/" GLOBAL_RESOURCES_DIRECTORY "/" GLOBAL_FONT_NAME;
static const char *globalFontPath = NULL;

void caerVisualizerSystemInit(void) {
	// Initialize the Allegro library.
	if (al_init()) {
		// Successfully initialized Allegro.
		caerLog(CAER_LOG_DEBUG, "Visualizer", "Allegro library initialized successfully.");
	}
	else {
		// Failed to initialize Allegro.
		caerLog(CAER_LOG_EMERGENCY, "Visualizer", "Failed to initialize Allegro library.");
		exit(EXIT_FAILURE);
	}

	// Set correct names.
	al_set_org_name("iniLabs");
	al_set_app_name("cAER");

	// Search for global font, first in system share dir, else in build dir.
	if (access(systemFont, R_OK) == 0) {
		globalFontPath = systemFont;
	}
	else {
		globalFontPath = buildFont;
	}

	// Now load addons: primitives to draw, fonts (and TTF) to write text.
	if (al_init_primitives_addon()) {
		// Successfully initialized Allegro primitives addon.
		caerLog(CAER_LOG_DEBUG, "Visualizer", "Allegro primitives addon initialized successfully.");
	}
	else {
		// Failed to initialize Allegro primitives addon.
		caerLog(CAER_LOG_EMERGENCY, "Visualizer", "Failed to initialize Allegro primitives addon.");
		exit(EXIT_FAILURE);
	}

	al_init_font_addon();

	if (al_init_ttf_addon()) {
		// Successfully initialized Allegro TTF addon.
		caerLog(CAER_LOG_DEBUG, "Visualizer", "Allegro TTF addon initialized successfully.");
	}
	else {
		// Failed to initialize Allegro TTF addon.
		caerLog(CAER_LOG_EMERGENCY, "Visualizer", "Failed to initialize Allegro TTF addon.");
		exit(EXIT_FAILURE);
	}

	// Determine biggest possible statistics string.
	size_t maxStatStringLength = (size_t) snprintf(NULL, 0, CAER_STATISTICS_STRING, UINT64_MAX, UINT64_MAX);

	char maxStatString[maxStatStringLength + 1];
	snprintf(maxStatString, maxStatStringLength + 1, CAER_STATISTICS_STRING, UINT64_MAX, UINT64_MAX);
	maxStatString[maxStatStringLength] = '\0';

	// Load statistics font into memory.
	ALLEGRO_FONT *font = al_load_font(globalFontPath, GLOBAL_FONT_SIZE, 0);
	if (font == NULL) {
		caerLog(CAER_LOG_ERROR, "Visualizer", "Failed to load display font '%s'.", globalFontPath);
	}

	// Determine statistics string width.
	if (font != NULL) {
		STATISTICS_WIDTH = (2 * GLOBAL_FONT_SPACING) + al_get_text_width(font, maxStatString);

		STATISTICS_HEIGHT = (2 * GLOBAL_FONT_SPACING) + GLOBAL_FONT_SIZE;

		al_destroy_font(font);
	}

	// Install main event sources: mouse and keyboard.
	if (al_install_mouse()) {
		// Successfully initialized Allegro mouse event source.
		caerLog(CAER_LOG_DEBUG, "Visualizer", "Allegro mouse event source initialized successfully.");
	}
	else {
		// Failed to initialize Allegro mouse event source.
		caerLog(CAER_LOG_EMERGENCY, "Visualizer", "Failed to initialize Allegro mouse event source.");
		exit(EXIT_FAILURE);
	}

	if (al_install_keyboard()) {
		// Successfully initialized Allegro keyboard event source.
		caerLog(CAER_LOG_DEBUG, "Visualizer", "Allegro keyboard event source initialized successfully.");
	}
	else {
		// Failed to initialize Allegro keyboard event source.
		caerLog(CAER_LOG_EMERGENCY, "Visualizer", "Failed to initialize Allegro keyboard event source.");
		exit(EXIT_FAILURE);
	}
}

caerVisualizerState caerVisualizerInit(caerVisualizerRenderer renderer, caerVisualizerEventHandler eventHandler,
	int32_t bitmapSizeX, int32_t bitmapSizeY, float defaultZoomFactor, bool defaultShowStatistics,
	caerModuleData parentModule) {
	// Allocate memory for visualizer state.
	caerVisualizerState state = calloc(1, sizeof(struct caer_visualizer_state));
	if (state == NULL) {
		caerLog(CAER_LOG_ERROR, parentModule->moduleSubSystemString, "Visualizer: Failed to allocate state memory.");
		return (NULL);
	}

	state->parentModule = parentModule;

	// Configuration.
	sshsNodePutIntIfAbsent(parentModule->moduleNode, "subsampleRendering", 1);
	sshsNodePutBoolIfAbsent(parentModule->moduleNode, "showStatistics", defaultShowStatistics);
	sshsNodePutFloatIfAbsent(parentModule->moduleNode, "zoomFactor", defaultZoomFactor);

	state->packetSubsampleRendering = sshsNodeGetInt(parentModule->moduleNode, "subsampleRendering");
	state->showStatistics = sshsNodeGetBool(parentModule->moduleNode, "showStatistics");
	state->displayWindowZoomFactor = sshsNodeGetFloat(parentModule->moduleNode, "zoomFactor");

	// Remember sizes.
	state->bitmapRendererSizeX = bitmapSizeX;
	state->bitmapRendererSizeY = bitmapSizeY;

	// Remember rendering and event handling function.
	state->renderer = renderer;
	state->eventHandler = eventHandler;

	// Enable packet statistics.
	if (!caerStatisticsStringInit(&state->packetStatistics)) {
		free(state);

		caerLog(CAER_LOG_ERROR, parentModule->moduleSubSystemString,
			"Visualizer: Failed to initialize statistics string.");
		return (NULL);
	}

	// Initialize ring-buffer to transfer data to render thread.
	state->dataTransfer = ringBufferInit(64);
	if (state->dataTransfer == NULL) {
		caerStatisticsStringExit(&state->packetStatistics);
		free(state);

		caerLog(CAER_LOG_ERROR, parentModule->moduleSubSystemString, "Visualizer: Failed to initialize ring-buffer.");
		return (NULL);
	}

	// Start separate rendering thread. Decouples presentation from
	// data processing and preparation. Communication over ring-buffer.
	atomic_store(&state->running, true);

	if (thrd_create(&state->renderingThread, &caerVisualizerRenderThread, state) != thrd_success) {
		ringBufferFree(state->dataTransfer);
		caerStatisticsStringExit(&state->packetStatistics);
		free(state);

		caerLog(CAER_LOG_ERROR, parentModule->moduleSubSystemString, "Visualizer: Failed to start rendering thread.");
		return (NULL);
	}

	caerLog(CAER_LOG_DEBUG, state->parentModule->moduleSubSystemString, "Visualizer: Initialized successfully.");

	return (state);
}

void caerVisualizerUpdate(caerVisualizerState state, caerEventPacketHeader packetHeader) {
	if (state == NULL || packetHeader == NULL) {
		return;
	}

	// Keep statistics up-to-date with all events, always.
	caerStatisticsStringUpdate(packetHeader, &state->packetStatistics);

	// Only render every Nth packet.
	state->packetSubsampleCount++;

	if (state->packetSubsampleCount >= state->packetSubsampleRendering) {
		state->packetSubsampleCount = 0;
	}
	else {
		return;
	}

	caerEventPacketHeader packetHeaderCopy = caerCopyEventPacketOnlyEvents(packetHeader);
	if (packetHeaderCopy == NULL) {
		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
			"Visualizer: Failed to copy event packet for rendering.");
		return;
	}

	if (!ringBufferPut(state->dataTransfer, packetHeaderCopy)) {
		free(packetHeaderCopy);

		caerLog(CAER_LOG_INFO, state->parentModule->moduleSubSystemString,
			"Visualizer: Failed to move event packet copy to ring-buffer (full).");
		return;
	}
}

void caerVisualizerExit(caerVisualizerState state) {
	if (state == NULL) {
		return;
	}

	// Shut down rendering thread and wait on it to finish.
	atomic_store(&state->running, false);

	if ((errno = thrd_join(state->renderingThread, NULL)) != thrd_success) {
		// This should never happen!
		caerLog(CAER_LOG_CRITICAL, state->parentModule->moduleSubSystemString,
			"Visualizer: Failed to join rendering thread. Error: %d.", errno);
	}

	// Now clean up the ring-buffer and its contents.
	caerEventPacketHeader packetHeader;
	while ((packetHeader = ringBufferGet(state->dataTransfer)) != NULL) {
		free(packetHeader);
	}

	ringBufferFree(state->dataTransfer);

	// Then the statistics string.
	caerStatisticsStringExit(&state->packetStatistics);

	// And finally the state memory.
	free(state);

	caerLog(CAER_LOG_DEBUG, state->parentModule->moduleSubSystemString, "Visualizer: Exited successfully.");
}

static bool caerVisualizerInitGraphics(caerVisualizerState state) {
	// Create display window.
	// When statistics are turned on, we need to add some space to the
	// X axis for displaying the whole line and the Y axis for spacing.
	int32_t displaySizeX = I32T((float ) state->bitmapRendererSizeX * state->displayWindowZoomFactor);
	int32_t displaySizeY = I32T((float ) state->bitmapRendererSizeY * state->displayWindowZoomFactor);

	if (state->showStatistics) {
		if (STATISTICS_WIDTH > displaySizeX) {
			displaySizeX = STATISTICS_WIDTH;
		}

		displaySizeY += STATISTICS_HEIGHT;
	}

	state->displayWindow = al_create_display(displaySizeX, displaySizeY);
	if (state->displayWindow == NULL) {
		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
			"Visualizer: Failed to create display window with sizeX=%d, sizeY=%d, zoomFactor=%f.", displaySizeX,
			displaySizeY, (double) state->displayWindowZoomFactor);
		return (false);
	}

	// Initialize window to all black.
	al_set_target_backbuffer(state->displayWindow);
	al_clear_to_color(al_map_rgb(0, 0, 0));
	al_flip_display();

	// Create memory bitmap for drawing into.
	al_set_new_bitmap_flags(ALLEGRO_MEMORY_BITMAP | ALLEGRO_MIN_LINEAR | ALLEGRO_MAG_LINEAR);
	state->bitmapRenderer = al_create_bitmap(state->bitmapRendererSizeX, state->bitmapRendererSizeY);
	if (state->bitmapRenderer == NULL) {
		// Clean up all memory that may have been used.
		caerVisualizerExitGraphics(state);

		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
			"Visualizer: Failed to create bitmap element with sizeX=%d, sizeY=%d.", state->bitmapRendererSizeX,
			state->bitmapRendererSizeY);
		return (false);
	}

	// Clear bitmap to all black.
	al_set_target_bitmap(state->bitmapRenderer);
	al_clear_to_color(al_map_rgb(0, 0, 0));

	// Timers and event queues for the rendering side.
	state->displayEventQueue = al_create_event_queue();
	if (state->displayEventQueue == NULL) {
		// Clean up all memory that may have been used.
		caerVisualizerExitGraphics(state);

		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString,
			"Visualizer: Failed to create event queue.");
		return (false);
	}

	state->displayTimer = al_create_timer(1.0f / VISUALIZER_REFRESH_RATE);
	if (state->displayTimer == NULL) {
		// Clean up all memory that may have been used.
		caerVisualizerExitGraphics(state);

		caerLog(CAER_LOG_ERROR, state->parentModule->moduleSubSystemString, "Visualizer: Failed to create timer.");
		return (false);
	}

	al_register_event_source(state->displayEventQueue, al_get_display_event_source(state->displayWindow));
	al_register_event_source(state->displayEventQueue, al_get_timer_event_source(state->displayTimer));
	al_register_event_source(state->displayEventQueue, al_get_keyboard_event_source());
	al_register_event_source(state->displayEventQueue, al_get_mouse_event_source());

	// Re-load font here so it's hardware accelerated.
	// A display must have been created and used as target for this to work.
	state->displayFont = al_load_font(globalFontPath, GLOBAL_FONT_SIZE, 0);
	if (state->displayFont == NULL) {
		caerLog(CAER_LOG_WARNING, state->parentModule->moduleSubSystemString,
			"Visualizer: Failed to load display font '%s'. Text rendering will not be possible.", globalFontPath);
	}

	// Everything fine, start timer for refresh.
	al_start_timer(state->displayTimer);

	return (true);
}

static void caerVisualizerUpdateScreen(caerVisualizerState state) {
	caerEventPacketHeader packetHeader = ringBufferGet(state->dataTransfer);

	repeat: if (packetHeader != NULL) {
		// Are there others? Only render last one, to avoid getting backed up!
		caerEventPacketHeader packetHeader2 = ringBufferGet(state->dataTransfer);

		if (packetHeader2 != NULL) {
			free(packetHeader);
			packetHeader = packetHeader2;
			goto repeat;
		}
	}

	if (packetHeader != NULL) {
		al_set_target_bitmap(state->bitmapRenderer);

		// Only clear bitmap to black if nothing has been
		// rendered since the last display flip.
		if (!state->bitmapDrawUpdate) {
			al_clear_to_color(al_map_rgb(0, 0, 0));
		}

		// Update bitmap with new content. (0, 0) is lower left corner.
		// NULL renderer is supported and simply does nothing (black screen).
		if (state->renderer != NULL) {
			bool didDrawSomething = (*state->renderer)(state, packetHeader);

			// Remember if something was drawn, even just once.
			if (!state->bitmapDrawUpdate) {
				state->bitmapDrawUpdate = didDrawSomething;
			}
		}

		// Free packet copy.
		free(packetHeader);
	}

	bool redraw = false;
	bool resize = false;
	ALLEGRO_EVENT displayEvent;

	handleEvents: al_wait_for_event(state->displayEventQueue, &displayEvent);

	if (displayEvent.type == ALLEGRO_EVENT_TIMER) {
		redraw = true;
	}
	else if (displayEvent.type == ALLEGRO_EVENT_DISPLAY_CLOSE) {
		sshsNodePutBool(state->parentModule->moduleNode, "shutdown", true);
	}
	else if (displayEvent.type == ALLEGRO_EVENT_KEY_DOWN) {
		// React to key presses, but only if they came from the corresponding display.
		if (displayEvent.keyboard.display == state->displayWindow) {
			if (displayEvent.keyboard.keycode == ALLEGRO_KEY_UP) {
				state->displayWindowZoomFactor++;
				resize = true;

				// Clip zoom factor.
				if (state->displayWindowZoomFactor > 50) {
					state->displayWindowZoomFactor = 50;
				}
			}
			else if (displayEvent.keyboard.keycode == ALLEGRO_KEY_DOWN) {
				state->displayWindowZoomFactor--;
				resize = true;

				// Clip zoom factor.
				if (state->displayWindowZoomFactor < 1) {
					state->displayWindowZoomFactor = 1;
				}
			}
		}
	}

	if (!al_is_event_queue_empty(state->displayEventQueue)) {
		// Handle all events before rendering, to avoid
		// having them backed up too much.
		goto handleEvents;
	}

	// Handle display resize (zoom).
	if (resize) {
		int32_t displaySizeX = I32T((float ) state->bitmapRendererSizeX * state->displayWindowZoomFactor);
		int32_t displaySizeY = I32T((float ) state->bitmapRendererSizeY * state->displayWindowZoomFactor);

		if (state->showStatistics) {
			if (STATISTICS_WIDTH > displaySizeX) {
				displaySizeX = STATISTICS_WIDTH;
			}

			displaySizeY += STATISTICS_HEIGHT;
		}

		al_resize_display(state->displayWindow, displaySizeX, displaySizeY);
	}

	// Render content to display.
	if (redraw && state->bitmapDrawUpdate) {
		state->bitmapDrawUpdate = false;

		al_set_target_backbuffer(state->displayWindow);
		al_clear_to_color(al_map_rgb(0, 0, 0));

		// Render statistics string.
		bool doStatistics = (state->showStatistics && state->displayFont != NULL);

		if (doStatistics) {
			al_draw_text(state->displayFont, al_map_rgb(255, 255, 255), GLOBAL_FONT_SPACING,
			GLOBAL_FONT_SPACING, 0, state->packetStatistics.currentStatisticsString);
		}

		// Blit bitmap to screen, taking zoom factor into consideration.
		al_draw_scaled_bitmap(state->bitmapRenderer, 0, 0, (float) state->bitmapRendererSizeX,
			(float) state->bitmapRendererSizeY, 0, (doStatistics) ? ((float) STATISTICS_HEIGHT) : (0),
			(float) state->bitmapRendererSizeX * state->displayWindowZoomFactor,
			(float) state->bitmapRendererSizeY * state->displayWindowZoomFactor, 0);

		al_flip_display();
	}
}

static void caerVisualizerExitGraphics(caerVisualizerState state) {
	al_set_target_bitmap(NULL);

	if (state->bitmapRenderer != NULL) {
		al_destroy_bitmap(state->bitmapRenderer);
		state->bitmapRenderer = NULL;
	}

	if (state->displayFont != NULL) {
		al_destroy_font(state->displayFont);
		state->displayFont = NULL;
	}

	// Destroy event queue first to ensure all sources get
	// unregistered before being destroyed in turn.
	if (state->displayEventQueue != NULL) {
		al_destroy_event_queue(state->displayEventQueue);
		state->displayEventQueue = NULL;
	}

	if (state->displayTimer != NULL) {
		al_destroy_timer(state->displayTimer);
		state->displayTimer = NULL;
	}

	if (state->displayWindow != NULL) {
		al_destroy_display(state->displayWindow);
		state->displayWindow = NULL;
	}
}

static int caerVisualizerRenderThread(void *visualizerState) {
	if (visualizerState == NULL) {
		return (thrd_error);
	}

	caerVisualizerState state = visualizerState;

	if (!caerVisualizerInitGraphics(state)) {
		return (thrd_error);
	}

	while (atomic_load_explicit(&state->running, memory_order_relaxed)) {
		caerVisualizerUpdateScreen(state);
	}

	caerVisualizerExitGraphics(state);

	return (thrd_success);
}

struct visualizer_module_state {
	caerVisualizerState visualizer;
};

typedef struct visualizer_module_state *visualizerModuleState;

// Init is deferred and called from Run, because we need actual packets.
static bool caerVisualizerModuleInit(caerModuleData moduleData, caerVisualizerRenderer renderer,
	caerVisualizerEventHandler eventHandler, caerEventPacketHeader packetHeader);
static void caerVisualizerModuleRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerVisualizerModuleExit(caerModuleData moduleData);

static struct caer_module_functions caerVisualizerFunctions = { .moduleInit = NULL, .moduleRun =
	&caerVisualizerModuleRun, .moduleConfig = NULL, .moduleExit = &caerVisualizerModuleExit };

void caerVisualizer(uint16_t moduleID, const char *name, caerVisualizerRenderer renderer,
	caerVisualizerEventHandler eventHandler, caerEventPacketHeader packetHeader) {
	// Concatenate name and 'Visualizer' prefix.
	size_t nameLength = (name != NULL) ? (strlen(name)) : (0);
	char visualizerName[10 + nameLength + 1]; // +1 for NUL termination.

	memcpy(visualizerName, "Visualizer", 10);
	if (name != NULL) {
		memcpy(visualizerName + 10, name, nameLength);
	}
	visualizerName[10 + nameLength] = '\0';

	caerModuleData moduleData = caerMainloopFindModule(moduleID, visualizerName);

	caerModuleSM(&caerVisualizerFunctions, moduleData, sizeof(struct visualizer_module_state), 3, renderer,
		eventHandler, packetHeader);
}

static bool caerVisualizerModuleInit(caerModuleData moduleData, caerVisualizerRenderer renderer,
	caerVisualizerEventHandler eventHandler, caerEventPacketHeader packetHeader) {
	visualizerModuleState state = moduleData->moduleState;

	// Get size information from source.
	int16_t sourceID = caerEventPacketHeaderGetEventSource(packetHeader);
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo((uint16_t) sourceID);

	// Default sizes if nothing else is specified in sourceInfo node.
	int16_t sizeX = 320;
	int16_t sizeY = 240;

	// Get sizes from sourceInfo node. visualizer prefix takes precedence,
	// for APS and DVS images, alternative prefixes are provided.
	if (sshsNodeAttributeExists(sourceInfoNode, "visualizerSizeX", SHORT)) {
		sizeX = sshsNodeGetShort(sourceInfoNode, "visualizerSizeX");
		sizeY = sshsNodeGetShort(sourceInfoNode, "visualizerSizeY");
	}
	else if (sshsNodeAttributeExists(sourceInfoNode, "dvsSizeX", SHORT)
		&& caerEventPacketHeaderGetEventType(packetHeader) == POLARITY_EVENT) {
		sizeX = sshsNodeGetShort(sourceInfoNode, "dvsSizeX");
		sizeY = sshsNodeGetShort(sourceInfoNode, "dvsSizeY");
	}
	else if (sshsNodeAttributeExists(sourceInfoNode, "apsSizeX", SHORT)
		&& caerEventPacketHeaderGetEventType(packetHeader) == FRAME_EVENT) {
		sizeX = sshsNodeGetShort(sourceInfoNode, "apsSizeX");
		sizeY = sshsNodeGetShort(sourceInfoNode, "apsSizeY");
	}

	state->visualizer = caerVisualizerInit(renderer, eventHandler, sizeX, sizeY, VISUALIZER_DEFAULT_ZOOM, true,
		moduleData);
	if (state->visualizer == NULL) {
		return (false);
	}

	return (true);
}

static void caerVisualizerModuleExit(caerModuleData moduleData) {
	visualizerModuleState state = moduleData->moduleState;

	// Shut down rendering.
	caerVisualizerExit(state->visualizer);
	state->visualizer = NULL;
}

static void caerVisualizerModuleRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	visualizerModuleState state = moduleData->moduleState;

	caerVisualizerRenderer renderer = va_arg(args, caerVisualizerRenderer);
	caerVisualizerEventHandler eventHandler = va_arg(args, caerVisualizerEventHandler);
	caerEventPacketHeader packetHeader = va_arg(args, caerEventPacketHeader);

	// Without a packet, we cannot initialize or render anything.
	if (packetHeader == NULL) {
		return;
	}

	// Initialize visualizer. Needs information from a packet (the source ID)!
	if (state->visualizer == NULL) {
		if (!caerVisualizerModuleInit(moduleData, renderer, eventHandler, packetHeader)) {
			return;
		}
	}

	// Render given packet.
	caerVisualizerUpdate(state->visualizer, packetHeader);
}

bool caerVisualizerRendererPolarityEvents(caerVisualizerState state, caerEventPacketHeader polarityEventPacketHeader) {
	UNUSED_ARGUMENT(state);

	if (caerEventPacketHeaderGetEventValid(polarityEventPacketHeader) == 0) {
		return (false);
	}

	// Render all valid events.
	CAER_POLARITY_ITERATOR_VALID_START((caerPolarityEventPacket) polarityEventPacketHeader)
		if (caerPolarityEventGetPolarity(caerPolarityIteratorElement)) {
			// ON polarity (green).
			al_put_pixel(caerPolarityEventGetX(caerPolarityIteratorElement),
				caerPolarityEventGetY(caerPolarityIteratorElement), al_map_rgb(0, 255, 0));
		}
		else {
			// OFF polarity (red).
			al_put_pixel(caerPolarityEventGetX(caerPolarityIteratorElement),
				caerPolarityEventGetY(caerPolarityIteratorElement), al_map_rgb(255, 0, 0));
		}
	CAER_POLARITY_ITERATOR_VALID_END

	return (true);
}

bool caerVisualizerRendererFrameEvents(caerVisualizerState state, caerEventPacketHeader frameEventPacketHeader) {
	UNUSED_ARGUMENT(state);

	if (caerEventPacketHeaderGetEventValid(frameEventPacketHeader) == 0) {
		return (false);
	}

	// Render only the last, valid frame.
	caerFrameEventPacket currFramePacket = (caerFrameEventPacket) frameEventPacketHeader;
	caerFrameEvent currFrameEvent;

	for (int32_t i = caerEventPacketHeaderGetEventNumber(&currFramePacket->packetHeader) - 1; i >= 0; i--) {
		currFrameEvent = caerFrameEventPacketGetEvent(currFramePacket, i);

		// Only operate on the last, valid frame.
		if (caerFrameEventIsValid(currFrameEvent)) {
			// Copy the frame content to the render bitmap.
			// Use frame sizes to correctly support small ROI frames.
			int32_t frameSizeX = caerFrameEventGetLengthX(currFrameEvent);
			int32_t frameSizeY = caerFrameEventGetLengthY(currFrameEvent);
			int32_t framePositionX = caerFrameEventGetPositionX(currFrameEvent);
			int32_t framePositionY = caerFrameEventGetPositionY(currFrameEvent);
			enum caer_frame_event_color_channels frameChannels = caerFrameEventGetChannelNumber(currFrameEvent);

			for (int32_t y = 0; y < frameSizeY; y++) {
				for (int32_t x = 0; x < frameSizeX; x++) {
					ALLEGRO_COLOR color;

					switch (frameChannels) {
						case GRAYSCALE: {
							uint8_t pixel = U8T(caerFrameEventGetPixelUnsafe(currFrameEvent, x, y) >> 8);
							color = al_map_rgb(pixel, pixel, pixel);
							break;
						}

						case RGB: {
							uint8_t pixelR = U8T(caerFrameEventGetPixelForChannelUnsafe(currFrameEvent, x, y, 0) >> 8);
							uint8_t pixelG = U8T(caerFrameEventGetPixelForChannelUnsafe(currFrameEvent, x, y, 1) >> 8);
							uint8_t pixelB = U8T(caerFrameEventGetPixelForChannelUnsafe(currFrameEvent, x, y, 2) >> 8);
							color = al_map_rgb(pixelR, pixelG, pixelB);
							break;
						}

						case RGBA:
						default: {
							uint8_t pixelR = U8T(caerFrameEventGetPixelForChannelUnsafe(currFrameEvent, x, y, 0) >> 8);
							uint8_t pixelG = U8T(caerFrameEventGetPixelForChannelUnsafe(currFrameEvent, x, y, 1) >> 8);
							uint8_t pixelB = U8T(caerFrameEventGetPixelForChannelUnsafe(currFrameEvent, x, y, 2) >> 8);
							uint8_t pixelA = U8T(caerFrameEventGetPixelForChannelUnsafe(currFrameEvent, x, y, 3) >> 8);
							color = al_map_rgba(pixelR, pixelG, pixelB, pixelA);
							break;
						}
					}

					al_put_pixel((framePositionX + x), (framePositionY + y), color);
				}
			}

			return (true);
		}
	}

	return (false);
}

#define RESET_LIMIT_POS(VAL, LIMIT) if ((VAL) > (LIMIT)) { (VAL) = (LIMIT); }
#define RESET_LIMIT_NEG(VAL, LIMIT) if ((VAL) < (LIMIT)) { (VAL) = (LIMIT); }

bool caerVisualizerRendererIMU6Events(caerVisualizerState state, caerEventPacketHeader imu6EventPacketHeader) {
	if (caerEventPacketHeaderGetEventValid(imu6EventPacketHeader) == 0) {
		return (false);
	}

	float scaleFactorAccel = 30;
	float scaleFactorGyro = 15;
	float lineThickness = 4;
	float maxSizeX = (float) state->bitmapRendererSizeX;
	float maxSizeY = (float) state->bitmapRendererSizeY;

	ALLEGRO_COLOR accelColor = al_map_rgb(0, 255, 0);
	ALLEGRO_COLOR gyroColor = al_map_rgb(255, 0, 255);

	float centerPointX = maxSizeX / 2;
	float centerPointY = maxSizeY / 2;

	float accelX = 0, accelY = 0, accelZ = 0;
	float gyroX = 0, gyroY = 0, gyroZ = 0;

	// Iterate over valid IMU events and average them.
	// This somewhat smoothes out the rendering.
	CAER_IMU6_ITERATOR_VALID_START((caerIMU6EventPacket) imu6EventPacketHeader)
		accelX += caerIMU6EventGetAccelX(caerIMU6IteratorElement);
		accelY += caerIMU6EventGetAccelY(caerIMU6IteratorElement);
		accelZ += caerIMU6EventGetAccelZ(caerIMU6IteratorElement);

		gyroX += caerIMU6EventGetGyroX(caerIMU6IteratorElement);
		gyroY += caerIMU6EventGetGyroY(caerIMU6IteratorElement);
		gyroZ += caerIMU6EventGetGyroZ(caerIMU6IteratorElement);
	CAER_IMU6_ITERATOR_VALID_END

	// Normalize values.
	int32_t validEvents = caerEventPacketHeaderGetEventValid(imu6EventPacketHeader);

	accelX /= (float) validEvents;
	accelY /= (float) validEvents;
	accelZ /= (float) validEvents;

	gyroX /= (float) validEvents;
	gyroY /= (float) validEvents;
	gyroZ /= (float) validEvents;

	// Acceleration X, Y as lines. Z as a circle.
	float accelXScaled = centerPointX + accelX * scaleFactorAccel;
	RESET_LIMIT_POS(accelXScaled, maxSizeX - 2 - lineThickness);
	RESET_LIMIT_NEG(accelXScaled, 1 + lineThickness);
	float accelYScaled = centerPointY - accelY * scaleFactorAccel;
	RESET_LIMIT_POS(accelYScaled, maxSizeY - 2 - lineThickness);
	RESET_LIMIT_NEG(accelYScaled, 1 + lineThickness);
	float accelZScaled = accelZ * scaleFactorAccel;
	RESET_LIMIT_POS(accelZScaled, centerPointY - 2 - lineThickness); // Circle max.
	RESET_LIMIT_NEG(accelZScaled, 1); // Circle min.

	al_draw_line(centerPointX, centerPointY, accelXScaled, accelYScaled, accelColor, lineThickness);
	al_draw_circle(centerPointX, centerPointY, accelZScaled, accelColor, lineThickness);

	// TODO: Add text for values. Check that displayFont is not NULL.
	//char valStr[128];
	//snprintf(valStr, 128, "%.2f,%.2f g", (double) accelX, (double) accelY);
	//al_draw_text(state->displayFont, accelColor, accelXScaled, accelYScaled, 0, valStr);

	// Gyroscope pitch(X), yaw(Y), roll(Z) as lines.
	float gyroXScaled = centerPointY - gyroX * scaleFactorGyro;
	RESET_LIMIT_POS(gyroXScaled, maxSizeY - 2 - lineThickness);
	RESET_LIMIT_NEG(gyroXScaled, 1 + lineThickness);
	float gyroYScaled = centerPointX + gyroY * scaleFactorGyro;
	RESET_LIMIT_POS(gyroYScaled, maxSizeX - 2 - lineThickness);
	RESET_LIMIT_NEG(gyroYScaled, 1 + lineThickness);
	float gyroZScaled = centerPointX + gyroZ * scaleFactorGyro;
	RESET_LIMIT_POS(gyroZScaled, maxSizeX - 2 - lineThickness);
	RESET_LIMIT_NEG(gyroZScaled, 1 + lineThickness);

	al_draw_line(centerPointX, centerPointY, gyroYScaled, gyroXScaled, gyroColor, lineThickness);
	al_draw_line(centerPointX, centerPointY - 20, gyroZScaled, centerPointY - 20, gyroColor, lineThickness);

	return (true);
}
