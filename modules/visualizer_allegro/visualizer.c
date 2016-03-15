#include "visualizer.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/portable_time.h"

#define GLOBAL_RESOURCES_DIR "ext/resources/"
#define GLOBAL_FONT_NAME "LiberationSans-Bold.ttf"
#define GLOBAL_FONT_SIZE 30 // in pixels
#define GLOBAL_FONT_SPACING 5 // in pixels

// Calculated at system init.
static int STATISTICS_WIDTH = 0;
static int STATISTICS_HEIGHT = 0;

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

	// Set up path to find local resources.
	ALLEGRO_PATH *globalResourcesPath = al_get_standard_path(ALLEGRO_RESOURCES_PATH);
	if (globalResourcesPath != NULL) {
		// Successfully loaded standard resources path.
		caerLog(CAER_LOG_DEBUG, "Visualizer", "Allegro standard resources path loaded successfully.");
	}
	else {
		// Failed to load standard resources path.
		caerLog(CAER_LOG_EMERGENCY, "Visualizer", "Failed to load Allegro standard resources path.");
		exit(EXIT_FAILURE);
	}

	al_append_path_component(globalResourcesPath, GLOBAL_RESOURCES_DIR);

	// Remember global font path.
	al_set_path_filename(globalResourcesPath, GLOBAL_FONT_NAME);
	globalFontPath = al_path_cstr(globalResourcesPath, ALLEGRO_NATIVE_PATH_SEP);

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

	// Determine statistics string width.
	if (font != NULL) {
		STATISTICS_WIDTH = al_get_text_width(font, maxStatString);

		STATISTICS_HEIGHT = (GLOBAL_FONT_SPACING + GLOBAL_FONT_SIZE + GLOBAL_FONT_SPACING);

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

bool caerVisualizerInit(caerVisualizerState state, int32_t bitmapSizeX, int32_t bitmapSizeY, int32_t zoomFactor,
bool doStatistics) {
	// Create display window.
	// When statistics are turned on, we need to add some space to the
	// X axis for displaying the whole line and the Y axis for spacing.
	int32_t displaySizeX = bitmapSizeX * zoomFactor;
	int32_t displaySizeY = bitmapSizeY * zoomFactor;

	if (doStatistics) {
		if (STATISTICS_WIDTH > displaySizeX) {
			displaySizeX = STATISTICS_WIDTH;
		}

		displaySizeY += STATISTICS_HEIGHT;
	}

	state->displayWindow = al_create_display(displaySizeX, displaySizeY);
	if (state->displayWindow == NULL) {
		caerLog(CAER_LOG_ERROR, "Visualizer",
			"Failed to create display element with sizeX=%d, sizeY=%d, zoomFactor=%d.", displaySizeX, displaySizeY,
			zoomFactor);
		return (false);
	}

	// Initialize window to all black.
	al_set_target_backbuffer(state->displayWindow);
	al_clear_to_color(al_map_rgb(0, 0, 0));
	al_flip_display();

	// Re-load font here so it's hardware accelerated.
	// A display must have been created and used as target for this to work.
	state->displayFont = al_load_font(globalFontPath, GLOBAL_FONT_SIZE, 0);
	if (state->displayFont == NULL) {
		caerLog(CAER_LOG_ERROR, "Visualizer", "Failed to load display font '%s'. Disabling statistics.",
			globalFontPath);
	}

	// Create memory bitmap for drawing into.
	al_set_new_bitmap_flags(ALLEGRO_MEMORY_BITMAP | ALLEGRO_MIN_LINEAR | ALLEGRO_MAG_LINEAR);
	state->bitmapRenderer = al_create_bitmap(bitmapSizeX, bitmapSizeY);
	if (state->bitmapRenderer == NULL) {
		caerLog(CAER_LOG_ERROR, "Visualizer", "Failed to create bitmap element with sizeX=%d, sizeY=%d.", bitmapSizeX,
			bitmapSizeY);
		return (false);
	}

	// Clear bitmap to all black.
	al_set_target_bitmap(state->bitmapRenderer);
	al_clear_to_color(al_map_rgb(0, 0, 0));

	// Remember sizes.
	state->bitmapRendererSizeX = bitmapSizeX;
	state->bitmapRendererSizeY = bitmapSizeY;
	state->displayWindowZoomFactor = zoomFactor;

	// Enable packet statistics, only if font to render them exists.
	if (doStatistics && state->displayFont != NULL) {
		caerStatisticsStringInit(&state->packetStatistics);
	}

	// Sub-sampling support. Default to none.
	state->packetSubsampleRendering = 1;
	state->packetSubsampleCount = 0;

	// Timers and event queues for the rendering side.
	state->displayEventQueue = al_create_event_queue();
	state->displayTimer = al_create_timer(1.0 / 60.0);

	al_register_event_source(state->displayEventQueue, al_get_display_event_source(state->displayWindow));
	al_register_event_source(state->displayEventQueue, al_get_timer_event_source(state->displayTimer));
	al_register_event_source(state->displayEventQueue, al_get_keyboard_event_source());
	al_register_event_source(state->displayEventQueue, al_get_mouse_event_source());

	al_start_timer(state->displayTimer);

	// Initialize mutex for locking between update and screen draw operations.
	mtx_init(&state->bitmapMutex, mtx_plain);

	atomic_store(&state->running, true);

	return (true);
}

void caerVisualizerUpdate(caerEventPacketHeader packetHeader, caerVisualizerState state) {
	// Only render ever Nth packet.
	state->packetSubsampleCount++;

	if (state->packetSubsampleCount >= state->packetSubsampleRendering) {
		state->packetSubsampleCount = 0;
	}
	else {
		return;
	}

	mtx_lock(&state->bitmapMutex);

	// Update statistics (if enabled).
	if (state->packetStatistics.currentStatisticsString != NULL) {
		caerStatisticsStringUpdate(packetHeader, &state->packetStatistics);
	}

	al_set_target_bitmap(state->bitmapRenderer);
	al_clear_to_color(al_map_rgb(0, 0, 0));

	// Update bitmap with new content. (0, 0) is lower left corner.
	if (caerEventPacketHeaderGetEventType(packetHeader) == POLARITY_EVENT) {
		// Render all valid events.
		CAER_POLARITY_ITERATOR_VALID_START((caerPolarityEventPacket) packetHeader)
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
	}
	else if (caerEventPacketHeaderGetEventType(packetHeader) == FRAME_EVENT) {
		// Render only the last, valid frame.
		caerFrameEventPacket currFramePacket = (caerFrameEventPacket) packetHeader;
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
								uint8_t pixelR = U8T(
									caerFrameEventGetPixelForChannelUnsafe(currFrameEvent, x, y, 0) >> 8);
								uint8_t pixelG = U8T(
									caerFrameEventGetPixelForChannelUnsafe(currFrameEvent, x, y, 1) >> 8);
								uint8_t pixelB = U8T(
									caerFrameEventGetPixelForChannelUnsafe(currFrameEvent, x, y, 2) >> 8);
								color = al_map_rgb(pixelR, pixelG, pixelB);
								break;
							}

							case RGBA:
							default: {
								uint8_t pixelR = U8T(
									caerFrameEventGetPixelForChannelUnsafe(currFrameEvent, x, y, 0) >> 8);
								uint8_t pixelG = U8T(
									caerFrameEventGetPixelForChannelUnsafe(currFrameEvent, x, y, 1) >> 8);
								uint8_t pixelB = U8T(
									caerFrameEventGetPixelForChannelUnsafe(currFrameEvent, x, y, 2) >> 8);
								uint8_t pixelA = U8T(
									caerFrameEventGetPixelForChannelUnsafe(currFrameEvent, x, y, 3) >> 8);
								color = al_map_rgba(pixelR, pixelG, pixelB, pixelA);
								break;
							}
						}

						al_put_pixel(framePositionX + x, framePositionY + y, color);
					}
				}

				break;
			}
		}
	}
	else if (caerEventPacketHeaderGetEventType(packetHeader) == IMU6_EVENT
		&& caerEventPacketHeaderGetEventValid(packetHeader) != 0) {
		float accelX = 0, accelY = 0, accelZ = 0;
		float gyroX = 0, gyroY = 0, gyroZ = 0;

		// Iterate over valid IMU events and average them.
		// This somewhat smoothes out the rendering.
		CAER_IMU6_ITERATOR_VALID_START((caerIMU6EventPacket) packetHeader)
			accelX += caerIMU6EventGetAccelX(caerIMU6IteratorElement);
			accelY += caerIMU6EventGetAccelY(caerIMU6IteratorElement);
			accelZ += caerIMU6EventGetAccelZ(caerIMU6IteratorElement);

			gyroX += caerIMU6EventGetGyroX(caerIMU6IteratorElement);
			gyroY += caerIMU6EventGetGyroY(caerIMU6IteratorElement);
			gyroZ += caerIMU6EventGetGyroZ(caerIMU6IteratorElement);
		CAER_IMU6_ITERATOR_VALID_END

		float scaleFactor = 30;
		float centerPoint = 100;

		ALLEGRO_COLOR accelColor = al_map_rgb(0, 255, 0);
		ALLEGRO_COLOR gyroColor = al_map_rgb(255, 0, 255);

		// Acceleration X, Y as lines. Z as a circle.
		accelX /= caerEventPacketHeaderGetEventValid(packetHeader);
		accelY /= caerEventPacketHeaderGetEventValid(packetHeader);
		accelZ /= caerEventPacketHeaderGetEventValid(packetHeader);

		al_draw_line(centerPoint, centerPoint, centerPoint + (accelX * scaleFactor),
			centerPoint - (accelY * scaleFactor), accelColor, 4);
		al_draw_circle(centerPoint, centerPoint, accelZ * scaleFactor, accelColor, 4);

		// Add text for values.
		char valStr[128];
		snprintf(valStr, 128, "%.2f,%.2f g", accelX, accelY);

		al_draw_text(state->displayFont, accelColor, centerPoint + (accelX * scaleFactor),
			centerPoint - (accelY * scaleFactor), 0, valStr);

		// Gyroscope pitch(X), yaw(Y), roll(Z) as lines.
		scaleFactor = 10;

		gyroX /= caerEventPacketHeaderGetEventValid(packetHeader);
		gyroY /= caerEventPacketHeaderGetEventValid(packetHeader);
		gyroZ /= caerEventPacketHeaderGetEventValid(packetHeader);

		al_draw_line(centerPoint, centerPoint, centerPoint + (gyroY * scaleFactor), centerPoint - (gyroX * scaleFactor),
			gyroColor, 4);
		al_draw_line(centerPoint, centerPoint - 25, centerPoint + (gyroZ * scaleFactor), centerPoint - 25, gyroColor,
			4);
	}

	mtx_unlock(&state->bitmapMutex);
}

void caerVisualizerUpdateScreen(caerVisualizerState state) {
	bool redraw = false;
	bool resize = false;
	ALLEGRO_EVENT displayEvent;

	handleEvents: al_wait_for_event(state->displayEventQueue, &displayEvent);

	if (displayEvent.type == ALLEGRO_EVENT_TIMER) {
		redraw = true;
	}
	else if (displayEvent.type == ALLEGRO_EVENT_DISPLAY_CLOSE) {
		// TODO: shutdown!
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
		int32_t displaySizeX = state->bitmapRendererSizeX * state->displayWindowZoomFactor;
		int32_t displaySizeY = state->bitmapRendererSizeY * state->displayWindowZoomFactor;

		if (state->packetStatistics.currentStatisticsString != NULL) {
			if (STATISTICS_WIDTH > displaySizeX) {
				displaySizeX = STATISTICS_WIDTH;
			}

			displaySizeY += STATISTICS_HEIGHT;
		}

		al_resize_display(state->displayWindow, displaySizeX, displaySizeY);
	}

	// Render content to display.
	if (redraw) {
		al_set_target_backbuffer(state->displayWindow);
		al_clear_to_color(al_map_rgb(0, 0, 0));

		mtx_lock(&state->bitmapMutex);

		// Render statistics string.
		bool doStatistics = (state->packetStatistics.currentStatisticsString != NULL);

		if (doStatistics) {
			al_draw_text(state->displayFont, al_map_rgb(255, 255, 255), GLOBAL_FONT_SPACING,
			GLOBAL_FONT_SPACING, 0, state->packetStatistics.currentStatisticsString);
		}

		// Blit bitmap to screen, taking zoom factor into consideration.
		al_draw_scaled_bitmap(state->bitmapRenderer, 0, 0, state->bitmapRendererSizeX, state->bitmapRendererSizeY, 0,
			(doStatistics) ? (STATISTICS_HEIGHT) : (0), state->bitmapRendererSizeX * state->displayWindowZoomFactor,
			state->bitmapRendererSizeY * state->displayWindowZoomFactor, ALLEGRO_FLIP_VERTICAL);

		mtx_unlock(&state->bitmapMutex);

		al_flip_display();
	}
}

void caerVisualizerExit(caerVisualizerState state) {
	al_set_target_bitmap(NULL);

	al_destroy_bitmap(state->bitmapRenderer);
	state->bitmapRenderer = NULL;

	if (state->displayFont != NULL) {
		al_destroy_font(state->displayFont);
		state->displayFont = NULL;
	}

	al_destroy_display(state->displayWindow);
	state->displayWindow = NULL;

	al_destroy_timer(state->displayTimer);
	state->displayTimer = NULL;

	al_destroy_event_queue(state->displayEventQueue);
	state->displayEventQueue = NULL;

	if (state->packetStatistics.currentStatisticsString != NULL) {
		caerStatisticsStringExit(&state->packetStatistics);
	}

	mtx_destroy(&state->bitmapMutex);

	atomic_store(&state->running, false);
}

struct visualizer_module_state {
	thrd_t renderingThread;
	struct caer_visualizer_state eventVisualizer;
	struct caer_visualizer_state frameVisualizer;
	struct caer_visualizer_state imuVisualizer;
};

typedef struct visualizer_module_state *visualizerModuleState;

static bool caerVisualizerModuleInit(caerModuleData moduleData);
static void caerVisualizerModuleRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerVisualizerModuleExit(caerModuleData moduleData);
static int caerVisualizerModuleRenderThread(void *moduleData);
static bool initializeEventRenderer(visualizerModuleState state, int16_t sourceID);
static bool initializeFrameRenderer(visualizerModuleState state, int16_t sourceID);
static bool initializeIMURenderer(visualizerModuleState state);

static struct caer_module_functions caerVisualizerFunctions = { .moduleInit = &caerVisualizerModuleInit, .moduleRun =
	&caerVisualizerModuleRun, .moduleConfig =
NULL, .moduleExit = &caerVisualizerModuleExit };

void caerVisualizer(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame,
	caerIMU6EventPacket imu) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "Visualizer");

	caerModuleSM(&caerVisualizerFunctions, moduleData, sizeof(struct visualizer_module_state), 3, polarity, frame, imu);
}

static bool caerVisualizerModuleInit(caerModuleData moduleData) {
	visualizerModuleState state = moduleData->moduleState;

	// Configuration.
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "showEvents", true);
#ifdef DVS128
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "showFrames", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "showIMU", true);
#else
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "showFrames", true);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "showIMU", true);
#endif

	sshsNodePutIntIfAbsent(moduleData->moduleNode, "subsampleEventsRendering", 1);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "subsampleFramesRendering", 1);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "subsampleIMURendering", 1);

	// Start separate rendering thread. Decouples presentation from
	// data processing and preparation. Communication over properly
	// locked bitmap.
	thrd_create(&state->renderingThread, &caerVisualizerModuleRenderThread, moduleData);

	return (true);
}

static void caerVisualizerModuleExit(caerModuleData moduleData) {
	visualizerModuleState state = moduleData->moduleState;

	// Wait on rendering thread.
	thrd_join(state->renderingThread, NULL);

	// Ensure render maps are freed.
	if (atomic_load(&state->eventVisualizer.running)) {
		caerVisualizerExit(&state->eventVisualizer);
	}

	if (atomic_load(&state->frameVisualizer.running)) {
		caerVisualizerExit(&state->frameVisualizer);
	}

	if (atomic_load(&state->imuVisualizer.running)) {
		caerVisualizerExit(&state->imuVisualizer);
	}
}

static void caerVisualizerModuleRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	visualizerModuleState state = moduleData->moduleState;

	// Polarity events to render.
	caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket);
	bool renderPolarity = sshsNodeGetBool(moduleData->moduleNode, "showEvents");

	// Frames to render.
	caerFrameEventPacket frame = va_arg(args, caerFrameEventPacket);
	bool renderFrame = sshsNodeGetBool(moduleData->moduleNode, "showFrames");

	// IMU events to render.
	caerIMU6EventPacket imu = va_arg(args, caerIMU6EventPacket);
	bool renderIMU = sshsNodeGetBool(moduleData->moduleNode, "showIMU");

	// Update polarity event rendering map.
	if (renderPolarity && polarity != NULL) {
		// If the event renderer is not allocated yet, do it.
		if (!atomic_load_explicit(&state->eventVisualizer.running, memory_order_relaxed)) {
			if (!initializeEventRenderer(state, caerEventPacketHeaderGetEventSource(&polarity->packetHeader))) {
				caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to initialize event visualizer.");
				return;
			}
		}

		// Update sub-sample value.
		state->eventVisualizer.packetSubsampleRendering = sshsNodeGetInt(moduleData->moduleNode,
			"subsampleEventsRendering");

		// Actually update polarity rendering.
		caerVisualizerUpdate(&polarity->packetHeader, &state->eventVisualizer);
	}
	else if (!renderPolarity) {
		if (atomic_load_explicit(&state->eventVisualizer.running, memory_order_relaxed)) {
			mtx_lock(&state->eventVisualizer.bitmapMutex);
			atomic_store(&state->eventVisualizer.running, false);
			mtx_unlock(&state->eventVisualizer.bitmapMutex);

			caerVisualizerExit(&state->eventVisualizer);
		}
	}

	// Select latest frame to render.
	if (renderFrame && frame != NULL) {
		// If the frame renderer is not allocated yet, do it.
		if (!atomic_load_explicit(&state->frameVisualizer.running, memory_order_relaxed)) {
			if (!initializeFrameRenderer(state, caerEventPacketHeaderGetEventSource(&frame->packetHeader))) {
				caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to initialize frame visualizer.");
				return;
			}
		}

		// Update sub-sample value.
		state->frameVisualizer.packetSubsampleRendering = sshsNodeGetInt(moduleData->moduleNode,
			"subsampleFramesRendering");

		// Actually update frame rendering.
		caerVisualizerUpdate(&frame->packetHeader, &state->frameVisualizer);
	}
	else if (!renderFrame) {
		if (atomic_load_explicit(&state->frameVisualizer.running, memory_order_relaxed)) {
			mtx_lock(&state->frameVisualizer.bitmapMutex);
			atomic_store(&state->frameVisualizer.running, false);
			mtx_unlock(&state->frameVisualizer.bitmapMutex);

			caerVisualizerExit(&state->frameVisualizer);
		}
	}

	// Render latest IMU event.
	if (renderIMU && imu != NULL) {
		// If the IMU renderer is not allocated yet, do it.
		if (!atomic_load_explicit(&state->imuVisualizer.running, memory_order_relaxed)) {
			if (!initializeIMURenderer(state)) {
				caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to initialize IMU visualizer.");
				return;
			}
		}

		// Update sub-sample value.
		state->imuVisualizer.packetSubsampleRendering = sshsNodeGetInt(moduleData->moduleNode, "subsampleIMURendering");

		// Actually update IMU rendering.
		caerVisualizerUpdate(&imu->packetHeader, &state->imuVisualizer);
	}
	else if (!renderIMU) {
		if (atomic_load_explicit(&state->imuVisualizer.running, memory_order_relaxed)) {
			mtx_lock(&state->imuVisualizer.bitmapMutex);
			atomic_store(&state->imuVisualizer.running, false);
			mtx_unlock(&state->imuVisualizer.bitmapMutex);

			caerVisualizerExit(&state->imuVisualizer);
		}
	}
}

static int caerVisualizerModuleRenderThread(void *moduleData) {
	caerModuleData data = moduleData;
	visualizerModuleState state = data->moduleState;

	while (atomic_load_explicit(&data->running, memory_order_relaxed)) {
		if (atomic_load_explicit(&state->eventVisualizer.running, memory_order_relaxed)) {
			caerVisualizerUpdateScreen(&state->eventVisualizer);
		}

		if (atomic_load_explicit(&state->frameVisualizer.running, memory_order_relaxed)) {
			caerVisualizerUpdateScreen(&state->frameVisualizer);
		}

		if (atomic_load_explicit(&state->imuVisualizer.running, memory_order_relaxed)) {
			caerVisualizerUpdateScreen(&state->imuVisualizer);
		}
	}

	return (thrd_success);
}

static bool initializeEventRenderer(visualizerModuleState state, int16_t sourceID) {
	// Get size information from source.
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo((uint16_t) sourceID);
	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "dvsSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "dvsSizeY");

	if (!caerVisualizerInit(&state->eventVisualizer, sizeX, sizeY, VISUALIZER_DEFAULT_ZOOM, true)) {
		return (false);
	}

	return (true);
}

static bool initializeFrameRenderer(visualizerModuleState state, int16_t sourceID) {
	// Get size information from source.
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo((uint16_t) sourceID);
	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "apsSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "apsSizeY");

	if (!caerVisualizerInit(&state->frameVisualizer, sizeX, sizeY, VISUALIZER_DEFAULT_ZOOM, true)) {
		return (false);
	}

	return (true);
}

static bool initializeIMURenderer(visualizerModuleState state) {
	if (!caerVisualizerInit(&state->imuVisualizer, 300, 300, VISUALIZER_DEFAULT_ZOOM, true)) {
		return (false);
	}

	return (true);
}
