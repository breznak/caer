#include "visualizer.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "modules/statistics/statistics.h"
#include "ext/portable_time.h"

#define GLFW_INCLUDE_GLEXT 1
#define GL_GLEXT_PROTOTYPES 1
#include <GLFW/glfw3.h>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

// We include glut.h so we can know which implementation we're using.
// We only support FreeGLUT currently.
#ifdef FREEGLUT
#include <GL/freeglut_ext.h>
#endif

#define TEXT_SPACING 20 // in pixels

#define SYSTEM_TIMEOUT 10 // in seconds

struct visualizer_state {
	GLFWwindow* window;
	uint32_t *eventRenderer;
	int16_t eventRendererSizeX;
	int16_t eventRendererSizeY;
	size_t eventRendererSlowDown;
	struct caer_statistics_state eventStatistics;
	uint32_t *frameRenderer;
	int32_t frameRendererSizeX;
	int32_t frameRendererSizeY;
	int32_t frameRendererPositionX;
	int32_t frameRendererPositionY;
	enum caer_frame_event_color_channels frameChannels;
	struct caer_statistics_state frameStatistics;
	int16_t subsampleRendering;
	int16_t subsampleCount;
};

typedef struct visualizer_state *visualizerState;

static bool caerVisualizerInit(caerModuleData moduleData);
static void caerVisualizerRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerVisualizerExit(caerModuleData moduleData);
static bool allocateEventRenderer(visualizerState state, int16_t sourceID);
static bool allocateFrameRenderer(visualizerState state, int16_t sourceID);

static struct caer_module_functions caerVisualizerFunctions = { .moduleInit = &caerVisualizerInit, .moduleRun =
	&caerVisualizerRun, .moduleConfig =
NULL, .moduleExit = &caerVisualizerExit };

void caerVisualizer(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "Visualizer");

	caerModuleSM(&caerVisualizerFunctions, moduleData, sizeof(struct visualizer_state), 2, polarity, frame);
}

static bool caerVisualizerInit(caerModuleData moduleData) {
	visualizerState state = moduleData->moduleState;

	if (glfwInit() == GL_FALSE) {
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to initialize GLFW.");
		return (false);
	}

	state->window = glfwCreateWindow(VISUALIZER_SCREEN_WIDTH, VISUALIZER_SCREEN_HEIGHT, "cAER Visualizer", NULL, NULL);
	if (state->window == NULL) {
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to create GLFW window.");
		return (false);
	}

	glfwMakeContextCurrent(state->window);

	glfwSwapInterval(0);

	glClearColor(0, 0, 0, 0);
	glShadeModel(GL_FLAT);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 2);

	// Configuration.
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "showEvents", true);
#ifdef DVS128
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "showFrames", false);
#else
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "showFrames", true);
#endif

	sshsNodePutShortIfAbsent(moduleData->moduleNode, "subsampleRendering", 1);

	state->subsampleRendering = sshsNodeGetShort(moduleData->moduleNode, "subsampleRendering");
	state->subsampleCount = 1;

#ifdef FREEGLUT
	// Statistics text.
	char fakeParam[] = "cAER Visualizer";
	char *fakeargv[] = { fakeParam, NULL };
	int fakeargc = 1;

	caerLog(CAER_LOG_INFO, moduleData->moduleSubSystemString, "Initializing GLUT.");
	glutInit(&fakeargc, fakeargv);
#endif

	if (!caerStatisticsStringInit(&state->eventStatistics)) {
		return (false);
	}

	state->eventStatistics.divisionFactor = 1000;

	if (!caerStatisticsStringInit(&state->frameStatistics)) {
		return (false);
	}

	state->frameStatistics.divisionFactor = 1;

	return (true);
}

static void caerVisualizerExit(caerModuleData moduleData) {
	visualizerState state = moduleData->moduleState;

#ifdef FREEGLUT
	glutExit();
#endif

	glfwDestroyWindow(state->window);

	glfwTerminate();

	// Ensure render maps are freed.
	if (state->eventRenderer != NULL) {
		free(state->eventRenderer);
		state->eventRenderer = NULL;
	}

	if (state->frameRenderer != NULL) {
		free(state->frameRenderer);
		state->frameRenderer = NULL;
	}

	// Statistics text.
	caerStatisticsStringExit(&state->eventStatistics);
	caerStatisticsStringExit(&state->frameStatistics);
}

static void caerVisualizerRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	visualizerState state = moduleData->moduleState;

	// Subsampling, only render every Nth packet.
	if (state->subsampleCount >= state->subsampleRendering) {
		// Polarity events to render.
		caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket);
		bool renderPolarity = sshsNodeGetBool(moduleData->moduleNode, "showEvents");

		// Frames to render.
		caerFrameEventPacket frame = va_arg(args, caerFrameEventPacket);
		bool renderFrame = sshsNodeGetBool(moduleData->moduleNode, "showFrames");

		// Update polarity event rendering map.
		if (renderPolarity && polarity != NULL) {
			// If the event renderer is not allocated yet, do it.
			if (state->eventRenderer == NULL) {
				if (!allocateEventRenderer(state, caerEventPacketHeaderGetEventSource(&polarity->packetHeader))) {
					// Failed to allocate memory, nothing to do.
					caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
						"Failed to allocate memory for eventRenderer.");
					return;
				}
			}

			if (state->subsampleRendering > 1) {
				memset(state->eventRenderer, 0,
					(size_t) (state->eventRendererSizeX * state->eventRendererSizeY) * sizeof(uint32_t));
			}

			CAER_POLARITY_ITERATOR_VALID_START(polarity)
				if (caerPolarityEventGetPolarity(caerPolarityIteratorElement)) {
					// Green.
					state->eventRenderer[(caerPolarityEventGetY(caerPolarityIteratorElement) * state->eventRendererSizeX)
						+ caerPolarityEventGetX(caerPolarityIteratorElement)] = be32toh(U32T(0x00FFUL << 16));
				}
				else {
					// Red.
					state->eventRenderer[(caerPolarityEventGetY(caerPolarityIteratorElement) * state->eventRendererSizeX)
						+ caerPolarityEventGetX(caerPolarityIteratorElement)] = be32toh(U32T(0x00FFUL << 24));
				}CAER_POLARITY_ITERATOR_VALID_END

			// Accumulate events over four polarity packets.
			if (state->subsampleRendering <= 1) {
				if (state->eventRendererSlowDown++ == 4) {
					state->eventRendererSlowDown = 0;

					memset(state->eventRenderer, 0,
						(size_t) (state->eventRendererSizeX * state->eventRendererSizeY) * sizeof(uint32_t));
				}
			}
		}

		// Select latest frame to render.
		if (renderFrame && frame != NULL) {
			// If the event renderer is not allocated yet, do it.
			if (state->frameRenderer == NULL) {
				if (!allocateFrameRenderer(state, caerEventPacketHeaderGetEventSource(&frame->packetHeader))) {
					// Failed to allocate memory, nothing to do.
					caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
						"Failed to allocate memory for frameRenderer.");
					return;
				}
			}

			caerFrameEvent currFrameEvent;

			for (int32_t i = caerEventPacketHeaderGetEventNumber(&frame->packetHeader) - 1; i >= 0; i--) {
				currFrameEvent = caerFrameEventPacketGetEvent(frame, i);

				// Only operate on the last valid frame.
				if (caerFrameEventIsValid(currFrameEvent)) {
					// Copy the frame content to the permanent frameRenderer.
					// Use frame sizes to correctly support small ROI frames.
					state->frameRendererSizeX = caerFrameEventGetLengthX(currFrameEvent);
					state->frameRendererSizeY = caerFrameEventGetLengthY(currFrameEvent);
					state->frameRendererPositionX = caerFrameEventGetPositionX(currFrameEvent);
					state->frameRendererPositionY = caerFrameEventGetPositionY(currFrameEvent);
					state->frameChannels = caerFrameEventGetChannelNumber(currFrameEvent);

					memcpy(state->frameRenderer, caerFrameEventGetPixelArrayUnsafe(currFrameEvent),
						((size_t) state->frameRendererSizeX * (size_t) state->frameRendererSizeY
							* (size_t) state->frameChannels * sizeof(uint16_t)));

					break;
				}
			}
		}

		// Detect if nothing happened for a long time.
		struct timespec currentTime;
		portable_clock_gettime_monotonic(&currentTime);

		uint64_t diffNanoTimeEvents = (uint64_t) (((int64_t) (currentTime.tv_sec
			- state->eventStatistics.lastTime.tv_sec) * 1000000000LL)
			+ (int64_t) (currentTime.tv_nsec - state->eventStatistics.lastTime.tv_nsec));
		bool noEventsTimeout = (diffNanoTimeEvents >= U64T(SYSTEM_TIMEOUT * 1000000000LL));

		uint64_t diffNanoTimeFrames = (uint64_t) (((int64_t) (currentTime.tv_sec
			- state->frameStatistics.lastTime.tv_sec) * 1000000000LL)
			+ (int64_t) (currentTime.tv_nsec - state->frameStatistics.lastTime.tv_nsec));
		bool noFramesTimeout = (diffNanoTimeFrames >= U64T(SYSTEM_TIMEOUT * 1000000000LL));

		// All rendering calls at the end.
		// Only execute if something actually changed (packets not null).
		if ((renderPolarity && (polarity != NULL || noEventsTimeout))
			|| (renderFrame && (frame != NULL || noFramesTimeout))) {
			glClear(GL_COLOR_BUFFER_BIT);

			// Render polarity events.
			if (renderPolarity) {
				// Write statistics text.
				caerStatisticsStringUpdate((caerEventPacketHeader) polarity, &state->eventStatistics);

#ifdef FREEGLUT
				if (noEventsTimeout) {
					glColor4f(1.0f, 0.0f, 0.0f, 1.0f); // RED
				}
				else {
					glColor4f(1.0f, 1.0f, 1.0f, 1.0f); // WHITE
				}

				glWindowPos2i(0, (state->eventRendererSizeY * PIXEL_ZOOM) + TEXT_SPACING);
				glutBitmapString(GLUT_BITMAP_HELVETICA_18,
					(const unsigned char *) state->eventStatistics.currentStatisticsString);

				glWindowPos2i(0, (state->eventRendererSizeY * PIXEL_ZOOM) + (2 * TEXT_SPACING));
				glutBitmapString(GLUT_BITMAP_HELVETICA_18,
					(noEventsTimeout) ? ((const unsigned char *) "NO EVENTS") : ((const unsigned char *) "EVENTS"));
#endif

				// Position and draw events.
				glWindowPos2i(0, 0);

				glDrawPixels(state->eventRendererSizeX, state->eventRendererSizeY, GL_RGBA, GL_UNSIGNED_BYTE,
					state->eventRenderer);
			}

			// Render latest frame.
			if (renderFrame) {
				// Write statistics text.
				caerStatisticsStringUpdate((caerEventPacketHeader) frame, &state->frameStatistics);

#ifdef FREEGLUT
				if (noFramesTimeout) {
					glColor4f(1.0f, 0.0f, 0.0f, 1.0f); // RED
				}
				else {
					glColor4f(1.0f, 1.0f, 1.0f, 1.0f); // WHITE
				}
#endif

				// Shift APS frame to the right of the Polarity rendering, if both are enabled.
				if (renderPolarity) {
#ifdef FREEGLUT
					glWindowPos2i(0, (state->eventRendererSizeY * PIXEL_ZOOM) + (4 * TEXT_SPACING));
					glutBitmapString(GLUT_BITMAP_HELVETICA_18,
						(const unsigned char *) state->frameStatistics.currentStatisticsString);

					glWindowPos2i(0, (state->eventRendererSizeY * PIXEL_ZOOM) + (5 * TEXT_SPACING));
					glutBitmapString(GLUT_BITMAP_HELVETICA_18,
						(noFramesTimeout) ? ((const unsigned char *) "NO FRAMES") : ((const unsigned char *) "FRAMES"));
#endif

					// Position and draw frames after events.
					glWindowPos2i(
						(state->eventRendererSizeX * PIXEL_ZOOM) + (state->frameRendererPositionX * PIXEL_ZOOM),
						(state->frameRendererPositionY * PIXEL_ZOOM));
				}
				else {
#ifdef FREEGLUT
					glWindowPos2i(0, (state->frameRendererSizeX * PIXEL_ZOOM) + TEXT_SPACING);
					glutBitmapString(GLUT_BITMAP_HELVETICA_18,
						(const unsigned char *) state->frameStatistics.currentStatisticsString);

					glWindowPos2i(0, (state->frameRendererSizeX * PIXEL_ZOOM) + (2 * TEXT_SPACING));
					glutBitmapString(GLUT_BITMAP_HELVETICA_18,
						(noFramesTimeout) ? ((const unsigned char *) "NO FRAMES") : ((const unsigned char *) "FRAMES"));
#endif

					// Position and draw frames.
					glWindowPos2i((state->frameRendererPositionX * PIXEL_ZOOM),
						(state->frameRendererPositionY * PIXEL_ZOOM));
				}

				switch (state->frameChannels) {
					case GRAYSCALE:
						glDrawPixels(state->frameRendererSizeX, state->frameRendererSizeY, GL_LUMINANCE,
						GL_UNSIGNED_SHORT, state->frameRenderer);
						break;

					case RGB:
						glDrawPixels(state->frameRendererSizeX, state->frameRendererSizeY, GL_RGB, GL_UNSIGNED_SHORT,
							state->frameRenderer);
						break;

					case RGBA:
						glDrawPixels(state->frameRendererSizeX, state->frameRendererSizeY, GL_RGBA, GL_UNSIGNED_SHORT,
							state->frameRenderer);
						break;
				}
			}

			// Apply zoom factor.
			glPixelZoom(PIXEL_ZOOM, PIXEL_ZOOM);

			// Do glfw update.
			glfwSwapBuffers(state->window);
			glfwPollEvents();
		}

		state->subsampleCount = 1;
	}
	else {
		state->subsampleCount++;
	}
}

static bool allocateEventRenderer(visualizerState state, int16_t sourceID) {
	// Get size information from source.
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo((uint16_t) sourceID);
	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "dvsSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "dvsSizeY");

	state->eventRenderer = calloc((size_t) (sizeX * sizeY), sizeof(uint32_t));
	if (state->eventRenderer == NULL) {
		return (false); // Failure.
	}

	// Assign maximum sizes for event renderer.
	state->eventRendererSizeX = sizeX;
	state->eventRendererSizeY = sizeY;

	return (true);
}

static bool allocateFrameRenderer(visualizerState state, int16_t sourceID) {
	// Get size information from source.
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo((uint16_t) sourceID);
	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "apsSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "apsSizeY");

	state->frameRenderer = calloc((size_t) (sizeX * sizeY * MAX_CHANNELS), sizeof(uint16_t));
	if (state->frameRenderer == NULL) {
		return (false); // Failure.
	}

	// Assign maximum sizes and defaults for frame renderer.
	state->frameRendererSizeX = sizeX;
	state->frameRendererSizeY = sizeY;
	state->frameRendererPositionX = 0;
	state->frameRendererPositionY = 0;
	state->frameChannels = MAX_CHANNELS;

	return (true);
}
