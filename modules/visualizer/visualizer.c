#include "visualizer.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "modules/statistics/statistics.h"

#define GLFW_INCLUDE_GLEXT 1
#define GL_GLEXT_PROTOTYPES 1
#include <GLFW/glfw3.h>

#include <GL/glut.h>
#include <GL/freeglut_ext.h>

#define TEXT_SPACING 20

struct visualizer_state {
	GLFWwindow* window;
	uint32_t *eventRenderer;
	uint16_t eventRendererSizeX;
	uint16_t eventRendererSizeY;
	size_t eventRendererSlowDown;
	struct caer_statistics_state eventStatistics;
	uint32_t *frameRenderer;
	uint16_t frameRendererSizeX;
	uint16_t frameRendererSizeY;
	uint16_t frameChannels;
	struct caer_statistics_state frameStatistics;
};

typedef struct visualizer_state *visualizerState;

static bool caerVisualizerInit(caerModuleData moduleData);
static void caerVisualizerRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerVisualizerExit(caerModuleData moduleData);
static bool allocateEventRenderer(visualizerState state, uint16_t sourceID);
static bool allocateFrameRenderer(visualizerState state, uint16_t sourceID);

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
		caerLog(LOG_ERROR, moduleData->moduleSubSystemString, "Failed to initialize GLFW.");
		return (false);
	}

	state->window = glfwCreateWindow(VISUALIZER_SCREEN_WIDTH, VISUALIZER_SCREEN_HEIGHT, "cAER Visualizer", NULL, NULL);
	if (state->window == NULL) {
		caerLog(LOG_ERROR, moduleData->moduleSubSystemString, "Failed to create GLFW window.");
		return (false);
	}

	glfwMakeContextCurrent(state->window);

	glfwSwapInterval(0);

	glClearColor(0, 0, 0, 0);
	glShadeModel(GL_FLAT);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 2);

	// Configuration.
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "showEvents", true);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "showFrames", true);

	// Statistics text.
	char fakeParam[] = "cAER Visualizer";
	char *fakeargv[] = { fakeParam, NULL };
	int fakeargc = 1;

	glutInit(&fakeargc, fakeargv);

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
				caerLog(LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for eventRenderer.");
				return;
			}
		}

		caerPolarityEvent currPolarityEvent;

		for (uint32_t i = 0; i < caerEventPacketHeaderGetEventNumber(&polarity->packetHeader); i++) {
			currPolarityEvent = caerPolarityEventPacketGetEvent(polarity, i);

			// Only operate on valid events!
			if (caerPolarityEventIsValid(currPolarityEvent)) {
				if (caerPolarityEventGetPolarity(currPolarityEvent)) {
					// Green.
					state->eventRenderer[(caerPolarityEventGetY(currPolarityEvent) * state->eventRendererSizeX)
						+ caerPolarityEventGetX(currPolarityEvent)] = be32toh(U32T(0xFF << 16));
				}
				else {
					// Red.
					state->eventRenderer[(caerPolarityEventGetY(currPolarityEvent) * state->eventRendererSizeX)
						+ caerPolarityEventGetX(currPolarityEvent)] = be32toh(U32T(0xFF << 24));
				}
			}
		}

		// Accumulate events over four polarity packets.
		if (state->eventRendererSlowDown++ == 4) {
			state->eventRendererSlowDown = 0;

			memset(state->eventRenderer, 0,
				(size_t) state->eventRendererSizeX * state->eventRendererSizeY * sizeof(uint32_t));
		}
	}

	// Select latest frame to render.
	if (renderFrame && frame != NULL) {
		// If the event renderer is not allocated yet, do it.
		if (state->frameRenderer == NULL) {
			if (!allocateFrameRenderer(state, caerEventPacketHeaderGetEventSource(&frame->packetHeader))) {
				// Failed to allocate memory, nothing to do.
				caerLog(LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for frameRenderer.");
				return;
			}
		}

		caerFrameEvent currFrameEvent;

		for (int64_t i = (int64_t) caerEventPacketHeaderGetEventNumber(&frame->packetHeader) - 1; i >= 0; i--) {
			currFrameEvent = caerFrameEventPacketGetEvent(frame, (uint32_t) i);

			// Only operate on the last valid frame.
			if (caerFrameEventIsValid(currFrameEvent)) {
				// Copy the frame content to the permanent frameRenderer.
				// Use frame sizes to correctly support small ROI frames.
				state->frameRendererSizeX = caerFrameEventGetLengthX(currFrameEvent);
				state->frameRendererSizeY = caerFrameEventGetLengthY(currFrameEvent);
				state->frameChannels = caerFrameEventGetChannelNumber(currFrameEvent);

				memcpy(state->frameRenderer, caerFrameEventGetPixelArrayUnsafe(currFrameEvent),
					(size_t) state->frameRendererSizeX * state->frameRendererSizeY * state->frameChannels
						* sizeof(uint16_t));

				break;
			}
		}
	}

	// All rendering calls at the end.
	// Only execute if something actually changed (packets not null).
	if ((renderPolarity && polarity != NULL) || (renderFrame && frame != NULL)) {
		glClear(GL_COLOR_BUFFER_BIT);

		// Render polarity events.
		if (renderPolarity) {
			// Write statistics text.
			caerStatisticsStringUpdate((caerEventPacketHeader) polarity, &state->eventStatistics);
			glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

			glWindowPos2i(0, (state->eventRendererSizeY * PIXEL_ZOOM) + TEXT_SPACING);
			glutBitmapString(GLUT_BITMAP_HELVETICA_18,
				(const unsigned char *) state->eventStatistics.currentStatisticsString);

			glWindowPos2i(0, (state->eventRendererSizeY * PIXEL_ZOOM) + (2 * TEXT_SPACING));
			glutBitmapString(GLUT_BITMAP_HELVETICA_18, (const unsigned char *) "EVENTS");

			// Position and draw events.
			glWindowPos2i(0, 0);

			glDrawPixels(state->eventRendererSizeX, state->eventRendererSizeY, GL_RGBA, GL_UNSIGNED_BYTE,
				state->eventRenderer);
		}

		// Render latest frame.
		if (renderFrame) {
			// Write statistics text.
			caerStatisticsStringUpdate((caerEventPacketHeader) frame, &state->frameStatistics);
			glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

			// Shift APS frame to the right of the Polarity rendering, if both are enabled.
			if (renderPolarity) {
				glWindowPos2i(0, (state->eventRendererSizeY * PIXEL_ZOOM) + (4 * TEXT_SPACING));
				glutBitmapString(GLUT_BITMAP_HELVETICA_18,
					(const unsigned char *) state->frameStatistics.currentStatisticsString);

				glWindowPos2i(0, (state->eventRendererSizeY * PIXEL_ZOOM) + (5 * TEXT_SPACING));
				glutBitmapString(GLUT_BITMAP_HELVETICA_18, (const unsigned char *) "FRAMES");

				// Position and draw frames after events.
				glWindowPos2i(state->eventRendererSizeX * PIXEL_ZOOM, 0);
			}
			else {
				glWindowPos2i(0, (state->frameRendererSizeX * PIXEL_ZOOM) + TEXT_SPACING);
				glutBitmapString(GLUT_BITMAP_HELVETICA_18,
					(const unsigned char *) state->frameStatistics.currentStatisticsString);

				glWindowPos2i(0, (state->frameRendererSizeX * PIXEL_ZOOM) + (2 * TEXT_SPACING));
				glutBitmapString(GLUT_BITMAP_HELVETICA_18, (const unsigned char *) "FRAMES");

				// Position and draw frames.
				glWindowPos2i(0, 0);
			}

			switch (state->frameChannels) {
				case 3:
					glDrawPixels(state->frameRendererSizeX, state->frameRendererSizeY, GL_RGB, GL_UNSIGNED_SHORT,
						state->frameRenderer);
					break;

				case 4:
					glDrawPixels(state->frameRendererSizeX, state->frameRendererSizeY, GL_RGBA, GL_UNSIGNED_SHORT,
						state->frameRenderer);
					break;

				case 1:
				default:
					glDrawPixels(state->frameRendererSizeX, state->frameRendererSizeY, GL_LUMINANCE,
					GL_UNSIGNED_SHORT, state->frameRenderer);
					break;
			}
		}

		// Apply zoom factor.
		glPixelZoom(PIXEL_ZOOM, PIXEL_ZOOM);

		// Do glfw update.
		glfwSwapBuffers(state->window);
		glfwPollEvents();
	}
}

static bool allocateEventRenderer(visualizerState state, uint16_t sourceID) {
	// Get size information from source.
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo(sourceID);
	uint16_t sizeX = sshsNodeGetShort(sourceInfoNode, "dvsSizeX");
	uint16_t sizeY = sshsNodeGetShort(sourceInfoNode, "dvsSizeY");

	state->eventRenderer = calloc((size_t) (sizeX * sizeY), sizeof(uint32_t));
	if (state->eventRenderer == NULL) {
		return (false); // Failure.
	}

	// Assign max sizes for event renderer.
	state->eventRendererSizeX = sizeX;
	state->eventRendererSizeY = sizeY;

	return (true);
}

static bool allocateFrameRenderer(visualizerState state, uint16_t sourceID) {
	// Get size information from source.
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo(sourceID);
	uint16_t sizeX = sshsNodeGetShort(sourceInfoNode, "apsSizeX");
	uint16_t sizeY = sshsNodeGetShort(sourceInfoNode, "apsSizeY");
	uint16_t channels = sshsNodeGetShort(sourceInfoNode, "apsChannels");

	state->frameRenderer = calloc((size_t) (sizeX * sizeY * channels), sizeof(uint16_t));
	if (state->frameRenderer == NULL) {
		return (false); // Failure.
	}

	// Assign max sizes for frame renderer.
	state->frameRendererSizeX = sizeX;
	state->frameRendererSizeY = sizeY;
	state->frameChannels = channels;

	return (true);
}
