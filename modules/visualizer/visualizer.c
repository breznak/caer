#include "visualizer.h"
#include "base/mainloop.h"
#include "base/module.h"
#include <GLFW/glfw3.h>

struct visualizer_state {
	GLFWwindow* window;
};

typedef struct visualizer_state *visualizerState;

static bool caerVisualizerInit(caerModuleData moduleData);
static void caerVisualizerRun(caerModuleData moduleData, size_t argsNumber,
		va_list args);
static void caerVisualizerExit(caerModuleData moduleData);

static struct caer_module_functions caerVisualizerFunctions = { .moduleInit =
		&caerVisualizerInit, .moduleRun = &caerVisualizerRun, .moduleConfig =
		NULL, .moduleExit = &caerVisualizerExit };

void caerVisualizer(uint16_t moduleID, caerPolarityEventPacket polarity,
		caerFrameEventPacket frame) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "Visualizer");

	caerModuleSM(&caerVisualizerFunctions, moduleData,
			sizeof(struct visualizer_state), 2, polarity, frame);
}

static bool caerVisualizerInit(caerModuleData moduleData) {
	visualizerState state = moduleData->moduleState;

	if (glfwInit() == GL_FALSE) {
		caerLog(LOG_ERROR, "Failed to initialize GLFW.");
		return (false);
	}

	state->window = glfwCreateWindow(VISUALIZER_SCREEN_WIDTH, VISUALIZER_SCREEN_HEIGHT, "cAER Visualizer", NULL, NULL);
	if (state->window == NULL) {
		caerLog(LOG_ERROR, "Failed to create GLFW window.");
		return (false);
	}

	glfwMakeContextCurrent(state->window);

	glClearColor(0, 0, 0, 0);
	glShadeModel(GL_FLAT);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 2);

	glDisable(GL_ALPHA_TEST);
	glDisable(GL_BLEND);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_DITHER);
	glDisable(GL_FOG);
	glDisable(GL_LIGHTING);
	glDisable(GL_LOGIC_OP);
	glDisable(GL_STENCIL_TEST);
	glDisable(GL_TEXTURE_1D);
	glDisable(GL_TEXTURE_2D);

	return (true);
}

static void caerVisualizerExit(caerModuleData moduleData) {
	visualizerState state = moduleData->moduleState;

	glfwDestroyWindow(state->window);

	glfwTerminate();
}

static void caerVisualizerRun(caerModuleData moduleData, size_t argsNumber,
		va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	visualizerState state = moduleData->moduleState;

	// Polarity events to render.
	caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket);

	// Frames to render.
	caerFrameEventPacket frame = va_arg(args, caerFrameEventPacket);

	if (frame != NULL) {
		// Render frames one by one.
		caerFrameEvent currFrameEvent;

		for (uint32_t i = 0; i < caerEventPacketHeaderGetEventNumber(&frame->packetHeader); i++) {
			currFrameEvent = caerFrameEventPacketGetEvent(frame, i);

			// Only operate on valid events!
			if (caerFrameEventIsValid(currFrameEvent)) {
				glClear(GL_COLOR_BUFFER_BIT);

				//glRasterPos2i(0, caerFrameEventGetLengthY(currFrameEvent) - 1);
				//glPixelZoom(1.0f, -1.0f);

				glDrawPixels(caerFrameEventGetLengthX(currFrameEvent), caerFrameEventGetLengthY(currFrameEvent),
					GL_LUMINANCE, GL_UNSIGNED_SHORT, caerFrameEventGetPixelArrayUnsafe(currFrameEvent));
				glPixelZoom(4, 4);

				glfwSwapBuffers(state->window);
				glfwPollEvents();
			}
		}
	}
}
