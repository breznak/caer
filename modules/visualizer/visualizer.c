#include "visualizer.h"
#include "base/mainloop.h"
#include "base/module.h"
#include <GLFW/glfw3.h>

const char *polarityShader = "#version 330 core"
	""
	"";

struct visualizer_state {
	GLFWwindow* window;
	uint32_t eventRenderer[CHIP_X * CHIP_Y];
	size_t slowDown;
	GLuint glPBOFrameID;
	GLuint glTextureFrameID;
	GLuint glPBOPolarityID;
	GLuint glTexturePolarityID;
};

typedef struct visualizer_state *visualizerState;

static bool caerVisualizerInit(caerModuleData moduleData);
static void caerVisualizerRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerVisualizerExit(caerModuleData moduleData);

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

	// Initialize OpenGL objects.
	glGenBuffers(1, &state->glPBOPolarityID);
	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, state->glPBOPolarityID);
	glBufferData(GL_PIXEL_UNPACK_BUFFER, CHIP_X * CHIP_Y * 4, NULL, GL_DYNAMIC_DRAW);
	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);

	glGenTextures(1, &state->glTexturePolarityID);
	glBindTexture(GL_TEXTURE_2D, state->glTexturePolarityID);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, CHIP_X, CHIP_Y, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
	glBindTexture(GL_TEXTURE_2D, 0);

	state->slowDown = 0;

	memset(state->eventRenderer, 0, CHIP_X * CHIP_Y * sizeof(uint32_t));

	// Configuration.
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "showEvents", true);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "showFrames", false);

	return (true);
}

static void caerVisualizerExit(caerModuleData moduleData) {
	visualizerState state = moduleData->moduleState;

	glfwDestroyWindow(state->window);

	glfwTerminate();
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

	if (renderPolarity && !renderFrame && polarity != NULL) {
		caerPolarityEvent currPolarityEvent;

		for (uint32_t i = 0; i < caerEventPacketHeaderGetEventNumber(&polarity->packetHeader); i++) {
			currPolarityEvent = caerPolarityEventPacketGetEvent(polarity, i);

			// Only operate on valid events!
			if (caerPolarityEventIsValid(currPolarityEvent)) {
				if (caerPolarityEventGetPolarity(currPolarityEvent)) {
					// Green.
					state->eventRenderer[(caerPolarityEventGetY(currPolarityEvent) * CHIP_X)
						+ caerPolarityEventGetX(currPolarityEvent)] = be32toh(U32T(0xFF << 16));
				}
				else {
					// Red.
					state->eventRenderer[(caerPolarityEventGetY(currPolarityEvent) * CHIP_X)
						+ caerPolarityEventGetX(currPolarityEvent)] = be32toh(U32T(0xFF << 24));
				}
			}
		}

		if (state->slowDown++ == 4) {
			state->slowDown = 0;

			glClear(GL_COLOR_BUFFER_BIT);

			glDrawPixels(CHIP_X, CHIP_Y, GL_RGBA, GL_UNSIGNED_BYTE, state->eventRenderer);
			glPixelZoom(PIXEL_ZOOM, PIXEL_ZOOM);

			glfwSwapBuffers(state->window);
			glfwPollEvents();

			memset(state->eventRenderer, 0, CHIP_X * CHIP_Y * sizeof(uint32_t));
		}
	}

	if (!renderPolarity && renderFrame && frame != NULL) {
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
				glPixelZoom(PIXEL_ZOOM, PIXEL_ZOOM);

				glfwSwapBuffers(state->window);
				glfwPollEvents();
			}
		}
	}
}
