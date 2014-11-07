#include "visualizer.h"
#include "base/mainloop.h"
#include "base/module.h"

struct visualizer_state {
};

typedef struct visualizer_state *visualizerState;

static void caerVisualizerRun(caerModuleData moduleData, size_t argsNumber,
		va_list args);

static struct caer_module_functions caerVisualizerFunctions =
		{ .moduleInit = NULL, .moduleRun = &caerVisualizerRun, .moduleConfig = NULL, .moduleExit = NULL };

void caerVisualizer(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "Visualizer");

	caerModuleSM(&caerVisualizerFunctions, moduleData,
			sizeof(struct visualizer_state), 2, polarity, frame);
}

static void caerVisualizerRun(caerModuleData moduleData, size_t argsNumber,
		va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Polarity events to render.
	caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket);

	// Frames to render.
	caerFrameEventPacket frame = va_arg(args, caerFrameEventPacket);

}
