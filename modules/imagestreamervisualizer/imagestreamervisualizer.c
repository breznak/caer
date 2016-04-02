/*
 *  accumulates a fixed number of events and generates png pictures
 *  it also displays png images when available
 *  federico.corradi@inilabs.com
 */

#include "imagestreamervisualizer.h"
#include "base/mainloop.h"
#include "base/module.h"

static bool caerImagestreamerVisualizerInit(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerImagestreamerVisualizerRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerImagestreamerVisualizerExit(caerModuleData moduleData);

static struct caer_module_functions caerImagestreamerVisualizerFunctions = { .moduleInit =
	&caerImagestreamerVisualizerInit, .moduleRun = &caerImagestreamerVisualizerRun, .moduleConfig =
NULL, .moduleExit = &caerImagestreamerVisualizerExit };

void caerImagestreamerVisualizer(uint16_t moduleID, unsigned char * disp_img, const int disp_img_size,
	double * classific_results, int * classific_sizes, int max_img_qty) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "ImageStreamerVisualizer");

	caerModuleSM(&caerImagestreamerVisualizerFunctions, moduleData, sizeof(struct imagestreamervisualizer_state), 5,
		disp_img, disp_img_size, classific_results, classific_sizes, max_img_qty);
}

static bool caerImagestreamerVisualizerInit(caerModuleData moduleData, size_t argsNumber, va_list args) {
	caerImagestreamerVisualizerState state = moduleData->moduleState;


	if (!caerVisualizerInit(&state->vis_state, IMAGESTREAMERVISUALIZER_SCREEN_WIDTH,
	IMAGESTREAMERVISUALIZER_SCREEN_HEIGHT, VISUALIZER_DEFAULT_ZOOM, false)) {
		return (false);
	}

	// Start separate rendering thread. Decouples presentation from
	// data processing and preparation. Communication over properly
	// locked bitmap.
	if (thrd_create(&state->renderingThread, &caerVisualizerRenderThread, &state->vis_state) != thrd_success) {
		return (false);
	}

	return (true);
}

static void caerImagestreamerVisualizerExit(caerModuleData moduleData) {
	caerImagestreamerVisualizerState state = moduleData->moduleState;

	// Shut down rendering threads and wait on them.
	if (atomic_load(&state->vis_state.running)) {
		atomic_store(&state->vis_state.running, false);
		thrd_join(state->renderingThread, NULL);
	}
}

static void caerImagestreamerVisualizerRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	caerImagestreamerVisualizerState state = moduleData->moduleState;

	// Interpret variable arguments (same as above in main function).
	unsigned char * disp_img = va_arg(args, char *);
	const int DISPLAY_IMG_SIZE = va_arg(args, const int);
	double * classific_results = va_arg(args, double *);
	int * classific_sizes = va_arg(args, int *);
	int max_img_qty = va_arg(args, int);

	//create one frame event packet, one single frame
	caerFrameEventPacket my_frame_packet = caerFrameEventPacketAllocate(1, moduleData->moduleID, 0,
	IMAGESTREAMERVISUALIZER_SCREEN_WIDTH, IMAGESTREAMERVISUALIZER_SCREEN_HEIGHT, 3);
	//get that single frame
	caerFrameEvent my_frame = caerFrameEventPacketGetEvent(my_frame_packet, 0);

	// only proceed if we have a display image
	if (disp_img == NULL) {
		return;
	}

	unsigned char *small_img = disp_img;
	//now put stuff into the frame
	int c, counter;
	counter = 0;
	for (int i = 0; i < IMAGESTREAMERVISUALIZER_SCREEN_WIDTH; i++) {
		for (int y = 0; y < IMAGESTREAMERVISUALIZER_SCREEN_HEIGHT; y++) {
			c = i * DISPLAY_IMG_SIZE + y;
			//depending on results display image with different color
			if(*classific_results == 0){
				my_frame->pixels[counter] = (uint16_t) (small_img[c] << 8);
				my_frame->pixels[counter+1] = (uint16_t) (small_img[c] << 8);
				my_frame->pixels[counter+2] = (uint16_t) (small_img[c] << 8);
			}else{
				my_frame->pixels[counter] = (uint16_t) (0);
				my_frame->pixels[counter+1] = (uint16_t) ( (int)(small_img[c]* *classific_results) << 8);
				my_frame->pixels[counter+2] = (uint16_t) (0);
			}
			counter += 3;
		}
	}

	//add info to the frame
	caerFrameEventSetLengthXLengthYChannelNumber(my_frame, IMAGESTREAMERVISUALIZER_SCREEN_WIDTH,
	IMAGESTREAMERVISUALIZER_SCREEN_HEIGHT, 3, my_frame_packet);
	//valido
	caerFrameEventValidate(my_frame, my_frame_packet);
	//update bitmap
	caerVisualizerUpdate(&my_frame_packet->packetHeader, &state->vis_state);

	free(my_frame_packet);
}
