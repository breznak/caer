/*
 *  accumulates a fixed number of events and generates frames
 *  federico.corradi@inilabs.com
 */
#include <limits.h>
#include <float.h>
#include "imagegenerator.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "modules/statistics/statistics.h"
#include "ext/portable_time.h"
#include "ext/buffers.h"
#include <string.h>
#include <stdio.h>

//#define TIMING
#ifdef TIMING
#include <time.h>
#endif

#include "main.h"
#include <libcaer/events/polarity.h>

struct imagegenerator_state {
	bool rectifyPolarities;
	int colorscale;
	//image matrix
	int64_t **ImageMap;
	int32_t numSpikes; // after how many spikes will we generate an image
	int32_t spikeCounter; // actual number of spikes seen so far, in range [0, numSpikes]
	int16_t sizeX;
	int16_t sizeY;
};

typedef struct imagegenerator_state *imagegeneratorState;

static bool caerImageGeneratorInit(caerModuleData moduleData);
static void caerImageGeneratorRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerImageGeneratorExit(caerModuleData moduleData);
static bool allocateImageMapSubsampled(imagegeneratorState state, int16_t sourceID, int16_t sizeX, int16_t sizeY);
static void caerImageGeneratorConfig(caerModuleData moduleData);

static struct caer_module_functions caerImageGeneratorFunctions = { .moduleInit = &caerImageGeneratorInit, .moduleRun =
	&caerImageGeneratorRun, .moduleConfig =
NULL, .moduleExit = &caerImageGeneratorExit };

void caerImageGenerator(uint16_t moduleID, caerPolarityEventPacket polarity, int classify_img_size, int *packet_hist,
	bool * haveimg) {

	caerModuleData moduleData = caerMainloopFindModule(moduleID, "ImageGenerator", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerImageGeneratorFunctions, moduleData, sizeof(struct imagegenerator_state), 4, polarity,
		classify_img_size, packet_hist, haveimg);

	return;
}

static bool caerImageGeneratorInit(caerModuleData moduleData) {

	// Ensure numSpikes is set.
	imagegeneratorState state = moduleData->moduleState;
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "numSpikes", 2000);
	state->numSpikes = sshsNodeGetInt(moduleData->moduleNode, "numSpikes");

	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "rectifyPolarities", true);
	state->rectifyPolarities = sshsNodeGetBool(moduleData->moduleNode, "rectifyPolarities");

	sshsNodePutIntIfAbsent(moduleData->moduleNode, "colorscale", 200);
	state->colorscale = sshsNodeGetInt(moduleData->moduleNode, "colorscale");

	state->ImageMap = NULL;

	return (true);
}

static void caerImageGeneratorConfig(caerModuleData moduleData) {

	// Ensure numSpikes is set.
	imagegeneratorState state = moduleData->moduleState;

	state->numSpikes = sshsNodeGetInt(moduleData->moduleNode, "numSpikes");
	state->rectifyPolarities = sshsNodeGetBool(moduleData->moduleNode, "rectifyPolarities");
	state->colorscale = sshsNodeGetInt(moduleData->moduleNode, "colorscale");

}

static void caerImageGeneratorExit(caerModuleData moduleData) {
	imagegeneratorState state = moduleData->moduleState;
	return;
}

//This function implement 3sigma normalization and converts the image in nullhop format
static bool normalize_image_map_sigma(imagegeneratorState state, int * hist, int size) {

	int sum = 0, count = 0;
	for (int i = 0; i < size; i++) {
		for (int j = 0; j < size; j++) {
			if (state->ImageMap[i][j] != 0) {
				sum += state->ImageMap[i][j];
				count++;
			}
		}
	}

	float mean = sum / count;

	float var = 0;
	for (int i = 0; i < size; i++) {
		for (int j = 0; j < size; j++) {
			if (state->ImageMap[i][j] != 0) {
				float f = state->ImageMap[i][j] - mean;
				var += f * f;
			}
		}
	}

	float sig = sqrt(var / count);
	if (sig < (0.1f / 255.0f)) {
		sig = 0.1f / 255.0f;
	}

	float numSDevs = 3;
	float mean_png_gray, range, halfrange;

	if (state->rectifyPolarities) {
		mean_png_gray = 0; // rectified
	}
	else {
		mean_png_gray = (127.0 / 255.0) * 256.0f; //256 included here for nullhop reshift
	}
	if (state->rectifyPolarities) {
		range = numSDevs * sig * (1.0f / 256.0f); //256 included here for nullhop reshift
		halfrange = 0;
	}
	else {
		range = numSDevs * sig * 2 * (1.0f / 256.0f); //256 included here for nullhop reshift
		halfrange = numSDevs * sig;
	}

	for (int col_idx = 0; col_idx < size; col_idx++) {
		for (int row_idx = 0; row_idx < size; row_idx++) {

			int linindex = row_idx * size + col_idx; //we have to increase column as fastest idx, then row in order to have a zs compatible system

			if (state->ImageMap[col_idx][row_idx] == 0) {

				hist[linindex] = mean_png_gray;

			}
			else {
				float f = (state->ImageMap[col_idx][row_idx] + halfrange) / range;

				if (f > 256) {
					f = 256; //256 included here for nullhop reshift
				}
				else if (f < 0) {
					f = 0;
				}

				hist[linindex] = floor(f); //shift by 256 included in previous computations
			}
		}
	}

	return (true);

}

static void caerImageGeneratorRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket);
	int CLASSIFY_IMG_SIZE = va_arg(args, int);
	int * hist = va_arg(args, int*);
	bool * haveimg = va_arg(args, bool*);
	haveimg[0] = false;

	// Only process packets with content.
	if (polarity == NULL) {
		return;
	}

	int sourceID = caerEventPacketHeaderGetEventSource(&polarity->packetHeader);
	sshsNode sourceInfoNodeCA = caerMainloopGetSourceInfo(sourceID);
	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	if (!sshsNodeAttributeExists(sourceInfoNode, "dataSizeX", SSHS_SHORT)) { //to do for visualizer change name of field to a more generic one
		sshsNodePutShort(sourceInfoNode, "dataSizeX", CLASSIFY_IMG_SIZE);
		sshsNodePutShort(sourceInfoNode, "dataSizeY", CLASSIFY_IMG_SIZE);
	}

	//update module state
	imagegeneratorState state = moduleData->moduleState;

	/* **** SPIKE SECTION START *** */
	// If the map is not allocated yet, do it.
	if (state->ImageMap == NULL) {
		if (!allocateImageMapSubsampled(state, caerEventPacketHeaderGetEventSource(&polarity->packetHeader),
			CLASSIFY_IMG_SIZE, CLASSIFY_IMG_SIZE)) {
			// Failed to allocate memory, nothing to do.
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for ImageMap.");
			return;
		}
	}

	if (polarity != NULL) {

		float cam_sizeX = sshsNodeGetShort(sourceInfoNodeCA, "dvsSizeX");
		float cam_sizeY = sshsNodeGetShort(sourceInfoNodeCA, "dvsSizeY");

		if (cam_sizeX == 0 || cam_sizeY == 0) {
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
				"Wrong device size not computing anything in here..");
			return;
		}

		float res_x = CLASSIFY_IMG_SIZE / cam_sizeX;
		float res_y = CLASSIFY_IMG_SIZE / cam_sizeY;

		if (state->colorscale <= 0) {
			caerLog(CAER_LOG_CRITICAL, "imagegenerator", "please select colorscale >0");
			exit(1);
		}

		caerEventPacketHeader *onlyvalid = caerEventPacketCopyOnlyValidEvents(polarity);
		if (onlyvalid == NULL) {
			return;
		}
		int32_t numtotevs = caerEventPacketHeaderGetEventValid(onlyvalid);

		caerPolarityEventPacket currPolarityPacket = (caerPolarityEventPacket) onlyvalid;

		//default is all events
		int numevs_start = 0;
		int numevs_end = numtotevs;

		if (numtotevs >= state->numSpikes) {
			//get rid of accumulated spikes
			for (int col_idx = 0; col_idx < state->sizeX; col_idx++) {
				for (int row_idx = 0; row_idx < state->sizeY; row_idx++) {
					state->ImageMap[col_idx][row_idx] = 0;
				}
			}
			state->spikeCounter = 0;
			//takes only the last 2000
			numevs_start = numtotevs - state->numSpikes;
			numevs_end = numtotevs;

		}
		else if ((numtotevs + state->spikeCounter) >= state->numSpikes) {
			//takes only the last 2000
			numevs_start = numtotevs - (state->numSpikes - state->spikeCounter);
			numevs_end = numtotevs;
		}

		for (int32_t caerPolarityIteratorCounter = numevs_start; caerPolarityIteratorCounter < numevs_end;
			caerPolarityIteratorCounter++) {

			caerPolarityEvent caerPolarityIteratorElement = caerPolarityEventPacketGetEvent(currPolarityPacket,
				caerPolarityIteratorCounter);

			if (!caerPolarityEventIsValid(caerPolarityIteratorElement)) {
				continue;
			} // Skip invalid polarity events.

			// Get coordinates and polarity (0 or 1) of latest spike.
			uint16_t x = caerPolarityEventGetX(caerPolarityIteratorElement);
			uint16_t y = caerPolarityEventGetY(caerPolarityIteratorElement);
			int pol = caerPolarityEventGetPolarity(caerPolarityIteratorElement);

			uint16_t pos_x = floor(res_x * x);
			uint16_t pos_y = floor(res_y * y);

			//Update image Map
			if (state->rectifyPolarities) {
				state->ImageMap[pos_x][pos_y] = state->ImageMap[pos_x][pos_y] + 1; //rectify events
			}
			else {
				if (pol == 0) {
					state->ImageMap[pos_x][pos_y] = state->ImageMap[pos_x][pos_y] - 1;
				}
				else {
					state->ImageMap[pos_x][pos_y] = state->ImageMap[pos_x][pos_y] + 1;
				}
			}

			if (state->ImageMap[pos_x][pos_y] > state->colorscale) {
				state->ImageMap[pos_x][pos_y] = state->colorscale;
			}
			else if (state->ImageMap[pos_x][pos_y] < -state->colorscale) {
				state->ImageMap[pos_x][pos_y] = -state->colorscale;
			}

			state->spikeCounter += 1;

			//If we saw enough spikes, generate Image from ImageMap.
			if (state->spikeCounter >= state->numSpikes) {

				// printf("generating image with numspikes %d\n", state->numSpikes);
				haveimg[0] = true; // we got an image to classify

#ifdef TIMING
				clock_t start = clock(), diff;
#endif

				if (!normalize_image_map_sigma(state, hist, CLASSIFY_IMG_SIZE)) {
					caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
						"Failed to normalize image map with 3 sigma range.");
					return;
				};
#ifdef TIMING
				diff = clock() - start;

				double time_spent = (double)( diff ) / CLOCKS_PER_SEC;
				caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Time taken %f second\n ", time_spent);
#endif

				//reset values
				for (int col_idx = 0; col_idx < state->sizeX; col_idx++) {
					for (int row_idx = 0; row_idx < state->sizeY; row_idx++) {
						state->ImageMap[col_idx][row_idx] = 0;
					}
				}
				state->spikeCounter = 0;

			}
		} //CAER_POLARITY_ITERATOR_VALID_END
		free(onlyvalid);
	}/* **** SPIKE SECTION END *** */

	caerImageGeneratorConfig(moduleData);

}

void caerImageGeneratorAddText(uint16_t moduleID, int * hist_packet, caerFrameEventPacket *imagegeneratorFrame,
	int size) {

	caerModuleData moduleData = caerMainloopFindModule(moduleID, "ImageGenerator", CAER_MODULE_PROCESSOR);

	// put info into frame
	if (*imagegeneratorFrame != NULL) {
		caerFrameEvent singleplot = caerFrameEventPacketGetEvent(*imagegeneratorFrame, 0);

		uint32_t counter = 0;
		for (size_t x = 0; x < size; x++) {
			for (size_t y = 0; y < size; y++) {
				int linindex = x * size + y;
				singleplot->pixels[counter] = (uint16_t) ((int) (hist_packet[linindex]) * 255); // red
				singleplot->pixels[counter + 1] = (uint16_t) ((int) (hist_packet[linindex]) * 255); // green
				singleplot->pixels[counter + 2] = (uint16_t) ((int) (hist_packet[linindex]) * 255); // blue
				counter += 3;
			}
		}
		//add info to the frame
		caerFrameEventSetLengthXLengthYChannelNumber(singleplot, size, size, 3, *imagegeneratorFrame);
	}

}

void caerImageGeneratorMakeFrame(uint16_t moduleID, int * hist_packet, caerFrameEventPacket *imagegeneratorFrame,
	int size) {

	caerModuleData moduleData = caerMainloopFindModule(moduleID, "ImageGenerator", CAER_MODULE_PROCESSOR);

	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	if (!sshsNodeAttributeExists(sourceInfoNode, "dataSizeX", SSHS_SHORT)) { //to do for visualizer change name of field to a more generic one
		sshsNodePutShort(sourceInfoNode, "dataSizeX", size);
		sshsNodePutShort(sourceInfoNode, "dataSizeY", size);
	}

	// put info into frame
	*imagegeneratorFrame = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, size, size, 3);
	caerMainloopFreeAfterLoop(&free, *imagegeneratorFrame);
	if (*imagegeneratorFrame != NULL) {
		caerFrameEvent singleplot = caerFrameEventPacketGetEvent(*imagegeneratorFrame, 0);

		uint32_t counter = 0;
		for (size_t x = 0; x < size; x++) {
			for (size_t y = 0; y < size; y++) {
				//COLOUR col  = GetColour((double) state->frequencyMap->buffer2d[y][x], state->colorscaleMin, state->colorscaleMax);
				int linindex = x * size + y;
				singleplot->pixels[counter] = (uint16_t) ((int) (hist_packet[linindex]) * 255); // red
				singleplot->pixels[counter + 1] = (uint16_t) ((int) (hist_packet[linindex]) * 255); // green
				singleplot->pixels[counter + 2] = (uint16_t) ((int) (hist_packet[linindex]) * 255); // blue
				counter += 3;
			}
		}

		//add info to the frame
		caerFrameEventSetLengthXLengthYChannelNumber(singleplot, size, size, 3, *imagegeneratorFrame);
		//validate frame
		caerFrameEventValidate(singleplot, *imagegeneratorFrame);
	}

}

static bool allocateImageMapSubsampled(imagegeneratorState state, int16_t sourceID, int16_t sizeX, int16_t sizeY) {
	// Get size information from source.
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo(U16T(sourceID));
	if (sourceInfoNode == NULL) {
		// This should never happen, but we handle it gracefully.
		caerLog(CAER_LOG_ERROR, __func__, "Failed to get source info to allocate image map.");
		return (false);
	}

	// Initialize double-indirection contiguous 2D array, so that array[x][y]
	// is possible, see http://c-faq.com/aryptr/dynmuldimary.html for info.
	state->ImageMap = calloc((size_t) sizeX, sizeof(int64_t *));
	if (state->ImageMap == NULL) {
		return (false); // Failure.
	}

	state->ImageMap[0] = calloc((size_t) (sizeX * sizeY), sizeof(int64_t));
	if (state->ImageMap[0] == NULL) {
		free(state->ImageMap);
		state->ImageMap = NULL;

		return (false); // Failure.
	}

	for (size_t i = 1; i < (size_t) sizeX; i++) {
		state->ImageMap[i] = state->ImageMap[0] + (i * (size_t) sizeY);
	}

	// Assign max ranges for arrays (0 to MAX-1).
	state->sizeX = sizeX;
	state->sizeY = sizeY;

	// Init counters
	state->spikeCounter = 0;

	return (true);
}
