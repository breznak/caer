/*
 * Mediantracker.c
 *
 *  Created on: Jan 2017
 *      Author: Tianyu
 */

// all caerLog are used to debug

#include "mediantracker.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/buffers.h"
#include "math.h"

struct MTFilter_state {
	float xmedian;
	float ymedian;
	float xstd;
	float ystd;
	float xmean;
	float ymean;
	int lastts;
	int dt;
	int prevlastts;
	float radius;
	float numStdDevsForBoundingBox;
	int tauUs;
};

static const int TICK_PER_MS = 1000;

typedef struct MTFilter_state *MTFilterState;

static bool caerMediantrackerInit(caerModuleData moduleData);
static void caerMediantrackerRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerMediantrackerConfig(caerModuleData moduleData);
static void caerMediantrackerExit(caerModuleData moduleData);
static void caerMediantrackerReset(caerModuleData moduleData, uint16_t resetCallSourceID);

static struct caer_module_functions caerMediantrackerFunctions = { .moduleInit = &caerMediantrackerInit, .moduleRun =
	&caerMediantrackerRun, .moduleConfig = &caerMediantrackerConfig, .moduleExit = &caerMediantrackerExit,
	.moduleReset = &caerMediantrackerReset };

void caerMediantrackerFilter(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "MTFilter", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerMediantrackerFunctions, moduleData, sizeof(struct MTFilter_state), 2, polarity, frame);
}

static bool caerMediantrackerInit(caerModuleData moduleData) {
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "tauUs", 25);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "numStdDevsForBoundingBox", 1.0f);

	MTFilterState state = moduleData->moduleState;

	state->xmedian = 0.0f;
	state->ymedian = 0.0f;
	state->xstd = 0.0f;
	state->ystd = 0.0f;
	state->xmean = 0.0f;
	state->ymean = 0.0f;
	state->lastts = 0;
	state->dt = 0;
	state->prevlastts = 0;
	state->numStdDevsForBoundingBox = sshsNodeGetFloat(moduleData->moduleNode, "numStdDevsForBoundingBox");
	state->radius = 10.0f;
	state->tauUs = sshsNodeGetInt(moduleData->moduleNode, "tauUs");

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	// Nothing that can fail here.
	return (true);
}
static void caerMediantrackerRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket);
	caerFrameEventPacket *frame = va_arg(args, caerFrameEventPacket*);

	// Only process packets with content.
	if (polarity == NULL) {
		return;
	}

	MTFilterState state = moduleData->moduleState;

	int sourceID = caerEventPacketHeaderGetEventSource(&polarity->packetHeader);
	sshsNode sourceInfoNodeCA = caerMainloopGetSourceInfo(sourceID);
	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	if (!sshsNodeAttributeExists(sourceInfoNode, "dataSizeX", SSHS_SHORT)) { //to do for visualizer change name of field to a more generic one
		sshsNodePutShort(sourceInfoNode, "dataSizeX", sshsNodeGetShort(sourceInfoNodeCA, "dvsSizeX"));
		sshsNodePutShort(sourceInfoNode, "dataSizeY", sshsNodeGetShort(sourceInfoNodeCA, "dvsSizeY"));
	}

	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "dataSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "dataSizeY");

	// get the size of the packet
	int n = caerEventPacketHeaderGetEventNumber(&polarity->packetHeader);

	// get last time stamp of the packet
	// update dt and prevlastts
	//int countForLastTime = 0;
	int maxLastTime = 0;
	CAER_POLARITY_ITERATOR_VALID_START(polarity)
		if (maxLastTime < caerPolarityEventGetTimestamp64(caerPolarityIteratorElement, polarity))
			maxLastTime = caerPolarityEventGetTimestamp64(caerPolarityIteratorElement, polarity);
	CAER_POLARITY_ITERATOR_VALID_END
	state->lastts = maxLastTime;
	state->dt = state->lastts - state->prevlastts;
	state->prevlastts = state->lastts;
	if (state->dt < 0)
		state->dt = 0;

	// save position of all events in the packet
	int xs[n];
	int ys[n];
	int index = 0;
	CAER_POLARITY_ITERATOR_VALID_START(polarity)
		xs[index] = caerPolarityEventGetX(caerPolarityIteratorElement);
		ys[index] = caerPolarityEventGetY(caerPolarityIteratorElement);
		index++;
	CAER_POLARITY_ITERATOR_VALID_END

	// get median
	int x, y;
	if (index % 2 != 0) { // odd number of events take middle one
		x = xs[index / 2];
		y = ys[index / 2];
	}
	else { // even number of events take avg of middle two
		x = (float) (((float) xs[index / 2 - 1] + xs[index / 2]) / 2.0f);
		y = (float) (((float) ys[index / 2 - 1] + ys[index / 2]) / 2.0f);
	}

	float fac = (float) state->dt / (float) state->tauUs / (float)TICK_PER_MS;
	if (fac > 1)
		fac = 1;
	state->xmedian = state->xmedian + (x - state->xmedian) * fac;
	state->ymedian = state->ymedian + (y - state->ymedian) * fac;

	// get mean
	int xsum = 0;
	int ysum = 0;
	for (int i = 0; i < index; i++) {
		xsum += xs[i];
		ysum += ys[i];
	}
	state->xmean = state->xmean + (xsum / n - state->xmean) * fac;
	state->ymean = state->ymean + (ysum / n - state->ymean) * fac;

	// get std
	float xvar = 0.0f;
	float yvar = 0.0f;
	float tmp;
	for (int i = 0; i < index; i++) {
		tmp = xs[i] - state->xmean;
		tmp *= tmp;
		xvar += tmp;

		tmp = ys[i] - state->ymean;
		tmp *= tmp;
		yvar += tmp;
	}
	if (index != 0) {
		xvar /= index;
		yvar /= index;
	}
	state->xstd = state->xstd + ((float) sqrt(xvar) - state->xstd) * fac;
	state->ystd = state->ystd + ((float) sqrt(yvar) - state->ystd) * fac;

	*frame = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, sizeX, sizeY, 3);
	if (*frame != NULL) {
		caerFrameEvent singleplot = caerFrameEventPacketGetEvent(*frame, 0);
		uint32_t counter = 0;
		for (size_t yy = 0; yy < sizeY; yy++) {
			for (size_t xx = 0; xx < sizeX; xx++) {
				if ((xx == (int) state->xmedian && yy == (int) state->ymedian)
					|| (xx == (int) (state->xmedian + state->xstd * state->numStdDevsForBoundingBox)
						&& yy <= (state->ymedian + state->ystd * state->numStdDevsForBoundingBox)
						&& yy >= (state->ymedian - state->ystd * state->numStdDevsForBoundingBox))
						|| (xx == (int) (state->xmedian - state->xstd * state->numStdDevsForBoundingBox)
							&& yy <= (state->ymedian + state->ystd * state->numStdDevsForBoundingBox)
							&& yy >= (state->ymedian - state->ystd * state->numStdDevsForBoundingBox))
							|| (yy == (int) (state->ymedian + state->ystd * state->numStdDevsForBoundingBox)
								&& xx <= (state->xmedian + state->xstd * state->numStdDevsForBoundingBox)
								&& xx >= (state->xmedian - state->xstd * state->numStdDevsForBoundingBox))
								|| (yy == (int) (state->ymedian - state->ystd * state->numStdDevsForBoundingBox)
									&& xx <= (state->xmedian + state->xstd * state->numStdDevsForBoundingBox)
									&& xx >= (state->xmedian - state->xstd * state->numStdDevsForBoundingBox))) {
					singleplot->pixels[counter] = (uint16_t) ((int) 1);		// red
					singleplot->pixels[counter + 1] = (uint16_t) ((int) 1);		// green
					singleplot->pixels[counter + 2] = (uint16_t) ((int) 65000);		// blue
				}
				else {
					singleplot->pixels[counter] = (uint16_t) ((int) 1);			// red
					singleplot->pixels[counter + 1] = (uint16_t) ((int) 1);		// green
					singleplot->pixels[counter + 2] = (uint16_t) ((int) 1);	// blue
				}
				counter += 3;
			}
		}
		//add info to the frame
		caerFrameEventSetLengthXLengthYChannelNumber(singleplot, sizeX, sizeY, 3, *frame);
		//validate frame
		caerFrameEventValidate(singleplot, *frame);

		CAER_POLARITY_ITERATOR_VALID_START(polarity)
			int xxx = caerPolarityEventGetX(caerPolarityIteratorElement);
			int yyy = caerPolarityEventGetY(caerPolarityIteratorElement);
			int pol = caerPolarityEventGetPolarity(caerPolarityIteratorElement);
			int address = 3 * (yyy * sizeX + xxx);
			if (pol == 0) {
				singleplot->pixels[address] = 65000; // red
				singleplot->pixels[address + 1] = 1; // green
				singleplot->pixels[address + 2] = 1; // blue
			}
			else {
				singleplot->pixels[address] = 1; // red
				singleplot->pixels[address + 1] = 65000; // green
				singleplot->pixels[address + 2] = 1; // blue
			}
		CAER_POLARITY_ITERATOR_VALID_END

	}
}

static void caerMediantrackerConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	MTFilterState state = moduleData->moduleState;
	state->tauUs = sshsNodeGetInt(moduleData->moduleNode, "tauUs");
	state->numStdDevsForBoundingBox = sshsNodeGetFloat(moduleData->moduleNode, "numStdDevsForBoundingBox");

}

static void caerMediantrackerExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

}

static void caerMediantrackerReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);

}

