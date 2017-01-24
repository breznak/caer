/*
 * Meanfilter.c
 *
 *  Created on: Jan 2017
 *      Author: Tianyu
 */

#include "meanfilter.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/buffers.h"
#include "math.h"

struct MeanFilter_state {
	//simple2DBufferFloat medianPoint;
	//simple2DBufferFloat stdPoint;
	//simple2DBufferFloat meanPoint;
//	float xmedian;
//	float ymedian;
//	float xstd;
//	float ystd;
	float xmean;
	float ymean;
//	int lastts;
//	int dt;
//	int prevlastts;
//	LowpassFilter xFilter;
//	LowpassFilter yFilter;
//	LowpassFilter xStdFilter;
//	LowpassFilter yStdFilter;
//	LowpassFilter xMeanFilter;
//	LowpassFilter yMeanFilter;
//	int tauUs;
//	float numStdDevsForBoundingBox;
	float alpha;
	float radius;
//	float beta;
};



typedef struct MeanFilter_state *MeanFilterState;

static bool caerMeanfilterInit(caerModuleData moduleData);
static void caerMeanfilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerMeanfilterConfig(caerModuleData moduleData);
static void caerMeanfilterExit(caerModuleData moduleData);
static void caerMeanfilterReset(caerModuleData moduleData, uint16_t resetCallSourceID);


static struct caer_module_functions caerMeanfilterFunctions = { .moduleInit =
	&caerMeanfilterInit, .moduleRun = &caerMeanfilterRun, .moduleConfig =
	&caerMeanfilterConfig, .moduleExit = &caerMeanfilterExit, .moduleReset =
	&caerMeanfilterReset };

void caerMeanfilterFilter(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "MeanFilter", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerMeanfilterFunctions, moduleData, sizeof(struct MeanFilter_state), 2, polarity, frame);
}

static bool caerMeanfilterInit(caerModuleData moduleData) {
//	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "xmedian", 0.0f);
//	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "ymedian", 0.0f);
//	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "xstd", 0.0f);
//	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "ystd", 0.0f);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "xmean", 0.0f);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "ymean", 0.0f);
//	sshsNodePutIntIfAbsent(moduleData->moduleNode, "lastts", 0);
//	sshsNodePutIntIfAbsent(moduleData->moduleNode, "dt", 0);
//	sshsNodePutIntIfAbsent(moduleData->moduleNode, "prevlastts", 0);
//	sshsNodePutIntIfAbsent(moduleData->moduleNode, "tauUs", 1000);
//	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "numStdDevsForBoundingBox", 1.0f);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "alpha", 0.01f);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "radius", 10.0f);

	MeanFilterState state = moduleData->moduleState;


//	state->xmedian = sshsNodeGetFloat(moduleData->moduleNode, "xmedian");
//	state->ymedian = sshsNodeGetFloat(moduleData->moduleNode, "ymedian");
//	state->xstd = sshsNodeGetFloat(moduleData->moduleNode, "xstd");
//	state->ystd = sshsNodeGetFloat(moduleData->moduleNode, "ystd");
	state->xmean = sshsNodeGetFloat(moduleData->moduleNode, "xmean");
	state->ymean = sshsNodeGetFloat(moduleData->moduleNode, "ymean");
//	state->lastts = sshsNodeGetInt(moduleData->moduleNode, "lastts");
//	state->dt = sshsNodeGetInt(moduleData->moduleNode, "dt");
//	state->prevlastts = sshsNodeGetInt(moduleData->moduleNode, "prevlastts");
//	state->tauUs = sshsNodeGetInt(moduleData->moduleNode, "tauUs");
//	state->numStdDevsForBoundingBox = sshsNodeGetFloat(moduleData->moduleNode, "numStdDevsForBoundingBox");
	state->alpha = sshsNodeGetFloat(moduleData->moduleNode, "alpha");
	state->radius = sshsNodeGetFloat(moduleData->moduleNode, "radius");

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	if (!sshsNodeAttributeExists(sourceInfoNode, "dataSizeX", SSHS_SHORT)) { //to do for visualizer change name of field to a more generic one
		sshsNodePutShort(sourceInfoNode, "dataSizeX", 240);
		sshsNodePutShort(sourceInfoNode, "dataSizeY", 180);
	}

	// Nothing that can fail here.
	return (true);
}
static void caerMeanfilterRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket);
	caerFrameEventPacket *frame = va_arg(args, caerFrameEventPacket*);

	// Only process packets with content.
	if (polarity == NULL) {
		return;
	}

	MeanFilterState state = moduleData->moduleState;

	// get the size of the packet
	//int n = caerEventPacketHeaderGetEventNumber(&polarity->packetHeader);

	// get last time stamp of the packet
	// update dt and prevlastts
	//int countForLastTime = 0;
	CAER_POLARITY_ITERATOR_VALID_START(polarity)
		state->xmean = (1-state->alpha)*state->xmean + state->alpha*caerPolarityEventGetX(caerPolarityIteratorElement);
		state->ymean = (1-state->alpha)*state->ymean + state->alpha*caerPolarityEventGetY(caerPolarityIteratorElement);
//	if (countForLastTime != n-1)
//			countForLastTime ++;
//		else
//			state->lastts = caerPolarityEventGetTimestamp64(caerPolarityIteratorElement, polarity);
	CAER_POLARITY_ITERATOR_VALID_END
//	state->dt = state->lastts - state->prevlastts;
//	state->prevlastts = state->lastts;
//
//	// save position of all events in the packet
//	int xs[n];
//	int ys[n];
//	int index = 0;
//	CAER_POLARITY_ITERATOR_VALID_START(polarity)
//		xs[index] = caerPolarityEventGetX(caerPolarityIteratorElement);
//		ys[index] = caerPolarityEventGetY(caerPolarityIteratorElement);
//		index++;
//	CAER_POLARITY_ITERATOR_VALID_END
//
//	// get median
//	float x, y;
//	if (n % 2 != 0) { // odd number of events take middle one
//		x = xs[n / 2];
//		y = ys[n / 2];
//	}
//	else { // even number of events take avg of middle two
//		x = (float) (((float) xs[n / 2 - 1] + xs[n / 2]) / 2.0f);
//		y = (float) (((float) ys[n / 2 - 1] + ys[n / 2]) / 2.0f);
//	}
//	//state->xmedian = state->xFilter.filter(x, state->lastts);
//	//state->ymedian = state->yFilter.filter(y, state->lastts);
//	state->xmedian = x;
//	state->xmedian = y;
//
//	// get mean
//	int xsum = 0;
//	int ysum = 0;
//	for (int i = 0; i < n; i++) {
//		xsum += xs[i];
//		ysum += ys[i];
//	}
//	//state->xmean = state->xMeanFilter.filter(xsum / index, state->lastts);
//	//state->ymean = state->yMeanFilter.filter(ysum / index, state->lastts);
//	state->xmean = xsum / n;
//	state->ymean = ysum / n;
//
//	// get std
//	float xvar = 0;
//	float yvar = 0;
//	float tmp;
//	for (int i = 0; i < n; i++) {
//		tmp = xs[i] - state->xmean;
//		tmp *= tmp;
//		xvar += tmp;
//
//		tmp = ys[i] - state->ymean;
//		tmp *= tmp;
//		yvar += tmp;
//	}
//	xvar /= n;
//	yvar /= n;
//	//state->xstd = state->xStdFilter.filter((float) sqrt(xvar), state->lastts);
//	//state->ystd = state->yStdFilter.filter((float) sqrt(yvar), state->lastts);
//	state->xstd = (float) sqrt(xvar);
//	state->ystd = (float) sqrt(yvar);



	// plot
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo(U16T(caerEventPacketHeaderGetEventSource(&polarity->packetHeader)));
	int16_t sizeX =  sshsNodeGetShort(sourceInfoNode, "dvsSizeX");
	int16_t sizeY =  sshsNodeGetShort(sourceInfoNode, "dvsSizeY");



		*frame = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, sizeX, sizeY, 3);
		if (*frame != NULL) {
			caerFrameEvent singleplot = caerFrameEventPacketGetEvent(*frame, 0);
			uint32_t counter = 0;
			for (size_t yy = 0; yy < sizeY; yy++) {
				for (size_t xx = 0; xx < sizeX; xx++) {
					if ( (xx== state->xmean && yy == state->ymean)||
							(xx == (state->xmean + state->radius) && yy<=(state->ymean + state->radius) && yy >=(state->ymean - state->radius) ||
							(xx == (state->xmean - state->radius) && yy<=(state->ymean + state->radius) && yy >=(state->ymean - state->radius) ||
							(yy == (state->ymean + state->radius) && xx<=(state->xmean + state->radius) && xx >=(state->xmean - state->radius) ||
							(yy == (state->ymean - state->radius) && xx<=(state->xmean + state->radius) && xx >=(state->xmean - state->radius)
							))))) {
									singleplot->pixels[counter] = (uint16_t) ( (int) 65000);		// red
									singleplot->pixels[counter + 1] = (uint16_t) ( (int) 1);		// green
									singleplot->pixels[counter + 2] = (uint16_t) ( (int) 1);		// blue
					}
					else {
						singleplot->pixels[counter] = (uint16_t) ( (int) 1);			// red
						singleplot->pixels[counter + 1] = (uint16_t) ( (int) 1);		// green
						singleplot->pixels[counter + 2] = (uint16_t) ( (int) 65000 );	// blue
					}
					counter += 3;
				}
			}
			//add info to the frame
			caerFrameEventSetLengthXLengthYChannelNumber(singleplot, sizeX, sizeY, 3, *frame);
			//validate frame
			caerFrameEventValidate(singleplot, *frame);
		}

}


static void caerMeanfilterConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	MeanFilterState state = moduleData->moduleState;
//	state->xmedian = sshsNodeGetFloat(moduleData->moduleNode, "xmedian");
//	state->ymedian = sshsNodeGetFloat(moduleData->moduleNode, "ymedian");
//	state->xstd = sshsNodeGetFloat(moduleData->moduleNode, "xstd");
//	state->ystd = sshsNodeGetFloat(moduleData->moduleNode, "ystd");
	state->xmean = sshsNodeGetFloat(moduleData->moduleNode, "xmean");
	state->ymean = sshsNodeGetFloat(moduleData->moduleNode, "ymean");
//	state->lastts = sshsNodeGetInt(moduleData->moduleNode, "lastts");
//	state->dt = sshsNodeGetInt(moduleData->moduleNode, "dt");
//	state->prevlastts = sshsNodeGetInt(moduleData->moduleNode, "prevlastts");
//	state->tauUs = sshsNodeGetInt(moduleData->moduleNode, "tauUs");
//	state->numStdDevsForBoundingBox = sshsNodeGetFloat(moduleData->moduleNode, "numStdDevsForBoundingBox");
	state->alpha = sshsNodeGetFloat(moduleData->moduleNode, "alpha");
	state->radius = sshsNodeGetFloat(moduleData->moduleNode, "radius");

}

static void caerMeanfilterExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	MeanFilterState state = moduleData->moduleState;

	// Ensure map is freed.
	//simple2DBufferFreeLong(state->blobMap);
}

static void caerMeanfilterReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);

	MeanFilterState state = moduleData->moduleState;

	// Reset timestamp map to all zeros (startup state).
	//simple2DBufferResetLong(state->blobMap);
}

