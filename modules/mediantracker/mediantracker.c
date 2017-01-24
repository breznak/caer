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
	//simple2DBufferFloat medianPoint;
	//simple2DBufferFloat stdPoint;
	//simple2DBufferFloat meanPoint;
	float xmedian;
	float ymedian;
	float xstd;
	float ystd;
	float xmean;
	float ymean;
	int lastts;
	int dt;
	int prevlastts;
//	LowpassFilter xFilter;
//	LowpassFilter yFilter;
//	LowpassFilter xStdFilter;
//	LowpassFilter yStdFilter;
//	LowpassFilter xMeanFilter;
//	LowpassFilter yMeanFilter;
	float radius;
	//int tauUs;
	float numStdDevsForBoundingBox;
	float alpha;
	//float beta;
};

//float alpha = 0.00001;
//float stdFactor = 10.0f;

typedef struct MTFilter_state *MTFilterState;

static bool caerMediantrackerInit(caerModuleData moduleData);
static void caerMediantrackerRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerMediantrackerConfig(caerModuleData moduleData);
static void caerMediantrackerExit(caerModuleData moduleData);
static void caerMediantrackerReset(caerModuleData moduleData, uint16_t resetCallSourceID);


static struct caer_module_functions caerMediantrackerFunctions = { .moduleInit =
	&caerMediantrackerInit, .moduleRun = &caerMediantrackerRun, .moduleConfig =
	&caerMediantrackerConfig, .moduleExit = &caerMediantrackerExit, .moduleReset =
	&caerMediantrackerReset };

void caerMediantrackerFilter(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "MTFilter", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerMediantrackerFunctions, moduleData, sizeof(struct MTFilter_state), 2, polarity, frame);
}

static bool caerMediantrackerInit(caerModuleData moduleData) {
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "xmedian", 0.0f);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "ymedian", 0.0f);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "xstd", 0.0f);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "ystd", 0.0f);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "xmean", 0.0f);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "ymean", 0.0f);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "lastts", 0);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "dt", 0);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "prevlastts", 0);
	//sshsNodePutIntIfAbsent(moduleData->moduleNode, "tauUs", 1000);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "numStdDevsForBoundingBox", 1.0f);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "alpha", 0.00004f);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "radius", 10.0f);
	//sshsNodePutFloatIfAbsent(moduleData->moduleNode, "beta", 0.0f);

	MTFilterState state = moduleData->moduleState;


	state->xmedian = sshsNodeGetFloat(moduleData->moduleNode, "xmedian");
	state->ymedian = sshsNodeGetFloat(moduleData->moduleNode, "ymedian");
	state->xstd = sshsNodeGetFloat(moduleData->moduleNode, "xstd");
	state->ystd = sshsNodeGetFloat(moduleData->moduleNode, "ystd");
	state->xmean = sshsNodeGetFloat(moduleData->moduleNode, "xmean");
	state->ymean = sshsNodeGetFloat(moduleData->moduleNode, "ymean");
	state->lastts = sshsNodeGetInt(moduleData->moduleNode, "lastts");
	state->dt = sshsNodeGetInt(moduleData->moduleNode, "dt");
	state->prevlastts = sshsNodeGetInt(moduleData->moduleNode, "prevlastts");
	//state->tauUs = sshsNodeGetInt(moduleData->moduleNode, "tauUs");
	state->numStdDevsForBoundingBox = sshsNodeGetFloat(moduleData->moduleNode, "numStdDevsForBoundingBox");
	state->radius = sshsNodeGetFloat(moduleData->moduleNode, "radius");
	state->alpha = sshsNodeGetFloat(moduleData->moduleNode, "alpha");
	//state->beta = sshsNodeGetFloat(moduleData->moduleNode, "beta");

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

	// get the size of the packet
	int n = caerEventPacketHeaderGetEventNumber(&polarity->packetHeader);
	//caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "n is %i", n);

	// get last time stamp of the packet
	// update dt and prevlastts
	//int countForLastTime = 0;
	int maxLastTime = 0;
	CAER_POLARITY_ITERATOR_VALID_START(polarity)
		if (maxLastTime < caerPolarityEventGetTimestamp64(caerPolarityIteratorElement, polarity))
			maxLastTime = caerPolarityEventGetTimestamp64(caerPolarityIteratorElement, polarity);
	CAER_POLARITY_ITERATOR_VALID_END
	state->lastts = maxLastTime;
	//caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "lastts is %i", state->lastts);
	state->dt = state->lastts - state->prevlastts;
	state->prevlastts = state->lastts;
	//caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "dt is %i", state->dt);
	if (state->dt < 0)
		state->dt = 0;

	// save position of all events in the packet
	int xs[n];
	int ys[n];
	int index = 0;
	CAER_POLARITY_ITERATOR_VALID_START(polarity)
		xs[index] = caerPolarityEventGetX(caerPolarityIteratorElement);
	//caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "xs is %i", xs[index]);
		ys[index] = caerPolarityEventGetY(caerPolarityIteratorElement);
		index++;
	CAER_POLARITY_ITERATOR_VALID_END

	// get median
	int x, y;
	if (index % 2 != 0) { // odd number of events take middle one
		x = xs[index / 2];
		y = ys[index / 2];
		//caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "x is %i", x);
	}
	else { // even number of events take avg of middle two
		x =  (float)(((float) xs[index / 2 - 1] + xs[index / 2]) / 2.0f);
		y =  (float)(((float) ys[index / 2 - 1] + ys[index / 2]) / 2.0f);
	}
	//caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "x is %i", x);
	//state->xmedian = state->xFilter.filter(x, state->lastts);
	//state->ymedian = state->yFilter.filter(y, state->lastts);
	//state->xmedian = x;//state->xmedian + (x - state->xmedian)*state->dt;
	//state->ymedian = y;//state->ymedian + (y - state->ymedian)*state->dt;;
	float fac = state->alpha*(float)state->dt;
	if (fac > 1)
		fac = 1;
	state->xmedian = state->xmedian + (x - state->xmedian)*fac;
	state->ymedian = state->ymedian + (y - state->ymedian)*fac;
	//caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "xmedian is %f", state->xmedian);


	// get mean
	int xsum = 0;
	int ysum = 0;
	for (int i = 0; i < index; i++) {
		xsum += xs[i];
		ysum += ys[i];
	}
	//state->xmean = state->xMeanFilter.filter(xsum / index, state->lastts);
	//state->ymean = state->yMeanFilter.filter(ysum / index, state->lastts);
	state->xmean = state->xmean + (xsum / n - state->xmean)*fac;
	state->ymean = state->ymean + (ysum / n - state->ymean)*fac;
	//state->xmean = (xsum / n );
	//state->ymean = (ysum / n );
	//caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "xmean is %f", state->xmean);


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
	if (index != 0){
		xvar /= index;
		yvar /= index;
	}
	//state->xstd = state->xStdFilter.filter((float) sqrt(xvar), state->lastts);
	//state->ystd = state->yStdFilter.filter((float) sqrt(yvar), state->lastts);
	state->xstd = state->xstd + ((float) sqrt(xvar) - state->xstd)*fac;
	state->ystd = state->ystd + ((float) sqrt(yvar) - state->ystd)*fac;
	//state->xstd = (float) sqrt(xvar);
	//state->ystd = (float) sqrt(yvar);



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
					if ( (xx== (int)state->xmedian && yy == (int)state->ymedian)||
							(xx == (int)(state->xmedian + state->xstd*state->numStdDevsForBoundingBox) && yy<=(state->ymedian + state->ystd*state->numStdDevsForBoundingBox) && yy >=(state->ymedian - state->ystd*state->numStdDevsForBoundingBox) ||
							(xx == (int)(state->xmedian - state->xstd*state->numStdDevsForBoundingBox) && yy<=(state->ymedian + state->ystd*state->numStdDevsForBoundingBox) && yy >=(state->ymedian - state->ystd*state->numStdDevsForBoundingBox) ||
							(yy == (int)(state->ymedian + state->ystd*state->numStdDevsForBoundingBox) && xx<=(state->xmedian + state->xstd*state->numStdDevsForBoundingBox) && xx >=(state->xmedian - state->xstd*state->numStdDevsForBoundingBox) ||
							(yy == (int)(state->ymedian - state->ystd*state->numStdDevsForBoundingBox) && xx<=(state->xmedian + state->xstd*state->numStdDevsForBoundingBox) && xx >=(state->xmedian - state->xstd*state->numStdDevsForBoundingBox)
									//(xx == (int)(state->xmedian + 10) && yy<=(state->ymedian + 20) && yy >=(state->ymedian - 20) ||
									//(xx == (int)(state->xmedian - 10) && yy<=(state->ymedian + 20) && yy >=(state->ymedian - 20) ||
									//(yy == (int)(state->ymedian + 20) && xx<=(state->xmedian + 10) && xx >=(state->xmedian - 10) ||
									//(yy == (int)(state->ymedian - 20) && xx<=(state->xmedian + 10) && xx >=(state->xmedian - 10)
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


static void caerMediantrackerConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	MTFilterState state = moduleData->moduleState;
	//state->xmedian = sshsNodeGetFloat(moduleData->moduleNode, "xmedian");
	//state->ymedian = sshsNodeGetFloat(moduleData->moduleNode, "ymedian");
	//state->xstd = sshsNodeGetFloat(moduleData->moduleNode, "xstd");
	//state->ystd = sshsNodeGetFloat(moduleData->moduleNode, "ystd");
	//state->xmean = sshsNodeGetFloat(moduleData->moduleNode, "xmean");
	//state->ymean = sshsNodeGetFloat(moduleData->moduleNode, "ymean");
	//state->lastts = sshsNodeGetInt(moduleData->moduleNode, "lastts");
	//state->dt = sshsNodeGetInt(moduleData->moduleNode, "dt");
	//state->prevlastts = sshsNodeGetInt(moduleData->moduleNode, "prevlastts");
	//state->tauUs = sshsNodeGetInt(moduleData->moduleNode, "tauUs");
	state->radius = sshsNodeGetFloat(moduleData->moduleNode, "radius");
	state->numStdDevsForBoundingBox = sshsNodeGetFloat(moduleData->moduleNode, "numStdDevsForBoundingBox");
	state->alpha = sshsNodeGetFloat(moduleData->moduleNode, "alpha");
	//state->beta = sshsNodeGetFloat(moduleData->moduleNode, "beta");

}

static void caerMediantrackerExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	MTFilterState state = moduleData->moduleState;

	// Ensure map is freed.
	//simple2DBufferFreeLong(state->blobMap);
}

static void caerMediantrackerReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);

	MTFilterState state = moduleData->moduleState;

	// Reset timestamp map to all zeros (startup state).
	//simple2DBufferResetLong(state->blobMap);
}

