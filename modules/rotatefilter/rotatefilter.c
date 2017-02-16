/*
 * rotatefilter.c
 *
 *  Created on: Feb 2017
 *      Author: Tianyu
 */

#include "rotatefilter.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/buffers.h"
#include "math.h"

struct RFilter_state {
	bool swapXY;
	bool rotate90deg;
	bool invertX;
	bool invertY;
	float angleDeg;
	bool checkBoundary;
	bool outBoundary;
};

typedef struct RFilter_state *RFilterState;

static bool caerRotatefilterInit(caerModuleData moduleData);
static void caerRotatefilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerRotatefilterConfig(caerModuleData moduleData);
static void caerRotatefilterExit(caerModuleData moduleData);
static void caerRotatefilterReset(caerModuleData moduleData, uint16_t resetCallSourceID);

static struct caer_module_functions caerRotatefilterFunctions = { .moduleInit = &caerRotatefilterInit, .moduleRun = &caerRotatefilterRun, .moduleConfig = &caerRotatefilterConfig, .moduleExit = &caerRotatefilterExit, .moduleReset = &caerRotatefilterReset };

void caerRotateFilter(uint16_t moduleID, caerPolarityEventPacket polarity) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "RFilter", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}
	caerModuleSM(&caerRotatefilterFunctions, moduleData, sizeof(struct RFilter_state), 2, polarity);
}

static bool caerRotatefilterInit(caerModuleData moduleData) {
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "swapXY", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "rotate90deg", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "invertX", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "invertY", false);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "angleDeg", 0.0f);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "checkBoundary", false);

	RFilterState state = moduleData->moduleState;

	state->swapXY= sshsNodeGetBool(moduleData->moduleNode, "swapXY");
	state->rotate90deg = sshsNodeGetBool(moduleData->moduleNode, "rotate90deg");
	state->invertX = sshsNodeGetBool(moduleData->moduleNode, "invertX");
	state->invertY = sshsNodeGetBool(moduleData->moduleNode, "invertY");
	state->angleDeg = sshsNodeGetFloat(moduleData->moduleNode, "angleDeg");
	state->checkBoundary = sshsNodeGetBool(moduleData->moduleNode, "checkBoundary");

	state->outBoundary = false;

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	// Nothing that can fail here.
	return (true);
}
static void caerRotatefilterRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket);

	//Only process packets with content.
	if (polarity == NULL) {
		return;
	}

	RFilterState state = moduleData->moduleState;

	int16_t sourceID = caerEventPacketHeaderGetEventSource(&polarity->packetHeader);
	sshsNode sourceInfoNodeCA = caerMainloopGetSourceInfo((uint16_t)sourceID);
	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	if (!sshsNodeAttributeExists(sourceInfoNode, "dataSizeX", SSHS_SHORT)) {
		sshsNodePutShort(sourceInfoNode, "dataSizeX", sshsNodeGetShort(sourceInfoNodeCA, "dvsSizeX"));
		sshsNodePutShort(sourceInfoNode, "dataSizeY", sshsNodeGetShort(sourceInfoNodeCA, "dvsSizeY"));
	}
	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "dataSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "dataSizeY");

	//Iterate over events
	CAER_POLARITY_ITERATOR_VALID_START(polarity)

	// Get values on which to operate.
	int64_t ts = caerPolarityEventGetTimestamp64(caerPolarityIteratorElement, polarity);
	uint16_t x = caerPolarityEventGetX(caerPolarityIteratorElement);
	uint16_t y = caerPolarityEventGetY(caerPolarityIteratorElement);
	bool eventType = caerPolarityEventGetPolarity(caerPolarityIteratorElement);

	if ((caerPolarityIteratorElement == NULL)){
		continue;
	}
	if ((x > sizeX) || (y > sizeY)) {
		continue;
	}

	// do rotate
	if (state->swapXY){
		state->outBoundary = ((x > sizeY) || (y > sizeX));
		if(state->checkBoundary && state->outBoundary){
			caerPolarityEventInvalidate(caerPolarityIteratorElement, polarity);
		}
		else {
			caerPolarityEventSetX(caerPolarityIteratorElement, y);
			caerPolarityEventSetY(caerPolarityIteratorElement, x);
		}
	}
	if (state->rotate90deg) {
		state->outBoundary = ((x > sizeY) || ((sizeY - y - 1) > sizeX));
		if(state->checkBoundary && state->outBoundary){
			caerPolarityEventInvalidate(caerPolarityIteratorElement, polarity);
		}
		else {
			caerPolarityEventSetX(caerPolarityIteratorElement, (sizeY - y - 1));
			caerPolarityEventSetY(caerPolarityIteratorElement, x);
		}
	}
	if (state->invertX) {
		caerPolarityEventSetX(caerPolarityIteratorElement, (sizeX - x - 1));
	}
	if (state->invertY) {
		caerPolarityEventSetY(caerPolarityIteratorElement, (sizeY - y - 1));
	}
	if (state->angleDeg != 0.0f) {
		float cosAng = cos(state->angleDeg * M_PI / 180.0f);
		float sinAng = sin(state->angleDeg * M_PI / 180.0f);
		int x2 = x - sizeX / 2;
		int y2 = y - sizeY / 2;
		int x3 = (int)(round(+cosAng * x2 - sinAng * y2));
		int y3 = (int)(round(+sinAng * x2 + cosAng * y2));
		int xr = x3 + sizeX / 2;
		int yr = y3 + sizeY / 2;
		state->outBoundary = ((xr < 0) || (xr > sizeX) || (yr < 0) || (yr > sizeY));
		if (state->checkBoundary && state->outBoundary){
			caerPolarityEventInvalidate(caerPolarityIteratorElement, polarity);
		}
		else {
			caerPolarityEventSetX(caerPolarityIteratorElement, xr);
			caerPolarityEventSetY(caerPolarityIteratorElement, yr);
		}
	}

	CAER_POLARITY_ITERATOR_VALID_END

}

static void caerRotatefilterConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	RFilterState state = moduleData->moduleState;
	state->swapXY= sshsNodeGetBool(moduleData->moduleNode, "swapXY");
	state->rotate90deg = sshsNodeGetBool(moduleData->moduleNode, "rotate90deg");
	state->invertX = sshsNodeGetBool(moduleData->moduleNode, "invertX");
	state->invertY = sshsNodeGetBool(moduleData->moduleNode, "invertY");
	state->angleDeg = sshsNodeGetFloat(moduleData->moduleNode, "angleDeg");
	state->checkBoundary = sshsNodeGetBool(moduleData->moduleNode, "checkBoundary");
}

static void caerRotatefilterExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

}

static void caerRotatefilterReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);
}
