/*
 * depressingsynapsefilter.c
 *
 *  Created on: Apr. 2017
 *      Author: Tianyu
 */

#include "depressingsynapsefilter.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/buffers.h"
#include "math.h"

struct DSFilter_state {
	bool swapXY;
	bool rotate90deg;
	bool invertX;
	bool invertY;
	float angleDeg;
};

typedef struct DSFilter_state *DSFilterState;

static bool caerDepressingSynapsefilterInit(caerModuleData moduleData);
static void caerDepressingSynapsefilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerDepressingSynapsefilterConfig(caerModuleData moduleData);
static void caerDepressingSynapsefilterExit(caerModuleData moduleData);
static void caerDepressingSynapsefilterReset(caerModuleData moduleData, uint16_t resetCallSourceID);
static void checkBoundary(int* x, int* y, int sizeX, int sizeY);

static struct caer_module_functions caerDepressingSynapsefilterFunctions = { .moduleInit = &caerDepressingSynapsefilterInit, .moduleRun = &caerDepressingSynapsefilterRun, .moduleConfig = &caerDepressingSynapsefilterConfig, .moduleExit = &caerDepressingSynapsefilterExit, .moduleReset = &caerDepressingSynapsefilterReset };

void caerDepressingSynapseFilter(uint16_t moduleID, caerPolarityEventPacket polarity) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "DSFilter", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}
	caerModuleSM(&caerDepressingSynapsefilterFunctions, moduleData, sizeof(struct DSFilter_state), 2, polarity);
}

static bool caerDepressingSynapsefilterInit(caerModuleData moduleData) {
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "swapXY", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "rotate90deg", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "invertX", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "invertY", false);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "angleDeg", 0.0f);

	RFilterState state = moduleData->moduleState;

	state->swapXY= sshsNodeGetBool(moduleData->moduleNode, "swapXY");
	state->rotate90deg = sshsNodeGetBool(moduleData->moduleNode, "rotate90deg");
	state->invertX = sshsNodeGetBool(moduleData->moduleNode, "invertX");
	state->invertY = sshsNodeGetBool(moduleData->moduleNode, "invertY");
	state->angleDeg = sshsNodeGetFloat(moduleData->moduleNode, "angleDeg");

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	// Nothing that can fail here.
	return (true);
}
static void caerDepressingSynapsefilterRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
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

	if ((caerPolarityIteratorElement == NULL)){
		continue;
	}
	if ((x >= sizeX) || (y >= sizeY)) {
		continue;
	}

	// do rotate
	if (state->swapXY){
		int newX = y;
		int newY = x;
		checkBoundary(&newX, &newY, sizeX, sizeY);
		caerPolarityEventSetX(caerPolarityIteratorElement, newX);
		caerPolarityEventSetY(caerPolarityIteratorElement, newY);

	}
	if (state->rotate90deg) {
		int newX = (sizeY - y - 1);
		int newY = x;
		checkBoundary(&newX, &newY, sizeX, sizeY);
		caerPolarityEventSetX(caerPolarityIteratorElement, newX);
		caerPolarityEventSetY(caerPolarityIteratorElement, newY);
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
		int newX = x3 + sizeX / 2;
		int newY = y3 + sizeY / 2;
		checkBoundary(&newX, &newY, sizeX, sizeY);
		caerPolarityEventSetX(caerPolarityIteratorElement, newX);
		caerPolarityEventSetY(caerPolarityIteratorElement, newY);
	}

	CAER_POLARITY_ITERATOR_VALID_END

}

static void checkBoundary(int* x, int* y, int sizeX, int sizeY){
	if (*x >= sizeX) {
		*x = sizeX-1;
	}
	if (*x < 0){
		*x = 0;
	}
	if (*y >= sizeY) {
		*y = sizeY-1;
	}
	if (*y < 0){
		*y = 0;
	}
}

static void caerDepressingSynapsefilterConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	RFilterState state = moduleData->moduleState;
	state->swapXY= sshsNodeGetBool(moduleData->moduleNode, "swapXY");
	state->rotate90deg = sshsNodeGetBool(moduleData->moduleNode, "rotate90deg");
	state->invertX = sshsNodeGetBool(moduleData->moduleNode, "invertX");
	state->invertY = sshsNodeGetBool(moduleData->moduleNode, "invertY");
	state->angleDeg = sshsNodeGetFloat(moduleData->moduleNode, "angleDeg");
}

static void caerDepressingSynapsefilterExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

}

static void caerDepressingSynapsefilterReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);
}
