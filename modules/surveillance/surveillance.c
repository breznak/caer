/*
 * Surveillance.c
 *
 *  Created on: Jan 2017
 *      Author: Tianyu
 */

#include "surveillance.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/buffers.h"
#include "math.h"

struct SVFilter_state {
	simple2DBufferLong blobMap;
	int32_t deltaT;
	int8_t subSampleBy;
};


static float visualThrehold = 40;
//static int timeThrehold = 10000;
//static int maxClusterDistance = 50;
static int maxClusterNum = 5;
static int currentClusterNum = 0;
static int clusterMassDecay = 10000;
static int defaultClusterRadius = 70;
static double mixingFactor = 0.01;
static float massForVisible = 40;
static int TimeStep = 100;
static float mVelocity = 0;


typedef struct cluster {
	int32_t xCenter;
	int32_t yCenter;
	float xRadius;
	float yRadius;
	float xVelocity;
	float yVelocity;
	float Radius;
	int32_t xTotal;
	int32_t yTotal;
	int32_t EventsNum;
	int64_t LastTime;
	int64_t birthTime;
	float mass;
	bool isEmpty;
	bool isVisible;
	//int64_t startVisibleTime;
}Cluster;

Cluster clusterList[5];





typedef struct SVFilter_state *SVFilterState;

static bool caerSurveillanceInit(caerModuleData moduleData);
static void caerSurveillanceRun(caerModuleData moduleData, size_t argsNumber, va_list args);
//static void caerSurveillanceRun(caerModuleData moduleData, size_t argsNumber, va_list args, ClusterList *clusterListHead);
static void caerSurveillanceConfig(caerModuleData moduleData);
static void caerSurveillanceExit(caerModuleData moduleData);
static void caerSurveillanceReset(caerModuleData moduleData, uint16_t resetCallSourceID);
static bool allocateBlobMap(SVFilterState state, int16_t sourceID);
static int getNearestCluster(uint16_t x, uint16_t y, int64_t ts);
static void updateCluster(int i, uint16_t x, uint16_t y, int64_t ts);
static Cluster newCluster(uint16_t x, uint16_t y, int64_t ts);
static int pruning( int64_t ts);
static void merge();

static struct caer_module_functions caerSurveillanceFunctions = { .moduleInit =
	&caerSurveillanceInit, .moduleRun = &caerSurveillanceRun, .moduleConfig =
	&caerSurveillanceConfig, .moduleExit = &caerSurveillanceExit, .moduleReset =
	&caerSurveillanceReset };

void caerSurveillanceFilter(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "SVFilter", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerSurveillanceFunctions, moduleData, sizeof(struct SVFilter_state), 2, polarity, frame);
}

static bool caerSurveillanceInit(caerModuleData moduleData) {
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "deltaT", 30000);
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "subSampleBy", 0);

	SVFilterState state = moduleData->moduleState;

	state->deltaT = sshsNodeGetInt(moduleData->moduleNode, "deltaT");
	state->subSampleBy = sshsNodeGetByte(moduleData->moduleNode, "subSampleBy");

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	sshsNode sourceInfoNodeCA = caerMainloopGetSourceInfo(1);  // TODO !!! -> remove hard CODED moduleID
	if (!sshsNodeAttributeExists(sourceInfoNode, "dataSizeX", SSHS_SHORT)) { //to do for visualizer change name of field to a more generic one
		sshsNodePutShort(sourceInfoNode, "dataSizeX", sshsNodeGetShort(sourceInfoNodeCA, "dvsSizeX"));
		sshsNodePutShort(sourceInfoNode, "dataSizeY", sshsNodeGetShort(sourceInfoNodeCA, "dvsSizeX"));
	}

	// initialize all cluster as empty
    int i;
    for (i=0; i<maxClusterNum; i++){
    	clusterList[i].isEmpty = true;
    	clusterList[i].isVisible = false;
    }
	// Nothing that can fail here.
	return (true);
}
static void caerSurveillanceRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket);
	caerFrameEventPacket *frame = va_arg(args, caerFrameEventPacket*);

	// Only process packets with content.
	if (polarity == NULL) {
		return;
	}

	SVFilterState state = moduleData->moduleState;

	// If the map is not allocated yet, do it.
	if (state->blobMap == NULL) {
		if (!allocateBlobMap(state, caerEventPacketHeaderGetEventSource(&polarity->packetHeader))) {
			// Failed to allocate memory, nothing to do.
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for blobMap.");
			return;
		}
	}

	// Iterate over events and filter out ones that are not supported by other
	// events within a certain region in the specified timeframe.
	CAER_POLARITY_ITERATOR_VALID_START(polarity)
		// Get values on which to operate.
		int64_t ts = caerPolarityEventGetTimestamp64(caerPolarityIteratorElement, polarity);
		uint16_t x = caerPolarityEventGetX(caerPolarityIteratorElement);
		uint16_t y = caerPolarityEventGetY(caerPolarityIteratorElement);

		// Apply sub-sampling.
		//x = U16T(x >> state->subSampleBy);
		//y = U16T(y >> state->subSampleBy);

		// Get value from map.
		int64_t lastTS = state->blobMap->buffer2d[x][y];

		/*if ((I64T(ts - lastTS) >= I64T(state->deltaT)) || (lastTS == 0)) {
			// Filter out invalid.
			caerPolarityEventInvalidate(caerPolarityIteratorElement, polarity);
		}*/

		// Update neighboring region.
		size_t sizeMaxX = (state->blobMap->sizeX - 1);
		size_t sizeMaxY = (state->blobMap->sizeY - 1);

//		caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "x equal %d", x);
//		caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "y equal %d", y);
//		caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "ts equal %d", ts);

		//state->blobMap->buffer2d[x - 1][y] = ts;

		//caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "from the surveillance filter %d", ts);

//		int i;
//		for(i=0;i<10;i++){
//			if (clusterList[i].isEmpty==1)
//				caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "clusterList %d is empty", i);
//		}




		// check nearestCluster exist?
		int chosenClusterIndex = getNearestCluster(x, y, ts);

		// if exist, update it
		if (chosenClusterIndex != -1){
			updateCluster(chosenClusterIndex, x, y, ts);
		}

		// if not, create new cluster
		else {
			if (currentClusterNum < maxClusterNum) {
				Cluster clusterNew = newCluster(x, y, ts);
				int i;
				for (i=0; i<maxClusterNum; i++){
					if (clusterList[i].isEmpty == true){
						clusterList[i] = clusterNew;
						currentClusterNum = currentClusterNum + 1;
					}
				}

				//caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "New cluster is created");
			}
			//else caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Cluster number is full");
		}

		// At each TimeStep point, do pruning
		if (ts % TimeStep == 0){
			int count;
			count = pruning(ts);
			//caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "%d cluster is deleted", count);
		}

		// merge clusters that track the same object, this function does not work well, may cause error
		//if (maxClusterNum > 1)
			//merge();


		CAER_POLARITY_ITERATOR_VALID_END

		sshsNode sourceInfoNode = caerMainloopGetSourceInfo(U16T(caerEventPacketHeaderGetEventSource(&polarity->packetHeader)));
		int16_t sizeX =  sshsNodeGetShort(sourceInfoNode, "dvsSizeX");
		int16_t sizeY =  sshsNodeGetShort(sourceInfoNode, "dvsSizeY");


		//fill the frame with the clusters
		*frame = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, sizeX, sizeY, 3);
		if (*frame != NULL) {
				caerFrameEvent singleplot = caerFrameEventPacketGetEvent(*frame, 0);

				uint32_t counter = 0;
				for (size_t x = 0; x < sizeX; x++) {
					for (size_t y = 0; y < sizeY; y++) {
							singleplot->pixels[counter] = (uint16_t) ( (int) 1);					// red
							singleplot->pixels[counter + 1] = (uint16_t) ( (int) 1);		// green
							singleplot->pixels[counter + 2] = (uint16_t) ( (int)65000 );			// blue
						counter += 3;
					}
				}

				//add info to the frame
				caerFrameEventSetLengthXLengthYChannelNumber(singleplot, sizeX, sizeY, 3, *frame);
				//validate frame
				caerFrameEventValidate(singleplot, *frame);
		}


//		//for all clusters
//		//loop over the frame and color the pixels of clusters of different colors
		int i;
		for (i=0; i<maxClusterNum; i++){
			if ((*frame != NULL ) && clusterList[i].isEmpty != true && clusterList[i].isVisible == true ||(*frame != NULL ) && clusterList[i].isEmpty != true && clusterList[i].mass > massForVisible) {
							clusterList[i].isVisible = true;
							//clusterList[i].startVisibleTime = ts;
							caerFrameEvent singleplot = caerFrameEventPacketGetEvent(*frame, 0);
							uint32_t counter = 0;
							for (size_t y = 0; y < sizeY; y++) {
								for (size_t x = 0; x < sizeX; x++) {
									if ( (x== clusterList[i].xCenter && y == clusterList[i].yCenter)||
										 (x == (clusterList[i].xCenter + clusterList[i].xRadius) && y<=(clusterList[i].yCenter + clusterList[i].yRadius) && y >=(clusterList[i].yCenter - clusterList[i].yRadius) ||
										 (x == (clusterList[i].xCenter - clusterList[i].xRadius) && y<=(clusterList[i].yCenter + clusterList[i].yRadius) && y >=(clusterList[i].yCenter - clusterList[i].yRadius) ||
										 (y == (clusterList[i].yCenter + clusterList[i].yRadius) && x<=(clusterList[i].xCenter + clusterList[i].xRadius) && x >=(clusterList[i].xCenter - clusterList[i].xRadius) ||
										 (y == (clusterList[i].yCenter - clusterList[i].yRadius) && x<=(clusterList[i].xCenter + clusterList[i].xRadius) && x >=(clusterList[i].xCenter - clusterList[i].xRadius)
									   ))))) {
										singleplot->pixels[counter] = (uint16_t) ( (int) 65000);		// red
										singleplot->pixels[counter + 1] = (uint16_t) ( (int) 1);		// green
										singleplot->pixels[counter + 2] = (uint16_t) ( (int) 1);		// blue
									}
									counter += 3;
								}
							}
					}
		}



}


static int getNearestCluster(uint16_t x, uint16_t y, int64_t ts){
	int closest = -1;
	int minDistance = 10000000;
	int currentDistance = 0;
	int i;
	for (i = 0; i < maxClusterNum; i++){
		if(clusterList[i].isEmpty != true){
			int64_t dt = ts - clusterList[i].LastTime;
			int xDiff = abs(x - clusterList[i].xCenter + clusterList[i].xVelocity * dt * mVelocity);
			int yDiff = abs(y - clusterList[i].yCenter + clusterList[i].yVelocity * dt * mVelocity);
			if (xDiff < clusterList[i].xRadius && yDiff < clusterList[i].yRadius) {
				currentDistance = xDiff + yDiff;
				if (currentDistance < minDistance){
					closest = i;
					minDistance = currentDistance;
				}
			}
		}
	}
	return (closest);
}


// position decided by moving a bit to new Event

static void updateCluster(int i, uint16_t x, uint16_t y, int64_t ts) {
	float m = mixingFactor;
	int64_t dt = ts - clusterList[i].LastTime;

	// get current position
	int32_t xCurrent = clusterList[i].xCenter + clusterList[i].xVelocity * dt * mVelocity;
	int32_t yCurrent = clusterList[i].yCenter + clusterList[i].yVelocity * dt * mVelocity;

	// get new position && check the boundary
	int32_t xNew = (1 - m) * xCurrent + m * x;
	int32_t yNew = (1 - m) * yCurrent + m * y;
	if (xNew < 0) xNew = 0;
	if (xNew > 240) xNew = 240;
	if (yNew < 0) yNew = 0;
	if (yNew > 180) yNew = 180;

	// get movement distance
	int32_t xMoveDistance = xNew - xCurrent;
	int32_t yMoveDistance = yNew - yCurrent;

	// update position
	clusterList[i].xCenter = xNew;
	clusterList[i].yCenter = yNew;

	// update velocity
	clusterList[i].xVelocity = (1-m) * clusterList[i].xVelocity + m * xMoveDistance;
	clusterList[i].yVelocity = (1-m) * clusterList[i].yVelocity + m * yMoveDistance;

	// update other information
	clusterList[i].EventsNum = clusterList[i].EventsNum + 1;
	clusterList[i].mass = 1 + clusterList[i].mass * exp(-dt / clusterMassDecay);
	clusterList[i].LastTime = ts;

}



static Cluster newCluster(uint16_t x, uint16_t y, int64_t ts) {
	Cluster clusterNew;
	clusterNew.xCenter = x;
	clusterNew.yCenter = y;
	clusterNew.xTotal = x;
	clusterNew.yTotal = y;
	clusterNew.xVelocity = 0;
	clusterNew.yVelocity = 0;
	clusterNew.xRadius = defaultClusterRadius;
	clusterNew.yRadius = defaultClusterRadius;
	clusterNew.EventsNum = 1;
	clusterNew.mass = 1;
	clusterNew.LastTime = ts;
	clusterNew.birthTime = ts;
	clusterNew.isEmpty = false;
	clusterNew.isVisible = false;
	return (clusterNew);
}


static int pruning(int64_t ts){
	int i;
	int c = 0;
	for (i=0; i<maxClusterNum; i++) {
		if (clusterList[i].isEmpty != true) {
			int64_t dt = ts - clusterList[i].LastTime;
			float mass = clusterList[i].mass * exp(-dt / clusterMassDecay);
			if ((ts - clusterList[i].birthTime) > clusterMassDecay && mass < visualThrehold){
				clusterList[i].isEmpty = true;
				clusterList[i].isVisible = false;
				currentClusterNum = currentClusterNum - 1;
				c++;
			}
		}
	}
	return (c);
}



static void merge() {
	int i,j;
	for (i = 0; i < maxClusterNum; i++) {
		if (clusterList[i].isEmpty != true) {
			for (j = i + 1; j < maxClusterNum; j++) {
				if (clusterList[i].isEmpty != true && clusterList[j].isEmpty != true) {

					// check merge condition
					float distanceC1C2 = abs(clusterList[i].xCenter - clusterList[j].xCenter) + abs(clusterList[i].yCenter - clusterList[j].yCenter);
							//float radiusC1C2 = sqrt(clusterList[i].xRadius * clusterList[i].xRadius + clusterList[i].yRadius * clusterList[i].yRadius) + sqrt(clusterList[j].xRadius * clusterList[j].xRadius + clusterList[j].yRadius * clusterList[j].yRadius);
					float radiusC1C2 = clusterList[i].xRadius + clusterList[i].yRadius + clusterList[j].xRadius + clusterList[j].yRadius;
							//&& clusterList[i].isVisible == true && clusterList[j].isVisible == true
					if (distanceC1C2 < radiusC1C2) {
						clusterList[i].isEmpty = true;
						currentClusterNum = currentClusterNum - 1;


						// find weak one and delete it
//						if (clusterList[i].mass < clusterList[j].mass)
//							weak = i;
//						else
//							weak = j;
						//float mMerge = clusterList[j].mass / (clusterList[i].mass + clusterList[j].mass);
						//clusterList[j].xCenter = (1 - mMerge) * clusterList[i].xCenter + mMerge * clusterList[j].xCenter;
						//if (clusterList[i].LastTime > clusterList[j].LastTime)
						//	clusterList[j].LastTime = clusterList[i].LastTime;


						//clusterList[i]->EventsNum = clusterList[i]->EventsNum + clusterList[j]->EventsNum;
						//clusterList[i]->xTotal = clusterList[i]->xTotal + clusterList[j]->xTotal;
						//clusterList[i]->xCenter = clusterList[i]->xTotal / clusterList[i]->EventsNum;
						//clusterList[i]->yTotal = clusterList[i]->yTotal + clusterList[j]->yTotal;
						//clusterList[i]->yCenter = clusterList[i]->yTotal / clusterList[i]->EventsNum;
						//if (clusterList[j]->LastTime > clusterList[i]->LastTime) clusterList[i]->LastTime = clusterList[j]->LastTime;

					}
				}
			}
		}
	}
}



static void caerSurveillanceConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	SVFilterState state = moduleData->moduleState;

	state->deltaT = sshsNodeGetInt(moduleData->moduleNode, "deltaT");
	state->subSampleBy = sshsNodeGetByte(moduleData->moduleNode, "subSampleBy");
}

static void caerSurveillanceExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	SVFilterState state = moduleData->moduleState;

	// Ensure map is freed.
	simple2DBufferFreeLong(state->blobMap);
}

static void caerSurveillanceReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);

	SVFilterState state = moduleData->moduleState;

	// Reset timestamp map to all zeros (startup state).
	simple2DBufferResetLong(state->blobMap);
}

static bool allocateBlobMap(SVFilterState state, int16_t sourceID) {
	// Get size information from source.
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo(U16T(sourceID));
	if (sourceInfoNode == NULL) {
		// This should never happen, but we handle it gracefully.
		caerLog(CAER_LOG_ERROR, __func__, "Failed to get source info to allocate timestamp map.");
		return (false);
	}

	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "dvsSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "dvsSizeY");

	state->blobMap = simple2DBufferInitLong((size_t) sizeX, (size_t) sizeY);
	if (state->blobMap == NULL) {
		return (false);
	}

	// TODO: size the map differently if subSampleBy is set!
	return (true);
}

