/*
 * rectangulartracker.c
 *
 *  Created on: Jan 2017
 *      Author: Tianyu
 */

#include "rectangulartracker.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/buffers.h"
#include "math.h"


typedef struct path {
	float location_x;
	float location_y;
	float velocityPPT_x;
	float velocityPPT_y;
	int64_t timestamp;
	int nEvents;
	struct path * next;
}Path;

typedef struct cluster {
	float location_x;
	float location_y;
	float velocity_x;
	float velocity_y;
	float birthLocation_x;
	float birthLocation_y;
	float lastPacketLocation_x;
	float lastPacketLocation_y;
	float velocityPPT_x;
	float velocityPPT_y;
	float velocityPPS_x;
	float velocityPPS_y;
	float angle;
	float cosAngle;
	float sinAngle;
	//Color color;
	int32_t numEvents;
	int32_t previousNumEvents;
	int64_t firstEventTimestamp;
	int64_t lastEventTimestamp;
	int64_t lastUpdateTime;
	float instantaneousEventRate;
	bool hasObtainedSupport;
	float averageEventDistance;
	float averageEventXDistance;
	float averageEventYDistance;
	int clusterNumber;
	float avgEventRate;
	float radius;
	float aspectRatio;
	float radius_x;
	float radius_y;
	Path * path;
	float avgISI;
	//float rgb[4];
	bool velocityValid;
	bool visibilityFlag;
	float instantaneousISI;
	float distanceToLastEvent;
	float distanceToLastEvent_x;
	float distanceToLastEvent_y;
	float mass;
	int64_t vFilterTime;
	bool isEmpty;
} Cluster;

struct RTFilter_state {
	Cluster clusterList[10];
	int currentClusterNum;
	bool dynamicSizeEnabled;
	bool dynamicAspectRatioEnabled;
	bool dynamicAngleEnabled;
	bool pathsEnabled;
	float thresholdMassForVisibleCluster;
	int maxClusterNum;
	float defaultClusterRadius;
	bool showPaths;
};

// constants
static const float VELPPS_SCALING = 1e6f;
static const int TICK_PER_MS = 1000;
static const float MAX_SCALE_RATIO = 2.0f;
static const float ASPECT_RATIO_MAX_DYNAMIC_ANGLE_DISABLED = 2.5f;
static const float ASPECT_RATIO_MIN_DYNAMIC_ANGLE_DISABLED = 0.5f;
static const float ASPECT_RATIO_MAX_DYNAMIC_ANGLE_ENABLED = 1.0f;
static const float ASPECT_RATIO_MIN_DYNAMIC_ANGLE_ENABLED = 0.5f;
static const float AVERAGE_VELOCITY_MIXING_FACTOR = 0.001f;


static int clusterMassDecayTauUs = 10000;
static float mixingFactor = 0.005f;
static float surround = 2.0f;
static int TimeStep = 100;
static bool smoothMove = false;
static float smoothWeight = 100.0f;
static float smoothPosition = 0.001f;
static float smoothIntegral = 0.001f;
static float velAngDiffDegToNotMerge = 60.0f;
static bool useVelocity = false;
static float thresholdVelocityForVisibleCluster = 0.0f;
static int pathLength = 100;

//static bool useEllipticalClusters = false;
//static bool colorClustersDifferentlyEnabled = false;
//static bool useOnePolarityOnlyEnabled = false;
//static bool useOffPolarityOnlyEnabled = false;
//static float aspectRatio = 1.0f;
//static float clusterSize = 0.1f;
static bool growMergedSizeEnabled = false;
//static bool showAllClusters = false;
static bool useNearestCluster = true; // use the nearest cluster to an event, not the first containing it.
static float predictiveVelocityFactor = 1; // making this M=10, for example,  will cause cluster to substantially lead the events, then slow down, speed up, etc.

static bool enableClusterExitPurging = true;
//static bool showClusterNumber = false;
//static bool showClusterEps = false;
//static bool showClusterRadius = false;
//static bool showClusterVelocity = false;
//static bool showClusterMass = false;
//static float velocityVectorScaling = 1.0f;
static bool initializeVelocityToAverage = false;
//static bool filterEventsEnabled = false; // enables filtering events so that output events only belong to clustera and point to the clusters.
static float velocityTauMs = 100.0f;
//static float frictionTauMs = 0.0 / 0.0; // Float.NaN in java;
static bool dontMergeEver = false;
static bool angleFollowsVelocity = false;

static float initialAngle = 0.0f;
static int clusterCounter = 0;

static float averageVelocityPPT_x = 0.0f;
static float averageVelocityPPT_y = 0.0f;



typedef struct RTFilter_state *RTFilterState;

static bool caerRectangulartrackerInit(caerModuleData moduleData);
static void caerRectangulartrackerRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerRectangulartrackerConfig(caerModuleData moduleData);
static void caerRectangulartrackerExit(caerModuleData moduleData);
static void caerRectangulartrackerReset(caerModuleData moduleData, uint16_t resetCallSourceID);
static int getNearestCluster(caerModuleData moduleData, uint16_t x, uint16_t y, int64_t ts);
static int getFirstContainingCluster(caerModuleData moduleData, uint16_t x, uint16_t y, int64_t ts);
static void updateClusterList(caerModuleData moduleData, int64_t ts, int16_t sizeX, int16_t sizeY);
static void pruneClusters(caerModuleData moduleData, int64_t ts, int16_t sizeX, int16_t sizeY);
static void mergeClusters(caerModuleData moduleData);
static void mergeC1C2(caerModuleData moduleData, int i, int j);
static int64_t getLifetime(Cluster *c);
static float getMassNow(Cluster *c, int64_t ts);
static float distanceToX(Cluster *c, uint16_t x, uint16_t y, int64_t ts);
static float distanceToY(Cluster *c, uint16_t x, uint16_t y, int64_t ts);
static float distanceC1C2(Cluster *c1, Cluster *c2);
static float distanceToEvent(Cluster *c, uint16_t x, uint16_t y);
static void updatePosition(Cluster *c, uint16_t x, uint16_t y);
static void checkAndSetClusterVisibilityFlag(caerModuleData moduleData);
static void setRadius(Cluster *c, float r);
static void updateMass(Cluster *c, int64_t ts);
static void updateClusterMasses(caerModuleData moduleData, int64_t ts);
static void addEvent(caerModuleData moduleData, Cluster *c, uint16_t x, uint16_t y, int64_t ts);
static void updateEventRate(Cluster *c, int64_t ts);
static Cluster newCluster(uint16_t x, uint16_t y, int64_t ts, float defaultClusterRadius);
static void updateAverageEventDistance(Cluster *c);
static void updateShape(caerModuleData moduleData, Cluster *c, uint16_t x, uint16_t y);
static void updateSize(Cluster *c, uint16_t x, uint16_t y, float defaultClusterRadius);
static void updateAspectRatio(Cluster *c, uint16_t x, uint16_t y, bool dynamicAngleEnabled);
static void updateAngle(Cluster *c, uint16_t x, uint16_t y);
static void setAngle(Cluster *c, float angle);
static bool hasHitEdge(Cluster *c, int16_t sizeX, int16_t sizeY);
static bool isOverlapping(Cluster *c1, Cluster *c2);
static float velocityAngleToRad(Cluster *c1, Cluster *c2);
static void updateClusterLocations(caerModuleData moduleData, int64_t ts);
static int getPathSize(Path * head);
static void removeLastPath(Path * head);
static void addPath(Path ** head, float x, float y, int64_t t, int events);
static void updateVelocity(Cluster *c, float thresholdMassForVisibleCluster);
static void updateClusterPaths(caerModuleData moduleData, int64_t ts);
static float getDXl(Cluster *c);
static float getDXs(Cluster *c);
static float getDYl(Cluster *c);
static float getDYs(Cluster *c);
static void drawCluster(caerFrameEvent singleplot, Cluster *c, int sizeX, int sizeY, bool showPaths);


static struct caer_module_functions caerRectangulartrackerFunctions = { .moduleInit = &caerRectangulartrackerInit, .moduleRun = &caerRectangulartrackerRun, .moduleConfig = &caerRectangulartrackerConfig, .moduleExit = &caerRectangulartrackerExit, .moduleReset = &caerRectangulartrackerReset };

void caerRectangulartrackerFilter(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "RTFilter", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}
	caerModuleSM(&caerRectangulartrackerFunctions, moduleData,
	sizeof(struct RTFilter_state), 2, polarity, frame);
}

static bool caerRectangulartrackerInit(caerModuleData moduleData) {
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "dynamicSizeEnabled", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "dynamicAspectRatioEnabled", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "dynamicAngleEnabled", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "pathsEnabled", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "showPaths", false);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "maxClusterNum", 3);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "thresholdMassForVisibleCluster", 60.0f);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "defaultClusterRadius", 25.0f);

	RTFilterState state = moduleData->moduleState;

	state->dynamicSizeEnabled = sshsNodeGetBool(moduleData->moduleNode, "dynamicSizeEnabled");
	state->dynamicAspectRatioEnabled = sshsNodeGetBool(moduleData->moduleNode, "dynamicAspectRatioEnabled");
	state->dynamicAngleEnabled = sshsNodeGetBool(moduleData->moduleNode, "dynamicAngleEnabled");
	state->pathsEnabled = sshsNodeGetBool(moduleData->moduleNode, "pathsEnabled");
	state->showPaths = sshsNodeGetBool(moduleData->moduleNode, "showPaths");
	state->maxClusterNum = sshsNodeGetInt(moduleData->moduleNode, "maxClusterNum");
	state->thresholdMassForVisibleCluster = sshsNodeGetFloat(moduleData->moduleNode, "thresholdMassForVisibleCluster");
	state->defaultClusterRadius = sshsNodeGetFloat(moduleData->moduleNode, "defaultClusterRadius");
	state->currentClusterNum = 0;

	// initialize all cluster as empty
	for (int i = 0; i < state->maxClusterNum; i++) {
		state->clusterList[i].location_x = 0.0f;
		state->clusterList[i].location_y = 0.0f;
		state->clusterList[i].velocity_x = 0.0f;
		state->clusterList[i].velocity_y = 0.0f;
		state->clusterList[i].birthLocation_x = 0.0f;
		state->clusterList[i].birthLocation_y = 0.0f;
		state->clusterList[i].lastPacketLocation_x = 0.0f;
		state->clusterList[i].lastPacketLocation_y = 0.0f;
		state->clusterList[i].velocityPPT_x = 0.0f;
		state->clusterList[i].velocityPPT_y = 0.0f;
		state->clusterList[i].velocityPPS_x = 0.0f;
		state->clusterList[i].velocityPPS_y = 0.0f;
		state->clusterList[i].angle = 0.0f;
		state->clusterList[i].cosAngle = 1.0f;
		state->clusterList[i].sinAngle = 0.0f;
//		//Color color;
		state->clusterList[i].numEvents = 0;
		state->clusterList[i].previousNumEvents = 0;
		state->clusterList[i].firstEventTimestamp = 0;
		state->clusterList[i].lastEventTimestamp = 0;
		state->clusterList[i].lastUpdateTime = 0;
		state->clusterList[i].instantaneousEventRate = 0.0f;
		state->clusterList[i].hasObtainedSupport = false;
		state->clusterList[i].averageEventDistance = 0.0f;
		state->clusterList[i].averageEventXDistance = 0.0f;
		state->clusterList[i].averageEventYDistance = 0.0f;
		state->clusterList[i].clusterNumber = 0;
		state->clusterList[i].avgEventRate = 0.0f;
		state->clusterList[i].radius = 0.0f;
		state->clusterList[i].aspectRatio = 0.0f;
		state->clusterList[i].radius_x = 0.0f;
		state->clusterList[i].radius_y= 0.0f;
		state->clusterList[i].velocityValid = false;
		state->clusterList[i].isEmpty = true;
		state->clusterList[i].visibilityFlag = false;
		state->clusterList[i].path = NULL;
	}

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	// Nothing that can fail here.
	return (true);
}
static void caerRectangulartrackerRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerPolarityEventPacket polarity = va_arg(args, caerPolarityEventPacket);
	caerFrameEventPacket *frame = va_arg(args, caerFrameEventPacket*);

	//Only process packets with content.
	if (polarity == NULL) {
		return;
	}

	RTFilterState state = moduleData->moduleState;

	int16_t sourceID = caerEventPacketHeaderGetEventSource(&polarity->packetHeader);
	sshsNode sourceInfoNodeCA = caerMainloopGetSourceInfo((uint16_t)sourceID);
	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	if (!sshsNodeAttributeExists(sourceInfoNode, "dataSizeX", SSHS_SHORT)) {
		sshsNodePutShort(sourceInfoNode, "dataSizeX", sshsNodeGetShort(sourceInfoNodeCA, "dvsSizeX"));
		sshsNodePutShort(sourceInfoNode, "dataSizeY", sshsNodeGetShort(sourceInfoNodeCA, "dvsSizeY"));
	}

	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "dataSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "dataSizeY");

	for (int i = 1; i < state->maxClusterNum; i++){
		state->clusterList[i].lastPacketLocation_x = state->clusterList[i].location_x;
		state->clusterList[i].lastPacketLocation_y = state->clusterList[i].location_y;
	}

	//Iterate over events
	CAER_POLARITY_ITERATOR_VALID_START(polarity)

	// Get values on which to operate.
	int64_t ts = caerPolarityEventGetTimestamp64(caerPolarityIteratorElement, polarity);
	uint16_t x = caerPolarityEventGetX(caerPolarityIteratorElement);
	uint16_t y = caerPolarityEventGetY(caerPolarityIteratorElement);

	if ((caerPolarityIteratorElement == NULL)){
		continue;
	}
	if ((x > sizeX) || (y > sizeY)) {
		continue;
	}

	// check nearestCluster exist?
	int chosenClusterIndex;
	if (useNearestCluster){
		chosenClusterIndex = getNearestCluster(moduleData, x, y, ts);
	}
	else {
		chosenClusterIndex = getFirstContainingCluster(moduleData, x, y, ts);
	}

	// if exist, update it
	if (chosenClusterIndex != -1){
		addEvent(moduleData, &(state->clusterList[chosenClusterIndex]), x, y, ts);
	}

	// if not, create new cluster
	else if (state->currentClusterNum < state->maxClusterNum) {
		Cluster clusterNew = newCluster(x, y, ts, state->defaultClusterRadius);
		int i;
		for (i=0; i<state->maxClusterNum; i++){
			if (state->clusterList[i].isEmpty == true){
				state->clusterList[i] = clusterNew;
				state->currentClusterNum++;
			}
		}
	}
	// At each TimeStep point, do update, pruning, merge ..
	if (ts % TimeStep == 0){
		updateClusterList(moduleData, ts, sizeX, sizeY);
	}

	CAER_POLARITY_ITERATOR_VALID_END

	//plot events
	*frame = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, sizeX, sizeY, 3);
	if (*frame != NULL) {
		caerFrameEvent singleplot = caerFrameEventPacketGetEvent(*frame, 0);
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

		//add info to the frame
		caerFrameEventSetLengthXLengthYChannelNumber(singleplot, sizeX, sizeY, 3, *frame);
		//validate frame
		caerFrameEventValidate(singleplot, *frame);
	}

	// plot clusters
	for (int i=0; i<state->maxClusterNum; i++){
		if ((*frame != NULL ) && (state->clusterList[i].isEmpty != true) && (state->clusterList[i].visibilityFlag == true)) {
			drawCluster(caerFrameEventPacketGetEvent(*frame, 0), &state->clusterList[i], sizeX, sizeY, state->showPaths);
		}
	}
}

static int getNearestCluster(caerModuleData moduleData, uint16_t x, uint16_t y, int64_t ts) {
	RTFilterState state = moduleData->moduleState;

	int closest = -1;
	float minDistance = 10000000.0f;
	float currentDistance = 0.0f;
	int i;
	for (i = 0; i < state->maxClusterNum; i++) {
		if (!state->clusterList[i].isEmpty) {
			float rX = state->clusterList[i].radius_x;
			float rY = state->clusterList[i].radius_y;
			if (state->dynamicSizeEnabled) {
				rX *= surround;
				rY *= surround; // the event is captured even when it is in "invisible surround"
			}
			float dx = distanceToX(&(state->clusterList[i]), x, y, ts);
			float dy = distanceToY(&(state->clusterList[i]), x, y, ts);
			if ((dx < rX) && (dy < rY)) {
				currentDistance = dx + dy;
				if (currentDistance < minDistance) {
					closest = i;
					minDistance = currentDistance;
					state->clusterList[i].distanceToLastEvent = minDistance;
					state->clusterList[i].distanceToLastEvent_x = dx;
					state->clusterList[i].distanceToLastEvent_y = dy;
				}
			}
		}
	}
	return (closest);
}

static int getFirstContainingCluster(caerModuleData moduleData, uint16_t x, uint16_t y, int64_t ts) {
	RTFilterState state = moduleData->moduleState;

	int closest = -1;
	float minDistance = 10000000.0f;
	float currentDistance = 0.0f;
	for (int i = 0; i < state->maxClusterNum; i++) {
		if (!state->clusterList[i].isEmpty) {
			float rX = state->clusterList[i].radius_x;
			float rY = state->clusterList[i].radius_y; // this is surround region for purposes of dynamicSize scaling of cluster size or
			// aspect ratio
			if (state->dynamicSizeEnabled) {
				rX *= surround;
				rY *= surround; // the event is captured even when it is in "invisible surround"
			}
			float dx = distanceToX(&state->clusterList[i], x, y, ts);
			float dy = distanceToY(&state->clusterList[i], x, y, ts);
			if ((dx < rX) && (dy < rY)) { // TODO needs to account for instantaneousAngle
				currentDistance = dx + dy;
				closest = i;
				minDistance = currentDistance;
				state->clusterList[i].distanceToLastEvent = minDistance;
				state->clusterList[i].distanceToLastEvent_x = dx;
				state->clusterList[i].distanceToLastEvent_y = dy;
				break;
			}
		}
	}
	return (closest);
}

static void updateClusterList(caerModuleData moduleData, int64_t ts, int16_t sizeX, int16_t sizeY) {
	pruneClusters(moduleData, ts, sizeX, sizeY);
	mergeClusters(moduleData);
	updateClusterLocations(moduleData, ts);
	updateClusterPaths(moduleData, ts);
	updateClusterMasses(moduleData, ts);
	checkAndSetClusterVisibilityFlag(moduleData);
}

static Cluster newCluster(uint16_t x, uint16_t y, int64_t ts, float defaultClusterRadius) {
	Cluster clusterNew;
	clusterNew.aspectRatio = 1.0f;
	clusterNew.radius = defaultClusterRadius;
	clusterNew.radius_x = defaultClusterRadius;
	clusterNew.radius_y = defaultClusterRadius;
	clusterNew.clusterNumber = ++clusterCounter;
	if (initializeVelocityToAverage) {
		clusterNew.velocityPPT_x = averageVelocityPPT_x;
		clusterNew.velocityPPT_y = averageVelocityPPT_y;
		clusterNew.velocityValid = true;
	}
	clusterNew.location_x = (float)x;
	clusterNew.location_y = (float)y;
	clusterNew.birthLocation_x = (float)x;
	clusterNew.birthLocation_y = (float)y;
	clusterNew.lastPacketLocation_x = (float)x;
	clusterNew.lastPacketLocation_y = (float)y;
	clusterNew.lastEventTimestamp = ts;
	clusterNew.lastUpdateTime = ts;
	clusterNew.firstEventTimestamp = ts;
	clusterNew.numEvents = 1;
	clusterNew.mass = 1.0f;
	clusterNew.angle = 0.0f;
	clusterNew.cosAngle = 1.0f;
	clusterNew.sinAngle = 0.0f;
	clusterNew.previousNumEvents = 0;
	clusterNew.hasObtainedSupport = false;
	clusterNew.avgEventRate = 0.0f;
	clusterNew.velocityValid = false;
	clusterNew.visibilityFlag = false;
	clusterNew.distanceToLastEvent = 1000000;
	clusterNew.distanceToLastEvent_x = 1000000;
	clusterNew.distanceToLastEvent_y = 1000000;
	clusterNew.path = NULL;
	clusterNew.isEmpty = false;
	return (clusterNew);
}

static void pruneClusters(caerModuleData moduleData, int64_t ts, int16_t sizeX, int16_t sizeY) {
	RTFilterState state = moduleData->moduleState;

	for (int i = 0; i < state->maxClusterNum; i++) {
		if (!state->clusterList[i].isEmpty){
			int64_t t0 = state->clusterList[i].lastEventTimestamp;
			int64_t timeSinceSupport = ts - t0;
			if (timeSinceSupport == 0) {
				continue;
			}
			bool massTooSmall = false;
			int64_t lifetime = getLifetime(&state->clusterList[i]);
			if (ts > state->clusterList[i].lastEventTimestamp) {
				lifetime = ts - state->clusterList[i].firstEventTimestamp;
			}
			float massThreshold = state->thresholdMassForVisibleCluster;
			if (((lifetime == 0) || (lifetime >= clusterMassDecayTauUs)) && (getMassNow(&state->clusterList[i], ts) < massThreshold)) {
				massTooSmall = true;
			}
			bool hitEdge = hasHitEdge(&state->clusterList[i], sizeX, sizeY);
			if ((t0 > ts) || massTooSmall || (timeSinceSupport < 0) || hitEdge) {
				state->clusterList[i].isEmpty = true;
				state->currentClusterNum--;
			}
		}
	}
}

static void mergeClusters(caerModuleData moduleData) {
	RTFilterState state = moduleData->moduleState;

	if (dontMergeEver) {
		return;
	}

	bool mergePending;
	int i, j;
	do {
		mergePending = false;
		for (i = 0; i < state->maxClusterNum; i++) {
			if (!state->clusterList[i].isEmpty) {
				for (j = i + 1; j < state->maxClusterNum; j++) {
					if (!state->clusterList[i].isEmpty && !state->clusterList[j].isEmpty) {
						bool overlapping = isOverlapping(&state->clusterList[i], &state->clusterList[j]);
						bool velSimilar = true;
						if (overlapping && (velAngDiffDegToNotMerge > 0)
								&& state->clusterList[i].visibilityFlag
								&& state->clusterList[j].visibilityFlag
								&& state->clusterList[i].velocityValid
								&& state->clusterList[j].velocityValid
								&& velocityAngleToRad(&state->clusterList[i], &state->clusterList[j]) > ((velAngDiffDegToNotMerge * (float)M_PI) / 180)) {
							velSimilar = false;
						}
						if (overlapping && velSimilar) {
							mergePending = true;
							goto stopLoop;
						}
					}
				}
			}
		}
		stopLoop:
		if (mergePending && (!state->clusterList[i].isEmpty) && (!state->clusterList[j].isEmpty)) {
			mergeC1C2(moduleData, i, j);
		}
	} while (mergePending);
}

static void mergeC1C2(caerModuleData moduleData, int i, int j) {

	RTFilterState state = moduleData->moduleState;

	int weaker = state->clusterList[i].mass > state->clusterList[j].mass ? j : i;
	int stronger = state->clusterList[i].mass > state->clusterList[j].mass ? i : j;

	float mass = state->clusterList[i].mass + state->clusterList[j].mass;
	state->clusterList[stronger].numEvents = state->clusterList[i].numEvents + state->clusterList[j].numEvents;
	state->clusterList[stronger].velocity_x = 0.0f;
	state->clusterList[stronger].velocity_y = 0.0f;
	state->clusterList[stronger].averageEventDistance = ((state->clusterList[stronger].averageEventDistance * state->clusterList[stronger].mass) + (state->clusterList[weaker].averageEventDistance * state->clusterList[weaker].mass)) / mass;
	state->clusterList[stronger].averageEventXDistance = ((state->clusterList[stronger].averageEventXDistance * state->clusterList[stronger].mass) + (state->clusterList[weaker].averageEventXDistance * state->clusterList[weaker].mass)) / mass;
	state->clusterList[stronger].averageEventYDistance = ((state->clusterList[stronger].averageEventYDistance * state->clusterList[stronger].mass) + (state->clusterList[weaker].averageEventYDistance * state->clusterList[weaker].mass)) / mass;

	state->clusterList[stronger].lastEventTimestamp = state->clusterList[stronger].lastEventTimestamp > state->clusterList[weaker].lastEventTimestamp ? state->clusterList[stronger].lastEventTimestamp : state->clusterList[weaker].lastEventTimestamp;
	state->clusterList[stronger].lastUpdateTime = state->clusterList[stronger].lastEventTimestamp;
	state->clusterList[stronger].mass = mass;

	if (growMergedSizeEnabled) {
		float R = (state->clusterList[i].radius + state->clusterList[j].radius);
		setRadius(&state->clusterList[stronger], R + (mixingFactor * R));
	}

	state->clusterList[weaker].isEmpty = true;
	state->currentClusterNum--;
}

int64_t getLifetime(Cluster *c) {
	return (c->lastUpdateTime - c->firstEventTimestamp);
}

float getMassNow(Cluster *c, int64_t ts) {
	float m;
	if ((c->lastEventTimestamp - ts) < 0) {
		m = c->mass * (float) exp(((float) (c->lastEventTimestamp - ts)) / (float)clusterMassDecayTauUs);
	}
	else {
		m = c->mass;
	}
	return (m);
}

float distanceToX(Cluster *c, uint16_t x, uint16_t y, int64_t ts) {
	int64_t dt = ts - c->lastUpdateTime;
	dt = (dt < 0) ? 0 : dt;
	float distance = fabsf(((((float)x - c->location_x) + (c->velocityPPT_x * (float)dt)) * c->cosAngle) + ((((float)y - c->location_y) + (c->velocityPPT_y * (float)dt)) * c->sinAngle));
	return (distance);
}

float distanceToY(Cluster *c, uint16_t x, uint16_t y, int64_t ts) {
	int64_t dt = ts - c->lastUpdateTime;
	dt = (dt < 0) ? 0 : dt;
	float distance = fabsf((((y - c->location_y) + (c->velocityPPT_y * (float)dt)) * c->cosAngle) - (((x - c->location_x) + (c->velocityPPT_x * (float)dt)) * c->sinAngle));
	return (distance);
}

static float distanceC1C2(Cluster *c1, Cluster *c2) {
	float dx = c1->location_x - c2->location_x;
	float dy = c1->location_y - c2->location_y;
	return (fabsf(dx) + fabsf(dy));
}

static float distanceToEvent(Cluster *c, uint16_t x, uint16_t y) {
	float dx = (float)x - c->location_x;
	float dy = (float)y - c->location_y;

	return (fabsf(dx) + fabsf(dy));
}

static void checkAndSetClusterVisibilityFlag(caerModuleData moduleData) {
	RTFilterState state = moduleData->moduleState;

	for (int i = 0; i < state->maxClusterNum; i++) {
		if (!state->clusterList[i].isEmpty){
			bool ret = true;
			if ((state->clusterList[i].numEvents < state->thresholdMassForVisibleCluster) || ((state->clusterList[i].numEvents > state->thresholdMassForVisibleCluster) && (state->clusterList[i].mass < state->thresholdMassForVisibleCluster))) {
				ret = false;
			}
			if (useVelocity) {
				float speed = (sqrt((state->clusterList[i].velocityPPT_x * state->clusterList[i].velocityPPT_x) + (state->clusterList[i].velocityPPT_y * state->clusterList[i].velocityPPT_y)) * 1e6f); // speed is in pixels/sec
				if (speed < thresholdVelocityForVisibleCluster) {
					ret = false;
				}
			}
			if (!state->clusterList[i].hasObtainedSupport && ret) {
				state->clusterList[i].birthLocation_x = state->clusterList[i].location_x;
				state->clusterList[i].birthLocation_y = state->clusterList[i].location_y; // reset location of birth to presumably less noisy current location.
			}
			state->clusterList[i].hasObtainedSupport = ret;
			state->clusterList[i].visibilityFlag = ret;
		}
	}
}

static void updateClusterLocations(caerModuleData moduleData, int64_t ts) {
	RTFilterState state = moduleData->moduleState;

	if (!useVelocity) {
		return;
	}

	for (int i = 0; i < state->maxClusterNum; i++) {
		if ((!state->clusterList[i].isEmpty) && state->clusterList[i].velocityValid) {
			int64_t dt = ts - state->clusterList[i].lastUpdateTime;
			if (dt <= 0) {
				continue; // bogus timestamp or doesn't need update
			}
			state->clusterList[i].location_x += state->clusterList[i].velocityPPT_x * (float)dt * predictiveVelocityFactor;
			state->clusterList[i].location_y += state->clusterList[i].velocityPPT_y * (float)dt * predictiveVelocityFactor;
			if (initializeVelocityToAverage) {
				// update average velocity metric for construction of new Clusters
				averageVelocityPPT_x = ((1.0f - AVERAGE_VELOCITY_MIXING_FACTOR) * averageVelocityPPT_x) + (AVERAGE_VELOCITY_MIXING_FACTOR * state->clusterList[i].velocityPPT_x);
				averageVelocityPPT_y = ((1.0f - AVERAGE_VELOCITY_MIXING_FACTOR) * averageVelocityPPT_y) + (AVERAGE_VELOCITY_MIXING_FACTOR * state->clusterList[i].velocityPPT_y);
			}
			state->clusterList[i].lastUpdateTime = ts;
		}
	}
}

static void updateClusterPaths(caerModuleData moduleData, int64_t ts) {
	RTFilterState state = moduleData->moduleState;

	if (!state->pathsEnabled && !useVelocity) {
		return;
	}
	for (int i = 0; i < state->maxClusterNum; i++){
		if (!state->clusterList[i].isEmpty){
			if (state->clusterList[i].numEvents == state->clusterList[i].previousNumEvents) {
				continue; // don't add point unless we had events that caused change in path (aside from prediction from velocityPPT)
			}
			addPath(&state->clusterList[i].path, state->clusterList[i].location_x, state->clusterList[i].location_y, ts, state->clusterList[i].numEvents - state->clusterList[i].previousNumEvents);
			state->clusterList[i].previousNumEvents = state->clusterList[i].numEvents;
			if (useVelocity){
				updateVelocity(&state->clusterList[i], state->thresholdMassForVisibleCluster);
			}
			int count = getPathSize(state->clusterList[i].path);
			if (count > pathLength) {
				removeLastPath(state->clusterList[i].path);
			}
		}
	}
}

static int getPathSize(Path * head) {
	int count = 0;
	Path * current = head;
	while (current != NULL) {
		current = current->next;
		count++;
	}
	return (count);
}

static void addPath(Path ** head, float x, float y, int64_t t, int events) {
	Path * new = malloc(sizeof(Path));
	new->location_x = x;
	new->location_y = y;
	new->timestamp = t;
	new->nEvents = events;
	new->next = *head;
	*head = new;
}

static void removeLastPath(Path * head){
	// if there is only one item in the list, remove it
	if (head->next == NULL) {
		free(head);
		head = NULL;
	}
	else {
		Path * current = head;
		while (current->next->next != NULL){
			current = current->next;
		}
		free(current->next);
		current->next = NULL;
	}
}

static void updateVelocity(Cluster *c, float thresholdMassForVisibleCluster) {
	if (getPathSize(c->path) < 2) {
		return;
	}

//	// update velocityPPT of cluster using last two path points
	Path *itr = c->path;
	Path *plast = itr->next;
//	itr = itr->next;
	int nevents = plast->nEvents;
	Path *pfirst = itr->next;
//	itr = itr->next;
	while ((nevents < thresholdMassForVisibleCluster) && (itr->next != NULL)) {
		nevents += pfirst->nEvents;
		pfirst = itr->next;
		itr = itr->next;
	}
	if (nevents < thresholdMassForVisibleCluster) {
		return;
	}

	int64_t dt = plast->timestamp - pfirst->timestamp;
	float vx = (plast->location_x - pfirst->location_x) / (float)dt;
	float vy = (plast->location_y - pfirst->location_y) / (float)dt;

	int64_t dtFilterTime = c->lastEventTimestamp - c->vFilterTime;
	if (dtFilterTime < 0){
		dtFilterTime = 0;
	}
	c->vFilterTime = c->lastEventTimestamp;
	float fac = (float)dtFilterTime / (float)velocityTauMs / (float)TICK_PER_MS;
	if (fac > 1){
		fac = 1;
	}
	c->velocityPPT_x = c->velocityPPT_x + (vx - c->velocityPPT_x) * fac;
	c->velocityPPT_y = c->velocityPPT_y + (vy - c->velocityPPT_y) * fac;
//	if (!Float.isNaN(frictionTauMs)) {
//		float factor = (float) Math.exp(-dt / (frictionTauMs * 1000));
//		velocityPPT.x = velocityPPT.x * factor;
//		velocityPPT.y = velocityPPT.y * factor;
//	}
	plast->velocityPPT_x = c->velocityPPT_x;
	plast->velocityPPT_y = c->velocityPPT_y;

	// float m1=1-velocityMixingFactor;
	// velocityPPT.x=m1*velocityPPT.x+velocityMixingFactor*vx;
	// velocityPPT.y=m1*velocityPPT.y+velocityMixingFactor*vy;
	c->velocityPPS_x = c->velocityPPT_x * VELPPS_SCALING;
	c->velocityPPS_y = c->velocityPPT_y * VELPPS_SCALING;
	c->velocityValid = true;
}

static void setRadius(Cluster *c, float r) {
	c->radius = r;
	c->radius_x = c->radius / c->aspectRatio;
	c->radius_y = c->radius * c->aspectRatio;
}

static void addEvent(caerModuleData moduleData, Cluster *c, uint16_t x, uint16_t y, int64_t ts) {
	updateMass(c, ts);
	updatePosition(c, x, y);
	updateEventRate(c, ts);
	updateAverageEventDistance(c);
	updateShape(moduleData, c, x, y);
	c->lastUpdateTime = ts;
}

static void updatePosition(Cluster *c, uint16_t x, uint16_t y) {
	float m = mixingFactor;
	float m1 = 1 - m;
	float newX = (float)x;
	float newY = (float)y;
	if (!smoothMove) {
		c->location_x = ((m1 * c->location_x) + (m * newX));
		c->location_y = ((m1 * c->location_y) + (m * newY));
	}
	else {
		float errX = (newX - c->location_x);
		float errY = (newY - c->location_y);
		m = m / smoothWeight;
		m1 = 1 - m;
		c->velocity_x = (m1 * c->velocity_x) + (m * errX);
		c->velocity_y = (m1 * c->velocity_y) + (m * errY);
		c->location_x = c->location_x + (c->velocity_x * smoothIntegral) + (errX * smoothPosition);
		c->location_y = c->location_y + (c->velocity_y * smoothIntegral) + (errY * smoothPosition);
	}
}

static void updateEventRate(Cluster *c, int64_t ts) {
	float m = mixingFactor;
	float m1 = 1 - m;
	int64_t prevLastTimestamp = c->lastEventTimestamp;
	c->lastEventTimestamp = ts;
	c->numEvents++;
	c->instantaneousISI = (float)(c->lastEventTimestamp - prevLastTimestamp);
	if (c->instantaneousISI <= 0) {
		c->instantaneousISI = 1;
	}

	c->avgISI = (m1 * c->avgISI) + (m * c->instantaneousISI);
	c->instantaneousEventRate = 1.0f / c->instantaneousISI;
	c->avgEventRate = (m1 * c->avgEventRate) + (m * c->instantaneousEventRate);
}

static void updateAverageEventDistance(Cluster *c) {
	float m = mixingFactor;
	float m1 = 1 - m;
	c->averageEventDistance = (m1 * c->averageEventDistance) + (m * c->distanceToLastEvent);
	c->averageEventXDistance = (m1 * c->averageEventXDistance) + (m * c->distanceToLastEvent_x);
	c->averageEventYDistance = (m1 * c->averageEventYDistance) + (m * c->distanceToLastEvent_y);
}

static void updateMass(Cluster *c, int64_t ts) {
	int64_t dt = c->lastEventTimestamp - ts;
	if (dt < 0){
		c->mass = 1 + (c->mass * (float) exp((float)dt / (float)clusterMassDecayTauUs));
		c->mass = (c->mass > 100000) ? 100000 : c->mass;
	}
}

static void updateClusterMasses(caerModuleData moduleData, int64_t ts) {
	RTFilterState state = moduleData->moduleState;

	for (int i = 0; i < state->maxClusterNum; i++) {
		if (!state->clusterList[i].isEmpty){
			updateMass(&state->clusterList[i], ts);
		}
	}
}


static void updateShape(caerModuleData moduleData, Cluster *c, uint16_t x, uint16_t y) {
	RTFilterState state = moduleData->moduleState;

	if (state->dynamicSizeEnabled) {
		updateSize(c, x, y, state->defaultClusterRadius);
	}
	if (state->dynamicAspectRatioEnabled) {
		updateAspectRatio(c, x, y, state->dynamicAngleEnabled);
	}
	// PI/2 for vertical positive, -Pi/2 for vertical negative event
	if (state->dynamicAngleEnabled) {
		updateAngle(c, x, y);
	}

	// turn cluster so that it is aligned along velocity
	if (angleFollowsVelocity && c->velocityValid && useVelocity) {
		float velAngle = (float) atan2(c->velocityPPS_y, c->velocityPPS_x);
		setAngle(c, velAngle);
	}
}

static void updateSize(Cluster *c, uint16_t x, uint16_t y, float defaultClusterRadius) {
	float m = mixingFactor;
	float m1 = 1 - m;

	float dist = distanceToEvent(c, x, y);
	float oldr = c->radius;
	float newr = m1 * oldr + m * dist;
	float f;
	if (newr > (f = defaultClusterRadius * MAX_SCALE_RATIO)) {
		newr = f;
	}
	else if (newr < (f = defaultClusterRadius / MAX_SCALE_RATIO)) {
		newr = f;
	}
	setRadius(c, newr);
}

static void updateAspectRatio(Cluster *c, uint16_t x, uint16_t y, bool dynamicAngleEnabled) {
	float m = mixingFactor;
	float m1 = 1 - m;
	float dx = (float)x - c->location_x;
	float dy = (float)y - c->location_y;
	float dw = (dx * c->cosAngle) + (dy * c->sinAngle); // dot dx,dy with unit vector of instantaneousAngle of cluster
	float dh = (-dx * c->sinAngle) + (dy * c->cosAngle); // and with normal to unit vector
	float oldAspectRatio = c->aspectRatio;
	float newAspectRatio = fabsf(dh / dw);
	if (dynamicAngleEnabled) {
		if (newAspectRatio > ASPECT_RATIO_MAX_DYNAMIC_ANGLE_ENABLED) {
			newAspectRatio = ASPECT_RATIO_MAX_DYNAMIC_ANGLE_ENABLED;
		}
		else if (newAspectRatio < ASPECT_RATIO_MIN_DYNAMIC_ANGLE_ENABLED) {
			newAspectRatio = ASPECT_RATIO_MIN_DYNAMIC_ANGLE_ENABLED;
		}
	}
	else {
		if (newAspectRatio > ASPECT_RATIO_MAX_DYNAMIC_ANGLE_DISABLED) {
			newAspectRatio = ASPECT_RATIO_MAX_DYNAMIC_ANGLE_DISABLED;
		}
		else if (newAspectRatio < ASPECT_RATIO_MIN_DYNAMIC_ANGLE_DISABLED) {
			newAspectRatio = ASPECT_RATIO_MIN_DYNAMIC_ANGLE_DISABLED;
		}
	}
	c->aspectRatio = (m1 * oldAspectRatio + m * newAspectRatio);
	setRadius(c, c->radius);
}

static void updateAngle(Cluster *c, uint16_t x, uint16_t y) {
	float dx = c->location_x - (float)x;
	float dy = c->location_y - (float)y;
	float newAngle = (float) atan2(dy, dx);
	if (newAngle < 0) {
		newAngle += (float) M_PI;
	}
	float diff = newAngle - c->angle;
	if ((diff) > ((float)M_PI / 2.0f)) {
		newAngle = newAngle - (float) M_PI;
	}
	else if (diff < (-(float)M_PI / 2.0f)) {
		newAngle = -(float) M_PI + newAngle;
	}
	float angleDistance = newAngle - c->angle;
	setAngle(c, c->angle + (mixingFactor * angleDistance));
}

static void setAngle(Cluster *c, float angle) {
	float m = mixingFactor;
	float m1 = 1 - m;
	if (c->angle != angle) {
		c->angle = angle;
		c->cosAngle = (float) cos(angle);
		c->sinAngle = (float) sin(angle);
		initialAngle = ( m1 * initialAngle) + (m * angle);
	}
}

static bool hasHitEdge(Cluster *c, int16_t sizeX, int16_t sizeY) {
	if (!enableClusterExitPurging) {
		return (false);
	}

	int lx = (int) c->location_x;
	int	ly = (int) c->location_y;
	int sx = sizeX;
	int	sy = sizeY;

	return ((lx <= 0) || (lx >= sx) || (ly <= 0) || (ly >= sy));
}

static bool isOverlapping(Cluster *c1, Cluster *c2) {
	bool overlapping = (distanceC1C2(c1, c2) < (c1->radius + c2->radius));
	return (overlapping);
}

static float velocityAngleToRad(Cluster *c1, Cluster *c2) {
	float s1 = (float) sqrt((c1->velocityPPS_x * c1->velocityPPS_x) + (c1->velocityPPS_y * c1->velocityPPS_y));
	float s2 = (float) sqrt((c2->velocityPPS_x * c2->velocityPPS_x) + (c2->velocityPPS_y * c2->velocityPPS_y));

	if ((s1 == 0.0f) || (s2 == 0.0f)) {
		return (0.0f);
	}
	float dot = (c1->velocityPPS_x * c2->velocityPPS_x) + (c1->velocityPPS_y * c2->velocityPPS_y);
	float angleRad = (float) acos(dot / s1 / s2);
	return (angleRad);
}

static float getDXl(Cluster *c){
	float rx = c->radius_x;
	float ry = c->radius_y;
	float angle = c->angle;
	float DXl = (rx / cos(angle)) - (ry + rx * tan(angle)) * sin(angle);
	return (DXl);
}

static float getDXs(Cluster *c){
	float rx = c->radius_x;
	float ry = c->radius_y;
	float angle = c->angle;
	float DXs = (rx / cos(angle)) + (ry - rx * tan(angle)) * sin(angle);
	return (DXs);
}

static float getDYl(Cluster *c){
	float rx = c->radius_x;
	float ry = c->radius_y;
	float angle = c->angle;
	float DYl = (ry + rx * tan(angle)) * cos(angle);
	return (DYl);
}

static float getDYs(Cluster *c){
	float rx = c->radius_x;
	float ry = c->radius_y;
	float angle = c->angle;
	float DYs = (ry - rx * tan(angle)) * cos(angle);
	return (DYs);
}

static void drawCluster(caerFrameEvent singleplot, Cluster *c, int sizeX, int sizeY, bool showPaths) {
	uint32_t counter = 0;
	for (size_t y = 0; y < sizeY; y++) {
		for (size_t x = 0; x < sizeX; x++) {
//			if (c->angle != 0){
//				float A = c->angle;
//				float Width = c->radius_x * 2;
//				float Height = c->radius_y * 2;
//				float UL_x = c->location_x + ( Width / 2 ) * cos(A) - ( Height / 2 ) * sin(A);
//				float UL_y = c->location_y + ( Height / 2 ) * cos(A)  + ( Width / 2 ) * sin(A);
//				float UR_x = c->location_x - ( Width / 2 ) * cos(A) - ( Height / 2 ) * sin(A);
//				float UR_y = c->location_y + ( Height / 2 ) * cos(A)  - ( Width / 2 ) * sin(A);
//				float BL_x = c->location_x + ( Width / 2 ) * cos(A) + ( Height / 2 ) * sin(A);
//				float BL_y = c->location_y - ( Height / 2 ) * cos(A)  + ( Width / 2 ) * sin(A);
//				float BR_x = c->location_x - ( Width / 2 ) * cos(A) + ( Height / 2 ) * sin(A);
//				float BR_y = c->location_y - ( Height / 2 ) * cos(A)  - ( Width / 2 ) * sin(A);
//
//				float Xa = UL_x;
//				float Ya = UL_y;
//				float Xb = BL_x;
//				float Yb = BL_y;
//				float Xc = BR_x;
//				float Yc = BR_y;
//				float Xd = UR_x;
//				float Yd = UR_y;
//				if ( (x == (int)c->location_x && y == (int)c->location_y)||
//						((x > Xa) && (x < Xd) && (y > Ya) && (y < Yd) && ((((x - Xa) / (y - Ya)) - ((Xd - Xa) / (Yd - Ya))) < 0.0001 )) ||
//						((x > Xb) && (x < Xc) && (y > Yb) && (y < Yc) && ((((x - Xb) / (y - Yb)) - ((Xc - Xb) / (Yc - Yb))) < 0.0001 )) ||
//						((x > Xa) && (x < Xb) && (y > Yb) && (y < Ya) && ((((x - Xa) / (y - Ya)) - ((Xb - Xa) / (Yb - Ya))) < 0.0001 )) ||
//						((x > Xd) && (x < Xc) && (y > Yc) && (y < Yd) && ((((x - Xd) / (y - Yd)) - ((Xc - Xd) / (Yc - Yd))) < 0.0001 )) )
//				{
//					singleplot->pixels[counter] = (uint16_t) ( (int) 65000);			// red
//					singleplot->pixels[counter + 1] = (uint16_t) ( (int) 65000);		// green
//					singleplot->pixels[counter + 2] = (uint16_t) ( (int) 65000);	// blue
//				}
//				counter += 3;
//			}



//
//			if (c->angle != 0){
//				float DXl = getDXl(c);
//				float DXs = getDXs(c);
//				float DYl = getDYl(c);
//				float DYs = getDYs(c);
//				float Xa = c->location_x - DXs;
//				float Xb = c->location_x - DXl;
//				float Xc = c->location_x + DXs;
//				float Xd = c->location_x + DXl;
//				float Ya = c->location_y + DYs;
//				float Yb = c->location_y - DYl;
//				float Yc = c->location_y - DYs;
//				float Yd = c->location_y + DYl;
//
//				if ( (x == (int)c->location_x && y == (int)c->location_y)||
//					 ((x > Xa) && (x < Xd) && (y > Ya) && (y < Yd) && ((((x - Xa) / (y - Ya)) - ((Xd - Xa) / (Yd - Ya))) < 0.0001 )) ||
//					 ((x > Xb) && (x < Xc) && (y > Yb) && (y < Yc) && ((((x - Xb) / (y - Yb)) - ((Xc - Xb) / (Yc - Yb))) < 0.0001 )) ||
//					 ((x > Xa) && (x < Xb) && (y > Yb) && (y < Ya) && ((((x - Xa) / (y - Ya)) - ((Xb - Xa) / (Yb - Ya))) < 0.0001 )) ||
//					 ((x > Xd) && (x < Xc) && (y > Yc) && (y < Yd) && ((((x - Xd) / (y - Yd)) - ((Xc - Xd) / (Yc - Yd))) < 0.0001 )) )
//				{
//					singleplot->pixels[counter] = (uint16_t) ( (int) 1);			// red
//					singleplot->pixels[counter + 1] = (uint16_t) ( (int) 1);		// green
//					singleplot->pixels[counter + 2] = (uint16_t) ( (int) 65000);	// blue
//				}
//				counter += 3;
//			}
			if (c->angle == 0) {
				if ( (x == (int)c->location_x && y == (int)c->location_y)||
					 (x == (int)(c->location_x + c->radius_x) && y <= (c->location_y + c->radius_y) && y >= (c->location_y - c->radius_y)) ||
					 (x == (int)(c->location_x - c->radius_x) && y <= (c->location_y + c->radius_y) && y >= (c->location_y - c->radius_y)) ||
					 (y == (int)(c->location_y + c->radius_y) && x <= (c->location_x + c->radius_x) && x >= (c->location_x - c->radius_x)) ||
					 (y == (int)(c->location_y - c->radius_y) && x <= (c->location_x + c->radius_x) && x >= (c->location_x - c->radius_x)) )
				{
					singleplot->pixels[counter] = (uint16_t) ( (int) 65000);			// red
					singleplot->pixels[counter + 1] = (uint16_t) ( (int) 65000);		// green
					singleplot->pixels[counter + 2] = (uint16_t) ( (int) 65000);	// blue
				}
				counter += 3;
			}
			if (showPaths) {
				Path *current = c->path;
				while (current != NULL){
					if (x == (int)current->location_x && y == (int)current->location_y)
					{	singleplot->pixels[counter] = (uint16_t) ( (int) 1);			// red
						singleplot->pixels[counter + 1] = (uint16_t) ( (int) 1);		// green
						singleplot->pixels[counter + 2] = (uint16_t) ( (int) 65000);	// blue
					}
					current = current->next;
				}
			}
		}
	}
}


static void caerRectangulartrackerConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	RTFilterState state = moduleData->moduleState;
	state->dynamicSizeEnabled = sshsNodeGetBool(moduleData->moduleNode, "dynamicSizeEnabled");
	state->dynamicAspectRatioEnabled = sshsNodeGetBool(moduleData->moduleNode, "dynamicAspectRatioEnabled");
	state->dynamicAngleEnabled = sshsNodeGetBool(moduleData->moduleNode, "dynamicAngleEnabled");
	state->pathsEnabled = sshsNodeGetBool(moduleData->moduleNode, "pathsEnabled");
	state->showPaths = sshsNodeGetBool(moduleData->moduleNode, "showPaths");
	state->maxClusterNum = sshsNodeGetInt(moduleData->moduleNode, "maxClusterNum");
	state->thresholdMassForVisibleCluster = sshsNodeGetFloat(moduleData->moduleNode, "thresholdMassForVisibleCluster");
	state->defaultClusterRadius = sshsNodeGetFloat(moduleData->moduleNode, "defaultClusterRadius");

}

static void caerRectangulartrackerExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	//RTFilterState state = moduleData->moduleState;

}

static void caerRectangulartrackerReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);

}
