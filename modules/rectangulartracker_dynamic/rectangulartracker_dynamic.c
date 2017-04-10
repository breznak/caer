/*
 * rectangulartracker_dynamic.c
 *
 *	This rectangular tracker has a dynamic linked list structure for the list of all clusters
 *	So the total number of clusters can be changed to any non-negative number during running
 *	May has bugs
 *
 *  Created on: Jan 2017
 *      Author: Tianyu
 */

#include "rectangulartracker_dynamic.h"
#include "base/mainloop.h"
#include "base/module.h"
//#include "ext/buffers.h"
#include "math.h"
#include "ext/colorjet/colorjet.h"
#include "wrapper.h"

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
	int64_t clusterNumber;
	float avgEventRate;
	float radius;
	float aspectRatio;
	float radius_x;
	float radius_y;
	Path * path;
	float avgISI;
	COLOUR color;
	bool velocityValid;
	bool visibilityFlag;
	float instantaneousISI;
	float distanceToLastEvent;
	float distanceToLastEvent_x;
	float distanceToLastEvent_y;
	float mass;
	int64_t vFilterTime;
	bool inBotZone;
	bool inTopZone;
} Cluster;

typedef struct clusterList {
	Cluster * cluster;
	struct clusterList * next;
} ClusterList;

struct RTFilter_state {
	ClusterList ** clusterBegin;
	int currentClusterNum;
	bool dynamicSizeEnabled;
	bool dynamicAspectRatioEnabled;
	bool dynamicAngleEnabled;
	bool pathsEnabled;
	float thresholdMassForVisibleCluster;
	int maxClusterNum;
	float defaultClusterRadius;
	bool showPaths;
	bool forceBoundary;
	bool smoothMove;
	bool useVelocity;
	bool initializeVelocityToAverage;
	bool growMergedSizeEnabled;
	bool angleFollowsVelocity;
	bool useNearestCluster;
	float aspectRatio;
	bool peopleCounting;
	bool resetCountingNum;
	int totalPeopleNum;
	float botLine;
	float topLine;
	float leftLine;
	float rightLine;
	bool useOnePolarityOnlyEnabled;
	bool useOffPolarityOnlyEnabled;
	bool showAllClusters;
	struct OpenCV* cpp_class;
	int64_t clusterCounter;
	bool dontMergeEver;
	int clusterMassDecayTauUs;
	int pathLength;
	float mixingFactor;
	int peopleIn;
	int peopleOut;
	bool disableEvents;
	int disableArea_small_x;
	int disableArea_small_y;
	int disableArea_big_x;
	int disableArea_big_y;
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
static const float FULL_BRIGHTNESS_LIFETIME = 100000.0f;


static float surround = 2.0f;
static bool updateTimeInitialized = false;
static int64_t nextUpdateTimeUs = 0;
static int updateIntervalUs = 1000;

static float smoothWeight = 100.0f;
static float smoothPosition = 0.001f;
static float smoothIntegral = 0.001f;
static float velAngDiffDegToNotMerge = 60.0f;

static float thresholdVelocityForVisibleCluster = 0.0f;

//static bool useEllipticalClusters = false;
//static bool colorClustersDifferentlyEnabled = false;

static float predictiveVelocityFactor = 1;

static bool enableClusterExitPurging = true;
//static bool showClusterNumber = false;
//static bool showClusterEps = false;
//static bool showClusterRadius = false;
//static bool showClusterVelocity = false;
//static bool showClusterMass = false;
//static float velocityVectorScaling = 1.0f;
//static bool filterEventsEnabled = false;
static float velocityTauMs = 100.0f;
//static float frictionTauMs = 0.0 / 0.0; // Float.NaN in java;

static float initialAngle = 0.0f;
static float averageVelocityPPT_x = 0.0f;
static float averageVelocityPPT_y = 0.0f;

// cluster list always begine from this pointer
ClusterList * clusterBeginPointer = NULL;


typedef struct RTFilter_state *RTFilterState;

static bool caerRectangulartrackerDynamicInit(caerModuleData moduleData);
static void caerRectangulartrackerDynamicRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerRectangulartrackerDynamicConfig(caerModuleData moduleData);
static void caerRectangulartrackerDynamicExit(caerModuleData moduleData);
static void caerRectangulartrackerDynamicReset(caerModuleData moduleData, uint16_t resetCallSourceID);
static Cluster * getNearestCluster(RTFilterState state, uint16_t x, uint16_t y, int64_t ts);
static Cluster * getFirstContainingCluster(RTFilterState state, uint16_t x, uint16_t y, int64_t ts);
static void updateClusterList(RTFilterState state, int64_t ts, int16_t sizeX, int16_t sizeY);
static void pruneClusters(RTFilterState state, int64_t ts, int16_t sizeX, int16_t sizeY);
static void mergeClusters(RTFilterState state);
static void mergeC1C2(RTFilterState state, Cluster * C1, Cluster * C2);
static int64_t getLifetime(Cluster *c);
static float getMassNow(Cluster *c, int64_t ts, int clusterMassDecayTauUs);
static float distanceToX(Cluster *c, uint16_t x, uint16_t y, int64_t ts);
static float distanceToY(Cluster *c, uint16_t x, uint16_t y, int64_t ts);
static float distanceC1C2(Cluster *c1, Cluster *c2);
static float distanceToEvent(Cluster *c, uint16_t x, uint16_t y);
static void updatePosition(Cluster *c, uint16_t x, uint16_t y, bool smoothMove, float mixingFactor);
static void checkAndSetClusterVisibilityFlag(RTFilterState state);
static void setRadius(Cluster *c, float r);
static void updateMass(Cluster *c, int64_t ts, int clusterMassDecayTauUs);
static void updateClusterMasses(RTFilterState state, int64_t ts);
static void addEvent(RTFilterState state, Cluster *c, uint16_t x, uint16_t y, int64_t ts);
static void updateEventRate(Cluster *c, int64_t ts, float mixingFactor);
static Cluster * generateNewCluster(RTFilterState state, uint16_t x, uint16_t y, int64_t ts);
static void updateAverageEventDistance(Cluster *c, float mixingFactor);
static void updateShape(RTFilterState state, Cluster *c, uint16_t x, uint16_t y);
static void updateSize(Cluster *c, uint16_t x, uint16_t y, float defaultClusterRadius, float mixingFactor);
static void updateAspectRatio(Cluster *c, uint16_t x, uint16_t y, bool dynamicAngleEnabled, float mixingFactor);
static void updateAngle(Cluster *c, uint16_t x, uint16_t y, float mixingFactor);
static void setAngle(Cluster *c, float angle, float mixingFactor);
static bool hasHitEdge(Cluster *c, int16_t sizeX, int16_t sizeY);
static bool isOverlapping(Cluster *c1, Cluster *c2);
static float velocityAngleToRad(Cluster *c1, Cluster *c2);
static void updateClusterLocations(RTFilterState state, int64_t ts);
static int getPathSize(Path * head);
static void removeLastPath(Path * head);
static void removeAllPath(Path * head);
static void addPath(Path ** head, float x, float y, int64_t t, int events);
static void updateVelocity(Cluster *c, float thresholdMassForVisibleCluster);
static void updateClusterPaths(RTFilterState state, int64_t ts);
static void drawCluster(caerFrameEvent singleplot, Cluster *c, int sizeX, int sizeY, bool showPaths, bool forceBoundary);
static void drawline(caerFrameEvent singleplot, float x1, float y1, float x2, float y2, int sizeX, int sizeY, COLOUR color);
static void drawpath(caerFrameEvent singleplot, Path *path, int sizeX);
static void updateCurrentClusterNum(RTFilterState state);
static void updateColor(Cluster *c);
static void checkCountingArea(RTFilterState state, int16_t sizeX, int16_t sizeY);
static void countPeople(caerFrameEvent singleplot, caerModuleData moduleData, int16_t sizeX, int16_t sizeY);

static void addCluster(ClusterList ** head, Cluster * newClusterPointer);
static void removeCluster(ClusterList ** head, int64_t clusterID);
static Cluster * findClusterByIndex(ClusterList ** head, int index);

static struct caer_module_functions caerRectangulartrackerDynamicFunctions = { .moduleInit = &caerRectangulartrackerDynamicInit, .moduleRun = &caerRectangulartrackerDynamicRun, .moduleConfig = &caerRectangulartrackerDynamicConfig, .moduleExit = &caerRectangulartrackerDynamicExit, .moduleReset = &caerRectangulartrackerDynamicReset };

void caerRectangulartrackerDynamicFilter(uint16_t moduleID, caerPolarityEventPacket polarity, caerFrameEventPacket frame) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "RT_Dynamic_Filter", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}
	caerModuleSM(&caerRectangulartrackerDynamicFunctions, moduleData,
	sizeof(struct RTFilter_state), 2, polarity, frame);
}

static bool caerRectangulartrackerDynamicInit(caerModuleData moduleData) {
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "dynamicSizeEnabled", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "dynamicAspectRatioEnabled", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "dynamicAngleEnabled", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "pathsEnabled", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "showPaths", false);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "maxClusterNum", 10);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "thresholdMassForVisibleCluster", 30.0f);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "defaultClusterRadius", 25.0f);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "forceBoundary", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "smoothMove", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "useVelocity", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "initializeVelocityToAverage", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "growMergedSizeEnabled", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "angleFollowsVelocity", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "useNearestCluster", true);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "aspectRatio", 1.0f);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "peopleCounting", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "resetCountingNum", false);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "totalPeopleNum", 0);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "botLine", 0.5f);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "topLine", 0.6f);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "leftLine", 0.01f);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "rightLine", 0.99f);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "useOnePolarityOnlyEnabled", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "useOffPolarityOnlyEnabled", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "showAllClusters", false);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "dontMergeEver", false);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "clusterMassDecayTauUs", 10000);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "pathLength", 100);
	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "mixingFactor", 0.005f);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "peopleIn", 0);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "peopleOut", 0);
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "disableEvents", false);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "disableArea_small_x", 0);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "disableArea_small_y", 0);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "disableArea_big_x", 0);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "disableArea_big_y", 0);

	RTFilterState state = moduleData->moduleState;

	state->dynamicSizeEnabled = sshsNodeGetBool(moduleData->moduleNode, "dynamicSizeEnabled");
	state->dynamicAspectRatioEnabled = sshsNodeGetBool(moduleData->moduleNode, "dynamicAspectRatioEnabled");
	state->dynamicAngleEnabled = sshsNodeGetBool(moduleData->moduleNode, "dynamicAngleEnabled");
	state->pathsEnabled = sshsNodeGetBool(moduleData->moduleNode, "pathsEnabled");
	state->showPaths = sshsNodeGetBool(moduleData->moduleNode, "showPaths");
	state->maxClusterNum = sshsNodeGetInt(moduleData->moduleNode, "maxClusterNum");
	state->thresholdMassForVisibleCluster = sshsNodeGetFloat(moduleData->moduleNode, "thresholdMassForVisibleCluster");
	state->defaultClusterRadius = sshsNodeGetFloat(moduleData->moduleNode, "defaultClusterRadius");
	state->forceBoundary = sshsNodeGetBool(moduleData->moduleNode, "forceBoundary");
	state->smoothMove = sshsNodeGetBool(moduleData->moduleNode, "smoothMove");
	state->useVelocity = sshsNodeGetBool(moduleData->moduleNode, "useVelocity");
	state->initializeVelocityToAverage = sshsNodeGetBool(moduleData->moduleNode, "initializeVelocityToAverage");
	state->growMergedSizeEnabled = sshsNodeGetBool(moduleData->moduleNode, "growMergedSizeEnabled");
	state->angleFollowsVelocity = sshsNodeGetBool(moduleData->moduleNode, "angleFollowsVelocity");
	state->useNearestCluster = sshsNodeGetBool(moduleData->moduleNode, "useNearestCluster");
	state->aspectRatio = sshsNodeGetFloat(moduleData->moduleNode, "aspectRatio");
	state->peopleCounting = sshsNodeGetBool(moduleData->moduleNode, "peopleCounting");
	state->resetCountingNum = sshsNodeGetBool(moduleData->moduleNode, "resetCountingNum");
	state->totalPeopleNum = sshsNodeGetInt(moduleData->moduleNode, "totalPeopleNum");
	state->botLine = sshsNodeGetFloat(moduleData->moduleNode, "botLine");
	state->topLine = sshsNodeGetFloat(moduleData->moduleNode, "topLine");
	state->leftLine = sshsNodeGetFloat(moduleData->moduleNode, "leftLine");
	state->rightLine = sshsNodeGetFloat(moduleData->moduleNode, "rightLine");
	state->useOnePolarityOnlyEnabled = sshsNodeGetBool(moduleData->moduleNode, "useOnePolarityOnlyEnabled");
	state->useOffPolarityOnlyEnabled = sshsNodeGetBool(moduleData->moduleNode, "useOffPolarityOnlyEnabled");
	state->showAllClusters = sshsNodeGetBool(moduleData->moduleNode, "showAllClusters");
	state->dontMergeEver = sshsNodeGetBool(moduleData->moduleNode, "dontMergeEver");
	state->clusterMassDecayTauUs = sshsNodeGetInt(moduleData->moduleNode, "clusterMassDecayTauUs");
	state->pathLength = sshsNodeGetInt(moduleData->moduleNode, "pathLength");
	state->mixingFactor = sshsNodeGetFloat(moduleData->moduleNode, "mixingFactor");

	state->currentClusterNum = 0;
	state->clusterCounter = 0;
	state->cpp_class = newOpenCV();

	// people counting initialization
	state->peopleIn = 0;
	state->peopleOut = 0;

	// disable events initialization
	state->disableEvents = false;
	state->disableArea_small_x = 0;
	state->disableArea_small_y = 0;
	state->disableArea_big_x = 0;
	state->disableArea_big_y = 0;

	state->clusterBegin = &clusterBeginPointer;
	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	// Nothing that can fail here.
	return (true);
}
static void caerRectangulartrackerDynamicRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
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

	ClusterList * current = *(state->clusterBegin);
	while (current != NULL) {
		current->cluster->lastPacketLocation_x = current->cluster->location_x;
		current->cluster->lastPacketLocation_y = current->cluster->location_y;
		current = current->next;
	}

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
	if (!caerPolarityEventIsValid(caerPolarityIteratorElement)){
		continue;
	}
	if ((x >= sizeX) || (y >= sizeY)) {
		continue;
	}
	if (state->disableEvents){
		if ((x > state->disableArea_small_x) && (x < state->disableArea_big_x) && (y > state->disableArea_small_y) && (y < state->disableArea_big_y)){
			continue;
		}
	}

	if (state->useOnePolarityOnlyEnabled) {
		if (state->useOffPolarityOnlyEnabled) {
			if (eventType == 1) {
				continue;
			}
		}
		else {
			if (eventType == 0) {
				continue;
			}
		}
	}
	updateCurrentClusterNum(state);

	// check nearestCluster exist?
	Cluster * chosenCluster = NULL;
	if (state->useNearestCluster){
		chosenCluster = getNearestCluster(state, x, y, ts);
	}
	else {
		chosenCluster = getFirstContainingCluster(state, x, y, ts);
	}

	// if exist, update it
	if (chosenCluster != NULL){
		addEvent(state, chosenCluster, x, y, ts);
	}

	// if not, create new cluster
	else if (state->currentClusterNum < state->maxClusterNum) {
		state->clusterCounter++;
		Cluster * newClusterPointer = generateNewCluster(state, x, y, ts);
		addCluster(state->clusterBegin, newClusterPointer);
	}

	if (!updateTimeInitialized) {
		nextUpdateTimeUs = ts + updateIntervalUs;
		updateTimeInitialized = true;
	}
	if (ts > nextUpdateTimeUs) {
		nextUpdateTimeUs = ts + updateIntervalUs;
		updateCurrentClusterNum(state);
		updateClusterList(state, ts, sizeX, sizeY);
	}

	CAER_POLARITY_ITERATOR_VALID_END

	//plot events
	*frame = caerFrameEventPacketAllocate(1, I16T(moduleData->moduleID), 0, sizeX, sizeY, 3);
	caerMainloopFreeAfterLoop(&free, *frame);
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
	current = *(state->clusterBegin);
	while (current != NULL){
		if ((*frame != NULL ) && (current->cluster->visibilityFlag || state->showAllClusters)) {
			updateColor(current->cluster);
			drawCluster(caerFrameEventPacketGetEvent(*frame, 0), current->cluster, sizeX, sizeY, state->showPaths, state->forceBoundary);
		}
		current = current->next;
	}

	// people counting
	if(state->peopleCounting) {
		countPeople(caerFrameEventPacketGetEvent(*frame, 0), moduleData, sizeX, sizeY);
	}
}

static Cluster * getNearestCluster(RTFilterState state, uint16_t x, uint16_t y, int64_t ts) {

	Cluster * closest = NULL;
	float minDistance = 10000000.0f;
	float currentDistance = 0.0f;

	ClusterList * current = *(state->clusterBegin);
	while (current != NULL) {
		float rX = current->cluster->radius_x;
		float rY = current->cluster->radius_y;
		if (state->dynamicSizeEnabled) {
			rX *= surround;
			rY *= surround; // the event is captured even when it is in "invisible surround"
		}
		float dx = distanceToX(current->cluster, x, y, ts);
		float dy = distanceToY(current->cluster, x, y, ts);
		if ((dx < rX) && (dy < rY)) {
			currentDistance = dx + dy;
			if (currentDistance < minDistance) {
				closest = current->cluster;
				minDistance = currentDistance;
				current->cluster->distanceToLastEvent = minDistance;
				current->cluster->distanceToLastEvent_x = dx;
				current->cluster->distanceToLastEvent_y = dy;
			}
		}
		current = current->next;
	}

	return (closest);
}


static Cluster * getFirstContainingCluster(RTFilterState state, uint16_t x, uint16_t y, int64_t ts) {

	Cluster * closest = NULL;
	float minDistance = 10000000.0f;
	float currentDistance = 0.0f;

	ClusterList * current = *(state->clusterBegin);
	while (current != NULL) {
		float rX = current->cluster->radius_x;
		float rY = current->cluster->radius_y; // this is surround region for purposes of dynamicSize scaling of cluster size or
		// aspect ratio
		if (state->dynamicSizeEnabled) {
			rX *= surround;
			rY *= surround; // the event is captured even when it is in "invisible surround"
		}
		float dx = distanceToX(current->cluster, x, y, ts);
		float dy = distanceToY(current->cluster, x, y, ts);
		if ((dx < rX) && (dy < rY)) {
			currentDistance = dx + dy;
			closest = current->cluster;
			minDistance = currentDistance;
			current->cluster->distanceToLastEvent = minDistance;
			current->cluster->distanceToLastEvent_x = dx;
			current->cluster->distanceToLastEvent_y = dy;
			break;
		}
		current = current->next;
	}
	return (closest);
}

static void updateClusterList(RTFilterState state, int64_t ts, int16_t sizeX, int16_t sizeY) {
	pruneClusters(state, ts, sizeX, sizeY);
	mergeClusters(state);
	updateClusterLocations(state, ts);
	updateClusterPaths(state, ts);
	updateClusterMasses(state, ts);
	checkAndSetClusterVisibilityFlag(state);
}

static Cluster * generateNewCluster(RTFilterState state, uint16_t x, uint16_t y, int64_t ts) {

	Cluster * clusterNew = malloc(sizeof(Cluster));
	clusterNew->location_x = (float)x;
	clusterNew->location_y = (float)y;
	clusterNew->velocity_x = 0.0f;
	clusterNew->velocity_y = 0.0f;
	clusterNew->birthLocation_x = (float)x;
	clusterNew->birthLocation_y = (float)y;
	clusterNew->lastPacketLocation_x = (float)x;
	clusterNew->lastPacketLocation_y = (float)y;
	clusterNew->velocityPPT_x = 0.0f;
	clusterNew->velocityPPT_y = 0.0f;
	clusterNew->velocityPPS_x = 0.0f;
	clusterNew->velocityPPS_y = 0.0f;
	clusterNew->velocityValid = false;
	if (state->initializeVelocityToAverage) {
		clusterNew->velocityPPT_x = averageVelocityPPT_x;
		clusterNew->velocityPPT_y = averageVelocityPPT_y;
		clusterNew->velocityValid = true;
	}
	clusterNew->angle = 0.0f;
	clusterNew->cosAngle = 1.0f;
	clusterNew->sinAngle = 0.0f;
	clusterNew->numEvents = 1;
	clusterNew->previousNumEvents = 0;
	clusterNew->lastEventTimestamp = ts;
	clusterNew->lastUpdateTime = ts;
	clusterNew->firstEventTimestamp = ts;
	clusterNew->instantaneousEventRate = 0.0f;
	clusterNew->hasObtainedSupport = false;
	clusterNew->averageEventDistance = 0.0f;
	clusterNew->averageEventXDistance = 0.0f;
	clusterNew->averageEventYDistance = 0.0f;
	clusterNew->clusterNumber = state->clusterCounter;
	clusterNew->avgEventRate = 0.0f;
	clusterNew->avgISI = 0.0f;
	clusterNew->aspectRatio = state->aspectRatio;
	clusterNew->radius = state->defaultClusterRadius;
	clusterNew->radius_x = state->defaultClusterRadius / state->aspectRatio;
	clusterNew->radius_y = state->defaultClusterRadius * state->aspectRatio;
	clusterNew->visibilityFlag = false;
	clusterNew->distanceToLastEvent = 1000000;
	clusterNew->distanceToLastEvent_x = 1000000;
	clusterNew->distanceToLastEvent_y = 1000000;
	clusterNew->instantaneousISI = 0.0f;
	clusterNew->mass = 1.0f;
	clusterNew->vFilterTime = 0.0f;
	clusterNew->path = NULL;
	clusterNew->inBotZone = false;
	clusterNew->inTopZone = false;
	return (clusterNew);
}

static void pruneClusters(RTFilterState state, int64_t ts, int16_t sizeX, int16_t sizeY) {
	updateCurrentClusterNum(state);

	ClusterList * current = *(state->clusterBegin);
	for (int i = 0; i < state->currentClusterNum; i++){
		int64_t t0 = current->cluster->lastEventTimestamp;
		int64_t timeSinceSupport = ts - t0;
		if (timeSinceSupport == 0) {
			continue;
		}
		bool massTooSmall = false;
		int64_t lifetime = getLifetime(current->cluster);
		if (ts > current->cluster->lastEventTimestamp) {
			lifetime = ts - current->cluster->firstEventTimestamp;
		}
		float massThreshold = state->thresholdMassForVisibleCluster;
		if (((lifetime == 0) || (lifetime >= state->clusterMassDecayTauUs)) && (getMassNow(current->cluster, ts, state->clusterMassDecayTauUs) < massThreshold)) {
			massTooSmall = true;
		}
		bool hitEdge = hasHitEdge(current->cluster, sizeX, sizeY);
		if ((t0 > ts) || massTooSmall || (timeSinceSupport < 0) || hitEdge) {
			removeAllPath(current->cluster->path);
			int64_t removeID = current->cluster->clusterNumber;
			current = current->next;
			removeCluster(state->clusterBegin, removeID);
			updateCurrentClusterNum(state);
		}
		else {
			current = current->next;
		}
	}

}

static void mergeClusters(RTFilterState state) {
	updateCurrentClusterNum(state);
	if (state->dontMergeEver) {
		return;
	}

	bool mergePending;
	int i, j;
	Cluster * C1 = NULL;
	Cluster * C2 = NULL;
	do {
		mergePending = false;
		for (i = 0; i < state->currentClusterNum; i++) {
			C1 = findClusterByIndex(state->clusterBegin, i);
			if (C1 != NULL) {
				for (j = i + 1; j < state->currentClusterNum; j++) {
					C2 = findClusterByIndex(state->clusterBegin, j);
					if ((C1 != NULL) && (C2 != NULL)) {
						bool overlapping = isOverlapping(C1, C2);
						bool velSimilar = true;
						if (overlapping && (velAngDiffDegToNotMerge > 0)
								&& C1->visibilityFlag
								&& C2->visibilityFlag
								&& C1->velocityValid
								&& C2->velocityValid
								&& velocityAngleToRad(C1, C2) > ((velAngDiffDegToNotMerge * (float)M_PI) / 180)) {
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
		if (mergePending && (C1 != NULL) && (C2 != NULL)) {
			mergeC1C2(state, C1, C2);
		}
	} while (mergePending);
	updateCurrentClusterNum(state);
}

static void mergeC1C2(RTFilterState state, Cluster * C1, Cluster * C2) {

	Cluster * weaker = C1->mass > C2->mass ? C2 : C1;
	Cluster * stronger = C1->mass > C2->mass ? C1 : C2;

	float mass = C1->mass + C2->mass;
	stronger->numEvents = C1->numEvents + C2->numEvents;
	stronger->velocity_x = 0.0f;
	stronger->velocity_y = 0.0f;
	stronger->averageEventDistance = ((stronger->averageEventDistance * stronger->mass) + (weaker->averageEventDistance * weaker->mass)) / mass;
	stronger->averageEventXDistance = ((stronger->averageEventXDistance * stronger->mass) + (weaker->averageEventXDistance * weaker->mass)) / mass;
	stronger->averageEventYDistance = ((stronger->averageEventYDistance * stronger->mass) + (weaker->averageEventYDistance * weaker->mass)) / mass;
	stronger->lastEventTimestamp = stronger->lastEventTimestamp > weaker->lastEventTimestamp ? stronger->lastEventTimestamp : weaker->lastEventTimestamp;
	stronger->lastUpdateTime = stronger->lastEventTimestamp;
	stronger->mass = mass;

	if (state->growMergedSizeEnabled) {
		//float R = (C1->radius + C2->radius);
		setRadius(stronger, stronger->radius + (state->mixingFactor * weaker->radius));
	}

	removeAllPath(weaker->path);
	removeCluster(state->clusterBegin, weaker->clusterNumber);
	updateCurrentClusterNum(state);
}

int64_t getLifetime(Cluster *c) {
	return (c->lastUpdateTime - c->firstEventTimestamp);
}

float getMassNow(Cluster *c, int64_t ts, int clusterMassDecayTauUs) {
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

static void checkAndSetClusterVisibilityFlag(RTFilterState state) {

	ClusterList * current = *(state->clusterBegin);
	while (current != NULL) {
		bool ret = true;
		if ((current->cluster->numEvents < state->thresholdMassForVisibleCluster) || ((current->cluster->numEvents > state->thresholdMassForVisibleCluster) && (current->cluster->mass < state->thresholdMassForVisibleCluster))) {
			ret = false;
		}
		if (state->useVelocity) {
			float speed = (sqrt((current->cluster->velocityPPT_x * current->cluster->velocityPPT_x) + (current->cluster->velocityPPT_y * current->cluster->velocityPPT_y)) * 1e6f); // speed is in pixels/sec
			if (speed < thresholdVelocityForVisibleCluster) {
				ret = false;
			}
		}
		if (!current->cluster->hasObtainedSupport && ret) {
			current->cluster->birthLocation_x = current->cluster->location_x;
			current->cluster->birthLocation_y = current->cluster->location_y; // reset location of birth to presumably less noisy current location.
		}
		current->cluster->hasObtainedSupport = ret;
		current->cluster->visibilityFlag = ret;
		current = current->next;
	}
}

static void updateClusterLocations(RTFilterState state, int64_t ts) {
	updateCurrentClusterNum(state);

	if (!state->useVelocity) {
		return;
	}

	ClusterList * current = *(state->clusterBegin);
	for (int i = 0; i < state->currentClusterNum; i++) {
		if ((current->cluster != NULL) && current->cluster->velocityValid) {
			int64_t dt = ts - current->cluster->lastUpdateTime;
			if (dt <= 0) {
				continue; // bogus timestamp or doesn't need update
			}
			current->cluster->location_x += current->cluster->velocityPPT_x * (float)dt * predictiveVelocityFactor;
			current->cluster->location_y += current->cluster->velocityPPT_y * (float)dt * predictiveVelocityFactor;
			if (state->initializeVelocityToAverage) {
				// update average velocity metric for construction of new Clusters
				averageVelocityPPT_x = ((1.0f - AVERAGE_VELOCITY_MIXING_FACTOR) * averageVelocityPPT_x) + (AVERAGE_VELOCITY_MIXING_FACTOR * current->cluster->velocityPPT_x);
				averageVelocityPPT_y = ((1.0f - AVERAGE_VELOCITY_MIXING_FACTOR) * averageVelocityPPT_y) + (AVERAGE_VELOCITY_MIXING_FACTOR * current->cluster->velocityPPT_y);
			}
			current->cluster->lastUpdateTime = ts;
		}
		current = current->next;
	}
}

static void updateClusterPaths(RTFilterState state, int64_t ts) {
	updateCurrentClusterNum(state);
	if (!state->pathsEnabled && !state->useVelocity) {
		return;
	}

	ClusterList * current = *(state->clusterBegin);
	for(int i = 0; i < state->currentClusterNum; i++){
		if (current->cluster->numEvents == current->cluster->previousNumEvents) {
			continue; // don't add point unless we had events that caused change in path (aside from prediction from velocityPPT)
		}
		addPath(&current->cluster->path, current->cluster->location_x, current->cluster->location_y, ts, current->cluster->numEvents - current->cluster->previousNumEvents);
		current->cluster->previousNumEvents = current->cluster->numEvents;
		if (state->useVelocity){
			updateVelocity(current->cluster, state->thresholdMassForVisibleCluster);
		}
		int count = getPathSize(current->cluster->path);
		if (count > state->pathLength) {
			removeLastPath(current->cluster->path);
		}
		current = current->next;
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

static void removeAllPath(Path * head){
	Path * current = head;
	Path * delete = current;
	while (current != NULL){
		delete = current;
		current = current->next;
		free(delete);
	}
}

static void updateVelocity(Cluster *c, float thresholdMassForVisibleCluster) {
	if (getPathSize(c->path) < 2) {
		return;
	}

//	// update velocityPPT of cluster using last two path points
	Path *itr = c->path; // create a iterator of paths

	Path *plast = itr;
	itr = itr->next;
	Path *pfirst = itr;
	itr = itr->next;

	int nevents = plast->nEvents;
	while ((nevents < thresholdMassForVisibleCluster) && (itr != NULL)) {
		nevents += pfirst->nEvents;
		pfirst = itr;
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

static void addEvent(RTFilterState state, Cluster *c, uint16_t x, uint16_t y, int64_t ts) {

	updateMass(c, ts, state->clusterMassDecayTauUs);
	updatePosition(c, x, y, state->smoothMove, state->mixingFactor);
	updateEventRate(c, ts, state->mixingFactor);
	updateAverageEventDistance(c, state->mixingFactor);
	updateShape(state, c, x, y);
	c->lastUpdateTime = ts;
}

static void updatePosition(Cluster *c, uint16_t x, uint16_t y, bool smoothMove, float mixingFactor) {
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

static void updateEventRate(Cluster *c, int64_t ts, float mixingFactor) {
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

static void updateAverageEventDistance(Cluster *c, float mixingFactor) {
	float m = mixingFactor;
	float m1 = 1 - m;
	c->averageEventDistance = (m1 * c->averageEventDistance) + (m * c->distanceToLastEvent);
	c->averageEventXDistance = (m1 * c->averageEventXDistance) + (m * c->distanceToLastEvent_x);
	c->averageEventYDistance = (m1 * c->averageEventYDistance) + (m * c->distanceToLastEvent_y);
}

static void updateMass(Cluster *c, int64_t ts, int clusterMassDecayTauUs) {
	int64_t dt = c->lastEventTimestamp - ts;
	if (dt < 0){
		c->mass = 1 + (c->mass * (float) exp((float)dt / (float)clusterMassDecayTauUs));
		c->mass = (c->mass > 100000) ? 100000 : c->mass;
	}
}

static void updateClusterMasses(RTFilterState state, int64_t ts) {

	ClusterList * current = *(state->clusterBegin);
	while (current != NULL) {
		updateMass(current->cluster, ts, state->clusterMassDecayTauUs);
		current = current->next;
	}

}

static void updateShape(RTFilterState state, Cluster *c, uint16_t x, uint16_t y) {

	if (state->dynamicSizeEnabled) {
		updateSize(c, x, y, state->defaultClusterRadius, state->mixingFactor);
	}
	if (state->dynamicAspectRatioEnabled) {
		updateAspectRatio(c, x, y, state->dynamicAngleEnabled, state->mixingFactor);
	}
	// PI/2 for vertical positive, -Pi/2 for vertical negative event
	if (state->dynamicAngleEnabled) {
		updateAngle(c, x, y, state->mixingFactor);
	}

	// turn cluster so that it is aligned along velocity
	if (state->angleFollowsVelocity && c->velocityValid && state->useVelocity) {
		float velAngle = (float) atan2(c->velocityPPS_y, c->velocityPPS_x);
		setAngle(c, velAngle, state->mixingFactor);
	}
}

static void updateSize(Cluster *c, uint16_t x, uint16_t y, float defaultClusterRadius, float mixingFactor) {
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

static void updateAspectRatio(Cluster *c, uint16_t x, uint16_t y, bool dynamicAngleEnabled, float mixingFactor) {
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

static void updateAngle(Cluster *c, uint16_t x, uint16_t y, float mixingFactor) {
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
	setAngle(c, c->angle + (mixingFactor * angleDistance), mixingFactor);
}

static void setAngle(Cluster *c, float angle, float mixingFactor) {
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

static void drawCluster(caerFrameEvent singleplot, Cluster *c, int sizeX, int sizeY, bool showPaths, bool forceBoundary) {

	float A = c->angle;
	float rx = c->radius_x;
	float ry = c->radius_y;
	float UL_x = c->location_x + rx * (float)cos(A) - ry * (float)sin(A);
	float UL_y = c->location_y + ry * (float)cos(A) + rx * (float)sin(A);
	float UR_x = c->location_x - rx * (float)cos(A) - ry * (float)sin(A);
	float UR_y = c->location_y + ry * (float)cos(A) - rx * (float)sin(A);
	float BL_x = c->location_x + rx * (float)cos(A) + ry * (float)sin(A);
	float BL_y = c->location_y - ry * (float)cos(A) + rx * (float)sin(A);
	float BR_x = c->location_x - rx * (float)cos(A) + ry * (float)sin(A);
	float BR_y = c->location_y - ry * (float)cos(A) - rx * (float)sin(A);

	if (forceBoundary){
		UL_x = (UL_x > (float)sizeX) ? (float)sizeX : UL_x;
		UL_x = (UL_x < 0) ? 0 : UL_x;
		UL_y = (UL_y > (float)sizeY) ? (float)sizeY : UL_y;
		UL_y = (UL_y < 0) ? 0 : UL_y;
		UR_x = (UR_x > (float)sizeX) ? (float)sizeX : UR_x;
		UR_x = (UR_x < 0) ? 0 : UR_x;
		UR_y = (UR_y > (float)sizeY) ? (float)sizeY : UR_y;
		UR_y = (UR_y < 0) ? 0 : UR_y;
		BL_x = (BL_x > (float)sizeX) ? (float)sizeX : BL_x;
		BL_x = (BL_x < 0) ? 0 : BL_x;
		BL_y = (BL_y > (float)sizeY) ? (float)sizeY : BL_y;
		BL_y = (BL_y < 0) ? 0 : BL_y;
		BR_x = (BR_x > (float)sizeX) ? (float)sizeX : BR_x;
		BR_x = (BR_x < 0) ? 0 : BR_x;
		BR_y = (BR_y > (float)sizeY) ? (float)sizeY : BR_y;
		BR_y = (BR_y < 0) ? 0 : BR_y;
	}

	drawline(singleplot, UL_x, UL_y, UR_x, UR_y, sizeX, sizeY, c->color);
	drawline(singleplot, UL_x, UL_y, BL_x, BL_y, sizeX, sizeY, c->color);
	drawline(singleplot, BL_x, BL_y, BR_x, BR_y, sizeX, sizeY, c->color);
	drawline(singleplot, UR_x, UR_y, BR_x, BR_y, sizeX, sizeY, c->color);

	if (showPaths) {
		drawpath(singleplot, c->path, sizeX);
	}
}

static void drawline(caerFrameEvent singleplot, float x1, float y1, float x2, float y2, int sizeX, int sizeY, COLOUR color){
	int x, y, xs, xl, ys, yl, dx, dy, p;
	if (x1 < x2){
		xs = round(x1);
		xl = round(x2);
	}
	else{
		xs = round(x2);
		xl = round(x1);
	}
	if (y1 < y2){
		ys = round(y1);
		yl = round(y2);
	}
	else{
		ys = round(y2);
		yl = round(y1);
	}

	if (xs == xl){
		for(y = ys; y <= yl; y++) {
			x = xs;
			if ((x >= sizeX) || (x < 0) || (y >= sizeY) || (y < 0)){
				continue;
			}
			p = 3*(y*sizeX + x);
			if ((p < 0) || (p >= 3 * sizeX * sizeY)){
				continue;
			}
			singleplot->pixels[p] = color.r;			// red
			singleplot->pixels[p + 1] = color.g;		// green
			singleplot->pixels[p + 2] = color.b;	// blue
		}
	}
	else if (ys == yl){
		for(x = xs; x <= xl; x++) {
			y = ys;
			if ((x >= sizeX) || (x < 0) || (y >= sizeY) || (y < 0)){
				continue;
			}
			p = 3*(y*sizeX + x);
			if ((p < 0) || (p >= 3 * sizeX * sizeY)){
				continue;
			}
			singleplot->pixels[p] = color.r;			// red
			singleplot->pixels[p + 1] = color.g;		// green
			singleplot->pixels[p + 2] = color.b;	// blue
		}
	}
	else {
		dx = xl - xs;
		dy = yl - ys;
		if (dx > dy){
			for(x = xs; x <= xl; x++) {
				y = (round)(y2 - ((y2-y1)/(x2-x1)) * (x2-(float)x));
				if ((x >= sizeX) || (x < 0) || (y >= sizeY) || (y < 0)){
					continue;
				}
				p = 3*(y*sizeX + x);
				if ((p < 0) || (p >= 3 * sizeX * sizeY)){
					continue;
				}
				singleplot->pixels[p] = color.r;			// red
				singleplot->pixels[p + 1] = color.g;		// green
				singleplot->pixels[p + 2] = color.b;	// blue
			}
		}
		else {
			for(y = ys; y <= yl; y++) {
				x = (round)(x2 - ((x2-x1)/(y2-y1)) * (y2-(float)y));
				if ((x >= sizeX) || (x < 0) || (y >= sizeY) || (y < 0)){
					continue;
				}
				p = 3*(y*sizeX + x);
				if ((p < 0) || (p >= 3 * sizeX * sizeY)){
					continue;
				}
				singleplot->pixels[p] = color.r;			// red
				singleplot->pixels[p + 1] = color.g;		// green
				singleplot->pixels[p + 2] = color.b;	// blue
			}
		}
	}
}

static void drawpath(caerFrameEvent singleplot, Path *path, int sizeX){
	Path *current = path;
	while (current != NULL){
		int x = (int)current->location_x;
		int y = (int)current->location_y;
		int p = 3*(y*sizeX + x);

		singleplot->pixels[p] = (uint16_t) ( (int) 65000);			// red
		singleplot->pixels[p + 1] = (uint16_t) ( (int) 1);		// green
		singleplot->pixels[p + 2] = (uint16_t) ( (int) 65000);	// blue

		current = current->next;
	}
}

static void updateCurrentClusterNum(RTFilterState state){

	state->currentClusterNum = 0;

	ClusterList * current = *(state->clusterBegin);
	while (current != NULL) {
		state->currentClusterNum++;
		current = current->next;
	}

}

static void updateColor(Cluster *c){
	float brightness = fmax(0.0f, fmin(1.0f, (float)getLifetime(c) / FULL_BRIGHTNESS_LIFETIME));
	c->color.r = (uint16_t)(65535.0f * brightness);
	c->color.g = (uint16_t)(65535.0f * brightness);
	c->color.b = (uint16_t)(65535.0f * brightness);
}

static void checkCountingArea(RTFilterState state, int16_t sizeX, int16_t sizeY){

	if(state->botLine < 0.0f){
		state->botLine = 0.0f;
	}
	if(state->botLine > sizeY){
		state->botLine = sizeY;
	}
	if(state->topLine < 0.0f){
		state->topLine = 0.0f;
	}
	if(state->topLine > sizeY){
		state->topLine = sizeY;
	}
	if(state->botLine > state->topLine){
		state->botLine = state->topLine;
	}
	if(state->leftLine < 0.0f){
		state->leftLine = 0.0f;
	}
	if(state->leftLine > sizeX){
		state->leftLine = sizeX;
	}
	if(state->rightLine < 0.0f){
		state->rightLine = 0.0f;
	}
	if(state->rightLine > sizeX){
		state->rightLine = sizeX;
	}
	if(state->leftLine > state->rightLine){
		state->leftLine = state->rightLine;
	}
}

static void countPeople(caerFrameEvent singleplot, caerModuleData moduleData, int16_t sizeX, int16_t sizeY){
	RTFilterState state = moduleData->moduleState;

	if (state->resetCountingNum){
		state->peopleIn = 0;
		state->peopleOut = 0;
		state->resetCountingNum = false;
		sshsNodePutBool(moduleData->moduleNode, "resetCountingNum", false);
	}
	checkCountingArea(state, sizeX, sizeY);
	float by = state->botLine * sizeY;
	float ty = state->topLine * sizeY;
	float lx = state->leftLine * sizeX;
	float rx = state->rightLine * sizeX;

	COLOUR lineColor;
	lineColor.b = 65535;
	lineColor.r = 65535;
	lineColor.g = 65535;

	drawline(singleplot, lx, ty, rx, ty, sizeX, sizeY, lineColor);
	drawline(singleplot, lx, by, rx, by, sizeX, sizeY, lineColor);
	drawline(singleplot, lx, ty, lx, by, sizeX, sizeY, lineColor);
	drawline(singleplot, rx, ty, rx, by, sizeX, sizeY, lineColor);

	//TODO make algorithm for x dimension.
	updateCurrentClusterNum(state);
	ClusterList * current = *(state->clusterBegin);
	for (int i=0; i<state->currentClusterNum; i++){
		if(!current->cluster->visibilityFlag){
			current = current->next;
			continue;
		}
		if((current->cluster->location_y < by) && !current->cluster->inBotZone){
			current->cluster->inBotZone = true;
		}
		if((current->cluster->location_y > ty) && !current->cluster->inTopZone){
			current->cluster->inTopZone = true;
		}
		if((current->cluster->location_y < by) && current->cluster->inTopZone){
			current->cluster->inTopZone = false;
			state->peopleIn++;
		}
		if((current->cluster->location_y > ty) && current->cluster->inBotZone){
			current->cluster->inBotZone = false;
			state->peopleOut++;
		}
		current = current->next;
	}
	state->totalPeopleNum = (state->peopleIn - state->peopleOut) > 0 ? (state->peopleIn-state->peopleOut) : 0;
	sshsNodePutInt(moduleData->moduleNode, "peopleIn", state->peopleIn);
	sshsNodePutInt(moduleData->moduleNode, "peopleOut", state->peopleOut);
	sshsNodePutInt(moduleData->moduleNode, "totalPeopleNum", state->totalPeopleNum);

	//add OpenCV info to the frame
	OpenCV_generate(state->cpp_class, state->peopleIn, state->peopleOut, &singleplot, sizeX, sizeY);
}

static void addCluster(ClusterList ** head, Cluster * newClusterPointer) {
    // always add new cluster at the first of list
	ClusterList * new_node;
    new_node = malloc(sizeof(ClusterList));
    new_node->cluster = newClusterPointer;
    new_node->next = (*head);
    (*head) = new_node;

    // always add new cluster at the end of list
//	if ((*head) != NULL){
//		ClusterList * current = (*head);
//		while (current->next != NULL){
//			current = current->next;
//		}
//		current->next = malloc(sizeof(ClusterList));
//		current->next->cluster = newClusterPointer;
//		current->next->next = NULL;
//	}
//	else {
//		(*head) = malloc(sizeof(ClusterList));
//		(*head)->cluster = newClusterPointer;
//		(*head)->next = NULL;
//	}
}

static void removeCluster(ClusterList ** head, int64_t clusterID){
	ClusterList *previous = NULL;
	ClusterList *current = NULL;

	if ((*head) == NULL) {
		return;
	}

	if ((*head)->cluster->clusterNumber == clusterID) {
		ClusterList * removeHead = (*head);
		(*head) = removeHead->next;
		free(removeHead);
		return;
	}

	previous = (*head);
	current = (*head)->next;
	while (current != NULL) {
		if (current->cluster->clusterNumber == clusterID) {
			previous->next = current->next;
			free(current);
			return;
		}
		previous = current;
		current  = current->next;
	}
}

static Cluster * findClusterByIndex(ClusterList ** head, int index){
	ClusterList * current = (*head);
	for(int i = 0; i < index; i++){
		if (current->next == NULL){
			return(NULL);
		}
		current = current->next;
	}
	return(current->cluster);
}

//static void removeAllCluster(ClusterList ** head){
//	ClusterList * current = head;
//	ClusterList * delete = current;
//	while (current != NULL){
//		delete = current;
//		current = current->next;
//		free(delete);
//	}
//}

static void caerRectangulartrackerDynamicConfig(caerModuleData moduleData) {
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
	state->forceBoundary = sshsNodeGetBool(moduleData->moduleNode, "forceBoundary");
	state->smoothMove = sshsNodeGetBool(moduleData->moduleNode, "smoothMove");
	state->useVelocity = sshsNodeGetBool(moduleData->moduleNode, "useVelocity");
	state->initializeVelocityToAverage = sshsNodeGetBool(moduleData->moduleNode, "initializeVelocityToAverage");
	state->growMergedSizeEnabled = sshsNodeGetBool(moduleData->moduleNode, "growMergedSizeEnabled");
	state->angleFollowsVelocity = sshsNodeGetBool(moduleData->moduleNode, "angleFollowsVelocity");
	state->useNearestCluster = sshsNodeGetBool(moduleData->moduleNode, "useNearestCluster");
	state->aspectRatio = sshsNodeGetFloat(moduleData->moduleNode, "aspectRatio");
	state->peopleCounting = sshsNodeGetBool(moduleData->moduleNode, "peopleCounting");
	state->resetCountingNum = sshsNodeGetBool(moduleData->moduleNode, "resetCountingNum");
	state->botLine = sshsNodeGetFloat(moduleData->moduleNode, "botLine");
	state->topLine = sshsNodeGetFloat(moduleData->moduleNode, "topLine");
	state->leftLine = sshsNodeGetFloat(moduleData->moduleNode, "leftLine");
	state->rightLine = sshsNodeGetFloat(moduleData->moduleNode, "rightLine");
	state->useOnePolarityOnlyEnabled = sshsNodeGetBool(moduleData->moduleNode, "useOnePolarityOnlyEnabled");
	state->useOffPolarityOnlyEnabled = sshsNodeGetBool(moduleData->moduleNode, "useOffPolarityOnlyEnabled");
	state->showAllClusters = sshsNodeGetBool(moduleData->moduleNode, "showAllClusters");
	state->dontMergeEver = sshsNodeGetBool(moduleData->moduleNode, "dontMergeEver");
	state->clusterMassDecayTauUs = sshsNodeGetInt(moduleData->moduleNode, "clusterMassDecayTauUs");
	state->pathLength = sshsNodeGetInt(moduleData->moduleNode, "pathLength");
	state->mixingFactor = sshsNodeGetFloat(moduleData->moduleNode, "mixingFactor");
	state->disableEvents = sshsNodeGetBool(moduleData->moduleNode, "disableEvents");
	state->disableArea_small_x = sshsNodeGetInt(moduleData->moduleNode, "disableArea_small_x");
	state->disableArea_small_y = sshsNodeGetInt(moduleData->moduleNode, "disableArea_small_y");
	state->disableArea_big_x = sshsNodeGetInt(moduleData->moduleNode, "disableArea_big_x");
	state->disableArea_big_y = sshsNodeGetInt(moduleData->moduleNode, "disableArea_big_y");
}

static void caerRectangulartrackerDynamicExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	RTFilterState state = moduleData->moduleState;
	deleteOpenCV(state->cpp_class);
}

static void caerRectangulartrackerDynamicReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);
}
