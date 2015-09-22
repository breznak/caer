#include "statistics.h"
#include "base/mainloop.h"
#include "base/module.h"
#include <time.h>

#define STAT_STRING "Total events/second: %10" PRIu64 " - Valid events/second: %10" PRIu64

static bool caerStatisticsInit(caerModuleData moduleData);
static void caerStatisticsRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerStatisticsExit(caerModuleData moduleData);

static struct caer_module_functions caerStatisticsFunctions = { .moduleInit = &caerStatisticsInit, .moduleRun =
	&caerStatisticsRun, .moduleConfig = NULL, .moduleExit = &caerStatisticsExit };

void caerStatistics(uint16_t moduleID, caerEventPacketHeader packetHeader, size_t divisionFactor) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "Statistics");

	caerModuleSM(&caerStatisticsFunctions, moduleData, sizeof(struct caer_statistics_state), 2, packetHeader,
		divisionFactor);
}

static bool caerStatisticsInit(caerModuleData moduleData) {
	caerStatisticsState state = moduleData->moduleState;

	size_t maxStatStringLength = (size_t) snprintf(NULL, 0, STAT_STRING, UINT64_MAX, UINT64_MAX);

	state->currentStatisticsString = calloc(maxStatStringLength + 1, sizeof(char)); // +1 for NUL termination.
	if (state->currentStatisticsString == NULL) {
		return (false);
	}

	return (true);
}

static void caerStatisticsRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerEventPacketHeader packetHeader = va_arg(args, caerEventPacketHeader);
	size_t divisionFactor = va_arg(args, size_t);

	caerStatisticsState state = moduleData->moduleState;
	state->divisionFactor = divisionFactor;

	caerUpdateStatisticsString(packetHeader, state);

	fprintf(stdout, "\r%s", state->currentStatisticsString);
	fflush(stdout);
}

static void caerStatisticsExit(caerModuleData moduleData) {
	caerStatisticsState state = moduleData->moduleState;

	// Reclaim string memory.
	if (state->currentStatisticsString != NULL) {
		free(state->currentStatisticsString);
		state->currentStatisticsString = NULL;
	}
}

void caerUpdateStatisticsString(caerEventPacketHeader packetHeader, caerStatisticsState state) {
	// Only non-NULL packets (with content!) contribute to the event count.
	if (packetHeader != NULL) {
		state->totalEventsCounter += caerEventPacketHeaderGetEventNumber(packetHeader);
		state->validEventsCounter += caerEventPacketHeaderGetEventValid(packetHeader);
	}

	// Print up-to-date statistic roughly every second, taking into account possible deviations.
	struct timespec currentTime;
	clock_gettime(CLOCK_MONOTONIC, &currentTime);

	uint64_t diffNanoTime = (uint64_t) (((int64_t) (currentTime.tv_sec - state->lastTime.tv_sec) * 1000000000)
		+ (int64_t) (currentTime.tv_nsec - state->lastTime.tv_nsec));

	// DiffNanoTime is the difference in nanoseconds; we want to trigger roughly every second.
	if (diffNanoTime >= 1000000000) {
		// Print current values.
		uint64_t totalEventsPerTime = (state->totalEventsCounter * (1000000000 / state->divisionFactor)) / diffNanoTime;
		uint64_t validEventsPerTime = (state->validEventsCounter * (1000000000 / state->divisionFactor)) / diffNanoTime;

		sprintf(state->currentStatisticsString, STAT_STRING, totalEventsPerTime, validEventsPerTime);

		// Reset for next update.
		state->totalEventsCounter = 0;
		state->validEventsCounter = 0;
		state->lastTime.tv_sec = currentTime.tv_sec;
		state->lastTime.tv_nsec = currentTime.tv_nsec;
	}
}
