#include "statistics.h"
#include "base/mainloop.h"
#include "base/module.h"
#include <time.h>

struct statistics_state {
	struct timespec lastTime;
	uint64_t totalEventsCounter;
	uint64_t validEventsCounter;
};

typedef struct statistics_state *statisticsState;

static void caerStatisticsRun(caerModuleData moduleData, size_t argsNumber,
		va_list args);

static struct caer_module_functions caerStatisticsFunctions =
		{ .moduleInit =
		NULL, .moduleRun = &caerStatisticsRun, .moduleConfig = NULL,
				.moduleExit = NULL };

void caerStatistics(uint16_t moduleID, caerEventPacketHeader packetHeader) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "Statistics");

	caerModuleSM(&caerStatisticsFunctions, moduleData,
			sizeof(struct statistics_state), 1, packetHeader);
}

static void caerStatisticsRun(caerModuleData moduleData, size_t argsNumber,
		va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerEventPacketHeader packetHeader = va_arg(args, caerEventPacketHeader);

	statisticsState state = moduleData->moduleState;

	// Only non-NULL packets (with content!) contribute to the event count.
	if (packetHeader != NULL) {
		state->totalEventsCounter += caerEventPacketHeaderGetEventNumber(
				packetHeader);
		state->validEventsCounter += caerEventPacketHeaderGetEventValid(
				packetHeader);
	}

	// Print up-to-date statistic roughly every second, taking into account possible deviations.
	struct timespec currentTime;
	clock_gettime(CLOCK_MONOTONIC, &currentTime);

	uint64_t diffNanoTime = (uint64_t) (((int64_t) (currentTime.tv_sec
			- state->lastTime.tv_sec) * 1000000000)
			+ (int64_t) (currentTime.tv_nsec - state->lastTime.tv_nsec));

	// DiffNanoTime is the difference in nanoseconds; we want to trigger roughly every second.
	if (diffNanoTime >= 1000000000) {
		// Print current values.
		uint64_t totalEventsPerTime = (state->totalEventsCounter * 1000000)
				/ diffNanoTime;
		uint64_t validEventsPerTime = (state->validEventsCounter * 1000000)
				/ diffNanoTime;

		fprintf(stdout,
				"\rTotal Kevents/second: %10" PRIu64 " - Valid Kevents/second: %10" PRIu64,
				totalEventsPerTime, validEventsPerTime);
		fflush(stdout);

		// Reset for next update.
		state->totalEventsCounter = 0;
		state->validEventsCounter = 0;
		state->lastTime.tv_sec = currentTime.tv_sec;
		state->lastTime.tv_nsec = currentTime.tv_nsec;
	}
}
