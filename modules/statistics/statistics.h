#ifndef STATISTICS_H_
#define STATISTICS_H_

#include "main.h"

#include <libcaer/events/common.h>
#include <time.h>
#include <sys/time.h>

#define CAER_STATISTICS_STRING_TOTAL "Total events/second: %10" PRIu64
#define CAER_STATISTICS_STRING_VALID "Valid events/second: %10" PRIu64

#define MAX_BUFFER_SIZE  20 // max size for buffer of counts

struct caer_statistics_state {
	uint64_t divisionFactor;
	char *currentStatisticsStringTotal;
	char *currentStatisticsStringValid;
	// Internal book-keeping.
	struct timespec lastTime;
	uint64_t totalEventsCounter;
	uint64_t validEventsCounter;
	uint64_t bufferCounts[MAX_BUFFER_SIZE]; 
	size_t bufferIdx;
};

typedef struct caer_statistics_state *caerStatisticsState;

// For reuse inside other modules.
bool caerStatisticsStringInit(caerStatisticsState state);
void caerStatisticsStringUpdate(caerEventPacketHeader packetHeader, caerStatisticsState state);
void caerStatisticsStringExit(caerStatisticsState state);
void caerStatisticsStringReset(caerStatisticsState state);

void caerStatistics(uint16_t moduleID, caerEventPacketHeader packetHeader, size_t divisionFactor);

//private


#endif /* STATISTICS_H_ */
