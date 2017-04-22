#ifndef STATISTICS_H_
#define STATISTICS_H_

#include "main.h"

#include <libcaer/events/common.h>
#include <time.h>
#include <sys/time.h>

#define CAER_STATISTICS_STRING_TOTAL "Total events/second: %10" PRIu64
#define CAER_STATISTICS_STRING_VALID "Valid events/second: %10" PRIu64
#define CAER_STATISTICS_STRING_STATS "Event counts: Min=  %10" PRIu64 "  Max=  %10" PRIu64 "  Avg=  %.2f" 


#define MAX_BUFFER_SIZE  20 // max size for buffer of counts //TODO make parameter of the run method 

struct caer_statistics_state {
	uint64_t divisionFactor;
	char *currentStatisticsStringTotal;
	char *currentStatisticsStringValid;
	char *currentStatisticsStringStats;
	// Internal book-keeping.
	struct timespec lastTime;
	uint64_t totalEventsCounter;
	uint64_t validEventsCounter;
	uint64_t currMin;
	uint64_t currMax;
	double currAvg;
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
uint64_t maxArr(uint64_t arr[]);
uint64_t minArr(uint64_t arr[]);
double avgArr(uint64_t arr[]);

#endif /* STATISTICS_H_ */
