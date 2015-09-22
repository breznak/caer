#ifndef STATISTICS_H_
#define STATISTICS_H_

#include "main.h"
#include "events/common.h"

struct caer_statistics_state {
	uint64_t divisionFactor;
	char *currentStatisticsString;
	// Internal book-keeping.
	struct timespec lastTime;
	uint64_t totalEventsCounter;
	uint64_t validEventsCounter;
};

typedef struct caer_statistics_state *caerStatisticsState;

// For reuse inside other modules.
void caerUpdateStatisticsString(caerEventPacketHeader packetHeader, caerStatisticsState state);

void caerStatistics(uint16_t moduleID, caerEventPacketHeader packetHeader, size_t divisionFactor);

#endif /* STATISTICS_H_ */
