/*
 * mainloop.h
 *
 *  Created on: Dec 9, 2013
 *      Author: chtekk
 */

#ifndef MAINLOOP_H_
#define MAINLOOP_H_

#include "main.h"
#include "module.h"
#include "ext/uthash/utarray.h"

struct caer_mainloop_data {
	pthread_t mainloop;
	uint16_t mainloopID;
	bool (*mainloopFunction)(void);
	sshsNode mainloopNode;
	atomic_ops_uint running;
	atomic_ops_uint dataAvailable;
	caerModuleData modules;
	UT_array *memoryToFree;
};

typedef struct caer_mainloop_data *caerMainloopData;

struct caer_mainloop_definition {
	uint16_t mlID;
	bool (*mlFunction)(void);
};

caerMainloopData caerMainloopGetReference(void);
void caerMainloopRun(struct caer_mainloop_definition (*mainLoops)[], size_t numLoops);
caerModuleData caerMainloopFindModule(uint16_t moduleID, const char *moduleShortName);
void caerMainloopFreeAfterLoop(void *memPtr);
sshsNode caerMainloopGetSourceInfo(uint16_t source);

static inline void caerMainloopDataAvailableIncrease(caerMainloopData mainloopData) {
	atomic_ops_uint_inc(&mainloopData->dataAvailable, ATOMIC_OPS_FENCE_RELEASE);
}

static inline void caerMainloopDataAvailableDecrease(caerMainloopData mainloopData) {
	atomic_ops_uint_dec(&mainloopData->dataAvailable, ATOMIC_OPS_FENCE_NONE);
}

#endif /* MAINLOOP_H_ */
