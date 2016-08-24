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

#ifdef HAVE_PTHREADS
	#include "ext/c11threads_posix.h"
#endif

struct caer_mainloop_data {
	thrd_t mainloop;
	uint16_t mainloopID;
	bool (*mainloopFunction)(void);
	sshsNode mainloopNode;
	atomic_bool running;
	atomic_uint_fast32_t dataAvailable;
	caerModuleData modules;
	UT_array *memoryToFree;
	UT_array *inputModules;
	UT_array *outputModules;
	UT_array *processorModules;
};

typedef struct caer_mainloop_data *caerMainloopData;

struct caer_mainloop_definition {
	uint16_t mlID;
	bool (*mlFunction)(void);
};

void caerMainloopRun(struct caer_mainloop_definition (*mainLoops)[], size_t numLoops);
caerModuleData caerMainloopFindModule(uint16_t moduleID, const char *moduleShortName, enum caer_module_type type);
void caerMainloopFreeAfterLoop(void (*func)(void *mem), void *memPtr);
caerMainloopData caerMainloopGetReference(void);
sshsNode caerMainloopGetSourceNode(uint16_t sourceID);
sshsNode caerMainloopGetSourceInfo(uint16_t sourceID);
void *caerMainloopGetSourceState(uint16_t sourceID);
void caerMainloopResetInputs(uint16_t sourceID);
void caerMainloopResetOutputs(uint16_t sourceID);
void caerMainloopResetProcessors(uint16_t sourceID);

#endif /* MAINLOOP_H_ */
