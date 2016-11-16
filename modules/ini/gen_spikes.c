#include "base/module.h"
#ifdef HAVE_PTHREADS
#include "ext/c11threads_posix.h"
#endif

#include <math.h>
#include <stdatomic.h>
#include <sys/types.h>
#include <fcntl.h>
#include <time.h>

#include "main.h"
#include "dynapse_common.h"
#include <libcaer/events/packetContainer.h>
#include <libcaer/events/spike.h>


#define STIM_POISSON 	1
#define STIM_REGULAR 	2
#define STIM_GAUSSIAN 	3

bool caerGenSpikeInit(caerModuleData moduleData);
void caerGenSpikeExit(caerModuleData moduleData);
int spikeGenThread(void *spikeGenState);

bool caerGenSpikeInit(caerModuleData moduleData) {

	caerInputDynapseState state = moduleData->moduleState;

	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "doStim", true);
	atomic_store(&state->genSpikeState.doStim, sshsNodeGetBool(moduleData->moduleNode, "doStim"));

	sshsNodePutByteIfAbsent(moduleData->moduleNode, "stimType", U8T(STIM_REGULAR));
	state->genSpikeState.stimType = sshsNodeGetByte(moduleData->moduleNode, "stimType");

	sshsNodePutLongIfAbsent(moduleData->moduleNode, "avr", 10);
	atomic_store(&state->genSpikeState.avr, sshsNodeGetLong(moduleData->moduleNode, "avr"));

	sshsNodePutLongIfAbsent(moduleData->moduleNode, "std", 1);
	atomic_store(&state->genSpikeState.std, sshsNodeGetLong(moduleData->moduleNode, "std"));

	sshsNodePutFloatIfAbsent(moduleData->moduleNode, "duration", 10);
	state->genSpikeState.duration = sshsNodeGetFloat(moduleData->moduleNode, "duration");

	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "repeat", true);
	state->genSpikeState.repeat = sshsNodeGetBool(moduleData->moduleNode, "repeat");


	// Start separate rendering thread. Decouples presentation from
	// data processing and preparation. Communication over ring-buffer.
	atomic_store(&state->genSpikeState.running, true);

	if (thrd_create(&state->genSpikeState.spikeGenThread, &spikeGenThread, state) != thrd_success) {
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "SpikeGen: Failed to start rendering thread.");
		return (NULL);
	}


	return (true);
}

void caerGenSpikeExit(caerModuleData moduleData) {
	caerInputDynapseState state = moduleData->moduleState;

	// Shut down rendering thread and wait on it to finish.
	atomic_store(&state->genSpikeState.running, false);

	if ((errno = thrd_join(state->genSpikeState.spikeGenThread, NULL)) != thrd_success) {
		// This should never happen!
		caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString,
			"SpikeGen: Failed to join rendering thread. Error: %d.", errno);
	}


}

int spikeGenThread(void *spikeGenState) {
	if (spikeGenState == NULL) {
		return (thrd_error);
	}

	caerInputDynapseState state = spikeGenState;

	thrd_set_name("SpikeGenThread");

	while (atomic_load_explicit(&state->genSpikeState.running, memory_order_relaxed)) {

		/* generate spikes*/
		if(state->genSpikeState.stimType == STIM_REGULAR){
			spiketrainReg(state);
		}else if(state->genSpikeState.stimType == STIM_POISSON){

		}else if(state->genSpikeState.stimType == STIM_GAUSSIAN){

		}


	}

	return (thrd_success);
}

void spiketrainReg(void *spikeGenState){

	if (spikeGenState == NULL) {
		return;
	}

	caerInputDynapseState state = spikeGenState;

	struct timespec tim;
	tim.tv_sec = 0;
	tim.tv_nsec = 1000000000L/atomic_load(&state->genSpikeState.avr);

	uint32_t value = 0 | 1 << 13 | 0 << 16 | 0 << 17 | 0 << 18 | 1 << 20; //address

	atomic_load(&state->genSpikeState.doStim);
	if(state->genSpikeState.doStim){
		nanosleep(&tim, NULL);
		// send spike stimulus
		// for now work on core id DYNAPSE_CONFIG_DYNAPSE_U2
		//caerDeviceConfigSet(state->moduleState, DYNAPSE_CONFIG_CHIP,
		//	DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U2);
		//caerDeviceConfigSet(state->moduleState,
		//				DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, value);
		caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,
				 DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U2);

		printf("sending spikes %d\n", state->genSpikeState.doStim);
	}

}


