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
void spiketrainReg(void *spikeGenState);

struct timespec tstart={0,0}, tend={0,0};

bool caerGenSpikeInit(caerModuleData moduleData) {

	caerInputDynapseState state = moduleData->moduleState;

	sshsNode spikeNode = sshsGetRelativeNode(moduleData->moduleNode, "spikeGen/");

	sshsNodePutBoolIfAbsent(spikeNode, "doStim", false);
	atomic_store(&state->genSpikeState.doStim, sshsNodeGetBool(spikeNode, "doStim"));
	atomic_store(&state->genSpikeState.started, false); // at start we do not stimulate

	sshsNodePutLongIfAbsent(spikeNode, "stim_type", U8T(STIM_REGULAR));
	atomic_store(&state->genSpikeState.stim_type, sshsNodeGetLong(spikeNode, "stim_type"));

	sshsNodePutLongIfAbsent(spikeNode, "stim_avr", 3);
	atomic_store(&state->genSpikeState.stim_avr, sshsNodeGetLong(spikeNode, "stim_avr"));

	sshsNodePutLongIfAbsent(spikeNode, "stim_std", 1);
	atomic_store(&state->genSpikeState.stim_std, sshsNodeGetLong(spikeNode, "stim_std"));

	sshsNodePutLongIfAbsent(spikeNode, "stim_duration", 10);
	atomic_store(&state->genSpikeState.stim_duration, sshsNodeGetLong(spikeNode, "stim_duration"));

	sshsNodePutBoolIfAbsent(spikeNode, "repeat", true);
	atomic_store(&state->genSpikeState.repeat, sshsNodeGetBool(spikeNode, "repeat"));

	//sshsNodePutBoolIfAbsent(spikeNode, "done", false);
	//state->genSpikeState.done = sshsNodeGetBool(spikeNode, "done");

	// Start separate rendering thread. Decouples presentation from
	// data processing and preparation. Communication over ring-buffer.
	atomic_store(&state->genSpikeState.running, true);

	if (thrd_create(&state->genSpikeState.spikeGenThread, &spikeGenThread, state) != thrd_success) {
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "SpikeGen: Failed to start thread.");
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
		if(state->genSpikeState.stim_type == STIM_REGULAR){
			spiketrainReg(state);
		}else if(state->genSpikeState.stim_type == STIM_POISSON){

		}else if(state->genSpikeState.stim_type == STIM_GAUSSIAN){

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
	atomic_load(&state->genSpikeState.stim_avr);
	if(state->genSpikeState.stim_avr > 0){
		tim.tv_nsec = 1000000000L/atomic_load(&state->genSpikeState.stim_avr);
	}else{
		tim.tv_nsec = 1000000000L;
	}

	uint32_t value = 0 | 1 << 13 | 0 << 16 | 0 << 17 | 0 << 18 | 1 << 20; //address

	/* load current values into thread stimulation*/
	atomic_load(&state->genSpikeState.repeat);
	atomic_load(&state->genSpikeState.done);
	atomic_load(&state->genSpikeState.doStim);
	atomic_load(&state->genSpikeState.started);

	if(!state->genSpikeState.started){
		clock_gettime(CLOCK_MONOTONIC, &tstart); /*start counting*/
		atomic_store(&state->genSpikeState.started, true);
	}

	clock_gettime(CLOCK_MONOTONIC, &tend);

	/*in case we finished one repetition*/
	atomic_load(&state->genSpikeState.stim_duration);
	if(  state->genSpikeState.stim_duration <= ((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) -
	           ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec)  &&
			   !state->genSpikeState.repeat){
				if(state->genSpikeState.done == false){
					caerLog(CAER_LOG_NOTICE, "spikeGen", "stimulation finished.\n");
				}
				atomic_store(&state->genSpikeState.done, true);/*end stimulation*/
	}

	if(state->genSpikeState.doStim && !state->genSpikeState.done){
		nanosleep(&tim, NULL);
		// send spikes
		caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,
				 DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U2);

	}

}


