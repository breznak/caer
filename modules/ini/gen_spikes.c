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

struct timespec tstart = { 0, 0 }, tend = { 0, 0 };

bool caerGenSpikeInit(caerModuleData moduleData) {

	caerInputDynapseState state = moduleData->moduleState;

	sshsNode deviceConfigNodeMain = sshsGetRelativeNode(moduleData->moduleNode, chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));

	sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");

	sshsNodePutBoolIfAbsent(spikeNode, "doStim", false);

	sshsNodePutIntIfAbsent(spikeNode, "stim_type", U8T(STIM_REGULAR));
	atomic_store(&state->genSpikeState.stim_type,
			sshsNodeGetInt(spikeNode, "stim_type"));

	sshsNodePutIntIfAbsent(spikeNode, "stim_avr", 3);
	atomic_store(&state->genSpikeState.stim_avr,
			sshsNodeGetInt(spikeNode, "stim_avr"));

	sshsNodePutIntIfAbsent(spikeNode, "stim_std", 1);
	atomic_store(&state->genSpikeState.stim_std,
			sshsNodeGetInt(spikeNode, "stim_std"));

	sshsNodePutIntIfAbsent(spikeNode, "stim_duration", 10);
	atomic_store(&state->genSpikeState.stim_duration,
			sshsNodeGetInt(spikeNode, "stim_duration"));

	sshsNodePutBoolIfAbsent(spikeNode, "repeat", false);
	atomic_store(&state->genSpikeState.repeat,
			sshsNodeGetBool(spikeNode, "repeat"));

	atomic_store(&state->genSpikeState.started, false);
	atomic_store(&state->genSpikeState.done, true);

	// Start separate stimulation thread.
	atomic_store(&state->genSpikeState.running, true);

	if (thrd_create(&state->genSpikeState.spikeGenThread, &spikeGenThread,
			state) != thrd_success) {
		caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString,
				"SpikeGen: Failed to start thread.");
		return (NULL);
	}

	/*address*/
	sshsNodePutBoolIfAbsent(spikeNode, "sx", false);
	atomic_store(&state->genSpikeState.sx, sshsNodeGetBool(spikeNode, "sx"));

	sshsNodePutBoolIfAbsent(spikeNode, "sy", false);
	atomic_store(&state->genSpikeState.sy, sshsNodeGetBool(spikeNode, "sy"));

	sshsNodePutIntIfAbsent(spikeNode, "core_d", 0);
	atomic_store(&state->genSpikeState.core_d, sshsNodeGetInt(spikeNode, "core_d"));

	sshsNodePutIntIfAbsent(spikeNode, "core_s", 0);
	atomic_store(&state->genSpikeState.core_s, sshsNodeGetInt(spikeNode, "core_s"));

	sshsNodePutIntIfAbsent(spikeNode, "address", 1);
	atomic_store(&state->genSpikeState.address, sshsNodeGetInt(spikeNode, "address"));

	sshsNodePutIntIfAbsent(spikeNode, "dx", 0);
	atomic_store(&state->genSpikeState.dx, sshsNodeGetInt(spikeNode, "dx"));

	sshsNodePutIntIfAbsent(spikeNode, "dy", 0);
	atomic_store(&state->genSpikeState.dy, sshsNodeGetInt(spikeNode, "dy"));

	sshsNodePutIntIfAbsent(spikeNode, "chip_id", 4);
	atomic_store(&state->genSpikeState.chip_id, sshsNodeGetInt(spikeNode, "chip_id"));

	return (true);
}

void caerGenSpikeExit(caerModuleData moduleData) {
	caerInputDynapseState state = moduleData->moduleState;

	// Shut down stimulation thread and wait on it to finish.
	atomic_store(&state->genSpikeState.running, false);

	if ((errno = thrd_join(state->genSpikeState.spikeGenThread, NULL))
			!= thrd_success) {
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

	while (atomic_load_explicit(&state->genSpikeState.running,
			memory_order_relaxed)) {

		/* generate spikes*/
		if (state->genSpikeState.stim_type == STIM_REGULAR) {
			spiketrainReg(state);
		} else if (state->genSpikeState.stim_type == STIM_POISSON) {

		} else if (state->genSpikeState.stim_type == STIM_GAUSSIAN) {

		}

	}

	return (thrd_success);
}

void spiketrainReg(void *spikeGenState) {

	if (spikeGenState == NULL) {
		return;
	}

	caerInputDynapseState state = spikeGenState;

	struct timespec tim;
	tim.tv_sec = 0;

	/* load current values into thread stimulation*/
	atomic_load(&state->genSpikeState.repeat);
	atomic_load(&state->genSpikeState.done);
	atomic_load(&state->genSpikeState.started);
	atomic_load(&state->genSpikeState.stim_avr);
	atomic_load(&state->genSpikeState.stim_duration);

	atomic_load(&state->genSpikeState.dx);			// 1
	atomic_load(&state->genSpikeState.dy);			// 1
	atomic_load(&state->genSpikeState.sx);			// 1
	atomic_load(&state->genSpikeState.sy);			// 0
	atomic_load(&state->genSpikeState.address);		// neuron id address
	atomic_load(&state->genSpikeState.chip_id);		// which chip has to ack
	atomic_load(&state->genSpikeState.core_s);		// from which core the event is from
	atomic_load(&state->genSpikeState.core_d);		// each bit is for one core 1111 goes to all cores

	if (state->genSpikeState.stim_avr > 0) {
		tim.tv_nsec = 1000000000L / atomic_load(&state->genSpikeState.stim_avr);
	} else {
		tim.tv_nsec = 1000000000L;
	}

	uint32_t value = state->genSpikeState.core_d |
			0 << 16 | 0 << 17 | 1 << 13 |
			state->genSpikeState.core_s << 18 |
			state->genSpikeState.address << 20 |
			state->genSpikeState.dx << 4 |
			state->genSpikeState.sx << 6 |
			state->genSpikeState.dy << 7 |
			state->genSpikeState.sy << 9;

	if (!state->genSpikeState.started) {
		LABELSTART:
			clock_gettime(CLOCK_MONOTONIC, &tstart);
	}

	clock_gettime(CLOCK_MONOTONIC, &tend);


	if (state->genSpikeState.stim_duration
			<= ((double) tend.tv_sec + 1.0e-9 * tend.tv_nsec)
					- ((double) tstart.tv_sec + 1.0e-9 * tstart.tv_nsec)) {
		if (state->genSpikeState.started) {
			caerLog(CAER_LOG_NOTICE, "spikeGen", "stimulation finished.\n");
		}
		atomic_store(&state->genSpikeState.done, true);
		atomic_store(&state->genSpikeState.started, false);
		if(state->genSpikeState.repeat){
			caerLog(CAER_LOG_NOTICE, "spikeGen", "stimulation re-started.\n");
			atomic_store(&state->genSpikeState.started, true);
			atomic_store(&state->genSpikeState.done, false);
			goto LABELSTART;
		}
	}

	if (!state->genSpikeState.done) {
		nanosleep(&tim, NULL);
		// send spikes
		caerDeviceConfigSet((caerDeviceHandle) state->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, state->genSpikeState.chip_id);

		/*send the spike*/
		caerDeviceConfigSet((caerDeviceHandle) state->deviceState,DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_CONTENT, value);
		caerLog(CAER_LOG_NOTICE, "spikeGen", "sending spikes %d \n", value);

	}

}

