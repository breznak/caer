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
#define STIM_PATTERNA   4
#define STIM_PATTERNB   5
#define STIM_PATTERNC   6

bool caerGenSpikeInit(caerModuleData moduleData);
void caerGenSpikeExit(caerModuleData moduleData);
int spikeGenThread(void *spikeGenState);
void spiketrainReg(void *spikeGenState);
void spiketrainPat(void *spikeGenState, uint32_t spikePattern[32][32]);
void SetCam(void *spikeGenState);
void ClearCam(void *spikeGenState);
void ClearAllCam(void *spikeGenState);
void WriteCam(void *spikeGenState, uint32_t preNeuronAddr, uint32_t postNeuronAddr, uint32_t camId, int16_t synapseType);

struct timespec tstart = { 0, 0 }, tend = { 0, 0 };
static int CamSeted = 0; //static bool CamSeted = false;
static int CamCleared = 0; //static bool CamCleared = false;
static int CamAllCleared = 0;

//caerDeviceHandle usb_handle;

bool caerGenSpikeInit(caerModuleData moduleData) {

	caerInputDynapseState state = moduleData->moduleState;
//	usb_handle = (caerDeviceHandle) state->deviceState;

	sshsNode deviceConfigNodeMain = sshsGetRelativeNode(moduleData->moduleNode, chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));

	sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");

	sshsNodePutBoolIfAbsent(spikeNode, "doStim", false); //false

	sshsNodePutIntIfAbsent(spikeNode, "stim_type", U8T(STIM_REGULAR)); //STIM_REGULAR
	atomic_store(&state->genSpikeState.stim_type, sshsNodeGetInt(spikeNode, "stim_type"));

	sshsNodePutIntIfAbsent(spikeNode, "stim_avr", 3);
	atomic_store(&state->genSpikeState.stim_avr,
			sshsNodeGetInt(spikeNode, "stim_avr"));

	sshsNodePutIntIfAbsent(spikeNode, "stim_std", 1);
	atomic_store(&state->genSpikeState.stim_std,
			sshsNodeGetInt(spikeNode, "stim_std"));

	sshsNodePutIntIfAbsent(spikeNode, "stim_duration", 10); //10
	atomic_store(&state->genSpikeState.stim_duration,
			sshsNodeGetInt(spikeNode, "stim_duration"));

	sshsNodePutBoolIfAbsent(spikeNode, "repeat", false); //false
	atomic_store(&state->genSpikeState.repeat, sshsNodeGetBool(spikeNode, "repeat"));

	sshsNodePutBoolIfAbsent(spikeNode, "setCam", false); //1 //false
	atomic_store(&state->genSpikeState.setCam, sshsNodeGetBool(spikeNode, "setCam"));

	sshsNodePutBoolIfAbsent(spikeNode, "clearCam", false); //1 //false
	atomic_store(&state->genSpikeState.setCam, sshsNodeGetBool(spikeNode, "clearCam"));

	sshsNodePutBoolIfAbsent(spikeNode, "clearAllCam", false); //1 //false
	atomic_store(&state->genSpikeState.setCam, sshsNodeGetBool(spikeNode, "clearAllCam"));

	atomic_store(&state->genSpikeState.started, false); //false
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

	sshsNodePutIntIfAbsent(spikeNode, "chip_id", DYNAPSE_CONFIG_DYNAPSE_U0); //4
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

	while (atomic_load_explicit(&state->genSpikeState.running, // the loop
			memory_order_relaxed)) {

		if(state->genSpikeState.setCam == true && CamSeted == 0) {
			SetCam(state);
			CamSeted = 1;
		} else if(state->genSpikeState.setCam == false && CamSeted == 1) {
			CamSeted = 0;
		}
		if (state->genSpikeState.clearCam == true && CamCleared == 0) {
			ClearCam(state);
			CamCleared = 1;
		} else if(state->genSpikeState.clearCam == false && CamCleared == 1) {
			CamCleared = 0;
		}
		if (state->genSpikeState.clearAllCam == true && CamAllCleared == 0) {
			ClearAllCam(state);
			CamAllCleared = 1;
		} else if(state->genSpikeState.clearAllCam == false && CamAllCleared == 1) {
			CamAllCleared = 0;
		}

		/* generate spikes*/

		if (state->genSpikeState.stim_type == STIM_REGULAR) {
			spiketrainReg(state);
		} else if (state->genSpikeState.stim_type == STIM_POISSON) {

		} else if (state->genSpikeState.stim_type == STIM_GAUSSIAN) {

		} else if (state->genSpikeState.stim_type == STIM_PATTERNA) {
			// generate pattern A
			uint32_t spikePatternA[32][32];
			int64_t rowId, colId;
			int cx, cy, r;
		    cx = 16;
		    cy = 16;
		    r  = 14;
		    for (rowId = 0; rowId < 32; rowId++)
			    for (colId = 0; colId < 32; colId++)
			    	spikePatternA[rowId][colId] = 0;
		    for (rowId = cx - r; rowId <= cx + r; rowId++)
		    	for (colId = cy - r; colId <= cy + r; colId++)
		    		if (((cx-rowId)*(cx-rowId)+(cy-colId)*(cy-colId)<=r*r+sqrt(r)) && ((cx-rowId)*(cx-rowId)+(cy-colId)*(cy-colId)>=r*r-r))
		    			spikePatternA[rowId][colId] = 1;
			spiketrainPat(state, spikePatternA);
		} else if (state->genSpikeState.stim_type == STIM_PATTERNB) {
		    //generate pattern B
			uint32_t spikePatternB[32][32];
			int64_t rowId, colId;
		    int64_t num = 16;
		    for(rowId = -num; rowId < num; rowId++) {
		    	for(colId = -num; colId < num; colId++) {
		    		if (abs((int)rowId) + abs((int)colId) == num) // Change this condition >= <=
		    			spikePatternB[rowId+16][colId+16] = 1;
		    		else
		    			spikePatternB[rowId+16][colId+16] = 0;
		    	}
		    }
			spiketrainPat(state, spikePatternB);
		} else if (state->genSpikeState.stim_type == STIM_PATTERNC) {
		    //generate pattern C
			uint32_t spikePatternC[32][32];
			int64_t rowId, colId;
			int64_t num = 16;
		    for(rowId = -num; rowId < num; rowId++) {
		    	for(colId = -num; colId < num; colId++) {
		    		if (abs((int)rowId) == abs((int)colId)) // Change this condition
		    			spikePatternC[rowId+16][colId+16] = 1;
		    		else
		    			spikePatternC[rowId+16][colId+16] = 0;
		    	}
		    }
			spiketrainPat(state, spikePatternC);
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


	if (atomic_load(&state->genSpikeState.stim_avr) > 0) {
		tim.tv_nsec = 1000000000L / atomic_load(&state->genSpikeState.stim_avr);
	} else {
		tim.tv_nsec = 1000000000L;
	}

	uint32_t value = atomic_load(&state->genSpikeState.core_d) |
			0 << 16 | 0 << 17 | 1 << 13 |
			atomic_load(&state->genSpikeState.core_s) << 18 |
			atomic_load(&state->genSpikeState.address) << 20 |
			atomic_load(&state->genSpikeState.dx) << 4 |
			atomic_load(&state->genSpikeState.sx) << 6 |
			atomic_load(&state->genSpikeState.dy) << 7 |
			atomic_load(&state->genSpikeState.sy) << 9;

	if (!atomic_load(&state->genSpikeState.started)) {
		LABELSTART:
			clock_gettime(CLOCK_MONOTONIC, &tstart);
	}

	clock_gettime(CLOCK_MONOTONIC, &tend);


	if (atomic_load(&state->genSpikeState.stim_duration)
			<= ((double) tend.tv_sec + 1.0e-9 * tend.tv_nsec)
					- ((double) tstart.tv_sec + 1.0e-9 * tstart.tv_nsec)) {
		if (atomic_load(&state->genSpikeState.started)) {
			caerLog(CAER_LOG_NOTICE, "spikeGen", "stimulation finished.\n");
		}
		atomic_store(&state->genSpikeState.done, true);
		atomic_store(&state->genSpikeState.started, false);
		if(atomic_load(&state->genSpikeState.repeat)){
			caerLog(CAER_LOG_NOTICE, "spikeGen", "stimulation re-started.\n");
			atomic_store(&state->genSpikeState.started, true);
			atomic_store(&state->genSpikeState.done, false);
			goto LABELSTART;
		}
	}

	if (!atomic_load(&state->genSpikeState.done)) {
		nanosleep(&tim, NULL);
		// send spikes
		caerDeviceConfigSet((caerDeviceHandle) state->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, atomic_load(&state->genSpikeState.chip_id));  //usb_handle
		//(TZ+0100): CRITICAL: Dynap-se ID-1 SN-00000003 [1:13]: Failed to send chip config, USB transfer failed on verification.
		/*send the spike*/
		caerDeviceConfigSet((caerDeviceHandle) state->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, value); //usb_handle
		caerLog(CAER_LOG_NOTICE, "spikeGen", "sending spikes %d \n", value);

	}

}

void spiketrainPat(void *spikeGenState, uint32_t spikePattern[32][32]) { //generate and send 32*32 input stimuli

	if (spikeGenState == NULL) {
		return;
	}

	caerInputDynapseState state = spikeGenState;
	caerDeviceHandle usb_handle = (caerDeviceHandle) state->deviceState;

	struct timespec tim;
	tim.tv_sec = 0;


	if (atomic_load(&state->genSpikeState.stim_avr) > 0) {
		tim.tv_nsec = 1000000000L / atomic_load(&state->genSpikeState.stim_avr);
	} else {
		tim.tv_nsec = 1000000000L;
	}

	//generate chip command for stimulating
	uint32_t value, valueSent;
	uint32_t value2DArray[32][32];
	int64_t rowId, colId;
	for (rowId = 0; rowId < 32; rowId++)
		for (colId = 0; colId < 32; colId++) {
			if (spikePattern[rowId][colId] == 1)
				value = 0xf | //atomic_load(&state->genSpikeState.core_d)
						0 << 16 | 0 << 17 | 1 << 13 |
//						atomic_load(&state->genSpikeState.core_s) << 18 |
//						atomic_load(&state->genSpikeState.address) << 20 |
						(((rowId / 16) << 1) | (colId / 16)) << 18 |
						(((rowId % 16) << 4) | (colId % 16)) << 20 |
						atomic_load(&state->genSpikeState.dx) << 4 |
						atomic_load(&state->genSpikeState.sx) << 6 |
						atomic_load(&state->genSpikeState.dy) << 7 |
						atomic_load(&state->genSpikeState.sy) << 9;
			else
				value = 0;
			value2DArray[rowId][colId] = value;
		}

	if (!atomic_load(&state->genSpikeState.started)) {
		LABELSTART:
			clock_gettime(CLOCK_MONOTONIC, &tstart);
	}

	clock_gettime(CLOCK_MONOTONIC, &tend);


	if (atomic_load(&state->genSpikeState.stim_duration)
			<= ((double) tend.tv_sec + 1.0e-9 * tend.tv_nsec)
					- ((double) tstart.tv_sec + 1.0e-9 * tstart.tv_nsec)) {
		if (atomic_load(&state->genSpikeState.started)) {
			caerLog(CAER_LOG_NOTICE, "spikeGen", "stimulation finished.\n");
		}
		atomic_store(&state->genSpikeState.done, true);
		atomic_store(&state->genSpikeState.started, false);
		if(atomic_load(&state->genSpikeState.repeat)){
			caerLog(CAER_LOG_NOTICE, "spikeGen", "stimulation re-started.\n");
			atomic_store(&state->genSpikeState.started, true);
			atomic_store(&state->genSpikeState.done, false);
			goto LABELSTART;
		}
	}

	if (!atomic_load(&state->genSpikeState.done)) {
		nanosleep(&tim, NULL);
		// send spikes
		caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, atomic_load(&state->genSpikeState.chip_id));
		//send the spike
		for (rowId = 0; rowId < 32; rowId++)
			for (colId = 0; colId < 32; colId++) {
				valueSent = value2DArray[rowId][colId];
				if (valueSent != 0) {
//					caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, atomic_load(&state->genSpikeState.chip_id));
					caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, valueSent);
				}
			}
		caerLog(CAER_LOG_NOTICE, "spikeGen", "sending spikes %d \n", value);
	}

}

void SetCam(void *spikeGenState) {
	if (spikeGenState == NULL) {
		return;
	}
	caerInputDynapseState state = spikeGenState;
	caerDeviceHandle usb_handle = (caerDeviceHandle) state->deviceState;
	caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, atomic_load(&state->genSpikeState.chip_id)); //0
	uint32_t neuronId;
	for (neuronId = 0; neuronId < 32 * 32; neuronId++) {
		WriteCam(state, neuronId, neuronId, 0, 3);
	}
}

void ClearCam(void *spikeGenState) {
	if (spikeGenState == NULL) {
		return;
	}
	caerInputDynapseState state = spikeGenState;
	caerDeviceHandle usb_handle = (caerDeviceHandle) state->deviceState;
	caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, atomic_load(&state->genSpikeState.chip_id)); //0
	uint32_t neuronId;
	for (neuronId = 0; neuronId < DYNAPSE_CONFIG_NUMNEURONS; neuronId++) {
		WriteCam(state, 0, neuronId, 0, 0);
	}
}

void ClearAllCam(void *spikeGenState) {
	if (spikeGenState == NULL) {
		return;
	}
	caerInputDynapseState state = spikeGenState;
	caerDeviceHandle usb_handle = (caerDeviceHandle) state->deviceState;
	caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, atomic_load(&state->genSpikeState.chip_id)); //0
	uint32_t neuronId, camId;
	for (neuronId = 0; neuronId < DYNAPSE_CONFIG_NUMNEURONS; neuronId++) {
		for (camId = 0; camId < 64; camId++) {
			WriteCam(state, 0, neuronId, camId, 0);
		}
	}
}

void WriteCam(void *spikeGenState, uint32_t preNeuronAddr, uint32_t postNeuronAddr, uint32_t camId, int16_t synapseType) {
	if (spikeGenState == NULL) {
		return;
	}
	caerInputDynapseState state = spikeGenState;
	caerDeviceHandle usb_handle = (caerDeviceHandle) state->deviceState;

	uint32_t bits;
	uint32_t ei = (synapseType & 0x2) >> 1;
	uint32_t fs = synapseType & 0x1;
    uint32_t address = preNeuronAddr & 0xff;
    uint32_t source_core = (preNeuronAddr & 0x300) >> 8;
    uint32_t coreId = (postNeuronAddr & 0x300) >> 8;
    uint32_t neuron_row = (postNeuronAddr & 0xf0) >> 4;
    uint32_t synapse_row = camId;
    uint32_t row = neuron_row << 6 | synapse_row;
	uint32_t column = postNeuronAddr & 0xf;
    bits = ei << 29 | fs << 28 | address << 20 | source_core << 18 | 1 << 17 | coreId << 15 | row << 5 | column;
    printf("Write CAM: \n");
    printf("Chip ID: %d\n", atomic_load(&state->genSpikeState.chip_id)); //0
    printf("Bits: %d\n", bits);
    caerDeviceConfigSet(usb_handle, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, bits);
}
