/*
 *
 *  Created on: Dec, 2016
 *      Author: federico.corradi@inilabs.com
 */

#include <time.h>
#include "monitorneufilter.h"
#include "libcaer/devices/dynapse.h"

struct MNFilter_state {
	caerInputDynapseState eventSourceModuleState;
	sshsNode eventSourceConfigNode;
	int dynapse_u0_c0;
	int dynapse_u0_c1;
	int dynapse_u0_c2;
	int dynapse_u0_c3;	//chip id core id
	int dynapse_u1_c0;
	int dynapse_u1_c1;
	int dynapse_u1_c2;
	int dynapse_u1_c3;
	int dynapse_u2_c0;
	int dynapse_u2_c1;
	int dynapse_u2_c2;
	int dynapse_u2_c3;
	int dynapse_u3_c0;
	int dynapse_u3_c1;
	int dynapse_u3_c2;
	int dynapse_u3_c3;
};

typedef struct MNFilter_state *MNFilterState;

static bool caerMonitorNeuFilterInit(caerModuleData moduleData);
static void caerMonitorNeuFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerMonitorNeuFilterExit(caerModuleData moduleData);
static void caerMonitorNeuFilterReset(caerModuleData moduleData, uint16_t resetCallSourceID);

static struct caer_module_functions caerMonitorNeuFilterFunctions = { .moduleInit =
	&caerMonitorNeuFilterInit, .moduleRun = &caerMonitorNeuFilterRun, .moduleExit = &caerMonitorNeuFilterExit, .moduleReset =
	&caerMonitorNeuFilterReset };

void caerMonitorNeuFilter(uint16_t moduleID,  int16_t eventSourceID) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "MonitorNeu", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerMonitorNeuFilterFunctions, moduleData, sizeof(struct MNFilter_state), 1, eventSourceID);
}

static bool caerMonitorNeuFilterInit(caerModuleData moduleData) {
	MNFilterState state = moduleData->moduleState;


	// defaults is first neurons of all cores
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "dynapse_u0_c0", 0);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "dynapse_u0_c1", 0);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "dynapse_u0_c2", 0);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "dynapse_u0_c3", 0);

	sshsNodePutIntIfAbsent(moduleData->moduleNode, "dynapse_u1_c0", 0);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "dynapse_u1_c1", 0);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "dynapse_u1_c2", 0);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "dynapse_u1_c3", 0);

	sshsNodePutIntIfAbsent(moduleData->moduleNode, "dynapse_u2_c0", 0);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "dynapse_u2_c1", 0);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "dynapse_u2_c2", 0);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "dynapse_u2_c3", 0);

	sshsNodePutIntIfAbsent(moduleData->moduleNode, "dynapse_u3_c0", 0);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "dynapse_u3_c1", 0);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "dynapse_u3_c2", 0);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "dynapse_u3_c3", 0);

	// variables
	state->dynapse_u0_c0 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c0");
	state->dynapse_u0_c1 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c1");
	state->dynapse_u0_c2 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c2");
	state->dynapse_u0_c3 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c3");

	state->dynapse_u1_c0 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c0");
	state->dynapse_u1_c1 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c1");
	state->dynapse_u1_c2 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c2");
	state->dynapse_u1_c3 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c3");

	state->dynapse_u2_c0 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c0");
	state->dynapse_u2_c1 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c1");
	state->dynapse_u2_c2 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c2");
	state->dynapse_u2_c3 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c3");

	state->dynapse_u3_c0 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c0");
	state->dynapse_u3_c1 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c1");
	state->dynapse_u3_c2 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c2");
	state->dynapse_u3_c3 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c3");

	// Nothing that can fail here.
	return (true);
}

static void caerMonitorNeuFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	int eventSourceID = va_arg(args, int);

	MNFilterState state = moduleData->moduleState;

	// --- start  usb handle / from spike event source id
	state->eventSourceModuleState = caerMainloopGetSourceState(U16T(eventSourceID));
	state->eventSourceConfigNode = caerMainloopGetSourceNode(U16T(eventSourceID));
	if(state->eventSourceModuleState == NULL || state->eventSourceConfigNode == NULL){
		return;
	}
	caerInputDynapseState stateSource = state->eventSourceModuleState;
	if(stateSource->deviceState == NULL){
		return;
	}
	// --- end usb handle

	// if changed we set it
	if(state->dynapse_u0_c0 != sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c0")){
		if(sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c0") < 0 || sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c0") > 255){
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Wrong neuron ID %d, please choose a value from [0,255]", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c0"));
		}else{
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U0);
			caerDeviceConfigSet(stateSource->deviceState,
					DYNAPSE_CONFIG_MONITOR_NEU, 0, (uint32_t) sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c0"));
			caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Monitoring neuron dynapse_u0_c0 num: %d", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c0"));
			state->dynapse_u0_c0 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c0");
		}
	}
	if(state->dynapse_u0_c1 != sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c1")){
		if(sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c1") < 0 || sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c1") > 255){
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Wrong neuron ID %d, please choose a value from [0,255]", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c1"));
		}else{
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U0);
			caerDeviceConfigSet(stateSource->deviceState,
					DYNAPSE_CONFIG_MONITOR_NEU, 1, (uint32_t) sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c1"));
			caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Monitoring neuron dynapse_u0_c1 num: %d", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c1"));
			state->dynapse_u0_c1 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c1");
		}
	}
	if(state->dynapse_u0_c2 != sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c2")){
		if(sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c2") < 0 || sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c2") > 255){
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Wrong neuron ID %d, please choose a value from [0,255]", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c2"));
		}else{
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U0);
			caerDeviceConfigSet(stateSource->deviceState,
					DYNAPSE_CONFIG_MONITOR_NEU, 2, (uint32_t) sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c2"));
			caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Monitoring neuron dynapse_u0_c2 num: %d", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c2"));
			state->dynapse_u0_c2 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c2");
		}
	}
	if(state->dynapse_u0_c3 != sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c3")){
		if(sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c3") < 0 || sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c3") > 255){
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Wrong neuron ID %d, please choose a value from [0,255]", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c3"));
		}else{
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U0);
			caerDeviceConfigSet(stateSource->deviceState,
					DYNAPSE_CONFIG_MONITOR_NEU, 3, (uint32_t) sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c3"));
			caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Monitoring neuron dynapse_u0_c3 num: %d", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c3"));
			state->dynapse_u0_c3 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u0_c3");
		}
	}

	if(state->dynapse_u1_c0 != sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c0")){
		if(sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c0") < 0 || sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c0") > 255){
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Wrong neuron ID %d, please choose a value from [0,255]", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c0"));
		}else{
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U1);
			caerDeviceConfigSet(stateSource->deviceState,
					DYNAPSE_CONFIG_MONITOR_NEU, 0, (uint32_t) sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c0"));
			caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Monitoring neuron dynapse_u1_c0 num: %d", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c0"));
			state->dynapse_u1_c0 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c0");
		}

	}
	if(state->dynapse_u1_c1 != sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c1")){
		if(sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c1") < 0 || sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c1") > 255){
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Wrong neuron ID %d, please choose a value from [0,255]", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c1"));
		}else{
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U1);
			caerDeviceConfigSet(stateSource->deviceState,
					DYNAPSE_CONFIG_MONITOR_NEU, 1, (uint32_t)  sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c1"));
			caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Monitoring neuron dynapse_u1_c1 num: %d", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c1"));
			state->dynapse_u1_c1 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c1");
		}
	}
	if(state->dynapse_u1_c2 != sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c2")){
		if(sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c2") < 0 || sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c2") > 255){
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Wrong neuron ID %d, please choose a value from [0,255]", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c2"));
		}else{
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U1);
			caerDeviceConfigSet(stateSource->deviceState,
					DYNAPSE_CONFIG_MONITOR_NEU, 2, (uint32_t) sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c2"));
			caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Monitoring neuron dynapse_u1_c2 num: %d", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c2"));
			state->dynapse_u1_c2 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c2");
		}
	}
	if(state->dynapse_u1_c3 != sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c3")){
		if(sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c3") < 0 || sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c3") > 255){
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Wrong neuron ID %d, please choose a value from [0,255]", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c3"));
		}else{
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U1);
			caerDeviceConfigSet(stateSource->deviceState,
					DYNAPSE_CONFIG_MONITOR_NEU, 3, (uint32_t) sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c3"));
			caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Monitoring neuron dynapse_u1_c3 num: %d", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c3"));
			state->dynapse_u1_c3 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u1_c3");
		}
	}

	if(state->dynapse_u2_c0 != sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c0")){
		if(sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c0") < 0 || sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c0") > 255){
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Wrong neuron ID %d, please choose a value from [0,255]", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c0"));
		}else{
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U2);
			caerDeviceConfigSet(stateSource->deviceState,
					DYNAPSE_CONFIG_MONITOR_NEU, 0, (uint32_t) sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c0"));
			caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Monitoring neuron dynapse_u2_c0 num: %d", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c0"));
			state->dynapse_u2_c0 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c0");
		}
	}
	if(state->dynapse_u2_c1 != sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c1")){
		if(sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c1") < 0 || sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c1") > 255){
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Wrong neuron ID %d, please choose a value from [0,255]", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c1"));
		}else{
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U2);
			caerDeviceConfigSet(stateSource->deviceState,
					DYNAPSE_CONFIG_MONITOR_NEU, 1, (uint32_t) sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c1"));
			caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Monitoring neuron dynapse_u2_c1 num: %d", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c1"));
			state->dynapse_u2_c1 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c1");
		}
	}
	if(state->dynapse_u2_c2 != sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c2")){
		if(sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c2") < 0 || sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c2") > 255){
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Wrong neuron ID %d, please choose a value from [0,255]", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c2"));
		}else{
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U2);
			caerDeviceConfigSet(stateSource->deviceState,
					DYNAPSE_CONFIG_MONITOR_NEU, 2, (uint32_t) sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c2"));
			caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Monitoring neuron dynapse_u2_c2 num: %d", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c2"));
			state->dynapse_u2_c2 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c2");
		}
	}
	if(state->dynapse_u2_c3 != sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c3")){
		if(sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c3") < 0 || sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c3") > 255){
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Wrong neuron ID %d, please choose a value from [0,255]", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c3"));
		}else{
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U2);
			caerDeviceConfigSet(stateSource->deviceState,
					DYNAPSE_CONFIG_MONITOR_NEU, 3, (uint32_t) sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c3"));
			caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Monitoring neuron dynapse_u2_c3 num: %d", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c3"));
			state->dynapse_u2_c3 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u2_c3");
		}
	}

	if(state->dynapse_u3_c0 != sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c0")){
		if(sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c0") < 0 || sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c0") > 255){
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Wrong neuron ID %d, please choose a value from [0,255]", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c0"));
		}else{
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U3);
			caerDeviceConfigSet(stateSource->deviceState,
					DYNAPSE_CONFIG_MONITOR_NEU, 0, (uint32_t) sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c0"));
			caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Monitoring neuron dynapse_u3_c0 num: %d", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c0"));
			state->dynapse_u3_c0 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c0");
		}
	}
	if(state->dynapse_u3_c1 != sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c1")){
		if(sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c1") < 0 || sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c1") > 255){
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Wrong neuron ID %d, please choose a value from [0,255]", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c1"));
		}else{
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U3);
			caerDeviceConfigSet(stateSource->deviceState,
					DYNAPSE_CONFIG_MONITOR_NEU, 1, (uint32_t) sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c1"));
			caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Monitoring neuron dynapse_u3_c1 num: %d", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c1"));
			state->dynapse_u3_c1 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c1");
		}
	}
	if(state->dynapse_u3_c2 != sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c2")){
		if(sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c2") < 0 || sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c2") > 255){
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Wrong neuron ID %d, please choose a value from [0,255]", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c2"));
		}else{
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U3);
			caerDeviceConfigSet(stateSource->deviceState,
					DYNAPSE_CONFIG_MONITOR_NEU, 2, (uint32_t) sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c2"));
			caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Monitoring neuron dynapse_u3_c2 num: %d", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c2"));
			state->dynapse_u3_c2 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c2");
		}
	}
	if(state->dynapse_u3_c3 != sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c3")){
		if(sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c3") < 0 || sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c3") > 255){
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Wrong neuron ID %d, please choose a value from [0,255]", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c3"));
		}else{
			caerDeviceConfigSet(stateSource->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U3);
			caerDeviceConfigSet(stateSource->deviceState,
					DYNAPSE_CONFIG_MONITOR_NEU, 3, (uint32_t) sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c3"));
			caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Monitoring neuron dynapse_u3_c3 num: %d", sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c3"));
			state->dynapse_u3_c3 = sshsNodeGetInt(moduleData->moduleNode, "dynapse_u3_c3");
		}
	}

}

static void caerMonitorNeuFilterExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.

}

static void caerMonitorNeuFilterReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);

}


