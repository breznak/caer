/*
 *  dynapseinit.c
 *
 *  Created on: Nov, 2016
 *      Author: federico
 */

#include "dynapseinit.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/buffers.h"

struct DIFilter_state {
	simple2DBufferLong timestampMap;
	int32_t deltaT;
	int8_t subSampleBy;
};

typedef struct DIFilter_state *DIFilterState;

static bool caerDynapseInitInit(caerModuleData moduleData);
static void caerDynapseInitRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerDynapseInitConfig(caerModuleData moduleData);
static void caerDynapseInitExit(caerModuleData moduleData);
static void caerDynapseInitReset(caerModuleData moduleData, uint16_t resetCallSourceID);

static struct caer_module_functions caerDynapseInitFunctions = { .moduleInit =
	&caerDynapseInitInit, .moduleRun = &caerDynapseInitRun, .moduleConfig =
	&caerDynapseInitConfig, .moduleExit = &caerDynapseInitExit, .moduleReset =
	&caerDynapseInitReset };

void caerDynapseInit(uint16_t moduleID, caerSpikeEventPacket spike) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "DynapseInit", CAER_MODULE_PROCESSOR);
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerDynapseInitFunctions, moduleData, sizeof(struct DIFilter_state), 1, spike);
}

static bool caerDynapseInitInit(caerModuleData moduleData) {
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "deltaT", 30000);
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "subSampleBy", 0);

	DIFilterState state = moduleData->moduleState;

	state->deltaT = sshsNodeGetInt(moduleData->moduleNode, "deltaT");
	state->subSampleBy = sshsNodeGetByte(moduleData->moduleNode, "subSampleBy");

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);
    
    
	return (true);
}

static void caerDynapseInitRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
    caerSpikeEventPacket spike = va_arg(args, caerSpikeEventPacket);

    // Only process packets with content.
    if (spike == NULL) {
        caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString, "NO SPIKES\n");
        return;
    }

	DIFilterState state = moduleData->moduleState;

    // Iterate over events and filter out ones that are not supported by other
    // events within a certain region in the specified timeframe.
    CAER_SPIKE_ITERATOR_VALID_START(spike)

        uint64_t ts = caerSpikeEventGetTimestamp(caerSpikeIteratorElement);
        uint64_t neuronId = caerSpikeEventGetNeuronID(caerSpikeIteratorElement);
        uint64_t sourcecoreId = caerSpikeEventGetSourceCoreID(caerSpikeIteratorElement); // which core is from
        uint64_t coreId = caerSpikeEventGetChipID(caerSpikeIteratorElement);// destination core (used as chip id)

        printf("SPIKE: ts %d , neuronID: %d , sourcecoreID: %d, ascoreID: %d\n",ts, neuronId, sourcecoreId, coreId);
    CAER_SPIKE_ITERATOR_ALL_END
    
    
}

static void caerDynapseInitConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	DIFilterState state = moduleData->moduleState;

	state->deltaT = sshsNodeGetInt(moduleData->moduleNode, "deltaT");
	state->subSampleBy = sshsNodeGetByte(moduleData->moduleNode, "subSampleBy");
}

static void caerDynapseInitExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	DIFilterState state = moduleData->moduleState;

}

static void caerDynapseInitReset(caerModuleData moduleData, uint16_t resetCallSourceID) {
	UNUSED_ARGUMENT(resetCallSourceID);

	DIFilterState state = moduleData->moduleState;

}
