#include "dynapse_common.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/buffers.h"
#include "libcaer/devices/dynapse.h"
#include "ext/colorjet/colorjet.h"

static uint32_t convertBias(const char *biasName, const char* lowhi, const char*cl, const char*sex, uint8_t enal,
	uint16_t fineValue, uint8_t coarseValue, uint8_t special);
static uint32_t generateCoarseFineBiasParent(sshsNode biasNode, const char *biasName);
static uint32_t generateCoarseFineBias(sshsNode biasNode);
static void systemConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void usbConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void spikeConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void sramConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void camConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void usbConfigSend(sshsNode node, caerModuleData moduleData);
static void biasConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void updateLowPowerBiases(caerModuleData moduleData, int chipid);
static void updateSilentBiases(caerModuleData moduleData, int chipid);
static char *int2bin(int a);
static bool EnableStimuliGen(caerModuleData moduleData);
static bool DisableStimuliGen(caerModuleData moduleData, int16_t eventSourceID);
bool caerDynapseSetBiasBits(void* spikeGenState, uint32_t chipId, uint32_t coreId, const char *biasName_t,
	uint8_t coarseValue, uint16_t fineValue, const char *lowHigh, const char *npBias);

bool EnableStimuliGen(caerModuleData moduleData) {
	sshsNode deviceConfigNode = sshsGetRelativeNode(moduleData->moduleNode, chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
	sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNode, "spikeGen/");

	sshsNodePutBool(spikeNode, "doStimPrimitiveBias", true);
	return (true);
}

bool DisableStimuliGen(caerModuleData moduleData, int16_t eventSourceID) {

	sshsNode deviceConfigNode = sshsGetRelativeNode(moduleData->moduleNode, chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));
	sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNode, "spikeGen/");

	sshsNodePutBool(spikeNode, "doStimPrimitiveBias", false);
	return (true);
}

const char *chipIDToName(int16_t chipID, bool withEndSlash) {
	switch (chipID) {
		case DYNAPSE_CONFIG_DYNAPSE_U0: {
			return ((withEndSlash) ? ("DYNAPSE_CONFIG_DYNAPSE_U0/") : ("DYNAPSE_CONFIG_DYNAPSE_U0"));
			break;
		}
		case DYNAPSE_CONFIG_DYNAPSE_U1: {
			return ((withEndSlash) ? ("DYNAPSE_CONFIG_DYNAPSE_U1/") : ("DYNAPSE_CONFIG_DYNAPSE_U1"));
			break;
		}
		case DYNAPSE_CONFIG_DYNAPSE_U2: {
			return ((withEndSlash) ? ("DYNAPSE_CONFIG_DYNAPSE_U2/") : ("DYNAPSE_CONFIG_DYNAPSE_U2"));
			break;
		}
		case DYNAPSE_CONFIG_DYNAPSE_U3: {
			return ((withEndSlash) ? ("DYNAPSE_CONFIG_DYNAPSE_U3/") : ("DYNAPSE_CONFIG_DYNAPSE_U3"));
			break;
		}
		case DYNAPSE_CHIP_DYNAPSE: {
			return ((withEndSlash) ? ("DYNAPSEFX2/") : ("DYNAPSEFX2"));
			break;
		}
	}
	printf("unknown device id exiting...\n");
	exit(1);
	return ((withEndSlash) ? ("Unknown/") : ("Unknown"));
}

static void mainloopDataNotifyIncrease(void *p) {
	caerMainloopData mainloopData = p;

	atomic_fetch_add_explicit(&mainloopData->dataAvailable, 1, memory_order_release);
}

static void mainloopDataNotifyDecrease(void *p) {
	caerMainloopData mainloopData = p;

	// No special memory order for decrease, because the acquire load to even start running
	// through a mainloop already synchronizes with the release store above.
	atomic_fetch_sub_explicit(&mainloopData->dataAvailable, 1, memory_order_relaxed);
}

static void moduleShutdownNotify(void *p) {
	sshsNode moduleNode = p;

	// Ensure parent also shuts down (on disconnected device for example).
	sshsNodePutBool(moduleNode, "running", false);
}

static void chipConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	if (event == SSHS_ATTRIBUTE_MODIFIED) {
		/*if (changeType == SSHS_INT
		 && caerStrEquals(changeKey, "BufferNumber")) {
		 caerDeviceConfigSet(((caerInputDynapseState) moduleData->moduleState)->deviceState, CAER_HOST_CONFIG_USB,
		 CAER_HOST_CONFIG_USB_BUFFER_NUMBER, U32T(changeValue.iint));
		 } else if (changeType == SSHS_INT
		 && caerStrEquals(changeKey, "BufferSize")) {
		 caerDeviceConfigSet(((caerInputDynapseState) moduleData->moduleState)->deviceState, CAER_HOST_CONFIG_USB,
		 CAER_HOST_CONFIG_USB_BUFFER_SIZE, U32T(changeValue.iint));
		 } else if (changeType == SSHS_SHORT
		 && caerStrEquals(changeKey, "EarlyPacketDelay")) {
		 caerDeviceConfigSet(((caerInputDynapseState) moduleData->moduleState)->deviceState, DYNAPSE_CONFIG_USB,
		 DYNAPSE_CONFIG_USB_EARLY_PACKET_DELAY, U32T(changeValue.ishort));
		 } else if (changeType == SSHS_BOOL && caerStrEquals(changeKey, "Run")) {
		 caerDeviceConfigSet(((caerInputDynapseState) moduleData->moduleState)->deviceState, DYNAPSE_CONFIG_USB,
		 DYNAPSE_CONFIG_USB_RUN, changeValue.boolean);
		 }*/
	}

}

static void systemConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerModuleData moduleData = userData;

	if (event == SSHS_ATTRIBUTE_MODIFIED) {
		if (changeType == SSHS_INT && caerStrEquals(changeKey, "PacketContainerMaxPacketSize")) {
			caerDeviceConfigSet(((caerInputDynapseState) moduleData->moduleState)->deviceState,
			CAER_HOST_CONFIG_PACKETS,
			CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_PACKET_SIZE, U32T(changeValue.iint));
		}
		else if (changeType == SSHS_INT && caerStrEquals(changeKey, "PacketContainerInterval")) {
			caerDeviceConfigSet(((caerInputDynapseState) moduleData->moduleState)->deviceState,
			CAER_HOST_CONFIG_PACKETS,
			CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_INTERVAL, U32T(changeValue.iint));
		}
	}
}

static void usbConfigSend(sshsNode node, caerModuleData moduleData) {
	caerDeviceConfigSet(((caerInputDynapseState) moduleData->moduleState)->deviceState,
	CAER_HOST_CONFIG_USB, CAER_HOST_CONFIG_USB_BUFFER_NUMBER, U32T(sshsNodeGetInt(node, "BufferNumber")));
	caerDeviceConfigSet(((caerInputDynapseState) moduleData->moduleState)->deviceState,
	CAER_HOST_CONFIG_USB, CAER_HOST_CONFIG_USB_BUFFER_SIZE, U32T(sshsNodeGetInt(node, "BufferSize")));

	caerDeviceConfigSet(((caerInputDynapseState) moduleData->moduleState)->deviceState,
	DYNAPSE_CONFIG_USB, DYNAPSE_CONFIG_USB_EARLY_PACKET_DELAY, U32T(sshsNodeGetShort(node, "EarlyPacketDelay")));
	caerDeviceConfigSet(((caerInputDynapseState) moduleData->moduleState)->deviceState,
	DYNAPSE_CONFIG_USB, DYNAPSE_CONFIG_USB_RUN, sshsNodeGetBool(node, "Run"));
}

static void sramConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerInputDynapseState state = userData;

	if (event == SSHS_ATTRIBUTE_MODIFIED) {
		if (changeType == SSHS_BOOL && caerStrEquals(changeKey, "doProg") && changeValue.boolean) {
			uint32_t chipid = sshsNodeGetInt(node, "chip_id");
			uint32_t neuron_id = sshsNodeGetInt(node, "neuron_id");
			uint32_t sram_addr = sshsNodeGetInt(node, "sram_addr");
			uint32_t core_id = sshsNodeGetInt(node, "core_id");
			uint32_t dest_core_id = sshsNodeGetInt(node, "dest_core_id");
			uint32_t dx = sshsNodeGetInt(node, "dx");
			uint32_t dy = sshsNodeGetInt(node, "dy");
			bool sx = sshsNodeGetBool(node, "sx");
			bool sy = sshsNodeGetBool(node, "sy");
			uint32_t virtual_core_id = sshsNodeGetInt(node, "virtual_core_id");

			// select chip
			caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,
			DYNAPSE_CONFIG_CHIP_ID, chipid);

			uint32_t bits = 1 << 4 | neuron_id << 7 | sram_addr << 5 | core_id << 15 | 1 << 17 | dest_core_id << 18
				| dx << 22 | sx << 24 | dy << 25 | sy << 27 | virtual_core_id << 28;
			caerLog(CAER_LOG_NOTICE, "samProg", "programming chip id %d with sram event %d\n", chipid, bits);
			caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,
			DYNAPSE_CONFIG_CHIP_CONTENT, bits);
		}
	}
}

static void camConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerInputDynapseState state = userData;

	if (event == SSHS_ATTRIBUTE_MODIFIED) {
		if (changeType == SSHS_BOOL && caerStrEquals(changeKey, "doProg")) {
			if (sshsNodeGetBool(node, "doProg")) {
				uint32_t chipid = sshsNodeGetInt(node, "chip_id");
				uint32_t col_addr = sshsNodeGetInt(node, "col_addr");
				uint32_t row_addr = sshsNodeGetInt(node, "row_addr");
				uint32_t cam_addr = sshsNodeGetInt(node, "cam_addr");
				uint32_t core_id = sshsNodeGetInt(node, "core_id");
				uint32_t core_s = sshsNodeGetInt(node, "core_s");
				uint32_t address = sshsNodeGetInt(node, "address");
				bool ei = sshsNodeGetBool(node, "ei");
				bool fs = sshsNodeGetBool(node, "fs");
				// select chip
				caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,
				DYNAPSE_CONFIG_CHIP_ID, chipid);
				// compose bit address
				uint32_t bits = col_addr | row_addr << 11 | cam_addr << 5 | core_id << 15 | 1 << 17 | core_s << 18
					| address << 20 | ei << 29 | fs << 28;
				caerLog(CAER_LOG_NOTICE, "camProg", "programming chip id %d with cam event %d\n", chipid, bits);
				if (bits > 0) {
					caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,
					DYNAPSE_CONFIG_CHIP_CONTENT, bits);
				}

			}

		}
	}

}

static void spikeConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	//caerModuleData moduleData = userData;
	caerInputDynapseState state = userData;

	if (event == SSHS_ATTRIBUTE_MODIFIED) {
		if (changeType == SSHS_BOOL && caerStrEquals(changeKey, "doStim")) { // && caerStrEquals(changeKey, "doStimBias")
		//atomic_load(&state->genSpikeState.doStim);
			if (changeValue.boolean) {
				caerLog(CAER_LOG_NOTICE, "spikeGen", "stimulation started.\n");
				atomic_store(&state->genSpikeState.done, false); // we just started
				atomic_store(&state->genSpikeState.started, true);
			}
			else {
				caerLog(CAER_LOG_NOTICE, "spikeGen", "stimulation ended.\n");
				atomic_store(&state->genSpikeState.started, false);
				atomic_store(&state->genSpikeState.done, true);
			}
		}
		else if (changeType == SSHS_INT && caerStrEquals(changeKey, "stim_type")) {
			atomic_store(&state->genSpikeState.stim_type, changeValue.iint);
		}
		else if (changeType == SSHS_INT && caerStrEquals(changeKey, "stim_avr")) {
			atomic_store(&state->genSpikeState.stim_avr, changeValue.iint);
		}
		else if (changeType == SSHS_INT && caerStrEquals(changeKey, "stim_std")) {
			atomic_store(&state->genSpikeState.stim_std, changeValue.iint);
		}
		else if (changeType == SSHS_INT && caerStrEquals(changeKey, "stim_duration")) {
			atomic_store(&state->genSpikeState.stim_duration, changeValue.iint);
		}
		else if (changeType == SSHS_BOOL && caerStrEquals(changeKey, "repeat")) {
			atomic_store(&state->genSpikeState.repeat, changeValue.boolean);
		}
		else if (changeType == SSHS_BOOL && caerStrEquals(changeKey, "teaching")) {
			atomic_store(&state->genSpikeState.teaching, changeValue.boolean);
		}
		else if (changeType == SSHS_BOOL && caerStrEquals(changeKey, "sendTeachingStimuli")) {
			atomic_store(&state->genSpikeState.sendTeachingStimuli, changeValue.boolean);
		}
		else if (changeType == SSHS_BOOL && caerStrEquals(changeKey, "sendInhibitoryStimuli")) {
			atomic_store(&state->genSpikeState.sendInhibitoryStimuli, changeValue.boolean);
		}
		else if (changeType == SSHS_BOOL && caerStrEquals(changeKey, "setCam")) {
			atomic_store(&state->genSpikeState.setCam, changeValue.boolean);
		}
		else if (changeType == SSHS_BOOL && caerStrEquals(changeKey, "setCamSingle")) {
			atomic_store(&state->genSpikeState.setCamSingle, changeValue.boolean);
		}
		else if (changeType == SSHS_BOOL && caerStrEquals(changeKey, "clearCam")) {
			atomic_store(&state->genSpikeState.clearCam, changeValue.boolean);
		}
		else if (changeType == SSHS_BOOL && caerStrEquals(changeKey, "clearAllCam")) {
			atomic_store(&state->genSpikeState.clearAllCam, changeValue.boolean);
		}
		else if (changeType == SSHS_BOOL && caerStrEquals(changeKey, "doStimPrimitiveBias")) {
			atomic_store(&state->genSpikeState.doStimPrimitiveBias, changeValue.boolean);
		}
		else if (changeType == SSHS_BOOL && caerStrEquals(changeKey, "doStimPrimitiveCam")) {
			atomic_store(&state->genSpikeState.doStimPrimitiveCam, changeValue.boolean);
		}
		else if (changeType == SSHS_BOOL && caerStrEquals(changeKey, "loadDefaultBiases")) {
			atomic_store(&state->genSpikeState.loadDefaultBiases, changeValue.boolean);
		}
		else if (changeType == SSHS_BOOL && caerStrEquals(changeKey, "running")) {
			atomic_store(&state->genSpikeState.running, changeValue.boolean);
		}
		else if (changeType == SSHS_BOOL && caerStrEquals(changeKey, "sx")) {
			atomic_store(&state->genSpikeState.sx, changeValue.boolean);
		}
		else if (changeType == SSHS_BOOL && caerStrEquals(changeKey, "sy")) {
			atomic_store(&state->genSpikeState.sy, changeValue.boolean);
		}
		else if (changeType == SSHS_INT && caerStrEquals(changeKey, "dx")) {
			atomic_store(&state->genSpikeState.dx, changeValue.iint);
		}
		else if (changeType == SSHS_INT && caerStrEquals(changeKey, "dy")) {
			atomic_store(&state->genSpikeState.dy, changeValue.iint);
		}
		else if (changeType == SSHS_INT && caerStrEquals(changeKey, "core_d")) {
			atomic_store(&state->genSpikeState.core_d, changeValue.iint);
		}
		else if (changeType == SSHS_INT && caerStrEquals(changeKey, "core_s")) {
			atomic_store(&state->genSpikeState.core_s, changeValue.iint);
		}
		else if (changeType == SSHS_INT && caerStrEquals(changeKey, "address")) {
			atomic_store(&state->genSpikeState.address, changeValue.iint);
		}
		else if (changeType == SSHS_INT && caerStrEquals(changeKey, "chip_id")) {
			atomic_store(&state->genSpikeState.chip_id, changeValue.iint);
		}

	}

}

static void usbConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerModuleData moduleData = userData;

	if (event == SSHS_ATTRIBUTE_MODIFIED) {
		if (changeType == SSHS_INT && caerStrEquals(changeKey, "BufferNumber")) {
			caerDeviceConfigSet(((caerInputDynapseState) moduleData->moduleState)->deviceState,
			CAER_HOST_CONFIG_USB, CAER_HOST_CONFIG_USB_BUFFER_NUMBER, U32T(changeValue.iint));
		}
		else if (changeType == SSHS_INT && caerStrEquals(changeKey, "BufferSize")) {
			caerDeviceConfigSet(((caerInputDynapseState) moduleData->moduleState)->deviceState,
			CAER_HOST_CONFIG_USB, CAER_HOST_CONFIG_USB_BUFFER_SIZE, U32T(changeValue.iint));
		}
		else if (changeType == SSHS_SHORT && caerStrEquals(changeKey, "EarlyPacketDelay")) {
			caerDeviceConfigSet(((caerInputDynapseState) moduleData->moduleState)->deviceState,
			DYNAPSE_CONFIG_USB, DYNAPSE_CONFIG_USB_EARLY_PACKET_DELAY, U32T(changeValue.ishort));
		}
		else if (changeType == SSHS_BOOL && caerStrEquals(changeKey, "Run")) {
			caerDeviceConfigSet(((caerInputDynapseState) moduleData->moduleState)->deviceState,
			DYNAPSE_CONFIG_USB, DYNAPSE_CONFIG_USB_RUN, changeValue.boolean);
		}
	}
}

uint32_t generatesBitsCoarseFineBiasSetting(sshsNode node, const char *biasName,
	uint8_t coarseValue, uint16_t fineValue, const char *hlbias, const char *currentLevel, const char *sex,
	bool enabled, int chipid) {

	// Add trailing slash to node name (required!).
	size_t biasNameLength = strlen(biasName);
	char biasNameFull[biasNameLength + 2];
	memcpy(biasNameFull, biasName, biasNameLength);
	biasNameFull[biasNameLength] = '/';
	biasNameFull[biasNameLength + 1] = '\0';

	// Device related configuration has its own sub-node.
	sshsNode deviceConfigNodeLP = sshsGetRelativeNode(node, chipIDToName(chipid, true));

	sshsNode biasNodeLP = sshsGetRelativeNode(deviceConfigNodeLP, "bias/");

	// Create configuration node for this particular bias.
	sshsNode biasConfigNode = sshsGetRelativeNode(biasNodeLP, biasNameFull);

	// Add bias settings.
	sshsNodePutByte(biasConfigNode, "coarseValue", I8T(coarseValue));
	sshsNodePutShort(biasConfigNode, "fineValue", I16T(fineValue));
	sshsNodePutString(biasConfigNode, "BiasLowHi", hlbias);
	sshsNodePutString(biasConfigNode, "currentLevel", currentLevel);
	sshsNodePutString(biasConfigNode, "sex", sex);
	sshsNodePutBool(biasConfigNode, "enabled", enabled);
	sshsNodePutBool(biasConfigNode, "special", false);

	//now send
	const char *nodeName = sshsNodeGetName(biasConfigNode);
	uint32_t value = generateCoarseFineBiasParent(biasConfigNode, nodeName);

	return (value);

}

static void updateCoarseFineBiasSetting(caerModuleData moduleData,
	const char *biasName, uint8_t coarseValue, uint16_t fineValue, const char *hlbias, const char *currentLevel,
	const char *sex, bool enabled, int chipid) {

	// Add trailing slash to node name (required!).
	size_t biasNameLength = strlen(biasName);
	char biasNameFull[biasNameLength + 2];
	memcpy(biasNameFull, biasName, biasNameLength);
	biasNameFull[biasNameLength] = '/';
	biasNameFull[biasNameLength + 1] = '\0';

	// Device related configuration has its own sub-node.
	sshsNode deviceConfigNodeLP = sshsGetRelativeNode(moduleData->moduleNode, chipIDToName(chipid, true));

	sshsNode biasNodeLP = sshsGetRelativeNode(deviceConfigNodeLP, "bias/");

	// Create configuration node for this particular bias.
	sshsNode biasConfigNode = sshsGetRelativeNode(biasNodeLP, biasNameFull);

	// Add bias settings.
	sshsNodePutByte(biasConfigNode, "coarseValue", I8T(coarseValue));
	sshsNodePutShort(biasConfigNode, "fineValue", I16T(fineValue));
	sshsNodePutString(biasConfigNode, "BiasLowHi", hlbias);
	sshsNodePutString(biasConfigNode, "currentLevel", currentLevel);
	sshsNodePutString(biasConfigNode, "sex", sex);
	sshsNodePutBool(biasConfigNode, "enabled", enabled);
	sshsNodePutBool(biasConfigNode, "special", false);

	//now send
	const char *nodeName = sshsNodeGetName(biasConfigNode);

	uint32_t value = generateCoarseFineBiasParent(biasConfigNode, nodeName);

	// finally send configuration via USB
	caerDeviceConfigSet(((caerInputDynapseState) moduleData->moduleState)->deviceState,
	DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, value);

}

static void createCoarseFineBiasSetting(sshsNode biasNode, const char *biasName, uint8_t coarseValue,
	uint16_t fineValue, const char *hlbias, const char *currentLevel, const char *sex, bool enabled) {
	// Add trailing slash to node name (required!).
	size_t biasNameLength = strlen(biasName);
	char biasNameFull[biasNameLength + 2];
	memcpy(biasNameFull, biasName, biasNameLength);
	biasNameFull[biasNameLength] = '/';
	biasNameFull[biasNameLength + 1] = '\0';

	// Create configuration node for this particular bias.
	sshsNode biasConfigNode = sshsGetRelativeNode(biasNode, biasNameFull);

	// Add bias settings.
	sshsNodePutByteIfAbsent(biasConfigNode, "coarseValue", I8T(coarseValue));
	sshsNodePutShortIfAbsent(biasConfigNode, "fineValue", I16T(fineValue));
	sshsNodePutStringIfAbsent(biasConfigNode, "BiasLowHi", hlbias);
	sshsNodePutStringIfAbsent(biasConfigNode, "currentLevel", currentLevel);
	sshsNodePutStringIfAbsent(biasConfigNode, "sex", sex);
	sshsNodePutBoolIfAbsent(biasConfigNode, "enabled", enabled);
	sshsNodePutBoolIfAbsent(biasConfigNode, "special", false);
}

static void biasConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(changeKey);
	UNUSED_ARGUMENT(changeType);
	UNUSED_ARGUMENT(changeValue);

	caerModuleData moduleData = userData;
	struct caer_dynapse_info devInfo = caerDynapseInfoGet(
		((caerInputDynapseState) moduleData->moduleState)->deviceState);

	if (event == SSHS_ATTRIBUTE_MODIFIED) {
		const char *nodeName = sshsNodeGetName(node);

		sshsNode parent = sshsNodeGetParent(node);
		const char *nodeParent = sshsNodeGetName(parent);
		sshsNode grandparent = sshsNodeGetParent(parent);
		const char *nodeGrandParent = sshsNodeGetName(grandparent);
		uint32_t value = generateCoarseFineBiasParent(node, nodeName);

		DisableStimuliGen(moduleData, 1);

		if (caerStrEquals(nodeGrandParent, "DYNAPSE_CONFIG_DYNAPSE_U0")) {
			int retval = caerDeviceConfigSet(((caerInputDynapseState) moduleData->moduleState)->deviceState,
			DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID,
			DYNAPSE_CONFIG_DYNAPSE_U0);
			if (retval == false) {
				caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString,
					"failed to set DYNAPSE_CONFIG_CHIP_ID to DYNAPSE_CONFIG_DYNAPSE_U0");
			}
		}
		else if (caerStrEquals(nodeGrandParent, "DYNAPSE_CONFIG_DYNAPSE_U1")) {
			int retval = caerDeviceConfigSet(((caerInputDynapseState) moduleData->moduleState)->deviceState,
			DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID,
			DYNAPSE_CONFIG_DYNAPSE_U1);
			if (retval == false) {
				caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString,
					"failed to set DYNAPSE_CONFIG_CHIP_ID to DYNAPSE_CONFIG_DYNAPSE_U1");
			}
		}
		else if (caerStrEquals(nodeGrandParent, "DYNAPSE_CONFIG_DYNAPSE_U2")) {
			int retval = caerDeviceConfigSet(((caerInputDynapseState) moduleData->moduleState)->deviceState,
			DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID,
			DYNAPSE_CONFIG_DYNAPSE_U2);
			if (retval == false) {
				caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString,
					"failed to set DYNAPSE_CONFIG_CHIP_ID to DYNAPSE_CONFIG_DYNAPSE_U2");
			}
		}
		else if (caerStrEquals(nodeGrandParent, "DYNAPSE_CONFIG_DYNAPSE_U3")) {
			int retval = caerDeviceConfigSet(((caerInputDynapseState) moduleData->moduleState)->deviceState,
			DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID,
			DYNAPSE_CONFIG_DYNAPSE_U3);
			if (retval == false) {
				caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString,
					"failed to set DYNAPSE_CONFIG_CHIP_ID to DYNAPSE_CONFIG_DYNAPSE_U3");
			}
		}

		// finally send configuration via USB
		int retval = caerDeviceConfigSet(((caerInputDynapseState) moduleData->moduleState)->deviceState,
		DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, value);

		EnableStimuliGen(moduleData);

		if (retval == false) {
			caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString, "failed to set bias");
		}

	}

}

static void updateLowPowerBiases(caerModuleData moduleData, int chipid) {

	// now set default low power biases
	updateCoarseFineBiasSetting(moduleData, "C0_IF_BUF_P", 3, 80, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_IF_RFR_N", 3, 3, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_IF_NMDA_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_IF_DC_P", 3, 60, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_IF_TAU1_N", 5, 10, "LowBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_IF_TAU2_N", 6, 100, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_IF_THR_N", 3, 30, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_IF_AHW_P", 7, 0, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_IF_AHTAU_N", 7, 35, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_IF_AHTHR_N", 7, 0, "HighBias", "Normal", "NBias", true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_IF_CASC_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_PULSE_PWLK_P", 3, 106, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_PS_WEIGHT_INH_F_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_PS_WEIGHT_EXC_S_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_PS_WEIGHT_EXC_F_N", 15, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_NPDPII_TAU_S_P", 7, 40, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_NPDPII_THR_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_NPDPII_THR_S_P", 7, 40, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_NPDPII_THR_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_NPDPII_THR_S_P", 7, 40, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_NPDPII_THR_F_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_NPDPIE_TAU_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_NPDPIE_THR_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_NPDPIE_TAU_F_P", 7, 40, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_NPDPIE_TAU_F_P", 7, 40, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_NPDPIE_THR_F_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_R2R_P", 4, 85, "HighBias", "Normal", "PBias", true, chipid);

	// core 1
	updateCoarseFineBiasSetting(moduleData, "C1_IF_BUF_P", 3, 80, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_IF_RFR_N", 3, 3, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_IF_NMDA_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_IF_DC_P", 3, 60, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_IF_TAU1_N", 5, 10, "LowBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_IF_TAU2_N", 6, 100, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_IF_THR_N", 3, 30, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_IF_AHW_P", 7, 0, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_IF_AHTAU_N", 7, 35, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_IF_AHTHR_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_IF_CASC_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_PULSE_PWLK_P", 3, 106, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_PS_WEIGHT_INH_F_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_PS_WEIGHT_EXC_S_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_PS_WEIGHT_EXC_F_N", 15, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_NPDPII_TAU_S_P", 7, 40, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_NPDPII_THR_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_NPDPII_THR_S_P", 7, 40, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_NPDPII_THR_F_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_NPDPIE_TAU_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_NPDPIE_THR_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_NPDPIE_TAU_F_P", 7, 40, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_NPDPIE_TAU_F_P", 7, 40, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_NPDPIE_THR_F_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_R2R_P", 4, 85, "HighBias", "Normal", "PBias", true,chipid);

	// core 2
	updateCoarseFineBiasSetting(moduleData, "C2_IF_BUF_P", 3, 80, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_IF_RFR_N", 3, 3, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_IF_NMDA_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_IF_DC_P", 3, 60, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_IF_TAU1_N", 5, 10, "LowBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_IF_TAU2_N", 6, 100, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_IF_THR_N", 3, 30, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_IF_AHW_P", 7, 0, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_IF_AHTAU_N", 7, 35, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_IF_AHTHR_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_IF_CASC_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_PULSE_PWLK_P", 3, 106, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_PS_WEIGHT_INH_F_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_PS_WEIGHT_EXC_S_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_PS_WEIGHT_EXC_F_N", 15, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_NPDPII_TAU_S_P", 7, 40, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_NPDPII_THR_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_NPDPII_THR_S_P", 7, 40, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_NPDPII_THR_F_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_NPDPIE_TAU_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_NPDPIE_THR_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_NPDPIE_TAU_F_P", 7, 40, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_NPDPIE_THR_F_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_R2R_P", 4, 85, "HighBias", "Normal", "PBias", true,chipid);

	// core 3
	updateCoarseFineBiasSetting(moduleData, "C3_IF_BUF_P", 3, 80, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_IF_RFR_N", 3, 3, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_IF_NMDA_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_IF_DC_P", 2, 30, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_IF_TAU1_N", 5, 10, "LowBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_IF_TAU2_N", 6, 100, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_IF_THR_N", 3, 30, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_IF_AHW_P", 7, 0, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_IF_AHTAU_N", 7, 35, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_IF_AHTHR_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_IF_CASC_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_PULSE_PWLK_P", 3, 106, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_PS_WEIGHT_INH_F_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_PS_WEIGHT_EXC_S_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_PS_WEIGHT_EXC_F_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_NPDPII_TAU_S_P", 7, 40, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_NPDPII_THR_S_P", 7, 40, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_NPDPII_THR_F_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_NPDPIE_TAU_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_NPDPIE_THR_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_NPDPIE_TAU_F_P", 7, 40, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_NPDPIE_THR_F_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_R2R_P", 4, 85, "HighBias", "Normal", "PBias", true,chipid);

	updateCoarseFineBiasSetting(moduleData, "D_BUFFER", 1, 2, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "D_SSP", 0, 7, "HighBias", "Normal", "PBias", true, chipid);
	updateCoarseFineBiasSetting(moduleData, "D_SSN", 0, 15, "HighBias", "Normal", "PBias", true, chipid);
	updateCoarseFineBiasSetting(moduleData, "U_BUFFER", 1, 2, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "U_SSP", 0, 7, "HighBias", "Normal", "PBias", true, chipid);
	updateCoarseFineBiasSetting(moduleData, "U_SSN", 0, 15, "HighBias", "Normal", "PBias", true, chipid);

}

static void updateSilentBiases(caerModuleData moduleData, int chipid) {

	// make chip silent while programming AER
	// core 0
	updateCoarseFineBiasSetting(moduleData, "C0_IF_BUF_P", 7, 0, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_IF_RFR_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_IF_NMDA_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_IF_DC_P", 7, 0, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_IF_TAU1_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_IF_TAU2_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_IF_THR_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_IF_AHW_P", 7, 0, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_IF_AHTAU_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_IF_AHTHR_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_IF_CASC_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_PULSE_PWLK_P", 7, 0, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_PS_WEIGHT_INH_F_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_PS_WEIGHT_EXC_S_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_PS_WEIGHT_EXC_F_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_NPDPII_TAU_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_NPDPII_THR_F_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_NPDPIE_TAU_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_NPDPIE_THR_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_NPDPIE_TAU_F_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_NPDPIE_THR_F_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C0_R2R_P", 7, 0, "HighBias", "Normal", "PBias", true,chipid);

	// core 1
	updateCoarseFineBiasSetting(moduleData, "C1_IF_BUF_P", 7, 0, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_IF_RFR_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_IF_NMDA_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_IF_DC_P", 7, 0, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_IF_TAU1_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_IF_TAU2_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_IF_THR_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_IF_AHW_P", 7, 0, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_IF_AHTAU_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_IF_AHTHR_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_IF_CASC_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_PULSE_PWLK_P", 7, 0, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_PS_WEIGHT_INH_F_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_PS_WEIGHT_EXC_S_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_PS_WEIGHT_EXC_F_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_NPDPII_TAU_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_NPDPII_THR_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_NPDPII_THR_F_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_NPDPIE_TAU_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_NPDPIE_THR_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_NPDPIE_TAU_F_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_NPDPIE_THR_F_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C1_R2R_P", 7, 0, "HighBias", "Normal", "PBias", true,chipid);

	// core 2
	updateCoarseFineBiasSetting(moduleData, "C2_IF_BUF_P", 7, 0, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_IF_RFR_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_IF_NMDA_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_IF_DC_P", 7, 0, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_IF_TAU1_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_IF_TAU2_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_IF_THR_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_IF_AHW_P", 7, 0, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_IF_AHTAU_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_IF_AHTHR_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_IF_CASC_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_PULSE_PWLK_P", 7, 0, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_PS_WEIGHT_INH_F_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_PS_WEIGHT_EXC_S_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_PS_WEIGHT_EXC_F_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_NPDPII_TAU_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_NPDPII_THR_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_NPDPII_THR_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_NPDPII_THR_F_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_NPDPIE_TAU_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_NPDPIE_THR_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_NPDPIE_TAU_F_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_NPDPIE_THR_F_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C2_R2R_P", 7, 0, "HighBias", "Normal", "PBias", true,chipid);

	// core 3
	updateCoarseFineBiasSetting(moduleData, "C3_IF_BUF_P", 7, 0, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_IF_RFR_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_IF_NMDA_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_IF_DC_P", 7, 0, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_IF_TAU1_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_IF_TAU2_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_IF_THR_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_IF_AHW_P", 7, 0, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_IF_AHTAU_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_IF_AHTHR_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_IF_CASC_N", 7, 0, "HighBias", "Normal", "NBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_PULSE_PWLK_P", 7, 0, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_PS_WEIGHT_INH_F_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_PS_WEIGHT_EXC_S_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_PS_WEIGHT_EXC_F_N", 7, 0, "HighBias", "Normal", "NBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_NPDPII_TAU_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_NPDPII_THR_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_NPDPII_THR_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_NPDPII_THR_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_NPDPII_THR_F_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_NPDPIE_TAU_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_NPDPIE_THR_S_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_NPDPIE_TAU_F_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_NPDPIE_THR_F_P", 7, 0, "HighBias", "Normal", "PBias",true, chipid);
	updateCoarseFineBiasSetting(moduleData, "C3_R2R_P", 7, 0, "HighBias", "Normal", "PBias", true,chipid);

	updateCoarseFineBiasSetting(moduleData, "D_BUFFER", 1, 2, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "D_SSP", 0, 7, "HighBias", "Normal", "PBias", true, chipid);
	updateCoarseFineBiasSetting(moduleData, "D_SSN", 0, 15, "HighBias", "Normal", "PBias", true, chipid);
	updateCoarseFineBiasSetting(moduleData, "U_BUFFER", 1, 2, "HighBias", "Normal", "PBias", true,chipid);
	updateCoarseFineBiasSetting(moduleData, "U_SSP", 0, 7, "HighBias", "Normal", "PBias", true, chipid);
	updateCoarseFineBiasSetting(moduleData, "U_SSN", 0, 15, "HighBias", "Normal", "PBias", true, chipid);

}

static void createDefaultConfiguration(caerModuleData moduleData, int chipid) {

	// Device related configuration has its own sub-node..
	sshsNode deviceConfigNode = sshsGetRelativeNode(moduleData->moduleNode, chipIDToName(chipid, true));

// Chip biases, defaults.
	sshsNode biasNode = sshsGetRelativeNode(deviceConfigNode, "bias/");

	createCoarseFineBiasSetting(biasNode, "C0_IF_BUF_P", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_IF_RFR_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_IF_NMDA_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_IF_DC_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_IF_TAU1_N", 7, 0, "LowBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_IF_TAU2_N", 7, 0, "LowBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_IF_THR_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_IF_AHW_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_IF_AHTAU_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_IF_AHTHR_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_IF_CASC_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_PULSE_PWLK_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_PS_WEIGHT_INH_F_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_PS_WEIGHT_EXC_S_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_PS_WEIGHT_EXC_F_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_NPDPII_TAU_S_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_NPDPII_THR_S_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_NPDPII_TAU_F_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_NPDPII_THR_F_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_NPDPIE_TAU_S_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_NPDPIE_THR_S_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_NPDPIE_TAU_F_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_NPDPIE_THR_F_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_R2R_P", 7, 0, "HighBias", "Normal", "PBias", true);

	createCoarseFineBiasSetting(biasNode, "C1_IF_BUF_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_IF_RFR_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_IF_NMDA_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_IF_DC_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_IF_TAU1_N", 7, 0, "LowBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_IF_TAU2_N", 7, 0, "LowBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_IF_THR_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_IF_AHW_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_IF_AHTAU_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_IF_AHTHR_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_IF_CASC_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_PULSE_PWLK_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_PS_WEIGHT_INH_F_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_PS_WEIGHT_EXC_S_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_PS_WEIGHT_EXC_F_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_NPDPII_TAU_S_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_NPDPII_THR_S_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_NPDPII_TAU_F_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_NPDPII_THR_F_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_NPDPIE_TAU_S_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_NPDPIE_THR_S_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_NPDPIE_TAU_F_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_NPDPIE_THR_F_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_R2R_P", 7, 0, "HighBias", "Normal", "PBias", true);

	createCoarseFineBiasSetting(biasNode, "C2_IF_BUF_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_IF_RFR_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_IF_NMDA_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_IF_DC_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_IF_TAU1_N", 7, 0, "LowBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_IF_TAU2_N", 7, 0, "LowBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_IF_THR_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_IF_AHW_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_IF_AHTAU_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_IF_AHTHR_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_IF_CASC_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_PULSE_PWLK_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_PS_WEIGHT_INH_F_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_PS_WEIGHT_EXC_S_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_PS_WEIGHT_EXC_F_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_NPDPII_TAU_S_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_NPDPII_THR_S_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_NPDPII_TAU_F_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_NPDPII_THR_F_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_NPDPIE_TAU_S_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_NPDPIE_THR_S_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_NPDPIE_TAU_F_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_NPDPIE_THR_F_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_R2R_P", 7, 0, "HighBias", "Normal", "PBias", true);

	createCoarseFineBiasSetting(biasNode, "C3_IF_BUF_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_IF_RFR_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_IF_NMDA_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_IF_DC_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_IF_TAU1_N", 7, 0, "LowBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_IF_TAU2_N", 7, 0, "LowBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_IF_THR_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_IF_AHW_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_IF_AHTAU_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_IF_AHTHR_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_IF_CASC_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_PULSE_PWLK_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_PS_WEIGHT_INH_S_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_PS_WEIGHT_INH_F_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_PS_WEIGHT_EXC_S_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_PS_WEIGHT_EXC_F_N", 7, 0, "HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_NPDPII_TAU_S_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_NPDPII_THR_S_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_NPDPII_TAU_F_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_NPDPII_THR_F_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_NPDPIE_TAU_S_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_NPDPIE_THR_S_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_NPDPIE_TAU_F_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_NPDPIE_THR_F_P", 7, 0, "HighBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_R2R_P", 7, 0, "HighBias", "Normal", "PBias", true);

	createCoarseFineBiasSetting(biasNode, "U_BUFFER", 1, 80, "LowBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "U_SSP", 0, 7, "LowBias", "Cascade", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "U_SSN", 0, 15, "LowBias", "Normal", "PBias", true);

	createCoarseFineBiasSetting(biasNode, "D_BUFFER", 1, 80, "LowBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "D_SSP", 0, 7, "LowBias", "Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "D_SSN", 0, 15, "LowBias", "Normal", "PBias", true);

}

static uint32_t convertBias(const char *biasName, const char* lowhi, const char*cl, const char*sex, uint8_t enal,
	uint16_t fineValue, uint8_t coarseValue, uint8_t special) {

	int32_t confbits;
	int32_t addr = 0;
	int32_t inbits = 0;

	/*start names*/
	if (caerStrEquals(biasName, "C0_PULSE_PWLK_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C0_PULSE_PWLK_P;
	}
	if (caerStrEquals(biasName, "C0_PS_WEIGHT_INH_S_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C0_PS_WEIGHT_INH_S_N;
	}
	if (caerStrEquals(biasName, "C0_PS_WEIGHT_INH_F_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C0_PS_WEIGHT_INH_F_N;
	}
	if (caerStrEquals(biasName, "C0_PS_WEIGHT_EXC_S_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C0_PS_WEIGHT_EXC_S_N;
	}
	if (caerStrEquals(biasName, "C0_PS_WEIGHT_EXC_F_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C0_PS_WEIGHT_EXC_F_N;
	}
	if (caerStrEquals(biasName, "C0_IF_RFR_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C0_IF_RFR_N;
	}
	if (caerStrEquals(biasName, "C0_IF_TAU1_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C0_IF_TAU1_N;
	}
	if (caerStrEquals(biasName, "C0_IF_AHTAU_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C0_IF_AHTAU_N;
	}
	if (caerStrEquals(biasName, "C0_IF_CASC_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C0_IF_CASC_N;
	}
	if (caerStrEquals(biasName, "C0_IF_TAU2_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C0_IF_TAU2_N;
	}
	if (caerStrEquals(biasName, "C0_IF_BUF_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C0_IF_BUF_P;
	}
	if (caerStrEquals(biasName, "C0_IF_AHTHR_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C0_IF_AHTHR_N;
	}
	if (caerStrEquals(biasName, "C0_IF_THR_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C0_IF_THR_N;
	}
	if (caerStrEquals(biasName, "C0_NPDPIE_THR_S_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C0_NPDPIE_THR_S_P;
	}
	if (caerStrEquals(biasName, "C0_NPDPIE_THR_F_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C0_NPDPIE_THR_F_P;
	}
	if (caerStrEquals(biasName, "C0_NPDPII_THR_F_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C0_NPDPII_THR_F_P;
	}
	if (caerStrEquals(biasName, "C0_NPDPII_THR_S_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C0_NPDPII_THR_S_P;
	}
	if (caerStrEquals(biasName, "C0_IF_NMDA_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C0_IF_NMDA_N;
	}
	if (caerStrEquals(biasName, "C0_IF_DC_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C0_IF_DC_P;
	}
	if (caerStrEquals(biasName, "C0_IF_AHW_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C0_IF_AHW_P;
	}
	if (caerStrEquals(biasName, "C0_NPDPII_TAU_S_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C0_NPDPII_TAU_S_P;
	}
	if (caerStrEquals(biasName, "C0_NPDPII_TAU_F_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C0_NPDPII_TAU_F_P;
	}
	if (caerStrEquals(biasName, "C0_NPDPIE_TAU_F_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C0_NPDPIE_TAU_F_P;
	}
	if (caerStrEquals(biasName, "C0_NPDPIE_TAU_S_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C0_NPDPIE_TAU_S_P;
	}
	if (caerStrEquals(biasName, "C0_R2R_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C0_R2R_P;
	}

	if (caerStrEquals(biasName, "C1_PULSE_PWLK_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C1_PULSE_PWLK_P;
	}
	if (caerStrEquals(biasName, "C1_PS_WEIGHT_INH_S_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C1_PS_WEIGHT_INH_S_N;
	}
	if (caerStrEquals(biasName, "C1_PS_WEIGHT_INH_F_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C1_PS_WEIGHT_INH_F_N;
	}
	if (caerStrEquals(biasName, "C1_PS_WEIGHT_EXC_S_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C1_PS_WEIGHT_EXC_S_N;
	}
	if (caerStrEquals(biasName, "C1_PS_WEIGHT_EXC_F_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C1_PS_WEIGHT_EXC_F_N;
	}
	if (caerStrEquals(biasName, "C1_IF_RFR_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C1_IF_RFR_N;
	}
	if (caerStrEquals(biasName, "C1_IF_TAU1_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C1_IF_TAU1_N;
	}
	if (caerStrEquals(biasName, "C1_IF_AHTAU_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C1_IF_AHTAU_N;
	}
	if (caerStrEquals(biasName, "C1_IF_CASC_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C1_IF_CASC_N;
	}
	if (caerStrEquals(biasName, "C1_IF_TAU2_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C1_IF_TAU2_N;
	}
	if (caerStrEquals(biasName, "C1_IF_BUF_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C1_IF_BUF_P;
	}
	if (caerStrEquals(biasName, "C1_IF_AHTHR_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C1_IF_AHTHR_N;
	}
	if (caerStrEquals(biasName, "C1_IF_THR_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C1_IF_THR_N;
	}
	if (caerStrEquals(biasName, "C1_NPDPIE_THR_S_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C1_NPDPIE_THR_S_P;
	}
	if (caerStrEquals(biasName, "C1_NPDPIE_THR_F_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C1_NPDPIE_THR_F_P;
	}
	if (caerStrEquals(biasName, "C1_NPDPII_THR_F_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C1_NPDPII_THR_F_P;
	}
	if (caerStrEquals(biasName, "C1_NPDPII_THR_S_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C1_NPDPII_THR_S_P;
	}
	if (caerStrEquals(biasName, "C1_IF_NMDA_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C1_IF_NMDA_N;
	}
	if (caerStrEquals(biasName, "C1_IF_DC_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C1_IF_DC_P;
	}
	if (caerStrEquals(biasName, "C1_IF_AHW_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C1_IF_AHW_P;
	}
	if (caerStrEquals(biasName, "C1_NPDPII_TAU_S_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C1_NPDPII_TAU_S_P;
	}
	if (caerStrEquals(biasName, "C1_NPDPII_TAU_F_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C1_NPDPII_TAU_F_P;
	}
	if (caerStrEquals(biasName, "C1_NPDPIE_TAU_F_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C1_NPDPIE_TAU_F_P;
	}
	if (caerStrEquals(biasName, "C1_NPDPIE_TAU_S_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C1_NPDPIE_TAU_S_P;
	}
	if (caerStrEquals(biasName, "C1_R2R_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C1_R2R_P;
	}

	if (caerStrEquals(biasName, "C2_PULSE_PWLK_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C2_PULSE_PWLK_P;
	}
	if (caerStrEquals(biasName, "C2_PS_WEIGHT_INH_S_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C2_PS_WEIGHT_INH_S_N;
	}
	if (caerStrEquals(biasName, "C2_PS_WEIGHT_INH_F_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C2_PS_WEIGHT_INH_F_N;
	}
	if (caerStrEquals(biasName, "C2_PS_WEIGHT_EXC_S_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C2_PS_WEIGHT_EXC_S_N;
	}
	if (caerStrEquals(biasName, "C2_PS_WEIGHT_EXC_F_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C2_PS_WEIGHT_EXC_F_N;
	}
	if (caerStrEquals(biasName, "C2_IF_RFR_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C2_IF_RFR_N;
	}
	if (caerStrEquals(biasName, "C2_IF_TAU1_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C2_IF_TAU1_N;
	}
	if (caerStrEquals(biasName, "C2_IF_AHTAU_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C2_IF_AHTAU_N;
	}
	if (caerStrEquals(biasName, "C2_IF_CASC_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C2_IF_CASC_N;
	}
	if (caerStrEquals(biasName, "C2_IF_TAU2_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C2_IF_TAU2_N;
	}
	if (caerStrEquals(biasName, "C2_IF_BUF_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C2_IF_BUF_P;
	}
	if (caerStrEquals(biasName, "C2_IF_AHTHR_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C2_IF_AHTHR_N;
	}
	if (caerStrEquals(biasName, "C2_IF_THR_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C2_IF_THR_N;
	}
	if (caerStrEquals(biasName, "C2_NPDPIE_THR_S_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C2_NPDPIE_THR_S_P;
	}
	if (caerStrEquals(biasName, "C2_NPDPIE_THR_F_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C2_NPDPIE_THR_F_P;
	}
	if (caerStrEquals(biasName, "C2_NPDPII_THR_F_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C2_NPDPII_THR_F_P;
	}
	if (caerStrEquals(biasName, "C2_NPDPII_THR_S_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C2_NPDPII_THR_S_P;
	}
	if (caerStrEquals(biasName, "C2_IF_NMDA_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C2_IF_NMDA_N;
	}
	if (caerStrEquals(biasName, "C2_IF_DC_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C2_IF_DC_P;
	}
	if (caerStrEquals(biasName, "C2_IF_AHW_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C2_IF_AHW_P;
	}
	if (caerStrEquals(biasName, "C2_NPDPII_TAU_S_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C2_NPDPII_TAU_S_P;
	}
	if (caerStrEquals(biasName, "C2_NPDPII_TAU_F_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C2_NPDPII_TAU_F_P;
	}
	if (caerStrEquals(biasName, "C2_NPDPIE_TAU_F_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C2_NPDPIE_TAU_F_P;
	}
	if (caerStrEquals(biasName, "C2_NPDPIE_TAU_S_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C2_NPDPIE_TAU_S_P;
	}
	if (caerStrEquals(biasName, "C2_R2R_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C2_R2R_P;
	}

	if (caerStrEquals(biasName, "C3_PULSE_PWLK_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C3_PULSE_PWLK_P;
	}
	if (caerStrEquals(biasName, "C3_PS_WEIGHT_INH_S_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C3_PS_WEIGHT_INH_S_N;
	}
	if (caerStrEquals(biasName, "C3_PS_WEIGHT_INH_F_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C3_PS_WEIGHT_INH_F_N;
	}
	if (caerStrEquals(biasName, "C3_PS_WEIGHT_EXC_S_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C3_PS_WEIGHT_EXC_S_N;
	}
	if (caerStrEquals(biasName, "C3_PS_WEIGHT_EXC_F_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C3_PS_WEIGHT_EXC_F_N;
	}
	if (caerStrEquals(biasName, "C3_IF_RFR_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C3_IF_RFR_N;
	}
	if (caerStrEquals(biasName, "C3_IF_TAU1_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C3_IF_TAU1_N;
	}
	if (caerStrEquals(biasName, "C3_IF_AHTAU_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C3_IF_AHTAU_N;
	}
	if (caerStrEquals(biasName, "C3_IF_CASC_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C3_IF_CASC_N;
	}
	if (caerStrEquals(biasName, "C3_IF_TAU2_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C3_IF_TAU2_N;
	}
	if (caerStrEquals(biasName, "C3_IF_BUF_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C3_IF_BUF_P;
	}
	if (caerStrEquals(biasName, "C3_IF_AHTHR_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C3_IF_AHTHR_N;
	}
	if (caerStrEquals(biasName, "C3_IF_THR_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C3_IF_THR_N;
	}
	if (caerStrEquals(biasName, "C3_NPDPIE_THR_S_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C3_NPDPIE_THR_S_P;
	}
	if (caerStrEquals(biasName, "C3_NPDPIE_THR_F_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C3_NPDPIE_THR_F_P;
	}
	if (caerStrEquals(biasName, "C3_NPDPII_THR_F_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C3_NPDPII_THR_F_P;
	}
	if (caerStrEquals(biasName, "C3_NPDPII_THR_S_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C3_NPDPII_THR_S_P;
	}
	if (caerStrEquals(biasName, "C3_IF_NMDA_N")) {
		addr = DYNAPSE_CONFIG_BIAS_C3_IF_NMDA_N;
	}
	if (caerStrEquals(biasName, "C3_IF_DC_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C3_IF_DC_P;
	}
	if (caerStrEquals(biasName, "C3_IF_AHW_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C3_IF_AHW_P;
	}
	if (caerStrEquals(biasName, "C3_NPDPII_TAU_S_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C3_NPDPII_TAU_S_P;
	}
	if (caerStrEquals(biasName, "C3_NPDPII_TAU_F_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C3_NPDPII_TAU_F_P;
	}
	if (caerStrEquals(biasName, "C3_NPDPIE_TAU_F_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C3_NPDPIE_TAU_F_P;
	}
	if (caerStrEquals(biasName, "C3_NPDPIE_TAU_S_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C3_NPDPIE_TAU_S_P;
	}
	if (caerStrEquals(biasName, "C3_R2R_P")) {
		addr = DYNAPSE_CONFIG_BIAS_C3_R2R_P;
	}

	if (caerStrEquals(biasName, "U_BUFFER")) {
		addr = DYNAPSE_CONFIG_BIAS_U_BUFFER;
	}
	if (caerStrEquals(biasName, "U_SSP")) {
		addr = DYNAPSE_CONFIG_BIAS_U_SSP;
	}
	if (caerStrEquals(biasName, "U_SSN")) {
		addr = DYNAPSE_CONFIG_BIAS_U_SSN;
	}
	if (caerStrEquals(biasName, "D_BUFFER")) {
		addr = DYNAPSE_CONFIG_BIAS_D_BUFFER;
	}
	if (caerStrEquals(biasName, "D_SSP")) {
		addr = DYNAPSE_CONFIG_BIAS_D_SSP;
	}
	if (caerStrEquals(biasName, "D_SSN")) {
		addr = DYNAPSE_CONFIG_BIAS_D_SSN;
	}

	uint8_t lws, ssx, cls;
	if (caerStrEquals(lowhi, "HighBias")) {
		lws = 1;
	}
	else {
		lws = 0;
	}
	if (caerStrEquals(sex, "NBias")) {
		ssx = 1;
	}
	else {
		ssx = 0;
	}
	if (caerStrEquals(cl, "Normal")) {
		cls = 1;
	}
	else {
		cls = 0;
	}

	caerLog(CAER_LOG_DEBUG, "BIAS CONFIGURE ", " biasName %s --> ADDR %d coarseValue %d\n", biasName, addr,
		coarseValue);

	/*end names*/
	if (enal == 1) {
		confbits = lws << 3 | cls << 2 | ssx << 1 | 1;
	}
	else {
		confbits = lws << 3 | cls << 2 | ssx << 1;
	}

	uint8_t coarseRev = 0;
	/*reverse*/

	/*same as: sum(1 << (2 - i) for i in range(3) if 2 >> i & 1)*/
	if (coarseValue == 0)
		coarseValue = 0;
	else if (coarseValue == 1)
		coarseValue = 4;
	else if (coarseValue == 2)
		coarseValue = 2;
	else if (coarseValue == 3)
		coarseValue = 6;
	else if (coarseValue == 4)
		coarseValue = 1;
	else if (coarseValue == 5)
		coarseValue = 5;
	else if (coarseValue == 6)
		coarseValue = 3;
	else if (coarseValue == 7)
		coarseValue = 7;

	coarseRev = coarseValue;

	// snn and ssp
	if (addr == DYNAPSE_CONFIG_BIAS_U_SSP || addr == DYNAPSE_CONFIG_BIAS_U_SSN || addr == DYNAPSE_CONFIG_BIAS_D_SSP
		|| addr == DYNAPSE_CONFIG_BIAS_D_SSN) {
		confbits = 0;
		inbits = addr << 18 | 1 << 16 | 63 << 10 | fineValue << 4 | confbits;
	}
	else if (addr == DYNAPSE_CONFIG_BIAS_D_BUFFER || addr == DYNAPSE_CONFIG_BIAS_U_BUFFER) {
		confbits = 0;
		inbits = addr << 18 | 1 << 16 | special << 15 | coarseRev << 12 | fineValue << 4;
	}
	else {
		inbits = addr << 18 | 1 << 16 | special << 15 | coarseRev << 12 | fineValue << 4 | confbits;
	}

	return inbits;

}

char *int2bin(int a) {
	char *str, *tmp;
	int cnt = 31;
	str = (char *) malloc(32); /*32 + 1 , because its a 32 bit bin number*/
	tmp = str;
	while (cnt > -1) {
		str[cnt] = '0';
		cnt--;
	}
	cnt = 31;
	while (a > 0) {
		if (a % 2 == 1) {
			str[cnt] = '1';
		}
		cnt--;
		a = a / 2;
	}
	return tmp;
}

static void biasConfigSend(sshsNode node, caerModuleData moduleData, struct caer_dynapse_info *devInfo) {

	// get the number of children biases
	uint32_t value;
	size_t biasNodesLength = 0;
	sshsNode *biasNodes = sshsNodeGetChildren(node, &biasNodesLength);
	char *nodeName = sshsNodeGetName(node);
	caerLog(CAER_LOG_DEBUG, moduleData->moduleSubSystemString, "BIAS LENGHT ... %d NAME %s\n", biasNodesLength,
		nodeName);

	// SEND DEFAULT BIASES TO ALL CHIPS in BOARD (0,3) only chip id 4 for now
	for (uint32_t this_chip = 4; this_chip < 5; this_chip++) {
		// Let's select this chip for configuration
		if (!caerDeviceConfigSet(((caerInputDynapseState) moduleData->moduleState)->deviceState,
		DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_ID, this_chip)) {
			caerLog(CAER_LOG_DEBUG, moduleData->moduleSubSystemString, "Failed to configure chip bits");
		}

		// send configuration, one bias per time
		if (biasNodes != NULL) {
			for (size_t i = 0; i < biasNodesLength; i++) {
				nodeName = sshsNodeGetName(biasNodes[i]);
				value = generateCoarseFineBiasParent(biasNodes[i], nodeName);

				// finally send configuration via USB
				caerDeviceConfigSet(((caerInputDynapseState) moduleData->moduleState)->deviceState,
				DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, value);

			}
			free(biasNodes);
		}
	}

}

static uint32_t generateCoarseFineBiasParent(sshsNode biasNode, const char * biasName) {
	// Add trailing slash to node name (required!).
	size_t biasNameLength = strlen(biasName);
	char biasNameFull[biasNameLength + 2];
	memcpy(biasNameFull, biasName, biasNameLength);
	biasNameFull[biasNameLength] = '/';
	biasNameFull[biasNameLength + 1] = '\0';

	// Get bias configuration node.
	sshsNode biasConfigNode = sshsGetRelativeNode(biasNode, biasNameFull);

	return (generateCoarseFineBias(biasNode));
}

static uint32_t generateCoarseFineBias(sshsNode biasNode) {

	char *biasName = sshsNodeGetName(biasNode);

	bool enal = sshsNodeGetBool(biasNode, "enabled");
	bool special = sshsNodeGetBool(biasNode, "special");
	uint8_t coarseValue = sshsNodeGetByte(biasNode, "coarseValue");
	uint16_t fineValue = sshsNodeGetShort(biasNode, "fineValue");
	char * lowhi = sshsNodeGetString(biasNode, "BiasLowHi");
	char * cl = sshsNodeGetString(biasNode, "currentLevel");
	char * sex = sshsNodeGetString(biasNode, "sex");

	// generates bits values
	uint8_t enabled;
	uint8_t specialed;
	if (enal)
		enabled = 1;
	else
		enabled = 0;
	if (special)
		specialed = 1;
	else
		specialed = 0;

	uint32_t bits = convertBias(biasName, lowhi, cl, sex, enal, fineValue, coarseValue, specialed);

	return (bits);
}

bool caerInputDYNAPSEInit(caerModuleData moduleData, uint16_t deviceType) {

// USB port/bus/SN settings/restrictions.
// These can be used to force connection to one specific device at startup.
	sshsNodePutShortIfAbsent(moduleData->moduleNode, "busNumber", 0);
	sshsNodePutShortIfAbsent(moduleData->moduleNode, "devAddress", 0);
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "serialNumber", "");

// Add auto-restart setting.
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "autoRestart", true);

/// Start data acquisition, and correctly notify mainloop of new data and module of exceptional
// shutdown cases (device pulled, ...).
	char *serialNumber = sshsNodeGetString(moduleData->moduleNode, "serialNumber");

	caerInputDynapseState state = moduleData->moduleState;

	state->deviceState = caerDeviceOpen(1, CAER_DEVICE_DYNAPSE, 0, 0, NULL);
	state->eventSourceConfigNode = moduleData->moduleNode;

	free(serialNumber);

	if (state->deviceState == NULL) {
		// Failed to open device.
		return (false);
	}

// Let's take a look at the information we have on the device.
	struct caer_dynapse_info dynapse_info = caerDynapseInfoGet(state->deviceState);

	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "%s --- ID: %d, Master: %d,  Logic: %d,  ChipID: %d.\n",
		dynapse_info.deviceString, dynapse_info.deviceID, dynapse_info.deviceIsMaster, dynapse_info.logicVersion,
		dynapse_info.chipID);

	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");

	sshsNodePutLong(sourceInfoNode, "highestTimestamp", -1);
	sshsNodePutShort(sourceInfoNode, "logicVersion", dynapse_info.logicVersion);
	sshsNodePutBool(sourceInfoNode, "deviceIsMaster", dynapse_info.deviceIsMaster);
	sshsNodePutShort(sourceInfoNode, "deviceID", dynapse_info.deviceID);
	sshsNodePutShort(sourceInfoNode, "chipID", dynapse_info.chipID);

	// Put source information for generic visualization, to be used to display and debug filter information.
	sshsNodePutShort(sourceInfoNode, "dataSizeX", 64);
	sshsNodePutShort(sourceInfoNode, "dataSizeY", 64);

// Generate source string for output modules.
	size_t sourceStringLength = (size_t) snprintf(NULL, 0, "#Source %" PRIu16 ": %s\r\n", moduleData->moduleID,
		chipIDToName(DYNAPSE_CONFIG_DYNAPSE_U2, false));

	char sourceString[sourceStringLength + 1];
	snprintf(sourceString, sourceStringLength + 1, "#Source %" PRIu16 ": %s\r\n", moduleData->moduleID,
		chipIDToName(DYNAPSE_CONFIG_DYNAPSE_U2, false));
	sourceString[sourceStringLength] = '\0';

	sshsNodePutString(sourceInfoNode, "sourceString", sourceString);

// Generate sub-system string for module.
	size_t subSystemStringLength = (size_t) snprintf(NULL, 0, "%s[SN %s, %" PRIu8 ":%" PRIu8 "]",
		moduleData->moduleSubSystemString, dynapse_info.deviceSerialNumber, dynapse_info.deviceUSBBusNumber,
		dynapse_info.deviceUSBDeviceAddress);

	char subSystemString[subSystemStringLength + 1];
	snprintf(subSystemString, subSystemStringLength + 1, "%s[SN %s, %" PRIu8 ":%" PRIu8 "]",
		moduleData->moduleSubSystemString, dynapse_info.deviceSerialNumber, dynapse_info.deviceUSBBusNumber,
		dynapse_info.deviceUSBDeviceAddress);
	subSystemString[subSystemStringLength] = '\0';

	caerModuleSetSubSystemString(moduleData, subSystemString);

	// Let's turn on blocking data-get mode to avoid wasting resources.
	caerDeviceConfigSet(state->deviceState, CAER_HOST_CONFIG_DATAEXCHANGE,
	CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING,true);

	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_RUN, true);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_AER,DYNAPSE_CONFIG_AER_RUN, true);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_REQ_DELAY, 30);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_REQ_EXTENSION, 30);

	// Create default settings and send them to the devices.
	createDefaultConfiguration(moduleData, DYNAPSE_CONFIG_DYNAPSE_U0);
	createDefaultConfiguration(moduleData, DYNAPSE_CONFIG_DYNAPSE_U1);
	createDefaultConfiguration(moduleData, DYNAPSE_CONFIG_DYNAPSE_U2);
	createDefaultConfiguration(moduleData, DYNAPSE_CONFIG_DYNAPSE_U3);

	// Update silent biases
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U0);
	updateSilentBiases(moduleData,  DYNAPSE_CONFIG_DYNAPSE_U0);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U1);
	updateSilentBiases(moduleData, DYNAPSE_CONFIG_DYNAPSE_U1);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U2);
	updateSilentBiases(moduleData, DYNAPSE_CONFIG_DYNAPSE_U2);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U3);
	updateSilentBiases(moduleData,  DYNAPSE_CONFIG_DYNAPSE_U3);

	// Clear SRAM --> DYNAPSE_CONFIG_DYNAPSE_U0
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Clearing SRAM ...\n");
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Device number  %d...\n", DYNAPSE_CONFIG_DYNAPSE_U0);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U0);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_DEFAULT_SRAM_EMPTY,DYNAPSE_CONFIG_DYNAPSE_U0, 0);
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, " Done.\n");
	// Clear CAM -->  DYNAPSE_CONFIG_DYNAPSE_U0
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Clearing CAM ...\n");
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Device number  %d...\n", DYNAPSE_CONFIG_DYNAPSE_U0);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U0);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CLEAR_CAM, 0, 0);
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, " Done.\n");

	// Clear SRAM --> DYNAPSE_CONFIG_DYNAPSE_U1
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Clearing SRAM ...\n");
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Device number  %d...\n", DYNAPSE_CONFIG_DYNAPSE_U1);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U1);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_DEFAULT_SRAM_EMPTY,DYNAPSE_CONFIG_DYNAPSE_U1, 0);
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, " Done.\n");
	// Clear CAM -->  DYNAPSE_CONFIG_DYNAPSE_U1
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Clearing CAM ...\n");
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Device number  %d...\n", DYNAPSE_CONFIG_DYNAPSE_U1);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U1);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CLEAR_CAM, 0, 0);
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, " Done.\n");

	// Clear SRAM --> DYNAPSE_CONFIG_DYNAPSE_U2
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Clearing SRAM ...\n");
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Device number  %d...\n", DYNAPSE_CONFIG_DYNAPSE_U2);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U2);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_DEFAULT_SRAM_EMPTY,DYNAPSE_CONFIG_DYNAPSE_U2, 0);
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, " Done.\n");
	// Clear CAM -->  DYNAPSE_CONFIG_DYNAPSE_U2
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Clearing CAM ...\n");
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Device number  %d...\n", DYNAPSE_CONFIG_DYNAPSE_U2);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U2);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CLEAR_CAM, 0, 0);
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, " Done.\n");

	// Clear SRAM --> DYNAPSE_CONFIG_DYNAPSE_U3
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Clearing SRAM ...\n");
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Device number  %d...\n", DYNAPSE_CONFIG_DYNAPSE_U3);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U3);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_DEFAULT_SRAM_EMPTY,DYNAPSE_CONFIG_DYNAPSE_U3, 0);
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, " Done.\n");
	// Clear CAM -->  DYNAPSE_CONFIG_DYNAPSE_U3
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Clearing CAM ...\n");
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Device number  %d...\n", DYNAPSE_CONFIG_DYNAPSE_U3);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U3);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CLEAR_CAM, 0, 0);
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, " Done.\n");

	//  DYNAPSE_CONFIG_DYNAPSE_U0
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U0);
	updateLowPowerBiases(moduleData, DYNAPSE_CONFIG_DYNAPSE_U0);
	//  DYNAPSE_CONFIG_DYNAPSE_U1
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U1);
	updateLowPowerBiases(moduleData, DYNAPSE_CONFIG_DYNAPSE_U1);
	// DYNAPSE_CONFIG_DYNAPSE_U2
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U2);
	updateLowPowerBiases(moduleData, DYNAPSE_CONFIG_DYNAPSE_U2);
	// DYNAPSE_CONFIG_DYNAPSE_U3
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U3);
	updateLowPowerBiases(moduleData, DYNAPSE_CONFIG_DYNAPSE_U3);

	// Configure SRAM for Monitoring--> DYNAPSE_CONFIG_DYNAPSE_U0
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Default SRAM ...\n");
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Device number  %d...\n", DYNAPSE_CONFIG_DYNAPSE_U0);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U0);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_DEFAULT_SRAM,DYNAPSE_CONFIG_DYNAPSE_U0, 0);
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, " Done.\n");

	// Configure SRAM for Monitoring--> DYNAPSE_CONFIG_DYNAPSE_U1
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Default SRAM ...\n");
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Device number  %d...\n", DYNAPSE_CONFIG_DYNAPSE_U1);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U1);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_DEFAULT_SRAM,DYNAPSE_CONFIG_DYNAPSE_U1, 0);
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, " Done.\n");

	// Configure SRAM for Monitoring--> DYNAPSE_CONFIG_DYNAPSE_U2
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Default SRAM ...\n");
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Device number  %d...\n", DYNAPSE_CONFIG_DYNAPSE_U2);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U2);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_DEFAULT_SRAM,DYNAPSE_CONFIG_DYNAPSE_U2, 0);
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, " Done.\n");

	// Configure SRAM for Monitoring--> DYNAPSE_CONFIG_DYNAPSE_U3
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Default SRAM ...\n");
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, "Device number  %d...\n", DYNAPSE_CONFIG_DYNAPSE_U3);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U3);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_DEFAULT_SRAM,DYNAPSE_CONFIG_DYNAPSE_U3, 0);
	caerLog(CAER_LOG_NOTICE, moduleData->moduleSubSystemString, " Done.\n");

	// Device related configuration has its own sub-node DYNAPSEFX2
	sshsNode deviceConfigNode = sshsGetRelativeNode(moduleData->moduleNode, chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));

	sshsNode chipNode = sshsGetRelativeNode(deviceConfigNode, "chip/");
	sshsNodeAddAttributeListener(chipNode, moduleData, &chipConfigListener);

	sshsNode sysNode = sshsGetRelativeNode(deviceConfigNode, "system/");
	sshsNodeAddAttributeListener(sysNode, moduleData, &systemConfigListener);

	sshsNode usbNode = sshsGetRelativeNode(deviceConfigNode, "usb/");
	sshsNodeAddAttributeListener(usbNode, moduleData, &usbConfigListener);

	// Device related configuration has its own sub-node.
	//DYNAPSE_CONFIG_DYNAPSE_U0
	sshsNode deviceConfigNodeU0 = sshsGetRelativeNode(moduleData->moduleNode,
		chipIDToName(DYNAPSE_CONFIG_DYNAPSE_U0, true));

	sshsNode biasNodeU0 = sshsGetRelativeNode(deviceConfigNodeU0, "bias/");

	size_t biasNodesLength = 0;
	sshsNode *biasNodesU0 = sshsNodeGetChildren(biasNodeU0, &biasNodesLength);

	if (biasNodesU0 != NULL) {
		for (size_t i = 0; i < biasNodesLength; i++) {
			// Add listener for this particular bias.
			sshsNodeAddAttributeListener(biasNodesU0[i], moduleData, &biasConfigListener);
		}

		free(biasNodesU0);
	}

	// Device related configuration has its own sub-node.
	//DYNAPSE_CONFIG_DYNAPSE_U1
	sshsNode deviceConfigNodeU1 = sshsGetRelativeNode(moduleData->moduleNode,
		chipIDToName(DYNAPSE_CONFIG_DYNAPSE_U1, true));

	sshsNode biasNodeU1 = sshsGetRelativeNode(deviceConfigNodeU1, "bias/");

	biasNodesLength = 0;
	sshsNode *biasNodesU1 = sshsNodeGetChildren(biasNodeU1, &biasNodesLength);

	if (biasNodesU1 != NULL) {
		for (size_t i = 0; i < biasNodesLength; i++) {
			// Add listener for this particular bias.
			sshsNodeAddAttributeListener(biasNodesU1[i], moduleData, &biasConfigListener);
		}

		free(biasNodesU1);
	}

	//DYNAPSE_CONFIG_DYNAPSE_U2
	sshsNode deviceConfigNodeU2 = sshsGetRelativeNode(moduleData->moduleNode,
		chipIDToName(DYNAPSE_CONFIG_DYNAPSE_U2, true));

	sshsNode biasNodeU2 = sshsGetRelativeNode(deviceConfigNodeU2, "bias/");

	biasNodesLength = 0;
	sshsNode *biasNodesU2 = sshsNodeGetChildren(biasNodeU2, &biasNodesLength);

	if (biasNodesU2 != NULL) {
		for (size_t i = 0; i < biasNodesLength; i++) {
			// Add listener for this particular bias.
			sshsNodeAddAttributeListener(biasNodesU2[i], moduleData, &biasConfigListener);
		}
		free(biasNodesU2);
	}

	//DYNAPSE_CONFIG_DYNAPSE_U3
	sshsNode deviceConfigNodeU3 = sshsGetRelativeNode(moduleData->moduleNode,
		chipIDToName(DYNAPSE_CONFIG_DYNAPSE_U3, true));

	sshsNode biasNodeU3 = sshsGetRelativeNode(deviceConfigNodeU3, "bias/");

	biasNodesLength = 0;
	sshsNode *biasNodesU3 = sshsNodeGetChildren(biasNodeU3, &biasNodesLength);

	if (biasNodesU3 != NULL) {
		for (size_t i = 0; i < biasNodesLength; i++) {
			// Add listener for this particular bias.
			sshsNodeAddAttributeListener(biasNodesU3[i], moduleData, &biasConfigListener);
		}
		free(biasNodesU3);
	}

	//spike Generator Node
	sshsNode deviceConfigNodeMain = sshsGetRelativeNode(moduleData->moduleNode,
		chipIDToName(DYNAPSE_CHIP_DYNAPSE, true));

	sshsNode spikeNode = sshsGetRelativeNode(deviceConfigNodeMain, "spikeGen/");
	sshsNodeAddAttributeListener(spikeNode, state, &spikeConfigListener);
	caerGenSpikeInit(moduleData); // init module and start thread

	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,
	DYNAPSE_CONFIG_CHIP_RUN, false);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_AER,
	DYNAPSE_CONFIG_AER_RUN, false);
	sleep(1);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,
	DYNAPSE_CONFIG_CHIP_RUN, true);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_AER,
	DYNAPSE_CONFIG_AER_RUN, true);

	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,
	DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U0);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_MONITOR_NEU, 0, 0); // core 0 neuron 0
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_MONITOR_NEU, 1, 5); //  core 1 neuron 5
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_MONITOR_NEU, 2, 60); // core 2 neuron 10
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_MONITOR_NEU, 3, 105); // core 3 neuron 20

	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,
	DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U1);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_MONITOR_NEU, 0, 0); // core 0 neuron 0
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_MONITOR_NEU, 1, 5); //  core 1 neuron 5
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_MONITOR_NEU, 2, 60); // core 2 neuron 10
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_MONITOR_NEU, 3, 105); // core 3 neuron 20

	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,
	DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U2);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_MONITOR_NEU, 0, 0); // core 0 neuron 0
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_MONITOR_NEU, 1, 5); //  core 1 neuron 5
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_MONITOR_NEU, 2, 60); // core 2 neuron 10
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_MONITOR_NEU, 3, 105); // core 3 neuron 20

	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP,
	DYNAPSE_CONFIG_CHIP_ID, DYNAPSE_CONFIG_DYNAPSE_U3);
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_MONITOR_NEU, 0, 10); // core 0 neuron 0
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_MONITOR_NEU, 1, 5); //  core 1 neuron 5
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_MONITOR_NEU, 2, 60); // core 2 neuron 10
	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_MONITOR_NEU, 3, 105); // core 3 neuron 20

	// Start data acquisition.
	bool ret = caerDeviceDataStart(state->deviceState, &mainloopDataNotifyIncrease, &mainloopDataNotifyDecrease,
		caerMainloopGetReference(), &moduleShutdownNotify, moduleData->moduleNode);

	if (!ret) {
		// Failed to start data acquisition, close device and exit.
		caerDeviceClose((caerDeviceHandle *) &state->deviceState);

		return (false);
	}

	return (true);

}

void caerInputDYNAPSEExit(caerModuleData moduleData) {
// Device related configuration has its own sub-node.
	struct caer_dynapse_info devInfo = caerDynapseInfoGet(
		((caerInputDynapseState) moduleData->moduleState)->deviceState);
	sshsNode deviceConfigNode = sshsGetRelativeNode(moduleData->moduleNode,
		chipIDToName(DYNAPSE_CONFIG_DYNAPSE_U2, true));

	caerDeviceDataStop(((caerInputDynapseState) moduleData->moduleState)->deviceState);

	caerDeviceClose((caerDeviceHandle *) &moduleData->moduleState);

	if (sshsNodeGetBool(moduleData->moduleNode, "autoRestart")) {
		// Prime input module again so that it will try to restart if new devices detected.
		sshsNodePutBool(moduleData->moduleNode, "running", true);
	}
}

void caerInputDYNAPSERun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerEventPacketContainer *container = va_arg(args, caerEventPacketContainer *);

	*container = caerDeviceDataGet(((caerInputDynapseState) moduleData->moduleState)->deviceState);

	if (*container != NULL) {
		caerMainloopFreeAfterLoop((void (*)(void *)) &caerEventPacketContainerFree, *container);

		sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
		sshsNodePutLong(sourceInfoNode, "highestTimestamp",
			caerEventPacketContainerGetHighestEventTimestamp(*container));

		// Detect timestamp reset and call all reset functions for processors and outputs.
		caerEventPacketHeader special = caerEventPacketContainerGetEventPacket(*container, SPECIAL_EVENT);

		if ((special != NULL) && (caerEventPacketHeaderGetEventNumber(special) == 1)
			&& (caerSpecialEventPacketFindEventByType((caerSpecialEventPacket) special, TIMESTAMP_RESET) != NULL)) {
			caerMainloopResetProcessors(moduleData->moduleID);
			caerMainloopResetOutputs(moduleData->moduleID);

			// Update master/slave information.
			struct caer_dynapse_info devInfo = caerDynapseInfoGet(
				((caerInputDynapseState) moduleData->moduleState)->deviceState);
			sshsNodePutBool(sourceInfoNode, "deviceIsMaster", devInfo.deviceIsMaster);
		}
	}
}

bool caerDynapseSetBiasBits(void* spikeGenState, uint32_t chipId, uint32_t coreId, const char *biasName_t,
	uint8_t coarseValue, uint16_t fineValue, const char *lowHigh, const char *npBias) {

	caerInputDynapseState state = spikeGenState;
	caerDeviceHandle usb_handle = (caerDeviceHandle) state->deviceState;
	struct caer_dynapse_info dynapse_info = caerDynapseInfoGet(state->deviceState);

	// Check if the pointer is valid.
	if (state->deviceState == NULL) {
		struct caer_dynapse_info emptyInfo = { 0, .deviceString = NULL };
		return (false);
	}

	size_t biasNameLength = strlen(biasName_t);
	char biasName[biasNameLength + 3];

	biasName[0] = 'C';
	if (coreId == 0)
		biasName[1] = '0';
	else if (coreId == 1)
		biasName[1] = '1';
	else if (coreId == 2)
		biasName[1] = '2';
	else if (coreId == 3)
		biasName[1] = '3';
	biasName[2] = '_';

	uint32_t i;
	for (i = 0; i < biasNameLength + 3; i++) {
		biasName[3 + i] = biasName_t[i];
	}

	uint32_t bits = generatesBitsCoarseFineBiasSetting(state->eventSourceConfigNode, biasName,
		coarseValue, fineValue, lowHigh, "Normal", npBias, true, chipId);

	caerDeviceConfigSet(state->deviceState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, bits);

	return (true);
}
