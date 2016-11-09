#include "dynapse_common.h"

static uint32_t convertBias(struct caer_bias_dynapse coarseFineBias, const char *biasName);
static uint32_t generateCoarseFineBiasParent(sshsNode biasNode, const char *biasName);

static void biasConfigListener(sshsNode node, void *userData,
		enum sshs_node_attribute_events event, const char *changeKey,
		enum sshs_node_attr_value_type changeType,
		union sshs_node_attr_value changeValue);

static inline const char *chipIDToName(int16_t chipID, bool withEndSlash) {
	switch (chipID) {
	case 64:
		return ((withEndSlash) ? ("DYNAPSEFX2/") : ("DYNAPSEFX2"));
		break;
	}

	return ((withEndSlash) ? ("Unknown/") : ("Unknown"));
}

static void mainloopDataNotifyIncrease(void *p) {
	caerMainloopData mainloopData = p;

	atomic_fetch_add_explicit(&mainloopData->dataAvailable, 1,
			memory_order_release);
}

static void mainloopDataNotifyDecrease(void *p) {
	caerMainloopData mainloopData = p;

	// No special memory order for decrease, because the acquire load to even start running
	// through a mainloop already synchronizes with the release store above.
	atomic_fetch_sub_explicit(&mainloopData->dataAvailable, 1,
			memory_order_relaxed);
}

static void moduleShutdownNotify(void *p) {
	sshsNode moduleNode = p;

	// Ensure parent also shuts down (on disconnected device for example).
	sshsNodePutBool(moduleNode, "running", false);
}

static void createSpecialBiasSetting(sshsNode biasNode, const char *biasName,
		uint8_t coarseValue, uint8_t fineValue) {

	// Add trailing slash to node name (required!).
	size_t biasNameLength = strlen(biasName);
	char biasNameFull[biasNameLength + 2];
	memcpy(biasNameFull, biasName, biasNameLength);
	biasNameFull[biasNameLength] = '/';
	biasNameFull[biasNameLength + 1] = '\0';

	// Create configuration node for this particular bias.
	sshsNode biasConfigNode = sshsGetRelativeNode(biasNode, biasNameFull);

	// Add bias settings.
	sshsNodePutShortIfAbsent(biasConfigNode, "coarseValue", I8T(coarseValue));
	sshsNodePutShortIfAbsent(biasConfigNode, "fineValue", I16T(fineValue));
	sshsNodePutBoolIfAbsent(biasConfigNode, "special", true);

}

static void createCoarseFineBiasSetting(sshsNode biasNode, const char *biasName,
		uint8_t coarseValue, uint8_t fineValue, const char *hlbias,
		const char *currentLevel, const char *sex, bool enabled) {
	// Add trailing slash to node name (required!).
	size_t biasNameLength = strlen(biasName);
	char biasNameFull[biasNameLength + 2];
	memcpy(biasNameFull, biasName, biasNameLength);
	biasNameFull[biasNameLength] = '/';
	biasNameFull[biasNameLength + 1] = '\0';

	// Create configuration node for this particular bias.
	sshsNode biasConfigNode = sshsGetRelativeNode(biasNode, biasNameFull);

	// Add bias settings.
	sshsNodePutShortIfAbsent(biasConfigNode, "coarseValue", I8T(coarseValue));
	sshsNodePutShortIfAbsent(biasConfigNode, "fineValue", I16T(fineValue));
	sshsNodePutStringIfAbsent(biasConfigNode, "BiasLowHi", hlbias);
	sshsNodePutStringIfAbsent(biasConfigNode, "currentLevel", currentLevel);
	sshsNodePutStringIfAbsent(biasConfigNode, "sex", sex);
	sshsNodePutBoolIfAbsent(biasConfigNode, "enabled", enabled);
	sshsNodePutBoolIfAbsent(biasConfigNode, "special", false);

}

static void biasConfigListener(sshsNode node, void *userData,
		enum sshs_node_attribute_events event, const char *changeKey,
		enum sshs_node_attr_value_type changeType,
		union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(changeKey);
	UNUSED_ARGUMENT(changeType);
	UNUSED_ARGUMENT(changeValue);

	caerModuleData moduleData = userData;
	struct caer_dynapse_info devInfo = caerDynapseInfoGet(moduleData->moduleState);

	if (event == SSHS_ATTRIBUTE_MODIFIED) {
		const char *nodeName = sshsNodeGetName(node);

		if (caerStrEquals(nodeName, "C0_IF_BUF_P")) {

		}
	}

}

static void createDefaultConfiguration(caerModuleData moduleData,
		struct caer_dynapse_info *devInfo) {
// Device related configuration has its own sub-node.
	sshsNode deviceConfigNode = sshsGetRelativeNode(moduleData->moduleNode,
			chipIDToName(devInfo->chipID, true));

// Chip biases, based on testing defaults.
	sshsNode biasNode = sshsGetRelativeNode(deviceConfigNode, "bias/");

	createCoarseFineBiasSetting(biasNode, "C0_IF_BUF_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_IF_RFR_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_IF_NMDA_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_IF_DC_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_IF_TAU1_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_IF_TAU2_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_IF_THR_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_IF_AHW_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_IF_AHTAU_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_IF_AHTHR_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_IF_CASC_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_PULSE_PWLK_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_PS_WEIGHT_INH_S_N", 7, 0,
			"HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_PS_WEIGHT_INH_F_N", 7, 0,
			"HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_PS_WEIGHT_EXC_S_N", 7, 0,
			"HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_PS_WEIGHT_EXC_F_N", 7, 0,
			"HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_NPDPII_TAU_S_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_NPDPII_THR_S_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_NPDPII_TAU_F_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_NPDPII_THR_F_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_NPDPIE_TAU_S_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_NPDPIE_THR_S_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_NPDPIE_TAU_F_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_NPDPIE_THR_F_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C0_R2R_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_IF_BUF_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_IF_RFR_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_IF_NMDA_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_IF_DC_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_IF_TAU1_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_IF_TAU2_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_IF_THR_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_IF_AHW_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_IF_AHTAU_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_IF_AHTHR_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_IF_CASC_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_PULSE_PWLK_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_PS_WEIGHT_INH_S_N", 7, 0,
			"HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_PS_WEIGHT_INH_F_N", 7, 0,
			"HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_PS_WEIGHT_EXC_S_N", 7, 0,
			"HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_PS_WEIGHT_EXC_F_N", 7, 0,
			"HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_NPDPII_TAU_S_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_NPDPII_THR_S_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_NPDPII_TAU_F_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_NPDPII_THR_F_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_NPDPIE_TAU_S_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_NPDPIE_THR_S_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_NPDPIE_TAU_F_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_NPDPIE_THR_F_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C1_R2R_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_IF_BUF_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_IF_RFR_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_IF_NMDA_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_IF_DC_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_IF_TAU1_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_IF_TAU2_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_IF_THR_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_IF_AHW_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_IF_AHTAU_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_IF_AHTHR_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_IF_CASC_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_PULSE_PWLK_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_PS_WEIGHT_INH_S_N", 7, 0,
			"HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_PS_WEIGHT_INH_F_N", 7, 0,
			"HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_PS_WEIGHT_EXC_S_N", 7, 0,
			"HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_PS_WEIGHT_EXC_F_N", 7, 0,
			"HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_NPDPII_TAU_S_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_NPDPII_THR_S_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_NPDPII_TAU_F_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_NPDPII_THR_F_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_NPDPIE_TAU_S_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_NPDPIE_THR_S_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_NPDPIE_TAU_F_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_NPDPIE_THR_F_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C2_R2R_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_IF_BUF_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_IF_RFR_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_IF_NMDA_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_IF_DC_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_IF_TAU1_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_IF_TAU2_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_IF_THR_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_IF_AHW_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_IF_AHTAU_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_IF_AHTHR_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_IF_CASC_N", 7, 0, "HighBias",
			"Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_PULSE_PWLK_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_PS_WEIGHT_INH_S_N", 7, 0,
			"HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_PS_WEIGHT_INH_F_N", 7, 0,
			"HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_PS_WEIGHT_EXC_S_N", 7, 0,
			"HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_PS_WEIGHT_EXC_F_N", 7, 0,
			"HighBias", "Normal", "NBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_NPDPII_TAU_S_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_NPDPII_THR_S_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_NPDPII_TAU_F_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_NPDPII_THR_F_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_NPDPIE_TAU_S_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_NPDPIE_THR_S_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_NPDPIE_TAU_F_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_NPDPIE_THR_F_P", 7, 0, "HighBias",
			"Normal", "PBias", true);
	createCoarseFineBiasSetting(biasNode, "C3_R2R_P", 7, 0, "HighBias",
			"Normal", "PBias", true);

	/*special biases*/
	createSpecialBiasSetting(biasNode, "U_Buffer", 1, 80);
	createSpecialBiasSetting(biasNode, "U_SSP", 0, 7);
	createSpecialBiasSetting(biasNode, "U_SSN", 0, 15);

	createSpecialBiasSetting(biasNode, "D_Buffer", 1, 80);
	createSpecialBiasSetting(biasNode, "D_SSP", 0, 7);
	createSpecialBiasSetting(biasNode, "D_SSN", 0, 15);

}

static uint32_t generateCoarseFineBias(sshsNode biasNode, const char *biasName) {
// Build up bias value from all its components.
	char *BiasLowHiStr = sshsNodeGetString(biasNode, "BiasLowHi");
	char *currentLevelStr = sshsNodeGetString(biasNode, "currentLevel");
	char *sexStr = sshsNodeGetString(biasNode, "sex");

	struct caer_bias_dynapse biasValue = { .coarseValue = U8T(
			sshsNodeGetShort(biasNode, "coarseValue")), .fineValue = U8T(
			sshsNodeGetShort(biasNode, "fineValue")), .BiasLowHi =
			caerStrEquals(BiasLowHiStr, "HighBias"), .currentLevel =
			caerStrEquals(currentLevelStr, "Normal"), .sex = caerStrEquals(
			sexStr, "NBias"), .enabled = U8T(
			sshsNodeGetBool(biasNode, "enabled")), };

// Free strings to avoid memory leaks.
	free(BiasLowHiStr);
	free(currentLevelStr);
	free(sexStr);

	return (convertBias(biasValue,biasName));
}

static uint32_t generateCoarseFineBiasParent(sshsNode biasNode, const char *biasName) {
// Add trailing slash to node name (required!).
	size_t biasNameLength = strlen(biasName);
	char biasNameFull[biasNameLength + 2];
	memcpy(biasNameFull, biasName, biasNameLength);
	biasNameFull[biasNameLength] = '/';
	biasNameFull[biasNameLength + 1] = '\0';

	// Get bias configuration node.
	sshsNode biasConfigNode = sshsGetRelativeNode(biasNode, biasNameFull);

	/*Here always get wrong node.... */
	caerLog(CAER_LOG_CRITICAL, "BIAS CONFIGURE ", " biasNameFull %s \n", biasNameFull);

	return (generateCoarseFineBias(biasConfigNode, biasNameFull));
}

static uint32_t convertBias(struct caer_bias_dynapse coarseFineBias, const char *biasName){

		int32_t confbits;
		int32_t addr = 0;
		int32_t inbits = 0;

		/*start names*/
		if(strcpy(biasName, "C0_PULSE_PWLK_P")){
			addr = DYNAPSE_CONFIG_BIAS_C0_PULSE_PWLK_P;
		}
		if(strcpy(biasName, "C0_PS_WEIGHT_INH_S_N")){
			addr = DYNAPSE_CONFIG_BIAS_C0_PS_WEIGHT_INH_S_N;
		}
		if(strcpy(biasName, "C0_PS_WEIGHT_INH_F_N")){
			addr = DYNAPSE_CONFIG_BIAS_C0_PS_WEIGHT_INH_F_N;
		}
		if(strcpy(biasName, "C0_PS_WEIGHT_EXC_S_N")){
			addr = DYNAPSE_CONFIG_BIAS_C0_PS_WEIGHT_EXC_S_N;
		}
		if(strcpy(biasName, "C0_PS_WEIGHT_EXC_F_N")){
			addr = DYNAPSE_CONFIG_BIAS_C0_PS_WEIGHT_EXC_F_N;
		}
		if(strcpy(biasName, "C0_IF_RFR_N")){
			addr = DYNAPSE_CONFIG_BIAS_C0_IF_RFR_N;
		}
		if(strcpy(biasName, "C0_IF_TAU1_N")){
			addr = DYNAPSE_CONFIG_BIAS_C0_IF_TAU1_N;
		}
		if(strcpy(biasName, "C0_IF_AHTAU_N")){
			addr = DYNAPSE_CONFIG_BIAS_C0_IF_AHTAU_N;
		}
		if(strcpy(biasName, "C0_IF_CASC_N")){
			addr = DYNAPSE_CONFIG_BIAS_C0_IF_CASC_N;
		}
		if(strcpy(biasName, "C0_IF_TAU2_N")){
			addr = DYNAPSE_CONFIG_BIAS_C0_IF_TAU2_N;
		}
		if(strcpy(biasName, "C0_IF_BUF_P")){
			addr = DYNAPSE_CONFIG_BIAS_C0_IF_BUF_P;
		}
		if(strcpy(biasName, "C0_IF_AHTHR_N")){
			addr = DYNAPSE_CONFIG_BIAS_C0_IF_AHTHR_N;
		}
		if(strcpy(biasName, "C0_IF_THR_N")){
			addr = DYNAPSE_CONFIG_BIAS_C0_IF_THR_N;
		}
		if(strcpy(biasName, "C0_NPDPIE_THR_S_P")){
			addr = DYNAPSE_CONFIG_BIAS_C0_NPDPIE_THR_S_P;
		}
		if(strcpy(biasName, "C0_NPDPIE_THR_F_P")){
			addr = DYNAPSE_CONFIG_BIAS_C0_NPDPIE_THR_F_P;
		}
		if(strcpy(biasName, "C0_NPDPII_THR_F_P")){
			addr = DYNAPSE_CONFIG_BIAS_C0_NPDPII_THR_F_P;
		}
		if(strcpy(biasName, "C0_NPDPII_THR_S_P")){
			addr = DYNAPSE_CONFIG_BIAS_C0_NPDPII_THR_S_P;
		}
		if(strcpy(biasName, "C0_IF_NMDA_N")){
			addr = DYNAPSE_CONFIG_BIAS_C0_IF_NMDA_N;
		}
		if(strcpy(biasName, "C0_IF_DC_P")){
			addr = DYNAPSE_CONFIG_BIAS_C0_IF_DC_P;
		}
		if(strcpy(biasName, "C0_IF_AHW_P")){
			addr = DYNAPSE_CONFIG_BIAS_C0_IF_AHW_P;
		}
		if(strcpy(biasName, "C0_NPDPII_TAU_S_P")){
			addr = DYNAPSE_CONFIG_BIAS_C0_NPDPII_TAU_S_P;
		}
		if(strcpy(biasName, "C0_NPDPII_TAU_F_P")){
			addr = DYNAPSE_CONFIG_BIAS_C0_NPDPII_TAU_F_P;
		}
		if(strcpy(biasName, "C0_NPDPIE_TAU_F_P")){
			addr = DYNAPSE_CONFIG_BIAS_C0_NPDPIE_TAU_F_P;
		}
		if(strcpy(biasName, "C0_NPDPIE_TAU_S_P")){
			addr = DYNAPSE_CONFIG_BIAS_C0_NPDPIE_TAU_S_P;
		}
		if(strcpy(biasName, "C0_R2R_P")){
			addr = DYNAPSE_CONFIG_BIAS_C0_R2R_P;
		}


		if(strcpy(biasName, "C1_PULSE_PWLK_P")){
			addr = DYNAPSE_CONFIG_BIAS_C1_PULSE_PWLK_P;
		}
		if(strcpy(biasName, "C1_PS_WEIGHT_INH_S_N")){
			addr = DYNAPSE_CONFIG_BIAS_C1_PS_WEIGHT_INH_S_N;
		}
		if(strcpy(biasName, "C1_PS_WEIGHT_INH_F_N")){
			addr = DYNAPSE_CONFIG_BIAS_C1_PS_WEIGHT_INH_F_N;
		}
		if(strcpy(biasName, "C1_PS_WEIGHT_EXC_S_N")){
			addr = DYNAPSE_CONFIG_BIAS_C1_PS_WEIGHT_EXC_S_N;
		}
		if(strcpy(biasName, "C1_PS_WEIGHT_EXC_F_N")){
			addr = DYNAPSE_CONFIG_BIAS_C1_PS_WEIGHT_EXC_F_N;
		}
		if(strcpy(biasName, "C1_IF_RFR_N")){
			addr = DYNAPSE_CONFIG_BIAS_C1_IF_RFR_N;
		}
		if(strcpy(biasName, "C1_IF_TAU1_N")){
			addr = DYNAPSE_CONFIG_BIAS_C1_IF_TAU1_N;
		}
		if(strcpy(biasName, "C1_IF_AHTAU_N")){
			addr = DYNAPSE_CONFIG_BIAS_C1_IF_AHTAU_N;
		}
		if(strcpy(biasName, "C1_IF_CASC_N")){
			addr = DYNAPSE_CONFIG_BIAS_C1_IF_CASC_N;
		}
		if(strcpy(biasName, "C1_IF_TAU2_N")){
			addr = DYNAPSE_CONFIG_BIAS_C1_IF_TAU2_N;
		}
		if(strcpy(biasName, "C1_IF_BUF_P")){
			addr = DYNAPSE_CONFIG_BIAS_C1_IF_BUF_P;
		}
		if(strcpy(biasName, "C1_IF_AHTHR_N")){
			addr = DYNAPSE_CONFIG_BIAS_C1_IF_AHTHR_N;
		}
		if(strcpy(biasName, "C1_IF_THR_N")){
			addr = DYNAPSE_CONFIG_BIAS_C1_IF_THR_N;
		}
		if(strcpy(biasName, "C1_NPDPIE_THR_S_P")){
			addr = DYNAPSE_CONFIG_BIAS_C1_NPDPIE_THR_S_P;
		}
		if(strcpy(biasName, "C1_NPDPIE_THR_F_P")){
			addr = DYNAPSE_CONFIG_BIAS_C1_NPDPIE_THR_F_P;
		}
		if(strcpy(biasName, "C1_NPDPII_THR_F_P")){
			addr = DYNAPSE_CONFIG_BIAS_C1_NPDPII_THR_F_P;
		}
		if(strcpy(biasName, "C1_NPDPII_THR_S_P")){
			addr = DYNAPSE_CONFIG_BIAS_C1_NPDPII_THR_S_P;
		}
		if(strcpy(biasName, "C1_IF_NMDA_N")){
			addr = DYNAPSE_CONFIG_BIAS_C1_IF_NMDA_N;
		}
		if(strcpy(biasName, "C1_IF_DC_P")){
			addr = DYNAPSE_CONFIG_BIAS_C1_IF_DC_P;
		}
		if(strcpy(biasName, "C1_IF_AHW_P")){
			addr = DYNAPSE_CONFIG_BIAS_C1_IF_AHW_P;
		}
		if(strcpy(biasName, "C1_NPDPII_TAU_S_P")){
			addr = DYNAPSE_CONFIG_BIAS_C1_NPDPII_TAU_S_P;
		}
		if(strcpy(biasName, "C1_NPDPII_TAU_F_P")){
			addr = DYNAPSE_CONFIG_BIAS_C1_NPDPII_TAU_F_P;
		}
		if(strcpy(biasName, "C1_NPDPIE_TAU_F_P")){
			addr = DYNAPSE_CONFIG_BIAS_C1_NPDPIE_TAU_F_P;
		}
		if(strcpy(biasName, "C1_NPDPIE_TAU_S_P")){
			addr = DYNAPSE_CONFIG_BIAS_C1_NPDPIE_TAU_S_P;
		}
		if(strcpy(biasName, "C1_R2R_P")){
			addr = DYNAPSE_CONFIG_BIAS_C1_R2R_P;
		}

		if(strcpy(biasName, "C2_PULSE_PWLK_P")){
			addr = DYNAPSE_CONFIG_BIAS_C2_PULSE_PWLK_P;
		}
		if(strcpy(biasName, "C2_PS_WEIGHT_INH_S_N")){
			addr = DYNAPSE_CONFIG_BIAS_C2_PS_WEIGHT_INH_S_N;
		}
		if(strcpy(biasName, "C2_PS_WEIGHT_INH_F_N")){
			addr = DYNAPSE_CONFIG_BIAS_C2_PS_WEIGHT_INH_F_N;
		}
		if(strcpy(biasName, "C2_PS_WEIGHT_EXC_S_N")){
			addr = DYNAPSE_CONFIG_BIAS_C2_PS_WEIGHT_EXC_S_N;
		}
		if(strcpy(biasName, "C2_PS_WEIGHT_EXC_F_N")){
			addr = DYNAPSE_CONFIG_BIAS_C2_PS_WEIGHT_EXC_F_N;
		}
		if(strcpy(biasName, "C2_IF_RFR_N")){
			addr = DYNAPSE_CONFIG_BIAS_C2_IF_RFR_N;
		}
		if(strcpy(biasName, "C2_IF_TAU1_N")){
			addr = DYNAPSE_CONFIG_BIAS_C2_IF_TAU1_N;
		}
		if(strcpy(biasName, "C2_IF_AHTAU_N")){
			addr = DYNAPSE_CONFIG_BIAS_C2_IF_AHTAU_N;
		}
		if(strcpy(biasName, "C2_IF_CASC_N")){
			addr = DYNAPSE_CONFIG_BIAS_C2_IF_CASC_N;
		}
		if(strcpy(biasName, "C2_IF_TAU2_N")){
			addr = DYNAPSE_CONFIG_BIAS_C2_IF_TAU2_N;
		}
		if(strcpy(biasName, "C2_IF_BUF_P")){
			addr = DYNAPSE_CONFIG_BIAS_C2_IF_BUF_P;
		}
		if(strcpy(biasName, "C2_IF_AHTHR_N")){
			addr = DYNAPSE_CONFIG_BIAS_C2_IF_AHTHR_N;
		}
		if(strcpy(biasName, "C2_IF_THR_N")){
			addr = DYNAPSE_CONFIG_BIAS_C2_IF_THR_N;
		}
		if(strcpy(biasName, "C2_NPDPIE_THR_S_P")){
			addr = DYNAPSE_CONFIG_BIAS_C2_NPDPIE_THR_S_P;
		}
		if(strcpy(biasName, "C2_NPDPIE_THR_F_P")){
			addr = DYNAPSE_CONFIG_BIAS_C2_NPDPIE_THR_F_P;
		}
		if(strcpy(biasName, "C2_NPDPII_THR_F_P")){
			addr = DYNAPSE_CONFIG_BIAS_C2_NPDPII_THR_F_P;
		}
		if(strcpy(biasName, "C2_NPDPII_THR_S_P")){
			addr = DYNAPSE_CONFIG_BIAS_C2_NPDPII_THR_S_P;
		}
		if(strcpy(biasName, "C2_IF_NMDA_N")){
			addr = DYNAPSE_CONFIG_BIAS_C2_IF_NMDA_N;
		}
		if(strcpy(biasName, "C2_IF_DC_P")){
			addr = DYNAPSE_CONFIG_BIAS_C2_IF_DC_P;
		}
		if(strcpy(biasName, "C2_IF_AHW_P")){
			addr = DYNAPSE_CONFIG_BIAS_C2_IF_AHW_P;
		}
		if(strcpy(biasName, "C2_NPDPII_TAU_S_P")){
			addr = DYNAPSE_CONFIG_BIAS_C2_NPDPII_TAU_S_P;
		}
		if(strcpy(biasName, "C2_NPDPII_TAU_F_P")){
			addr = DYNAPSE_CONFIG_BIAS_C2_NPDPII_TAU_F_P;
		}
		if(strcpy(biasName, "C2_NPDPIE_TAU_F_P")){
			addr = DYNAPSE_CONFIG_BIAS_C2_NPDPIE_TAU_F_P;
		}
		if(strcpy(biasName, "C2_NPDPIE_TAU_S_P")){
			addr = DYNAPSE_CONFIG_BIAS_C2_NPDPIE_TAU_S_P;
		}
		if(strcpy(biasName, "C2_R2R_P")){
			addr = DYNAPSE_CONFIG_BIAS_C2_R2R_P;
		}

		if(strcpy(biasName, "C3_PULSE_PWLK_P")){
			addr = DYNAPSE_CONFIG_BIAS_C3_PULSE_PWLK_P;
		}
		if(strcpy(biasName, "C3_PS_WEIGHT_INH_S_N")){
			addr = DYNAPSE_CONFIG_BIAS_C3_PS_WEIGHT_INH_S_N;
		}
		if(strcpy(biasName, "C3_PS_WEIGHT_INH_F_N")){
			addr = DYNAPSE_CONFIG_BIAS_C3_PS_WEIGHT_INH_F_N;
		}
		if(strcpy(biasName, "C3_PS_WEIGHT_EXC_S_N")){
			addr = DYNAPSE_CONFIG_BIAS_C3_PS_WEIGHT_EXC_S_N;
		}
		if(strcpy(biasName, "C3_PS_WEIGHT_EXC_F_N")){
			addr = DYNAPSE_CONFIG_BIAS_C3_PS_WEIGHT_EXC_F_N;
		}
		if(strcpy(biasName, "C3_IF_RFR_N")){
			addr = DYNAPSE_CONFIG_BIAS_C3_IF_RFR_N;
		}
		if(strcpy(biasName, "C3_IF_TAU1_N")){
			addr = DYNAPSE_CONFIG_BIAS_C3_IF_TAU1_N;
		}
		if(strcpy(biasName, "C3_IF_AHTAU_N")){
			addr = DYNAPSE_CONFIG_BIAS_C3_IF_AHTAU_N;
		}
		if(strcpy(biasName, "C3_IF_CASC_N")){
			addr = DYNAPSE_CONFIG_BIAS_C3_IF_CASC_N;
		}
		if(strcpy(biasName, "C3_IF_TAU2_N")){
			addr = DYNAPSE_CONFIG_BIAS_C3_IF_TAU2_N;
		}
		if(strcpy(biasName, "C3_IF_BUF_P")){
			addr = DYNAPSE_CONFIG_BIAS_C3_IF_BUF_P;
		}
		if(strcpy(biasName, "C3_IF_AHTHR_N")){
			addr = DYNAPSE_CONFIG_BIAS_C3_IF_AHTHR_N;
		}
		if(strcpy(biasName, "C3_IF_THR_N")){
			addr = DYNAPSE_CONFIG_BIAS_C3_IF_THR_N;
		}
		if(strcpy(biasName, "C3_NPDPIE_THR_S_P")){
			addr = DYNAPSE_CONFIG_BIAS_C3_NPDPIE_THR_S_P;
		}
		if(strcpy(biasName, "C3_NPDPIE_THR_F_P")){
			addr = DYNAPSE_CONFIG_BIAS_C3_NPDPIE_THR_F_P;
		}
		if(strcpy(biasName, "C3_NPDPII_THR_F_P")){
			addr = DYNAPSE_CONFIG_BIAS_C3_NPDPII_THR_F_P;
		}
		if(strcpy(biasName, "C3_NPDPII_THR_S_P")){
			addr = DYNAPSE_CONFIG_BIAS_C3_NPDPII_THR_S_P;
		}
		if(strcpy(biasName, "C3_IF_NMDA_N")){
			addr = DYNAPSE_CONFIG_BIAS_C3_IF_NMDA_N;
		}
		if(strcpy(biasName, "C3_IF_DC_P")){
			addr = DYNAPSE_CONFIG_BIAS_C3_IF_DC_P;
		}
		if(strcpy(biasName, "C3_IF_AHW_P")){
			addr = DYNAPSE_CONFIG_BIAS_C3_IF_AHW_P;
		}
		if(strcpy(biasName, "C3_NPDPII_TAU_S_P")){
			addr = DYNAPSE_CONFIG_BIAS_C3_NPDPII_TAU_S_P;
		}
		if(strcpy(biasName, "C3_NPDPII_TAU_F_P")){
			addr = DYNAPSE_CONFIG_BIAS_C3_NPDPII_TAU_F_P;
		}
		if(strcpy(biasName, "C3_NPDPIE_TAU_F_P")){
			addr = DYNAPSE_CONFIG_BIAS_C3_NPDPIE_TAU_F_P;
		}
		if(strcpy(biasName, "C3_NPDPIE_TAU_S_P")){
			addr = DYNAPSE_CONFIG_BIAS_C3_NPDPIE_TAU_S_P;
		}
		if(strcpy(biasName, "C3_R2R_P")){
			addr = DYNAPSE_CONFIG_BIAS_C3_R2R_P;
		}


		if(strcpy(biasName, "U_BUFFER")){
			addr = DYNAPSE_CONFIG_BIAS_U_BUFFER;
		}
		if(strcpy(biasName, "U_SSP")){
			addr = DYNAPSE_CONFIG_BIAS_U_SSP;
		}
		if(strcpy(biasName, "U_SSN")){
			addr = DYNAPSE_CONFIG_BIAS_U_SSN;
		}
		if(strcpy(biasName, "D_BUFFER")){
			addr = DYNAPSE_CONFIG_BIAS_D_BUFFER;
		}
		if(strcpy(biasName, "D_SSP")){
			addr = DYNAPSE_CONFIG_BIAS_D_SSP;
		}
		if(strcpy(biasName, "D_SSN")){
			addr = DYNAPSE_CONFIG_BIAS_D_SSN;
		}

		caerLog(CAER_LOG_CRITICAL, "BIAS CONFIGURE ", " biasName %s --> ADDR %d\n", biasName, addr);

		/*end names*/

		confbits = coarseFineBias.BiasLowHi << 3 | coarseFineBias.currentLevel << 2 | coarseFineBias.sex << 1 | coarseFineBias.enabled;

		// snn and ssp
		if(addr == 51 || addr == 52 || addr == 115 || addr == 116){
			inbits = addr << 18 | 1 << 16 | 63 << 10 | coarseFineBias.fineValue << 4 | confbits;
		}else{
			inbits = addr << 18 | 1 << 16 | coarseFineBias.special << 15 | coarseFineBias.coarseValue << 12 | coarseFineBias.fineValue << 4 | confbits;
		}

		caerLog(CAER_LOG_CRITICAL, "BIAS CONFIGURE ", "coarseFineBias.fineValue %d , "
									"coarseFineBias.currentLevel %d, "
									"coarseFineBias.coarseValue %d ,  "
									"coarseFineBias.special %d --> ADDR %d\n",
									coarseFineBias.fineValue, coarseFineBias.currentLevel,
									coarseFineBias.coarseValue,  coarseFineBias.special, addr);

		return inbits;

}

static void biasConfigSend(sshsNode node, caerModuleData moduleData,
		struct caer_dynapse_info *devInfo) {

	uint32_t value;

  //  10100010111000000001101
  //1110100011111110000001101



	value = generateCoarseFineBiasParent(node, "C0_IF_BUF_P");
	caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString, "BIAS C0_IF_BUF_P VALUE ... %d\n", value);
	caerDeviceConfigSet(moduleData->moduleState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, value);

	value = generateCoarseFineBiasParent(node, "C0_IF_RFR_N");
	caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString, "BIAS C0_IF_RFR_N VALUE ... %d\n", value);
	caerDeviceConfigSet(moduleData->moduleState, DYNAPSE_CONFIG_CHIP, DYNAPSE_CONFIG_CHIP_CONTENT, value);

/*#define DYNAPSE_CONFIG_BIAS_C0_PULSE_PWLK_P             	0
#define DYNAPSE_CONFIG_BIAS_C0_PS_WEIGHT_INH_S_N            2
#define DYNAPSE_CONFIG_BIAS_C0_PS_WEIGHT_INH_F_N            4
#define DYNAPSE_CONFIG_BIAS_C0_PS_WEIGHT_EXC_S_N         	6
#define DYNAPSE_CONFIG_BIAS_C0_PS_WEIGHT_EXC_F_N        	8
#define DYNAPSE_CONFIG_BIAS_C0_IF_RFR_N          			10
#define DYNAPSE_CONFIG_BIAS_C0_IF_TAU1_N         			12
#define DYNAPSE_CONFIG_BIAS_C0_IF_AHTAU_N           		14
#define DYNAPSE_CONFIG_BIAS_C0_IF_CASC_N 					16
#define DYNAPSE_CONFIG_BIAS_C0_IF_TAU2_N         			18
#define DYNAPSE_CONFIG_BIAS_C0_IF_BUF_P               		20
#define DYNAPSE_CONFIG_BIAS_C0_IF_AHTHR_N             		22
#define DYNAPSE_CONFIG_BIAS_C0_IF_THR_N            			24
#define DYNAPSE_CONFIG_BIAS_C0_NPDPIE_THR_S_P             	26
#define DYNAPSE_CONFIG_BIAS_C0_NPDPIE_THR_F_P            	38
#define DYNAPSE_CONFIG_BIAS_C0_NPDPII_THR_F_P      			30
#define DYNAPSE_CONFIG_BIAS_C0_NPDPII_THR_S_P            	32
#define DYNAPSE_CONFIG_BIAS_C0_IF_NMDA_N            		34
#define DYNAPSE_CONFIG_BIAS_C0_IF_DC_P           			36
#define DYNAPSE_CONFIG_BIAS_C0_IF_AHW_P          			38
#define DYNAPSE_CONFIG_BIAS_C0_NPDPII_TAU_S_P          		40
#define DYNAPSE_CONFIG_BIAS_C0_NPDPII_TAU_F_P 				42
#define DYNAPSE_CONFIG_BIAS_C0_NPDPIE_TAU_F_P         		44
#define DYNAPSE_CONFIG_BIAS_C0_NPDPIE_TAU_S_P               46
#define DYNAPSE_CONFIG_BIAS_C0_R2R_P               			48

#define DYNAPSE_CONFIG_BIAS_C1_PULSE_PWLK_P             	1
#define DYNAPSE_CONFIG_BIAS_C1_PS_WEIGHT_INH_S_N            3
#define DYNAPSE_CONFIG_BIAS_C1_PS_WEIGHT_INH_F_N            5
#define DYNAPSE_CONFIG_BIAS_C1_PS_WEIGHT_EXC_S_N         	7
#define DYNAPSE_CONFIG_BIAS_C1_PS_WEIGHT_EXC_F_N        	9
#define DYNAPSE_CONFIG_BIAS_C1_IF_RFR_N          			11
#define DYNAPSE_CONFIG_BIAS_C1_IF_TAU1_N         			13
#define DYNAPSE_CONFIG_BIAS_C1_IF_AHTAU_N           		15
#define DYNAPSE_CONFIG_BIAS_C1_IF_CASC_N 					17
#define DYNAPSE_CONFIG_BIAS_C1_IF_TAU2_N         			19
#define DYNAPSE_CONFIG_BIAS_C1_IF_BUF_P               		21
#define DYNAPSE_CONFIG_BIAS_C1_IF_AHTHR_N             		23
#define DYNAPSE_CONFIG_BIAS_C1_IF_THR_N            			25
#define DYNAPSE_CONFIG_BIAS_C1_NPDPIE_THR_S_P             	27
#define DYNAPSE_CONFIG_BIAS_C1_NPDPIE_THR_F_P            	29
#define DYNAPSE_CONFIG_BIAS_C1_NPDPII_THR_F_P      			31
#define DYNAPSE_CONFIG_BIAS_C1_NPDPII_THR_S_P            	33
#define DYNAPSE_CONFIG_BIAS_C1_IF_NMDA_N            		35
#define DYNAPSE_CONFIG_BIAS_C1_IF_DC_P           			37
#define DYNAPSE_CONFIG_BIAS_C1_IF_AHW_P          			39
#define DYNAPSE_CONFIG_BIAS_C1_NPDPII_TAU_S_P          		41
#define DYNAPSE_CONFIG_BIAS_C1_NPDPII_TAU_F_P 				43
#define DYNAPSE_CONFIG_BIAS_C1_NPDPIE_TAU_F_P         		45
#define DYNAPSE_CONFIG_BIAS_C1_NPDPIE_TAU_S_P               47
#define DYNAPSE_CONFIG_BIAS_C1_R2R_P               			49

#define DYNAPSE_CONFIG_BIAS_U_BUFFER         				50
#define DYNAPSE_CONFIG_BIAS_U_SSP               			51
#define DYNAPSE_CONFIG_BIAS_U_SSN               			52

#define DYNAPSE_CONFIG_BIAS_C2_PULSE_PWLK_P             	64
#define DYNAPSE_CONFIG_BIAS_C2_PS_WEIGHT_INH_S_N            66
#define DYNAPSE_CONFIG_BIAS_C2_PS_WEIGHT_INH_F_N            68
#define DYNAPSE_CONFIG_BIAS_C2_PS_WEIGHT_EXC_S_N         	70
#define DYNAPSE_CONFIG_BIAS_C2_PS_WEIGHT_EXC_F_N        	72
#define DYNAPSE_CONFIG_BIAS_C2_IF_RFR_N          			74
#define DYNAPSE_CONFIG_BIAS_C2_IF_TAU1_N         			76
#define DYNAPSE_CONFIG_BIAS_C2_IF_AHTAU_N           		78
#define DYNAPSE_CONFIG_BIAS_C2_IF_CASC_N 					80
#define DYNAPSE_CONFIG_BIAS_C2_IF_TAU2_N         			82
#define DYNAPSE_CONFIG_BIAS_C2_IF_BUF_P               		84
#define DYNAPSE_CONFIG_BIAS_C2_IF_AHTHR_N             		86
#define DYNAPSE_CONFIG_BIAS_C2_IF_THR_N            			88
#define DYNAPSE_CONFIG_BIAS_C2_NPDPIE_THR_S_P             	90
#define DYNAPSE_CONFIG_BIAS_C2_NPDPIE_THR_F_P            	92
#define DYNAPSE_CONFIG_BIAS_C2_NPDPII_THR_F_P      			94
#define DYNAPSE_CONFIG_BIAS_C2_NPDPII_THR_S_P            	96
#define DYNAPSE_CONFIG_BIAS_C2_IF_NMDA_N            		98
#define DYNAPSE_CONFIG_BIAS_C2_IF_DC_P           			100
#define DYNAPSE_CONFIG_BIAS_C2_IF_AHW_P          			102
#define DYNAPSE_CONFIG_BIAS_C2_NPDPII_TAU_S_P          		104
#define DYNAPSE_CONFIG_BIAS_C2_NPDPII_TAU_F_P 				106
#define DYNAPSE_CONFIG_BIAS_C2_NPDPIE_TAU_F_P         		108
#define DYNAPSE_CONFIG_BIAS_C2_NPDPIE_TAU_S_P               110
#define DYNAPSE_CONFIG_BIAS_C2_R2R_P               			112

#define DYNAPSE_CONFIG_BIAS_C3_PULSE_PWLK_P             	65
#define DYNAPSE_CONFIG_BIAS_C3_PS_WEIGHT_INH_S_N            67
#define DYNAPSE_CONFIG_BIAS_C3_PS_WEIGHT_INH_F_N            69
#define DYNAPSE_CONFIG_BIAS_C3_PS_WEIGHT_EXC_S_N         	71
#define DYNAPSE_CONFIG_BIAS_C3_PS_WEIGHT_EXC_F_N        	73
#define DYNAPSE_CONFIG_BIAS_C3_IF_RFR_N          			75
#define DYNAPSE_CONFIG_BIAS_C3_IF_TAU1_N         			77
#define DYNAPSE_CONFIG_BIAS_C3_IF_AHTAU_N           		79
#define DYNAPSE_CONFIG_BIAS_C3_IF_CASC_N 					81
#define DYNAPSE_CONFIG_BIAS_C3_IF_TAU2_N         			83
#define DYNAPSE_CONFIG_BIAS_C3_IF_BUF_P               		85
#define DYNAPSE_CONFIG_BIAS_C3_IF_AHTHR_N             		87
#define DYNAPSE_CONFIG_BIAS_C3_IF_THR_N            			89
#define DYNAPSE_CONFIG_BIAS_C3_NPDPIE_THR_S_P             	91
#define DYNAPSE_CONFIG_BIAS_C3_NPDPIE_THR_F_P            	93
#define DYNAPSE_CONFIG_BIAS_C3_NPDPII_THR_F_P      			95
#define DYNAPSE_CONFIG_BIAS_C3_NPDPII_THR_S_P            	97
#define DYNAPSE_CONFIG_BIAS_C3_IF_NMDA_N            		99
#define DYNAPSE_CONFIG_BIAS_C3_IF_DC_P           			101
#define DYNAPSE_CONFIG_BIAS_C3_IF_AHW_P          			103
#define DYNAPSE_CONFIG_BIAS_C3_NPDPII_TAU_S_P          		105
#define DYNAPSE_CONFIG_BIAS_C3_NPDPII_TAU_F_P 				107
#define DYNAPSE_CONFIG_BIAS_C3_NPDPIE_TAU_F_P         		109
#define DYNAPSE_CONFIG_BIAS_C3_NPDPIE_TAU_S_P               111
#define DYNAPSE_CONFIG_BIAS_C3_R2R_P               			113

#define DYNAPSE_CONFIG_BIAS_D_BUFFER         				114
#define DYNAPSE_CONFIG_BIAS_D_SSP               			115
#define DYNAPSE_CONFIG_BIAS_D_SSN               			116*/




}


static void sendDefaultConfiguration(caerModuleData moduleData,
		struct caer_dynapse_info *devInfo) {
	// Device related configuration has its own sub-node.
	sshsNode deviceConfigNode = sshsGetRelativeNode(moduleData->moduleNode,
			chipIDToName(devInfo->chipID, true));

	// send default bias configuration
	biasConfigSend(sshsGetRelativeNode(deviceConfigNode, "bias/"), moduleData, devInfo);



}

bool caerInputDYNAPSEInit(caerModuleData moduleData, uint16_t deviceType) {

	caerLog(CAER_LOG_DEBUG, moduleData->moduleSubSystemString,
			"Initializing module ...");

// USB port/bus/SN settings/restrictions.
// These can be used to force connection to one specific device at startup.
	sshsNodePutShortIfAbsent(moduleData->moduleNode, "busNumber", 0);
	sshsNodePutShortIfAbsent(moduleData->moduleNode, "devAddress", 0);
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "serialNumber", "");

// Add auto-restart setting.
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "autoRestart", true);

/// Start data acquisition, and correctly notify mainloop of new data and module of exceptional
// shutdown cases (device pulled, ...).
	char *serialNumber = sshsNodeGetString(moduleData->moduleNode,
			"serialNumber");
	caerLog(CAER_LOG_CRITICAL, moduleData->moduleSubSystemString,
			"Initializing module ... %d", deviceType);

	moduleData->moduleState = caerDeviceOpen(1, CAER_DEVICE_DYNAPSE, 0, 0,
			NULL);

	free(serialNumber);

	if (moduleData->moduleState == NULL) {
		// Failed to open device.
		return (false);
	}

// Let's take a look at the information we have on the device.
	struct caer_dynapse_info dynapse_info = caerDynapseInfoGet(
			moduleData->moduleState);

	printf("%s --- ID: %d, Master: %d,  Logic: %d,  ChipID: %d.\n",
			dynapse_info.deviceString, dynapse_info.deviceID,
			dynapse_info.deviceIsMaster, dynapse_info.logicVersion,
			dynapse_info.chipID);

	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode,
			"sourceInfo/");

	sshsNodePutLong(sourceInfoNode, "highestTimestamp", -1);
	sshsNodePutShort(sourceInfoNode, "logicVersion", dynapse_info.logicVersion);
	sshsNodePutBool(sourceInfoNode, "deviceIsMaster",
			dynapse_info.deviceIsMaster);
	sshsNodePutShort(sourceInfoNode, "deviceID", dynapse_info.deviceID);
	sshsNodePutShort(sourceInfoNode, "chipID", dynapse_info.chipID);

// Generate source string for output modules.
	size_t sourceStringLength = (size_t) snprintf(NULL, 0,
			"#Source %" PRIu16 ": %s\r\n", moduleData->moduleID,
			chipIDToName(dynapse_info.chipID, false));

	char sourceString[sourceStringLength + 1];
	snprintf(sourceString, sourceStringLength + 1,
			"#Source %" PRIu16 ": %s\r\n", moduleData->moduleID,
			chipIDToName(dynapse_info.chipID, false));
	sourceString[sourceStringLength] = '\0';

	sshsNodePutString(sourceInfoNode, "sourceString", sourceString);

// Generate sub-system string for module.
	size_t subSystemStringLength = (size_t) snprintf(NULL, 0,
			"%s[SN %s, %" PRIu8 ":%" PRIu8 "]",
			moduleData->moduleSubSystemString, dynapse_info.deviceSerialNumber,
			dynapse_info.deviceUSBBusNumber,
			dynapse_info.deviceUSBDeviceAddress);

	char subSystemString[subSystemStringLength + 1];
	snprintf(subSystemString, subSystemStringLength + 1,
			"%s[SN %s, %" PRIu8 ":%" PRIu8 "]",
			moduleData->moduleSubSystemString, dynapse_info.deviceSerialNumber,
			dynapse_info.deviceUSBBusNumber,
			dynapse_info.deviceUSBDeviceAddress);
	subSystemString[subSystemStringLength] = '\0';

	caerModuleSetSubSystemString(moduleData, subSystemString);

// Ensure good defaults for data acquisition settings.
// No blocking behavior due to mainloop notification, and no auto-start of
// all producers to ensure cAER settings are respected.
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_DATAEXCHANGE,
	CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, false);
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_DATAEXCHANGE,
	CAER_HOST_CONFIG_DATAEXCHANGE_START_PRODUCERS, false);
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_DATAEXCHANGE,
	CAER_HOST_CONFIG_DATAEXCHANGE_STOP_PRODUCERS, true);

// Create default settings and send them to the device.
	createDefaultConfiguration(moduleData, &dynapse_info);
	sendDefaultConfiguration(moduleData, &dynapse_info);

// Start data acquisition.
	bool ret = caerDeviceDataStart(moduleData->moduleState,
			&mainloopDataNotifyIncrease, &mainloopDataNotifyDecrease,
			caerMainloopGetReference(), &moduleShutdownNotify,
			moduleData->moduleNode);

	if (!ret) {
		// Failed to start data acquisition, close device and exit.
		caerDeviceClose((caerDeviceHandle *) &moduleData->moduleState);

		return (false);
	}

// Device related configuration has its own sub-node.
	sshsNode deviceConfigNode = sshsGetRelativeNode(moduleData->moduleNode,
			chipIDToName(dynapse_info.chipID, true));

	sshsNode biasNode = sshsGetRelativeNode(deviceConfigNode, "bias/");

	size_t biasNodesLength = 0;
	sshsNode *biasNodes = sshsNodeGetChildren(biasNode, &biasNodesLength);

	if (biasNodes != NULL) {
		for (size_t i = 0; i < biasNodesLength; i++) {
			// Add listener for this particular bias.
			sshsNodeAddAttributeListener(biasNodes[i], moduleData,
					&biasConfigListener);
		}

		free(biasNodes);
	}

	return (true);

}

void caerInputDYNAPSEExit(caerModuleData moduleData) {
// Device related configuration has its own sub-node.
	struct caer_dynapse_info devInfo = caerDynapseInfoGet(
			moduleData->moduleState);
	sshsNode deviceConfigNode = sshsGetRelativeNode(moduleData->moduleNode,
			chipIDToName(devInfo.chipID, true));

	caerDeviceDataStop(moduleData->moduleState);

	caerDeviceClose((caerDeviceHandle *) &moduleData->moduleState);

	if (sshsNodeGetBool(moduleData->moduleNode, "autoRestart")) {
		// Prime input module again so that it will try to restart if new devices detected.
		sshsNodePutBool(moduleData->moduleNode, "running", true);
	}
}

void caerInputDYNAPSERun(caerModuleData moduleData, size_t argsNumber,
		va_list args) {
	UNUSED_ARGUMENT(argsNumber);

// Interpret variable arguments (same as above in main function).
	caerEventPacketContainer *container = va_arg(args,
			caerEventPacketContainer *);

	*container = caerDeviceDataGet(moduleData->moduleState);

	if (*container != NULL) {
		caerMainloopFreeAfterLoop(
				(void (*)(void *)) &caerEventPacketContainerFree, *container);

		sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode,
				"sourceInfo/");
		sshsNodePutLong(sourceInfoNode, "highestTimestamp",
				caerEventPacketContainerGetHighestEventTimestamp(*container));

		// Detect timestamp reset and call all reset functions for processors and outputs.
		caerEventPacketHeader special = caerEventPacketContainerGetEventPacket(
				*container, SPECIAL_EVENT);

		if ((special != NULL)
				&& (caerEventPacketHeaderGetEventNumber(special) == 1)
				&& (caerSpecialEventPacketFindEventByType(
						(caerSpecialEventPacket) special, TIMESTAMP_RESET)
						!= NULL)) {
			caerMainloopResetProcessors(moduleData->moduleID);
			caerMainloopResetOutputs(moduleData->moduleID);

			// Update master/slave information.
			struct caer_dynapse_info devInfo = caerDynapseInfoGet(
					moduleData->moduleState);
			sshsNodePutBool(sourceInfoNode, "deviceIsMaster",
					devInfo.deviceIsMaster);
		}
	}
}

