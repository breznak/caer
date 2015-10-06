#include "davis_common.h"

static void createVDACBiasSetting(sshsNode biasNode, const char *biasName, uint8_t currentValue, uint8_t voltageValue) {
	// Add trailing slash to node name (required!).
	size_t biasNameLength = strlen(biasName);
	char biasNameFull[biasNameLength + 2];
	memcpy(biasNameFull, biasName, biasNameLength);
	biasNameFull[biasNameLength] = '/';
	biasNameFull[biasNameLength + 1] = '\0';

	// Create configuration node for this particular bias.
	sshsNode biasConfigNode = sshsGetRelativeNode(biasNode, biasNameFull);

	// Add bias settings.
	sshsNodePutByteIfAbsent(biasConfigNode, "currentValue", currentValue);
	sshsNodePutByteIfAbsent(biasConfigNode, "voltageValue", voltageValue);
}

static uint16_t generateVDACBias(sshsNode biasNode, const char *biasName) {
	// Add trailing slash to node name (required!).
	size_t biasNameLength = strlen(biasName);
	char biasNameFull[biasNameLength + 2];
	memcpy(biasNameFull, biasName, biasNameLength);
	biasNameFull[biasNameLength] = '/';
	biasNameFull[biasNameLength + 1] = '\0';

	// Get bias configuration node.
	sshsNode biasConfigNode = sshsGetRelativeNode(biasNode, biasNameFull);

	// Build up bias value from all its components.
	struct caer_bias_vdac biasValue = { .voltageValue = sshsNodeGetByte(biasConfigNode, "voltageValue"), .currentValue =
		sshsNodeGetByte(biasConfigNode, "currentValue"), };

	return (caerBiasVDACGenerate(biasValue));
}

static void createCoarseFineBiasSetting(sshsNode biasNode, const char *biasName, const char *type, const char *sex,
	uint8_t coarseValue, uint8_t fineValue, bool enabled) {
	// Add trailing slash to node name (required!).
	size_t biasNameLength = strlen(biasName);
	char biasNameFull[biasNameLength + 2];
	memcpy(biasNameFull, biasName, biasNameLength);
	biasNameFull[biasNameLength] = '/';
	biasNameFull[biasNameLength + 1] = '\0';

	// Create configuration node for this particular bias.
	sshsNode biasConfigNode = sshsGetRelativeNode(biasNode, biasNameFull);

	// Add bias settings.
	sshsNodePutStringIfAbsent(biasConfigNode, "type", type);
	sshsNodePutStringIfAbsent(biasConfigNode, "sex", sex);
	sshsNodePutByteIfAbsent(biasConfigNode, "coarseValue", coarseValue);
	sshsNodePutByteIfAbsent(biasConfigNode, "fineValue", fineValue);
	sshsNodePutBoolIfAbsent(biasConfigNode, "enabled", enabled);
	sshsNodePutStringIfAbsent(biasConfigNode, "currentLevel", "Normal");
}

static uint16_t generateCoarseFineBias(sshsNode biasNode, const char *biasName) {
	// Add trailing slash to node name (required!).
	size_t biasNameLength = strlen(biasName);
	char biasNameFull[biasNameLength + 2];
	memcpy(biasNameFull, biasName, biasNameLength);
	biasNameFull[biasNameLength] = '/';
	biasNameFull[biasNameLength + 1] = '\0';

	// Get bias configuration node.
	sshsNode biasConfigNode = sshsGetRelativeNode(biasNode, biasNameFull);

	// Build up bias value from all its components.
	struct caer_bias_coarsefine biasValue = { .coarseValue = sshsNodeGetByte(biasConfigNode, "coarseValue"),
		.fineValue = sshsNodeGetByte(biasConfigNode, "fineValue"),
		.enabled = sshsNodeGetBool(biasConfigNode, "enabled"), .sexN = caerStrEquals(
			sshsNodeGetString(biasConfigNode, "sex"), "N"), .typeNormal = caerStrEquals(
			sshsNodeGetString(biasConfigNode, "type"), "Normal"), .currentLevelNormal = caerStrEquals(
			sshsNodeGetString(biasConfigNode, "currentLevel"), "Normal"), };

	return (caerBiasCoarseFineGenerate(biasValue));
}

static void createShiftedSourceBiasSetting(sshsNode biasNode, const char *biasName, uint8_t regValue, uint8_t refValue,
	const char *operatingMode, const char *voltageLevel) {
	// Add trailing slash to node name (required!).
	size_t biasNameLength = strlen(biasName);
	char biasNameFull[biasNameLength + 2];
	memcpy(biasNameFull, biasName, biasNameLength);
	biasNameFull[biasNameLength] = '/';
	biasNameFull[biasNameLength + 1] = '\0';

	// Create configuration node for this particular bias.
	sshsNode biasConfigNode = sshsGetRelativeNode(biasNode, biasNameFull);

	// Add bias settings.
	sshsNodePutByteIfAbsent(biasConfigNode, "regValue", regValue);
	sshsNodePutByteIfAbsent(biasConfigNode, "refValue", refValue);
	sshsNodePutStringIfAbsent(biasConfigNode, "operatingMode", operatingMode);
	sshsNodePutStringIfAbsent(biasConfigNode, "voltageLevel", voltageLevel);
}

static uint16_t generateShiftedSourceBias(sshsNode biasNode, const char *biasName) {
	// Add trailing slash to node name (required!).
	size_t biasNameLength = strlen(biasName);
	char biasNameFull[biasNameLength + 2];
	memcpy(biasNameFull, biasName, biasNameLength);
	biasNameFull[biasNameLength] = '/';
	biasNameFull[biasNameLength + 1] = '\0';

	// Get bias configuration node.
	sshsNode biasConfigNode = sshsGetRelativeNode(biasNode, biasNameFull);

	// Build up bias value from all its components.
	struct caer_bias_shiftedsource biasValue = { .refValue = sshsNodeGetByte(biasConfigNode, "refValue"), .regValue =
		sshsNodeGetByte(biasConfigNode, "regValue"), .operatingMode =
		(caerStrEquals(sshsNodeGetString(biasConfigNode, "operatingMode"), "HiZ")) ?
			(HI_Z) :
			((caerStrEquals(sshsNodeGetString(biasConfigNode, "operatingMode"), "TiedToRail")) ?
				(TIED_TO_RAIL) : (SHIFTED_SOURCE)), .voltageLevel =
		(caerStrEquals(sshsNodeGetString(biasConfigNode, "voltageLevel"), "SingleDiode")) ?
			(SINGLE_DIODE) :
			((caerStrEquals(sshsNodeGetString(biasConfigNode, "voltageLevel"), "DoubleDiode")) ?
				(DOUBLE_DIODE) : (SPLIT_GATE)), };

	return (caerBiasShiftedSourceGenerate(biasValue));
}

bool deviceOpenInfo(caerModuleData moduleData, davisCommonState cstate, uint16_t VID, uint16_t PID, uint8_t DID_TYPE) {
	// USB port/bus/SN settings/restrictions.
	// These can be used to force connection to one specific device.
	sshsNode selectorNode = sshsGetRelativeNode(moduleData->moduleNode, "usbDevice/");

	sshsNodePutByteIfAbsent(selectorNode, "BusNumber", 0);
	sshsNodePutByteIfAbsent(selectorNode, "DevAddress", 0);
	sshsNodePutStringIfAbsent(selectorNode, "SerialNumber", "");

	// Try to open a DAVIS device on a specific USB port.
	cstate->deviceHandle = deviceOpen(cstate->deviceContext, VID, PID, DID_TYPE,
		sshsNodeGetByte(selectorNode, "BusNumber"), sshsNodeGetByte(selectorNode, "DevAddress"));

	// Update module log string, make it accessible in cstate space.
	caerModuleSetSubSystemString(moduleData, fullLogString);

	// Now check if the Serial Number matches.
	char *configSerialNumber = sshsNodeGetString(selectorNode, "SerialNumber");

	free(configSerialNumber);

	// So now we have a working connection to the device we want. Let's get some data!
	// Put global source information into SSHS, so it's globally available.
	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");

	if (cstate->apsInvertXY) {
		sshsNodePutShort(sourceInfoNode, "apsSizeX", cstate->apsSizeY);
		sshsNodePutShort(sourceInfoNode, "apsSizeY", cstate->apsSizeX);
	}
	else {
		sshsNodePutShort(sourceInfoNode, "apsSizeX", cstate->apsSizeX);
		sshsNodePutShort(sourceInfoNode, "apsSizeY", cstate->apsSizeY);
	}
	sshsNodePutShort(sourceInfoNode, "apsChannels", cstate->apsChannels);

	if (cstate->dvsInvertXY) {
		sshsNodePutShort(sourceInfoNode, "dvsSizeX", cstate->dvsSizeY);
		sshsNodePutShort(sourceInfoNode, "dvsSizeY", cstate->dvsSizeX);
	}
	else {
		sshsNodePutShort(sourceInfoNode, "dvsSizeX", cstate->dvsSizeX);
		sshsNodePutShort(sourceInfoNode, "dvsSizeY", cstate->dvsSizeY);
	}

	sshsNodePutShort(sourceInfoNode, "apsOriginalDepth", DAVIS_ADC_DEPTH);
	sshsNodePutShort(sourceInfoNode, "apsOriginalChannels", DAVIS_COLOR_CHANNELS);
	sshsNodePutBool(sourceInfoNode, "apsHasGlobalShutter", spiConfigReceive(cstate->deviceHandle, FPGA_APS, 7));
	sshsNodePutBool(sourceInfoNode, "apsHasExternalADC", spiConfigReceive(cstate->deviceHandle, FPGA_APS, 32));
	sshsNodePutBool(sourceInfoNode, "apsHasInternalADC", spiConfigReceive(cstate->deviceHandle, FPGA_APS, 33));

	sshsNodePutShort(sourceInfoNode, "logicVersion", U16T(spiConfigReceive(cstate->deviceHandle, FPGA_SYSINFO, 0)));
	sshsNodePutBool(sourceInfoNode, "deviceIsMaster", spiConfigReceive(cstate->deviceHandle, FPGA_SYSINFO, 2));

	return (true);
}

void createCommonConfiguration(caerModuleData moduleData, davisCommonState cstate) {
	// First, always create all needed setting nodes, set their default values
	// and add their listeners.
	sshsNode biasNode = sshsGetRelativeNode(moduleData->moduleNode, "bias/");
	biasDescriptor *biases = cstate->chipBiases;

	if (cstate->chipID == CHIP_DAVIS240A || cstate->chipID == CHIP_DAVIS240B || cstate->chipID == CHIP_DAVIS240C) {
		createCoarseFineBiasSetting(biases, biasNode, "DiffBn", 0, "Normal", "N", 4, 39, true);
		createCoarseFineBiasSetting(biases, biasNode, "OnBn", 1, "Normal", "N", 5, 255, true);
		createCoarseFineBiasSetting(biases, biasNode, "OffBn", 2, "Normal", "N", 4, 0, true);
		createCoarseFineBiasSetting(biases, biasNode, "ApsCasEpc", 3, "Cascode", "N", 5, 185, true);
		createCoarseFineBiasSetting(biases, biasNode, "DiffCasBnc", 4, "Cascode", "N", 5, 115, true);
		createCoarseFineBiasSetting(biases, biasNode, "ApsROSFBn", 5, "Normal", "N", 6, 219, true);
		createCoarseFineBiasSetting(biases, biasNode, "LocalBufBn", 6, "Normal", "N", 5, 164, true);
		createCoarseFineBiasSetting(biases, biasNode, "PixInvBn", 7, "Normal", "N", 5, 129, true);
		createCoarseFineBiasSetting(biases, biasNode, "PrBp", 8, "Normal", "P", 2, 58, true);
		createCoarseFineBiasSetting(biases, biasNode, "PrSFBp", 9, "Normal", "P", 1, 16, true);
		createCoarseFineBiasSetting(biases, biasNode, "RefrBp", 10, "Normal", "P", 4, 25, true);
		createCoarseFineBiasSetting(biases, biasNode, "AEPdBn", 11, "Normal", "N", 6, 91, true);
		createCoarseFineBiasSetting(biases, biasNode, "LcolTimeoutBn", 12, "Normal", "N", 5, 49, true);
		createCoarseFineBiasSetting(biases, biasNode, "AEPuXBp", 13, "Normal", "P", 4, 80, true);
		createCoarseFineBiasSetting(biases, biasNode, "AEPuYBp", 14, "Normal", "P", 7, 152, true);
		createCoarseFineBiasSetting(biases, biasNode, "IFThrBn", 15, "Normal", "N", 5, 255, true);
		createCoarseFineBiasSetting(biases, biasNode, "IFRefrBn", 16, "Normal", "N", 5, 255, true);
		createCoarseFineBiasSetting(biases, biasNode, "PadFollBn", 17, "Normal", "N", 7, 215, true);
		createCoarseFineBiasSetting(biases, biasNode, "ApsOverflowLevel", 18, "Normal", "N", 6, 253, true);

		createCoarseFineBiasSetting(biases, biasNode, "BiasBuffer", 19, "Normal", "N", 5, 254, true);

		createShiftedSourceBiasSetting(biases, biasNode, "SSP", 20, 33, 1, "ShiftedSource", "SplitGate");
		createShiftedSourceBiasSetting(biases, biasNode, "SSN", 21, 33, 1, "ShiftedSource", "SplitGate");
	}

	if (cstate->chipID == CHIP_DAVIS640) {
		// Slow down pixels for big 640x480 array.
		createCoarseFineBiasSetting(biases, biasNode, "PrBp", 14, "Normal", "P", 2, 3, true);
		createCoarseFineBiasSetting(biases, biasNode, "PrSFBp", 15, "Normal", "P", 1, 1, true);
	}

	if (cstate->chipID == CHIP_DAVIS128 || cstate->chipID == CHIP_DAVIS346A || cstate->chipID == CHIP_DAVIS346B
		|| cstate->chipID == CHIP_DAVIS346C || cstate->chipID == CHIP_DAVIS640 || cstate->chipID == CHIP_DAVIS208) {
		createVDACBiasSetting(biases, biasNode, "ApsOverflowLevel", 0, 6, 27);
		createVDACBiasSetting(biases, biasNode, "ApsCas", 1, 6, 21);
		createVDACBiasSetting(biases, biasNode, "AdcRefHigh", 2, 7, 30);
		createVDACBiasSetting(biases, biasNode, "AdcRefLow", 3, 7, 1);
		if (cstate->chipID == CHIP_DAVIS346A || cstate->chipID == CHIP_DAVIS346B || cstate->chipID == CHIP_DAVIS346C
			|| cstate->chipID == CHIP_DAVIS640) {
			// Only DAVIS346 and 640 have ADC testing.
			createVDACBiasSetting(biases, biasNode, "AdcTestVoltage", 4, 7, 21);
		}

		createCoarseFineBiasSetting(biases, biasNode, "LocalBufBn", 8, "Normal", "N", 5, 164, true);
		createCoarseFineBiasSetting(biases, biasNode, "PadFollBn", 9, "Normal", "N", 7, 215, true);
		createCoarseFineBiasSetting(biases, biasNode, "DiffBn", 10, "Normal", "N", 4, 39, true);
		createCoarseFineBiasSetting(biases, biasNode, "OnBn", 11, "Normal", "N", 5, 255, true);
		createCoarseFineBiasSetting(biases, biasNode, "OffBn", 12, "Normal", "N", 4, 1, true);
		createCoarseFineBiasSetting(biases, biasNode, "PixInvBn", 13, "Normal", "N", 5, 129, true);
		createCoarseFineBiasSetting(biases, biasNode, "PrBp", 14, "Normal", "P", 2, 58, true);
		createCoarseFineBiasSetting(biases, biasNode, "PrSFBp", 15, "Normal", "P", 1, 16, true);
		createCoarseFineBiasSetting(biases, biasNode, "RefrBp", 16, "Normal", "P", 4, 25, true);
		createCoarseFineBiasSetting(biases, biasNode, "ReadoutBufBp", 17, "Normal", "P", 6, 20, true);
		createCoarseFineBiasSetting(biases, biasNode, "ApsROSFBn", 18, "Normal", "N", 6, 219, true);
		createCoarseFineBiasSetting(biases, biasNode, "AdcCompBp", 19, "Normal", "P", 5, 20, true);
		createCoarseFineBiasSetting(biases, biasNode, "ColSelLowBn", 20, "Normal", "N", 0, 1, true);
		createCoarseFineBiasSetting(biases, biasNode, "DACBufBp", 21, "Normal", "P", 6, 60, true);
		createCoarseFineBiasSetting(biases, biasNode, "LcolTimeoutBn", 22, "Normal", "N", 5, 49, true);
		createCoarseFineBiasSetting(biases, biasNode, "AEPdBn", 23, "Normal", "N", 6, 91, true);
		createCoarseFineBiasSetting(biases, biasNode, "AEPuXBp", 24, "Normal", "P", 4, 80, true);
		createCoarseFineBiasSetting(biases, biasNode, "AEPuYBp", 25, "Normal", "P", 7, 152, true);
		createCoarseFineBiasSetting(biases, biasNode, "IFRefrBn", 26, "Normal", "N", 5, 255, true);
		createCoarseFineBiasSetting(biases, biasNode, "IFThrBn", 27, "Normal", "N", 5, 255, true);

		createCoarseFineBiasSetting(biases, biasNode, "BiasBuffer", 34, "Normal", "N", 5, 254, true);

		createShiftedSourceBiasSetting(biases, biasNode, "SSP", 35, 33, 1, "ShiftedSource", "SplitGate");
		createShiftedSourceBiasSetting(biases, biasNode, "SSN", 36, 33, 1, "ShiftedSource", "SplitGate");
	}

	if (cstate->chipID == CHIP_DAVIS208) {
		createVDACBiasSetting(biases, biasNode, "ResetHighPass", 6, 7, 63);
		createVDACBiasSetting(biases, biasNode, "RefSS", 7, 5, 11);

		createCoarseFineBiasSetting(biases, biasNode, "RegBiasBp", 28, "Normal", "P", 5, 20, true);
		createCoarseFineBiasSetting(biases, biasNode, "RefSSBn", 30, "Normal", "N", 5, 20, true);
	}

	if (cstate->chipID == CHIP_DAVISRGB) {
		createVDACBiasSetting(biases, biasNode, "ApsCas", 0, 4, 21);
		createVDACBiasSetting(biases, biasNode, "OVG1Lo", 1, 4, 21);
		createVDACBiasSetting(biases, biasNode, "OVG2Lo", 2, 0, 0);
		createVDACBiasSetting(biases, biasNode, "TX2OVG2Hi", 3, 0, 63);
		createVDACBiasSetting(biases, biasNode, "Gnd07", 4, 4, 13);
		createVDACBiasSetting(biases, biasNode, "AdcTestVoltage", 5, 0, 21);
		createVDACBiasSetting(biases, biasNode, "AdcRefHigh", 6, 7, 63);
		createVDACBiasSetting(biases, biasNode, "AdcRefLow", 7, 7, 0);

		createCoarseFineBiasSetting(biases, biasNode, "IFRefrBn", 8, "Normal", "N", 5, 255, false);
		createCoarseFineBiasSetting(biases, biasNode, "IFThrBn", 9, "Normal", "N", 5, 255, false);
		createCoarseFineBiasSetting(biases, biasNode, "LocalBufBn", 10, "Normal", "N", 5, 164, false);
		createCoarseFineBiasSetting(biases, biasNode, "PadFollBn", 11, "Normal", "N", 7, 209, false);
		createCoarseFineBiasSetting(biases, biasNode, "PixInvBn", 13, "Normal", "N", 4, 164, true);
		createCoarseFineBiasSetting(biases, biasNode, "DiffBn", 14, "Normal", "N", 4, 54, true);
		createCoarseFineBiasSetting(biases, biasNode, "OnBn", 15, "Normal", "N", 6, 63, true);
		createCoarseFineBiasSetting(biases, biasNode, "OffBn", 16, "Normal", "N", 2, 138, true);
		createCoarseFineBiasSetting(biases, biasNode, "PrBp", 17, "Normal", "P", 1, 108, true);
		createCoarseFineBiasSetting(biases, biasNode, "PrSFBp", 18, "Normal", "P", 1, 108, true);
		createCoarseFineBiasSetting(biases, biasNode, "RefrBp", 19, "Normal", "P", 4, 28, true);
		createCoarseFineBiasSetting(biases, biasNode, "ArrayBiasBufferBn", 20, "Normal", "N", 6, 128, true);
		createCoarseFineBiasSetting(biases, biasNode, "ArrayLogicBufferBn", 22, "Normal", "N", 5, 255, true);
		createCoarseFineBiasSetting(biases, biasNode, "FalltimeBn", 23, "Normal", "N", 7, 41, true);
		createCoarseFineBiasSetting(biases, biasNode, "RisetimeBp", 24, "Normal", "P", 6, 162, true);
		createCoarseFineBiasSetting(biases, biasNode, "ReadoutBufBp", 25, "Normal", "P", 6, 20, false);
		createCoarseFineBiasSetting(biases, biasNode, "ApsROSFBn", 26, "Normal", "N", 6, 255, true);
		createCoarseFineBiasSetting(biases, biasNode, "AdcCompBp", 27, "Normal", "P", 4, 159, true);
		createCoarseFineBiasSetting(biases, biasNode, "DACBufBp", 28, "Normal", "P", 6, 194, true);
		createCoarseFineBiasSetting(biases, biasNode, "LcolTimeoutBn", 30, "Normal", "N", 5, 49, true);
		createCoarseFineBiasSetting(biases, biasNode, "AEPdBn", 31, "Normal", "N", 6, 91, true);
		createCoarseFineBiasSetting(biases, biasNode, "AEPuXBp", 32, "Normal", "P", 4, 80, true);
		createCoarseFineBiasSetting(biases, biasNode, "AEPuYBp", 33, "Normal", "P", 7, 152, true);

		createCoarseFineBiasSetting(biases, biasNode, "BiasBuffer", 34, "Normal", "N", 6, 251, true);

		createShiftedSourceBiasSetting(biases, biasNode, "SSP", 35, 33, 1, "TiedToRail", "SplitGate");
		createShiftedSourceBiasSetting(biases, biasNode, "SSN", 36, 33, 2, "ShiftedSource", "SplitGate");
	}

	sshsNode chipNode = sshsGetRelativeNode(moduleData->moduleNode, "chip/");
	configChainDescriptor *configChain = cstate->chipConfigChain;

	createByteConfigSetting(configChain, chipNode, "DigitalMux0", 128, 0);
	createByteConfigSetting(configChain, chipNode, "DigitalMux1", 129, 0);
	createByteConfigSetting(configChain, chipNode, "DigitalMux2", 130, 0);
	createByteConfigSetting(configChain, chipNode, "DigitalMux3", 131, 0);
	createByteConfigSetting(configChain, chipNode, "AnalogMux0", 132, 0);
	createByteConfigSetting(configChain, chipNode, "AnalogMux1", 133, 0);
	createByteConfigSetting(configChain, chipNode, "AnalogMux2", 134, 0);
	createByteConfigSetting(configChain, chipNode, "BiasMux0", 135, 0);

	createBoolConfigSetting(configChain, chipNode, "ResetCalibNeuron", 136, true);
	createBoolConfigSetting(configChain, chipNode, "TypeNCalibNeuron", 137, false);
	createBoolConfigSetting(configChain, chipNode, "ResetTestPixel", 138, true);
	createBoolConfigSetting(configChain, chipNode, "AERnArow", 140, false); // Use nArow in the AER state machine.
	createBoolConfigSetting(configChain, chipNode, "UseAOut", 141, false); // Enable analog pads for aMUX output (testing).

	if (cstate->chipID == CHIP_DAVIS240A || cstate->chipID == CHIP_DAVIS240B) {
		createBoolConfigSetting(configChain, chipNode, "SpecialPixelControl", 139, false);
	}

	if (cstate->chipID == CHIP_DAVIS128 || cstate->chipID == CHIP_DAVIS208 || cstate->chipID == CHIP_DAVIS346A
		|| cstate->chipID == CHIP_DAVIS346B || cstate->chipID == CHIP_DAVIS346C || cstate->chipID == CHIP_DAVIS640
		|| cstate->chipID == CHIP_DAVISRGB) {
		// Select which grey counter to use with the internal ADC: '0' means the external grey counter is used, which
		// has to be supplied off-chip. '1' means the on-chip grey counter is used instead.
		createBoolConfigSetting(configChain, chipNode, "SelectGrayCounter", 143, true);
	}

	if (cstate->chipID == CHIP_DAVIS346A || cstate->chipID == CHIP_DAVIS346B || cstate->chipID == CHIP_DAVIS346C
		|| cstate->chipID == CHIP_DAVIS640 || cstate->chipID == CHIP_DAVISRGB) {
		// Test ADC functionality: if true, the ADC takes its input voltage not from the pixel, but from the
		// VDAC 'AdcTestVoltage'. If false, the voltage comes from the pixels.
		createBoolConfigSetting(configChain, chipNode, "TestADC", 144, false);
	}

	if (cstate->chipID == CHIP_DAVIS208) {
		createBoolConfigSetting(configChain, chipNode, "SelectPreAmpAvg", 145, false);
		createBoolConfigSetting(configChain, chipNode, "SelectBiasRefSS", 146, false);
		createBoolConfigSetting(configChain, chipNode, "SelectSense", 147, true);
		createBoolConfigSetting(configChain, chipNode, "SelectPosFb", 148, false);
		createBoolConfigSetting(configChain, chipNode, "SelectHighPass", 149, false);
	}

	if (cstate->chipID == CHIP_DAVISRGB) {
		createBoolConfigSetting(configChain, chipNode, "AdjustOVG1Lo", 145, true);
		createBoolConfigSetting(configChain, chipNode, "AdjustOVG2Lo", 146, false);
		createBoolConfigSetting(configChain, chipNode, "AdjustTX2OVG2Hi", 147, false);
	}

	// Subsystem 0: Multiplexer
	sshsNode muxNode = sshsGetRelativeNode(moduleData->moduleNode, "multiplexer/");

	sshsNodePutBoolIfAbsent(muxNode, "Run", true);
	sshsNodePutBoolIfAbsent(muxNode, "TimestampRun", true);
	sshsNodePutBoolIfAbsent(muxNode, "TimestampReset", false);
	sshsNodePutBoolIfAbsent(muxNode, "ForceChipBiasEnable", false);
	sshsNodePutBoolIfAbsent(muxNode, "DropDVSOnTransferStall", true);
	sshsNodePutBoolIfAbsent(muxNode, "DropAPSOnTransferStall", false);
	sshsNodePutBoolIfAbsent(muxNode, "DropIMUOnTransferStall", false);
	sshsNodePutBoolIfAbsent(muxNode, "DropExtInputOnTransferStall", true);

	// Subsystem 1: DVS AER
	sshsNode dvsNode = sshsGetRelativeNode(moduleData->moduleNode, "dvs/");

	sshsNodePutBoolIfAbsent(dvsNode, "Run", true);
	sshsNodePutByteIfAbsent(dvsNode, "AckDelayRow", 4);
	sshsNodePutByteIfAbsent(dvsNode, "AckDelayColumn", 0);
	sshsNodePutByteIfAbsent(dvsNode, "AckExtensionRow", 1);
	sshsNodePutByteIfAbsent(dvsNode, "AckExtensionColumn", 0);
	sshsNodePutBoolIfAbsent(dvsNode, "WaitOnTransferStall", false);
	sshsNodePutBoolIfAbsent(dvsNode, "FilterRowOnlyEvents", true);
	sshsNodePutBoolIfAbsent(dvsNode, "ExternalAERControl", false);

	// Subsystem 2: APS ADC
	sshsNode apsNode = sshsGetRelativeNode(moduleData->moduleNode, "aps/");

	// Only support GS on chips that have it available.
	bool globalShutterSupported = sshsNodeGetBool(sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/"),
		"apsHasGlobalShutter");
	if (globalShutterSupported) {
		sshsNodePutBoolIfAbsent(apsNode, "GlobalShutter", globalShutterSupported);
	}

	sshsNodePutBoolIfAbsent(apsNode, "Run", true);
	sshsNodePutBoolIfAbsent(apsNode, "ResetRead", true);
	sshsNodePutBoolIfAbsent(apsNode, "WaitOnTransferStall", true);
	sshsNodePutShortIfAbsent(apsNode, "StartColumn0", 0);
	sshsNodePutShortIfAbsent(apsNode, "StartRow0", 0);
	sshsNodePutShortIfAbsent(apsNode, "EndColumn0", U16T(cstate->apsSizeX - 1));
	sshsNodePutShortIfAbsent(apsNode, "EndRow0", U16T(cstate->apsSizeY - 1));
	sshsNodePutIntIfAbsent(apsNode, "Exposure", 4000); // in µs, converted to cycles later
	sshsNodePutIntIfAbsent(apsNode, "FrameDelay", 1000); // in µs, converted to cycles later
	sshsNodePutShortIfAbsent(apsNode, "ResetSettle", 10); // in cycles
	sshsNodePutShortIfAbsent(apsNode, "ColumnSettle", 30); // in cycles
	sshsNodePutShortIfAbsent(apsNode, "RowSettle", 8); // in cycles
	sshsNodePutShortIfAbsent(apsNode, "NullSettle", 3); // in cycles

	bool integratedADCSupported = sshsNodeGetBool(sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/"),
		"apsHasInternalADC");
	if (integratedADCSupported) {
		sshsNodePutBoolIfAbsent(apsNode, "UseInternalADC", true);
		sshsNodePutBoolIfAbsent(apsNode, "SampleEnable", true);
		sshsNodePutShortIfAbsent(apsNode, "SampleSettle", 60); // in cycles
		sshsNodePutShortIfAbsent(apsNode, "RampReset", 10); // in cycles
		sshsNodePutBoolIfAbsent(apsNode, "RampShortReset", true);
	}

	// DAVIS RGB has additional timing counters.
	if (cstate->chipID == CHIP_DAVISRGB) {
		sshsNodePutShortIfAbsent(apsNode, "TransferTime", 3000); // in cycles
		sshsNodePutShortIfAbsent(apsNode, "RSFDSettleTime", 3000); // in cycles
		sshsNodePutShortIfAbsent(apsNode, "GSPDResetTime", 3000); // in cycles
		sshsNodePutShortIfAbsent(apsNode, "GSResetFallTime", 3000); // in cycles
		sshsNodePutShortIfAbsent(apsNode, "GSTXFallTime", 3000); // in cycles
		sshsNodePutShortIfAbsent(apsNode, "GSFDResetTime", 3000); // in cycles
	}

	// Subsystem 3: IMU
	sshsNode imuNode = sshsGetRelativeNode(moduleData->moduleNode, "imu/");

	sshsNodePutBoolIfAbsent(imuNode, "Run", true);
	sshsNodePutBoolIfAbsent(imuNode, "TempStandby", false);
	sshsNodePutBoolIfAbsent(imuNode, "AccelXStandby", false);
	sshsNodePutBoolIfAbsent(imuNode, "AccelYStandby", false);
	sshsNodePutBoolIfAbsent(imuNode, "AccelZStandby", false);
	sshsNodePutBoolIfAbsent(imuNode, "GyroXStandby", false);
	sshsNodePutBoolIfAbsent(imuNode, "GyroYStandby", false);
	sshsNodePutBoolIfAbsent(imuNode, "GyroZStandby", false);
	sshsNodePutBoolIfAbsent(imuNode, "LowPowerCycle", false);
	sshsNodePutByteIfAbsent(imuNode, "LowPowerWakeupFrequency", 1);
	sshsNodePutByteIfAbsent(imuNode, "SampleRateDivider", 0);
	sshsNodePutByteIfAbsent(imuNode, "DigitalLowPassFilter", 1);
	sshsNodePutByteIfAbsent(imuNode, "AccelFullScale", 1);
	sshsNodePutByteIfAbsent(imuNode, "GyroFullScale", 1);

	// Subsystem 4: External Input
	sshsNode extNode = sshsGetRelativeNode(moduleData->moduleNode, "externalInput/");

	sshsNodePutBoolIfAbsent(extNode, "RunDetector", false);
	sshsNodePutBoolIfAbsent(extNode, "DetectRisingEdges", false);
	sshsNodePutBoolIfAbsent(extNode, "DetectFallingEdges", false);
	sshsNodePutBoolIfAbsent(extNode, "DetectPulses", true);
	sshsNodePutBoolIfAbsent(extNode, "DetectPulsePolarity", true);
	sshsNodePutIntIfAbsent(extNode, "DetectPulseLength", 10);

	// Subsystem 9: FX2/3 USB Configuration
	sshsNode usbNode = sshsGetRelativeNode(moduleData->moduleNode, "usb/");

	sshsNodePutBoolIfAbsent(usbNode, "Run", true);
	sshsNodePutShortIfAbsent(usbNode, "EarlyPacketDelay", 8); // 125µs time-slices, so 1ms

	sshsNodePutIntIfAbsent(usbNode, "BufferNumber", 8);
	sshsNodePutIntIfAbsent(usbNode, "BufferSize", 8192);

	sshsNode sysNode = sshsGetRelativeNode(moduleData->moduleNode, "system/");

	// Packet settings (size (in events) and time interval (in µs)).
	sshsNodePutIntIfAbsent(sysNode, "PolarityPacketMaxSize", 4096);
	sshsNodePutIntIfAbsent(sysNode, "PolarityPacketMaxInterval", 5000);
	sshsNodePutIntIfAbsent(sysNode, "FramePacketMaxSize", 4);
	sshsNodePutIntIfAbsent(sysNode, "FramePacketMaxInterval", 20000);
	sshsNodePutIntIfAbsent(sysNode, "IMU6PacketMaxSize", 32);
	sshsNodePutIntIfAbsent(sysNode, "IMU6PacketMaxInterval", 4000);
	sshsNodePutIntIfAbsent(sysNode, "SpecialPacketMaxSize", 128);
	sshsNodePutIntIfAbsent(sysNode, "SpecialPacketMaxInterval", 1000);

	// Ring-buffer setting (only changes value on module init/shutdown cycles).
	sshsNodePutIntIfAbsent(sysNode, "DataExchangeBufferSize", 64);

	// Add auto-restart setting.
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "auto-restart", true);

	// Install default listeners to signal configuration updates asynchronously.
	sshsNodeAddAttrListener(muxNode, cstate->deviceHandle, &MultiplexerConfigListener);
	sshsNodeAddAttrListener(dvsNode, cstate->deviceHandle, &DVSConfigListener);
	sshsNodeAddAttrListener(apsNode, moduleData->moduleState, &APSConfigListener);
	sshsNodeAddAttrListener(imuNode, cstate->deviceHandle, &IMUConfigListener);
	sshsNodeAddAttrListener(extNode, cstate->deviceHandle, &ExternalInputDetectorConfigListener);
	sshsNodeAddAttrListener(usbNode, cstate->deviceHandle, &USBConfigListener);
	sshsNodeAddAttrListener(usbNode, moduleData, &HostConfigListener);
	sshsNodeAddAttrListener(sysNode, moduleData, &HostConfigListener);
}

void caerInputDAVISCommonExit(caerModuleData moduleData) {

	if (sshsNodeGetBool(moduleData->moduleNode, "auto-restart")) {
		// Prime input module again so that it will try to restart if new devices detected.
		sshsNodePutBool(moduleData->moduleNode, "shutdown", false);
	}
}

void caerInputDAVISCommonRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
}

static void USBConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	libusb_device_handle *devHandle = userData;

	if (event == ATTRIBUTE_MODIFIED) {
		if (changeType == BOOL && str_equals(changeKey, "Run")) {
			spiConfigSend(devHandle, FPGA_USB, 0, changeValue.boolean);
		}
		else if (changeType == SHORT && str_equals(changeKey, "EarlyPacketDelay")) {
			spiConfigSend(devHandle, FPGA_USB, 1, changeValue.ushort);
		}
	}
}

static void sendUSBConfig(sshsNode moduleNode, libusb_device_handle *devHandle) {
	sshsNode usbNode = sshsGetRelativeNode(moduleNode, "usb/");

	spiConfigSend(devHandle, FPGA_USB, 1, sshsNodeGetShort(usbNode, "EarlyPacketDelay"));
	spiConfigSend(devHandle, FPGA_USB, 0, sshsNodeGetBool(usbNode, "Run"));
}

static void MultiplexerConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	libusb_device_handle *devHandle = userData;

	if (event == ATTRIBUTE_MODIFIED) {
		if (changeType == BOOL && str_equals(changeKey, "Run")) {
			spiConfigSend(devHandle, FPGA_MUX, 0, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "TimestampRun")) {
			spiConfigSend(devHandle, FPGA_MUX, 1, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "TimestampReset")) {
			spiConfigSend(devHandle, FPGA_MUX, 2, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "ForceChipBiasEnable")) {
			spiConfigSend(devHandle, FPGA_MUX, 3, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "DropDVSOnTransferStall")) {
			spiConfigSend(devHandle, FPGA_MUX, 4, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "DropAPSOnTransferStall")) {
			spiConfigSend(devHandle, FPGA_MUX, 5, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "DropIMUOnTransferStall")) {
			spiConfigSend(devHandle, FPGA_MUX, 6, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "DropExtInputOnTransferStall")) {
			spiConfigSend(devHandle, FPGA_MUX, 7, changeValue.boolean);
		}
	}
}

static void sendMultiplexerConfig(sshsNode moduleNode, libusb_device_handle *devHandle) {
	sshsNode muxNode = sshsGetRelativeNode(moduleNode, "multiplexer/");

	spiConfigSend(devHandle, FPGA_MUX, 3, sshsNodeGetBool(muxNode, "ForceChipBiasEnable"));
	spiConfigSend(devHandle, FPGA_MUX, 4, sshsNodeGetBool(muxNode, "DropDVSOnTransferStall"));
	spiConfigSend(devHandle, FPGA_MUX, 5, sshsNodeGetBool(muxNode, "DropAPSOnTransferStall"));
	spiConfigSend(devHandle, FPGA_MUX, 6, sshsNodeGetBool(muxNode, "DropIMUOnTransferStall"));
	spiConfigSend(devHandle, FPGA_MUX, 7, sshsNodeGetBool(muxNode, "DropExtInputOnTransferStall"));
	spiConfigSend(devHandle, FPGA_MUX, 1, sshsNodeGetBool(muxNode, "TimestampRun"));
	spiConfigSend(devHandle, FPGA_MUX, 0, sshsNodeGetBool(muxNode, "Run"));
}

static void DVSConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	libusb_device_handle *devHandle = userData;

	if (event == ATTRIBUTE_MODIFIED) {
		if (changeType == BOOL && str_equals(changeKey, "Run")) {
			spiConfigSend(devHandle, FPGA_DVS, 3, changeValue.boolean);
		}
		else if (changeType == BYTE && str_equals(changeKey, "AckDelayRow")) {
			spiConfigSend(devHandle, FPGA_DVS, 4, changeValue.ubyte);
		}
		else if (changeType == BYTE && str_equals(changeKey, "AckDelayColumn")) {
			spiConfigSend(devHandle, FPGA_DVS, 5, changeValue.ubyte);
		}
		else if (changeType == BYTE && str_equals(changeKey, "AckExtensionRow")) {
			spiConfigSend(devHandle, FPGA_DVS, 6, changeValue.ubyte);
		}
		else if (changeType == BYTE && str_equals(changeKey, "AckExtensionColumn")) {
			spiConfigSend(devHandle, FPGA_DVS, 7, changeValue.ubyte);
		}
		else if (changeType == BOOL && str_equals(changeKey, "WaitOnTransferStall")) {
			spiConfigSend(devHandle, FPGA_DVS, 8, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "FilterRowOnlyEvents")) {
			spiConfigSend(devHandle, FPGA_DVS, 9, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "ExternalAERControl")) {
			spiConfigSend(devHandle, FPGA_DVS, 10, changeValue.boolean);
		}
	}
}

static void sendDVSConfig(sshsNode moduleNode, libusb_device_handle *devHandle) {
	sshsNode dvsNode = sshsGetRelativeNode(moduleNode, "dvs/");

	spiConfigSend(devHandle, FPGA_DVS, 4, sshsNodeGetByte(dvsNode, "AckDelayRow"));
	spiConfigSend(devHandle, FPGA_DVS, 5, sshsNodeGetByte(dvsNode, "AckDelayColumn"));
	spiConfigSend(devHandle, FPGA_DVS, 6, sshsNodeGetByte(dvsNode, "AckExtensionRow"));
	spiConfigSend(devHandle, FPGA_DVS, 7, sshsNodeGetByte(dvsNode, "AckExtensionColumn"));
	spiConfigSend(devHandle, FPGA_DVS, 8, sshsNodeGetBool(dvsNode, "WaitOnTransferStall"));
	spiConfigSend(devHandle, FPGA_DVS, 9, sshsNodeGetBool(dvsNode, "FilterRowOnlyEvents"));
	spiConfigSend(devHandle, FPGA_DVS, 10, sshsNodeGetBool(dvsNode, "ExternalAERControl"));
	spiConfigSend(devHandle, FPGA_DVS, 3, sshsNodeGetBool(dvsNode, "Run"));
}

static void APSConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);
	davisCommonState state = userData;
	libusb_device_handle *devHandle = state->deviceHandle;

	if (event == ATTRIBUTE_MODIFIED) {
		if (changeType == BOOL && str_equals(changeKey, "Run")) {
			spiConfigSend(devHandle, FPGA_APS, 4, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "ResetRead")) {
			spiConfigSend(devHandle, FPGA_APS, 5, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "WaitOnTransferStall")) {
			spiConfigSend(devHandle, FPGA_APS, 6, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "GlobalShutter")) {
			spiConfigSend(devHandle, FPGA_APS, 8, changeValue.boolean);
		}
		else if (changeType == SHORT && str_equals(changeKey, "StartColumn0")) {
			spiConfigSend(devHandle, FPGA_APS, 9, changeValue.ushort);
			state->apsWindow0SizeX = U16T(sshsNodeGetShort(node, "EndColumn0") + 1 - changeValue.ushort);

			// Update start offset for absolute pixel position (reset map).
			state->apsWindow0StartX = changeValue.ushort;
		}
		else if (changeType == SHORT && str_equals(changeKey, "StartRow0")) {
			spiConfigSend(devHandle, FPGA_APS, 10, changeValue.ushort);
			state->apsWindow0SizeY = U16T(sshsNodeGetShort(node, "EndRow0") + 1 - changeValue.ushort);

			// Update start offset for absolute pixel position (reset map).
			state->apsWindow0StartY = changeValue.ushort;
		}
		else if (changeType == SHORT && str_equals(changeKey, "EndColumn0")) {
			spiConfigSend(devHandle, FPGA_APS, 11, changeValue.ushort);
			state->apsWindow0SizeX = U16T(changeValue.ushort + 1 - sshsNodeGetShort(node, "StartColumn0"));
		}
		else if (changeType == SHORT && str_equals(changeKey, "EndRow0")) {
			spiConfigSend(devHandle, FPGA_APS, 12, changeValue.ushort);
			state->apsWindow0SizeY = U16T(changeValue.ushort + 1 - sshsNodeGetShort(node, "StartRow0"));
		}
		else if (changeType == INT && str_equals(changeKey, "Exposure")) {
			spiConfigSend(devHandle, FPGA_APS, 13, changeValue.uint * EXT_ADC_FREQ);
		}
		else if (changeType == INT && str_equals(changeKey, "FrameDelay")) {
			spiConfigSend(devHandle, FPGA_APS, 14, changeValue.uint * EXT_ADC_FREQ);
		}
		else if (changeType == SHORT && str_equals(changeKey, "ResetSettle")) {
			spiConfigSend(devHandle, FPGA_APS, 15, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "ColumnSettle")) {
			spiConfigSend(devHandle, FPGA_APS, 16, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "RowSettle")) {
			spiConfigSend(devHandle, FPGA_APS, 17, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "NullSettle")) {
			spiConfigSend(devHandle, FPGA_APS, 18, changeValue.ushort);
		}
		else if (changeType == BOOL && str_equals(changeKey, "UseInternalADC")) {
			spiConfigSend(devHandle, FPGA_APS, 34, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "SampleEnable")) {
			spiConfigSend(devHandle, FPGA_APS, 35, changeValue.boolean);
		}
		else if (changeType == SHORT && str_equals(changeKey, "SampleSettle")) {
			spiConfigSend(devHandle, FPGA_APS, 36, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "RampReset")) {
			spiConfigSend(devHandle, FPGA_APS, 37, changeValue.ushort);
		}
		else if (changeType == BOOL && str_equals(changeKey, "RampShortReset")) {
			spiConfigSend(devHandle, FPGA_APS, 38, changeValue.boolean);
		}
	}
}

static void sendAPSConfig(sshsNode moduleNode, libusb_device_handle *devHandle) {
	sshsNode apsNode = sshsGetRelativeNode(moduleNode, "aps/");

	// GS may not exist on chips that don't have it.
	if (sshsNodeAttrExists(apsNode, "GlobalShutter", BOOL)) {
		spiConfigSend(devHandle, FPGA_APS, 8, sshsNodeGetBool(apsNode, "GlobalShutter"));
	}

	// UseInternalADC may not exist on chips that don't have integrated ADC.
	if (sshsNodeAttrExists(apsNode, "UseInternalADC", BOOL)) {
		spiConfigSend(devHandle, FPGA_APS, 34, sshsNodeGetBool(apsNode, "UseInternalADC"));
	}

	// SampleEnable may not exist on chips that don't have integrated ADC.
	if (sshsNodeAttrExists(apsNode, "SampleEnable", BOOL)) {
		spiConfigSend(devHandle, FPGA_APS, 35, sshsNodeGetBool(apsNode, "SampleEnable"));
	}

	// SampleSettle may not exist on chips that don't have integrated ADC.
	if (sshsNodeAttrExists(apsNode, "SampleSettle", SHORT)) {
		spiConfigSend(devHandle, FPGA_APS, 36, sshsNodeGetShort(apsNode, "SampleSettle"));
	}

	// RampReset may not exist on chips that don't have integrated ADC.
	if (sshsNodeAttrExists(apsNode, "RampReset", SHORT)) {
		spiConfigSend(devHandle, FPGA_APS, 37, sshsNodeGetShort(apsNode, "RampReset"));
	}

	// RampShortReset may not exist on chips that don't have integrated ADC.
	if (sshsNodeAttrExists(apsNode, "RampShortReset", BOOL)) {
		spiConfigSend(devHandle, FPGA_APS, 38, sshsNodeGetBool(apsNode, "RampShortReset"));
	}

	spiConfigSend(devHandle, FPGA_APS, 5, sshsNodeGetBool(apsNode, "ResetRead"));
	spiConfigSend(devHandle, FPGA_APS, 6, sshsNodeGetBool(apsNode, "WaitOnTransferStall"));
	spiConfigSend(devHandle, FPGA_APS, 9, sshsNodeGetShort(apsNode, "StartColumn0"));
	spiConfigSend(devHandle, FPGA_APS, 10, sshsNodeGetShort(apsNode, "StartRow0"));
	spiConfigSend(devHandle, FPGA_APS, 11, sshsNodeGetShort(apsNode, "EndColumn0"));
	spiConfigSend(devHandle, FPGA_APS, 12, sshsNodeGetShort(apsNode, "EndRow0"));
	spiConfigSend(devHandle, FPGA_APS, 13, sshsNodeGetInt(apsNode, "Exposure") * EXT_ADC_FREQ); // in µs, converted to cycles here
	spiConfigSend(devHandle, FPGA_APS, 14, sshsNodeGetInt(apsNode, "FrameDelay") * EXT_ADC_FREQ); // in µs, converted to cycles here
	spiConfigSend(devHandle, FPGA_APS, 15, sshsNodeGetShort(apsNode, "ResetSettle")); // in cycles
	spiConfigSend(devHandle, FPGA_APS, 16, sshsNodeGetShort(apsNode, "ColumnSettle")); // in cycles
	spiConfigSend(devHandle, FPGA_APS, 17, sshsNodeGetShort(apsNode, "RowSettle")); // in cycles
	spiConfigSend(devHandle, FPGA_APS, 18, sshsNodeGetShort(apsNode, "NullSettle")); // in cycles
	spiConfigSend(devHandle, FPGA_APS, 4, sshsNodeGetBool(apsNode, "Run"));

	// DAVIS RGB
	if (sshsNodeAttrExists(apsNode, "TransferTime", SHORT)) {
		spiConfigSend(devHandle, FPGA_APS, 50, sshsNodeGetShort(apsNode, "TransferTime"));
		spiConfigSend(devHandle, FPGA_APS, 51, sshsNodeGetShort(apsNode, "RSFDSettleTime"));
		spiConfigSend(devHandle, FPGA_APS, 52, sshsNodeGetShort(apsNode, "GSPDResetTime"));
		spiConfigSend(devHandle, FPGA_APS, 53, sshsNodeGetShort(apsNode, "GSResetFallTime"));
		spiConfigSend(devHandle, FPGA_APS, 54, sshsNodeGetShort(apsNode, "GSTXFallTime"));
		spiConfigSend(devHandle, FPGA_APS, 55, sshsNodeGetShort(apsNode, "GSFDResetTime"));
	}
}

static void IMUConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	libusb_device_handle *devHandle = userData;

	if (event == ATTRIBUTE_MODIFIED) {
		if (changeType == BOOL && str_equals(changeKey, "Run")) {
			spiConfigSend(devHandle, FPGA_IMU, 0, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "TempStandby")) {
			spiConfigSend(devHandle, FPGA_IMU, 1, changeValue.boolean);
		}
		else if (changeType == BOOL
			&& (str_equals(changeKey, "AccelXStandby") || str_equals(changeKey, "AccelYStandby")
				|| str_equals(changeKey, "AccelZStandby"))) {
			uint8_t accelStandby = 0;
			accelStandby |= U8T(sshsNodeGetBool(node, "AccelXStandby") << 2);
			accelStandby |= U8T(sshsNodeGetBool(node, "AccelYStandby") << 1);
			accelStandby |= U8T(sshsNodeGetBool(node, "AccelZStandby") << 0);

			spiConfigSend(devHandle, FPGA_IMU, 2, accelStandby);
		}
		else if (changeType == BOOL
			&& (str_equals(changeKey, "GyroXStandby") || str_equals(changeKey, "GyroYStandby")
				|| str_equals(changeKey, "GyroZStandby"))) {
			uint8_t gyroStandby = 0;
			gyroStandby |= U8T(sshsNodeGetBool(node, "GyroXStandby") << 2);
			gyroStandby |= U8T(sshsNodeGetBool(node, "GyroYStandby") << 1);
			gyroStandby |= U8T(sshsNodeGetBool(node, "GyroZStandby") << 0);

			spiConfigSend(devHandle, FPGA_IMU, 3, gyroStandby);
		}
		else if (changeType == BOOL && str_equals(changeKey, "LowPowerCycle")) {
			spiConfigSend(devHandle, FPGA_IMU, 4, changeValue.boolean);
		}
		else if (changeType == BYTE && str_equals(changeKey, "LowPowerWakeupFrequency")) {
			spiConfigSend(devHandle, FPGA_IMU, 5, changeValue.ubyte);
		}
		else if (changeType == BYTE && str_equals(changeKey, "SampleRateDivider")) {
			spiConfigSend(devHandle, FPGA_IMU, 6, changeValue.ubyte);
		}
		else if (changeType == BYTE && str_equals(changeKey, "DigitalLowPassFilter")) {
			spiConfigSend(devHandle, FPGA_IMU, 7, changeValue.ubyte);
		}
		else if (changeType == BYTE && str_equals(changeKey, "AccelFullScale")) {
			spiConfigSend(devHandle, FPGA_IMU, 8, changeValue.ubyte);
		}
		else if (changeType == BYTE && str_equals(changeKey, "GyroFullScale")) {
			spiConfigSend(devHandle, FPGA_IMU, 9, changeValue.ubyte);
		}
	}
}

static void sendIMUConfig(sshsNode moduleNode, libusb_device_handle *devHandle) {
	sshsNode imuNode = sshsGetRelativeNode(moduleNode, "imu/");

	uint8_t accelStandby = 0;
	accelStandby |= U8T(sshsNodeGetBool(imuNode, "AccelXStandby") << 2);
	accelStandby |= U8T(sshsNodeGetBool(imuNode, "AccelYStandby") << 1);
	accelStandby |= U8T(sshsNodeGetBool(imuNode, "AccelZStandby") << 0);

	uint8_t gyroStandby = 0;
	gyroStandby |= U8T(sshsNodeGetBool(imuNode, "GyroXStandby") << 2);
	gyroStandby |= U8T(sshsNodeGetBool(imuNode, "GyroYStandby") << 1);
	gyroStandby |= U8T(sshsNodeGetBool(imuNode, "GyroZStandby") << 0);

	spiConfigSend(devHandle, FPGA_IMU, 1, sshsNodeGetBool(imuNode, "TempStandby"));
	spiConfigSend(devHandle, FPGA_IMU, 2, accelStandby);
	spiConfigSend(devHandle, FPGA_IMU, 3, gyroStandby);
	spiConfigSend(devHandle, FPGA_IMU, 4, sshsNodeGetBool(imuNode, "LowPowerCycle"));
	spiConfigSend(devHandle, FPGA_IMU, 5, sshsNodeGetByte(imuNode, "LowPowerWakeupFrequency"));
	spiConfigSend(devHandle, FPGA_IMU, 6, sshsNodeGetByte(imuNode, "SampleRateDivider"));
	spiConfigSend(devHandle, FPGA_IMU, 7, sshsNodeGetByte(imuNode, "DigitalLowPassFilter"));
	spiConfigSend(devHandle, FPGA_IMU, 8, sshsNodeGetByte(imuNode, "AccelFullScale"));
	spiConfigSend(devHandle, FPGA_IMU, 9, sshsNodeGetByte(imuNode, "GyroFullScale"));
	spiConfigSend(devHandle, FPGA_IMU, 0, sshsNodeGetBool(imuNode, "Run"));
}

static void ExternalInputDetectorConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	libusb_device_handle *devHandle = userData;

	if (event == ATTRIBUTE_MODIFIED) {
		if (changeType == BOOL && str_equals(changeKey, "RunDetector")) {
			spiConfigSend(devHandle, FPGA_EXTINPUT, 0, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "DetectRisingEdges")) {
			spiConfigSend(devHandle, FPGA_EXTINPUT, 1, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "DetectFallingEdges")) {
			spiConfigSend(devHandle, FPGA_EXTINPUT, 2, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "DetectPulses")) {
			spiConfigSend(devHandle, FPGA_EXTINPUT, 3, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "DetectPulsePolarity")) {
			spiConfigSend(devHandle, FPGA_EXTINPUT, 4, changeValue.boolean);
		}
		else if (changeType == INT && str_equals(changeKey, "DetectPulseLength")) {
			spiConfigSend(devHandle, FPGA_EXTINPUT, 5, changeValue.uint);
		}
	}
}

static void sendExternalInputDetectorConfig(sshsNode moduleNode, libusb_device_handle *devHandle) {
	sshsNode extNode = sshsGetRelativeNode(moduleNode, "externalInput/");

	spiConfigSend(devHandle, FPGA_EXTINPUT, 1, sshsNodeGetBool(extNode, "DetectRisingEdges"));
	spiConfigSend(devHandle, FPGA_EXTINPUT, 2, sshsNodeGetBool(extNode, "DetectFallingEdges"));
	spiConfigSend(devHandle, FPGA_EXTINPUT, 3, sshsNodeGetBool(extNode, "DetectPulses"));
	spiConfigSend(devHandle, FPGA_EXTINPUT, 4, sshsNodeGetBool(extNode, "DetectPulsePolarity"));
	spiConfigSend(devHandle, FPGA_EXTINPUT, 5, sshsNodeGetInt(extNode, "DetectPulseLength"));
	spiConfigSend(devHandle, FPGA_EXTINPUT, 0, sshsNodeGetBool(extNode, "RunDetector"));
}
