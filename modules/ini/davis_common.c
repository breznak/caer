#include "davis_common.h"

static void createDefaultConfiguration(caerModuleData moduleData, struct caer_davis_info devInfo);
static void sendDefaultConfiguration(caerModuleData moduleData);
static void createVDACBiasSetting(sshsNode biasNode, const char *biasName, uint8_t currentValue, uint8_t voltageValue);
static uint16_t generateVDACBias(sshsNode biasNode, const char *biasName);
static void createCoarseFineBiasSetting(sshsNode biasNode, const char *biasName, const char *type, const char *sex,
	uint8_t coarseValue, uint8_t fineValue, bool enabled);
static uint16_t generateCoarseFineBias(sshsNode biasNode, const char *biasName);
static void createShiftedSourceBiasSetting(sshsNode biasNode, const char *biasName, uint8_t regValue, uint8_t refValue,
	const char *operatingMode, const char *voltageLevel);
static uint16_t generateShiftedSourceBias(sshsNode biasNode, const char *biasName);
static void mainloopDataNotifyIncrease(void *p);
static void mainloopDataNotifyDecrease(void *p);
static void moduleShutdownNotify(void *p);
static void biasConfigSend(sshsNode node, caerModuleData moduleData);
static void biasConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void chipConfigSend(sshsNode node, caerModuleData moduleData);
static void chipConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void muxConfigSend(sshsNode node, caerModuleData moduleData);
static void muxConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void dvsConfigSend(sshsNode node, caerModuleData moduleData);
static void dvsConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void apsConfigSend(sshsNode node, caerModuleData moduleData);
static void apsConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void imuConfigSend(sshsNode node, caerModuleData moduleData);
static void imuConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void extInputConfigSend(sshsNode node, caerModuleData moduleData);
static void extInputConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void usbConfigSend(sshsNode node, caerModuleData moduleData);
static void usbConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void systemConfigSend(sshsNode node, caerModuleData moduleData);
static void systemConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);

bool caerInputDAVISInit(caerModuleData moduleData, uint16_t deviceType) {
	caerLog(CAER_LOG_DEBUG, moduleData->moduleSubSystemString, "Initializing module ...");

	// USB port/bus/SN settings/restrictions.
	// These can be used to force connection to one specific device at startup.
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "BusNumber", 0);
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "DevAddress", 0);
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "SerialNumber", "");

	// Add auto-restart setting.
	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "Auto-Restart", true);

	/// Start data acquisition, and correctly notify mainloop of new data and module of exceptional
	// shutdown cases (device pulled, ...).
	char *serialNumber = sshsNodeGetString(moduleData->moduleNode, "SerialNumber");
	moduleData->moduleState = caerDeviceOpen(moduleData->moduleID, deviceType,
		sshsNodeGetByte(moduleData->moduleNode, "BusNumber"), sshsNodeGetByte(moduleData->moduleNode, "DevAddress"),
		serialNumber);
	free(serialNumber);

	if (moduleData->moduleState == NULL) {
		// Failed to open device.
		return (false);
	}

	// Put global source information into SSHS.
	struct caer_davis_info devInfo = caerDavisInfoGet(moduleData->moduleState);

	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");

	sshsNodePutShort(sourceInfoNode, "logicVersion", devInfo.logicVersion);
	sshsNodePutBool(sourceInfoNode, "deviceIsMaster", devInfo.deviceIsMaster);
	sshsNodePutShort(sourceInfoNode, "chipID", devInfo.chipID);

	sshsNodePutShort(sourceInfoNode, "dvsSizeX", devInfo.dvsSizeX);
	sshsNodePutShort(sourceInfoNode, "dvsSizeY", devInfo.dvsSizeY);
	sshsNodePutBool(sourceInfoNode, "dvsHasPixelFilter", devInfo.dvsHasPixelFilter);
	sshsNodePutBool(sourceInfoNode, "dvsHasBackgroundActivityFilter", devInfo.dvsHasBackgroundActivityFilter);
	sshsNodePutBool(sourceInfoNode, "dvsHasTestEventGenerator", devInfo.dvsHasTestEventGenerator);

	sshsNodePutShort(sourceInfoNode, "apsSizeX", devInfo.apsSizeX);
	sshsNodePutShort(sourceInfoNode, "apsSizeY", devInfo.apsSizeY);
	sshsNodePutByte(sourceInfoNode, "apsColorFilter", devInfo.apsColorFilter);
	sshsNodePutBool(sourceInfoNode, "apsHasGlobalShutter", devInfo.apsHasGlobalShutter);
	sshsNodePutBool(sourceInfoNode, "apsHasQuadROI", devInfo.apsHasQuadROI);
	sshsNodePutBool(sourceInfoNode, "apsHasExternalADC", devInfo.apsHasExternalADC);
	sshsNodePutBool(sourceInfoNode, "apsHasInternalADC", devInfo.apsHasInternalADC);

	sshsNodePutBool(sourceInfoNode, "extInputHasGenerator", devInfo.extInputHasGenerator);

	caerModuleSetSubSystemString(moduleData, devInfo.deviceString);

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
	createDefaultConfiguration(moduleData, devInfo);
	sendDefaultConfiguration(moduleData);

	// Start data acquisition.
	bool ret = caerDeviceDataStart(moduleData->moduleState, &mainloopDataNotifyIncrease, &mainloopDataNotifyDecrease,
	NULL, &moduleShutdownNotify, moduleData->moduleNode);

	if (!ret) {
		// Failed to start data acquisition, close device and exit.
		caerDeviceClose((caerDeviceHandle *) &moduleData->moduleState);

		return (false);
	}

	return (true);
}

void caerInputDAVISExit(caerModuleData moduleData) {
	caerDeviceDataStop(moduleData->moduleState);

	caerDeviceClose((caerDeviceHandle *) &moduleData->moduleState);

	if (sshsNodeGetBool(moduleData->moduleNode, "Auto-Restart")) {
		// Prime input module again so that it will try to restart if new devices detected.
		sshsNodePutBool(moduleData->moduleNode, "shutdown", false);
	}
}

void caerInputDAVISRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerEventPacketContainer *container = va_arg(args, caerEventPacketContainer *);

	*container = caerDeviceDataGet(moduleData->moduleState);

	if (*container != NULL) {
		caerMainloopFreeAfterLoop((void (*)(void *)) &caerEventPacketContainerFree, *container);
	}
}

static void createDefaultConfiguration(caerModuleData moduleData, struct caer_davis_info devInfo) {
	// First, always create all needed setting nodes, set their default values
	// and add their listeners.
	sshsNode biasNode = sshsGetRelativeNode(moduleData->moduleNode, "bias/");

	if (devInfo.chipID == DAVIS_CHIP_DAVIS240A || devInfo.chipID == DAVIS_CHIP_DAVIS240B
		|| devInfo.chipID == DAVIS_CHIP_DAVIS240C) {
		createCoarseFineBiasSetting(biasNode, "DiffBn", "Normal", "N", 4, 39, true);
		createCoarseFineBiasSetting(biasNode, "OnBn", "Normal", "N", 5, 255, true);
		createCoarseFineBiasSetting(biasNode, "OffBn", "Normal", "N", 4, 0, true);
		createCoarseFineBiasSetting(biasNode, "ApsCasEpc", "Cascode", "N", 5, 185, true);
		createCoarseFineBiasSetting(biasNode, "DiffCasBnc", "Cascode", "N", 5, 115, true);
		createCoarseFineBiasSetting(biasNode, "ApsROSFBn", "Normal", "N", 6, 219, true);
		createCoarseFineBiasSetting(biasNode, "LocalBufBn", "Normal", "N", 5, 164, true);
		createCoarseFineBiasSetting(biasNode, "PixInvBn", "Normal", "N", 5, 129, true);
		createCoarseFineBiasSetting(biasNode, "PrBp", "Normal", "P", 2, 58, true);
		createCoarseFineBiasSetting(biasNode, "PrSFBp", "Normal", "P", 1, 16, true);
		createCoarseFineBiasSetting(biasNode, "RefrBp", "Normal", "P", 4, 25, true);
		createCoarseFineBiasSetting(biasNode, "AEPdBn", "Normal", "N", 6, 91, true);
		createCoarseFineBiasSetting(biasNode, "LcolTimeoutBn", "Normal", "N", 5, 49, true);
		createCoarseFineBiasSetting(biasNode, "AEPuXBp", "Normal", "P", 4, 80, true);
		createCoarseFineBiasSetting(biasNode, "AEPuYBp", "Normal", "P", 7, 152, true);
		createCoarseFineBiasSetting(biasNode, "IFThrBn", "Normal", "N", 5, 255, true);
		createCoarseFineBiasSetting(biasNode, "IFRefrBn", "Normal", "N", 5, 255, true);
		createCoarseFineBiasSetting(biasNode, "PadFollBn", "Normal", "N", 7, 215, true);
		createCoarseFineBiasSetting(biasNode, "ApsOverflowLevel", "Normal", "N", 6, 253, true);

		createCoarseFineBiasSetting(biasNode, "BiasBuffer", "Normal", "N", 5, 254, true);

		createShiftedSourceBiasSetting(biasNode, "SSP", 33, 1, "ShiftedSource", "SplitGate");
		createShiftedSourceBiasSetting(biasNode, "SSN", 33, 1, "ShiftedSource", "SplitGate");
	}

	if (devInfo.chipID == DAVIS_CHIP_DAVIS640) {
		// Slow down pixels for big 640x480 array.
		createCoarseFineBiasSetting(biasNode, "PrBp", "Normal", "P", 2, 3, true);
		createCoarseFineBiasSetting(biasNode, "PrSFBp", "Normal", "P", 1, 1, true);
	}

	if (devInfo.chipID == DAVIS_CHIP_DAVIS128 || devInfo.chipID == DAVIS_CHIP_DAVIS346A
		|| devInfo.chipID == DAVIS_CHIP_DAVIS346B || devInfo.chipID == DAVIS_CHIP_DAVIS346C
		|| devInfo.chipID == DAVIS_CHIP_DAVIS640 || devInfo.chipID == DAVIS_CHIP_DAVIS208) {
		createVDACBiasSetting(biasNode, "ApsOverflowLevel", 6, 27);
		createVDACBiasSetting(biasNode, "ApsCas", 6, 21);
		createVDACBiasSetting(biasNode, "AdcRefHigh", 7, 30);
		createVDACBiasSetting(biasNode, "AdcRefLow", 7, 1);
		if (devInfo.chipID == DAVIS_CHIP_DAVIS346A || devInfo.chipID == DAVIS_CHIP_DAVIS346B
			|| devInfo.chipID == DAVIS_CHIP_DAVIS346C || devInfo.chipID == DAVIS_CHIP_DAVIS640) {
			// Only DAVIS346 and 640 have ADC testing.
			createVDACBiasSetting(biasNode, "AdcTestVoltage", 7, 21);
		}

		createCoarseFineBiasSetting(biasNode, "LocalBufBn", "Normal", "N", 5, 164, true);
		createCoarseFineBiasSetting(biasNode, "PadFollBn", "Normal", "N", 7, 215, true);
		createCoarseFineBiasSetting(biasNode, "DiffBn", "Normal", "N", 4, 39, true);
		createCoarseFineBiasSetting(biasNode, "OnBn", "Normal", "N", 5, 255, true);
		createCoarseFineBiasSetting(biasNode, "OffBn", "Normal", "N", 4, 1, true);
		createCoarseFineBiasSetting(biasNode, "PixInvBn", "Normal", "N", 5, 129, true);
		createCoarseFineBiasSetting(biasNode, "PrBp", "Normal", "P", 2, 58, true);
		createCoarseFineBiasSetting(biasNode, "PrSFBp", "Normal", "P", 1, 16, true);
		createCoarseFineBiasSetting(biasNode, "RefrBp", "Normal", "P", 4, 25, true);
		createCoarseFineBiasSetting(biasNode, "ReadoutBufBp", "Normal", "P", 6, 20, true);
		createCoarseFineBiasSetting(biasNode, "ApsROSFBn", "Normal", "N", 6, 219, true);
		createCoarseFineBiasSetting(biasNode, "AdcCompBp", "Normal", "P", 5, 20, true);
		createCoarseFineBiasSetting(biasNode, "ColSelLowBn", "Normal", "N", 0, 1, true);
		createCoarseFineBiasSetting(biasNode, "DACBufBp", "Normal", "P", 6, 60, true);
		createCoarseFineBiasSetting(biasNode, "LcolTimeoutBn", "Normal", "N", 5, 49, true);
		createCoarseFineBiasSetting(biasNode, "AEPdBn", "Normal", "N", 6, 91, true);
		createCoarseFineBiasSetting(biasNode, "AEPuXBp", "Normal", "P", 4, 80, true);
		createCoarseFineBiasSetting(biasNode, "AEPuYBp", "Normal", "P", 7, 152, true);
		createCoarseFineBiasSetting(biasNode, "IFRefrBn", "Normal", "N", 5, 255, true);
		createCoarseFineBiasSetting(biasNode, "IFThrBn", "Normal", "N", 5, 255, true);

		createCoarseFineBiasSetting(biasNode, "BiasBuffer", "Normal", "N", 5, 254, true);

		createShiftedSourceBiasSetting(biasNode, "SSP", 33, 1, "ShiftedSource", "SplitGate");
		createShiftedSourceBiasSetting(biasNode, "SSN", 33, 1, "ShiftedSource", "SplitGate");
	}

	if (devInfo.chipID == DAVIS_CHIP_DAVIS208) {
		createVDACBiasSetting(biasNode, "ResetHighPass", 7, 63);
		createVDACBiasSetting(biasNode, "RefSS", 5, 11);

		createCoarseFineBiasSetting(biasNode, "RegBiasBp", "Normal", "P", 5, 20, true);
		createCoarseFineBiasSetting(biasNode, "RefSSBn", "Normal", "N", 5, 20, true);
	}

	if (devInfo.chipID == DAVIS_CHIP_DAVISRGB) {
		createVDACBiasSetting(biasNode, "ApsCas", 4, 21);
		createVDACBiasSetting(biasNode, "OVG1Lo", 4, 21);
		createVDACBiasSetting(biasNode, "OVG2Lo", 0, 0);
		createVDACBiasSetting(biasNode, "TX2OVG2Hi", 0, 63);
		createVDACBiasSetting(biasNode, "Gnd07", 4, 13);
		createVDACBiasSetting(biasNode, "AdcTestVoltage", 0, 21);
		createVDACBiasSetting(biasNode, "AdcRefHigh", 7, 63);
		createVDACBiasSetting(biasNode, "AdcRefLow", 7, 0);

		createCoarseFineBiasSetting(biasNode, "IFRefrBn", "Normal", "N", 5, 255, false);
		createCoarseFineBiasSetting(biasNode, "IFThrBn", "Normal", "N", 5, 255, false);
		createCoarseFineBiasSetting(biasNode, "LocalBufBn", "Normal", "N", 5, 164, false);
		createCoarseFineBiasSetting(biasNode, "PadFollBn", "Normal", "N", 7, 209, false);
		createCoarseFineBiasSetting(biasNode, "PixInvBn", "Normal", "N", 4, 164, true);
		createCoarseFineBiasSetting(biasNode, "DiffBn", "Normal", "N", 4, 54, true);
		createCoarseFineBiasSetting(biasNode, "OnBn", "Normal", "N", 6, 63, true);
		createCoarseFineBiasSetting(biasNode, "OffBn", "Normal", "N", 2, 138, true);
		createCoarseFineBiasSetting(biasNode, "PrBp", "Normal", "P", 1, 108, true);
		createCoarseFineBiasSetting(biasNode, "PrSFBp", "Normal", "P", 1, 108, true);
		createCoarseFineBiasSetting(biasNode, "RefrBp", "Normal", "P", 4, 28, true);
		createCoarseFineBiasSetting(biasNode, "ArrayBiasBufferBn", "Normal", "N", 6, 128, true);
		createCoarseFineBiasSetting(biasNode, "ArrayLogicBufferBn", "Normal", "N", 5, 255, true);
		createCoarseFineBiasSetting(biasNode, "FalltimeBn", "Normal", "N", 7, 41, true);
		createCoarseFineBiasSetting(biasNode, "RisetimeBp", "Normal", "P", 6, 162, true);
		createCoarseFineBiasSetting(biasNode, "ReadoutBufBp", "Normal", "P", 6, 20, false);
		createCoarseFineBiasSetting(biasNode, "ApsROSFBn", "Normal", "N", 6, 255, true);
		createCoarseFineBiasSetting(biasNode, "AdcCompBp", "Normal", "P", 4, 159, true);
		createCoarseFineBiasSetting(biasNode, "DACBufBp", "Normal", "P", 6, 194, true);
		createCoarseFineBiasSetting(biasNode, "LcolTimeoutBn", "Normal", "N", 5, 49, true);
		createCoarseFineBiasSetting(biasNode, "AEPdBn", "Normal", "N", 6, 91, true);
		createCoarseFineBiasSetting(biasNode, "AEPuXBp", "Normal", "P", 4, 80, true);
		createCoarseFineBiasSetting(biasNode, "AEPuYBp", "Normal", "P", 7, 152, true);

		createCoarseFineBiasSetting(biasNode, "BiasBuffer", "Normal", "N", 6, 251, true);

		createShiftedSourceBiasSetting(biasNode, "SSP", 33, 1, "TiedToRail", "SplitGate");
		createShiftedSourceBiasSetting(biasNode, "SSN", 33, 2, "ShiftedSource", "SplitGate");
	}

	sshsNode chipNode = sshsGetRelativeNode(moduleData->moduleNode, "chip/");

	sshsNodePutByteIfAbsent(chipNode, "DigitalMux0", 0);
	sshsNodePutByteIfAbsent(chipNode, "DigitalMux1", 0);
	sshsNodePutByteIfAbsent(chipNode, "DigitalMux2", 0);
	sshsNodePutByteIfAbsent(chipNode, "DigitalMux3", 0);
	sshsNodePutByteIfAbsent(chipNode, "AnalogMux0", 0);
	sshsNodePutByteIfAbsent(chipNode, "AnalogMux1", 0);
	sshsNodePutByteIfAbsent(chipNode, "AnalogMux2", 0);
	sshsNodePutByteIfAbsent(chipNode, "BiasMux0", 0);

	sshsNodePutBoolIfAbsent(chipNode, "ResetCalibNeuron", true);
	sshsNodePutBoolIfAbsent(chipNode, "TypeNCalibNeuron", false);
	sshsNodePutBoolIfAbsent(chipNode, "ResetTestPixel", true);
	sshsNodePutBoolIfAbsent(chipNode, "AERnArow", false); // Use nArow in the AER state machine.
	sshsNodePutBoolIfAbsent(chipNode, "UseAOut", false); // Enable analog pads for aMUX output (testing).

	if (devInfo.chipID == DAVIS_CHIP_DAVIS240A || devInfo.chipID == DAVIS_CHIP_DAVIS240B) {
		sshsNodePutBoolIfAbsent(chipNode, "SpecialPixelControl", false);
	}

	if (devInfo.chipID == DAVIS_CHIP_DAVIS128 || devInfo.chipID == DAVIS_CHIP_DAVIS208
		|| devInfo.chipID == DAVIS_CHIP_DAVIS346A || devInfo.chipID == DAVIS_CHIP_DAVIS346B
		|| devInfo.chipID == DAVIS_CHIP_DAVIS346C || devInfo.chipID == DAVIS_CHIP_DAVIS640
		|| devInfo.chipID == DAVIS_CHIP_DAVISRGB) {
		// Select which grey counter to use with the internal ADC: '0' means the external grey counter is used, which
		// has to be supplied off-chip. '1' means the on-chip grey counter is used instead.
		sshsNodePutBoolIfAbsent(chipNode, "SelectGrayCounter", true);
	}

	if (devInfo.chipID == DAVIS_CHIP_DAVIS346A || devInfo.chipID == DAVIS_CHIP_DAVIS346B
		|| devInfo.chipID == DAVIS_CHIP_DAVIS346C || devInfo.chipID == DAVIS_CHIP_DAVIS640
		|| devInfo.chipID == DAVIS_CHIP_DAVISRGB) {
		// Test ADC functionality: if true, the ADC takes its input voltage not from the pixel, but from the
		// VDAC 'AdcTestVoltage'. If false, the voltage comes from the pixels.
		sshsNodePutBoolIfAbsent(chipNode, "TestADC", false);
	}

	if (devInfo.chipID == DAVIS_CHIP_DAVIS208) {
		sshsNodePutBoolIfAbsent(chipNode, "SelectPreAmpAvg", false);
		sshsNodePutBoolIfAbsent(chipNode, "SelectBiasRefSS", false);
		sshsNodePutBoolIfAbsent(chipNode, "SelectSense", true);
		sshsNodePutBoolIfAbsent(chipNode, "SelectPosFb", false);
		sshsNodePutBoolIfAbsent(chipNode, "SelectHighPass", false);
	}

	if (devInfo.chipID == DAVIS_CHIP_DAVISRGB) {
		sshsNodePutBoolIfAbsent(chipNode, "AdjustOVG1Lo", true);
		sshsNodePutBoolIfAbsent(chipNode, "AdjustOVG2Lo", false);
		sshsNodePutBoolIfAbsent(chipNode, "AdjustTX2OVG2Hi", false);
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

	if (devInfo.dvsHasPixelFilter) {
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel0Row", devInfo.apsSizeY);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel0Column", devInfo.apsSizeX);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel1Row", devInfo.apsSizeY);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel1Column", devInfo.apsSizeX);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel2Row", devInfo.apsSizeY);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel2Column", devInfo.apsSizeX);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel3Row", devInfo.apsSizeY);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel3Column", devInfo.apsSizeX);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel4Row", devInfo.apsSizeY);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel4Column", devInfo.apsSizeX);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel5Row", devInfo.apsSizeY);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel5Column", devInfo.apsSizeX);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel6Row", devInfo.apsSizeY);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel6Column", devInfo.apsSizeX);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel7Row", devInfo.apsSizeY);
		sshsNodePutShortIfAbsent(dvsNode, "FilterPixel7Column", devInfo.apsSizeX);
	}

	if (devInfo.dvsHasBackgroundActivityFilter) {
		sshsNodePutBoolIfAbsent(dvsNode, "FilterBackgroundActivity", true);
		sshsNodePutIntIfAbsent(dvsNode, "FilterBackgroundActivityDeltaTime", 20000);
	}

	if (devInfo.dvsHasTestEventGenerator) {
		sshsNodePutBoolIfAbsent(dvsNode, "TestEventGeneratorEnable", false);
	}

	// Subsystem 2: APS ADC
	sshsNode apsNode = sshsGetRelativeNode(moduleData->moduleNode, "aps/");

	// Only support GS on chips that have it available.

	if (devInfo.apsHasGlobalShutter) {
		sshsNodePutBoolIfAbsent(apsNode, "GlobalShutter", true);
	}

	sshsNodePutBoolIfAbsent(apsNode, "Run", true);
	sshsNodePutBoolIfAbsent(apsNode, "ResetRead", true);
	sshsNodePutBoolIfAbsent(apsNode, "WaitOnTransferStall", true);
	sshsNodePutShortIfAbsent(apsNode, "StartColumn0", 0);
	sshsNodePutShortIfAbsent(apsNode, "StartRow0", 0);
	sshsNodePutShortIfAbsent(apsNode, "EndColumn0", U16T(devInfo.apsSizeX - 1));
	sshsNodePutShortIfAbsent(apsNode, "EndRow0", U16T(devInfo.apsSizeY - 1));
	sshsNodePutIntIfAbsent(apsNode, "Exposure", 4000); // in µs, converted to cycles later
	sshsNodePutIntIfAbsent(apsNode, "FrameDelay", 1000); // in µs, converted to cycles later
	sshsNodePutShortIfAbsent(apsNode, "ResetSettle", 10); // in cycles
	sshsNodePutShortIfAbsent(apsNode, "ColumnSettle", 30); // in cycles
	sshsNodePutShortIfAbsent(apsNode, "RowSettle", 8); // in cycles
	sshsNodePutShortIfAbsent(apsNode, "NullSettle", 3); // in cycles

	if (devInfo.apsHasQuadROI) {
		sshsNodePutShortIfAbsent(apsNode, "StartColumn1", devInfo.apsSizeX);
		sshsNodePutShortIfAbsent(apsNode, "StartRow1", devInfo.apsSizeY);
		sshsNodePutShortIfAbsent(apsNode, "EndColumn1", devInfo.apsSizeX);
		sshsNodePutShortIfAbsent(apsNode, "EndRow1", devInfo.apsSizeY);
		sshsNodePutShortIfAbsent(apsNode, "StartColumn2", devInfo.apsSizeX);
		sshsNodePutShortIfAbsent(apsNode, "StartRow2", devInfo.apsSizeY);
		sshsNodePutShortIfAbsent(apsNode, "EndColumn2", devInfo.apsSizeX);
		sshsNodePutShortIfAbsent(apsNode, "EndRow2", devInfo.apsSizeY);
		sshsNodePutShortIfAbsent(apsNode, "StartColumn3", devInfo.apsSizeX);
		sshsNodePutShortIfAbsent(apsNode, "StartRow3", devInfo.apsSizeY);
		sshsNodePutShortIfAbsent(apsNode, "EndColumn3", devInfo.apsSizeX);
		sshsNodePutShortIfAbsent(apsNode, "EndRow3", devInfo.apsSizeY);
	}

	if (devInfo.apsHasInternalADC) {
		sshsNodePutBoolIfAbsent(apsNode, "UseInternalADC", true);
		sshsNodePutBoolIfAbsent(apsNode, "SampleEnable", true);
		sshsNodePutShortIfAbsent(apsNode, "SampleSettle", 60); // in cycles
		sshsNodePutShortIfAbsent(apsNode, "RampReset", 10); // in cycles
		sshsNodePutBoolIfAbsent(apsNode, "RampShortReset", true);
	}

	// DAVIS RGB has additional timing counters.
	if (devInfo.chipID == DAVIS_CHIP_DAVISRGB) {
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

	if (devInfo.extInputHasGenerator) {
		sshsNodePutBoolIfAbsent(extNode, "RunGenerator", false);
		sshsNodePutBoolIfAbsent(extNode, "GenerateUseCustomSignal", false);
		sshsNodePutBoolIfAbsent(extNode, "GeneratePulsePolarity", true);
		sshsNodePutIntIfAbsent(extNode, "GeneratePulseInterval", 10);
		sshsNodePutIntIfAbsent(extNode, "GeneratePulseLength", 5);
	}

	// Subsystem 9: FX2/3 USB Configuration and USB buffer settings.
	sshsNode usbNode = sshsGetRelativeNode(moduleData->moduleNode, "usb/");
	sshsNodePutBoolIfAbsent(usbNode, "Run", true);
	sshsNodePutShortIfAbsent(usbNode, "EarlyPacketDelay", 8); // 125µs time-slices, so 1ms

	sshsNodePutIntIfAbsent(usbNode, "BufferNumber", 8);
	sshsNodePutIntIfAbsent(usbNode, "BufferSize", 8192);

	sshsNodeAddAttrListener(usbNode, moduleData, &usbConfigListener);

	sshsNode sysNode = sshsGetRelativeNode(moduleData->moduleNode, "system/");

	// Packet settings (size (in events) and time interval (in µs)).
	sshsNodePutIntIfAbsent(sysNode, "PacketContainerMaxSize", 4096 + 128);
	sshsNodePutIntIfAbsent(sysNode, "PacketContainerMaxInterval", 5000);
	sshsNodePutIntIfAbsent(sysNode, "PolarityPacketMaxSize", 4096);
	sshsNodePutIntIfAbsent(sysNode, "PolarityPacketMaxInterval", 5000);
	sshsNodePutIntIfAbsent(sysNode, "SpecialPacketMaxSize", 128);
	sshsNodePutIntIfAbsent(sysNode, "SpecialPacketMaxInterval", 1000);
	sshsNodePutIntIfAbsent(sysNode, "FramePacketMaxSize", 4);
	sshsNodePutIntIfAbsent(sysNode, "FramePacketMaxInterval", 20000);
	sshsNodePutIntIfAbsent(sysNode, "IMU6PacketMaxSize", 32);
	sshsNodePutIntIfAbsent(sysNode, "IMU6PacketMaxInterval", 4000);

	// Ring-buffer setting (only changes value on module init/shutdown cycles).
	sshsNodePutIntIfAbsent(sysNode, "DataExchangeBufferSize", 64);

	sshsNodeAddAttrListener(sysNode, moduleData, &systemConfigListener);
}

static void sendDefaultConfiguration(caerModuleData moduleData) {
	// Send cAER configuration to libcaer and device.
	biasConfigSend(sshsGetRelativeNode(moduleData->moduleNode, "bias/"), moduleData);
	chipConfigSend(sshsGetRelativeNode(moduleData->moduleNode, "chip/"), moduleData);
	systemConfigSend(sshsGetRelativeNode(moduleData->moduleNode, "system/"), moduleData);
	usbConfigSend(sshsGetRelativeNode(moduleData->moduleNode, "usb/"), moduleData);
	muxConfigSend(sshsGetRelativeNode(moduleData->moduleNode, "multiplexer/"), moduleData);
	dvsConfigSend(sshsGetRelativeNode(moduleData->moduleNode, "dvs/"), moduleData);
	apsConfigSend(sshsGetRelativeNode(moduleData->moduleNode, "aps/"), moduleData);
	imuConfigSend(sshsGetRelativeNode(moduleData->moduleNode, "imu/"), moduleData);
	extInputConfigSend(sshsGetRelativeNode(moduleData->moduleNode, "externalInput/"), moduleData);
}

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

static void mainloopDataNotifyIncrease(void *p) {
	UNUSED_ARGUMENT(p);

	caerMainloopDataAvailableIncrease();
}

static void mainloopDataNotifyDecrease(void *p) {
	UNUSED_ARGUMENT(p);

	caerMainloopDataAvailableDecrease();
}

static void moduleShutdownNotify(void *p) {
	sshsNode moduleNode = p;

	// Ensure parent also shuts down (on disconnected device for example).
	sshsNodePutBool(moduleNode, "shutdown", true);
}

static void biasConfigSend(sshsNode node, caerModuleData moduleData) {

}

static void biasConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerModuleData moduleData = userData;

}

static void chipConfigSend(sshsNode node, caerModuleData moduleData) {

}

static void chipConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerModuleData moduleData = userData;

}

static void muxConfigSend(sshsNode node, caerModuleData moduleData) {

}

static void muxConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerModuleData moduleData = userData;

}

static void dvsConfigSend(sshsNode node, caerModuleData moduleData) {

}

static void dvsConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerModuleData moduleData = userData;

}

static void apsConfigSend(sshsNode node, caerModuleData moduleData) {

}

static void apsConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerModuleData moduleData = userData;

}

static void imuConfigSend(sshsNode node, caerModuleData moduleData) {

}

static void imuConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerModuleData moduleData = userData;

}

static void extInputConfigSend(sshsNode node, caerModuleData moduleData) {

}

static void extInputConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerModuleData moduleData = userData;

}

static void usbConfigSend(sshsNode node, caerModuleData moduleData) {
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_USB, CAER_HOST_CONFIG_USB_BUFFER_NUMBER,
		sshsNodeGetInt(node, "BufferNumber"));
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_USB, CAER_HOST_CONFIG_USB_BUFFER_SIZE,
		sshsNodeGetInt(node, "BufferSize"));
}

static void usbConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerModuleData moduleData = userData;

	if (event == ATTRIBUTE_MODIFIED) {
		if (changeType == INT && caerStrEquals(changeKey, "BufferNumber")) {
			caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_USB, CAER_HOST_CONFIG_USB_BUFFER_NUMBER,
				changeValue.uint);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "BufferSize")) {
			caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_USB, CAER_HOST_CONFIG_USB_BUFFER_SIZE,
				changeValue.uint);
		}
	}
}

static void systemConfigSend(sshsNode node, caerModuleData moduleData) {
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS, CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_SIZE,
		sshsNodeGetInt(node, "PacketContainerMaxSize"));
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS,
	CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_INTERVAL, sshsNodeGetInt(node, "PacketContainerMaxInterval"));
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS, CAER_HOST_CONFIG_PACKETS_MAX_POLARITY_SIZE,
		sshsNodeGetInt(node, "PolarityPacketMaxSize"));
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS,
	CAER_HOST_CONFIG_PACKETS_MAX_POLARITY_INTERVAL, sshsNodeGetInt(node, "PolarityPacketMaxInterval"));
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS, CAER_HOST_CONFIG_PACKETS_MAX_SPECIAL_SIZE,
		sshsNodeGetInt(node, "SpecialPacketMaxSize"));
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS,
	CAER_HOST_CONFIG_PACKETS_MAX_SPECIAL_INTERVAL, sshsNodeGetInt(node, "SpecialPacketMaxInterval"));

	// Changes only take effect on module start!
	caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_DATAEXCHANGE,
	CAER_HOST_CONFIG_DATAEXCHANGE_BUFFER_SIZE, sshsNodeGetInt(node, "DataExchangeBufferSize"));
}

static void systemConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	caerModuleData moduleData = userData;

	if (event == ATTRIBUTE_MODIFIED) {
		if (changeType == INT && caerStrEquals(changeKey, "PacketContainerMaxSize")) {
			caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS,
			CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_SIZE, changeValue.uint);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "PacketContainerMaxInterval")) {
			caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS,
			CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_INTERVAL, changeValue.uint);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "PolarityPacketMaxSize")) {
			caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS,
			CAER_HOST_CONFIG_PACKETS_MAX_POLARITY_SIZE, changeValue.uint);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "PolarityPacketMaxInterval")) {
			caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS,
			CAER_HOST_CONFIG_PACKETS_MAX_POLARITY_INTERVAL, changeValue.uint);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "SpecialPacketMaxSize")) {
			caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS,
			CAER_HOST_CONFIG_PACKETS_MAX_SPECIAL_SIZE, changeValue.uint);
		}
		else if (changeType == INT && caerStrEquals(changeKey, "SpecialPacketMaxInterval")) {
			caerDeviceConfigSet(moduleData->moduleState, CAER_HOST_CONFIG_PACKETS,
			CAER_HOST_CONFIG_PACKETS_MAX_SPECIAL_INTERVAL, changeValue.uint);
		}
	}
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

static void DVSFilterConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	libusb_device_handle *devHandle = userData;

	if (event == ATTRIBUTE_MODIFIED) {
		if (changeType == SHORT && str_equals(changeKey, "FilterPixel0Row")) {
			spiConfigSend(devHandle, FPGA_DVS, 12, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel0Column")) {
			spiConfigSend(devHandle, FPGA_DVS, 13, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel1Row")) {
			spiConfigSend(devHandle, FPGA_DVS, 14, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel1Column")) {
			spiConfigSend(devHandle, FPGA_DVS, 15, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel2Row")) {
			spiConfigSend(devHandle, FPGA_DVS, 16, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel2Column")) {
			spiConfigSend(devHandle, FPGA_DVS, 17, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel3Row")) {
			spiConfigSend(devHandle, FPGA_DVS, 18, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel3Column")) {
			spiConfigSend(devHandle, FPGA_DVS, 19, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel4Row")) {
			spiConfigSend(devHandle, FPGA_DVS, 20, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel4Column")) {
			spiConfigSend(devHandle, FPGA_DVS, 21, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel5Row")) {
			spiConfigSend(devHandle, FPGA_DVS, 22, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel5Column")) {
			spiConfigSend(devHandle, FPGA_DVS, 23, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel6Row")) {
			spiConfigSend(devHandle, FPGA_DVS, 24, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel6Column")) {
			spiConfigSend(devHandle, FPGA_DVS, 25, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel7Row")) {
			spiConfigSend(devHandle, FPGA_DVS, 26, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel7Column")) {
			spiConfigSend(devHandle, FPGA_DVS, 27, changeValue.ushort);
		}
		else if (changeType == BOOL && str_equals(changeKey, "FilterBackgroundActivity")) {
			spiConfigSend(devHandle, FPGA_DVS, 29, changeValue.boolean);
		}
		else if (changeType == INT && str_equals(changeKey, "FilterBackgroundActivityDeltaTime")) {
			spiConfigSend(devHandle, FPGA_DVS, 30, changeValue.uint);
		}
		else if (changeType == BOOL && str_equals(changeKey, "TestEventGeneratorEnable")) {
			spiConfigSend(devHandle, FPGA_DVS, 32, changeValue.boolean);
		}
	}
}

static void sendDVSFilterConfig(sshsNode moduleNode, libusb_device_handle *devHandle) {
	sshsNode dvsNode = sshsGetRelativeNode(moduleNode, "dvs/");

	spiConfigSend(devHandle, FPGA_DVS, 12, sshsNodeGetShort(dvsNode, "FilterPixel0Row"));
	spiConfigSend(devHandle, FPGA_DVS, 13, sshsNodeGetShort(dvsNode, "FilterPixel0Column"));
	spiConfigSend(devHandle, FPGA_DVS, 14, sshsNodeGetShort(dvsNode, "FilterPixel1Row"));
	spiConfigSend(devHandle, FPGA_DVS, 15, sshsNodeGetShort(dvsNode, "FilterPixel1Column"));
	spiConfigSend(devHandle, FPGA_DVS, 16, sshsNodeGetShort(dvsNode, "FilterPixel2Row"));
	spiConfigSend(devHandle, FPGA_DVS, 17, sshsNodeGetShort(dvsNode, "FilterPixel2Column"));
	spiConfigSend(devHandle, FPGA_DVS, 18, sshsNodeGetShort(dvsNode, "FilterPixel3Row"));
	spiConfigSend(devHandle, FPGA_DVS, 19, sshsNodeGetShort(dvsNode, "FilterPixel3Column"));
	spiConfigSend(devHandle, FPGA_DVS, 20, sshsNodeGetShort(dvsNode, "FilterPixel4Row"));
	spiConfigSend(devHandle, FPGA_DVS, 21, sshsNodeGetShort(dvsNode, "FilterPixel4Column"));
	spiConfigSend(devHandle, FPGA_DVS, 22, sshsNodeGetShort(dvsNode, "FilterPixel5Row"));
	spiConfigSend(devHandle, FPGA_DVS, 23, sshsNodeGetShort(dvsNode, "FilterPixel5Column"));
	spiConfigSend(devHandle, FPGA_DVS, 24, sshsNodeGetShort(dvsNode, "FilterPixel6Row"));
	spiConfigSend(devHandle, FPGA_DVS, 25, sshsNodeGetShort(dvsNode, "FilterPixel6Column"));
	spiConfigSend(devHandle, FPGA_DVS, 26, sshsNodeGetShort(dvsNode, "FilterPixel7Row"));
	spiConfigSend(devHandle, FPGA_DVS, 27, sshsNodeGetShort(dvsNode, "FilterPixel7Column"));
	spiConfigSend(devHandle, FPGA_DVS, 29, sshsNodeGetBool(dvsNode, "FilterBackgroundActivity"));
	spiConfigSend(devHandle, FPGA_DVS, 30, sshsNodeGetInt(dvsNode, "FilterBackgroundActivityDeltaTime"));
	spiConfigSend(devHandle, FPGA_DVS, 32, sshsNodeGetBool(dvsNode, "TestEventGeneratorEnable"));
}

static void APSQuadROIConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	libusb_device_handle *devHandle = userData;

	if (event == ATTRIBUTE_MODIFIED) {
		if (changeType == SHORT && str_equals(changeKey, "StartColumn1")) {
			spiConfigSend(devHandle, FPGA_APS, 20, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "StartRow1")) {
			spiConfigSend(devHandle, FPGA_APS, 21, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "EndColumn1")) {
			spiConfigSend(devHandle, FPGA_APS, 22, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "EndRow1")) {
			spiConfigSend(devHandle, FPGA_APS, 23, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "StartColumn2")) {
			spiConfigSend(devHandle, FPGA_APS, 24, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "StartRow2")) {
			spiConfigSend(devHandle, FPGA_APS, 25, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "EndColumn2")) {
			spiConfigSend(devHandle, FPGA_APS, 26, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "EndRow2")) {
			spiConfigSend(devHandle, FPGA_APS, 27, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "StartColumn3")) {
			spiConfigSend(devHandle, FPGA_APS, 28, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "StartRow3")) {
			spiConfigSend(devHandle, FPGA_APS, 29, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "EndColumn3")) {
			spiConfigSend(devHandle, FPGA_APS, 30, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "EndRow3")) {
			spiConfigSend(devHandle, FPGA_APS, 31, changeValue.ushort);
		}
	}
}

static void sendAPSQuadROIConfig(sshsNode moduleNode, libusb_device_handle *devHandle) {
	sshsNode apsNode = sshsGetRelativeNode(moduleNode, "aps/");

	spiConfigSend(devHandle, FPGA_APS, 20, sshsNodeGetShort(apsNode, "StartColumn1"));
	spiConfigSend(devHandle, FPGA_APS, 21, sshsNodeGetShort(apsNode, "StartRow1"));
	spiConfigSend(devHandle, FPGA_APS, 22, sshsNodeGetShort(apsNode, "EndColumn1"));
	spiConfigSend(devHandle, FPGA_APS, 23, sshsNodeGetShort(apsNode, "EndRow1"));
	spiConfigSend(devHandle, FPGA_APS, 24, sshsNodeGetShort(apsNode, "StartColumn2"));
	spiConfigSend(devHandle, FPGA_APS, 25, sshsNodeGetShort(apsNode, "StartRow2"));
	spiConfigSend(devHandle, FPGA_APS, 26, sshsNodeGetShort(apsNode, "EndColumn2"));
	spiConfigSend(devHandle, FPGA_APS, 27, sshsNodeGetShort(apsNode, "EndRow2"));
	spiConfigSend(devHandle, FPGA_APS, 28, sshsNodeGetShort(apsNode, "StartColumn3"));
	spiConfigSend(devHandle, FPGA_APS, 29, sshsNodeGetShort(apsNode, "StartRow3"));
	spiConfigSend(devHandle, FPGA_APS, 30, sshsNodeGetShort(apsNode, "EndColumn3"));
	spiConfigSend(devHandle, FPGA_APS, 31, sshsNodeGetShort(apsNode, "EndRow3"));
}

static void ExternalInputGeneratorConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	libusb_device_handle *devHandle = userData;

	if (event == ATTRIBUTE_MODIFIED) {
		if (changeType == BOOL && str_equals(changeKey, "RunGenerator")) {
			spiConfigSend(devHandle, FPGA_EXTINPUT, 7, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "GenerateUseCustomSignal")) {
			spiConfigSend(devHandle, FPGA_EXTINPUT, 8, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "GeneratePulsePolarity")) {
			spiConfigSend(devHandle, FPGA_EXTINPUT, 9, changeValue.boolean);
		}
		else if (changeType == INT && str_equals(changeKey, "GeneratePulseInterval")) {
			spiConfigSend(devHandle, FPGA_EXTINPUT, 10, changeValue.uint);
		}
		else if (changeType == INT && str_equals(changeKey, "GeneratePulseLength")) {
			spiConfigSend(devHandle, FPGA_EXTINPUT, 11, changeValue.uint);
		}
	}
}

static void sendExternalInputGeneratorConfig(sshsNode moduleNode, libusb_device_handle *devHandle) {
	sshsNode extNode = sshsGetRelativeNode(moduleNode, "externalInput/");

	spiConfigSend(devHandle, FPGA_EXTINPUT, 8, sshsNodeGetBool(extNode, "GenerateUseCustomSignal"));
	spiConfigSend(devHandle, FPGA_EXTINPUT, 9, sshsNodeGetBool(extNode, "GeneratePulsePolarity"));
	spiConfigSend(devHandle, FPGA_EXTINPUT, 10, sshsNodeGetInt(extNode, "GeneratePulseInterval"));
	spiConfigSend(devHandle, FPGA_EXTINPUT, 11, sshsNodeGetInt(extNode, "GeneratePulseLength"));
	spiConfigSend(devHandle, FPGA_EXTINPUT, 7, sshsNodeGetBool(extNode, "RunGenerator"));
}

