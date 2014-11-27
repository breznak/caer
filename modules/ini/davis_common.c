#include "davis_common.h"
#include <pthread.h>
#include <unistd.h>

#define CHECK_MONOTONIC_TIMESTAMP(CURR_TS, LAST_TS) \
	if (CURR_TS <= LAST_TS) { \
		caerLog(LOG_ALERT, state->sourceSubSystemString, \
			"DAVISFX3: non-monotonic time-stamp detected: lastTimestamp=%" PRIu32 ", currentTimestamp=%" PRIu32 ", difference=%" PRIu32 ".", \
			LAST_TS, CURR_TS, (LAST_TS - CURR_TS)); \
	}

static void LIBUSB_CALL libUsbDataCallback(struct libusb_transfer *transfer);
static void dataTranslator(davisCommonState state, uint8_t *buffer, size_t bytesSent);

void freeAllMemory(davisCommonState state) {
	if (state->currentPolarityPacket != NULL) {
		free(state->currentPolarityPacket);
		state->currentPolarityPacket = NULL;
	}

	if (state->currentFramePacket != NULL) {
		free(state->currentFramePacket);
		state->currentFramePacket = NULL;
	}

	if (state->currentIMU6Packet != NULL) {
		free(state->currentIMU6Packet);
		state->currentIMU6Packet = NULL;
	}

	if (state->currentSpecialPacket != NULL) {
		free(state->currentSpecialPacket);
		state->currentSpecialPacket = NULL;
	}

	if (state->apsCurrentResetFrame != NULL) {
		free(state->apsCurrentResetFrame);
		state->apsCurrentResetFrame = NULL;
	}

	if (state->dataExchangeBuffer != NULL) {
		ringBufferFree(state->dataExchangeBuffer);
		state->dataExchangeBuffer = NULL;
	}
}

void createAddressedCoarseFineBiasSetting(sshsNode biasNode, const char *biasName, const char *type, const char *sex,
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

uint16_t generateAddressedCoarseFineBias(sshsNode biasNode, const char *biasName) {
	// Add trailing slash to node name (required!).
	size_t biasNameLength = strlen(biasName);
	char biasNameFull[biasNameLength + 2];
	memcpy(biasNameFull, biasName, biasNameLength);
	biasNameFull[biasNameLength] = '/';
	biasNameFull[biasNameLength + 1] = '\0';

	// Get bias configuration node.
	sshsNode biasConfigNode = sshsGetRelativeNode(biasNode, biasNameFull);

	uint16_t biasValue = 0;

	// Build up bias value from all its components.
	if (sshsNodeGetBool(biasConfigNode, "enabled")) {
		biasValue |= 0x01;
	}
	if (str_equals(sshsNodeGetString(biasConfigNode, "sex"), "N")) {
		biasValue |= 0x02;
	}
	if (str_equals(sshsNodeGetString(biasConfigNode, "type"), "Normal")) {
		biasValue |= 0x04;
	}
	if (str_equals(sshsNodeGetString(biasConfigNode, "currentLevel"), "Normal")) {
		biasValue |= 0x08;
	}

	biasValue |= (uint16_t) ((sshsNodeGetByte(biasConfigNode, "fineValue") & 0xFF) << 4);

	// Reverse coarse part. TODO: remove for revision 2 boards!
	uint8_t coarseValue = (sshsNodeGetByte(biasConfigNode, "coarseValue") & 0x07);
	uint8_t reversedCoarseValue = (uint8_t) (((coarseValue * 0x0802LU & 0x22110LU)
		| (coarseValue * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16);

	// Reversing the byte produces a fully reversed byte, so the lower three
	// bits end up reversed, but as the highest three bits! That means shifting
	// them right by 5, and then shifting them left by 12 to put them in their
	// final position. This can be expressed by just a left shift of 7 (12 - 5).
	biasValue |= (uint16_t) (reversedCoarseValue << 7);

	return (biasValue);
}

void createShiftedSourceBiasSetting(sshsNode biasNode, const char *biasName, uint8_t regValue, uint8_t refValue,
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

uint16_t generateShiftedSourceBias(sshsNode biasNode, const char *biasName) {
	// Add trailing slash to node name (required!).
	size_t biasNameLength = strlen(biasName);
	char biasNameFull[biasNameLength + 2];
	memcpy(biasNameFull, biasName, biasNameLength);
	biasNameFull[biasNameLength] = '/';
	biasNameFull[biasNameLength + 1] = '\0';

	// Get bias configuration node.
	sshsNode biasConfigNode = sshsGetRelativeNode(biasNode, biasNameFull);

	uint16_t biasValue = 0;

	if (str_equals(sshsNodeGetString(biasConfigNode, "operatingMode"), "HiZ")) {
		biasValue |= 0x01;
	}
	else if (str_equals(sshsNodeGetString(biasConfigNode, "operatingMode"), "TiedToRail")) {
		biasValue |= 0x02;
	}

	if (str_equals(sshsNodeGetString(biasConfigNode, "voltageLevel"), "SingleDiode")) {
		biasValue |= (0x01 << 2);
	}
	else if (str_equals(sshsNodeGetString(biasConfigNode, "voltageLevel"), "DoubleDiode")) {
		biasValue |= (0x02 << 2);
	}

	biasValue |= (uint16_t) ((sshsNodeGetByte(biasConfigNode, "refValue") & 0x3F) << 4);

	biasValue |= (uint16_t) ((sshsNodeGetByte(biasConfigNode, "regValue") & 0x3F) << 10);

	return (biasValue);
}

void spiConfigSend(libusb_device_handle *devHandle, uint8_t moduleAddr, uint8_t paramAddr, uint32_t param) {
	uint8_t spiConfig[4] = { 0 };

	spiConfig[0] = U8T(param >> 24);
	spiConfig[1] = U8T(param >> 16);
	spiConfig[2] = U8T(param >> 8);
	spiConfig[3] = U8T(param >> 0);

	libusb_control_transfer(devHandle, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
	VR_FPGA_CONFIG, moduleAddr, paramAddr, spiConfig, sizeof(spiConfig), 0);
}

uint32_t spiConfigReceive(libusb_device_handle *devHandle, uint8_t moduleAddr, uint8_t paramAddr) {
	uint32_t returnedParam = 0;
	uint8_t spiConfig[4] = { 0 };

	libusb_control_transfer(devHandle, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
	VR_FPGA_CONFIG, moduleAddr, paramAddr, spiConfig, sizeof(spiConfig), 0);

	returnedParam |= U32T(spiConfig[0] << 24);
	returnedParam |= U32T(spiConfig[1] << 16);
	returnedParam |= U32T(spiConfig[2] << 8);
	returnedParam |= U32T(spiConfig[3] << 0);

	return (returnedParam);
}

bool deviceOpenInfo(caerModuleData moduleData, davisCommonState cstate, uint16_t VID, uint16_t PID, uint8_t DID_TYPE) {
	// USB port/bus/SN settings/restrictions.
	// These can be used to force connection to one specific device.
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "usbBusNumber", 0);
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "usbDevAddress", 0);
	sshsNodePutStringIfAbsent(moduleData->moduleNode, "usbSerialNumber", "");

	// Initialize libusb using a separate context for each device.
	// This is to correctly support one thread per device.
	if ((errno = libusb_init(&cstate->deviceContext)) != LIBUSB_SUCCESS) {
		caerLog(LOG_CRITICAL, moduleData->moduleSubSystemString, "Failed to initialize libusb context. Error: %s (%d).",
			libusb_strerror(errno), errno);
		return (false);
	}

	// Try to open a DAVIS device on a specific USB port.
	cstate->deviceHandle = deviceOpen(cstate->deviceContext, VID, PID, DID_TYPE,
		sshsNodeGetByte(moduleData->moduleNode, "usbBusNumber"),
		sshsNodeGetByte(moduleData->moduleNode, "usbDevAddress"));
	if (cstate->deviceHandle == NULL) {
		libusb_exit(cstate->deviceContext);

		caerLog(LOG_CRITICAL, moduleData->moduleSubSystemString, "Failed to open device.");
		return (false);
	}

	// At this point we can get some more precise data on the device and update
	// the logging string to reflect that and be more informative.
	uint8_t busNumber = libusb_get_bus_number(libusb_get_device(cstate->deviceHandle));
	uint8_t devAddress = libusb_get_device_address(libusb_get_device(cstate->deviceHandle));

	char serialNumber[8 + 1];
	libusb_get_string_descriptor_ascii(cstate->deviceHandle, 3, (unsigned char *) serialNumber, 8 + 1);
	serialNumber[8] = '\0'; // Ensure NUL termination.

	size_t fullLogStringLength = (size_t) snprintf(NULL, 0, "%s SN-%s [%" PRIu8 ":%" PRIu8 "]",
		moduleData->moduleSubSystemString, serialNumber, busNumber, devAddress);
	char fullLogString[fullLogStringLength + 1];
	snprintf(fullLogString, fullLogStringLength + 1, "%s SN-%s [%" PRIu8 ":%" PRIu8 "]",
		moduleData->moduleSubSystemString, serialNumber, busNumber, devAddress);

	// Update module log string, make it accessible in cstate space.
	caerModuleSetSubSystemString(moduleData, fullLogString);
	cstate->sourceSubSystemString = moduleData->moduleSubSystemString;

	// Now check if the Serial Number matches.
	char *configSerialNumber = sshsNodeGetString(moduleData->moduleNode, "usbSerialNumber");

	if (!str_equals(configSerialNumber, "") && !str_equals(configSerialNumber, serialNumber)) {
		libusb_close(cstate->deviceHandle);
		libusb_exit(cstate->deviceContext);

		caerLog(LOG_CRITICAL, moduleData->moduleSubSystemString, "Device Serial Number doesn't match.");
		return (false);
	}

	free(configSerialNumber);

	// So now we have a working connection to the device we want. Let's get some data!
	cstate->chipID = U16T(spiConfigReceive(cstate->deviceHandle, 6, 1));
	cstate->apsSizeX = U16T(spiConfigReceive(cstate->deviceHandle, 6, 2));
	cstate->apsSizeY = U16T(spiConfigReceive(cstate->deviceHandle, 6, 3));
	cstate->dvsSizeX = U16T(spiConfigReceive(cstate->deviceHandle, 6, 4));
	cstate->dvsSizeY = U16T(spiConfigReceive(cstate->deviceHandle, 6, 5));
	cstate->apsGlobalShutter = spiConfigReceive(cstate->deviceHandle, 6, 6);

	// Put global source information into SSHS, so it's globally available.
	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	sshsNodePutShort(sourceInfoNode, "dvsSizeX", cstate->dvsSizeX);
	sshsNodePutShort(sourceInfoNode, "dvsSizeY", cstate->dvsSizeY);
	sshsNodePutShort(sourceInfoNode, "apsSizeX", cstate->apsSizeX);
	sshsNodePutShort(sourceInfoNode, "apsSizeY", cstate->apsSizeY);
	//sshsNodePutShort(sourceInfoNode, "apsOriginalDepth", DAVIS_ADC_DEPTH);
	//sshsNodePutShort(sourceInfoNode, "apsOriginalChannels", DAVIS_COLOR_CHANNELS);

	return (true);
}

void createCommonConfiguration(caerModuleData moduleData, davisCommonState cstate) {
	sshsNode biasNode = sshsGetRelativeNode(moduleData->moduleNode, "bias/");

	createAddressedCoarseFineBiasSetting(biasNode, "DiffBn", "Normal", "N", 3, 72, true);
	createAddressedCoarseFineBiasSetting(biasNode, "OnBn", "Normal", "N", 2, 112, true);
	createAddressedCoarseFineBiasSetting(biasNode, "OffBn", "Normal", "N", 3, 6,
	true);
	createAddressedCoarseFineBiasSetting(biasNode, "ApsCasEpc", "Cascode", "N", 2, 144, true);
	createAddressedCoarseFineBiasSetting(biasNode, "DiffCasBnc", "Cascode", "N", 2, 115, true);
	createAddressedCoarseFineBiasSetting(biasNode, "ApsROSFBn", "Normal", "N", 1, 188, true);
	createAddressedCoarseFineBiasSetting(biasNode, "LocalBufBn", "Normal", "N", 2, 164, true);
	createAddressedCoarseFineBiasSetting(biasNode, "PixInvBn", "Normal", "N", 1, 129, true);
	createAddressedCoarseFineBiasSetting(biasNode, "PrBp", "Normal", "P", 6, 255, true);
	createAddressedCoarseFineBiasSetting(biasNode, "PrSFBp", "Normal", "P", 5, 2, true);
	createAddressedCoarseFineBiasSetting(biasNode, "RefrBp", "Normal", "P", 3, 19, true);
	createAddressedCoarseFineBiasSetting(biasNode, "AEPdBn", "Normal", "N", 0, 140, true);
	createAddressedCoarseFineBiasSetting(biasNode, "LcolTimeoutBn", "Normal", "N", 6, 132, true);
	createAddressedCoarseFineBiasSetting(biasNode, "AEPuXBp", "Normal", "P", 1, 80, true);
	createAddressedCoarseFineBiasSetting(biasNode, "AEPuYBp", "Normal", "P", 1, 152, true);
	createAddressedCoarseFineBiasSetting(biasNode, "IFThrBn", "Normal", "N", 2, 255, true);
	createAddressedCoarseFineBiasSetting(biasNode, "IFRefrBn", "Normal", "N", 2, 255, true);
	createAddressedCoarseFineBiasSetting(biasNode, "PadFollBn", "Normal", "N", 0, 211, true);
	createAddressedCoarseFineBiasSetting(biasNode, "apsOverflowLevel", "Normal", "N", 0, 36, true);
	createAddressedCoarseFineBiasSetting(biasNode, "biasBuffer", "Normal", "N", 1, 251, true);

	createShiftedSourceBiasSetting(biasNode, "SSP", 33, 1, "TiedToRail", "SplitGate");
	createShiftedSourceBiasSetting(biasNode, "SSN", 33, 2, "ShiftedSource", "SplitGate");

	sshsNode chipNode = sshsGetRelativeNode(moduleData->moduleNode, "chip/");

	sshsNodePutBoolIfAbsent(chipNode, "useAout", false);
	sshsNodePutBoolIfAbsent(chipNode, "nArow", false);
	sshsNodePutBoolIfAbsent(chipNode, "resetTestpixel", true);
	sshsNodePutBoolIfAbsent(chipNode, "typeNCalib", false);
	sshsNodePutBoolIfAbsent(chipNode, "resetCalib", true);

	sshsNodePutBoolIfAbsent(chipNode, "hotPixelSuppression", false);

	sshsNodePutByteIfAbsent(chipNode, "AnalogMux0", 0);
	sshsNodePutByteIfAbsent(chipNode, "AnalogMux1", 0);
	sshsNodePutByteIfAbsent(chipNode, "AnalogMux2", 0);
	sshsNodePutByteIfAbsent(chipNode, "BiasMux", 0);
	sshsNodePutByteIfAbsent(chipNode, "DigitalMux0", 0);
	sshsNodePutByteIfAbsent(chipNode, "DigitalMux1", 0);
	sshsNodePutByteIfAbsent(chipNode, "DigitalMux2", 0);
	sshsNodePutByteIfAbsent(chipNode, "DigitalMux3", 0);

	sshsNode logicNode = sshsGetRelativeNode(moduleData->moduleNode, "logic/");

	// Subsystem 0: Multiplexer
	sshsNode muxNode = sshsGetRelativeNode(logicNode, "Multiplexer/");

	sshsNodePutBoolIfAbsent(muxNode, "Run", 0);
	sshsNodePutBoolIfAbsent(muxNode, "TimestampRun", 0);
	sshsNodePutBoolIfAbsent(muxNode, "TimestampReset", 0);
	sshsNodePutBoolIfAbsent(muxNode, "ForceChipBiasEnable", 0);
	sshsNodePutBoolIfAbsent(muxNode, "DropDVSOnTransferStall", 1);
	sshsNodePutBoolIfAbsent(muxNode, "DropAPSOnTransferStall", 0);
	sshsNodePutBoolIfAbsent(muxNode, "DropIMUOnTransferStall", 1);
	sshsNodePutBoolIfAbsent(muxNode, "DropExtInputOnTransferStall", 1);

	// Subsystem 1: DVS AER
	sshsNode dvsNode = sshsGetRelativeNode(logicNode, "DVS/");

	sshsNodePutBoolIfAbsent(dvsNode, "Run", 0);
	sshsNodePutByteIfAbsent(dvsNode, "AckDelay", 8);
	sshsNodePutByteIfAbsent(dvsNode, "AckExtension", 2);
	sshsNodePutBoolIfAbsent(dvsNode, "WaitOnTransferStall", 0);

	// Subsystem 2: APS ADC
	sshsNode apsNode = sshsGetRelativeNode(logicNode, "APS/");

	sshsNodePutBoolIfAbsent(apsNode, "Run", 0);
	sshsNodePutBoolIfAbsent(apsNode, "ForceADCRunning", 0);
	sshsNodePutBoolIfAbsent(apsNode, "GlobalShutter", 1);
	sshsNodePutShortIfAbsent(apsNode, "StartColumn", 0);
	sshsNodePutShortIfAbsent(apsNode, "StartRow", 0);
	sshsNodePutShortIfAbsent(apsNode, "EndColumn", U16T(cstate->apsSizeX - 1));
	sshsNodePutShortIfAbsent(apsNode, "EndRow", U16T(cstate->apsSizeY - 1));
	sshsNodePutIntIfAbsent(apsNode, "Exposure", 2000); // in µs, converted to cycles later
	sshsNodePutIntIfAbsent(apsNode, "FrameDelay", 200); // in µs, converted to cycles later
	sshsNodePutShortIfAbsent(apsNode, "ResetSettle", 10); // in cycles
	sshsNodePutByteIfAbsent(apsNode, "ColumnSettle", 10); // in cycles
	sshsNodePutByteIfAbsent(apsNode, "RowSettle", 10); // in cycles
	sshsNodePutBoolIfAbsent(apsNode, "GSTXGateOpenReset_S", 1);
	sshsNodePutBoolIfAbsent(apsNode, "ResetRead", 1);
	sshsNodePutBoolIfAbsent(apsNode, "WaitOnTransferStall", 0);

	// Subsystem 3: IMU
	sshsNode imuNode = sshsGetRelativeNode(logicNode, "IMU/");

	sshsNodePutBoolIfAbsent(imuNode, "Run", 0);
	sshsNodePutBoolIfAbsent(imuNode, "TempStandby", 0);
	sshsNodePutBoolIfAbsent(imuNode, "AccelXStandby", 0);
	sshsNodePutBoolIfAbsent(imuNode, "AccelYStandby", 0);
	sshsNodePutBoolIfAbsent(imuNode, "AccelZStandby", 0);
	sshsNodePutBoolIfAbsent(imuNode, "GyroXStandby", 0);
	sshsNodePutBoolIfAbsent(imuNode, "GyroYStandby", 0);
	sshsNodePutBoolIfAbsent(imuNode, "GyroZStandby", 0);
	sshsNodePutBoolIfAbsent(imuNode, "LowPowerCycle", 0);
	sshsNodePutByteIfAbsent(imuNode, "LowPowerWakeupFrequency", 1);
	sshsNodePutByteIfAbsent(imuNode, "SampleRateDivider", 0);
	sshsNodePutByteIfAbsent(imuNode, "DigitalLowPassFilter", 1);
	sshsNodePutByteIfAbsent(imuNode, "AccelFullScale", 1);
	sshsNodePutByteIfAbsent(imuNode, "GyroFullScale", 1);

	// Subsystem 4: External Input
	sshsNode extNode = sshsGetRelativeNode(logicNode, "ExternalInput/");

	sshsNodePutBoolIfAbsent(extNode, "RunDetector", 0);
	sshsNodePutBoolIfAbsent(extNode, "DetectRisingEdges", 0);
	sshsNodePutBoolIfAbsent(extNode, "DetectFallingEdges", 0);
	sshsNodePutBoolIfAbsent(extNode, "DetectPulses", 1);
	sshsNodePutBoolIfAbsent(extNode, "DetectPulsePolarity", 1);
	sshsNodePutIntIfAbsent(extNode, "DetectPulseLength", 10);

	// Subsystem 9: FX2/3 USB Configuration
	sshsNode fxNode = sshsGetRelativeNode(logicNode, "USB/");

	sshsNodePutBoolIfAbsent(fxNode, "Run", 1);
	sshsNodePutShortIfAbsent(fxNode, "EarlyPacketDelay", 8);

	// USB buffer settings.
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "bufferNumber", 8);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "bufferSize", 8192);

	// Packet settings (size (in events) and time interval (in µs)).
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "polarityPacketMaxSize", 4096);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "polarityPacketMaxInterval", 5000);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "framePacketMaxSize", 4);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "framePacketMaxInterval", 20000);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "imu6PacketMaxSize", 32);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "imu6PacketMaxInterval", 4000);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "specialPacketMaxSize", 128);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "specialPacketMaxInterval", 1000);

	// Ring-buffer setting (only changes value on module init/shutdown cycles).
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "dataExchangeBufferSize", 64);

	// Install default listener to signal configuration updates asynchronously.
	sshsNodeAddAttrListener(biasNode, moduleData, &caerInputDAVISCommonConfigListener);
	sshsNodeAddAttrListener(chipNode, moduleData, &caerInputDAVISCommonConfigListener);
	sshsNodeAddAttrListener(muxNode, moduleData, &caerInputDAVISCommonConfigListener);
	sshsNodeAddAttrListener(dvsNode, moduleData, &caerInputDAVISCommonConfigListener);
	sshsNodeAddAttrListener(apsNode, moduleData, &caerInputDAVISCommonConfigListener);
	sshsNodeAddAttrListener(imuNode, moduleData, &caerInputDAVISCommonConfigListener);
	sshsNodeAddAttrListener(extNode, moduleData, &caerInputDAVISCommonConfigListener);
	sshsNodeAddAttrListener(fxNode, moduleData, &caerInputDAVISCommonConfigListener);
	sshsNodeAddAttrListener(moduleData->moduleNode, moduleData, &caerInputDAVISCommonConfigListener);
}

bool initializeCommonConfiguration(caerModuleData moduleData, davisCommonState cstate,
	void *dataAcquisitionThread(void *inPtr)) {
	// Initialize state fields.
	cstate->maxPolarityPacketSize = sshsNodeGetInt(moduleData->moduleNode, "polarityPacketMaxSize");
	cstate->maxPolarityPacketInterval = sshsNodeGetInt(moduleData->moduleNode, "polarityPacketMaxInterval");

	cstate->maxFramePacketSize = sshsNodeGetInt(moduleData->moduleNode, "framePacketMaxSize");
	cstate->maxFramePacketInterval = sshsNodeGetInt(moduleData->moduleNode, "framePacketMaxInterval");

	cstate->maxIMU6PacketSize = sshsNodeGetInt(moduleData->moduleNode, "imu6PacketMaxSize");
	cstate->maxIMU6PacketInterval = sshsNodeGetInt(moduleData->moduleNode, "imu6PacketMaxInterval");

	cstate->maxSpecialPacketSize = sshsNodeGetInt(moduleData->moduleNode, "specialPacketMaxSize");
	cstate->maxSpecialPacketInterval = sshsNodeGetInt(moduleData->moduleNode, "specialPacketMaxInterval");

	cstate->currentPolarityPacket = caerPolarityEventPacketAllocate(cstate->maxPolarityPacketSize, cstate->sourceID);
	cstate->currentPolarityPacketPosition = 0;

	cstate->currentFramePacket = caerFrameEventPacketAllocate(cstate->maxFramePacketSize, cstate->sourceID,
		cstate->apsSizeX, cstate->apsSizeY, DAVIS_COLOR_CHANNELS);
	cstate->currentFramePacketPosition = 0;

	cstate->currentIMU6Packet = caerIMU6EventPacketAllocate(cstate->maxIMU6PacketSize, cstate->sourceID);
	cstate->currentIMU6PacketPosition = 0;

	cstate->currentSpecialPacket = caerSpecialEventPacketAllocate(cstate->maxSpecialPacketSize, cstate->sourceID);
	cstate->currentSpecialPacketPosition = 0;

	cstate->wrapAdd = 0;
	cstate->lastTimestamp = 0;
	cstate->currentTimestamp = 0;
	cstate->dvsTimestamp = 0;
	cstate->dvsLastY = 0;
	cstate->dvsGotY = false;
	cstate->dvsTranslateRowOnlyEvents = false;
	cstate->apsGlobalShutter = sshsNodeGetBool(sshsGetRelativeNode(moduleData->moduleNode, "logic/APS/"),
		"GlobalShutter");
	cstate->apsCurrentReadoutType = APS_READOUT_RESET;
	for (size_t i = 0; i < APS_READOUT_TYPES_NUM; i++) {
		cstate->apsCountX[i] = 0;
		cstate->apsCountY[i] = 0;
	}
	cstate->apsCurrentResetFrame = calloc((size_t) cstate->apsSizeX * cstate->apsSizeY, DAVIS_COLOR_CHANNELS);
	if (cstate->apsCurrentResetFrame == NULL) {
		freeAllMemory(cstate);

		caerLog(LOG_CRITICAL, moduleData->moduleSubSystemString, "Failed to allocate reset frame array.");
		return (false);
	}

	// Store reference to parent mainloop, so that we can correctly notify
	// the availability or not of data to consume.
	cstate->mainloopNotify = caerMainloopGetReference();

	// Create data exchange buffers.
	cstate->dataExchangeBuffer = ringBufferInit(sshsNodeGetInt(moduleData->moduleNode, "dataExchangeBufferSize"));
	if (cstate->dataExchangeBuffer == NULL) {
		freeAllMemory(cstate);

		caerLog(LOG_CRITICAL, moduleData->moduleSubSystemString, "Failed to initialize data exchange buffer.");
		return (false);
	}

	// Start data acquisition thread.
	if ((errno = pthread_create(&cstate->dataAcquisitionThread, NULL, dataAcquisitionThread, moduleData)) != 0) {
		freeAllMemory(cstate);

		caerLog(LOG_CRITICAL, moduleData->moduleSubSystemString,
			"Failed to start data acquisition thread. Error: %s (%d).", caerLogStrerror(errno),
			errno);
		return (false);
	}

	return (true);
}

void caerInputDAVISCommonRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerPolarityEventPacket *polarity = va_arg(args, caerPolarityEventPacket *);
	caerFrameEventPacket *frame = va_arg(args, caerFrameEventPacket *);
	caerIMU6EventPacket *imu6 = va_arg(args, caerIMU6EventPacket *);
	caerSpecialEventPacket *special = va_arg(args, caerSpecialEventPacket *);

	davisCommonState state = moduleData->moduleState;

	// Check what the user wants.
	bool wantPolarity = false, havePolarity = false;
	bool wantFrame = false, haveFrame = false;
	bool wantIMU6 = false, haveIMU6 = false;
	bool wantSpecial = false, haveSpecial = false;

	if (polarity != NULL) {
		wantPolarity = true;
	}

	if (frame != NULL) {
		wantFrame = true;
	}

	if (imu6 != NULL) {
		wantIMU6 = true;
	}

	if (special != NULL) {
		wantSpecial = true;
	}

	void *packet;
	while ((packet = ringBufferLook(state->dataExchangeBuffer)) != NULL) {
		// Check what kind it is and assign accordingly.
		caerEventPacketHeader packetHeader = packet;

		// Check polarity events first, then frame, then IMU6, finally special.
		if (packetHeader->eventType == POLARITY_EVENT) {
			// Throw away unwanted packets first.
			if (!wantPolarity) {
				caerMainloopDataAvailableDecrease(state->mainloopNotify);
				free(ringBufferGet(state->dataExchangeBuffer));
				continue;
			}

			// At this point packet is something we want, so we see if we can
			// assign it to one of the output pointers. This will be possible if
			// the output is still free, if not, we have to wait until next loop
			// iteration to fit this somewhere, and so we exit.
			if (!havePolarity) {
				caerMainloopDataAvailableDecrease(state->mainloopNotify);
				*polarity = ringBufferGet(state->dataExchangeBuffer);
				havePolarity = true;

				// Ensure memory gets recycled after the loop is over.
				caerMainloopFreeAfterLoop(*polarity);

				continue;
			}

			// Couldn't fit the current packet into any free output pointers,
			// break off and defer to next iteration of mainloop.
			break;
		}
		else if (packetHeader->eventType == FRAME_EVENT) {
			// Throw away unwanted packets first.
			if (!wantFrame) {
				caerMainloopDataAvailableDecrease(state->mainloopNotify);
				free(ringBufferGet(state->dataExchangeBuffer));
				continue;
			}

			// At this point packet is something we want, so we see if we can
			// assign it to one of the output pointers. This will be possible if
			// the output is still free, if not, we have to wait until next loop
			// iteration to fit this somewhere, and so we exit.
			if (!haveFrame) {
				caerMainloopDataAvailableDecrease(state->mainloopNotify);
				*frame = ringBufferGet(state->dataExchangeBuffer);
				haveFrame = true;

				// Ensure memory gets recycled after the loop is over.
				caerMainloopFreeAfterLoop(*frame);

				continue;
			}

			// Couldn't fit the current packet into any free output pointers,
			// break off and defer to next iteration of mainloop.
			break;
		}
		else if (packetHeader->eventType == IMU6_EVENT) {
			// Throw away unwanted packets first.
			if (!wantIMU6) {
				caerMainloopDataAvailableDecrease(state->mainloopNotify);
				free(ringBufferGet(state->dataExchangeBuffer));
				continue;
			}

			// At this point packet is something we want, so we see if we can
			// assign it to one of the output pointers. This will be possible if
			// the output is still free, if not, we have to wait until next loop
			// iteration to fit this somewhere, and so we exit.
			if (!haveIMU6) {
				caerMainloopDataAvailableDecrease(state->mainloopNotify);
				*imu6 = ringBufferGet(state->dataExchangeBuffer);
				haveIMU6 = true;

				// Ensure memory gets recycled after the loop is over.
				caerMainloopFreeAfterLoop(*imu6);

				continue;
			}

			// Couldn't fit the current packet into any free output pointers,
			// break off and defer to next iteration of mainloop.
			break;
		}
		else if (packetHeader->eventType == SPECIAL_EVENT) {
			// Throw away unwanted packets first.
			if (!wantSpecial) {
				caerMainloopDataAvailableDecrease(state->mainloopNotify);
				free(ringBufferGet(state->dataExchangeBuffer));
				continue;
			}

			// At this point packet is something we want, so we see if we can
			// assign it to one of the output pointers. This will be possible if
			// the output is still free, if not, we have to wait until next loop
			// iteration to fit this somewhere, and so we exit.
			if (!haveSpecial) {
				caerMainloopDataAvailableDecrease(state->mainloopNotify);
				*special = ringBufferGet(state->dataExchangeBuffer);
				haveSpecial = true;

				// Ensure memory gets recycled after the loop is over.
				caerMainloopFreeAfterLoop(*special);

				continue;
			}

			// Couldn't fit the current packet into any free output pointers,
			// break off and defer to next iteration of mainloop.
			break;
		}
	}
}

void allocateDataTransfers(davisCommonState state, uint32_t bufferNum, uint32_t bufferSize) {
	atomic_ops_uint_store(&state->dataTransfersLength, 0, ATOMIC_OPS_FENCE_NONE);

	// Set number of transfers and allocate memory for the main transfer array.
	state->dataTransfers = calloc(bufferNum, sizeof(struct libusb_transfer *));
	if (state->dataTransfers == NULL) {
		caerLog(LOG_CRITICAL, state->sourceSubSystemString,
			"Failed to allocate memory for %" PRIu32 " libusb transfers (data channel). Error: %s (%d).", bufferNum,
			caerLogStrerror(errno), errno);
		return;
	}

	// Allocate transfers and set them up.
	for (size_t i = 0; i < bufferNum; i++) {
		state->dataTransfers[i] = libusb_alloc_transfer(0);
		if (state->dataTransfers[i] == NULL) {
			caerLog(LOG_CRITICAL, state->sourceSubSystemString,
				"Unable to allocate further libusb transfers (data channel, %zu of %" PRIu32 ").", i, bufferNum);
			return;
		}

		// Create data buffer.
		state->dataTransfers[i]->length = (int) bufferSize;
		state->dataTransfers[i]->buffer = malloc(bufferSize);
		if (state->dataTransfers[i]->buffer == NULL) {
			caerLog(LOG_CRITICAL, state->sourceSubSystemString,
				"Unable to allocate buffer for libusb transfer %zu (data channel). Error: %s (%d).", i,
				caerLogStrerror(errno), errno);

			libusb_free_transfer(state->dataTransfers[i]);
			state->dataTransfers[i] = NULL;

			return;
		}

		// Initialize Transfer.
		state->dataTransfers[i]->dev_handle = state->deviceHandle;
		state->dataTransfers[i]->endpoint = DATA_ENDPOINT;
		state->dataTransfers[i]->type = LIBUSB_TRANSFER_TYPE_BULK;
		state->dataTransfers[i]->callback = &libUsbDataCallback;
		state->dataTransfers[i]->user_data = state;
		state->dataTransfers[i]->timeout = 0;
		state->dataTransfers[i]->flags = LIBUSB_TRANSFER_FREE_BUFFER;

		if ((errno = libusb_submit_transfer(state->dataTransfers[i])) == LIBUSB_SUCCESS) {
			atomic_ops_uint_inc(&state->dataTransfersLength, ATOMIC_OPS_FENCE_NONE);
		}
		else {
			caerLog(LOG_CRITICAL, state->sourceSubSystemString,
				"Unable to submit libusb transfer %zu (data channel). Error: %s (%d).", i, libusb_strerror(errno),
				errno);

			// The transfer buffer is freed automatically here thanks to
			// the LIBUSB_TRANSFER_FREE_BUFFER flag set above.
			libusb_free_transfer(state->dataTransfers[i]);
			state->dataTransfers[i] = NULL;

			return;
		}
	}
}

void deallocateDataTransfers(davisCommonState state) {
	// This will change later on, but we still need it.
	uint32_t transfersNum = (uint32_t) atomic_ops_uint_load(&state->dataTransfersLength, ATOMIC_OPS_FENCE_NONE);

	// Cancel all current transfers first.
	for (size_t i = 0; i < transfersNum; i++) {
		errno = libusb_cancel_transfer(state->dataTransfers[i]);
		if (errno != LIBUSB_SUCCESS && errno != LIBUSB_ERROR_NOT_FOUND) {
			caerLog(LOG_CRITICAL, state->sourceSubSystemString,
				"Unable to cancel libusb transfer %zu (data channel). Error: %s (%d).", i, libusb_strerror(errno),
				errno);
			// Proceed with canceling all transfers regardless of errors.
		}
	}

	// Wait for all transfers to go away (0.1 seconds timeout).
	struct timeval te = { .tv_sec = 0, .tv_usec = 100000 };

	while (atomic_ops_uint_load(&state->dataTransfersLength, ATOMIC_OPS_FENCE_NONE) > 0) {
		libusb_handle_events_timeout(state->deviceContext, &te);
	}

	// The buffers and transfers have been deallocated in the callback.
	// Only the transfers array remains.
	free(state->dataTransfers);
}

static void LIBUSB_CALL libUsbDataCallback(struct libusb_transfer *transfer) {
	davisCommonState state = transfer->user_data;

	if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
		// Handle event data.
		dataTranslator(state, transfer->buffer, (size_t) transfer->actual_length);
	}

	if (transfer->status != LIBUSB_TRANSFER_CANCELLED && transfer->status != LIBUSB_TRANSFER_NO_DEVICE) {
		// Submit transfer again.
		if (libusb_submit_transfer(transfer) == LIBUSB_SUCCESS) {
			return;
		}
	}

	// Cannot recover (cancelled, no device, or other critical error).
	// Signal this by adjusting the counter, free and exit.
	atomic_ops_uint_dec(&state->dataTransfersLength, ATOMIC_OPS_FENCE_NONE);
	libusb_free_transfer(transfer);
}

static void dataTranslator(davisCommonState state, uint8_t *buffer, size_t bytesSent) {
	// Truncate off any extra partial event.
	if ((bytesSent & 0x01) != 0) {
		caerLog(LOG_ALERT, state->sourceSubSystemString, "%zu bytes sent via USB, which is not a multiple of two.",
			bytesSent);
		bytesSent &= (size_t) ~0x01;
	}

	for (size_t i = 0; i < bytesSent; i += 2) {
		bool forcePacketCommit = false;

		uint16_t event = le16toh(*((uint16_t * ) (&buffer[i])));

		// Check if timestamp.
		if ((event & 0x8000) != 0) {
			// Is a timestamp! Expand to 32 bits. (Tick is 1µs already.)
			state->lastTimestamp = state->currentTimestamp;
			state->currentTimestamp = state->wrapAdd + (event & 0x7FFF);

			// Check monotonicity of timestamps.
			CHECK_MONOTONIC_TIMESTAMP(state->currentTimestamp, state->lastTimestamp);
		}
		else {
			// Look at the code, to determine event and data type.
			uint8_t code = (uint8_t) ((event & 0x7000) >> 12);
			uint16_t data = (event & 0x0FFF);

			switch (code) {
				case 0: // Special event
					switch (data) {
						case 0: // Ignore this, but log it.
							caerLog(LOG_ERROR, state->sourceSubSystemString, "Caught special reserved event!");
							break;

						case 1: { // Timetamp reset
							state->wrapAdd = 0;
							state->lastTimestamp = 0;
							state->currentTimestamp = 0;
							state->dvsTimestamp = 0;

							caerLog(LOG_INFO, state->sourceSubSystemString, "Timestamp reset event received.");

							// Create timestamp reset event.
							caerSpecialEvent currentResetEvent = caerSpecialEventPacketGetEvent(
								state->currentSpecialPacket, state->currentSpecialPacketPosition++);
							caerSpecialEventSetTimestamp(currentResetEvent, UINT32_MAX);
							caerSpecialEventSetType(currentResetEvent, TIMESTAMP_RESET);
							caerSpecialEventValidate(currentResetEvent, state->currentSpecialPacket);

							// Commit packets when doing a reset to clearly separate them.
							forcePacketCommit = true;

							break;
						}

						case 2: // External trigger (falling edge)
						case 3: // External trigger (rising edge)
						case 4: { // External trigger (pulse)
							caerSpecialEvent currentExtTriggerEvent = caerSpecialEventPacketGetEvent(
								state->currentSpecialPacket, state->currentSpecialPacketPosition++);
							caerSpecialEventSetTimestamp(currentExtTriggerEvent, state->currentTimestamp);
							caerSpecialEventSetType(currentExtTriggerEvent, EXTERNAL_TRIGGER);
							caerSpecialEventValidate(currentExtTriggerEvent, state->currentSpecialPacket);
							break;
						}

						case 5: { // IMU Start (6 axes)
							caerIMU6Event currentIMUEvent = caerIMU6EventPacketGetEvent(state->currentIMU6Packet,
								state->currentIMU6PacketPosition);
							caerIMU6EventSetTimestamp(currentIMUEvent, state->currentTimestamp);
							break;
						}

						case 7: // IMU End
							break;

						case 8: { // APS Frame Start
							caerLog(LOG_DEBUG, state->sourceSubSystemString, "APS Frame Start");

							state->apsCurrentReadoutType = APS_READOUT_RESET;
							for (size_t j = 0; j < APS_READOUT_TYPES_NUM; j++) {
								state->apsCountX[j] = 0;
								state->apsCountY[j] = 0;
							}

							// Write out start of frame timestamp.
							caerFrameEvent currentFrameEvent = caerFrameEventPacketGetEvent(state->currentFramePacket,
								state->currentFramePacketPosition);
							caerFrameEventSetTSStartOfFrame(currentFrameEvent, state->currentTimestamp);

							// Setup frame.
							caerFrameEventSetChannelNumber(currentFrameEvent, DAVIS_COLOR_CHANNELS);
							caerFrameEventSetLengthXY(currentFrameEvent, state->currentFramePacket, state->apsSizeX,
								state->apsSizeY);

							break;
						}

						case 9: { // APS Frame End
							caerLog(LOG_DEBUG, state->sourceSubSystemString, "APS Frame End");

							bool validFrame = true;

							for (size_t j = 0; j < APS_READOUT_TYPES_NUM; j++) {
								caerLog(LOG_DEBUG, state->sourceSubSystemString, "APS Frame End: CountX[%zu] is %d.", j,
									state->apsCountX[j]);

								if (state->apsCountX[j] != state->apsSizeX) {
									caerLog(LOG_ERROR, state->sourceSubSystemString,
										"APS Frame End: wrong column count [%zu - %d] detected.", j,
										state->apsCountX[j]);
									validFrame = false;
								}
							}

							// Write out end of frame timestamp.
							caerFrameEvent currentFrameEvent = caerFrameEventPacketGetEvent(state->currentFramePacket,
								state->currentFramePacketPosition);
							caerFrameEventSetTSEndOfFrame(currentFrameEvent, state->currentTimestamp);

							// Validate event and advance frame packet position.
							if (validFrame) {
								caerFrameEventValidate(currentFrameEvent, state->currentFramePacket);
							}
							state->currentFramePacketPosition++;

							break;
						}

						case 10: { // APS Reset Column Start
							caerLog(LOG_DEBUG, state->sourceSubSystemString, "APS Reset Column Start");

							state->apsCurrentReadoutType = APS_READOUT_RESET;
							state->apsCountY[state->apsCurrentReadoutType] = 0;

							// The first Reset Column Read Start is also the start
							// of the exposure for the RS.
							if (!state->apsGlobalShutter && state->apsCountX[APS_READOUT_RESET] == 0) {
								caerFrameEvent currentFrameEvent = caerFrameEventPacketGetEvent(
									state->currentFramePacket, state->currentFramePacketPosition);
								caerFrameEventSetTSStartOfExposure(currentFrameEvent, state->currentTimestamp);
							}

							break;
						}

						case 11: { // APS Signal Column Start
							caerLog(LOG_DEBUG, state->sourceSubSystemString, "APS Signal Column Start");

							state->apsCurrentReadoutType = APS_READOUT_SIGNAL;
							state->apsCountY[state->apsCurrentReadoutType] = 0;

							// The first Signal Column Read Start is also always the end
							// of the exposure time, for both RS and GS.
							if (state->apsCountX[APS_READOUT_SIGNAL] == 0) {
								caerFrameEvent currentFrameEvent = caerFrameEventPacketGetEvent(
									state->currentFramePacket, state->currentFramePacketPosition);
								caerFrameEventSetTSEndOfExposure(currentFrameEvent, state->currentTimestamp);
							}

							break;
						}

						case 12: { // APS Column End
							caerLog(LOG_DEBUG, state->sourceSubSystemString, "APS Column End");

							if (state->apsCountY[state->apsCurrentReadoutType] != state->apsSizeY) {
								caerLog(LOG_ERROR, state->sourceSubSystemString,
									"APS Column End: wrong row count [%d - %d] detected.", state->apsCurrentReadoutType,
									state->apsCountY[state->apsCurrentReadoutType]);
							}

							caerLog(LOG_DEBUG, state->sourceSubSystemString, "APS Column End: CountX[%d] is %d.",
								state->apsCurrentReadoutType, state->apsCountX[state->apsCurrentReadoutType]);
							caerLog(LOG_DEBUG, state->sourceSubSystemString, "APS Column End: CountY[%d] is %d.",
								state->apsCurrentReadoutType, state->apsCountY[state->apsCurrentReadoutType]);

							state->apsCountX[state->apsCurrentReadoutType]++;

							// The last Reset Column Read End is also the start
							// of the exposure for the GS.
							if (state->apsGlobalShutter && state->apsCurrentReadoutType == APS_READOUT_RESET
								&& state->apsCountX[APS_READOUT_RESET] == state->apsSizeX) {
								caerFrameEvent currentFrameEvent = caerFrameEventPacketGetEvent(
									state->currentFramePacket, state->currentFramePacketPosition);
								caerFrameEventSetTSStartOfExposure(currentFrameEvent, state->currentTimestamp);
							}

							break;
						}

						default:
							caerLog(LOG_ERROR, state->sourceSubSystemString,
								"Caught special event that can't be handled.");
							break;
					}
					break;

				case 1: // Y address
					// Check range conformity.
					if (data >= state->dvsSizeY) {
						caerLog(LOG_ALERT, state->sourceSubSystemString, "Y address out of range (0-%d): %" PRIu16 ".",
							state->dvsSizeY - 1, data);
						continue; // Skip invalid Y address (don't update lastY).
					}

					if (state->dvsTranslateRowOnlyEvents && state->dvsGotY) {
						caerSpecialEvent currentRowOnlyEvent = caerSpecialEventPacketGetEvent(
							state->currentSpecialPacket, state->currentSpecialPacketPosition++);
						// Use the previous timestamp here, since this refers to the previous Y.
						caerSpecialEventSetTimestamp(currentRowOnlyEvent, state->dvsTimestamp);
						caerSpecialEventSetType(currentRowOnlyEvent, ROW_ONLY);
						caerSpecialEventSetData(currentRowOnlyEvent, state->dvsLastY);
						caerSpecialEventValidate(currentRowOnlyEvent, state->currentSpecialPacket);

						caerLog(LOG_DEBUG, state->sourceSubSystemString, "Row-only event at address Y=%" PRIu16 ".",
							state->dvsLastY);
					}

					state->dvsLastY = data;
					state->dvsGotY = true;
					state->dvsTimestamp = state->currentTimestamp;

					break;

				case 2: // X address, Polarity OFF
				case 3: { // X address, Polarity ON
					// Check range conformity.
					if (data >= state->dvsSizeX) {
						caerLog(LOG_ALERT, state->sourceSubSystemString, "X address out of range (0-%d): %" PRIu16 ".",
							state->dvsSizeX - 1, data);
						continue; // Skip invalid event.
					}

					caerPolarityEvent currentPolarityEvent = caerPolarityEventPacketGetEvent(
						state->currentPolarityPacket, state->currentPolarityPacketPosition++);
					caerPolarityEventSetTimestamp(currentPolarityEvent, state->dvsTimestamp);
					caerPolarityEventSetPolarity(currentPolarityEvent, (code & 0x01));
					caerPolarityEventSetY(currentPolarityEvent, state->dvsLastY);
					caerPolarityEventSetX(currentPolarityEvent, data);
					caerPolarityEventValidate(currentPolarityEvent, state->currentPolarityPacket);

					state->dvsGotY = false;

					break;
				}

				case 4: {
					// First, let's normalize the ADC value to 16bit generic depth.
					data = U16T(data << (16 - DAVIS_ADC_DEPTH));

					// If reset read, we store the values in a local array. If signal read, we
					// store the final pixel value directly in the output frame event. We already
					// do the subtraction between reset and signal here, to avoid carrying that
					// around all the time and consuming memory. This way we can also only take
					// infrequent reset reads and re-use them for multiple frames, which can heavily
					// reduce traffic, and should not impact image quality heavily, at least in GS.
					caerFrameEvent currentFrameEvent = caerFrameEventPacketGetEvent(state->currentFramePacket,
						state->currentFramePacketPosition);

					uint16_t xPos = U16T(state->apsSizeX - 1 - state->apsCountX[state->apsCurrentReadoutType]);
					uint16_t yPos = U16T(state->apsSizeY - 1 - state->apsCountY[state->apsCurrentReadoutType]);

					size_t pixelPosition = (size_t) (yPos * caerFrameEventGetLengthX(currentFrameEvent)) + xPos;

					if (state->apsCurrentReadoutType == APS_READOUT_RESET) {
						state->apsCurrentResetFrame[pixelPosition] = data;
					}
					else {
						int32_t pixelValue = state->apsCurrentResetFrame[pixelPosition] - data;
						caerFrameEventGetPixelArrayUnsafe(currentFrameEvent)[pixelPosition] = htole16(
							U16T((pixelValue < 0) ? (0) : (pixelValue)));
					}

					caerLog(LOG_DEBUG, state->sourceSubSystemString, "APS ADC Sample: row is %d.",
						state->apsCountY[state->apsCurrentReadoutType]);

					state->apsCountY[state->apsCurrentReadoutType]++;

					break;
				}

				case 5: // Misc 8bit data, used currently only
						// for IMU events in DAVIS FX3 boards
					break;

				case 7: // Timestamp wrap
					// Each wrap is 2^15 µs (~32ms), and we have
					// to multiply it with the wrap counter,
					// which is located in the data part of this
					// event.
					state->wrapAdd += (uint32_t) (0x8000 * data);

					state->lastTimestamp = state->currentTimestamp;
					state->currentTimestamp = state->wrapAdd;

					// Check monotonicity of timestamps.
					CHECK_MONOTONIC_TIMESTAMP(state->currentTimestamp, state->lastTimestamp)
					;

					caerLog(LOG_DEBUG, state->sourceSubSystemString,
						"Timestamp wrap event received with multiplier of %" PRIu16 ".", data);
					break;

				default:
					caerLog(LOG_ERROR, state->sourceSubSystemString, "Caught event that can't be handled.");
					break;
			}
		}

		// Commit packet to the ring-buffer, so they can be processed by the
		// main-loop, when their stated conditions are met.
		if (forcePacketCommit
			|| (state->currentPolarityPacketPosition
				>= caerEventPacketHeaderGetEventCapacity(&state->currentPolarityPacket->packetHeader))
			|| ((state->currentPolarityPacketPosition > 1)
				&& (caerPolarityEventGetTimestamp(
					caerPolarityEventPacketGetEvent(state->currentPolarityPacket,
						state->currentPolarityPacketPosition - 1))
					- caerPolarityEventGetTimestamp(caerPolarityEventPacketGetEvent(state->currentPolarityPacket, 0))
					>= state->maxPolarityPacketInterval))) {
			if (!ringBufferPut(state->dataExchangeBuffer, state->currentPolarityPacket)) {
				// Failed to forward packet, drop it.
				free(state->currentPolarityPacket);
				caerLog(LOG_INFO, state->sourceSubSystemString,
					"Dropped Polarity Event Packet because ring-buffer full!");
			}
			else {
				caerMainloopDataAvailableIncrease(state->mainloopNotify);
			}

			// Allocate new packet for next iteration.
			state->currentPolarityPacket = caerPolarityEventPacketAllocate(state->maxPolarityPacketSize,
				state->sourceID);
			state->currentPolarityPacketPosition = 0;
		}

		if (forcePacketCommit
			|| (state->currentSpecialPacketPosition
				>= caerEventPacketHeaderGetEventCapacity(&state->currentSpecialPacket->packetHeader))
			|| ((state->currentSpecialPacketPosition > 1)
				&& (caerSpecialEventGetTimestamp(
					caerSpecialEventPacketGetEvent(state->currentSpecialPacket,
						state->currentSpecialPacketPosition - 1))
					- caerSpecialEventGetTimestamp(caerSpecialEventPacketGetEvent(state->currentSpecialPacket, 0))
					>= state->maxSpecialPacketInterval))) {
			retry_special: if (!ringBufferPut(state->dataExchangeBuffer, state->currentSpecialPacket)) {
				// Failed to forward packet, drop it, unless it contains a timestamp
				// related change, those are critical, so we just spin until we can
				// deliver that one. (Easily detected by forcePacketCommit!)
				if (forcePacketCommit) {
					goto retry_special;
				}
				else {
					// Failed to forward packet, drop it.
					free(state->currentSpecialPacket);
					caerLog(LOG_INFO, state->sourceSubSystemString,
						"Dropped Special Event Packet because ring-buffer full!");
				}
			}
			else {
				caerMainloopDataAvailableIncrease(state->mainloopNotify);
			}

			// Allocate new packet for next iteration.
			state->currentSpecialPacket = caerSpecialEventPacketAllocate(state->maxSpecialPacketSize, state->sourceID);
			state->currentSpecialPacketPosition = 0;
		}

		if (forcePacketCommit
			|| (state->currentFramePacketPosition
				>= caerEventPacketHeaderGetEventCapacity(&state->currentFramePacket->packetHeader))
			|| ((state->currentFramePacketPosition > 1)
				&& (caerFrameEventGetTSStartOfExposure(
					caerFrameEventPacketGetEvent(state->currentFramePacket, state->currentFramePacketPosition - 1))
					- caerFrameEventGetTSStartOfExposure(caerFrameEventPacketGetEvent(state->currentFramePacket, 0))
					>= state->maxFramePacketInterval))) {
			if (!ringBufferPut(state->dataExchangeBuffer, state->currentFramePacket)) {
				// Failed to forward packet, drop it.
				free(state->currentFramePacket);
				caerLog(LOG_INFO, state->sourceSubSystemString, "Dropped Frame Event Packet because ring-buffer full!");
			}
			else {
				caerMainloopDataAvailableIncrease(state->mainloopNotify);
			}

			// Allocate new packet for next iteration.
			state->currentFramePacket = caerFrameEventPacketAllocate(state->maxFramePacketSize, state->sourceID,
				state->apsSizeX, state->apsSizeY, DAVIS_COLOR_CHANNELS);
			state->currentFramePacketPosition = 0;
		}
	}
}

libusb_device_handle *deviceOpen(libusb_context *devContext, uint16_t devVID, uint16_t devPID, uint8_t devType,
	uint8_t busNumber, uint8_t devAddress) {
	libusb_device_handle *devHandle = NULL;
	libusb_device **devicesList;

	ssize_t result = libusb_get_device_list(devContext, &devicesList);

	if (result >= 0) {
		// Cycle thorough all discovered devices and find a match.
		for (size_t i = 0; i < (size_t) result; i++) {
			struct libusb_device_descriptor devDesc;

			if (libusb_get_device_descriptor(devicesList[i], &devDesc) != LIBUSB_SUCCESS) {
				continue;
			}

			// Check if this is the device we want (VID/PID).
			if (devDesc.idVendor == devVID && devDesc.idProduct == devPID
				&& (uint8_t) ((devDesc.bcdDevice & 0xFF00) >> 8) == devType) {
				// If a USB port restriction is given, honor it.
				if (busNumber > 0 && libusb_get_bus_number(devicesList[i]) != busNumber) {
					continue;
				}

				if (devAddress > 0 && libusb_get_device_address(devicesList[i]) != devAddress) {
					continue;
				}

				if (libusb_open(devicesList[i], &devHandle) != LIBUSB_SUCCESS) {
					devHandle = NULL;

					continue;
				}

				// Check that the active configuration is set to number 1. If not, do so.
				int activeConfiguration;
				if (libusb_get_configuration(devHandle, &activeConfiguration) != LIBUSB_SUCCESS) {
					libusb_close(devHandle);
					devHandle = NULL;

					continue;
				}

				if (activeConfiguration != 1) {
					if (libusb_set_configuration(devHandle, 1) != LIBUSB_SUCCESS) {
						libusb_close(devHandle);
						devHandle = NULL;

						continue;
					}
				}

				// Claim interface 0 (default).
				if (libusb_claim_interface(devHandle, 0) != LIBUSB_SUCCESS) {
					libusb_close(devHandle);
					devHandle = NULL;

					continue;
				}

				// Found and configured it!
				break;
			}
		}

		libusb_free_device_list(devicesList, true);
	}

	return (devHandle);
}

void deviceClose(libusb_device_handle *devHandle) {
	// Release interface 0 (default).
	libusb_release_interface(devHandle, 0);

	libusb_close(devHandle);
}

void caerInputDAVISCommonConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(changeValue);

	caerModuleData data = userData;

	// Distinguish changes to biases, or USB transfers, or others, by
	// using configUpdate like a bit-field.
	if (event == ATTRIBUTE_MODIFIED) {
		// Changes to the bias node.
		if (str_equals(sshsNodeGetName(node), "bias")) {
			atomic_ops_uint_or(&data->configUpdate, (0x01 << 0), ATOMIC_OPS_FENCE_NONE);
		}

		// Changes to the chip config node.
		if (str_equals(sshsNodeGetName(node), "chip")) {
			atomic_ops_uint_or(&data->configUpdate, (0x01 << 1), ATOMIC_OPS_FENCE_NONE);
		}

		// Changes to the FPGA config nodes.
		if (str_equals(sshsNodeGetName(node), "Multiplexer")) {
			atomic_ops_uint_or(&data->configUpdate, (0x01 << 2), ATOMIC_OPS_FENCE_NONE);
		}

		if (str_equals(sshsNodeGetName(node), "DVS")) {
			atomic_ops_uint_or(&data->configUpdate, (0x01 << 3), ATOMIC_OPS_FENCE_NONE);
		}

		if (str_equals(sshsNodeGetName(node), "APS")) {
			atomic_ops_uint_or(&data->configUpdate, (0x01 << 4), ATOMIC_OPS_FENCE_NONE);
		}

		if (str_equals(sshsNodeGetName(node), "IMU")) {
			atomic_ops_uint_or(&data->configUpdate, (0x01 << 5), ATOMIC_OPS_FENCE_NONE);
		}

		if (str_equals(sshsNodeGetName(node), "ExternalInput")) {
			atomic_ops_uint_or(&data->configUpdate, (0x01 << 6), ATOMIC_OPS_FENCE_NONE);
		}

		if (str_equals(sshsNodeGetName(node), "USB")) {
			atomic_ops_uint_or(&data->configUpdate, (0x01 << 7), ATOMIC_OPS_FENCE_NONE);
		}

		// Changes to the USB transfer settings (requires reallocation).
		if (changeType == INT && (str_equals(changeKey, "bufferNumber") || str_equals(changeKey, "bufferSize"))) {
			atomic_ops_uint_or(&data->configUpdate, (0x01 << 8), ATOMIC_OPS_FENCE_NONE);
		}

		// Changes to packet size and interval.
		if (changeType == INT
			&& (str_equals(changeKey, "polarityPacketMaxSize") || str_equals(changeKey, "polarityPacketMaxInterval")
				|| str_equals(changeKey, "framePacketMaxSize") || str_equals(changeKey, "framePacketMaxInterval")
				|| str_equals(changeKey, "imu6PacketMaxSize") || str_equals(changeKey, "imu6PacketMaxInterval")
				|| str_equals(changeKey, "specialPacketMaxSize") || str_equals(changeKey, "specialPacketMaxInterval"))) {
			atomic_ops_uint_or(&data->configUpdate, (0x01 << 9), ATOMIC_OPS_FENCE_NONE);
		}
	}
}
