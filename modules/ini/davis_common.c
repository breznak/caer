#include "davis_common.h"
#include <pthread.h>
#include <unistd.h>

static void freeAllMemory(davisCommonState state);
static void createVDACBiasSetting(biasDescriptor *chipBiases, sshsNode biasNode, const char *biasName,
	uint8_t biasAddress, uint8_t currentValue, uint8_t voltageValue);
static uint16_t generateVDACBias(sshsNode biasNode, const char *biasName);
static void createCoarseFineBiasSetting(biasDescriptor *chipBiases, sshsNode biasNode, const char *biasName,
	uint8_t biasAddress, const char *type, const char *sex, uint8_t coarseValue, uint8_t fineValue, bool enabled);
static uint16_t generateCoarseFineBias(sshsNode biasNode, const char *biasName);
static void createShiftedSourceBiasSetting(biasDescriptor *chipBiases, sshsNode biasNode, const char *biasName,
	uint8_t biasAddress, uint8_t regValue, uint8_t refValue, const char *operatingMode, const char *voltageLevel);
static uint16_t generateShiftedSourceBias(sshsNode biasNode, const char *biasName);
static void createBoolConfigSetting(configChainDescriptor *chipConfigChain, sshsNode configNode, const char *configName,
	uint8_t configAddress, bool defaultValue);
static void createByteConfigSetting(configChainDescriptor *chipConfigChain, sshsNode configNode, const char *configName,
	uint8_t configAddress, uint8_t defaultValue);
static void LIBUSB_CALL libUsbDataCallback(struct libusb_transfer *transfer);
static void dataTranslator(davisCommonState state, uint8_t *buffer, size_t bytesSent);
static libusb_device_handle *deviceOpen(libusb_context *devContext, uint16_t devVID, uint16_t devPID, uint8_t devType,
	uint8_t busNumber, uint8_t devAddress);
static void deviceClose(libusb_device_handle *devHandle);
static void sendUSBConfig(sshsNode moduleNode, libusb_device_handle *devHandle);
static void sendMultiplexerConfig(sshsNode moduleNode, libusb_device_handle *devHandle);
static void sendDVSConfig(sshsNode moduleNode, libusb_device_handle *devHandle);
static void sendAPSConfig(sshsNode moduleNode, libusb_device_handle *devHandle);
static void sendIMUConfig(sshsNode moduleNode, libusb_device_handle *devHandle);
static void sendExternalInputDetectorConfig(sshsNode moduleNode, libusb_device_handle *devHandle);
static void USBConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void MultiplexerConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void DVSConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void APSConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void IMUConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void ExternalInputDetectorConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void HostConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void reallocateUSBBuffers(sshsNode moduleNode, davisCommonState state);
static void updatePacketSizesIntervals(sshsNode moduleNode, davisCommonState state);

static inline void checkMonotonicTimestamp(davisCommonState state) {
	if (state->currentTimestamp <= state->lastTimestamp) {
		caerLog(LOG_ALERT, state->sourceSubSystemString,
			"Timestamps: non strictly-monotonic timestamp detected: lastTimestamp=%" PRIu32 ", currentTimestamp=%" PRIu32 ", difference=%" PRIu32 ".",
			state->lastTimestamp, state->currentTimestamp, (state->lastTimestamp - state->currentTimestamp));
	}
}

static inline void initFrame(davisCommonState state, caerFrameEvent currentFrameEvent) {
	state->apsCurrentReadoutType = APS_READOUT_RESET;
	for (size_t j = 0; j < APS_READOUT_TYPES_NUM; j++) {
		state->apsCountX[j] = 0;
		state->apsCountY[j] = 0;
	}

	if (currentFrameEvent != NULL) {
		// Write out start of frame timestamp.
		caerFrameEventSetTSStartOfFrame(currentFrameEvent, state->currentTimestamp);

		// Setup frame.
		caerFrameEventSetChannelNumber(currentFrameEvent, DAVIS_COLOR_CHANNELS);
		caerFrameEventSetLengthXY(currentFrameEvent, state->currentFramePacket, state->apsWindow0SizeX,
			state->apsWindow0SizeY);
	}
}

static inline float calculateIMUAccelScale(uint8_t imuAccelScale) {
	// Accelerometer scale is:
	// 0 - +-2 g - 16384 LSB/g
	// 1 - +-4 g - 8192 LSB/g
	// 2 - +-8 g - 4096 LSB/g
	// 3 - +-16 g - 2048 LSB/g
	float accelScale = 65536.0f / (float) U32T(4 * (1 << imuAccelScale));

	return (accelScale);
}

static inline float calculateIMUGyroScale(uint8_t imuGyroScale) {
	// Gyroscope scale is:
	// 0 - +-250 °/s - 131 LSB/°/s
	// 1 - +-500 °/s - 65.5 LSB/°/s
	// 2 - +-1000 °/s - 32.8 LSB/°/s
	// 3 - +-2000 °/s - 16.4 LSB/°/s
	float gyroScale = 65536.0f / (float) U32T(500 * (1 << imuGyroScale));

	return (gyroScale);
}

static void freeAllMemory(davisCommonState state) {
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

static void createVDACBiasSetting(biasDescriptor *chipBiases, sshsNode biasNode, const char *biasName,
	uint8_t biasAddress, uint8_t currentValue, uint8_t voltageValue) {
	size_t biasNameLength = strlen(biasName);

	// Find first free bias descriptor slot.
	size_t i;
	for (i = 0; i < BIAS_MAX_NUM_DESC; i++) {
		if (chipBiases[i] == NULL) {
			// Found empty slot.
			break;
		}
	}

	// Allocate memory for bias descriptor.
	chipBiases[i] = calloc(1, sizeof(struct bias_descriptor) + biasNameLength + 1); // +1 for string closing NUL character.
	if (chipBiases[i] == NULL) {
		caerLog(LOG_EMERGENCY, "DAVIS VDAC Bias", "Unable to allocate memory for bias configuration.");
		exit(EXIT_FAILURE);
	}

	// Setup bias descriptor for dynamic configuration.
	chipBiases[i]->address = biasAddress;
	chipBiases[i]->generatorFunction = &generateVDACBias;
	chipBiases[i]->nameLength = biasNameLength;
	memcpy(chipBiases[i]->name, biasName, biasNameLength);
	chipBiases[i]->name[biasNameLength] = '\0';

	// Add trailing slash to node name (required!).
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

	uint16_t biasValue = 0;

	// Build up bias value from all its components.
	biasValue |= U16T((sshsNodeGetByte(biasConfigNode, "voltageValue") & 0x3F) << 0);
	biasValue |= U16T((sshsNodeGetByte(biasConfigNode, "currentValue") & 0x07) << 6);

	return (biasValue);
}

static void createCoarseFineBiasSetting(biasDescriptor *chipBiases, sshsNode biasNode, const char *biasName,
	uint8_t biasAddress, const char *type, const char *sex, uint8_t coarseValue, uint8_t fineValue, bool enabled) {
	size_t biasNameLength = strlen(biasName);

	// Find first free bias descriptor slot.
	size_t i;
	for (i = 0; i < BIAS_MAX_NUM_DESC; i++) {
		if (chipBiases[i] == NULL) {
			// Found empty slot.
			break;
		}
	}

	// Allocate memory for bias descriptor.
	chipBiases[i] = calloc(1, sizeof(struct bias_descriptor) + biasNameLength + 1); // +1 for string closing NUL character.
	if (chipBiases[i] == NULL) {
		caerLog(LOG_EMERGENCY, "DAVIS CF Bias", "Unable to allocate memory for bias configuration.");
		exit(EXIT_FAILURE);
	}

	// Setup bias descriptor for dynamic configuration.
	chipBiases[i]->address = biasAddress;
	chipBiases[i]->generatorFunction = &generateCoarseFineBias;
	chipBiases[i]->nameLength = biasNameLength;
	memcpy(chipBiases[i]->name, biasName, biasNameLength);
	chipBiases[i]->name[biasNameLength] = '\0';

	// Add trailing slash to node name (required!).
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

	biasValue |= U16T((sshsNodeGetByte(biasConfigNode, "fineValue") & 0xFF) << 4);
	biasValue |= U16T((sshsNodeGetByte(biasConfigNode, "coarseValue") & 0x07) << 12);

	return (biasValue);
}

static void createShiftedSourceBiasSetting(biasDescriptor *chipBiases, sshsNode biasNode, const char *biasName,
	uint8_t biasAddress, uint8_t regValue, uint8_t refValue, const char *operatingMode, const char *voltageLevel) {
	size_t biasNameLength = strlen(biasName);

	// Find first free bias descriptor slot.
	size_t i;
	for (i = 0; i < BIAS_MAX_NUM_DESC; i++) {
		if (chipBiases[i] == NULL) {
			// Found empty slot.
			break;
		}
	}

	// Allocate memory for bias descriptor.
	chipBiases[i] = calloc(1, sizeof(struct bias_descriptor) + biasNameLength + 1); // +1 for string closing NUL character.
	if (chipBiases[i] == NULL) {
		caerLog(LOG_EMERGENCY, "DAVIS SS Bias", "Unable to allocate memory for bias configuration.");
		exit(EXIT_FAILURE);
	}

	// Setup bias descriptor for dynamic configuration.
	chipBiases[i]->address = biasAddress;
	chipBiases[i]->generatorFunction = &generateShiftedSourceBias;
	chipBiases[i]->nameLength = biasNameLength;
	memcpy(chipBiases[i]->name, biasName, biasNameLength);
	chipBiases[i]->name[biasNameLength] = '\0';

	// Add trailing slash to node name (required!).
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

	biasValue |= U16T((sshsNodeGetByte(biasConfigNode, "refValue") & 0x3F) << 4);
	biasValue |= U16T((sshsNodeGetByte(biasConfigNode, "regValue") & 0x3F) << 10);

	return (biasValue);
}

static void createBoolConfigSetting(configChainDescriptor *chipConfigChain, sshsNode configNode, const char *configName,
	uint8_t configAddress, bool defaultValue) {
	size_t configNameLength = strlen(configName);

	// Find first free config-chain descriptor slot.
	size_t i;
	for (i = 0; i < CONFIGCHAIN_MAX_NUM_DESC; i++) {
		if (chipConfigChain[i] == NULL) {
			// Found empty slot.
			break;
		}
	}

	// Allocate memory for config-chain descriptor.
	chipConfigChain[i] = calloc(1, sizeof(struct configchain_descriptor) + configNameLength + 1); // +1 for string closing NUL character.
	if (chipConfigChain[i] == NULL) {
		caerLog(LOG_EMERGENCY, "DAVIS Bool ConfigChain", "Unable to allocate memory for config-chain configuration.");
		exit(EXIT_FAILURE);
	}

	// Setup config-chain descriptor for dynamic configuration.
	chipConfigChain[i]->address = configAddress;
	chipConfigChain[i]->type = BOOL;
	chipConfigChain[i]->nameLength = configNameLength;
	memcpy(chipConfigChain[i]->name, configName, configNameLength);
	chipConfigChain[i]->name[configNameLength] = '\0';

	// Update SSHS node with current configuration.
	sshsNodePutBoolIfAbsent(configNode, configName, defaultValue);
}

static void createByteConfigSetting(configChainDescriptor *chipConfigChain, sshsNode configNode, const char *configName,
	uint8_t configAddress, uint8_t defaultValue) {
	size_t configNameLength = strlen(configName);

	// Find first free config-chain descriptor slot.
	size_t i;
	for (i = 0; i < CONFIGCHAIN_MAX_NUM_DESC; i++) {
		if (chipConfigChain[i] == NULL) {
			// Found empty slot.
			break;
		}
	}

	// Allocate memory for config-chain descriptor.
	chipConfigChain[i] = calloc(1, sizeof(struct configchain_descriptor) + configNameLength + 1); // +1 for string closing NUL character.
	if (chipConfigChain[i] == NULL) {
		caerLog(LOG_EMERGENCY, "DAVIS Byte ConfigChain", "Unable to allocate memory for config-chain configuration.");
		exit(EXIT_FAILURE);
	}

	// Setup config-chain descriptor for dynamic configuration.
	chipConfigChain[i]->address = configAddress;
	chipConfigChain[i]->type = BYTE;
	chipConfigChain[i]->nameLength = configNameLength;
	memcpy(chipConfigChain[i]->name, configName, configNameLength);
	chipConfigChain[i]->name[configNameLength] = '\0';

	// Update SSHS node with current configuration.
	sshsNodePutByteIfAbsent(configNode, configName, defaultValue);
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
	sshsNode selectorNode = sshsGetRelativeNode(moduleData->moduleNode, "usbDevice/");

	sshsNodePutByteIfAbsent(selectorNode, "BusNumber", 0);
	sshsNodePutByteIfAbsent(selectorNode, "DevAddress", 0);
	sshsNodePutStringIfAbsent(selectorNode, "SerialNumber", "");

	// Initialize libusb using a separate context for each device.
	// This is to correctly support one thread per device.
	if ((errno = libusb_init(&cstate->deviceContext)) != LIBUSB_SUCCESS) {
		caerLog(LOG_CRITICAL, moduleData->moduleSubSystemString, "Failed to initialize libusb context. Error: %s (%d).",
			libusb_strerror(errno), errno);
		return (false);
	}

	// Try to open a DAVIS device on a specific USB port.
	cstate->deviceHandle = deviceOpen(cstate->deviceContext, VID, PID, DID_TYPE,
		sshsNodeGetByte(selectorNode, "BusNumber"), sshsNodeGetByte(selectorNode, "DevAddress"));
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
		sshsNodeGetName(moduleData->moduleNode), serialNumber, busNumber, devAddress);
	char fullLogString[fullLogStringLength + 1];
	snprintf(fullLogString, fullLogStringLength + 1, "%s SN-%s [%" PRIu8 ":%" PRIu8 "]",
		sshsNodeGetName(moduleData->moduleNode), serialNumber, busNumber, devAddress);

	// Update module log string, make it accessible in cstate space.
	caerModuleSetSubSystemString(moduleData, fullLogString);
	cstate->sourceSubSystemString = moduleData->moduleSubSystemString;

	// Now check if the Serial Number matches.
	char *configSerialNumber = sshsNodeGetString(selectorNode, "SerialNumber");

	if (!str_equals(configSerialNumber, "") && !str_equals(configSerialNumber, serialNumber)) {
		libusb_close(cstate->deviceHandle);
		libusb_exit(cstate->deviceContext);

		caerLog(LOG_CRITICAL, moduleData->moduleSubSystemString, "Device Serial Number doesn't match.");
		return (false);
	}

	free(configSerialNumber);

	// So now we have a working connection to the device we want. Let's get some data!
	cstate->chipID = U16T(spiConfigReceive(cstate->deviceHandle, FPGA_SYSINFO, 1));
	cstate->apsSizeX = U16T(spiConfigReceive(cstate->deviceHandle, FPGA_SYSINFO, 2));
	cstate->apsSizeY = U16T(spiConfigReceive(cstate->deviceHandle, FPGA_SYSINFO, 3));
	cstate->dvsSizeX = U16T(spiConfigReceive(cstate->deviceHandle, FPGA_SYSINFO, 4));
	cstate->dvsSizeY = U16T(spiConfigReceive(cstate->deviceHandle, FPGA_SYSINFO, 5));

	// Put global source information into SSHS, so it's globally available.
	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	sshsNodePutShort(sourceInfoNode, "logicVersion", U16T(spiConfigReceive(cstate->deviceHandle, FPGA_SYSINFO, 0)));
	sshsNodePutShort(sourceInfoNode, "dvsSizeX", cstate->dvsSizeX);
	sshsNodePutShort(sourceInfoNode, "dvsSizeY", cstate->dvsSizeY);
	sshsNodePutShort(sourceInfoNode, "apsSizeX", cstate->apsSizeX);
	sshsNodePutShort(sourceInfoNode, "apsSizeY", cstate->apsSizeY);
	sshsNodePutShort(sourceInfoNode, "apsOriginalDepth", DAVIS_ADC_DEPTH);
	sshsNodePutShort(sourceInfoNode, "apsOriginalChannels", DAVIS_COLOR_CHANNELS);
	sshsNodePutBool(sourceInfoNode, "apsHasGlobalShutter", spiConfigReceive(cstate->deviceHandle, FPGA_SYSINFO, 6));
	sshsNodePutBool(sourceInfoNode, "apsHasIntegratedADC", spiConfigReceive(cstate->deviceHandle, FPGA_SYSINFO, 7));
	sshsNodePutBool(sourceInfoNode, "deviceIsMaster", spiConfigReceive(cstate->deviceHandle, FPGA_SYSINFO, 8));

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

	if (cstate->chipID == CHIP_DAVIS128 || cstate->chipID == CHIP_DAVIS346A || cstate->chipID == CHIP_DAVIS346B
		|| cstate->chipID == CHIP_DAVIS640 || cstate->chipID == CHIP_DAVIS208) {
		createVDACBiasSetting(biases, biasNode, "ApsOverflowLevel", 0, 6, 27);
		createVDACBiasSetting(biases, biasNode, "ApsCas", 1, 6, 21);
		createVDACBiasSetting(biases, biasNode, "AdcRefHigh", 2, 6, 52);
		createVDACBiasSetting(biases, biasNode, "AdcRefLow", 3, 6, 23);
		createVDACBiasSetting(biases, biasNode, "AdcTestVoltage", 4, 6, 35);

		createCoarseFineBiasSetting(biases, biasNode, "LocalBufBn", 8, "Normal", "N", 5, 164, true);
		createCoarseFineBiasSetting(biases, biasNode, "PadFollBn", 9, "Normal", "N", 7, 215, true);
		createCoarseFineBiasSetting(biases, biasNode, "DiffBn", 10, "Normal", "N", 4, 39, true);
		createCoarseFineBiasSetting(biases, biasNode, "OnBn", 11, "Normal", "N", 5, 255, true);
		createCoarseFineBiasSetting(biases, biasNode, "OffBn", 12, "Normal", "N", 4, 0, true);
		createCoarseFineBiasSetting(biases, biasNode, "PixInvBn", 13, "Normal", "N", 5, 129, true);
		createCoarseFineBiasSetting(biases, biasNode, "PrBp", 14, "Normal", "P", 2, 58, true);
		createCoarseFineBiasSetting(biases, biasNode, "PrSFBp", 15, "Normal", "P", 1, 16, true);
		createCoarseFineBiasSetting(biases, biasNode, "RefrBp", 16, "Normal", "P", 4, 25, true);
		createCoarseFineBiasSetting(biases, biasNode, "ReadoutBufBp", 17, "Normal", "P", 6, 20, true);
		createCoarseFineBiasSetting(biases, biasNode, "ApsROSFBn", 18, "Normal", "N", 6, 219, true);
		createCoarseFineBiasSetting(biases, biasNode, "AdcCompBp", 19, "Normal", "P", 4, 20, true);
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
		createVDACBiasSetting(biases, biasNode, "ResetHighPass", 5, 7, 63);
		createVDACBiasSetting(biases, biasNode, "RefSS", 6, 5, 11);

		createCoarseFineBiasSetting(biases, biasNode, "RegBiasBp", 28, "Normal", "P", 5, 20, true);
		createCoarseFineBiasSetting(biases, biasNode, "RefSSBn", 30, "Normal", "N", 5, 20, true);
	}

	if (cstate->chipID == CHIP_DAVISRGB) {
		createVDACBiasSetting(biases, biasNode, "ApsCasBpc", 0, 6, 21);
		createVDACBiasSetting(biases, biasNode, "OVG1Lo", 1, 6, 27);
		createVDACBiasSetting(biases, biasNode, "OVG2Lo", 2, 0, 0);
		createVDACBiasSetting(biases, biasNode, "TX2OVG2Hi", 3, 4, 63);
		createVDACBiasSetting(biases, biasNode, "Gnd07", 4, 5, 13);
		createVDACBiasSetting(biases, biasNode, "vADCTest", 5, 6, 35);
		createVDACBiasSetting(biases, biasNode, "AdcRefHigh", 6, 6, 52);
		createVDACBiasSetting(biases, biasNode, "AdcRefLow", 7, 6, 23);

		createCoarseFineBiasSetting(biases, biasNode, "IFRefrBn", 8, "Normal", "N", 5, 255, true);
		createCoarseFineBiasSetting(biases, biasNode, "IFThrBn", 9, "Normal", "N", 5, 255, true);
		createCoarseFineBiasSetting(biases, biasNode, "LocalBufBn", 10, "Normal", "N", 5, 164, true);
		createCoarseFineBiasSetting(biases, biasNode, "PadFollBn", 11, "Normal", "N", 7, 215, true);
		createCoarseFineBiasSetting(biases, biasNode, "PixInvBn", 13, "Normal", "N", 5, 129, true);
		createCoarseFineBiasSetting(biases, biasNode, "DiffBn", 14, "Normal", "N", 4, 39, true);
		createCoarseFineBiasSetting(biases, biasNode, "OnBn", 15, "Normal", "N", 5, 255, true);
		createCoarseFineBiasSetting(biases, biasNode, "OffBn", 16, "Normal", "N", 4, 0, true);
		createCoarseFineBiasSetting(biases, biasNode, "PrBp", 17, "Normal", "P", 2, 58, true);
		createCoarseFineBiasSetting(biases, biasNode, "PrSFBp", 18, "Normal", "P", 1, 16, true);
		createCoarseFineBiasSetting(biases, biasNode, "RefrBp", 19, "Normal", "P", 4, 25, true);
		createCoarseFineBiasSetting(biases, biasNode, "ArrayBiasBufferBn", 20, "Normal", "N", 4, 10, true);
		createCoarseFineBiasSetting(biases, biasNode, "ArrayLogicBufferBn", 22, "Normal", "N", 4, 10, true);
		createCoarseFineBiasSetting(biases, biasNode, "FalltimeBn", 23, "Normal", "N", 3, 20, true);
		createCoarseFineBiasSetting(biases, biasNode, "RisetimeBp", 24, "Normal", "P", 3, 20, true);
		createCoarseFineBiasSetting(biases, biasNode, "ReadoutBufBp", 25, "Normal", "P", 6, 20, true);
		createCoarseFineBiasSetting(biases, biasNode, "ApsROSFBn", 26, "Normal", "N", 6, 219, true);
		createCoarseFineBiasSetting(biases, biasNode, "AdcCompBp", 27, "Normal", "P", 4, 20, true);
		createCoarseFineBiasSetting(biases, biasNode, "DACBufBp", 28, "Normal", "P", 6, 60, true);
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

	if (cstate->chipID == CHIP_DAVIS128 || cstate->chipID == CHIP_DAVIS208 || cstate->chipID == CHIP_DAVIS240A
		|| cstate->chipID == CHIP_DAVIS240B || cstate->chipID == CHIP_DAVIS240C) {
		createBoolConfigSetting(configChain, chipNode, "HotPixelSuppression", 139, false);
	}

	if (cstate->chipID == CHIP_DAVIS128 || cstate->chipID == CHIP_DAVIS208 || cstate->chipID == CHIP_DAVIS346A
		|| cstate->chipID == CHIP_DAVIS346B || cstate->chipID == CHIP_DAVIS640 || cstate->chipID == CHIP_DAVISRGB) {
		// Select which grey counter to use with the internal ADC: '0' means the external grey counter is used, which
		// has to be supplied off-chip. '1' means the on-chip grey counter is used instead.
		createBoolConfigSetting(configChain, chipNode, "SelectGrayCounter", 143, true);
	}

	if (cstate->chipID == CHIP_DAVIS346A || cstate->chipID == CHIP_DAVIS346B || cstate->chipID == CHIP_DAVIS640
		|| cstate->chipID == CHIP_DAVISRGB) {
		// Test ADC functionality: if true, the ADC takes its input voltage not from the pixel, but from the
		// VDAC 'AdcTestVoltage'. If false, the voltage comes from the pixels.
		createBoolConfigSetting(configChain, chipNode, "TestADC", 144, false);
	}

	if (cstate->chipID == CHIP_DAVIS208) {
		createBoolConfigSetting(configChain, chipNode, "SelectPreAmpAvg", 145, false);
		createBoolConfigSetting(configChain, chipNode, "SelectBiasRefSS", 146, false);
		createBoolConfigSetting(configChain, chipNode, "SelectSense", 147, false);
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

	sshsNodePutBoolIfAbsent(muxNode, "Run", 1);
	sshsNodePutBoolIfAbsent(muxNode, "TimestampRun", 1);
	sshsNodePutBoolIfAbsent(muxNode, "TimestampReset", 0);
	sshsNodePutBoolIfAbsent(muxNode, "ForceChipBiasEnable", 0);
	sshsNodePutBoolIfAbsent(muxNode, "DropDVSOnTransferStall", 1);
	sshsNodePutBoolIfAbsent(muxNode, "DropAPSOnTransferStall", 0);
	sshsNodePutBoolIfAbsent(muxNode, "DropIMUOnTransferStall", 1);
	sshsNodePutBoolIfAbsent(muxNode, "DropExtInputOnTransferStall", 1);

	// Subsystem 1: DVS AER
	sshsNode dvsNode = sshsGetRelativeNode(moduleData->moduleNode, "dvs/");

	sshsNodePutBoolIfAbsent(dvsNode, "Run", 1);
	sshsNodePutByteIfAbsent(dvsNode, "AckDelayRow", 4);
	sshsNodePutByteIfAbsent(dvsNode, "AckDelayColumn", 0);
	sshsNodePutByteIfAbsent(dvsNode, "AckExtensionRow", 1);
	sshsNodePutByteIfAbsent(dvsNode, "AckExtensionColumn", 0);
	sshsNodePutBoolIfAbsent(dvsNode, "WaitOnTransferStall", 0);
	sshsNodePutBoolIfAbsent(dvsNode, "FilterRowOnlyEvents", 1);

	// Subsystem 2: APS ADC
	sshsNode apsNode = sshsGetRelativeNode(moduleData->moduleNode, "aps/");

	// Only support GS on chips that have it available.
	bool globalShutterSupported = sshsNodeGetBool(sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/"),
		"apsHasGlobalShutter");
	if (globalShutterSupported) {
		sshsNodePutBoolIfAbsent(apsNode, "GlobalShutter", globalShutterSupported);
	}

	sshsNodePutBoolIfAbsent(apsNode, "Run", 1);
	sshsNodePutBoolIfAbsent(apsNode, "ForceADCRunning", 0);
	sshsNodePutShortIfAbsent(apsNode, "StartColumn0", 0);
	sshsNodePutShortIfAbsent(apsNode, "StartRow0", 0);
	sshsNodePutShortIfAbsent(apsNode, "EndColumn0", U16T(cstate->apsSizeX - 1));
	sshsNodePutShortIfAbsent(apsNode, "EndRow0", U16T(cstate->apsSizeY - 1));
	sshsNodePutIntIfAbsent(apsNode, "Exposure", 2000); // in µs, converted to cycles later
	sshsNodePutIntIfAbsent(apsNode, "FrameDelay", 200); // in µs, converted to cycles later
	sshsNodePutShortIfAbsent(apsNode, "ResetSettle", 10); // in cycles
	sshsNodePutShortIfAbsent(apsNode, "ColumnSettle", 30); // in cycles
	sshsNodePutShortIfAbsent(apsNode, "RowSettle", 10); // in cycles
	sshsNodePutShortIfAbsent(apsNode, "NullSettle", 10); // in cycles
	sshsNodePutBoolIfAbsent(apsNode, "ResetRead", 1);
	sshsNodePutBoolIfAbsent(apsNode, "WaitOnTransferStall", 0);

	// Subsystem 3: IMU
	sshsNode imuNode = sshsGetRelativeNode(moduleData->moduleNode, "imu/");

	sshsNodePutBoolIfAbsent(imuNode, "Run", 1);
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
	sshsNode extNode = sshsGetRelativeNode(moduleData->moduleNode, "externalInput/");

	sshsNodePutBoolIfAbsent(extNode, "RunDetector", 0);
	sshsNodePutBoolIfAbsent(extNode, "DetectRisingEdges", 0);
	sshsNodePutBoolIfAbsent(extNode, "DetectFallingEdges", 0);
	sshsNodePutBoolIfAbsent(extNode, "DetectPulses", 1);
	sshsNodePutBoolIfAbsent(extNode, "DetectPulsePolarity", 1);
	sshsNodePutIntIfAbsent(extNode, "DetectPulseLength", 10);

	// Subsystem 9: FX2/3 USB Configuration
	sshsNode usbNode = sshsGetRelativeNode(moduleData->moduleNode, "usb/");

	sshsNodePutBoolIfAbsent(usbNode, "Run", 1);
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

bool initializeCommonConfiguration(caerModuleData moduleData, davisCommonState cstate,
	void *dataAcquisitionThread(void *inPtr)) {
	// Initialize state fields.
	updatePacketSizesIntervals(moduleData->moduleNode, cstate);

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
	sshsNode imuNode = sshsGetRelativeNode(moduleData->moduleNode, "imu/");
	cstate->imuIgnoreEvents = false;
	cstate->imuCount = 0;
	cstate->imuTmpData = 0;
	cstate->imuAccelScale = calculateIMUAccelScale(sshsNodeGetByte(imuNode, "AccelFullScale"));
	cstate->imuGyroScale = calculateIMUGyroScale(sshsNodeGetByte(imuNode, "GyroFullScale"));
	sshsNode apsNode = sshsGetRelativeNode(moduleData->moduleNode, "aps/");
	cstate->apsIgnoreEvents = false;
	cstate->apsWindow0StartX = sshsNodeGetShort(apsNode, "StartColumn0");
	cstate->apsWindow0StartY = sshsNodeGetShort(apsNode, "StartRow0");
	cstate->apsWindow0SizeX = U16T(
		sshsNodeGetShort(apsNode, "EndColumn0") + 1 - sshsNodeGetShort(apsNode, "StartColumn0"));
	cstate->apsWindow0SizeY = U16T(sshsNodeGetShort(apsNode, "EndRow0") + 1 - sshsNodeGetShort(apsNode, "StartRow0"));
	cstate->apsResetRead = sshsNodeGetBool(apsNode, "ResetRead");
	if (sshsNodeAttrExists(apsNode, "GlobalShutter", BOOL)) {
		cstate->apsGlobalShutter = sshsNodeGetBool(apsNode, "GlobalShutter");
	}
	else {
		cstate->apsGlobalShutter = false;
	}
	initFrame(cstate, NULL);
	cstate->apsCurrentResetFrame = calloc((size_t) cstate->apsSizeX * cstate->apsSizeY * DAVIS_COLOR_CHANNELS,
		sizeof(uint16_t));
	if (cstate->apsCurrentResetFrame == NULL) {
		freeAllMemory(cstate);

		caerLog(LOG_CRITICAL, moduleData->moduleSubSystemString, "Failed to allocate reset frame array.");
		return (false);
	}

	// Store reference to parent mainloop, so that we can correctly notify
	// the availability or not of data to consume.
	cstate->mainloopNotify = caerMainloopGetReference();

	// Create data exchange buffers. Size is fixed until module restart.
	cstate->dataExchangeBuffer = ringBufferInit(
		sshsNodeGetInt(sshsGetRelativeNode(moduleData->moduleNode, "system/"), "DataExchangeBufferSize"));
	if (cstate->dataExchangeBuffer == NULL) {
		freeAllMemory(cstate);

		caerLog(LOG_CRITICAL, moduleData->moduleSubSystemString, "Failed to initialize data exchange buffer.");
		return (false);
	}

	// Start data acquisition thread.
	if ((errno = pthread_create(&cstate->dataAcquisitionThread, NULL, dataAcquisitionThread, moduleData)) != 0) {
		freeAllMemory(cstate);

		caerLog(LOG_CRITICAL, moduleData->moduleSubSystemString,
			"Failed to start data acquisition thread. Error: %s (%d).", caerLogStrerror(errno), errno);
		return (false);
	}

	return (true);
}

void caerInputDAVISCommonExit(caerModuleData moduleData) {
	caerLog(LOG_DEBUG, moduleData->moduleSubSystemString, "Shutting down ...");

	// The common state is always the first member of the moduleState structure
	// for the DAVIS modules, so we can trust it being at address offset 0.
	davisCommonState cstate = moduleData->moduleState;

	// Wait for data acquisition thread to terminate...
	if ((errno = pthread_join(cstate->dataAcquisitionThread, NULL)) != 0) {
		// This should never happen!
		caerLog(LOG_CRITICAL, moduleData->moduleSubSystemString,
			"Failed to join data acquisition thread. Error: %s (%d).", caerLogStrerror(errno), errno);
	}

	// Finally, close the device fully.
	deviceClose(cstate->deviceHandle);

	// Destroy libusb context.
	libusb_exit(cstate->deviceContext);

	// Empty ringbuffer.
	void *packet;
	while ((packet = ringBufferGet(cstate->dataExchangeBuffer)) != NULL) {
		caerMainloopDataAvailableDecrease(cstate->mainloopNotify);
		free(packet);
	}

	// Free remaining incomplete packets.
	freeAllMemory(cstate);

	caerLog(LOG_DEBUG, moduleData->moduleSubSystemString, "Shutdown successful.");
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
	// Set number of transfers and allocate memory for the main transfer array.
	state->dataTransfers = calloc(bufferNum, sizeof(struct libusb_transfer *));
	if (state->dataTransfers == NULL) {
		caerLog(LOG_CRITICAL, state->sourceSubSystemString,
			"Failed to allocate memory for %" PRIu32 " libusb transfers (data channel). Error: %s (%d).", bufferNum,
			caerLogStrerror(errno), errno);
		return;
	}
	state->dataTransfersLength = bufferNum;

	// Allocate transfers and set them up.
	for (size_t i = 0; i < bufferNum; i++) {
		state->dataTransfers[i] = libusb_alloc_transfer(0);
		if (state->dataTransfers[i] == NULL) {
			caerLog(LOG_CRITICAL, state->sourceSubSystemString,
				"Unable to allocate further libusb transfers (data channel, %zu of %" PRIu32 ").", i, bufferNum);
			continue;
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

			continue;
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
			state->activeDataTransfers++;
		}
		else {
			caerLog(LOG_CRITICAL, state->sourceSubSystemString,
				"Unable to submit libusb transfer %zu (data channel). Error: %s (%d).", i, libusb_strerror(errno),
				errno);

			// The transfer buffer is freed automatically here thanks to
			// the LIBUSB_TRANSFER_FREE_BUFFER flag set above.
			libusb_free_transfer(state->dataTransfers[i]);
			state->dataTransfers[i] = NULL;

			continue;
		}
	}

	if (state->activeDataTransfers == 0) {
		// Didn't manage to allocate any USB transfers, free array memory and log failure.
		free(state->dataTransfers);
		state->dataTransfers = NULL;
		state->dataTransfersLength = 0;

		caerLog(LOG_CRITICAL, state->sourceSubSystemString, "Unable to allocate any libusb transfers.");
	}
}

void deallocateDataTransfers(davisCommonState state) {
	// Cancel all current transfers first.
	for (size_t i = 0; i < state->dataTransfersLength; i++) {
		if (state->dataTransfers[i] != NULL) {
			errno = libusb_cancel_transfer(state->dataTransfers[i]);
			if (errno != LIBUSB_SUCCESS && errno != LIBUSB_ERROR_NOT_FOUND) {
				caerLog(LOG_CRITICAL, state->sourceSubSystemString,
					"Unable to cancel libusb transfer %zu (data channel). Error: %s (%d).", i, libusb_strerror(errno),
					errno);
				// Proceed with trying to cancel all transfers regardless of errors.
			}
		}
	}

	// Wait for all transfers to go away (0.1 seconds timeout).
	struct timeval te = { .tv_sec = 0, .tv_usec = 100000 };

	while (state->activeDataTransfers > 0) {
		libusb_handle_events_timeout(state->deviceContext, &te);
	}

	// The buffers and transfers have been deallocated in the callback.
	// Only the transfers array remains, which we free here.
	free(state->dataTransfers);
	state->dataTransfers = NULL;
	state->dataTransfersLength = 0;
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
	state->activeDataTransfers--;
	for (size_t i = 0; i < state->dataTransfersLength; i++) {
		// Remove from list, so we don't try to cancel it later on.
		if (state->dataTransfers[i] == transfer) {
			state->dataTransfers[i] = NULL;
		}
	}
	libusb_free_transfer(transfer);
}

static void dataTranslator(davisCommonState state, uint8_t *buffer, size_t bytesSent) {
	// Truncate off any extra partial event.
	if ((bytesSent & 0x01) != 0) {
		caerLog(LOG_ALERT, state->sourceSubSystemString, "%zu bytes received via USB, which is not a multiple of two.",
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
			checkMonotonicTimestamp(state);
		}
		else {
			// Get all current events, so we don't have to duplicate code in every branch.
			caerPolarityEvent currentPolarityEvent = caerPolarityEventPacketGetEvent(state->currentPolarityPacket,
				state->currentPolarityPacketPosition);
			caerFrameEvent currentFrameEvent = caerFrameEventPacketGetEvent(state->currentFramePacket,
				state->currentFramePacketPosition);
			caerIMU6Event currentIMU6Event = caerIMU6EventPacketGetEvent(state->currentIMU6Packet,
				state->currentIMU6PacketPosition);
			caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(state->currentSpecialPacket,
				state->currentSpecialPacketPosition);

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
							caerSpecialEventSetTimestamp(currentSpecialEvent, UINT32_MAX);
							caerSpecialEventSetType(currentSpecialEvent, TIMESTAMP_RESET);
							caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
							state->currentSpecialPacketPosition++;

							// Commit packets when doing a reset to clearly separate them.
							forcePacketCommit = true;

							// Update Master/Slave status on incoming TS resets.
							//sshsNode sourceInfoNode = caerMainloopGetSourceInfo(state->sourceID);
							//sshsNodePutBool(sourceInfoNode, "deviceIsMaster",
							//	spiConfigReceive(state->deviceHandle, FPGA_SYSINFO, 8));

							break;
						}

						case 2: { // External input (falling edge)
							caerLog(LOG_DEBUG, state->sourceSubSystemString,
								"External input (falling edge) event received.");

							caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
							caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT_FALLING_EDGE);
							caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
							state->currentSpecialPacketPosition++;
							break;
						}

						case 3: { // External input (rising edge)
							caerLog(LOG_DEBUG, state->sourceSubSystemString,
								"External input (rising edge) event received.");

							caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
							caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT_RISING_EDGE);
							caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
							state->currentSpecialPacketPosition++;
							break;
						}

						case 4: { // External input (pulse)
							caerLog(LOG_DEBUG, state->sourceSubSystemString, "External input (pulse) event received.");

							caerSpecialEventSetTimestamp(currentSpecialEvent, state->currentTimestamp);
							caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT_PULSE);
							caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
							state->currentSpecialPacketPosition++;
							break;
						}

						case 5: { // IMU Start (6 axes)
							caerLog(LOG_DEBUG, state->sourceSubSystemString, "IMU6 Start event received.");

							state->imuIgnoreEvents = false;
							state->imuCount = 0;

							caerIMU6EventSetTimestamp(currentIMU6Event, state->currentTimestamp);
							break;
						}

						case 7: // IMU End
							caerLog(LOG_DEBUG, state->sourceSubSystemString, "IMU End event received.");
							if (state->imuIgnoreEvents) {
								break;
							}

							if (state->imuCount == IMU6_COUNT) {
								caerIMU6EventValidate(currentIMU6Event, state->currentIMU6Packet);
								state->currentIMU6PacketPosition++;
							}
							else {
								caerLog(LOG_INFO, state->sourceSubSystemString,
									"IMU End: failed to validate IMU sample count (%" PRIu8 "), discarding samples.",
									state->imuCount);
							}
							break;

						case 8: { // APS Global Shutter Frame Start
							caerLog(LOG_DEBUG, state->sourceSubSystemString, "APS GS Frame Start event received.");
							state->apsIgnoreEvents = false;
							state->apsGlobalShutter = true;
							state->apsResetRead = true;

							initFrame(state, currentFrameEvent);

							break;
						}

						case 9: { // APS Rolling Shutter Frame Start
							caerLog(LOG_DEBUG, state->sourceSubSystemString, "APS RS Frame Start event received.");
							state->apsIgnoreEvents = false;
							state->apsGlobalShutter = false;
							state->apsResetRead = true;

							initFrame(state, currentFrameEvent);

							break;
						}

						case 10: { // APS Frame End
							caerLog(LOG_DEBUG, state->sourceSubSystemString, "APS Frame End event received.");
							if (state->apsIgnoreEvents) {
								break;
							}

							bool validFrame = true;

							for (size_t j = 0; j < APS_READOUT_TYPES_NUM; j++) {
								uint16_t checkValue = caerFrameEventGetLengthX(currentFrameEvent);

								// Check reset read against zero if disabled.
								if (j == APS_READOUT_RESET && !state->apsResetRead) {
									checkValue = 0;
								}

								caerLog(LOG_DEBUG, state->sourceSubSystemString, "APS Frame End: CountX[%zu] is %d.", j,
									state->apsCountX[j]);

								if (state->apsCountX[j] != checkValue) {
									caerLog(LOG_ERROR, state->sourceSubSystemString,
										"APS Frame End: wrong column count [%zu - %d] detected.", j,
										state->apsCountX[j]);
									validFrame = false;
								}
							}

							// Write out end of frame timestamp.
							caerFrameEventSetTSEndOfFrame(currentFrameEvent, state->currentTimestamp);

							// Validate event and advance frame packet position.
							if (validFrame) {
								caerFrameEventValidate(currentFrameEvent, state->currentFramePacket);
							}
							state->currentFramePacketPosition++;

							break;
						}

						case 11: { // APS Reset Column Start
							caerLog(LOG_DEBUG, state->sourceSubSystemString, "APS Reset Column Start event received.");
							if (state->apsIgnoreEvents) {
								break;
							}

							state->apsCurrentReadoutType = APS_READOUT_RESET;
							state->apsCountY[state->apsCurrentReadoutType] = 0;

							// The first Reset Column Read Start is also the start
							// of the exposure for the RS.
							if (!state->apsGlobalShutter && state->apsCountX[APS_READOUT_RESET] == 0) {
								caerFrameEventSetTSStartOfExposure(currentFrameEvent, state->currentTimestamp);
							}

							break;
						}

						case 12: { // APS Signal Column Start
							caerLog(LOG_DEBUG, state->sourceSubSystemString, "APS Signal Column Start event received.");
							if (state->apsIgnoreEvents) {
								break;
							}

							state->apsCurrentReadoutType = APS_READOUT_SIGNAL;
							state->apsCountY[state->apsCurrentReadoutType] = 0;

							// The first Signal Column Read Start is also always the end
							// of the exposure time, for both RS and GS.
							if (state->apsCountX[APS_READOUT_SIGNAL] == 0) {
								caerFrameEventSetTSEndOfExposure(currentFrameEvent, state->currentTimestamp);
							}

							break;
						}

						case 13: { // APS Column End
							caerLog(LOG_DEBUG, state->sourceSubSystemString, "APS Column End event received.");
							if (state->apsIgnoreEvents) {
								break;
							}

							caerLog(LOG_DEBUG, state->sourceSubSystemString, "APS Column End: CountX[%d] is %d.",
								state->apsCurrentReadoutType, state->apsCountX[state->apsCurrentReadoutType]);
							caerLog(LOG_DEBUG, state->sourceSubSystemString, "APS Column End: CountY[%d] is %d.",
								state->apsCurrentReadoutType, state->apsCountY[state->apsCurrentReadoutType]);

							if (state->apsCountY[state->apsCurrentReadoutType]
								!= caerFrameEventGetLengthY(currentFrameEvent)) {
								caerLog(LOG_ERROR, state->sourceSubSystemString,
									"APS Column End: wrong row count [%d - %d] detected.", state->apsCurrentReadoutType,
									state->apsCountY[state->apsCurrentReadoutType]);
							}

							state->apsCountX[state->apsCurrentReadoutType]++;

							// The last Reset Column Read End is also the start
							// of the exposure for the GS.
							if (state->apsGlobalShutter && state->apsCurrentReadoutType == APS_READOUT_RESET
								&& state->apsCountX[APS_READOUT_RESET] == caerFrameEventGetLengthX(currentFrameEvent)) {
								caerFrameEventSetTSStartOfExposure(currentFrameEvent, state->currentTimestamp);
							}

							break;
						}

						case 14: { // APS Global Shutter Frame Start with no Reset Read
							caerLog(LOG_DEBUG, state->sourceSubSystemString,
								"APS GS NORST Frame Start event received.");
							state->apsIgnoreEvents = false;
							state->apsGlobalShutter = true;
							state->apsResetRead = false;

							initFrame(state, currentFrameEvent);

							// If reset reads are disabled, the start of exposure is closest to
							// the start of frame.
							caerFrameEventSetTSStartOfExposure(currentFrameEvent, state->currentTimestamp);

							break;
						}

						case 15: { // APS Rolling Shutter Frame Start with no Reset Read
							caerLog(LOG_DEBUG, state->sourceSubSystemString,
								"APS RS NORST Frame Start event received.");
							state->apsIgnoreEvents = false;
							state->apsGlobalShutter = false;
							state->apsResetRead = false;

							initFrame(state, currentFrameEvent);

							// If reset reads are disabled, the start of exposure is closest to
							// the start of frame.
							caerFrameEventSetTSStartOfExposure(currentFrameEvent, state->currentTimestamp);

							break;
						}

						case 16:
						case 17:
						case 18:
						case 19:
						case 20:
						case 21:
						case 22:
						case 23:
						case 24:
						case 25:
						case 26:
						case 27:
						case 28:
						case 29:
						case 30:
						case 31: {
							caerLog(LOG_DEBUG, state->sourceSubSystemString,
								"IMU Scale Config event (%" PRIu16 ") received.", data);
							if (state->imuIgnoreEvents) {
								break;
							}

							// Set correct IMU accel and gyro scales, used to interpret subsequent
							// IMU samples from the device.
							state->imuAccelScale = calculateIMUAccelScale((data >> 2) & 0x03);
							state->imuGyroScale = calculateIMUGyroScale(data & 0x03);

							// At this point the IMU event count should be zero (reset by start).
							if (state->imuCount != 0) {
								caerLog(LOG_INFO, state->sourceSubSystemString,
									"IMU Scale Config: previous IMU start event missed, attempting recovery.");
							}

							// Increase IMU count by one, to a total of one (0+1=1).
							// This way we can recover from the above error of missing start, and we can
							// later discover if the IMU Scale Config event actually arrived itself.
							state->imuCount = 1;

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
						caerLog(LOG_ALERT, state->sourceSubSystemString,
							"DVS: Y address out of range (0-%d): %" PRIu16 ".", state->dvsSizeY - 1, data);
						break; // Skip invalid Y address (don't update lastY).
					}

					if (state->dvsGotY) {
						// Use the previous timestamp here, since this refers to the previous Y.
						caerSpecialEventSetTimestamp(currentSpecialEvent, state->dvsTimestamp);
						caerSpecialEventSetType(currentSpecialEvent, DVS_ROW_ONLY);
						caerSpecialEventSetData(currentSpecialEvent, state->dvsLastY);
						caerSpecialEventValidate(currentSpecialEvent, state->currentSpecialPacket);
						state->currentSpecialPacketPosition++;

						caerLog(LOG_DEBUG, state->sourceSubSystemString,
							"DVS: row-only event received for address Y=%" PRIu16 ".", state->dvsLastY);
					}

					state->dvsLastY = data;
					state->dvsGotY = true;
					state->dvsTimestamp = state->currentTimestamp;

					break;

				case 2: // X address, Polarity OFF
				case 3: { // X address, Polarity ON
					// Check range conformity.
					if (data >= state->dvsSizeX) {
						caerLog(LOG_ALERT, state->sourceSubSystemString,
							"DVS: X address out of range (0-%d): %" PRIu16 ".", state->dvsSizeX - 1, data);
						break; // Skip invalid event.
					}

					caerPolarityEventSetTimestamp(currentPolarityEvent, state->dvsTimestamp);
					caerPolarityEventSetPolarity(currentPolarityEvent, (code & 0x01));
					caerPolarityEventSetY(currentPolarityEvent, state->dvsLastY);
					caerPolarityEventSetX(currentPolarityEvent, data);
					caerPolarityEventValidate(currentPolarityEvent, state->currentPolarityPacket);
					state->currentPolarityPacketPosition++;

					state->dvsGotY = false;

					break;
				}

				case 4: {
					if (state->apsIgnoreEvents) {
						break;
					}

					// First, let's normalize the ADC value to 16bit generic depth.
					data = U16T(data << (16 - DAVIS_ADC_DEPTH));

					// Let's check that apsCountY is not above the maximum. This could happen
					// if start/end of column events are discarded (no wait on transfer stall).
					if (state->apsCountY[state->apsCurrentReadoutType] >= caerFrameEventGetLengthY(currentFrameEvent)) {
						caerLog(LOG_DEBUG, state->sourceSubSystemString,
							"APS ADC sample: row count is at maximum, discarding further samples.");
						break;
					}

					// If reset read, we store the values in a local array. If signal read, we
					// store the final pixel value directly in the output frame event. We already
					// do the subtraction between reset and signal here, to avoid carrying that
					// around all the time and consuming memory. This way we can also only take
					// infrequent reset reads and re-use them for multiple frames, which can heavily
					// reduce traffic, and should not impact image quality heavily, at least in GS.
					uint16_t xPos = U16T(
						caerFrameEventGetLengthX(currentFrameEvent) - 1
							- state->apsCountX[state->apsCurrentReadoutType]);
					uint16_t yPos = U16T(
						caerFrameEventGetLengthY(currentFrameEvent) - 1
							- state->apsCountY[state->apsCurrentReadoutType]);
					size_t pixelPosition = (size_t) (yPos * caerFrameEventGetLengthX(currentFrameEvent)) + xPos;

					uint16_t xPosAbs = U16T(xPos + state->apsWindow0StartX);
					uint16_t yPosAbs = U16T(yPos + state->apsWindow0StartY);
					size_t pixelPositionAbs = (size_t) (yPosAbs * state->apsSizeX) + xPosAbs;

					if (state->apsCurrentReadoutType == APS_READOUT_RESET) {
						state->apsCurrentResetFrame[pixelPositionAbs] = data;
					}
					else {
						int32_t pixelValue = state->apsCurrentResetFrame[pixelPositionAbs] - data;
						caerFrameEventGetPixelArrayUnsafe(currentFrameEvent)[pixelPosition] = htole16(
							U16T((pixelValue < 0) ? (0) : (pixelValue)));
					}

					caerLog(LOG_DEBUG, state->sourceSubSystemString,
						"APS ADC Sample: column=%" PRIu16 ", row=%" PRIu16 ", xPos=%" PRIu16 ", yPos=%" PRIu16 ", data=%" PRIu16 ".",
						state->apsCountX[state->apsCurrentReadoutType], state->apsCountY[state->apsCurrentReadoutType],
						xPos, yPos, data);

					state->apsCountY[state->apsCurrentReadoutType]++;

					break;
				}

				case 5: {
					// Misc 8bit data, used currently only
					// for IMU events in DAVIS FX3 boards.
					uint8_t misc8Code = U8T((data & 0x0F00) >> 8);
					uint8_t misc8Data = U8T(data & 0x00FF);

					switch (misc8Code) {
						case 0:
							if (state->imuIgnoreEvents) {
								break;
							}

							// Detect missing IMU end events.
							if (state->imuCount >= IMU6_COUNT) {
								caerLog(LOG_INFO, state->sourceSubSystemString,
									"IMU data: IMU samples count is at maximum, discarding further samples.");
								break;
							}

							// IMU data event.
							switch (state->imuCount) {
								case 0:
									caerLog(LOG_ERROR, state->sourceSubSystemString,
										"IMU data: missing IMU Scale Config event. Parsing of IMU events will still be attempted, but be aware that Accel/Gyro scale conversions may be inaccurate.");
									state->imuCount = 1;
									// Fall through to next case, as if imuCount was equal to 1.

								case 1:
								case 3:
								case 5:
								case 7:
								case 9:
								case 11:
								case 13:
									state->imuTmpData = misc8Data;
									break;

								case 2: {
									uint16_t accelX = U16T((state->imuTmpData << 8) | misc8Data);
									caerIMU6EventSetAccelX(currentIMU6Event, accelX / state->imuAccelScale);
									break;
								}

								case 4: {
									uint16_t accelY = U16T((state->imuTmpData << 8) | misc8Data);
									caerIMU6EventSetAccelY(currentIMU6Event, accelY / state->imuAccelScale);
									break;
								}

								case 6: {
									uint16_t accelZ = U16T((state->imuTmpData << 8) | misc8Data);
									caerIMU6EventSetAccelZ(currentIMU6Event, accelZ / state->imuAccelScale);
									break;
								}

									// Temperature is signed. Formula for converting to °C:
									// (SIGNED_VAL / 340) + 36.53
								case 8: {
									int16_t temp = (int16_t) U16T((state->imuTmpData << 8) | misc8Data);
									caerIMU6EventSetTemp(currentIMU6Event, (temp / 340.0f) + 36.53f);
									break;
								}

								case 10: {
									uint16_t gyroX = U16T((state->imuTmpData << 8) | misc8Data);
									caerIMU6EventSetGyroX(currentIMU6Event, gyroX / state->imuGyroScale);
									break;
								}

								case 12: {
									uint16_t gyroY = U16T((state->imuTmpData << 8) | misc8Data);
									caerIMU6EventSetGyroY(currentIMU6Event, gyroY / state->imuGyroScale);
									break;
								}

								case 14: {
									uint16_t gyroZ = U16T((state->imuTmpData << 8) | misc8Data);
									caerIMU6EventSetGyroZ(currentIMU6Event, gyroZ / state->imuGyroScale);
									break;
								}
							}

							state->imuCount++;

							break;

						default:
							caerLog(LOG_ERROR, state->sourceSubSystemString,
								"Caught Misc8 event that can't be handled.");
							break;
					}

					break;
				}

				case 7: // Timestamp wrap
					// Each wrap is 2^15 µs (~32ms), and we have
					// to multiply it with the wrap counter,
					// which is located in the data part of this
					// event.
					state->wrapAdd += (uint32_t) (0x8000 * data);

					state->lastTimestamp = state->currentTimestamp;
					state->currentTimestamp = state->wrapAdd;

					// Check monotonicity of timestamps.
					checkMonotonicTimestamp(state);

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

			// Ignore all APS events, until a new APS Start event comes in.
			// This is to correctly support the forced packet commits that a TS reset,
			// or a timeout condition, impose. Continuing to parse events would result
			// in a corrupted state of the first event in the new packet, as it would
			// be incomplete and miss vital initialization data.
			state->apsIgnoreEvents = true;
		}

		if (forcePacketCommit
			|| (state->currentIMU6PacketPosition
				>= caerEventPacketHeaderGetEventCapacity(&state->currentIMU6Packet->packetHeader))
			|| ((state->currentIMU6PacketPosition > 1)
				&& (caerIMU6EventGetTimestamp(
					caerIMU6EventPacketGetEvent(state->currentIMU6Packet, state->currentIMU6PacketPosition - 1))
					- caerIMU6EventGetTimestamp(caerIMU6EventPacketGetEvent(state->currentIMU6Packet, 0))
					>= state->maxIMU6PacketInterval))) {
			if (!ringBufferPut(state->dataExchangeBuffer, state->currentIMU6Packet)) {
				// Failed to forward packet, drop it.
				free(state->currentIMU6Packet);
				caerLog(LOG_INFO, state->sourceSubSystemString, "Dropped IMU6 Event Packet because ring-buffer full!");
			}
			else {
				caerMainloopDataAvailableIncrease(state->mainloopNotify);
			}

			// Allocate new packet for next iteration.
			state->currentIMU6Packet = caerIMU6EventPacketAllocate(state->maxIMU6PacketSize, state->sourceID);
			state->currentIMU6PacketPosition = 0;

			// Ignore all IMU events, until a new IMU Start event comes in.
			// This is to correctly support the forced packet commits that a TS reset,
			// or a timeout condition, impose. Continuing to parse events would result
			// in a corrupted state of the first event in the new packet, as it would
			// be incomplete and miss vital initialization data.
			state->imuIgnoreEvents = true;
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
	}
}

static libusb_device_handle *deviceOpen(libusb_context *devContext, uint16_t devVID, uint16_t devPID, uint8_t devType,
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

static void deviceClose(libusb_device_handle *devHandle) {
	// Release interface 0 (default).
	libusb_release_interface(devHandle, 0);

	libusb_close(devHandle);
}

void sendEnableDataConfig(sshsNode moduleNode, libusb_device_handle *devHandle) {
	sendUSBConfig(moduleNode, devHandle);
	sendMultiplexerConfig(moduleNode, devHandle);
	sendDVSConfig(moduleNode, devHandle);
	sendAPSConfig(moduleNode, devHandle);
	sendIMUConfig(moduleNode, devHandle);
	sendExternalInputDetectorConfig(moduleNode, devHandle);
}

void sendDisableDataConfig(libusb_device_handle *devHandle) {
	spiConfigSend(devHandle, FPGA_EXTINPUT, 0, 0);
	spiConfigSend(devHandle, FPGA_IMU, 0, 0);
	spiConfigSend(devHandle, FPGA_APS, 1, 0); // Ensure ADC turns off.
	spiConfigSend(devHandle, FPGA_APS, 0, 0);
	spiConfigSend(devHandle, FPGA_DVS, 0, 0);
	spiConfigSend(devHandle, FPGA_MUX, 3, 0); // Ensure chip turns off.
	spiConfigSend(devHandle, FPGA_MUX, 1, 0); // Turn off timestamp too.
	spiConfigSend(devHandle, FPGA_MUX, 0, 0);
	spiConfigSend(devHandle, FPGA_USB, 0, 0);
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
			spiConfigSend(devHandle, FPGA_DVS, 0, changeValue.boolean);
		}
		else if (changeType == BYTE && str_equals(changeKey, "AckDelayRow")) {
			spiConfigSend(devHandle, FPGA_DVS, 1, changeValue.ubyte);
		}
		else if (changeType == BYTE && str_equals(changeKey, "AckDelayColumn")) {
			spiConfigSend(devHandle, FPGA_DVS, 2, changeValue.ubyte);
		}
		else if (changeType == BYTE && str_equals(changeKey, "AckExtensionRow")) {
			spiConfigSend(devHandle, FPGA_DVS, 3, changeValue.ubyte);
		}
		else if (changeType == BYTE && str_equals(changeKey, "AckExtensionColumn")) {
			spiConfigSend(devHandle, FPGA_DVS, 4, changeValue.ubyte);
		}
		else if (changeType == BOOL && str_equals(changeKey, "WaitOnTransferStall")) {
			spiConfigSend(devHandle, FPGA_DVS, 5, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "FilterRowOnlyEvents")) {
			spiConfigSend(devHandle, FPGA_DVS, 6, changeValue.boolean);
		}
	}
}

static void sendDVSConfig(sshsNode moduleNode, libusb_device_handle *devHandle) {
	sshsNode dvsNode = sshsGetRelativeNode(moduleNode, "dvs/");

	spiConfigSend(devHandle, FPGA_DVS, 1, sshsNodeGetByte(dvsNode, "AckDelayRow"));
	spiConfigSend(devHandle, FPGA_DVS, 2, sshsNodeGetByte(dvsNode, "AckDelayColumn"));
	spiConfigSend(devHandle, FPGA_DVS, 3, sshsNodeGetByte(dvsNode, "AckExtensionRow"));
	spiConfigSend(devHandle, FPGA_DVS, 4, sshsNodeGetByte(dvsNode, "AckExtensionColumn"));
	spiConfigSend(devHandle, FPGA_DVS, 5, sshsNodeGetBool(dvsNode, "WaitOnTransferStall"));
	spiConfigSend(devHandle, FPGA_DVS, 6, sshsNodeGetBool(dvsNode, "FilterRowOnlyEvents"));
	spiConfigSend(devHandle, FPGA_DVS, 0, sshsNodeGetBool(dvsNode, "Run"));
}

static void APSConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);
	davisCommonState state = userData;
	libusb_device_handle *devHandle = state->deviceHandle;

	if (event == ATTRIBUTE_MODIFIED) {
		if (changeType == BOOL && str_equals(changeKey, "Run")) {
			spiConfigSend(devHandle, FPGA_APS, 0, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "ForceADCRunning")) {
			spiConfigSend(devHandle, FPGA_APS, 1, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "GlobalShutter")) {
			spiConfigSend(devHandle, FPGA_APS, 2, changeValue.boolean);
		}
		else if (changeType == SHORT && str_equals(changeKey, "StartColumn0")) {
			// The APS chip view is flipped on both axes. Reverse and exchange.
			uint16_t endColumn0 = changeValue.ushort;
			endColumn0 = U16T(state->apsSizeX - 1 - endColumn0);

			spiConfigSend(devHandle, FPGA_APS, 5, endColumn0);
			state->apsWindow0SizeX = U16T(sshsNodeGetShort(node, "EndColumn0") + 1 - changeValue.ushort);

			// Update start offset for absolute pixel position (reset map).
			state->apsWindow0StartX = changeValue.ushort;
		}
		else if (changeType == SHORT && str_equals(changeKey, "StartRow0")) {
			// The APS chip view is flipped on both axes. Reverse and exchange.
			uint16_t endRow0 = changeValue.ushort;
			endRow0 = U16T(state->apsSizeY - 1 - endRow0);

			spiConfigSend(devHandle, FPGA_APS, 6, endRow0);
			state->apsWindow0SizeY = U16T(sshsNodeGetShort(node, "EndRow0") + 1 - changeValue.ushort);

			// Update start offset for absolute pixel position (reset map).
			state->apsWindow0StartY = changeValue.ushort;
		}
		else if (changeType == SHORT && str_equals(changeKey, "EndColumn0")) {
			// The APS chip view is flipped on both axes. Reverse and exchange.
			uint16_t startColumn0 = changeValue.ushort;
			startColumn0 = U16T(state->apsSizeX - 1 - startColumn0);

			spiConfigSend(devHandle, FPGA_APS, 3, startColumn0);
			state->apsWindow0SizeX = U16T(changeValue.ushort + 1 - sshsNodeGetShort(node, "StartColumn0"));
		}
		else if (changeType == SHORT && str_equals(changeKey, "EndRow0")) {
			// The APS chip view is flipped on both axes. Reverse and exchange.
			uint16_t startRow0 = changeValue.ushort;
			startRow0 = U16T(state->apsSizeY - 1 - startRow0);

			spiConfigSend(devHandle, FPGA_APS, 4, startRow0);
			state->apsWindow0SizeY = U16T(changeValue.ushort + 1 - sshsNodeGetShort(node, "StartRow0"));
		}
		else if (changeType == INT && str_equals(changeKey, "Exposure")) {
			spiConfigSend(devHandle, FPGA_APS, 7, changeValue.uint * EXT_ADC_FREQ);
		}
		else if (changeType == INT && str_equals(changeKey, "FrameDelay")) {
			spiConfigSend(devHandle, FPGA_APS, 8, changeValue.uint * EXT_ADC_FREQ);
		}
		else if (changeType == SHORT && str_equals(changeKey, "ResetSettle")) {
			spiConfigSend(devHandle, FPGA_APS, 9, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "ColumnSettle")) {
			spiConfigSend(devHandle, FPGA_APS, 10, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "RowSettle")) {
			spiConfigSend(devHandle, FPGA_APS, 11, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "NullSettle")) {
			spiConfigSend(devHandle, FPGA_APS, 12, changeValue.ushort);
		}
		else if (changeType == BOOL && str_equals(changeKey, "ResetRead")) {
			spiConfigSend(devHandle, FPGA_APS, 13, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "WaitOnTransferStall")) {
			spiConfigSend(devHandle, FPGA_APS, 14, changeValue.boolean);
		}
	}
}

static void sendAPSConfig(sshsNode moduleNode, libusb_device_handle *devHandle) {
	sshsNode apsNode = sshsGetRelativeNode(moduleNode, "aps/");
	sshsNode infoNode = sshsGetRelativeNode(moduleNode, "sourceInfo/");

	spiConfigSend(devHandle, FPGA_APS, 1, sshsNodeGetBool(apsNode, "ForceADCRunning"));

	// GS may not exist on chips that don't have it.
	if (sshsNodeAttrExists(apsNode, "GlobalShutter", BOOL)) {
		spiConfigSend(devHandle, FPGA_APS, 2, sshsNodeGetBool(apsNode, "GlobalShutter"));
	}

	// The APS chip view is flipped on both axes. Reverse and exchange.
	uint16_t endColumn0 = sshsNodeGetShort(apsNode, "StartColumn0");
	endColumn0 = U16T(sshsNodeGetShort(infoNode, "apsSizeX") - 1 - endColumn0);

	spiConfigSend(devHandle, FPGA_APS, 5, endColumn0);

	// The APS chip view is flipped on both axes. Reverse and exchange.
	uint16_t endRow0 = sshsNodeGetShort(apsNode, "StartRow0");
	endRow0 = U16T(sshsNodeGetShort(infoNode, "apsSizeY") - 1 - endRow0);

	spiConfigSend(devHandle, FPGA_APS, 6, endRow0);

	// The APS chip view is flipped on both axes. Reverse and exchange.
	uint16_t startColumn0 = sshsNodeGetShort(apsNode, "EndColumn0");
	startColumn0 = U16T(sshsNodeGetShort(infoNode, "apsSizeX") - 1 - startColumn0);

	spiConfigSend(devHandle, FPGA_APS, 3, startColumn0);

	// The APS chip view is flipped on both axes. Reverse and exchange.
	uint16_t startRow0 = sshsNodeGetShort(apsNode, "EndRow0");
	startRow0 = U16T(sshsNodeGetShort(infoNode, "apsSizeY") - 1 - startRow0);

	spiConfigSend(devHandle, FPGA_APS, 4, startRow0);

	spiConfigSend(devHandle, FPGA_APS, 7, sshsNodeGetInt(apsNode, "Exposure") * EXT_ADC_FREQ); // in µs, converted to cycles here
	spiConfigSend(devHandle, FPGA_APS, 8, sshsNodeGetInt(apsNode, "FrameDelay") * EXT_ADC_FREQ); // in µs, converted to cycles here
	spiConfigSend(devHandle, FPGA_APS, 9, sshsNodeGetShort(apsNode, "ResetSettle")); // in cycles
	spiConfigSend(devHandle, FPGA_APS, 10, sshsNodeGetShort(apsNode, "ColumnSettle")); // in cycles
	spiConfigSend(devHandle, FPGA_APS, 11, sshsNodeGetShort(apsNode, "RowSettle")); // in cycles
	spiConfigSend(devHandle, FPGA_APS, 12, sshsNodeGetShort(apsNode, "NullSettle")); // in cycles
	spiConfigSend(devHandle, FPGA_APS, 13, sshsNodeGetBool(apsNode, "ResetRead"));
	spiConfigSend(devHandle, FPGA_APS, 14, sshsNodeGetBool(apsNode, "WaitOnTransferStall"));
	spiConfigSend(devHandle, FPGA_APS, 0, sshsNodeGetBool(apsNode, "Run"));
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

static void HostConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(changeValue);

	caerModuleData data = userData;

	// Distinguish changes to USB transfers or packet sizes, by
	// using configUpdate like a bit-field.
	if (event == ATTRIBUTE_MODIFIED) {
		if (str_equals(sshsNodeGetName(node), "usb")) {
			// Changes to the USB transfer settings (requires reallocation).
			if (changeType == INT && (str_equals(changeKey, "BufferNumber") || str_equals(changeKey, "BufferSize"))) {
				atomic_ops_uint_or(&data->configUpdate, (0x01 << 0), ATOMIC_OPS_FENCE_NONE);
			}
		}
		else if (str_equals(sshsNodeGetName(node), "system")) {
			// Changes to packet size and interval.
			if (changeType == INT
				&& (str_equals_upto(changeKey, "PolarityPacket", 14) || str_equals_upto(changeKey, "FramePacket", 11)
					|| str_equals_upto(changeKey, "IMU6Packet", 10) || str_equals_upto(changeKey, "SpecialPacket", 13))) {
				atomic_ops_uint_or(&data->configUpdate, (0x01 << 1), ATOMIC_OPS_FENCE_NONE);
			}
		}
	}
}

void dataAcquisitionThreadConfig(caerModuleData moduleData) {
	// The common state is always the first member of the moduleState structure
	// for the DAVIS modules, so we can trust it being at address offset 0.
	davisCommonState cstate = moduleData->moduleState;

	// Get the current value to examine by atomic exchange, since we don't
	// want there to be any possible store between a load/store pair.
	uintptr_t configUpdate = atomic_ops_uint_swap(&moduleData->configUpdate, 0, ATOMIC_OPS_FENCE_NONE);

	if (configUpdate & (0x01 << 0)) {
		reallocateUSBBuffers(moduleData->moduleNode, cstate);
	}

	if (configUpdate & (0x01 << 1)) {
		updatePacketSizesIntervals(moduleData->moduleNode, cstate);
	}
}

static void reallocateUSBBuffers(sshsNode moduleNode, davisCommonState state) {
	sshsNode usbNode = sshsGetRelativeNode(moduleNode, "usb/");

	deallocateDataTransfers(state);
	allocateDataTransfers(state, sshsNodeGetInt(usbNode, "BufferNumber"), sshsNodeGetInt(usbNode, "BufferSize"));
}

static void updatePacketSizesIntervals(sshsNode moduleNode, davisCommonState state) {
	sshsNode sysNode = sshsGetRelativeNode(moduleNode, "system/");

	// Packet settings (size (in events) and time interval (in µs)).
	state->maxPolarityPacketSize = sshsNodeGetInt(sysNode, "PolarityPacketMaxSize");
	state->maxPolarityPacketInterval = sshsNodeGetInt(sysNode, "PolarityPacketMaxInterval");

	state->maxFramePacketSize = sshsNodeGetInt(sysNode, "FramePacketMaxSize");
	state->maxFramePacketInterval = sshsNodeGetInt(sysNode, "FramePacketMaxInterval");

	state->maxIMU6PacketSize = sshsNodeGetInt(sysNode, "IMU6PacketMaxSize");
	state->maxIMU6PacketInterval = sshsNodeGetInt(sysNode, "IMU6PacketMaxInterval");

	state->maxSpecialPacketSize = sshsNodeGetInt(sysNode, "SpecialPacketMaxSize");
	state->maxSpecialPacketInterval = sshsNodeGetInt(sysNode, "SpecialPacketMaxInterval");
}
