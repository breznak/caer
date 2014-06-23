/*
 * davisFX3.c
 *
 *  Created on: Nov 26, 2013
 *      Author: chtekk
 */

#include "davis_fx3.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/ringbuffer/ringbuffer.h"
#include <pthread.h>
#include <libusb.h>
#include <unistd.h>

struct davisFX3_state {
	// Data Acquisition Thread -> Mainloop Exchange
	pthread_t dataAcquisitionThread;
	RingBuffer dataExchangeBuffer;
	caerMainloopData mainloopNotify;
	uint16_t sourceID;
	// USB Device State
	libusb_context *deviceContext;
	libusb_device_handle *deviceHandle;
	// Data Acquisition Thread State
	struct libusb_transfer **dataTransfers;
	atomic_ops_uint dataTransfersLength;
	struct libusb_transfer **debugTransfers;
	atomic_ops_uint debugTransfersLength;
	uint32_t wrapAdd;
	uint32_t lastTimestamp;
	uint32_t currentTimestamp;
	uint16_t lastY;
	bool gotY;
	// Polarity Packet State
	caerPolarityEventPacket currentPolarityPacket;
	uint32_t currentPolarityPacketPosition;
	uint32_t maxPolarityPacketSize;
	uint32_t maxPolarityPacketInterval;
	// Frame Packet State
	caerFrameEventPacket currentFramePacket;
	uint32_t currentFramePacketPosition;
	uint32_t maxFramePacketSize;
	uint32_t maxFramePacketInterval;
	// IMU6 Packet State
	caerIMU6EventPacket currentIMU6Packet;
	uint32_t currentIMU6PacketPosition;
	uint32_t maxIMU6PacketSize;
	uint32_t maxIMU6PacketInterval;
	// Special Packet State
	caerSpecialEventPacket currentSpecialPacket;
	uint32_t currentSpecialPacketPosition;
	uint32_t maxSpecialPacketSize;
	uint32_t maxSpecialPacketInterval;
};

typedef struct davisFX3_state *davisFX3State;

static bool caerInputDAViSFX3Init(caerModuleData moduleData);
static void caerInputDAViSFX3Run(caerModuleData moduleData, size_t argsNumber, va_list args);
// CONFIG: Nothing to do here in the main thread!
// Biases are configured asynchronously, and buffer sizes in the data
// acquisition thread itself. Resetting the main config_refresh flag
// will also happen there.
static void caerInputDAViSFX3Exit(caerModuleData moduleData);

static struct caer_module_functions caerInputDAViSFX3Functions = { .moduleInit = &caerInputDAViSFX3Init, .moduleRun =
	&caerInputDAViSFX3Run, .moduleConfig = NULL, .moduleExit = &caerInputDAViSFX3Exit };

void caerInputDAViSFX3(uint16_t moduleID, caerPolarityEventPacket *polarity, caerFrameEventPacket *frame,
	caerIMU6EventPacket *imu6, caerSpecialEventPacket *special) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "DAViSFX3");

	// IMPORTANT: THE CONTENT OF OUTPUT ARGUMENTS MUST BE SET TO NULL!
	if (polarity != NULL) {
		*polarity = NULL;
	}
	if (frame != NULL) {
		*frame = NULL;
	}
	if (imu6 != NULL) {
		*imu6 = NULL;
	}
	if (special != NULL) {
		*special = NULL;
	}

	caerModuleSM(&caerInputDAViSFX3Functions, moduleData, sizeof(struct davisFX3_state), 4, polarity, frame, imu6,
		special);
}

static void *dataAcquisitionThread(void *inPtr);
static void dataAcquisitionThreadConfig(caerModuleData data);
static void allocateDataTransfers(davisFX3State state, uint32_t bufferNum, uint32_t bufferSize);
static void deallocateDataTransfers(davisFX3State state);
static void LIBUSB_CALL libUsbDataCallback(struct libusb_transfer *transfer);
static void dataTranslator(davisFX3State state, uint8_t *buffer, size_t bytesSent);
static void allocateDebugTransfers(davisFX3State state);
static void deallocateDebugTransfers(davisFX3State state);
static void LIBUSB_CALL libUsbDebugCallback(struct libusb_transfer *transfer);
static void debugTranslator(davisFX3State state, uint8_t *buffer, size_t bytesSent);
static void sendBiases(sshsNode biasNode, libusb_device_handle *devHandle);
static void sendChipSR(sshsNode chipNode, libusb_device_handle *devHandle);
static void sendFpgaSR(sshsNode fpgaNode, libusb_device_handle *devHandle);
static libusb_device_handle *deviceOpen(libusb_context *devContext, uint8_t busNumber, uint8_t devAddress);
static void deviceClose(libusb_device_handle *devHandle);
static void caerInputDAViSFX3ConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);

static inline void freeAllPackets(davisFX3State state) {
	free(state->currentPolarityPacket);
	free(state->currentFramePacket);
	free(state->currentIMU6Packet);
	free(state->currentSpecialPacket);
}

static inline void createAddressedCoarseFineBiasSetting(sshsNode biasNode, const char *biasName, const char *type,
	const char *sex, uint8_t coarseValue, uint8_t fineValue, bool enabled) {
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

static inline void sendAddressedCoarseFineBias(sshsNode biasNode, libusb_device_handle *devHandle, uint16_t biasAddress,
	const char *biasName) {
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

	// Reverse coarse part.
	uint8_t coarseValue = (sshsNodeGetByte(biasConfigNode, "coarseValue") & 0x07);
	uint8_t reversedCoarseValue = (uint8_t) (((coarseValue * 0x0802LU & 0x22110LU)
		| (coarseValue * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16);

	// Reversing the byte produces a fully reversed byte, so the lower three
	// bits end up reversed, but as the highest three bits! That means shifting
	// them right by 5, and then shifting them left by 12 to put them in their
	// final position. This can be expressed by just a left shift of 7 (12 - 5).
	biasValue |= (uint16_t) (reversedCoarseValue << 7);

	// All biases are two byte quantities.
	uint8_t bias[2];

	// Put the value in.
	bias[0] = U8T(biasValue >> 8);
	bias[1] = U8T(biasValue >> 0);

	libusb_control_transfer(devHandle, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
	VR_CHIP_BIAS, biasAddress, 0, bias, sizeof(bias), 0);
}

static inline void createShiftedSourceBiasSetting(sshsNode biasNode, const char *biasName, uint8_t regValue,
	uint8_t refValue, const char *operatingMode, const char *voltageLevel) {
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

static inline void sendShiftedSourceBias(sshsNode biasNode, libusb_device_handle *devHandle, uint16_t biasAddress,
	const char *biasName) {
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

	// All biases are two byte quantities.
	uint8_t bias[2];

	// Put the value in.
	bias[0] = U8T(biasValue >> 8);
	bias[1] = U8T(biasValue >> 0);

	libusb_control_transfer(devHandle, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
	VR_CHIP_BIAS, biasAddress, 0, bias, sizeof(bias), 0);
}

static bool caerInputDAViSFX3Init(caerModuleData moduleData) {
	caerLog(LOG_DEBUG, "Initializing DAViSFX3 module ...");

	// First, always create all needed setting nodes, set their default values
	// and add their listeners.
	// Set default biases, from SBRet20s_gs.xml settings.
	sshsNode biasNode = sshsGetRelativeNode(moduleData->moduleNode, "bias/");
	createAddressedCoarseFineBiasSetting(biasNode, "DiffBn", "Normal", "N", 3, 72, true);
	createAddressedCoarseFineBiasSetting(biasNode, "OnBn", "Normal", "N", 2, 112, true);
	createAddressedCoarseFineBiasSetting(biasNode, "OffBn", "Normal", "N", 3, 6, true);
	createAddressedCoarseFineBiasSetting(biasNode, "ApsCasEpc", "Cascode", "N", 2, 144, true);
	createAddressedCoarseFineBiasSetting(biasNode, "DiffCasBnc", "Cascode", "N", 2, 115, true);
	createAddressedCoarseFineBiasSetting(biasNode, "ApsROSFBn", "Normal", "N", 1, 188, true);
	createAddressedCoarseFineBiasSetting(biasNode, "LocalBufBn", "Normal", "N", 2, 164, true);
	createAddressedCoarseFineBiasSetting(biasNode, "PixInvBn", "Normal", "N", 1, 129, true);
	createAddressedCoarseFineBiasSetting(biasNode, "PrBp", "Normal", "P", 6, 255, true);
	createAddressedCoarseFineBiasSetting(biasNode, "PrSFBp", "Normal", "P", 5, 2, true);
	createAddressedCoarseFineBiasSetting(biasNode, "RefrBp", "Normal", "P", 3, 19, true);
	createAddressedCoarseFineBiasSetting(biasNode, "AEPdBn", "Normal", "N", 0, 140, true);
	createAddressedCoarseFineBiasSetting(biasNode, "LcolTimeoutBn", "Normal", "N", 2, 55, true);
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
	sshsNodePutBoolIfAbsent(chipNode, "globalShutter", true);
	sshsNodePutBoolIfAbsent(chipNode, "useAout", false);
	sshsNodePutBoolIfAbsent(chipNode, "nArow", false);
	sshsNodePutBoolIfAbsent(chipNode, "hotPixelSuppression", false);
	sshsNodePutBoolIfAbsent(chipNode, "resetTestpixel", true);
	sshsNodePutBoolIfAbsent(chipNode, "typeNCalib", false);
	sshsNodePutBoolIfAbsent(chipNode, "resetCalib", true);

	sshsNode fpgaNode = sshsGetRelativeNode(moduleData->moduleNode, "fpga/");
	sshsNodePutShortIfAbsent(fpgaNode, "frameDelay", 4345);
	sshsNodePutShortIfAbsent(fpgaNode, "resSettle", 11);
	sshsNodePutShortIfAbsent(fpgaNode, "rowSettle", 10);
	sshsNodePutShortIfAbsent(fpgaNode, "colSettle", 300);
	sshsNodePutShortIfAbsent(fpgaNode, "exposure", 28);

	// USB port settings/restrictions.
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "usbBusNumber", 0);
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "usbDevAddress", 0);

	// USB buffer settings.
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "bufferNumber", 8);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "bufferSize", 8192);

	// Packet settings (size (in events) and time interval (in µs)).
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "polarityPacketMaxSize", 4096);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "polarityPacketMaxInterval", 5000);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "framePacketMaxSize", 4);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "framePacketMaxInterval", 50000);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "imu6PacketMaxSize", 32);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "imu6PacketMaxInterval", 10000);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "specialPacketMaxSize", 128);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "specialPacketMaxInterval", 1000);

	// Ring-buffer setting (only changes value on module init/shutdown cycles).
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "dataExchangeBufferSize", 64);

	// Install default listener to signal configuration updates asynchronously.
	sshsNodeAddAttrListener(biasNode, moduleData, &caerInputDAViSFX3ConfigListener);
	sshsNodeAddAttrListener(chipNode, moduleData, &caerInputDAViSFX3ConfigListener);
	sshsNodeAddAttrListener(fpgaNode, moduleData, &caerInputDAViSFX3ConfigListener);
	sshsNodeAddAttrListener(moduleData->moduleNode, moduleData, &caerInputDAViSFX3ConfigListener);

	davisFX3State state = moduleData->moduleState;

	// Data source is the same as the module ID (but accessible in state-space).
	state->sourceID = moduleData->moduleID;

	// Put global source information into SSHS.
	sshsNode sourceInfoNode = sshsGetRelativeNode(moduleData->moduleNode, "sourceInfo/");
	sshsNodePutShort(sourceInfoNode, "dvsSizeX", DAVIS_FX3_ARRAY_SIZE_X);
	sshsNodePutShort(sourceInfoNode, "dvsSizeY", DAVIS_FX3_ARRAY_SIZE_Y);
	sshsNodePutShort(sourceInfoNode, "frameSizeX", DAVIS_FX3_ARRAY_SIZE_X);
	sshsNodePutShort(sourceInfoNode, "frameSizeY", DAVIS_FX3_ARRAY_SIZE_Y);
	sshsNodePutShort(sourceInfoNode, "frameADCDepth", DAVIS_FX3_ADC_DEPTH);

	// Initialize state fields.
	state->maxPolarityPacketSize = sshsNodeGetInt(moduleData->moduleNode, "polarityPacketMaxSize");
	state->maxPolarityPacketInterval = sshsNodeGetInt(moduleData->moduleNode, "polarityPacketMaxInterval");

	state->maxFramePacketSize = sshsNodeGetInt(moduleData->moduleNode, "framePacketMaxSize");
	state->maxFramePacketInterval = sshsNodeGetInt(moduleData->moduleNode, "framePacketMaxInterval");

	state->maxIMU6PacketSize = sshsNodeGetInt(moduleData->moduleNode, "imu6PacketMaxSize");
	state->maxIMU6PacketInterval = sshsNodeGetInt(moduleData->moduleNode, "imu6PacketMaxInterval");

	state->maxSpecialPacketSize = sshsNodeGetInt(moduleData->moduleNode, "specialPacketMaxSize");
	state->maxSpecialPacketInterval = sshsNodeGetInt(moduleData->moduleNode, "specialPacketMaxInterval");

	state->currentPolarityPacket = caerPolarityEventPacketAllocate(state->maxPolarityPacketSize, state->sourceID);
	state->currentPolarityPacketPosition = 0;

	state->currentFramePacket = caerFrameEventPacketAllocate(state->maxFramePacketSize, state->sourceID,
	DAVIS_FX3_ADC_DEPTH, DAVIS_FX3_ARRAY_SIZE_Y, DAVIS_FX3_ARRAY_SIZE_X);
	state->currentFramePacketPosition = 0;

	state->currentIMU6Packet = caerIMU6EventPacketAllocate(state->maxIMU6PacketSize, state->sourceID);
	state->currentIMU6PacketPosition = 0;

	state->currentSpecialPacket = caerSpecialEventPacketAllocate(state->maxSpecialPacketSize, state->sourceID);
	state->currentSpecialPacketPosition = 0;

	state->wrapAdd = 0;
	state->lastTimestamp = 0;
	state->currentTimestamp = 0;
	state->lastY = 0;
	state->gotY = false;

	// Store reference to parent mainloop, so that we can correctly notify
	// the availability or not of data to consume.
	state->mainloopNotify = caerMainloopGetReference();

	// Create data exchange buffers.
	state->dataExchangeBuffer = ringBufferInit(sshsNodeGetInt(moduleData->moduleNode, "dataExchangeBufferSize"));
	if (state->dataExchangeBuffer == NULL) {
		freeAllPackets(state);

		caerLog(LOG_CRITICAL, "Failed to initialize data exchange buffer.");
		return (false);
	}

	// Initialize libusb using a separate context for each device.
	// This is to correctly support one thread per device.
	if ((errno = libusb_init(&state->deviceContext)) != LIBUSB_SUCCESS) {
		freeAllPackets(state);
		ringBufferFree(state->dataExchangeBuffer);

		caerLog(LOG_CRITICAL, "Failed to initialize libusb context. Error: %s (%d).", libusb_strerror(errno), errno);
		return (false);
	}

	// Try to open a DAViSFX3 device on a specific USB port.
	state->deviceHandle = deviceOpen(state->deviceContext, sshsNodeGetByte(moduleData->moduleNode, "usbBusNumber"),
		sshsNodeGetByte(moduleData->moduleNode, "usbDevAddress"));
	if (state->deviceHandle == NULL) {
		freeAllPackets(state);
		ringBufferFree(state->dataExchangeBuffer);
		libusb_exit(state->deviceContext);

		caerLog(LOG_CRITICAL, "Failed to open DAViSFX3 device.");
		return (false);
	}

	// Start data acquisition thread.
	if ((errno = pthread_create(&state->dataAcquisitionThread, NULL, &dataAcquisitionThread, moduleData)) != 0) {
		freeAllPackets(state);
		ringBufferFree(state->dataExchangeBuffer);
		deviceClose(state->deviceHandle);
		libusb_exit(state->deviceContext);

		caerLog(LOG_CRITICAL, "Failed to start data acquisition thread. Error: %s (%d).", caerLogStrerror(errno),
		errno);
		return (false);
	}

	caerLog(LOG_DEBUG, "Initialized DAViSFX3 module successfully with device Bus=%" PRIu8 ":Addr=%" PRIu8 ".",
		libusb_get_bus_number(libusb_get_device(state->deviceHandle)),
		libusb_get_device_address(libusb_get_device(state->deviceHandle)));
	return (true);
}

static void caerInputDAViSFX3Exit(caerModuleData moduleData) {
	caerLog(LOG_DEBUG, "Shutting down DAViSFX3 module ...");

	davisFX3State state = moduleData->moduleState;

	// Wait for data acquisition thread to terminate...
	if ((errno = pthread_join(state->dataAcquisitionThread, NULL)) != 0) {
		// This should never happen!
		caerLog(LOG_CRITICAL, "Failed to join data acquisition thread. Error: %s (%d).", caerLogStrerror(errno),
		errno);
	}

	// Finally, close the device fully.
	deviceClose(state->deviceHandle);

	// Destroy libusb context.
	libusb_exit(state->deviceContext);

	// Empty ringbuffer.
	void *packet;
	while ((packet = ringBufferGet(state->dataExchangeBuffer)) != NULL) {
		caerMainloopDataAvailableDecrease(state->mainloopNotify);
		free(packet);
	}

	// And destroy it.
	ringBufferFree(state->dataExchangeBuffer);

	// Free remaining incomplete packets.
	freeAllPackets(state);

	caerLog(LOG_DEBUG, "Shutdown DAViSFX3 module successfully.");
}

static void caerInputDAViSFX3Run(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	caerPolarityEventPacket *polarity = va_arg(args, caerPolarityEventPacket *);
	caerFrameEventPacket *frame = va_arg(args, caerFrameEventPacket *);
	caerIMU6EventPacket *imu6 = va_arg(args, caerIMU6EventPacket *);
	caerSpecialEventPacket *special = va_arg(args, caerSpecialEventPacket *);

	davisFX3State state = moduleData->moduleState;

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

static void *dataAcquisitionThread(void *inPtr) {
	caerLog(LOG_DEBUG, "DAViSFX3: initializing data acquisition thread ...");

	// inPtr is a pointer to module data.
	caerModuleData data = inPtr;
	davisFX3State state = data->moduleState;

	// Send default start-up biases and config values to device before enabling it.
	sendBiases(sshsGetRelativeNode(data->moduleNode, "bias/"), state->deviceHandle);
	sendChipSR(sshsGetRelativeNode(data->moduleNode, "chip/"), state->deviceHandle);
	sendFpgaSR(sshsGetRelativeNode(data->moduleNode, "fpga/"), state->deviceHandle);

	// Create buffers as specified in config file.
	allocateDebugTransfers(state);
	allocateDataTransfers(state, sshsNodeGetInt(data->moduleNode, "bufferNumber"),
		sshsNodeGetInt(data->moduleNode, "bufferSize"));

	// Enable AER data transfer on USB end-point 6.
	libusb_control_transfer(state->deviceHandle,
		LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE, VR_DATA_ENABLE, 1, 0,
		NULL, 0, 0);

	// Handle USB events (1 second timeout).
	struct timeval te = { .tv_sec = 0, .tv_usec = 1000000 };

	caerLog(LOG_DEBUG, "DAViSFX3: data acquisition thread ready to process events.");

	while (atomic_ops_uint_load(&data->running, ATOMIC_OPS_FENCE_NONE) != 0
		&& atomic_ops_uint_load(&state->dataTransfersLength, ATOMIC_OPS_FENCE_NONE) > 0) {
		// Check config refresh, in this case to adjust buffer sizes.
		if (atomic_ops_uint_load(&data->configUpdate, ATOMIC_OPS_FENCE_NONE) != 0) {
			dataAcquisitionThreadConfig(data);
		}

		libusb_handle_events_timeout(state->deviceContext, &te);
	}

	caerLog(LOG_DEBUG, "DAViSFX3: shutting down data acquisition thread ...");

	// Disable AER data transfer on USB end-point 6.
	libusb_control_transfer(state->deviceHandle,
		LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE, VR_DATA_ENABLE, 0, 0,
		NULL, 0, 0);

	// Cancel all transfers and handle them.
	deallocateDataTransfers(state);
	deallocateDebugTransfers(state);

	// Ensure parent also shuts down (on disconnected device for example).
	sshsNodePutBool(data->moduleNode, "shutdown", true);

	caerLog(LOG_DEBUG, "DAViSFX3: data acquisition thread shut down.");

	return (NULL);
}

static void dataAcquisitionThreadConfig(caerModuleData moduleData) {
	davisFX3State state = moduleData->moduleState;

	// Get the current value to examine by atomic exchange, since we don't
	// want there to be any possible store between a load/store pair.
	uintptr_t configUpdate = atomic_ops_uint_swap(&moduleData->configUpdate, 0, ATOMIC_OPS_FENCE_NONE);

	if (configUpdate & (0x01 << 0)) {
		// Bias update required.
		sendBiases(sshsGetRelativeNode(moduleData->moduleNode, "bias/"), state->deviceHandle);
	}

	if (configUpdate & (0x01 << 1)) {
		// Chip config update required.
		sendChipSR(sshsGetRelativeNode(moduleData->moduleNode, "chip/"), state->deviceHandle);
	}

	if (configUpdate & (0x01 << 2)) {
		// FPGA config update required.
		sendFpgaSR(sshsGetRelativeNode(moduleData->moduleNode, "fpga/"), state->deviceHandle);
	}

	if (configUpdate & (0x01 << 3)) {
		// Do buffer size change: cancel all and recreate them.
		deallocateDataTransfers(state);
		allocateDataTransfers(state, sshsNodeGetInt(moduleData->moduleNode, "bufferNumber"),
			sshsNodeGetInt(moduleData->moduleNode, "bufferSize"));
	}

	if (configUpdate & (0x01 << 4)) {
		// Update maximum size and interval settings for packets.
		state->maxPolarityPacketSize = sshsNodeGetInt(moduleData->moduleNode, "polarityPacketMaxSize");
		state->maxPolarityPacketInterval = sshsNodeGetInt(moduleData->moduleNode, "polarityPacketMaxInterval");

		state->maxFramePacketSize = sshsNodeGetInt(moduleData->moduleNode, "framePacketMaxSize");
		state->maxFramePacketInterval = sshsNodeGetInt(moduleData->moduleNode, "framePacketMaxInterval");

		state->maxIMU6PacketSize = sshsNodeGetInt(moduleData->moduleNode, "imu6PacketMaxSize");
		state->maxIMU6PacketInterval = sshsNodeGetInt(moduleData->moduleNode, "imu6PacketMaxInterval");

		state->maxSpecialPacketSize = sshsNodeGetInt(moduleData->moduleNode, "specialPacketMaxSize");
		state->maxSpecialPacketInterval = sshsNodeGetInt(moduleData->moduleNode, "specialPacketMaxInterval");
	}
}

static void allocateDataTransfers(davisFX3State state, uint32_t bufferNum, uint32_t bufferSize) {
	atomic_ops_uint_store(&state->dataTransfersLength, 0, ATOMIC_OPS_FENCE_NONE);

	// Set number of transfers and allocate memory for the main transfer array.
	state->dataTransfers = calloc(bufferNum, sizeof(struct libusb_transfer *));
	if (state->dataTransfers == NULL) {
		caerLog(LOG_CRITICAL,
			"Failed to allocate memory for %" PRIu32 " libusb transfers (data channel). Error: %s (%d).", bufferNum,
			caerLogStrerror(errno), errno);
		return;
	}

	// Allocate transfers and set them up.
	for (size_t i = 0; i < bufferNum; i++) {
		state->dataTransfers[i] = libusb_alloc_transfer(0);
		if (state->dataTransfers[i] == NULL) {
			caerLog(LOG_CRITICAL, "Unable to allocate further libusb transfers (data channel, %zu of %" PRIu32 ").", i,
				bufferNum);
			return;
		}

		// Create data buffer.
		state->dataTransfers[i]->length = (int) bufferSize;
		state->dataTransfers[i]->buffer = malloc(bufferSize);
		if (state->dataTransfers[i]->buffer == NULL) {
			caerLog(LOG_CRITICAL, "Unable to allocate buffer for libusb transfer %zu (data channel). Error: %s (%d).",
				i, caerLogStrerror(errno), errno);

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
			caerLog(LOG_CRITICAL, "Unable to submit libusb transfer %zu (data channel). Error: %s (%d).", i,
				libusb_strerror(errno), errno);

			// The transfer buffer is freed automatically here thanks to
			// the LIBUSB_TRANSFER_FREE_BUFFER flag set above.
			libusb_free_transfer(state->dataTransfers[i]);
			state->dataTransfers[i] = NULL;

			return;
		}
	}
}

static void deallocateDataTransfers(davisFX3State state) {
	// This will change later on, but we still need it.
	uint32_t transfersNum = (uint32_t) atomic_ops_uint_load(&state->dataTransfersLength, ATOMIC_OPS_FENCE_NONE);

	// Cancel all current transfers first.
	for (size_t i = 0; i < transfersNum; i++) {
		errno = libusb_cancel_transfer(state->dataTransfers[i]);
		if (errno != LIBUSB_SUCCESS && errno != LIBUSB_ERROR_NOT_FOUND) {
			caerLog(LOG_CRITICAL, "Unable to cancel libusb transfer %zu (data channel). Error: %s (%d).", i,
				libusb_strerror(errno), errno);
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
	davisFX3State state = transfer->user_data;

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

static void dataTranslator(davisFX3State state, uint8_t *buffer, size_t bytesSent) {
	// Truncate off any extra partial event.
	if ((bytesSent & 0x01) != 0) {
		caerLog(LOG_ALERT, "%zu bytes sent via USB, which is not a multiple of two.", bytesSent);
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
			if (state->currentTimestamp < state->lastTimestamp) {
				caerLog(LOG_ALERT,
					"DAViSFX3: non-monotonic time-stamp detected: lastTimestamp=%" PRIu32 ", currentTimestamp=%" PRIu32 ".",
					state->lastTimestamp, state->currentTimestamp);
			}
		}
		else {
			// Look at the code, to determine event and data type.
			uint8_t code = (uint8_t) ((event & 0x7000) >> 12);
			uint16_t data = (event & 0x0FFF);

			switch (code) {
				case 0: // Special event
					switch (data) {
						case 0: // Ignore this, but log it.
							caerLog(LOG_ERROR, "Caught special reserved event!");
							break;

						case 1: // Timetamp reset
							state->wrapAdd = 0;
							state->lastTimestamp = 0;
							state->currentTimestamp = 0;

							caerLog(LOG_INFO, "Timestamp reset event received.");

							// Create timestamp reset event.
							caerSpecialEvent currentResetEvent = caerSpecialEventPacketGetEvent(
								state->currentSpecialPacket, state->currentSpecialPacketPosition++);
							caerSpecialEventSetTimestamp(currentResetEvent, UINT32_MAX);
							caerSpecialEventSetType(currentResetEvent, TIMESTAMP_RESET);
							caerSpecialEventValidate(currentResetEvent, state->currentSpecialPacket);

							// Commit packets when doing a reset to clearly separate them.
							forcePacketCommit = true;

							break;

						case 2: { // External trigger
							caerSpecialEvent currentExtTriggerEvent = caerSpecialEventPacketGetEvent(
								state->currentSpecialPacket, state->currentSpecialPacketPosition++);
							caerSpecialEventSetTimestamp(currentExtTriggerEvent, state->currentTimestamp);
							caerSpecialEventSetType(currentExtTriggerEvent, EXTERNAL_TRIGGER);
							caerSpecialEventValidate(currentExtTriggerEvent, state->currentSpecialPacket);
							break;
						}

						default:
							caerLog(LOG_ERROR, "Caught special event that can't be handled.");
							break;
					}
					break;

				case 1: // Y address
					// Check range conformity.
					if (data >= DAVIS_FX3_ARRAY_SIZE_Y) {
						caerLog(LOG_ALERT, "Y address out of range (0-%d): %" PRIu16 ".",
						DAVIS_FX3_ARRAY_SIZE_Y - 1, data);
						continue; // Skip invalid Y address (don't update lastY).
					}

					if (state->gotY) {
						caerSpecialEvent currentRowOnlyEvent = caerSpecialEventPacketGetEvent(
							state->currentSpecialPacket, state->currentSpecialPacketPosition++);
						caerSpecialEventSetTimestamp(currentRowOnlyEvent, state->currentTimestamp);
						caerSpecialEventSetType(currentRowOnlyEvent, ROW_ONLY);
						caerSpecialEventValidate(currentRowOnlyEvent, state->currentSpecialPacket);

						caerLog(LOG_DEBUG, "Row only event at address Y=%" PRIu16 ".", state->lastY);
					}

					state->lastY = data;
					state->gotY = true;

					break;

				case 2: // X address, Polarity OFF
				case 3: { // X address, Polarity ON
					// Check range conformity.
					if (data >= DAVIS_FX3_ARRAY_SIZE_X) {
						caerLog(LOG_ALERT, "X address out of range (0-%d): %" PRIu16 ".",
						DAVIS_FX3_ARRAY_SIZE_X - 1, data);
						continue; // Skip invalid event.
					}

					caerPolarityEvent currentPolarityEvent = caerPolarityEventPacketGetEvent(
						state->currentPolarityPacket, state->currentPolarityPacketPosition++);
					caerPolarityEventSetTimestamp(currentPolarityEvent, state->currentTimestamp);
					caerPolarityEventSetPolarity(currentPolarityEvent, (code & 0x01));
					caerPolarityEventSetY(currentPolarityEvent, state->lastY);
					caerPolarityEventSetX(currentPolarityEvent, data);
					caerPolarityEventValidate(currentPolarityEvent, state->currentPolarityPacket);

					state->gotY = false;

					break;
				}

				case 7: // Timestamp wrap
					// Each wrap is 2^15 µs (~32ms), and we have
					// to multiply it with the wrap counter,
					// which is located in the data part of this
					// event.
					state->wrapAdd += (uint32_t) (0x8000 * data);

					caerLog(LOG_INFO, "Timestamp wrap event received with multiplier of %" PRIu16 ".", data);
					break;

				default:
					caerLog(LOG_ERROR, "Caught event that can't be handled.");
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
				caerLog(LOG_DEBUG, "Dropped Polarity Event Packet because ring-buffer full!");
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
					caerLog(LOG_DEBUG, "Dropped Special Event Packet because ring-buffer full!");
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

static void allocateDebugTransfers(davisFX3State state) {
	atomic_ops_uint_store(&state->debugTransfersLength, 0, ATOMIC_OPS_FENCE_NONE);

	// Set number of transfers and allocate memory for the main transfer array.
	state->debugTransfers = calloc(DEBUG_TRANSFER_NUM, sizeof(struct libusb_transfer *));
	if (state->debugTransfers == NULL) {
		caerLog(LOG_CRITICAL,
			"Failed to allocate memory for %" PRIu32 " libusb transfers (debug channel). Error: %s (%d).",
			DEBUG_TRANSFER_NUM, caerLogStrerror(errno), errno);
		return;
	}

	// Allocate transfers and set them up.
	for (size_t i = 0; i < DEBUG_TRANSFER_NUM; i++) {
		state->debugTransfers[i] = libusb_alloc_transfer(0);
		if (state->debugTransfers[i] == NULL) {
			caerLog(LOG_CRITICAL, "Unable to allocate further libusb transfers (debug channel, %zu of %" PRIu32 ").", i,
			DEBUG_TRANSFER_NUM);
			return;
		}

		// Create data buffer.
		state->debugTransfers[i]->length = DEBUG_TRANSFER_SIZE;
		state->debugTransfers[i]->buffer = malloc(DEBUG_TRANSFER_SIZE);
		if (state->debugTransfers[i]->buffer == NULL) {
			caerLog(LOG_CRITICAL, "Unable to allocate buffer for libusb transfer %zu (debug channel). Error: %s (%d).",
				i, caerLogStrerror(errno), errno);

			libusb_free_transfer(state->debugTransfers[i]);
			state->debugTransfers[i] = NULL;

			return;
		}

		// Initialize Transfer.
		state->debugTransfers[i]->dev_handle = state->deviceHandle;
		state->debugTransfers[i]->endpoint = DEBUG_ENDPOINT;
		state->debugTransfers[i]->type = LIBUSB_TRANSFER_TYPE_INTERRUPT;
		state->debugTransfers[i]->callback = &libUsbDebugCallback;
		state->debugTransfers[i]->user_data = state;
		state->debugTransfers[i]->timeout = 0;
		state->debugTransfers[i]->flags = LIBUSB_TRANSFER_FREE_BUFFER;

		if ((errno = libusb_submit_transfer(state->debugTransfers[i])) == LIBUSB_SUCCESS) {
			atomic_ops_uint_inc(&state->debugTransfersLength, ATOMIC_OPS_FENCE_NONE);
		}
		else {
			caerLog(LOG_CRITICAL, "Unable to submit libusb transfer %zu (debug channel). Error: %s (%d).", i,
				libusb_strerror(errno), errno);

			// The transfer buffer is freed automatically here thanks to
			// the LIBUSB_TRANSFER_FREE_BUFFER flag set above.
			libusb_free_transfer(state->debugTransfers[i]);
			state->debugTransfers[i] = NULL;

			return;
		}
	}
}

static void deallocateDebugTransfers(davisFX3State state) {
	// This will change later on, but we still need it.
	uint32_t transfersNum = (uint32_t) atomic_ops_uint_load(&state->debugTransfersLength, ATOMIC_OPS_FENCE_NONE);

	// Cancel all current transfers first.
	for (size_t i = 0; i < transfersNum; i++) {
		errno = libusb_cancel_transfer(state->debugTransfers[i]);
		if (errno != LIBUSB_SUCCESS && errno != LIBUSB_ERROR_NOT_FOUND) {
			caerLog(LOG_CRITICAL, "Unable to cancel libusb transfer %zu (debug channel). Error: %s (%d).", i,
				libusb_strerror(errno), errno);
			// Proceed with canceling all transfers regardless of errors.
		}
	}

	// Wait for all transfers to go away (0.1 seconds timeout).
	struct timeval te = { .tv_sec = 0, .tv_usec = 100000 };

	while (atomic_ops_uint_load(&state->debugTransfersLength, ATOMIC_OPS_FENCE_NONE) > 0) {
		libusb_handle_events_timeout(state->deviceContext, &te);
	}

	// The buffers and transfers have been deallocated in the callback.
	// Only the transfers array remains.
	free(state->debugTransfers);
}

static void LIBUSB_CALL libUsbDebugCallback(struct libusb_transfer *transfer) {
	davisFX3State state = transfer->user_data;

	if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
		// Handle debug data.
		debugTranslator(state, transfer->buffer, (size_t) transfer->actual_length);
	}

	if (transfer->status != LIBUSB_TRANSFER_CANCELLED && transfer->status != LIBUSB_TRANSFER_NO_DEVICE) {
		// Submit transfer again.
		if (libusb_submit_transfer(transfer) == LIBUSB_SUCCESS) {
			return;
		}
	}

	// Cannot recover (cancelled, no device, or other critical error).
	// Signal this by adjusting the counter, free and exit.
	atomic_ops_uint_dec(&state->debugTransfersLength, ATOMIC_OPS_FENCE_NONE);
	libusb_free_transfer(transfer);
}

static void debugTranslator(davisFX3State state, uint8_t *buffer, size_t bytesSent) {
	UNUSED_ARGUMENT(state);

	// Check if this is a debug message (length 7-64 bytes).
	if (bytesSent >= 7 && buffer[0] == 0x00) {
		// Debug message, log this.
		caerLog(LOG_ERROR, "Error message from DAViSFX3: '%s' (code %u at time %u).", &buffer[6], buffer[1],
			*((uint32_t *) &buffer[2]));
	}
	else {
		// Unknown/invalid debug message, log this.
		caerLog(LOG_WARNING, "Unknown/invalid debug message from DAViSFX3.");
	}
}

static void sendBiases(sshsNode biasNode, libusb_device_handle *devHandle) {
	// Biases are addressable now!
	sendAddressedCoarseFineBias(biasNode, devHandle, 0, "DiffBn");
	sendAddressedCoarseFineBias(biasNode, devHandle, 1, "OnBn");
	sendAddressedCoarseFineBias(biasNode, devHandle, 2, "OffBn");
	sendAddressedCoarseFineBias(biasNode, devHandle, 3, "ApsCasEpc");
	sendAddressedCoarseFineBias(biasNode, devHandle, 4, "DiffCasBnc");
	sendAddressedCoarseFineBias(biasNode, devHandle, 5, "ApsROSFBn");
	sendAddressedCoarseFineBias(biasNode, devHandle, 6, "LocalBufBn");
	sendAddressedCoarseFineBias(biasNode, devHandle, 7, "PixInvBn");
	sendAddressedCoarseFineBias(biasNode, devHandle, 8, "PrBp");
	sendAddressedCoarseFineBias(biasNode, devHandle, 9, "PrSFBp");
	sendAddressedCoarseFineBias(biasNode, devHandle, 10, "RefrBp");
	sendAddressedCoarseFineBias(biasNode, devHandle, 11, "AEPdBn");
	sendAddressedCoarseFineBias(biasNode, devHandle, 12, "LcolTimeoutBn");
	sendAddressedCoarseFineBias(biasNode, devHandle, 13, "AEPuXBp");
	sendAddressedCoarseFineBias(biasNode, devHandle, 14, "AEPuYBp");
	sendAddressedCoarseFineBias(biasNode, devHandle, 15, "IFThrBn");
	sendAddressedCoarseFineBias(biasNode, devHandle, 16, "IFRefrBn");
	sendAddressedCoarseFineBias(biasNode, devHandle, 17, "PadFollBn");
	sendAddressedCoarseFineBias(biasNode, devHandle, 18, "apsOverflowLevel");
	sendAddressedCoarseFineBias(biasNode, devHandle, 19, "biasBuffer");
	sendShiftedSourceBias(biasNode, devHandle, 20, "SSP");
	sendShiftedSourceBias(biasNode, devHandle, 21, "SSN");
}

void sendChipSR(sshsNode chipNode, libusb_device_handle *devHandle) {
	// A total of 56 bits (7 bytes) of configuration
	uint8_t chipSR[7] = { 0 };

	// Muxes are all kept at zero for now (no control). (TODO)

	// Bytes 2-4 contain the actual 24 configuration bits. 17 are unused.
	bool globalShutter = sshsNodeGetBool(chipNode, "globalShutter");
	if (globalShutter) {
		// Flip bit on if enabled.
		chipSR[4] |= (1 << 6);
	}

	bool useAout = sshsNodeGetBool(chipNode, "useAout");
	if (useAout) {
		// Flip bit on if enabled.
		chipSR[4] |= (1 << 5);
	}

	bool nArow = sshsNodeGetBool(chipNode, "nArow");
	if (nArow) {
		// Flip bit on if enabled.
		chipSR[4] |= (1 << 4);
	}

	bool hotPixelSuppression = sshsNodeGetBool(chipNode, "hotPixelSuppression");
	if (hotPixelSuppression) {
		// Flip bit on if enabled.
		chipSR[4] |= (1 << 3);
	}

	bool resetTestpixel = sshsNodeGetBool(chipNode, "resetTestpixel");
	if (resetTestpixel) {
		// Flip bit on if enabled.
		chipSR[4] |= (1 << 2);
	}

	bool typeNCalib = sshsNodeGetBool(chipNode, "typeNCalib");
	if (typeNCalib) {
		// Flip bit on if enabled.
		chipSR[4] |= (1 << 1);
	}

	bool resetCalib = sshsNodeGetBool(chipNode, "resetCalib");
	if (resetCalib) {
		// Flip bit on if enabled.
		chipSR[4] |= (1 << 0);
	}

	libusb_control_transfer(devHandle, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
	VR_CHIP_DIAG, 0, 0, chipSR, sizeof(chipSR), 0);
}

void sendFpgaSR(sshsNode fpgaNode, libusb_device_handle *devHandle) {
	// Five two byte configuration settings
	uint8_t fpgaSR[5 * 2] = { 0 };

	uint16_t frameDelay = sshsNodeGetShort(fpgaNode, "frameDelay");
	fpgaSR[0] = U8T(frameDelay >> 8);
	fpgaSR[1] = U8T(frameDelay >> 0);

	uint16_t resSettle = sshsNodeGetShort(fpgaNode, "resSettle");
	fpgaSR[2] = U8T(resSettle >> 8);
	fpgaSR[3] = U8T(resSettle >> 0);

	uint16_t rowSettle = sshsNodeGetShort(fpgaNode, "rowSettle");
	fpgaSR[4] = U8T(rowSettle >> 8);
	fpgaSR[5] = U8T(rowSettle >> 0);

	uint16_t colSettle = sshsNodeGetShort(fpgaNode, "colSettle");
	fpgaSR[6] = U8T(colSettle >> 8);
	fpgaSR[7] = U8T(colSettle >> 0);

	uint16_t exposure = sshsNodeGetShort(fpgaNode, "exposure");
	fpgaSR[8] = U8T(exposure >> 8);
	fpgaSR[9] = U8T(exposure >> 0);

	libusb_control_transfer(devHandle, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
	VR_FPGA_SREG, 0, 0, fpgaSR, sizeof(fpgaSR), 0);
}

static libusb_device_handle *deviceOpen(libusb_context *devContext, uint8_t busNumber, uint8_t devAddress) {
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
			if (devDesc.idVendor == DAVIS_FX3_VID && devDesc.idProduct == DAVIS_FX3_PID
				&& (uint8_t) ((devDesc.bcdDevice & 0xFF00) >> 8) == DAVIS_FX3_DID_TYPE) {
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

static void caerInputDAViSFX3ConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(changeValue);

	caerModuleData data = userData;

	// Distinguish changes to biases, or USB transfers, or others, by
	// using configUpdate like a bit-field.
	if (event == ATTRIBUTE_MODIFIED) {
		// Changes to the bias node.
		if (str_equals(sshsNodeGetName(node), "bias") && changeType == SHORT) {
			atomic_ops_uint_or(&data->configUpdate, (0x01 << 0), ATOMIC_OPS_FENCE_NONE);
		}

		// Changes to the chip config node.
		if (str_equals(sshsNodeGetName(node), "chip") && changeType == BOOL) {
			atomic_ops_uint_or(&data->configUpdate, (0x01 << 1), ATOMIC_OPS_FENCE_NONE);
		}

		// Changes to the FPGA config node.
		if (str_equals(sshsNodeGetName(node), "fpga") && changeType == SHORT) {
			atomic_ops_uint_or(&data->configUpdate, (0x01 << 2), ATOMIC_OPS_FENCE_NONE);
		}

		// Changes to the USB transfer settings (requires reallocation).
		if (changeType == INT && (str_equals(changeKey, "bufferNumber") || str_equals(changeKey, "bufferSize"))) {
			atomic_ops_uint_or(&data->configUpdate, (0x01 << 3), ATOMIC_OPS_FENCE_NONE);
		}

		// Changes to packet size and interval.
		if (changeType == INT
			&& (str_equals(changeKey, "polarityPacketMaxSize") || str_equals(changeKey, "polarityPacketMaxInterval")
				|| str_equals(changeKey, "framePacketMaxSize") || str_equals(changeKey, "framePacketMaxInterval")
				|| str_equals(changeKey, "imu6PacketMaxSize") || str_equals(changeKey, "imu6PacketMaxInterval")
				|| str_equals(changeKey, "specialPacketMaxSize") || str_equals(changeKey, "specialPacketMaxInterval"))) {
			atomic_ops_uint_or(&data->configUpdate, (0x01 << 4), ATOMIC_OPS_FENCE_NONE);
		}
	}
}
