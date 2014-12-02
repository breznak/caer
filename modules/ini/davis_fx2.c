#include "davis_common.h"
#include "davis_fx2.h"
#include "base/module.h"
#include <pthread.h>
#include <unistd.h>

struct davisFX2_state {
	// State for data management, common to all DAVISes.
	struct davisCommon_state cstate;
};

typedef struct davisFX2_state *davisFX2State;

static bool caerInputDAVISFX2Init(caerModuleData moduleData);
// RUN: common to all DAVIS systems.
// CONFIG: Nothing to do here in the main thread!
// Biases are configured asynchronously, and buffer sizes in the data
// acquisition thread itself. Resetting the main config_refresh flag
// will also happen there.
static void caerInputDAVISFX2Exit(caerModuleData moduleData);

static struct caer_module_functions caerInputDAVISFX2Functions = { .moduleInit = &caerInputDAVISFX2Init, .moduleRun =
	&caerInputDAVISCommonRun, .moduleConfig = NULL, .moduleExit = &caerInputDAVISFX2Exit };

void caerInputDAVISFX2(uint16_t moduleID, caerPolarityEventPacket *polarity, caerFrameEventPacket *frame,
	caerIMU6EventPacket *imu6, caerSpecialEventPacket *special) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "DAVISFX2");

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

	caerModuleSM(&caerInputDAVISFX2Functions, moduleData, sizeof(struct davisFX2_state), 4, polarity, frame, imu6,
		special);
}

static void *dataAcquisitionThread(void *inPtr);
static void dataAcquisitionThreadConfig(caerModuleData data);
static void sendBias(libusb_device_handle *devHandle, uint16_t biasAddress, uint16_t biasValue);
static void sendBiases(sshsNode moduleNode, libusb_device_handle *devHandle);
static void sendChipSR(sshsNode moduleNode, libusb_device_handle *devHandle);

static bool caerInputDAVISFX2Init(caerModuleData moduleData) {
	caerLog(LOG_DEBUG, moduleData->moduleSubSystemString, "Initializing module ...");

	davisFX2State state = moduleData->moduleState;
	davisCommonState cstate = &state->cstate;

	// Data source is the same as the module ID (but accessible in cstate-space).
	// Same thing for subSystemString.
	cstate->sourceID = moduleData->moduleID;
	cstate->sourceSubSystemString = moduleData->moduleSubSystemString;

	// First, we need to connect to the device and ask it what chip it's got,
	// and retain that information for later stages.
	if (!deviceOpenInfo(moduleData, cstate, DAVIS_FX2_VID, DAVIS_FX2_PID, DAVIS_FX2_DID_TYPE)) {
		return (false);
	}

	createCommonConfiguration(moduleData, cstate);

	if (!initializeCommonConfiguration(moduleData, cstate, &dataAcquisitionThread)) {
		return (false);
	}

	caerLog(LOG_DEBUG, moduleData->moduleSubSystemString, "Initialized DAVISFX2 module successfully.");
	return (true);
}

static void caerInputDAVISFX2Exit(caerModuleData moduleData) {
	caerLog(LOG_DEBUG, moduleData->moduleSubSystemString, "Shutting down ...");

	davisFX2State state = moduleData->moduleState;
	davisCommonState cstate = &state->cstate;

	// Wait for data acquisition thread to terminate...
	if ((errno = pthread_join(state->cstate.dataAcquisitionThread, NULL)) != 0) {
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

static void *dataAcquisitionThread(void *inPtr) {
	// inPtr is a pointer to module data.
	caerModuleData data = inPtr;
	davisFX2State state = data->moduleState;
	davisCommonState cstate = &state->cstate;

	caerLog(LOG_DEBUG, data->moduleSubSystemString, "Initializing data acquisition thread ...");

	// Create buffers as specified in config file.
	sshsNode usbNode = sshsGetRelativeNode(data->moduleNode, "usb/");
	allocateDataTransfers(cstate, sshsNodeGetInt(usbNode, "BufferNumber"), sshsNodeGetInt(usbNode, "BufferSize"));

	// Send default start-up biases and config values to device before enabling it.
	sendBiases(data->moduleNode, cstate->deviceHandle);
	sendChipSR(data->moduleNode, cstate->deviceHandle);
	sendEnableDataConfig(data->moduleNode, cstate->deviceHandle);

	// Handle USB events (1 second timeout).
	struct timeval te = { .tv_sec = 0, .tv_usec = 1000000 };

	caerLog(LOG_DEBUG, data->moduleSubSystemString, "Data acquisition thread ready to process events.");

	while (atomic_ops_uint_load(&data->running, ATOMIC_OPS_FENCE_NONE) != 0
		&& atomic_ops_uint_load(&cstate->dataTransfersLength, ATOMIC_OPS_FENCE_NONE) > 0) {
		// Check config refresh, in this case to adjust buffer sizes.
		if (atomic_ops_uint_load(&data->configUpdate, ATOMIC_OPS_FENCE_NONE) != 0) {
			dataAcquisitionThreadConfig(data);
		}

		libusb_handle_events_timeout(cstate->deviceContext, &te);
	}

	caerLog(LOG_DEBUG, data->moduleSubSystemString, "Shutting down data acquisition thread ...");

	// Disable all data transfer on USB end-point.
	sendDisableDataConfig(cstate->deviceHandle);

	// Cancel all transfers and handle them.
	deallocateDataTransfers(cstate);

	// Ensure parent also shuts down (on disconnected device for example).
	sshsNodePutBool(data->moduleNode, "shutdown", true);

	caerLog(LOG_DEBUG, data->moduleSubSystemString, "Data acquisition thread shut down.");

	return (NULL);
}

static void dataAcquisitionThreadConfig(caerModuleData moduleData) {
	davisFX2State state = moduleData->moduleState;
	davisCommonState cstate = &state->cstate;

	// Get the current value to examine by atomic exchange, since we don't
	// want there to be any possible store between a load/store pair.
	uintptr_t configUpdate = atomic_ops_uint_swap(&moduleData->configUpdate, 0, ATOMIC_OPS_FENCE_NONE);

	if (configUpdate & (0x01 << 0)) {
		sendBiases(moduleData->moduleNode, cstate->deviceHandle);
	}

	if (configUpdate & (0x01 << 1)) {
		sendChipSR(moduleData->moduleNode, cstate->deviceHandle);
	}

	if (configUpdate & (0x01 << 2)) {
		sendMultiplexerConfig(moduleData->moduleNode, cstate->deviceHandle);

		// If timestamp reset was changed, we put it back to OFF, since
		// it's just an instant pulse to the device.
		sshsNode muxNode = sshsGetRelativeNode(moduleData->moduleNode, "multiplexer/");
		if (sshsNodeGetBool(muxNode, "TimestampReset")) {
			sshsNodePutBool(muxNode, "TimestampReset", 0);
		}
	}

	if (configUpdate & (0x01 << 3)) {
		sendDVSConfig(moduleData->moduleNode, cstate->deviceHandle);
	}

	if (configUpdate & (0x01 << 4)) {
		sendAPSConfig(moduleData->moduleNode, cstate->deviceHandle);
	}

	if (configUpdate & (0x01 << 5)) {
		sendIMUConfig(moduleData->moduleNode, cstate->deviceHandle);
	}

	if (configUpdate & (0x01 << 6)) {
		sendExternalInputDetectorConfig(moduleData->moduleNode, cstate->deviceHandle);
	}

	if (configUpdate & (0x01 << 7)) {
		sendUSBConfig(moduleData->moduleNode, cstate->deviceHandle);
	}

	if (configUpdate & (0x01 << 8)) {
		reallocateUSBBuffers(moduleData->moduleNode, cstate);
	}

	if (configUpdate & (0x01 << 9)) {
		updatePacketSizesIntervals(moduleData->moduleNode, cstate);
	}
}

static void sendBias(libusb_device_handle *devHandle, uint16_t biasAddress, uint16_t biasValue) {
	// All biases are two byte quantities.
	uint8_t bias[2];

	// Put the value in.
	bias[0] = U8T(biasValue >> 8);
	bias[1] = U8T(biasValue >> 0);

	libusb_control_transfer(devHandle, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
	VR_CHIP_BIAS, biasAddress, 0, bias, sizeof(bias), 0);
}

static void sendBiases(sshsNode moduleNode, libusb_device_handle *devHandle) {
	sshsNode biasNode = sshsGetRelativeNode(moduleNode, "bias/");

	// Biases are addressable now!
	sendBias(devHandle, 0, generateAddressedCoarseFineBias(biasNode, "DiffBn"));
	sendBias(devHandle, 1, generateAddressedCoarseFineBias(biasNode, "OnBn"));
	sendBias(devHandle, 2, generateAddressedCoarseFineBias(biasNode, "OffBn"));
	sendBias(devHandle, 3, generateAddressedCoarseFineBias(biasNode, "ApsCasEpc"));
	sendBias(devHandle, 4, generateAddressedCoarseFineBias(biasNode, "DiffCasBnc"));
	sendBias(devHandle, 5, generateAddressedCoarseFineBias(biasNode, "ApsROSFBn"));
	sendBias(devHandle, 6, generateAddressedCoarseFineBias(biasNode, "LocalBufBn"));
	sendBias(devHandle, 7, generateAddressedCoarseFineBias(biasNode, "PixInvBn"));
	sendBias(devHandle, 8, generateAddressedCoarseFineBias(biasNode, "PrBp"));
	sendBias(devHandle, 9, generateAddressedCoarseFineBias(biasNode, "PrSFBp"));
	sendBias(devHandle, 10, generateAddressedCoarseFineBias(biasNode, "RefrBp"));
	sendBias(devHandle, 11, generateAddressedCoarseFineBias(biasNode, "AEPdBn"));
	sendBias(devHandle, 12, generateAddressedCoarseFineBias(biasNode, "LcolTimeoutBn"));
	sendBias(devHandle, 13, generateAddressedCoarseFineBias(biasNode, "AEPuXBp"));
	sendBias(devHandle, 14, generateAddressedCoarseFineBias(biasNode, "AEPuYBp"));
	sendBias(devHandle, 15, generateAddressedCoarseFineBias(biasNode, "IFThrBn"));
	sendBias(devHandle, 16, generateAddressedCoarseFineBias(biasNode, "IFRefrBn"));
	sendBias(devHandle, 17, generateAddressedCoarseFineBias(biasNode, "PadFollBn"));
	sendBias(devHandle, 18, generateAddressedCoarseFineBias(biasNode, "ApsOverflowLevel"));
	sendBias(devHandle, 19, generateAddressedCoarseFineBias(biasNode, "BiasBuffer"));
	sendBias(devHandle, 20, generateShiftedSourceBias(biasNode, "SSP"));
	sendBias(devHandle, 21, generateShiftedSourceBias(biasNode, "SSN"));
}

static void sendChipSR(sshsNode moduleNode, libusb_device_handle *devHandle) {
	sshsNode chipNode = sshsGetRelativeNode(moduleNode, "chip/");
	sshsNode apsNode = sshsGetRelativeNode(moduleNode, "aps/");

	// A total of 56 bits (7 bytes) of configuration.
	uint8_t chipSR[7] = { 0 };

	// Debug muxes control.
	chipSR[0] |= U8T((sshsNodeGetByte(chipNode, "DigitalMux3") & 0x0F) << 4);
	chipSR[0] |= U8T((sshsNodeGetByte(chipNode, "DigitalMux2") & 0x0F) << 0);
	chipSR[1] |= U8T((sshsNodeGetByte(chipNode, "DigitalMux1") & 0x0F) << 4);
	chipSR[1] |= U8T((sshsNodeGetByte(chipNode, "DigitalMux0") & 0x0F) << 0);

	chipSR[5] |= U8T((sshsNodeGetByte(chipNode, "AnalogMux2") & 0x0F) << 4);
	chipSR[5] |= U8T((sshsNodeGetByte(chipNode, "AnalogMux1") & 0x0F) << 0);
	chipSR[6] |= U8T((sshsNodeGetByte(chipNode, "AnalogMux0") & 0x0F) << 4);

	chipSR[6] |= U8T((sshsNodeGetByte(chipNode, "BiasMux") & 0x0F) << 0);

	// Bytes 2-4 contain the actual 24 configuration bits. 17 are unused.
	bool globalShutter = sshsNodeGetBool(apsNode, "GlobalShutter");
	if (globalShutter) {
		// Flip bit on if enabled.
		chipSR[4] |= (1 << 6);
	}

	bool useAout = sshsNodeGetBool(chipNode, "UseAout");
	if (useAout) {
		// Flip bit on if enabled.
		chipSR[4] |= (1 << 5);
	}

	bool nArow = sshsNodeGetBool(chipNode, "nArow");
	if (nArow) {
		// Flip bit on if enabled.
		chipSR[4] |= (1 << 4);
	}

	bool hotPixelSuppression = sshsNodeGetBool(chipNode, "HotPixelSuppression");
	if (hotPixelSuppression) {
		// Flip bit on if enabled.
		chipSR[4] |= (1 << 3);
	}

	bool resetTestpixel = sshsNodeGetBool(chipNode, "ResetTestPixel");
	if (resetTestpixel) {
		// Flip bit on if enabled.
		chipSR[4] |= (1 << 2);
	}

	bool typeNCalib = sshsNodeGetBool(chipNode, "TypeNCalibNeuron");
	if (typeNCalib) {
		// Flip bit on if enabled.
		chipSR[4] |= (1 << 1);
	}

	bool resetCalib = sshsNodeGetBool(chipNode, "ResetCalibNeuron");
	if (resetCalib) {
		// Flip bit on if enabled.
		chipSR[4] |= (1 << 0);
	}

	libusb_control_transfer(devHandle, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
	VR_CHIP_DIAG, 0, 0, chipSR, sizeof(chipSR), 0);
}
