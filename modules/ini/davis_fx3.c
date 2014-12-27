#include "davis_common.h"
#include "davis_fx3.h"
#include "base/module.h"
#include <pthread.h>
#include <unistd.h>

struct davisFX3_state {
	// State for data management, common to all DAVISes.
	struct davisCommon_state cstate;
	// Debug transfer support (FX3 only).
	struct libusb_transfer **debugTransfers;
	atomic_ops_uint debugTransfersLength;
};

typedef struct davisFX3_state *davisFX3State;

static bool caerInputDAVISFX3Init(caerModuleData moduleData);
// RUN: common to all DAVIS systems.
// CONFIG: Nothing to do here in the main thread!
// Biases are configured asynchronously, and buffer sizes in the data
// acquisition thread itself. Resetting the main config_refresh flag
// will also happen there.
static void caerInputDAVISFX3Exit(caerModuleData moduleData);

static struct caer_module_functions caerInputDAVISFX3Functions = { .moduleInit = &caerInputDAVISFX3Init, .moduleRun =
	&caerInputDAVISCommonRun, .moduleConfig = NULL, .moduleExit = &caerInputDAVISFX3Exit };

void caerInputDAVISFX3(uint16_t moduleID, caerPolarityEventPacket *polarity, caerFrameEventPacket *frame,
	caerIMU6EventPacket *imu6, caerSpecialEventPacket *special) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "DAVISFX3");

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

	caerModuleSM(&caerInputDAVISFX3Functions, moduleData, sizeof(struct davisFX3_state), 4, polarity, frame, imu6,
		special);
}

static void *dataAcquisitionThread(void *inPtr);
static void dataAcquisitionThreadConfig(caerModuleData data);
static void allocateDebugTransfers(davisFX3State state);
static void deallocateDebugTransfers(davisFX3State state);
static void LIBUSB_CALL libUsbDebugCallback(struct libusb_transfer *transfer);
static void debugTranslator(davisFX3State state, uint8_t *buffer, size_t bytesSent);
static void sendBiases(sshsNode moduleNode, libusb_device_handle *devHandle);
static void sendChipSR(sshsNode moduleNode, libusb_device_handle *devHandle);
static void sendDVSFilterConfig(sshsNode moduleNode, libusb_device_handle *devHandle);
static void sendAPSQuadROIConfig(sshsNode moduleNode, libusb_device_handle *devHandle);
static void sendExternalInputGeneratorConfig(sshsNode moduleNode, libusb_device_handle *devHandle);

static bool caerInputDAVISFX3Init(caerModuleData moduleData) {
	caerLog(LOG_DEBUG, moduleData->moduleSubSystemString, "Initializing module ...");

	davisFX3State state = moduleData->moduleState;
	davisCommonState cstate = &state->cstate;

	// Data source is the same as the module ID (but accessible in cstate-space).
	// Same thing for subSystemString.
	cstate->sourceID = moduleData->moduleID;
	cstate->sourceSubSystemString = moduleData->moduleSubSystemString;

	// First, we need to connect to the device and ask it what chip it's got,
	// and retain that information for later stages.
	if (!deviceOpenInfo(moduleData, cstate, DAVIS_FX3_VID, DAVIS_FX3_PID, DAVIS_FX3_DID_TYPE)) {
		return (false);
	}

	createCommonConfiguration(moduleData, cstate);

	// Subsystem 1: DVS AER (Pixel and BA filtering support present only in FX3)
	sshsNode dvsNode = sshsGetRelativeNode(moduleData->moduleNode, "dvs/");

	sshsNodePutShortIfAbsent(dvsNode, "FilterPixel0Row", cstate->apsSizeY);
	sshsNodePutShortIfAbsent(dvsNode, "FilterPixel0Column", cstate->apsSizeX);
	sshsNodePutShortIfAbsent(dvsNode, "FilterPixel1Row", cstate->apsSizeY);
	sshsNodePutShortIfAbsent(dvsNode, "FilterPixel1Column", cstate->apsSizeX);
	sshsNodePutShortIfAbsent(dvsNode, "FilterPixel2Row", cstate->apsSizeY);
	sshsNodePutShortIfAbsent(dvsNode, "FilterPixel2Column", cstate->apsSizeX);
	sshsNodePutShortIfAbsent(dvsNode, "FilterPixel3Row", cstate->apsSizeY);
	sshsNodePutShortIfAbsent(dvsNode, "FilterPixel3Column", cstate->apsSizeX);
	sshsNodePutShortIfAbsent(dvsNode, "FilterPixel4Row", cstate->apsSizeY);
	sshsNodePutShortIfAbsent(dvsNode, "FilterPixel4Column", cstate->apsSizeX);
	sshsNodePutShortIfAbsent(dvsNode, "FilterPixel5Row", cstate->apsSizeY);
	sshsNodePutShortIfAbsent(dvsNode, "FilterPixel5Column", cstate->apsSizeX);
	sshsNodePutShortIfAbsent(dvsNode, "FilterPixel6Row", cstate->apsSizeY);
	sshsNodePutShortIfAbsent(dvsNode, "FilterPixel6Column", cstate->apsSizeX);
	sshsNodePutShortIfAbsent(dvsNode, "FilterPixel7Row", cstate->apsSizeY);
	sshsNodePutShortIfAbsent(dvsNode, "FilterPixel7Column", cstate->apsSizeX);
	sshsNodePutBoolIfAbsent(dvsNode, "FilterBackgroundActivity", 0);
	sshsNodePutIntIfAbsent(dvsNode, "FilterBackgroundActivityDeltaTime", 20000);

	// Subsystem 2: APS ADC (Quad-ROI support present only in FX3)
	sshsNode apsNode = sshsGetRelativeNode(moduleData->moduleNode, "aps/");

	sshsNodePutShortIfAbsent(apsNode, "StartColumn1", cstate->apsSizeX);
	sshsNodePutShortIfAbsent(apsNode, "StartRow1", cstate->apsSizeY);
	sshsNodePutShortIfAbsent(apsNode, "EndColumn1", cstate->apsSizeX);
	sshsNodePutShortIfAbsent(apsNode, "EndRow1", cstate->apsSizeY);
	sshsNodePutShortIfAbsent(apsNode, "StartColumn2", cstate->apsSizeX);
	sshsNodePutShortIfAbsent(apsNode, "StartRow2", cstate->apsSizeY);
	sshsNodePutShortIfAbsent(apsNode, "EndColumn2", cstate->apsSizeX);
	sshsNodePutShortIfAbsent(apsNode, "EndRow2", cstate->apsSizeY);
	sshsNodePutShortIfAbsent(apsNode, "StartColumn3", cstate->apsSizeX);
	sshsNodePutShortIfAbsent(apsNode, "StartRow3", cstate->apsSizeY);
	sshsNodePutShortIfAbsent(apsNode, "EndColumn3", cstate->apsSizeX);
	sshsNodePutShortIfAbsent(apsNode, "EndRow3", cstate->apsSizeY);

	// Subsystem 4: External Input (Generator module present only in FX3)
	sshsNode extNode = sshsGetRelativeNode(moduleData->moduleNode, "externalInput/");

	sshsNodePutBoolIfAbsent(extNode, "RunGenerator", 0);
	sshsNodePutBoolIfAbsent(extNode, "GenerateUseCustomSignal", 0);
	sshsNodePutBoolIfAbsent(extNode, "GeneratePulsePolarity", 1);
	sshsNodePutIntIfAbsent(extNode, "GeneratePulseInterval", 10);
	sshsNodePutIntIfAbsent(extNode, "GeneratePulseLength", 5);

	if (!initializeCommonConfiguration(moduleData, cstate, &dataAcquisitionThread)) {
		return (false);
	}

	caerLog(LOG_DEBUG, moduleData->moduleSubSystemString, "Initialized DAVISFX3 module successfully.");
	return (true);
}

static void caerInputDAVISFX3Exit(caerModuleData moduleData) {
	caerLog(LOG_DEBUG, moduleData->moduleSubSystemString, "Shutting down ...");

	davisFX3State state = moduleData->moduleState;
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
	davisFX3State state = data->moduleState;
	davisCommonState cstate = &state->cstate;

	caerLog(LOG_DEBUG, data->moduleSubSystemString, "Initializing data acquisition thread ...");

	// Create buffers as specified in config file.
	sshsNode usbNode = sshsGetRelativeNode(data->moduleNode, "usb/");
	allocateDebugTransfers(state);
	allocateDataTransfers(cstate, sshsNodeGetInt(usbNode, "BufferNumber"), sshsNodeGetInt(usbNode, "BufferSize"));

	// Send default start-up biases and config values to device before enabling it.
	sendBiases(data->moduleNode, cstate->deviceHandle);
	sendChipSR(data->moduleNode, cstate->deviceHandle);
	sendEnableDataConfig(data->moduleNode, cstate->deviceHandle);
	sendExternalInputGeneratorConfig(data->moduleNode, cstate->deviceHandle); // FX3 only.

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
	spiConfigSend(cstate->deviceHandle, FPGA_EXTINPUT, 6, 0); // FX3 only.
	sendDisableDataConfig(cstate->deviceHandle);

	// Cancel all transfers and handle them.
	deallocateDataTransfers(cstate);
	deallocateDebugTransfers(state);

	// Ensure parent also shuts down (on disconnected device for example).
	sshsNodePutBool(data->moduleNode, "shutdown", true);

	caerLog(LOG_DEBUG, data->moduleSubSystemString, "Data acquisition thread shut down.");

	return (NULL);
}

static void dataAcquisitionThreadConfig(caerModuleData moduleData) {
	davisFX3State state = moduleData->moduleState;
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
		sendDVSFilterConfig(moduleData->moduleNode, cstate->deviceHandle);
	}

	if (configUpdate & (0x01 << 4)) {
		sendAPSConfig(moduleData->moduleNode, cstate->deviceHandle);
		sendAPSQuadROIConfig(moduleData->moduleNode, cstate->deviceHandle);
	}

	if (configUpdate & (0x01 << 5)) {
		sendIMUConfig(moduleData->moduleNode, cstate->deviceHandle);
	}

	if (configUpdate & (0x01 << 6)) {
		sendExternalInputDetectorConfig(moduleData->moduleNode, cstate->deviceHandle);
		sendExternalInputGeneratorConfig(moduleData->moduleNode, cstate->deviceHandle);
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

static void allocateDebugTransfers(davisFX3State state) {
	atomic_ops_uint_store(&state->debugTransfersLength, 0, ATOMIC_OPS_FENCE_NONE);

	// Set number of transfers and allocate memory for the main transfer array.
	state->debugTransfers = calloc(DEBUG_TRANSFER_NUM, sizeof(struct libusb_transfer *));
	if (state->debugTransfers == NULL) {
		caerLog(LOG_CRITICAL, state->cstate.sourceSubSystemString,
			"Failed to allocate memory for %" PRIu32 " libusb transfers (debug channel). Error: %s (%d).",
			DEBUG_TRANSFER_NUM, caerLogStrerror(errno), errno);
		return;
	}

	// Allocate transfers and set them up.
	for (size_t i = 0; i < DEBUG_TRANSFER_NUM; i++) {
		state->debugTransfers[i] = libusb_alloc_transfer(0);
		if (state->debugTransfers[i] == NULL) {
			caerLog(LOG_CRITICAL, state->cstate.sourceSubSystemString,
				"Unable to allocate further libusb transfers (debug channel, %zu of %" PRIu32 ").", i,
				DEBUG_TRANSFER_NUM);
			return;
		}

		// Create data buffer.
		state->debugTransfers[i]->length = DEBUG_TRANSFER_SIZE;
		state->debugTransfers[i]->buffer = malloc(DEBUG_TRANSFER_SIZE);
		if (state->debugTransfers[i]->buffer == NULL) {
			caerLog(LOG_CRITICAL, state->cstate.sourceSubSystemString,
				"Unable to allocate buffer for libusb transfer %zu (debug channel). Error: %s (%d).", i,
				caerLogStrerror(errno), errno);

			libusb_free_transfer(state->debugTransfers[i]);
			state->debugTransfers[i] = NULL;

			return;
		}

		// Initialize Transfer.
		state->debugTransfers[i]->dev_handle = state->cstate.deviceHandle;
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
			caerLog(LOG_CRITICAL, state->cstate.sourceSubSystemString,
				"Unable to submit libusb transfer %zu (debug channel). Error: %s (%d).", i, libusb_strerror(errno),
				errno);

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
			caerLog(LOG_CRITICAL, state->cstate.sourceSubSystemString,
				"Unable to cancel libusb transfer %zu (debug channel). Error: %s (%d).", i, libusb_strerror(errno),
				errno);
			// Proceed with canceling all transfers regardless of errors.
		}
	}

	// Wait for all transfers to go away (0.1 seconds timeout).
	struct timeval te = { .tv_sec = 0, .tv_usec = 100000 };

	while (atomic_ops_uint_load(&state->debugTransfersLength, ATOMIC_OPS_FENCE_NONE) > 0) {
		libusb_handle_events_timeout(state->cstate.deviceContext, &te);
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
		caerLog(LOG_ERROR, state->cstate.sourceSubSystemString, "Error message: '%s' (code %u at time %u).", &buffer[6],
			buffer[1], *((uint32_t *) &buffer[2]));
	}
	else {
		// Unknown/invalid debug message, log this.
		caerLog(LOG_WARNING, state->cstate.sourceSubSystemString, "Unknown/invalid debug message.");
	}
}

static void sendBiases(sshsNode moduleNode, libusb_device_handle *devHandle) {
	// TODO: depends on used chip.
}

static void sendChipSR(sshsNode moduleNode, libusb_device_handle *devHandle) {
	// TODO: depends on used chip.
}

static void sendDVSFilterConfig(sshsNode moduleNode, libusb_device_handle *devHandle) {
	sshsNode dvsNode = sshsGetRelativeNode(moduleNode, "dvs/");

	spiConfigSend(devHandle, FPGA_DVS, 7, sshsNodeGetShort(dvsNode, "FilterPixel0Row"));
	spiConfigSend(devHandle, FPGA_DVS, 8, sshsNodeGetShort(dvsNode, "FilterPixel0Column"));
	spiConfigSend(devHandle, FPGA_DVS, 9, sshsNodeGetShort(dvsNode, "FilterPixel1Row"));
	spiConfigSend(devHandle, FPGA_DVS, 10, sshsNodeGetShort(dvsNode, "FilterPixel1Column"));
	spiConfigSend(devHandle, FPGA_DVS, 11, sshsNodeGetShort(dvsNode, "FilterPixel2Row"));
	spiConfigSend(devHandle, FPGA_DVS, 12, sshsNodeGetShort(dvsNode, "FilterPixel2Column"));
	spiConfigSend(devHandle, FPGA_DVS, 13, sshsNodeGetShort(dvsNode, "FilterPixel3Row"));
	spiConfigSend(devHandle, FPGA_DVS, 15, sshsNodeGetShort(dvsNode, "FilterPixel3Column"));
	spiConfigSend(devHandle, FPGA_DVS, 15, sshsNodeGetShort(dvsNode, "FilterPixel4Row"));
	spiConfigSend(devHandle, FPGA_DVS, 16, sshsNodeGetShort(dvsNode, "FilterPixel4Column"));
	spiConfigSend(devHandle, FPGA_DVS, 17, sshsNodeGetShort(dvsNode, "FilterPixel5Row"));
	spiConfigSend(devHandle, FPGA_DVS, 18, sshsNodeGetShort(dvsNode, "FilterPixel5Column"));
	spiConfigSend(devHandle, FPGA_DVS, 19, sshsNodeGetShort(dvsNode, "FilterPixel6Row"));
	spiConfigSend(devHandle, FPGA_DVS, 20, sshsNodeGetShort(dvsNode, "FilterPixel6Column"));
	spiConfigSend(devHandle, FPGA_DVS, 21, sshsNodeGetShort(dvsNode, "FilterPixel7Row"));
	spiConfigSend(devHandle, FPGA_DVS, 22, sshsNodeGetShort(dvsNode, "FilterPixel7Column"));
	spiConfigSend(devHandle, FPGA_DVS, 23, sshsNodeGetBool(dvsNode, "FilterBackgroundActivity"));
	spiConfigSend(devHandle, FPGA_DVS, 24, sshsNodeGetInt(dvsNode, "FilterBackgroundActivityDeltaTime"));
}

static void sendAPSQuadROIConfig(sshsNode moduleNode, libusb_device_handle *devHandle) {
	sshsNode apsNode = sshsGetRelativeNode(moduleNode, "aps/");

	spiConfigSend(devHandle, FPGA_APS, 15, sshsNodeGetShort(apsNode, "StartColumn1"));
	spiConfigSend(devHandle, FPGA_APS, 16, sshsNodeGetShort(apsNode, "StartRow1"));
	spiConfigSend(devHandle, FPGA_APS, 17, sshsNodeGetShort(apsNode, "EndColumn1"));
	spiConfigSend(devHandle, FPGA_APS, 18, sshsNodeGetShort(apsNode, "EndRow1"));
	spiConfigSend(devHandle, FPGA_APS, 19, sshsNodeGetShort(apsNode, "StartColumn2"));
	spiConfigSend(devHandle, FPGA_APS, 20, sshsNodeGetShort(apsNode, "StartRow2"));
	spiConfigSend(devHandle, FPGA_APS, 21, sshsNodeGetShort(apsNode, "EndColumn2"));
	spiConfigSend(devHandle, FPGA_APS, 22, sshsNodeGetShort(apsNode, "EndRow2"));
	spiConfigSend(devHandle, FPGA_APS, 23, sshsNodeGetShort(apsNode, "StartColumn3"));
	spiConfigSend(devHandle, FPGA_APS, 24, sshsNodeGetShort(apsNode, "StartRow3"));
	spiConfigSend(devHandle, FPGA_APS, 25, sshsNodeGetShort(apsNode, "EndColumn3"));
	spiConfigSend(devHandle, FPGA_APS, 26, sshsNodeGetShort(apsNode, "EndRow3"));
}

static void sendExternalInputGeneratorConfig(sshsNode moduleNode, libusb_device_handle *devHandle) {
	sshsNode extNode = sshsGetRelativeNode(moduleNode, "externalInput/");

	spiConfigSend(devHandle, FPGA_EXTINPUT, 6, sshsNodeGetBool(extNode, "RunGenerator"));
	spiConfigSend(devHandle, FPGA_EXTINPUT, 7, sshsNodeGetBool(extNode, "GenerateUseCustomSignal"));
	spiConfigSend(devHandle, FPGA_EXTINPUT, 8, sshsNodeGetBool(extNode, "GeneratePulsePolarity"));
	spiConfigSend(devHandle, FPGA_EXTINPUT, 9, sshsNodeGetInt(extNode, "GeneratePulseInterval"));
	spiConfigSend(devHandle, FPGA_EXTINPUT, 10, sshsNodeGetInt(extNode, "GeneratePulseLength"));
}
