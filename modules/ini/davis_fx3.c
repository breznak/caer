#include "davis_common.h"
#include "davis_fx3.h"
#include "base/module.h"
#include <pthread.h>
#include <unistd.h>

struct davisFX3_state {
	// State for data management, common to all DAVISes.
	struct davisCommon_state cstate;
	// Debug transfer support (FX3 only).
	struct libusb_transfer *debugTransfers[DEBUG_TRANSFER_NUM];
	size_t activeDebugTransfers;
};

typedef struct davisFX3_state *davisFX3State;

static bool caerInputDAVISFX3Init(caerModuleData moduleData);
// RUN: common to all DAVIS systems.
// CONFIG: Nothing to do here in the main thread!
// Biases are configured asynchronously, and buffer sizes in the data
// acquisition thread itself. Resetting the main config_refresh flag
// will also happen there.
// EXIT: common to all DAVIS systems.

static struct caer_module_functions caerInputDAVISFX3Functions = { .moduleInit = &caerInputDAVISFX3Init, .moduleRun =
	&caerInputDAVISCommonRun, .moduleConfig = NULL, .moduleExit = &caerInputDAVISCommonExit };

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
static void allocateDebugTransfers(davisFX3State state);
static void deallocateDebugTransfers(davisFX3State state);
static void LIBUSB_CALL libUsbDebugCallback(struct libusb_transfer *transfer);
static void debugTranslator(davisFX3State state, uint8_t *buffer, size_t bytesSent);
static void sendBiases(sshsNode moduleNode, libusb_device_handle *devHandle);
static void sendChipSR(sshsNode moduleNode, libusb_device_handle *devHandle);
static void sendDVSFilterConfig(sshsNode moduleNode, libusb_device_handle *devHandle);
static void sendAPSQuadROIConfig(sshsNode moduleNode, libusb_device_handle *devHandle);
static void sendExternalInputGeneratorConfig(sshsNode moduleNode, libusb_device_handle *devHandle);
static void BiasesListener(sshsNode node, void *userData, enum sshs_node_attribute_events event, const char *changeKey,
	enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void ChipSRListener(sshsNode node, void *userData, enum sshs_node_attribute_events event, const char *changeKey,
	enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void DVSFilterConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void APSQuadROIConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);
static void ExternalInputGeneratorConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue);

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

	// Create common default value configuration.
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

	// Install default listeners to signal configuration updates asynchronously.
	sshsNodeAddAttrListener(dvsNode, cstate->deviceHandle, &DVSFilterConfigListener);
	sshsNodeAddAttrListener(apsNode, cstate->deviceHandle, &APSQuadROIConfigListener);
	sshsNodeAddAttrListener(extNode, cstate->deviceHandle, &ExternalInputGeneratorConfigListener);
	sshsNodeAddAttrListener(sshsGetRelativeNode(moduleData->moduleNode, "chip/"), moduleData, &ChipSRListener);
	// The chip SR needs to be updated also when GlobalShutter in APS changes.
	sshsNodeAddAttrListener(sshsGetRelativeNode(moduleData->moduleNode, "aps/"), moduleData, &ChipSRListener);

	// Walk all bias nodes and install the default handler for changes.
	size_t numBiasNodes;
	sshsNode *biasNodes = sshsNodeGetChildren(sshsGetRelativeNode(moduleData->moduleNode, "bias/"), &numBiasNodes);

	for (size_t i = 0; i < numBiasNodes; i++) {
		sshsNodeAddAttrListener(biasNodes[i], cstate->deviceHandle, &BiasesListener);
	}

	free(biasNodes);

	if (!initializeCommonConfiguration(moduleData, cstate, &dataAcquisitionThread)) {
		return (false);
	}

	caerLog(LOG_DEBUG, moduleData->moduleSubSystemString, "Initialized DAVISFX3 module successfully.");
	return (true);
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
	sendDVSFilterConfig(data->moduleNode, cstate->deviceHandle); // FX3 only.
	sendAPSQuadROIConfig(data->moduleNode, cstate->deviceHandle); // FX3 only.
	sendExternalInputGeneratorConfig(data->moduleNode, cstate->deviceHandle); // FX3 only.
	sendEnableDataConfig(data->moduleNode, cstate->deviceHandle);

	// Handle USB events (1 second timeout).
	struct timeval te = { .tv_sec = 0, .tv_usec = 1000000 };

	caerLog(LOG_DEBUG, data->moduleSubSystemString, "Data acquisition thread ready to process events.");

	while (atomic_ops_uint_load(&data->running, ATOMIC_OPS_FENCE_NONE) != 0 && cstate->activeDataTransfers > 0) {
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

static void allocateDebugTransfers(davisFX3State state) {
	// Set number of transfers and allocate memory for the main transfer array.

	// Allocate transfers and set them up.
	for (size_t i = 0; i < DEBUG_TRANSFER_NUM; i++) {
		state->debugTransfers[i] = libusb_alloc_transfer(0);
		if (state->debugTransfers[i] == NULL) {
			caerLog(LOG_CRITICAL, state->cstate.sourceSubSystemString,
				"Unable to allocate further libusb transfers (debug channel, %zu of %" PRIu32 ").", i,
				DEBUG_TRANSFER_NUM);
			continue;
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

			continue;
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
			state->activeDebugTransfers++;
		}
		else {
			caerLog(LOG_CRITICAL, state->cstate.sourceSubSystemString,
				"Unable to submit libusb transfer %zu (debug channel). Error: %s (%d).", i, libusb_strerror(errno),
				errno);

			// The transfer buffer is freed automatically here thanks to
			// the LIBUSB_TRANSFER_FREE_BUFFER flag set above.
			libusb_free_transfer(state->debugTransfers[i]);
			state->debugTransfers[i] = NULL;

			continue;
		}
	}

	if (state->activeDebugTransfers == 0) {
		// Didn't manage to allocate any USB transfers, log failure.
		caerLog(LOG_CRITICAL, state->cstate.sourceSubSystemString, "Unable to allocate any libusb transfers.");
	}
}

static void deallocateDebugTransfers(davisFX3State state) {
	// Cancel all current transfers first.
	for (size_t i = 0; i < DEBUG_TRANSFER_NUM; i++) {
		if (state->debugTransfers[i] != NULL) {
			errno = libusb_cancel_transfer(state->debugTransfers[i]);
			if (errno != LIBUSB_SUCCESS && errno != LIBUSB_ERROR_NOT_FOUND) {
				caerLog(LOG_CRITICAL, state->cstate.sourceSubSystemString,
					"Unable to cancel libusb transfer %zu (debug channel). Error: %s (%d).", i, libusb_strerror(errno),
					errno);
				// Proceed with trying to cancel all transfers regardless of errors.
			}
		}
	}

	// Wait for all transfers to go away (0.1 seconds timeout).
	struct timeval te = { .tv_sec = 0, .tv_usec = 100000 };

	while (state->activeDebugTransfers > 0) {
		libusb_handle_events_timeout(state->cstate.deviceContext, &te);
	}
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
	state->activeDebugTransfers--;
	for (size_t i = 0; i < DEBUG_TRANSFER_NUM; i++) {
		// Remove from list, so we don't try to cancel it later on.
		if (state->debugTransfers[i] == transfer) {
			state->debugTransfers[i] = NULL;
		}
	}
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

static void BiasesListener(sshsNode node, void *userData, enum sshs_node_attribute_events event, const char *changeKey,
	enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(changeKey);
	UNUSED_ARGUMENT(changeType);
	UNUSED_ARGUMENT(changeValue);

	libusb_device_handle *devHandle = userData;

	// This is currently for DAVIS240 only.
	if (event == ATTRIBUTE_MODIFIED) {
		if (str_equals(sshsNodeGetName(node), "DiffBn")) {
			spiConfigSend(devHandle, FPGA_CHIPBIAS, 0,
				generateAddressedCoarseFineBias(sshsNodeGetParent(node), "DiffBn"));
		}
		else if (str_equals(sshsNodeGetName(node), "OnBn")) {
			spiConfigSend(devHandle, FPGA_CHIPBIAS, 1,
				generateAddressedCoarseFineBias(sshsNodeGetParent(node), "OnBn"));
		}
		else if (str_equals(sshsNodeGetName(node), "OffBn")) {
			spiConfigSend(devHandle, FPGA_CHIPBIAS, 2,
				generateAddressedCoarseFineBias(sshsNodeGetParent(node), "OffBn"));
		}
		else if (str_equals(sshsNodeGetName(node), "ApsCasEpc")) {
			spiConfigSend(devHandle, FPGA_CHIPBIAS, 3,
				generateAddressedCoarseFineBias(sshsNodeGetParent(node), "ApsCasEpc"));
		}
		else if (str_equals(sshsNodeGetName(node), "DiffCasBnc")) {
			spiConfigSend(devHandle, FPGA_CHIPBIAS, 4,
				generateAddressedCoarseFineBias(sshsNodeGetParent(node), "DiffCasBnc"));
		}
		else if (str_equals(sshsNodeGetName(node), "ApsROSFBn")) {
			spiConfigSend(devHandle, FPGA_CHIPBIAS, 5,
				generateAddressedCoarseFineBias(sshsNodeGetParent(node), "ApsROSFBn"));
		}
		else if (str_equals(sshsNodeGetName(node), "LocalBufBn")) {
			spiConfigSend(devHandle, FPGA_CHIPBIAS, 6,
				generateAddressedCoarseFineBias(sshsNodeGetParent(node), "LocalBufBn"));
		}
		else if (str_equals(sshsNodeGetName(node), "PixInvBn")) {
			spiConfigSend(devHandle, FPGA_CHIPBIAS, 7,
				generateAddressedCoarseFineBias(sshsNodeGetParent(node), "PixInvBn"));
		}
		else if (str_equals(sshsNodeGetName(node), "PrBp")) {
			spiConfigSend(devHandle, FPGA_CHIPBIAS, 8,
				generateAddressedCoarseFineBias(sshsNodeGetParent(node), "PrBp"));
		}
		else if (str_equals(sshsNodeGetName(node), "PrSFBp")) {
			spiConfigSend(devHandle, FPGA_CHIPBIAS, 9,
				generateAddressedCoarseFineBias(sshsNodeGetParent(node), "PrSFBp"));
		}
		else if (str_equals(sshsNodeGetName(node), "RefrBp")) {
			spiConfigSend(devHandle, FPGA_CHIPBIAS, 10,
				generateAddressedCoarseFineBias(sshsNodeGetParent(node), "RefrBp"));
		}
		else if (str_equals(sshsNodeGetName(node), "AEPdBn")) {
			spiConfigSend(devHandle, FPGA_CHIPBIAS, 11,
				generateAddressedCoarseFineBias(sshsNodeGetParent(node), "AEPdBn"));
		}
		else if (str_equals(sshsNodeGetName(node), "LcolTimeoutBn")) {
			spiConfigSend(devHandle, FPGA_CHIPBIAS, 12,
				generateAddressedCoarseFineBias(sshsNodeGetParent(node), "LcolTimeoutBn"));
		}
		else if (str_equals(sshsNodeGetName(node), "AEPuXBp")) {
			spiConfigSend(devHandle, FPGA_CHIPBIAS, 13,
				generateAddressedCoarseFineBias(sshsNodeGetParent(node), "AEPuXBp"));
		}
		else if (str_equals(sshsNodeGetName(node), "AEPuYBp")) {
			spiConfigSend(devHandle, FPGA_CHIPBIAS, 14,
				generateAddressedCoarseFineBias(sshsNodeGetParent(node), "AEPuYBp"));
		}
		else if (str_equals(sshsNodeGetName(node), "IFThrBn")) {
			spiConfigSend(devHandle, FPGA_CHIPBIAS, 15,
				generateAddressedCoarseFineBias(sshsNodeGetParent(node), "IFThrBn"));
		}
		else if (str_equals(sshsNodeGetName(node), "IFRefrBn")) {
			spiConfigSend(devHandle, FPGA_CHIPBIAS, 16,
				generateAddressedCoarseFineBias(sshsNodeGetParent(node), "IFRefrBn"));
		}
		else if (str_equals(sshsNodeGetName(node), "PadFollBn")) {
			spiConfigSend(devHandle, FPGA_CHIPBIAS, 17,
				generateAddressedCoarseFineBias(sshsNodeGetParent(node), "PadFollBn"));
		}
		else if (str_equals(sshsNodeGetName(node), "ApsOverflowLevel")) {
			spiConfigSend(devHandle, FPGA_CHIPBIAS, 18,
				generateAddressedCoarseFineBias(sshsNodeGetParent(node), "ApsOverflowLevel"));
		}
		else if (str_equals(sshsNodeGetName(node), "BiasBuffer")) {
			spiConfigSend(devHandle, FPGA_CHIPBIAS, 19,
				generateAddressedCoarseFineBias(sshsNodeGetParent(node), "BiasBuffer"));
		}
		else if (str_equals(sshsNodeGetName(node), "SSP")) {
			spiConfigSend(devHandle, FPGA_CHIPBIAS, 20, generateShiftedSourceBias(sshsNodeGetParent(node), "SSP"));
		}
		else if (str_equals(sshsNodeGetName(node), "SSN")) {
			spiConfigSend(devHandle, FPGA_CHIPBIAS, 21, generateShiftedSourceBias(sshsNodeGetParent(node), "SSN"));
		}
	}
}

static void sendBiases(sshsNode moduleNode, libusb_device_handle *devHandle) {
	// This is currently for DAVIS240 only.
	sshsNode biasNode = sshsGetRelativeNode(moduleNode, "bias/");

	/*spiConfigSend(devHandle, FPGA_CHIPBIAS, 0, generateAddressedCoarseFineBias(biasNode, "DiffBn"));
	 spiConfigSend(devHandle, FPGA_CHIPBIAS, 1, generateAddressedCoarseFineBias(biasNode, "OnBn"));
	 spiConfigSend(devHandle, FPGA_CHIPBIAS, 2, generateAddressedCoarseFineBias(biasNode, "OffBn"));
	 spiConfigSend(devHandle, FPGA_CHIPBIAS, 3, generateAddressedCoarseFineBias(biasNode, "ApsCasEpc"));
	 spiConfigSend(devHandle, FPGA_CHIPBIAS, 4, generateAddressedCoarseFineBias(biasNode, "DiffCasBnc"));
	 spiConfigSend(devHandle, FPGA_CHIPBIAS, 5, generateAddressedCoarseFineBias(biasNode, "ApsROSFBn"));
	 spiConfigSend(devHandle, FPGA_CHIPBIAS, 6, generateAddressedCoarseFineBias(biasNode, "LocalBufBn"));
	 spiConfigSend(devHandle, FPGA_CHIPBIAS, 7, generateAddressedCoarseFineBias(biasNode, "PixInvBn"));
	 spiConfigSend(devHandle, FPGA_CHIPBIAS, 8, generateAddressedCoarseFineBias(biasNode, "PrBp"));
	 spiConfigSend(devHandle, FPGA_CHIPBIAS, 9, generateAddressedCoarseFineBias(biasNode, "PrSFBp"));
	 spiConfigSend(devHandle, FPGA_CHIPBIAS, 10, generateAddressedCoarseFineBias(biasNode, "RefrBp"));
	 spiConfigSend(devHandle, FPGA_CHIPBIAS, 11, generateAddressedCoarseFineBias(biasNode, "AEPdBn"));
	 spiConfigSend(devHandle, FPGA_CHIPBIAS, 12, generateAddressedCoarseFineBias(biasNode, "LcolTimeoutBn"));
	 spiConfigSend(devHandle, FPGA_CHIPBIAS, 13, generateAddressedCoarseFineBias(biasNode, "AEPuXBp"));
	 spiConfigSend(devHandle, FPGA_CHIPBIAS, 14, generateAddressedCoarseFineBias(biasNode, "AEPuYBp"));
	 spiConfigSend(devHandle, FPGA_CHIPBIAS, 15, generateAddressedCoarseFineBias(biasNode, "IFThrBn"));
	 spiConfigSend(devHandle, FPGA_CHIPBIAS, 16, generateAddressedCoarseFineBias(biasNode, "IFRefrBn"));
	 spiConfigSend(devHandle, FPGA_CHIPBIAS, 17, generateAddressedCoarseFineBias(biasNode, "PadFollBn"));
	 spiConfigSend(devHandle, FPGA_CHIPBIAS, 18, generateAddressedCoarseFineBias(biasNode, "ApsOverflowLevel"));

	 spiConfigSend(devHandle, FPGA_CHIPBIAS, 19, generateAddressedCoarseFineBias(biasNode, "BiasBuffer"));

	 spiConfigSend(devHandle, FPGA_CHIPBIAS, 20, generateShiftedSourceBias(biasNode, "SSP"));
	 spiConfigSend(devHandle, FPGA_CHIPBIAS, 21, generateShiftedSourceBias(biasNode, "SSN"));*/

	/*spiConfigSend(devHandle, FPGA_CHIPBIAS, 0, generateVDACBias(biasNode, "ApsOverflowLevel"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 1, generateVDACBias(biasNode, "ApsCas"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 2, generateVDACBias(biasNode, "AdcRefHigh"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 3, generateVDACBias(biasNode, "AdcRefLow"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 4, generateVDACBias(biasNode, "AdcTestVoltage"));

	spiConfigSend(devHandle, FPGA_CHIPBIAS, 8, generateAddressedCoarseFineBias(biasNode, "LocalBufBn"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 9, generateAddressedCoarseFineBias(biasNode, "PadFollBn"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 10, generateAddressedCoarseFineBias(biasNode, "DiffBn"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 11, generateAddressedCoarseFineBias(biasNode, "OnBn"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 12, generateAddressedCoarseFineBias(biasNode, "OffBn"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 13, generateAddressedCoarseFineBias(biasNode, "PixInvBn"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 14, generateAddressedCoarseFineBias(biasNode, "PrBp"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 15, generateAddressedCoarseFineBias(biasNode, "PrSFBp"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 16, generateAddressedCoarseFineBias(biasNode, "RefrBp"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 17, generateAddressedCoarseFineBias(biasNode, "ReadoutBufBp"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 18, generateAddressedCoarseFineBias(biasNode, "ApsROSFBn"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 19, generateAddressedCoarseFineBias(biasNode, "AdcCompBp"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 20, generateAddressedCoarseFineBias(biasNode, "ColSelLowBn"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 21, generateAddressedCoarseFineBias(biasNode, "DACBufBp"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 22, generateAddressedCoarseFineBias(biasNode, "LcolTimeoutBn"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 23, generateAddressedCoarseFineBias(biasNode, "AEPdBn"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 24, generateAddressedCoarseFineBias(biasNode, "AEPuXBp"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 25, generateAddressedCoarseFineBias(biasNode, "AEPuYBp"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 26, generateAddressedCoarseFineBias(biasNode, "IFRefrBn"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 27, generateAddressedCoarseFineBias(biasNode, "IFThrBn"));

	spiConfigSend(devHandle, FPGA_CHIPBIAS, 34, generateAddressedCoarseFineBias(biasNode, "BiasBuffer"));

	spiConfigSend(devHandle, FPGA_CHIPBIAS, 35, generateShiftedSourceBias(biasNode, "SSP"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 36, generateShiftedSourceBias(biasNode, "SSN"));*/

	spiConfigSend(devHandle, FPGA_CHIPBIAS, 0, generateVDACBias(biasNode, "ApsCasBpc"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 1, generateVDACBias(biasNode, "OVG1Lo"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 2, generateVDACBias(biasNode, "OVG2Lo"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 3, generateVDACBias(biasNode, "TX2OVG2Hi"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 4, generateVDACBias(biasNode, "Gnd07"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 5, generateVDACBias(biasNode, "vADCTest"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 6, generateVDACBias(biasNode, "AdcRefHigh"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 7, generateVDACBias(biasNode, "AdcRefLow"));

	spiConfigSend(devHandle, FPGA_CHIPBIAS, 8, generateAddressedCoarseFineBias(biasNode, "IFRefrBn"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 9, generateAddressedCoarseFineBias(biasNode, "IFThrBn"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 10, generateAddressedCoarseFineBias(biasNode, "LocalBufBn"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 11, generateAddressedCoarseFineBias(biasNode, "PadFollBn"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 13, generateAddressedCoarseFineBias(biasNode, "PixInvBn"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 14, generateAddressedCoarseFineBias(biasNode, "DiffBn"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 15, generateAddressedCoarseFineBias(biasNode, "OnBn"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 16, generateAddressedCoarseFineBias(biasNode, "OffBn"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 17, generateAddressedCoarseFineBias(biasNode, "PrBp"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 18, generateAddressedCoarseFineBias(biasNode, "PrSFBp"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 19, generateAddressedCoarseFineBias(biasNode, "RefrBp"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 20, generateAddressedCoarseFineBias(biasNode, "ArrayBiasBufferBn"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 22, generateAddressedCoarseFineBias(biasNode, "ArrayLogicBufferBn"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 23, generateAddressedCoarseFineBias(biasNode, "FalltimeBn"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 24, generateAddressedCoarseFineBias(biasNode, "RisetimeBp"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 25, generateAddressedCoarseFineBias(biasNode, "ReadoutBufBp"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 26, generateAddressedCoarseFineBias(biasNode, "ApsROSFBn"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 27, generateAddressedCoarseFineBias(biasNode, "AdcCompBp"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 28, generateAddressedCoarseFineBias(biasNode, "DACBufBp"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 30, generateAddressedCoarseFineBias(biasNode, "LcolTimeoutBn"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 31, generateAddressedCoarseFineBias(biasNode, "AEPdBn"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 32, generateAddressedCoarseFineBias(biasNode, "AEPuXBp"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 33, generateAddressedCoarseFineBias(biasNode, "AEPuYBp"));

	spiConfigSend(devHandle, FPGA_CHIPBIAS, 34, generateAddressedCoarseFineBias(biasNode, "BiasBuffer"));

	spiConfigSend(devHandle, FPGA_CHIPBIAS, 35, generateShiftedSourceBias(biasNode, "SSP"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 36, generateShiftedSourceBias(biasNode, "SSN"));
}

static void ChipSRListener(sshsNode node, void *userData, enum sshs_node_attribute_events event, const char *changeKey,
	enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(changeValue);

	// This is currently for DAVIS240 only.
	caerModuleData moduleData = userData;
	libusb_device_handle *devHandle = ((davisFX3State) moduleData->moduleState)->cstate.deviceHandle;

	if (event == ATTRIBUTE_MODIFIED) {
		if (str_equals(sshsNodeGetName(node), "aps")) {
			if (changeType == BOOL && str_equals(changeKey, "GlobalShutter")) {
				spiConfigSend(devHandle, FPGA_CHIPBIAS, 142, changeValue.boolean);
			}
		}
		else if (str_equals(sshsNodeGetName(node), "chip")) {
			if (changeType == BYTE && str_equals(changeKey, "AnalogMux0")) {
				spiConfigSend(devHandle, FPGA_CHIPBIAS, 132, changeValue.ubyte);
			}
			else if (changeType == BYTE && str_equals(changeKey, "AnalogMux1")) {
				spiConfigSend(devHandle, FPGA_CHIPBIAS, 133, changeValue.ubyte);
			}
			else if (changeType == BYTE && str_equals(changeKey, "AnalogMux2")) {
				spiConfigSend(devHandle, FPGA_CHIPBIAS, 134, changeValue.ubyte);
			}
			else if (changeType == BYTE && str_equals(changeKey, "BiasMux")) {
				spiConfigSend(devHandle, FPGA_CHIPBIAS, 135, changeValue.ubyte);
			}
			else if (changeType == BYTE && str_equals(changeKey, "DigitalMux0")) {
				spiConfigSend(devHandle, FPGA_CHIPBIAS, 128, changeValue.ubyte);
			}
			else if (changeType == BYTE && str_equals(changeKey, "DigitalMux1")) {
				spiConfigSend(devHandle, FPGA_CHIPBIAS, 129, changeValue.ubyte);
			}
			else if (changeType == BYTE && str_equals(changeKey, "DigitalMux2")) {
				spiConfigSend(devHandle, FPGA_CHIPBIAS, 130, changeValue.ubyte);
			}
			else if (changeType == BYTE && str_equals(changeKey, "DigitalMux3")) {
				spiConfigSend(devHandle, FPGA_CHIPBIAS, 131, changeValue.ubyte);
			}
			else if (changeType == BOOL && str_equals(changeKey, "UseAout")) {
				spiConfigSend(devHandle, FPGA_CHIPBIAS, 141, changeValue.boolean);
			}
			else if (changeType == BOOL && str_equals(changeKey, "nArow")) {
				spiConfigSend(devHandle, FPGA_CHIPBIAS, 140, changeValue.boolean);
			}
			else if (changeType == BOOL && str_equals(changeKey, "ResetTestPixel")) {
				spiConfigSend(devHandle, FPGA_CHIPBIAS, 138, changeValue.boolean);
			}
			else if (changeType == BOOL && str_equals(changeKey, "TypeNCalibNeuron")) {
				spiConfigSend(devHandle, FPGA_CHIPBIAS, 137, changeValue.boolean);
			}
			else if (changeType == BOOL && str_equals(changeKey, "ResetCalibNeuron")) {
				spiConfigSend(devHandle, FPGA_CHIPBIAS, 136, changeValue.boolean);
			}
			else if (changeType == BOOL && str_equals(changeKey, "HotPixelSuppression")) {
				spiConfigSend(devHandle, FPGA_CHIPBIAS, 139, changeValue.boolean);
			}
		}
	}
}

static void sendChipSR(sshsNode moduleNode, libusb_device_handle *devHandle) {
	// This is currently for DAVIS240 only.
	sshsNode chipNode = sshsGetRelativeNode(moduleNode, "chip/");
	sshsNode apsNode = sshsGetRelativeNode(moduleNode, "aps/");

	// Debug muxes control.
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 131, sshsNodeGetByte(chipNode, "DigitalMux3"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 130, sshsNodeGetByte(chipNode, "DigitalMux2"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 129, sshsNodeGetByte(chipNode, "DigitalMux1"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 128, sshsNodeGetByte(chipNode, "DigitalMux0"));

	spiConfigSend(devHandle, FPGA_CHIPBIAS, 134, sshsNodeGetByte(chipNode, "AnalogMux2"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 133, sshsNodeGetByte(chipNode, "AnalogMux1"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 132, sshsNodeGetByte(chipNode, "AnalogMux0"));

	spiConfigSend(devHandle, FPGA_CHIPBIAS, 135, sshsNodeGetByte(chipNode, "BiasMux"));

	// GS may not exist on chips that don't have it.
	if (sshsNodeAttrExists(apsNode, "GlobalShutter", BOOL)) {
		spiConfigSend(devHandle, FPGA_CHIPBIAS, 142, sshsNodeGetBool(apsNode, "GlobalShutter"));
	}

	spiConfigSend(devHandle, FPGA_CHIPBIAS, 141, sshsNodeGetBool(chipNode, "UseAout"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 140, sshsNodeGetBool(chipNode, "nArow"));
	// Not on 346: spiConfigSend(devHandle, FPGA_CHIPBIAS, 139, sshsNodeGetBool(chipNode, "HotPixelSuppression"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 138, sshsNodeGetBool(chipNode, "ResetTestPixel"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 137, sshsNodeGetBool(chipNode, "TypeNCalibNeuron"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 136, sshsNodeGetBool(chipNode, "ResetCalibNeuron"));
	// 346 only.
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 143, sshsNodeGetBool(chipNode, "SelectGrayCounter"));
	//spiConfigSend(devHandle, FPGA_CHIPBIAS, 144, sshsNodeGetBool(chipNode, "TestADC"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 145, sshsNodeGetBool(chipNode, "AdjOVG1Lo"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 146, sshsNodeGetBool(chipNode, "AdjOVG2Lo"));
	spiConfigSend(devHandle, FPGA_CHIPBIAS, 147, sshsNodeGetBool(chipNode, "AdjTX2OVG2Hi"));
}

static void DVSFilterConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	libusb_device_handle *devHandle = userData;

	if (event == ATTRIBUTE_MODIFIED) {
		if (changeType == SHORT && str_equals(changeKey, "FilterPixel0Row")) {
			spiConfigSend(devHandle, FPGA_DVS, 7, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel0Column")) {
			spiConfigSend(devHandle, FPGA_DVS, 8, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel1Row")) {
			spiConfigSend(devHandle, FPGA_DVS, 9, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel1Column")) {
			spiConfigSend(devHandle, FPGA_DVS, 10, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel2Row")) {
			spiConfigSend(devHandle, FPGA_DVS, 11, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel2Column")) {
			spiConfigSend(devHandle, FPGA_DVS, 12, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel3Row")) {
			spiConfigSend(devHandle, FPGA_DVS, 13, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel3Column")) {
			spiConfigSend(devHandle, FPGA_DVS, 14, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel4Row")) {
			spiConfigSend(devHandle, FPGA_DVS, 15, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel4Column")) {
			spiConfigSend(devHandle, FPGA_DVS, 16, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel5Row")) {
			spiConfigSend(devHandle, FPGA_DVS, 17, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel5Column")) {
			spiConfigSend(devHandle, FPGA_DVS, 18, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel6Row")) {
			spiConfigSend(devHandle, FPGA_DVS, 19, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel6Column")) {
			spiConfigSend(devHandle, FPGA_DVS, 20, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel7Row")) {
			spiConfigSend(devHandle, FPGA_DVS, 21, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "FilterPixel7Column")) {
			spiConfigSend(devHandle, FPGA_DVS, 22, changeValue.ushort);
		}
		else if (changeType == BOOL && str_equals(changeKey, "FilterBackgroundActivity")) {
			spiConfigSend(devHandle, FPGA_DVS, 23, changeValue.boolean);
		}
		else if (changeType == INT && str_equals(changeKey, "FilterBackgroundActivityDeltaTime")) {
			spiConfigSend(devHandle, FPGA_DVS, 24, changeValue.uint);
		}
	}
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

static void APSQuadROIConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	libusb_device_handle *devHandle = userData;

	if (event == ATTRIBUTE_MODIFIED) {
		if (changeType == SHORT && str_equals(changeKey, "StartColumn1")) {
			spiConfigSend(devHandle, FPGA_APS, 15, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "StartRow1")) {
			spiConfigSend(devHandle, FPGA_APS, 16, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "EndColumn1")) {
			spiConfigSend(devHandle, FPGA_APS, 17, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "EndRow1")) {
			spiConfigSend(devHandle, FPGA_APS, 18, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "StartColumn2")) {
			spiConfigSend(devHandle, FPGA_APS, 19, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "StartRow2")) {
			spiConfigSend(devHandle, FPGA_APS, 20, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "EndColumn2")) {
			spiConfigSend(devHandle, FPGA_APS, 21, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "EndRow2")) {
			spiConfigSend(devHandle, FPGA_APS, 22, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "StartColumn3")) {
			spiConfigSend(devHandle, FPGA_APS, 23, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "StartRow3")) {
			spiConfigSend(devHandle, FPGA_APS, 24, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "EndColumn3")) {
			spiConfigSend(devHandle, FPGA_APS, 25, changeValue.ushort);
		}
		else if (changeType == SHORT && str_equals(changeKey, "EndRow3")) {
			spiConfigSend(devHandle, FPGA_APS, 26, changeValue.ushort);
		}
	}
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

static void ExternalInputGeneratorConfigListener(sshsNode node, void *userData, enum sshs_node_attribute_events event,
	const char *changeKey, enum sshs_node_attr_value_type changeType, union sshs_node_attr_value changeValue) {
	UNUSED_ARGUMENT(node);

	libusb_device_handle *devHandle = userData;

	if (event == ATTRIBUTE_MODIFIED) {
		if (changeType == BOOL && str_equals(changeKey, "RunGenerator")) {
			spiConfigSend(devHandle, FPGA_EXTINPUT, 6, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "GenerateUseCustomSignal")) {
			spiConfigSend(devHandle, FPGA_EXTINPUT, 7, changeValue.boolean);
		}
		else if (changeType == BOOL && str_equals(changeKey, "GeneratePulsePolarity")) {
			spiConfigSend(devHandle, FPGA_EXTINPUT, 8, changeValue.boolean);
		}
		else if (changeType == INT && str_equals(changeKey, "GeneratePulseInterval")) {
			spiConfigSend(devHandle, FPGA_EXTINPUT, 9, changeValue.uint);
		}
		else if (changeType == INT && str_equals(changeKey, "GeneratePulseLength")) {
			spiConfigSend(devHandle, FPGA_EXTINPUT, 10, changeValue.uint);
		}
	}
}

static void sendExternalInputGeneratorConfig(sshsNode moduleNode, libusb_device_handle *devHandle) {
	sshsNode extNode = sshsGetRelativeNode(moduleNode, "externalInput/");

	spiConfigSend(devHandle, FPGA_EXTINPUT, 7, sshsNodeGetBool(extNode, "GenerateUseCustomSignal"));
	spiConfigSend(devHandle, FPGA_EXTINPUT, 8, sshsNodeGetBool(extNode, "GeneratePulsePolarity"));
	spiConfigSend(devHandle, FPGA_EXTINPUT, 9, sshsNodeGetInt(extNode, "GeneratePulseInterval"));
	spiConfigSend(devHandle, FPGA_EXTINPUT, 10, sshsNodeGetInt(extNode, "GeneratePulseLength"));
	spiConfigSend(devHandle, FPGA_EXTINPUT, 6, sshsNodeGetBool(extNode, "RunGenerator"));
}
